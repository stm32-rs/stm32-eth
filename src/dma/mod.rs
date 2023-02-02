//! Ethernet DMA access and configuration.

use core::borrow::Borrow;

use cortex_m::peripheral::NVIC;

use crate::{peripherals::ETHERNET_DMA, stm32::Interrupt};

#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::*;

pub(crate) mod raw_descriptor;

mod rx;
pub use rx::{RxDescriptor, RxDescriptorRing, RxError, RxPacket};

mod tx;
pub use tx::{TxDescriptor, TxDescriptorRing, TxError};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

mod packet_id;
pub use packet_id::PacketId;

use rx::RxRing;
use tx::{RunningState, TxRing};

use self::raw_descriptor::DESC_SIZE;

const _RXDESC_SIZE: usize = core::mem::size_of::<RxDescriptor>();
const _TXDESC_SIZE: usize = core::mem::size_of::<TxDescriptor>();

/// Assert that our descriptors have the same size.
///
/// This is necessary as we only have a single Descriptor Skip Length
/// value which applies to both TX and RX descriptors.
const _ASSERT_DESCRIPTOR_SIZES: () = assert!(_RXDESC_SIZE == _TXDESC_SIZE);

const DESC_WORD_SKIP: u8 = (core::mem::size_of::<RxDescriptor>() / 4 - DESC_SIZE) as u8;

/// The maximum transmission unit of this Ethernet peripheral.
///
/// From the datasheet: *VLAN Frame maxsize = 1522*
pub const MTU: usize = 1522;

/// An error that can occur when retrieving a timestamp from an
/// RX or TX descriptor handled by the DMA.
#[cfg(feature = "ptp")]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimestampError {
    /// The descriptor with the given packet ID has not been
    /// timestamped yet.
    NotYetTimestamped,
    /// No active descriptors have the given packet ID.
    IdNotFound,
}

/// Ethernet DMA.
pub struct EthernetDMA<'rx, 'tx> {
    eth_dma: ETHERNET_DMA,
    rx_ring: RxRing<'rx, rx::Running>,
    tx_ring: TxRing<'tx, tx::Running>,
}

impl<'rx, 'tx> EthernetDMA<'rx, 'tx> {
    /// Create and initialise the ethernet DMA
    ///
    /// # Note
    /// - Make sure that the buffers reside in a memory region that is
    /// accessible by the peripheral. Core-Coupled Memory (CCM) is
    /// usually not accessible.
    pub(crate) fn new(
        eth_dma: ETHERNET_DMA,
        rx_buffer: RxDescriptorRing<'rx>,
        tx_buffer: TxDescriptorRing<'tx>,
    ) -> Self {
        // reset DMA bus mode register
        eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

        // Wait until done
        while eth_dma.dmabmr.read().sr().bit_is_set() {}

        // operation mode register
        eth_dma.dmaomr.modify(|_, w| {
            // Dropping of TCP/IP checksum error frames disable
            w.dtcefd()
                .set_bit()
                // Receive store and forward
                .rsf()
                .set_bit()
                // Disable flushing of received frames
                .dfrf()
                .set_bit()
                // Transmit store and forward
                .tsf()
                .set_bit()
                // Forward error frames
                .fef()
                .set_bit()
                // Operate on second frame
                .osf()
                .set_bit()
        });

        // bus mode register
        eth_dma.dmabmr.modify(|_, w| {
            // For any non-f107 chips, we must use enhanced descriptor format to support checksum
            // offloading and/or timestamps.
            #[cfg(not(feature = "stm32f1xx-hal"))]
            let w = w.edfe().set_bit();

            unsafe {
                // Address-aligned beats
                w.aab()
                    .set_bit()
                    // Fixed burst
                    .fb()
                    .set_bit()
                    // Rx DMA PBL
                    .rdp()
                    .bits(32)
                    // Programmable burst length
                    .pbl()
                    .bits(32)
                    // Rx Tx priority ratio 2:1
                    .pm()
                    .bits(0b01)
                    // Use separate PBL
                    .usp()
                    .set_bit()
            }
        });

        // Configure word skip length.
        eth_dma.dmabmr.modify(|_, w| w.dsl().bits(DESC_WORD_SKIP));

        let rx_ring = RxRing::new(rx_buffer).start(&eth_dma);
        let tx_ring = TxRing::new(tx_buffer).start(&eth_dma);

        EthernetDMA {
            eth_dma,
            rx_ring,
            tx_ring,
        }
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html) to
    /// clear interrupt pending bits. Otherwise the interrupt will
    /// reoccur immediately.
    pub fn enable_interrupt(&self) {
        self.eth_dma.dmaier.modify(|_, w| {
            w
                // Normal interrupt summary enable
                .nise()
                .set_bit()
                // Receive Interrupt Enable
                .rie()
                .set_bit()
                // Transmit Interrupt Enable
                .tie()
                .set_bit()
        });

        // Enable ethernet interrupts
        unsafe {
            NVIC::unmask(Interrupt::ETH);
        }
    }

    /// Calls [`eth_interrupt_handler()`]
    #[cfg_attr(
        feature = "ptp",
        doc = " and collects/caches TX timestamps. (See [`EthernetDMA::get_timestamp_for_id`] for retrieval)"
    )]
    pub fn interrupt_handler(&mut self) -> InterruptReasonSummary {
        let eth_dma = &self.eth_dma;
        let status = eth_interrupt_handler_impl(eth_dma);
        #[cfg(feature = "ptp")]
        self.collect_timestamps();
        status
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call `recv_next()` to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        self.rx_ring.running_state(&self.eth_dma).is_running()
    }

    ///
    pub fn tx_state(&self) -> RunningState {
        self.tx_ring.running_state(&self.eth_dma)
    }

    fn recv_next_impl<'rx_borrow>(
        eth_dma: &ETHERNET_DMA,
        rx_ring: &'rx_borrow mut RxRing<rx::Running>,
        rx_packet_id: Option<PacketId>,
    ) -> Result<RxPacket<'rx_borrow>, RxError> {
        rx_ring.recv_next(eth_dma, rx_packet_id.map(|p| p.into()))
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(&mut self, packet_id: Option<PacketId>) -> Result<RxPacket, RxError> {
        Self::recv_next_impl(&self.eth_dma, &mut self.rx_ring, packet_id)
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        self.tx_ring.is_running(&self.eth_dma)
    }

    pub(crate) fn send_impl<F: FnOnce(&mut [u8]) -> R, R>(
        eth_dma: &ETHERNET_DMA,
        tx_ring: &mut TxRing<tx::Running>,
        length: usize,
        tx_packet_id: Option<PacketId>,
        f: F,
    ) -> Result<R, TxError> {
        let result = tx_ring.send(length, tx_packet_id.map(|p| p.into()), f);
        tx_ring.demand_poll(eth_dma);
        result
    }

    /// Send a packet
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
        f: F,
    ) -> Result<R, TxError> {
        Self::send_impl(&self.eth_dma, &mut self.tx_ring, length, packet_id, f)
    }

    #[cfg(feature = "ptp")]
    /// Get a timestamp for the given ID
    ///
    /// Both RX and TX timestamps can be obtained reliably as follows:
    /// 1. When an ethernet interrupt occurs, call [`EthernetDMA::interrupt_handler`] (_not_ [`eth_interrupt_handler`]).
    /// 2. Before calling [`interrupt_handler`](EthernetDMA::interrupt_handler) again, retrieve timestamps of sent and received frames using this function.
    ///
    /// Retrieving RX timestamps can also be done using [`RxPacket::timestamp`].
    pub fn get_timestamp_for_id<'a, PKT>(
        &mut self,
        packet_id: PKT,
    ) -> Result<Timestamp, TimestampError>
    where
        PKT: Into<PacketId>,
    {
        let Self {
            tx_ring, rx_ring, ..
        } = self;

        let internal_packet_id = packet_id.into();

        tx_ring
            .get_timestamp_for_id(internal_packet_id.clone())
            .or_else(|_| rx_ring.get_timestamp_for_id(internal_packet_id))
    }

    /// Collect the timestamps from the TX descriptor
    /// ring
    #[cfg(feature = "ptp")]
    fn collect_timestamps(&mut self) {
        self.tx_ring.collect_timestamps();
    }
}

impl<'rx, 'tx> Drop for EthernetDMA<'rx, 'tx> {
    fn drop(&mut self) {
        self.rx_ring.stop(&self.eth_dma);
        self.tx_ring.stop(&self.eth_dma);
    }
}

/// A summary of the reasons for the interrupt
/// that occured
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy)]
pub struct InterruptReasonSummary {
    /// The interrupt was caused by an RX event.
    pub is_rx: bool,
    /// The interrupt was caused by an TX event.
    pub is_tx: bool,
    /// The interrupt was caused by an error event.
    pub is_error: bool,
}

/// The handler for `ETH` interrupts.
///
/// There are two ways to call this:
///
/// * Indirectly by using [`EthernetDMA::interrupt_handler`] driver instance that your interrupt handler has access to.
/// * By unsafely getting `Peripherals`.
pub fn eth_interrupt_handler(eth_dma: &crate::hal::pac::ETHERNET_DMA) -> InterruptReasonSummary {
    let eth_dma: &ETHERNET_DMA = eth_dma.borrow();
    eth_interrupt_handler_impl(eth_dma)
}

fn eth_interrupt_handler_impl(eth_dma: &ETHERNET_DMA) -> InterruptReasonSummary {
    let status = eth_dma.dmasr.read();

    let status = InterruptReasonSummary {
        is_rx: status.rs().bit_is_set(),
        is_tx: status.ts().bit_is_set(),
        is_error: status.ais().bit_is_set(),
    };

    eth_dma
        .dmasr
        .write(|w| w.nis().set_bit().ts().set_bit().rs().set_bit());

    status
}
