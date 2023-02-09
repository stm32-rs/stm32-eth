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

const DESC_WORD_SKIP: u8 = ((_RXDESC_SIZE / 4) - DESC_SIZE) as u8;

const _ASSERT_DESC_WORD_SKIP_SIZE: () = assert!(DESC_WORD_SKIP <= 0b111);

/// The maximum transmission unit of this Ethernet peripheral.
///
/// From the datasheet: *VLAN Frame maxsize = 1522*
pub const MTU: usize = 1522;

/// An error that can occur when retrieving a timestamp from an
/// RX or TX descriptor handled by the DMA.
#[cfg(feature = "ptp")]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimestampError {
    /// The descriptor with the given packet ID has not been
    /// timestamped yet.
    NotYetTimestamped,
    /// No active descriptors have the given packet ID.
    IdNotFound,
}

/// Ethernet DMA.
pub struct EthernetDMA<'rx, 'tx> {
    parts: DmaParts,
    rx_ring: RxRing<'rx, rx::Running>,
    tx_ring: TxRing<'tx, tx::Running>,
}

pub(crate) struct DmaParts {
    pub eth_dma: ETHERNET_DMA,
    #[cfg(feature = "stm32h7xx-hal")]
    pub eth_mtl: crate::stm32::ETHERNET_MTL,
}

impl<'rx, 'tx> EthernetDMA<'rx, 'tx> {
    fn eth_dma(&self) -> &ETHERNET_DMA {
        &self.parts.eth_dma
    }

    /// Create and initialise the ethernet DMA
    ///
    /// # Note
    /// - Make sure that the buffers reside in a memory region that is
    /// accessible by the peripheral. Core-Coupled Memory (CCM) is
    /// usually not accessible.
    pub(crate) fn new(
        parts: DmaParts,
        rx_buffer: RxDescriptorRing<'rx>,
        tx_buffer: TxDescriptorRing<'tx>,
    ) -> Self {
        let DmaParts {
            eth_dma,
            #[cfg(feature = "stm32h7xx-hal")]
            eth_mtl,
        } = &parts;

        #[cfg(feature = "f-series")]
        {
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
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // reset DMA bus mode register
            parts.eth_dma.dmamr.modify(|_, w| w.swr().set_bit());
            // Wait until done
            while eth_dma.dmamr.read().swr().bit_is_set() {}

            // Rx Tx priority ratio 2:1
            eth_dma.dmamr.modify(|_, w| w.pr().variant(0b001));

            eth_dma
                .dmaccr
                .modify(|_, w| w.dsl().variant(DESC_WORD_SKIP));

            // Operation mode registers
            eth_mtl.mtlrx_qomr.modify(|_, w| {
                w
                    // Dropping of TCP/IP checksum error frames disable
                    .dis_tcp_ef()
                    .set_bit()
                    // Receive store and forward
                    .rsf()
                    .set_bit()
                    // Forward error frames
                    .fep()
                    .set_bit()
                    // Forward undersized but good frames
                    .fup()
                    .set_bit()
            });

            // Transmit store and forward
            eth_mtl.mtltx_qomr.modify(|_, w| w.tsf().set_bit());

            eth_dma.dmasbmr.modify(|_, w| w.aal().set_bit());

            eth_dma.dmacrx_cr.modify(|_, w| {
                w
                    // RX DMA programmable burst length.
                    .rxpbl()
                    .variant(32)
                    // Receive buffer size
                    .rbsz()
                    .variant(rx_buffer.first_buffer().len() as u16)
            });

            eth_dma.dmactx_cr.modify(|_, w| {
                w
                    // TX DMA programmable burst length.
                    .txpbl()
                    .variant(32)
                    // Operate on second packet
                    .osf()
                    .set_bit()
            });
        }

        let rx_ring = RxRing::new(rx_buffer).start(&eth_dma);
        let tx_ring = TxRing::new(tx_buffer).start(&eth_dma);

        EthernetDMA {
            parts,
            rx_ring,
            tx_ring,
        }
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`] to  clear interrupt pending
    /// bits. Otherwise the interrupt will reoccur immediately.
    pub fn enable_interrupt(&self) {
        #[cfg(feature = "f-series")]
        self.eth_dma().dmaier.modify(|_, w| {
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

        #[cfg(feature = "stm32h7xx-hal")]
        self.eth_dma().dmacier.modify(|_, w| {
            w
                // Normal interrupt summary enable
                .nie()
                .set_bit()
                // Receive Interrupt Enable
                .rie()
                .set_bit()
                // Transmit Interrupt Enable
                .tie()
                .set_bit()
                // Abnormal Interrupt Summary enable
                .aie()
                .set_bit()
                // Receive Buffer Unavailable
                .rbue()
                .set_bit()
                // Transmit Buffer Unavailable
                .tbue()
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
        let status = eth_interrupt_handler_impl(self.eth_dma());
        #[cfg(feature = "ptp")]
        self.collect_timestamps();
        status
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call `recv_next()` to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        self.rx_ring.running_state().is_running()
    }

    /// Get the current state of Tx DMA
    pub fn tx_state(&self) -> RunningState {
        self.tx_ring.running_state()
    }

    /// Receive the next packet (if any is ready), or return [`Err`]
    /// immediately.
    pub fn recv_next(&mut self, packet_id: Option<PacketId>) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(packet_id.map(|p| p.into()))
    }

    /// Check if there is a packet available for reading.
    ///
    /// If this function returns true, it is guaranteed that the
    /// next call to [`EthernetDMA::recv_next`] will return [`Ok`].
    pub fn rx_available(&mut self) -> bool {
        self.rx_ring.available()
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        self.tx_ring.is_running()
    }

    /// Send a packet
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
        f: F,
    ) -> Result<R, TxError> {
        self.tx_ring.send(length, packet_id.map(|p| p.into()), f)
    }

    /// Check if sending a packet now would succeed.
    ///
    /// If this function returns true, it is guaranteed that
    /// the next call to [`EthernetDMA::send`] will return [`Ok`]
    pub fn tx_available(&mut self) -> bool {
        self.tx_ring.available()
    }

    #[cfg(feature = "ptp")]
    /// Get a timestamp for the given ID
    ///
    /// Both RX and TX timestamps can be obtained reliably as follows:
    /// 1. When an ethernet interrupt occurs, call [`EthernetDMA::interrupt_handler`] (_not_ [`eth_interrupt_handler`]).
    /// 2. Before calling [`interrupt_handler`](EthernetDMA::interrupt_handler) again, retrieve timestamps of sent and received frames using this function.
    ///
    /// Retrieving RX timestamps can also be done using [`RxPacket::timestamp`].
    pub fn get_timestamp_for_id<'a, PKT>(&self, packet_id: PKT) -> Result<Timestamp, TimestampError>
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
        self.rx_ring.stop(&self.parts.eth_dma);
        self.tx_ring.stop(&self.parts.eth_dma);
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
    #[cfg(feature = "f-series")]
    let (is_rx, is_tx, is_error) = {
        // Read register
        let status = eth_dma.dmasr.read();

        // Reset bits
        eth_dma.dmasr.write(|w| {
            w.nis()
                .set_bit()
                .ts()
                .set_bit()
                .rs()
                .set_bit()
                .ais()
                .set_bit()
        });

        (
            status.rs().bit_is_set(),
            status.ts().bit_is_set(),
            status.ais().bit_is_set(),
        )
    };

    #[cfg(feature = "stm32h7xx-hal")]
    let (is_rx, is_tx, is_error) = {
        // Read register
        let status = eth_dma.dmacsr.read();

        // Reset bits
        eth_dma.dmacsr.write(|w| {
            w.nis()
                .set_bit()
                .ais()
                .set_bit()
                .ti()
                .set_bit()
                .ri()
                .set_bit()
                .rbu()
                .set_bit()
                .tbu()
                .set_bit()
        });

        (
            status.ri().bit_is_set(),
            status.ti().bit_is_set(),
            status.ais().bit_is_set(),
        )
    };

    let status = InterruptReasonSummary {
        is_rx,
        is_tx,
        is_error,
    };

    status
}
