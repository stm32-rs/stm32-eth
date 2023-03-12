//! Ethernet DMA access and configuration.

use cortex_m::peripheral::NVIC;

use crate::{peripherals::ETHERNET_DMA, stm32::Interrupt};

mod cache;
pub(crate) use cache::Cache;

#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::*;

#[cfg(feature = "async-await")]
use futures::task::AtomicWaker;

#[cfg(any(feature = "ptp", feature = "async-await"))]
use core::task::Poll;

pub(crate) mod generic_ring;

mod rx;
pub use rx::{
    RunningState as RxRunningState, RxDescriptor, RxDescriptorRing, RxError, RxPacket, RxRing,
};

mod tx;
pub use tx::{
    RunningState as TxRunningState, TxDescriptor, TxDescriptorRing, TxError, TxPacket, TxRing,
};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

mod packet_id;
pub use packet_id::PacketId;

const _RXDESC_SIZE: usize = core::mem::size_of::<RxDescriptor>();
const _TXDESC_SIZE: usize = core::mem::size_of::<TxDescriptor>();

/// Assert that our descriptors have the same size.
///
/// This is necessary as we only have a single Descriptor Skip Length
/// value which applies to both TX and RX descriptors.
const _ASSERT_DESCRIPTOR_SIZES: () = assert!(_RXDESC_SIZE == _TXDESC_SIZE);

const _ASSERT_DESCRIPTOR_ALIGN: () = assert!(_RXDESC_SIZE % 4 == 0);

const DESC_WORD_SKIP: u8 = ((_RXDESC_SIZE / 4) - self::generic_ring::DESC_SIZE) as u8;

const _ASSERT_DESC_WORD_SKIP_SIZE: () = assert!(DESC_WORD_SKIP <= 0b111);

/// From the datasheet: *VLAN Frame maxsize = 1522*
pub const MTU: usize = 1522;

pub(crate) struct DmaParts {
    pub eth_dma: ETHERNET_DMA,
    #[cfg(feature = "stm32h7xx-hal")]
    pub eth_mtl: crate::stm32::ETHERNET_MTL,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
/// This struct is returned if a packet ID is not associated
/// with any TX or RX descriptors.
pub struct PacketIdNotFound;

/// Ethernet DMA.
pub struct EthernetDMA<'rx, 'tx> {
    parts: DmaParts,
    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx>,
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

        let mut dma = EthernetDMA {
            parts,
            rx_ring: RxRing::new(rx_buffer),
            tx_ring: TxRing::new(tx_buffer),
        };

        dma.rx_ring.start(&dma.parts.eth_dma);
        dma.tx_ring.start(&dma.parts.eth_dma);

        dma
    }

    /// Split the [`EthernetDMA`] into concurrently operating send and
    /// receive parts.
    pub fn split(&mut self) -> (&mut RxRing<'rx>, &mut TxRing<'tx>) {
        (&mut self.rx_ring, &mut self.tx_ring)
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`EthernetDMA::interrupt_handler()`] or [`stm32_eth::eth_interrupt_handler`](crate::eth_interrupt_handler)
    /// to clear interrupt pending bits. Otherwise the interrupt will reoccur immediately.
    ///
    /// [`EthernetPTP::interrupt_handler()`]: crate::ptp::EthernetPTP::interrupt_handler
    #[cfg_attr(
        feature = "ptp",
        doc = "If you have PTP enabled, you must also call [`EthernetPTP::interrupt_handler()`] if you wish to make use of the PTP timestamp trigger feature."
    )]
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

    #[cfg(feature = "stm32h7xx-hal")]
    fn panic_fbe() -> ! {
        // SAFETY: we only perform atomic reads/writes through `eth_dma`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        let tx_descriptor_addr = eth_dma.dmaccatx_dr.read().bits();
        let tx_buffer_addr = eth_dma.dmaccatx_br.read().bits();

        let rx_descriptor_addr = eth_dma.dmaccarx_dr.read().bits();
        let rx_buffer_addr = eth_dma.dmaccarx_br.read().bits();

        // TODO: add a link to a/the github issue describing this problem,
        // and how to solve it.
        panic!("Fatal bus error! Is the descriptor and buffer memory accessible by the Ethernet MAC/DMA? TXDESC: {:08X}, TXBUF: {:08X}, RXDESC: {:08X}, TXDESC: {:08X}", tx_descriptor_addr, tx_buffer_addr, rx_descriptor_addr, rx_buffer_addr);
    }

    /// Handle the DMA parts of the `ETH` interrupt.
    pub(crate) fn interrupt_handler() -> InterruptReasonSummary {
        // SAFETY: we only perform atomic reads/writes through `eth_dma`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

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
                    .eti()
                    .set_bit()
                    .eri()
                    .set_bit()
            });

            if status.fbe().bit_is_set() {
                EthernetDMA::panic_fbe();
            }

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

        #[cfg(feature = "async-await")]
        {
            if status.is_tx {
                EthernetDMA::tx_waker().wake();
            }

            if status.is_rx {
                EthernetDMA::rx_waker().wake();
            }
        }

        status
    }

    /// Try to receive a packet.
    ///
    /// If no packet is available, this function returns [`Err(RxError::WouldBlock)`](RxError::WouldBlock).
    ///
    /// It may also return another kind of [`RxError`].
    pub fn recv_next(&mut self, packet_id: Option<PacketId>) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(packet_id.map(Into::into))
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call [`EthernetDMA::recv_next()`] to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        RxRing::running_state().is_running()
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        TxRing::is_running()
    }

    /// Try to send a packet with data.
    ///
    /// If there are no free TX slots, this function will
    /// return [`Err(TxError::WouldBlock)`](TxError::WouldBlock).
    pub fn send<F>(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
        f: F,
    ) -> Result<(), TxError>
    where
        F: FnOnce(&mut [u8]),
    {
        let mut tx_packet = self.tx_ring.send_next(length, packet_id)?;
        f(&mut tx_packet);
        tx_packet.send();

        Ok(())
    }

    /// Check if there is a packet available for reading.
    ///
    /// If this function returns true, it is guaranteed that the
    /// next call to [`EthernetDMA::recv_next`] will return [`Ok`].
    pub fn rx_available(&mut self) -> bool {
        self.rx_ring.next_entry_available()
    }

    /// Check if sending a packet now would succeed.
    ///
    /// If this function returns true, it is guaranteed that
    /// the next call to [`EthernetDMA::send`] will return [`Ok`]
    pub fn tx_available(&mut self) -> bool {
        self.tx_ring.next_entry_available()
    }
}

impl Drop for EthernetDMA<'_, '_> {
    // On drop, stop all DMA actions.
    fn drop(&mut self) {
        self.tx_ring.stop(self.eth_dma());
        self.rx_ring.stop(self.eth_dma());
    }
}

#[cfg(feature = "async-await")]
impl<'rx, 'tx> EthernetDMA<'rx, 'tx> {
    pub(crate) fn rx_waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }

    pub(crate) fn tx_waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }

    /// Receive a packet.
    ///
    /// See [`RxRing::recv`].
    pub async fn recv(&mut self, packet_id: Option<PacketId>) -> RxPacket {
        self.rx_ring.recv(packet_id).await
    }

    /// Prepare a packet for sending.
    ///
    /// See [`TxRing::prepare_packet`].
    pub async fn prepare_packet(&mut self, length: usize, packet_id: Option<PacketId>) -> TxPacket {
        self.tx_ring.prepare_packet(length, packet_id).await
    }

    /// Wait for an RX or TX interrupt to have
    /// occured.
    pub async fn rx_or_tx(&mut self) {
        let mut polled_once = false;
        core::future::poll_fn(|ctx| {
            if polled_once {
                Poll::Ready(())
            } else {
                polled_once = true;
                EthernetDMA::rx_waker().register(ctx.waker());
                EthernetDMA::tx_waker().register(ctx.waker());
                Poll::Pending
            }
        })
        .await;
    }
}

#[cfg(feature = "ptp")]
impl EthernetDMA<'_, '_> {
    /// Try to get the timestamp for the given packet ID.
    ///
    /// This function will attempt to find both RX and TX timestamps,
    /// so make sure that the provided packet ID is unique between the two.
    pub fn poll_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Poll<Result<Option<Timestamp>, PacketIdNotFound>> {
        // Check if it's a TX packet
        let tx = self.poll_tx_timestamp(packet_id);

        if tx != Poll::Ready(Err(PacketIdNotFound)) {
            return tx;
        }

        // It's not a TX packet, check if it's an RX packet
        Poll::Ready(self.rx_timestamp(packet_id))
    }

    /// Get the RX timestamp for the given packet ID.
    pub fn rx_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        self.rx_ring.timestamp(packet_id)
    }

    /// Blockingly wait until the TX timestamp for
    /// the given ID is available.
    pub fn wait_for_tx_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        self.tx_ring.wait_for_timestamp(packet_id)
    }

    /// Poll to check if the TX timestamp for the given
    /// ID is available.
    pub fn poll_tx_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Poll<Result<Option<Timestamp>, PacketIdNotFound>> {
        self.tx_ring.poll_timestamp(packet_id)
    }

    /// Get the TX timestamp for the given ID.
    #[cfg(feature = "async-await")]
    pub async fn tx_timestamp(
        &mut self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        self.tx_ring.timestamp(packet_id).await
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
