use core::marker::PhantomData;

use super::{raw_descriptor::DescriptorRing, PacketId};
use crate::peripherals::ETHERNET_DMA;

#[cfg(feature = "ptp")]
use super::{Timestamp, TimestampError};

#[cfg(feature = "f-series")]
mod f_series_desc;
#[cfg(feature = "f-series")]
use f_series_desc as descriptor;

#[cfg(feature = "stm32h7xx-hal")]
mod h_desc;
#[cfg(feature = "stm32h7xx-hal")]
use h_desc as descriptor;

pub use descriptor::TxDescriptor;

pub struct NotRunning;
pub struct Running;

/// A TX descriptor ring.
pub type TxDescriptorRing<'rx> = DescriptorRing<'rx, TxDescriptor>;

/// Errors that can occur during Ethernet TX
#[derive(Debug, PartialEq)]
pub enum TxError {
    /// Ring buffer is full
    WouldBlock,
}

/// Tx DMA state
pub(crate) struct TxRing<'data, STATE> {
    ring: TxDescriptorRing<'data>,
    next_entry: usize,
    state: PhantomData<STATE>,
}

impl<'data, STATE> TxRing<'data, STATE> {
    pub fn running_state(&self, eth_dma: &ETHERNET_DMA) -> RunningState {
        #[cfg(feature = "f-series")]
        let tx_status = eth_dma.dmasr().read().tps().bits();

        #[cfg(feature = "stm32h7xx-hal")]
        let tx_status = eth_dma.dmadsr.read().tps0().bits();

        match tx_status {
            // Reset or Stop Transmit Command issued
            0b000 => RunningState::Stopped,
            // Fetching transmit transfer descriptor
            0b001 => RunningState::Running,
            // Waiting for status
            0b010 => RunningState::Running,
            // Reading Data from host memory buffer and queuing it to transmit buffer
            0b011 => RunningState::Running,

            0b101 => RunningState::Reserved,

            // Transmit descriptor unavailable
            0b110 => RunningState::Suspended,

            #[cfg(feature = "f-series")]
            0b100 => RunningState::Reserved,

            // Timestamp write
            #[cfg(feature = "stm32h7xx-hal")]
            0b100 => RunningState::Running,
            // Closing Tx descriptor
            #[cfg(feature = "stm32h7xx-hal")]
            0b111 => RunningState::Running,

            _ => RunningState::Unknown,
        }
    }
}

impl<'data> TxRing<'data, NotRunning> {
    /// Allocate
    ///
    /// `start()` will be needed before `send()`
    pub fn new(ring: TxDescriptorRing<'data>) -> Self {
        TxRing {
            ring,
            next_entry: 0,
            state: Default::default(),
        }
    }

    /// Start the Tx DMA engine
    pub fn start(mut self, eth_dma: &ETHERNET_DMA) -> TxRing<'data, Running> {
        // Setup ring
        for (descriptor, buffer) in self.ring.descriptors_and_buffers() {
            descriptor.setup(buffer);
        }

        #[cfg(feature = "f-series")]
        // Set end of ring register
        self.ring.last_descriptor().set_end_of_ring();

        let ring_ptr = self.ring.descriptors_start_address();

        #[cfg(feature = "f-series")]
        // Register TxDescriptor
        eth_dma
            .dmatdlar
            // Note: unsafe block required for `stm32f107`.
            .write(|w| unsafe { w.stl().bits(ring_ptr as u32) });

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // TODO: assert that ethernet DMA can access
            // the memory in these rings
            assert!(self.ring.descriptors().count() >= 4);

            // Assert that the descriptors are properly aligned.
            assert!(ring_ptr as u32 & !0b11 == ring_ptr as u32);
            assert!(
                self.ring.last_descriptor_mut() as *const _ as u32 & !0b11
                    == self.ring.last_descriptor_mut() as *const _ as u32
            );

            // Set the start pointer.
            eth_dma
                .dmactx_dlar
                .write(|w| unsafe { w.bits(ring_ptr as u32) });

            // Set the Transmit Descriptor Ring Length
            eth_dma.dmactx_rlr.write(|w| {
                w.tdrl()
                    .variant((self.ring.descriptors().count() - 1) as u16)
            });

            // Set the tail pointer
            eth_dma
                .dmactx_dtpr
                .write(|w| unsafe { w.bits(self.ring.last_descriptor_mut() as *const _ as u32) });
        }

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);

        // We don't need a compiler fence here because all interactions with `Descriptor` are
        // volatiles

        #[cfg(feature = "f-series")]
        let start_reg = &eth_dma.dmaomr;
        #[cfg(feature = "stm32h7xx-hal")]
        let start_reg = &eth_dma.dmactx_cr;

        // Start transmission
        start_reg.modify(|_, w| w.st().set_bit());

        TxRing {
            ring: self.ring,
            next_entry: self.next_entry,
            state: Default::default(),
        }
    }
}

impl<'data> TxRing<'data, Running> {
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
        f: F,
    ) -> Result<R, TxError> {
        let entries_len = self.ring.len();
        let entry_num = self.next_entry;

        let (descriptor, buffer) = self.ring.get(entry_num);

        assert!(length <= buffer.len());

        if descriptor.prepare_packet() {
            let r = f(&mut buffer[0..length]);

            descriptor.send(packet_id, &buffer[0..length]);

            self.next_entry = (self.next_entry + 1) % entries_len;

            Ok(r)
        } else {
            Err(TxError::WouldBlock)
        }
    }

    /// Demand that the DMA engine polls the current `TxDescriptor`
    /// (when we just transferred ownership to the hardware).
    pub fn demand_poll(&self, eth_dma: &ETHERNET_DMA) {
        #[cfg(feature = "stm32h7xx-hal")]
        // To issue a poll demand, write a value to
        // the tail pointer. We just re-write the
        // current value.
        eth_dma
            .dmactx_dtpr
            .modify(|r, w| unsafe { w.bits(r.bits()) });

        #[cfg(feature = "f-series")]
        eth_dma.dmatpdr.write(|w| {
            #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
            {
                w.tpd().poll()
            }
            #[cfg(feature = "stm32f1xx-hal")]
            unsafe {
                // TODO: There is no nice `poll` method for `stm32f107`?
                w.tpd().bits(0)
            }
        });
    }

    /// Is the Tx DMA engine running?
    pub fn is_running(&self, eth_dma: &ETHERNET_DMA) -> bool {
        self.running_state(eth_dma).is_running()
    }

    pub fn stop(&mut self, eth_dma: &ETHERNET_DMA) {
        #[cfg(feature = "f-series")]
        let start_reg = &eth_dma.dmaomr;
        #[cfg(feature = "stm32h7xx-hal")]
        let start_reg = &eth_dma.dmactx_cr;

        // Start transmission
        start_reg.modify(|_, w| w.st().clear_bit());

        while self.running_state(eth_dma) != RunningState::Stopped {}
    }
}

#[cfg(feature = "ptp")]
impl<'data> TxRing<'data, Running> {
    pub(crate) fn collect_timestamps(&mut self) {
        for descriptor in self.ring.descriptors_mut() {
            f_descriptor.attach_timestamp();
        }
    }

    pub(crate) fn get_timestamp_for_id(&self, id: PacketId) -> Result<Timestamp, TimestampError> {
        let descriptor = if let Some(descriptor) =
            self.ring.descriptors().find(|d| d.packet_id() == Some(&id))
        {
            f_descriptor
        } else {
            return Err(TimestampError::IdNotFound);
        };

        f_descriptor
            .timestamp()
            .map(|t| *t)
            .ok_or(TimestampError::NotYetTimestamped)
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub enum RunningState {
    /// Reset or Stop Transmit Command issued
    Stopped,
    /// Fetching transmit transfer descriptor;
    /// Waiting for status;
    /// Reading Data from host memory buffer and queuing it to transmit buffer
    Running,
    /// Reserved for future use
    Reserved,
    /// Transmit descriptor unavailable
    Suspended,
    /// Invalid value
    Unknown,
}

impl RunningState {
    pub fn is_running(&self) -> bool {
        self == &RunningState::Running
    }
}

pub struct TxPacket<'a> {
    buffer: &'a mut [u8],
}

impl<'a> core::ops::Deref for TxPacket<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer
    }
}

impl<'a> core::ops::DerefMut for TxPacket<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer
    }
}
