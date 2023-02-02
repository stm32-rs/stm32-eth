use core::marker::PhantomData;

use super::{raw_descriptor::DescriptorRing, PacketId};
use crate::peripherals::ETHERNET_DMA;

#[cfg(feature = "ptp")]
use crate::{dma::TimestampError, ptp::Timestamp};

mod descriptor;
pub use descriptor::RxDescriptor;

/// Errors that can occur during RX
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub enum RxError {
    /// Receiving would block
    WouldBlock,
    /// The received packet was truncated
    Truncated,
    /// An error occured with the DMA
    DmaError,
}

/// An RX descriptor ring.
pub type RxDescriptorRing<'rx> = DescriptorRing<'rx, RxDescriptor>;

pub struct NotRunning;
pub struct Running;

/// Rx DMA state
pub struct RxRing<'data, STATE> {
    ring: RxDescriptorRing<'data>,
    next_entry: usize,
    state: PhantomData<STATE>,
}

impl<'data, STATE> RxRing<'data, STATE> {
    /// Demand that the DMA engine polls the current `RxDescriptor`
    /// (when in `RunningState::Stopped`.)
    pub fn demand_poll(&self, eth_dma: &ETHERNET_DMA) {
        eth_dma.dmarpdr.write(|w| unsafe { w.rpd().bits(1) });
    }

    /// Get current `RunningState`
    pub fn running_state(&self, eth_dma: &ETHERNET_DMA) -> RunningState {
        match eth_dma.dmasr.read().rps().bits() {
            //  Reset or Stop Receive Command issued
            0b000 => RunningState::Stopped,
            //  Fetching receive transfer descriptor
            0b001 => RunningState::Running,
            //  Waiting for receive packet
            0b011 => RunningState::Running,
            //  Receive descriptor unavailable
            0b100 => RunningState::Stopped,
            //  Closing receive descriptor
            0b101 => RunningState::Running,
            //  Transferring the receive packet data from receive buffer to host memory
            0b111 => RunningState::Running,
            _ => RunningState::Unknown,
        }
    }
}

impl<'data> RxRing<'data, NotRunning> {
    /// Allocate
    pub fn new(ring: RxDescriptorRing<'data>) -> Self {
        RxRing {
            ring,
            next_entry: 0,
            state: Default::default(),
        }
    }

    /// Start the RX ring
    pub fn start(mut self, eth_dma: &ETHERNET_DMA) -> RxRing<'data, Running> {
        // Setup ring
        for (entry, buffer) in self.ring.descriptors_and_buffers() {
            entry.setup(buffer);
        }

        self.ring
            .descriptors_and_buffers()
            .last()
            .map(|(desc, _)| desc.set_end_of_ring());

        self.next_entry = 0;
        let ring_ptr = self.ring.descriptors_start_address();

        // Set the RxDma ring start address.
        eth_dma
            .dmardlar
            .write(|w| unsafe { w.srl().bits(ring_ptr as u32) });

        // Start receive
        eth_dma.dmaomr.modify(|_, w| w.sr().set_bit());

        self.demand_poll(eth_dma);

        RxRing {
            ring: self.ring,
            next_entry: self.next_entry,
            state: Default::default(),
        }
    }
}

impl<'data> RxRing<'data, Running> {
    /// Stop the DMA engine.
    pub fn stop(&mut self, eth_dma: &ETHERNET_DMA) {
        eth_dma.dmaomr.modify(|_, w| w.sr().clear_bit());

        while self.running_state(eth_dma) != RunningState::Stopped {}
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(
        &mut self,
        eth_dma: &ETHERNET_DMA,
        #[allow(unused_variables)] packet_id: Option<PacketId>,
    ) -> Result<RxPacket, RxError> {
        if !self.running_state(eth_dma).is_running() {
            self.demand_poll(eth_dma);
        }

        let entries_len = self.ring.len();
        let (descriptor, buffer) = self.ring.get(self.next_entry);

        let mut res = descriptor.take_received(packet_id);

        if res.as_mut().err() != Some(&mut RxError::WouldBlock) {
            self.next_entry = (self.next_entry + 1) % entries_len;
        }

        res.map(move |_| {
            #[cfg(feature = "ptp")]
            let timestamp = descriptor.read_timestamp();

            RxPacket {
                entry: descriptor,
                buffer,
                #[cfg(feature = "ptp")]
                timestamp,
            }
        })
    }
}

#[cfg(feature = "ptp")]
impl<'data, STATE> RxRing<'data, STATE> {
    pub fn get_timestamp_for_id(&self, id: PacketId) -> Result<Timestamp, TimestampError> {
        for descriptor in self.ring.descriptors() {
            if let (Some(packet_id), Some(timestamp)) =
                (descriptor.packet_id(), descriptor.timestamp())
            {
                if packet_id == &id {
                    return Ok(timestamp.clone());
                }
            }
        }
        return Err(TimestampError::IdNotFound);
    }
}

/// Running state of the `RxRing`
#[derive(PartialEq, Eq, Debug)]
pub enum RunningState {
    Unknown,
    Stopped,
    Running,
}

impl RunningState {
    /// whether self equals to `RunningState::Running`
    pub fn is_running(&self) -> bool {
        *self == RunningState::Running
    }
}

/// A received packet.
///
/// This packet implements [Deref<\[u8\]>](core::ops::Deref) and should be used
/// as a slice.
pub struct RxPacket<'a> {
    entry: &'a mut RxDescriptor,
    buffer: &'a mut [u8],
    #[cfg(feature = "ptp")]
    timestamp: Option<Timestamp>,
}

impl<'a> core::ops::Deref for RxPacket<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer[0..self.entry.frame_length()]
    }
}

impl<'a> core::ops::DerefMut for RxPacket<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer[0..self.entry.frame_length()]
    }
}

impl<'a> Drop for RxPacket<'a> {
    fn drop(&mut self) {
        self.entry.set_owned();
    }
}

impl<'a> RxPacket<'a> {
    /// Pass the received packet back to the DMA engine.
    pub fn free(self) {
        drop(self)
    }

    /// Get the timestamp associated with this packet
    #[cfg(feature = "ptp")]
    pub fn timestamp(&self) -> Option<Timestamp> {
        self.timestamp
    }
}
