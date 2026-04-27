pub(crate) use self::descriptor::RxDescriptor;

use self::descriptor::RxDescriptorError;
pub use self::descriptor::RxRingEntry;

use super::PacketId;
use crate::peripherals::ETHERNET_DMA;

mod descriptor;
pub use descriptor::RxPacket;

#[cfg(feature = "ptp")]
use crate::{dma::PacketIdNotFound, ptp::Timestamp};

#[cfg(feature = "async-await")]
use core::task::Poll;

/// Errors that can occur during RX
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub enum RxError {
    /// The received packet was truncated
    Truncated,
    /// An error occured with the DMA
    DmaError,
    /// Receiving would block
    WouldBlock,
}

impl From<RxDescriptorError> for RxError {
    fn from(value: RxDescriptorError) -> Self {
        match value {
            RxDescriptorError::Truncated => Self::Truncated,
            RxDescriptorError::DmaError => Self::DmaError,
        }
    }
}

/// Rx DMA state
pub struct RxRing<'a> {
    entries: &'a mut [RxRingEntry],
    next_entry: usize,
}

impl<'a> RxRing<'a> {
    /// Allocate
    pub(crate) fn new(eth_dma: &ETHERNET_DMA, entries: &'a mut [RxRingEntry]) -> Self {
        let mut previous: Option<&mut RxRingEntry> = None;

        for entry in entries.iter_mut() {
            if let Some(prev_entry) = &mut previous {
                prev_entry.setup(Some(entry));
            }
            previous = Some(entry);
        }
        if let Some(entry) = &mut previous {
            entry.setup(None);
        }

        let ring_ptr = entries[0].desc() as *const RxDescriptor;

        // Register RxDescriptor
        eth_dma
            .dmardlar
            .write(|w| unsafe { w.srl().bits(ring_ptr as u32) });

        // We already have fences in `set_owned`, which is called in `setup`

        // Start receive
        eth_dma.dmaomr.modify(|_, w| w.sr().set_bit());

        RxRing {
            entries,
            next_entry: 0,
        }
    }

    /// Stop the RX DMA
    pub(crate) fn stop(&self, eth_dma: &ETHERNET_DMA) {
        eth_dma.dmaomr.modify(|_, w| w.sr().clear_bit());

        // DMA accesses do not stop before the running state
        // of the DMA has changed to something other than
        // running.
        while self.running_state().is_running() {}
    }

    /// Demand that the DMA engine polls the current `RxDescriptor`
    /// (when in [`RunningState::Stopped`].)
    fn demand_poll(&self) {
        // SAFETY: we only perform an atomic write to `dmarpdr`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };
        eth_dma.dmarpdr.write(|w| unsafe { w.rpd().bits(1) });
    }

    /// Get current `RunningState`
    pub fn running_state(&self) -> RunningState {
        // SAFETY: we only perform an atomic read of `dmasr`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };
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

    /// Check if we can receive a new packet
    pub fn next_entry_available(&self) -> bool {
        if !self.running_state().is_running() {
            self.demand_poll();
        }

        self.entries[self.next_entry].is_available()
    }

    /// Receive the next packet (if any is ready).
    fn recv_next_impl(
        &mut self,
        // NOTE(allow): packet_id is unused if ptp is disabled.
        #[allow(unused_variables)] packet_id: Option<PacketId>,
    ) -> Result<RxPacket<'_>, RxError> {
        if !self.running_state().is_running() {
            self.demand_poll();
        }

        let entries_len = self.entries.len();
        let entry_num = self.next_entry;
        let entry = &mut self.entries[entry_num];

        if let Some(entry) = entry.as_available() {
            self.next_entry = (self.next_entry + 1) % entries_len;
            let packet = entry.recv(packet_id)?;
            Ok(packet)
        } else {
            Err(RxError::WouldBlock)
        }
    }

    /// Receive the next packet (if any is ready), or return [`Err`]
    /// immediately.
    pub fn recv_next(&'_ mut self, packet_id: Option<PacketId>) -> Result<RxPacket<'_>, RxError> {
        self.recv_next_impl(packet_id.map(|p| p.into()))
    }

    /// Receive the next packet.
    ///
    /// The returned [`RxPacket`] can be used as a slice, and
    /// will contain the ethernet data.
    #[cfg(feature = "async-await")]
    pub async fn recv(&'_ mut self, packet_id: Option<PacketId>) -> RxPacket<'_> {
        core::future::poll_fn(|ctx| {
            if self.next_entry_available() {
                Poll::Ready(())
            } else {
                crate::dma::EthernetDMA::rx_waker().register(ctx.waker());
                Poll::Pending
            }
        })
        .await;

        self.recv_next_impl(packet_id).unwrap()
    }
}

#[cfg(feature = "ptp")]
impl<'a> RxRing<'a> {
    /// Get the timestamp for a specific ID
    pub fn timestamp(&self, id: &PacketId) -> Result<Option<Timestamp>, PacketIdNotFound> {
        let entry = self.entries.iter().find(|e| e.desc().has_packet_id(id));

        let entry = entry.ok_or(PacketIdNotFound)?;

        Ok(entry.desc().read_timestamp())
    }
}

/// Running state of the `RxRing`
#[derive(PartialEq, Eq, Debug)]
pub enum RunningState {
    /// Running state is unknown.
    Unknown,
    /// The RX DMA is stopped.
    Stopped,
    /// The RX DMA is running.
    Running,
}

impl RunningState {
    /// whether self equals to `RunningState::Running`
    pub fn is_running(&self) -> bool {
        *self == RunningState::Running
    }
}
