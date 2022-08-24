use crate::{
    dma::{Timestamp, TimestampError},
    stm32::ETHERNET_DMA,
    PacketId,
};

use core::{
    ops::{Deref, DerefMut},
    sync::atomic::{self, Ordering},
};

use crate::{
    desc::Descriptor,
    ring::{RingDescriptor, RingEntry},
};

/// Owned by DMA engine
const TXDESC_0_OWN: u32 = 1 << 31;
/// Interrupt on completion
const TXDESC_0_IC: u32 = 1 << 30;
/// First segment of frame
const TXDESC_0_FS: u32 = 1 << 28;
/// Last segment of frame
const TXDESC_0_LS: u32 = 1 << 29;
/// Checksum insertion control
const TXDESC_0_CIC0: u32 = 1 << 23;
const TXDESC_0_CIC1: u32 = 1 << 22;
/// Timestamp this packet
const TXDESC_0_TIMESTAMP_ENABLE: u32 = 1 << 25;
/// This descriptor contains a timestamp
const TXDESC_0_TIMESTAMP_STATUS: u32 = 1 << 17;
/// Transmit end of ring
const TXDESC_0_TER: u32 = 1 << 21;
/// Second address chained
const TXDESC_0_TCH: u32 = 1 << 20;
/// Error status
const TXDESC_0_ES: u32 = 1 << 15;
/// TX done bit

const TXDESC_1_TBS_SHIFT: usize = 0;
const TXDESC_1_TBS_MASK: u32 = 0x0fff << TXDESC_1_TBS_SHIFT;

/// Errors that can occur during Ethernet TX
#[derive(Debug, PartialEq)]
pub enum TxError {
    /// Ring buffer is full
    WouldBlock,
}

/// A TX DMA Ring Descriptor
#[repr(C)]
#[derive(Clone)]
pub struct TxDescriptor {
    pub(crate) desc: Descriptor,
    pub(crate) packet_id: Option<PacketId>,
    cached_timestamp: Option<Timestamp>,
    buffer_address: Option<u32>,
    next_descriptor: Option<u32>,
}

impl Default for TxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl TxDescriptor {
    /// Creates an zeroed TxDescriptor.
    pub const fn new() -> Self {
        Self {
            desc: Descriptor::new(),
            packet_id: None,
            cached_timestamp: None,
            buffer_address: None,
            next_descriptor: None,
        }
    }

    /// Is owned by the DMA engine?
    pub(crate) fn is_owned(&self) -> bool {
        (self.desc.read(0) & TXDESC_0_OWN) == TXDESC_0_OWN
    }

    /// Pass ownership to the DMA engine
    fn set_owned(&mut self) {
        self.write_buffer1();
        self.write_buffer2();

        // "Preceding reads and writes cannot be moved past subsequent writes."
        // #[cfg(feature = "fence")]
        atomic::fence(Ordering::Release);
        atomic::compiler_fence(Ordering::Release);
        unsafe {
            self.desc.modify(0, |w| w | TXDESC_0_OWN);
        }

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        // #[cfg(feature = "fence")]
        atomic::fence(Ordering::SeqCst);
    }

    #[allow(unused)]
    fn has_error(&self) -> bool {
        (self.desc.read(0) & TXDESC_0_ES) == TXDESC_0_ES
    }

    /// Rewrite buffer1 to the last value we wrote to it
    ///
    /// In our case, the address of the data buffer for this descriptor
    fn write_buffer1(&mut self) {
        let buffer_addr = self
            .buffer_address
            .expect("Writing buffer2 of a TX descriptor, but `buffer_address` is None");

        unsafe {
            self.desc.write(2, buffer_addr);
        }
    }

    fn set_buffer1(&mut self, buffer: *const u8) {
        self.buffer_address = Some(buffer as u32);
        self.write_buffer1();
    }

    fn set_buffer1_len(&mut self, len: usize) {
        unsafe {
            self.desc.modify(1, |w| {
                (w & !TXDESC_1_TBS_MASK) | ((len as u32) << TXDESC_1_TBS_SHIFT)
            });
        }
    }

    /// Rewrite buffer2 to the last value we wrote it to
    ///
    /// In our case, the address of the next descriptor (may be zero)
    fn write_buffer2(&mut self) {
        let next_descriptor = self
            .next_descriptor
            .expect("Writing buffer2 of a TX descriptor, but `next_descriptor` is None");
        unsafe {
            self.desc.write(3, next_descriptor);
        }
    }

    // points to next descriptor (RCH)
    fn set_buffer2(&mut self, buffer: *const u8) {
        self.next_descriptor = Some(buffer as u32);
        self.write_buffer2();
    }

    fn set_end_of_ring(&mut self) {
        unsafe {
            self.desc.modify(0, |w| w | TXDESC_0_TER);
        }
    }

    fn is_last(tdes0: u32) -> bool {
        tdes0 & TXDESC_0_LS == TXDESC_0_LS
    }

    fn timestamp(&mut self) -> Option<Timestamp> {
        let tdes0 = self.desc.read(0);

        let contains_timestamp = (tdes0 & TXDESC_0_TIMESTAMP_STATUS) == TXDESC_0_TIMESTAMP_STATUS;

        if !self.is_owned() && contains_timestamp && Self::is_last(tdes0) {
            #[cfg(not(feature = "stm32f107"))]
            let (seconds, subseconds) = { (self.desc.read(7), self.desc.read(6)) };
            #[cfg(feature = "stm32f107")]
            let (seconds, subseconds) = { (self.desc.read(3), self.desc.read(2)) };

            let timestamp = Timestamp::new(seconds, subseconds);

            // Clear the timestamp status bit, because we're currently reading it
            unsafe {
                self.desc.write(0, tdes0 & !(TXDESC_0_TIMESTAMP_STATUS));
            }

            Some(timestamp)
        } else {
            None
        }
    }

    /// Enable PTP timestamping
    fn set_timestamping(&mut self) {
        unsafe {
            self.desc.modify(0, |w| {
                w | TXDESC_0_TIMESTAMP_ENABLE | TXDESC_0_FS | TXDESC_0_LS
            });
        }
    }

    /// Enable TX interrupt
    fn set_interrupt(&mut self) {
        unsafe {
            self.desc.modify(0, |w| w | TXDESC_0_IC);
        }
    }
}

/// A TX DMA Ring Descriptor entry
pub type TxRingEntry = RingEntry<TxDescriptor>;

impl RingDescriptor for TxDescriptor {
    fn setup(&mut self, buffer: *const u8, _len: usize, next: Option<&Self>) {
        // Defer this initialization to this function, so we can have `RingEntry` on bss.
        unsafe {
            self.desc.write(
                0,
                TXDESC_0_TCH
                    | TXDESC_0_IC
                    | TXDESC_0_FS
                    | TXDESC_0_LS
                    | TXDESC_0_CIC0
                    | TXDESC_0_CIC1,
            );
        }
        self.set_buffer1(buffer);
        match next {
            Some(next) => self.set_buffer2(&next.desc as *const Descriptor as *const u8),
            None => {
                #[allow(clippy::zero_ptr)]
                self.set_buffer2(0 as *const u8);
                self.set_end_of_ring();
            }
        };
    }
}

impl TxRingEntry {
    /// The initial value for a TxRingEntry
    pub const TX_INIT: Self = Self::new();

    fn prepare_packet(&mut self, length: usize, packet_id: Option<PacketId>) -> Option<TxPacket> {
        assert!(length <= self.as_slice().len());

        if !self.desc().is_owned() {
            self.desc_mut().set_buffer1_len(length);

            if packet_id.is_some() {
                self.desc_mut().set_timestamping();
            }
            self.desc_mut().packet_id = packet_id;

            self.desc_mut().set_interrupt();

            Some(TxPacket {
                entry: self,
                length,
            })
        } else {
            None
        }
    }
}

pub struct TxPacket<'a> {
    entry: &'a mut TxRingEntry,
    length: usize,
}

impl<'a> Deref for TxPacket<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.entry.as_slice()[0..self.length]
    }
}

impl<'a> DerefMut for TxPacket<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.entry.as_mut_slice()[0..self.length]
    }
}

impl<'a> TxPacket<'a> {
    // Pass to DMA engine
    pub fn send(self) {
        self.entry.desc_mut().set_owned();
    }
}

/// Tx DMA state
pub struct TxRing<'a> {
    pub(crate) entries: &'a mut [TxRingEntry],
    next_entry: usize,
}

impl<'a> TxRing<'a> {
    pub fn collect_timestamps(&mut self) {
        for entry in self.entries.iter_mut() {
            // Clear all old timestamps
            entry.desc_mut().cached_timestamp.take();

            if entry.desc().packet_id.is_some() {
                if let Some(timestamp) = entry.desc_mut().timestamp() {
                    entry.desc_mut().cached_timestamp = Some(timestamp);
                }
            }
        }
    }

    pub fn get_timestamp_for_id(&mut self, id: PacketId) -> Result<Timestamp, TimestampError> {
        let mut id_found = false;
        for entry in self.entries.iter_mut() {
            let TxDescriptor {
                cached_timestamp: timestamp,
                packet_id,
                ..
            } = entry.desc_mut();

            if let Some(packet_id) = packet_id {
                if packet_id == &id {
                    id_found = true;
                    if let Some(timestamp) = timestamp {
                        let ts = *timestamp;
                        entry.desc_mut().cached_timestamp.take();
                        return Ok(ts);
                    }
                }
            }
        }

        if !id_found {
            Err(TimestampError::IdNotFound)
        } else {
            Err(TimestampError::NotYetTimestamped)
        }
    }

    /// Allocate
    ///
    /// `start()` will be needed before `send()`
    pub fn new(entries: &'a mut [TxRingEntry]) -> Self {
        TxRing {
            entries,
            next_entry: 0,
        }
    }

    /// Start the Tx DMA engine
    pub fn start(&mut self, eth_dma: &ETHERNET_DMA) {
        // Setup ring
        {
            let mut previous: Option<&mut TxRingEntry> = None;
            for entry in self.entries.iter_mut() {
                if let Some(prev_entry) = &mut previous {
                    prev_entry.setup(Some(entry));
                }
                previous = Some(entry);
            }
            if let Some(entry) = &mut previous {
                entry.setup(None);
            }
        }

        let ring_ptr = self.entries[0].desc() as *const TxDescriptor;
        // Register TxDescriptor
        eth_dma
            .dmatdlar
            // Note: unsafe block required for `stm32f107`.
            .write(|w| unsafe { w.stl().bits(ring_ptr as u32) });

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::Release);

        // We don't need a compiler fence here because all interactions with `Descriptor` are
        // volatiles

        // Start transmission
        eth_dma.dmaomr.modify(|_, w| w.st().set_bit());
    }

    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
        f: F,
    ) -> Result<R, TxError> {
        let entries_len = self.entries.len();
        let entry_num = self.next_entry;

        match self.entries[entry_num].prepare_packet(length, packet_id) {
            Some(mut pkt) => {
                let r = f(pkt.deref_mut());
                pkt.send();

                self.next_entry += 1;
                if self.next_entry >= entries_len {
                    self.next_entry = 0;
                }
                Ok(r)
            }
            None => Err(TxError::WouldBlock),
        }
    }

    /// Demand that the DMA engine polls the current `TxDescriptor`
    /// (when we just transferred ownership to the hardware).
    pub fn demand_poll(&self, eth_dma: &ETHERNET_DMA) {
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

    fn running_state(&self, eth_dma: &ETHERNET_DMA) -> RunningState {
        match eth_dma.dmasr.read().tps().bits() {
            // Reset or Stop Transmit Command issued
            0b000 => RunningState::Stopped,
            // Fetching transmit transfer descriptor
            0b001 => RunningState::Running,
            // Waiting for status
            0b010 => RunningState::Running,
            // Reading Data from host memory buffer and queuing it to transmit buffer
            0b011 => RunningState::Running,
            0b100 | 0b101 => RunningState::Reserved,
            // Transmit descriptor unavailable
            0b110 => RunningState::Suspended,
            _ => RunningState::Unknown,
        }
    }
}

#[derive(Debug, PartialEq)]
enum RunningState {
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
        *self == RunningState::Running
    }
}
