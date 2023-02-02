use core::sync::atomic::{self, Ordering};

use crate::dma::{raw_descriptor::RawDescriptor, PacketId};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

// Transmit end of ring
const TXDESC_0_TER: u32 = 1 << 21;
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
// NOTE(allow): packet_id is unused if ptp is disabled.
#[allow(dead_code)]
const TXDESC_0_TIMESTAMP_ENABLE: u32 = 1 << 25;
/// This descriptor contains a timestamp
// NOTE(allow): packet_id is unused if ptp is disabled.
#[allow(dead_code)]
const TXDESC_0_TIMESTAMP_STATUS: u32 = 1 << 17;
/// Error status
const TXDESC_0_ES: u32 = 1 << 15;
/// Transmit buffer 1 size
const TXDESC_1_TBS1_SHIFT: usize = 0;
/// Transmit buffer 1 size mask
const TXDESC_1_TBS1_MASK: u32 = 0x0fff << TXDESC_1_TBS1_SHIFT;
/// Transmit buffer 2 size
const TXDESC_1_TBS2_SHIFT: usize = 16;
/// Transmit buffer 2 size mask
const TXDESC_1_TBS2_MASK: u32 = 0x0fff << TXDESC_1_TBS2_SHIFT;

/// A TX DMA Ring Descriptor
#[repr(C)]
#[repr(align(4))]
#[derive(Clone, Copy)]
pub struct TxDescriptor {
    inner_raw: RawDescriptor,
    packet_id: Option<PacketId>,
    #[cfg(feature = "ptp")]
    cached_timestamp: Option<Timestamp>,
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
            inner_raw: RawDescriptor::new(),
            packet_id: None,
            #[cfg(feature = "ptp")]
            cached_timestamp: None,
        }
    }

    pub(super) fn setup(&mut self, buffer: &mut [u8]) {
        self.set_buffer(buffer);
        unsafe {
            self.inner_raw.write(
                0,
                TXDESC_0_CIC0 | TXDESC_0_CIC1 | TXDESC_0_FS | TXDESC_0_LS | TXDESC_0_IC,
            );
        }
    }

    #[allow(unused)]
    fn has_error(&self) -> bool {
        (self.inner_raw.read(0) & TXDESC_0_ES) == TXDESC_0_ES
    }

    /// Is owned by the DMA engine?
    pub fn is_owned(&self) -> bool {
        (self.inner_raw.read(0) & TXDESC_0_OWN) == TXDESC_0_OWN
    }

    // NOTE(allow): packet_id is unused if ptp is disabled.
    #[allow(dead_code)]
    fn is_last(tdes0: u32) -> bool {
        tdes0 & TXDESC_0_LS == TXDESC_0_LS
    }

    /// Pass ownership to the DMA engine
    pub(super) fn send(&mut self, packet_id: Option<PacketId>, buffer: &[u8]) {
        self.set_buffer(buffer);

        let extra_flags = if packet_id.is_some() {
            if cfg!(feature = "ptp") {
                TXDESC_0_TIMESTAMP_ENABLE
            } else {
                0
            }
        } else {
            0
        };

        self.packet_id = packet_id;

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::Release);
        atomic::compiler_fence(Ordering::Release);

        unsafe { self.inner_raw.modify(0, |w| w | extra_flags | TXDESC_0_OWN) };

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::SeqCst);
    }

    /// Configure the buffer to use for transmitting,
    /// setting it to `buffer`.
    fn set_buffer(&mut self, buffer: &[u8]) {
        unsafe {
            let ptr = buffer.as_ptr();

            // Set buffer pointer 2 to the provided buffer.
            self.inner_raw.write(3, ptr as u32);

            self.inner_raw.modify(1, |w| {
                // If we set tbs1 to 0, the DMA will
                // ignore this buffer.
                let w = w & !TXDESC_1_TBS1_MASK;
                // Configure RBS2 as the provided buffer.
                let w = w & !TXDESC_1_TBS2_MASK;
                w | ((buffer.len() as u32) << TXDESC_1_TBS2_SHIFT) & TXDESC_1_TBS2_MASK
            });
        }
    }

    // Set the end of ring bit.
    pub(super) fn set_end_of_ring(&mut self) {
        unsafe { self.inner_raw.modify(0, |w| w | TXDESC_0_TER) };
    }

    pub(super) fn packet_id(&self) -> Option<&PacketId> {
        self.packet_id.as_ref()
    }
}

#[cfg(feature = "ptp")]
impl TxDescriptor {
    fn read_timestamp(&mut self) -> Option<Timestamp> {
        let tdes0 = self.inner_raw.read(0);

        let contains_timestamp = (tdes0 & TXDESC_0_TIMESTAMP_STATUS) == TXDESC_0_TIMESTAMP_STATUS;

        if !self.is_owned() && contains_timestamp && Self::is_last(tdes0) {
            Timestamp::from_descriptor(&self.inner_raw)
        } else {
            None
        }
    }

    pub(super) fn attach_timestamp(&mut self) {
        self.cached_timestamp = self.read_timestamp();
    }

    pub(super) fn timestamp(&self) -> Option<&Timestamp> {
        self.cached_timestamp.as_ref()
    }
}

impl TxDescriptor {
    /// The initial value for a TxDescriptor
    pub const TX_INIT: Self = Self::new();

    pub(crate) fn prepare_packet(&mut self) -> bool {
        !self.is_owned()
    }
}
