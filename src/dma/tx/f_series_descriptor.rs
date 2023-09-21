use crate::dma::{generic_ring::RawDescriptor, Cache, PacketId};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

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
// NOTE(allow): packet_id is unused if ptp is disabled.
#[allow(dead_code)]
const TXDESC_0_TIMESTAMP_STATUS: u32 = 1 << 17;
/// Transmit end of ring
const TXDESC_0_TER: u32 = 1 << 21;
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
#[derive(Clone, Copy)]
pub struct TxDescriptor {
    desc: RawDescriptor,
    last: bool,
    cache: Cache,
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
            desc: RawDescriptor::new(),
            last: false,
            cache: Cache::new(),
        }
    }

    pub(super) fn setup(&mut self) {
        (0..crate::dma::generic_ring::DESC_SIZE).for_each(|i| unsafe { self.desc.write(i, 0) });
    }

    #[allow(unused)]
    pub(super) fn has_error(&self) -> bool {
        (self.desc.read(0) & TXDESC_0_ES) == TXDESC_0_ES
    }

    /// Is owned by the DMA engine?
    pub(super) fn is_owned(&self) -> bool {
        (self.desc.read(0) & TXDESC_0_OWN) == TXDESC_0_OWN
    }

    pub(super) fn is_available(&self) -> bool {
        !self.is_owned()
    }

    // NOTE(allow): packet_id is unused if ptp is disabled.
    #[allow(dead_code)]
    pub(super) fn is_last(&self) -> bool {
        self.desc.read(0) & TXDESC_0_LS == TXDESC_0_LS
    }

    pub(crate) fn send(&mut self, packet_id: Option<PacketId>, buffer: &[u8]) {
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

        let extra_flags = if self.last {
            extra_flags | TXDESC_0_TER
        } else {
            extra_flags
        };

        self.cache.set_id_and_clear_ts(packet_id);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);

        unsafe {
            self.desc.modify(0, |w| {
                w | extra_flags
                    | TXDESC_0_OWN
                    | TXDESC_0_CIC0
                    | TXDESC_0_CIC1
                    | TXDESC_0_FS
                    | TXDESC_0_LS
                    | TXDESC_0_IC
            })
        };

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    }

    /// Configure the buffer to use for transmitting,
    /// setting it to `buffer`.
    fn set_buffer(&mut self, buffer: &[u8]) {
        unsafe {
            let ptr = buffer.as_ptr();

            // Set buffer pointer 2 to the provided buffer.
            self.desc.write(3, ptr as u32);

            self.desc.modify(1, |w| {
                // If we set tbs1 to 0, the DMA will
                // ignore this buffer.
                let w = w & !TXDESC_1_TBS1_MASK;
                // Configure RBS2 as the provided buffer.
                let w = w & !TXDESC_1_TBS2_MASK;
                w | ((buffer.len() as u32) << TXDESC_1_TBS2_SHIFT) & TXDESC_1_TBS2_MASK
            });
        }
    }

    pub(crate) fn set_end_of_ring(&mut self) {
        self.last = true;
    }
}

#[cfg(feature = "ptp")]
impl TxDescriptor {
    pub(super) fn has_packet_id(&self, packet_id: &PacketId) -> bool {
        self.cache.id().as_ref() == Some(packet_id)
    }

    /// For the TxDescriptor we ignore [`Cache::ts`] because:
    /// * We're only really using the cache so that the size of RxDescriptor and TxDescriptor
    ///   is the same.
    /// * We want to be able to retrieve the timestamp immutably.
    /// * The Timestamp in the TX descriptor is valid until we perform another transmission.
    pub(super) fn timestamp(&self) -> Option<Timestamp> {
        let tdes0 = self.desc.read(0);

        let contains_timestamp = (tdes0 & TXDESC_0_TIMESTAMP_STATUS) == TXDESC_0_TIMESTAMP_STATUS;

        if !self.is_owned() && contains_timestamp && self.is_last() {
            #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
            let (high, low) = { (self.desc.read(7), self.desc.read(6)) };

            #[cfg(feature = "stm32f1xx-hal")]
            let (high, low) = { (self.desc.read(3), self.desc.read(2)) };

            Some(Timestamp::from_parts(high, low))
        } else {
            None
        }
    }
}
