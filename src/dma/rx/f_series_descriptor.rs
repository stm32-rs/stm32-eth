use crate::dma::generic_ring::RawDescriptor;
use crate::dma::Cache;

use crate::dma::{rx::RxDescriptorError, PacketId};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

/// RX timestamp valid
/// NOTE(allow): unused if not(feature = "ptp")
#[allow(unused)]
const RXDESC_0_TIMESTAMP_VALID: u32 = 1 << 7;
/// Owned by DMA engine
const RXDESC_0_OWN: u32 = 1 << 31;
/// First descriptor
const RXDESC_0_FS: u32 = 1 << 9;
/// Last descriptor
const RXDESC_0_LS: u32 = 1 << 8;
/// Error summary
const RXDESC_0_ES: u32 = 1 << 15;
/// Frame length
const RXDESC_0_FL_MASK: u32 = 0x3FFF;
const RXDESC_0_FL_SHIFT: usize = 16;

/// Receive buffer 2 size
const RXDESC_1_RBS2_SHIFT: usize = 16;
/// Receive buffer 2 size mask
const RXDESC_1_RBS2_MASK: u32 = 0x0fff << RXDESC_1_RBS2_SHIFT;

/// Receive end of ring
const RXDESC_1_RER: u32 = 1 << 15;

#[repr(C)]
/// An RX DMA Descriptor
#[derive(Clone, Copy)]
pub struct RxDescriptor {
    desc: RawDescriptor,
    cache: Cache,
    last: bool,
}

impl Default for RxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl RxDescriptor {
    /// Creates an zeroed RxDescriptor.
    pub const fn new() -> Self {
        Self {
            desc: RawDescriptor::new(),
            last: false,
            cache: Cache::new(),
        }
    }

    pub(super) fn setup(&mut self, end_of_ring: bool, buffer: &mut [u8]) {
        self.last = end_of_ring;
        self.set_owned(buffer);
    }

    /// Is owned by the DMA engine?
    fn is_owned(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_OWN) == RXDESC_0_OWN
    }

    pub(super) fn is_available(&self) -> bool {
        !self.is_owned()
    }

    /// Pass ownership to the DMA engine
    pub fn set_owned(&mut self, buffer: &mut [u8]) {
        self.set_buffer(buffer);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);

        unsafe {
            self.desc.write(0, RXDESC_0_OWN);
        }

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    }

    /// Configure the buffer and its length.
    fn set_buffer(&mut self, buffer: &[u8]) {
        let buffer_ptr = buffer.as_ptr();
        let buffer_len = buffer.len();

        unsafe {
            let mut w = (buffer_len << RXDESC_1_RBS2_SHIFT) as u32 & RXDESC_1_RBS2_MASK;

            if self.last {
                w |= RXDESC_1_RER;
            }

            self.desc.write(1, w);

            self.desc.write(3, buffer_ptr as u32);
        }
    }

    /// Only call this if [`RxRingEntry::is_available`]
    pub(super) fn recv(
        &mut self,
        packet_id: Option<PacketId>,
        buffer: &mut [u8],
    ) -> Result<(), RxDescriptorError> {
        if self.has_error() {
            self.set_owned(buffer);
            Err(RxDescriptorError::DmaError)
        } else if self.is_first() && self.is_last() {
            // "Subsequent reads and writes cannot be moved ahead of preceding reads."
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);

            // Set the Packet ID for this descriptor.
            self.cache.set_id_and_clear_ts(packet_id);

            #[cfg(feature = "ptp")]
            self.cache.set_ts(self.read_timestamp());

            Ok(())
        } else {
            self.set_owned(buffer);
            Err(RxDescriptorError::Truncated)
        }
    }

    fn has_error(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_ES) == RXDESC_0_ES
    }

    /// Descriptor contains first buffer of frame
    fn is_first(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_FS) == RXDESC_0_FS
    }

    /// Descriptor contains last buffers of frame
    fn is_last(&self) -> bool {
        (self.desc.read(0) & RXDESC_0_LS) == RXDESC_0_LS
    }

    pub(super) fn frame_len(&self) -> usize {
        ((self.desc.read(0) >> RXDESC_0_FL_SHIFT) & RXDESC_0_FL_MASK) as usize
    }
}

#[cfg(feature = "ptp")]
impl RxDescriptor {
    pub(super) fn has_packet_id(&self, id: &PacketId) -> bool {
        Some(id) == self.cache.id().as_ref()
    }

    fn read_timestamp(&self) -> Option<Timestamp> {
        #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
        let (high, low) = { (self.desc.read(7), self.desc.read(6)) };

        #[cfg(feature = "stm32f1xx-hal")]
        let (high, low) = { (self.desc.read(3), self.desc.read(2)) };

        #[cfg(not(feature = "stm32f1xx-hal"))]
        let is_valid = {
            /// RX timestamp
            const RXDESC_0_TIMESTAMP_VALID: u32 = 1 << 7;
            self.desc.read(0) & RXDESC_0_TIMESTAMP_VALID == RXDESC_0_TIMESTAMP_VALID
        };

        #[cfg(feature = "stm32f1xx-hal")]
        // There is no direct "timestamp valid" indicator bit
        // on STM32F1XX, but if it's invalid it will be written
        // as all ones.
        let is_valid = high != 0xFFFF_FFFF || low != 0xFFFF_FFFF;

        let timestamp = Timestamp::from_parts(high, low);

        if is_valid && self.is_last() {
            Some(timestamp)
        } else {
            None
        }
    }

    /// Get PTP timestamp if available
    pub(super) fn timestamp(&self) -> Option<Timestamp> {
        self.cache.ts()
    }
}
