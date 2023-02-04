use core::sync::atomic::{self, Ordering};

use crate::dma::{raw_descriptor::RawDescriptor, PacketId, RxError};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

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

/// Receive buffer 1 size
const RXDESC_1_RBS1_SHIFT: usize = 0;
/// Receive buffer 1 size mask
const RXDESC_1_RBS1_MASK: u32 = 0x0fff << RXDESC_1_RBS1_SHIFT;

/// Receive buffer 2 size
const RXDESC_1_RBS2_SHIFT: usize = 16;
/// Receive buffer 2 size mask
const RXDESC_1_RBS2_MASK: u32 = 0x0fff << RXDESC_1_RBS2_SHIFT;

/// Receive end of ring
const RXDESC_1_RER: u32 = 1 << 15;

#[repr(C)]
#[repr(align(4))]
#[derive(Clone, Copy)]
/// An RX DMA Descriptor.
pub struct RxDescriptor {
    inner_raw: RawDescriptor,
    packet_id: Option<PacketId>,
    #[cfg(feature = "ptp")]
    cached_timestamp: Option<Timestamp>,
}

impl Default for RxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl RxDescriptor {
    /// Creates a new [`RxDescriptor`].
    pub const fn new() -> Self {
        Self {
            inner_raw: RawDescriptor::new(),
            packet_id: None,
            #[cfg(feature = "ptp")]
            cached_timestamp: None,
        }
    }

    pub(super) fn setup(&mut self, buffer: &mut [u8]) {
        self.set_owned(buffer);
    }

    /// Is owned by the DMA engine?
    fn is_owned(&self) -> bool {
        (self.inner_raw.read(0) & RXDESC_0_OWN) == RXDESC_0_OWN
    }

    /// Pass ownership to the DMA engine
    ///
    /// Overrides old timestamp data
    pub(super) fn set_owned(&mut self, buffer: &mut [u8]) {
        self.set_buffer(buffer);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::Release);
        atomic::compiler_fence(Ordering::Release);

        unsafe {
            self.inner_raw.modify(0, |w| w | RXDESC_0_OWN);
        }

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::SeqCst);
    }

    fn has_error(&self) -> bool {
        (self.inner_raw.read(0) & RXDESC_0_ES) == RXDESC_0_ES
    }

    /// Descriptor contains first buffer of frame
    fn is_first(&self) -> bool {
        (self.inner_raw.read(0) & RXDESC_0_FS) == RXDESC_0_FS
    }

    /// Descriptor contains last buffers of frame
    fn is_last(&self) -> bool {
        (self.inner_raw.read(0) & RXDESC_0_LS) == RXDESC_0_LS
    }

    /// Configure the buffer and its length.
    fn set_buffer(&mut self, buffer: &[u8]) {
        let buffer_ptr = buffer.as_ptr();
        let buffer_len = buffer.len();

        unsafe {
            self.inner_raw.modify(1, |w| {
                // If rbs1 == 0, RBS1 will be ignored
                let w = w & !(RXDESC_1_RBS1_MASK);
                // Mask out any previous value of rbs2
                let w = w & !(RXDESC_1_RBS2_MASK);
                // Set the length of RBS2
                let w = w | ((buffer_len << RXDESC_1_RBS2_SHIFT) as u32 & RXDESC_1_RBS2_MASK);
                w
            });

            self.inner_raw.write(3, buffer_ptr as u32);
        }
    }

    pub(super) fn frame_length(&self) -> usize {
        ((self.inner_raw.read(0) >> RXDESC_0_FL_SHIFT) & RXDESC_0_FL_MASK) as usize
    }

    pub(super) fn take_received(
        &mut self,
        packet_id: Option<PacketId>,
        buffer: &mut [u8],
    ) -> Result<(), RxError> {
        if self.is_owned() {
            Err(RxError::WouldBlock)
        } else if self.has_error() {
            self.set_owned(buffer);
            Err(RxError::DmaError)
        } else if self.is_first() && self.is_last() {
            // "Subsequent reads and writes cannot be moved ahead of preceding reads."
            atomic::compiler_fence(Ordering::Acquire);

            self.packet_id = packet_id;

            // Cache the PTP timestamps if PTP is enabled.
            #[cfg(feature = "ptp")]
            self.attach_timestamp();

            Ok(())
        } else {
            self.set_owned(buffer);
            Err(RxError::Truncated)
        }
    }

    pub(super) fn set_end_of_ring(&mut self) {
        unsafe { self.inner_raw.modify(1, |w| w | RXDESC_1_RER) }
    }
}

#[cfg(feature = "ptp")]
impl RxDescriptor {
    pub(super) fn packet_id(&self) -> Option<&PacketId> {
        self.packet_id.as_ref()
    }

    /// Get PTP timestamps if available
    pub(super) fn read_timestamp(&self) -> Option<Timestamp> {
        #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
        let (high, low) = { (self.inner_raw.read(7), self.inner_raw.read(6)) };

        #[cfg(feature = "stm32f1xx-hal")]
        let (high, low) = { (self.inner_raw.read(3), self.inner_raw.read(2)) };

        #[cfg(not(feature = "stm32f1xx-hal"))]
        let is_valid = {
            /// RX timestamp
            const RXDESC_0_TIMESTAMP_VALID: u32 = 1 << 7;
            self.inner_raw.read(0) & RXDESC_0_TIMESTAMP_VALID == RXDESC_0_TIMESTAMP_VALID
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

    fn attach_timestamp(&mut self) {
        self.cached_timestamp = self.read_timestamp();
    }

    pub(super) fn timestamp(&self) -> Option<&Timestamp> {
        self.cached_timestamp.as_ref()
    }
}
