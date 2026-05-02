use super::{rx::RxDescriptor, tx::TxDescriptor};

use buffer::Buffer;

pub trait RingDescriptor {
    fn setup(&mut self, buffer: *const u8, len: usize, next: Option<&Self>);
}
mod buffer {
    use core::cell::UnsafeCell;

    use crate::dma::MTU;

    #[repr(C, align(8))]
    pub struct Buffer {
        buffer: UnsafeCell<[u8; MTU]>,
    }

    impl Buffer {
        pub const fn new() -> Self {
            Self {
                buffer: UnsafeCell::new([0u8; _]),
            }
        }

        pub const fn len(&self) -> usize {
            MTU
        }

        pub fn as_mut_ptr(&mut self) -> *mut u8 {
            &raw mut self.buffer as _
        }

        /// # Safety
        /// The caller must guarantee that the DMA does not
        /// own this buffer for the duration of the borrow.
        pub unsafe fn get(&self) -> &[u8; MTU] {
            // SAFETY: the shared reference to `self` guarantees
            // that we have shared access to the buffer for the
            // lifetime of `self` (which is what we're returning).
            // The caller guarantees that the DMA does
            // not own the buffer.
            unsafe { &*self.buffer.get() }
        }

        /// # Safety
        /// The caller must guarantee that the DMA does not
        /// own this buffer for the duration of the borrow.
        pub unsafe fn get_mut(&mut self) -> &mut [u8; MTU] {
            // SAFETY: the exclusive reference to `self` guarantees
            // that no other borrows of the buffer currently exist
            // and will not for the lifetime of `self`.
            // The caller guarantees that the DMA does not
            // own the buffer.
            unsafe { &mut *self.buffer.get() }
        }
    }
}

/// An entry in a DMA Descriptor ring
#[repr(C, align(8))]
pub struct RingEntry<T: RingDescriptor> {
    desc: T,
    pub(crate) buffer: Buffer,
}

impl<T: RingDescriptor + Default> Default for RingEntry<T> {
    fn default() -> Self {
        RingEntry {
            desc: T::default(),
            buffer: Buffer::new(),
        }
    }
}

impl RingEntry<TxDescriptor> {
    /// The initial value of a TxRingDescriptor
    pub const INIT: Self = Self::new();

    /// Creates a RingEntry with a TxDescriptor.
    pub const fn new() -> Self {
        RingEntry {
            desc: TxDescriptor::new(),
            buffer: Buffer::new(),
        }
    }
}

impl RingEntry<RxDescriptor> {
    /// The initial value of an RxRingDescriptor
    pub const INIT: Self = Self::new();

    /// Creates a RingEntry with a RxDescriptor.
    pub const fn new() -> Self {
        RingEntry {
            desc: RxDescriptor::new(),
            buffer: Buffer::new(),
        }
    }
}

impl<T: RingDescriptor> RingEntry<T> {
    pub(crate) fn setup(&mut self, next: Option<&Self>) {
        let buffer = self.buffer.as_mut_ptr();
        let len = self.buffer.len();
        self.desc_mut()
            .setup(buffer, len, next.map(|next| next.desc()));
    }

    #[inline]
    pub(crate) fn desc(&self) -> &T {
        &self.desc
    }

    #[inline]
    pub(crate) fn desc_mut(&mut self) -> &mut T {
        &mut self.desc
    }
}
