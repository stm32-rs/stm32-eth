use core::cell::UnsafeCell;

use super::{rx::RxDescriptor, tx::TxDescriptor, MTU};

pub trait RingDescriptor {
    fn setup(&mut self, buffer: *const u8, len: usize, next: Option<&Self>);
}

#[repr(C, align(8))]
pub struct Buffer {
    buffer: [u8; MTU],
}

impl Buffer {
    pub const fn new() -> Self {
        Self { buffer: [0; MTU] }
    }
}

impl core::ops::Deref for Buffer {
    type Target = [u8; MTU];

    fn deref(&self) -> &Self::Target {
        &self.buffer
    }
}

impl core::ops::DerefMut for Buffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer
    }
}

/// An entry in a DMA Descriptor ring
#[repr(C, align(8))]
pub struct RingEntry<T: RingDescriptor> {
    desc: T,
    pub(crate) buffer: UnsafeCell<Buffer>,
}

impl<T: RingDescriptor + Default> Default for RingEntry<T> {
    fn default() -> Self {
        RingEntry {
            desc: T::default(),
            buffer: UnsafeCell::new(Buffer::new()),
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
            buffer: UnsafeCell::new(Buffer::new()),
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
            buffer: UnsafeCell::new(Buffer::new()),
        }
    }
}

impl<T: RingDescriptor> RingEntry<T> {
    pub(crate) fn setup(&mut self, next: Option<&Self>) {
        let buffer = self.buffer.get();

        self.desc_mut()
            .setup(buffer as _, MTU, next.map(|next| next.desc()));
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
