use alloc::allocator::{Alloc, Layout};
use alloc::heap::Heap;
use core::slice;
use core::ops::{Deref, DerefMut};

/// An aligned buffer, representing a `&[u8]` slice
///
/// The ethernet hardware drops the last bits of Rx/Tx DMA buffers,
/// hence this need for aligned allocations.
///
/// TODO: rather use the `aligned` crate?
pub struct Buffer {
    buffer: &'static mut [u8],
    length: usize,
}

impl Buffer {
    /// Manually allocate a buffer by passing the 64-bit
    /// [`ALIGNMENT`](constant.ALIGNMENT.html) to the allocator.
    pub fn new(size: usize) -> Self {
        let layout = Layout::from_size_align(size, super::ALIGNMENT)
            .expect("layout");
        let ptr = unsafe { Heap.alloc(layout) }
            .expect("alloc");
        let buffer = unsafe { slice::from_raw_parts_mut(ptr, size) };
        Buffer { buffer, length: 0 }
    }

    /// Allocation size
    ///
    /// Use `len()` to obtain the current slice's length through
    /// `deref()`.
    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    /// Get immutable slice
    pub fn as_slice(&self) -> &[u8] {
        &self.buffer[0..self.length]
    }

    /// Get mutable slice
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.buffer[0..self.length]
    }

    /// Get memory address for RX/TX Descriptors
    pub fn as_ptr(&self) -> *const u8 {
        self.buffer.as_ptr()
    }

    /// Set length of the current slice.
    pub fn set_len(&mut self, len: usize) {
        self.length = len.min(self.capacity());
    }
}

/// Free the buffer via the allocator
impl Drop for Buffer {
    fn drop(&mut self) {
        let layout = Layout::from_size_align(self.buffer.len(), super::ALIGNMENT)
            .expect("layout");
        unsafe {
            Heap.dealloc(self.buffer.as_mut_ptr(), layout);
        }
    }
}

/// Dereference to immutable slice
impl Deref for Buffer {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

/// Dereference to mutable slice
impl DerefMut for Buffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}
