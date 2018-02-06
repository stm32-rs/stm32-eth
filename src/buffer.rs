use alloc::allocator::{Alloc, Layout};
use alloc::heap::Heap;
use core::slice;
use core::ops::{Deref, DerefMut};

/// An aligned buffer
///
/// TODO: rather use the `aligned` crate?
pub struct Buffer {
    buffer: &'static mut [u8],
    length: usize,
}

impl Buffer {
    pub fn new(size: usize) -> Self {
        let layout = Layout::from_size_align(size, super::ALIGNMENT)
            .expect("layout");
        let ptr = unsafe { Heap.alloc(layout) }
            .expect("alloc");
        let buffer = unsafe { slice::from_raw_parts_mut(ptr, size) };
        Buffer { buffer, length: 0 }
    }

    pub fn capacity(&self) -> usize {
        self.buffer.len()
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buffer[0..self.length]
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.buffer[0..self.length]
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.buffer.as_ptr()
    }

    pub fn set_len(&mut self, len: usize) {
        self.length = len.min(self.capacity());
    }
}

impl Drop for Buffer {
    fn drop(&mut self) {
        let layout = Layout::from_size_align(self.buffer.len(), super::ALIGNMENT)
            .expect("layout");
        unsafe {
            Heap.dealloc(self.buffer.as_mut_ptr(), layout);
        }
    }
}

impl Deref for Buffer {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        self.as_slice()
    }
}

impl DerefMut for Buffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.as_mut_slice()
    }
}
