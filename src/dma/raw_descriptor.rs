use volatile_register::{RO, RW};

use crate::MTU;

#[cfg(not(feature = "stm32f1xx-hal"))]
pub(crate) const DESC_SIZE: usize = 8;

#[cfg(feature = "stm32f1xx-hal")]
pub(crate) const DESC_SIZE: usize = 4;

#[repr(C)]
#[repr(align(8))]
#[derive(Clone, Copy)]
pub struct RawDescriptor {
    pub(crate) desc: [u32; DESC_SIZE],
}

impl Default for RawDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl RawDescriptor {
    pub const fn new() -> Self {
        Self {
            desc: [0; DESC_SIZE],
        }
    }

    fn r(&self, n: usize) -> &RO<u32> {
        let ro = &self.desc[n] as *const _ as *const RO<u32>;
        unsafe { &*ro }
    }

    unsafe fn rw(&mut self, n: usize) -> &mut RW<u32> {
        let rw = &mut self.desc[n] as *mut _ as *mut RW<u32>;
        &mut *rw
    }

    pub fn read(&self, n: usize) -> u32 {
        self.r(n).read()
    }

    pub unsafe fn write(&mut self, n: usize, value: u32) {
        self.rw(n).write(value)
    }

    pub unsafe fn modify<F>(&mut self, n: usize, f: F)
    where
        F: FnOnce(u32) -> u32,
    {
        self.rw(n).modify(f)
    }
}

pub struct DescriptorRing<'data, T> {
    descriptors: &'data mut [T],
    buffers: &'data mut [[u8; MTU]],
}

pub trait DescriptorRingEntry {
    /// Set up this [`Descriptor`] with the given buffer.
    fn setup(&mut self, buffer: &mut [u8]);
}

impl<'data, T> DescriptorRing<'data, T>
where
    T: DescriptorRingEntry,
{
    pub fn new(descriptors: &'data mut [T], buffers: &'data mut [[u8; MTU]]) -> Self {
        assert!(descriptors.len() == buffers.len());
        buffers.iter().for_each(|b| assert!(b.len() <= super::MTU));

        Self {
            descriptors,
            buffers,
        }
    }

    pub fn len(&self) -> usize {
        self.descriptors.len()
    }

    pub fn get(&mut self, index: usize) -> (&mut T, &mut [u8]) {
        (&mut self.descriptors[index], &mut self.buffers[index])
    }

    pub fn descriptors(&mut self) -> impl Iterator<Item = &mut T> {
        self.descriptors.iter_mut()
    }

    pub fn descriptors_and_buffers(&mut self) -> impl Iterator<Item = (&mut T, &mut [u8; MTU])> {
        self.descriptors.iter_mut().zip(self.buffers.iter_mut())
    }

    pub fn descriptors_start_address(&self) -> *const T {
        self.descriptors.as_ptr()
    }
}
