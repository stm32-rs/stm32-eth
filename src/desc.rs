use core::ops::{Deref, DerefMut};

use aligned::Aligned;
use volatile_register::{RO, RW};


#[repr(C)]
pub struct Descriptor {
    desc: Aligned<u64, [u32; 4]>,
}

impl Clone for Descriptor {
    fn clone(&self) -> Self {
        Descriptor {
            desc: Aligned(self.desc.array.clone())
        }
    }
}

impl Default for Descriptor {
    fn default() -> Self {
        Descriptor {
            desc: Aligned([0; 4]),
        }
    }
}

impl Descriptor {
    fn r(&self, n: usize) -> &RO<u32> {
        let ro = &self.desc.deref()[n] as *const _ as *const RO<u32>;
        unsafe { &*ro }
    }

    unsafe fn rw(&mut self, n: usize) -> &mut RW<u32> {
        let rw = &mut self.desc.deref_mut()[n] as *mut _ as *mut RW<u32>;
        &mut *rw
    }

    pub fn read(&self, n: usize) -> u32 {
        self.r(n).read()
    }

    pub unsafe fn write(&mut self, n: usize, value: u32) {
        self.rw(n).write(value)
    }

    pub unsafe fn modify<F>(&mut self, n: usize, f: F)
        where F: FnOnce(u32) -> u32
    {
        self.rw(n).modify(f)
    }
}
