use volatile_register::{RO, RW};

#[cfg(not(feature = "stm32f1xx-hal"))]
const DESC_SIZE: usize = 8;

#[cfg(feature = "stm32f1xx-hal")]
const DESC_SIZE: usize = 4;

#[repr(C, align(8))]
#[derive(Clone)]
pub struct Descriptor([u32; DESC_SIZE]);

impl Default for Descriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl Descriptor {
    pub const fn new() -> Self {
        Self([0; _])
    }

    pub fn clear(&mut self) {
        // SAFETY: writing zeros to descriptors returns
        // ownership to the MCU.

        #[cfg(not(feature = "stm32f1xx-hal"))]
        unsafe {
            self.write::<0>(0);
            self.write::<0>(1);
            self.write::<0>(2);
            self.write::<0>(3);
            self.write::<0>(4);
            self.write::<0>(5);
            self.write::<0>(6);
            self.write::<0>(7);
        }

        #[cfg(feature = "stm32f1xx-hal")]
        unsafe {
            self.write::<0>(0);
            self.write::<0>(1);
            self.write::<0>(2);
            self.write::<0>(3);
        }
    }

    /// # Safety
    /// `n` must be less than `DESC_SIZE`
    unsafe fn r(&self, n: usize) -> &RO<u32> {
        let ro = unsafe { self.0.as_ptr().add(n) } as *const RO<u32>;
        unsafe { &*ro }
    }

    /// # Safety
    /// `n` must be less than `DESC_SIZE`
    unsafe fn rw(&mut self, n: usize) -> &mut RW<u32> {
        let rw = unsafe { self.0.as_mut_ptr().add(n) } as *mut RW<u32>;
        &mut *rw
    }

    pub fn read<const N: usize>(&self) -> u32 {
        const {
            assert!(N < DESC_SIZE);
        }
        // SAFETY: `N < DESC_SIZE`
        unsafe { self.r(N) }.read()
    }

    /// SAFETY: the side-effects of this write must be
    /// accounted for.
    pub unsafe fn write<const N: usize>(&mut self, value: u32) {
        const {
            assert!(N < DESC_SIZE);
        }
        // SAFETY: `N < DESC_SIZE`
        unsafe { self.rw(N).write(value) }
    }

    /// SAFETY: the side-effects of this write must be
    /// accounted for.
    pub unsafe fn modify<F, const N: usize>(&mut self, f: F)
    where
        F: FnOnce(u32) -> u32,
    {
        const {
            assert!(N < DESC_SIZE);
        }
        // SAFETY: `N < DESC_SIZE`
        unsafe { self.rw(N).modify(f) }
    }
}
