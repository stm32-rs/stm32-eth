use alloc_cortex_m::CortexMHeap;

#[global_allocator]
pub static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

// These symbols come from a linker script
extern "C" {
    static mut _sheap: u32;
    static mut _eheap: u32;
}

/// Initialize the heap allocator `ALLOCATOR`
pub fn init() -> usize {
    let start = unsafe { &mut _sheap as *mut u32 as usize };
    let end = unsafe { &mut _eheap as *mut u32 as usize };
    unsafe { ALLOCATOR.init(start, end - start) }
    end - start
}
