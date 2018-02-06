use alloc::vec_deque::VecDeque;
use alloc::allocator::{Alloc, Layout};
use volatile_register::RW;
use alloc::heap::Heap;
use board::ETHERNET_DMA;

use super::buffer::Buffer;

// Owned by DMA engine
const TXDESC_0_OWN: u32 = 1 << 31;
// First segment of frame
const TXDESC_0_FS: u32 = 1 << 28;
// Last segment of frame
const TXDESC_0_LS: u32 = 1 << 29;
// Transmit end of ring
const TXDESC_0_TER: u32 = 1 << 21;
// Second address chained
const TXDESC_0_TCH: u32 = 1 << 20;
// Second address chained
const TXDESC_0_ES: u32 = 1 << 15;

const TXDESC_1_TBS_SHIFT: usize = 0;
const TXDESC_1_TBS_MASK: u32 = 0x0fff << TXDESC_1_TBS_SHIFT;


#[repr(C)]
struct TxDescriptor {
    tdesc: &'static mut [RW<u32>; 4],
}

impl Default for TxDescriptor {
    fn default() -> Self {
        let mut this = Self::new();
        this.write(0, TXDESC_0_TCH | TXDESC_0_FS | TXDESC_0_LS);
        this.write(1, 0);
        this.write(2, 0);
        this.write(3, 0);
        this
    }
}

impl Drop for TxDescriptor {
    fn drop(&mut self) {
        unsafe {
            Heap.dealloc(self.tdesc.as_mut_ptr() as *mut u8, Self::memory_layout())
        }
    }
}

impl TxDescriptor {
    fn memory_layout() -> Layout {
        Layout::from_size_align(4 * 4, super::ALIGNMENT)
            .unwrap()
    }
    
    fn new() -> Self {
        let mem = unsafe {
            Heap.alloc(Self::memory_layout())
        }.expect("alloc with memory_layout") as *mut [u32; 4];

        TxDescriptor {
            tdesc: unsafe { &mut *(mem as *mut [RW<u32>; 4]) },
        }
    }

    fn as_raw_ptr(&self) -> *const u8 {
        self.tdesc.as_ptr() as *const u8
    }
    
    fn read(&self, i: usize) -> u32 {
        self.tdesc[i].read()
    }

    fn write(&mut self, i: usize, data: u32) {
        unsafe { self.tdesc[i].write(data) }
    }

    fn modify<F>(&mut self, i: usize, f: F)
        where F: (FnOnce(u32) -> u32) {

        unsafe { self.tdesc[i].modify(f) }
    }
    
    /// Is owned by the DMA engine?
    pub fn is_owned(&self) -> bool {
        (self.read(0) & TXDESC_0_OWN) == TXDESC_0_OWN
    }

    pub fn set_owned(&mut self) {
        self.modify(0, |w| w | TXDESC_0_OWN);
    }

    pub fn has_error(&self) -> bool {
        (self.read(0) & TXDESC_0_ES) == TXDESC_0_ES
    }

    /// Descriptor contains first buffer of frame
    pub fn set_first(&mut self) {
        self.modify(0, |w| w | TXDESC_0_FS);
    }

    /// Descriptor contains last buffers of frame
    pub fn set_last(&mut self) {
        self.modify(0, |w| w | TXDESC_0_LS);
    }

    pub fn set_buffer1(&mut self, buffer: *const u8, len: usize) {
        self.write(2, buffer as u32);
        self.modify(1, |w| {
            (w & !TXDESC_1_TBS_MASK) |
            ((len as u32) << TXDESC_1_TBS_SHIFT)
        });
    }

    // points to next descriptor (RCH)
    pub fn set_buffer2(&mut self, buffer: *const u8) {
        self.write(3, buffer as u32);
    }

    pub fn set_end_of_ring(&mut self) {
        self.modify(1, |w| w | TXDESC_0_TER);
    }
}

struct TxRingEntry {
    desc: TxDescriptor,
    buffer: Option<Buffer>,
}

impl TxRingEntry {
    pub fn new() -> Self {
        let desc = TxDescriptor::default();
        TxRingEntry {
            desc,
            buffer: None,
        }
    }

    pub fn send(&mut self, buffer: Buffer, next: Option<&TxRingEntry>) {
        self.desc.set_buffer1(buffer.as_ptr(), buffer.len());
        self.buffer = Some(buffer);

        match next {
            Some(next_buffer) => {
                let ptr = next_buffer.desc.as_raw_ptr();
                self.desc.set_buffer2(ptr);
            },
            // For the last in the ring
            None => {
                self.desc.set_buffer2(0 as *const u8);
                self.desc.set_end_of_ring();
            },
        }

        self.desc.set_owned();
    }
}

pub struct TxRing {
    entries: VecDeque<TxRingEntry>,
}

impl TxRing {
    pub fn new() -> Self {
        let mut entries = VecDeque::with_capacity(2);
        entries.push_back(TxRingEntry::new());

        TxRing {
            entries,
        }
    }

    pub fn send(&mut self, buffer: Buffer) {
        self.flush();
        
        let i = self.entries.len() - 1;
        let next_entry = {
            let mut entry = &mut self.entries[i];
            let next_entry = TxRingEntry::new();
            entry.send(buffer, Some(&next_entry));
            next_entry
        };
        self.entries.push_back(next_entry);
    }

    /// Flushes entries that have been processed by the DMA engine.
    pub fn flush(&mut self) -> usize {
        fn is_done(entry: &TxRingEntry) -> bool {
            // returned by DMA engine?
            (! entry.desc.is_owned()) &&
            // had a buffer associated (was used)
            entry.buffer.is_some()
        }

        let mut flushed = 0;
        while is_done(&self.entries[0]) {
            self.entries.pop_front();
            flushed += 1;
        }
        flushed
    }

    pub fn queue_len(&self) -> usize {
        self.entries.len()
    }

    pub fn start(&self, eth_dma: &ETHERNET_DMA) {
        let ring_ptr = self.entries[0].desc.as_raw_ptr();
        // Register TxDescriptor
        eth_dma.dmatdlar.write(|w| unsafe { w.stl().bits(ring_ptr as u32) });

        // Start transmission
        eth_dma.dmaomr.modify(|_, w| w.st().set_bit());

        self.start_dma(eth_dma);
    }

    /// Start DMA engine
    pub fn start_dma(&self, eth_dma: &ETHERNET_DMA) {
        eth_dma.dmatpdr.write(|w| unsafe { w.tpd().bits(1) });
    }

    pub fn is_running(&self, eth_dma: &ETHERNET_DMA) -> bool {
        eth_dma.dmasr.read().ts().bit()
    }
}
