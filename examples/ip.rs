#![no_std]
#![feature(used)]
//#![feature(core_intrinsics)]
//#![feature(lang_items)]
#![feature(alloc, global_allocator, allocator_api, box_heap)]
#![feature(const_fn)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
#[macro_use(exception, interrupt)]
extern crate stm32f429x;
extern crate alloc_cortex_m;
#[macro_use(vec)]
extern crate alloc;
extern crate stm32f4x9_eth as eth;
extern crate smoltcp;
extern crate log;

use cortex_m::asm;
use stm32f429x::{Peripherals, CorePeripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use alloc_cortex_m::CortexMHeap;

use core::fmt::Write;
use cortex_m_semihosting::hio;

use core::str::FromStr;
use smoltcp::phy::Device;
//use smoltcp::phy::wait as phy_wait;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr,
                    Ipv4Address, Icmpv4Repr, Icmpv4Packet};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, IcmpSocket, IcmpSocketBuffer, IcmpPacketBuffer, IcmpEndpoint};
use alloc::btree_map::BTreeMap;
use log::{Record, Level, Metadata, LevelFilter};

use eth::Eth;
use eth::smoltcp_phy;

static mut LOGGER: HioLogger = HioLogger {};

struct HioLogger {}

impl log::Log for HioLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut stdout = hio::hstdout().unwrap();
            writeln!(stdout, "{} - {}", record.level(), record.args())
                .unwrap();
        }
    }
    fn flush(&self) {}
}


const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
const DST_MAC: [u8; 6] = [0x00, 0x00, 0xBE, 0xEF, 0xDE, 0xAD];
const ETH_TYPE: [u8; 2] = [0x80, 0x00];

static TIME: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));

#[global_allocator]
pub static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

// These symbols come from a linker script
extern "C" {
    static mut _sheap: u32;
    static mut _eheap: u32;
}

/// Initialize the heap allocator `ALLOCATOR`
pub fn init_alloc() -> usize {
    let start = unsafe { &mut _sheap as *mut u32 as usize };
    let end = unsafe { &mut _eheap as *mut u32 as usize };
    unsafe { ALLOCATOR.init(start, end - start) }
    end - start
}

fn main() {
    unsafe { log::set_logger(&LOGGER); }
    log::set_max_level(LevelFilter::Trace);
    
    let heap_size = init_alloc();
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Heap: {} bytes", heap_size).unwrap();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    writeln!(stdout, "Enabling ethernet...").unwrap();
    eth::setup(&p);
    let mut eth = Eth::new(p.ETHERNET_MAC, p.ETHERNET_DMA, 32);
    eth.enable_interrupt(&mut cp.NVIC);

    let local_addr = Ipv4Address::new(10, 0, 0, 1);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    let neighbor_cache = NeighborCache::new(BTreeMap::new());
    let ethernet_addr = EthernetAddress([0x02, 0x00, 0x00, 0x00, 0x00, 0x02]);
    let mut iface = EthernetInterfaceBuilder::new(eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs([ip_addr])
        .neighbor_cache(neighbor_cache)
        .finalize();
    let mut sockets = SocketSet::new(vec![]);

    loop {
        iface.poll(&mut sockets, 0).expect("poll error");
        asm::wfi();
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(100 * stm32f429x::SYST::get_ticks_per_10ms());
    syst.enable_counter();
    syst.enable_interrupt();

    if ! SYST::is_precise() {
        let mut stderr = hio::hstderr().unwrap();
        writeln!(
            stderr,
            "Warning: SYSTICK with source {:?} is not precise",
            syst.get_clock_source()
        ).unwrap();
    }
}

fn systick_interrupt_handler() {
    cortex_m::interrupt::free(|cs| {
        let mut time =
            TIME.borrow(cs)
            .borrow_mut();
        *time += 1;
    })
}

#[used]
exception!(SYS_TICK, systick_interrupt_handler);


fn eth_interrupt_handler() {
    let p = unsafe { Peripherals::steal() };

    // Clear interrupt flags
    eth::eth_interrupt_handler(&p.ETHERNET_DMA);
}

#[used]
interrupt!(ETH, eth_interrupt_handler);
