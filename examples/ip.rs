#![no_std]
#![feature(used)]
#![feature(core_intrinsics)]
#![feature(alloc, global_allocator, allocator_api, box_heap)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
#[macro_use(exception, interrupt)]
extern crate stm32f429 as board;
extern crate alloc_cortex_m;
#[macro_use(vec)]
extern crate alloc;
extern crate stm32f4x9_eth as eth;
extern crate smoltcp;
extern crate log;

use cortex_m::asm;
use board::{Peripherals, CorePeripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use alloc_cortex_m::CortexMHeap;

use core::fmt::Write;
use cortex_m_semihosting::hio;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr,
                    Ipv4Address};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use alloc::btree_map::BTreeMap;
use log::{Record, Level, Metadata, LevelFilter};

use eth::Eth;

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

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

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
    unsafe { log::set_logger(&LOGGER).unwrap(); }
    log::set_max_level(LevelFilter::Info);
    
    let heap_size = init_alloc();
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Heap: {} bytes", heap_size).unwrap();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    writeln!(stdout, "Enabling ethernet...").unwrap();
    eth::setup(&p);
    let eth = Eth::new(p.ETHERNET_MAC, p.ETHERNET_DMA, 32);
    eth.enable_interrupt(&mut cp.NVIC);

    let local_addr = Ipv4Address::new(10, 0, 0, 1);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    let neighbor_cache = NeighborCache::new(BTreeMap::new());
    let ethernet_addr = EthernetAddress(SRC_MAC);
    let mut iface = EthernetInterfaceBuilder::new(eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs([ip_addr])
        .neighbor_cache(neighbor_cache)
        .finalize();

    let mut sockets = SocketSet::new(vec![]);
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(vec![0; 2048]),
        TcpSocketBuffer::new(vec![0; 2048])
    );
    let server_handle = sockets.add(server_socket);

    loop {
        let time: u64 = cortex_m::interrupt::free(|cs| {
            *TIME.borrow(cs)
                .borrow()
        });
        match iface.poll(&mut sockets, Instant::from_millis(time as i64)) {
            Ok(true) => {
                let mut socket = sockets.get::<TcpSocket>(server_handle);
                if !socket.is_open() {
                    socket.listen(80)
                        .or_else(|e| {
                            writeln!(stdout, "TCP listen error: {:?}", e)
                        })
                        .unwrap();
                }

                if socket.can_send() {
                    write!(socket, "hello\n")
                        .map(|_| {
                            socket.close();
                        })
                        .or_else(|e| {
                            writeln!(stdout, "TCP send error: {:?}", e)
                        })
                        .unwrap();
                }
            },
            Ok(false) =>
                asm::wfi(),
            Err(_) =>
                // Ignore malformed packets
                (),
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms());
    syst.enable_counter();
    syst.enable_interrupt();
}

fn systick_interrupt_handler() {
    cortex_m::interrupt::free(|cs| {
        let mut time =
            TIME.borrow(cs)
            .borrow_mut();
        *time += 10;
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
