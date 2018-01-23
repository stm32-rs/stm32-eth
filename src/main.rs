#![no_std]
#![feature(used)]
//#![feature(core_intrinsics)]
//#![feature(lang_items)]
#![feature(alloc, global_allocator, allocator_api, box_heap)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
#[macro_use(exception, interrupt)]
extern crate stm32f429x;
extern crate alloc_cortex_m;
extern crate alloc;
extern crate volatile_register;

use cortex_m::asm;
use stm32f429x::interrupt::Interrupt;
use stm32f429x::{Peripherals, CorePeripherals};

use core::fmt::Write;
use cortex_m_semihosting::hio;

mod init_alloc;
pub use init_alloc::ALLOCATOR;
mod eth;
use eth::Eth;

fn main() {
    let heap_size = init_alloc::init();
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Heap: {} bytes", heap_size).unwrap();

    let p = Peripherals::take()
        .expect("Peripherals");
    let mut cp = CorePeripherals::take()
        .expect("CorePeripherals");

    writeln!(stdout, "Enabling ethernet...").unwrap();

    let mut eth = cortex_m::interrupt::free(|cs| {
        let mut eth = Eth::new(p.ETHERNET_MAC, p.ETHERNET_DMA);
        eth.init_pins(cs, &p.RCC, &p.GPIOA, &p.GPIOB, &p.GPIOC, &p.GPIOG);
        eth.init(cs, &p.RCC, &p.SYSCFG, &mut cp.NVIC);
        eth.start_rx(8);
        // eth.start_rx(1);
        eth
    });

    while ! eth.status().link_detected() {
        writeln!(stdout, "Ethernet: waiting for link").unwrap();
    }
    let status = eth.status();
    writeln!(
        stdout,
        "Ethernet: link detected with {} Mbps/{}",
        status.speed(),
        match status.is_full_duplex() {
            Some(true) => "FD",
            Some(false) => "HD",
            None => "?",
        }
    ).unwrap();

    loop {
        // asm::wfi();
        // writeln!(stdout, "I").unwrap();
        match eth.recv_next() {
            None => (),
            Some(pkt_len) => {
                writeln!(stdout, "Rx: {} bytes", pkt_len);
            },
        }
    }
}
