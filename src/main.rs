#![feature(used)]
//#![feature(core_intrinsics)]
//#![feature(lang_items)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate stm32f429x;

use cortex_m::asm;
use stm32f429x::interrupt::Interrupt;
use stm32f429x::Peripherals;

use core::fmt::Write;
use cortex_m_semihosting::hio;

mod eth;
use eth::Eth;

fn main() {
    let p = Peripherals::take()
        .expect("Peripherals");

    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Enabling ethernet...").unwrap();

    let eth = cortex_m::interrupt::free(|cs| {
        let eth = Eth::new(p.ETHERNET_MAC, p.ETHERNET_DMA);
        eth.init_pins(cs, &p.RCC, &p.GPIOA, &p.GPIOB, &p.GPIOC, &p.GPIOG);
        eth.init(cs, &p.RCC, &p.SYSCFG);
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

}

// As we are not using interrupts, we just register a dummy catch all handler
#[link_section = ".vector_table.interrupts"]
#[used]
static INTERRUPTS: [extern "C" fn(); 240] = [default_handler; 240];

extern "C" fn default_handler() {
    cortex_m::interrupt::free(|cs| {
        let mut stdout = hio::hstdout().unwrap();
        writeln!(stdout, "I").unwrap();
    });
}

// interrupt!(Interrupt::Eth, 
