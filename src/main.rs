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
use stm32f429x::{Peripherals, CorePeripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use core::fmt::Write;
use cortex_m_semihosting::hio;

mod init_alloc;
pub use init_alloc::ALLOCATOR;
mod eth;
use eth::Eth;

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
const DST_MAC: [u8; 6] = [0x00, 0x00, 0xBE, 0xEF, 0xDE, 0xAD];
const ETH_TYPE: [u8; 2] = [0x80, 0x00];

static TIME: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));

fn main() {
    let heap_size = init_alloc::init();
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Heap: {} bytes", heap_size).unwrap();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    writeln!(stdout, "Enabling ethernet...").unwrap();
    eth::setup(&p);
    let mut eth = Eth::new(p.ETHERNET_MAC, p.ETHERNET_DMA, 32);
    eth.enable_interrupt(&mut cp.NVIC);

    // Main loop
    let mut last_stats_time = 0usize;
    let mut rx_bytes = 0usize;
    let mut rx_pkts = 0usize;
    let mut tx_bytes = 0usize;
    let mut tx_pkts = 0usize;
    let mut last_status = None;

    loop {
        let time: usize = cortex_m::interrupt::free(|cs| {
            *TIME.borrow(cs)
                .borrow()
        });

        // print stats every 30 seconds
        if time >= last_stats_time + 30 {
            let t = time - last_stats_time;
            writeln!(
                stdout, "T={}\tRx:\t{} KB/s\t{} pps\tTx:\t{} KB/s\t{} pps",
                time,
                rx_bytes / 1024 / t, rx_pkts / t,
                tx_bytes / 1024 / t, tx_pkts / t
            ).unwrap();
            // Reset
            rx_bytes = 0;
            rx_pkts = 0;
            tx_bytes = 0;
            tx_pkts = 0;
            last_stats_time = time;
        }

        // Link change detection
        let status = eth.status();
        if last_status
            .map(|last_status| last_status != status)
            .unwrap_or(true)
        {
            if ! status.link_detected() {
                writeln!(
                    stdout,
                    "Ethernet: no link detected"
                ).unwrap();
            } else {
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

            last_status = Some(status);
        }

        // handle rx packet
        let mut recvd = 0usize;
        while let Some(pkt) = eth.recv_next() {
            rx_bytes += pkt.len();
            rx_pkts += 1;

            recvd += 1;
            if recvd > 8 {
                break;
            }
        }

        // fill tx queue
        if status.link_detected() {
            let mut sent = 0usize;
            const SIZE: usize = 1500;
            while eth.queue_len() < 128 && sent < 8 {
                let mut buf = eth::Buffer::new(SIZE);
                buf.set_len(SIZE);
                buf.as_mut_slice()[0..6].copy_from_slice(&DST_MAC);
                buf.as_mut_slice()[6..12].copy_from_slice(&SRC_MAC);
                buf.as_mut_slice()[12..14].copy_from_slice(&ETH_TYPE);
                eth.send(buf);

                tx_bytes += SIZE;
                tx_pkts += 1;
                sent += 1;
            }
        }

        // wait for next interrupt
        // asm::wfi();
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
