//! For build and run instructions, see README.md
//!
//! An example that generates some empty ethernet packets.
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use core::default::Default;
use cortex_m_rt::{entry, exception};

use cortex_m::interrupt::Mutex;
use stm32_eth::{
    mac::{phy::BarePhy, Phy},
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
    Parts,
};

use stm32_eth::dma::TxError;

pub mod common;

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
const DST_MAC: [u8; 6] = [0x00, 0x00, 0xBE, 0xEF, 0xDE, 0xAD];
const ETH_TYPE: [u8; 2] = [0x80, 0x00];
const PHY_ADDR: u8 = 0;

static TIME: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let (clocks, gpio, ethernet) = common::setup_peripherals(p);

    setup_systick(&mut cp.SYST);

    defmt::info!("Enabling ethernet...");
    let (eth_pins, mdio, mdc, _) = common::setup_pins(gpio);

    let (rx_ring, tx_ring) = crate::common::setup_rings();

    let Parts {
        mut dma,
        mac,
        #[cfg(feature = "ptp")]
            ptp: _,
    } = stm32_eth::new(ethernet, rx_ring, tx_ring, clocks, eth_pins).unwrap();
    dma.enable_interrupt();

    // Main loop
    let mut last_stats_time = 0usize;
    let mut rx_bytes = 0usize;
    let mut rx_pkts = 0usize;
    let mut tx_bytes = 0usize;
    let mut tx_pkts = 0usize;
    let mut last_link_up = false;

    let mut phy = BarePhy::new(mac.with_mii(mdio, mdc), PHY_ADDR, Default::default());

    loop {
        let time: usize = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());

        // print stats every 30 seconds
        if time >= last_stats_time + 30 {
            let t = time - last_stats_time;
            defmt::info!(
                "T={}\tRx:\t{} KB/s\t{} pps\tTx:\t{} KB/s\t{} pps",
                time,
                rx_bytes / 1024 / t,
                rx_pkts / t,
                tx_bytes / 1024 / t,
                tx_pkts / t
            );

            // Reset
            rx_bytes = 0;
            rx_pkts = 0;
            tx_bytes = 0;
            tx_pkts = 0;
            last_stats_time = time;
        }

        // Link change detection
        let link_up = phy.phy_link_up();
        if link_up != last_link_up {
            if link_up {
                defmt::info!("Ethernet: no link detected");
            } else {
                defmt::info!("Ethernet: link detected!");
            }
            last_link_up = link_up;
        }

        cortex_m::interrupt::free(|cs| {
            let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            *eth_pending = false;
        });

        // handle rx packet
        {
            let mut recvd = 0usize;
            while let Ok(pkt) = dma.recv_next(None) {
                rx_bytes += pkt.len();
                rx_pkts += 1;
                pkt.free();

                recvd += 1;
                if recvd > 16 {
                    // Break arbitrarily to process tx eventually
                    break;
                }
            }
        }
        if !dma.rx_is_running() {
            defmt::info!("RX stopped");
        }

        // fill tx queue
        const SIZE: usize = 1500;
        if phy.phy_link_up() {
            'egress: loop {
                let r = dma.send(SIZE, None, |buf| {
                    buf[0..6].copy_from_slice(&DST_MAC);
                    buf[6..12].copy_from_slice(&SRC_MAC);
                    buf[12..14].copy_from_slice(&ETH_TYPE);
                });

                match r {
                    Ok(()) => {
                        tx_bytes += SIZE;
                        tx_pkts += 1;
                    }
                    Err(TxError::WouldBlock) => break 'egress,
                }
            }
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(100 * SYST::get_ticks_per_10ms());
    syst.enable_counter();
    syst.enable_interrupt();

    if !SYST::is_precise() {
        use cortex_m::peripheral::syst::SystClkSource::*;

        defmt::error!(
            "Warning: SYSTICK with source {:?} is not precise",
            match syst.get_clock_source() {
                Core => "Core",
                External => "External",
            }
        );
    }
}

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        let mut time = TIME.borrow(cs).borrow_mut();
        *time += 1;
    })
}

#[interrupt]
fn ETH() {
    cortex_m::interrupt::free(|cs| {
        let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
        *eth_pending = true;
    });

    stm32_eth::eth_interrupt_handler();
}
