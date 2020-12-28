//! Example assumes use of an STM32F429 connected to a network via a single PHY whose address is
//! `0`. The PHY is expected to be accessible via SMI with an implementation of the standard basic
//! status register as described in the IEEE 802.3 Ethernet standard.

#![no_std]
#![no_main]

extern crate panic_itm;

use core::cell::RefCell;
use core::default::Default;
use cortex_m_rt::{entry, exception};

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use stm32_eth::{
    hal::gpio::{GpioExt, Speed},
    hal::rcc::RccExt,
    hal::time::U32Ext,
    smi,
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
};

use core::fmt::Write;
use cortex_m_semihosting::hio;

use stm32_eth::{Eth, EthPins, RingEntry, TxError};

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
const DST_MAC: [u8; 6] = [0x00, 0x00, 0xBE, 0xEF, 0xDE, 0xAD];
const ETH_TYPE: [u8; 2] = [0x80, 0x00];
const PHY_ADDR: u8 = 0;

static TIME: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let mut stdout = hio::hstdout().unwrap();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    // HCLK must be at least 25MHz to use the ethernet peripheral
    let clocks = rcc.cfgr.sysclk(32.mhz()).hclk(32.mhz()).freeze();

    setup_systick(&mut cp.SYST);

    writeln!(stdout, "Enabling ethernet...").unwrap();
    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();

    let eth_pins = EthPins {
        ref_clk: gpioa.pa1,
        crs: gpioa.pa7,
        tx_en: gpiog.pg11,
        tx_d0: gpiog.pg13,
        tx_d1: gpiob.pb13,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };

    let mut mdio = gpioa.pa2.into_alternate_af11().set_speed(Speed::VeryHigh);
    let mut mdc = gpioc.pc1.into_alternate_af11().set_speed(Speed::VeryHigh);

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 8] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC,
        p.ETHERNET_DMA,
        &mut rx_ring[..],
        &mut tx_ring[..],
        clocks,
        eth_pins,
    )
    .unwrap();
    eth.enable_interrupt();

    // Main loop
    let mut last_stats_time = 0usize;
    let mut rx_bytes = 0usize;
    let mut rx_pkts = 0usize;
    let mut tx_bytes = 0usize;
    let mut tx_pkts = 0usize;
    let mut last_link_up = false;

    loop {
        let time: usize = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());

        // print stats every 30 seconds
        if time >= last_stats_time + 30 {
            let t = time - last_stats_time;
            writeln!(
                stdout,
                "T={}\tRx:\t{} KB/s\t{} pps\tTx:\t{} KB/s\t{} pps",
                time,
                rx_bytes / 1024 / t,
                rx_pkts / t,
                tx_bytes / 1024 / t,
                tx_pkts / t
            )
            .unwrap();
            // Reset
            rx_bytes = 0;
            rx_pkts = 0;
            tx_bytes = 0;
            tx_pkts = 0;
            last_stats_time = time;
        }

        // Link change detection
        let link_up = link_detected(eth.smi(&mut mdio, &mut mdc));
        if link_up != last_link_up {
            if link_up {
                writeln!(stdout, "Ethernet: no link detected").unwrap();
            } else {
                writeln!(stdout, "Ethernet: link detected!").unwrap();
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
            while let Ok(pkt) = eth.recv_next() {
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
        if !eth.rx_is_running() {
            writeln!(stdout, "RX stopped").unwrap();
        }

        // fill tx queue
        const SIZE: usize = 1500;
        if link_detected(eth.smi(&mut mdio, &mut mdc)) {
            'egress: loop {
                let r = eth.send(SIZE, |buf| {
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

        cortex_m::interrupt::free(|cs| {
            let eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            if !*eth_pending {
                asm::wfi();
            }
        });
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(100 * SYST::get_ticks_per_10ms());
    syst.enable_counter();
    syst.enable_interrupt();

    if !SYST::is_precise() {
        let mut stderr = hio::hstderr().unwrap();
        writeln!(
            stderr,
            "Warning: SYSTICK with source {:?} is not precise",
            syst.get_clock_source()
        )
        .unwrap();
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

    // Clear interrupt flags
    let p = unsafe { Peripherals::steal() };
    stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);
}

fn link_detected<Mdio, Mdc>(smi: smi::Smi<Mdio, Mdc>) -> bool
where
    Mdio: smi::MdioPin,
    Mdc: smi::MdcPin,
{
    const STATUS_REG_ADDR: u8 = 0x3;
    const STATUS_REG_UP_MASK: u16 = 1 << 2;
    let status = smi.read(PHY_ADDR, STATUS_REG_ADDR);
    (status & STATUS_REG_UP_MASK) == STATUS_REG_UP_MASK
}
