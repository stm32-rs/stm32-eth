#![no_std]
#![no_main]

extern crate panic_itm;

use core::cell::RefCell;
use core::default::Default;
use cortex_m_rt::{entry, exception};

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use stm32_eth::{
    hal::gpio::GpioExt,
    hal::rcc::RccExt,
    hal::time::U32Ext,
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
};

use cortex_m_semihosting::hprintln;

use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry, TxError};

static TIME: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    // HCLK must be at least 25MHz to use the ethernet peripheral
    let clocks = rcc.cfgr.sysclk(32.mhz()).hclk(32.mhz()).freeze();

    setup_systick(&mut cp.SYST);

    hprintln!("Enabling ethernet...").unwrap();
    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    // let gpiog = p.GPIOG.split();

    let eth_pins = EthPins {
        ref_clk: gpioa.pa1,
        md_io: gpioa.pa2,
        md_clk: gpioc.pc1,
        crs: gpioa.pa7,
        tx_en: gpiob.pb11,
        tx_d0: gpiob.pb12,
        tx_d1: gpiob.pb13,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 8] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC,
        p.ETHERNET_DMA,
        &mut rx_ring[..],
        &mut tx_ring[..],
        PhyAddress::_1,
        clocks,
        eth_pins,
    )
    .unwrap();
    eth.enable_interrupt();

    let mut last_status = None;

    loop {
        let status = eth.status();

        if last_status
            .map(|last_status| last_status != status)
            .unwrap_or(true)
        {
            if !status.link_detected() {
                hprintln!("Ethernet: no link detected").unwrap();
            } else {
                hprintln!(
                    "Ethernet: link detected with {} Mbps/{}",
                    status.speed(),
                    match status.is_full_duplex() {
                        Some(true) => "FD",
                        Some(false) => "HD",
                        None => "?",
                    }
                )
                .unwrap();
            }

            last_status = Some(status);
        }

        if status.link_detected() {
            const SIZE: usize = 42;

            const DST_MAC: [u8; 6] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
            const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
            const ETH_TYPE: [u8; 2] = [0x08, 0x06]; // ARP
            const HTYPE: [u8; 2] = [0x00, 0x01];
            const PTYPE: [u8; 2] = [0x08, 0x00]; // IP
            const HLEN: [u8; 1] = [0x06]; // MAC length
            const PLEN: [u8; 1] = [0x04]; // IPv4
            const OPER: [u8; 2] = [0x00, 0x01];
            const SENDER_IP: [u8; 4] = [0xc0, 0xa8, 0x01, 0x64]; // 192.168.1.100
            const TARGET_MAC: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
            const TARGET_IP: [u8; 4] = [0xc0, 0xa8, 0x01, 0xFE];

            let r = eth.send(SIZE, |buf| {
                buf[0..6].copy_from_slice(&DST_MAC);
                buf[6..12].copy_from_slice(&SRC_MAC);
                buf[12..14].copy_from_slice(&ETH_TYPE);
                buf[14..16].copy_from_slice(&HTYPE);

                buf[16..18].copy_from_slice(&PTYPE);
                buf[18..19].copy_from_slice(&HLEN);
                buf[19..20].copy_from_slice(&PLEN);
                buf[20..22].copy_from_slice(&OPER);
                buf[22..28].copy_from_slice(&SRC_MAC);
                buf[28..32].copy_from_slice(&SENDER_IP);

                buf[32..38].copy_from_slice(&TARGET_MAC);
                buf[38..42].copy_from_slice(&TARGET_IP);
            });

            match r {
                Ok(()) => {
                    hprintln!("ARP sent").unwrap();
                }
                Err(TxError::WouldBlock) => hprintln!("ARP failed").unwrap(),
            }
        } else {
            hprintln!("Down").unwrap();
        }

        cortex_m::interrupt::free(|cs| {
            let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            *eth_pending = false;

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
