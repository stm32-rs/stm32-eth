// cargo build --example arp --features=stm32f407
// This example uses the STM32F407 and the KSZ8051R as PHY. If necessary the pins,
// the PHY register addresses and masks have to be adapted, as well as the IPs.
// With Wireshark, you can see the ARP packets, which should look like this:
// No.  Time        Source          Destination     Protocol    Length  Info
// 1	0.000000000	Cetia_ad:be:ef	Broadcast	    ARP	        60	    Who has 10.0.0.2? Tell 10.0.0.10

#![no_std]
#![no_main]

extern crate panic_itm;

use core::cell::RefCell;
use core::default::Default;
use cortex_m_rt::{entry, exception};

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use fugit::RateExtU32;
use stm32_eth::{
    hal::gpio::{GpioExt, Speed},
    hal::rcc::RccExt,
    mac::{phy::BarePhy, Phy},
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
};

use cortex_m_semihosting::hprintln;

use stm32_eth::{EthPins, RingEntry, TxError};

const PHY_ADDR: u8 = 0;

static TIME: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    // HCLK must be at least 25MHz to use the ethernet peripheral
    let clocks = rcc.cfgr.sysclk(32.MHz()).hclk(32.MHz()).freeze();

    setup_systick(&mut cp.SYST);

    hprintln!("Enabling ethernet...").unwrap();
    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();

    let eth_pins = EthPins {
        ref_clk: gpioa.pa1,
        crs: gpioa.pa7,
        tx_en: gpiob.pb11,
        tx_d0: gpiog.pg13,
        tx_d1: gpiog.pg14,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };

    let mut mdio = gpioa.pa2.into_alternate();
    mdio.set_speed(Speed::VeryHigh);
    let mut mdc = gpioc.pc1.into_alternate();
    mdc.set_speed(Speed::VeryHigh);

    // ETH_PHY_RESET(RST#) PB2 Chip Reset (active-low)
    let _eth_reset = gpiob.pb2.into_push_pull_output().set_high();

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 8] = Default::default();
    let (mut eth_dma, eth_mac) = stm32_eth::new(
        p.ETHERNET_MAC,
        p.ETHERNET_MMC,
        p.ETHERNET_DMA,
        &mut rx_ring[..],
        &mut tx_ring[..],
        clocks,
        eth_pins,
    )
    .unwrap();
    eth_dma.enable_interrupt();

    let mut last_link_up = false;

    let mut bare_phy = BarePhy::new(eth_mac.with_mii(mdio, mdc), PHY_ADDR, Default::default());

    loop {
        let link_up = bare_phy.phy_link_up();

        if link_up != last_link_up {
            if link_up {
                hprintln!("Ethernet: link detected").unwrap();
            } else {
                hprintln!("Ethernet: no link detected").unwrap();
            }
            last_link_up = link_up;
        }

        if link_up {
            const SIZE: usize = 42;

            const DST_MAC: [u8; 6] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
            const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
            const ETH_TYPE: [u8; 2] = [0x08, 0x06]; // ARP
            const HTYPE: [u8; 2] = [0x00, 0x01]; // Hardware Type: ethernet
            const PTYPE: [u8; 2] = [0x08, 0x00]; // IP
            const HLEN: [u8; 1] = [0x06]; // MAC length
            const PLEN: [u8; 1] = [0x04]; // IPv4
            const OPER: [u8; 2] = [0x00, 0x01]; // Operation: request
            const SENDER_IP: [u8; 4] = [0x0A, 00, 0x00, 0x0A]; // 10.0.0.10
            const TARGET_MAC: [u8; 6] = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
            const TARGET_IP: [u8; 4] = [0x0A, 0x00, 0x00, 0x02]; // 10.0.0.2

            let r = eth_dma.send(SIZE, |buf| {
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
