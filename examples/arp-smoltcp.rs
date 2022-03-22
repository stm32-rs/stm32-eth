// cargo build --example arp-smoltcp --features=stm32f407,smi,smoltcp-phy,smoltcp/socket-tcp,smoltcp/socket-icmp
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

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
use fugit::RateExtU32;
use smoltcp::wire::{
    ArpOperation, ArpPacket, ArpRepr, EthernetAddress, EthernetFrame, EthernetProtocol,
    EthernetRepr, Ipv4Address,
};
use stm32_eth::{
    hal::gpio::{GpioExt, Speed},
    hal::rcc::RccExt,
    // hal::time::U32Ext,
    smi,
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
};
use stm32_eth::{Eth, EthPins, RingEntry, TxError};

const PHY_REG_BSR: u8 = 0x01;
const PHY_REG_BSR_UP: u16 = 1 << 2;

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

    let mut mdio = gpioa.pa2.into_alternate().set_speed(Speed::VeryHigh);
    let mut mdc = gpioc.pc1.into_alternate().set_speed(Speed::VeryHigh);

    // ETH_PHY_RESET(RST#) PB2 Chip Reset (active-low)
    let _eth_reset = gpiob.pb2.into_push_pull_output().set_high();

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

    let mut last_link_up = false;

    loop {
        let link_up = link_detected(eth.smi(&mut mdio, &mut mdc));

        if link_up != last_link_up {
            if link_up {
                hprintln!("Ethernet: link detected").unwrap();
            } else {
                hprintln!("Ethernet: no link detected").unwrap();
            }
            last_link_up = link_up;
        }

        if link_up {
            const SIZE: usize = 14 + 28; // ETH + ARP

            let src_mac = EthernetAddress::from_bytes(&[0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF]);

            let arp_buffer = [0; 28];
            let mut packet =
                ArpPacket::new_checked(arp_buffer).expect("ArpPacket: buffer size is not correct");
            let arp = ArpRepr::EthernetIpv4 {
                operation: ArpOperation::Request,
                source_hardware_addr: src_mac,
                source_protocol_addr: Ipv4Address::new(10, 0, 0, 10),
                target_hardware_addr: EthernetAddress::from_bytes(&[0x00; 6]),
                target_protocol_addr: Ipv4Address::new(10, 0, 0, 2),
            };
            arp.emit(&mut packet);

            let eth_buffer = [0; SIZE]; // ETH + ARP
            let mut frame = EthernetFrame::new_checked(eth_buffer)
                .expect("EthernetFrame: buffer size is not correct");
            let header = EthernetRepr {
                src_addr: src_mac,
                dst_addr: EthernetAddress::BROADCAST,
                ethertype: EthernetProtocol::Arp,
            };
            header.emit(&mut frame);
            frame.payload_mut().copy_from_slice(&packet.into_inner());

            let r = eth.send(SIZE, |buf| {
                buf[0..SIZE].copy_from_slice(&frame.into_inner());
            });

            match r {
                Ok(()) => {
                    hprintln!("ARP-smoltcp sent").unwrap();
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

fn link_detected<Mdio, Mdc>(smi: smi::Smi<Mdio, Mdc>) -> bool
where
    Mdio: smi::MdioPin,
    Mdc: smi::MdcPin,
{
    let status = smi.read(PHY_ADDR, PHY_REG_BSR);
    (status & PHY_REG_BSR_UP) == PHY_REG_BSR_UP
}
