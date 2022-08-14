//! For build and run instructions, see README.md
//!
//! With Wireshark, you can see the ARP packets, which should look like this:
//! No.  Time        Source          Destination     Protocol    Length  Info
//! 1	0.000000000	Cetia_ad:be:ef	Broadcast	    ARP	        60	    Who has 10.0.0.2? Tell 10.0.0.10

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use core::default::Default;

use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use smoltcp::wire::{
    ArpOperation, ArpPacket, ArpRepr, EthernetAddress, EthernetFrame, EthernetProtocol,
    EthernetRepr, Ipv4Address,
};
use stm32_eth::{
    mac::{phy::BarePhy, Phy},
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
};
use stm32_eth::{RingEntry, TxError};

pub mod common;

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

    let (eth_pins, mdio, mdc) = common::setup_pins(gpio);

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 8] = Default::default();
    let (mut eth_dma, eth_mac) = stm32_eth::new(
        ethernet.mac,
        ethernet.mmc,
        ethernet.dma,
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
                defmt::info!("Ethernet: link detected");
            } else {
                defmt::info!("Ethernet: no link detected");
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

            let r = eth_dma.send(SIZE, |buf| {
                buf[0..SIZE].copy_from_slice(&frame.into_inner());
            });

            match r {
                Ok(()) => {
                    defmt::info!("ARP-smoltcp sent");
                }
                Err(TxError::WouldBlock) => defmt::info!("ARP failed"),
            }
        } else {
            defmt::info!("Down");
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
