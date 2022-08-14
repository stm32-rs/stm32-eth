#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! This example starts a TCP listening server that should transmit `Hello` to
//! any connecting client, and then close the connection.

use defmt_rtt as _;
use panic_probe as _;

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use stm32_eth::stm32::{interrupt, CorePeripherals, Peripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use smoltcp::iface::{InterfaceBuilder, NeighborCache};
use smoltcp::socket::{TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};

pub mod common;

use stm32_eth::RingEntry;

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let (clocks, gpio, ethernet) = common::setup_peripherals(p);

    setup_systick(&mut cp.SYST);

    defmt::info!("Enabling ethernet...");

    let (eth_pins, _mdio, _mdc) = common::setup_pins(gpio);

    let mut rx_ring: [RingEntry<_>; 8] = Default::default();
    let mut tx_ring: [RingEntry<_>; 2] = Default::default();
    let (mut eth_dma, _eth_mac) = stm32_eth::new(
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

    let local_addr = Ipv4Address::new(10, 0, 0, 1);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let ethernet_addr = EthernetAddress(SRC_MAC);

    let mut sockets: [_; 1] = Default::default();
    let mut iface = InterfaceBuilder::new(&mut eth_dma, &mut sockets[..])
        .hardware_addr(ethernet_addr.into())
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();

    let mut server_rx_buffer = [0; 2048];
    let mut server_tx_buffer = [0; 2048];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    let server_handle = iface.add_socket(server_socket);

    defmt::info!("Ready, listening at {}", ip_addr);
    loop {
        let time: u64 = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());
        cortex_m::interrupt::free(|cs| {
            let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            *eth_pending = false;
        });
        match iface.poll(Instant::from_millis(time as i64)) {
            Ok(true) => {
                let socket = iface.get_socket::<TcpSocket>(server_handle);
                if !socket.is_open() {
                    if let Err(e) = socket.listen(80) {
                        defmt::error!("TCP listen error: {:?}", e)
                    }
                }

                if socket.can_send() {
                    if let Err(e) = socket.send_slice(b"hello\n").map(|_| socket.close()) {
                        defmt::info!("TCP send error: {:?}", e);
                    }
                }
            }
            Ok(false) => {
                // Sleep if no ethernet work is pending
                cortex_m::interrupt::free(|cs| {
                    let eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
                    if !*eth_pending {
                        asm::wfi();
                        // Awaken by interrupt
                    }
                });
            }
            Err(e) =>
            // Ignore malformed packets
            {
                defmt::info!("Error: {:?}", e);
            }
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
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
