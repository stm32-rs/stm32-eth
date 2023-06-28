#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! This example starts a TCP listening server at the address 10.0.0.1/24, on port 80, that
//!  should transmit `hello` to any connecting client, and then close the connection.

use defmt_rtt as _;
use panic_probe as _;

use cortex_m_rt::{entry, exception};
use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use stm32_eth::stm32::{interrupt, CorePeripherals, Peripherals, SYST};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use smoltcp::socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpCidr, Ipv4Address, Ipv4Cidr};

pub mod common;

use stm32_eth::{
    dma::{RxRingEntry, TxRingEntry},
    Parts,
};

const IP_ADDRESS: Ipv4Address = Ipv4Address::new(10, 0, 0, 1);
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

    let (eth_pins, _mdio, _mdc, _) = common::setup_pins(gpio);

    let mut rx_ring: [RxRingEntry; 2] = Default::default();
    let mut tx_ring: [TxRingEntry; 2] = Default::default();
    let Parts {
        mut dma,
        mac: _,
        #[cfg(feature = "ptp")]
            ptp: _,
    } = stm32_eth::new(
        ethernet,
        &mut rx_ring[..],
        &mut tx_ring[..],
        clocks,
        eth_pins,
    )
    .unwrap();
    dma.enable_interrupt();

    let ethernet_addr = EthernetAddress(SRC_MAC);

    let config = Config::new(ethernet_addr.into());
    let mut iface = Interface::new(config, &mut &mut dma, Instant::ZERO);

    iface.update_ip_addrs(|addr| {
        addr.push(IpCidr::Ipv4(Ipv4Cidr::new(IP_ADDRESS, 24))).ok();
    });

    let mut sockets = [SocketStorage::EMPTY];
    let mut sockets = SocketSet::new(&mut sockets[..]);

    let mut server_rx_buffer = [0; 512];
    let mut server_tx_buffer = [0; 512];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    let server_handle = sockets.add(server_socket);

    loop {
        let time: u64 = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());
        cortex_m::interrupt::free(|cs| {
            let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            *eth_pending = false;
        });

        iface.poll(
            Instant::from_millis(time as i64),
            &mut &mut dma,
            &mut sockets,
        );

        let socket = sockets.get_mut::<TcpSocket>(server_handle);

        if !socket.is_listening() && !socket.is_open() {
            socket.abort();
            if let Err(e) = socket.listen(80) {
                defmt::error!("TCP listen error: {:?}", e)
            } else {
                defmt::info!("Listening at {}:80...", IP_ADDRESS);
            }
        } else {
            match socket.send_slice(b"hello\n") {
                Ok(_) => {
                    while sockets.get::<TcpSocket>(server_handle).send_queue() != 0 {
                        // Poll to get the message out of the door
                        iface.poll(
                            Instant::from_millis(time as i64 + 1),
                            &mut &mut dma,
                            &mut sockets,
                        );
                    }

                    // Abort the connection
                    let socket = sockets.get_mut::<TcpSocket>(server_handle);
                    socket.abort();
                    defmt::info!("Transmitted hello! Closing socket...");

                    iface.poll(
                        Instant::from_millis(time as i64),
                        &mut &mut dma,
                        &mut sockets,
                    );
                }
                Err(_) => {}
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

    stm32_eth::eth_interrupt_handler();
}
