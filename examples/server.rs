#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! A very rudimentary PTP synchronization example built using RTIC.
//!
//! The example requires that at least two nodes are running at the same time,
//! and the time synchronization that occurs does not explicitly compensate for
//! network delays.
//!
//! All nodes send traffic to a specific MAC address (AB:CD:EF:12:34:56) with an unused
//! EtherType (0xFFFF), containing nothing but the raw value of a [`Timestamp`]. Upon reception
//! of such a frame, the node will parse the timestamp, compare it to when the frame was received
//! according to the local time, and do one of following:
//!
//! 1. If the difference is larger than 20 microseconds, the current local time is set to the
//!    received value.
//! 2. If the difference is smaller than or equal to 20 microseconds, the PTP addend value is updated
//!    to compensate for the observed difference.
//!
//! When using the internal oscillator of an STM32, step 2 will (almost) never occur, as the frequency
//! drift and error with this clock is too great to accurately compensate for. However,
//! if a more accurate High Speed External oscillator is connected to your MCU, even this very basic
//! synchronization scheme can synchronize the rate of time on two nodes to within a few PPMs.
//!
//! To activate the HSE configuration for the examples, set the `STM32_ETH_EXAMPLE_HSE` environment variable
//! to `oscillator` or `bypass` when compiling examples.

use defmt_rtt as _;
use panic_probe as _;
use stm32_eth::dma::PacketId;

mod common;

/// Protocol:
/// 1. Client sends empty message 0x0
/// 2. Server sends empty message 0x01, (client records RX time t1')
/// 3. Server sends message 0x02 with TX time of #2 (client records TX time t1)
/// 4. Client sends empty message 0x03 (client records TX time t2)
/// 5. Server sends message 0x04 with RX time of #4 (client records RX time t2')
/// 6. Server sends empty message 0x05 (client records RX time t3')
/// 7. Server sends message 0x06 with timestamp of #6 (client records TX time t3)

pub struct Server {
    expected_number: u8,

    t1_sent: bool,
    t1_id: Option<PacketId>,

    t3_sent: bool,
    t3_id: Option<PacketId>,
}

impl Default for Server {
    fn default() -> Self {
        Self {
            expected_number: 0,
            t1_sent: false,
            t1_id: None,
            t3_sent: false,
            t3_id: None,
        }
    }
}

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use crate::{common::EthernetPhy, Server};

    use ieee802_3_miim::{phy::PhySpeed, Phy};
    use systick_monotonic::Systick;

    use stm32_eth::{
        dma::{EthernetDMA, RxRingEntry, TxRingEntry},
        mac::Speed,
        ptp::EthernetPTP,
        Parts,
    };

    use smoltcp::{
        iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage},
        socket::udp,
        wire::{
            EthernetAddress, HardwareAddress, IpAddress, IpCidr, IpEndpoint, IpListenEndpoint,
            Ipv4Address,
        },
    };

    const SERVER_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xFF];

    fn now() -> smoltcp::time::Instant {
        let now_micros = monotonics::now().ticks() * 1000;
        smoltcp::time::Instant::from_micros(now_micros as i64)
    }

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        server: Server,
        dma: EthernetDMA<'static, 'static>,
        interface: Interface,
        sockets: SocketSet<'static>,
        udp_socket: SocketHandle,
        ptp: EthernetPTP,
    }

    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    #[init(local = [
        rx_ring: [RxRingEntry; 2] = [RxRingEntry::new(),RxRingEntry::new()],
        tx_ring: [TxRingEntry; 2] = [TxRingEntry::new(),TxRingEntry::new()],
        rx_meta_storage: [udp::PacketMetadata; 8] = [udp::PacketMetadata::EMPTY; 8],
        rx_payload_storage: [u8; 1024] = [0u8; 1024],
        tx_meta_storage: [udp::PacketMetadata; 8] = [udp::PacketMetadata::EMPTY; 8],
        tx_payload_storage: [u8; 1024] = [0u8; 1024],
        sockets: [SocketStorage<'static>; 8] = [SocketStorage::EMPTY; 8],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Pre-init");
        let core = cx.core;
        let p = cx.device;

        let rx_ring = cx.local.rx_ring;
        let tx_ring = cx.local.tx_ring;

        let rx_meta_storage = cx.local.rx_meta_storage;
        let rx_payload_storage = cx.local.rx_payload_storage;
        let tx_meta_storage = cx.local.tx_meta_storage;
        let tx_payload_storage = cx.local.tx_payload_storage;
        let sockets = cx.local.sockets;

        let (clocks, gpio, ethernet) = crate::common::setup_peripherals(p);
        let mono = Systick::new(core.SYST, clocks.hclk().raw());

        defmt::info!("Setting up pins");
        let (pins, mdio, mdc, pps) = crate::common::setup_pins(gpio);

        defmt::info!("Configuring ethernet");

        let Parts {
            mut dma,
            mac,
            mut ptp,
        } = stm32_eth::new_with_mii(ethernet, rx_ring, tx_ring, clocks, pins, mdio, mdc).unwrap();

        ptp.enable_pps(pps);

        let mut cfg = Config::new();
        cfg.hardware_addr = Some(HardwareAddress::Ethernet(EthernetAddress(SERVER_ADDR)));

        let mut interface = Interface::new(cfg, &mut &mut dma);
        interface.update_ip_addrs(|a| {
            a.push(IpCidr::new(IpAddress::v4(10, 0, 0, 1), 24)).ok();
        });

        let rx_buffer =
            udp::PacketBuffer::new(&mut rx_meta_storage[..], &mut rx_payload_storage[..]);
        let tx_buffer =
            udp::PacketBuffer::new(&mut tx_meta_storage[..], &mut tx_payload_storage[..]);
        let mut udp_socket = udp::Socket::new(rx_buffer, tx_buffer);

        udp_socket
            .bind(IpListenEndpoint {
                addr: None,
                port: 1337,
            })
            .ok()
            .unwrap();

        let mut sockets = SocketSet::new(&mut sockets[..]);
        let udp_socket = sockets.add(udp_socket);

        defmt::info!("Enabling interrupts");
        dma.enable_interrupt();

        match EthernetPhy::from_miim(mac, 0) {
            Ok(mut phy) => {
                defmt::info!(
                    "Resetting PHY as an extra step. Type: {}",
                    phy.ident_string()
                );

                phy.phy_init();

                defmt::info!("Waiting for link up.");

                while !phy.phy_link_up() {}

                defmt::info!("Link up.");

                if let Some(speed) = phy.speed().map(|s| match s {
                    PhySpeed::HalfDuplexBase10T => Speed::HalfDuplexBase10T,
                    PhySpeed::FullDuplexBase10T => Speed::FullDuplexBase10T,
                    PhySpeed::HalfDuplexBase100Tx => Speed::HalfDuplexBase100Tx,
                    PhySpeed::FullDuplexBase100Tx => Speed::FullDuplexBase100Tx,
                }) {
                    phy.get_miim().set_speed(speed);
                    defmt::info!("Detected link speed: {}", speed);
                } else {
                    defmt::warn!("Failed to detect link speed.");
                }
            }
            Err(_) => {
                defmt::info!("Not resetting unsupported PHY. Cannot detect link speed.");
            }
        };

        (
            Shared {
                dma,
                ptp,
                server: Default::default(),
                interface,
                sockets,
                udp_socket,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    #[task(binds = ETH, shared = [dma, ptp, server, interface, sockets, udp_socket], local = [tx_id_ctr: u32 = 0x8000_0000], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (dma, server, ptp, interface, sockets, udp_socket) = (
            cx.shared.dma,
            cx.shared.server,
            cx.shared.ptp,
            cx.shared.interface,
            cx.shared.sockets,
            cx.shared.udp_socket,
        );

        let recv_data =
            |udp_socket: &mut udp::Socket, copy_buf: &mut [u8], dma: &mut EthernetDMA<'_, '_>| {
                if let Ok((buf, meta)) = udp_socket.recv() {
                    copy_buf[..buf.len()].copy_from_slice(buf);

                    let timestamp = dma
                        .get_timestamp_for_id(meta.packet_id().unwrap())
                        .ok()
                        .unwrap();

                    Some((buf.len(), timestamp))
                } else {
                    None
                }
            };

        let send_data = |iface: &mut Interface, udp_socket: &mut udp::Socket, buf: &[u8]| {
            let (buffer, tx_id) = udp_socket
                .send_marked(
                    iface,
                    buf.len(),
                    IpEndpoint {
                        addr: IpAddress::Ipv4(Ipv4Address([10, 0, 0, 2])),
                        port: 1337,
                    },
                )
                .unwrap();
            buffer.copy_from_slice(buf);
            tx_id
        };

        (dma, ptp, server, interface, sockets, udp_socket).lock(
            |mut dma, ptp, server, interface, sockets, udp_socket| {
                dma.interrupt_handler();
                ptp.interrupt_handler();

                while interface.poll(now(), &mut dma, sockets) {}

                let udp_socket = sockets.get_mut::<udp::Socket>(*udp_socket);

                if let (false, Some(t1_id)) = (server.t1_sent, &mut server.t1_id) {
                    if let Some(t1) = dma.get_timestamp_for_id(t1_id.clone()).ok() {
                        // Step 3
                        // Server sends message 0x02 with the TX time of #2
                        server.expected_number = 0x03;

                        defmt::info!("Step 3");

                        let mut buffer = [0u8; 42];
                        buffer[0] = 0x02;
                        buffer[1..9].copy_from_slice(&t1.raw().to_le_bytes());
                        send_data(interface, udp_socket, &buffer[..9]);
                        server.t1_sent = true;
                    } else {
                        defmt::error!("Didn't get TX id... (Step 3)");
                    }
                } else if let Some(t3_id) = &mut server.t3_id {
                    if let Some(t3) = dma.get_timestamp_for_id(t3_id.clone()).ok() {
                        // Step 7.
                        // Server sends message 0x06 with the timestsamp of #6
                        let mut buffer = [0u8; 9];
                        buffer[0] = 0x06;
                        buffer[1..9].copy_from_slice(&t3.raw().to_le_bytes());
                        send_data(interface, udp_socket, &buffer[..9]);
                        server.t3_sent = true;

                        defmt::info!("Step 7");

                        // We're done, reset
                        *server = Default::default();
                    } else {
                        defmt::error!("Didn'tget TX id... (Step 7)")
                    }
                }

                let mut buf = [0u8; 60];
                while let Some((data, rx_timestamp)) = recv_data(udp_socket, &mut buf, dma) {
                    if data <= 0 {
                        return;
                    }

                    let msg_id = buf[0];

                    // Verify that we're receiving the correct message.
                    if msg_id != server.expected_number {
                        *server = Default::default();
                        udp_socket.close();
                        udp_socket
                            .bind(IpListenEndpoint {
                                addr: None,
                                port: 1337,
                            })
                            .ok()
                            .unwrap();

                        defmt::error!("Got unexpected message {}", msg_id);
                    } else {
                        defmt::info!("Got {}", msg_id);
                        // Advance to the next message.
                        server.expected_number += 1;
                    }

                    if msg_id == 0 {
                        // Step 1
                        // Step 2
                        // Server sends empty message 0x01.

                        defmt::info!("Step 1");
                        defmt::info!("Step 2");
                        let t1_id = send_data(interface, udp_socket, &[0x1]);
                        server.t1_id = Some(t1_id.into());
                        server.t1_sent = false;
                    } else if msg_id == 0x03 {
                        // Step 4
                        // Step 5
                        defmt::info!("Step 4");
                        defmt::info!("Step 5");
                        let mut buffer = [0u8; 9];
                        buffer[0] = 0x04;
                        buffer[1..9].copy_from_slice(&rx_timestamp.raw().to_le_bytes());
                        send_data(interface, udp_socket, &buffer[..9]);

                        defmt::info!("Step 6");
                        // Step 6
                        buffer[0] = 0x05;
                        let t3_id = send_data(interface, udp_socket, &buffer[..1]);
                        server.t3_id = Some(t3_id.into());
                    }
                }

                while interface.poll(now(), &mut dma, sockets) {}
            },
        );
    }
}
