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

use core::ops::Neg;

use defmt_rtt as _;
use panic_probe as _;
use stm32_eth::{dma::PacketId, ptp::Timestamp};

mod common;

/// Protocol:
/// 1. Client sends empty message 0x0
/// 2. Server sends empty message 0x01, (client records RX time t1')
/// 3. Server sends message 0x02 with TX time of #2 (client records TX time t1)
/// 4. Client sends empty message 0x03 (client records TX time t2)
/// 5. Server sends message 0x04 with RX time of #4 (client records RX time t2')
/// 6. Server sends empty message 0x05 (client records RX time t3')
/// 7. Server sends message 0x06 with timestamp of #6 (client records TX time t3)

#[derive(defmt::Format)]
pub struct Client {
    expected_number: u8,

    // Message 0x01
    t1_prim: Option<Timestamp>, // t1'
    // Message 0x02
    t1: Option<Timestamp>, // t1

    t2_packet_id: Option<PacketId>, // packet ID for t2
    // Message 0x03
    t2: Option<Timestamp>, // t2
    // Message 0x04
    t2_prim: Option<Timestamp>, // t2'

    // Message 0x05
    t3_prim: Option<Timestamp>, // t3'
    t3: Option<Timestamp>,      // t3
}

impl Default for Client {
    fn default() -> Self {
        Self {
            expected_number: 0,
            t1: None,
            t1_prim: None,
            t2_packet_id: None,
            t2: None,
            t2_prim: None,
            t3: None,
            t3_prim: None,
        }
    }
}

impl Client {
    pub fn print_timestamps(&self) {
        defmt::debug!("Timestamp information: ");
        defmt::debug!("t1:  {}", self.t1);
        defmt::debug!("t1': {}", self.t1_prim);
        defmt::debug!("t2:  {}", self.t2);
        defmt::debug!("t2': {}", self.t2_prim);
        defmt::debug!("t3:  {}", self.t3);
        defmt::debug!("t3': {}", self.t3_prim);
    }

    pub fn calculate_offset(&self) -> Option<Timestamp> {
        let t1 = self.t1?;
        let t1_prim = self.t1_prim?;
        let t2 = self.t2?;
        let t2_prim = self.t2_prim?;
        let _t3 = self.t3?;
        let _t3_prim = self.t3_prim?;

        let double_offset = t1_prim - t1 - t2_prim + t2;

        let raw = double_offset.raw();
        let offset = (raw / 2).neg();
        let offset = Timestamp::new_raw(offset);

        Some(offset)
    }
}

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use fugit::ExtU64;
    use smoltcp::{
        iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage},
        socket::udp,
        wire::{
            EthernetAddress, HardwareAddress, IpAddress, IpCidr, IpEndpoint, IpListenEndpoint,
            Ipv4Address,
        },
    };

    use crate::{common::EthernetPhy, Client};

    use ieee802_3_miim::{phy::PhySpeed, Phy};
    use systick_monotonic::Systick;

    use stm32_eth::{
        dma::{EthernetDMA, RxRingEntry, TxRingEntry},
        mac::Speed,
        ptp::{EthernetPTP, Timestamp},
        Parts,
    };

    const CLIENT_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    fn now() -> smoltcp::time::Instant {
        let now_micros = monotonics::now().ticks() * 1000;
        smoltcp::time::Instant::from_micros(now_micros as i64)
    }

    #[local]
    struct Local {
        start_addend: u32,
    }

    #[shared]
    struct Shared {
        client: Client,
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
        let start_addend = ptp.addend();

        let mut cfg = Config::new();
        cfg.hardware_addr = Some(HardwareAddress::Ethernet(EthernetAddress(CLIENT_ADDR)));

        let mut interface = Interface::new(cfg, &mut &mut dma);
        interface.update_ip_addrs(|a| {
            a.push(IpCidr::new(IpAddress::v4(10, 0, 0, 2), 24)).ok();
        });

        let rx_buffer =
            udp::PacketBuffer::new(&mut rx_meta_storage[..], &mut rx_payload_storage[..]);
        let tx_buffer =
            udp::PacketBuffer::new(&mut tx_meta_storage[..], &mut tx_payload_storage[..]);
        let udp_socket = udp::Socket::new(rx_buffer, tx_buffer);

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

        sender::spawn().ok();

        (
            Shared {
                dma,
                ptp,
                client: Default::default(),
                interface,
                sockets,
                udp_socket,
            },
            Local { start_addend },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [dma, ptp, client, interface, sockets, udp_socket])]
    fn sender(cx: sender::Context) {
        sender::spawn_after(50.millis()).ok();

        let (dma, client, interface, sockets, udp_socket) = (
            cx.shared.dma,
            cx.shared.client,
            cx.shared.interface,
            cx.shared.sockets,
            cx.shared.udp_socket,
        );

        (interface, sockets, dma, client, udp_socket).lock(
            |iface, sockets, mut dma, client, udp_socket| {
                let socket = sockets.get_mut::<udp::Socket>(*udp_socket);

                *client = Default::default();
                socket.close();
                socket
                    .bind(IpListenEndpoint {
                        addr: None,
                        port: 1337,
                    })
                    .ok()
                    .unwrap();

                // Step 1
                // Client sends empty message 0x0
                defmt::info!("Starting exchange...");
                defmt::trace!("Step 1");
                let (buffer, _packet_id) = socket
                    .send_marked(
                        iface,
                        1,
                        IpEndpoint {
                            addr: IpAddress::Ipv4(Ipv4Address([10, 0, 0, 1])),
                            port: 1337,
                        },
                    )
                    .unwrap();
                buffer[0] = 0x0;
                client.expected_number = 1;
                iface.poll(now(), &mut dma, sockets);
            },
        );
    }

    #[task(binds = ETH, shared = [dma, ptp, client, interface, sockets, udp_socket], local = [start_addend, addend_integrator: f32 = 0.0], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (dma, client, ptp, addend_integrator, interface, sockets, udp_socket) = (
            cx.shared.dma,
            cx.shared.client,
            cx.shared.ptp,
            cx.local.addend_integrator,
            cx.shared.interface,
            cx.shared.sockets,
            cx.shared.udp_socket,
        );

        let start_addend = *cx.local.start_addend;

        (dma, ptp, client, interface, sockets, udp_socket).lock(
            |mut dma, ptp, client, iface, sockets, udp_socket| {
                use core::convert::TryInto;

                dma.interrupt_handler();
                ptp.interrupt_handler();

                while iface.poll(now(), &mut dma, sockets) {}

                let udp_socket = sockets.get_mut::<udp::Socket>(*udp_socket);

                let recv_data = |udp_socket: &mut udp::Socket,
                                 copy_buf: &mut [u8],
                                 dma: &mut EthernetDMA<'_, '_>| {
                    if let Ok((buf, meta)) = udp_socket.recv() {
                        copy_buf[..buf.len()].copy_from_slice(buf);

                        if let Some(timestamp) =
                            dma.get_timestamp_for_id(meta.packet_id().unwrap()).ok()
                        {
                            Some((buf.len(), timestamp))
                        } else {
                            defmt::error!("Failed to obtain RX timestamp.");
                            None
                        }
                    } else {
                        None
                    }
                };

                let send_data =
                    |iface: &mut Interface, udp_socket: &mut udp::Socket, buf: &[u8]| {
                        let (buffer, tx_id) = udp_socket
                            .send_marked(
                                iface,
                                buf.len(),
                                IpEndpoint {
                                    addr: IpAddress::Ipv4(Ipv4Address([10, 0, 0, 1])),
                                    port: 1337,
                                },
                            )
                            .unwrap();
                        buffer.copy_from_slice(buf);
                        tx_id
                    };

                if let (true, Some(resp_packet_id), None) =
                    (client.t2.is_none(), &mut client.t2_packet_id, client.t2)
                {
                    // Step 4
                    // Client records TX time t2
                    defmt::trace!("Step 4");
                    client.t2 =
                        if let Some(tx_ts) = dma.get_timestamp_for_id(resp_packet_id.clone()).ok() {
                            Some(tx_ts)
                        } else {
                            defmt::error!("Did not get TX timestamp... (Step 4)");
                            None
                        }
                }

                let mut buf = [0u8; 60];
                while let Some((data, rx_timestamp)) = recv_data(udp_socket, &mut buf, dma) {
                    if data <= 0 {
                        return;
                    }

                    let msg_id = buf[0];
                    let data = &buf[1..data];

                    // Verify that we're receiving the correct message.
                    if msg_id != client.expected_number {
                        *client = Default::default();
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
                        // Advance to the next message.
                        client.expected_number += 1;
                    }

                    if msg_id == 0x01 {
                        // Step 2
                        // Client records RX time t1'
                        defmt::trace!("Step 2");
                        client.t1_prim = Some(rx_timestamp);
                    } else if msg_id == 0x02 {
                        let timestamp = Timestamp::new_raw(i64::from_le_bytes(
                            data[0..8].try_into().ok().unwrap(),
                        ));
                        // Step 3
                        // Client records TX time t1
                        defmt::trace!("Step 3");
                        client.t1 = Some(timestamp);
                        client.t2_packet_id = Some(send_data(iface, udp_socket, &[0x03]).into());
                        client.expected_number = 0x04;
                    } else if msg_id == 0x04 {
                        let timestamp = Timestamp::new_raw(i64::from_le_bytes(
                            data[0..8].try_into().ok().unwrap(),
                        ));
                        // Step 5
                        // Client records RX time t2'
                        defmt::trace!("Step 5");
                        client.t2_prim = Some(timestamp);
                    } else if msg_id == 0x05 {
                        // Step 6
                        // Client records RX time t3'
                        defmt::trace!("Step 6");
                        client.t3_prim = Some(rx_timestamp);
                    } else if msg_id == 0x06 {
                        // Step 7
                        // Client records TX time t3
                        defmt::trace!("Step 7");
                        let timestamp = Timestamp::new_raw(i64::from_le_bytes(
                            data[0..8].try_into().ok().unwrap(),
                        ));
                        client.t3 = Some(timestamp);

                        client.print_timestamps();

                        let now = ptp.get_time();
                        if let Some(offset) = client.calculate_offset() {
                            if offset.seconds() > 0 || offset.nanos() > 200_000 {
                                *addend_integrator = 0.0;
                                defmt::info!("Updating time. Offset {} ", offset);
                                let updated_time = now + offset;
                                ptp.set_time(updated_time);
                            } else {
                                let mut offset_nanos = offset.nanos() as i64;
                                if offset.is_negative() {
                                    offset_nanos *= -1;
                                }

                                let error = (offset_nanos * start_addend as i64) / 1_000_000_000;
                                *addend_integrator += error as f32 / 500.;

                                defmt::info!(
                                    "Error: {}. Integrator: {}, Offset: {} ns",
                                    error,
                                    addend_integrator,
                                    offset_nanos
                                );

                                let new_addend =
                                    (start_addend as i64 + error / 4 + (*addend_integrator as i64))
                                        as u32;
                                ptp.set_addend(new_addend);
                            }
                        }

                        *client = Default::default();
                    }
                }

                while iface.poll(now(), &mut dma, sockets) {}
            },
        );
    }
}
