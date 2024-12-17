#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! A very rudimentary PTP synchronization example built using RTIC & smoltcp.
//!
//! For this to work, it requires that another node is running the server example.
//!
//! For more information, see the description of examples/timesync/client.rs

use core::ops::Neg;

use defmt_rtt as _;
use panic_probe as _;
use stm32_eth::ptp::Timestamp;

#[path = "../common.rs"]
mod common;

/// Protocol:
/// 1. Client sends empty message 0x0
/// 2. Server sends empty message 0x01, (client records RX time t1')
/// 3. Server sends message 0x02 with TX time of #2 (client records TX time t1)
/// 4. Client sends empty message 0x03 (client records TX time t2)
/// 5. Server sends message 0x04 with RX time of #4 (client records RX time t2')
/// 6. Server sends empty message 0x05 (client records RX time t3')
/// 7. Server sends message 0x06 with timestamp of #6 (client records TX time t3)

pub fn calculate_offset(
    t1: Timestamp,
    t1_prim: Timestamp,
    t2: Timestamp,
    t2_prim: Timestamp,
) -> Timestamp {
    let double_offset = t1_prim - t1 - t2_prim + t2;

    let raw = double_offset.raw();
    let offset = (raw / 2).neg();
    let offset = Timestamp::new_raw(offset);

    offset
}

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use core::task::Poll;

    use smoltcp::{
        iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage},
        socket::udp,
        wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint, IpListenEndpoint, Ipv4Address},
    };

    use crate::common::EthernetPhy;

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
        ptp.set_pps_freq(10);
        let start_addend = ptp.addend();

        let cfg = Config::new(EthernetAddress(CLIENT_ADDR).into());

        let mut interface = Interface::new(cfg, &mut &mut dma, smoltcp::time::Instant::ZERO);
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

        runner::spawn().ok();

        (
            Shared {
                dma,
                ptp,
                interface,
                sockets,
                udp_socket,
            },
            Local { start_addend },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [interface, dma, sockets, udp_socket, ptp], local = [addend_integrator: f32 = 0.0, start_addend])]
    fn runner(cx: runner::Context) {
        use core::convert::TryInto;
        use fugit::ExtU64;

        runner::spawn_after(100.millis()).ok();
        let start = monotonics::now();

        let (mut interface, mut dma, mut sockets, mut udp_socket, mut ptp) = (
            cx.shared.interface,
            cx.shared.dma,
            cx.shared.sockets,
            cx.shared.udp_socket,
            cx.shared.ptp,
        );

        let (addend_integrator, start_addend) =
            (cx.local.addend_integrator, *cx.local.start_addend);

        let udp_socket = udp_socket.lock(|v| *v);
        let mut buf = [0u8; 128];

        sockets.lock(|sockets| {
            let udp_socket = sockets.get_mut::<udp::Socket>(udp_socket);
            udp_socket.close();
            udp_socket
                .bind(IpListenEndpoint {
                    addr: None,
                    port: 1337,
                })
                .ok()
                .unwrap();
        });

        macro_rules! recv {
            () => {
                loop {
                    if monotonics::now() - 500u64.millis() > start {
                        return;
                    }

                    let res = (&mut sockets).lock(|sockets| {
                        let udp_socket = sockets.get_mut::<udp::Socket>(udp_socket);
                        if let Ok((size, meta)) = udp_socket.recv_slice(&mut buf) {
                            if let Poll::Ready(Ok(Some(timestamp))) =
                                dma.lock(|dma| dma.poll_timestamp(&meta.meta.into()))
                            {
                                Ok((&buf[..size], timestamp))
                            } else {
                                Err(true)
                            }
                        } else {
                            Err(false)
                        }
                    });

                    if let Ok(res) = res {
                        break res;
                    } else if let Err(true) = res {
                        return;
                    }
                }
            };
        }

        macro_rules! send {
            ($data:expr) => {{
                let packet_id = (&mut sockets, &mut interface).lock(|sockets, interface| {
                    let udp_socket = sockets.get_mut::<udp::Socket>(udp_socket);

                    let packet_id = dma.lock(|dma| dma.next_packet_id()).into();

                    let mut meta: udp::UdpMetadata = IpEndpoint {
                        addr: IpAddress::Ipv4(Ipv4Address::new(10, 0, 0, 1)),
                        port: 1337,
                    }
                    .into();
                    meta.meta = packet_id;

                    udp_socket.send_slice($data, meta).unwrap();

                    dma.lock(|mut dma| {
                        interface.poll(now(), &mut dma, sockets);
                    });

                    packet_id
                });

                loop {
                    if monotonics::now() - 500u64.millis() > start {
                        return;
                    }

                    let timestamp = dma.lock(|dma| dma.poll_timestamp(&packet_id.into()));

                    if let Poll::Ready(Ok(Some(timestamp))) = timestamp {
                        break timestamp;
                    }
                }
            }};
        }

        // Protocol:
        // 1. Client sends empty message 0x0
        // 2. Server sends empty message 0x01, (client records RX time t1')
        // 3. Server sends message 0x02 with TX time of #2 (client records TX time t1)
        // 4. Client sends empty message 0x03 (client records TX time t2)
        // 5. Server sends message 0x04 with RX time of #4 (client records RX time t2')

        defmt::info!("Step 1");
        send!(&[0x00]);

        defmt::info!("Step 2");
        let (buf, t1_prim) = recv!();

        if buf[0] != 0x01 {
            defmt::error!("Expected message 0x01, got {}", buf);
            return;
        }

        defmt::info!("Step 3");
        let (buf, _) = recv!();

        if buf[0] != 0x02 {
            defmt::error!("Expected message 0x02, got {}", buf);
            return;
        }

        let t1 = Timestamp::new_raw(i64::from_le_bytes(
            buf[1..9].try_into().expect("Infallible"),
        ));

        defmt::info!("Step 4");
        let t2 = send!(&[0x03]);

        defmt::info!("Step 5");
        let (buf, _) = recv!();

        if buf[0] != 0x04 {
            defmt::error!("Expected message 0x04, got {}", buf);
            return;
        }

        let t2_prim = Timestamp::new_raw(i64::from_le_bytes(
            buf[1..9].try_into().expect("Infallible"),
        ));

        let offset = crate::calculate_offset(t1, t1_prim, t2, t2_prim);

        ptp.lock(|ptp| {
            let now = EthernetPTP::get_time();
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
                    (start_addend as i64 + error / 4 + (*addend_integrator as i64)) as u32;
                ptp.set_addend(new_addend);
            }
        });
    }

    #[task(binds = ETH, shared = [dma, interface, sockets], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (dma, interface, sockets) = (cx.shared.dma, cx.shared.interface, cx.shared.sockets);

        stm32_eth::eth_interrupt_handler();

        (dma, interface, sockets).lock(|mut dma, interface, sockets| {
            interface.poll(now(), &mut dma, sockets);
        })
    }
}
