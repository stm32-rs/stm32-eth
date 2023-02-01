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
        dma::{EthernetDMA, PacketId, RxRingEntry, TxRingEntry},
        mac::Speed,
        ptp::EthernetPTP,
        Parts,
    };

    const SERVER_ADDR: [u8; 6] = [0xEF, 0xBE, 0xAD, 0xDE, 0x00, 0x00];
    const CLIENT_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
    const ETH_TYPE: [u8; 2] = [0xFF, 0xFF]; // Custom/unknown ethertype

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        server: Server,
        dma: EthernetDMA<'static, 'static>,
        ptp: EthernetPTP,
    }

    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    #[init(local = [
        rx_ring: [RxRingEntry; 2] = [RxRingEntry::new(),RxRingEntry::new()],
        tx_ring: [TxRingEntry; 2] = [TxRingEntry::new(),TxRingEntry::new()],
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Pre-init");
        let core = cx.core;
        let p = cx.device;

        let rx_ring = cx.local.rx_ring;
        let tx_ring = cx.local.tx_ring;

        let (clocks, gpio, ethernet) = crate::common::setup_peripherals(p);
        let mono = Systick::new(core.SYST, clocks.hclk().raw());

        defmt::info!("Setting up pins");
        let (pins, mdio, mdc, pps) = crate::common::setup_pins(gpio);

        defmt::info!("Configuring ethernet");

        let Parts { dma, mac, mut ptp } =
            stm32_eth::new_with_mii(ethernet, rx_ring, tx_ring, clocks, pins, mdio, mdc).unwrap();

        ptp.enable_pps(pps);

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
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    #[task(binds = ETH, shared = [dma, ptp, server], local = [tx_id_ctr: u32 = 0x8000_0000], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (dma, server, ptp, tx_id_ctr) = (
            cx.shared.dma,
            cx.shared.server,
            cx.shared.ptp,
            cx.local.tx_id_ctr,
        );

        let recv_data = |buf: &mut [u8], dma: &mut EthernetDMA| {
            while let Ok(packet) = dma.recv_next(None) {
                if packet[0..6] == SERVER_ADDR
                    && packet[6..12] == CLIENT_ADDR
                    && packet[12..14] == ETH_TYPE
                {
                    buf[0..packet.len() - 14].copy_from_slice(&packet[14..]);
                    let len = packet.len() - 14;
                    return Some((len, packet.timestamp().unwrap()));
                } else {
                    continue;
                }
            }
            None
        };

        let mut send_data = |data: &[u8], dma: &mut EthernetDMA| {
            let tx_id_val = *tx_id_ctr;
            let packet_id = PacketId(tx_id_val);
            dma.send(14 + data.len(), Some(packet_id.clone()), |buf| {
                // Write the Ethernet Header and the current timestamp value to
                // the frame.
                buf[0..6].copy_from_slice(&CLIENT_ADDR);
                buf[6..12].copy_from_slice(&SERVER_ADDR);
                buf[12..14].copy_from_slice(&ETH_TYPE);
                buf[14..14 + data.len()].copy_from_slice(data);
            })
            .ok();
            *tx_id_ctr += 1;
            *tx_id_ctr |= 0x8000_0000;
            packet_id
        };

        (dma, ptp, server).lock(|dma, ptp, server| {
            dma.interrupt_handler();
            ptp.interrupt_handler();

            if let (false, Some(t1_id)) = (server.t1_sent, &mut server.t1_id) {
                if let Some(t1) = dma.get_timestamp_for_id(t1_id.clone()).ok() {
                    // Step 3
                    // Server sends message 0x02 with the TX time of #2
                    server.expected_number = 0x03;

                    defmt::info!("Step 3");

                    let mut buffer = [0u8; 42];
                    buffer[0] = 0x02;
                    buffer[1..9].copy_from_slice(&t1.raw().to_le_bytes());
                    send_data(&buffer[..9], dma);
                    server.t1_sent = true;
                }
            } else if let Some(t3_id) = &mut server.t3_id {
                if let Some(t3) = dma.get_timestamp_for_id(t3_id.clone()).ok() {
                    // Step 7.
                    // Server sends message 0x06 with the timestsamp of #6
                    let mut buffer = [0u8; 9];
                    buffer[0] = 0x06;
                    buffer[1..9].copy_from_slice(&t3.raw().to_le_bytes());
                    send_data(&buffer[..9], dma);
                    server.t3_sent = true;

                    defmt::info!("Step 7");

                    // We're done, reset
                    *server = Default::default();
                }
            }

            let mut buf = [0u8; 60];
            if let Some((data, rx_timestamp)) = recv_data(&mut buf, dma) {
                if data <= 0 {
                    return;
                }

                let msg_id = buf[0];

                // Verify that we're receiving the correct message.
                if msg_id != server.expected_number {
                    *server = Default::default();
                    defmt::error!("Got unexpected message");
                } else {
                    // Advance to the next message.
                    server.expected_number += 1;
                }

                defmt::info!("{}", msg_id);

                if msg_id == 0 {
                    // Step 1
                    // Step 2
                    // Server sends empty message 0x01.

                    defmt::info!("Step 1");
                    defmt::info!("Step 2");
                    let t1_id = send_data(&[0x01], dma);
                    server.t1_id = Some(t1_id);
                    server.t1_sent = false;
                } else if msg_id == 0x03 {
                    // Step 4
                    // Step 5
                    defmt::info!("Step 4");
                    defmt::info!("Step 5");
                    let mut buffer = [0u8; 9];
                    buffer[0] = 0x04;
                    buffer[1..9].copy_from_slice(&rx_timestamp.raw().to_le_bytes());
                    send_data(&buffer[..9], dma);

                    defmt::info!("Step 6");
                    // Step 6
                    buffer[0] = 0x05;
                    let t3_id = send_data(&buffer[..1], dma);
                    server.t3_id = Some(t3_id);
                }
            }
        });
    }
}
