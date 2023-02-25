#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! A very rudimentary PTP-based time synchronization server.
//!
//! This example runs the server component of the exchange.
//!
//! See [`timesync_client`] for more information.
//!
//! [`timesync_client`]: ../timesync_client/index.html

use defmt_rtt as _;
use panic_probe as _;

#[path = "../common.rs"]
mod common;

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use crate::common::EthernetPhy;
    use core::task::Poll;

    use ieee802_3_miim::{phy::PhySpeed, Phy};
    use systick_monotonic::Systick;

    use stm32_eth::{
        dma::{EthernetDMA, PacketId},
        mac::Speed,
        Parts,
    };

    const BROADCAST: [u8; 6] = [0xFF; 6];
    const CLIENT_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
    #[allow(dead_code)]
    const SERVER_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xFF];
    const ETHER_TYPE: [u8; 2] = [0xFF, 0xFF];

    #[local]
    struct Local {
        dma: EthernetDMA<'static, 'static>,
    }

    #[shared]
    struct Shared {}

    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Pre-init");
        let core = cx.core;
        let p = cx.device;

        let (rx_ring, tx_ring) = crate::common::setup_rings();

        let (clocks, gpio, ethernet) = crate::common::setup_peripherals(p);
        let mono = Systick::new(core.SYST, clocks.hclk().raw());

        defmt::info!("Setting up pins");
        let (pins, mdio, mdc, pps) = crate::common::setup_pins(gpio);

        defmt::info!("Configuring ethernet");

        let Parts { dma, mac, mut ptp } =
            stm32_eth::new_with_mii(ethernet, rx_ring, tx_ring, clocks, pins, mdio, mdc).unwrap();

        // Enable PPS, and set its frequency to 1024 Hz
        #[cfg(not(feature = "stm32f107"))]
        ptp.set_pps_freq(10);
        ptp.enable_pps(pps);

        defmt::info!("Enabling interrupts");
        dma.enable_interrupt();

        runner::spawn().ok();

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

        (Shared {}, Local { dma }, init::Monotonics(mono))
    }

    #[task(local = [packet_id: u32 = 0, dma])]
    fn runner(cx: runner::Context) {
        use fugit::ExtU64;

        runner::spawn().ok();

        let start = monotonics::now();

        let (packet_id, dma) = (cx.local.packet_id, cx.local.dma);

        let mut buf = [0u8; 128];

        macro_rules! recv {
            () => {
                loop {
                    if monotonics::now() - 500u64.millis() > start {
                        return;
                    }

                    let res = if let Ok(rx_packet) = dma.recv_next(None) {
                        if rx_packet.len() > 14
                            && &rx_packet[6..12] == &CLIENT_ADDR
                            && &rx_packet[12..14] == &ETHER_TYPE
                        {
                            if let Some(timestamp) = rx_packet.timestamp() {
                                let data_len = rx_packet.len() - 14;
                                buf[..data_len].copy_from_slice(&rx_packet[14..14 + data_len]);
                                Ok((&buf[..data_len], timestamp))
                            } else {
                                Err(true)
                            }
                        } else {
                            Err(false)
                        }
                    } else {
                        Err(false)
                    };

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
                let current_id = PacketId(*packet_id);
                *packet_id += 1;
                let current_clone = current_id.clone();
                dma.send(14 + $data.len(), Some(current_clone), |buf| {
                    buf[0..6].copy_from_slice(&BROADCAST);
                    buf[6..12].copy_from_slice(&SERVER_ADDR);
                    buf[12..14].copy_from_slice(&ETHER_TYPE);
                    buf[14..14 + $data.len()].copy_from_slice($data);
                })
                .unwrap();

                loop {
                    if monotonics::now() - 500u64.millis() > start {
                        return;
                    }

                    let current_id = current_id.clone();
                    if let Poll::Ready(Ok(Some(timestamp))) = dma.poll_timestamp(&current_id) {
                        break timestamp;
                    }
                }
            }};
        }

        let (m0, _) = recv!();

        defmt::trace!("Step 1");

        if !m0.starts_with(&[0x00]) {
            defmt::error!("Expected message 0x00, got {}", m0);
            return;
        }

        defmt::trace!("Step 2");
        let t1 = send!(&[0x1]);

        defmt::trace!("Step 3");
        let mut data = [0u8; 9];
        data[0] = 0x2;
        data[1..9].copy_from_slice(&t1.raw().to_le_bytes());
        let _ = send!(&data);

        defmt::trace!("Step 4");
        let (m3, t2_prim) = recv!();
        if !m3.starts_with(&[0x03]) {
            defmt::error!("Expected message 0x03, got {}", m3);
            return;
        }

        defmt::trace!("Step 5");
        data[0] = 0x04;
        data[1..9].copy_from_slice(&t2_prim.raw().to_le_bytes());
        send!(&data);

        defmt::info!("Performed an exchange.");
        defmt::debug!(
            "The exchange took {} ms",
            (monotonics::now() - start).to_millis()
        );
    }

    #[task(binds = ETH, priority = 2)]
    fn eth_interrupt(_: eth_interrupt::Context) {
        stm32_eth::eth_interrupt_handler();
    }
}
