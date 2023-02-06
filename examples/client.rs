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

    pub fn calculate_offset(&self) -> Timestamp {
        let t1 = self.t1.unwrap();
        let t1_prim = self.t1_prim.unwrap();
        let t2 = self.t2.unwrap();
        let t2_prim = self.t2_prim.unwrap();
        let _t3 = self.t3.unwrap();
        let _t3_prim = self.t3_prim.unwrap();

        let double_offset = t1_prim - t1 - t2_prim + t2;

        let raw = double_offset.raw();
        let offset = (raw / 2).neg();
        let offset = Timestamp::new_raw(offset);

        offset
    }
}

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use fugit::ExtU64;

    use crate::{common::EthernetPhy, Client};

    use ieee802_3_miim::{phy::PhySpeed, Phy};
    use systick_monotonic::Systick;

    use stm32_eth::{
        dma::{EthernetDMA, PacketId, RxRingEntry, TxRingEntry},
        mac::Speed,
        ptp::{EthernetPTP, Timestamp},
        Parts,
    };

    const SERVER_ADDR: [u8; 6] = [0xEF, 0xBE, 0xAD, 0xDE, 0x00, 0x00];
    const CLIENT_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
    const ETH_TYPE: [u8; 2] = [0xFF, 0xFF]; // Custom/unknown ethertype

    #[local]
    struct Local {
        start_addend: u32,
    }

    #[shared]
    struct Shared {
        client: Client,
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
        let start_addend = ptp.addend();

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
            },
            Local { start_addend },
            init::Monotonics(mono),
        )
    }

    #[task(shared = [dma, ptp, client])]
    fn sender(cx: sender::Context) {
        sender::spawn_after(200.millis()).ok();

        let (mut dma, mut client) = (cx.shared.dma, cx.shared.client);

        let mut send_data = |data: &[u8]| {
            dma.lock(|dma| {
                dma.send(14 + data.len(), None, |buf| {
                    // Write the Ethernet Header and the current timestamp value to
                    // the frame.
                    buf[0..6].copy_from_slice(&SERVER_ADDR);
                    buf[6..12].copy_from_slice(&CLIENT_ADDR);
                    buf[12..14].copy_from_slice(&ETH_TYPE);
                    buf[14..14 + data.len()].copy_from_slice(data);
                })
                .ok();
            })
        };

        client.lock(|client| {
            *client = Default::default();
            // Step 1
            // Client sends empty message 0x0
            defmt::info!("Starting exchange...");
            defmt::trace!("Step 1");
            send_data(&[0x0]);
            client.expected_number = 1;
        });
    }

    #[task(binds = ETH, shared = [dma, ptp, client], local = [tx_id_ctr: u32 = 0x8000_0000, start_addend, addend_integrator: i64 = 0], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (dma, client, ptp, tx_id_ctr, addend_integrator) = (
            cx.shared.dma,
            cx.shared.client,
            cx.shared.ptp,
            cx.local.tx_id_ctr,
            cx.local.addend_integrator,
        );

        let recv_data = |buf: &mut [u8], dma: &mut EthernetDMA| {
            while let Ok(packet) = dma.recv_next(None) {
                if &packet[0..6] == CLIENT_ADDR
                    && &packet[6..12] == SERVER_ADDR
                    && &packet[12..14] == ETH_TYPE
                {
                    buf[0..packet.len() - 14].copy_from_slice(&packet[14..]);
                    return Some((packet.len(), packet.timestamp().unwrap()));
                } else {
                    continue;
                }
            }
            None
        };

        let mut send_data = |data: &[u8], dma: &mut EthernetDMA| {
            let tx_id_val = *tx_id_ctr;
            let packet_id = PacketId(tx_id_val);
            dma.send(42, Some(packet_id.clone()), |buf| {
                // Write the Ethernet Header and the current timestamp value to
                // the frame.
                buf[0..6].copy_from_slice(&SERVER_ADDR);
                buf[6..12].copy_from_slice(&CLIENT_ADDR);
                buf[12..14].copy_from_slice(&ETH_TYPE);
                buf[14..14 + data.len()].copy_from_slice(data);
            })
            .ok();
            *tx_id_ctr += 1;
            *tx_id_ctr |= 0x8000_0000;
            packet_id
        };

        let start_addend = *cx.local.start_addend;

        (dma, ptp, client).lock(|dma, ptp, client| {
            use core::convert::TryInto;

            dma.interrupt_handler();
            ptp.interrupt_handler();

            if let (true, Some(resp_packet_id), None) =
                (client.t2.is_none(), &mut client.t2_packet_id, client.t2)
            {
                // Step 4
                // Client records TX time t2
                defmt::trace!("Step 4");
                client.t2 = dma.get_timestamp_for_id(resp_packet_id.clone()).ok();
            }

            let mut buf = [0u8; 60];
            if let Some((data, rx_timestamp)) = recv_data(&mut buf, dma) {
                if data <= 0 {
                    return;
                }

                let msg_id = buf[0];
                let data = &buf[1..data];

                // Verify that we're receiving the correct message.
                if msg_id != client.expected_number {
                    *client = Default::default();
                    defmt::error!("Got unexpected message");
                    sender::spawn_after(200.millis()).ok();
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
                    let timestamp =
                        Timestamp::new_raw(i64::from_le_bytes(data[0..8].try_into().ok().unwrap()));
                    // Step 3
                    // Client records TX time t1
                    defmt::trace!("Step 3");
                    client.t1 = Some(timestamp);
                    client.t2_packet_id = Some(send_data(&[0x03], dma));
                    client.expected_number = 0x04;
                    return;
                } else if msg_id == 0x04 {
                    let timestamp =
                        Timestamp::new_raw(i64::from_le_bytes(data[0..8].try_into().ok().unwrap()));
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
                    let timestamp =
                        Timestamp::new_raw(i64::from_le_bytes(data[0..8].try_into().ok().unwrap()));
                    client.t3 = Some(timestamp);

                    client.print_timestamps();

                    let now = ptp.get_time();
                    let offset = client.calculate_offset();

                    if offset.seconds() > 0 || offset.nanos() > 200_000 {
                        *addend_integrator = 0;
                        defmt::info!("Updating time. Offset {} ", offset);
                        let updated_time = now + offset;
                        ptp.set_time(updated_time);
                    } else {
                        let mut offset_nanos = offset.nanos() as i64;
                        if offset.is_negative() {
                            offset_nanos *= -1;
                        }

                        let error = (offset_nanos * start_addend as i64) / 1_000_000_000;
                        *addend_integrator += error / 200;

                        defmt::info!(
                            "Error: {}. Integrator: {}, Offset: {} ns",
                            error,
                            addend_integrator,
                            offset.nanos()
                        );

                        let new_addend =
                            (start_addend as i64 + error / 4 + *addend_integrator) as u32;
                        ptp.set_addend(new_addend);
                    }

                    *client = Default::default();
                    sender::spawn_after(200.millis()).ok();
                }
            }
        });
    }
}
