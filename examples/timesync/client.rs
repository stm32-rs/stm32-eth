#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! # The `timesync` example
//!  
//! A very basic PTP-based time synchronization example built using RTIC.
//!
//! For this example to do anything, it is necessary that the server (`timesync/server.rs`) is
//! already running and that broadcast ethernet traffic can flow between the two nodes.
//!
//! # Notes on running this example
//!
//! When using the internal oscillator of an STM32, accurately syncing is very difficult, as the
//! frequency drift and error between two instances of that clock is difficult to accurately
//! compensate for with the very limited protocol that this example uses.
//!
//! If a more accurate High Speed External oscillator is used, the rate of time and global time
//! on the two nodes can be synchronized to within a few tenths of PPMs.
//!
//! To activate the HSE configuration for the examples, set the `STM32_ETH_EXAMPLE_HSE`
//! environment variable to `oscillator` or `bypass` when compiling examples.
//!
//! # Monitoring the accuracy of synchronization
//!
//! Generally, the output displayed by the client with `DEFMT_LOG=info` is quite accurate.
//!
//! However, if you wish to observe the difference externally the PPS pin can be used. In
//! these examples, it is always enabled. By default, it will run on PB5. If you wish to use
//! the alternate PPS pin (PG8), set the `STM32_ETH_EXAMPLE_PPS_PIN` environment variable
//! to `alternate`  when compiling the examples.
//!
//! # What does this example do?
//!
//! The nodes will exchange messages according to the following protocol:
//! 1. Client sends empty message 0x0 to initiate an exchange
//! 2. Server sends empty message 0x01, (client records RX time t1')
//! 3. Server sends message 0x02 with TX time of #2 (client records TX time t1)
//! 4. Client sends empty message 0x03 (client records TX time t2)
//! 5. Server sends message 0x04 with RX time of #4 (client records RX time t2')
//!
//! Once the exchange is finished, the client will do the following:
//! 1. If the calculated time difference is larger than 200 microseconds, the current local time is
//!    set to the received value.
//! 2. If the difference is smaller than or equal to 200 microseconds, the PTP addend value is
//!    updated to compensate for the observed difference using a basic PI integrator.
//!
//! See the [PTP article on Wikipedia] for more information on how/why this works.
//!
//! [PTP article on Wikipedia]: https://en.wikipedia.org/wiki/Precision_Time_Protocol#Synchronization
use core::ops::Neg;

use defmt_rtt as _;
use panic_probe as _;
use stm32_eth::ptp::Timestamp;

#[path = "../common.rs"]
mod common;

/// Calculate the time offset, as laid out by the [PTP article on wikipedia].
///
/// [PTP article on wikipedia]: https://en.wikipedia.org/wiki/Precision_Time_Protocol#Synchronization
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

    use crate::common::EthernetPhy;
    use core::task::Poll;

    use ieee802_3_miim::{phy::PhySpeed, Phy};
    use systick_monotonic::Systick;

    use stm32_eth::{
        dma::{EthernetDMA, PacketId, RxRingEntry, TxRingEntry},
        mac::Speed,
        ptp::{EthernetPTP, Timestamp},
        Parts,
    };

    const BROADCAST: [u8; 6] = [0xFF; 6];
    #[allow(dead_code)]
    const CLIENT_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];
    const SERVER_ADDR: [u8; 6] = [0x80, 0x00, 0xDE, 0xAD, 0xBE, 0xFF];
    const ETHER_TYPE: [u8; 2] = [0xFF, 0xFF];

    #[local]
    struct Local {
        start_addend: u32,
        dma: EthernetDMA<'static, 'static>,
        ptp: EthernetPTP,
    }

    #[shared]
    struct Shared {}

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

        // Enable PPS, and set its frequency to 1024 Hz
        #[cfg(not(feature = "stm32f107"))]
        ptp.set_pps_freq(10);
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

        runner::spawn().ok();

        (
            Shared {},
            Local {
                start_addend,
                ptp,
                dma,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [addend_integrator: f32 = 0.0, start_addend, packet_id: u32 = 0, dma, ptp])]
    fn runner(cx: runner::Context) {
        use fugit::ExtU64;

        runner::spawn_after(100.millis()).ok();

        let (dma, ptp) = (cx.local.dma, cx.local.ptp);

        let start = monotonics::now();

        let (addend_integrator, start_addend, packet_id) = (
            cx.local.addend_integrator,
            *cx.local.start_addend,
            cx.local.packet_id,
        );

        let mut buf = [0u8; 128];

        macro_rules! recv {
            () => {
                loop {
                    if monotonics::now() - 500u64.millis() > start {
                        return;
                    }

                    let res = if let Ok(rx_packet) = dma.recv_next(None) {
                        if rx_packet.len() > 14
                            && &rx_packet[6..12] == &SERVER_ADDR
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
                    buf[6..12].copy_from_slice(&CLIENT_ADDR);
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

        use core::convert::TryInto;

        defmt::trace!("Step 1");
        send!(&[0x00]);

        defmt::trace!("Step 2");
        let (buf, t1_prim) = recv!();

        if !buf.starts_with(&[0x01]) {
            defmt::error!("Expected message 0x01, got {}", buf);
            return;
        }

        defmt::trace!("Step 3");
        let (buf, _) = recv!();

        if !buf.starts_with(&[0x02]) {
            defmt::error!("Expected message 0x02, got {}", buf);
            return;
        }

        let t1 = Timestamp::new_raw(i64::from_le_bytes(
            buf[1..9].try_into().expect("Infallible"),
        ));

        defmt::trace!("Step 4");
        let t2 = send!(&[0x03]);

        defmt::trace!("Step 5");
        let (buf, _) = recv!();

        if !buf.starts_with(&[0x04]) {
            defmt::error!("Expected message 0x04, got {}", buf);
            return;
        }

        let t2_prim = Timestamp::new_raw(i64::from_le_bytes(
            buf[1..9].try_into().expect("Infallible"),
        ));

        let offset = crate::calculate_offset(t1, t1_prim, t2, t2_prim);

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

            defmt::debug!(
                "DATA: {}, {}, {}, {}",
                error,
                addend_integrator,
                offset_nanos,
                now
            );

            let new_addend = (start_addend as i64 + error / 4 + (*addend_integrator as i64)) as u32;
            ptp.set_addend(new_addend);
        }

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
