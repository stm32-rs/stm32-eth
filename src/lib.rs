//! An abstraction layer for ethernet periperhals embedded in STM32 processors.
//!
//! For initialisation, see [`new`], and [`new_with_mii`]
#![no_std]

use mac::Speed;

/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal::pac as stm32;

/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal::pac as stm32;

/// Re-export
#[cfg(feature = "stm32f1xx-hal")]
pub use stm32f1xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f1xx-hal")]
pub use stm32f1xx_hal::pac as stm32;

use hal::rcc::Clocks;
use stm32::{ETHERNET_DMA, ETHERNET_MAC, ETHERNET_MMC, ETHERNET_PTP};

mod dma;
pub use dma::{eth_interrupt_handler, EthernetDMA};
mod ring;
pub use ring::RingEntry;
mod desc;
pub mod mac;
pub use mac::{EthernetMAC, EthernetMACWithMii, WrongClock};
mod rx;
pub use rx::{RxDescriptor, RxError, RxRingEntry};
mod tx;
pub use tx::{TxDescriptor, TxError, TxRingEntry};
pub mod setup;
pub use setup::EthPins;
use setup::{
    AlternateVeryHighSpeed, RmiiCrsDv, RmiiRefClk, RmiiRxD0, RmiiRxD1, RmiiTxD0, RmiiTxD1, RmiiTxEN,
};
mod packet_id;
pub use packet_id::PacketId;
mod ptp;
use ptp::setup_ptp;

#[cfg(feature = "smoltcp-phy")]
pub use smoltcp;
#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::*;

/// From the datasheet: *VLAN Frame maxsize = 1522*
const MTU: usize = 1522;

/// Create and initialise the ethernet driver.
///
/// Initialize and start tx and rx DMA engines.
/// Sets up the peripheral clocks and GPIO configuration,
/// and configures the ETH MAC and DMA peripherals.
/// Automatically sets slew rate to VeryHigh.
///
/// This method does not initialise the external PHY. Interacting with a PHY
/// can be done by using the struct returned from [`EthernetMAC::smi`].
///
/// /// # Note
/// - Make sure that the buffers reside in a memory region that is
/// accessible by the peripheral. Core-Coupled Memory (CCM) is
/// usually not accessible.
/// - HCLK must be at least 25 MHz.
pub fn new<'rx, 'tx, REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>(
    eth_mac: ETHERNET_MAC,
    eth_mmc: ETHERNET_MMC,
    eth_dma: ETHERNET_DMA,
    eth_ptp: ETHERNET_PTP,
    rx_buffer: &'rx mut [RxRingEntry],
    tx_buffer: &'tx mut [TxRingEntry],
    clocks: Clocks,
    pins: EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>,
    initial_speed: Option<Speed>,
) -> Result<(EthernetDMA<'rx, 'tx>, EthernetMAC), WrongClock>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
{
    let speed = initial_speed.unwrap_or(Speed::FullDuplexBase100Tx);

    let mut mac = EthernetMAC::new(eth_mac, eth_mmc, clocks, pins, speed)?;

    let _ = setup_ptp(&mut mac, eth_ptp);

    let dma = EthernetDMA::new(&mac, eth_dma, rx_buffer, tx_buffer);

    Ok((dma, mac))
}

/// Create and initialise the ethernet driver.
///
/// Initialize and start tx and rx DMA engines.
/// Sets up the peripheral clocks and GPIO configuration,
/// and configures the ETH MAC and DMA peripherals.
/// Automatically sets slew rate to VeryHigh.
///
/// This method does not initialise the external PHY.
///
/// The MII for the external PHY can be accessed through the
/// returned [`EthernetMACWithMii`], .
///
/// # Note
/// - Make sure that the buffers reside in a memory region that is
/// accessible by the peripheral. Core-Coupled Memory (CCM) is
/// usually not accessible.
/// - HCLK must be at least 25 MHz.
pub fn new_with_mii<'rx, 'tx, REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1, MDIO, MDC>(
    eth_mac: ETHERNET_MAC,
    eth_mmc: ETHERNET_MMC,
    eth_dma: ETHERNET_DMA,
    eth_ptp: ETHERNET_PTP,
    rx_buffer: &'rx mut [RxRingEntry],
    tx_buffer: &'tx mut [TxRingEntry],
    clocks: Clocks,
    pins: EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>,
    mdio: MDIO,
    mdc: MDC,
    initial_speed: Option<Speed>,
) -> Result<(EthernetDMA<'rx, 'tx>, EthernetMACWithMii<MDIO, MDC>), WrongClock>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
    MDIO: mac::MdioPin,
    MDC: mac::MdcPin,
{
    let speed = initial_speed.unwrap_or(Speed::FullDuplexBase100Tx);

    let mut mac = EthernetMAC::new(eth_mac, eth_mmc, clocks, pins, speed)?.with_mii(mdio, mdc);

    let _ = setup_ptp(&mut mac, eth_ptp);

    let dma = EthernetDMA::new(&mac, eth_dma, rx_buffer, tx_buffer);

    Ok((dma, mac))
}

/// This block ensures that README.md is checked when `cargo test` is run.
///
/// Taken from https://github.com/rp-rs/pio-rs/blob/b52d3ba9c031ffa72bdd6f16b5fa8c0c04f0e2e0/src/lib.rs#L963
#[cfg(doctest)]
mod test_readme {
    macro_rules! external_doc_test {
        ($x:expr) => {
            #[doc = $x]
            extern "C" {}
        };
    }
    external_doc_test!(include_str!("../README.md"));
}
