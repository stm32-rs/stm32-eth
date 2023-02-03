//! An abstraction layer for ethernet periperhals embedded in STM32 processors.
//!
//! For initialisation, see [`new`], and [`new_with_mii`]
#![no_std]
#![deny(missing_docs)]

#[cfg(not(feature = "device-selected"))]
compile_error!("No device was selected! Exactly one stm32fxxx feature must be selected.");

#[cfg(feature = "stm32h7xx-hal")]
pub use stm32h7xx_hal as hal;

/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal as hal;

/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal as hal;

/// Re-export
#[cfg(feature = "stm32f1xx-hal")]
pub use stm32f1xx_hal as hal;

#[cfg(feature = "device-selected")]
pub use hal::pac as stm32;
#[cfg(all(feature = "device-selected", not(feature = "stm32h7xx-hal")))]
use hal::rcc::Clocks;
#[cfg(all(feature = "device-selected", feature = "stm32h7xx-hal"))]
use hal::rcc::CoreClocks as Clocks;

#[cfg(feature = "device-selected")]
pub mod dma;
#[doc(inline)]
#[cfg(feature = "device-selected")]
pub use dma::{eth_interrupt_handler, MTU};

#[cfg(feature = "device-selected")]
pub mod mac;

#[cfg(feature = "device-selected")]
pub mod setup;
#[doc(inline)]
#[cfg(feature = "device-selected")]
pub use setup::{EthPins, Parts, PartsIn};

#[cfg(feature = "device-selected")]
pub(crate) mod peripherals;

#[cfg(feature = "ptp")]
pub mod ptp;

#[cfg(feature = "smoltcp-phy")]
pub use smoltcp;

#[cfg(feature = "device-selected")]
use {
    dma::{DmaParts, EthernetDMA, RxDescriptorRing, TxDescriptorRing},
    mac::{EthernetMAC, Speed, WrongClock},
    setup::*,
};

#[cfg(all(feature = "device-selected", feature = "f-series"))]
use mac::{EthernetMACWithMii, MdcPin, MdioPin};

#[cfg(all(feature = "device-selected", feature = "ptp"))]
use ptp::EthernetPTP;

/// Create and initialise the ethernet driver.
///
/// Initialize and start tx and rx DMA engines.
/// Sets up the peripheral clocks and GPIO configuration,
/// and configures the ETH MAC and DMA peripherals.
/// Automatically sets slew rate to VeryHigh.
///
/// The speed of the MAC is set to [`Speed::FullDuplexBase100Tx`].
/// This can be changed using [`EthernetMAC::set_speed`].
///
/// This method does not initialise the external PHY. Interacting with a PHY
/// can be done by using the struct returned from [`EthernetMAC::mii`].
///
/// # Note
/// - Make sure that the buffers reside in a memory region that is
/// accessible by the peripheral. Core-Coupled Memory (CCM) is
/// usually not accessible.
/// - HCLK must be at least 25 MHz.
#[cfg(feature = "device-selected")]
pub fn new<'rx, 'tx, REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>(
    parts: PartsIn,
    rx_buffer: RxDescriptorRing<'rx>,
    tx_buffer: TxDescriptorRing<'tx>,
    clocks: Clocks,
    pins: EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>,
) -> Result<Parts<'rx, 'tx, EthernetMAC>, WrongClock>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
{
    // Configure all of the pins correctly

    pins.setup_pins();

    // Set up the clocks and reset the MAC periperhal
    setup::setup();

    let eth_mac = parts.mac.into();

    // Congfigure and start up the ethernet DMA.
    let dma = EthernetDMA::new(
        DmaParts {
            eth_dma: parts.dma,
            #[cfg(feature = "stm32h7xx-hal")]
            eth_mtl: parts.mtl,
        },
        rx_buffer,
        tx_buffer,
    );

    // Configure the ethernet PTP
    #[cfg(feature = "ptp")]
    let ptp = EthernetPTP::new(parts.ptp.into(), clocks, &dma);

    #[cfg(not(feature = "stm32h7xx-hal"))]
    let mmc = parts.mmc;
    #[cfg(feature = "stm32h7xx-hal")]
    let mmc = ();

    // Configure the ethernet MAC
    let mac = EthernetMAC::new(eth_mac, mmc, clocks, Speed::FullDuplexBase100Tx, &dma)?;

    let parts = Parts {
        mac,
        dma,
        #[cfg(feature = "ptp")]
        ptp,
    };

    Ok(parts)
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
/// The speed of the MAC is set to [`Speed::FullDuplexBase100Tx`].
/// This can be changed using [`EthernetMAC::set_speed`].
///
/// The MII for the external PHY can be accessed through the
/// returned [`EthernetMACWithMii`], .
///
/// # Note
/// - Make sure that the buffers reside in a memory region that is
/// accessible by the peripheral. Core-Coupled Memory (CCM) is
/// usually not accessible.
/// - HCLK must be at least 25 MHz.
#[cfg(all(feature = "device-selected", feature = "f-series"))]
pub fn new_with_mii<'rx, 'tx, REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1, MDIO, MDC>(
    parts: PartsIn,
    rx_buffer: RxDescriptorRing<'rx>,
    tx_buffer: TxDescriptorRing<'tx>,
    clocks: Clocks,
    pins: EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>,
    mdio: MDIO,
    mdc: MDC,
) -> Result<Parts<'rx, 'tx, EthernetMACWithMii<MDIO, MDC>>, WrongClock>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
    MDIO: MdioPin,
    MDC: MdcPin,
{
    // Configure all of the pins correctly
    pins.setup_pins();

    // Set up the clocks and reset the MAC periperhal
    setup::setup();

    let eth_mac = parts.mac.into();

    // Congfigure and start up the ethernet DMA.
    let dma = EthernetDMA::new(parts.dma.into(), rx_buffer, tx_buffer);

    // Configure the ethernet PTP
    #[cfg(feature = "ptp")]
    let ptp = EthernetPTP::new(parts.ptp.into(), clocks, &dma);

    // Configure the ethernet MAC
    let mac = EthernetMAC::new(eth_mac, parts.mmc, clocks, Speed::FullDuplexBase100Tx, &dma)?
        .with_mii(mdio, mdc);

    let parts = Parts {
        mac,
        dma,
        #[cfg(feature = "ptp")]
        ptp,
    };

    Ok(parts)
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
