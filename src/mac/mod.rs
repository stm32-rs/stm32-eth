//! Ethernet MAC driver implementation

use core::ops::Deref;

use crate::{
    hal::rcc::Clocks,
    stm32::{ETHERNET_MAC, ETHERNET_MMC},
};

mod miim;
pub use miim::*;

mod consts {
    /* For HCLK 60-100 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
    /* For HCLK 100-150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
    /* For HCLK 20-35 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
    /* For HCLK 35-60 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
    /* For HCLK over 150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
}
use self::consts::*;

/// HCLK must be at least 25MHz to use the ethernet peripheral.
/// This (empty) struct is returned to indicate that it is not set
/// correctly
#[derive(Debug)]
pub struct WrongClock;

/// Ethernet media access control (MAC).
pub struct EthernetMAC {
    pub(crate) eth_mac: ETHERNET_MAC,
}

impl EthernetMAC {
    /// Create a new EthernetMAC that does not own its MDIO and MDC pins.
    ///     
    /// HCLK must be at least 25MHz, else this function will return `Err(WrongClock)`.
    ///
    /// This method does not initialise the external PHY. However, you can access SMI
    /// `read` and `write` functions through the `smi` and `with_smi` functions.
    ///
    /// Additionally, an optional `impl` of the [`ieee802_3_miim::Miim`] trait is available
    /// with the `ieee802_3_miim` feature (enabled by default), for PHY communication.
    pub(crate) fn new(
        eth_mac: ETHERNET_MAC,
        eth_mmc: ETHERNET_MMC,
        clocks: Clocks,
    ) -> Result<Self, WrongClock> {
        let clock_frequency = clocks.hclk().to_Hz();

        let clock_range = match clock_frequency {
            0..=24_999_999 => return Err(WrongClock),
            25_000_000..=34_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_16,
            35_000_000..=59_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_26,
            60_000_000..=99_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_42,
            100_000_000..=149_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_62,
            _ => ETH_MACMIIAR_CR_HCLK_DIV_102,
        };

        // Set clock range in MAC MII address register
        eth_mac
            .macmiiar
            .modify(|_, w| unsafe { w.cr().bits(clock_range) });

        // Configuration Register
        eth_mac.maccr.modify(|_, w| {
            // CRC stripping for Type frames. STM32F1xx do not have this bit.
            #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
            let w = w.cstf().set_bit();

            // Fast Ethernet speed
            w.fes()
                .set_bit()
                // Duplex mode
                .dm()
                .set_bit()
                // IPv4 checksum offload
                .ipco()
                .set_bit()
                // Automatic pad/CRC stripping
                .apcs()
                .set_bit()
                // Retry disable in half-duplex mode
                .rd()
                .set_bit()
                // Receiver enable
                .re()
                .set_bit()
                // Transmitter enable
                .te()
                .set_bit()
        });

        // Frame filter register
        eth_mac.macffr.modify(|_, w| {
            // Receive All
            w.ra()
                .set_bit()
                // Promiscuous mode
                .pm()
                .set_bit()
        });

        // Flow Control Register
        eth_mac.macfcr.modify(|_, w| {
            // Pause time
            w.pt().bits(0x100)
        });

        // Disable all MMC RX interrupts
        eth_mmc
            .mmcrimr
            .write(|w| w.rgufm().set_bit().rfaem().set_bit().rfcem().set_bit());

        // Disable all MMC TX interrupts
        eth_mmc
            .mmctimr
            .write(|w| w.tgfm().set_bit().tgfmscm().set_bit().tgfscm().set_bit());

        // Fix incorrect TGFM bit position until https://github.com/stm32-rs/stm32-rs/pull/689
        // is released and used by HALs.
        eth_mmc
            .mmctimr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 21)) });

        Ok(Self { eth_mac })
    }

    /// Borrow access to the MAC's SMI.
    ///
    /// Allows for controlling and monitoring any PHYs that may be accessible via the MDIO/MDC
    /// pins.
    ///
    /// Exclusive access to the `MDIO` and `MDC` is required to ensure that are not used elsewhere
    /// for the duration of Mii communication.
    pub fn mii<'eth, 'pins, Mdio, Mdc>(
        &'eth mut self,
        mdio: &'pins mut Mdio,
        mdc: &'pins mut Mdc,
    ) -> Stm32Mii<'eth, 'pins, Mdio, Mdc>
    where
        Mdio: MdioPin,
        Mdc: MdcPin,
    {
        Stm32Mii::new(self, mdio, mdc)
    }

    /// Turn this [`EthernetMAC`] into an [`EthernetMACWithMii`]
    pub fn with_mii<MDIO, MDC>(self, mdio: MDIO, mdc: MDC) -> EthernetMACWithMii<MDIO, MDC>
    where
        MDIO: MdioPin,
        MDC: MdcPin,
    {
        EthernetMACWithMii {
            eth_mac: self,
            mdio,
            mdc,
        }
    }
}

/// Ethernet media access control (MAC) with owned MII
///
/// This version of the struct owns it's MII pins,
/// allowing it to be used directly, instead of requiring
/// that a  [`Miim`] is created.
pub struct EthernetMACWithMii<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    pub(crate) eth_mac: EthernetMAC,
    mdio: MDIO,
    mdc: MDC,
}

impl<MDIO, MDC> EthernetMACWithMii<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    /// Create a new EthernetMAC with owned MDIO and MDC pins.
    ///
    /// To interact with a connected Phy, use the `read` and `write` functions.
    ///
    /// Functionality for interacting with PHYs from the `ieee802_3_miim` crate
    /// is available if the default feature `ieee802_3_miim` is enabled.
    pub fn new(eth_mac: EthernetMAC, mdio: MDIO, mdc: MDC) -> Self {
        Self { eth_mac, mdio, mdc }
    }

    /// Release the owned MDIO and MDC pins, and return an EthernetMAC that
    /// has to borrow the MDIO and MDC pins.
    pub fn release_pins(self) -> (EthernetMAC, MDIO, MDC) {
        (self.eth_mac, self.mdio, self.mdc)
    }
}

impl<MDIO, MDC> Deref for EthernetMACWithMii<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    type Target = EthernetMAC;

    fn deref(&self) -> &Self::Target {
        &self.eth_mac
    }
}

impl<MDIO, MDC> EthernetMACWithMii<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    /// Read MII register `reg` from the PHY at address `phy`
    pub fn read(&mut self, phy: u8, reg: u8) -> u16 {
        self.eth_mac
            .mii(&mut self.mdio, &mut self.mdc)
            .read(phy, reg)
    }

    /// Write the value `data` to MII register `reg` to the PHY at address `phy`
    pub fn write(&mut self, phy: u8, reg: u8, data: u16) {
        self.eth_mac
            .mii(&mut self.mdio, &mut self.mdc)
            .write(phy, reg, data)
    }
}

#[cfg(feature = "ieee802_3_miim")]
impl<MDIO, MDC> miim::Miim for EthernetMACWithMii<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    fn read(&mut self, phy: u8, reg: u8) -> u16 {
        self.read(phy, reg)
    }

    fn write(&mut self, phy: u8, reg: u8, data: u16) {
        self.write(phy, reg, data)
    }
}
