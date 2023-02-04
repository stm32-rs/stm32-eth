//! Ethernet MAC access and configuration.

use crate::{dma::EthernetDMA, peripherals::ETHERNET_MAC, Clocks};

#[cfg(feature = "f-series")]
mod miim;
#[cfg(feature = "f-series")]
pub use miim::*;

pub(crate) struct MacParts {
    pub eth_mac: ETHERNET_MAC,
    #[cfg(feature = "f-series")]
    pub eth_mmc: crate::stm32::ETHERNET_MMC,
}

impl MacParts {
    fn enable_promicious_mode(&self) {
        let Self { eth_mac, .. } = self;

        #[cfg(feature = "f-series")]
        let (mac_filter, flow_control) = (&eth_mac.macffr, &eth_mac.macfcr);
        #[cfg(feature = "stm32h7xx-hal")]
        let (mac_filter, flow_control) = (&eth_mac.macpfr, &eth_mac.macqtx_fcr);

        // Frame filter register
        mac_filter.modify(|_, w| {
            // Receive All
            w.ra()
                .set_bit()
                // Promiscuous mode
                .pm()
                .set_bit()
        });
        // Flow Control Register
        flow_control.modify(|_, w| {
            // Pause time
            #[allow(unused_unsafe)]
            unsafe {
                w.pt().bits(0x100)
            }
        });
    }

    fn disable_mmc_interrupts(&self) {
        #[cfg(feature = "f-series")]
        {
            let eth_mmc = &self.eth_mmc;
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
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            let eth_mac = &self.eth_mac;

            // Disable all MMC RX interrupts
            eth_mac.mmc_rx_interrupt_mask.write(|w| {
                w.rxlpitrcim()
                    .set_bit()
                    .rxlpiuscim()
                    .set_bit()
                    .rxucgpim()
                    .set_bit()
                    .rxalgnerpim()
                    .set_bit()
                    .rxcrcerpim()
                    .set_bit()
            });

            // Disable all MMC TX interrupts
            eth_mac.mmc_tx_interrupt_mask.write(|w| {
                w.txlpitrcim()
                    .set_bit()
                    .txlpiuscim()
                    .set_bit()
                    .txgpktim()
                    .set_bit()
                    .txmcolgpim()
                    .set_bit()
                    .txscolgpim()
                    .set_bit()
            });
        }
    }
}

/// Speeds at which this MAC can be configured
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    /// 10Base-T half duplex
    HalfDuplexBase10T,
    /// 10Base-T full duplex
    FullDuplexBase10T,
    /// 100Base-Tx half duplex
    HalfDuplexBase100Tx,
    /// 100Base-Tx full duplex
    FullDuplexBase100Tx,
}

#[cfg(feature = "f-series")]
use self::consts::*;
#[cfg(feature = "f-series")]
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

/// HCLK must be at least 25MHz to use the ethernet peripheral.
/// This (empty) struct is returned to indicate that it is not set
/// correctly
#[derive(Debug)]
pub struct WrongClock;

/// Ethernet media access control (MAC).
///
// impl note: access to the MACIMR register should _only_ be performed
// atomically.
pub struct EthernetMAC {
    eth_mac: ETHERNET_MAC,
}

impl EthernetMAC {
    /// Create a new EthernetMAC that does not own its MDIO and MDC pins.
    ///     
    /// HCLK must be at least 25MHz, else this function will return `Err(WrongClock)`.
    ///
    /// This method does not initialise the external PHY. However, you can access SMI
    /// `read` and `write` functions through the [`Self::mii`] and [`Self::with_miim`] functions.
    ///
    /// Additionally, an `impl` of the [`ieee802_3_miim::Miim`] trait is available
    /// for PHY communication.
    pub(crate) fn new(
        parts: MacParts,
        #[allow(unused)] clocks: Clocks,
        initial_speed: Speed,
        // Note(_dma): this field exists to ensure that the MAC is not
        // initialized before the DMA. If MAC is started before the DMA,
        // it doesn't work.
        _dma: &EthernetDMA,
    ) -> Result<Self, WrongClock> {
        let eth_mac = &parts.eth_mac;

        // TODO: configure MDIOS
        #[cfg(feature = "f-series")]
        {
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
        }

        // Configuration Register
        eth_mac.maccr.modify(|_, w| {
            // CRC stripping for Type frames. STM32F1xx do not have this bit.
            #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
            let w = w.cstf().set_bit();

            #[cfg(feature = "f-series")]
            let w = w
                // IPv4 checksum offload
                .ipco()
                .set_bit()
                // Automatic pad/CRC stripping
                .apcs()
                .set_bit()
                // Retry disable in half-duplex mode
                .rd()
                .set_bit();

            #[cfg(feature = "stm32h7xx-hal")]
            let w = w
                // IPv4 checksum offload
                .acs()
                .set_bit()
                // Automatic pad/CRC stripping
                .ipc()
                .set_bit()
                // Retry disable in half-duplex mode
                .dr()
                .set_bit();

            w
                // Receiver enable
                .re()
                .set_bit()
                // Transmitter enable
                .te()
                .set_bit()
        });

        parts.enable_promicious_mode();
        parts.disable_mmc_interrupts();

        let mut me = Self {
            eth_mac: parts.eth_mac,
        };

        me.set_speed(initial_speed);

        Ok(me)
    }

    /// Set the Ethernet Speed at which the MAC communicates
    ///
    /// Note that this does _not_ affect the PHY in any way. To
    /// configure the PHY, use [`EthernetMACWithMii`] (see: [`Self::with_mii`])
    /// or [`Stm32Mii`] (see: [`Self::mii`])
    pub fn set_speed(&mut self, speed: Speed) {
        self.eth_mac.maccr.modify(|_, w| match speed {
            Speed::HalfDuplexBase10T => w.fes().clear_bit().dm().clear_bit(),
            Speed::FullDuplexBase10T => w.fes().clear_bit().dm().set_bit(),
            Speed::HalfDuplexBase100Tx => w.fes().set_bit().dm().clear_bit(),
            Speed::FullDuplexBase100Tx => w.fes().set_bit().dm().set_bit(),
        });
    }

    /// Get the Ethernet Speed at which the MAC communicates
    pub fn get_speed(&self) -> Speed {
        let cr = self.eth_mac.maccr.read();
        match (cr.fes().bit_is_set(), cr.dm().bit_is_set()) {
            (false, false) => Speed::HalfDuplexBase10T,
            (false, true) => Speed::FullDuplexBase10T,
            (true, false) => Speed::HalfDuplexBase100Tx,
            (true, true) => Speed::FullDuplexBase100Tx,
        }
    }

    /// Borrow access to the MAC's SMI.
    ///
    /// Allows for controlling and monitoring any PHYs that may be accessible via the MDIO/MDC
    /// pins.
    ///
    /// Exclusive access to the `MDIO` and `MDC` is required to ensure that are not used elsewhere
    /// for the duration of Mii communication.
    #[cfg(feature = "f-series")]
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
    #[cfg(feature = "f-series")]
    pub fn with_mii<MDIO, MDC>(self, mdio: MDIO, mdc: MDC) -> EthernetMACWithMii<MDIO, MDC>
    where
        MDIO: MdioPin,
        MDC: MdcPin,
    {
        EthernetMACWithMii::new(self, mdio, mdc)
    }

    #[cfg(all(feature = "ptp", feature = "f-series"))]
    pub(crate) fn mask_timestamp_trigger_interrupt() {
        // SAFETY: MACIMR only receives atomic writes.
        let mac = &unsafe { &*ETHERNET_MAC::ptr() };
        mac.macimr.write(|w| w.tstim().set_bit());
    }

    // NOTE(allow): only used on F4 and F7
    #[cfg(all(feature = "ptp", feature = "f-series"))]
    pub(crate) fn unmask_timestamp_trigger_interrupt() {
        // SAFETY: MACIMR only receives atomic writes.
        let mac = &unsafe { &*ETHERNET_MAC::ptr() };
        mac.macimr.write(|w| w.tstim().clear_bit());
    }
}
