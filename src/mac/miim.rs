pub use ieee802_3_miim::Miim;

pub use ieee802_3_miim::*;

use crate::{peripherals::ETHERNET_MAC, stm32::ethernet_mac::MACMIIAR};

use super::EthernetMAC;

/// MDIO pin types.
///
/// # Safety
/// Only pins specified as ETH_MDIO in a part's reference manual
/// may implement this trait
pub unsafe trait MdioPin {}

/// MDC pin types.
///
/// # Safety
/// Only pins specified as ETH_MDC in a part's reference manual
/// may implement this trait
pub unsafe trait MdcPin {}

#[inline(always)]
fn miim_wait_ready(iar: &MACMIIAR) {
    while iar.read().mb().bit_is_set() {}
}

#[inline(always)]
fn miim_write(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8, data: u16) {
    miim_wait_ready(&eth_mac.macmiiar);
    eth_mac.macmiidr.write(|w| w.md().bits(data));

    miim_wait_ready(&eth_mac.macmiiar);

    eth_mac.macmiiar.modify(|_, w| {
        w.pa()
            .bits(phy)
            .mr()
            .bits(reg)
            /* Write operation MW=1*/
            .mw()
            .set_bit()
            .mb()
            .set_bit()
    });
    miim_wait_ready(&eth_mac.macmiiar);
}

#[inline(always)]
fn miim_read(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8) -> u16 {
    miim_wait_ready(&eth_mac.macmiiar);
    eth_mac.macmiiar.modify(|_, w| {
        w.pa()
            .bits(phy)
            .mr()
            .bits(reg)
            /* Read operation MW=0 */
            .mw()
            .clear_bit()
            .mb()
            .set_bit()
    });
    miim_wait_ready(&eth_mac.macmiiar);

    // Return value:
    eth_mac.macmiidr.read().md().bits()
}

/// Serial Management Interface
///
/// Borrows an [`EthernetMAC`] and holds a mutable borrow to the SMI pins.
pub struct Stm32Mii<'mac, 'pins, Mdio, Mdc> {
    mac: &'mac mut EthernetMAC,
    _mdio: &'pins mut Mdio,
    _mdc: &'pins mut Mdc,
}

impl<'mac, 'pins, Mdio, Mdc> Stm32Mii<'mac, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    /// Read MII register `reg` from the PHY at address `phy`
    pub fn read(&mut self, phy: u8, reg: u8) -> u16 {
        miim_read(&mut self.mac.eth_mac, phy, reg)
    }

    /// Write the value `data` to MII register `reg` to the PHY at address `phy`
    pub fn write(&mut self, phy: u8, reg: u8, data: u16) {
        miim_write(&mut self.mac.eth_mac, phy, reg, data)
    }
}

impl<'eth, 'pins, Mdio, Mdc> Miim for Stm32Mii<'eth, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    fn read(&mut self, phy: u8, reg: u8) -> u16 {
        self.read(phy, reg)
    }

    fn write(&mut self, phy: u8, reg: u8, data: u16) {
        self.write(phy, reg, data)
    }
}

impl<'eth, 'pins, Mdio, Mdc> Stm32Mii<'eth, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    /// Create a temporary [`Stm32Mii`] instance.
    ///
    /// Temporarily take exclusive access to the MDIO and MDC pins to ensure they are not used
    /// elsewhere for the duration of SMI communication.
    pub fn new(mac: &'eth mut EthernetMAC, _mdio: &'pins mut Mdio, _mdc: &'pins mut Mdc) -> Self {
        Self { mac, _mdio, _mdc }
    }
}

#[cfg(feature = "stm32f4xx-hal")]
mod pin_impls {
    use crate::hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate};

    const AF11: u8 = 11;

    unsafe impl super::MdioPin for PA2<Alternate<AF11>> {}
    unsafe impl super::MdcPin for PC1<Alternate<AF11>> {}
}

#[cfg(feature = "stm32f7xx-hal")]
mod pin_impls {
    use crate::hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate};

    const AF11: u8 = 11;

    unsafe impl super::MdioPin for PA2<Alternate<AF11>> {}
    unsafe impl super::MdcPin for PC1<Alternate<AF11>> {}
}

#[cfg(feature = "stm32f1xx-hal")]
mod pin_impls {
    use crate::hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate, PushPull};

    unsafe impl super::MdioPin for PA2<Alternate<PushPull>> {}
    unsafe impl super::MdcPin for PC1<Alternate<PushPull>> {}
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
    /// is available.
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

impl<MDIO, MDC> DerefMut for EthernetMACWithMii<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.eth_mac
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
