#[cfg(feature = "ieee802_3_miim")]
pub use ieee802_3_miim::Miim;

#[cfg(feature = "ieee802_3_miim")]
pub use ieee802_3_miim::*;

use crate::{
    stm32::{ethernet_mac::MACMIIAR, ETHERNET_MAC},
    EthernetMAC,
};

/// MDIO pin types.
pub unsafe trait MdioPin {}
/// MDC pin types.
pub unsafe trait MdcPin {}

#[inline(always)]
fn miim_wait_ready(iar: &MACMIIAR) {
    while iar.read().mb().bit_is_set() {}
}

#[inline(always)]
fn miim_write(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8, data: u16) {
    eth_mac.macmiidr.write(|w| w.md().bits(data));

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
    pub fn read(&mut self, phy: u8, reg: u8) -> u16 {
        miim_read(&mut self.mac.eth_mac, phy, reg)
    }

    pub fn write(&mut self, phy: u8, reg: u8, data: u16) {
        miim_write(&mut self.mac.eth_mac, phy, reg, data)
    }
}

#[cfg(feature = "ieee802_3_miim")]
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
