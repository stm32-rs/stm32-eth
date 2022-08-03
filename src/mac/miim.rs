#[cfg(feature = "ieee802_3_miim")]
pub use ieee802_3_miim::Miim;

#[cfg(feature = "ieee802_3_miim")]
pub use ieee802_3_miim::*;

use crate::stm32::ethernet_mac::{MACMIIAR, MACMIIDR};

/// MDIO pin types.
pub unsafe trait MdioPin {}
/// MDC pin types.
pub unsafe trait MdcPin {}

#[inline(always)]
fn miim_wait_ready(iar: &MACMIIAR) {
    while iar.read().mb().bit_is_set() {}
}

#[inline(always)]
pub(crate) fn miim_write(iar: &MACMIIAR, dr: &MACMIIDR, phy: u8, reg: u8, data: u16) {
    dr.write(|w| w.md().bits(data));

    iar.modify(|_, w| {
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
    miim_wait_ready(iar);
}

#[inline(always)]
pub(crate) fn miim_read(iar: &MACMIIAR, dr: &MACMIIDR, phy: u8, reg: u8) -> u16 {
    iar.modify(|_, w| {
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
    miim_wait_ready(iar);

    // Return value:
    dr.read().md().bits()
}

/// Serial Management Interface
///
/// Borrows [`MACMIIAR`] and [`MACMIIDR`] from (ETHERNET_MAC)[`crate::stm32::ETHERNET_MAC`], and holds a mutable borrow
/// to the SMI pins.
pub struct Stm32Miim<'eth, 'pins, Mdio, Mdc> {
    macmiiar: &'eth MACMIIAR,
    macmiidr: &'eth MACMIIDR,
    _mdio: &'pins mut Mdio,
    _mdc: &'pins mut Mdc,
}

impl<'eth, 'pins, Mdio, Mdc> Stm32Miim<'eth, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    pub fn read(&mut self, phy: u8, reg: u8) -> u16 {
        miim_read(&self.macmiiar, &self.macmiidr, phy, reg)
    }

    pub fn write(&mut self, phy: u8, reg: u8, data: u16) {
        miim_write(&self.macmiiar, &self.macmiidr, phy, reg, data)
    }
}

#[cfg(feature = "ieee802_3_miim")]
impl<'eth, 'pins, Mdio, Mdc> Miim for Stm32Miim<'eth, 'pins, Mdio, Mdc>
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

impl<'eth, 'pins, Mdio, Mdc> Stm32Miim<'eth, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    /// Create a temporary `Smi` instance.
    ///
    /// Temporarily take exclusive access to the MDIO and MDC pins to ensure they are not used
    /// elsewhere for the duration of SMI communication.
    pub fn new(
        macmiiar: &'eth MACMIIAR,
        macmiidr: &'eth MACMIIDR,
        _mdio: &'pins mut Mdio,
        _mdc: &'pins mut Mdc,
    ) -> Self {
        Self {
            macmiiar,
            macmiidr,
            _mdio,
            _mdc,
        }
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
