use crate::stm32::ethernet_mac::{MACMIIAR, MACMIIDR};

/// MDIO pin types.
pub unsafe trait MdioPin {}
/// MDC pin types.
pub unsafe trait MdcPin {}

pub trait StationManagement {
    fn read(&self, phy: u8, reg: u8) -> u16;
    fn write(&mut self, phy: u8, reg: u8, data: u16);
}

#[inline(always)]
fn smi_wait_ready(iar: &MACMIIAR) {
    while iar.read().mb().bit_is_set() {}
}

#[inline(always)]
pub(crate) fn smi_write(iar: &MACMIIAR, dr: &MACMIIDR, phy: u8, reg: u8, data: u16) {
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
    smi_wait_ready(iar);
}

#[inline(always)]
pub(crate) fn smi_read(iar: &MACMIIAR, dr: &MACMIIDR, phy: u8, reg: u8) -> u16 {
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
    smi_wait_ready(iar);

    // Return value:
    dr.read().md().bits()
}

/// Station Management Interface with pins and registers borrowed from [`EthernetMAC`].
///
/// Can also be constructed by borrowing from [`ETHERNET_MAC`] and borrowing the pins manually.
///
/// Provides access to the MIIM implementation exposed by the MCU's MAC API.
pub struct Smi<'eth, 'pins, Mdio, Mdc> {
    macmiiar: &'eth MACMIIAR,
    macmiidr: &'eth MACMIIDR,
    _mdio: &'pins mut Mdio,
    _mdc: &'pins mut Mdc,
}

impl<'eth, 'pins, Mdio, Mdc> StationManagement for Smi<'eth, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    /// Read an SMI register
    fn read(&self, phy: u8, reg: u8) -> u16 {
        smi_read(&self.macmiiar, &self.macmiidr, phy, reg)
    }

    /// Write an SMI register
    fn write(&mut self, phy: u8, reg: u8, data: u16) {
        smi_write(&self.macmiiar, &self.macmiidr, phy, reg, data)
    }
}

impl<'eth, 'pins, Mdio, Mdc> Smi<'eth, 'pins, Mdio, Mdc>
where
    Mdio: MdioPin,
    Mdc: MdcPin,
{
    /// Create the temporary `Smi` instance.
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

    /// Read an SMI register
    pub fn read(&self, phy: u8, reg: u8) -> u16 {
        smi_read(&self.macmiiar, &self.macmiidr, phy, reg)
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
