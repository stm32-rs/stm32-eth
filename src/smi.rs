#[cfg(feature = "stm32f4xx-hal")]
use stm32f4xx_hal::stm32;
#[cfg(feature = "stm32f7xx-hal")]
use stm32f7xx_hal::device as stm32;

use stm32::ethernet_mac::{MACMIIAR, MACMIIDR};

/// MDIO pin types.
pub unsafe trait MdioPin {}
/// MDC pin types.
pub unsafe trait MdcPin {}

/// Station Management Interface.
///
/// Provides access to the MIIM implementation exposed by the MCU's MAC API.
pub struct Smi<'eth, 'pins, Mdio, Mdc> {
    macmiiar: &'eth MACMIIAR,
    macmiidr: &'eth MACMIIDR,
    _mdio: &'pins mut Mdio,
    _mdc: &'pins mut Mdc,
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

    /// Wait for not busy
    fn wait_ready(&self) {
        while self.macmiiar.read().mb().bit_is_set() {}
    }

    fn read_data(&self) -> u16 {
        self.macmiidr.read().md().bits()
    }

    /// Read an SMI register
    pub fn read(&self, phy: u8, reg: u8) -> u16 {
        self.macmiiar.modify(|_, w| {
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
        self.wait_ready();

        // Return value:
        self.read_data()
    }

    fn write_data(&self, data: u16) {
        self.macmiidr.write(|w| w.md().bits(data));
    }

    /// Write an SMI register
    pub fn write(&self, phy: u8, reg: u8, data: u16) {
        self.write_data(data);
        self.macmiiar.modify(|_, w| {
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
        self.wait_ready();
    }
}

#[cfg(feature = "device-selected")]
mod pin_impls {
    #[cfg(feature = "stm32f4xx-hal")]
    use stm32f4xx_hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate, AF11};
    #[cfg(feature = "stm32f7xx-hal")]
    use stm32f7xx_hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate, AF11};

    unsafe impl super::MdioPin for PA2<Alternate<AF11>> {}
    unsafe impl super::MdcPin for PC1<Alternate<AF11>> {}
}
