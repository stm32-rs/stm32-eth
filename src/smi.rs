#[cfg(feature = "stm32f4xx-hal")]
use stm32f4xx_hal::stm32;
#[cfg(feature = "stm32f7xx-hal")]
use stm32f7xx_hal::pac as stm32;

use stm32::ethernet_mac::{MACMIIAR, MACMIIDR};

/// Station Management Interface
pub struct SMI<'a> {
    macmiiar: &'a MACMIIAR,
    macmiidr: &'a MACMIIDR,
}

impl<'a> SMI<'a> {
    /// Allocate
    pub fn new(macmiiar: &'a MACMIIAR, macmiidr: &'a MACMIIDR) -> Self {
        SMI { macmiiar, macmiidr }
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

    /// Helper: `read()` and `write()` by OR-ing the current value of
    /// the register `reg` with `mask`.
    pub fn set_bits(&self, phy: u8, reg: u8, mask: u16) {
        let value = self.read(phy, reg);
        self.write(phy, reg, value | mask);
    }
}
