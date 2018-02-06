use board::ethernet_mac::{MACMIIAR, MACMIIDR};

pub struct SMI<'a> {
    macmiiar: &'a MACMIIAR,
    macmiidr: &'a MACMIIDR,
}

impl<'a> SMI<'a> {
    pub fn new(macmiiar: &'a MACMIIAR, macmiidr: &'a MACMIIDR) -> Self {
        SMI {
            macmiiar,
            macmiidr,
        }
    }

    /// Wait for not busy
    fn wait_ready(&self) {
        while self.macmiiar.read().mb().bit_is_set() {}
    }
    
    pub fn read(&self, phy: u8, reg: u8) -> u16 {
        self.macmiiar.modify(|_, w| {
            unsafe {
                w.pa().bits(phy)
                    .mr().bits(reg)
                    /* Read operation MW=0 */
                    .mw().clear_bit()
                    .mb().set_bit()
            }
        });
        self.wait_ready();

        // Return value:
        self.macmiidr.read().td().bits()
    }

    pub fn write(&self, phy: u8, reg: u8, data: u16) {
        self.macmiidr.modify(|_, w| {
            unsafe { w.td().bits(data) }
        });
        self.macmiiar.modify(|_, w| {
            unsafe {
                w.pa().bits(phy)
                    .mr().bits(reg)
                    /* Write operation MW=1*/
                    .mw().set_bit()
                    .mb().set_bit()
            }
        });
        self.wait_ready();
    }

    pub fn set_bits(&self, phy: u8, reg: u8, mask: u16) {
        let value = self.read(phy, reg);
        self.write(phy, reg, value | mask);
    }
}
