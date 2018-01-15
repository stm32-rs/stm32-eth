use stm32f429x::{ETHERNET_MAC, EthernetMac};
use cortex_m::interrupt::CriticalSection;

pub struct SMI<'cs> {
    eth_mac: &'cs EthernetMac,
}

impl SMI<'static> {
    pub unsafe fn new() -> Self {
        SMI {
            eth_mac: &*ETHERNET_MAC.get(),
        }
    }
}
    
impl<'cs> SMI<'cs> {
    pub fn with(cs: &'cs CriticalSection) -> Self {
        SMI {
            eth_mac: ETHERNET_MAC.borrow(cs),
        }
    }

    /// Wait for not busy
    fn wait_ready(&self) {
        while self.eth_mac.macmiiar.read().mb().bits() != 0 {}
    }
    
    pub fn read(&self, phy: u8, reg: u8) -> u16 {
        /* Read operation MW=0 */
        self.eth_mac.macmiiar.modify(|_, w| {
            unsafe {
                w.pa().bits(phy)
                    .mr().bits(reg)
                    .mw().bits(0)
                    .mb().bits(1)
            }
        });
        self.wait_ready();

        // Return value:
        self.eth_mac.macmiidr.read().td().bits()
    }

    pub fn write(&self, phy: u8, reg: u8, data: u16) {
        self.eth_mac.macmiidr.modify(|_, w| {
            unsafe { w.td().bits(data) }
        });
        /* Write operation MW=1*/
        self.eth_mac.macmiiar.modify(|_, w| {
            unsafe {
                w.pa().bits(phy)
                    .mr().bits(reg)
                    .mw().bits(1)
                    .mb().bits(1)
            }
        });
        self.wait_ready();
    }

    pub fn set_bits(&self, phy: u8, reg: u8, mask: u16) {
        let value = self.read(phy, reg);
        self.write(phy, reg, value | mask);
    }
}
