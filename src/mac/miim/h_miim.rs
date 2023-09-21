use crate::peripherals::ETHERNET_MAC;
use crate::stm32::ethernet_mac::MACMDIOAR;

use super::{MdcPin, MdioPin};

use crate::hal::gpio::{Alternate, PA2, PC1};

#[inline(always)]
fn miim_wait_ready(iar: &MACMDIOAR) {
    while iar.read().mb().bit_is_set() {}
}

#[inline(always)]
pub(crate) fn miim_write(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8, data: u16) {
    miim_wait_ready(&eth_mac.macmdioar);

    eth_mac.macmdiodr.write(|w| unsafe { w.md().bits(data) });

    miim_wait_ready(&eth_mac.macmdioar);

    eth_mac.macmdioar.modify(|_, w| unsafe {
        w.pa()
            .bits(phy)
            .rda()
            .bits(reg)
            /* Write operation GOC=01*/
            .goc()
            .variant(0b01)
            .mb()
            .set_bit()
    });

    miim_wait_ready(&eth_mac.macmdioar);
}

#[inline(always)]
pub(crate) fn miim_read(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8) -> u16 {
    miim_wait_ready(&eth_mac.macmdioar);

    eth_mac.macmdioar.modify(|_, w| unsafe {
        w.pa()
            .bits(phy)
            .rda()
            .bits(reg)
            /* Write operation GOC=11*/
            .goc()
            .variant(0b11)
            .mb()
            .set_bit()
    });

    miim_wait_ready(&eth_mac.macmdioar);

    // Return value:
    eth_mac.macmdiodr.read().md().bits()
}

unsafe impl MdcPin for PC1<Alternate<11>> {}
unsafe impl MdioPin for PA2<Alternate<11>> {}
