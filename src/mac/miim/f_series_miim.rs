use crate::peripherals::ETHERNET_MAC;
use crate::stm32::ethernet_mac::MACMIIAR;

#[inline(always)]
fn miim_wait_ready(iar: &MACMIIAR) {
    while iar.read().mb().bit_is_set() {}
}

#[inline(always)]
pub(crate) fn miim_write(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8, data: u16) {
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
pub(crate) fn miim_read(eth_mac: &mut ETHERNET_MAC, phy: u8, reg: u8) -> u16 {
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

#[cfg(feature = "stm32f4xx-hal")]
mod pin_impls {
    use crate::hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate};

    const AF11: u8 = 11;

    unsafe impl crate::mac::MdioPin for PA2<Alternate<AF11>> {}
    unsafe impl crate::mac::MdcPin for PC1<Alternate<AF11>> {}
}

#[cfg(feature = "stm32f7xx-hal")]
mod pin_impls {
    use crate::hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate};

    const AF11: u8 = 11;

    unsafe impl crate::mac::MdioPin for PA2<Alternate<AF11>> {}
    unsafe impl crate::mac::MdcPin for PC1<Alternate<AF11>> {}
}

#[cfg(feature = "stm32f1xx-hal")]
mod pin_impls {
    use crate::hal::gpio::{gpioa::PA2, gpioc::PC1, Alternate, PushPull};

    unsafe impl crate::mac::MdioPin for PA2<Alternate<PushPull>> {}
    unsafe impl crate::mac::MdcPin for PC1<Alternate<PushPull>> {}
}
