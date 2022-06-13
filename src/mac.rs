use crate::stm32::{
    ethernet_mac::{MACMIIAR, MACMIIDR},
    ETHERNET_MAC,
};

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
fn smi_write(iar: &MACMIIAR, dr: &MACMIIDR, phy: u8, reg: u8, data: u16) {
    dr.write(|w| {
        #[cfg_attr(not(feature = "stm32f107"), allow(unused_unsafe))]
        unsafe {
            w.md().bits(data)
        }
    });

    iar.modify(|_, w| {
        #[cfg_attr(not(feature = "stm32f107"), allow(unused_unsafe))]
        unsafe {
            w.pa()
                .bits(phy)
                .mr()
                .bits(reg)
                /* Write operation MW=1*/
                .mw()
                .set_bit()
                .mb()
                .set_bit()
        }
    });
    smi_wait_ready(iar);
}

#[inline(always)]
fn smi_read(iar: &MACMIIAR, dr: &MACMIIDR, phy: u8, reg: u8) -> u16 {
    iar.modify(|_, w| {
        #[cfg_attr(not(feature = "stm32f107"), allow(unused_unsafe))]
        unsafe {
            w.pa()
                .bits(phy)
                .mr()
                .bits(reg)
                /* Read operation MW=0 */
                .mw()
                .clear_bit()
                .mb()
                .set_bit()
        }
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

/// Ethernet media access control (MAC).
pub struct EthernetMAC<SMI> {
    pub(crate) eth_mac: ETHERNET_MAC,
    _state: SMI,
}

/// Access to the MAC's SMI can be obtained by borrowing [`MdioPin`] and [`MdcPin`] to it. For [`StationManagement`], see [`EthernetMAC::smi`]
pub struct BorrowedSmi;

/// The SMI (and its pins) are owned. An [`EthernetMAC`] that owns
/// its SMI is [`StationManagement`]
pub struct OwnedSmi<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    _mdio: MDIO,
    _mdc: MDC,
}

impl EthernetMAC<BorrowedSmi> {
    pub fn new(eth_mac: ETHERNET_MAC) -> Self {
        Self {
            eth_mac,
            _state: BorrowedSmi {},
        }
    }

    /// Borrow access to the MAC's SMI.
    ///
    /// Allows for controlling and monitoring any PHYs that may be accessible via the MDIO/MDC
    /// pins.
    ///
    /// Exclusive access to the `MDIO` and `MDC` is required to ensure that are not used elsewhere
    /// for the duration of SMI communication.
    pub fn smi<'eth, 'pins, Mdio, Mdc>(
        &'eth mut self,
        mdio: &'pins mut Mdio,
        mdc: &'pins mut Mdc,
    ) -> Smi<'eth, 'pins, Mdio, Mdc>
    where
        Mdio: MdioPin,
        Mdc: MdcPin,
    {
        Smi::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, mdio, mdc)
    }
}

impl<MDIO, MDC> EthernetMAC<OwnedSmi<MDIO, MDC>>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    pub fn new(eth_mac: ETHERNET_MAC, mdio: MDIO, mdc: MDC) -> Self {
        Self {
            eth_mac,
            _state: OwnedSmi {
                _mdio: mdio,
                _mdc: mdc,
            },
        }
    }
}

impl<MDIO, MDC> StationManagement for EthernetMAC<OwnedSmi<MDIO, MDC>>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    fn read(&self, phy: u8, reg: u8) -> u16 {
        smi_read(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, phy, reg)
    }

    fn write(&mut self, phy: u8, reg: u8, data: u16) {
        smi_write(
            &self.eth_mac.macmiiar,
            &self.eth_mac.macmiidr,
            phy,
            reg,
            data,
        )
    }
}
