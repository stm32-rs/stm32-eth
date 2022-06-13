use crate::smi::{self, read, write, StationManagement};
use crate::stm32::ETHERNET_MAC;

pub struct NoSmi;

/// Ethernet media access control (MAC).
pub struct EthernetMAC<SMI> {
    pub(crate) eth_mac: ETHERNET_MAC,
    _state: SMI,
}

impl EthernetMAC<NoSmi> {
    pub fn new(eth_mac: ETHERNET_MAC) -> Self {
        Self {
            eth_mac,
            _state: NoSmi {},
        }
    }
}

pub struct BorrowedSmi;

pub struct OwnedSmi<MDIO, MDC>
where
    MDIO: smi::MdioPin,
    MDC: smi::MdcPin,
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
    ) -> smi::Smi<'eth, 'pins, Mdio, Mdc>
    where
        Mdio: smi::MdioPin,
        Mdc: smi::MdcPin,
    {
        smi::Smi::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, mdio, mdc)
    }
}

impl<MDIO, MDC> EthernetMAC<OwnedSmi<MDIO, MDC>>
where
    MDIO: smi::MdioPin,
    MDC: smi::MdcPin,
{
    pub fn new_owned(eth_mac: ETHERNET_MAC, mdio: MDIO, mdc: MDC) -> Self {
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
    MDIO: smi::MdioPin,
    MDC: smi::MdcPin,
{
    fn read(&self, phy: u8, reg: u8) -> u16 {
        read(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, phy, reg)
    }

    fn write(&mut self, phy: u8, reg: u8, data: u16) {
        write(
            &self.eth_mac.macmiiar,
            &self.eth_mac.macmiidr,
            phy,
            reg,
            data,
        )
    }
}
