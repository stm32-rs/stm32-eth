use crate::stm32::ETHERNET_MAC;

mod smi;
pub use smi::*;

/// Ethernet media access control (MAC).
pub struct EthernetMAC {
    pub(crate) eth_mac: ETHERNET_MAC,
}

impl EthernetMAC {
    /// Create a new EthernetMAC that does not own its MDIO and MDC pins.
    pub fn new(eth_mac: ETHERNET_MAC) -> Self {
        Self { eth_mac }
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

    /// Turn this [`EthernetMAC`] into an [`EthernetMACWithSmi`]
    pub fn with_smi<MDIO, MDC>(self, mdio: MDIO, mdc: MDC) -> EthernetMACWithSmi<MDIO, MDC>
    where
        MDIO: MdioPin,
        MDC: MdcPin,
    {
        EthernetMACWithSmi {
            eth_mac: self.eth_mac,
            mdio: mdio,
            mdc: mdc,
        }
    }
}

/// Ethernet media access control (MAC)
///
/// This version of the struct owns it's Station Management Interface
/// pins, allowing it to be used [`StationManagement`] directly.
pub struct EthernetMACWithSmi<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    pub(crate) eth_mac: ETHERNET_MAC,
    mdio: MDIO,
    mdc: MDC,
}

impl<MDIO, MDC> EthernetMACWithSmi<MDIO, MDC>
where
    MDIO: MdioPin,
    MDC: MdcPin,
{
    /// Create a new EthernetMAC with owned MDIO and MDC pins.
    ///
    /// To interact with a connected Phy, use this struct's impl of
    /// [`StationManagement`]
    pub fn new(eth_mac: ETHERNET_MAC, mdio: MDIO, mdc: MDC) -> Self {
        Self {
            eth_mac,
            mdio: mdio,
            mdc: mdc,
        }
    }

    /// Release the owned MDIO and MDC pins, and return an EthernetMAC that
    /// has to borrow the MDIO and MDC pins.
    pub fn release_pins(self) -> (EthernetMAC, MDIO, MDC) {
        (
            EthernetMAC {
                eth_mac: self.eth_mac,
            },
            self.mdio,
            self.mdc,
        )
    }
}

impl<MDIO, MDC> StationManagement for EthernetMACWithSmi<MDIO, MDC>
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
