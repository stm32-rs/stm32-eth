//! SMSC LAN87xxA (LAN8742A, LAN8720A) Ethernet PHYs
use core::convert::TryFrom;

use crate::{mac::SerialManagement, LinkSpeed};

/// SMSC LAN8720A Ethernet PHY
pub type LAN8720A<SMI> = LAN87xxA<SMI, false>;
/// SMSC LAN8742A Ethernet PHY
pub type LAN8742A<SMI> = LAN87xxA<SMI, true>;

use self::phy_consts::*;
#[allow(dead_code)]
mod phy_consts {
    pub const PHY_REG_BCR: u8 = 0x00;
    pub const PHY_REG_BSR: u8 = 0x01;
    pub const PHY_REG_ID1: u8 = 0x02;
    pub const PHY_REG_ID2: u8 = 0x03;
    pub const PHY_REG_ANTX: u8 = 0x04;
    pub const PHY_REG_ANRX: u8 = 0x05;
    pub const PHY_REG_ANEXP: u8 = 0x06;
    pub const PHY_REG_ANNPTX: u8 = 0x07;
    pub const PHY_REG_ANNPRX: u8 = 0x08;
    pub const PHY_REG_SSR: u8 = 0x1F; // Special Status Register
    pub const PHY_REG_CTL: u8 = 0x0D; // Ethernet PHY Register Control
    pub const PHY_REG_ADDAR: u8 = 0x0E; // Ethernet PHY Address or Data

    pub const PHY_REG_WUCSR: u16 = 0x8010;

    pub const PHY_REG_BCR_COLTEST: u16 = 1 << 7;
    pub const PHY_REG_BCR_FD: u16 = 1 << 8;
    pub const PHY_REG_BCR_ANRST: u16 = 1 << 9;
    pub const PHY_REG_BCR_ISOLATE: u16 = 1 << 10;
    pub const PHY_REG_BCR_POWERDN: u16 = 1 << 11;
    pub const PHY_REG_BCR_AN: u16 = 1 << 12;
    pub const PHY_REG_BCR_100M: u16 = 1 << 13;
    pub const PHY_REG_BCR_LOOPBACK: u16 = 1 << 14;
    pub const PHY_REG_BCR_RESET: u16 = 1 << 15;

    pub const PHY_REG_ANTX_100BTXFD: u16 = 1 << 8;
    pub const PHY_REG_ANTX_100BTXHD: u16 = 1 << 7;
    pub const PHY_REG_ANTX_10BTXFD: u16 = 1 << 6;
    pub const PHY_REG_ANTX_10BTXHD: u16 = 1 << 5;

    pub const PHY_REG_BSR_JABBER: u16 = 1 << 1;
    pub const PHY_REG_BSR_UP: u16 = 1 << 2;
    pub const PHY_REG_BSR_FAULT: u16 = 1 << 4;
    pub const PHY_REG_BSR_ANDONE: u16 = 1 << 5;

    pub const PHY_REG_SSR_ANDONE: u16 = 1 << 12;
    pub const PHY_REG_SSR_SPEED: u16 = 0b111 << 2;
    pub const PHY_REG_SSR_10BASE_HD: u16 = 0b001 << 2;
    pub const PHY_REG_SSR_10BASE_FD: u16 = 0b101 << 2;
    pub const PHY_REG_SSR_100BASE_HD: u16 = 0b010 << 2;
    pub const PHY_REG_SSR_100BASE_FD: u16 = 0b110 << 2;
}

/// An SMSC LAN87XXA Ethernet PHY.
///
/// EXT_WUCSR_CLEAR is used to determine if the "WU CSR" bit
/// in extended registers should be cleared
///
/// This type should not be used directly. Use [`LAN8720A`] or [`LAN8742A`] instead.
pub struct LAN87xxA<S, const EXT_WUCSR_CLEAR: bool> {
    phy_addr: u8,
    smi: S,
}

impl<S, const EXT_WUCSR_CLEAR: bool> LAN87xxA<S, EXT_WUCSR_CLEAR>
where
    S: SerialManagement,
{
    /// Create a new LAN87XXA based PHY
    pub fn new(smi: S, phy_addr: u8) -> Self {
        LAN87xxA { smi, phy_addr }
    }

    fn write(&mut self, reg: u8, data: u16) {
        self.smi.write(self.phy_addr, reg, data)
    }

    /// Writes a value to an extended PHY register in MMD address space.
    ///
    /// Only available in `LAN8742A` PHYs
    fn smi_write_ext(&mut self, reg_addr: u16, reg_data: u16) {
        self.write(PHY_REG_CTL, 0x0003); // set address
        self.write(PHY_REG_ADDAR, reg_addr);
        self.write(PHY_REG_CTL, 0x4003); // set data
        self.write(PHY_REG_ADDAR, reg_data);
    }

    fn read(&mut self, reg: u8) -> u16 {
        self.smi.read(self.phy_addr, reg)
    }

    /// Reset PHY and wait for it to come out of reset.
    pub fn phy_reset(&mut self) {
        self.write(PHY_REG_BCR, PHY_REG_BCR_RESET);
        while self.read(PHY_REG_BCR) & PHY_REG_BCR_RESET == PHY_REG_BCR_RESET {}
    }

    /// Initialize the PHY
    pub fn phy_init(&mut self) {
        if EXT_WUCSR_CLEAR {
            // Clear WU CSR
            self.smi_write_ext(PHY_REG_WUCSR, 0);
        }

        // Enable auto-negotiation
        self.write(
            PHY_REG_BCR,
            PHY_REG_BCR_AN | PHY_REG_BCR_ANRST | PHY_REG_BCR_100M,
        );

        // Advertise all available link speeds and duplex modes
        self.write(
            PHY_REG_ANTX,
            PHY_REG_ANTX_100BTXFD
                | PHY_REG_ANTX_100BTXHD
                | PHY_REG_ANTX_10BTXFD
                | PHY_REG_ANTX_10BTXHD,
        );
    }

    /// Poll PHY to determine link status.
    pub fn poll_link(&mut self) -> bool {
        let bsr = self.read(PHY_REG_BSR);
        let ssr = self.read(PHY_REG_SSR);

        // No link without autonegotiate
        if bsr & PHY_REG_BSR_ANDONE == 0 {
            return false;
        }
        // No link if link is down
        if bsr & PHY_REG_BSR_UP == 0 {
            return false;
        }
        // No link if autonegotiate incomplete
        if ssr & PHY_REG_SSR_ANDONE == 0 {
            return false;
        }
        // Got link
        true
    }

    /// Get the link speed
    ///
    /// If this returns `None`, some sort of corruption occured, or the PHY is
    /// in an illegal state
    pub fn link_speed(&mut self) -> Option<LinkSpeed> {
        let link_data = self.read(PHY_REG_SSR);
        let link_data = ((link_data >> 2) & 0b111) as u8;
        LinkSpeed::try_from(link_data).ok()
    }

    /// Check if the link is up
    pub fn link_established(&mut self) -> bool {
        self.poll_link()
    }

    /// Block until a link is established
    pub fn block_until_link(&mut self) {
        while !self.link_established() {}
    }

    /// Release the underlying [`SerialManagement`]
    pub fn release(self) -> S {
        self.smi
    }
}

impl<S, const EXT_WUCSR_CLEAR: bool> super::Phy for LAN87xxA<S, EXT_WUCSR_CLEAR>
where
    S: SerialManagement,
{
    type LinkSpeed = Option<LinkSpeed>;

    fn reset(&mut self) {
        self.phy_reset()
    }

    fn init(&mut self) {
        self.phy_init()
    }

    fn poll_link(&mut self) -> bool {
        self.poll_link()
    }

    fn link_speed(&mut self) -> Self::LinkSpeed {
        self.link_speed()
    }
}
