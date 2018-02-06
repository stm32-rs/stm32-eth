use core::option::Option;
use board::ethernet_mac::{MACMIIAR, MACMIIDR};

use smi::SMI;

#[allow(dead_code)]
mod consts {
    pub const PHY_REG_BCR: u8 = 0x00;
    pub const PHY_REG_BSR: u8 = 0x01;
    pub const PHY_REG_ID1: u8 = 0x02;
    pub const PHY_REG_ID2: u8 = 0x03;
    pub const PHY_REG_ANTX: u8 = 0x04;
    pub const PHY_REG_ANRX: u8 = 0x05;
    pub const PHY_REG_ANEXP: u8 = 0x06;
    pub const PHY_REG_ANNPTX: u8 = 0x07;
    pub const PHY_REG_ANNPRX: u8 = 0x08;
    pub const PHY_REG_SSR: u8 = 0x1F;  // Special Status Register

    pub const PHY_REG_BCR_COLTEST: u16 = 1 << 7;
    pub const PHY_REG_BCR_FD: u16 = 1 << 8;
    pub const PHY_REG_BCR_ANRST: u16 = 1 << 9;
    pub const PHY_REG_BCR_ISOLATE: u16 = 1 << 10;
    pub const PHY_REG_BCR_POWERDN: u16 = 1 << 11;
    pub const PHY_REG_BCR_AN: u16 = 1 << 12;
    pub const PHY_REG_BCR_100M: u16 = 1 << 13;
    pub const PHY_REG_BCR_LOOPBACK: u16 = 1 << 14;
    pub const PHY_REG_BCR_RESET: u16 = 1 << 15;

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
use self::consts::*;

/// http://ww1.microchip.com/downloads/en/DeviceDoc/DS_LAN8742_00001989A.pdf
/// https://github.com/libopencm3/libopencm3/blob/master/lib/ethernet/mac_stm32fxx7.c
pub struct Phy<'a> {
    smi: SMI<'a>,
    phy: u8,
}

impl<'a> Phy<'a> {
    pub fn new(macmiiar: &'a MACMIIAR, macmiidr: &'a MACMIIDR, phy: u8) -> Self {
        let smi = SMI::new(macmiiar, macmiidr);

        Phy {
            smi,
            phy,
        }
    }

    pub fn status(&self) -> PhyStatus {
        PhyStatus {
            bsr: self.smi.read(self.phy, PHY_REG_BSR),
            ssr: self.smi.read(self.phy, PHY_REG_SSR),
        }
    }

    pub fn reset(&self) -> &Self {
        self.smi.set_bits(
            self.phy,
            PHY_REG_BCR,
            PHY_REG_BCR_RESET
        );

        // wait until reset bit is cleared by phy
        while (self.smi.read(
            self.phy,
            PHY_REG_BCR
        ) & PHY_REG_BCR_RESET) == PHY_REG_BCR_RESET {}

        self
    }

    pub fn set_autoneg(&self) -> &Self {
        self.smi.set_bits(
            self.phy,
            PHY_REG_BCR,
            PHY_REG_BCR_AN | PHY_REG_BCR_ANRST | PHY_REG_BCR_100M
        );

        self
    }
}

#[derive(Copy, Clone)]
pub struct PhyStatus {
    /// TOOD: pub for debugging
    pub bsr: u16,
    pub ssr: u16,
}

impl PhyStatus {
    pub fn link_detected(&self) -> bool {
        (self.bsr & PHY_REG_BSR_UP) == PHY_REG_BSR_UP
    }

    pub fn autoneg_done(&self) -> bool {
        (self.bsr & PHY_REG_BSR_ANDONE) == PHY_REG_BSR_ANDONE ||
        (self.ssr & PHY_REG_SSR_ANDONE) == PHY_REG_SSR_ANDONE
    }

    pub fn is_full_duplex(&self) -> Option<bool> {
        match self.ssr & PHY_REG_SSR_SPEED {
            PHY_REG_SSR_10BASE_HD |
            PHY_REG_SSR_100BASE_HD =>
                Some(false),
            PHY_REG_SSR_10BASE_FD |
            PHY_REG_SSR_100BASE_FD =>
                Some(true),
            _ =>
                None,
        }
    }
    
    pub fn speed(&self) -> u32 {
        match self.ssr & PHY_REG_SSR_SPEED {
            PHY_REG_SSR_10BASE_HD |
            PHY_REG_SSR_10BASE_FD =>
                10,
            PHY_REG_SSR_100BASE_HD |
            PHY_REG_SSR_100BASE_FD =>
                100,
            _ =>
                0,
        }
    }

    pub fn remote_fault(&self) -> bool {
        (self.bsr & PHY_REG_BSR_FAULT) == PHY_REG_BSR_FAULT
    }
}

impl PartialEq for PhyStatus {
    fn eq(&self, other: &PhyStatus) -> bool {
        (self.link_detected() == false &&
         other.link_detected() == false)
        ||
        (self.link_detected() == other.link_detected() &&
         self.is_full_duplex() == other.is_full_duplex() &&
         self.speed() == other.speed())
    }
}
