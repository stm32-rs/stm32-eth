use cortex_m::interrupt::CriticalSection;
use core::option::Option;

use eth::smi::SMI;

const PHY_REG_BCR: u8 = 0x00;
const PHY_REG_BSR: u8 = 0x01;
const PHY_REG_ID1: u8 = 0x02;
const PHY_REG_ID2: u8 = 0x03;
const PHY_REG_ANTX: u8 = 0x04;
const PHY_REG_ANRX: u8 = 0x05;
const PHY_REG_ANEXP: u8 = 0x06;
const PHY_REG_ANNPTX: u8 = 0x07;
const PHY_REG_ANNPRX: u8 = 0x08;
const PHY_REG_SSR: u8 = 0x1F;  // Special Status Register

const PHY_REG_BCR_COLTEST: u16 = 1 << 7;
const PHY_REG_BCR_FD: u16 = 1 << 8;
const PHY_REG_BCR_ANRST: u16 = 1 << 9;
const PHY_REG_BCR_ISOLATE: u16 = 1 << 10;
const PHY_REG_BCR_POWERDN: u16 = 1 << 11;
const PHY_REG_BCR_AN: u16 = 1 << 12;
const PHY_REG_BCR_100M: u16 = 1 << 13;
const PHY_REG_BCR_LOOPBACK: u16 = 1 << 14;
const PHY_REG_BCR_RESET: u16 = 1 << 15;

const PHY_REG_BSR_JABBER: u16 = 1 << 1;
const PHY_REG_BSR_UP: u16 = 1 << 2;
const PHY_REG_BSR_FAULT: u16 = 1 << 4;
const PHY_REG_BSR_ANDONE: u16 = 1 << 5;

const PHY_REG_SSR_ANDONE: u16 = 1 << 12;
const PHY_REG_SSR_SPEED: u16 = 0b111 << 2;
const PHY_REG_SSR_10BASE_HD: u16 = 0b001 << 2;
const PHY_REG_SSR_10BASE_FD: u16 = 0b101 << 2;
const PHY_REG_SSR_100BASE_HD: u16 = 0b010 << 2;
const PHY_REG_SSR_100BASE_FD: u16 = 0b110 << 2;

/// http://ww1.microchip.com/downloads/en/DeviceDoc/DS_LAN8742_00001989A.pdf
/// https://github.com/libopencm3/libopencm3/blob/master/lib/ethernet/mac_stm32fxx7.c
pub struct Phy<'cs> {
    phy: u8,
    smi: SMI<'cs>
}

impl Phy<'static> {
    pub unsafe fn new(phy: u8) -> Self {
        Phy {
            phy,
            smi: SMI::new(),
        }
    }
}
    

impl<'cs> Phy<'cs> {
    pub fn with(cs: &'cs CriticalSection, phy: u8) -> Self {
        Phy {
            phy,
            smi: SMI::with(cs),
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
