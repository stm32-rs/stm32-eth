#![no_std]

/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f7xx-hal")]
pub use stm32f7xx_hal::pac as stm32;

/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f4xx-hal")]
pub use stm32f4xx_hal::pac as stm32;

/// Re-export
#[cfg(feature = "stm32f1xx-hal")]
pub use stm32f1xx_hal as hal;
/// Re-export
#[cfg(feature = "stm32f1xx-hal")]
pub use stm32f1xx_hal::pac as stm32;

use hal::rcc::Clocks;
use stm32::{Interrupt, ETHERNET_DMA, ETHERNET_MAC, ETHERNET_MMC, ETHERNET_PTP, NVIC};

mod ring;
#[cfg(feature = "smi")]
pub mod smi;
pub use ring::RingEntry;
mod desc;
mod rx;
pub use rx::{RxDescriptor, RxError, RxRingEntry};
use rx::{RxPacket, RxRing};
mod tx;
use tx::TxRing;
pub use tx::{TxDescriptor, TxError, TxRingEntry};
pub mod setup;
pub use setup::EthPins;
use setup::{
    AlternateVeryHighSpeed, RmiiCrsDv, RmiiRefClk, RmiiRxD0, RmiiRxD1, RmiiTxD0, RmiiTxD1, RmiiTxEN,
};

#[cfg(feature = "smoltcp-phy")]
pub use smoltcp;
#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::*;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct Timestamp {
    pub seconds: u32,
    pub nanos: u32,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimestampError {
    NotYetTimestamped,
    IdNotFound,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub struct PacketId(pub(crate) usize);

/// From the datasheet: *VLAN Frame maxsize = 1522*
const MTU: usize = 1522;

mod consts {
    /* For HCLK 60-100 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
    /* For HCLK 100-150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
    /* For HCLK 20-35 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
    /* For HCLK 35-60 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
    /* For HCLK over 150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
}
use self::consts::*;

/// HCLK must be at least 25MHz to use the ethernet peripheral.
#[derive(Debug)]
pub struct WrongClock;

/// Ethernet DMA.
pub struct EthernetDMA<'rx, 'tx, const TX_SIZE: usize> {
    eth_dma: ETHERNET_DMA,
    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx, TX_SIZE>,
}
/// Ethernet media access control (MAC).
pub struct EthernetMAC {
    eth_mac: ETHERNET_MAC,
}

/// Create and initialise the ethernet driver.
///
/// Initialize and start tx and rx DMA engines.
/// Sets up the peripheral clocks and GPIO configuration,
/// and configures the ETH MAC and DMA peripherals.
/// Automatically sets slew rate to VeryHigh.
/// If you wish to use another configuration, please see
/// [new_unchecked](new_unchecked).
///
/// This method does not initialise the external PHY. However it does return an
/// [EthernetMAC](EthernetMAC) which implements the
/// [StationManagement](smi::StationManagement) trait. This can be used to
/// communicate with the external PHY.
///
/// # Note
/// - Make sure that the buffers reside in a memory region that is
/// accessible by the peripheral. Core-Coupled Memory (CCM) is
/// usually not accessible.
/// - HCLK must be at least 25 MHz.
pub fn new<'rx, 'tx, REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1, const TX_SIZE: usize>(
    eth_mac: ETHERNET_MAC,
    eth_mmc: ETHERNET_MMC,
    eth_dma: ETHERNET_DMA,
    eth_ptp: ETHERNET_PTP,
    rx_buffer: &'rx mut [RxRingEntry],
    tx_buffer: &'tx mut [TxRingEntry],
    clocks: Clocks,
    pins: EthPins<REFCLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>,
) -> Result<(EthernetDMA<'rx, 'tx, TX_SIZE>, EthernetMAC), WrongClock>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
{
    pins.setup_pins();
    unsafe {
        new_unchecked(
            eth_mac, eth_mmc, eth_dma, eth_ptp, rx_buffer, tx_buffer, clocks,
        )
    }
}

/// Create and initialise the ethernet driver (without GPIO configuration and validation).
///
/// This method does not initialise the external PHY. However it does return an
/// [EthernetMAC](EthernetMAC) which implements the
/// [StationManagement](smi::StationManagement) trait. This can be used to
/// communicate with the external PHY.
///
/// # Note
/// - Make sure that the buffers reside in a memory region that is
/// accessible by the peripheral. Core-Coupled Memory (CCM) is
/// usually not accessible.
/// - HCLK must be at least 25MHz.
pub unsafe fn new_unchecked<'rx, 'tx, const TX_SIZE: usize>(
    eth_mac: ETHERNET_MAC,
    eth_mmc: ETHERNET_MMC,
    eth_dma: ETHERNET_DMA,
    eth_ptp: ETHERNET_PTP,
    rx_buffer: &'rx mut [RxRingEntry],
    tx_buffer: &'tx mut [TxRingEntry],
    clocks: Clocks,
) -> Result<(EthernetDMA<'rx, 'tx, TX_SIZE>, EthernetMAC), WrongClock> {
    setup::setup();

    let clock_frequency = clocks.hclk().to_Hz();

    let clock_range = match clock_frequency {
        0..=24_999_999 => return Err(WrongClock),
        25_000_000..=34_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_16,
        35_000_000..=59_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_26,
        60_000_000..=99_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_42,
        100_000_000..=149_999_999 => ETH_MACMIIAR_CR_HCLK_DIV_62,
        _ => ETH_MACMIIAR_CR_HCLK_DIV_102,
    };

    // reset DMA bus mode register
    eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

    // Wait until done
    while eth_dma.dmabmr.read().sr().bit_is_set() {}

    // Setup PTP timestamping

    eth_ptp.ptptscr.write(|w| {
        w.tse()
            .set_bit()
            .tsfcu()
            .set_bit()
            .tsssr()
            .set_bit()
            .tssarfe()
            .set_bit()
    });

    // Set sub-second increment to 20ns and initial addend to HCLK/(1/20ns) (HCLK=100MHz)
    eth_ptp.ptpssir.write(|w| w.stssi().bits(20));
    eth_ptp.ptptsar.write(|w| w.tsa().bits(1 << 31));
    eth_ptp.ptptscr.modify(|_, w| w.ttsaru().set_bit());
    while eth_ptp.ptptscr.read().ttsaru().bit_is_set() {}

    // Initialise timestamp
    eth_ptp.ptptscr.modify(|_, w| w.tssti().set_bit());
    while eth_ptp.ptptscr.read().tssti().bit_is_set() {}

    // set clock range in MAC MII address register
    eth_mac.macmiiar.modify(|_, w| w.cr().bits(clock_range));

    // Configuration Register
    eth_mac.maccr.modify(|_, w| {
        // CRC stripping for Type frames. STM32F1xx do not have this bit.
        #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
        let w = w.cstf().set_bit();

        // Fast Ethernet speed
        w.fes()
            .set_bit()
            // Duplex mode
            .dm()
            .set_bit()
            // IPv4 checksum offload
            .ipco()
            .set_bit()
            // Automatic pad/CRC stripping
            .apcs()
            .set_bit()
            // Retry disable in half-duplex mode
            .rd()
            .set_bit()
            // Receiver enable
            .re()
            .set_bit()
            // Transmitter enable
            .te()
            .set_bit()
    });
    // frame filter register
    eth_mac.macffr.modify(|_, w| {
        // Receive All
        w.ra()
            .set_bit()
            // Promiscuous mode
            .pm()
            .set_bit()
    });
    // Flow Control Register
    eth_mac.macfcr.modify(|_, w| {
        // Pause time
        w.pt().bits(0x100)
    });
    // operation mode register
    eth_dma.dmaomr.modify(|_, w| {
        // Dropping of TCP/IP checksum error frames disable
        w.dtcefd()
            .set_bit()
            // Receive store and forward
            .rsf()
            .set_bit()
            // Disable flushing of received frames
            .dfrf()
            .set_bit()
            // Transmit store and forward
            .tsf()
            .set_bit()
            // Forward error frames
            .fef()
            .set_bit()
            // Operate on second frame
            .osf()
            .set_bit()
    });

    // disable all MMC RX interrupts
    eth_mmc
        .mmcrimr
        .write(|w| w.rgufm().set_bit().rfaem().set_bit().rfcem().set_bit());
    // disable all MMC TX interrupts
    eth_mmc
        .mmctimr
        .write(|w| w.tgfm().set_bit().tgfmscm().set_bit().tgfscm().set_bit());

    // bus mode register
    eth_dma.dmabmr.modify(|_, w| {
        // For any non-f107 chips, we must use enhanced descriptor format to support checksum
        // offloading and/or timestamps.
        #[cfg(not(feature = "stm32f107"))]
        let w = w.edfe().set_bit();

        // Address-aligned beats
        w.aab()
            .set_bit()
            // Fixed burst
            .fb()
            .set_bit()
            // Rx DMA PBL
            .rdp()
            .bits(32)
            // Programmable burst length
            .pbl()
            .bits(32)
            // Rx Tx priority ratio 2:1
            .pm()
            .bits(0b01)
            // Use separate PBL
            .usp()
            .set_bit()
    });

    let mut dma = EthernetDMA {
        eth_dma,
        rx_ring: RxRing::new(rx_buffer),
        tx_ring: TxRing::new(tx_buffer),
    };
    let mac = EthernetMAC { eth_mac };

    dma.rx_ring.start(&dma.eth_dma);
    dma.tx_ring.start(&dma.eth_dma);

    Ok((dma, mac))
}

impl<'rx, 'tx, const TX_SIZE: usize> EthernetDMA<'rx, 'tx, TX_SIZE> {
    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html) to
    /// clear interrupt pending bits. Otherwise the interrupt will
    /// reoccur immediately.
    pub fn enable_interrupt(&self) {
        self.eth_dma.dmaier.modify(|_, w| {
            w
                // Normal interrupt summary enable
                .nise()
                .set_bit()
                // Receive Interrupt Enable
                .rie()
                .set_bit()
                // Transmit Interrupt Enable
                .tie()
                .set_bit()
        });

        // Enable ethernet interrupts
        unsafe {
            NVIC::unmask(Interrupt::ETH);
        }
    }

    /// Call in interrupt handler to clear interrupt reason, when
    /// [`enable_interrupt()`](struct.EthernetDMA.html#method.enable_interrupt).
    ///
    /// There are two ways to call this:
    ///
    /// * Via the [`EthernetDMA`](struct.EthernetDMA.html) driver instance that your interrupt handler has access to.
    /// * By unsafely getting `Peripherals`.
    ///
    /// TODO: could return interrupt reason
    pub fn interrupt_handler(&self) {
        self.eth_dma
            .dmasr
            .write(|w| w.nis().set_bit().rs().set_bit().ts().set_bit());
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call `recv_next()` to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        self.rx_ring.running_state(&self.eth_dma).is_running()
    }

    /// Receive the next packet (if any is ready), or return `None`
    /// immediately.
    pub fn recv_next(&mut self) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(&self.eth_dma)
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        self.tx_ring.is_running(&self.eth_dma)
    }

    /// Send a packet
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(
        &mut self,
        length: usize,
        with_packet_id: Option<PacketId>,
        f: F,
    ) -> Result<R, TxError> {
        let result = self.tx_ring.send(length, with_packet_id, f);
        self.tx_ring.demand_poll(&self.eth_dma);
        result
    }

    pub fn get_timestamp<'a, PKT>(&mut self, packet_id: PKT) -> Result<Timestamp, TimestampError>
    where
        PKT: Into<PacketId>,
    {
        self.tx_ring.get_timestamp_for_id(packet_id.into())
    }
}

#[cfg(feature = "smi")]
impl EthernetMAC {
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

/// This block ensures that README.md is checked when `cargo test` is run.
///
/// Taken from https://github.com/rp-rs/pio-rs/blob/b52d3ba9c031ffa72bdd6f16b5fa8c0c04f0e2e0/src/lib.rs#L963
#[cfg(doctest)]
mod test_readme {
    macro_rules! external_doc_test {
        ($x:expr) => {
            #[doc = $x]
            extern "C" {}
        };
    }
    external_doc_test!(include_str!("../README.md"));
}
