#![no_std]

/// Re-export
pub use stm32f7xx_hal as hal;
/// Re-export
pub use stm32f7xx_hal::device;
use stm32f7xx_hal::device::{ETHERNET_MAC, ETHERNET_DMA, NVIC, Interrupt};

use cortex_m::asm;

pub mod phy;
use phy::{Phy, PhyStatus};
mod smi;
mod ring;
pub use ring::RingEntry;
mod desc;
mod rx;
use rx::{RxRing, RxRingEntry, RxPacket};
pub use rx::{RxDescriptor, RxError};
mod tx;
use tx::{TxRing, TxRingEntry};
pub use tx::{TxDescriptor, TxError};
mod setup;
pub use setup::setup;
#[cfg(feature = "nucleo-f767zi")]
pub use setup::setup_pins;

#[cfg(feature = "smoltcp-phy")]
pub use smoltcp;
#[cfg(feature = "smoltcp-phy")]
mod smoltcp_phy;
#[cfg(feature = "smoltcp-phy")]
pub use smoltcp_phy::{EthRxToken, EthTxToken};


const PHY_ADDR: u8 = 0;
/// From the datasheet: *VLAN Frame maxsize = 1522*
const MTU: usize = 1522;

#[allow(dead_code)]
mod consts {
    /* For HCLK 60-100 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
    /* For HCLK 100-150 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
    /* For HCLK 20-35 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
    /* For HCLK 35-60 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
    /* For HCLK 150-168 MHz */
    pub const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
}
use self::consts::*;

/// Ethernet driver for *STM32* chips with a *LAN8742*
/// [`Phy`](phy/struct.Phy.html) like they're found on STM Nucleo-144
/// boards.
pub struct Eth<'rx, 'tx> {
    eth_mac: ETHERNET_MAC,
    eth_dma: ETHERNET_DMA,
    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx>,
}

impl<'rx, 'tx> Eth<'rx, 'tx> {
    /// Initialize and start tx and rx DMA engines.
    ///
    /// You must call [`setup()`](fn.setup.html) before to initialize
    /// the hardware!
    ///
    /// Make sure that the buffers reside in a memory region that is
    /// accessible by the peripheral. Core-Coupled Memory (CCM) is
    /// usually not.
    ///
    /// Other than that, initializes and starts the Ethernet hardware
    /// so that you can [`send()`](#method.send) and
    /// [`recv_next()`](#method.recv_next).
    pub fn new(
        eth_mac: ETHERNET_MAC, eth_dma: ETHERNET_DMA,
        rx_buffer: &'rx mut [RxRingEntry], tx_buffer: &'tx mut [TxRingEntry]
    ) -> Self {
        let mut eth = Eth {
            eth_mac,
            eth_dma,
            rx_ring: RxRing::new(rx_buffer),
            tx_ring: TxRing::new(tx_buffer),
        };
        eth.init();
        eth.rx_ring.start(&eth.eth_dma);
        eth.tx_ring.start(&eth.eth_dma);
        eth
    }

    fn init(&mut self) -> &Self {
        self.reset_dma_and_wait();

        // set clock range in MAC MII address register
        let clock_range = ETH_MACMIIAR_CR_HCLK_DIV_26;
        self.eth_mac.macmiiar.modify(|_, w| unsafe { w.cr().bits(clock_range) });

        self.get_phy()
            .reset()
            .set_autoneg();

        // Configuration Register
        self.eth_mac.maccr.modify(|_, w| {
            // CRC stripping for Type frames
            w.cstf().set_bit()
                // Fast Ethernet speed
                .fes().set_bit()
                // Duplex mode
                .dm().set_bit()
                // Automatic pad/CRC stripping
                .apcs().set_bit()
                // Retry disable in half-duplex mode
                .rd().set_bit()
                // Receiver enable
                .re().set_bit()
                // Transmitter enable
                .te().set_bit()
        });
        // frame filter register
        self.eth_mac.macffr.modify(|_, w| {
            // Receive All
            w.ra().set_bit()
                // Promiscuous mode
                .pm().set_bit()
        });
        // Flow Control Register
        self.eth_mac.macfcr.modify(|_, w| {
            // Pause time
            w.pt().bits(0x100)
        });
        // operation mode register
        self.eth_dma.dmaomr.modify(|_, w| {
            // Dropping of TCP/IP checksum error frames disable
            w.dtcefd().set_bit()
                // Receive store and forward
                .rsf().set_bit()
                // Disable flushing of received frames
                .dfrf().set_bit()
                // Transmit store and forward
                .tsf().set_bit()
                // Forward error frames
                .fef().set_bit()
                // Operate on second frame
                .osf().set_bit()
        });
        // bus mode register
        self.eth_dma.dmabmr.modify(|_, w| unsafe {
            // Address-aligned beats
            w.aab().set_bit()
                // Fixed burst
                .fb().set_bit()
                // Rx DMA PBL
                .rdp().bits(32)
                // Programmable burst length
                .pbl().bits(32)
                // Rx Tx priority ratio 2:1
                .pm().bits(0b01)
                // Use separate PBL
                .usp().set_bit()
        });

        self
    }

    /// reset DMA bus mode register
    fn reset_dma_and_wait(&self) {
        self.eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

        // Wait until done
        while self.eth_dma.dmabmr.read().sr().bit_is_set() {}
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html) to
    /// clear interrupt pending bits. Otherwise the interrupt will
    /// reoccur immediately.
    pub fn enable_interrupt(&self) {
        self.eth_dma.dmaier.modify(|_, w|
            w
                // Normal interrupt summary enable
                .nise().set_bit()
                // Receive Interrupt Enable
                .rie().set_bit()
                // Transmit Interrupt Enable
                .tie().set_bit()
        );

        // Enable ethernet interrupts
        unsafe {
            NVIC::unmask(Interrupt::ETH);
        }
    }

    /// Calls [`eth_interrupt_handler()`](fn.eth_interrupt_handler.html)
    pub fn interrupt_handler(&self) {
        eth_interrupt_handler(&self.eth_dma);
    }

    /// Construct a PHY driver
    pub fn get_phy<'a>(&'a self) -> Phy<'a> {
        Phy::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, PHY_ADDR)
    }

    /// Obtain PHY status
    pub fn status(&self) -> PhyStatus {
        self.get_phy().status()
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
    pub fn send<F: FnOnce(&mut [u8]) -> R, R>(&mut self, length: usize, f: F) -> Result<R, TxError> {
        let result = self.tx_ring.send(length, f);
        //Wait for the memory to sync with the cache
        asm::dsb();
        self.tx_ring.demand_poll(&self.eth_dma);
        result
    }
}

/// Call in interrupt handler to clear interrupt reason, when
/// [`enable_interrupt()`](struct.Eth.html#method.enable_interrupt).
///
/// There are two ways to call this:
///
/// * Via the [`Eth`](struct.Eth.html) driver instance that your interrupt handler has access to.
/// * By unsafely getting `Peripherals`.
///
/// TODO: could return interrupt reason
pub fn eth_interrupt_handler(eth_dma: &ETHERNET_DMA) {
    eth_dma.dmasr.write(|w|
        w
        .nis().set_bit()
        .rs().set_bit()
        .ts().set_bit()
    );
}
