use cortex_m::interrupt;
use cortex_m::interrupt::CriticalSection;
use stm32f429x::*;

// For debug output
use core::fmt::Write;
use cortex_m_semihosting::hio;


mod phy;
use self::phy::{Phy, PhyStatus};
mod smi;
mod rx;
use self::rx::RxRing;
mod buffer;
pub use self::buffer::Buffer;

pub const ALIGNMENT: usize = 0b1000;
const PHY_ADDR: u8 = 0;
const MTU: usize = 1518;

#[allow(dead_code)]
mod consts {
    pub const GPIO_MODER_AFM: u8 = 0b10;
    pub const GPIO_OTYPER_PUSHPULL: bool = false;
    pub const GPIO_PUPDR_NONE: u8 = 0b00;
    pub const GPIO_OSPEEDR_HIGH: u8 = 0b10;

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

pub struct Eth {
    eth_mac: ETHERNET_MAC,
    eth_dma: ETHERNET_DMA,
    rx: RxRing,
}

impl Eth {
    pub fn new(eth_mac: ETHERNET_MAC, eth_dma: ETHERNET_DMA) -> Self {
        Eth {
            eth_mac,
            eth_dma,
            rx: RxRing::new(MTU),
        }
    }

    pub fn get_phy<'a>(&'a self) -> Phy<'a> {
        Phy::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, PHY_ADDR)
    }
    
    pub fn init<'cs>(&mut self, cs: &'cs CriticalSection, rcc: &RCC, syscfg: &SYSCFG, nvic: &mut NVIC) -> &Self {
        let mut stdout = hio::hstdout().unwrap();

        writeln!(stdout, "Enabling clocks").unwrap();
        // enable syscfg clock
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

        // enable ethernet clocks
        rcc.ahb1enr.modify(|_, w| {
            w.ethmacen().set_bit()
                .ethmactxen().set_bit()
                .ethmacrxen().set_bit()
        });
        writeln!(stdout, "Clocks enabled").unwrap();

        writeln!(stdout, "Resetting Ethernet").unwrap();
        Self::reset_pulse(cs, rcc);
        writeln!(stdout, "Ethernet reset").unwrap();
        self.reset_dma_and_wait(cs);

        self.eth_dma.dmaier.modify(|_, w|
            w
                // Normal interrupt summary enable
                .nise().set_bit()
                // Receive Interrupt Enable
                .rie().set_bit()
                // Transmit Interrupt Enable
                // .tie().set_bit()
        );
        // unsafe { nvic.set_priority(Interrupt::ETH, 0x10); }
        // Enable ethernet interrupts
        nvic.enable(Interrupt::ETH);

        // set clock range in MAC MII address register
        let clock_range = ETH_MACMIIAR_CR_HCLK_DIV_16;
        self.eth_mac.macmiiar.modify(|_, w| unsafe { w.cr().bits(clock_range) });

        writeln!(stdout, "Ethernet configured").unwrap();

        self.get_phy()
            .reset(cs)
            .set_autoneg();
        writeln!(stdout, "Phy reset").unwrap();

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
        self.eth_mac.macfcr.modify(|_, w| unsafe {
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
                .rtpr().bits(0b01)
            // Use separate PBL
                .usp().set_bit()
        });

        self
    }

    fn reset_pulse<'cs>(cs: &'cs CriticalSection, rcc: &RCC) {
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
    }

    /// reset DMA bus mode register
    fn reset_dma_and_wait<'cs>(&self, _: &'cs CriticalSection) {
        self.eth_dma.dmabmr.modify(|_, w| w.sr().set_bit());

        // Wait until done
        while self.eth_dma.dmabmr.read().sr().bit_is_set() {}
    }

    /// Set RMII pins to
    /// * Alternate function mode
    /// * Push-pull mode
    /// * No pull-up resistor
    /// * High-speed
    /// * Alternate function 11
    pub fn init_pins<'cs>(&self, cs: &'cs CriticalSection, rcc: &RCC, gpioa: &GPIOA, gpiob: &GPIOB, gpioc: &GPIOC, gpiog: &GPIOG) -> &Self {
        rcc.ahb1enr.modify(|_, w| {
            w.gpioaen().set_bit()
                .gpioben().set_bit()
                .gpiocen().set_bit()
                .gpiogen().set_bit()
        });

        // PA1 RMII Reference Clock - SB13 ON
        gpioa.moder.modify(|_, w| unsafe { w.moder1().bits(GPIO_MODER_AFM) });
        gpioa.otyper.modify(|_, w| w.ot1().bit(GPIO_OTYPER_PUSHPULL));
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr1().bits(GPIO_PUPDR_NONE) });
        gpioa.ospeedr.modify(|_, w| unsafe { w.ospeedr1().bits(GPIO_OSPEEDR_HIGH) });
        gpioa.afrl.modify(|_, w| unsafe { w.afrl1().bits(11) });
        // PA2 RMII MDIO - SB160 ON
        gpioa.moder.modify(|_, w| unsafe { w.moder2().bits(GPIO_MODER_AFM) });
        gpioa.otyper.modify(|_, w| w.ot2().bit(GPIO_OTYPER_PUSHPULL));
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr2().bits(GPIO_PUPDR_NONE) });
        gpioa.ospeedr.modify(|_, w| unsafe { w.ospeedr2().bits(GPIO_OSPEEDR_HIGH) });
        gpioa.afrl.modify(|_, w| unsafe { w.afrl2().bits(11) });
        // PC1 RMII MDC - SB164 ON
        gpioc.moder.modify(|_, w| unsafe { w.moder1().bits(GPIO_MODER_AFM) });
        gpioc.otyper.modify(|_, w| w.ot1().bit(GPIO_OTYPER_PUSHPULL));
        gpioc.pupdr.modify(|_, w| unsafe { w.pupdr1().bits(GPIO_PUPDR_NONE) });
        gpioc.ospeedr.modify(|_, w| unsafe { w.ospeedr1().bits(GPIO_OSPEEDR_HIGH) });
        gpioc.afrl.modify(|_, w| unsafe { w.afrl1().bits(11) });
        // PA7 RMII RX Data Valid D11 JP6 ON
        gpioa.moder.modify(|_, w| unsafe { w.moder7().bits(GPIO_MODER_AFM) });
        gpioa.otyper.modify(|_, w| w.ot7().bit(GPIO_OTYPER_PUSHPULL));
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr7().bits(GPIO_PUPDR_NONE) });
        gpioa.ospeedr.modify(|_, w| unsafe { w.ospeedr7().bits(GPIO_OSPEEDR_HIGH) });
        gpioa.afrl.modify(|_, w| unsafe { w.afrl7().bits(11) });
        // PC4 RMII RXD0 - SB178 ON
        gpioc.moder.modify(|_, w| unsafe { w.moder4().bits(GPIO_MODER_AFM) });
        gpioc.otyper.modify(|_, w| w.ot4().bit(GPIO_OTYPER_PUSHPULL));
        gpioc.pupdr.modify(|_, w| unsafe { w.pupdr4().bits(GPIO_PUPDR_NONE) });
        gpioc.ospeedr.modify(|_, w| unsafe { w.ospeedr4().bits(GPIO_OSPEEDR_HIGH) });
        gpioc.afrl.modify(|_, w| unsafe { w.afrl4().bits(11) });
        // PC5 RMII RXD1 - SB181 ON
        gpioc.moder.modify(|_, w| unsafe { w.moder5().bits(GPIO_MODER_AFM) });
        gpioc.otyper.modify(|_, w| w.ot5().bit(GPIO_OTYPER_PUSHPULL));
        gpioc.pupdr.modify(|_, w| unsafe { w.pupdr5().bits(GPIO_PUPDR_NONE) });
        gpioc.ospeedr.modify(|_, w| unsafe { w.ospeedr5().bits(GPIO_OSPEEDR_HIGH) });
        gpioc.afrl.modify(|_, w| unsafe { w.afrl5().bits(11) });
        // PG11 RMII TX Enable - SB183 ON
        gpiog.moder.modify(|_, w| unsafe { w.moder11().bits(GPIO_MODER_AFM) });
        gpiog.otyper.modify(|_, w| w.ot11().bit(GPIO_OTYPER_PUSHPULL));
        gpiog.pupdr.modify(|_, w| unsafe { w.pupdr11().bits(GPIO_PUPDR_NONE) });
        gpiog.ospeedr.modify(|_, w| unsafe { w.ospeedr11().bits(GPIO_OSPEEDR_HIGH) });
        gpiog.afrh.modify(|_, w| unsafe { w.afrh11().bits(11) });
        // PG13 RXII TXD0 - SB182 ON
        gpiog.moder.modify(|_, w| unsafe { w.moder13().bits(GPIO_MODER_AFM) });
        gpiog.otyper.modify(|_, w| w.ot13().bit(GPIO_OTYPER_PUSHPULL));
        gpiog.pupdr.modify(|_, w| unsafe { w.pupdr13().bits(GPIO_PUPDR_NONE) });
        gpiog.ospeedr.modify(|_, w| unsafe { w.ospeedr13().bits(GPIO_OSPEEDR_HIGH) });
        gpiog.afrh.modify(|_, w| unsafe { w.afrh13().bits(11) });
        // PB13 RMII TXD1 I2S_A_CK JP7 ON
        gpiob.moder.modify(|_, w| unsafe { w.moder13().bits(GPIO_MODER_AFM) });
        gpiob.otyper.modify(|_, w| w.ot13().bit(GPIO_OTYPER_PUSHPULL));
        gpiob.pupdr.modify(|_, w| unsafe { w.pupdr13().bits(GPIO_PUPDR_NONE) });
        gpiob.ospeedr.modify(|_, w| unsafe { w.ospeedr13().bits(GPIO_OSPEEDR_HIGH) });
        gpiob.afrh.modify(|_, w| unsafe { w.afrh13().bits(11) });
        // …or this one?
        // gpiog.moder.modify(|_, w| unsafe { w.moder14().bits(GPIO_MODER_AFM) });
        // gpiog.otyper.modify(|_, w| w.ot14().bit(GPIO_OTYPER_PUSHPULL));
        // gpiog.pupdr.modify(|_, w| unsafe { w.pupdr14().bits(GPIO_PUPDR_NONE) });
        // gpiog.ospeedr.modify(|_, w| unsafe { w.ospeedr14().bits(GPIO_OSPEEDR_HIGH) });
        // gpiog.afrh.modify(|_, w| unsafe { w.afrh14().bits(11) });

        self
    }

    pub fn status(&self) -> PhyStatus {
        self.get_phy().status()
    }


    pub fn start_rx(&mut self, ring_length: usize) -> &mut Self {
        self.rx.start(ring_length, &self.eth_dma);

        self
    }

    pub fn rx_is_running(&self) -> bool {
        self.rx.running_state(&self.eth_dma).is_running()
    }

    pub fn recv_next(&mut self) -> Option<Buffer> {
        self.rx.recv_next(&self.eth_dma)
    }
}
