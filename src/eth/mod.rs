use cortex_m::interrupt::CriticalSection;
use stm32f429x::*;

// For debug output
use core::fmt::Write;
use cortex_m_semihosting::hio;


mod phy;
use self::phy::{Phy, PhyStatus};
mod smi;

const PHY_ADDR: u8 = 0;

pub struct Eth {
    eth_mac: ETHERNET_MAC,
    eth_dma: ETHERNET_DMA,
}

impl Eth {
    pub fn new(eth_mac: ETHERNET_MAC, eth_dma: ETHERNET_DMA) -> Self {
        Eth {
            eth_mac,
            eth_dma,
        }
    }

    pub fn get_phy<'a>(&'a self) -> Phy<'a> {
        Phy::new(&self.eth_mac.macmiiar, &self.eth_mac.macmiidr, PHY_ADDR)
    }
    
    pub fn init<'cs>(&self, cs: &'cs CriticalSection, rcc: &RCC, syscfg: &SYSCFG) -> &Self {
        let mut stdout = hio::hstdout().unwrap();

        writeln!(stdout, "Enabling clocks").unwrap();
        // enable syscfg clock
        rcc.apb2enr.write(|w| w.syscfgen().set_bit());

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
        // self.reset_dma_and_wait(cs);
        writeln!(stdout, "Ethernet reset").unwrap();

        // set clock range in MAC MII address register
        let clock_range = ETH_MACMIIAR_CR_HCLK_DIV_16;
        self.eth_mac.macmiiar.modify(|_, w| unsafe { w.cr().bits(clock_range) });

        writeln!(stdout, "Ethernet configured").unwrap();

        self.get_phy()
            .reset(cs)
            .set_autoneg();
        writeln!(stdout, "Phy reset").unwrap();

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
    pub fn init_pins<'cs>(&self, cs: &'cs CriticalSection, rcc: &RCC, gpioa: &GPIOA, gpiob: &GPIOB, gpioc: &GPIOC, gpiog: &GPIOG) {
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
    }

    pub fn status(&self) -> PhyStatus {
        self.get_phy().status()
    }
}

const GPIO_MODER_AFM: u8 = 0b10;
const GPIO_OTYPER_PUSHPULL: bool = false;
const GPIO_PUPDR_NONE: u8 = 0b00;
const GPIO_OSPEEDR_HIGH: u8 = 0b10;

/* For HCLK 60-100 MHz */
const ETH_MACMIIAR_CR_HCLK_DIV_42: u8 = 0;
/* For HCLK 100-150 MHz */
const ETH_MACMIIAR_CR_HCLK_DIV_62: u8 = 1;
/* For HCLK 20-35 MHz */
const ETH_MACMIIAR_CR_HCLK_DIV_16: u8 = 2;
/* For HCLK 35-60 MHz */
const ETH_MACMIIAR_CR_HCLK_DIV_26: u8 = 3;
/* For HCLK 150-168 MHz */
const ETH_MACMIIAR_CR_HCLK_DIV_102: u8 = 4;
