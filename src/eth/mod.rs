use cortex_m::interrupt::CriticalSection;
use stm32f429x::*;

// For debug output
use core::fmt::Write;
use cortex_m_semihosting::hio;


mod phy;
use self::phy::{Phy, PhyStatus};
mod smi;


pub struct Eth {
}

// impl Eth<'static> {
//     pub unsafe fn new() -> Self {
//         Eth {
//         }
//     }
// }
    

impl Eth {
    pub fn new() -> Self {
        Eth {
        }
    }

    pub fn init<'cs>(&self, cs: &'cs CriticalSection) -> &Self {
        let mut stdout = hio::hstdout().unwrap();

        let rcc = RCC.borrow(cs);
        // enable syscfg clock
        rcc.apb2enr.write(|w| w.syscfgen().enabled());

        Self::init_pins(cs);
        Self::reset_pulse(cs);

        // enable ethernet clocks
        rcc.ahb1enr.modify(|_, w| {
            w.ethmacen().enabled()
                .ethmactxen().enabled()
                .ethmacrxen().enabled()
        });

        let syscfg = SYSCFG.borrow(cs);
        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        syscfg.pmc.modify(|_, w| unsafe { w.mii_rmii_sel().bits(1) });

        Self::reset_dma_and_wait(cs);

        // set clock range in MAC MII address register
        let eth_mac = ETHERNET_MAC.borrow(cs);
        let clock_range = ETH_MACMIIAR_CR_HCLK_DIV_16;
        eth_mac.macmiiar.write(|w| unsafe { w.cr().bits(clock_range) });

        Phy::with(cs, 0)
            .reset()
            .set_autoneg();

        self
    }

    fn reset_pulse<'cs>(cs: &'cs CriticalSection) {
        let rcc = RCC.borrow(cs);
        rcc.ahb1rstr.modify(|_, w| unsafe { w.ethmacrst().bits(1) });
        rcc.ahb1rstr.modify(|_, w| unsafe { w.ethmacrst().bits(0) });
    }

    /// reset DMA bus mode register
    fn reset_dma_and_wait<'cs>(cs: &'cs CriticalSection) {
        let eth_dma = ETHERNET_DMA.borrow(cs);

        eth_dma.dmabmr.modify(|_, w| unsafe { w.sr().bits(1) });

        // Wait until done
        while eth_dma.dmabmr.read().sr().bits() != 0 {}
    }
    
    /// Set RMII pins to
    /// * Alternate function mode
    /// * Push-pull mode
    /// * No pull-up resistor
    /// * High-speed
    /// * Alternate function 11
    fn init_pins<'cs>(cs: &'cs CriticalSection) {
        let rcc = RCC.borrow(cs);
        rcc.ahb1enr.modify(|_, w| {
            w.gpioaen().enabled()
                .gpioben().enabled()
                .gpiocen().enabled()
                .gpiogen().enabled()
        });
        
        let gpioa = GPIOA.borrow(cs);
        let gpiob = GPIOB.borrow(cs);
        let gpioc = GPIOC.borrow(cs);
        let gpiog = GPIOG.borrow(cs);
        // PA1 RMII Reference Clock - SB13 ON
        gpioa.moder.modify(|_, w| w.moder1().afm());
        gpioa.otyper.modify(|_, w| w.ot1().pushpull());
        gpioa.pupdr.modify(|_, w| w.pupdr1().none());
        gpioa.ospeedr.modify(|_, w| w.ospeedr1().high());
        gpioa.afrl.modify(|_, w| unsafe { w.afrl1().bits(11) });
        // PA2 RMII MDIO - SB160 ON
        gpioa.moder.modify(|_, w| w.moder2().afm());
        gpioa.otyper.modify(|_, w| w.ot2().pushpull());
        gpioa.pupdr.modify(|_, w| w.pupdr2().none());
        gpioa.ospeedr.modify(|_, w| w.ospeedr2().high());
        gpioa.afrl.modify(|_, w| unsafe { w.afrl2().bits(11) });
        // PC1 RMII MDC - SB164 ON
        gpioc.moder.modify(|_, w| w.moder1().afm());
        gpioc.otyper.modify(|_, w| w.ot1().pushpull());
        gpioc.pupdr.modify(|_, w| w.pupdr1().none());
        gpioc.ospeedr.modify(|_, w| w.ospeedr1().high());
        gpioc.afrl.modify(|_, w| unsafe { w.afrl1().bits(11) });
        // PA7 RMII RX Data Valid D11 JP6 ON
        gpioa.moder.modify(|_, w| w.moder7().afm());
        gpioa.otyper.modify(|_, w| w.ot7().pushpull());
        gpioa.pupdr.modify(|_, w| w.pupdr7().none());
        gpioa.ospeedr.modify(|_, w| w.ospeedr7().high());
        gpioa.afrl.modify(|_, w| unsafe { w.afrl7().bits(11) });
        // PC4 RMII RXD0 - SB178 ON
        gpioc.moder.modify(|_, w| w.moder4().afm());
        gpioc.otyper.modify(|_, w| w.ot4().pushpull());
        gpioc.pupdr.modify(|_, w| w.pupdr4().none());
        gpioc.ospeedr.modify(|_, w| w.ospeedr4().high());
        gpioc.afrl.modify(|_, w| unsafe { w.afrl4().bits(11) });
        // PC5 RMII RXD1 - SB181 ON
        gpioc.moder.modify(|_, w| w.moder5().afm());
        gpioc.otyper.modify(|_, w| w.ot5().pushpull());
        gpioc.pupdr.modify(|_, w| w.pupdr5().none());
        gpioc.ospeedr.modify(|_, w| w.ospeedr5().high());
        gpioc.afrl.modify(|_, w| unsafe { w.afrl5().bits(11) });
        // PG11 RMII TX Enable - SB183 ON
        gpiog.moder.modify(|_, w| w.moder11().afm());
        gpiog.otyper.modify(|_, w| w.ot11().pushpull());
        gpiog.pupdr.modify(|_, w| w.pupdr11().none());
        gpiog.ospeedr.modify(|_, w| w.ospeedr11().high());
        gpiog.afrh.modify(|_, w| unsafe { w.afrh11().bits(11) });
        // PG13 RXII TXD0 - SB182 ON
        gpiog.moder.modify(|_, w| w.moder13().afm());
        gpiog.otyper.modify(|_, w| w.ot13().pushpull());
        gpiog.pupdr.modify(|_, w| w.pupdr13().none());
        gpiog.ospeedr.modify(|_, w| w.ospeedr13().high());
        gpiog.afrh.modify(|_, w| unsafe { w.afrh13().bits(11) });
        // PB13 RMII TXD1 I2S_A_CK JP7 ON
        gpiob.moder.modify(|_, w| w.moder13().afm());
        gpiob.otyper.modify(|_, w| w.ot13().pushpull());
        gpiob.pupdr.modify(|_, w| w.pupdr13().none());
        gpiob.ospeedr.modify(|_, w| w.ospeedr13().high());
        gpiob.afrh.modify(|_, w| unsafe { w.afrh13().bits(11) });
        // …or this one?
        // gpiog.moder.modify(|_, w| w.moder14().afm());
        // gpiog.otyper.modify(|_, w| w.ot14().pushpull());
        // gpiog.pupdr.modify(|_, w| w.pupdr14().none());
        // gpiog.ospeedr.modify(|_, w| w.ospeedr14().high());
        // gpiog.afrh.modify(|_, w| unsafe { w.afrh14().bits(11) });
    }

    pub fn status(&self) -> PhyStatus {
        let phy = unsafe { Phy::new(0) };
        phy.status()
    }
}

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
