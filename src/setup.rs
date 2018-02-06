use board::*;

#[allow(dead_code)]
mod consts {
    pub const GPIO_MODER_AFM: u8 = 0b10;
    pub const GPIO_OTYPER_PUSHPULL: bool = false;
    pub const GPIO_PUPDR_NONE: u8 = 0b00;
    pub const GPIO_OSPEEDR_HIGH: u8 = 0b10;
}
use self::consts::*;

pub fn setup(p: &Peripherals) {
    init_pins(&p.RCC, &p.GPIOA, &p.GPIOB, &p.GPIOC, &p.GPIOG);
    
    // enable syscfg clock
    p.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    // select MII or RMII mode
    // 0 = MII, 1 = RMII
    p.SYSCFG.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

    // enable ethernet clocks
    p.RCC.ahb1enr.modify(|_, w| {
        w.ethmacen().set_bit()
            .ethmactxen().set_bit()
            .ethmacrxen().set_bit()
    });

    reset_pulse(&p.RCC);
}

fn reset_pulse(rcc: &RCC) {
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
}

/// Set RMII pins to
/// * Alternate function mode
/// * Push-pull mode
/// * No pull-up resistor
/// * High-speed
/// * Alternate function 11
pub fn init_pins(rcc: &RCC, gpioa: &GPIOA, gpiob: &GPIOB, gpioc: &GPIOC, gpiog: &GPIOG) {
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
}
