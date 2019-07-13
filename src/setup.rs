use stm32f2xx_hal::stm32::{RCC, SYSCFG};
#[cfg(feature = "nucleo-f429zi")]
use stm32f2xx_hal::gpio::{
    gpioa::{PA1, PA2, PA7},
    gpiob::{PB13},
    gpioc::{PC1, PC4, PC5},
    gpiog::{PG11, PG13},
    Speed::VeryHigh,
};

#[cfg(feature = "f2-custom")]
use stm32f2xx_hal::gpio::{
    gpioa::{PA1, PA2, PA7},
    gpiob::{PB13, PB11, PB12},
    gpioc::{PC1, PC4, PC5},
    // gpiog::{PG11, PG13},
    Speed::VeryHigh,
};

/// Initialize GPIO pins. Enable syscfg and ethernet clocks. Reset the
/// Ethernet MAC.
///
/// If supported, you should also call `setup_pins()`.
pub fn setup(rcc: &RCC, syscfg: &SYSCFG) {
    // enable syscfg clock
    rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    // select MII or RMII mode
    // 0 = MII, 1 = RMII
    syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

    // enable ethernet clocks
    rcc.ahb1enr.modify(|_, w| {
        w.ethmacen()  .set_bit()
         .ethmactxen().set_bit()
         .ethmacrxen().set_bit()
    });

    reset_pulse(&rcc);
}

fn reset_pulse(rcc: &RCC) {
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
    rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
}

/// Pin setup for the **STM32 Nucleo-F429ZI** dev board
/// (feature: `nucleo-f429zi`)
///
/// Set RMII pins to
/// * Alternate function 11
/// * High-speed
///
/// This function consumes the pins so that you cannot use them
/// anywhere else by accident.
#[cfg(feature = "nucleo-f429zi")]
pub fn setup_pins<M1, M2, M3, M4, M5, M6, M7, M8, M9>(
    pa1: PA1<M1>, pa2: PA2<M2>, pa7: PA7<M3>, pb13: PB13<M4>, pc1: PC1<M5>,
    pc4: PC4<M6>, pc5: PC5<M7>, pg11: PG11<M8>, pg13: PG13<M9>
) {
    // PA1 RMII Reference Clock - SB13 ON
    pa1.into_alternate_af11().set_speed(VeryHigh);
    // PA2 RMII MDIO - SB160 ON
    pa2.into_alternate_af11().set_speed(VeryHigh);
    // PC1 RMII MDC - SB164 ON
    pc1.into_alternate_af11().set_speed(VeryHigh);
    // PA7 RMII RX Data Valid D11 JP6 ON
    pa7.into_alternate_af11().set_speed(VeryHigh);
    // PC4 RMII RXD0 - SB178 ON
    pc4.into_alternate_af11().set_speed(VeryHigh);
    // PC5 RMII RXD1 - SB181 ON
    pc5.into_alternate_af11().set_speed(VeryHigh);
    // PG11 RMII TX Enable - SB183 ON
    pg11.into_alternate_af11().set_speed(VeryHigh);
    // PG13 RXII TXD0 - SB182 ON
    pg13.into_alternate_af11().set_speed(VeryHigh);
    // PB13 RMII TXD1 I2S_A_CK JP7 ON
    pb13.into_alternate_af11().set_speed(VeryHigh);
}

/// Pin setup for the **A custom** dev board
/// (feature: `f2-custom`)
///
/// Set RMII pins to
/// * Alternate function 11
/// * High-speed
///
/// This function consumes the pins so that you cannot use them
/// anywhere else by accident.
#[cfg(feature = "f2-custom")]
pub fn setup_pins<M1, M2, M3, M4, M5, M6, M7, M8, M9>(
    pa1: PA1<M1>, pa2: PA2<M2>, pa7: PA7<M3>, pb13: PB13<M4>, pc1: PC1<M5>,
    pc4: PC4<M6>, pc5: PC5<M7>, pb11: PB11<M8>, pb12: PB12<M9>
) {
    // PA1 RMII Reference Clock - SB13 ON
    pa1.into_alternate_af11().set_speed(VeryHigh);
    // PA2 RMII MDIO - SB160 ON
    pa2.into_alternate_af11().set_speed(VeryHigh);
    // PC1 RMII MDC - SB164 ON
    pc1.into_alternate_af11().set_speed(VeryHigh);
    // PA7 RMII RX Data Valid D11 JP6 ON
    pa7.into_alternate_af11().set_speed(VeryHigh);
    // PC4 RMII RXD0 - SB178 ON
    pc4.into_alternate_af11().set_speed(VeryHigh);
    // PC5 RMII RXD1 - SB181 ON
    pc5.into_alternate_af11().set_speed(VeryHigh);
    // PB11 RMII TX Enable - SB183 ON
    pb11.into_alternate_af11().set_speed(VeryHigh);
    // PG13 RXII TXD0 - SB182 ON
    pb12.into_alternate_af11().set_speed(VeryHigh);
    // PB13 RMII TXD1 I2S_A_CK JP7 ON
    pb13.into_alternate_af11().set_speed(VeryHigh);
}
