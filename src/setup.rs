#[cfg(feature = "stm32f4xx-hal")]
use stm32f4xx_hal::{
    bb,
    gpio::{
        gpioa::{PA1, PA2, PA7},
        gpiob::{PB11, PB12, PB13},
        gpioc::{PC1, PC4, PC5},
        gpiog::{PG11, PG13, PG14},
        Floating, Input,
        Speed::VeryHigh,
    },
    stm32::{RCC, SYSCFG},
};

#[cfg(feature = "stm32f7xx-hal")]
use cortex_m::interrupt;
#[cfg(feature = "stm32f7xx-hal")]
use stm32f7xx_hal::{
    device::{RCC, SYSCFG},
    gpio::{
        gpioa::{PA1, PA2, PA7},
        gpiob::{PB11, PB12, PB13},
        gpioc::{PC1, PC4, PC5},
        gpiog::{PG11, PG13, PG14},
        Floating, Input,
        Speed::VeryHigh,
    },
};

// Enable syscfg and ethernet clocks. Reset the Ethernet MAC.
pub(crate) fn setup() {
    #[cfg(feature = "stm32f4xx-hal")]
    unsafe {
        const SYSCFG_BIT: u8 = 14;
        const ETH_MAC_BIT: u8 = 25;
        const ETH_TX_BIT: u8 = 26;
        const ETH_RX_BIT: u8 = 27;
        const MII_RMII_BIT: u8 = 23;

        //NOTE(unsafe) This will only be used for atomic writes with no side-effects
        let rcc = &*RCC::ptr();
        let syscfg = &*SYSCFG::ptr();

        // Enable syscfg clock
        bb::set(&rcc.apb2enr, SYSCFG_BIT);

        if rcc.ahb1enr.read().ethmacen().bit_is_set() {
            // pmc must be changed with the ethernet controller disabled or under reset
            bb::clear(&rcc.ahb1enr, ETH_MAC_BIT);
        }
        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        bb::set(&syscfg.pmc, MII_RMII_BIT);

        // enable ethernet clocks
        bb::set(&rcc.ahb1enr, ETH_MAC_BIT);
        bb::set(&rcc.ahb1enr, ETH_TX_BIT);
        bb::set(&rcc.ahb1enr, ETH_RX_BIT);

        // reset pulse
        bb::set(&rcc.ahb1rstr, ETH_MAC_BIT);
        bb::clear(&rcc.ahb1rstr, ETH_MAC_BIT);
    }
    #[cfg(feature = "stm32f7xx-hal")]
    //stm32f7xx-hal does not currently have bitbanding
    interrupt::free(|_| unsafe {
        //NOTE(unsafe) Interrupt free and we only modify mac bits
        let rcc = &*RCC::ptr();
        let syscfg = &*SYSCFG::ptr();
        // enable syscfg clock
        rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

        if rcc.ahb1enr.read().ethmacen().bit_is_set() {
            // pmc must be changed with the ethernet controller disabled or under reset
            rcc.ahb1enr.modify(|_, w| w.ethmacen().clear_bit());
        }

        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        syscfg.pmc.modify(|_, w| w.mii_rmii_sel().set_bit());

        // enable ethernet clocks
        rcc.ahb1enr.modify(|_, w| {
            w.ethmacen()
                .set_bit()
                .ethmactxen()
                .set_bit()
                .ethmacrxen()
                .set_bit()
        });

        //reset pulse
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
    });

    #[cfg(feature = "stm32f1xx-hal")]
    cortex_m::interrupt::free(|_| unsafe {
        let afio = &*crate::stm32::AFIO::ptr();
        let rcc = &*crate::stm32::RCC::ptr();

        // enable AFIO clock
        rcc.apb2enr.modify(|_, w| w.afioen().set_bit());

        if rcc.ahbenr.read().ethmacen().bit_is_set() {
            // ethernet controller must be disabled when configuring mapr
            rcc.ahbenr.modify(|_, w| w.ethmacen().clear_bit());
        }

        // select MII or RMII mode
        // 0 = MII, 1 = RMII
        afio.mapr.modify(|_, w| w.mii_rmii_sel().set_bit());

        // enable ethernet clocks
        rcc.ahbenr.modify(|_, w| {
            w.ethmacen()
                .set_bit()
                .ethmactxen()
                .set_bit()
                .ethmacrxen()
                .set_bit()
        });

        // Reset pulse.
        rcc.ahbrstr.modify(|_, w| w.ethmacrst().set_bit());
        rcc.ahbrstr.modify(|_, w| w.ethmacrst().clear_bit());

        // Workaround for the issue mentioned in the Errata (2.20.11) related to wfi:
        //
        // "
        // If a WFI/WFE instruction is executed to put the system in sleep mode while the Ethernet
        // MAC master clock on the AHB bus matrix is ON and all remaining masters clocks are OFF,
        // the Ethernet DMA is unable to perform any AHB master accesses during sleep mode.
        //
        // Workaround: Enable DMA1 or DMA2 clocks in the RCC_AHBENR register before executing the
        // WFI/WFE instruction.
        // "
        if rcc.ahbenr.read().dma1en().is_disabled() && rcc.ahbenr.read().dma2en().is_disabled() {
            rcc.ahbenr.modify(|_, w| w.dma2en().enabled());
            while rcc.ahbenr.read().dma2en().is_disabled() {}
        }
    });

}

/// RMII Reference Clock.
pub unsafe trait RmiiRefClk {}

/// RMII MDIO.
pub unsafe trait MDIO {}

/// RMII MDC.
pub unsafe trait MDC {}

/// RMII RX Data Valid.
pub unsafe trait RmiiCrsDv {}

/// RMII TX Enable.
pub unsafe trait RmiiTxEN {}

/// RMII TXD0.
pub unsafe trait RmiiTxD0 {}

/// RMII TXD1.
pub unsafe trait RmiiTxD1 {}

/// RMII RXD0.
pub unsafe trait RmiiRxD0 {}

/// RMII RXD1.
pub unsafe trait RmiiRxD1 {}

/// Trait needed to setup the pins for the Ethernet peripheral.
pub trait AlternateVeryHighSpeed {
    /// Puts the pin in the Alternate Function 11 with Very High Speed.
    fn into_af11_very_high_speed(self);
}

pub struct EthPins<REFCLK, IO, CLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1> {
    pub ref_clk: REFCLK,
    pub md_io: IO,
    pub md_clk: CLK,
    pub crs: CRS,
    pub tx_en: TXEN,
    pub tx_d0: TXD0,
    pub tx_d1: TXD1,
    pub rx_d0: RXD0,
    pub rx_d1: RXD1,
}

impl<REFCLK, IO, CLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>
    EthPins<REFCLK, IO, CLK, CRS, TXEN, TXD0, TXD1, RXD0, RXD1>
where
    REFCLK: RmiiRefClk + AlternateVeryHighSpeed,
    IO: MDIO + AlternateVeryHighSpeed,
    CLK: MDC + AlternateVeryHighSpeed,
    CRS: RmiiCrsDv + AlternateVeryHighSpeed,
    TXEN: RmiiTxEN + AlternateVeryHighSpeed,
    TXD0: RmiiTxD0 + AlternateVeryHighSpeed,
    TXD1: RmiiTxD1 + AlternateVeryHighSpeed,
    RXD0: RmiiRxD0 + AlternateVeryHighSpeed,
    RXD1: RmiiRxD1 + AlternateVeryHighSpeed,
{
    /// Pin setup.
    ///
    /// Set RMII pins to
    /// * Alternate function 11
    /// * High-speed
    ///
    /// This function consumes the pins so that you cannot use them
    /// anywhere else by accident.
    pub fn setup_pins(self) {
        self.ref_clk.into_af11_very_high_speed();
        self.md_io.into_af11_very_high_speed();
        self.md_clk.into_af11_very_high_speed();
        self.crs.into_af11_very_high_speed();
        self.tx_en.into_af11_very_high_speed();
        self.tx_d0.into_af11_very_high_speed();
        self.tx_d1.into_af11_very_high_speed();
        self.rx_d0.into_af11_very_high_speed();
        self.rx_d1.into_af11_very_high_speed();
    }
}

#[allow(unused_macros)]
macro_rules! impl_pins {
    ( $($traity:ident: [$($pin:ty,)+],)+ ) => {
        $(
            $(
                unsafe impl $traity for $pin {}

                impl AlternateVeryHighSpeed for $pin {
                    fn into_af11_very_high_speed(self) {
                        self.into_alternate_af11().set_speed(VeryHigh);
                    }
                }
            )+
        )+
    };
}

#[cfg(all(feature = "device-selected", any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal")))]
impl_pins!(
    RmiiRefClk: [
        PA1<Input<Floating>>,
    ],
    MDIO: [
        PA2<Input<Floating>>,
    ],
    MDC: [
        PC1<Input<Floating>>,
    ],
    RmiiCrsDv: [
        PA7<Input<Floating>>,
    ],
    RmiiTxEN: [
        PB11<Input<Floating>>,
        PG11<Input<Floating>>,
    ],
    RmiiTxD0: [
        PB12<Input<Floating>>,
        PG13<Input<Floating>>,
    ],
    RmiiTxD1: [
        PB13<Input<Floating>>,
        PG14<Input<Floating>>,
    ],
    RmiiRxD0: [
        PC4<Input<Floating>>,
    ],
    RmiiRxD1: [
        PC5<Input<Floating>>,
    ],
);

#[cfg(feature = "stm32f1xx-hal")]
mod stm32f1 {
    use super::*;
    use stm32f1xx_hal::gpio::{
        gpioa, gpiob, gpioc, gpiod, Alternate, Floating, Input, PushPull,
    };

    // STM32F1xx's require access to the CRL/CRH registers to change pin mode. As a result, we
    // require that pins are already in the necessary mode before constructing `EthPins` as it
    // would be inconvenient to pass CRL and CRH through to the `AlternateVeryHighSpeed` callsite.

    type PA1 = gpioa::PA1<Input<Floating>>;
    type PA2 = gpioa::PA2<Alternate<PushPull>>;
    type PA7 = gpioa::PA7<Input<Floating>>;
    type PB11 = gpiob::PB11<Alternate<PushPull>>;
    type PB12 = gpiob::PB12<Alternate<PushPull>>;
    type PB13 = gpiob::PB13<Alternate<PushPull>>;
    type PC1 = gpioc::PC1<Alternate<PushPull>>;
    type PC4 = gpioc::PC4<Input<Floating>>;
    type PC5 = gpioc::PC5<Input<Floating>>;
    type PD8 = gpiod::PD8<Input<Floating>>;
    type PD9 = gpiod::PD9<Input<Floating>>;
    type PD10 = gpiod::PD10<Input<Floating>>;

    unsafe impl RmiiRefClk for PA1 {}
    unsafe impl MDIO for PA2 {}
    unsafe impl MDC for PC1 {}
    unsafe impl RmiiCrsDv for PA7 {}
    unsafe impl RmiiCrsDv for PD8 {}
    unsafe impl RmiiTxEN for PB11 {}
    unsafe impl RmiiTxD0 for PB12 {}
    unsafe impl RmiiTxD1 for PB13 {}
    unsafe impl RmiiRxD0 for PC4 {}
    unsafe impl RmiiRxD0 for PD9 {}
    unsafe impl RmiiRxD1 for PC5 {}
    unsafe impl RmiiRxD1 for PD10 {}

    macro_rules! impl_alt_very_high_speed {
        ($($PIN:ident),*) => {
            $(
                impl AlternateVeryHighSpeed for $PIN {
                    fn into_af11_very_high_speed(self) {}
                }
            )*
        }
    }

    impl_alt_very_high_speed!(PA1, PA2, PA7, PB11, PB12, PB13, PC1, PC4, PC5, PD8, PD9, PD10);
}
