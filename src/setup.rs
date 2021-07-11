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
    pac::{RCC, SYSCFG},
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

#[cfg(feature = "device-selected")]
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
