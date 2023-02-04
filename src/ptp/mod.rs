//! PTP access and configuration.
//!
//! See [`EthernetPTP`] for a more details.

use crate::{dma::EthernetDMA, Clocks};

#[cfg(feature = "f-series")]
use crate::{mac::EthernetMAC, peripherals::ETHERNET_PTP};

mod timestamp;
pub use timestamp::Timestamp;

mod subseconds;
pub use subseconds::{Subseconds, NANOS_PER_SECOND, SUBSECONDS_PER_SECOND, SUBSECONDS_TO_SECONDS};

mod pps_pin;
pub use pps_pin::PPSPin;

/// Access to the IEEE 1508v2 PTP peripheral present on the ethernet peripheral.
///
/// On STM32FXXX's, the PTP peripheral has/uses the following important parts:
/// * HCLK (the chip's high speed clock, configured externally).
/// * The global timestamp (`global_time`, a [`Timestamp`]).
/// * A subsecond increment register (`subsecond_increment`, a [`Subseconds`] with a value of 0 to 255, see [`EthernetPTP::subsecond_increment`]).
/// * An accumulator register (`accumulator`, an [`u32`]).
/// * An addend register (`addend`, an [`u32`], see [`EthernetPTP::addend`] and [`EthernetPTP::set_addend`]).
///
/// To ensure that `global_time` advances at the correct rate, the system performs the following steps:
/// 1. On every clock of HCLK, `addend` is added to `accumulator`.
/// 2. If `accumulator` overflows during step 1, add `subsecond_increment` to `global_time`.
///
/// When a new [`EthernetPTP`] is created, it is assumed that the frequency of HCLK is exactly correct.
/// Using HCLK, values for `subsecond_increment` and `addend` are calculated so that `global_time` represents
/// real-time.
///
/// Subsequently, `addend` can be adjusted to compensate for possible errors in HCLK, using [`EthernetPTP::addend`] and [`EthernetPTP::set_addend`]
///
/// To assess the correctness of the current speed at which `global_time` is running, one can use the
/// following equation:
///
/// ```no_compile
/// clock_ratio = ((2^31 / subsecond_increment) / (HCLK_HZ * (addend / 2^32)))
/// ```
/// Values greater than 1 indicate that the provided `HCLK_HZ` is less than the actual frequency of HCLK, which should
/// be compensated by increasing `addend`. Values less than 1 indicate that the provided `HCLK_HZ` is greater than the
/// actual frequency of HCLK, which should be compensated by decreasing `addend`.
///
/// [`NonZeroU8`]: core::num::NonZeroU8
pub struct EthernetPTP {
    #[cfg(feature = "f-series")]
    eth_ptp: ETHERNET_PTP,
}

impl EthernetPTP {
    // Calculate the `addend` required for running `global_time` at
    // the correct rate
    const fn calculate_regs(hclk: u32) -> (Subseconds, u32) {
        let half_hclk = hclk / 2;

        // Calculate the closest `subsecond_increment` we can use if we want to update at a
        // frequency of `half_hclk`
        let stssi = Subseconds::nearest_increment(half_hclk);
        let half_rate_subsec_increment_hz = stssi.hertz();

        // Calculate the `addend` required for running `global_time` at
        // the correct rate, given that we increment `global_time` by `stssi` every
        // time `accumulator` overflows.
        let tsa = ((half_rate_subsec_increment_hz as u64 * u32::MAX as u64) / hclk as u64) as u32;
        (stssi, tsa)
    }

    #[cfg(feature = "stm32h7xx-hal")]
    /// # Safety
    /// The reference to the registerblock obtained using this function
    /// must _only_ be used to change strictly PTP related registers.
    unsafe fn mac(&self) -> &crate::stm32::ethernet_mac::RegisterBlock {
        &*crate::peripherals::ETHERNET_MAC::ptr()
    }

    pub(crate) fn new(
        #[cfg(feature = "f-series")] eth_ptp: ETHERNET_PTP,
        clocks: Clocks,
        // Note(_dma): this field exists to ensure that the PTP is not
        // initialized before the DMA. If PTP is started before the DMA,
        // it doesn't work.
        _dma: &EthernetDMA,
    ) -> Self {
        #[cfg(feature = "f-series")]
        // Mask timestamp interrupt register
        EthernetMAC::mask_timestamp_trigger_interrupt();

        let hclk = clocks.hclk().to_Hz();

        let (stssi, tsa) = Self::calculate_regs(hclk);

        #[cfg(feature = "f-series")]
        let mut me = {
            // Setup PTP timestamping in fine mode.
            eth_ptp.ptptscr.write(|w| {
                // Enable snapshots for all frames.
                #[cfg(not(feature = "stm32f1xx-hal"))]
                let w = w.tssarfe().set_bit();

                w.tse().set_bit().tsfcu().set_bit()
            });

            // Set up subsecond increment
            eth_ptp
                .ptpssir
                .write(|w| unsafe { w.stssi().bits(stssi.raw() as u8) });

            Self { eth_ptp }
        };

        #[cfg(feature = "stm32h7xx-hal")]
        let mut me = {
            let me = Self {};

            // SAFETY: we only write to `mactscr` (timestamp control register)
            let mac = unsafe { me.mac() };

            mac.mactscr.modify(|_, w| {
                w
                    // Enable timestamp snapshots for all frames
                    .tsenall()
                    .set_bit()
                    // Enable fine-grain update mode
                    .tscfupdt()
                    .set_bit()
                    // Enable all timestamps
                    .tsena()
                    .set_bit()
                    // Tell MAC to overwrite non-read timestamps
                    .txtsstsm()
                    .set_bit()
            });

            // Set up the subsecond increment
            mac.macssir
                .write(|w| unsafe { w.ssinc().bits(stssi.raw() as u8) });

            me
        };

        me.set_addend(tsa);
        me.set_time(Timestamp::new_unchecked(false, 0, 0));

        me
    }

    /// Get the configured subsecond increment.
    pub fn subsecond_increment(&self) -> Subseconds {
        #[cfg(feature = "f-series")]
        return Subseconds::new_unchecked(self.eth_ptp.ptpssir.read().stssi().bits() as u32);
        #[cfg(feature = "stm32h7xx-hal")]
        // SAFETY: we only read `macssir` (subsecond register).
        return Subseconds::new_unchecked(unsafe {
            self.mac().macssir.read().ssinc().bits() as u32
        });
    }

    /// Get the currently configured PTP clock addend.
    pub fn addend(&self) -> u32 {
        #[cfg(feature = "f-series")]
        return self.eth_ptp.ptptsar.read().bits();
        #[cfg(feature = "stm32h7xx-hal")]
        // SAFETY: we only read `mactsar` (timestamp addend register).
        return unsafe { self.mac().mactsar.read().bits() };
    }

    /// Set the PTP clock addend.
    #[inline(always)]
    pub fn set_addend(&mut self, rate: u32) {
        #[cfg(feature = "f-series")]
        let ptp = &self.eth_ptp;
        #[cfg(feature = "f-series")]
        ptp.ptptsar.write(|w| unsafe { w.bits(rate) });

        #[cfg(feature = "f-series")]
        {
            #[cfg(feature = "stm32f1xx-hal")]
            {
                while ptp.ptptscr.read().tsaru().bit_is_set() {}
                ptp.ptptscr.modify(|_, w| w.tsaru().set_bit());
                while ptp.ptptscr.read().tsaru().bit_is_set() {}
            }

            #[cfg(not(feature = "stm32f1xx-hal"))]
            {
                while ptp.ptptscr.read().ttsaru().bit_is_set() {}
                ptp.ptptscr.modify(|_, w| w.ttsaru().set_bit());
                while ptp.ptptscr.read().ttsaru().bit_is_set() {}
            }
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // SAFETY: we only write to `mactsar` (timestamp addend register)
            // and `mactscr` (timestamp control register)
            let (mactsar, mactscr) = unsafe {
                let mac = self.mac();
                (&mac.mactsar, &mac.mactscr)
            };

            mactsar.write(|w| unsafe { w.tsar().bits(rate) });

            while mactscr.read().tsaddreg().bit_is_set() {}
            mactscr.modify(|_, w| w.tsaddreg().set_bit());
            while mactscr.read().tsaddreg().bit_is_set() {}
        }
    }

    /// Set the current time.
    pub fn set_time(&mut self, time: Timestamp) {
        let seconds = time.seconds();
        // TODO(stm32h7): figure out if the time being signed
        // means that we have a two's complement number or not
        // (the RM makes it read as though it may be).
        let subseconds = time.subseconds_signed();

        #[cfg(feature = "f-series")]
        {
            let ptp = &self.eth_ptp;

            ptp.ptptshur.write(|w| unsafe { w.bits(seconds) });
            ptp.ptptslur.write(|w| unsafe { w.bits(subseconds) });

            // Initialise timestamp
            while ptp.ptptscr.read().tssti().bit_is_set() {}
            ptp.ptptscr.modify(|_, w| w.tssti().set_bit());
            while ptp.ptptscr.read().tssti().bit_is_set() {}
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // SAFETY: we only write to `mactscr` (timestamp control register), `macstsur`
            // (timestamp update seconds register) and `macstnur` (timestmap update subsecond/nanosecond
            // register)
            let (mactscr, macstsur, macstnur) = unsafe {
                let mac = self.mac();
                (&mac.mactscr, &mac.macstsur, &mac.macstnur)
            };

            macstsur.write(|w| unsafe { w.bits(seconds) });
            macstnur.write(|w| unsafe { w.bits(subseconds) });

            while mactscr.read().tsinit().bit_is_set() {}
            mactscr.modify(|_, w| w.tsinit().set_bit());
            while mactscr.read().tsinit().bit_is_set() {}
        }
    }

    /// Add the provided time to the current time, atomically.
    ///
    /// If `time` is negative, it will instead be subtracted from the
    /// system time.
    pub fn update_time(&mut self, time: Timestamp) {
        let seconds = time.seconds();
        let subseconds = time.subseconds_signed();

        #[cfg(feature = "f-series")]
        {
            let ptp = &self.eth_ptp;

            ptp.ptptshur.write(|w| unsafe { w.bits(seconds) });
            ptp.ptptslur.write(|w| unsafe { w.bits(subseconds) });

            // Add timestamp to global time

            let read_status = || {
                let scr = ptp.ptptscr.read();
                scr.tsstu().bit_is_set() || scr.tssti().bit_is_set()
            };

            while read_status() {}
            ptp.ptptscr.modify(|_, w| w.tsstu().set_bit());
            while ptp.ptptscr.read().tsstu().bit_is_set() {}
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // SAFETY: we only write to `mactscr` (timestamp control register), `macstsur`
            // (timestamp update seconds register) and `macstnur` (timestmap update subsecond/nanosecond
            // register)
            let (mactscr, macstsur, macstnur) = unsafe {
                let mac = self.mac();
                (&mac.mactscr, &mac.macstsur, &mac.macstnur)
            };

            macstsur.write(|w| unsafe { w.bits(seconds) });
            macstnur.write(|w| unsafe { w.bits(subseconds) });

            while mactscr.read().tsupdt().bit_is_set() {}
            mactscr.modify(|_, w| w.tsupdt().set_bit());
            while mactscr.read().tsupdt().bit_is_set() {}
        }
    }

    /// Get the current time.
    pub fn get_time(&self) -> Timestamp {
        let try_read_time = || {
            #[cfg(feature = "f-series")]
            let (seconds, subseconds, seconds_after) = {
                let seconds = self.eth_ptp.ptptshr.read().bits();
                let subseconds = self.eth_ptp.ptptslr.read().bits();
                let seconds2 = self.eth_ptp.ptptshr.read().bits();
                (seconds, subseconds, seconds2)
            };

            #[cfg(feature = "stm32h7xx-hal")]
            let (seconds, subseconds, seconds_after) = {
                let (macstsr, macstnr) = unsafe {
                    let mac = self.mac();
                    (&mac.macstsr, &mac.macstnr)
                };

                let seconds = macstsr.read().bits();
                let subseconds = macstnr.read().bits();
                let seconds2 = macstsr.read().bits();
                (seconds, subseconds, seconds2)
            };

            if seconds == seconds_after {
                Ok(Timestamp::from_parts(seconds, subseconds))
            } else {
                Err(())
            }
        };

        loop {
            if let Ok(res) = try_read_time() {
                return res;
            }
        }
    }

    /// Enable the PPS output on the provided pin.
    pub fn enable_pps<P>(&mut self, pin: P) -> P::Output
    where
        P: PPSPin,
    {
        pin.enable()
    }
}

/// Setting and configuring target time interrupts on the STM32F107 does not
/// make any sense: we can generate the interrupt, but it is impossible to
/// clear the flag as the register required to do so does not exist.
#[cfg(all(not(feature = "stm32f1xx-hal")))]
impl EthernetPTP {
    /// Configure the target time.
    fn set_target_time(&mut self, timestamp: Timestamp) {
        let (high, low) = (timestamp.seconds(), timestamp.subseconds_signed());

        #[cfg(feature = "f-series")]
        {
            self.eth_ptp
                .ptptthr
                .write(|w| unsafe { w.ttsh().bits(high) });
            self.eth_ptp
                .ptpttlr
                .write(|w| unsafe { w.ttsl().bits(low) });
        }

        #[cfg(feature = "stm32h7xx-hal")]
        {
            // SAFETY: we only write to `ppsttsr` (PPS target time seconds register) and
            // `ppsttnr` (PPS target time subseconds register)
            let (ppsttsr, ppsttnr) = unsafe {
                let mac = self.mac();
                (&mac.macppsttsr, &mac.macppsttnr)
            };

            ppsttsr.write(|w| unsafe { w.bits(high) });
            ppsttnr.write(|w| unsafe { w.bits(low) });
        }
    }

    /// Configure the target time interrupt.
    ///
    /// You must call [`EthernetPTP::interrupt_handler`] in the `ETH`
    /// interrupt to detect (and clear) the correct status bits.
    pub fn configure_target_time_interrupt(&mut self, timestamp: Timestamp) {
        self.set_target_time(timestamp);
        #[cfg(feature = "f-series")]
        {
            self.eth_ptp.ptptscr.modify(|_, w| w.tsite().set_bit());
            EthernetMAC::unmask_timestamp_trigger_interrupt();
        }
    }

    /// Returns a boolean indicating whether or not the interrupt
    /// was caused by a Timestamp trigger and clears the interrupt
    /// flag.
    pub fn interrupt_handler(&mut self) -> bool {
        #[cfg(feature = "f-series")]
        let is_tsint = {
            let is_tsint = self.eth_ptp.ptptssr.read().tsttr().bit_is_set();
            if is_tsint {
                self.eth_ptp.ptptscr.modify(|_, w| w.tsite().clear_bit());
                EthernetMAC::mask_timestamp_trigger_interrupt();
            }
            is_tsint
        };

        #[cfg(feature = "stm32h7xx-hal")]
        let is_tsint = {
            // SAFETY: we only write to `mactssr` (Timestamp Status register)
            let mactssr = unsafe { &self.mac().mactssr };

            // Reading the bit clears it, and deasserts the interrupt.
            mactssr.read().tstargt0().bit_is_set()
        };

        is_tsint
    }

    #[cfg(feature = "f-series")]
    /// Configure the PPS output frequency.
    ///
    /// The PPS output frequency becomes `2 ^ pps_freq`. `pps_freq` is
    /// clamped to `[0..31]`.
    pub fn set_pps_freq(&mut self, pps_freq: u8) {
        let pps_freq = pps_freq.max(31);

        // SAFETY: we atomically write to the PTPPPSCR register, which is
        // not read or written to anywhere else. The SVD files are incorrectly
        // saying that the bits in this register are read-only.
        unsafe {
            let ptpppscr = self.eth_ptp.ptpppscr.as_ptr() as *mut u32;
            core::ptr::write_volatile(ptpppscr, pps_freq as u32);
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
mod test {

    use super::*;

    // Test that we get accurate addend and subsecond_increment values
    // with the provided clock speeds.
    #[test]
    fn hclk_to_regs() {
        for hclk_hz in (25..180).map(|v| v * 1_000_000) {
            let (stssi, tsa) = EthernetPTP::calculate_regs(hclk_hz);

            let stssi = stssi.raw() as f64;
            let tsa = tsa as f64;

            // calculate the clock ratio
            let clock_ratio = (SUBSECONDS_PER_SECOND as f64 / stssi)
                / (hclk_hz as f64 * (tsa / 0xFFFF_FFFFu32 as f64));

            let ppm = (clock_ratio - 1f64) * 1_000_000f64;

            assert!(ppm <= 0.06, "{} at {}", ppm, hclk_hz);
        }
    }
}
