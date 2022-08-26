use crate::{
    desc::Descriptor,
    hal::rcc::Clocks,
    stm32::{ETHERNET_MAC, ETHERNET_PTP},
};

/// A timestamp produced by the PTP periperhal
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug)]
pub struct Timestamp {
    nanos: u64,
}

impl Timestamp {
    const NANOS_PER_SECOND: u64 = 1_000_000_000;

    #[cfg(not(feature = "stm32f1xx-hal"))]
    fn new(seconds: u32, nanos: u32) -> Self {
        Self {
            nanos: (seconds as u64 * Self::NANOS_PER_SECOND) + nanos as u64,
        }
    }

    #[cfg(feature = "stm32f1xx-hal")]
    fn new(nanos: u64) -> Self {
        Self { nanos }
    }

    /// Get the second component of this timestamp
    pub fn seconds(&self) -> u32 {
        (self.nanos / Self::NANOS_PER_SECOND) as u32
    }

    /// Get the nanosecond component of this timestamp
    pub fn nanos(&self) -> u32 {
        let nanos = self.nanos % Self::NANOS_PER_SECOND;
        nanos as u32
    }

    /// Get the raw value of this timestamp, in nanoseconds
    pub fn nanos_raw(&self) -> u64 {
        self.nanos
    }

    /// Create a timestamp from the given descriptor
    ///
    /// # Safety
    /// On non-f1 parts, the caller must ensure that the timestamp data
    /// is valid.
    pub unsafe fn from_descriptor(desc: &Descriptor) -> Option<Self> {
        // NOTE: The Bit 31 of the `subseconds`/`nanos` value indicates
        // the signedness of the system time. We should probably deal with that
        // in the future.

        #[cfg(not(feature = "stm32f1xx-hal"))]
        {
            let (seconds, nanos) = { (desc.read(7), desc.read(6)) };
            Some(Timestamp::new(seconds, nanos))
        }

        #[cfg(feature = "stm32f1xx-hal")]
        {
            let (seconds, subseconds) = { (desc.read(3), desc.read(2)) };

            // The timestamp registers are written to all-ones if
            // timestamping was no succesfull
            if seconds == 0xFFFF_FFFF && subseconds == 0xFFFF_FFFF {
                None
            } else {
                Some(Timestamp::new((seconds as u64) << 31 | subseconds as u64))
            }
        }
    }
}

pub(crate) fn setup_ptp(_eth_mac: &ETHERNET_MAC, eth_ptp: ETHERNET_PTP, clocks: Clocks) {
    // Mask timestamp interrupt register, required for stm32f107 according to AN3411
    #[cfg(feature = "stm32f1xx-hal")]
    _eth_mac.macimr.modify(|_, w| w.tstim().set_bit());

    if true {
        // Setup PTP timestamping
        eth_ptp.ptptscr.write(|w| {
            #[cfg(not(feature = "stm32f1xx-hal"))]
            let w = w.tsssr().set_bit().tssarfe().set_bit();

            w.tse().set_bit().tsfcu().set_bit()
        });

        // Set sub-second increment to 20ns and initial addend to HCLK/(1/20ns) (HCLK=100MHz)
        eth_ptp.ptpssir.write(|w| unsafe { w.stssi().bits(20) });

        // TSA = 2^32 / (HCLK / sub-second-increment-in-hz)
        //     = 2^32 / (HCLK / 50_000_000)
        //     = (50_000_000 * 2^32) / HCLK
        //     = (50_000_000 << 32) / HCLK
        const NOM: u64 = 50_000_000 << 32;
        let tsa = (NOM / clocks.hclk().to_Hz() as u64) as u32;

        eth_ptp.ptptsar.write(|w| unsafe { w.tsa().bits(tsa) });

        #[cfg(feature = "stm32f1xx-hal")]
        {
            eth_ptp.ptptscr.modify(|_, w| w.tsaru().set_bit());
            while eth_ptp.ptptscr.read().tsaru().bit_is_set() {}
        }

        #[cfg(not(feature = "stm32f1xx-hal"))]
        {
            eth_ptp.ptptscr.modify(|_, w| w.ttsaru().set_bit());
            while eth_ptp.ptptscr.read().ttsaru().bit_is_set() {}
        }

        // Initialise timestamp
        eth_ptp.ptptscr.modify(|_, w| w.tssti().set_bit());
        while eth_ptp.ptptscr.read().tssti().bit_is_set() {}
    }
}
