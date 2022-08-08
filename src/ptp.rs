use crate::stm32::ETHERNET_PTP;

use crate::EthernetMAC;

pub(crate) fn setup_ptp(mac: &mut EthernetMAC, eth_ptp: ETHERNET_PTP) {
    // Mask timestamp interrupt register, required for stm32f107 according to AN3411
    #[cfg(feature = "stm32f107")]
    mac.mask_timestamp_interrupt();

    if false {
        // Setup PTP timestamping
        eth_ptp.ptptscr.write(|w| {
            #[cfg(not(feature = "stm32f107"))]
            let w = w.tsssr().set_bit().tssarfe().set_bit();

            w.tse().set_bit().tsfcu().set_bit()
        });

        // Set sub-second increment to 20ns and initial addend to HCLK/(1/20ns) (HCLK=100MHz)
        eth_ptp.ptpssir.write(|w| unsafe { w.stssi().bits(20) });
        eth_ptp.ptptsar.write(|w| unsafe { w.tsa().bits(1 << 31) });

        #[cfg(feature = "stm32f107")]
        {
            eth_ptp.ptptscr.modify(|_, w| w.tsaru().set_bit());
            while eth_ptp.ptptscr.read().tsaru().bit_is_set() {}
        }

        #[cfg(not(feature = "stm32f107"))]
        {
            eth_ptp.ptptscr.modify(|_, w| w.ttsaru().set_bit());
            while eth_ptp.ptptscr.read().ttsaru().bit_is_set() {}
        }

        // Initialise timestamp
        eth_ptp.ptptscr.modify(|_, w| w.tssti().set_bit());
        while eth_ptp.ptptscr.read().tssti().bit_is_set() {}
    }
}
