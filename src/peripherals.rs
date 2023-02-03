//! Re-export or add abstracted versions of `ETHERNET_MAC`, `ETHERNET_DMA`
//! and `ETHERNET_PTP` that introduce a delay for some registers on F4 parts.

#[cfg(any(feature = "stm32f107", feature = "stm32f7xx-hal"))]
pub use crate::hal::pac::{ETHERNET_DMA, ETHERNET_MAC, ETHERNET_PTP};

#[cfg(feature = "stm32h7xx-hal")]
pub use crate::hal::pac::{ETHERNET_DMA, ETHERNET_MAC};

#[cfg(feature = "stm32f4xx-hal")]
pub use pac_override_impl::{ETHERNET_DMA, ETHERNET_MAC, ETHERNET_PTP};

#[cfg(feature = "stm32f4xx-hal")]
mod pac_override_impl {
    #![allow(non_camel_case_types)]

    macro_rules ! setup_use {
        ($(($feat:literal, $name:ident)),*) => {
            $(
                #[cfg(feature = $feat)]
                use stm32f4::$name as stm32;
            )*
        }
    }

    setup_use!(
        ("stm32f407", stm32f407),
        ("stm32f417", stm32f407),
        ("stm32f427", stm32f427),
        ("stm32f429", stm32f429),
        ("stm32f437", stm32f427),
        ("stm32f439", stm32f429),
        ("stm32f469", stm32f469),
        ("stm32f479", stm32f469)
    );

    use core::marker::PhantomData;

    use stm32::{
        ethernet_dma::*, ethernet_mac::*, ethernet_ptp::*, ETHERNET_DMA as PAC_ETHERNET_DMA,
        ETHERNET_MAC as PAC_ETHERNET_MAC, ETHERNET_PTP as PAC_ETHERNET_PTP,
    };
    use stm32f4::{Readable, Reg, RegisterSpec, Resettable, Writable, W};

    pub struct DelayedReg<T: RegisterSpec> {
        inner: Reg<T>,
    }

    impl<REG: Readable> DelayedReg<REG> {
        /// Reads the contents of a `Readable` register.
        ///
        /// You can read the raw contents of a register by using `bits`:
        /// ```ignore
        /// let bits = periph.reg.read().bits();
        /// ```
        /// or get the content of a particular field of a register:
        /// ```ignore
        /// let reader = periph.reg.read();
        /// let bits = reader.field1().bits();
        /// let flag = reader.field2().bit_is_set();
        /// ```
        #[inline(always)]
        pub fn read(&self) -> REG::Reader {
            self.inner.read()
        }
    }

    impl<REG: Readable + Writable> DelayedReg<REG> {
        /// Modifies the contents of the register by reading and then writing it.
        ///
        /// This version of `modify` _always_ adds a 29 cycle delay to mitigate the effects
        /// of an erratum.
        ///
        /// E.g. to do a read-modify-write sequence to change parts of a register:
        /// ```ignore
        /// periph.reg.modify(|r, w| unsafe { w.bits(
        ///    r.bits() | 3
        /// ) });
        /// ```
        /// or
        /// ```ignore
        /// periph.reg.modify(|_, w| w
        ///     .field1().bits(newfield1bits)
        ///     .field2().set_bit()
        ///     .field3().variant(VARIANT)
        /// );
        /// ```
        /// Other fields will have the value they had before the call to `modify`.
        #[inline(always)]
        pub fn modify<F>(&self, f: F)
        where
            for<'w> F: FnOnce(&REG::Reader, &'w mut REG::Writer) -> &'w mut W<REG>,
        {
            delay();
            self.inner.modify(f)
        }
    }

    impl<REG: Writable + Resettable> DelayedReg<REG> {
        /// Writes the reset value to `Writable` register.
        ///
        /// Resets the register to its initial state.
        /// This version of `reset` _always_ adds a 29 cycle delay to mitigate the effects
        /// of an erratum.
        #[inline(always)]
        #[allow(unused)]
        pub fn reset(&self) {
            delay();
            self.inner.reset()
        }

        /// Writes bits to a `Writable` register.
        ///
        /// This version of `wrote` _always_ adds a 29 cycle delay to mitigate the effects
        /// of an erratum.
        ///
        /// You can write raw bits into a register:
        /// ```ignore
        /// periph.reg.write(|w| unsafe { w.bits(rawbits) });
        /// ```
        /// or write only the fields you need:
        /// ```ignore
        /// periph.reg.write(|w| w
        ///     .field1().bits(newfield1bits)
        ///     .field2().set_bit()
        ///     .field3().variant(VARIANT)
        /// );
        /// ```
        /// In the latter case, other fields will be set to their reset value.
        #[inline(always)]
        #[allow(unused)]
        pub fn write<F>(&self, f: F)
        where
            F: FnOnce(&mut REG::Writer) -> &mut W<REG>,
        {
            delay();
            self.inner.write(f)
        }
    }

    #[repr(C)]
    pub struct MacRegisterBlock {
        #[doc = "0x00 - Ethernet MAC configuration register"]
        pub maccr: DelayedReg<maccr::MACCR_SPEC>,
        #[doc = "0x04 - Ethernet MAC frame filter register"]
        pub macffr: DelayedReg<macffr::MACFFR_SPEC>,
        #[doc = "0x08 - Ethernet MAC hash table high register"]
        pub machthr: DelayedReg<machthr::MACHTHR_SPEC>,
        #[doc = "0x0c - Ethernet MAC hash table low register"]
        pub machtlr: DelayedReg<machtlr::MACHTLR_SPEC>,
        #[doc = "0x10 - Ethernet MAC MII address register"]
        pub macmiiar: Reg<macmiiar::MACMIIAR_SPEC>,
        #[doc = "0x14 - Ethernet MAC MII data register"]
        pub macmiidr: Reg<macmiidr::MACMIIDR_SPEC>,
        #[doc = "0x18 - Ethernet MAC flow control register"]
        pub macfcr: DelayedReg<macfcr::MACFCR_SPEC>,
        #[doc = "0x1c - Ethernet MAC VLAN tag register"]
        pub macvlantr: DelayedReg<macvlantr::MACVLANTR_SPEC>,
        _reserved8: [u8; 0x0c],
        #[doc = "0x2c - Ethernet MAC PMT control and status register"]
        pub macpmtcsr: DelayedReg<macpmtcsr::MACPMTCSR_SPEC>,
        _reserved9: [u8; 0x04],
        #[doc = "0x34 - Ethernet MAC debug register"]
        pub macdbgr: Reg<macdbgr::MACDBGR_SPEC>,
        #[doc = "0x38 - Ethernet MAC interrupt status register"]
        pub macsr: Reg<macsr::MACSR_SPEC>,
        #[doc = "0x3c - Ethernet MAC interrupt mask register"]
        pub macimr: Reg<macimr::MACIMR_SPEC>,
        #[doc = "0x40 - Ethernet MAC address 0 high register"]
        pub maca0hr: DelayedReg<maca0hr::MACA0HR_SPEC>,
        #[doc = "0x44 - Ethernet MAC address 0 low register"]
        pub maca0lr: DelayedReg<maca0lr::MACA0LR_SPEC>,
        #[doc = "0x48 - Ethernet MAC address 1 high register"]
        pub maca1hr: DelayedReg<maca1hr::MACA1HR_SPEC>,
        #[doc = "0x4c - Ethernet MAC address1 low register"]
        pub maca1lr: DelayedReg<maca1lr::MACA1LR_SPEC>,
        #[doc = "0x50 - Ethernet MAC address 2 high register"]
        pub maca2hr: DelayedReg<maca2hr::MACA2HR_SPEC>,
        #[doc = "0x54 - Ethernet MAC address 2 low register"]
        pub maca2lr: DelayedReg<maca2lr::MACA2LR_SPEC>,
        #[doc = "0x58 - Ethernet MAC address 3 high register"]
        pub maca3hr: DelayedReg<maca3hr::MACA3HR_SPEC>,
        #[doc = "0x5c - Ethernet MAC address 3 low register"]
        pub maca3lr: DelayedReg<maca3lr::MACA3LR_SPEC>,
    }

    pub struct ETHERNET_MAC {
        _marker: PhantomData<*const ()>,
    }

    unsafe impl core::marker::Send for ETHERNET_MAC {}

    impl ETHERNET_MAC {
        #[doc = r"Pointer to the register block"]
        pub const PTR: *const MacRegisterBlock = 0x4002_8000 as *const _;

        #[doc = r"Return the pointer to the register block"]
        #[inline(always)]
        pub const fn ptr() -> *const MacRegisterBlock {
            Self::PTR
        }
    }

    impl core::ops::Deref for ETHERNET_MAC {
        type Target = MacRegisterBlock;

        fn deref(&self) -> &Self::Target {
            unsafe { &*Self::PTR }
        }
    }

    impl From<PAC_ETHERNET_MAC> for ETHERNET_MAC {
        fn from(_value: PAC_ETHERNET_MAC) -> Self {
            Self {
                _marker: Default::default(),
            }
        }
    }

    #[repr(C)]
    pub struct DmaRegisterBlock {
        #[doc = "0x00 - Ethernet DMA bus mode register"]
        pub dmabmr: DelayedReg<dmabmr::DMABMR_SPEC>,
        #[doc = "0x04 - Ethernet DMA transmit poll demand register"]
        pub dmatpdr: Reg<dmatpdr::DMATPDR_SPEC>,
        #[doc = "0x08 - EHERNET DMA receive poll demand register"]
        pub dmarpdr: Reg<dmarpdr::DMARPDR_SPEC>,
        #[doc = "0x0c - Ethernet DMA receive descriptor list address register"]
        pub dmardlar: Reg<dmardlar::DMARDLAR_SPEC>,
        #[doc = "0x10 - Ethernet DMA transmit descriptor list address register"]
        pub dmatdlar: Reg<dmatdlar::DMATDLAR_SPEC>,
        #[doc = "0x14 - Ethernet DMA status register"]
        pub dmasr: Reg<dmasr::DMASR_SPEC>,
        #[doc = "0x18 - Ethernet DMA operation mode register"]
        pub dmaomr: DelayedReg<dmaomr::DMAOMR_SPEC>,
        #[doc = "0x1c - Ethernet DMA interrupt enable register"]
        pub dmaier: Reg<dmaier::DMAIER_SPEC>,
        #[doc = "0x20 - Ethernet DMA missed frame and buffer overflow counter register"]
        pub dmamfbocr: Reg<dmamfbocr::DMAMFBOCR_SPEC>,
        #[doc = "0x24 - Ethernet DMA receive status watchdog timer register"]
        pub dmarswtr: Reg<dmarswtr::DMARSWTR_SPEC>,
        _reserved10: [u8; 0x20],
        #[doc = "0x48 - Ethernet DMA current host transmit descriptor register"]
        pub dmachtdr: Reg<dmachtdr::DMACHTDR_SPEC>,
        #[doc = "0x4c - Ethernet DMA current host receive descriptor register"]
        pub dmachrdr: Reg<dmachrdr::DMACHRDR_SPEC>,
        #[doc = "0x50 - Ethernet DMA current host transmit buffer address register"]
        pub dmachtbar: Reg<dmachtbar::DMACHTBAR_SPEC>,
        #[doc = "0x54 - Ethernet DMA current host receive buffer address register"]
        pub dmachrbar: Reg<dmachrbar::DMACHRBAR_SPEC>,
    }

    pub struct ETHERNET_DMA {
        _marker: PhantomData<*const ()>,
    }

    unsafe impl core::marker::Send for ETHERNET_DMA {}

    impl ETHERNET_DMA {
        #[doc = r"Pointer to the register block"]
        pub const PTR: *const DmaRegisterBlock = 0x4002_9000 as *const _;
    }

    impl core::ops::Deref for ETHERNET_DMA {
        type Target = DmaRegisterBlock;

        fn deref(&self) -> &Self::Target {
            unsafe { &*Self::PTR }
        }
    }

    impl core::borrow::Borrow<ETHERNET_DMA> for PAC_ETHERNET_DMA {
        fn borrow(&self) -> &ETHERNET_DMA {
            &ETHERNET_DMA {
                _marker: PhantomData,
            }
        }
    }

    impl From<PAC_ETHERNET_DMA> for ETHERNET_DMA {
        fn from(_value: PAC_ETHERNET_DMA) -> Self {
            Self {
                _marker: Default::default(),
            }
        }
    }

    #[repr(C)]
    pub struct PtpRegisterBlock {
        #[doc = "0x00 - Ethernet PTP time stamp control register"]
        pub ptptscr: DelayedReg<ptptscr::PTPTSCR_SPEC>,
        #[doc = "0x04 - Ethernet PTP subsecond increment register"]
        pub ptpssir: Reg<ptpssir::PTPSSIR_SPEC>,
        #[doc = "0x08 - Ethernet PTP time stamp high register"]
        pub ptptshr: Reg<ptptshr::PTPTSHR_SPEC>,
        #[doc = "0x0c - Ethernet PTP time stamp low register"]
        pub ptptslr: Reg<ptptslr::PTPTSLR_SPEC>,
        #[doc = "0x10 - Ethernet PTP time stamp high update register"]
        pub ptptshur: Reg<ptptshur::PTPTSHUR_SPEC>,
        #[doc = "0x14 - Ethernet PTP time stamp low update register"]
        pub ptptslur: Reg<ptptslur::PTPTSLUR_SPEC>,
        #[doc = "0x18 - Ethernet PTP time stamp addend register"]
        pub ptptsar: Reg<ptptsar::PTPTSAR_SPEC>,
        #[doc = "0x1c - Ethernet PTP target time high register"]
        pub ptptthr: Reg<ptptthr::PTPTTHR_SPEC>,
        #[doc = "0x20 - Ethernet PTP target time low register"]
        pub ptpttlr: Reg<ptpttlr::PTPTTLR_SPEC>,
        _reserved9: [u8; 0x04],
        #[doc = "0x28 - Ethernet PTP time stamp status register"]
        pub ptptssr: Reg<ptptssr::PTPTSSR_SPEC>,
        #[doc = "0x2c - Ethernet PTP PPS control register"]
        pub ptpppscr: Reg<ptpppscr::PTPPPSCR_SPEC>,
    }

    pub struct ETHERNET_PTP {
        _marker: PhantomData<*const ()>,
    }

    unsafe impl core::marker::Send for ETHERNET_PTP {}

    impl ETHERNET_PTP {
        #[doc = r"Pointer to the register block"]
        pub const PTR: *const PtpRegisterBlock = 0x4002_8700 as *const _;
    }

    impl core::ops::Deref for ETHERNET_PTP {
        type Target = PtpRegisterBlock;

        fn deref(&self) -> &Self::Target {
            unsafe { &*Self::PTR }
        }
    }

    impl From<PAC_ETHERNET_PTP> for ETHERNET_PTP {
        fn from(_value: PAC_ETHERNET_PTP) -> Self {
            Self {
                _marker: Default::default(),
            }
        }
    }

    #[inline(always)]
    /// Preventative measure for issues described in:
    /// Section 2.11.5 of [ES0182, Rev 13] (for F40x and F41x)
    /// Section 2.17.5 of [ES0206, Rev 18] (for F42x and F43x)
    /// Section 2.18.5 of [ES0321, Rev 8] (for F46x and F47x)
    ///
    /// [ES0182, Rev 13]: https://www.st.com/resource/en/errata_sheet/es0182-stm32f405407xx-and-stm32f415417xx-device-limitations-stmicroelectronics.pdf
    /// [ES0206, Rev 18]: https://www.st.com/resource/en/errata_sheet/es0206-stm32f427437-and-stm32f429439-line-limitations-stmicroelectronics.pdf
    /// [ES0321, Rev 8]: https://www.st.com/resource/en/errata_sheet/es0321-stm32f469xx-and-stm32f479xx-device-errata-stmicroelectronics.pdf
    fn delay() {
        // Max speed: 180_000_000 Hz
        //
        // Minimum required delay: 4 clocks @ 25 MHz = 160 ns
        //
        // Minimum required delay clocks: ceil(180_000_000/25_000_000) * 4 = 29 clocks = 161.111... ns
        cortex_m::asm::delay(29);
    }
}
