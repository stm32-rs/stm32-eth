//! Hardware filtering of Ethernet frames

use crate::hal::pac::ETHERNET_MAC;

mod destination;
pub use destination::*;

mod source;
pub use source::*;

mod multicast;
pub use multicast::*;

mod hash_table;
pub use hash_table::*;

mod control;
pub use control::*;

/// The frame filtering that the MAC should apply to
/// received Ethernet frames.
#[derive(Debug, Clone)]
pub enum FrameFilteringMode {
    /// No frame filtering on any frames.
    Promiscuous,
    /// Perform frame filtering based on the provided
    /// configuration.
    Filter(FrameFiltering),
}

impl Default for FrameFilteringMode {
    fn default() -> Self {
        Self::Promiscuous
    }
}

impl FrameFilteringMode {
    pub(crate) fn configure(&self, eth_mac: &ETHERNET_MAC) {
        match self {
            FrameFilteringMode::Promiscuous => eth_mac.macffr.write(|w| w.pm().set_bit()),
            FrameFilteringMode::Filter(config) => config.configure(eth_mac),
        }
    }
}

/// The type of frame filtering that the MAC
/// should perform.
///
/// The total amount of [`MacAddressFilter`]s in this configuration object may
/// be at most 3.
#[derive(Debug, Clone)]
pub struct FrameFiltering {
    /// The MAC address of this station. This address is always
    /// used for Destination Address filtering.
    pub base_address: Mac,

    /// Extra address filters to be used.
    pub address_filters: [Option<MacAddressFilter>; 3],

    /// Frame filtering applied to frames based on
    /// their destination address.
    pub destination_address_filter: DestinationAddressFiltering,

    /// Frame filtering applied to frames based on
    /// their source address.
    pub source_address_filter: SourceAddressFiltering,

    /// Frame filtering applied to frames based on
    /// whether they have a multicast address or not.
    pub multicast_address_filter: MulticastAddressFiltering,

    /// Control frame filtering mode,
    pub control_filter: ControlFrameFiltering,

    /// Hash table configuration.
    pub hash_table_value: HashTableValue,

    /// Enable or disable broadcast frame filtering.
    ///
    /// If set to `true`, broadcast frames will be filtered out.
    pub filter_broadcast: bool,

    /// Enable or disable receive all mode.
    ///
    /// If receive all mode is disabled, address filtering
    /// is applied to incoming frames (and corresponding bits
    /// in the descriptors are configured), but the MAC does not
    /// drop any frames.
    pub receive_all: bool,
}

impl FrameFiltering {
    /// Create a new basic [`FrameFiltering`] that:
    /// * Does not filter out frames destined for `station_addr` or an address
    /// contained in `extra_address`.
    /// * Does not filter out multicast frames.
    /// * Does not filter out broadcast frames.
    /// * Filters out all control frames.
    pub const fn filter_destinations(station_addr: Mac, extra_addresses: &[Mac]) -> Self {
        assert!(extra_addresses.len() <= 3);

        let mut address_filters = [None, None, None];

        let mut i = 0;
        loop {
            address_filters[i] = Some(MacAddressFilter::new(
                AddressFilterType::Destination,
                extra_addresses[i],
                MacAddressFilterMask::empty(),
            ));

            i += 1;
            if i == 3 || i == extra_addresses.len() {
                break;
            }
        }

        FrameFiltering {
            base_address: station_addr,
            address_filters,
            destination_address_filter: DestinationAddressFiltering {
                perfect_filtering: PerfectDestinationAddressFilteringMode::Normal,
                hash_table_filtering: false,
            },
            source_address_filter: SourceAddressFiltering::Ignore,
            multicast_address_filter: MulticastAddressFiltering::PassAll,
            control_filter: ControlFrameFiltering::BlockAll,
            hash_table_value: HashTableValue::new(),
            filter_broadcast: false,
            receive_all: false,
        }
    }

    fn configure(&self, eth_mac: &ETHERNET_MAC) {
        let FrameFiltering {
            base_address,
            address_filters,
            destination_address_filter,
            source_address_filter,
            multicast_address_filter,
            control_filter,
            hash_table_value: hash_table_filtering,
            filter_broadcast,
            receive_all,
        } = self;

        eth_mac
            .maca0hr
            .write(|w| w.maca0h().bits(base_address.high()));
        eth_mac
            .maca0lr
            .write(|w| w.maca0l().bits(base_address.low()));

        let daif = match &destination_address_filter.perfect_filtering {
            PerfectDestinationAddressFilteringMode::Normal => false,
            PerfectDestinationAddressFilteringMode::Inverse => true,
        };
        let hu = destination_address_filter.hash_table_filtering;

        let (saf, saif) = match &source_address_filter {
            SourceAddressFiltering::Ignore => (false, false),
            SourceAddressFiltering::Normal => (true, false),
            SourceAddressFiltering::Inverse => (true, true),
        };

        let (pam, hm) = match &multicast_address_filter {
            MulticastAddressFiltering::PassAll => (true, false),
            MulticastAddressFiltering::DestinationAddressHash => (false, true),
            MulticastAddressFiltering::DestinationAddress => (false, false),
        };

        let pcf = match &control_filter {
            ControlFrameFiltering::BlockAll => 0b00,
            ControlFrameFiltering::NoPause => 0b01,
            ControlFrameFiltering::AllowAll => 0b10,
            ControlFrameFiltering::AddressFilter => 0b11,
        };

        macro_rules! next_addr_reg {
            ($idx:literal, $regh:ident, $regl:ident, $ah:ident, $al:ident) => {
                if let Some(filter) = &address_filters[$idx] {
                    let sa = match filter.ty {
                        AddressFilterType::Destination => false,
                        AddressFilterType::Source => true,
                    };

                    eth_mac.$regh.write(|w| {
                        w.ae()
                            .set_bit()
                            .sa()
                            .bit(sa)
                            .mbc()
                            .bits(filter.mask.bits())
                            .$ah()
                            .bits(filter.address.high())
                    });

                    // This operation is only unsafe for register maca2lr STM32F107
                    //
                    // NOTE(safety): this operation is only unsafe for a single one
                    // of the lower-address-part registers, so this should be fine.
                    #[allow(unused_unsafe)]
                    eth_mac
                        .$regl
                        .write(|w| unsafe { w.$al().bits(filter.address.low()) });
                } else {
                    eth_mac.$regh.write(|w| w.ae().clear_bit());
                }
            };
        }

        next_addr_reg!(0, maca1hr, maca1lr, maca1h, maca1l);
        next_addr_reg!(1, maca2hr, maca2lr, maca2h, maca2l);
        next_addr_reg!(2, maca3hr, maca3lr, maca3h, maca3l);

        eth_mac.macffr.write(|w| {
            w.hpf()
                .clear_bit()
                .pm()
                .clear_bit()
                .ra()
                .bit(*receive_all)
                .saf()
                .bit(saf)
                .saif()
                .bit(saif)
                .pcf()
                .bits(pcf)
                .bfd()
                .bit(*filter_broadcast)
                .pam()
                .bit(pam)
                .daif()
                .bit(daif)
                .hm()
                .bit(hm)
                .hu()
                .bit(hu)
        });

        eth_mac
            .machthr
            .write(|w| w.hth().bits(hash_table_filtering.high));

        eth_mac
            .machtlr
            .write(|w| w.htl().bits(hash_table_filtering.low));
    }
}

/// A big-endian MAC address.
#[derive(Debug, Clone, Copy)]
pub struct Mac([u8; 6]);

impl Mac {
    /// Create a new MAC address with the given big-endian value.
    pub fn new(address: [u8; 6]) -> Self {
        Self(address)
    }

    /// Get the raw bytes of this MAC address.
    pub fn raw(&self) -> &[u8; 6] {
        &self.0
    }
    /// Returns `true` if this MAC is locally administred, i.e. it has the I/G bit set.
    pub fn is_multicast(&self) -> bool {
        (self.0[0] & 0b1) == 0b1
    }

    /// Returns `true` if this MAC is locally administred, i.e. it has the U/L bit set.
    pub fn is_locally_administred(&self) -> bool {
        (self.0[0] & 0b10) == 0b10
    }

    /// Returns `true` if this MAC is the broadcast address.
    pub fn is_broadcast(&self) -> bool {
        self.0.iter().all(|v| v == &0xFF)
    }

    /// Return bytes in a form that can be put into the high portion
    /// of a MAC address register by converting them to little-endian
    fn high(&self) -> u16 {
        let high_bytes = [self.0[5], self.0[4]];
        u16::from_ne_bytes(high_bytes)
    }

    /// Return bytes in a form that can be put into the low portion
    /// of a MAC address register by converting them to little-endian.
    fn low(&self) -> u32 {
        let low_bytes = [self.0[3], self.0[2], self.0[1], self.0[0]];
        u32::from_ne_bytes(low_bytes)
    }
}

impl From<[u8; 6]> for Mac {
    fn from(value: [u8; 6]) -> Self {
        Self::new(value)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Mac {
    fn format(&self, fmt: defmt::Formatter) {
        let addr = self.0;
        defmt::write!(
            fmt,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            addr[0],
            addr[1],
            addr[2],
            addr[3],
            addr[4],
            addr[5]
        )
    }
}

#[derive(Debug, Clone, Copy)]
/// The type of an address filter.
pub enum AddressFilterType {
    /// Filter based on Source Address.
    Source,
    /// Filter based on Destination address.
    Destination,
}

/// A MAC address filter
#[derive(Debug, Clone, Copy)]

pub struct MacAddressFilter {
    /// The address on which this filter
    /// should operate.
    pub ty: AddressFilterType,
    /// The address that this filter should use.
    pub address: Mac,
    /// The byte mask that should be used to mask
    /// out specific parts of the to-be-compared address
    /// for comparison.
    pub mask: MacAddressFilterMask,
}

impl MacAddressFilter {
    /// Create a new MAC address filter.
    pub const fn new(ty: AddressFilterType, address: Mac, mask: MacAddressFilterMask) -> Self {
        Self { ty, address, mask }
    }
}

bitflags::bitflags! {
    /// A mask to be applied when comparing a [`MacAddressFilter`]
    /// to an incoming address.
    pub struct MacAddressFilterMask: u8 {
        /// Ignore byte 5 (the most significant byte)
        /// during address comparison.
        const BYTE5 = 1 << 5;
        /// Ignore byte 4 during address comparison.
        const BYTE4 = 1 << 4;
        /// Ignore byte 3 during address comparison.
        const BYTE3 = 1 << 3;
        /// Ignore byte 2 during address comparison.
        const BYTE2 = 1 << 2;
        /// Ignore byte 1 during address comparison.
        const BYTE1 = 1 << 1;
        /// Ignore byte 0 during address comparison.
        const BYTE0 = 1 << 0;
    }
}
