/// The type of filtering that the MAC should apply to
/// frames that are to be transmitted.
#[derive(Debug, Clone)]
pub struct DestinationAddressFiltering {
    /// Filtering to be performed based on perfect address matches.
    pub perfect_filtering: PerfectDestinationAddressFilteringMode,
    /// Enable or disable hash table filtering for destination
    /// addresses.
    pub hash_table_filtering: bool,
}

impl DestinationAddressFiltering {
    /// Create a new [`DestinationAddressFiltering`] that does
    /// not filter any frames.
    pub const fn new() -> Self {
        Self {
            perfect_filtering: PerfectDestinationAddressFilteringMode::new(),
            hash_table_filtering: false,
        }
    }
}

impl Default for DestinationAddressFiltering {
    fn default() -> Self {
        Self::new()
    }
}

/// The type of destination address filtering that
/// the MAC should apply to frames.
#[derive(Debug, Clone)]

pub enum PerfectDestinationAddressFilteringMode {
    /// Filter frames by their Destination Address, based on
    /// the addresses configured with [`AddressFilterType::Destination`].
    ///
    /// [`AddressFilterType::Destination`]: `super::AddressFilterType::Destination`
    Normal,
    /// Filter frames by their Destination Address, based on
    /// the inverse of the addresses configured with
    /// [`AddressFilterType::Destination`].
    ///
    /// [`AddressFilterType::Destination`]: `super::AddressFilterType::Destination`
    Inverse,
}

impl PerfectDestinationAddressFilteringMode {
    /// Create a new [`PerfectDestinationAddressFilteringMode`] that filters
    /// out all frames.
    pub const fn new() -> Self {
        Self::Normal
    }
}

impl Default for PerfectDestinationAddressFilteringMode {
    fn default() -> Self {
        Self::new()
    }
}
