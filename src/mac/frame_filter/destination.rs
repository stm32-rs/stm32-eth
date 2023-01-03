/// The type of destination address filtering that
/// the MAC should apply to incoming frames.
#[derive(Debug, Clone)]
pub struct DaFilter {
    /// Filtering to be performed based on perfect address matches.
    pub perfect_filtering: PerfectDaFilterMode,
    /// Enable or disable hash table filtering for destination
    /// addresses.
    pub hash_table_filtering: bool,
}

impl DaFilter {
    /// Create a new [`DaFilter`] that does
    /// not filter any frames.
    pub const fn new() -> Self {
        Self {
            perfect_filtering: PerfectDaFilterMode::new(),
            hash_table_filtering: false,
        }
    }
}

impl Default for DaFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// The type of destination address filtering that
/// the MAC should apply to incoming frames.
#[derive(Debug, Clone)]

pub enum PerfectDaFilterMode {
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

impl PerfectDaFilterMode {
    /// Create a new [`PerfectDaFilterMode`] that filters
    /// out all frames.
    pub const fn new() -> Self {
        Self::Normal
    }
}

impl Default for PerfectDaFilterMode {
    fn default() -> Self {
        Self::new()
    }
}
