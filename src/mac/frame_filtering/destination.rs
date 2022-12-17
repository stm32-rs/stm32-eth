use heapless::Vec;

use super::MacAddressFilter;

/// The type of filtering that the MAC should apply to
/// frames that are to be transmitted.
#[derive(Debug, Clone)]
pub struct DestinationAddressFiltering {
    /// Filtering to be performed based on perfect address matches.
    pub perfect_filtering: PerfectDestinationAddressFiltering,
    /// Enable or disable hash table filtering for destination
    /// addresses.
    pub hash_table_filtering: bool,
}

impl DestinationAddressFiltering {
    /// Create a new [`DestinationAddressFiltering`] that does
    /// not filter any frames.
    pub const fn new() -> Self {
        Self {
            perfect_filtering: PerfectDestinationAddressFiltering::new(),
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

pub enum PerfectDestinationAddressFiltering {
    /// Filter frames by their Destination Address, based on
    /// the provided addresses.
    Normal(Vec<MacAddressFilter, 3>),
    /// Filter frames by their Destination Address, based on
    /// the inverse of the provided addresses.
    Inverse(Vec<MacAddressFilter, 3>),
}

impl PerfectDestinationAddressFiltering {
    /// Create a new [`PerfectDestinationAddressFiltering`] that filters
    /// out all frames.
    pub const fn new() -> Self {
        Self::Normal(Vec::new())
    }
}

impl Default for PerfectDestinationAddressFiltering {
    fn default() -> Self {
        Self::new()
    }
}
