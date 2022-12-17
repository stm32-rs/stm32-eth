use heapless::Vec;

use super::MacAddressFilter;

/// The type of frame filtering that the MAC should perform
/// on received frames.
#[derive(Debug, Clone)]

pub enum SourceAddressFiltering {
    /// Source address filtering never fails.
    Ignore,
    /// Filter frames by their Source Address, based on
    /// the provided addresses.
    Normal(Vec<MacAddressFilter, 3>),
    /// Filter frames by their Source Address, based on
    /// the inverse of the provided addresses.
    Inverse(Vec<MacAddressFilter, 3>),
}

impl SourceAddressFiltering {
    /// Create a new [`SourceAddressFiltering`] that
    /// does not filter any frames.
    pub const fn new() -> Self {
        Self::Inverse(Vec::new())
    }
}

impl Default for SourceAddressFiltering {
    fn default() -> Self {
        Self::new()
    }
}
