/// The type of source address frame filtering that
/// the MAC should perform to received frames.
#[derive(Debug, Clone)]

pub enum SaFilter {
    /// Source address filtering never fails.
    Ignore,
    /// Filter frames by their Source Address, based on
    /// the addresses configured with [`AddressFilterType::Source`].
    ///
    /// [`AddressFilterType::Source`]: `super::AddressFilterType::Destination`
    Normal,
    /// Filter frames by their Source Address, based on
    /// the addresses configured with [`AddressFilterType::Source`].
    ///
    /// [`AddressFilterType::Source`]: `super::AddressFilterType::Destination`
    Inverse,
}

impl SaFilter {
    /// Create a new [`SaFilter`] that
    /// does not filter any frames.
    pub const fn new() -> Self {
        Self::Inverse
    }
}

impl Default for SaFilter {
    fn default() -> Self {
        Self::new()
    }
}
