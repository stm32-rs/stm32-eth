/// The type of frame filtering that the MAC should perform
/// on received frames.
#[derive(Debug, Clone)]

pub enum SourceAddressFiltering {
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

impl SourceAddressFiltering {
    /// Create a new [`SourceAddressFiltering`] that
    /// does not filter any frames.
    pub const fn new() -> Self {
        Self::Inverse
    }
}

impl Default for SourceAddressFiltering {
    fn default() -> Self {
        Self::new()
    }
}
