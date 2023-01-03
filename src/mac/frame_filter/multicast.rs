/// Multicast address filtering
#[derive(Debug, Clone)]
pub enum MulticastAddressFilter {
    /// All received multicast frames are passed to the
    /// application.
    PassAll,
    /// Only multicast frames whose destination address
    /// passes the hash table check are passed to the
    /// application.
    DestinationAddressHash,
    /// Only multicast frames whose destination address
    /// is equal to one of the provided destination addresses
    /// are passed to the application.
    DestinationAddress,
}

impl MulticastAddressFilter {
    /// Create a new MulticastAddressFiltering that does not
    /// filter any multicast frames.
    pub const fn new() -> Self {
        Self::PassAll
    }
}

impl Default for MulticastAddressFilter {
    fn default() -> Self {
        Self::new()
    }
}
