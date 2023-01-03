#[derive(Debug, Clone)]
/// Configuration of the hash table used by the MAC for
/// destination address filtering.
pub struct HashTableValue {
    /// The low register of the hash table.
    pub low: u32,
    /// The high register of the hash table.
    pub high: u32,
}

impl HashTableValue {
    /// Create a new hash table value that filters
    /// out all frames.
    pub const fn new() -> Self {
        Self { low: 0, high: 0 }
    }
}

/// By default, the hash table is configured so that
/// it filters out all frames.
impl Default for HashTableValue {
    fn default() -> Self {
        Self::new()
    }
}
