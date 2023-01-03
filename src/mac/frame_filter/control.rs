/// The type of frame filtering that the MAC should perform
/// on received control frames,
#[derive(Debug, Clone)]

pub enum ControlFrameFilter {
    /// Prevent all control frames from reaching
    /// the application.
    BlockAll,
    /// Allow all control frames to reach the application,
    /// except for PAUSE control frames.
    NoPause,
    /// Allow all control frames to reach the application.
    AllowAll,
    /// Block control frames that do not pass the
    /// configured address filter, but allow those that do.
    AddressFilter,
}

impl ControlFrameFilter {
    /// Create a new [`ControlFrameFilter`] that filters out
    /// all control frames.
    pub const fn new() -> Self {
        Self::BlockAll
    }
}

impl Default for ControlFrameFilter {
    fn default() -> Self {
        Self::new()
    }
}
