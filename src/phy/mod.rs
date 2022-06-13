#[cfg(feature = "phy-lan87xxa")]
mod lan87xxa;
#[cfg(feature = "phy-lan87xxa")]
pub use lan87xxa::*;

pub trait Phy {
    type LinkSpeed;

    /// Reset the PHY
    ///
    // TODO: describe what "reset" actually is
    fn reset(&mut self);

    /// Initialize the PHY
    ///
    // TODO: describe what "init" actually is
    fn init(&mut self);

    /// Poll the link status of this PHY. If it returns `true`, the
    /// link is up. If it returns `false`, the link is down.
    /// 
    // TODO: return an associated type with link information?
    fn poll_link(&mut self) -> bool;

    /// Get the link speed reported by the PHY
    fn link_speed(&mut self) -> Self::LinkSpeed;

    /// Check whether the PHY reports that a link is established
    fn link_established(&mut self) -> bool {
        self.poll_link()
    }
}
