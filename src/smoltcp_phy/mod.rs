mod smoltcp_phy;
pub use smoltcp_phy::{EthRxToken, EthTxToken};

mod smoltcp_ptp_phy;
pub use smoltcp_ptp_phy::EthernetPTPDMA;
