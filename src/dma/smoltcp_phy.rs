use super::rx::RxRing;
use super::tx::TxRing;
use super::EthernetDMA;

#[cfg(feature = "ptp")]
use super::PacketId;

use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::time::Instant;

/// Use this Ethernet driver with [smoltcp](https://github.com/smoltcp-rs/smoltcp)
impl<'a, 'rx, 'tx> Device for &'a mut EthernetDMA<'rx, 'tx> {
    type RxToken<'token> = EthRxToken<'token, 'rx> where Self: 'token;
    type TxToken<'token> = EthTxToken<'token, 'tx> where Self: 'token;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = crate::dma::MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::ignored();
        caps
    }

    fn receive(&mut self, _timestamp: Instant) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.tx_available() && self.rx_available() {
            #[cfg(feature = "ptp")]
            let rx_packet_id = self.next_packet_id();

            let EthernetDMA {
                rx_ring, tx_ring, ..
            } = self;

            let rx = EthRxToken {
                rx_ring,
                #[cfg(feature = "ptp")]
                meta: rx_packet_id,
            };

            let tx = EthTxToken {
                tx_ring,
                #[cfg(feature = "ptp")]
                meta: None,
            };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        if self.tx_available() {
            let EthernetDMA { tx_ring, .. } = self;
            Some(EthTxToken {
                tx_ring,
                #[cfg(feature = "ptp")]
                meta: None,
            })
        } else {
            None
        }
    }
}

/// An Ethernet RX token that can be consumed in order to receive
/// an ethernet packet.
pub struct EthRxToken<'a, 'rx> {
    rx_ring: &'a mut RxRing<'rx>,
    #[cfg(feature = "ptp")]
    meta: PacketId,
}

impl<'dma, 'rx> RxToken for EthRxToken<'dma, 'rx> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        #[cfg(feature = "ptp")]
        let meta = Some(self.meta.into());

        #[cfg(not(feature = "ptp"))]
        let meta = None;

        // NOTE(unwrap): an `EthRxToken` is only created when `eth.rx_available()`
        let mut packet = self.rx_ring.recv_next(meta).ok().unwrap();
        let result = f(&mut packet);
        packet.free();
        result
    }

    #[cfg(feature = "ptp")]
    fn meta(&self) -> smoltcp::phy::PacketMeta {
        self.meta.clone().into()
    }
}

/// Just a reference to [`EthernetDMA`] for sending a
/// packet later with [`TxToken::consume()`].
pub struct EthTxToken<'a, 'tx> {
    tx_ring: &'a mut TxRing<'tx>,
    #[cfg(feature = "ptp")]
    meta: Option<PacketId>,
}

impl<'dma, 'tx> TxToken for EthTxToken<'dma, 'tx> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        #[cfg(feature = "ptp")]
        let meta = self.meta.map(Into::into);
        #[cfg(not(feature = "ptp"))]
        let meta = None;

        // NOTE(unwrap): an `EthTxToken` is only created if
        // there is a descriptor available for sending.
        let mut tx_packet = self.tx_ring.send_next(len, meta).ok().unwrap();
        let res = f(&mut tx_packet);
        tx_packet.send();
        res
    }

    #[cfg(feature = "ptp")]
    fn set_meta(&mut self, meta: smoltcp::phy::PacketMeta) {
        self.meta = Some(meta.into());
    }
}
