use super::rx::RxRing;
use super::tx::TxRing;
use super::EthernetDMA;
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
            let EthernetDMA {
                rx_ring, tx_ring, ..
            } = self;

            let rx = EthRxToken { rx_ring };

            let tx = EthTxToken { tx_ring };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        if self.tx_available() {
            let EthernetDMA { tx_ring, .. } = self;
            Some(EthTxToken { tx_ring })
        } else {
            None
        }
    }
}

/// An Ethernet RX token that can be consumed in order to receive
/// an ethernet packet.
pub struct EthRxToken<'a, 'rx> {
    rx_ring: &'a mut RxRing<'rx>,
}

impl<'dma, 'rx> RxToken for EthRxToken<'dma, 'rx> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // NOTE(unwrap): an `EthRxToken` is only created when `eth.rx_available()`
        let mut packet = self.rx_ring.recv_next(None).ok().unwrap();

        let result = f(&mut packet);
        packet.free();
        result
    }
}

/// Just a reference to [`EthernetDMA`] for sending a
/// packet later with [`TxToken::consume()`].
pub struct EthTxToken<'a, 'tx> {
    tx_ring: &'a mut TxRing<'tx>,
}

impl<'dma, 'tx> TxToken for EthTxToken<'dma, 'tx> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // NOTE(unwrap): an `EthTxToken` is only created if
        // there is a descriptor available for sending.
        let mut tx_packet = self.tx_ring.send_next(len, None).ok().unwrap();
        let res = f(&mut tx_packet);
        tx_packet.send();
        res
    }
}
