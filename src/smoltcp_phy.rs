use crate::{rx::RxPacket, tx::TxError, EthernetDMA};
use core::intrinsics::transmute;
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::time::Instant;
use smoltcp::Error;

/// Use this Ethernet driver with [smoltcp](https://github.com/smoltcp-rs/smoltcp)
impl<'a, 'rx, 'tx, 'b> Device<'a> for &'b mut EthernetDMA<'rx, 'tx> {
    type RxToken = EthRxToken<'a>;
    type TxToken = EthTxToken<'a>;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = super::MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::ignored();
        caps
    }

    fn receive(&mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let self_ = unsafe {
            // HACK: eliminate lifetimes
            transmute::<&mut EthernetDMA<'rx, 'tx>, &mut EthernetDMA<'a, 'a>>(*self)
        };
        let eth = self_ as *mut EthernetDMA<'a, 'a>;
        match self_.recv_next() {
            Ok(packet) => {
                let rx = EthRxToken { packet };
                let tx = EthTxToken { eth };
                Some((rx, tx))
            }
            Err(_) => None,
        }
    }

    fn transmit(&mut self) -> Option<Self::TxToken> {
        let eth = unsafe {
            transmute::<&mut EthernetDMA<'rx, 'tx>, &mut EthernetDMA<'a, 'a>>(*self)
                as *mut EthernetDMA<'a, 'a>
        };
        Some(EthTxToken { eth })
    }
}

/// An Ethernet RX token that can be consumed in order to receive
/// an ethernet packet.
pub struct EthRxToken<'a> {
    packet: RxPacket<'a>,
}

impl<'a> RxToken for EthRxToken<'a> {
    fn consume<R, F>(mut self, _timestamp: Instant, f: F) -> Result<R, Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, Error>,
    {
        let result = f(&mut self.packet);
        self.packet.free();
        result
    }
}

/// Just a reference to [`Eth`](../struct.EthernetDMA.html) for sending a
/// packet later with [`consume()`](#method.consume).
pub struct EthTxToken<'a> {
    eth: *mut EthernetDMA<'a, 'a>,
}

impl<'a> TxToken for EthTxToken<'a> {
    /// Allocate a [`Buffer`](../struct.Buffer.html), yield with
    /// `f(buffer)`, and send it as an Ethernet packet.
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R, Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, Error>,
    {
        let eth = unsafe { &mut *self.eth };
        match eth.send(len, f) {
            Err(TxError::WouldBlock) => Err(Error::Exhausted),
            Ok(r) => r,
        }
    }
}
