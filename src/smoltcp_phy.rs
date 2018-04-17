use core::ops::Deref;
use smoltcp::Error;
use smoltcp::time::Instant;
use smoltcp::phy::{DeviceCapabilities, Device, RxToken, TxToken};
use Eth;
use rx::{RxPacket, RxError};
use tx::TxError;


/// Use this Ethernet driver with [smoltcp](https://github.com/m-labs/smoltcp)
impl<'a: 'rx + 'tx, 'rx, 'tx> Device<'a> for Eth<'rx, 'tx> {
    type RxToken = EthRxToken<'a>;
    type TxToken = EthTxToken<'a>;

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities::default()
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let rx = match self.recv_next() {
            Ok(packet) =>
                EthRxToken { packet },
            Err(_) =>
                // Silently dropping errors, TODO: log?
                return None,
        };

        let tx = EthTxToken { eth: self };
        Some((rx, tx))
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(EthTxToken {
            eth: self,
        })
    }
}

pub struct EthRxToken<'a> {
    packet: RxPacket<'a>
}

impl<'a> RxToken for EthRxToken<'a> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R, Error>
        where F: FnOnce(&[u8]) -> Result<R, Error>
    {
        f(self.packet.deref())
    }
}

/// Just a reference to [`Eth`](../struct.Eth.html) for sending a
/// packet later with [`consume()`](#method.consume)
pub struct EthTxToken<'a> {
    eth: &'a mut Eth<'a, 'a>,
}

impl<'a> TxToken for EthTxToken<'a> {
    /// Allocate a [`Buffer`](../struct.Buffer.html), yield with
    /// `f(buffer)`, and send it as an Ethernet packet.
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R, Error>
        where F: FnOnce(&mut [u8]) -> Result<R, Error>
    {
        match self.eth.send(len, f) {
            Err(TxError::WouldBlock) =>
                Err(Error::Exhausted),
            Ok(r) => r,
        }
    }
}
