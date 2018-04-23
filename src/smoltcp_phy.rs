use core::ops::Deref;
use core::cell::RefCell;
use smoltcp::Error;
use smoltcp::time::Instant;
use smoltcp::phy::{DeviceCapabilities, Device, RxToken, TxToken};
use Eth;
use rx::RxError;
use tx::TxError;


pub struct EthPhy<'rx, 'tx> {
    eth: RefCell<Eth<'rx, 'tx>>
}

impl<'rx, 'tx> EthPhy<'rx, 'tx> {
    pub fn new(eth: Eth<'rx, 'tx>) -> Self {
        EthPhy {
            eth: RefCell::new(eth)
        }
    }
}

/// Use this Ethernet driver with [smoltcp](https://github.com/m-labs/smoltcp)
impl<'a, 'rx: 'a, 'tx: 'a> Device<'a> for &'a mut EthPhy<'rx, 'tx> {
    type RxToken = EthRxToken<'a, 'rx, 'tx>;
    type TxToken = EthTxToken<'a, 'rx, 'tx>;

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities::default()
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let rx = EthRxToken { phy: self };
        let tx = EthTxToken { phy: self };
        Some((rx, tx))
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(EthTxToken {
            phy: self,
        })
    }
}

pub struct EthRxToken<'a, 'rx: 'a, 'tx: 'a> {
    phy: &'a EthPhy<'rx, 'tx>,
}

impl<'a, 'rx, 'tx> RxToken for EthRxToken<'a, 'rx, 'tx> {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> Result<R, Error>
        where F: FnOnce(&[u8]) -> Result<R, Error>
    {
        let mut eth = self.phy.eth.borrow_mut();
        let result = match eth.recv_next() {
            Ok(packet) => {
                let result = f(packet.deref());
                packet.free();
                result
            },
            Err(RxError::WouldBlock) =>
                Err(Error::Exhausted),
            Err(RxError::Truncated) =>
                Err(Error::Truncated),
            Err(_) =>
                Err(Error::Dropped),
        };
        result
    }
}

/// Just a reference to [`Eth`](../struct.Eth.html) for sending a
/// packet later with [`consume()`](#method.consume)
pub struct EthTxToken<'a, 'rx: 'a, 'tx: 'a> {
    phy: &'a EthPhy<'rx, 'tx>,
}

impl<'a, 'rx, 'tx> TxToken for EthTxToken<'a, 'rx, 'tx> {
    /// Allocate a [`Buffer`](../struct.Buffer.html), yield with
    /// `f(buffer)`, and send it as an Ethernet packet.
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R, Error>
        where F: FnOnce(&mut [u8]) -> Result<R, Error>
    {
        let mut eth = self.phy.eth.borrow_mut();
        match eth.send(len, f) {
            Err(TxError::WouldBlock) =>
                Err(Error::Exhausted),
            Ok(r) => r,
        }
    }
}
