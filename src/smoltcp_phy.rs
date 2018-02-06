use smoltcp::Error;
use smoltcp::phy::{DeviceCapabilities, Device, RxToken, TxToken};
use buffer::Buffer;
use super::Eth;

use core::fmt::Write;
use cortex_m_semihosting::hio;

impl<'a> Device<'a> for Eth {
    type RxToken = EthRxToken;
    type TxToken = EthTxToken<'a>;

    fn capabilities(&self) -> DeviceCapabilities {
        DeviceCapabilities::default()
    }

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        match self.recv_next() {
            Some(buffer) => {
                let rx = EthRxToken { buffer };
                let tx = EthTxToken { eth: self };
                Some((rx, tx))
            },
            None => None,
        }
    }

    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        Some(EthTxToken {
            eth: self,
        })
    }
}

pub struct EthRxToken {
    buffer: Buffer,
}

impl RxToken for EthRxToken {
    fn consume<R, F>(self, _timestamp: u64, f: F) -> Result<R, Error>
        where F: FnOnce(&[u8]) -> Result<R, Error>
    {
        f(self.buffer.as_slice())
    }
}

pub struct EthTxToken<'a> {
    eth: &'a mut Eth,
}

impl<'a> TxToken for EthTxToken<'a> {
    fn consume<R, F>(self, _timestamp: u64, len: usize, f: F) -> Result<R, Error>
        where F: FnOnce(&mut [u8]) -> Result<R, Error>
    {
        let mut buffer = Buffer::new(len);
        buffer.set_len(len);
        let result = f(buffer.as_mut_slice());
        self.eth.send(buffer);
        result
    }
}
