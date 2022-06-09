use crate::PacketId;
use crate::{rx::RxPacket, tx::TxError, EthernetDMA};
use core::intrinsics::transmute;
use core::num::NonZeroUsize;
use smoltcp::phy::{ChecksumCapabilities, Device, DeviceCapabilities, RxToken, TxToken};
use smoltcp::time::Instant;
use smoltcp::Error;

pub use smoltcp::phy::PacketId as SmolTcpPacketId;

impl From<&smoltcp::phy::PacketId> for PacketId {
    // Convert a usize to a NonZeroUsize, losslessly
    fn from(input: &smoltcp::phy::PacketId) -> Self {
        let input = input.id();
        let value = if input == 0 {
            // This should never collide: if we have packets with number `0`, other temporally close
            // packet numbers will be close to `0` or close to `usize::MAX`, so usize::MAX/2
            // should be the furthest we can get from any collisions
            NonZeroUsize::new(usize::MAX / 2).expect("Will be valid")
        } else {
            NonZeroUsize::new(input).expect("Will be valid")
        };
        Self(value)
    }
}

/// Use this Ethernet driver with [smoltcp](https://github.com/m-labs/smoltcp)
impl<'a, 'rx, 'tx, 'b, const TX_SIZE: usize> Device<'a> for &'b mut EthernetDMA<'rx, 'tx, TX_SIZE> {
    type RxToken = EthRxToken<'a>;
    type TxToken = EthTxToken<'a, TX_SIZE>;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = crate::MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::ignored();
        caps
    }

    fn receive(
        &mut self,
        tx_packet_id: Option<smoltcp::phy::PacketId>,
    ) -> Option<(Self::RxToken, Self::TxToken)> {
        let self_ = unsafe {
            // HACK: eliminate lifetimes
            transmute::<&mut EthernetDMA<'rx, 'tx, TX_SIZE>, &mut EthernetDMA<'a, 'a, TX_SIZE>>(
                *self,
            )
        };
        let eth = self_ as *mut EthernetDMA<'a, 'a, TX_SIZE>;
        match self_.recv_next() {
            Ok(packet) => {
                let rx = EthRxToken { packet };
                let tx = EthTxToken {
                    eth,
                    packet_id: tx_packet_id,
                };
                Some((rx, tx))
            }
            Err(_) => None,
        }
    }

    fn transmit(&mut self, packet_id: Option<smoltcp::phy::PacketId>) -> Option<Self::TxToken> {
        let eth = unsafe {
            transmute::<&mut EthernetDMA<'rx, 'tx, TX_SIZE>, &mut EthernetDMA<'a, 'a, TX_SIZE>>(
                *self,
            ) as *mut EthernetDMA<'a, 'a, TX_SIZE>
        };
        Some(EthTxToken { eth, packet_id })
    }
}

pub struct EthRxToken<'a> {
    pub(super) packet: RxPacket<'a>,
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
pub struct EthTxToken<'a, const TX_SIZE: usize> {
    pub(super) eth: *mut EthernetDMA<'a, 'a, TX_SIZE>,
    packet_id: Option<smoltcp::phy::PacketId>,
}

impl<'a, const TX_SIZE: usize> TxToken for EthTxToken<'a, TX_SIZE> {
    /// Allocate a [`Buffer`](../struct.Buffer.html), yield with
    /// `f(buffer)`, and send it as an Ethernet packet.
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> Result<R, Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, Error>,
    {
        let eth = unsafe { &mut *self.eth };
        match eth.send(len, self.packet_id.as_ref().map(|v| v.into()), f) {
            Err(TxError::WouldBlock) => Err(Error::Exhausted),
            Ok(r) => r,
        }
    }
}
