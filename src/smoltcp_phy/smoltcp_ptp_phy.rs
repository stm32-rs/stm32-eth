use crate::tx::TxDescriptorHandle;
use crate::{rx::RxPacket, EthernetDMA};
use crate::{EthRxToken, EthTxToken};
use core::convert::TryInto;
use core::intrinsics::transmute;

pub use smoltcp::{
    iface,
    phy::{Device, DeviceCapabilities},
    time::Instant,
    wire::{
        self, HardwareAddress,
        IpAddress::{self, *},
        Ipv4Address,
    },
};

pub struct EthernetPTPDMA<'rx, 'tx> {
    pub(crate) local_address: Option<(IpAddress, u16)>,
    pub(crate) dma: EthernetDMA<'rx, 'tx>,
}

impl<'rx, 'tx> EthernetPTPDMA<'rx, 'tx> {
    pub fn inner_mut(&mut self) -> &mut EthernetDMA<'rx, 'tx> {
        &mut self.dma
    }

    pub fn set_local_address(&mut self, address: IpAddress, port: u16) {
        self.local_address = Some((address, port));
    }

    fn try_parse(&mut self, packet: &RxPacket) -> Result<(), ()> {
        macro_rules! u {
            ($v:expr) => {
                $v.map_err(|_| ())?
            };
        }

        let (address, port) = if let Some(addr) = self.local_address {
            addr
        } else {
            return Err(());
        };

        let frame = u!(wire::EthernetFrame::new_checked(&**packet));
        let ethernet_parsed = u!(wire::EthernetRepr::parse(&frame));

        if ethernet_parsed.ethertype == wire::EthernetProtocol::Ipv4 {
            let ipv4_packet = u!(wire::Ipv4Packet::new_checked(frame.payload()));
            let checksum_cap = self.capabilities().checksum;
            let ip_parsed = u!(wire::Ipv4Repr::parse(&ipv4_packet, &checksum_cap));

            if IpAddress::Ipv4(ip_parsed.dst_addr) == address {
                let udp_packet = u!(wire::UdpPacket::new_checked(ipv4_packet.payload()));
                let udp_parsed = u!(wire::UdpRepr::parse(
                    &udp_packet,
                    &ip_parsed.src_addr.into(),
                    &ip_parsed.dst_addr.into(),
                    &checksum_cap
                ));
                if udp_parsed.dst_port == port {
                    todo!("Received a packet that should be timestamped")
                }
            }
        }

        Err(())
    }

    pub fn send_timestamped<'a>(
        &mut self,
        timestamp: Instant,
        neighbor_cache: &iface::NeighborCache,
        source_hw_address: wire::EthernetAddress,
        dest_address: IpAddress,
        data: &[u8],
    ) -> Option<TxDescriptorHandle> {
        let (addr, port) = self
            .local_address
            .expect("Local address for timestamped sends not set.");

        let dest_hw_addr = if let iface::NeighborAnswer::Found(HardwareAddress::Ethernet(addr)) =
            neighbor_cache.lookup(&dest_address, timestamp)
        {
            addr
        } else {
            defmt::error!("No route");
            return None;
        };

        let data_len: u16 = if let Ok(data_len) = data.len().try_into() {
            // TODO: determine if data_len will fit in MTU
            data_len
        } else {
            defmt::error!("Data len does not fit in u16");
            return None;
        };

        let result = if let (Ipv4(source_address_v4), Ipv4(dest_address_v4)) = (addr, dest_address)
        {
            let ip_header_len = 20;
            let total_len = wire::ETHERNET_HEADER_LEN as u16
                + ip_header_len
                + wire::UDP_HEADER_LEN as u16
                + data_len;

            self.dma.send(total_len as usize, true, |buffer| {
                let mut frame = wire::EthernetFrame::new_unchecked(buffer);
                frame.set_src_addr(source_hw_address);
                frame.set_dst_addr(dest_hw_addr);
                frame.set_ethertype(wire::EthernetProtocol::Ipv4);

                let mut ipv4_packet = wire::Ipv4Packet::new_unchecked(frame.payload_mut());
                ipv4_packet.set_header_len(16);
                ipv4_packet.set_total_len(ip_header_len + wire::UDP_HEADER_LEN as u16 + data_len);
                ipv4_packet.set_version(0x04);
                ipv4_packet.set_dst_addr(dest_address_v4);
                ipv4_packet.set_src_addr(source_address_v4);
                ipv4_packet.set_dont_frag(true);
                ipv4_packet.set_more_frags(false);
                ipv4_packet.set_frag_offset(0);
                ipv4_packet.set_next_header(wire::IpProtocol::Udp);
                ipv4_packet.set_hop_limit(64);
                ipv4_packet.fill_checksum();

                // TODO: set more fields?

                let payload = ipv4_packet.payload_mut();
                let mut udp_packet = wire::UdpPacket::new_unchecked(payload);
                udp_packet.set_src_port(port);
                udp_packet.set_dst_port(port);

                udp_packet.set_len((wire::UDP_HEADER_LEN + data.len()) as u16);
                udp_packet.payload_mut()[..data.len()].copy_from_slice(data);
                udp_packet.fill_checksum(&addr, &dest_address);
            })
        } else {
            return None;
        };

        let result = result.unwrap().1;

        Some(result)
    }
}

/// Use this Ethernet driver with [smoltcp](https://github.com/m-labs/smoltcp), and
/// support PTP capabilities by hijacking parts of the smoltcp device
///
// IMPLEMENTATION NOTE: this implementation _must_ mirror the implementation of
// [`smoltcp::Device`] for [`EthernetDMA`] as closely as possible
impl<'a, 'rx, 'tx, 'b> Device<'a> for &'b mut EthernetPTPDMA<'rx, 'tx> {
    type RxToken = EthRxToken<'a>;
    type TxToken = EthTxToken<'a>;

    fn capabilities(&self) -> DeviceCapabilities {
        // IMPORTANT: mirror this from [`EthernetDMA::capabilities`]
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = crate::MTU;
        caps.max_burst_size = Some(1);
        caps
    }

    fn receive(&mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        let self_ = unsafe {
            // HACK: eliminate lifetimes
            transmute::<&mut EthernetDMA<'rx, 'tx>, &mut EthernetDMA<'a, 'a>>(&mut self.dma)
        };
        let eth = self_ as *mut EthernetDMA<'a, 'a>;
        match self_.recv_next() {
            Ok(packet) => {
                self.try_parse(&packet).ok();
                let rx = EthRxToken { packet };
                let tx = EthTxToken { eth };
                Some((rx, tx))
            }
            Err(_) => None,
        }
    }

    fn transmit(&mut self) -> Option<Self::TxToken> {
        let eth = unsafe {
            // HACK: eliminate lifetimes
            transmute::<&mut EthernetDMA<'rx, 'tx>, &mut EthernetDMA<'a, 'a>>(&mut self.dma)
        };
        Some(EthTxToken { eth })
    }
}
