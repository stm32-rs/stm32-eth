#![no_std]
#![no_main]

extern crate panic_itm;

use cortex_m::interrupt;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::{
    gpio::GpioExt,
    stm32::{Peripherals, CorePeripherals, SYST, TIM2},
    time::U32Ext,
    rcc::RccExt,
};

use core::cell::Cell;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, Ipv4Address, IpCidr, IpEndpoint};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder, Routes};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;

use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry};

const SELF_IP: IpAddress = IpAddress::Ipv4(Ipv4Address([192, 168, 1, 100]));
const REMOTE_IP: IpAddress = IpAddress::Ipv4(Ipv4Address([192, 168, 1, 1]));
const PORT: u16 = 54321;
const PKT_SIZE: usize = 1446;
// Sends a packet every N 10 MHz ticks.
// With 1240, the problem disappears.
const PKT_EVERY: u32 = 1237;

static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_10mhz(&p);

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(180.mhz()).hclk(180.mhz()).freeze();

    setup_systick(&mut cp.SYST);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    let pins = EthPins {
        ref_clk: gpioa.pa1,
        md_io: gpioa.pa2,
        md_clk: gpioc.pc1,
        crs: gpioa.pa7,
        tx_en: gpiog.pg11,
        tx_d0: gpiog.pg13,
        tx_d1: gpiob.pb13,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };

    // set up ring buffers for network handling tokens
    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 16] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..],
        PhyAddress::_0,
        clocks,
        pins
    ).unwrap();

    let ethernet_addr = EthernetAddress([0x46, 0x52, 0x4d, 0x02, 0x02, 0x02]);
    let mut ip_addrs = [IpCidr::new(SELF_IP, 24)];
    let mut neighbor_storage = [None; 16];
    let mut routes_storage = [None; 2];
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(NeighborCache::new(&mut neighbor_storage[..]))
        .routes(Routes::new(&mut routes_storage[..]))
        .finalize();

    // set up buffers for packet content and metadata
    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 16];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*16];

    // create the UDP socket
    let udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    let udp_handle = sockets.add(udp_socket);

    let mut gen = Generator::new(p.TIM2);
    sockets.get::<UdpSocket>(udp_handle).bind((SELF_IP, PORT)).unwrap();

    gen.start();

    loop {
        // process packets
        {
            let mut socket = sockets.get::<UdpSocket>(udp_handle);
            gen.maybe_send_data(&mut socket);
        }
        // handle ethernet
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        if let Err(_e) = iface.poll(&mut sockets, time) {
            //warn!("poll: {}", e);
        }
    }
}

struct Generator {
    endpoint: IpEndpoint,
    timer: TIM2,
    buf_no: u32,
    run: bool,
    time: u64,
    lastpkt: u64,
}

impl Generator {
    fn new(timer: TIM2) -> Self {
        Generator { timer, endpoint: (REMOTE_IP, PORT).into(),
                    buf_no: 0, time: 0, lastpkt: 0, run: false }
    }

    fn maybe_send_data(&mut self, sock: &mut UdpSocket) {
        // keep track of 64-bit time
        let low_time = self.timer.cnt.read().bits();
        let overflow = if low_time < self.time as u32 { 1 << 32 } else { 0 };
        self.time = ((self.time & 0xFFFF_FFFF_0000_0000) + overflow) | low_time as u64;

        // calculate time since last packet
        let elapsed = (self.time - self.lastpkt) as u32;
        if elapsed < PKT_EVERY {
            return;
        }

        // now we can send a packet
        match sock.send(PKT_SIZE, self.endpoint) {
            Ok(buf) => {
                self.buf_no += 1;
                buf[0] = self.buf_no as u8;
                buf[1] = (self.buf_no >> 8) as u8;
                buf[2] = (self.buf_no >> 16) as u8;
                buf[3] = (self.buf_no >> 24) as u8;
                // time
                buf[4] = self.time as u8;
                buf[5] = (self.time >> 8) as u8;
                buf[6] = (self.time >> 16) as u8;
                buf[7] = (self.time >> 24) as u8;
                buf[8] = (self.time >> 32) as u8;
                buf[9] = (self.time >> 40) as u8;
                buf[10] = (self.time >> 48) as u8;
                buf[11] = (self.time >> 56) as u8;
                self.lastpkt = self.time - (elapsed % PKT_EVERY) as u64;
            }
            Err(_e) => ()//warn!("send: {}", e),
        }
    }

    fn start(&mut self) {
        self.run = true;
        self.time = 0;
        self.lastpkt = 0;
        // reset the timer
        self.timer.cnt.write(|w| unsafe { w.bits(0) });
        self.timer.cr1.write(|w| w.cen().set_bit());
    }
}

fn setup_systick(syst: &mut SYST) {
    // systick is used for advancing the Ethernet clock for timeouts etc.
    syst.set_reload(22_500 - 1); // every ms
    syst.enable_counter();
    syst.enable_interrupt();
}

fn setup_10mhz(p: &Peripherals) {
    p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
    p.TIM2.psc.write(|w| w.psc().bits(8)); // 90 MHz/9
    p.TIM2.egr.write(|w| w.ug().set_bit());
}

#[exception]
fn SysTick() {
    interrupt::free(|cs| {
        let time = ETH_TIME.borrow(cs);
        time.set(time.get().wrapping_add(1));
    });
}
