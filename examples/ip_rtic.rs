#![no_main]
#![no_std]

extern crate panic_itm;

use stm32_eth::{
    hal::gpio::GpioExt, hal::rcc::RccExt, hal::time::U32Ext, RxDescriptor, TxDescriptor,
};

use rtic::cyccnt::U32Ext as _;

use core::fmt::Write;
use cortex_m_semihosting::hio;

use log::{Level, LevelFilter, Metadata, Record};
use smoltcp::iface::{EthernetInterface, EthernetInterfaceBuilder, NeighborCache};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};

use rtic::export::DWT;
use stm32_eth::smoltcp::iface::Neighbor;
use stm32_eth::smoltcp::socket::{SocketHandle, SocketSetItem};
use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry};

static mut LOGGER: HioLogger = HioLogger {};

struct HioLogger {}

impl log::Log for HioLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut stdout = hio::hstdout().unwrap();
            writeln!(stdout, "{} - {}", record.level(), record.args()).unwrap();
        }
    }
    fn flush(&self) {}
}

const MS_PERIOD: u32 = 16_000;

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

type Ethernet = EthernetInterface<'static, 'static, 'static, &'static mut Eth<'static, 'static>>;
type Sockets = SocketSet<'static, 'static, 'static>;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(0)]
        time: u64,
        #[init(false)]
        eth_pending: bool,
        server_handle: SocketHandle,
        sockets: Sockets,
        interface: Ethernet,
    }

    #[init(schedule = [ms_tick])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut TX_RING: [RingEntry<TxDescriptor>; 2] = [
            RingEntry::<TxDescriptor>::new(),
            RingEntry::<TxDescriptor>::new(),
        ];
        static mut RX_RING: [RingEntry<RxDescriptor>; 8] = [
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
            RingEntry::<RxDescriptor>::new(),
        ];

        static mut IP_ADDRS: Option<[IpCidr; 1]> = None;

        static mut NEIGHBOR_STORAGE: [Option<(IpAddress, Neighbor)>; 16] = [None; 16];

        static mut ETH: Option<Eth<'static, 'static>> = None;

        static mut SOCKETS_STORAGE: [Option<SocketSetItem<'static, 'static>>; 2] = [None, None];
        static mut SERVER_RX_BUFFER: [u8; 2048] = [0; 2048];
        static mut SERVER_TX_BUFFER: [u8; 2048] = [0; 2048];

        unsafe {
            log::set_logger(&LOGGER).unwrap();
        }
        log::set_max_level(LevelFilter::Info);

        let mut core: rtic::Peripherals = cx.core;
        let device: stm32f4xx_hal::stm32::Peripherals = cx.device;

        // enable CYCCNT
        core.DCB.enable_trace();
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        let rcc = device.RCC.constrain();
        // HCLK must be at least 25MHz to use the ethernet peripheral
        let clocks = rcc.cfgr.sysclk(32.mhz()).hclk(32.mhz()).freeze();

        log::info!("Enabling ethernet...");
        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        let gpiog = device.GPIOG.split();

        let eth_pins = EthPins {
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

        let eth = Eth::new(
            device.ETHERNET_MAC,
            device.ETHERNET_DMA,
            RX_RING.as_mut(),
            TX_RING.as_mut(),
            PhyAddress::_0,
            clocks,
            eth_pins,
        )
        .unwrap();
        eth.enable_interrupt();

        let ip_addr = IpCidr::new(IpAddress::from(Ipv4Address::new(10, 0, 0, 1)), 24);
        *IP_ADDRS = Some([ip_addr]);
        *ETH = Some(eth);

        let interface = EthernetInterfaceBuilder::new(ETH.as_mut().unwrap())
            .ethernet_addr(EthernetAddress(SRC_MAC))
            .ip_addrs(IP_ADDRS.as_mut().unwrap().as_mut())
            .neighbor_cache(NeighborCache::new(NEIGHBOR_STORAGE.as_mut()))
            .finalize();

        let server_socket = TcpSocket::new(
            TcpSocketBuffer::new(SERVER_RX_BUFFER.as_mut()),
            TcpSocketBuffer::new(SERVER_TX_BUFFER.as_mut()),
        );

        let mut sockets = SocketSet::new(SOCKETS_STORAGE.as_mut());
        let server_handle = sockets.add(server_socket);

        log::info!("Ready, listening at {}", ip_addr);

        let now = cx.start;
        cx.schedule.ms_tick(now + MS_PERIOD.cycles()).unwrap();

        init::LateResources {
            server_handle,
            sockets,
            interface,
        }
    }

    #[idle(resources = [eth_pending, time, interface, sockets, server_handle])]
    fn idle(mut cx: idle::Context) -> ! {
        let pending: &mut bool = cx.resources.eth_pending;
        let interface: &mut Ethernet = cx.resources.interface;
        let sockets: &mut Sockets = cx.resources.sockets;
        let handle: &mut SocketHandle = cx.resources.server_handle;

        loop {
            let time: u64 = cx.resources.time.lock(|time| *time);
            *pending = false;
            match interface.poll(sockets, Instant::from_millis(time as i64)) {
                Ok(true) => {
                    let mut socket = sockets.get::<TcpSocket>(*handle);
                    if !socket.is_open() {
                        if let Err(e) = socket.listen(80) {
                            log::error!("TCP listen error: {:?}", e)
                        }
                    }

                    if socket.can_send() {
                        if let Err(e) = write!(socket, "hello\n").map(|_| {
                            socket.close();
                        }) {
                            log::error!("TCP send error: {:?}", e)
                        }
                    }
                }
                Ok(false) => {}
                Err(e) =>
                // Ignore malformed packets
                {
                    log::error!("Malformed packet.");
                }
            }
        }
    }

    #[task(binds = ETH, resources = [])]
    fn eth_handler(cx: eth_handler::Context) {
        // Clear interrupt flags
        let p = unsafe { stm32f4xx_hal::stm32::Peripherals::steal() };
        stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);
    }

    #[task(resources = [time], schedule = [ms_tick])]
    fn ms_tick(cx: ms_tick::Context) {
        *cx.resources.time += 1;

        cx.schedule
            .ms_tick(cx.scheduled + (MS_PERIOD).cycles())
            .unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
