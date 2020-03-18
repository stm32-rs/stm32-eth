#![no_std]
#![no_main]

extern crate panic_itm;

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use stm32f7xx_hal::{
    gpio::GpioExt,
    device::{Peripherals, CorePeripherals, SYST, interrupt},
};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use core::fmt::Write;
use cortex_m_semihosting::hio;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr,
                    Ipv4Address};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use log::{Record, Level, Metadata, LevelFilter};

use stm32_eth::{Eth, RingEntry};

static mut LOGGER: HioLogger = HioLogger {};

struct HioLogger {}

impl log::Log for HioLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut stdout = hio::hstdout().unwrap();
            writeln!(stdout, "{} - {}", record.level(), record.args())
                .unwrap();
        }
    }
    fn flush(&self) {}
}

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    unsafe { log::set_logger(&LOGGER).unwrap(); }
    log::set_max_level(LevelFilter::Info);
    
    let mut stdout = hio::hstdout().unwrap();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    writeln!(stdout, "Enabling ethernet...").unwrap();
    stm32_eth::setup(&p.RCC, &p.SYSCFG);
    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    stm32_eth::setup_pins(
        gpioa.pa1, gpioa.pa2, gpioa.pa7, gpiob.pb13, gpioc.pc1,
        gpioc.pc4, gpioc.pc5, gpiog.pg11, gpiog.pg13
    );

    let mut rx_ring: [RingEntry<_>; 8] = Default::default();
    let mut tx_ring: [RingEntry<_>; 2] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );
    eth.enable_interrupt(&mut cp.NVIC);

    let local_addr = Ipv4Address::new(10, 0, 0, 1);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let ethernet_addr = EthernetAddress(SRC_MAC);
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();

    let mut server_rx_buffer = [0; 2048];
    let mut server_tx_buffer = [0; 2048];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let server_handle = sockets.add(server_socket);

    writeln!(stdout, "Ready, listening at {}", ip_addr).unwrap();
    loop {
        let time: u64 = cortex_m::interrupt::free(|cs| {
            *TIME.borrow(cs)
                .borrow()
        });
        cortex_m::interrupt::free(|cs| {
            let mut eth_pending =
                ETH_PENDING.borrow(cs)
                .borrow_mut();
            *eth_pending = false;
        });
        match iface.poll(&mut sockets, Instant::from_millis(time as i64)) {
            Ok(true) => {
                let mut socket = sockets.get::<TcpSocket>(server_handle);
                if !socket.is_open() {
                    socket.listen(80)
                        .or_else(|e| {
                            writeln!(stdout, "TCP listen error: {:?}", e)
                        })
                        .unwrap();
                }

                if socket.can_send() {
                    write!(socket, "hello\n")
                        .map(|_| {
                            socket.close();
                        })
                        .or_else(|e| {
                            writeln!(stdout, "TCP send error: {:?}", e)
                        })
                        .unwrap();
                }
            },
            Ok(false) => {
                // Sleep if no ethernet work is pending
                cortex_m::interrupt::free(|cs| {
                    let eth_pending =
                        ETH_PENDING.borrow(cs)
                        .borrow_mut();
                    if ! *eth_pending {
                        asm::wfi();
                        // Awaken by interrupt
                    }
                });
            },
            Err(e) =>
                // Ignore malformed packets
                writeln!(stdout, "Error: {:?}", e).unwrap(),
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| {
        let mut time =
            TIME.borrow(cs)
            .borrow_mut();
        *time += 1;
    })
}

#[interrupt]
fn ETH() {
    cortex_m::interrupt::free(|cs| {
        let mut eth_pending =
            ETH_PENDING.borrow(cs)
            .borrow_mut();
        *eth_pending = true;
    });

    // Clear interrupt flags
    let p = unsafe { Peripherals::steal() };
    stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);
}
