// A copy of the `ip.rs` example, but for the STM32F107.

#![no_std]
#![no_main]

use panic_rtt_target as _;

use cortex_m::asm;
use cortex_m_rt::{entry, exception};
use stm32_eth::{
    hal::gpio::GpioExt,
    hal::rcc::RccExt,
    stm32::{interrupt, CorePeripherals, Peripherals, SYST},
    RxDescriptor, TxDescriptor,
};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use core::fmt::Write;

use smoltcp::iface::{EthernetInterfaceBuilder, NeighborCache};
use smoltcp::socket::{SocketSet, TcpSocket, TcpSocketBuffer};
use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};
use stm32_eth::{Eth, EthPins, PhyAddress, RingEntry};

use stm32f1xx_hal::{prelude::*, flash::FlashExt};
use rtt_target::{rprintln, rtt_init_print};

const SRC_MAC: [u8; 6] = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));
static ETH_PENDING: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // HCLK must be at least 25MHz to use the ethernet peripheral
    rprintln!("Setting up clocks");
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .freeze(&mut flash.acr);

    rprintln!("Setting up systick");
    setup_systick(&mut cp.SYST);

    //writeln!(stdout, "Enabling ethernet...").unwrap();
    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

    let ref_clk = gpioa.pa1.into_floating_input(&mut gpioa.crl);
    let md_io = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let crs = gpioa.pa7.into_floating_input(&mut gpioa.crl);
    let md_clk = gpioc.pc1.into_alternate_push_pull(&mut gpioc.crl);
    let tx_en = gpiob.pb11.into_alternate_push_pull(&mut gpiob.crh);
    let tx_d0 = gpiob.pb12.into_alternate_push_pull(&mut gpiob.crh);
    let tx_d1 = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
    let rx_d0 = gpioc.pc4.into_floating_input(&mut gpioc.crl);
    let rx_d1 = gpioc.pc5.into_floating_input(&mut gpioc.crl);

    let eth_pins = EthPins {
        ref_clk,
        md_io,
        md_clk,
        crs,
        tx_en,
        tx_d0,
        tx_d1,
        rx_d0,
        rx_d1,
    };

    rprintln!("Constructing `Eth`");
    let mut rx_ring: [RingEntry<_>; 8] = [
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
        RingEntry::<RxDescriptor>::new(),
    ];
    let mut tx_ring: [RingEntry<_>; 2] = [
        RingEntry::<TxDescriptor>::new(),
        RingEntry::<TxDescriptor>::new(),
    ];
    let mut eth = Eth::new(
        p.ETHERNET_MAC,
        p.ETHERNET_DMA,
        &mut rx_ring[..],
        &mut tx_ring[..],
        PhyAddress::_0,
        clocks,
        eth_pins,
    )
    .unwrap();
    eth.enable_interrupt();

    rprintln!("Setting up TCP/IP");
    let local_addr = Ipv4Address::new(10, 101, 0, 1);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 16);
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
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let server_handle = sockets.add(server_socket);

    rprintln!("Ready, listening at {}", ip_addr);
    loop {
        let time: u64 = cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow());
        cortex_m::interrupt::free(|cs| {
            let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
            *eth_pending = false;
        });
        match iface.poll(&mut sockets, Instant::from_millis(time as i64)) {
            Ok(true) => {
                let mut socket = sockets.get::<TcpSocket>(server_handle);
                if !socket.is_open() {
                    socket
                        .listen(80)
                        .unwrap_or_else(|e| rprintln!("TCP listen error: {:?}", e));
                }

                if socket.can_send() {
                    write!(socket, "hello\n")
                        .map(|_| {
                            socket.close();
                        })
                        .unwrap_or_else(|e| rprintln!("TCP send error: {:?}", e));
                }
            }
            Ok(false) => {
                // Sleep if no ethernet work is pending
                cortex_m::interrupt::free(|cs| {
                    let eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
                    if !*eth_pending {
                        // Awaken by interrupt
                        asm::wfi();
                    }
                });
            }
            Err(e) =>
            // Ignore malformed packets
            {
                rprintln!("Error: {:?}", e);
            }
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
        let mut time = TIME.borrow(cs).borrow_mut();
        *time += 1;
    })
}

#[interrupt]
fn ETH() {
    cortex_m::interrupt::free(|cs| {
        let mut eth_pending = ETH_PENDING.borrow(cs).borrow_mut();
        *eth_pending = true;
    });

    // Clear interrupt flags
    let p = unsafe { Peripherals::steal() };
    stm32_eth::eth_interrupt_handler(&p.ETHERNET_DMA);
}
