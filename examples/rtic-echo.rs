#![no_std]
#![no_main]

//! A simple TCP echo server using RTIC.
//!
//! Starts a TCP echo server on port `1337` at `ADDRESS`. `ADDRESS` is `10.0.0.1/24` by default.
//!
//! By default, the example assumes that the default RMII pins are used.
//! To use it on an stm32-nucleo-f746zg dev board, the `rtic-echo-example-altpin` feature should be enabled. This may work on other
//! boards, but that hasn't been tested so your mileage may vary.
//!
//! To run this, install `probe-run` (`cargo install probe-run --version '~0.3'`), and ensure that `probe-run` can
//! attach to your test board.
//! Then, use the following command:
//! DEFMT_LOG=info PROBE_RUN_CHIP=<probe-run chip> cargo run --example rtic-echo --features <chip here>,rtic-echo-example --target <correct target for chip> --release

use defmt_rtt as _;
use panic_probe as _;

use smoltcp::{
    iface::{self, SocketStorage},
    wire::{self, IpAddress, Ipv4Address},
};

const ADDRESS: (IpAddress, u16) = (IpAddress::Ipv4(Ipv4Address::new(10, 0, 0, 1)), 1337);

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use systick_monotonic::Systick;

    use stm32_eth::{
        hal::{gpio::GpioExt, rcc::RccExt},
        EthernetDMA, RxRingEntry, TxRingEntry,
    };

    use fugit::RateExtU32;

    use smoltcp::{
        iface::{self, Interface, SocketHandle},
        socket::TcpSocket,
        socket::TcpSocketBuffer,
        wire::EthernetAddress,
    };

    use crate::NetworkStorage;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {
        #[lock_free]
        interface: Interface<'static, &'static mut EthernetDMA<'static, 'static>>,
        #[lock_free]
        tcp_handle: SocketHandle,
    }

    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    fn now_fn() -> smoltcp::time::Instant {
        let time = monotonics::now().duration_since_epoch().ticks();
        smoltcp::time::Instant::from_millis(time as i64)
    }

    #[init(local = [
        rx_ring: [RxRingEntry; 4] = [RxRingEntry::new(),RxRingEntry::new(),RxRingEntry::new(),RxRingEntry::new()],
        tx_ring: [TxRingEntry; 4] = [TxRingEntry::new(),TxRingEntry::new(),TxRingEntry::new(),TxRingEntry::new()],
        storage: NetworkStorage = NetworkStorage::new(),
        dma: core::mem::MaybeUninit<EthernetDMA<'static, 'static>> = core::mem::MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Pre-init");
        let core = cx.core;
        let p = cx.device;

        let rx_ring = cx.local.rx_ring;
        let tx_ring = cx.local.tx_ring;

        #[cfg(any(feature = "stm32f7xx-hal", feature = "stm32f4xx-hal"))]
        let (clocks, mono) = {
            let rcc = p.RCC.constrain();

            let clocks = rcc.cfgr.sysclk(100.MHz()).hclk(100.MHz()).freeze();

            let mono = Systick::new(core.SYST, clocks.hclk().raw());

            (clocks, mono)
        };

        #[cfg(feature = "stm32f1xx-hal")]
        let (clocks, mono) = {
            use stm32_eth::hal::flash::FlashExt;

            let rcc = p.RCC.constrain();
            let mut flash = p.FLASH.constrain();

            let clocks = rcc
                .cfgr
                .sysclk(32.MHz())
                .hclk(32.MHz())
                .freeze(&mut flash.acr);

            let mono = Systick::new(core.SYST, clocks.hclk().raw());

            (clocks, mono)
        };

        #[cfg(feature = "stm32f1xx-hal")]
        let gpiog = ();

        #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal"))]
        let gpiog = p.GPIOG.split();

        defmt::info!("Setting up pins");
        let (pins, mdio, mdc) =
            crate::pins::get_pins(p.GPIOA.split(), p.GPIOB.split(), p.GPIOC.split(), gpiog);

        defmt::info!("Configuring ethernet");

        let (dma, mac) = stm32_eth::new_with_mii(
            p.ETHERNET_MAC,
            p.ETHERNET_MMC,
            p.ETHERNET_DMA,
            rx_ring,
            tx_ring,
            clocks,
            pins,
            mdio,
            mdc,
        )
        .unwrap();

        let dma = cx.local.dma.write(dma);

        defmt::info!("Enabling interrupts");
        dma.enable_interrupt();

        defmt::info!("Setting up smoltcp");
        let store = cx.local.storage;

        let mut routes = smoltcp::iface::Routes::new(&mut store.routes_cache[..]);
        routes
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .ok();

        let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

        let mac_addr = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05];

        let rx_buffer = TcpSocketBuffer::new(&mut store.tcp_socket_storage.rx_storage[..]);
        let tx_buffer = TcpSocketBuffer::new(&mut store.tcp_socket_storage.tx_storage[..]);

        let socket = TcpSocket::new(rx_buffer, tx_buffer);

        let mut interface = iface::InterfaceBuilder::new(dma, &mut store.sockets[..])
            .hardware_addr(EthernetAddress::from_bytes(&mac_addr).into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();

        let tcp_handle = interface.add_socket(socket);

        let socket = interface.get_socket::<TcpSocket>(tcp_handle);
        socket.listen(crate::ADDRESS).ok();

        interface.poll(now_fn()).unwrap();

        if let Ok(mut phy) = crate::EthernetPhy::from_miim(mac, 0) {
            defmt::info!(
                "Resetting PHY as an extra step. Type: {}",
                phy.ident_string()
            );

            phy.phy_init();
        } else {
            defmt::info!("Not resetting unsupported PHY.");
        }

        defmt::info!("Setup done.");

        (
            Shared {
                interface,
                tcp_handle,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    #[task(binds = ETH, shared = [interface, tcp_handle], local = [data: [u8; 512] = [0u8; 512]], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (iface, tcp_handle, buffer) =
            (cx.shared.interface, cx.shared.tcp_handle, cx.local.data);

        let interrupt_reason = iface.device_mut().interrupt_handler();
        defmt::debug!("Got an ethernet interrupt! Reason: {}", interrupt_reason);

        iface.poll(now_fn()).ok();

        let socket = iface.get_socket::<TcpSocket>(*tcp_handle);
        if let Ok(recv_bytes) = socket.recv_slice(buffer) {
            if recv_bytes > 0 {
                socket.send_slice(&buffer[..recv_bytes]).ok();
                defmt::info!("Echoed {} bytes.", recv_bytes);
            }
        }

        if !socket.is_listening() && !socket.is_open() {
            socket.abort();
            socket.listen(crate::ADDRESS).ok();
            defmt::warn!("Disconnected... Reopening listening socket.");
        }

        iface.poll(now_fn()).ok();
    }
}

/// All storage required for networking
pub struct NetworkStorage {
    pub ip_addrs: [wire::IpCidr; 1],
    pub sockets: [iface::SocketStorage<'static>; 1],
    pub tcp_socket_storage: TcpSocketStorage,
    pub neighbor_cache: [Option<(wire::IpAddress, iface::Neighbor)>; 8],
    pub routes_cache: [Option<(wire::IpCidr, iface::Route)>; 8],
}

impl NetworkStorage {
    const IP_INIT: wire::IpCidr =
        wire::IpCidr::Ipv4(wire::Ipv4Cidr::new(wire::Ipv4Address::new(10, 0, 0, 1), 24));

    pub const fn new() -> Self {
        NetworkStorage {
            ip_addrs: [Self::IP_INIT],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [SocketStorage::EMPTY; 1],
            tcp_socket_storage: TcpSocketStorage::new(),
        }
    }
}

/// Storage of TCP sockets
#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx_storage: [u8; 512],
    tx_storage: [u8; 512],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 512],
            tx_storage: [0; 512],
        }
    }
}

mod pins {
    pub use pins::*;

    #[cfg(any(feature = "stm32f4xx-hal", feature = "stm32f7xx-hal",))]
    mod pins {
        use stm32_eth::{hal::gpio::*, EthPins};

        pub type RefClk = PA1<Input>;
        pub type Crs = PA7<Input>;
        pub type TxD1 = PB13<Input>;
        pub type RxD0 = PC4<Input>;
        pub type RxD1 = PC5<Input>;

        #[cfg(not(feature = "rtic-echo-example-altpin"))]
        pub type TxEn = PB11<Input>;
        #[cfg(not(feature = "rtic-echo-example-altpin"))]
        pub type TxD0 = PB12<Input>;

        #[cfg(all(feature = "rtic-echo-example-altpin"))]
        pub type TxEn = PG11<Input>;
        #[cfg(feature = "rtic-echo-example-altpin")]
        pub type TxD0 = PG13<Input>;

        pub type Mdio = PA2<Alternate<11>>;
        pub type Mdc = PC1<Alternate<11>>;

        pub fn get_pins(
            gpioa: gpioa::Parts,
            gpiob: gpiob::Parts,
            gpioc: gpioc::Parts,
            #[allow(unused_variables)] gpiog: gpiog::Parts,
        ) -> (
            EthPins<RefClk, Crs, TxEn, TxD0, TxD1, RxD0, RxD1>,
            Mdio,
            Mdc,
        ) {
            let ref_clk = gpioa.pa1.into_floating_input();
            let crs = gpioa.pa7.into_floating_input();
            let tx_d1 = gpiob.pb13.into_floating_input();
            let rx_d0 = gpioc.pc4.into_floating_input();
            let rx_d1 = gpioc.pc5.into_floating_input();

            #[cfg(not(feature = "rtic-echo-example-altpin"))]
            let (tx_en, tx_d0) = (
                gpiob.pb11.into_floating_input(),
                gpiob.pb12.into_floating_input(),
            );

            #[cfg(feature = "rtic-echo-example-altpin")]
            let (tx_en, tx_d0) = (
                gpiog.pg11.into_floating_input(),
                gpiog.pg13.into_floating_input(),
            );

            #[cfg(feature = "stm32f4xx-hal")]
            let (mdio, mdc) = {
                let mut mdio = gpioa.pa2.into_alternate();
                mdio.set_speed(Speed::VeryHigh);
                let mut mdc = gpioc.pc1.into_alternate();
                mdc.set_speed(Speed::VeryHigh);
                (mdio, mdc)
            };

            #[cfg(any(feature = "stm32f7xx-hal"))]
            let (mdio, mdc) = (
                gpioa.pa2.into_alternate().set_speed(Speed::VeryHigh),
                gpioc.pc1.into_alternate().set_speed(Speed::VeryHigh),
            );

            (
                EthPins {
                    ref_clk,
                    crs,
                    tx_en,
                    tx_d0,
                    tx_d1,
                    rx_d0,
                    rx_d1,
                },
                mdio,
                mdc,
            )
        }
    }

    #[cfg(any(feature = "stm32f1xx-hal"))]
    mod pins {
        use stm32_eth::{
            hal::gpio::{Alternate, Input, PushPull, *},
            EthPins,
        };

        pub type RefClk = PA1<Input<Floating>>;
        pub type Crs = PA7<Input<Floating>>;
        pub type TxEn = PB11<Alternate<PushPull>>;
        pub type TxD0 = PB12<Alternate<PushPull>>;
        pub type TxD1 = PB13<Alternate<PushPull>>;
        pub type RxD0 = PC4<Input<Floating>>;
        pub type RxD1 = PC5<Input<Floating>>;

        pub type Mdio = PA2<Alternate<PushPull>>;
        pub type Mdc = PC1<Alternate<PushPull>>;

        pub fn get_pins(
            mut gpioa: gpioa::Parts,
            mut gpiob: gpiob::Parts,
            mut gpioc: gpioc::Parts,
            _: (),
        ) -> (
            EthPins<RefClk, Crs, TxEn, TxD0, TxD1, RxD0, RxD1>,
            Mdio,
            Mdc,
        ) {
            let ref_clk = gpioa.pa1.into_floating_input(&mut gpioa.crl);
            let mdio = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
            let crs = gpioa.pa7.into_floating_input(&mut gpioa.crl);

            let mdc = gpioc.pc1.into_alternate_push_pull(&mut gpioc.crl);
            let rx_d0 = gpioc.pc4.into_floating_input(&mut gpioc.crl);
            let rx_d1 = gpioc.pc5.into_floating_input(&mut gpioc.crl);

            let tx_en = gpiob.pb11.into_alternate_push_pull(&mut gpiob.crh);
            let tx_d0 = gpiob.pb12.into_alternate_push_pull(&mut gpiob.crh);
            let tx_d1 = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);

            let pins = EthPins {
                ref_clk,
                crs,
                tx_en,
                tx_d0,
                tx_d1,
                rx_d0,
                rx_d1,
            };

            (pins, mdio, mdc)
        }
    }
}

use ieee802_3_miim::{
    phy::{
        lan87xxa::{LAN8720A, LAN8742A},
        BarePhy, KSZ8081R,
    },
    Miim, Pause, Phy,
};

/// An ethernet PHY
pub enum EthernetPhy<M: Miim> {
    /// LAN8720A
    LAN8720A(LAN8720A<M>),
    /// LAN8742A
    LAN8742A(LAN8742A<M>),
    /// KSZ8081R
    KSZ8081R(KSZ8081R<M>),
}

impl<M: Miim> Phy<M> for EthernetPhy<M> {
    fn best_supported_advertisement(&self) -> ieee802_3_miim::AutoNegotiationAdvertisement {
        unimplemented!()
    }

    fn get_miim(&mut self) -> &mut M {
        match self {
            EthernetPhy::LAN8720A(phy) => phy.get_miim(),
            EthernetPhy::LAN8742A(phy) => phy.get_miim(),
            EthernetPhy::KSZ8081R(phy) => phy.get_miim(),
        }
    }

    fn get_phy_addr(&self) -> u8 {
        match self {
            EthernetPhy::LAN8720A(phy) => phy.get_phy_addr(),
            EthernetPhy::LAN8742A(phy) => phy.get_phy_addr(),
            EthernetPhy::KSZ8081R(phy) => phy.get_phy_addr(),
        }
    }
}

impl<M: Miim> EthernetPhy<M> {
    /// Attempt to create one of the known PHYs from the given
    /// MIIM.
    ///
    /// Returns an error if the PHY does not support the extended register
    /// set, or if the PHY's identifier does not correspond to a known PHY.
    pub fn from_miim(miim: M, phy_addr: u8) -> Result<Self, ()> {
        let mut bare = BarePhy::new(miim, phy_addr, Pause::NoPause);
        let phy_ident = bare.phy_ident().ok_or(())?;
        let miim = bare.release();
        match phy_ident & 0xFFFFFFF0 {
            0x0007C0F0 => Ok(Self::LAN8720A(LAN8720A::new(miim, phy_addr))),
            0x0007C130 => Ok(Self::LAN8742A(LAN8742A::new(miim, phy_addr))),
            0x00221560 => Ok(Self::KSZ8081R(KSZ8081R::new(miim, phy_addr))),
            _ => Err(()),
        }
    }

    /// Get a string describing the type of PHY
    pub const fn ident_string(&self) -> &'static str {
        match self {
            EthernetPhy::LAN8720A(_) => "LAN8720A",
            EthernetPhy::LAN8742A(_) => "LAN8742A",
            EthernetPhy::KSZ8081R(_) => "KSZ8081R",
        }
    }

    /// Initialize the PHY
    pub fn phy_init(&mut self) {
        match self {
            EthernetPhy::LAN8720A(phy) => phy.phy_init(),
            EthernetPhy::LAN8742A(phy) => phy.phy_init(),
            EthernetPhy::KSZ8081R(phy) => {
                phy.set_autonegotiation_advertisement(phy.best_supported_advertisement());
            }
        }
    }
}
