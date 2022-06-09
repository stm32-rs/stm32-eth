# Rust Ethernet Driver for STM32F* microcontrollers

[![Build Status](https://travis-ci.org/stm32-rs/stm32-eth.svg?branch=master)](https://travis-ci.org/stm32-rs/stm32-eth)

## Supported microcontrollers

* STM32F107
* STM32F4xx
* STM32F7xx

Please send pull requests.

## Building Examples

```bash
cargo build --example="pktgen" --features="stm32f429"
cargo build --example="ip" --features="stm32f429 smoltcp-phy log smoltcp/socket-tcp smoltcp/socket-icmp smoltcp/log smoltcp/verbose"
cargo build --example="ip-f107" --features="stm32f107 smoltcp-phy log smoltcp/socket-tcp smoltcp/socket-icmp smoltcp/log smoltcp/verbose"
```

## Usage

Add one of the following to the `[dependencies]` section in your `Cargo.toml` (with the correct MCU specified):

```toml
stm32-eth = { version = "0.3.0", features = ["stm32f429"] } # For stm32f4xx-like MCUs
stm32-eth = { version = "0.3.0", features = ["stm32f767"] } # For stm32f7xx-like MCUs
stm32-eth = { version = "0.3.0", features = ["stm32f107"] } # For stm32f107
```

`stm32_eth` re-exports the underlying HAL as `stm32_eth::hal`.

In `src/main.rs` add:

```rust,no_run
use stm32_eth::{
    hal::gpio::GpioExt,
    hal::rcc::RccExt,
    stm32::Peripherals,
    RingEntry,
    EthPins,
};
use fugit::RateExtU32;

fn main() {
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    // HCLK must be at least 25MHz to use the ethernet peripheral
    let clocks = rcc.cfgr.sysclk(32.MHz()).hclk(32.MHz()).freeze();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();

    let eth_pins = EthPins {
        ref_clk: gpioa.pa1,
        crs: gpioa.pa7,
        tx_en: gpiog.pg11,
        tx_d0: gpiog.pg13,
        tx_d1: gpiob.pb13,
        rx_d0: gpioc.pc4,
        rx_d1: gpioc.pc5,
    };

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 8] = Default::default();
    let (mut eth_dma, _eth_mac) = stm32_eth::new(
        p.ETHERNET_MAC,
        p.ETHERNET_MMC,
        p.ETHERNET_DMA,
        &mut rx_ring[..],
        &mut tx_ring[..],
        clocks,
        eth_pins,
    )
    .unwrap();
    eth_dma.enable_interrupt();

    if let Ok(pkt) = eth_dma.recv_next() {
        // handle received pkt
    }

    let size = 42;
    eth_dma.send(size, |buf| {
        // write up to `size` bytes into buf before it is being sent
    }).expect("send");
}
```

## [smoltcp] support

Use feature-flag `smoltcp-phy`
