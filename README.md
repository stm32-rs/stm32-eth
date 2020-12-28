# Rust Ethernet Driver for STM32F* microcontrollers

[![Build Status](https://travis-ci.org/stm32-rs/stm32-eth.svg?branch=master)](https://travis-ci.org/stm32-rs/stm32-eth)

## Supported microcontrollers

* STM32F4xx
* STM32F7xx

Please send pull requests.

## Building Examples
```
cargo build --example="pktgen" --features="stm32f429"
cargo build --example="ip" --features="stm32f429 smoltcp-phy log smoltcp/socket-tcp smoltcp/socket-icmp smoltcp/log smoltcp/verbose"
```

## Usage

Add to the `[dependencies]` section in your `Cargo.toml`:
```rust
stm32f4xx-hal = { version = "0.8.3", features = ["stm32f429"] }
stm32-eth = { version = "0.2.0", features = ["stm32f429"] }
```
or
```rust
stm32f7xx-hal = { version = "0.2.0", features = ["stm32f767"] }
stm32-eth = { version = "0.2.0", features = ["stm32f767"]}
```

In `src/main.rs` add:
```rust
use stm32_eth::{
    hal::gpio::GpioExt,
    hal::rcc::RccExt,
    stm32::Peripherals,
};


use stm32_eth::{Eth, RingEntry};

fn main() {
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    // HCLK must be at least 25MHz to use the ethernet peripheral
    let clocks = rcc.cfgr.sysclk(32.mhz()).hclk(32.mhz()).freeze();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();

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

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 8] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC,
        p.ETHERNET_DMA,
        &mut rx_ring[..],
        &mut tx_ring[..],
        clocks,
        eth_pins,
    )
    .unwrap();
    eth.enable_interrupt();

    if let Ok(pkt) = eth.recv_next() {
        // handle received pkt
    }

    eth.send(size, |buf| {
        // write up to `size` bytes into buf before it is being sent
    }).expect("send");
}
```

## [smoltcp] support

Use feature-flag `smoltcp-phy`
