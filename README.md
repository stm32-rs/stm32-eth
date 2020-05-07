# Rust Ethernet Driver for STM32F* microcontrollers

[![Build Status](https://travis-ci.org/stm32-rs/stm32-eth.svg?branch=master)](https://travis-ci.org/stm32-rs/stm32-eth)

## Supported microcontrollers

* STM32F4xx
* STM32F7xx

Please send pull requests.

## Building Examples
```
cargo build --example="pktgen" --features="nucleo-f767zi"
cargo build --example="ip" --features="nucleo-f767zi" --features="smoltcp-phy"
```

## Usage

Add to the `[dependencies]` section in your `Cargo.toml`:
```rust
stm32f4xx-hal = { version = "*", features = ["stm32f429"] }
stm32-eth = { version = "0.1.1", features = ["nucleo-f429zi"] }
```
or
```rust
stm32f7xx-hal = { git = "https://github.com/stm32-rs/stm32f7xx-hal", features = ["stm32f767"] }
stm32-eth = { version = "0.1.1", features = ["nucleo-f767zi"]}
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

    // Setup pins and initialize clocks.
    stm32_eth::setup(&p.RCC, &p.SYSCFG);
    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    stm32_eth::setup_pins(
        gpioa.pa1, gpioa.pa2, gpioa.pa7, gpiob.pb13, gpioc.pc1,
        gpioc.pc4, gpioc.pc5, gpiog.pg11, gpiog.pg13
    );
    // Allocate the ring buffers
    let mut rx_ring: [RingEntry<_>; 8] = Default::default();
    let mut tx_ring: [RingEntry<_>; 2] = Default::default();
    let clocks = p.RCC.constrain().cfgr.freeze();
    // Instantiate driver
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..],
        &clocks
    );
    // If you have a handler, enable interrupts
    eth.enable_interrupt(&mut cp.NVIC);


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
