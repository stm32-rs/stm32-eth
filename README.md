# Rust Ethernet Driver for STM32F* microcontrollers

[![Build Status](https://travis-ci.org/astro/stm32-eth.svg?branch=master)](https://travis-ci.org/astro/stm32-eth)

## Supported microcontrollers

* STM32F4xx

Please send pull requests.


## Usage

Add to the `[dependencies]` section in your `Cargo.toml`:
```rust
stm32f4xx-hal = { version = "*", features = ["stm32f429"] }
stm32-eth = { version = "0.1.0", feautes = ["nucleo-f429zi"] }
```

In `src/main.rs` add:
```rust
use stm32f4xx_hal::{
    gpio::GpioExt,
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
    // Instantiate driver
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
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
