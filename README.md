# Rust Ethernet Driver for STM32F* microcontrollers

[![Build Status](https://travis-ci.org/astro/stm32-eth.svg?branch=master)](https://travis-ci.org/astro/stm32-eth)

## Supported microcontrollers

* STM32F429 with feature `target-stm32f429`
* STM32F7x9 with feature `target-stm32f7x9` (untested)
  
### Potentially supported microcontrollers

Similar hardware registers seem to appear in these microcontroller models:

* STM32F107xx
* STM32F20x
* STM32F21x
* STM32F40x
* STM32F41x
* STM32F427
* STM32F437
* STM32F439
* STM32F469
* STM32F479
* STM32F7x5
* STM32F7x6
* STM32F7x7
* STM32F7x

Please send pull requests.


## Usage

Add to the `[dependencies]` section in your `Cargo.toml`:
```rust
stm32-eth = { version = "*", features = ["target-stm32f429"] }
```

In `src/main.rs` add:
```rust
extern crate stm32f429 as target;
use target::Peripherals;

extern crate stm32_eth;
use stm32_eth::{Eth, RingEntry};

fn main() {
    let p = Peripherals::take().unwrap();

    // Setup pins and initialize clocks.
    eth::setup(&p);
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

The `master` branch of `stm32-eth` contains support for the current
`master` branch of the [smoltcp] TCP/IP stack. It has been removed for
release because it is incompatible with latest [smoltcp] stable
release `0.4.0`.

[smoltcp]: https://github.com/m-labs/smoltcp
