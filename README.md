# Rust Ethernet Driver for STM32F* microcontrollers

[![Build Status](https://travis-ci.org/stm32-rs/stm32-eth.svg?branch=master)](https://travis-ci.org/stm32-rs/stm32-eth)

## Supported microcontrollers

* STM32F107
* STM32F4xx
* STM32F7xx

Please send pull requests.

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


## `smoltcp` support

Use feature-flag `smoltcp-phy`

## Examples

The examples should run and compile on any MCU that has an 802.3 compatible PHY capable of generating the required 50 MHz clock signal connected to the default RMII pins.

The examples use `defmt` and `defmt_rtt` for logging, and `panic_probe` over `defmt_rtt` for printing panic backtraces.

To run or build them, the following steps should be taken:

1. Determine the correct compilation target for the MCU that you're using. For `stm32f107`, it is `thumbv7m-none-eabi`. For all others, it is `thumbv7em-none-eabihf`.
2. Determine the MCU feature necessary for running on your MCU, e.g. `stm32f745`.
3. Determine the Additional required features (see section below) necessary to build the example.
4. Follow the rest of the instructions in the ["Building examples"](#building-examples) or ["Running examples"](#running-examples) subsections.

### Additional required features

Besides the feature selecting the correct MCU to be used when building and/or running an example, the following additional features are required:

| Example       | Additional required features                         |
| ------------- | ---------------------------------------------------- |
| `arp-smoltcp` | `defmt,smoltcp-phy,smoltcp/socket-raw`               |
| `arp`         | `defmt`                                              |
| `ip`          | `defmt,smoltcp-phy,smoltcp/defmt,smoltcp/socket-tcp` |
| `pktgen`      | `defmt`                                              |
| `rtic-echo`   | `rtic-echo-example`                                  |

#### 144-pin nucleo boards

For `stm32` 144-pin nucleo boards that contain an MCU supported by this crate the `example-nucleo-pins` feature should be activated. This causes the examples to use PG11 as TX_EN and PG13 as TXD0, instead of PB11 and PB12, which is the configuration used on these boards.

### Building examples
Run the following command:
```bash
cargo build --release --example <example> --features <MCU feature>,<additional required features> --target <MCU compilation target>
```

For example, if we wish to build the `arp-smoltcp` example for an `stm32f429`, we should run the following command:

```bash
cargo build --release --example arp-smoltcp --features stm32f429,smoltcp-phy,smoltcp/socket-tcp --target thumbv7em-none-eabihf
```

### Running examples
Install `probe-run` with `cargo install probe-run --version '~0.3'`

Find the correct value for `PROBE_RUN_CHIP` for your MCU from the list provided by `probe-run --list-chips`.

Ensure that `probe-run` can attach to your MCU

Then, run the following command:
```bash
DEFMT_LOG=info PROBE_RUN_CHIP=<probe-run chip> cargo run --release --example <example> --features <MCU feature>,<additional required features> --target <MCU compilation target>
```

For example, if we wish to run the `rtic-echo` example on an `STM32F107RCT6`, we should run the following command:

```bash
DEFMT_LOG=info PROBE_RUN_CHIP=STM32F107RC cargo run --release --example rtic-echo --features stm32f107,rtic-echo-example --target thumbv7m-none-eabi
```

### Other pin configurations

If the usage of different pins is required, the types and `setup_pins` function in `examples/common.rs` should be edited. If the pin configuration is for a `nucleo` board or other commonly used board, a PR with the changes is most welcome.
