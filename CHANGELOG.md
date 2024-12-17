## Unreleased

## [0.8.0](https://github.com/stm32-rs/stm32-eth/tree/v0.8.0)
* Update version of `smoltcp` to `v0.12.0` ([#99])

[#99]: https://github.com/stm32-rs/stm32-eth/pull/99

## [0.7.0](https://github.com/stm32-rs/stm32-eth/tree/v0.7.0)
* Update hals ([#97])
    * `stm32f4xx-hal` from `0.20` to `0.21`
    * `stm32f7xx-hal` from `0.7` to `0.7`

[#97]: https://github.com/stm32-rs/stm32-eth/pull/97

## [0.6.0](https://github.com/stm32-rs/stm32-eth/tree/v0.6.0)
* Update version of `smoltcp` to `v0.11.0` ([#92])
* Remove re-export of `smoltcp`, and update examples ([#94])

[#92]: https://github.com/stm32-rs/stm32-eth/pull/92
[#94]: https://github.com/stm32-rs/stm32-eth/pull/94

## [0.5.2](https://github.com/stm32-rs/stm32-eth/tree/v0.5.2)
* Fix the PPS pin frequency so it's not always running at the max frequency ([#89])

[#89]: https://github.com/stm32-rs/stm32-eth/pull/89

## [0.5.1](https://github.com/stm32-rs/stm32-eth/tree/v0.5.1)
* Ensure `packet_id` overflow does not panic ([#87])

[#87]: https://github.com/stm32-rs/stm32-eth/pull/87

## [0.5.0](https://github.com/stm32-rs/stm32-eth/tree/v0.5.0)
### Changes
* Update to `smoltcp` ~~v0.9~~ v0.10 (~~[#71]~~ [#81])
* Remove argument from `eth_interrupt_handler` ([#75], fixes [#44])
* [#75]
    * Make EthernetDMA drop safe
    * Add `EthernetDMA::split`: support for using RX and TX independently
    * Make the interrupt handler methods and `EthernetPTP::get_time` static FNs
    * Add feature `async-await`, used for accessing RX and/or TX asynchronously, as well as the ethernet PTP timestamp interrupt

### Examples
* Add `timesync-server` and `timesync-client` examples ([#72])
* Add a `smoltcp` based PTP example ([#82])
* Add `async-rtic-timestamp` example (nightly only) ([#75])

[#44]: https://github.com/stm32-rs/stm32-eth/issues/44
[#71]: https://github.com/stm32-rs/stm32-eth/pull/71
[#72]: https://github.com/stm32-rs/stm32-eth/pull/72
[#75]: https://github.com/stm32-rs/stm32-eth/pull/75
[#81]: https://github.com/stm32-rs/stm32-eth/pull/81
[#82]: https://github.com/stm32-rs/stm32-eth/pull/82

## [0.4.1](https://github.com/stm32-rs/stm32-eth/tree/v0.4.1)
* Fix a bug when caching timestamps in the TX path ([#73])

[#73]: https://github.com/stm32-rs/stm32-eth/pull/73

## [0.4.0](https://github.com/stm32-rs/stm32-eth/tree/v0.4.0)
* Remove the `smi` feature and always enable miim/smi. Use `ieee802_3_miim` for SMI access ([#45])
* Update stm32f1xx-hal and stm32f4xx-hal to their latests version as of 15-12-2022.
* Allow for configuration of MAC speed. ([#53], fixes [#24])
* Fix [#57](https://github.com/stm32-rs/stm32-eth/issues/57). ([#58])
* Move all DMA related files into modules under `dma` ([#66])
* Add support for the PTP peripheral ([#66])
* Use `PartsIn` and `Parts` as structs for initalization & configuration ([#66])
* CI
    * Test compilability of examples more extensively
    * Move away from actions-rs
* Examples:
    * Switch to `defmt` as logger
    * Use `probe-run` as runner
    * Ensure that all examples build (and hopefully run) for all supported MCUs
    * Add more extensive example run and build docs
    * Remove arp-smoltcp example
    * Add `rtic-echo` example
    * Use a more simple `memory.x` that works for all supported MCUs
    * Add `rtic-timestamp` example

[#45]: https://github.com/stm32-rs/stm32-eth/pull/45
[#24]: https://github.com/stm32-rs/stm32-eth/pull/24
[#53]: https://github.com/stm32-rs/stm32-eth/pull/53
[#58]: https://github.com/stm32-rs/stm32-eth/pull/58
[#66]: https://github.com/stm32-rs/stm32-eth/pull/66

## [0.3.0](https://github.com/stm32-rs/stm32-eth/tree/v0.3.0)

* Enable ICMP, TCP, and UDP checksum offloading for IPv4 and IPv6. ([#48])
* Separate MAC and DMA into separate structs for separate access. ([#39])
* Add support for `stm32f107` and fix an MMC interrupt bug. ([#43], [#42], [#41])
* Update the HALs and dependencies to their latest versions as of 12-07-2022.
* Add more examples

[#48]: https://github.com/stm32-rs/stm32-eth/pull/48
[#39]: https://github.com/stm32-rs/stm32-eth/pull/39
[#43]: https://github.com/stm32-rs/stm32-eth/pull/43
[#42]: https://github.com/stm32-rs/stm32-eth/pull/42
[#41]: https://github.com/stm32-rs/stm32-eth/pull/41
