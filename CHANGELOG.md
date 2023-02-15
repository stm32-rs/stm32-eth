## Unreleased
* Update to `smoltcp` v0.9 ([#71])
* Add `timesync-server` and `timesync-client` examples ([#72])
* Remove argument from `eth_interrupt_handler` ([#75], fixes [#44])
* Make EthernetDMA drop safe, add `fn split`, feature `async-await`, and `async-rtic-timestamp` example ([#75])


[#44]: https://github.com/stm32-rs/stm32-eth/issues/44
[#71]: https://github.com/stm32-rs/stm32-eth/pull/71
[#72]: https://github.com/stm32-rs/stm32-eth/pull/72
[#75]: https://github.com/stm32-rs/stm32-eth/pull/75

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