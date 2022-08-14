## Unreleased
* Add `RTIC` example
* Use `ieee802_3_miim` for SMI access.

## [0.3.0](https://github.com/stm32-rs/stm32-eth/tree/v0.3.0)

* Enable ICMP, TCP, and UDP checksum offloading for IPv4 and IPv6. ([#48](https://github.com/stm32-rs/stm32-eth/pull/48))
* Separate MAC and DMA into separate structs for separate access. ([#39](https://github.com/stm32-rs/stm32-eth/pull/39))
* Add support for `stm32f107` and fix an MMC interrupt bug. ([#43](https://github.com/stm32-rs/stm32-eth/pull/43), [#42](https://github.com/stm32-rs/stm32-eth/pull/42), [#41](https://github.com/stm32-rs/stm32-eth/pull/41))
* Update the HALs and dependencies to their latest versions as of 12-07-2022.
* Add more examples
