fn main() {
    println!("cargo:rustc-link-search=memory.x");

    let hse = std::env::var("STM32_ETH_EXAMPLE_HSE");

    if let Ok(hse) = hse {
        if hse == "bypass" {
            println!("cargo:rustc-cfg=hse=\"bypass\"")
        } else if hse == "oscillator" {
            println!("cargo:rustc-cfg=hse=\"oscillator\"");
        } else if hse != "off" {
            panic!("Invalid STM32_ETH_EXAMPLE_HSE value. Allowed values: bypass, oscillator, off")
        }
    }

    let example_pins = std::env::var("STM32_ETH_EXAMPLE_PINS");

    if let Ok(pins) = example_pins {
        if pins == "nucleo" {
            println!("cargo:rustc-cfg=pins=\"nucleo\"")
        } else if pins != "default" {
            panic!("Invalid STM32_ETH_EXAMPLE_PINS value. Allowed values: nucleo, default");
        }
    }

    let pps_pin = std::env::var("STM32_ETH_EXAMPLE_PPS_PIN");

    if let Ok(pps_pin) = pps_pin {
        if pps_pin == "alternate" {
            println!("cargo:rustc-cfg=pps=\"alternate\"")
        } else if pps_pin != "default" {
            panic!("Invalid STM32_ETH_EXAMPLE_PPS_PIN value. Allowed values: alternate, default");
        }
    }

    println!("cargo:rerun-if-env-changed=STM32_ETH_EXAMPLE_PPS_PIN");
    println!("cargo:rerun-if-env-changed=STM32_ETH_EXAMPLE_HSE");
    println!("cargo:rerun-if-env-changed=STM32_ETH_EXAMPLE_PINS");
}
