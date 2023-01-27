fn main() {
    #[cfg(feature = "stm32f1xx-hal")]
    println!("cargo:rustc-link-search=memory.x");

    let hse = std::env::var("EXAMPLE_HSE");

    if let Ok(hse) = hse {
        if hse == "bypass" {
            println!("cargo:rustc-cfg=hse=\"bypass\"")
        } else if hse == "oscillator" {
            println!("cargo:rustc-cfg=hse=\"oscillator\"");
        } else if hse == "off" {
        } else {
            panic!("Invalid EXAMPLE_HSE value. Allowed values: bypass, oscillator, off")
        }
    }

    println!("cargo:rerun-if-env-changed=EXAMPLE_HSE");
}
