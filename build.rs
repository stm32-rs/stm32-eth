fn main() {
    #[cfg(feature = "stm32f1xx-hal")]
    println!("cargo:rustc-link-search=memory.x")
}
