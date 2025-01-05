#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32, path = "esp32.rs")]
mod touch_impl;

pub use touch_impl::*;
