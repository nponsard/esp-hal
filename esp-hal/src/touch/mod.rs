//! # Capacitive Touch Sensor
//!
//! ## Overview
//!
//! The touch sensor peripheral allows for cheap and robust user interfaces by
//! e.g., dedicating a part of the pcb as touch button.
//!
//! ## Examples
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::touch::{Touch, TouchPad};
//! let touch_pin0 = peripherals.GPIO2;
//! let touch = Touch::continuous_mode(peripherals.TOUCH, None);
//! let mut touchpad = TouchPad::new(touch_pin0, &touch);
//! // ... give the peripheral some time for the measurement
//! let touch_val = touchpad.read();
//! # }
//! ```
//! 
//! ## Implementation State:
//!
//! Mostly feature complete, missing:
//!
//! - Touch sensor slope control
//! - Deep Sleep support (wakeup from Deep Sleep)

#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32, path = "esp32.rs")]
mod touch_impl;

pub use touch_impl::*;
