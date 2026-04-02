//! # zenoh-ros2-nostd
//!
//! A `no_std` ROS2 topic pub/sub library over Zenoh for embedded systems.
//!
//! Designed for use with the [embassy](https://embassy.dev/) async runtime.
//! Communicates with standard ROS2 nodes via `rmw_zenoh_cpp`-compatible
//! key expressions and CDR serialization.
//!
//! ## Features
//!
//! - `defmt` (default) — Enable `defmt` logging for embedded targets
//! - `log` — Enable `log` crate logging for std targets
//! - `alloc` — Enable `alloc`-dependent APIs (Vec, String)

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(feature = "alloc")]
extern crate alloc;

#[macro_use]
pub mod logging;

pub mod buf;
pub mod cdr;
pub mod error;
pub mod ros2;
pub mod session;
pub mod transport;
