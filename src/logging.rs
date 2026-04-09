//! Logging abstraction layer.
//!
//! Routes log macros to `defmt` or `log` depending on enabled features.
//! If neither feature is enabled, logging is a no-op.

#![allow(unused_macros)]

// ---- defmt backend ----

#[cfg(feature = "defmt")]
macro_rules! ros2_trace {
    ($($arg:tt)*) => { ::defmt::trace!($($arg)*) };
}

#[cfg(feature = "defmt")]
macro_rules! ros2_debug {
    ($($arg:tt)*) => { ::defmt::debug!($($arg)*) };
}

#[cfg(feature = "defmt")]
macro_rules! ros2_info {
    ($($arg:tt)*) => { ::defmt::info!($($arg)*) };
}

#[cfg(feature = "defmt")]
macro_rules! ros2_warn {
    ($($arg:tt)*) => { ::defmt::warn!($($arg)*) };
}

#[cfg(feature = "defmt")]
macro_rules! ros2_error {
    ($($arg:tt)*) => { ::defmt::error!($($arg)*) };
}

// ---- log backend ----

#[cfg(all(feature = "log", not(feature = "defmt")))]
macro_rules! ros2_trace {
    ($($arg:tt)*) => { ::log::trace!($($arg)*) };
}

#[cfg(all(feature = "log", not(feature = "defmt")))]
macro_rules! ros2_debug {
    ($($arg:tt)*) => { ::log::debug!($($arg)*) };
}

#[cfg(all(feature = "log", not(feature = "defmt")))]
macro_rules! ros2_info {
    ($($arg:tt)*) => { ::log::info!($($arg)*) };
}

#[cfg(all(feature = "log", not(feature = "defmt")))]
macro_rules! ros2_warn {
    ($($arg:tt)*) => { ::log::warn!($($arg)*) };
}

#[cfg(all(feature = "log", not(feature = "defmt")))]
macro_rules! ros2_error {
    ($($arg:tt)*) => { ::log::error!($($arg)*) };
}

// ---- no-op backend ----

#[cfg(not(any(feature = "defmt", feature = "log")))]
macro_rules! ros2_trace {
    ($($arg:tt)*) => {};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
macro_rules! ros2_debug {
    ($($arg:tt)*) => {};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
macro_rules! ros2_info {
    ($($arg:tt)*) => {};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
macro_rules! ros2_warn {
    ($($arg:tt)*) => {};
}

#[cfg(not(any(feature = "defmt", feature = "log")))]
macro_rules! ros2_error {
    ($($arg:tt)*) => {};
}
