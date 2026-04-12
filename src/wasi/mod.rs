//! WASI-specific adapters for socket I/O and time.
//!
//! This module is only available when the `wasi` feature is enabled.
//! It provides:
//! - [`WasiTcpStream`](socket::WasiTcpStream) — a TCP socket implementing `embedded-io-async` traits
//! - Time utilities using WASI monotonic clock

pub(crate) mod socket;
pub(crate) mod time;

pub use socket::WasiTcpStream;
