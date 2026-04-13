//! WASI-specific adapters for socket I/O.
//!
//! This module is only available when the `wasi` feature is enabled.
//! It provides:
//! - [`WasiTcpStream`](socket::WasiTcpStream) — a TCP socket implementing `embedded-io-async` traits

pub(crate) mod socket;

pub use socket::WasiTcpStream;
