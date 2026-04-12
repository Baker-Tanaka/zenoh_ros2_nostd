//! WASI TCP socket adapter implementing `embedded-io-async` traits.
//!
//! Wraps a WASI file descriptor in `Read` / `Write` for use with the
//! Zenoh session.
//!
//! # Usage
//!
//! For WASI Preview 1, TCP sockets are obtained either by:
//! - Pre-opened file descriptors (passed by the host runtime)
//! - Host-specific socket APIs
//!
//! ```rust,ignore
//! // From a pre-opened fd provided by the host runtime:
//! let stream = WasiTcpStream::from_raw_fd(3);
//! let node = Node::builder("wasi_node").build(stream).await?;
//! ```

use embedded_io_async::{ErrorType, Read, Write};

/// Error type for WASI socket operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WasiIoError;

impl embedded_io_async::Error for WasiIoError {
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

// WASI Preview 1 I/O vector for fd_read / fd_write.
#[cfg(target_os = "wasi")]
#[repr(C)]
struct WasiIovec {
    buf: *mut u8,
    buf_len: usize,
}

#[cfg(target_os = "wasi")]
#[repr(C)]
struct WasiCiovec {
    buf: *const u8,
    buf_len: usize,
}

#[cfg(target_os = "wasi")]
#[link(wasm_import_module = "wasi_snapshot_preview1")]
extern "C" {
    fn fd_read(fd: u32, iovs: *const WasiIovec, iovs_len: usize, nread: *mut usize) -> u16;
    fn fd_write(fd: u32, iovs: *const WasiCiovec, iovs_len: usize, nwritten: *mut usize) -> u16;
}

/// A TCP stream backed by a WASI file descriptor.
///
/// Implements `embedded_io_async::Read` and `Write` so it can be passed
/// directly to [`Node::builder().build()`](crate::sdk::Node::builder).
pub struct WasiTcpStream {
    fd: u32,
}

impl WasiTcpStream {
    /// Create a `WasiTcpStream` from a raw WASI file descriptor.
    ///
    /// The fd must be a connected TCP socket (e.g., pre-opened by the host).
    /// No validation is performed — the caller must ensure the fd is valid.
    pub const fn from_raw_fd(fd: u32) -> Self {
        Self { fd }
    }

    /// The underlying WASI file descriptor.
    pub const fn raw_fd(&self) -> u32 {
        self.fd
    }
}

impl ErrorType for WasiTcpStream {
    type Error = WasiIoError;
}

impl Read for WasiTcpStream {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        #[cfg(target_os = "wasi")]
        {
            let iov = WasiIovec {
                buf: buf.as_mut_ptr(),
                buf_len: buf.len(),
            };
            let mut nread: usize = 0;
            let rc = unsafe { fd_read(self.fd, &iov, 1, &mut nread) };
            if rc == 0 {
                Ok(nread)
            } else {
                Err(WasiIoError)
            }
        }
        #[cfg(not(target_os = "wasi"))]
        {
            let _ = buf;
            Err(WasiIoError)
        }
    }
}

impl Write for WasiTcpStream {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        #[cfg(target_os = "wasi")]
        {
            let ciov = WasiCiovec {
                buf: buf.as_ptr(),
                buf_len: buf.len(),
            };
            let mut nwritten: usize = 0;
            let rc = unsafe { fd_write(self.fd, &ciov, 1, &mut nwritten) };
            if rc == 0 {
                Ok(nwritten)
            } else {
                Err(WasiIoError)
            }
        }
        #[cfg(not(target_os = "wasi"))]
        {
            let _ = buf;
            Err(WasiIoError)
        }
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
