//! TCP frame read/write — length-prefixed framing over `embedded-io-async`.

use crate::error::TransportError;
use embedded_io_async::{Read, Write};

/// Maximum batch/frame size for TCP transport.
///
/// Zenoh default for embedded is 8192 bytes. Can be tuned.
pub const DEFAULT_BATCH_SIZE: usize = 8192;

/// Write a length-prefixed frame to the transport.
///
/// Format: `[u16 LE length][payload]`
pub async fn write_frame<W: Write>(writer: &mut W, payload: &[u8]) -> Result<(), TransportError> {
    let len = payload.len();
    if len > u16::MAX as usize {
        return Err(TransportError::FrameTooLarge);
    }
    let len_bytes = (len as u16).to_le_bytes();
    writer
        .write_all(&len_bytes)
        .await
        .map_err(|_| TransportError::Io)?;
    writer
        .write_all(payload)
        .await
        .map_err(|_| TransportError::Io)?;
    Ok(())
}

/// Read a length-prefixed frame from the transport into `buf`.
///
/// Returns the number of payload bytes read.
pub async fn read_frame<R: Read>(reader: &mut R, buf: &mut [u8]) -> Result<usize, TransportError> {
    // Read the 2-byte length prefix
    let mut len_bytes = [0u8; 2];
    reader
        .read_exact(&mut len_bytes)
        .await
        .map_err(|_| TransportError::Io)?;
    let len = u16::from_le_bytes(len_bytes) as usize;

    if len == 0 {
        return Ok(0);
    }
    if len > buf.len() {
        return Err(TransportError::FrameTooLarge);
    }

    reader
        .read_exact(&mut buf[..len])
        .await
        .map_err(|_| TransportError::Io)?;
    Ok(len)
}
