//! Zenoh wire encoding/decoding (VByte, message header, payload).
//!
//! Implements the binary encoding for the subset of zenoh protocol
//! messages needed for client-mode topic pub/sub.

use super::protocol::*;
use crate::error::TransportError;

// ====== VByte (unsigned LEB128) ======

/// Encode a u64 as VByte (unsigned LEB128) into `buf`.
/// Returns the number of bytes written.
pub fn encode_vbyte(buf: &mut [u8], mut value: u64) -> Result<usize, TransportError> {
    let mut pos = 0;
    loop {
        if value < 0x80 {
            if pos >= buf.len() {
                return Err(TransportError::FrameTooLarge);
            }
            buf[pos] = value as u8;
            pos += 1;
            return Ok(pos);
        }
        if pos >= buf.len() {
            return Err(TransportError::FrameTooLarge);
        }
        buf[pos] = ((value & 0x7F) as u8) | 0x80;
        pos += 1;
        value >>= 7;
    }
}

/// Decode a VByte (unsigned LEB128) from `buf`.
/// Returns (value, bytes_consumed).
pub fn decode_vbyte(buf: &[u8]) -> Result<(u64, usize), TransportError> {
    let mut value: u64 = 0;
    let mut shift = 0u32;
    for (i, &byte) in buf.iter().enumerate() {
        value |= ((byte & 0x7F) as u64) << shift;
        if byte & 0x80 == 0 {
            return Ok((value, i + 1));
        }
        shift += 7;
        if shift >= 64 {
            return Err(TransportError::InvalidEncoding);
        }
    }
    Err(TransportError::InvalidEncoding)
}

// ====== ZenohId encoding ======

/// Encode a ZenohId: [length: u8][bytes].
pub fn encode_zenoh_id(buf: &mut [u8], zid: &ZenohId) -> Result<usize, TransportError> {
    let id_bytes = zid.as_bytes();
    if buf.len() < 1 + id_bytes.len() {
        return Err(TransportError::FrameTooLarge);
    }
    // Length is encoded as (byte_count - 1) in lower 4 bits of a zint
    buf[0] = (id_bytes.len() as u8).wrapping_sub(1);
    buf[1..1 + id_bytes.len()].copy_from_slice(id_bytes);
    Ok(1 + id_bytes.len())
}

/// Decode a ZenohId from buf. Returns (ZenohId, bytes_consumed).
pub fn decode_zenoh_id(buf: &[u8]) -> Result<(ZenohId, usize), TransportError> {
    if buf.is_empty() {
        return Err(TransportError::InvalidEncoding);
    }
    let len = (buf[0] as usize) + 1; // stored as len-1
    if buf.len() < 1 + len {
        return Err(TransportError::InvalidEncoding);
    }
    Ok((ZenohId::from_bytes(&buf[1..1 + len]), 1 + len))
}

// ====== Slice / byte array encoding ======

/// Encode a byte slice as [vbyte length][bytes].
pub fn encode_slice(buf: &mut [u8], data: &[u8]) -> Result<usize, TransportError> {
    let mut pos = encode_vbyte(buf, data.len() as u64)?;
    if pos + data.len() > buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos..pos + data.len()].copy_from_slice(data);
    pos += data.len();
    Ok(pos)
}

/// Decode a byte slice from [vbyte length][bytes].
/// Returns (slice, bytes_consumed).
pub fn decode_slice(buf: &[u8]) -> Result<(&[u8], usize), TransportError> {
    let (len, hdr) = decode_vbyte(buf)?;
    let len = len as usize;
    if hdr + len > buf.len() {
        return Err(TransportError::InvalidEncoding);
    }
    Ok((&buf[hdr..hdr + len], hdr + len))
}

// ====== String encoding ======

/// Encode a string as [vbyte length][utf8 bytes] (no null terminator).
pub fn encode_string(buf: &mut [u8], s: &str) -> Result<usize, TransportError> {
    encode_slice(buf, s.as_bytes())
}

// ====== InitSyn encoding ======

/// Encode an InitSyn message into `buf`.
/// Returns the number of bytes written.
pub fn encode_init_syn(buf: &mut [u8], msg: &InitSyn) -> Result<usize, TransportError> {
    let mut pos = 0;

    // Header byte
    let mut header = transport_id::INIT;
    if msg.batch_size.is_some() {
        header |= init_flag::S;
    }
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = header;
    pos += 1;

    // Version
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = msg.version;
    pos += 1;

    // WhatAmI (encoded as zint)
    pos += encode_vbyte(&mut buf[pos..], msg.whatami as u64)?;

    // ZenohId
    pos += encode_zenoh_id(&mut buf[pos..], &msg.zid)?;

    // Batch size extension (if S flag)
    if let Some(bs) = msg.batch_size {
        // Extension header: type=1 (QoS), more=0
        if pos + 3 > buf.len() {
            return Err(TransportError::FrameTooLarge);
        }
        buf[pos] = 0x01; // extension type
        pos += 1;
        buf[pos..pos + 2].copy_from_slice(&bs.to_le_bytes());
        pos += 2;
    }

    Ok(pos)
}

// ====== InitAck decoding ======

/// Decode an InitAck message from `buf`.
pub fn decode_init_ack(buf: &[u8]) -> Result<(InitAck, usize), TransportError> {
    let mut pos = 0;

    if buf.is_empty() {
        return Err(TransportError::InvalidEncoding);
    }

    let header = buf[pos];
    pos += 1;

    let msg_id = header & 0x1F;
    if msg_id != transport_id::INIT {
        return Err(TransportError::InvalidEncoding);
    }
    if header & init_flag::A == 0 {
        return Err(TransportError::InvalidEncoding); // Expected ACK
    }
    let has_batch_size = header & init_flag::S != 0;
    let has_ext = header & init_flag::Z != 0;

    // Version
    if pos >= buf.len() {
        return Err(TransportError::InvalidEncoding);
    }
    let version = buf[pos];
    pos += 1;

    // WhatAmI
    let (whatami_val, n) = decode_vbyte(&buf[pos..])?;
    pos += n;
    let whatami = match whatami_val {
        0x01 => WhatAmI::Router,
        0x02 => WhatAmI::Peer,
        0x04 => WhatAmI::Client,
        _ => return Err(TransportError::InvalidEncoding),
    };

    // ZenohId
    let (zid, n) = decode_zenoh_id(&buf[pos..])?;
    pos += n;

    // Cookie
    let (cookie_bytes, n) = decode_slice(&buf[pos..])?;
    pos += n;
    let mut cookie = heapless::Vec::new();
    cookie
        .extend_from_slice(cookie_bytes)
        .map_err(|_| TransportError::FrameTooLarge)?;

    // Batch size extension
    let mut batch_size = None;
    if has_batch_size {
        if pos + 3 > buf.len() {
            return Err(TransportError::InvalidEncoding);
        }
        let _ext_type = buf[pos];
        pos += 1;
        batch_size = Some(u16::from_le_bytes([buf[pos], buf[pos + 1]]));
        pos += 2;
    }

    // Skip remaining extensions
    if has_ext {
        // TODO: properly skip chained extensions
    }

    Ok((
        InitAck {
            version,
            whatami,
            zid,
            cookie,
            batch_size,
        },
        pos,
    ))
}

// ====== OpenSyn encoding ======

/// Encode an OpenSyn message.
pub fn encode_open_syn(buf: &mut [u8], msg: &OpenSyn) -> Result<usize, TransportError> {
    let mut pos = 0;

    // Header
    let header = transport_id::OPEN; // No T flag = lease in milliseconds
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = header;
    pos += 1;

    // Lease (ms, as vbyte)
    pos += encode_vbyte(&mut buf[pos..], msg.lease_ms)?;

    // Initial SN
    pos += encode_vbyte(&mut buf[pos..], msg.initial_sn)?;

    // Cookie
    pos += encode_slice(&mut buf[pos..], &msg.cookie)?;

    Ok(pos)
}

// ====== OpenAck decoding ======

/// Decode an OpenAck message.
pub fn decode_open_ack(buf: &[u8]) -> Result<(OpenAck, usize), TransportError> {
    let mut pos = 0;

    if buf.is_empty() {
        return Err(TransportError::InvalidEncoding);
    }

    let header = buf[pos];
    pos += 1;

    let msg_id = header & 0x1F;
    if msg_id != transport_id::OPEN {
        return Err(TransportError::InvalidEncoding);
    }
    if header & open_flag::A == 0 {
        return Err(TransportError::InvalidEncoding); // Expected ACK
    }
    let lease_in_seconds = header & open_flag::T != 0;

    // Lease
    let (lease_val, n) = decode_vbyte(&buf[pos..])?;
    pos += n;
    let lease_ms = if lease_in_seconds {
        lease_val * 1000
    } else {
        lease_val
    };

    // Initial SN
    let (initial_sn, n) = decode_vbyte(&buf[pos..])?;
    pos += n;

    Ok((OpenAck { lease_ms, initial_sn }, pos))
}

// ====== KeepAlive ======

/// Encode a KeepAlive message (just a header byte).
pub fn encode_keepalive(buf: &mut [u8]) -> Result<usize, TransportError> {
    if buf.is_empty() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[0] = transport_id::KEEP_ALIVE;
    Ok(1)
}

/// Check if the first byte is a KeepAlive header.
pub fn is_keepalive(header: u8) -> bool {
    header & 0x1F == transport_id::KEEP_ALIVE
}

// ====== Close ======

/// Encode a Close message.
pub fn encode_close(buf: &mut [u8], reason: u8) -> Result<usize, TransportError> {
    if buf.len() < 2 {
        return Err(TransportError::FrameTooLarge);
    }
    buf[0] = transport_id::CLOSE;
    buf[1] = reason;
    Ok(2)
}

// ====== Frame ======

/// Encode a Frame header + sequence number.
/// The caller appends network message(s) after this.
/// Returns position after the frame header.
pub fn encode_frame_header(
    buf: &mut [u8],
    sn: u64,
    reliable: bool,
) -> Result<usize, TransportError> {
    let mut pos = 0;
    let mut header = transport_id::FRAME;
    if reliable {
        header |= frame_flag::R;
    }
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = header;
    pos += 1;

    pos += encode_vbyte(&mut buf[pos..], sn)?;

    Ok(pos)
}

/// Decode a Frame header. Returns (sn, reliable, body_start_pos).
pub fn decode_frame_header(buf: &[u8]) -> Result<(u64, bool, usize), TransportError> {
    if buf.is_empty() {
        return Err(TransportError::InvalidEncoding);
    }
    let header = buf[0];
    if header & 0x1F != transport_id::FRAME {
        return Err(TransportError::InvalidEncoding);
    }
    let reliable = header & frame_flag::R != 0;
    let (sn, n) = decode_vbyte(&buf[1..])?;
    Ok((sn, reliable, 1 + n))
}

// ====== Push + Put (for publishing) ======

/// Encode a Push + Put network message (inline key expression, no extensions).
///
/// This is the most common message for publishing topic data.
/// Format: [Push header][scope vbyte][suffix string][Put header][encoding vbyte][payload slice]
pub fn encode_push_put(
    buf: &mut [u8],
    key_expr: &str,
    encoding_id: u64,
    payload: &[u8],
) -> Result<usize, TransportError> {
    let mut pos = 0;

    // Push header: inline key (scope=0, suffix=key_expr), N flag for suffix
    let push_header = network_id::PUSH | push_flag::N;
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = push_header;
    pos += 1;

    // Scope = 0 (inline)
    pos += encode_vbyte(&mut buf[pos..], 0)?;

    // Suffix (the key expression string)
    pos += encode_string(&mut buf[pos..], key_expr)?;

    // Put header: no timestamp, no extensions
    let put_header = zenoh_id::PUT;
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = put_header;
    pos += 1;

    // Encoding
    pos += encode_vbyte(&mut buf[pos..], encoding_id)?;

    // Payload
    pos += encode_slice(&mut buf[pos..], payload)?;

    Ok(pos)
}

// ====== Declare messages ======

/// Encode a Declare KeyExpr message.
///
/// Registers a key expression with the router so it can be
/// referenced by a numeric ID in subsequent messages.
pub fn encode_declare_keyexpr(
    buf: &mut [u8],
    key_id: u16,
    key_expr: &str,
) -> Result<usize, TransportError> {
    let mut pos = 0;

    // Network Declare header
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = network_id::DECLARE;
    pos += 1;

    // Number of declarations = 1 (we batch one at a time for simplicity)
    // Actually in zenoh, declarations are just listed with an "end" marker.
    // Declaration: DeclareKeyExpr
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = declare_id::D_KEYEXPR;
    pos += 1;

    // Key expression ID
    pos += encode_vbyte(&mut buf[pos..], key_id as u64)?;

    // Wire expression: scope=0 + suffix
    pos += encode_vbyte(&mut buf[pos..], 0)?; // scope
    pos += encode_string(&mut buf[pos..], key_expr)?;

    Ok(pos)
}

/// Encode a Declare Subscriber message.
pub fn encode_declare_subscriber(
    buf: &mut [u8],
    sub_id: u32,
    key_id: u16,
) -> Result<usize, TransportError> {
    let mut pos = 0;

    // Network Declare header
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = network_id::DECLARE;
    pos += 1;

    // Declaration: DeclareSubscriber
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = declare_id::D_SUBSCRIBER;
    pos += 1;

    // Subscriber ID
    pos += encode_vbyte(&mut buf[pos..], sub_id as u64)?;

    // Wire expression referencing declared keyexpr
    pos += encode_vbyte(&mut buf[pos..], key_id as u64)?; // scope = key_id (mapped)

    Ok(pos)
}

// ====== Identify received message type ======

/// Identify a transport message type from its header byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportMsgKind {
    Init,
    Open,
    Close,
    KeepAlive,
    Frame,
    Fragment,
    Other(u8),
}

/// Parse the transport message kind from the header byte.
pub fn parse_transport_msg_kind(header: u8) -> TransportMsgKind {
    match header & 0x1F {
        transport_id::INIT => TransportMsgKind::Init,
        transport_id::OPEN => TransportMsgKind::Open,
        transport_id::CLOSE => TransportMsgKind::Close,
        transport_id::KEEP_ALIVE => TransportMsgKind::KeepAlive,
        transport_id::FRAME => TransportMsgKind::Frame,
        transport_id::FRAGMENT => TransportMsgKind::Fragment,
        other => TransportMsgKind::Other(other),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vbyte_small() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 42).unwrap();
        assert_eq!(n, 1);
        assert_eq!(buf[0], 42);

        let (val, consumed) = decode_vbyte(&buf).unwrap();
        assert_eq!(val, 42);
        assert_eq!(consumed, 1);
    }

    #[test]
    fn test_vbyte_multibyte() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 300).unwrap();
        assert_eq!(n, 2);

        let (val, consumed) = decode_vbyte(&buf).unwrap();
        assert_eq!(val, 300);
        assert_eq!(consumed, 2);
    }

    #[test]
    fn test_vbyte_large() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 1_000_000).unwrap();

        let (val, consumed) = decode_vbyte(&buf[..n]).unwrap();
        assert_eq!(val, 1_000_000);
        assert_eq!(consumed, n);
    }

    #[test]
    fn test_zenoh_id_roundtrip() {
        let zid = ZenohId::from_bytes(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let mut buf = [0u8; 32];
        let n = encode_zenoh_id(&mut buf, &zid).unwrap();

        let (decoded, consumed) = decode_zenoh_id(&buf[..n]).unwrap();
        assert_eq!(consumed, n);
        assert_eq!(decoded.as_bytes(), zid.as_bytes());
    }

    #[test]
    fn test_init_syn_encode() {
        let msg = InitSyn {
            version: PROTO_VERSION,
            whatami: WhatAmI::Client,
            zid: ZenohId::from_bytes(&[0xAB; 8]),
            batch_size: None,
        };

        let mut buf = [0u8; 64];
        let n = encode_init_syn(&mut buf, &msg).unwrap();
        assert!(n > 0);

        // Verify header
        assert_eq!(buf[0] & 0x1F, transport_id::INIT);
        assert_eq!(buf[0] & init_flag::A, 0); // SYN, not ACK
    }

    #[test]
    fn test_keepalive_encode() {
        let mut buf = [0u8; 4];
        let n = encode_keepalive(&mut buf).unwrap();
        assert_eq!(n, 1);
        assert!(is_keepalive(buf[0]));
    }

    #[test]
    fn test_frame_header_roundtrip() {
        let mut buf = [0u8; 16];
        let n = encode_frame_header(&mut buf, 42, true).unwrap();

        let (sn, reliable, body_pos) = decode_frame_header(&buf[..n]).unwrap();
        assert_eq!(sn, 42);
        assert!(reliable);
        assert_eq!(body_pos, n);
    }
}
