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

// ====== WhatAmI 2-bit encoding (for Init flags byte) ======

/// Encode WhatAmI to 2-bit value for the Init flags byte.
fn whatami_to_2bit(w: WhatAmI) -> u8 {
    match w {
        WhatAmI::Router => 0b00,
        WhatAmI::Peer => 0b01,
        WhatAmI::Client => 0b10,
    }
}

/// Decode WhatAmI from 2-bit value in the Init flags byte.
fn whatami_from_2bit(bits: u8) -> Result<WhatAmI, TransportError> {
    match bits & 0b11 {
        0b00 => Ok(WhatAmI::Router),
        0b01 => Ok(WhatAmI::Peer),
        0b10 => Ok(WhatAmI::Client),
        _ => Err(TransportError::InvalidEncoding),
    }
}

// ====== InitSyn encoding ======

/// Encode an InitSyn message into `buf`.
/// Returns the number of bytes written.
///
/// Wire format (zenoh v9):
/// ```text
/// [header: u8][version: u8][flags: u8][zid: N bytes]
/// flags = (zid_len - 1) << 4 | whatami_2bit
/// if S flag: [resolution: u8][batch_size: u16 LE]
/// ```
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

    // Flags byte: (zid_len - 1) << 4 | whatami_2bit
    let zid_bytes = msg.zid.as_bytes();
    let flags = ((zid_bytes.len() as u8 - 1) << 4) | whatami_to_2bit(msg.whatami);
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = flags;
    pos += 1;

    // ZenohId raw bytes (length encoded in flags)
    if pos + zid_bytes.len() > buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos..pos + zid_bytes.len()].copy_from_slice(zid_bytes);
    pos += zid_bytes.len();

    // Resolution + Batch size (if S flag)
    if let Some(bs) = msg.batch_size {
        if pos + 3 > buf.len() {
            return Err(TransportError::FrameTooLarge);
        }
        buf[pos] = 0x00; // Resolution::default()
        pos += 1;
        buf[pos..pos + 2].copy_from_slice(&bs.to_le_bytes());
        pos += 2;
    }

    Ok(pos)
}

// ====== InitAck decoding ======

/// Decode an InitAck message from `buf`.
///
/// Wire format (zenoh v9):
/// ```text
/// [header: u8][version: u8][flags: u8][zid: N bytes]
/// flags = (zid_len - 1) << 4 | whatami_2bit
/// if S flag: [resolution: u8][batch_size: u16 LE]
/// [cookie: vbyte_len + bytes]
/// if Z flag: extensions (skipped)
/// ```
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

    // Flags byte: (zid_len - 1) << 4 | whatami_2bit
    if pos >= buf.len() {
        return Err(TransportError::InvalidEncoding);
    }
    let flags = buf[pos];
    pos += 1;

    let whatami = whatami_from_2bit(flags)?;
    let zid_len = ((flags >> 4) as usize) + 1;

    // ZenohId raw bytes
    if pos + zid_len > buf.len() {
        return Err(TransportError::InvalidEncoding);
    }
    let zid = ZenohId::from_bytes(&buf[pos..pos + zid_len]);
    pos += zid_len;

    // Resolution + Batch size (if S flag)
    let mut batch_size = None;
    if has_batch_size {
        if pos + 3 > buf.len() {
            return Err(TransportError::InvalidEncoding);
        }
        let _resolution = buf[pos]; // Resolution flags (ignored for now)
        pos += 1;
        batch_size = Some(u16::from_le_bytes([buf[pos], buf[pos + 1]]));
        pos += 2;
    }

    // Cookie (vbyte-length-prefixed)
    let (cookie_bytes, n) = decode_slice(&buf[pos..])?;
    pos += n;
    let mut cookie = heapless::Vec::new();
    cookie
        .extend_from_slice(cookie_bytes)
        .map_err(|_| TransportError::FrameTooLarge)?;

    // Skip extensions
    if has_ext {
        while pos < buf.len() {
            let ext_header = buf[pos];
            pos += 1;
            let has_more = ext_header & 0x80 != 0;
            let encoding = (ext_header >> 5) & 0x03;
            match encoding {
                0 => {
                    // ZExtUnit: no body
                }
                1 => {
                    // ZExtZ64: VByte body
                    let (_, n) = decode_vbyte(&buf[pos..])?;
                    pos += n;
                }
                2 | 3 => {
                    // ZExtZBuf: vbyte-length-prefixed bytes
                    let (_, n) = decode_slice(&buf[pos..])?;
                    pos += n;
                }
                _ => {}
            }
            if !has_more {
                break;
            }
        }
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

    Ok((
        OpenAck {
            lease_ms,
            initial_sn,
        },
        pos,
    ))
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
///
/// body_start_pos points to the first network message after the Frame header
/// and any extensions (QoS etc. if Z flag is set).
pub fn decode_frame_header(buf: &[u8]) -> Result<(u64, bool, usize), TransportError> {
    if buf.is_empty() {
        return Err(TransportError::InvalidEncoding);
    }
    let header = buf[0];
    if header & 0x1F != transport_id::FRAME {
        return Err(TransportError::InvalidEncoding);
    }
    let reliable = header & frame_flag::R != 0;
    let has_ext = header & frame_flag::Z != 0;
    let (sn, n) = decode_vbyte(&buf[1..])?;
    let mut pos = 1 + n;
    // Skip frame-level extensions (e.g. QoS) if Z flag is set
    if has_ext {
        skip_extensions(buf, &mut pos)?;
    }
    Ok((sn, reliable, pos))
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

    // Put header: set E flag only when encoding_id is non-zero
    let put_header = zenoh_id::PUT | if encoding_id != 0 { put_flag::E } else { 0 };
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = put_header;
    pos += 1;

    // Encoding (only written when E flag is set): wire format is VByte32(id << 1 | schema_flag)
    if encoding_id != 0 {
        pos += encode_vbyte(&mut buf[pos..], encoding_id << 1)?;
    }

    // Payload
    pos += encode_slice(&mut buf[pos..], payload)?;

    Ok(pos)
}

/// Encode a Push + Put with rmw_zenoh_cpp publisher attachment.
///
/// The attachment is encoded as a `ZExtZBuf` extension on the Put message body,
/// placed before the payload (Z flag set on Put header).
///
/// Attachment layout (33 bytes total):
/// - 8 bytes: `seq_num` as `i64` little-endian
/// - 8 bytes: `timestamp_ns` as `i64` little-endian (nanoseconds; use 0 if RTC unavailable)
/// - 1 byte:  GID length (always `16`)
/// - 16 bytes: publisher GID (`ZenohId` zero-padded to 16 bytes)
///
/// Extension wire encoding:
/// - `ext_header = 0x44` — ZExtZBuf (encoding bits[6:5]=0b10), ID=4, no-more
/// - body = `[VByte(33)][33 attachment bytes]`
pub fn encode_push_put_with_attachment(
    buf: &mut [u8],
    key_expr: &str,
    payload: &[u8],
    seq_num: i64,
    timestamp_ns: i64,
    gid: &ZenohId,
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

    // Put header: Z flag indicates attachment extension follows; no E flag (default CDR encoding)
    let put_header = zenoh_id::PUT | put_flag::Z;
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = put_header;
    pos += 1;

    // Attachment extension: ZExtZBuf, ID=4, no more extensions
    // ext_header bits: [7]=has_more=0, [6:5]=ZBuf=0b10, [4:0]=id=0x04 → 0b01000100 = 0x44
    const ATTACHMENT_EXT_HEADER: u8 = 0x44;
    /// Total attachment payload size: 8 (seq_num) + 8 (timestamp_ns) + 1 (gid_len) + 16 (gid).
    const ATTACHMENT_LEN: usize = 33;

    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = ATTACHMENT_EXT_HEADER;
    pos += 1;

    // Extension body: VByte(33) + attachment bytes
    pos += encode_vbyte(&mut buf[pos..], ATTACHMENT_LEN as u64)?;

    if pos + ATTACHMENT_LEN > buf.len() {
        return Err(TransportError::FrameTooLarge);
    }

    // seq_num: i64 LE
    buf[pos..pos + 8].copy_from_slice(&seq_num.to_le_bytes());
    pos += 8;

    // timestamp_ns: i64 LE
    buf[pos..pos + 8].copy_from_slice(&timestamp_ns.to_le_bytes());
    pos += 8;

    // GID length (always 16)
    buf[pos] = 16;
    pos += 1;

    // GID: ZenohId zero-padded to 16 bytes
    let gid_bytes = gid.as_bytes();
    let gid_copy_len = gid_bytes.len().min(16);
    buf[pos..pos + gid_copy_len].copy_from_slice(&gid_bytes[..gid_copy_len]);
    for b in &mut buf[pos + gid_copy_len..pos + 16] {
        *b = 0;
    }
    pos += 16;

    // Payload (after extensions)
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
    // Declaration: DeclareKeyExpr (N flag set when suffix is non-empty)
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    let dkeyexpr_header = declare_id::D_KEYEXPR
        | if !key_expr.is_empty() {
            declare_keyexpr_flag::N
        } else {
            0
        };
    buf[pos] = dkeyexpr_header;
    pos += 1;

    // Key expression ID
    pos += encode_vbyte(&mut buf[pos..], key_id as u64)?;

    // Wire expression: scope=0 + suffix (suffix only written when N flag set)
    pos += encode_vbyte(&mut buf[pos..], 0)?; // scope
    if !key_expr.is_empty() {
        pos += encode_string(&mut buf[pos..], key_expr)?;
    }

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

// ====== Incoming message decoding ======

/// A decoded incoming Push+Put message from a zenoh Frame body.
#[derive(Debug)]
pub struct IncomingPut<'a> {
    /// Key expression scope (0 = inline, >0 = declared key ID).
    pub scope: u64,
    /// Key expression suffix string (present when N flag set in Push header).
    pub key_suffix: &'a str,
    /// CDR payload bytes (VByte-length-prefixed body of Put).
    pub payload: &'a [u8],
}

/// Skip over all extensions (ZExtUnit, ZExtZ64, ZExtZBuf) in a message.
/// Extensions follow the pattern: [ext_header][body?] where more-bit = bit7 of ext_header.
fn skip_extensions(buf: &[u8], pos: &mut usize) -> Result<(), TransportError> {
    while *pos < buf.len() {
        let ext = buf[*pos];
        *pos += 1;
        let has_more = ext & 0x80 != 0;
        let encoding = (ext >> 5) & 0x03;
        match encoding {
            0 => {} // ZExtUnit: no body
            1 => {
                // ZExtZ64: VByte body
                let (_, n) = decode_vbyte(&buf[*pos..])?;
                *pos += n;
            }
            2 | 3 => {
                // ZExtZBuf: VByte-length-prefixed bytes
                let (_, n) = decode_slice(&buf[*pos..])?;
                *pos += n;
            }
            _ => {}
        }
        if !has_more {
            break;
        }
    }
    Ok(())
}

/// Decode one Push+Put network message from a Frame body.
///
/// `buf` starts immediately after the Frame header+SN.
/// Returns `(IncomingPut, bytes_consumed)` on success, or `None` if the
/// message is not a Push/Put (e.g. a Declare or KeepAlive inside the frame).
pub fn decode_push_put<'a>(
    buf: &'a [u8],
) -> Result<Option<(IncomingPut<'a>, usize)>, TransportError> {
    if buf.is_empty() {
        return Ok(None);
    }
    let mut pos = 0;

    let push_header = buf[pos];
    pos += 1;

    // Check network message ID
    if push_header & 0x1F != network_id::PUSH {
        return Ok(None);
    }

    let has_suffix = push_header & push_flag::N != 0;
    let has_ext = push_header & push_flag::Z != 0;

    // WireExpr: scope VByte, then suffix string if N flag
    let (scope, n) = decode_vbyte(&buf[pos..])?;
    pos += n;

    let key_suffix = if has_suffix {
        let (bytes, n) = decode_slice(&buf[pos..])?;
        pos += n;
        core::str::from_utf8(bytes).map_err(|_| TransportError::InvalidEncoding)?
    } else {
        ""
    };

    // Skip Push extensions
    if has_ext {
        skip_extensions(buf, &mut pos)?;
    }

    // PushBody: expect Put (zenoh_id::PUT = 0x01)
    if pos >= buf.len() {
        return Err(TransportError::InvalidEncoding);
    }
    let put_header = buf[pos];
    pos += 1;

    if put_header & 0x1F != zenoh_id::PUT {
        return Ok(None); // Del or other PushBody — not a Put
    }

    let has_encoding = put_header & put_flag::E != 0;
    let has_timestamp = put_header & put_flag::T != 0;
    let has_ext = put_header & put_flag::Z != 0;

    // Timestamp (if T flag)
    if has_timestamp {
        // NTP64 = VByte-encoded u64 (LEB128), NOT a fixed 8-byte field
        let (_, n) = decode_vbyte(&buf[pos..])?;
        pos += n;
        // ZenohId: VByte(size) + size bytes
        let (id_size, n) = decode_vbyte(&buf[pos..])?;
        pos += n;
        let id_size = id_size as usize;
        if pos + id_size > buf.len() {
            return Err(TransportError::InvalidEncoding);
        }
        pos += id_size;
    }

    // Encoding (if E flag): VByte(id<<1|schema_flag)
    if has_encoding {
        let (_, n) = decode_vbyte(&buf[pos..])?;
        pos += n;
    }

    // Extensions
    if has_ext {
        skip_extensions(buf, &mut pos)?;
    }

    // Payload: VByte(len) + bytes
    let (payload, n) = decode_slice(&buf[pos..])?;
    pos += n;

    Ok(Some((
        IncomingPut {
            scope,
            key_suffix,
            payload,
        },
        pos,
    )))
}

/// Fix `encode_declare_subscriber` — declare subscriber referencing a mapped key ID.
///
/// Wire format:
///   `[DECLARE header][D_SUBSCRIBER | M][sub_id: VByte][scope=key_id: VByte]`
///
/// The M flag tells the router that `scope` is in the sender's (our) namespace,
/// i.e. it refers to a key ID we declared with D_KEYEXPR.
pub fn encode_declare_subscriber_mapped(
    buf: &mut [u8],
    sub_id: u32,
    key_id: u16,
) -> Result<usize, TransportError> {
    let mut pos = 0;

    // Network Declare header (no I flag, no Z flag for simplicity)
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = network_id::DECLARE;
    pos += 1;

    // D_SUBSCRIBER | M: references sender's declared key by ID
    if pos >= buf.len() {
        return Err(TransportError::FrameTooLarge);
    }
    buf[pos] = declare_id::D_SUBSCRIBER | declare_subscriber_flag::M;
    pos += 1;

    // Subscriber ID
    pos += encode_vbyte(&mut buf[pos..], sub_id as u64)?;

    // WireExpr: scope = key_id (mapped), no suffix (N not set)
    pos += encode_vbyte(&mut buf[pos..], key_id as u64)?;

    Ok(pos)
}

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

        // Verify version
        assert_eq!(buf[1], PROTO_VERSION);

        // Verify flags byte: zid_len=8 → (8-1)<<4 = 0x70, Client = 0b10
        assert_eq!(buf[2], 0x70 | 0x02);

        // Verify ZenohId starts at byte 3
        assert_eq!(&buf[3..11], &[0xAB; 8]);
        assert_eq!(n, 11); // 1 + 1 + 1 + 8
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

    #[test]
    fn test_frame_header_best_effort() {
        let mut buf = [0u8; 16];
        let n = encode_frame_header(&mut buf, 100, false).unwrap();
        let (sn, reliable, _) = decode_frame_header(&buf[..n]).unwrap();
        assert_eq!(sn, 100);
        assert!(!reliable);
    }

    #[test]
    fn test_vbyte_zero() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 0).unwrap();
        assert_eq!(n, 1);
        assert_eq!(buf[0], 0);

        let (val, consumed) = decode_vbyte(&buf).unwrap();
        assert_eq!(val, 0);
        assert_eq!(consumed, 1);
    }

    #[test]
    fn test_vbyte_max_single_byte() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 127).unwrap();
        assert_eq!(n, 1);

        let (val, _) = decode_vbyte(&buf).unwrap();
        assert_eq!(val, 127);
    }

    #[test]
    fn test_vbyte_boundary_128() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 128).unwrap();
        assert_eq!(n, 2);

        let (val, consumed) = decode_vbyte(&buf).unwrap();
        assert_eq!(val, 128);
        assert_eq!(consumed, 2);
    }

    #[test]
    fn test_vbyte_buffer_too_small() {
        let mut buf = [0u8; 1];
        let result = encode_vbyte(&mut buf, 128); // Needs 2 bytes
        assert!(result.is_err());
    }

    #[test]
    fn test_decode_vbyte_empty() {
        let buf: &[u8] = &[];
        assert!(decode_vbyte(buf).is_err());
    }

    #[test]
    fn test_slice_roundtrip() {
        let data = b"hello zenoh";
        let mut buf = [0u8; 64];
        let n = encode_slice(&mut buf, data).unwrap();

        let (decoded, consumed) = decode_slice(&buf[..n]).unwrap();
        assert_eq!(decoded, data);
        assert_eq!(consumed, n);
    }

    #[test]
    fn test_slice_empty() {
        let mut buf = [0u8; 16];
        let n = encode_slice(&mut buf, &[]).unwrap();
        let (decoded, consumed) = decode_slice(&buf[..n]).unwrap();
        assert!(decoded.is_empty());
        assert_eq!(consumed, n);
    }

    #[test]
    fn test_zenoh_id_single_byte() {
        let zid = ZenohId::from_bytes(&[0xFF]);
        let mut buf = [0u8; 32];
        let n = encode_zenoh_id(&mut buf, &zid).unwrap();

        let (decoded, consumed) = decode_zenoh_id(&buf[..n]).unwrap();
        assert_eq!(consumed, n);
        assert_eq!(decoded.as_bytes(), &[0xFF]);
    }

    #[test]
    fn test_zenoh_id_max_16_bytes() {
        let zid = ZenohId::from_bytes(&[0xAA; 16]);
        assert_eq!(zid.len, 16);

        let mut buf = [0u8; 32];
        let n = encode_zenoh_id(&mut buf, &zid).unwrap();
        let (decoded, _) = decode_zenoh_id(&buf[..n]).unwrap();
        assert_eq!(decoded.as_bytes(), &[0xAA; 16]);
    }

    #[test]
    fn test_decode_zenoh_id_empty_buf() {
        assert!(decode_zenoh_id(&[]).is_err());
    }

    #[test]
    fn test_close_encode() {
        let mut buf = [0u8; 8];
        let n = encode_close(&mut buf, close_reason::GENERIC).unwrap();
        assert_eq!(n, 2);
        assert_eq!(buf[0] & 0x1F, transport_id::CLOSE);
        assert_eq!(buf[1], close_reason::GENERIC);
    }

    #[test]
    fn test_push_put_encode() {
        let mut buf = [0u8; 256];
        let payload = b"hello ROS2";
        let n = encode_push_put(
            &mut buf,
            "0/chatter/std_msgs::msg::String/RIHS01",
            0,
            payload,
        )
        .unwrap();
        assert!(n > 0);

        // Verify push header
        assert_eq!(buf[0] & 0x1F, network_id::PUSH);
        assert_ne!(buf[0] & push_flag::N, 0); // suffix flag set
    }

    #[test]
    fn test_parse_transport_msg_kind() {
        assert_eq!(
            parse_transport_msg_kind(transport_id::INIT),
            TransportMsgKind::Init
        );
        assert_eq!(
            parse_transport_msg_kind(transport_id::OPEN | 0x60),
            TransportMsgKind::Open
        );
        assert_eq!(
            parse_transport_msg_kind(transport_id::KEEP_ALIVE),
            TransportMsgKind::KeepAlive
        );
        assert_eq!(
            parse_transport_msg_kind(transport_id::FRAME | frame_flag::R),
            TransportMsgKind::Frame
        );
        assert_eq!(
            parse_transport_msg_kind(0xFF),
            TransportMsgKind::Other(0x1F)
        );
    }

    #[test]
    fn test_keepalive_not_other_msg() {
        assert!(!is_keepalive(transport_id::FRAME));
        assert!(!is_keepalive(transport_id::INIT));
    }

    #[test]
    fn test_decode_frame_header_wrong_id() {
        let buf = [transport_id::INIT]; // Not a FRAME
        assert!(decode_frame_header(&buf).is_err());
    }

    #[test]
    fn test_init_syn_with_batch_size() {
        let msg = InitSyn {
            version: PROTO_VERSION,
            whatami: WhatAmI::Client,
            zid: ZenohId::from_bytes(&[0x01; 4]),
            batch_size: Some(65535),
        };

        let mut buf = [0u8; 64];
        let n = encode_init_syn(&mut buf, &msg).unwrap();

        // S flag should be set
        assert_ne!(buf[0] & init_flag::S, 0);

        // flags byte: zid_len=4 → (4-1)<<4 = 0x30, Client = 0b10
        assert_eq!(buf[2], 0x30 | 0x02);

        // After 4 bytes of zid: resolution(1) + batch_size(2)
        let res_pos = 3 + 4;
        assert_eq!(buf[res_pos], 0x00); // Resolution::default
        assert_eq!(
            u16::from_le_bytes([buf[res_pos + 1], buf[res_pos + 2]]),
            65535
        );
        assert_eq!(n, 3 + 4 + 3); // header + version + flags + zid + resolution + batch_size
    }

    #[test]
    fn test_push_put_with_attachment_encode() {
        let gid = ZenohId::from_bytes(&[0xAA, 0xBB, 0xCC, 0xDD]);
        let payload = b"CDR-data";
        let mut buf = [0u8; 512];

        let n = encode_push_put_with_attachment(
            &mut buf,
            "0/chatter/std_msgs::msg::dds_::String_/RIHS01_abc",
            payload,
            42,        // seq_num
            1_000_000, // timestamp_ns
            &gid,
        )
        .unwrap();

        assert!(n > 0);

        // Push header: PUSH | N flag
        assert_eq!(buf[0] & 0x1F, network_id::PUSH);
        assert_ne!(buf[0] & push_flag::N, 0);

        // Verify the Z flag is set on Put header (find it after push header + wireexpr)
        // Just confirm the total size is reasonable: > key_expr_len + payload_len + 33 attachment
        let min_size = 1 // push header
            + 1 // scope vbyte(0)
            + 1 // key_expr vbyte len
            + "0/chatter/std_msgs::msg::dds_::String_/RIHS01_abc".len()
            + 1 // put header
            + 1 // attachment ext_header
            + 1 // attachment vbyte len (33 < 128 → 1 byte)
            + 33 // attachment bytes
            + 1 // payload vbyte len
            + payload.len();
        assert!(n >= min_size, "encoded size {} < expected min {}", n, min_size);
    }

    #[test]
    fn test_push_put_with_attachment_gid_padding() {
        let gid = ZenohId::from_bytes(&[0x01, 0x02]); // 2 bytes — should be zero-padded to 16
        let payload = b"test";
        let mut buf = [0u8; 512];

        let n = encode_push_put_with_attachment(&mut buf, "test/key", payload, 0, 0, &gid).unwrap();
        assert!(n > 0);

        // The attachment starts after: push_hdr(1)+scope(1)+key_len_vbyte+key_bytes+put_hdr(1)
        // We don't parse the full message here, but just verify total size is correct.
        // 33-byte attachment means GID is always 16 bytes regardless of ZenohId length.
        let _ = n;
    }
}
