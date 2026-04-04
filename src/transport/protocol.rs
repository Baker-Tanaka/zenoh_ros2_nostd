//! Zenoh protocol message types (minimal subset for client-mode pub/sub).
//!
//! Implements the message definitions for Zenoh protocol v8,
//! compatible with zenoh router 1.x.

/// Zenoh protocol version (matches zenoh 1.x).
pub const PROTO_VERSION: u8 = 9;

// --- WhatAmI ---

/// Node role in the zenoh network.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WhatAmI {
    Router = 0x01,
    Peer = 0x02,
    Client = 0x04,
}

// --- ZenohId ---

/// Zenoh node identifier (up to 16 bytes).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct ZenohId {
    pub bytes: [u8; 16],
    pub len: u8,
}

impl ZenohId {
    /// Create a ZenohId from a byte slice (up to 16 bytes).
    pub fn from_bytes(data: &[u8]) -> Self {
        let len = data.len().min(16);
        let mut bytes = [0u8; 16];
        bytes[..len].copy_from_slice(&data[..len]);
        Self {
            bytes,
            len: len as u8,
        }
    }

    /// Get the ID as a byte slice.
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
}

impl core::fmt::Debug for ZenohId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in self.as_bytes() {
            write!(f, "{:02x}", b)?;
        }
        Ok(())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ZenohId {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{=[u8]:x}", self.as_bytes());
    }
}

// --- Transport message IDs ---

pub mod transport_id {
    pub const OAM: u8 = 0x00;
    pub const INIT: u8 = 0x01;
    pub const OPEN: u8 = 0x02;
    pub const CLOSE: u8 = 0x03;
    pub const KEEP_ALIVE: u8 = 0x04;
    pub const FRAME: u8 = 0x05;
    pub const FRAGMENT: u8 = 0x06;
    pub const JOIN: u8 = 0x07;
}

// --- Header flag positions ---

pub mod init_flag {
    /// ACK flag: 0 = InitSyn, 1 = InitAck.
    pub const A: u8 = 1 << 5;
    /// Batch size extension present.
    pub const S: u8 = 1 << 6;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

pub mod open_flag {
    /// ACK flag: 0 = OpenSyn, 1 = OpenAck.
    pub const A: u8 = 1 << 5;
    /// Lease unit: 0 = milliseconds, 1 = seconds.
    pub const T: u8 = 1 << 6;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

pub mod frame_flag {
    /// Reliability: 0 = best-effort, 1 = reliable.
    pub const R: u8 = 1 << 5;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

// --- Network message IDs ---

pub mod network_id {
    pub const PUSH: u8 = 0x1d;
    pub const REQUEST: u8 = 0x1c;
    pub const RESPONSE: u8 = 0x1b;
    pub const RESPONSE_FINAL: u8 = 0x1a;
    pub const INTEREST: u8 = 0x19;
    pub const DECLARE: u8 = 0x1e;
    pub const OAM: u8 = 0x1f;
}

pub mod push_flag {
    /// Named: wire expression has a name/suffix string.
    pub const N: u8 = 1 << 5;
    /// Mapping: sender's declared key ID is used for the scope.
    pub const M: u8 = 1 << 6;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

// --- Zenoh message IDs ---

pub mod zenoh_id {
    pub const OAM: u8 = 0x00;
    pub const PUT: u8 = 0x01;
    pub const DEL: u8 = 0x02;
    pub const QUERY: u8 = 0x03;
    pub const REPLY: u8 = 0x04;
    pub const ERR: u8 = 0x05;
}

pub mod put_flag {
    /// Timestamp present.
    pub const T: u8 = 1 << 5;
    /// Encoding present (custom encoding).
    pub const E: u8 = 1 << 6;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

// --- Declaration IDs ---

pub mod declare_id {
    pub const D_KEYEXPR: u8 = 0x00;
    pub const U_KEYEXPR: u8 = 0x01;
    pub const D_SUBSCRIBER: u8 = 0x02;
    pub const U_SUBSCRIBER: u8 = 0x03;
    pub const D_QUERYABLE: u8 = 0x04;
    pub const U_QUERYABLE: u8 = 0x05;
    pub const D_TOKEN: u8 = 0x06;
    pub const U_TOKEN: u8 = 0x07;
    pub const D_FINAL: u8 = 0x1a;
}

pub mod declare_keyexpr_flag {
    /// Named: wire expression has a name/suffix string.
    pub const N: u8 = 1 << 5;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

pub mod declare_subscriber_flag {
    /// Named: wire expression has a suffix string.
    pub const N: u8 = 1 << 5;
    /// Mapped: scope references a sender-declared key ID.
    pub const M: u8 = 1 << 6;
    /// More extensions follow.
    pub const Z: u8 = 1 << 7;
}

// --- High-level transport messages ---

/// InitSyn: sent by client to start a session.
#[derive(Debug, Clone)]
pub struct InitSyn {
    pub version: u8,
    pub whatami: WhatAmI,
    pub zid: ZenohId,
    pub batch_size: Option<u16>,
}

/// InitAck: reply from router.
#[derive(Debug, Clone)]
pub struct InitAck {
    pub version: u8,
    pub whatami: WhatAmI,
    pub zid: ZenohId,
    pub cookie: heapless::Vec<u8, 256>,
    pub batch_size: Option<u16>,
}

/// OpenSyn: sent by client after receiving InitAck.
#[derive(Debug, Clone)]
pub struct OpenSyn {
    pub lease_ms: u64,
    pub initial_sn: u64,
    pub cookie: heapless::Vec<u8, 256>,
}

/// OpenAck: final reply from router, session is open.
#[derive(Debug, Clone)]
pub struct OpenAck {
    pub lease_ms: u64,
    pub initial_sn: u64,
}

/// KeepAlive message.
#[derive(Debug, Clone, Copy)]
pub struct KeepAlive;

/// Close message.
#[derive(Debug, Clone, Copy)]
pub struct Close {
    pub reason: u8,
}

// --- Network-level structures ---

/// Wire expression: references a key expression by scope ID and/or suffix.
#[derive(Debug, Clone)]
pub struct WireExpr<'a> {
    /// Resource ID scope (0 = root / inline).
    pub scope: u64,
    /// Optional string suffix (e.g., the full key expression for inline mode).
    pub suffix: Option<&'a str>,
}

/// Close reason codes.
pub mod close_reason {
    pub const GENERIC: u8 = 0x00;
    pub const UNSUPPORTED: u8 = 0x01;
    pub const INVALID: u8 = 0x02;
    pub const MAX_SESSIONS: u8 = 0x03;
    pub const MAX_LINKS: u8 = 0x04;
    pub const EXPIRED: u8 = 0x05;
}
