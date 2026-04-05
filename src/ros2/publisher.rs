//! Typed ROS2 topic publisher.
//!
//! [`Publisher`] is designed to be placed in a `static` so any async task can
//! call [`Publisher::send`] without holding a session reference.  The Zenoh
//! task drains queued payloads via the [`PublisherDrain`] trait inside
//! [`Node::spin`].
//!
//! # Usage
//! ```rust,ignore
//! use zenoh_ros2_nostd::ros2::{Publisher, TopicKeyExpr};
//!
//! static CHATTER_PUB: Publisher<StringMsg, 144, 4> = Publisher::new(CHATTER_TOPIC);
//!
//! // From any async task:
//! CHATTER_PUB.send(&msg).await?;
//!
//! // Register with the node so it drains and transmits:
//! node.register_publisher(&CHATTER_PUB);
//! ```

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use serde::Serialize;

use super::keyexpr::TopicKeyExpr;
use crate::cdr;
use crate::error::Error;

// ── CDR payload envelope ──────────────────────────────────────────────────────

/// A CDR-encoded message with its sequence number and timestamp, ready to send.
///
/// This is an internal type owned by the publisher's channel.
pub struct CdrPayload<const N: usize> {
    /// CDR-encoded bytes (with 4-byte encapsulation header).
    pub data: heapless::Vec<u8, N>,
    /// Per-publisher sequence number (rmw_zenoh_cpp attachment).
    pub seq_num: i64,
    /// Nanoseconds since boot (used as approximate timestamp).
    pub timestamp_ns: i64,
}

// ── PublisherDrain — object-safe trait for Node routing ──────────────────────

/// Object-safe trait implemented by [`Publisher`].
///
/// [`Node`](super::node::Node) uses this trait to drain pending CDR payloads
/// from each registered publisher during [`Node::spin`].
///
/// # Safety
/// Implementations must be `Sync` (they are placed in `static` globals).
pub trait PublisherDrain: Sync {
    /// Topic key expression for this publisher.
    fn topic_ke(&self) -> &TopicKeyExpr;

    /// Try to drain one CDR payload into `out_buf`.
    ///
    /// Returns `Some((bytes_written, seq_num, timestamp_ns))` if a pending
    /// message was available and successfully copied.  Returns `None` if the
    /// queue is empty.
    ///
    /// **Precondition**: `out_buf.len()` must be `≥` the publisher's `CDR_CAP`.
    fn try_drain_into(&self, out_buf: &mut [u8]) -> Option<(usize, i64, i64)>;
}

// ── Publisher ─────────────────────────────────────────────────────────────────

/// A typed ROS2 topic publisher, suitable for use as a `static`.
///
/// - `M`       — message type (must implement [`serde::Serialize`])
/// - `CDR_CAP` — CDR buffer capacity per message (bytes, must include the 4-byte header)
/// - `QUEUE`   — maximum number of messages buffered before `send` blocks
///
/// # Example
/// ```rust,ignore
/// static PUB: Publisher<StringMsg, 144, 4> = Publisher::new(CHATTER_TOPIC);
///
/// // From any async task:
/// PUB.send(&msg).await?;
/// ```
pub struct Publisher<M, const CDR_CAP: usize, const QUEUE: usize> {
    topic: TopicKeyExpr,
    channel: Channel<CriticalSectionRawMutex, CdrPayload<CDR_CAP>, QUEUE>,
    /// Per-publisher sequence counter (rmw_zenoh_cpp attachment requirement).
    seq_num: Mutex<CriticalSectionRawMutex, i64>,
    _phantom: core::marker::PhantomData<fn(M) -> M>,
}

impl<M: Serialize, const CDR_CAP: usize, const QUEUE: usize> Publisher<M, CDR_CAP, QUEUE> {
    /// Create a new publisher.  Safe to call as a `static` initializer.
    pub const fn new(topic: TopicKeyExpr) -> Self {
        Self {
            topic,
            channel: Channel::new(),
            seq_num: Mutex::new(0),
            _phantom: core::marker::PhantomData,
        }
    }

    /// Serialize `msg` to CDR and enqueue it for the Zenoh task to transmit.
    ///
    /// Awaits (yields) if the queue is full until space is available.
    /// The sequence number and timestamp are stamped at the time of this call.
    pub async fn send(&self, msg: &M) -> Result<(), Error> {
        let mut raw = [0u8; CDR_CAP];
        let n = cdr::serialize_with_header(&mut raw, msg).map_err(Error::Cdr)?;

        let seq = {
            let mut guard = self.seq_num.lock().await;
            let cur = *guard;
            *guard = guard.wrapping_add(1);
            cur
        };
        // Convert microseconds to nanoseconds for the rmw_zenoh_cpp attachment timestamp.
        let ts = (embassy_time::Instant::now().as_micros() as i64).saturating_mul(1_000);

        let mut data: heapless::Vec<u8, CDR_CAP> = heapless::Vec::new();
        let _ = data.extend_from_slice(&raw[..n]);

        self.channel
            .send(CdrPayload {
                data,
                seq_num: seq,
                timestamp_ns: ts,
            })
            .await;
        Ok(())
    }
}

impl<M: Serialize, const CDR_CAP: usize, const QUEUE: usize> PublisherDrain
    for Publisher<M, CDR_CAP, QUEUE>
{
    fn topic_ke(&self) -> &TopicKeyExpr {
        &self.topic
    }

    fn try_drain_into(&self, out_buf: &mut [u8]) -> Option<(usize, i64, i64)> {
        let payload = self.channel.try_receive().ok()?;
        let n = payload.data.len().min(out_buf.len());
        out_buf[..n].copy_from_slice(&payload.data[..n]);
        Some((n, payload.seq_num, payload.timestamp_ns))
    }
}
