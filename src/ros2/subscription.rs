//! Typed ROS2 topic subscription.
//!
//! [`Subscription`] is designed to be placed in a `static` so any async task
//! can call [`Subscription::try_recv`] or [`Subscription::recv`] without
//! holding a session reference.  The Zenoh task pushes incoming payloads into
//! the subscription's internal queue via the [`SubscriptionDispatch`] trait
//! inside [`Node::spin`](super::node::Node::spin).
//!
//! # Usage
//! ```rust,ignore
//! use zenoh_ros2_nostd::ros2::Subscription;
//!
//! static CHATTER_SUB: Subscription<StringMsg, 144, 4> = Subscription::new();
//!
//! // Register with the node to start receiving:
//! node.subscribe(CHATTER_TOPIC, &CHATTER_SUB).await?;
//!
//! // From any async task:
//! while let Some(result) = CHATTER_SUB.try_recv() {
//!     let msg = result?;
//! }
//! ```

use core::marker::PhantomData;

use serde::Deserialize;

use super::locality::Locality;
use crate::cdr;
use crate::error::Error;
use crate::session::subscriber::Subscriber;

// ── SubscriptionDispatch — object-safe trait for Node routing ────────────────

/// Object-safe trait implemented by [`Subscription`].
///
/// [`Node`](super::node::Node) stores `&'static dyn SubscriptionDispatch`
/// references in its routing table and calls [`push_raw`](Self::push_raw)
/// when an incoming message key matches the registered key ID.
pub trait SubscriptionDispatch: Sync {
    /// Push a raw CDR payload (with 4-byte encapsulation header) into the
    /// subscription's internal queue for later retrieval by the application.
    fn push_raw(&self, payload: &[u8]);

    /// Locality of this subscription.
    ///
    /// Defaults to [`Locality::Any`] for backward compatibility with external
    /// implementations that do not override this method.
    fn locality(&self) -> Locality {
        Locality::Any
    }
}

// ── Subscription ──────────────────────────────────────────────────────────────

/// A typed ROS2 topic subscription, suitable for use as a `static`.
///
/// - `M`        — message type (must implement [`serde::Deserialize`])
/// - `MSG_SIZE` — maximum raw CDR payload size per message (bytes)
/// - `QUEUE`    — number of received messages buffered
///
/// # Example
/// ```rust,ignore
/// static SUB: Subscription<StringMsg, 144, 4> = Subscription::new();
/// ```
pub struct Subscription<M, const MSG_SIZE: usize, const QUEUE: usize> {
    inner: Subscriber<MSG_SIZE, QUEUE>,
    locality: Locality,
    _phantom: PhantomData<fn() -> M>,
}

impl<M, const MSG_SIZE: usize, const QUEUE: usize> Subscription<M, MSG_SIZE, QUEUE>
where
    M: for<'de> Deserialize<'de>,
{
    /// Create a new subscription.  Safe to call as a `static` initializer.
    pub const fn new() -> Self {
        Self {
            inner: Subscriber::new(),
            locality: Locality::Any,
            _phantom: PhantomData,
        }
    }

    /// Create a subscription with explicit locality control.
    ///
    /// Safe to call as a `static` initializer.
    ///
    /// ```rust,ignore
    /// static LOCAL_SUB: Subscription<SensorMsg, 64, 2> =
    ///     Subscription::with_locality(Locality::SessionLocal);
    /// ```
    pub const fn with_locality(locality: Locality) -> Self {
        Self {
            inner: Subscriber::new(),
            locality,
            _phantom: PhantomData,
        }
    }

    /// Await the next message from the topic, deserializing from CDR.
    ///
    /// Yields until a message is available.
    pub async fn recv(&self) -> Result<M, Error> {
        let raw = self.inner.recv().await;
        cdr::deserialize_with_header::<M>(&raw.payload)
            .map(|(m, _)| m)
            .map_err(Error::Cdr)
    }

    /// Try to receive the next message without blocking.
    ///
    /// Returns `None` if the queue is empty.
    pub fn try_recv(&self) -> Option<Result<M, Error>> {
        let raw = self.inner.try_recv()?;
        Some(
            cdr::deserialize_with_header::<M>(&raw.payload)
                .map(|(m, _)| m)
                .map_err(Error::Cdr),
        )
    }

    /// Drain and discard all pending messages from the internal queue.
    ///
    /// Call this after a session reconnect to prevent stale messages (received
    /// before the disconnect) from being processed in the new session context.
    ///
    /// ```rust,ignore
    /// // In zenoh_task, before re-subscribing after reconnect:
    /// CHATTER_SUB.clear();
    /// node.subscribe(CHATTER_TOPIC, CHATTER_SUB.as_dispatch()).await?;
    /// ```
    pub fn clear(&self) {
        self.inner.clear();
    }

    /// Return a `&'static dyn SubscriptionDispatch` suitable for [`Node::subscribe`](super::node::Node::subscribe).
    ///
    /// This hides the `as &'static dyn SubscriptionDispatch` boilerplate from the caller.
    ///
    /// ```rust,ignore
    /// node.subscribe(CHATTER_TOPIC, CHATTER_SUB.as_dispatch()).await?;
    /// ```
    pub fn as_dispatch(&'static self) -> &'static dyn SubscriptionDispatch {
        self
    }
}

impl<M, const MSG_SIZE: usize, const QUEUE: usize> SubscriptionDispatch
    for Subscription<M, MSG_SIZE, QUEUE>
{
    fn push_raw(&self, payload: &[u8]) {
        self.inner.push(payload);
    }

    fn locality(&self) -> Locality {
        self.locality
    }
}
