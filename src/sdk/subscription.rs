//! Typed ROS2 topic subscription handle for the SDK.
//!
//! [`SubscriptionHandle`] provides a channel-based interface for receiving
//! deserialized messages.  It wraps the internal `ros2::Subscription`.
//!
//! The SDK [`Node`](super::node::Node) also supports callback-based
//! subscriptions that don't require a handle — see
//! [`Node::create_subscription`](super::node::Node::create_subscription).
//!
//! # Example
//!
//! ```rust,ignore
//! let sub = node.create_subscription_channel::<StringMsg>("chatter", QoS::default());
//! // In another task:
//! let msg = sub.recv().await?;
//! ```

use serde::Deserialize;

use crate::error::Error;
use crate::ros2::subscription::Subscription;

/// A typed subscription handle for channel-based message reception.
///
/// Created by [`Node::create_subscription_channel`](super::node::Node).
///
/// - `M`: message type (must implement `serde::Deserialize`)
/// - `MSG_SIZE`: maximum raw CDR payload size per message
/// - `QUEUE`: number of received messages buffered
#[must_use = "drop した場合でも Subscription はキューを保持しますが、recv() を呼べる唯一の手段が失われます。変数に束縛してください。"]
pub struct SubscriptionHandle<
    M: for<'de> Deserialize<'de> + 'static,
    const MSG_SIZE: usize = 512,
    const QUEUE: usize = 4,
> {
    inner: &'static Subscription<M, MSG_SIZE, QUEUE>,
}

impl<M: for<'de> Deserialize<'de> + 'static, const MSG_SIZE: usize, const QUEUE: usize>
    SubscriptionHandle<M, MSG_SIZE, QUEUE>
{
    /// Create a handle from a static subscription reference.
    pub(crate) fn new(inner: &'static Subscription<M, MSG_SIZE, QUEUE>) -> Self {
        Self { inner }
    }

    /// Await the next message, deserializing from CDR.
    pub async fn recv(&self) -> Result<M, Error> {
        self.inner.recv().await
    }

    /// Try to receive the next message without blocking.
    ///
    /// Returns `None` if the queue is empty.
    pub fn try_recv(&self) -> Option<Result<M, Error>> {
        self.inner.try_recv()
    }

    /// Discard all pending messages.
    ///
    /// Call after reconnect to prevent stale messages from being processed.
    pub fn clear(&self) {
        self.inner.clear();
    }
}

impl<M: for<'de> Deserialize<'de> + 'static, const MSG_SIZE: usize, const QUEUE: usize> Clone
    for SubscriptionHandle<M, MSG_SIZE, QUEUE>
{
    fn clone(&self) -> Self {
        *self
    }
}

impl<M: for<'de> Deserialize<'de> + 'static, const MSG_SIZE: usize, const QUEUE: usize> Copy
    for SubscriptionHandle<M, MSG_SIZE, QUEUE>
{
}
