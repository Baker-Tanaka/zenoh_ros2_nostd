//! Typed ROS2 topic publisher handle for the SDK.
//!
//! [`PublisherHandle`] is returned by [`Node::create_publisher`](super::node::Node::create_publisher)
//! and can be used from any async task to publish messages.
//!
//! Unlike the internal `ros2::Publisher` which requires `static` placement,
//! `PublisherHandle` is an owned handle backed by a `&'static` internal publisher.
//!
//! # Example
//!
//! ```rust,ignore
//! let pub_handle = node.create_publisher::<Twist>("cmd_vel", QoS::default());
//! pub_handle.publish(&twist_msg).await?;
//! ```

use serde::Serialize;

use crate::error::Error;
use crate::ros2::publisher::Publisher;

/// A typed publisher handle returned by [`Node::create_publisher`](super::node::Node).
///
/// Wraps a `&'static Publisher` internally. The handle itself is lightweight
/// and can be cloned (it is `Copy`).
///
/// - `M`: message type (must implement `serde::Serialize`)
/// - `CDR_CAP`: maximum CDR-encoded message size in bytes (including 4-byte header)
/// - `QUEUE`: number of messages buffered before `publish` blocks
pub struct PublisherHandle<
    M: Serialize + 'static,
    const CDR_CAP: usize = 512,
    const QUEUE: usize = 4,
> {
    inner: &'static Publisher<M, CDR_CAP, QUEUE>,
}

impl<M: Serialize + 'static, const CDR_CAP: usize, const QUEUE: usize>
    PublisherHandle<M, CDR_CAP, QUEUE>
{
    /// Create a handle from a static publisher reference.
    pub(crate) fn new(inner: &'static Publisher<M, CDR_CAP, QUEUE>) -> Self {
        Self { inner }
    }

    /// Serialize `msg` to CDR and enqueue for transmission.
    ///
    /// Awaits if the internal queue is full.
    pub async fn publish(&self, msg: &M) -> Result<(), Error> {
        self.inner.send(msg).await
    }

    /// Try to publish without blocking.
    ///
    /// Returns `Err(Error::BufferFull)` if the queue is full.
    pub fn try_publish(&self, msg: &M) -> Result<(), Error> {
        self.inner.try_send(msg)
    }
}

impl<M: Serialize + 'static, const CDR_CAP: usize, const QUEUE: usize> Clone
    for PublisherHandle<M, CDR_CAP, QUEUE>
{
    fn clone(&self) -> Self {
        *self
    }
}

impl<M: Serialize + 'static, const CDR_CAP: usize, const QUEUE: usize> Copy
    for PublisherHandle<M, CDR_CAP, QUEUE>
{
}
