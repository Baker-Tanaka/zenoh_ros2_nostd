//! Typed ROS2 topic subscriber.
//!
//! Receives CDR-encoded messages from the session's message channel
//! and deserializes them into the target ROS2 message type.

use core::marker::PhantomData;

use serde::Deserialize;

use crate::cdr;
use crate::error::Error;
use crate::session::subscriber::{SubMessage, Subscriber};

/// A typed topic subscriber that deserializes CDR messages.
///
/// - `M`: the ROS2 message type (must implement `Deserialize`).
/// - `MSG_SIZE`: max raw payload size per message.
/// - `QUEUE`: subscriber channel capacity.
pub struct TopicSubscriber<M, const MSG_SIZE: usize, const QUEUE: usize> {
    inner: Subscriber<MSG_SIZE, QUEUE>,
    _phantom: PhantomData<M>,
}

impl<M, const MSG_SIZE: usize, const QUEUE: usize> TopicSubscriber<M, MSG_SIZE, QUEUE>
where
    M: for<'de> Deserialize<'de>,
{
    /// Create a new typed topic subscriber.
    pub const fn new() -> Self {
        Self {
            inner: Subscriber::new(),
            _phantom: PhantomData,
        }
    }

    /// Receive and deserialize the next message.
    pub async fn recv(&self) -> Result<M, Error> {
        let raw: SubMessage<MSG_SIZE> = self.inner.recv().await;
        let (msg, _) = cdr::deserialize_with_header::<M>(&raw.payload).map_err(Error::Cdr)?;
        Ok(msg)
    }

    /// Try to receive and deserialize a message without blocking.
    pub fn try_recv(&self) -> Option<Result<M, Error>> {
        let raw = self.inner.try_recv()?;
        Some(
            cdr::deserialize_with_header::<M>(&raw.payload)
                .map(|(msg, _)| msg)
                .map_err(Error::Cdr),
        )
    }

    /// Access the inner (raw) subscriber for pushing data from the rx loop.
    pub fn inner(&self) -> &Subscriber<MSG_SIZE, QUEUE> {
        &self.inner
    }
}
