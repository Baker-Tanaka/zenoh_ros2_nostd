//! Typed ROS2 topic publisher.
//!
//! Publishes serde-serializable ROS2 messages as CDR over zenoh.

use embedded_io_async::{Read, Write};
use serde::Serialize;

use super::keyexpr::TopicKeyExpr;
use crate::cdr;
use crate::error::Error;
use crate::session::Session;

/// A typed topic publisher that serializes messages to CDR.
///
/// `TopicPublisher` holds a reference to the session and a scratch buffer
/// for CDR serialization. The scratch buffer must be large enough to hold
/// the CDR-encoded message plus the 4-byte encapsulation header.
pub struct TopicPublisher<'a, T: Read + Write, const TX: usize, const RX: usize> {
    session: &'a Session<T, TX, RX>,
    topic_ke: &'a TopicKeyExpr,
    cdr_buf: &'a mut [u8],
}

impl<'a, T: Read + Write, const TX: usize, const RX: usize> TopicPublisher<'a, T, TX, RX> {
    /// Create a new topic publisher.
    pub fn new(
        session: &'a Session<T, TX, RX>,
        topic_ke: &'a TopicKeyExpr,
        cdr_buf: &'a mut [u8],
    ) -> Self {
        Self {
            session,
            topic_ke,
            cdr_buf,
        }
    }

    /// Publish a message.
    ///
    /// The message is serialized to CDR with the encapsulation header,
    /// then sent as a zenoh Put on the topic's key expression.
    pub async fn publish<M: Serialize>(&mut self, msg: &M) -> Result<(), Error> {
        // Serialize to CDR with encapsulation header
        let n = cdr::serialize_with_header(self.cdr_buf, msg).map_err(|e| Error::Cdr(e))?;

        // Build key expression
        let ke = self.topic_ke.to_key_expr().map_err(|_| Error::InvalidArgument)?;

        // Publish
        self.session.put(ke.as_str(), &self.cdr_buf[..n]).await?;

        Ok(())
    }

    /// Get the topic key expression definition.
    pub fn topic(&self) -> &TopicKeyExpr {
        self.topic_ke
    }
}
