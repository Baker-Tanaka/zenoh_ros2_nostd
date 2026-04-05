//! Typed ROS2 topic publisher.
//!
//! Publishes serde-serializable ROS2 messages as CDR over zenoh.
//! Each message includes the rmw_zenoh_cpp-compatible publisher attachment
//! (sequence number, timestamp, and publisher GID).

use embedded_io_async::{Read, Write};
use serde::Serialize;

use super::keyexpr::TopicKeyExpr;
use crate::cdr;
use crate::error::Error;
use crate::session::Session;
use crate::transport::protocol::ZenohId;

/// A typed topic publisher that serializes messages to CDR.
///
/// `TopicPublisher` holds a reference to the session and a scratch buffer
/// for CDR serialization. The scratch buffer must be large enough to hold
/// the CDR-encoded message plus the 4-byte encapsulation header.
///
/// Each call to [`publish`] automatically increments an internal sequence
/// number and attaches the publisher GID, making the output compatible with
/// `rmw_zenoh_cpp` subscribers.
pub struct TopicPublisher<'a, T: Read + Write, const TX: usize, const RX: usize> {
    session: &'a Session<T, TX, RX>,
    topic_ke: &'a TopicKeyExpr,
    cdr_buf: &'a mut [u8],
    /// Monotonically increasing sequence number per publish call.
    seq_num: u64,
    /// Publisher GID — copied from the session's ZenohId at construction time.
    gid: ZenohId,
}

impl<'a, T: Read + Write, const TX: usize, const RX: usize> TopicPublisher<'a, T, TX, RX> {
    /// Create a new topic publisher.
    pub fn new(
        session: &'a Session<T, TX, RX>,
        topic_ke: &'a TopicKeyExpr,
        cdr_buf: &'a mut [u8],
    ) -> Self {
        let gid = *session.zid();
        Self {
            session,
            topic_ke,
            cdr_buf,
            seq_num: 0,
            gid,
        }
    }

    /// Publish a message.
    ///
    /// The message is serialized to CDR with the encapsulation header,
    /// then sent as a zenoh Put on the topic's key expression.
    /// An rmw_zenoh_cpp-compatible attachment (sequence number, timestamp, GID)
    /// is included so that ROS2 subscribers can track message ordering.
    pub async fn publish<M: Serialize>(&mut self, msg: &M) -> Result<(), Error> {
        // Serialize to CDR with encapsulation header
        let n = cdr::serialize_with_header(self.cdr_buf, msg).map_err(|e| Error::Cdr(e))?;

        // Build key expression
        let ke = self.topic_ke.to_key_expr().map_err(|_| Error::InvalidArgument)?;

        // Sequence number and timestamp
        let seq = self.seq_num as i64;
        self.seq_num = self.seq_num.wrapping_add(1);

        // Use elapsed time in nanoseconds as timestamp (no RTC available on bare-metal MCU)
        let timestamp_ns = embassy_time::Instant::now().as_micros() as i64 * 1000;

        // Publish with attachment
        self.session
            .put_with_attachment(ke.as_str(), &self.cdr_buf[..n], seq, timestamp_ns, &self.gid)
            .await?;

        Ok(())
    }

    /// Get the topic key expression definition.
    pub fn topic(&self) -> &TopicKeyExpr {
        self.topic_ke
    }
}
