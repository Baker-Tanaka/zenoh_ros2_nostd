//! Subscriber handle for receiving data from the session.
//!
//! Subscribers receive data via an embassy Channel.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::Vec;

/// A received message from a subscriber.
pub struct SubMessage<const N: usize> {
    /// The raw payload bytes.
    pub payload: Vec<u8, N>,
}

/// Subscriber that receives messages via an embassy Channel.
///
/// - `MSG_SIZE`: max payload size per message.
/// - `QUEUE`: channel capacity (number of messages buffered).
pub struct Subscriber<const MSG_SIZE: usize, const QUEUE: usize> {
    channel: Channel<CriticalSectionRawMutex, SubMessage<MSG_SIZE>, QUEUE>,
}

impl<const MSG_SIZE: usize, const QUEUE: usize> Subscriber<MSG_SIZE, QUEUE> {
    /// Create a new subscriber with an internal message channel.
    pub const fn new() -> Self {
        Self {
            channel: Channel::new(),
        }
    }

    /// Wait for the next message (blocks the async task).
    pub async fn recv(&self) -> SubMessage<MSG_SIZE> {
        self.channel.receive().await
    }

    /// Try to receive a message without blocking.
    pub fn try_recv(&self) -> Option<SubMessage<MSG_SIZE>> {
        self.channel.try_receive().ok()
    }

    /// Push a message into the subscriber's channel (called by the session rx loop).
    pub fn push(&self, payload: &[u8]) {
        let mut msg_payload = Vec::new();
        // Silently truncate if payload exceeds MSG_SIZE
        let len = payload.len().min(MSG_SIZE);
        let _ = msg_payload.extend_from_slice(&payload[..len]);
        let _ = self.channel.try_send(SubMessage {
            payload: msg_payload,
        });
    }
}
