//! ROS2 Node abstraction.
//!
//! A node represents a participant in the ROS2 graph.
//! It owns a session reference and provides methods to create
//! publishers and subscribers with correct key expressions.

use embedded_io_async::{Read, Write};

use super::keyexpr::TopicKeyExpr;
use super::topic_publisher::TopicPublisher;
use crate::session::Session;

/// A ROS2 node running on the embedded device.
///
/// Provides the interface for creating typed topic publishers.
pub struct Node<'a, T: Read + Write, const TX: usize, const RX: usize> {
    session: &'a Session<T, TX, RX>,
    /// Node namespace (e.g., "" for root).
    namespace: &'a str,
    /// Node name (e.g., "mcu_sensor").
    name: &'a str,
}

impl<'a, T: Read + Write, const TX: usize, const RX: usize> Node<'a, T, TX, RX> {
    /// Create a new ROS2 node bound to a zenoh session.
    pub fn new(session: &'a Session<T, TX, RX>, namespace: &'a str, name: &'a str) -> Self {
        ros2_info!("node created: ns={} name={}", namespace, name);
        Self {
            session,
            namespace,
            name,
        }
    }

    /// Create a typed topic publisher.
    ///
    /// - `topic_ke`: the topic key expression definition.
    /// - `cdr_buf`: a mutable buffer for CDR serialization scratch space.
    pub fn create_publisher<'b>(
        &'b self,
        topic_ke: &'b TopicKeyExpr,
        cdr_buf: &'b mut [u8],
    ) -> TopicPublisher<'b, T, TX, RX> {
        TopicPublisher::new(self.session, topic_ke, cdr_buf)
    }

    /// Get the node namespace.
    pub fn namespace(&self) -> &str {
        self.namespace
    }

    /// Get the node name.
    pub fn name(&self) -> &str {
        self.name
    }

    /// Get a reference to the underlying session.
    pub fn session(&self) -> &Session<T, TX, RX> {
        self.session
    }
}
