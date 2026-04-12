//! Convenience re-exports for `use zenoh_ros2_nostd::prelude::*`.
//!
//! Imports the most commonly used SDK types, traits, and ROS2 message
//! utilities in a single glob import.
//!
//! # Example
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd::prelude::*;
//!
//! // Now you can use Node, Publisher, Subscription, TopicKeyExpr, etc.
//! // without individual imports.
//! static MY_PUB: Publisher<MyMsg, 256, 4> = Publisher::new(MY_TOPIC);
//! ```

// SDK public API
pub use crate::sdk::{
    Node, NodeBuilder, NodeCallbacks, PublisherHandle, SubscriptionHandle, TimerHandle,
};

// Error types
pub use crate::error::Error;

// ROS2 types needed by users
pub use crate::ros2::keyexpr::{ActionKeyExprs, TopicKeyExpr};
pub use crate::ros2::message_trait::RosMessage;
pub use crate::ros2::msg;
pub use crate::ros2::publisher::Publisher;
pub use crate::ros2::qos::{Durability, History, Qos, Reliability};
pub use crate::ros2::subscription::Subscription;
pub use crate::session::reconnect::ReconnectPolicy;
pub use crate::transport::protocol::ZenohId;

// Re-export embassy_time::Duration so users don't need to depend on embassy-time
pub use embassy_time::Duration;

// Re-export serde traits for message definitions
pub use serde::{Deserialize, Serialize};

// Re-export derive macro when the `derive` feature is enabled
#[cfg(feature = "derive")]
pub use crate::RosMessage;
