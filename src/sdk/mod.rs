//! Public SDK — rclpy-like high-level API for ROS2 over Zenoh.
//!
//! This is the **only** public module of the crate.  All other modules
//! (`ros2`, `session`, `transport`, `buf`, `cdr`) are internal implementation
//! details and should not be used directly.
//!
//! # Two styles
//!
//! ## A. Callback function style
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd::prelude::*;
//!
//! let mut node = Node::builder("talker")
//!     .domain_id(0)
//!     .build(transport)
//!     .await?;
//!
//! let pub_h = node.register_static_publisher(&MY_PUB).await?;
//! node.subscribe_with_dispatch(CHATTER_TOPIC, &MY_SUB).await?;
//! node.spin().await;
//! ```
//!
//! ## B. Trait implementation style
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd::prelude::*;
//!
//! struct MyNode { /* ... */ }
//! impl NodeCallbacks for MyNode {
//!     fn on_message(&mut self, topic: &str, payload: &[u8]) { /* ... */ }
//!     fn on_timer(&mut self) { /* ... */ }
//! }
//! ```

pub mod node;
pub mod publisher;
pub mod subscription;
pub mod timer;
pub mod traits;

pub use node::{Node, NodeBuilder};
pub use publisher::PublisherHandle;
pub use subscription::SubscriptionHandle;
pub use timer::TimerHandle;
pub use traits::NodeCallbacks;
