//! # zenoh-ros2-nostd
//!
//! A `#![no_std]` ROS2 topic pub/sub library over Zenoh for embedded systems.
//!
//! Designed for use with the [embassy](https://embassy.dev/) async runtime.
//! Communicates with standard ROS2 nodes via `rmw_zenoh_cpp`-compatible
//! key expressions and CDR serialization.
//!
//! ## Getting Started
//!
//! The public API lives in the [`sdk`] module. Use [`prelude`] for convenience:
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd::prelude::*;
//!
//! // Define your ROS2 message (or use pre-defined ones from `msg` module)
//! #[derive(Serialize, Deserialize)]
//! struct StringMsg {
//!     data: heapless::String<128>,
//! }
//!
//! // Define topic key expression
//! const CHATTER: TopicKeyExpr = msg::std_msgs::String::CHATTER;
//!
//! // Static publisher & subscription (required for no_std)
//! static PUB: Publisher<StringMsg, 137, 4> = Publisher::new(CHATTER);
//! static SUB: Subscription<StringMsg, 137, 4> = Subscription::new();
//!
//! async fn run(transport: impl embedded_io_async::Read + embedded_io_async::Write) {
//!     let mut node = Node::builder("my_node")
//!         .domain_id(0)
//!         .build(transport)
//!         .await
//!         .unwrap();
//!
//!     let pub_handle = node.register_static_publisher(&PUB).await.unwrap();
//!     let sub_handle = node.subscribe_with_dispatch(CHATTER, &SUB).await.unwrap();
//!
//!     // Publish from any task
//!     pub_handle.publish(&StringMsg { data: "hello".into() }).await.unwrap();
//!
//!     // Receive in another task
//!     let msg = sub_handle.recv().await.unwrap();
//!
//!     // Or drive everything with spin()
//!     node.spin().await;
//! }
//! ```
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────┐
//! │  sdk/   (Node, PublisherHandle) │  ← Public API (rclpy-like)
//! ├─────────────────────────────────┤
//! │  ros2/  (KeyExpr, QoS, Msg)    │  ← Internal: ROS2 protocol
//! ├─────────────────────────────────┤
//! │  session/  (Session, Reconnect) │  ← Internal: session management
//! ├─────────────────────────────────┤
//! │  transport/  (Handshake, Frame) │  ← Internal: Zenoh v9 protocol
//! ├─────────────────────────────────┤
//! │  buf/ + cdr/                    │  ← Internal: buffer & serialization
//! └─────────────────────────────────┘
//! ```
//!
//! ## Features
//!
//! | Feature | Default | Description |
//! |---------|---------|-------------|
//! | `defmt` | ✓ | Enable `defmt` logging for embedded targets |
//! | `log` | | Enable `log` crate logging for std targets |
//! | `alloc` | | Enable `alloc`-dependent APIs |
//! | `wasi` | | Enable WASI socket/time adapters for `wasm32-wasip1` |
//! | `derive` | | Enable `#[derive(RosMessage)]` proc macro |
//!
//! ## Derive Macro
//!
//! With the `derive` feature, use `#[derive(RosMessage)]` to auto-generate
//! message metadata:
//!
//! ```rust,ignore
//! #[derive(Serialize, Deserialize, RosMessage)]
//! #[ros_message(
//!     type_name = "my_pkg::msg::dds_::MyMsg_",
//!     type_hash = "RIHS01_...",
//! )]
//! struct MyMsg {
//!     value: f64,
//! }
//!
//! // Now MyMsg::TYPE_NAME, MyMsg::TYPE_HASH, and MyMsg::topic() are available
//! const MY_TOPIC: TopicKeyExpr = MyMsg::topic(0, "my_topic");
//! ```

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

#[cfg(feature = "alloc")]
extern crate alloc;

// Re-export derive macro when the `derive` feature is enabled.
#[cfg(feature = "derive")]
pub use zenoh_ros2_nostd_derive::RosMessage;

// Hidden re-export for proc-macro generated code.
// The derive macro references `zenoh_ros2_nostd::__private::RosMessage` to
// avoid requiring users to import the trait manually.
#[doc(hidden)]
pub mod __private {
    pub use crate::ros2::keyexpr::TopicKeyExpr;
    pub use crate::ros2::message_trait::RosMessage;
}

#[macro_use]
pub mod logging;

// ── Public API ────────────────────────────────────────────────────────────────

pub mod prelude;
pub mod sdk;

// ── Public support types ──────────────────────────────────────────────────────

pub mod error;

// ── Internal modules ──────────────────────────────────────────────────────────
// These are `pub` only because integration tests import from them directly.
// Users should rely on `sdk` and `prelude` instead.

#[doc(hidden)]
pub mod buf;
#[doc(hidden)]
pub mod cdr;
#[doc(hidden)]
pub mod ros2;
#[doc(hidden)]
pub mod session;
#[doc(hidden)]
pub mod transport;

// ── WASI-specific adapters (feature-gated) ────────────────────────────────────

#[cfg(feature = "wasi")]
pub mod wasi;
