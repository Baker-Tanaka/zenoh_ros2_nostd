//! Trait-based node callback interface (rclpy class-inheritance style).
//!
//! Implement [`NodeCallbacks`] on a user struct to receive callbacks for
//! initialization, incoming messages, and timer ticks.
//!
//! # Example
//!
//! ```rust,ignore
//! struct MyNode { /* state */ }
//!
//! impl NodeCallbacks for MyNode {
//!     fn on_init(&mut self, ctx: &mut NodeContext) {
//!         ctx.create_subscription::<StringMsg>("chatter", QoS::default());
//!     }
//!     fn on_message(&mut self, topic: &str, payload: &[u8]) {
//!         // handle incoming message
//!     }
//!     fn on_timer(&mut self) {
//!         // periodic work
//!     }
//! }
//! ```

/// Callback trait for the trait-based node style.
///
/// All methods have default no-op implementations so the user only
/// overrides what they need.
///
/// Register publishers, subscriptions, and timers directly on the
/// [`Node`](super::node::Node) before calling
/// [`Node::spin_with_callbacks`](super::node::Node::spin_with_callbacks).
pub trait NodeCallbacks {
    /// Called when an incoming message arrives on a subscribed topic.
    ///
    /// `topic` is the key expression suffix (topic name portion).
    /// `payload` is the raw CDR bytes including the 4-byte encapsulation header.
    fn on_message(&mut self, _topic: &str, _payload: &[u8]) {}

    /// Called when a registered timer fires.
    fn on_timer(&mut self) {}
}
