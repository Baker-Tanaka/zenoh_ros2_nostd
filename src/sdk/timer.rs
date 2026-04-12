//! Timer handle for periodic callbacks.
//!
//! [`TimerHandle`] allows cancelling or checking the state of a timer
//! registered with the node via [`Node::create_timer`](super::node::Node).
//!
//! Timers are driven internally by the node's `spin()` loop using
//! `embassy_time::Timer`.

use embassy_time::Duration;

/// Handle to a registered timer.
///
/// Currently a lightweight marker. Future extensions may allow
/// cancellation or period adjustment.
pub struct TimerHandle {
    /// Timer period.
    pub(crate) period: Duration,
    /// Timer ID within the node (index into the timers vector).
    pub(crate) id: u8,
}

impl TimerHandle {
    /// Get the timer period.
    pub fn period(&self) -> Duration {
        self.period
    }

    /// Get the timer ID.
    pub fn id(&self) -> u8 {
        self.id
    }
}
