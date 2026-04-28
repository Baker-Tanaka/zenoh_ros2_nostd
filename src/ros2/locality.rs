//! Locality control for ROS2 pub/sub over Zenoh.
//!
//! Controls whether a [`Publisher`](super::publisher::Publisher) sends only to
//! local subscribers (within the same node process) or also across the network,
//! and whether a [`Subscription`](super::subscription::Subscription) receives
//! only locally-dispatched messages or also network traffic.

/// Controls the locality of message delivery for publishers and subscribers.
///
/// Matches the semantics of `rmw_zenoh_cpp`'s `RMW_PUBLISHER_LOCAL_ONLY` and
/// `RMW_SUBSCRIPTION_LOCAL_ONLY` modes, adapted for single-node embedded use.
///
/// # Default
/// [`Locality::Any`] — both local and remote delivery (backward-compatible).
///
/// # Example
/// ```rust,ignore
/// use zenoh_ros2_nostd::prelude::*;
///
/// // MCU-internal task-to-task channel (no network traffic)
/// static LOCAL_PUB: Publisher<SensorMsg, { cdr_size_of!(bytes(8)) }, 2> =
///     Publisher::with_locality(SENSOR_TOPIC, Locality::SessionLocal);
/// static LOCAL_SUB: Subscription<SensorMsg, { cdr_size_of!(bytes(8)) }, 2> =
///     Subscription::with_locality(Locality::SessionLocal);
///
/// // Normal network publisher (default behaviour)
/// static NET_PUB: Publisher<SensorMsg, { cdr_size_of!(bytes(8)) }, 2> =
///     Publisher::new(SENSOR_TOPIC);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Locality {
    /// Deliver to both local subscribers and the remote network.
    ///
    /// This is the default and matches the behaviour of all existing code.
    Any,

    /// Deliver only within the same session (same MCU process).
    ///
    /// **Publisher**: the message is dispatched directly to matching local
    /// [`Subscription`](super::subscription::Subscription)s.  No frame is sent
    /// to the Zenoh router, and no liveliness token is declared.
    ///
    /// **Subscription**: the subscription is not declared to the router
    /// (`DeclareSubscriber` is skipped).  Only messages from a local
    /// `SessionLocal` or `Any` publisher are received.
    SessionLocal,

    /// Deliver only across the network (remote endpoints).
    ///
    /// **Publisher**: frames are written to the transport as usual, but the
    /// message is NOT dispatched to local subscriptions.
    ///
    /// **Subscription**: receives network traffic normally.  Local publishers
    /// with `Remote` locality will not dispatch to it.
    Remote,
}

impl Default for Locality {
    fn default() -> Self {
        Locality::Any
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Locality {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Locality::Any => defmt::write!(fmt, "Any"),
            Locality::SessionLocal => defmt::write!(fmt, "SessionLocal"),
            Locality::Remote => defmt::write!(fmt, "Remote"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_any() {
        assert_eq!(Locality::default(), Locality::Any);
    }

    #[test]
    fn is_copy() {
        let a = Locality::SessionLocal;
        let b = a; // copy
        assert_eq!(a, b);
    }
}
