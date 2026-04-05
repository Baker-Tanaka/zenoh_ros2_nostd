//! Configuration types for ROS2-over-Zenoh embedded sessions.
//!
//! [`ZenohRos2Config`] bundles the Zenoh session identity with ROS2-specific
//! settings (domain ID, lease time).  Construct one per device and share it
//! via a `static` or by reference.

use crate::transport::protocol::ZenohId;

/// Configuration for a ROS2-over-Zenoh embedded session.
///
/// # Example
///
/// ```rust,ignore
/// use zenoh_ros2_nostd::ros2::ZenohRos2Config;
/// use zenoh_ros2_nostd::transport::protocol::ZenohId;
///
/// const ZID: ZenohId = ZenohId::from_bytes(&[0xBA, 0xBE, 0xCA, 0xFE]);
///
/// const CONFIG: ZenohRos2Config = ZenohRos2Config::new(ZID)
///     .with_domain_id(0)
///     .with_lease_ms(10_000);
/// ```
#[derive(Clone, Copy, Debug)]
pub struct ZenohRos2Config {
    /// Zenoh node identifier — **must be unique per device** in the network.
    pub zid: ZenohId,
    /// ROS2 domain ID.  Match `ROS_DOMAIN_ID` on the host side (default: `0`).
    pub domain_id: u32,
    /// Keepalive lease offered to the router in milliseconds.
    /// The router may negotiate a lower value; `10_000` is a safe default.
    pub lease_ms: u64,
}

impl ZenohRos2Config {
    /// Create a configuration with sensible embedded defaults
    /// (domain ID `0`, lease `10 000` ms).
    pub const fn new(zid: ZenohId) -> Self {
        Self {
            zid,
            domain_id: 0,
            lease_ms: 10_000,
        }
    }

    /// Override the ROS2 domain ID.
    pub const fn with_domain_id(mut self, id: u32) -> Self {
        self.domain_id = id;
        self
    }

    /// Override the keepalive lease duration in milliseconds.
    pub const fn with_lease_ms(mut self, ms: u64) -> Self {
        self.lease_ms = ms;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_defaults() {
        let zid = ZenohId::from_bytes(&[0x01]);
        let cfg = ZenohRos2Config::new(zid);
        assert_eq!(cfg.domain_id, 0);
        assert_eq!(cfg.lease_ms, 10_000);
    }

    #[test]
    fn test_builders() {
        let zid = ZenohId::from_bytes(&[0x02]);
        let cfg = ZenohRos2Config::new(zid)
            .with_domain_id(5)
            .with_lease_ms(30_000);
        assert_eq!(cfg.domain_id, 5);
        assert_eq!(cfg.lease_ms, 30_000);
    }
}
