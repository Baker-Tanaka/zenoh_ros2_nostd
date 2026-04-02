//! ROS2 QoS (Quality of Service) profiles.
//!
//! Defines the QoS settings relevant for zenoh-based ROS2 communication.

/// Reliability QoS policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reliability {
    /// Best effort delivery — messages may be lost.
    BestEffort,
    /// Reliable delivery — messages are retransmitted until acknowledged.
    Reliable,
}

/// Durability QoS policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// Volatile — no history kept for late joiners.
    Volatile,
    /// Transient local — publisher keeps last N samples for late joiners.
    TransientLocal,
}

/// History QoS policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum History {
    /// Keep last N samples.
    KeepLast(u32),
    /// Keep all samples (not recommended on embedded).
    KeepAll,
}

/// Complete QoS profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Qos {
    pub reliability: Reliability,
    pub durability: Durability,
    pub history: History,
}

impl Qos {
    /// Default QoS: reliable, volatile, keep-last(10).
    pub const DEFAULT: Self = Self {
        reliability: Reliability::Reliable,
        durability: Durability::Volatile,
        history: History::KeepLast(10),
    };

    /// Sensor data QoS: best-effort, volatile, keep-last(5).
    pub const SENSOR_DATA: Self = Self {
        reliability: Reliability::BestEffort,
        durability: Durability::Volatile,
        history: History::KeepLast(5),
    };

    /// Parameter events QoS: reliable, volatile, keep-last(1000).
    pub const PARAMETERS: Self = Self {
        reliability: Reliability::Reliable,
        durability: Durability::Volatile,
        history: History::KeepLast(1000),
    };

    /// Encode QoS as a string for the liveliness token.
    ///
    /// Format: `<reliability><durability>` (single-char codes).
    pub fn to_liveliness_str(&self) -> &'static str {
        match (self.reliability, self.durability) {
            (Reliability::BestEffort, Durability::Volatile) => "BV",
            (Reliability::BestEffort, Durability::TransientLocal) => "BT",
            (Reliability::Reliable, Durability::Volatile) => "RV",
            (Reliability::Reliable, Durability::TransientLocal) => "RT",
        }
    }
}

impl Default for Qos {
    fn default() -> Self {
        Self::DEFAULT
    }
}
