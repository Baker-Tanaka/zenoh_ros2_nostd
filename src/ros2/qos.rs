//! ROS2 QoS (Quality of Service) profiles.
//!
//! Implements rmw_zenoh_cpp-compatible QoS encoding for liveliness tokens
//! and basic QoS compatibility checking.
//!
//! ## QoS keyexpr format (rmw_zenoh_cpp)
//!
//! ```text
//! <reliability>:<durability>:<history>,<depth>:<deadline_s>,<deadline_ns>:<lifespan_s>,<lifespan_ns>:<liveliness>,<lease_s>,<lease_ns>
//! ```
//!
//! Fields matching the rmw_zenoh_cpp default are omitted (empty string).
//! Example: `::,10:,:,:,,` (all defaults except depth=10)

use heapless::String;

/// Maximum QoS keyexpr string length.
pub const MAX_QOS_KEYEXPR_LEN: usize = 128;

// ── rmw_zenoh_cpp default baseline ────────────────────────────────────────────
// Fields that match these values are omitted in the liveliness token.
const RMW_DEFAULT_RELIABILITY: u8 = 1; // RELIABLE
const RMW_DEFAULT_DURABILITY: u8 = 2; // VOLATILE
const RMW_DEFAULT_HISTORY: u8 = 1; // KEEP_LAST
const RMW_DEFAULT_DEPTH: u32 = 42;
#[allow(dead_code)]
const RMW_DEFAULT_LIVELINESS: u8 = 1; // AUTOMATIC

/// Reliability QoS policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reliability {
    /// Best effort delivery — messages may be lost.
    BestEffort,
    /// Reliable delivery — messages are retransmitted until acknowledged.
    Reliable,
}

impl Reliability {
    /// RMW numeric value for the liveliness token.
    const fn rmw_value(self) -> u8 {
        match self {
            Self::BestEffort => 2,
            Self::Reliable => 1,
        }
    }
}

/// Durability QoS policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Durability {
    /// Volatile — no history kept for late joiners.
    Volatile,
    /// Transient local — publisher keeps last N samples for late joiners.
    TransientLocal,
}

impl Durability {
    /// RMW numeric value for the liveliness token.
    const fn rmw_value(self) -> u8 {
        match self {
            Self::Volatile => 2,
            Self::TransientLocal => 1,
        }
    }
}

/// History QoS policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum History {
    /// Keep last N samples.
    KeepLast(u32),
    /// Keep all samples (not recommended on embedded).
    KeepAll,
}

impl History {
    /// RMW numeric value for the history kind.
    const fn rmw_value(&self) -> u8 {
        match self {
            Self::KeepLast(_) => 1,
            Self::KeepAll => 2,
        }
    }

    /// Depth value (0 for KeepAll).
    const fn depth(&self) -> u32 {
        match self {
            Self::KeepLast(d) => *d,
            Self::KeepAll => 0,
        }
    }
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

    /// Encode QoS as rmw_zenoh_cpp-compatible keyexpr string.
    ///
    /// Format: `<reliability>:<durability>:<history>,<depth>:<deadline_s>,<deadline_ns>:<lifespan_s>,<lifespan_ns>:<liveliness>,<lease_s>,<lease_ns>`
    ///
    /// Fields matching the rmw_zenoh_cpp default baseline are omitted.
    /// Deadline, lifespan, and lease duration always use defaults (INFINITE).
    pub fn to_rmw_qos_keyexpr(&self) -> Result<String<MAX_QOS_KEYEXPR_LEN>, ()> {
        let mut s = String::new();

        // Reliability (default: RELIABLE=1)
        let rel = self.reliability.rmw_value();
        if rel != RMW_DEFAULT_RELIABILITY {
            push_u8(&mut s, rel)?;
        }
        s.push(':').map_err(|_| ())?;

        // Durability (default: VOLATILE=2)
        let dur = self.durability.rmw_value();
        if dur != RMW_DEFAULT_DURABILITY {
            push_u8(&mut s, dur)?;
        }
        s.push(':').map_err(|_| ())?;

        // History (default: KEEP_LAST=1)
        let hist = self.history.rmw_value();
        if hist != RMW_DEFAULT_HISTORY {
            push_u8(&mut s, hist)?;
        }
        s.push(',').map_err(|_| ())?;

        // Depth (default: 42)
        let depth = self.history.depth();
        if depth != RMW_DEFAULT_DEPTH {
            push_u32(&mut s, depth)?;
        }
        s.push(':').map_err(|_| ())?;

        // Deadline: sec,nsec (always default → empty)
        s.push(',').map_err(|_| ())?;
        s.push(':').map_err(|_| ())?;

        // Lifespan: sec,nsec (always default → empty)
        s.push(',').map_err(|_| ())?;
        s.push(':').map_err(|_| ())?;

        // Liveliness (default: AUTOMATIC=1), lease_sec, lease_nsec
        s.push(',').map_err(|_| ())?;
        s.push(',').map_err(|_| ())?;

        Ok(s)
    }

    /// Check if a subscriber QoS is compatible with a publisher QoS.
    ///
    /// ROS2 QoS compatibility rules:
    /// - Reliable pub ↔ any sub: OK
    /// - Best-effort pub + reliable sub: INCOMPATIBLE
    /// - Volatile pub + transient-local sub: INCOMPATIBLE
    /// - Transient-local pub ↔ any sub: OK
    pub fn is_compatible(pub_qos: &Qos, sub_qos: &Qos) -> bool {
        // Reliability: best-effort publisher cannot serve reliable subscriber
        if pub_qos.reliability == Reliability::BestEffort
            && sub_qos.reliability == Reliability::Reliable
        {
            return false;
        }
        // Durability: volatile publisher cannot serve transient-local subscriber
        if pub_qos.durability == Durability::Volatile
            && sub_qos.durability == Durability::TransientLocal
        {
            return false;
        }
        true
    }
}

impl Default for Qos {
    fn default() -> Self {
        Self::DEFAULT
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

fn push_u8<const N: usize>(s: &mut String<N>, val: u8) -> Result<(), ()> {
    if val >= 100 {
        s.push((b'0' + val / 100) as char).map_err(|_| ())?;
    }
    if val >= 10 {
        s.push((b'0' + (val / 10) % 10) as char).map_err(|_| ())?;
    }
    s.push((b'0' + val % 10) as char).map_err(|_| ())
}

fn push_u32<const N: usize>(s: &mut String<N>, val: u32) -> Result<(), ()> {
    if val == 0 {
        s.push('0').map_err(|_| ())?;
        return Ok(());
    }
    let mut buf = [0u8; 10];
    let mut pos = buf.len();
    let mut v = val;
    while v > 0 {
        pos -= 1;
        buf[pos] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    let digits = core::str::from_utf8(&buf[pos..]).map_err(|_| ())?;
    s.push_str(digits).map_err(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_qos() {
        let qos = Qos::default();
        assert_eq!(qos.reliability, Reliability::Reliable);
        assert_eq!(qos.durability, Durability::Volatile);
        assert_eq!(qos.history, History::KeepLast(10));
    }

    #[test]
    fn test_sensor_data_qos() {
        assert_eq!(Qos::SENSOR_DATA.reliability, Reliability::BestEffort);
        assert_eq!(Qos::SENSOR_DATA.durability, Durability::Volatile);
        assert_eq!(Qos::SENSOR_DATA.history, History::KeepLast(5));
    }

    #[test]
    fn test_rmw_qos_keyexpr_default() {
        // Qos::DEFAULT = reliable(1), volatile(2), keep_last(10)
        // reliability=1 matches default → empty
        // durability=2 matches default → empty
        // history=1 matches default → empty
        // depth=10 ≠ 42 → "10"
        let s = Qos::DEFAULT.to_rmw_qos_keyexpr().unwrap();
        assert_eq!(s.as_str(), "::,10:,:,:,,");
    }

    #[test]
    fn test_rmw_qos_keyexpr_sensor_data() {
        // best_effort(2), volatile(2), keep_last(5)
        // reliability=2 ≠ 1 → "2"
        // durability=2 matches → empty
        // depth=5 ≠ 42 → "5"
        let s = Qos::SENSOR_DATA.to_rmw_qos_keyexpr().unwrap();
        assert_eq!(s.as_str(), "2::,5:,:,:,,");
    }

    #[test]
    fn test_rmw_qos_keyexpr_transient_local() {
        let qos = Qos {
            reliability: Reliability::Reliable,
            durability: Durability::TransientLocal,
            history: History::KeepLast(1),
        };
        // durability=1 ≠ 2 → "1"
        // depth=1 ≠ 42 → "1"
        let s = qos.to_rmw_qos_keyexpr().unwrap();
        assert_eq!(s.as_str(), ":1:,1:,:,:,,");
    }

    #[test]
    fn test_rmw_qos_keyexpr_keep_all() {
        let qos = Qos {
            reliability: Reliability::Reliable,
            durability: Durability::Volatile,
            history: History::KeepAll,
        };
        // history=2 ≠ 1 → "2"
        // depth=0 ≠ 42 → "0"
        let s = qos.to_rmw_qos_keyexpr().unwrap();
        assert_eq!(s.as_str(), "::2,0:,:,:,,");
    }

    #[test]
    fn test_rmw_qos_keyexpr_depth_42_matches_default() {
        let qos = Qos {
            reliability: Reliability::Reliable,
            durability: Durability::Volatile,
            history: History::KeepLast(42),
        };
        // Everything matches default → all empty
        let s = qos.to_rmw_qos_keyexpr().unwrap();
        assert_eq!(s.as_str(), "::,:,:,:,,");
    }

    #[test]
    fn test_compatibility_reliable_reliable() {
        assert!(Qos::is_compatible(&Qos::DEFAULT, &Qos::DEFAULT));
    }

    #[test]
    fn test_compatibility_best_effort_best_effort() {
        assert!(Qos::is_compatible(&Qos::SENSOR_DATA, &Qos::SENSOR_DATA));
    }

    #[test]
    fn test_compatibility_reliable_pub_best_effort_sub() {
        assert!(Qos::is_compatible(&Qos::DEFAULT, &Qos::SENSOR_DATA));
    }

    #[test]
    fn test_incompatible_best_effort_pub_reliable_sub() {
        assert!(!Qos::is_compatible(&Qos::SENSOR_DATA, &Qos::DEFAULT));
    }

    #[test]
    fn test_incompatible_volatile_pub_transient_local_sub() {
        let sub_qos = Qos {
            reliability: Reliability::Reliable,
            durability: Durability::TransientLocal,
            history: History::KeepLast(10),
        };
        assert!(!Qos::is_compatible(&Qos::DEFAULT, &sub_qos));
    }

    #[test]
    fn test_compatible_transient_local_pub_volatile_sub() {
        let pub_qos = Qos {
            reliability: Reliability::Reliable,
            durability: Durability::TransientLocal,
            history: History::KeepLast(10),
        };
        assert!(Qos::is_compatible(&pub_qos, &Qos::DEFAULT));
    }
}
