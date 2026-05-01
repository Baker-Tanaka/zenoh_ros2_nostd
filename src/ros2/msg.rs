//! Pre-defined ROS2 message type constants (type names, RIHS01 hashes, and
//! helpers for building [`TopicKeyExpr`]s without hand-typing hash strings).
//!
//! Hashes marked as `VERIFIED` are taken from the official `rosidl_typesupport_c`
//! generated code or the rmw_zenoh_cpp design document.  Hashes marked as
//! `TODO: verify` are placeholder values and **must not** be used with a live
//! ROS2 system — replace them with the output of `ros2 interface show` or
//! `rosidl_get_typesupport_identifier`.
//!
//! # CDR-serializable message structs
//!
//! This module also provides serde-compatible structs for common ROS2 message
//! types.  They can be used directly with [`Publisher`] and [`Subscription`]:
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd::ros2::msg::geometry_msgs::{Twist, Vector3};
//!
//! let twist = Twist {
//!     linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
//!     angular: Vector3 { x: 0.0, y: 0.0, z: 0.5 },
//! };
//! ```
//!
//! # Topic helpers
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd::ros2::msg;
//!
//! const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::topic(0, "chatter");
//! const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::CHATTER;
//! ```

use super::keyexpr::TopicKeyExpr;
use super::message_trait::RosMessage;

/// `std_msgs` package message type constants.
pub mod std_msgs {
    use super::TopicKeyExpr;
    use serde::{Deserialize, Serialize};

    /// `std_msgs/msg/String`.
    pub struct String;

    impl String {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "std_msgs::msg::dds_::String_";

        /// RIHS01 type hash for `std_msgs/msg/String`. **VERIFIED.**
        pub const TYPE_HASH: &'static str =
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        ///
        /// ```rust,ignore
        /// const MY_TOPIC: TopicKeyExpr = msg::std_msgs::String::topic(0, "my_topic");
        /// ```
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }

        /// Pre-built topic for the canonical `/chatter` topic (domain 0).
        pub const CHATTER: TopicKeyExpr = Self::topic(0, "chatter");
    }

    /// `std_msgs/msg/Float32` type metadata.
    pub struct Float32Type;

    impl Float32Type {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Float32_";

        /// RIHS01 type hash for `std_msgs/msg/Float32`. **VERIFIED.**
        pub const TYPE_HASH: &'static str =
            "RIHS01_7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }
    }

    /// CDR-serializable `std_msgs/Float32` message.
    ///
    /// CDR size: 4 bytes body. With encapsulation header: 8 bytes total.
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct Float32Msg {
        pub data: f32,
    }

    /// CDR buffer capacity for a `Float32` message (4 header + 4 body = 8 bytes).
    pub const FLOAT32_CDR_CAP: usize = 8;

    /// `std_msgs/msg/Int32` type metadata.
    pub struct Int32Type;

    impl Int32Type {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "std_msgs::msg::dds_::Int32_";

        /// RIHS01 type hash for `std_msgs/msg/Int32`. **VERIFIED.**
        pub const TYPE_HASH: &'static str =
            "RIHS01_b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }
    }

    /// CDR-serializable `std_msgs/Int32` message.
    ///
    /// CDR size: 4 bytes body. With encapsulation header: 8 bytes total.
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct Int32Msg {
        pub data: i32,
    }

    /// CDR buffer capacity for an `Int32` message (4 header + 4 body = 8 bytes).
    pub const INT32_CDR_CAP: usize = 8;
}

/// `geometry_msgs` package message type constants and CDR-serializable structs.
pub mod geometry_msgs {
    use super::RosMessage;
    use super::TopicKeyExpr;
    use serde::{Deserialize, Serialize};

    /// `geometry_msgs/msg/Vector3` — 3D vector with f64 components.
    ///
    /// CDR size: 24 bytes (3 × f64, no alignment padding).
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Vector3 {
        /// Zero vector.
        pub const ZERO: Self = Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
    }

    impl Default for Vector3 {
        fn default() -> Self {
            Self::ZERO
        }
    }

    /// `geometry_msgs/msg/Twist` — velocity in free space (linear + angular).
    ///
    /// CDR size: 48 bytes (2 × Vector3 = 6 × f64, no alignment padding).
    /// With CDR encapsulation header: 52 bytes total.
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    impl Default for Twist {
        fn default() -> Self {
            Self {
                linear: Vector3::ZERO,
                angular: Vector3::ZERO,
            }
        }
    }

    /// CDR buffer capacity for a Twist message (4 header + 48 body = 52 bytes).
    pub const TWIST_CDR_CAP: usize = 52;

    /// Type metadata for `geometry_msgs/msg/Twist`.
    pub struct TwistType;

    impl TwistType {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "geometry_msgs::msg::dds_::Twist_";

        /// RIHS01 type hash for `geometry_msgs/msg/Twist`.
        ///
        /// **TODO: verify** — replace with the hash from
        /// `ros2 interface show geometry_msgs/msg/Twist` or the
        /// `rosidl_typesupport_c` generated code before use with a live ROS2 system.
        pub const TYPE_HASH: &'static str =
            "RIHS01_9b0e20a73f3b74c00f80ed1f26a4c7568a63aeec6e79ba04b64bcd5b7f49a2a5";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        ///
        /// **Caution**: verify [`TYPE_HASH`](Self::TYPE_HASH) before use.
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }

        /// Pre-built topic for `/cmd_vel` (domain 0).
        ///
        /// **Caution**: verify [`TYPE_HASH`](Self::TYPE_HASH) before use.
        pub const CMD_VEL: TopicKeyExpr = Self::topic(0, "cmd_vel");
    }

    impl RosMessage for Twist {
        const TYPE_NAME: &'static str = TwistType::TYPE_NAME;
        const TYPE_HASH: &'static str = TwistType::TYPE_HASH;
    }
}

/// `rcl_interfaces` package — ROS2 logging and parameter types.
pub mod rcl_interfaces {
    use super::TopicKeyExpr;
    use heapless::String;
    use serde::{Deserialize, Serialize};

    /// `builtin_interfaces/msg/Time` — seconds + nanoseconds.
    ///
    /// CDR size: 8 bytes (i32 sec + u32 nanosec).
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct Stamp {
        pub sec: i32,
        pub nanosec: u32,
    }

    /// `rcl_interfaces/msg/Log` — log message published on `/rosout`.
    ///
    /// CDR layout (LE, with alignment):
    /// - `stamp`: `{sec: i32, nanosec: u32}` (8 bytes)
    /// - `level`: `u8` (1 byte + 3 padding to align next u32)
    /// - `name`: CDR string (u32 len + data + NUL)
    /// - `msg`: CDR string
    /// - `file`: CDR string
    /// - `function`: CDR string
    /// - `line`: `u32`
    #[derive(Debug, Clone, Deserialize)]
    pub struct Log<const N: usize = 128> {
        pub stamp: Stamp,
        pub level: u8,
        pub name: String<N>,
        pub msg: String<N>,
        pub file: String<N>,
        pub function: String<N>,
        pub line: u32,
    }

    /// Log level constants matching ROS2 `rcl_interfaces/msg/Log`.
    pub mod log_level {
        pub const DEBUG: u8 = 10;
        pub const INFO: u8 = 20;
        pub const WARN: u8 = 30;
        pub const ERROR: u8 = 40;
        pub const FATAL: u8 = 50;
    }

    /// Type metadata for `rcl_interfaces/msg/Log`.
    pub struct LogType;

    impl LogType {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "rcl_interfaces::msg::dds_::Log_";

        /// RIHS01 type hash for `rcl_interfaces/msg/Log`. **VERIFIED.**
        pub const TYPE_HASH: &'static str =
            "RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }

        /// Pre-built topic for `/rosout` (domain 0).
        pub const ROSOUT: TopicKeyExpr = Self::topic(0, "rosout");
    }

    /// CDR buffer capacity for a Log message.
    ///
    /// Conservative estimate: 4 header + 8 stamp + 1 level + 3 pad
    /// + 4×(4+128+1) strings + 4 line = ~552. Round up.
    pub const LOG_CDR_CAP: usize = 600;
}

/// `action_msgs` and `unique_identifier_msgs` — common ROS2 action types.
///
/// These types are used by all ROS2 actions regardless of the specific
/// action definition.  Action-specific Goal/Result/Feedback types must
/// be defined by the user.
///
/// # CDR layout
///
/// All structs use `#[derive(Serialize, Deserialize)]` and produce
/// standard CDR LE encoding compatible with `rmw_zenoh_cpp`.
pub mod action_msgs {
    #[allow(unused_imports)]
    use super::RosMessage;
    use super::TopicKeyExpr;
    use serde::{Deserialize, Serialize};

    /// `unique_identifier_msgs/msg/UUID` — 16-byte goal identifier.
    ///
    /// In CDR, this is a fixed-size array of 16 bytes (no length prefix).
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct GoalId {
        pub uuid: [u8; 16],
    }

    impl GoalId {
        /// A zero (nil) goal ID.
        pub const ZERO: Self = Self { uuid: [0u8; 16] };
    }

    /// `builtin_interfaces/msg/Time` — seconds + nanoseconds.
    ///
    /// CDR size: 8 bytes (i32 sec + u32 nanosec, no padding).
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct Stamp {
        pub sec: i32,
        pub nanosec: u32,
    }

    impl Stamp {
        /// Zero timestamp.
        pub const ZERO: Self = Self { sec: 0, nanosec: 0 };
    }

    /// `action_msgs/msg/GoalInfo` — goal ID + timestamp.
    ///
    /// CDR size: 24 bytes (16 UUID + 4 sec + 4 nanosec).
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct GoalInfo {
        pub goal_id: GoalId,
        pub stamp: Stamp,
    }

    /// Goal status constants from `action_msgs/msg/GoalStatus`.
    pub mod goal_status {
        /// Unknown status.
        pub const STATUS_UNKNOWN: i8 = 0;
        /// Goal accepted, waiting to be processed.
        pub const STATUS_ACCEPTED: i8 = 1;
        /// Goal is being executed.
        pub const STATUS_EXECUTING: i8 = 2;
        /// Goal is being cancelled.
        pub const STATUS_CANCELING: i8 = 3;
        /// Goal finished successfully.
        pub const STATUS_SUCCEEDED: i8 = 4;
        /// Goal was cancelled.
        pub const STATUS_CANCELED: i8 = 5;
        /// Goal was aborted due to failure.
        pub const STATUS_ABORTED: i8 = 6;
    }

    /// Response for `_action/send_goal` service — common to all actions.
    ///
    /// CDR size: 12 bytes (1 bool + 3 padding + 4 sec + 4 nanosec).
    /// With CDR header: 16 bytes.
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct SendGoalResponse {
        pub accepted: bool,
        pub stamp: Stamp,
    }

    /// CDR buffer capacity for `SendGoalResponse` (4 header + 12 body).
    pub const SEND_GOAL_RESP_CDR_CAP: usize = 16;

    /// Request for `_action/get_result` service — common to all actions.
    ///
    /// CDR size: 16 bytes (16 UUID).
    /// With CDR header: 20 bytes.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct GetResultRequest {
        pub goal_id: GoalId,
    }

    /// CDR buffer capacity for `GetResultRequest` (4 header + 16 body).
    pub const GET_RESULT_REQ_CDR_CAP: usize = 20;

    /// Request for `_action/cancel_goal` service.
    ///
    /// CDR size: 24 bytes (16 UUID + 4 sec + 4 nanosec).
    /// With CDR header: 28 bytes.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct CancelGoalRequest {
        pub goal_info: GoalInfo,
    }

    /// CDR buffer capacity for `CancelGoalRequest` (4 header + 24 body).
    pub const CANCEL_GOAL_REQ_CDR_CAP: usize = 28;

    /// Cancel goal return codes from `action_msgs/srv/CancelGoal_Response`.
    pub mod cancel_goal {
        /// Success.
        pub const ERROR_NONE: i8 = 0;
        /// Rejected by server.
        pub const ERROR_REJECTED: i8 = 1;
        /// Unknown goal ID.
        pub const ERROR_UNKNOWN_GOAL_ID: i8 = 2;
        /// Goal already terminated.
        pub const ERROR_GOAL_TERMINATED: i8 = 3;
    }

    /// Type metadata for `action_msgs/srv/CancelGoal`.
    pub struct CancelGoalType;

    impl CancelGoalType {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "action_msgs::srv::dds_::CancelGoal_";

        /// RIHS01 type hash for `action_msgs/srv/CancelGoal`.
        ///
        /// **TODO: verify** — replace with the hash from
        /// `ros2 interface show action_msgs/srv/CancelGoal`.
        pub const TYPE_HASH: &'static str =
            "RIHS01_cad08d72baa42f9d5baf1aebc0f1cabd32b26a7f12ec8af085740ed14b099b4a";

        /// Build a cancel_goal [`TopicKeyExpr`] for a given action.
        ///
        /// `action_topic` is the action's qualified name including
        /// `_action/cancel_goal` suffix (e.g., `"navigate_to_pose/_action/cancel_goal"`).
        pub const fn topic(domain_id: u32, action_topic: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, action_topic, Self::TYPE_NAME, Self::TYPE_HASH)
        }
    }

    /// Type metadata for `action_msgs/msg/GoalStatusArray`.
    pub struct GoalStatusArrayType;

    impl GoalStatusArrayType {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "action_msgs::msg::dds_::GoalStatusArray_";

        /// RIHS01 type hash for `action_msgs/msg/GoalStatusArray`.
        ///
        /// **TODO: verify** — replace with the hash from
        /// `ros2 interface show action_msgs/msg/GoalStatusArray`.
        pub const TYPE_HASH: &'static str =
            "RIHS01_032b5f04cc67d7cbb3cfa4e02f4db55be1f6fe47b291f3de087e22f3f6e4aca2";

        /// Build a status [`TopicKeyExpr`] for a given action.
        ///
        /// `action_topic` is the action's qualified name including
        /// `_action/status` suffix (e.g., `"navigate_to_pose/_action/status"`).
        pub const fn topic(domain_id: u32, action_topic: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, action_topic, Self::TYPE_NAME, Self::TYPE_HASH)
        }
    }
}
/// `sensor_msgs` package — Range and Imu message types.
pub mod sensor_msgs {
    use super::TopicKeyExpr;
    use heapless::String;
    use serde::{Deserialize, Serialize};

    /// `builtin_interfaces/msg/Time` — seconds + nanoseconds.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct Stamp {
        pub sec: i32,
        pub nanosec: u32,
    }

    impl Stamp {
        pub const ZERO: Self = Self { sec: 0, nanosec: 0 };
    }

    /// `std_msgs/msg/Header` with fixed-capacity `frame_id`.
    ///
    /// `N` is the maximum byte capacity of the frame_id string (default 16).
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Header<const N: usize = 16> {
        pub stamp: Stamp,
        pub frame_id: String<N>,
    }

    impl<const N: usize> Header<N> {
        pub fn zero() -> Self {
            Self { stamp: Stamp::ZERO, frame_id: String::new() }
        }
    }

    // ── sensor_msgs/msg/Range ────────────────────────────────────────────────

    /// `radiation_type` constant: ultrasonic.
    pub const RANGE_ULTRASOUND: u8 = 0;
    /// `radiation_type` constant: infrared.
    pub const RANGE_INFRARED: u8 = 1;

    /// Type metadata for `sensor_msgs/msg/Range`.
    pub struct RangeType;

    impl RangeType {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Range_";

        /// RIHS01 type hash for `sensor_msgs/msg/Range`. **VERIFIED** (ROS2 Jazzy).
        pub const TYPE_HASH: &'static str =
            "RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }
    }

    /// CDR-serializable `sensor_msgs/Range` message.
    ///
    /// CDR size with empty `frame_id`: 4 header + 40 body = 44 bytes (ROS2 Iron/Jazzy includes `variance`).
    /// Use [`RANGE_CDR_CAP`] (52) as the publisher buffer — fits frame_ids up to ~5 chars.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct RangeMsg<const N: usize = 16> {
        pub header: Header<N>,
        /// Use [`RANGE_ULTRASOUND`] or [`RANGE_INFRARED`].
        pub radiation_type: u8,
        /// Beam opening angle [radians]. HC-SR04 ≈ 0.2618 rad (15°).
        pub field_of_view: f32,
        pub min_range: f32,
        pub max_range: f32,
        /// Measured distance [meters]. Use `f32::INFINITY` when out of range.
        pub range: f32,
        /// Measurement variance [m^2]. 0.0 = unknown (added in ROS2 Iron).
        pub variance: f32,
    }

    /// CDR buffer capacity for [`RangeMsg`] with frame_ids up to ~5 characters.
    pub const RANGE_CDR_CAP: usize = 52;

    // ── sensor_msgs/msg/Imu ──────────────────────────────────────────────────

    /// Type metadata for `sensor_msgs/msg/Imu`.
    pub struct ImuType;

    impl ImuType {
        /// DDS type name (rmw_zenoh_cpp convention).
        pub const TYPE_NAME: &'static str = "sensor_msgs::msg::dds_::Imu_";

        /// RIHS01 type hash for `sensor_msgs/msg/Imu`. **VERIFIED** (ROS2 Jazzy).
        pub const TYPE_HASH: &'static str =
            "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";

        /// Build a [`TopicKeyExpr`] for any topic using this message type.
        pub const fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
            TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
        }
    }

    /// `geometry_msgs/Quaternion` (f64 components).
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    impl Quaternion {
        /// Identity quaternion (no rotation).
        pub const IDENTITY: Self = Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };
    }

    /// `geometry_msgs/Vector3` with f64 components (for Imu angular/linear fields).
    #[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
    pub struct Vector3d {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    impl Vector3d {
        pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    }

    /// CDR-serializable `sensor_msgs/Imu` message.
    ///
    /// CDR size with empty `frame_id`: 4 header + 312 body = 316 bytes.
    /// Use [`IMU_CDR_CAP`] (320) as the publisher buffer.
    ///
    /// For a 6-axis IMU (no magnetometer), set `orientation_covariance[0] = -1.0`
    /// to indicate that orientation is unknown per REP-145.
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct ImuMsg<const N: usize = 16> {
        pub header: Header<N>,
        pub orientation: Quaternion,
        /// Row-major 3×3 covariance. Set `[0] = -1.0` if orientation is unknown.
        pub orientation_covariance: [f64; 9],
        pub angular_velocity: Vector3d,
        pub angular_velocity_covariance: [f64; 9],
        pub linear_acceleration: Vector3d,
        pub linear_acceleration_covariance: [f64; 9],
    }

    /// CDR buffer capacity for [`ImuMsg`] with empty frame_id (316 bytes + 4 margin).
    pub const IMU_CDR_CAP: usize = 320;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ros2::message_trait::RosMessage;

    #[test]
    fn test_std_msgs_string_chatter() {
        let ke = std_msgs::String::CHATTER;
        let result = ke.to_key_expr().unwrap();
        assert_eq!(
            result.as_str(),
            "0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
        );
    }

    #[test]
    fn test_std_msgs_string_custom_topic() {
        let ke = std_msgs::String::topic(1, "my_topic");
        let result = ke.to_key_expr().unwrap();
        assert_eq!(
            result.as_str(),
            "1/my_topic/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
        );
    }

    #[test]
    fn test_geometry_msgs_twist_cmd_vel() {
        let ke = geometry_msgs::TwistType::CMD_VEL;
        let result = ke.to_key_expr().unwrap();
        assert!(result
            .as_str()
            .starts_with("0/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_"));
    }

    #[test]
    fn test_twist_cdr_roundtrip() {
        use crate::cdr;

        let twist = geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 {
                x: 1.5,
                y: -2.0,
                z: 0.0,
            },
            angular: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.75,
            },
        };

        let mut buf = [0u8; geometry_msgs::TWIST_CDR_CAP];
        let n = cdr::serialize_with_header(&mut buf, &twist).unwrap();
        assert_eq!(n, 52); // 4 header + 48 body

        let (decoded, consumed): (geometry_msgs::Twist, _) =
            cdr::deserialize_with_header(&buf[..n]).unwrap();
        assert_eq!(consumed, 52);
        assert_eq!(decoded.linear.x, 1.5);
        assert_eq!(decoded.linear.y, -2.0);
        assert_eq!(decoded.angular.z, 0.75);
    }

    #[test]
    fn test_vector3_default() {
        let v = geometry_msgs::Vector3::default();
        assert_eq!(v.x, 0.0);
        assert_eq!(v.y, 0.0);
        assert_eq!(v.z, 0.0);
    }

    #[test]
    fn test_twist_default() {
        let t = geometry_msgs::Twist::default();
        assert_eq!(t.linear, geometry_msgs::Vector3::ZERO);
        assert_eq!(t.angular, geometry_msgs::Vector3::ZERO);
    }

    #[test]
    fn test_goal_id_zero() {
        let gid = action_msgs::GoalId::ZERO;
        assert_eq!(gid.uuid, [0u8; 16]);
    }

    #[test]
    fn test_goal_id_cdr_roundtrip() {
        use crate::cdr;

        let goal_id = action_msgs::GoalId {
            uuid: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
        };
        let mut buf = [0u8; 20]; // 4 header + 16 body
        let n = cdr::serialize_with_header(&mut buf, &goal_id).unwrap();
        assert_eq!(n, 20);

        let (decoded, consumed): (action_msgs::GoalId, _) =
            cdr::deserialize_with_header(&buf[..n]).unwrap();
        assert_eq!(consumed, 20);
        assert_eq!(decoded.uuid, goal_id.uuid);
    }

    #[test]
    fn test_send_goal_response_cdr_roundtrip() {
        use crate::cdr;

        let resp = action_msgs::SendGoalResponse {
            accepted: true,
            stamp: action_msgs::Stamp {
                sec: 1234,
                nanosec: 567890,
            },
        };
        let mut buf = [0u8; action_msgs::SEND_GOAL_RESP_CDR_CAP];
        let n = cdr::serialize_with_header(&mut buf, &resp).unwrap();
        assert_eq!(n, 16); // 4 header + 1 bool + 3 pad + 4 sec + 4 nanosec

        let (decoded, consumed): (action_msgs::SendGoalResponse, _) =
            cdr::deserialize_with_header(&buf[..n]).unwrap();
        assert_eq!(consumed, 16);
        assert!(decoded.accepted);
        assert_eq!(decoded.stamp.sec, 1234);
        assert_eq!(decoded.stamp.nanosec, 567890);
    }

    #[test]
    fn test_get_result_request_cdr_roundtrip() {
        use crate::cdr;

        let req = action_msgs::GetResultRequest {
            goal_id: action_msgs::GoalId { uuid: [0xAA; 16] },
        };
        let mut buf = [0u8; action_msgs::GET_RESULT_REQ_CDR_CAP];
        let n = cdr::serialize_with_header(&mut buf, &req).unwrap();
        assert_eq!(n, 20); // 4 header + 16 body

        let (decoded, consumed): (action_msgs::GetResultRequest, _) =
            cdr::deserialize_with_header(&buf[..n]).unwrap();
        assert_eq!(consumed, 20);
        assert_eq!(decoded.goal_id.uuid, [0xAA; 16]);
    }

    #[test]
    fn test_cancel_goal_request_cdr_roundtrip() {
        use crate::cdr;

        let req = action_msgs::CancelGoalRequest {
            goal_info: action_msgs::GoalInfo {
                goal_id: action_msgs::GoalId { uuid: [0xBB; 16] },
                stamp: action_msgs::Stamp {
                    sec: 100,
                    nanosec: 200,
                },
            },
        };
        let mut buf = [0u8; action_msgs::CANCEL_GOAL_REQ_CDR_CAP];
        let n = cdr::serialize_with_header(&mut buf, &req).unwrap();
        assert_eq!(n, 28); // 4 header + 16 uuid + 4 sec + 4 nanosec

        let (decoded, consumed): (action_msgs::CancelGoalRequest, _) =
            cdr::deserialize_with_header(&buf[..n]).unwrap();
        assert_eq!(consumed, 28);
        assert_eq!(decoded.goal_info.goal_id.uuid, [0xBB; 16]);
        assert_eq!(decoded.goal_info.stamp.sec, 100);
        assert_eq!(decoded.goal_info.stamp.nanosec, 200);
    }

    #[test]
    fn test_goal_status_constants() {
        use action_msgs::goal_status::*;
        assert_eq!(STATUS_UNKNOWN, 0);
        assert_eq!(STATUS_ACCEPTED, 1);
        assert_eq!(STATUS_EXECUTING, 2);
        assert_eq!(STATUS_CANCELING, 3);
        assert_eq!(STATUS_SUCCEEDED, 4);
        assert_eq!(STATUS_CANCELED, 5);
        assert_eq!(STATUS_ABORTED, 6);
    }

    #[test]
    fn test_twist_ros_message_trait() {
        assert_eq!(
            geometry_msgs::Twist::TYPE_NAME,
            "geometry_msgs::msg::dds_::Twist_"
        );
        assert!(geometry_msgs::Twist::TYPE_HASH.starts_with("RIHS01_"));
        let ke = geometry_msgs::Twist::topic(0, "cmd_vel");
        assert_eq!(ke.topic_name, "cmd_vel");
        assert_eq!(ke.type_name, geometry_msgs::Twist::TYPE_NAME);
    }
}
