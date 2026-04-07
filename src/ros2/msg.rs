//! Pre-defined ROS2 message type constants (type names, RIHS01 hashes, and
//! helpers for building [`TopicKeyExpr`]s without hand-typing hash strings).
//!
//! Hashes marked as `VERIFIED` are taken from the official `rosidl_typesupport_c`
//! generated code or the rmw_zenoh_cpp design document.  Hashes marked as
//! `TODO: verify` are placeholder values and **must not** be used with a live
//! ROS2 system — replace them with the output of `ros2 interface show` or
//! `rosidl_get_typesupport_identifier`.
//!
//! # Example
//! ```rust,ignore
//! use zenoh_ros2_nostd::ros2::msg;
//!
//! // Instead of manually specifying type name and hash:
//! const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::topic(0, "chatter");
//!
//! // Or use the pre-built topic (domain 0, topic "/chatter"):
//! const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::CHATTER;
//! ```

use super::keyexpr::TopicKeyExpr;

/// `std_msgs` package message type constants.
pub mod std_msgs {
    use super::TopicKeyExpr;

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
}

/// `geometry_msgs` package message type constants.
pub mod geometry_msgs {
    use super::TopicKeyExpr;

    /// `geometry_msgs/msg/Twist`.
    pub struct Twist;

    impl Twist {
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let ke = geometry_msgs::Twist::CMD_VEL;
        let result = ke.to_key_expr().unwrap();
        assert!(result.as_str().starts_with("0/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_"));
    }
}
