//! Trait for ROS2 message types with associated metadata.
//!
//! Implementors provide the DDS type name, RIHS01 type hash, and a helper
//! to construct [`TopicKeyExpr`]s without repeating those constants.
//!
//! # Derive macro
//!
//! Use `#[derive(RosMessage)]` from `zenoh-ros2-nostd-derive` to auto-generate
//! this trait implementation:
//!
//! ```rust,ignore
//! use zenoh_ros2_nostd_derive::RosMessage;
//! use serde::{Serialize, Deserialize};
//!
//! #[derive(Serialize, Deserialize, RosMessage)]
//! #[ros_message(
//!     type_name = "std_msgs::msg::dds_::String_",
//!     type_hash = "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
//! )]
//! struct StringMsg {
//!     data: heapless::String<128>,
//! }
//! ```

use super::keyexpr::TopicKeyExpr;

/// Trait for ROS2 message types carrying their DDS type name and RIHS01 hash.
///
/// This trait provides the metadata needed to build Zenoh key expressions
/// compatible with `rmw_zenoh_cpp`.
///
/// # Associated constants
///
/// - [`TYPE_NAME`](Self::TYPE_NAME): DDS type name with `dds_::` prefix and `_` suffix
///   (e.g., `"std_msgs::msg::dds_::String_"`)
/// - [`TYPE_HASH`](Self::TYPE_HASH): RIHS01 type hash from `rosidl`
///   (e.g., `"RIHS01_df668c..."`)
pub trait RosMessage {
    /// DDS type name following the `rmw_zenoh_cpp` convention.
    ///
    /// Format: `<package>::msg::dds_::<TypeName>_`
    ///
    /// Examples:
    /// - `"std_msgs::msg::dds_::String_"`
    /// - `"geometry_msgs::msg::dds_::Twist_"`
    const TYPE_NAME: &'static str;

    /// RIHS01 type hash from `rosidl`.
    ///
    /// Obtain from `ros2 interface show <type>` or the `rosidl_typesupport_c`
    /// generated code.
    const TYPE_HASH: &'static str;

    /// Build a [`TopicKeyExpr`] for a given domain ID and topic name.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// const CHATTER: TopicKeyExpr = StringMsg::topic(0, "chatter");
    /// ```
    fn topic(domain_id: u32, topic_name: &'static str) -> TopicKeyExpr {
        TopicKeyExpr::new(domain_id, topic_name, Self::TYPE_NAME, Self::TYPE_HASH)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestMsg;

    impl RosMessage for TestMsg {
        const TYPE_NAME: &'static str = "test_pkg::msg::dds_::TestMsg_";
        const TYPE_HASH: &'static str =
            "RIHS01_0000000000000000000000000000000000000000000000000000000000000000";
    }

    #[test]
    fn test_ros_message_trait_constants() {
        assert_eq!(TestMsg::TYPE_NAME, "test_pkg::msg::dds_::TestMsg_");
        assert!(TestMsg::TYPE_HASH.starts_with("RIHS01_"));
    }

    #[test]
    fn test_ros_message_topic_builder() {
        let ke = TestMsg::topic(0, "test_topic");
        assert_eq!(ke.domain_id, 0);
        assert_eq!(ke.topic_name, "test_topic");
        assert_eq!(ke.type_name, TestMsg::TYPE_NAME);
        assert_eq!(ke.type_hash, TestMsg::TYPE_HASH);
    }

    #[test]
    fn test_ros_message_topic_with_domain() {
        let ke = TestMsg::topic(42, "my/topic");
        assert_eq!(ke.domain_id, 42);
        assert_eq!(ke.topic_name, "my/topic");
    }
}

#[cfg(all(test, feature = "derive"))]
mod derive_tests {
    // Allow the proc-macro to resolve `zenoh_ros2_nostd::__private::RosMessage`
    // when expanding inside the crate itself.
    extern crate self as zenoh_ros2_nostd;

    use super::RosMessage;
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, crate::RosMessage)]
    #[ros_message(
        type_name = "test_pkg::msg::dds_::DerivedMsg_",
        type_hash = "RIHS01_aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
    )]
    struct DerivedMsg {
        value: u32,
    }

    #[test]
    fn test_derive_ros_message_type_name() {
        assert_eq!(DerivedMsg::TYPE_NAME, "test_pkg::msg::dds_::DerivedMsg_");
    }

    #[test]
    fn test_derive_ros_message_type_hash() {
        assert_eq!(
            DerivedMsg::TYPE_HASH,
            "RIHS01_aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
        );
    }

    #[test]
    fn test_derive_ros_message_topic() {
        let ke = DerivedMsg::topic(0, "test_derived");
        assert_eq!(ke.topic_name, "test_derived");
        assert_eq!(ke.type_name, "test_pkg::msg::dds_::DerivedMsg_");
    }
}
