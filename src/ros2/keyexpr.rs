//! ROS2-compatible key expression generation for rmw_zenoh_cpp.
//!
//! Key expression format (for topics):
//!   `<domain_id>/<topic_name>/<type_name>/<type_hash>`
//!
//! Examples:
//!   `0/chatter/std_msgs::msg::String/RIHS01_...`
//!   `0/cmd_vel/geometry_msgs::msg::Twist/RIHS01_...`

use heapless::String;

/// Maximum length for a generated key expression.
pub const MAX_KEY_EXPR_LEN: usize = 256;

/// A ROS2 topic key expression builder.
///
/// Generates zenoh key expressions compatible with rmw_zenoh_cpp.
#[derive(Debug, Clone, Copy)]
pub struct TopicKeyExpr {
    /// ROS2 domain ID (default: 0).
    pub domain_id: u32,
    /// Topic name without leading slash (e.g., "cmd_vel").
    pub topic_name: &'static str,
    /// Fully qualified type name (e.g., "geometry_msgs::msg::Twist").
    pub type_name: &'static str,
    /// Type hash in RIHS format (e.g., "RIHS01_...").
    pub type_hash: &'static str,
}

impl TopicKeyExpr {
    /// Create a new topic key expression.
    pub const fn new(
        domain_id: u32,
        topic_name: &'static str,
        type_name: &'static str,
        type_hash: &'static str,
    ) -> Self {
        Self {
            domain_id,
            topic_name,
            type_name,
            type_hash,
        }
    }

    /// Build the zenoh key expression string.
    ///
    /// Format: `<domain_id>/<topic_name>/<type_name>/<type_hash>`
    pub fn to_key_expr(&self) -> Result<String<MAX_KEY_EXPR_LEN>, ()> {
        let mut s = String::new();

        // Domain ID
        write_u32(&mut s, self.domain_id)?;
        push_char(&mut s, '/')?;

        // Topic name (strip leading slash if present)
        let topic = self.topic_name.strip_prefix('/').unwrap_or(self.topic_name);
        push_str(&mut s, topic)?;
        push_char(&mut s, '/')?;

        // Type name
        push_str(&mut s, self.type_name)?;
        push_char(&mut s, '/')?;

        // Type hash
        push_str(&mut s, self.type_hash)?;

        Ok(s)
    }
}

/// Build a topic key expression from dynamic (non-static) string slices.
///
/// Same format as `TopicKeyExpr::to_key_expr` but accepts `&str`.
pub fn build_topic_key_expr(
    domain_id: u32,
    topic_name: &str,
    type_name: &str,
    type_hash: &str,
) -> Result<String<MAX_KEY_EXPR_LEN>, ()> {
    let mut s = String::new();

    write_u32(&mut s, domain_id)?;
    push_char(&mut s, '/')?;

    let topic = topic_name.strip_prefix('/').unwrap_or(topic_name);
    push_str(&mut s, topic)?;
    push_char(&mut s, '/')?;

    push_str(&mut s, type_name)?;
    push_char(&mut s, '/')?;

    push_str(&mut s, type_hash)?;

    Ok(s)
}

// ---- Helpers (no core::fmt::Write needed, avoiding alloc) ----

fn push_str<const N: usize>(s: &mut String<N>, val: &str) -> Result<(), ()> {
    s.push_str(val).map_err(|_| ())
}

fn push_char<const N: usize>(s: &mut String<N>, c: char) -> Result<(), ()> {
    s.push(c).map_err(|_| ())
}

fn write_u32<const N: usize>(s: &mut String<N>, val: u32) -> Result<(), ()> {
    if val == 0 {
        return push_char(s, '0');
    }
    let mut buf = [0u8; 10]; // max digits for u32
    let mut pos = buf.len();
    let mut v = val;
    while v > 0 {
        pos -= 1;
        buf[pos] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    let digits = core::str::from_utf8(&buf[pos..]).map_err(|_| ())?;
    push_str(s, digits)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_key_expr() {
        let ke = TopicKeyExpr::new(
            0,
            "chatter",
            "std_msgs::msg::dds_::String_",
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
        );
        let result = ke.to_key_expr().unwrap();
        assert_eq!(
            result.as_str(),
            "0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
        );
    }

    #[test]
    fn test_key_expr_strips_leading_slash() {
        let ke = TopicKeyExpr::new(
            0,
            "/cmd_vel",
            "geometry_msgs::msg::dds_::Twist_",
            "RIHS01_xyz",
        );
        let result = ke.to_key_expr().unwrap();
        assert_eq!(
            result.as_str(),
            "0/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_xyz"
        );
    }

    #[test]
    fn test_key_expr_nonzero_domain() {
        let ke = TopicKeyExpr::new(
            42,
            "scan",
            "sensor_msgs::msg::dds_::LaserScan_",
            "RIHS01_hash",
        );
        let result = ke.to_key_expr().unwrap();
        assert_eq!(
            result.as_str(),
            "42/scan/sensor_msgs::msg::dds_::LaserScan_/RIHS01_hash"
        );
    }

    #[test]
    fn test_build_dynamic() {
        let result = build_topic_key_expr(
            0,
            "joint_states",
            "sensor_msgs::msg::dds_::JointState_",
            "RIHS01_abc",
        )
        .unwrap();
        assert_eq!(
            result.as_str(),
            "0/joint_states/sensor_msgs::msg::dds_::JointState_/RIHS01_abc"
        );
    }
}
