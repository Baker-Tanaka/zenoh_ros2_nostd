//! Liveliness token generation for rmw_zenoh_cpp compatibility.
//!
//! Liveliness tokens allow ROS2 graph discovery. Each entity
//! (publisher/subscriber) announces its presence via a liveliness key.
//!
//! Format (rmw_zenoh_cpp):
//! ```text
//! @ros2_lv/<domain_id>/<zid>/<nid>/<entity_id>/<entity_type>/<enclave>/<namespace>/<node_name>/<topic>/<type>/<hash>/<qos>
//! ```
//!
//! Name mangling: all `/` in enclave, namespace, node_name, topic, type, and
//! type_hash are replaced with `%` (the `SLASH_REPLACEMENT` used by
//! rmw_zenoh_cpp). An empty or root-only field becomes `%`.

use heapless::String;

use super::qos::Qos;
use crate::transport::protocol::ZenohId;

/// Maximum liveliness key expression length.
pub const MAX_LIVELINESS_LEN: usize = 512;

/// Entity type for liveliness tokens.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityType {
    /// Message publisher.
    Publisher,
    /// Message subscriber.
    Subscriber,
    /// Service server.
    ServiceServer,
    /// Service client.
    ServiceClient,
}

impl EntityType {
    /// String code used in the liveliness token.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Publisher => "MP",
            Self::Subscriber => "MS",
            Self::ServiceServer => "SS",
            Self::ServiceClient => "SC",
        }
    }
}

/// Build a liveliness token key expression.
///
/// This token is used by rmw_zenoh_cpp for graph discovery.
pub fn build_liveliness_token(
    domain_id: u32,
    zid: &ZenohId,
    nid: u32,
    entity_id: u32,
    entity_type: EntityType,
    enclave: &str,
    namespace: &str,
    node_name: &str,
    topic_name: &str,
    type_name: &str,
    type_hash: &str,
    qos: &Qos,
) -> Result<String<MAX_LIVELINESS_LEN>, ()> {
    let mut s = String::new();

    // Prefix
    s.push_str("@ros2_lv/").map_err(|_| ())?;

    // Domain ID
    push_u32(&mut s, domain_id)?;
    s.push('/').map_err(|_| ())?;

    // Zenoh ID (hex)
    for b in zid.as_bytes() {
        push_hex_byte(&mut s, *b)?;
    }
    s.push('/').map_err(|_| ())?;

    // Node ID
    push_u32(&mut s, nid)?;
    s.push('/').map_err(|_| ())?;

    // Entity ID
    push_u32(&mut s, entity_id)?;
    s.push('/').map_err(|_| ())?;

    // Entity type
    s.push_str(entity_type.as_str()).map_err(|_| ())?;
    s.push('/').map_err(|_| ())?;

    // Enclave (mangled; empty/unset → "%")
    push_mangled_absolute(&mut s, enclave)?;
    s.push('/').map_err(|_| ())?;

    // Namespace (mangled; "/" or "" → "%")
    push_mangled_absolute(&mut s, namespace)?;
    s.push('/').map_err(|_| ())?;

    // Node name (mangled)
    push_mangled(&mut s, node_name)?;
    s.push('/').map_err(|_| ())?;

    // Topic name (mangled absolute — prepends '%' for leading '/')
    push_mangled_absolute(&mut s, topic_name)?;
    s.push('/').map_err(|_| ())?;

    // Type name (mangled)
    push_mangled(&mut s, type_name)?;
    s.push('/').map_err(|_| ())?;

    // Type hash (mangled)
    push_mangled(&mut s, type_hash)?;
    s.push('/').map_err(|_| ())?;

    // QoS (rmw_zenoh_cpp-compatible keyexpr encoding)
    let qos_keyexpr = qos.to_rmw_qos_keyexpr()?;
    s.push_str(qos_keyexpr.as_str()).map_err(|_| ())?;

    Ok(s)
}

// --- Helpers ---

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

fn push_hex_byte<const N: usize>(s: &mut String<N>, byte: u8) -> Result<(), ()> {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    s.push(HEX[(byte >> 4) as usize] as char).map_err(|_| ())?;
    s.push(HEX[(byte & 0x0F) as usize] as char)
        .map_err(|_| ())?;
    Ok(())
}

/// Mangle a name: replace every `/` with `%` (rmw_zenoh_cpp convention).
/// An empty string or a bare `/` becomes `%`.
fn push_mangled<const N: usize>(s: &mut String<N>, name: &str) -> Result<(), ()> {
    if name.is_empty() || name == "/" {
        s.push('%').map_err(|_| ())?;
        return Ok(());
    }
    for c in name.chars() {
        if c == '/' {
            s.push('%').map_err(|_| ())?;
        } else {
            s.push(c).map_err(|_| ())?;
        }
    }
    Ok(())
}

/// Mangle an absolute ROS2 name (enclave, namespace, topic).
///
/// These names canonically start with `/` in ROS2 (e.g. `/baker_link/status`),
/// but our `TopicKeyExpr` stores them without the leading slash. This helper
/// ensures the mangled output always starts with `%` (the mangled `/`).
///
/// - `""` or `"/"` → `%`
/// - `"/foo/bar"` → `%foo%bar` (already absolute)
/// - `"foo/bar"` → `%foo%bar` (prepends `%` for missing leading `/`)
fn push_mangled_absolute<const N: usize>(s: &mut String<N>, name: &str) -> Result<(), ()> {
    if name.is_empty() || name == "/" {
        s.push('%').map_err(|_| ())?;
        return Ok(());
    }
    // If it doesn't start with '/', prepend '%' (the mangled leading '/').
    if !name.starts_with('/') {
        s.push('%').map_err(|_| ())?;
    }
    for c in name.chars() {
        if c == '/' {
            s.push('%').map_err(|_| ())?;
        } else {
            s.push(c).map_err(|_| ())?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_liveliness_token() {
        let zid = ZenohId::from_bytes(&[0x01, 0x02, 0x03, 0x04]);
        let token = build_liveliness_token(
            0,
            &zid,
            0,
            1,
            EntityType::Publisher,
            "",
            "",
            "mcu_node",
            "cmd_vel",
            "geometry_msgs::msg::Twist",
            "RIHS01_abc",
            &Qos::DEFAULT,
        )
        .unwrap();

        assert!(token.starts_with("@ros2_lv/0/"));
        // enclave="%", namespace="%", node="mcu_node", topic="%cmd_vel"
        assert!(token.contains("/MP/%/%/mcu_node/%cmd_vel/"));
        assert!(token.contains("/geometry_msgs::msg::Twist/"));
        // Qos::DEFAULT = reliable(1), volatile(2), keep_last(10)
        // rmw format: all defaults except depth=10
        assert!(token.ends_with("/::,10:,:,:,,"));
    }

    #[test]
    fn test_mangling_with_slashes() {
        let zid = ZenohId::from_bytes(&[0xAA]);
        let token = build_liveliness_token(
            0,
            &zid,
            0,
            1,
            EntityType::Publisher,
            "",
            "/my_ns",
            "node1",
            "baker_link/status",
            "std_msgs::msg::dds_::String_",
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            &Qos::DEFAULT,
        )
        .unwrap();

        // namespace="/my_ns" → "%my_ns", topic="baker_link/status" → "%baker_link%status"
        assert!(token.contains("/MP/%/%my_ns/node1/%baker_link%status/"));
    }
}
