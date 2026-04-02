//! Liveliness token generation for rmw_zenoh_cpp compatibility.
//!
//! Liveliness tokens allow ROS2 graph discovery. Each entity
//! (publisher/subscriber) announces its presence via a liveliness key.
//!
//! Format:
//! ```text
//! @ros2_lv/<domain_id>/<zid>/<nid>/<entity_id>/<entity_type>/<namespace>/<node_name>/<topic>/<type>/<hash>/<qos>
//! ```

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
}

impl EntityType {
    /// String code used in the liveliness token.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Publisher => "MP",
            Self::Subscriber => "MS",
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

    // Namespace (strip leading slash)
    let ns = namespace.strip_prefix('/').unwrap_or(namespace);
    if !ns.is_empty() {
        s.push_str(ns).map_err(|_| ())?;
    }
    s.push('/').map_err(|_| ())?;

    // Node name
    s.push_str(node_name).map_err(|_| ())?;
    s.push('/').map_err(|_| ())?;

    // Topic name (strip leading slash)
    let topic = topic_name.strip_prefix('/').unwrap_or(topic_name);
    s.push_str(topic).map_err(|_| ())?;
    s.push('/').map_err(|_| ())?;

    // Type name
    s.push_str(type_name).map_err(|_| ())?;
    s.push('/').map_err(|_| ())?;

    // Type hash
    s.push_str(type_hash).map_err(|_| ())?;
    s.push('/').map_err(|_| ())?;

    // QoS
    s.push_str(qos.to_liveliness_str()).map_err(|_| ())?;

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
    s.push(HEX[(byte & 0x0F) as usize] as char).map_err(|_| ())?;
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
            "mcu_node",
            "cmd_vel",
            "geometry_msgs::msg::Twist",
            "RIHS01_abc",
            &Qos::DEFAULT,
        )
        .unwrap();

        assert!(token.starts_with("@ros2_lv/0/"));
        assert!(token.contains("/MP/"));
        assert!(token.contains("/mcu_node/"));
        assert!(token.contains("/cmd_vel/"));
        assert!(token.contains("/geometry_msgs::msg::Twist/"));
        assert!(token.ends_with("/RV"));
    }
}
