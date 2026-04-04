---
description: "Use when working on ROS2 key expressions, QoS profiles, Node creation, topic publishers, topic subscribers, liveliness tokens, or CDR serialization for ROS2 messages."
applyTo: "src/ros2/**"
---
# ROS2 Layer Conventions

## rmw_zenoh_cpp Interoperability

This crate communicates with standard ROS2 nodes via `rmw_zenoh_cpp`.
All key expressions, liveliness tokens, serialization, and attachments
MUST follow the `rmw_zenoh_cpp` design document exactly.

Reference: https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md

## Key Expression Format (rmw_zenoh_cpp compatible)

```
<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>
```

- `<domain_id>` — Value of `ROS_DOMAIN_ID` (default `0`)
- `<fully_qualified_name>` — Topic/service name including namespace (e.g., `chatter`, `robot1/chatter`)
- `<type_name>` — **DDS convention**: `std_msgs::msg::dds_::String_` (note `dds_::` prefix and `_` suffix)
- `<type_hash>` — RIHS01 type hash from rosidl

Examples:
- `0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18`
- `0/robot1/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18`
- `0/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_...`

**IMPORTANT**: Do NOT use `std_msgs::msg::String` (without `dds_::` and trailing `_`). The DDS convention is required for rmw_zenoh_cpp compatibility.

## Liveliness Token Format

Node token:
```
@ros2_lv/<domain_id>/<session_id>/<node_id>/<node_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>
```

Publisher/Subscriber token:
```
@ros2_lv/<domain_id>/<session_id>/<node_id>/<entity_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>/<mangled_qualified_name>/<type_name>/<type_hash>/<qos>
```

Rules:
- **Mangling**: Replace `/` with `%` in names (e.g., `/chatter` → `%chatter`, `/robot1/cmd_vel` → `%robot1%cmd_vel`)
- **Empty enclave/namespace**: Use `%` as placeholder
- Entity kinds: `NN` (node), `MP` (publisher), `MS` (subscriber), `SS` (service server), `SC` (service client)
- `<session_id>` is the hex-encoded Zenoh session ID
- QoS encoding uses compact format: `<reliability><durability>,<depth>:<deadline>:<lifespan>:<liveliness>,<lease_duration>`
  - Example: `::,10:,:,:,,` (system_default, depth=10)

Examples:
```
@ros2_lv/0/aac3178e146ba6f1fc6e6a4085e77f21/0/0/NN/%/%/listener
@ros2_lv/0/aac3178e146ba6f1fc6e6a4085e77f21/0/10/MS/%/%/listener/%chatter/std_msgs::msg::dds_::String_/RIHS01_.../::,10:,:,:,,
@ros2_lv/0/8b20917502ee955ac4476e0266340d5c/0/10/MP/%/%/talker/%chatter/std_msgs::msg::dds_::String_/RIHS01_.../::,7:,:,:,,
```

## Publisher Attachment

When publishing via zenoh `put`, include an attachment with:
- 8 bytes: sequence number (`int64_t`, little-endian)
- 8 bytes: source timestamp (nanoseconds since UNIX EPOCH, `int64_t`, little-endian)
- 1 byte: GID length (always `16`)
- 16 bytes: publisher GID (the session's ZenohId, zero-padded to 16 bytes)

## CDR Serialization
- Messages are serialized with CDR LE encapsulation header: `[0x00, 0x01, 0x00, 0x00]`
- Use `cdr::serialize_with_header()` / `cdr::deserialize_with_header()` for ROS2 messages
- CDR alignment: primitives aligned to their size (u32 → 4-byte, u64 → 8-byte)
- Strings: `[u32 length including null][utf8 bytes][null terminator]`

## QoS Presets
- `Qos::DEFAULT` — reliable, volatile, keep-last(10)
- `Qos::SENSOR_DATA` — best-effort, volatile, keep-last(5)
- `Qos::PARAMETERS` — reliable, volatile, keep-last(1000)

## Message Types
- ROS2 message structs must implement `serde::Serialize` (for publishing) and `serde::Deserialize` (for subscribing)
- Use `#[derive(Serialize, Deserialize)]` on message types
- Field order in struct must match the ROS2 IDL definition exactly
- Use DDS type name convention: `pkg::msg::dds_::TypeName_`

## Session Topology
- Our MCU runs as **Zenoh client** (WhatAmI::Client) — connects to router only
- rmw_zenoh_cpp nodes run as **Zenoh peers** by default — connect to router + peer-to-peer
- The Zenoh router forwards data between clients and peers
- Router is required for MCU↔ROS2 communication

## Node & Publisher Pattern
```rust
let node = Node::new(&session, "", "my_node");
let mut pub = node.create_publisher(&topic_ke, &mut cdr_buf);
pub.publish(&my_msg).await?;
```
