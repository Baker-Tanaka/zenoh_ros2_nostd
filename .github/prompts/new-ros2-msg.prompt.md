---
description: "Create a new ROS2 message type struct with serde derive, CDR-compatible field layout, and optional test scaffolding"
agent: "agent"
---
# New ROS2 Message Type

Create a new ROS2 message type definition as a Rust struct. The struct must:

1. Be `no_std` compatible — use `heapless::String<N>` instead of `String`, `heapless::Vec<T, N>` instead of `Vec`
2. Derive `serde::Serialize` and `serde::Deserialize`
3. Derive `Debug, Clone, PartialEq`
4. Match the exact field order from the ROS2 IDL definition (CDR serialization is order-dependent)
5. Include a `///` doc comment with the original ROS2 message path (e.g., `geometry_msgs/msg/Twist`)

Also create:
- A CDR roundtrip test in `#[cfg(test)] mod tests` verifying `serialize_with_header` → `deserialize_with_header`
- A `TopicKeyExpr` constant with the correct type name and a placeholder `RIHS01_` hash

Place the struct in an appropriate location (e.g., a new file under `src/ros2/msg/` or inline if small).
