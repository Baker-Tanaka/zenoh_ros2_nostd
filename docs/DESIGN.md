# Design Document — zenoh-ros2-nostd v0.5

> This document describes the SDK redesign targeting rclpy-like ergonomics,
> WASI target support, and Gazebo integration.

## 1. Goals

| # | Goal | Rationale |
|---|------|-----------|
| G1 | `#![no_std]` remains the foundation | MCU deployability is the project's core value |
| G2 | WASI (wasm32-wasip1/wasip2) target | Enable Gazebo sim integration without hardware |
| G3 | rclpy-like high-level SDK | Lower barrier to entry for ROS2 developers |
| G4 | Embassy-native async internally | Leverage mature embedded async ecosystem |
| G5 | Incremental ROS2 feature coverage | pub/sub → service → action, phased delivery |

## 2. Target Matrix

| Target | Transport | Executor | Feature flag |
|--------|-----------|----------|-------------|
| `thumbv6m-none-eabi` (Cortex-M0) | embedded-io-async (SPI/UART/Ethernet) | embassy-executor | (default) |
| `riscv32imc-unknown-none-elf` (ESP32-C3) | embassy-net WiFi/TCP | embassy-executor | (default) |
| `wasm32-wasip1` / `wasip2` | WASI socket → TCP | WASI reactor (host runtime) | `wasi` |
| `x86_64` (host test) | tokio TcpStream + embedded-io-adapters | tokio | `std` (dev) |

### WASI specifics

- Network: WASI `sock_connect` → TCP to zenoh router (same as MCU path)
- Executor: the WASI host (wasmtime/wasmer) provides the event loop;
  the crate only needs `embedded-io-async` Read/Write, not an executor
- Time: `embassy-time` with a WASI-compatible time driver, or a thin
  `Instant::now()` shim using `clock_time_get`
- Feature: `#[cfg(feature = "wasi")]` gates WASI-specific socket adapters
  and any `std`-like facilities

## 3. Architecture (Revised)

```
┌─────────────────────────────────────────────────┐
│  Public SDK API  (rclpy-like)                   │  ← User-facing
│    Node, create_publisher, create_subscription  │
│    Callback style  /  Trait (NodeCallbacks) style│
├─────────────────────────────────────────────────┤
│  ros2/  (internal)                              │  ← Key expressions, QoS, liveliness,
│    keyexpr, msg, qos, liveliness                │     CDR encode/decode
├─────────────────────────────────────────────────┤
│  session/  (internal)                           │  ← Session state machine, reconnect
├─────────────────────────────────────────────────┤
│  transport/  (internal)                         │  ← Zenoh v9 wire protocol
├─────────────────────────────────────────────────┤
│  buf/ + cdr/  (internal)                        │  ← Buffer pool, CDR serde
└─────────────────────────────────────────────────┘
         │
    embedded-io-async (Read + Write)
         │
   ┌─────┼──────────────┐
   │ TCP Socket (MCU)   │  WASI socket (wasm)
   └────────────────────┘
```

### Layer visibility

| Layer | Visibility | Notes |
|-------|-----------|-------|
| SDK API (`sdk/`) | `pub` | The only public interface |
| `ros2/` | `pub(crate)` | Internal, re-exported selectively by SDK |
| `session/` | `pub(crate)` | Internal |
| `transport/` | `pub(crate)` | Internal |
| `buf/`, `cdr/` | `pub(crate)` | Internal |

> **Migration**: The current `ros2::Node`, `Publisher`, `Subscription` become
> internal implementation details. The new `sdk` module wraps them with
> rclpy-like ergonomics.

## 4. SDK API Design

### 4.1 Callback Function Style (rclpy `create_subscription` pattern)

```rust
use zenoh_ros2_nostd::prelude::*;

// Message definitions (serde-based, same as current)
#[derive(Serialize, Deserialize)]
struct Twist { linear: Vector3, angular: Vector3 }

async fn run(transport: impl ReadWrite) {
    let mut node = Node::builder("teleop_node")
        .namespace("/robot1")
        .domain_id(0)
        .build(transport)
        .await
        .unwrap();

    // Publisher — returns a handle, send from any task
    let cmd_pub = node.create_publisher::<Twist>("cmd_vel", QoS::default());

    // Subscription with async callback
    node.create_subscription::<StringMsg>("chatter", QoS::default(), |msg: StringMsg| {
        // Process message
    });

    // Timer callback (embassy-time internally)
    let twist = Twist { /* ... */ };
    node.create_timer(Duration::from_millis(100), move || {
        cmd_pub.publish(&twist);
    });

    // Blocks forever, dispatching callbacks + keepalive
    node.spin().await;
}
```

### 4.2 Trait Implementation Style (rclpy class inheritance pattern)

```rust
use zenoh_ros2_nostd::prelude::*;

struct TeleopNode {
    cmd_pub: PublisherHandle<Twist>,
}

impl NodeCallbacks for TeleopNode {
    // Called once after the node connects
    fn on_init(&mut self, node: &mut NodeContext) {
        self.cmd_pub = node.create_publisher("cmd_vel", QoS::default());
        node.create_subscription::<StringMsg>("chatter", QoS::default());
        node.create_timer(Duration::from_millis(100));
    }

    // Dispatched by topic name (or by associated type)
    fn on_message(&mut self, topic: &str, payload: &[u8]) {
        // Deserialize and process
    }

    fn on_timer(&mut self) {
        let twist = Twist { /* ... */ };
        self.cmd_pub.publish(&twist);
    }
}

async fn run(transport: impl ReadWrite) {
    let my_node = TeleopNode { /* ... */ };
    let node = Node::builder("teleop_node")
        .build_with_callbacks(transport, my_node)
        .await
        .unwrap();
    node.spin().await;
}
```

### 4.3 Key design principles

| Principle | Detail |
|-----------|--------|
| No `static` required | Current API requires `static Publisher` — new API uses owned handles |
| No const generics in public API | Buffer sizes are inferred or configured via builder |
| Transport-agnostic | `impl Read + Write` — works on MCU, WASI, and host |
| Embassy internal only | Users don't import embassy directly; SDK re-exports `Duration` etc. |
| `no_std` by default | `alloc` feature enables `String`/`Vec` convenience; core API stays heapless |

### 4.4 PublisherHandle / SubscriptionHandle

```rust
/// Lightweight handle returned by `node.create_publisher()`.
/// Can be cloned and sent to other tasks (uses internal Channel).
pub struct PublisherHandle<M> { /* ... */ }

impl<M: Serialize> PublisherHandle<M> {
    pub async fn publish(&self, msg: &M) -> Result<(), Error>;
    pub fn try_publish(&self, msg: &M) -> Result<(), Error>;
}
```

## 5. Feature Flags

| Feature | Default | Description |
|---------|---------|-------------|
| `defmt` | ✅ | defmt logging for embedded |
| `log` | | std log crate |
| `alloc` | | Enable alloc-dependent APIs |
| `wasi` | | WASI socket adapter + time shim |
| `embassy` | ✅ | Embassy sync/time (disable for bare-metal polling) |

## 6. WASI Integration

### 6.1 Socket adapter

```rust
// src/wasi/socket.rs
#[cfg(feature = "wasi")]
pub struct WasiTcpStream { /* wasi fd */ }

#[cfg(feature = "wasi")]
impl embedded_io_async::Read for WasiTcpStream { /* ... */ }

#[cfg(feature = "wasi")]
impl embedded_io_async::Write for WasiTcpStream { /* ... */ }
```

### 6.2 Time driver

For WASI, `embassy-time` needs a time driver. Options:
1. Use `wasi::clocks::monotonic_clock` as the time source
2. Provide a minimal `embassy-time-driver` impl for WASI
3. Feature-gate timer-dependent code (keepalive interval) behind `embassy`

### 6.3 Gazebo example topology

```
┌──────────────────┐     ┌──────────────┐     ┌──────────────────┐
│ WASM module      │─TCP─│ zenoh router │─TCP─│ ROS2 + Gazebo    │
│ (wasmtime)       │     │ :7447        │     │ turtlebot3 sim   │
│ this crate       │     │              │     │ rmw_zenoh_cpp    │
│ wasm32-wasip1    │     │              │     │                  │
└──────────────────┘     └──────────────┘     └──────────────────┘
```

## 7. Module Layout (post-refactor)

```
src/
├── lib.rs              # #![no_std], feature gates, re-exports
├── prelude.rs          # pub use of common types for `use crate::prelude::*`
├── sdk/                # ← NEW: public high-level API
│   ├── mod.rs
│   ├── node.rs         # Node builder, spin loop, callback dispatch
│   ├── publisher.rs    # PublisherHandle<M>
│   ├── subscription.rs # SubscriptionHandle<M>, callback registration
│   ├── timer.rs        # Timer handle, periodic callback
│   └── traits.rs       # NodeCallbacks trait
├── ros2/               # (pub(crate)) ROS2 protocol details
│   ├── keyexpr.rs
│   ├── liveliness.rs
│   ├── msg.rs
│   └── qos.rs
├── session/            # (pub(crate)) Session state machine
├── transport/          # (pub(crate)) Zenoh v9 wire protocol
├── buf/                # (pub(crate)) Static buffer pool
├── cdr/                # (pub(crate)) CDR serialization
├── wasi/               # ← NEW: WASI-specific adapters
│   ├── mod.rs
│   ├── socket.rs       # WasiTcpStream (embedded-io-async impl)
│   └── time.rs         # Monotonic clock shim
├── error.rs
└── logging.rs
```

## 8. Migration Path

| Current API | New API | Change |
|------------|---------|--------|
| `NodeBuilder::new(zid).open(tcp)` | `Node::builder("name").build(tcp)` | ZID auto-generated or optional |
| `static PUB: Publisher<M, N, Q>` | `node.create_publisher::<M>(topic)` | No more static, no const generics in API |
| `static SUB: Subscription<M, N, Q>` | `node.create_subscription(topic, cb)` | Callback-driven |
| `node.spin(&mut buf)` | `node.spin().await` | Internal buffer management |
| `TopicKeyExpr::new(...)` | `msg::std_msgs::String::topic(...)` | Keep, add convenience `"topic_name"` resolution |

## 9. Open Questions

1. **ZenohId generation**: Auto-generate from WASI random, or require user to provide?
   - Recommendation: auto-generate with `wasi::random::get_random_bytes`, fallback to user-provided
2. **Max publishers/subscribers**: Keep `MAX_PUBS=4, MAX_SUBS=4` or make configurable?
   - Recommendation: builder method `.max_publishers(8)` with const generic default
3. **Callback vs Channel**: Should subscriptions always use callbacks, or offer both?
   - Recommendation: both — `create_subscription(topic, callback)` and `create_subscription_channel(topic)` returning a `SubscriptionHandle`
4. **Service/Action wire format**: Need to analyze `rmw_zenoh_cpp` source for request/reply patterns
   - Defer to Phase 3/4
