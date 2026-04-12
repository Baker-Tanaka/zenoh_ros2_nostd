//! WASI / Host demo — trait-based (class) Node pattern.
//!
//! Demonstrates the **`NodeCallbacks` trait style** of the SDK API.
//! A `ChatterNode` struct implements `NodeCallbacks` to:
//! - Publish numbered `std_msgs/String` messages on a 500ms timer
//! - Receive and print incoming messages on `/chatter`
//!
//! This mirrors the rclpy pattern of subclassing `Node` and defining
//! callback methods.
//!
//! # Host (native) mode — recommended for quick verification
//!
//! ```sh
//! cd examples/wasi_chatter_class
//! cargo run
//! ```
//!
//! # WASI mode
//!
//! ```sh
//! cargo build --target wasm32-wasip1 --features wasi
//! socat EXEC:"wasmtime run target/wasm32-wasip1/debug/wasi-chatter-class.wasm",fdin=3,fdout=3 \
//!       TCP:localhost:7447
//! ```
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────┐   TCP    ┌──────────────┐   TCP   ┌──────────────┐
//! │ ChatterNode        │────────→ │ Zenoh Router │←──────→ │ ROS2 Node    │
//! │ (host or wasmtime) │          │ :7447        │         │ rmw_zenoh_cpp│
//! └────────────────────┘          └──────────────┘         └──────────────┘
//! ```

use zenoh_ros2_nostd::prelude::*;

// ── Message type ──────────────────────────────────────────────────────────────

/// `std_msgs/msg/String` — a single UTF-8 string field.
#[derive(Serialize, Deserialize)]
struct StringMsg {
    data: heapless::String<128>,
}

// ── Topic definitions ─────────────────────────────────────────────────────────

/// Key expression for `/chatter` (domain 0, std_msgs/String).
const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::CHATTER;

/// CDR buffer capacity for StringMsg (4 header + 4 len + 128 chars + 1 nul).
const STRING_CDR_CAP: usize = 137;

// ── Static publisher & subscription ───────────────────────────────────────────

static CHATTER_PUB: Publisher<StringMsg, STRING_CDR_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, STRING_CDR_CAP, 4> = Subscription::new();

// ── ChatterNode — trait-based callback struct ─────────────────────────────────

/// A ROS2-like node class that publishes and subscribes on `/chatter`.
///
/// Implements [`NodeCallbacks`] to receive `on_message` and `on_timer`.
struct ChatterNode {
    /// Publisher handle (set after registration).
    pub_handle: Option<PublisherHandle<StringMsg, STRING_CDR_CAP, 4>>,
    /// Message counter.
    count: u32,
    /// Maximum number of messages to publish (0 = unlimited).
    max_messages: u32,
}

impl ChatterNode {
    const fn new(max_messages: u32) -> Self {
        Self {
            pub_handle: None,
            count: 0,
            max_messages,
        }
    }
}

impl NodeCallbacks for ChatterNode {
    fn on_message(&mut self, topic: &str, payload: &[u8]) {
        // Deserialize the incoming CDR message
        match zenoh_ros2_nostd::cdr::deserialize_with_header::<StringMsg>(payload) {
            Ok((msg, _)) => {
                println!("[chatter-class] rx on {}: \"{}\"", topic, msg.data.as_str());
            }
            Err(e) => {
                println!("[chatter-class] rx deserialize error: {:?}", e);
            }
        }
    }

    fn on_timer(&mut self) {
        if self.max_messages > 0 && self.count >= self.max_messages {
            return;
        }

        self.count += 1;

        // Build message
        let mut data = heapless::String::<128>::new();
        let _ = core::fmt::Write::write_fmt(
            &mut data,
            format_args!("Hello from ChatterNode! [{}]", self.count),
        );

        if let Some(ref pub_handle) = self.pub_handle {
            match pub_handle.try_publish(&StringMsg { data }) {
                Ok(()) => {
                    println!("[chatter-class] tx #{}", self.count);
                }
                Err(e) => {
                    println!("[chatter-class] publish error: {:?}", e);
                }
            }
        }
    }
}

// ── Entry points ──────────────────────────────────────────────────────────────

#[cfg(target_arch = "wasm32")]
fn main() {
    use zenoh_ros2_nostd::wasi::WasiTcpStream;

    const PREOPEN_TCP_FD: u32 = 3;
    println!("[chatter-class] WASI mode, fd={}", PREOPEN_TCP_FD);
    let stream = WasiTcpStream::from_raw_fd(PREOPEN_TCP_FD);
    block_on(run(stream));
}

#[cfg(not(target_arch = "wasm32"))]
#[tokio::main(flavor = "current_thread")]
async fn main() {
    use embedded_io_adapters::tokio_1::FromTokio;
    use tokio::net::TcpStream;

    let addr = std::env::var("ZENOH_ROUTER_ADDR").unwrap_or_else(|_| "127.0.0.1:7447".to_string());
    println!("[chatter-class] connecting to {}", addr);

    let tcp = TcpStream::connect(&addr).await.expect("TCP connect failed");
    let stream = FromTokio::new(tcp);
    run(stream).await;
}

async fn run<T: embedded_io_async::Read + embedded_io_async::Write>(transport: T) {
    // Build node via the SDK builder
    let (mut node, mut callbacks) = match NodeBuilder::new("chatter_class_node")
        .domain_id(0)
        .build_with_callbacks(transport, ChatterNode::new(20))
        .await
    {
        Ok(pair) => {
            println!("[chatter-class] connected (lease={}ms)", pair.0.lease_ms());
            pair
        }
        Err(e) => {
            println!("[chatter-class] build failed: {:?}", e);
            return;
        }
    };

    // Register publisher
    match node.register_static_publisher(&CHATTER_PUB).await {
        Ok(handle) => {
            callbacks.pub_handle = Some(handle);
            println!("[chatter-class] publisher registered for /chatter");
        }
        Err(e) => {
            println!("[chatter-class] publisher registration failed: {:?}", e);
            return;
        }
    }

    // Register subscription
    match node
        .subscribe_with_dispatch(CHATTER_TOPIC, &CHATTER_SUB)
        .await
    {
        Ok(_sub_handle) => {
            println!("[chatter-class] subscribed to /chatter");
        }
        Err(e) => {
            println!("[chatter-class] subscribe failed: {:?}", e);
            return;
        }
    }

    // Register timer (500ms period — publishes in on_timer callback)
    let _timer = node.register_timer(Duration::from_millis(500));
    println!("[chatter-class] timer registered (500ms)");

    // Spin — drives publish drain, receive dispatch, keepalive, and timer callbacks
    println!("[chatter-class] spinning...");
    node.spin_with_callbacks(&mut callbacks).await;

    println!("[chatter-class] disconnected");
}

// ── WASI single-threaded executor ─────────────────────────────────────────────

#[cfg(target_arch = "wasm32")]
fn block_on<F: core::future::Future>(f: F) -> F::Output {
    use core::pin::Pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    const VTABLE: RawWakerVTable =
        RawWakerVTable::new(|p| RawWaker::new(p, &VTABLE), |_| {}, |_| {}, |_| {});

    let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
    let mut cx = Context::from_waker(&waker);
    let mut f = f;
    let mut f = unsafe { Pin::new_unchecked(&mut f) };

    loop {
        match f.as_mut().poll(&mut cx) {
            Poll::Ready(val) => return val,
            Poll::Pending => {}
        }
    }
}
