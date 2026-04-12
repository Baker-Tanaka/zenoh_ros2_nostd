//! WASI / Host demo — publish `std_msgs/String` to `/chatter` via Zenoh.
//!
//! Publishes numbered string messages that are easy to verify visually
//! with `ros2 topic echo /chatter` or the included `verify_sub.py` script.
//!
//! # Host (native) mode — recommended for quick verification
//!
//! ```sh
//! # From the dev container (ZENOH_ROUTER_ADDR is set automatically):
//! cd examples/wasi_turtlebot3
//! cargo run
//! ```
//!
//! # WASI mode
//!
//! ```sh
//! cargo build --target wasm32-wasip1 --features wasi
//! socat EXEC:"wasmtime run target/wasm32-wasip1/debug/wasi-turtlebot3.wasm",fdin=3,fdout=3 \
//!       TCP:localhost:7447
//! ```
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────┐   TCP    ┌──────────────┐   TCP   ┌──────────────┐
//! │ This example       │────────→ │ Zenoh Router │←──────→ │ ROS2 Node    │
//! │ (host or wasmtime) │          │ :7447        │         │ rmw_zenoh_cpp│
//! └────────────────────┘          └──────────────┘         └──────────────┘
//! ```

use zenoh_ros2_nostd::ros2::msg::std_msgs;
use zenoh_ros2_nostd::transport::{codec, frame, handshake, protocol::ZenohId};

/// Number of messages to publish before exiting.
const NUM_MESSAGES: u32 = 20;

#[cfg(target_arch = "wasm32")]
fn main() {
    use zenoh_ros2_nostd::wasi::WasiTcpStream;

    const PREOPEN_TCP_FD: u32 = 3;
    println!("[chatter-pub] WASI mode, fd={}", PREOPEN_TCP_FD);
    let stream = WasiTcpStream::from_raw_fd(PREOPEN_TCP_FD);
    block_on(run(stream));
}

#[cfg(not(target_arch = "wasm32"))]
#[tokio::main(flavor = "current_thread")]
async fn main() {
    use embedded_io_adapters::tokio_1::FromTokio;
    use tokio::net::TcpStream;

    let addr = std::env::var("ZENOH_ROUTER_ADDR").unwrap_or_else(|_| "127.0.0.1:7447".to_string());
    println!("[chatter-pub] connecting to {}", addr);

    let tcp = TcpStream::connect(&addr).await.expect("TCP connect failed");
    let stream = FromTokio::new(tcp);
    run(stream).await;
}

async fn run<T: embedded_io_async::Read + embedded_io_async::Write>(mut stream: T) {
    // Zenoh handshake
    let our_zid = ZenohId::from_bytes(&[0xDA, 0x51, 0x7B, 0x03]);
    let mut tx = [0u8; 1024];
    let mut rx = [0u8; 4096];

    let _hs = match handshake::client_handshake(&mut stream, &our_zid, &mut tx, &mut rx).await {
        Ok(h) => {
            println!("[chatter-pub] connected (lease={}ms)", h.lease_ms);
            h
        }
        Err(e) => {
            println!("[chatter-pub] handshake failed: {:?}", e);
            return;
        }
    };

    // Declare key expression for /chatter
    let key_expr = match std_msgs::String::CHATTER.to_key_expr() {
        Ok(ke) => ke,
        Err(_) => {
            println!("[chatter-pub] failed to build key expression");
            return;
        }
    };

    let mut sn: u64 = 0;
    let mut pos = 0;
    pos += codec::encode_frame_header(&mut tx[pos..], sn, true).unwrap_or(0);
    sn += 1;
    pos += codec::encode_declare_keyexpr(&mut tx[pos..], 1, key_expr.as_str()).unwrap_or(0);
    if frame::write_frame(&mut stream, &tx[..pos]).await.is_err() {
        println!("[chatter-pub] failed to declare key expression");
        return;
    }
    println!("[chatter-pub] declared: {}", key_expr.as_str());

    // Publish loop
    let mut seq: i64 = 0;
    for i in 0..NUM_MESSAGES {
        seq += 1;

        // Build CDR String message: header(4) + len(4) + utf8 + nul
        let mut cdr_buf = [0u8; 256];
        let text = format_msg(i);
        let cdr_len = encode_cdr_string(&mut cdr_buf, text.as_bytes());

        pos = 0;
        pos += codec::encode_frame_header(&mut tx[pos..], sn, true).unwrap_or(0);
        sn += 1;
        pos += codec::encode_push_put_with_attachment(
            &mut tx[pos..],
            key_expr.as_str(),
            &cdr_buf[..cdr_len],
            seq,
            0,
            &our_zid,
        )
        .unwrap_or(0);

        if frame::write_frame(&mut stream, &tx[..pos]).await.is_err() {
            println!("[chatter-pub] write failed at msg {}", i);
            return;
        }

        println!("[chatter-pub] #{:>2} published: \"{}\"", seq, text.as_str());

        // KeepAlive every 5 messages
        if i % 5 == 4 {
            let n = codec::encode_keepalive(&mut tx).unwrap_or(1);
            let _ = frame::write_frame(&mut stream, &tx[..n]).await;
        }

        // Delay (platform-specific)
        delay().await;
    }

    // Graceful close
    let n = codec::encode_close(&mut tx, 0x00).unwrap_or(2);
    let _ = frame::write_frame(&mut stream, &tx[..n]).await;
    println!(
        "[chatter-pub] done — published {} messages to /chatter",
        seq
    );
}

/// Format a message string. Uses a small stack buffer to avoid alloc.
fn format_msg(i: u32) -> heapless::String<128> {
    use core::fmt::Write;
    let mut s = heapless::String::<128>::new();
    let _ = write!(s, "Hello from zenoh-ros2-nostd! [{}]", i);
    s
}

/// Encode a CDR String: [0x00,0x01,0x00,0x00] + u32_le(len+1) + bytes + 0x00
fn encode_cdr_string(buf: &mut [u8], text: &[u8]) -> usize {
    let str_len = (text.len() + 1) as u32; // +1 for NUL terminator
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x00; // CDR LE header
    buf[4..8].copy_from_slice(&str_len.to_le_bytes());
    buf[8..8 + text.len()].copy_from_slice(text);
    buf[8 + text.len()] = 0x00; // NUL terminator
    8 + text.len() + 1
}

/// Platform-specific delay between publishes.
#[cfg(not(target_arch = "wasm32"))]
async fn delay() {
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;
}

#[cfg(target_arch = "wasm32")]
async fn delay() {
    for _ in 0..500_000u32 {
        core::hint::spin_loop();
    }
}

/// Trivial single-threaded executor for WASI blocking I/O.
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
