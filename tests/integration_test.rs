//! Integration tests for zenoh-ros2-nostd against a real zenohd router.
//!
//! These tests require a running zenohd on localhost:7447.
//! Run with: `cargo test --test integration_test --no-default-features -- --ignored`
//!
//! Or automatically (zenohd started/stopped by the test harness):
//! ```sh
//! zenohd --no-multicast-scouting -l tcp/127.0.0.1:7447 &
//! cargo test --test integration_test --no-default-features -- --ignored
//! kill %1
//! ```

use std::process::{Child, Command};
use std::time::Duration;

use embedded_io_adapters::tokio_1::FromTokio;
use tokio::net::TcpStream;

use zenoh_ros2_nostd::transport::{codec, frame, handshake, protocol::*};

/// Return the Zenoh router address, honouring ZENOH_ROUTER_ADDR env var.
///
/// Inside the dev container this is set to `zenoh-router:7447` by compose.
/// Defaults to `127.0.0.1:7447` for local runs without docker.
fn router_addr() -> String {
    std::env::var("ZENOH_ROUTER_ADDR").unwrap_or_else(|_| "127.0.0.1:7447".to_string())
}

/// Helper: ensure zenohd is running. Returns Some(child) if we started one, None if pre-existing.
fn ensure_zenohd() -> Option<Child> {
    // Check if zenohd is already running on the configured address
    if std::net::TcpStream::connect_timeout(&router_addr().parse().unwrap(), Duration::from_secs(1))
        .is_ok()
    {
        return None;
    }

    let child = Command::new("zenohd")
        .args(["--no-multicast-scouting", "-l", "tcp/127.0.0.1:7447"])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .expect("Failed to start zenohd. Is it installed?");
    // Give zenohd time to bind the port
    std::thread::sleep(Duration::from_secs(2));
    child.into()
}

/// Helper: stop zenohd only if we started it.
fn cleanup_zenohd(child: Option<Child>) {
    if let Some(mut c) = child {
        let _ = c.kill();
        let _ = c.wait();
    }
}

/// Helper: check if zenohd is reachable at the configured router address.
async fn zenohd_reachable() -> bool {
    tokio::time::timeout(
        Duration::from_secs(2),
        TcpStream::connect(router_addr().as_str()),
    )
    .await
    .map(|r| r.is_ok())
    .unwrap_or(false)
}

/// Test 1: TCP connection + Zenoh v9 handshake with real zenohd.
///
/// Verifies: InitSyn → InitAck → OpenSyn → OpenAck completes successfully.
#[tokio::test]
#[ignore]
async fn test_handshake_with_zenohd() {
    let zenohd = ensure_zenohd();

    let result = async {
        assert!(
            zenohd_reachable().await,
            "zenohd not reachable on {}",
            router_addr()
        );

        let tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link = FromTokio::new(tcp);

        let our_zid = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x01]);
        let mut tx_buf = [0u8; 512];
        let mut rx_buf = [0u8; 4096];

        let result =
            handshake::client_handshake(&mut link, &our_zid, &mut tx_buf, &mut rx_buf).await;

        assert!(result.is_ok(), "Handshake failed: {:?}", result.err());
        let hs = result.unwrap();
        assert!(hs.lease_ms > 0, "Lease should be > 0, got {}", hs.lease_ms);
        assert!(hs.router_zid.len > 0, "Router ZID should be non-empty");

        eprintln!(
            "Handshake OK: router_zid={:?}, lease={}ms",
            hs.router_zid, hs.lease_ms
        );
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 2: Send KeepAlive after handshake.
///
/// Verifies the session stays alive with KeepAlive messages.
#[tokio::test]
#[ignore]
async fn test_keepalive_after_handshake() {
    let zenohd = ensure_zenohd();

    let result = async {
        let tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link = FromTokio::new(tcp);

        let our_zid = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x02]);
        let mut tx_buf = [0u8; 512];
        let mut rx_buf = [0u8; 4096];

        let _hs = handshake::client_handshake(&mut link, &our_zid, &mut tx_buf, &mut rx_buf)
            .await
            .expect("Handshake failed");

        // Send a KeepAlive
        let n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
        frame::write_frame(&mut link, &tx_buf[..n])
            .await
            .expect("write keepalive frame");

        // Send another KeepAlive after a short delay
        tokio::time::sleep(Duration::from_millis(100)).await;

        let n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
        frame::write_frame(&mut link, &tx_buf[..n])
            .await
            .expect("write second keepalive frame");

        eprintln!("KeepAlive sent successfully (2x)");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 3: Declare a key expression after handshake.
///
/// Verifies that DeclareKeyExpr messages are accepted by the router.
#[tokio::test]
#[ignore]
async fn test_declare_keyexpr() {
    let zenohd = ensure_zenohd();

    let result = async {
        let tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link = FromTokio::new(tcp);

        let our_zid = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x03]);
        let mut tx_buf = [0u8; 512];
        let mut rx_buf = [0u8; 4096];

        let _hs = handshake::client_handshake(
            &mut link,
            &our_zid,
            &mut tx_buf,
            &mut rx_buf,
        )
        .await
        .expect("Handshake failed");

        // Build a ROS2-compatible key expression
        let key_expr =
            "0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

        // Encode declare keyexpr inside a frame
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_buf[pos..], 0, true)
            .expect("encode frame header");
        pos += codec::encode_declare_keyexpr(&mut tx_buf[pos..], 1, key_expr)
            .expect("encode declare keyexpr");

        frame::write_frame(&mut link, &tx_buf[..pos])
            .await
            .expect("write declare keyexpr frame");

        eprintln!("DeclareKeyExpr sent: {}", key_expr);

        // If the router rejects, it would close the connection.
        // Send a keepalive to verify the connection is still alive.
        tokio::time::sleep(Duration::from_millis(200)).await;

        let n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
        frame::write_frame(&mut link, &tx_buf[..n])
            .await
            .expect("connection still alive after declare");

        eprintln!("Connection alive after DeclareKeyExpr — OK");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 4: Publish a CDR-encoded std_msgs/String on /chatter topic.
///
/// Verifies the full pipeline: handshake → declare → put with CDR payload.
#[tokio::test]
#[ignore]
async fn test_publish_chatter_topic() {
    let zenohd = ensure_zenohd();

    let result = async {
        let tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link = FromTokio::new(tcp);

        let our_zid = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x04]);
        let mut tx_buf = [0u8; 1024];
        let mut rx_buf = [0u8; 4096];

        let _hs = handshake::client_handshake(
            &mut link,
            &our_zid,
            &mut tx_buf,
            &mut rx_buf,
        )
        .await
        .expect("Handshake failed");

        // Key expression for /chatter topic
        let key_expr =
            "0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

        // CDR-encode a std_msgs::msg::String { data: "Hello from MCU!" }
        // CDR LE encapsulation header + CDR-encoded string
        let message = "Hello from MCU!";
        let mut cdr_buf = [0u8; 256];
        let cdr_len = build_cdr_string(&mut cdr_buf, message);

        // Send as a Push+Put inside a Frame
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_buf[pos..], 0, true)
            .expect("encode frame header");
        pos += codec::encode_push_put(
            &mut tx_buf[pos..],
            key_expr,
            0, // encoding_id = empty (CDR is implicit for ROS2)
            &cdr_buf[..cdr_len],
        )
        .expect("encode push put");

        frame::write_frame(&mut link, &tx_buf[..pos])
            .await
            .expect("write put frame");

        eprintln!(
            "Published to {}: \"{}\" ({} CDR bytes)",
            key_expr, message, cdr_len
        );

        // Verify connection still alive
        tokio::time::sleep(Duration::from_millis(200)).await;
        let n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
        frame::write_frame(&mut link, &tx_buf[..n])
            .await
            .expect("connection alive after put");

        eprintln!("Put accepted by router — OK");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 5: Graceful close after handshake.
#[tokio::test]
#[ignore]
async fn test_close_session() {
    let zenohd = ensure_zenohd();

    let result = async {
        let tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link = FromTokio::new(tcp);

        let our_zid = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x05]);
        let mut tx_buf = [0u8; 512];
        let mut rx_buf = [0u8; 4096];

        let _hs = handshake::client_handshake(&mut link, &our_zid, &mut tx_buf, &mut rx_buf)
            .await
            .expect("Handshake failed");

        // Send Close
        let n = codec::encode_close(&mut tx_buf, 0x00).expect("encode close");
        frame::write_frame(&mut link, &tx_buf[..n])
            .await
            .expect("write close frame");

        eprintln!("Close sent — session closed gracefully");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

// --- Helper: build CDR-encoded std_msgs::msg::String ---

/// Manually build CDR LE encoding for std_msgs::msg::String.
///
/// CDR format for std_msgs::msg::String:
///   [4 bytes: encapsulation header 0x00 0x01 0x00 0x00]
///   [4 bytes: string length (u32 LE, including null terminator)]
///   [N bytes: UTF-8 string data]
///   [1 byte:  null terminator]
fn build_cdr_string(buf: &mut [u8], s: &str) -> usize {
    let mut pos = 0;

    // CDR LE encapsulation header
    buf[pos..pos + 4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
    pos += 4;

    // String length (including null terminator)
    let str_len = (s.len() + 1) as u32;
    buf[pos..pos + 4].copy_from_slice(&str_len.to_le_bytes());
    pos += 4;

    // String data
    buf[pos..pos + s.len()].copy_from_slice(s.as_bytes());
    pos += s.len();

    // Null terminator
    buf[pos] = 0;
    pos += 1;

    pos
}
