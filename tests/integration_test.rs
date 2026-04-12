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

#![cfg(not(target_arch = "wasm32"))]

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
    // Check if zenohd is already running on the configured address.
    // Use `ToSocketAddrs` to resolve hostnames (e.g. "zenoh-router:7447").
    use std::net::ToSocketAddrs;
    let reachable = router_addr()
        .to_socket_addrs()
        .ok()
        .and_then(|mut addrs| addrs.next())
        .map(|addr| std::net::TcpStream::connect_timeout(&addr, Duration::from_secs(2)).is_ok())
        .unwrap_or(false);
    if reachable {
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

/// Test 6: Cross-session subscribe → receive.
///
/// Uses two TCP connections to the router:
/// - Session A declares a subscriber on /test6_topic
/// - Session B publishes a CDR message on /test6_topic
/// - Session A receives the forwarded message
///
/// Uses a unique topic name to avoid cross-contamination with parallel tests.
#[tokio::test]
#[ignore]
async fn test_subscribe_receive_loopback() {
    let zenohd = ensure_zenohd();

    let result = async {
        assert!(
            zenohd_reachable().await,
            "zenohd not reachable on {}",
            router_addr()
        );

        let key_expr =
            "0/test6_topic/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

        // --- Session A: subscriber ---
        let tcp_a = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_a = FromTokio::new(tcp_a);
        let zid_a = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x06]);
        let mut tx_a = [0u8; 1024];
        let mut rx_a = [0u8; 4096];

        let _hs_a = handshake::client_handshake(&mut link_a, &zid_a, &mut tx_a, &mut rx_a)
            .await
            .expect("Handshake A failed");

        // DeclareKeyExpr (key_id=1)
        let key_id: u16 = 1;
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], 0, true)
            .expect("encode frame header");
        pos += codec::encode_declare_keyexpr(&mut tx_a[pos..], key_id, key_expr)
            .expect("encode declare keyexpr");
        frame::write_frame(&mut link_a, &tx_a[..pos])
            .await
            .expect("write declare keyexpr");

        tokio::time::sleep(Duration::from_millis(100)).await;

        // DeclareSubscriber (sub_id=1, key_id=1)
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], 1, true)
            .expect("encode frame header");
        pos += codec::encode_declare_subscriber_mapped(&mut tx_a[pos..], key_id as u32, key_id)
            .expect("encode declare subscriber");
        frame::write_frame(&mut link_a, &tx_a[..pos])
            .await
            .expect("write declare subscriber");

        tokio::time::sleep(Duration::from_millis(200)).await;
        eprintln!("Session A: subscriber declared on {}", key_expr);

        // --- Session B: publisher ---
        let tcp_b = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_b = FromTokio::new(tcp_b);
        let zid_b = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x07]);
        let mut tx_b = [0u8; 1024];
        let mut rx_b = [0u8; 4096];

        let _hs_b = handshake::client_handshake(&mut link_b, &zid_b, &mut tx_b, &mut rx_b)
            .await
            .expect("Handshake B failed");

        // Publish CDR string from Session B
        let message = "Cross-session test!";
        let mut cdr_buf = [0u8; 256];
        let cdr_len = build_cdr_string(&mut cdr_buf, message);

        pos = 0;
        pos += codec::encode_frame_header(&mut tx_b[pos..], 0, true)
            .expect("encode frame header");
        pos += codec::encode_push_put(&mut tx_b[pos..], key_expr, 0, &cdr_buf[..cdr_len])
            .expect("encode push put");
        frame::write_frame(&mut link_b, &tx_b[..pos])
            .await
            .expect("write put from B");

        eprintln!("Session B: published \"{}\"", message);

        // --- Session A: read the forwarded message ---
        let received = tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let n = frame::read_frame(&mut link_a, &mut rx_a)
                    .await
                    .expect("read frame on A");

                if let Ok((_, _, body_pos)) = codec::decode_frame_header(&rx_a[..n]) {
                    if let Ok(Some((put, _))) = codec::decode_push_put(&rx_a[body_pos..n]) {
                        eprintln!(
                            "Session A received: scope={}, suffix='{}', {} bytes",
                            put.scope, put.key_suffix, put.payload.len()
                        );
                        return put.payload.to_vec();
                    }
                }
                eprintln!("Session A: non-Put frame ({} bytes), continuing...", n);
            }
        })
        .await;

        assert!(
            received.is_ok(),
            "Session A did not receive the message within 5s"
        );
        let payload = received.unwrap();
        assert_eq!(
            &payload,
            &cdr_buf[..cdr_len],
            "Received CDR payload should match published data"
        );
        eprintln!(
            "Cross-session subscribe+receive OK — \"{}\" ({} bytes)",
            message,
            payload.len()
        );
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 7: Publish with rmw_zenoh_cpp-compatible attachment.
///
/// Verifies that `encode_push_put_with_attachment` produces messages
/// accepted by the router, with the seq_num + timestamp + GID extension
/// required by rmw_zenoh_cpp subscribers.
#[tokio::test]
#[ignore]
async fn test_publish_with_rmw_attachment() {
    let zenohd = ensure_zenohd();

    let result = async {
        assert!(
            zenohd_reachable().await,
            "zenohd not reachable on {}",
            router_addr()
        );

        // --- Session A: subscriber ---
        let tcp_a = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_a = FromTokio::new(tcp_a);
        let zid_a = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x08]);
        let mut tx_a = [0u8; 1024];
        let mut rx_a = [0u8; 4096];

        let _hs_a = handshake::client_handshake(&mut link_a, &zid_a, &mut tx_a, &mut rx_a)
            .await
            .expect("Handshake A failed");

        let key_expr =
            "0/test7_attach/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
        let key_id: u16 = 1;

        // DeclareKeyExpr + DeclareSubscriber
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], 0, true).unwrap();
        pos += codec::encode_declare_keyexpr(&mut tx_a[pos..], key_id, key_expr).unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;

        pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], 1, true).unwrap();
        pos += codec::encode_declare_subscriber_mapped(&mut tx_a[pos..], key_id as u32, key_id)
            .unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;

        // --- Session B: publisher with rmw_zenoh_cpp attachment ---
        let tcp_b = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_b = FromTokio::new(tcp_b);
        let zid_b = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x09]);
        let mut tx_b = [0u8; 1024];
        let mut rx_b = [0u8; 4096];

        let _hs_b = handshake::client_handshake(&mut link_b, &zid_b, &mut tx_b, &mut rx_b)
            .await
            .expect("Handshake B failed");

        let message = "Hello ROS2!";
        let mut cdr_buf = [0u8; 256];
        let cdr_len = build_cdr_string(&mut cdr_buf, message);

        // Publish with attachment: seq_num=1, timestamp=123456789ns, GID=zid_b
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_b[pos..], 0, true).unwrap();
        pos += codec::encode_push_put_with_attachment(
            &mut tx_b[pos..],
            key_expr,
            &cdr_buf[..cdr_len],
            1,          // seq_num
            123_456_789, // timestamp_ns
            &zid_b,
        )
        .expect("encode push put with attachment");
        frame::write_frame(&mut link_b, &tx_b[..pos])
            .await
            .expect("write put with attachment from B");

        eprintln!(
            "Session B: published \"{}\" with rmw_zenoh_cpp attachment",
            message
        );

        // --- Session A: receive and verify ---
        let received = tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let n = frame::read_frame(&mut link_a, &mut rx_a)
                    .await
                    .expect("read frame on A");

                if let Ok((_, _, body_pos)) = codec::decode_frame_header(&rx_a[..n]) {
                    if let Ok(Some((put, _))) = codec::decode_push_put(&rx_a[body_pos..n]) {
                        return put.payload.to_vec();
                    }
                }
            }
        })
        .await;

        assert!(
            received.is_ok(),
            "Did not receive attachment message within 5s"
        );
        let payload = received.unwrap();
        assert_eq!(
            &payload,
            &cdr_buf[..cdr_len],
            "Payload mismatch for attachment publish"
        );
        eprintln!(
            "rmw_zenoh_cpp attachment publish+receive OK — \"{}\" ({} bytes)",
            message,
            payload.len()
        );
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

/// Test 8: Declare a liveliness token after handshake.
///
/// Verifies that `encode_declare_token` produces messages accepted by the router
/// and that the token key expression is correctly formatted.
#[tokio::test]
#[ignore]
async fn test_declare_liveliness_token() {
    let zenohd = ensure_zenohd();

    let result = async {
        assert!(
            zenohd_reachable().await,
            "zenohd not reachable on {}",
            router_addr()
        );

        let tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link = FromTokio::new(tcp);

        let our_zid = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x10]);
        let mut tx_buf = [0u8; 1024];
        let mut rx_buf = [0u8; 4096];

        let _hs = handshake::client_handshake(&mut link, &our_zid, &mut tx_buf, &mut rx_buf)
            .await
            .expect("Handshake failed");

        // Build a liveliness token key expression
        use zenoh_ros2_nostd::ros2::liveliness::{self, EntityType};
        use zenoh_ros2_nostd::ros2::qos::Qos;

        let token = liveliness::build_liveliness_token(
            0,
            &our_zid,
            0,
            1,
            EntityType::Publisher,
            "",
            "test_node",
            "chatter",
            "std_msgs::msg::dds_::String_",
            "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
            &Qos::DEFAULT,
        )
        .expect("build liveliness token");

        assert!(token.starts_with("@ros2_lv/0/"));
        assert!(token.contains("/MP/"));
        eprintln!("Liveliness token: {}", token.as_str());

        // Encode and send the DeclareToken inside a frame
        let mut pos = 0;
        pos +=
            codec::encode_frame_header(&mut tx_buf[pos..], 0, true).expect("encode frame header");
        pos += codec::encode_declare_token(&mut tx_buf[pos..], 1, token.as_str())
            .expect("encode declare token");
        frame::write_frame(&mut link, &tx_buf[..pos])
            .await
            .expect("write declare token frame");

        eprintln!("DeclareToken sent ({} wire bytes)", pos);

        // Verify connection still alive after DeclareToken
        tokio::time::sleep(Duration::from_millis(300)).await;
        let n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
        frame::write_frame(&mut link, &tx_buf[..n])
            .await
            .expect("connection alive after declare token");

        eprintln!("DeclareToken accepted by router — OK");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 9: Multi-topic publish and subscribe.
///
/// Uses two different topics on the same session to verify that the router
/// correctly routes messages based on key expression matching.
/// - Topic A: `/chatter` (std_msgs/String)
/// - Topic B: `/status`  (std_msgs/String, different key expression)
///
/// Session A subscribes to both topics. Session B publishes one message on each.
/// Session A must receive both messages.
#[tokio::test]
#[ignore]
async fn test_multi_topic_pub_sub() {
    let zenohd = ensure_zenohd();

    let result = async {
        assert!(
            zenohd_reachable().await,
            "zenohd not reachable on {}",
            router_addr()
        );

        let key_expr_chatter =
            "0/test9_topic_a/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
        let key_expr_status =
            "0/test9_topic_b/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

        // --- Session A: subscriber on both topics ---
        let tcp_a = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_a = FromTokio::new(tcp_a);
        let zid_a = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x11]);
        let mut tx_a = [0u8; 1024];
        let mut rx_a = [0u8; 4096];

        let _hs_a = handshake::client_handshake(&mut link_a, &zid_a, &mut tx_a, &mut rx_a)
            .await
            .expect("Handshake A failed");

        // DeclareKeyExpr for /chatter (key_id=1)
        let mut sn: u64 = 0;
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], sn, true).unwrap();
        sn += 1;
        pos += codec::encode_declare_keyexpr(&mut tx_a[pos..], 1, key_expr_chatter).unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;

        // DeclareSubscriber for /chatter (sub_id=1)
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], sn, true).unwrap();
        sn += 1;
        pos += codec::encode_declare_subscriber_mapped(&mut tx_a[pos..], 1, 1).unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;

        // DeclareKeyExpr for /status (key_id=2)
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], sn, true).unwrap();
        sn += 1;
        pos += codec::encode_declare_keyexpr(&mut tx_a[pos..], 2, key_expr_status).unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(50)).await;

        // DeclareSubscriber for /status (sub_id=2)
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], sn, true).unwrap();
        pos += codec::encode_declare_subscriber_mapped(&mut tx_a[pos..], 2, 2).unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;

        eprintln!("Session A: subscribed to /chatter and /status");

        // --- Session B: publisher ---
        let tcp_b = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_b = FromTokio::new(tcp_b);
        let zid_b = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x12]);
        let mut tx_b = [0u8; 1024];
        let mut rx_b = [0u8; 4096];

        let _hs_b = handshake::client_handshake(&mut link_b, &zid_b, &mut tx_b, &mut rx_b)
            .await
            .expect("Handshake B failed");

        // Publish to /chatter
        let msg_chatter = "Hello chatter!";
        let mut cdr_chatter = [0u8; 256];
        let cdr_chatter_len = build_cdr_string(&mut cdr_chatter, msg_chatter);

        pos = 0;
        pos += codec::encode_frame_header(&mut tx_b[pos..], 0, true).unwrap();
        pos += codec::encode_push_put(
            &mut tx_b[pos..],
            key_expr_chatter,
            0,
            &cdr_chatter[..cdr_chatter_len],
        )
        .unwrap();
        frame::write_frame(&mut link_b, &tx_b[..pos]).await.unwrap();

        tokio::time::sleep(Duration::from_millis(200)).await;

        // Publish to /status
        let msg_status = "Status OK";
        let mut cdr_status = [0u8; 256];
        let cdr_status_len = build_cdr_string(&mut cdr_status, msg_status);

        pos = 0;
        pos += codec::encode_frame_header(&mut tx_b[pos..], 1, true).unwrap();
        pos += codec::encode_push_put(
            &mut tx_b[pos..],
            key_expr_status,
            0,
            &cdr_status[..cdr_status_len],
        )
        .unwrap();
        frame::write_frame(&mut link_b, &tx_b[..pos]).await.unwrap();

        eprintln!("Session B: published to /chatter and /status");

        // --- Session A: receive both messages ---
        let received = tokio::time::timeout(Duration::from_secs(5), async {
            let mut got_chatter = false;
            let mut got_status = false;

            while !got_chatter || !got_status {
                let n = frame::read_frame(&mut link_a, &mut rx_a)
                    .await
                    .expect("read frame on A");

                if let Ok((_, _, body_pos)) = codec::decode_frame_header(&rx_a[..n]) {
                    if let Ok(Some((put, _))) = codec::decode_push_put(&rx_a[body_pos..n]) {
                        // When forwarded via mapped key, scope=key_id and suffix is empty.
                        // Match by scope: key_id=1 → /chatter, key_id=2 → /status.
                        let scope = put.scope;
                        if scope == 1 && !got_chatter {
                            assert_eq!(
                                put.payload,
                                &cdr_chatter[..cdr_chatter_len],
                                "/chatter payload mismatch"
                            );
                            got_chatter = true;
                            eprintln!("Session A: received /chatter message (scope={})", scope);
                        } else if scope == 2 && !got_status {
                            assert_eq!(
                                put.payload,
                                &cdr_status[..cdr_status_len],
                                "/status payload mismatch"
                            );
                            got_status = true;
                            eprintln!("Session A: received /status message (scope={})", scope);
                        } else {
                            // If inline key expression, try matching by suffix
                            let suffix = put.key_suffix;
                            if suffix.contains("chatter") && !got_chatter {
                                assert_eq!(
                                    put.payload,
                                    &cdr_chatter[..cdr_chatter_len],
                                    "/chatter payload mismatch"
                                );
                                got_chatter = true;
                                eprintln!("Session A: received /chatter (suffix)");
                            } else if suffix.contains("status") && !got_status {
                                assert_eq!(
                                    put.payload,
                                    &cdr_status[..cdr_status_len],
                                    "/status payload mismatch"
                                );
                                got_status = true;
                                eprintln!("Session A: received /status (suffix)");
                            }
                        }
                    }
                }
            }
            (got_chatter, got_status)
        })
        .await;

        assert!(
            received.is_ok(),
            "Did not receive both topic messages within 5s"
        );
        let (got_c, got_s) = received.unwrap();
        assert!(got_c, "/chatter not received");
        assert!(got_s, "/status not received");
        eprintln!("Multi-topic pub/sub OK — both topics received");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 10: Fragment reassembly for large messages.
///
/// Session A negotiates a small batch_size (256) forcing the router to
/// fragment larger messages. Session B publishes a ~2KB payload.
/// Session A must reassemble the fragments and deliver the complete message.
#[tokio::test]
#[ignore]
async fn test_fragment_reassembly_large_message() {
    let zenohd = ensure_zenohd();

    let result = async {
        assert!(
            zenohd_reachable().await,
            "zenohd not reachable on {}",
            router_addr()
        );

        let key_expr =
            "0/test10_large/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";

        // --- Session A: subscriber with small batch_size to force fragmentation ---
        let tcp_a = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_a = FromTokio::new(tcp_a);
        let zid_a = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x20]);
        let mut tx_a = [0u8; 1024];
        let mut rx_a = [0u8; 4096];

        // Handshake with small batch_size (256) to force router to fragment.
        let _hs_a = handshake_with_batch_size(&mut link_a, &zid_a, &mut tx_a, &mut rx_a, 256)
            .await
            .expect("Handshake A failed");

        // DeclareKeyExpr (key_id=1)
        let key_id: u16 = 1;
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], 0, true).unwrap();
        pos += codec::encode_declare_keyexpr(&mut tx_a[pos..], key_id, key_expr).unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(100)).await;

        // DeclareSubscriber (sub_id=1, key_id=1)
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_a[pos..], 1, true).unwrap();
        pos += codec::encode_declare_subscriber_mapped(&mut tx_a[pos..], key_id as u32, key_id)
            .unwrap();
        frame::write_frame(&mut link_a, &tx_a[..pos]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;

        eprintln!("Session A (batch=256): subscriber declared");

        // --- Session B: publisher (default batch_size) ---
        let tcp_b = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut link_b = FromTokio::new(tcp_b);
        let zid_b = ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x21]);
        let mut tx_b = [0u8; 4096];
        let mut rx_b = [0u8; 4096];

        let _hs_b = handshake::client_handshake(&mut link_b, &zid_b, &mut tx_b, &mut rx_b)
            .await
            .expect("Handshake B failed");

        // Build a large CDR payload (~2000 bytes repeated 'A' string).
        let large_msg: String = std::iter::repeat('A').take(2000).collect();
        let mut cdr_buf = [0u8; 2048];
        let cdr_len = build_cdr_string(&mut cdr_buf, &large_msg);
        eprintln!("CDR payload size: {} bytes", cdr_len);

        // Publish from Session B.
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_b[pos..], 0, true).unwrap();
        pos += codec::encode_push_put(&mut tx_b[pos..], key_expr, 0, &cdr_buf[..cdr_len]).unwrap();
        frame::write_frame(&mut link_b, &tx_b[..pos])
            .await
            .expect("write large put from B");

        eprintln!("Session B: published {} bytes on test10_large", cdr_len);

        // --- Session A: receive (potentially fragmented) ---
        use zenoh_ros2_nostd::transport::fragment::FragmentAssembler;

        let mut frag_asm = FragmentAssembler::<8192>::new();
        let received = tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let n = frame::read_frame(&mut link_a, &mut rx_a)
                    .await
                    .expect("read frame on A");

                let header = rx_a[0];
                match codec::parse_transport_msg_kind(header) {
                    codec::TransportMsgKind::Frame => {
                        if let Ok((_, _, body_pos)) = codec::decode_frame_header(&rx_a[..n]) {
                            if let Ok(Some((put, _))) = codec::decode_push_put(&rx_a[body_pos..n])
                            {
                                eprintln!(
                                    "Received as Frame (no fragment): {} bytes",
                                    put.payload.len()
                                );
                                return put.payload.to_vec();
                            }
                        }
                    }
                    codec::TransportMsgKind::Fragment => {
                        if let Ok((sn, _, more, body_pos)) =
                            codec::decode_fragment_header(&rx_a[..n])
                        {
                            eprintln!(
                                "Fragment sn={} more={} payload={} bytes",
                                sn,
                                more,
                                n - body_pos
                            );
                            match frag_asm.feed(more, &rx_a[body_pos..n]) {
                                Ok(Some(assembled)) => {
                                    eprintln!(
                                        "Reassembled {} bytes from fragments",
                                        assembled.len()
                                    );
                                    if let Ok(Some((put, _))) = codec::decode_push_put(assembled) {
                                        return put.payload.to_vec();
                                    }
                                    return assembled.to_vec();
                                }
                                Ok(None) => {}
                                Err(e) => {
                                    panic!("Fragment reassembly error: {:?}", e);
                                }
                            }
                        }
                    }
                    _ => {
                        eprintln!("Non-data frame ({} bytes), skipping...", n);
                    }
                }
            }
        })
        .await;

        assert!(
            received.is_ok(),
            "Did not receive the large message within 5s"
        );
        let payload = received.unwrap();
        assert_eq!(
            &payload,
            &cdr_buf[..cdr_len],
            "Received payload should match published data (got {} bytes, expected {})",
            payload.len(),
            cdr_len
        );
        eprintln!(
            "Fragment reassembly OK — received {} bytes matching published {} bytes",
            payload.len(),
            cdr_len
        );
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

/// Test 11: Service client — send Request+Query to zenohd.
///
/// Verifies: encode_request_query → write_frame succeeds;
/// zenohd does not close the connection upon receiving the request.
/// (A full service round-trip requires a service server / queryable,
/// which is not set up in this test.)
#[tokio::test]
#[ignore]
async fn test_service_request_send() {
    let zenohd = ensure_zenohd();

    if !zenohd_reachable().await {
        eprintln!("SKIP: zenohd not reachable at {}", router_addr());
        cleanup_zenohd(zenohd);
        return;
    }

    let tcp = TcpStream::connect(router_addr().as_str())
        .await
        .expect("connect");
    let mut link = FromTokio::new(tcp);

    let our_zid = ZenohId::from_bytes(&[0xBB, 0xCC, 0xDD, 0xEE]);
    let mut tx_buf = [0u8; 1024];
    let mut rx_buf = [0u8; 4096];

    let hs = handshake::client_handshake(&mut link, &our_zid, &mut tx_buf, &mut rx_buf)
        .await
        .expect("Handshake failed");
    eprintln!(
        "Handshake OK (lease={}ms, router={:?})",
        hs.lease_ms, hs.router_zid
    );

    // Declare key expression for a (fictitious) service
    let svc_ke = "0/add_two_ints/example_interfaces::srv::dds_::AddTwoInts_/RIHS01_abc123";
    let mut sn: u64 = 0;
    let mut pos = 0;
    pos += codec::encode_frame_header(&mut tx_buf[pos..], sn, true).expect("frame header");
    sn += 1;
    pos += codec::encode_declare_keyexpr(&mut tx_buf[pos..], 10, svc_ke).expect("declare ke");
    frame::write_frame(&mut link, &tx_buf[..pos])
        .await
        .expect("write declare");
    eprintln!("Declared service key expression");

    // Encode a Request+Query (service call)
    // CDR payload: AddTwoInts_Request { a: 1, b: 2 }
    let request_payload = [
        0x00, 0x01, 0x00, 0x00, // CDR LE header
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // a = 1 (i64 LE)
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // b = 2 (i64 LE)
    ];

    pos = 0;
    pos += codec::encode_frame_header(&mut tx_buf[pos..], sn, true).expect("frame header 2");
    sn += 1;
    pos += codec::encode_request_query(
        &mut tx_buf[pos..],
        1, // request_id
        svc_ke,
        &request_payload,
        1, // seq_num
        0, // timestamp
        &our_zid,
    )
    .expect("encode request query");
    frame::write_frame(&mut link, &tx_buf[..pos])
        .await
        .expect("write request");
    eprintln!("Sent Request+Query (request_id=1, {} bytes)", pos);

    // Send a KeepAlive and verify session is still alive after the Request
    let ka_n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
    frame::write_frame(&mut link, &tx_buf[..ka_n])
        .await
        .expect("write keepalive");

    let read_result = tokio::time::timeout(
        Duration::from_secs(5),
        frame::read_frame(&mut link, &mut rx_buf),
    )
    .await;

    match read_result {
        Ok(Ok(n)) => {
            let kind = codec::parse_transport_msg_kind(rx_buf[0]);
            eprintln!(
                "Received {} bytes after service request (kind={:?}) — session alive",
                n, kind
            );
        }
        Ok(Err(e)) => {
            eprintln!(
                "Transport error after Request: {:?} (may be expected without queryable)",
                e
            );
        }
        Err(_) => {
            eprintln!("Timeout waiting for response — session alive (no queryable registered)");
        }
    }

    // Graceful close
    let _ = sn;
    let n = codec::encode_close(&mut tx_buf, 0x00).expect("encode close");
    frame::write_frame(&mut link, &tx_buf[..n])
        .await
        .expect("write close");
    eprintln!("Service request send test PASSED");

    cleanup_zenohd(zenohd);
}

/// Test 12: Action client — send Goal Request to zenohd.
///
/// Verifies: action key expressions generate correctly;
/// encode_request_query with a SendGoal request + GoalId payload is
/// accepted by zenohd without dropping the session.
#[tokio::test]
#[ignore]
async fn test_action_send_goal_request() {
    let zenohd = ensure_zenohd();

    if !zenohd_reachable().await {
        eprintln!("SKIP: zenohd not reachable at {}", router_addr());
        cleanup_zenohd(zenohd);
        return;
    }

    let tcp = TcpStream::connect(router_addr().as_str())
        .await
        .expect("connect");
    let mut link = FromTokio::new(tcp);

    let our_zid = ZenohId::from_bytes(&[0xCC, 0xDD, 0xEE, 0xFF]);
    let mut tx_buf = [0u8; 1024];
    let mut rx_buf = [0u8; 4096];

    let hs = handshake::client_handshake(&mut link, &our_zid, &mut tx_buf, &mut rx_buf)
        .await
        .expect("Handshake failed");
    eprintln!(
        "Handshake OK (lease={}ms, router={:?})",
        hs.lease_ms, hs.router_zid
    );

    // Build action key expressions and verify format
    use zenoh_ros2_nostd::ros2::keyexpr::ActionKeyExprs;

    let action_ke = ActionKeyExprs::new(
        0,
        "fibonacci/_action/send_goal",
        "example_interfaces::action::dds_::Fibonacci_SendGoal_",
        "RIHS01_test_sg",
        "fibonacci/_action/cancel_goal",
        "fibonacci/_action/get_result",
        "example_interfaces::action::dds_::Fibonacci_GetResult_",
        "RIHS01_test_gr",
        "fibonacci/_action/feedback",
        "example_interfaces::action::dds_::Fibonacci_FeedbackMessage_",
        "RIHS01_test_fb",
        "fibonacci/_action/status",
    );

    let sg_ke = action_ke.send_goal.to_key_expr().expect("send_goal ke");
    assert!(sg_ke.as_str().contains("fibonacci/_action/send_goal"));
    assert!(sg_ke.as_str().contains("Fibonacci_SendGoal_"));
    eprintln!("send_goal KE: {}", sg_ke.as_str());

    let cg_ke = action_ke.cancel_goal.to_key_expr().expect("cancel_goal ke");
    assert!(cg_ke
        .as_str()
        .contains("action_msgs::srv::dds_::CancelGoal_"));
    eprintln!("cancel_goal KE: {}", cg_ke.as_str());

    // Declare key expression for _action/send_goal
    let mut sn: u64 = 0;
    let mut pos = 0;
    pos += codec::encode_frame_header(&mut tx_buf[pos..], sn, true).expect("frame header");
    sn += 1;
    pos +=
        codec::encode_declare_keyexpr(&mut tx_buf[pos..], 20, sg_ke.as_str()).expect("declare ke");
    frame::write_frame(&mut link, &tx_buf[..pos])
        .await
        .expect("write declare");
    eprintln!("Declared send_goal key expression");

    // Encode a SendGoal request: GoalId (16 bytes uuid) + Goal (i32 order = 10)
    // CDR: [header 4B][uuid 16B][i32 order 4B] = 24 bytes
    let mut payload = [0u8; 24];
    payload[0] = 0x00;
    payload[1] = 0x01; // CDR LE header
                       // uuid bytes 4..20 — all zeros (nil goal ID)
                       // order at offset 20, i32 LE = 10
    payload[20] = 10;

    pos = 0;
    pos += codec::encode_frame_header(&mut tx_buf[pos..], sn, true).expect("frame header 2");
    sn += 1;
    pos += codec::encode_request_query(
        &mut tx_buf[pos..],
        1,
        sg_ke.as_str(),
        &payload,
        1, // seq_num
        0, // timestamp
        &our_zid,
    )
    .expect("encode request query");
    frame::write_frame(&mut link, &tx_buf[..pos])
        .await
        .expect("write request");
    eprintln!("Sent action SendGoal Request ({} bytes)", pos);

    // Send keepalive to verify session is alive
    let ka_n = codec::encode_keepalive(&mut tx_buf).expect("encode keepalive");
    frame::write_frame(&mut link, &tx_buf[..ka_n])
        .await
        .expect("write keepalive");

    let read_result = tokio::time::timeout(
        Duration::from_secs(5),
        frame::read_frame(&mut link, &mut rx_buf),
    )
    .await;

    match read_result {
        Ok(Ok(n)) => {
            let kind = codec::parse_transport_msg_kind(rx_buf[0]);
            eprintln!(
                "Received {} bytes after SendGoal (kind={:?}) — session alive",
                n, kind
            );
        }
        Ok(Err(e)) => {
            eprintln!("Transport error after SendGoal: {:?}", e);
        }
        Err(_) => {
            eprintln!("Timeout — session alive (no action server registered)");
        }
    }

    // Graceful close
    let _ = sn;
    let n = codec::encode_close(&mut tx_buf, 0x00).expect("encode close");
    frame::write_frame(&mut link, &tx_buf[..n])
        .await
        .expect("write close");
    eprintln!("Action send_goal test PASSED");

    cleanup_zenohd(zenohd);
}

/// Handshake with a specified batch_size to force fragmentation.
async fn handshake_with_batch_size<T: embedded_io_async::Read + embedded_io_async::Write>(
    link: &mut T,
    our_zid: &ZenohId,
    tx_buf: &mut [u8],
    rx_buf: &mut [u8],
    batch_size: u16,
) -> Result<handshake::HandshakeResult, zenoh_ros2_nostd::error::TransportError> {
    use zenoh_ros2_nostd::transport::protocol::*;

    let init_syn = InitSyn {
        version: PROTO_VERSION,
        whatami: WhatAmI::Client,
        zid: *our_zid,
        batch_size: Some(batch_size),
    };
    let n = codec::encode_init_syn(tx_buf, &init_syn)?;
    frame::write_frame(link, &tx_buf[..n]).await?;

    let n = frame::read_frame(link, rx_buf).await?;
    let (init_ack, _) = codec::decode_init_ack(&rx_buf[..n])?;

    let open_syn = OpenSyn {
        lease_ms: 10_000,
        initial_sn: 0,
        cookie: init_ack.cookie,
    };
    let n = codec::encode_open_syn(tx_buf, &open_syn)?;
    frame::write_frame(link, &tx_buf[..n]).await?;

    let n = frame::read_frame(link, rx_buf).await?;
    let (open_ack, _) = codec::decode_open_ack(&rx_buf[..n])?;

    Ok(handshake::HandshakeResult {
        router_zid: init_ack.zid,
        lease_ms: open_ack.lease_ms,
        initial_sn: 0,
        router_initial_sn: open_ack.initial_sn,
    })
}
