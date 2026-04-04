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
    use std::net::ToSocketAddrs;
    // Check if zenohd is already running on the configured address.
    // Uses ToSocketAddrs so hostname-based addresses (e.g. "zenoh-router:7447") are resolved.
    let reachable = router_addr()
        .to_socket_addrs()
        .ok()
        .and_then(|mut it| it.next())
        .map(|addr| std::net::TcpStream::connect_timeout(&addr, Duration::from_millis(500)).is_ok())
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

/// Decode a CDR LE std_msgs/String from `buf`. Returns the string content or None on error.
fn decode_cdr_string(buf: &[u8]) -> Option<String> {
    if buf.len() < 8 {
        return None;
    }
    // Skip 4-byte encapsulation header
    let str_len = u32::from_le_bytes(buf[4..8].try_into().ok()?) as usize;
    if str_len == 0 || buf.len() < 8 + str_len {
        return None;
    }
    // str_len includes null terminator — take str_len - 1 bytes
    let s = std::str::from_utf8(&buf[8..8 + str_len - 1]).ok()?;
    Some(s.to_string())
}

// ====== Test 6: Subscribe to /chatter and receive from ROS2 talker ======

/// Test 6: Subscribe to /chatter and receive a CDR-encoded message.
///
/// Uses two zenoh sessions:
/// - subscriber session: declares subscriber on the chatter key expression
/// - publisher session: publishes a CDR-encoded String message
/// The zenoh router forwards the published message to the subscriber.
///
/// This test proves end-to-end pub/sub via zenoh without requiring a live ROS2 node.
#[tokio::test]
#[ignore]
async fn test_subscribe_chatter_topic() {
    let zenohd = ensure_zenohd();

    const KEY_EXPR: &str = "0/chatter/std_msgs::msg::dds_::String_/\
        RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
    const TEST_MSG: &str = "Hello from test_subscribe_chatter_topic";

    let result = async {
        // ── Subscriber session ──────────────────────────────────────────────
        let sub_tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut sub_link = FromTokio::new(sub_tcp);
        let mut tx_buf = [0u8; 512];
        let mut rx_buf = [0u8; 8192];

        handshake::client_handshake(
            &mut sub_link,
            &ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x60]),
            &mut tx_buf,
            &mut rx_buf,
        )
        .await
        .expect("Subscriber handshake failed");

        // Declare key expression (ID 1)
        let key_id: u16 = 1;
        let mut pos = 0;
        pos += codec::encode_frame_header(&mut tx_buf[pos..], 0, true).unwrap();
        pos += codec::encode_declare_keyexpr(&mut tx_buf[pos..], key_id, KEY_EXPR).unwrap();
        frame::write_frame(&mut sub_link, &tx_buf[..pos])
            .await
            .unwrap();

        // Declare subscriber
        pos = 0;
        pos += codec::encode_frame_header(&mut tx_buf[pos..], 1, true).unwrap();
        pos += codec::encode_declare_subscriber_mapped(&mut tx_buf[pos..], key_id as u32, key_id)
            .unwrap();
        frame::write_frame(&mut sub_link, &tx_buf[..pos])
            .await
            .unwrap();
        eprintln!("Subscriber declared on '{}'", KEY_EXPR);

        // Small delay so the router can propagate the subscription before we publish
        tokio::time::sleep(Duration::from_millis(200)).await;

        // ── Publisher session ───────────────────────────────────────────────
        let pub_tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let mut pub_link = FromTokio::new(pub_tcp);
        let mut pub_tx = [0u8; 1024];
        let mut pub_rx = [0u8; 4096];

        handshake::client_handshake(
            &mut pub_link,
            &ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x61]),
            &mut pub_tx,
            &mut pub_rx,
        )
        .await
        .expect("Publisher handshake failed");

        // Publish CDR-encoded String
        let mut cdr_buf = [0u8; 256];
        let cdr_len = build_cdr_string(&mut cdr_buf, TEST_MSG);

        pos = 0;
        pos += codec::encode_frame_header(&mut pub_tx[pos..], 0, true).unwrap();
        pos +=
            codec::encode_push_put(&mut pub_tx[pos..], KEY_EXPR, 0, &cdr_buf[..cdr_len]).unwrap();
        frame::write_frame(&mut pub_link, &pub_tx[..pos])
            .await
            .unwrap();
        eprintln!("Published: \"{}\" ({} CDR bytes)", TEST_MSG, cdr_len);

        // ── Receive on subscriber session ───────────────────────────────────
        let deadline = tokio::time::Instant::now() + Duration::from_secs(5);
        let mut received: Option<Vec<u8>> = None;

        while tokio::time::Instant::now() < deadline && received.is_none() {
            let read_result = tokio::time::timeout(
                Duration::from_secs(1),
                frame::read_frame(&mut sub_link, &mut rx_buf),
            )
            .await;

            match read_result {
                Ok(Ok(n)) => {
                    let msg_buf = &rx_buf[..n];
                    match codec::parse_transport_msg_kind(msg_buf[0]) {
                        codec::TransportMsgKind::Frame => {
                            if let Ok((_, _, body_pos)) = codec::decode_frame_header(msg_buf) {
                                let body = &msg_buf[body_pos..];
                                if let Ok(Some((put, _))) = codec::decode_push_put(body) {
                                    eprintln!(
                                        "Received Push/Put: scope={} suffix='{}' payload_len={}",
                                        put.scope,
                                        put.key_suffix,
                                        put.payload.len()
                                    );
                                    // Filter: only accept the message we published (other tests
                                    // may publish to the same topic concurrently)
                                    if let Some(s) = decode_cdr_string(put.payload) {
                                        if s == TEST_MSG {
                                            received = Some(put.payload.to_vec());
                                        }
                                    }
                                }
                            }
                        }
                        codec::TransportMsgKind::KeepAlive => {}
                        other => eprintln!("Other msg: {:?}", other),
                    }
                }
                Ok(Err(e)) => panic!("Read error: {:?}", e),
                Err(_) => eprintln!("Read timeout, retrying..."),
            }
        }

        let payload = received.expect("No Push/Put received within 5 seconds");

        // Decode CDR
        let decoded = decode_cdr_string(&payload).expect("CDR decode failed");
        eprintln!("Decoded message: \"{}\"", decoded);
        assert_eq!(decoded, TEST_MSG, "Message content mismatch");

        eprintln!("Subscribe test PASSED — E2E pub/sub via zenoh router verified");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}

// ====== Test 7: Full ROS2 pub/sub using Session API ======

/// Test 7: Use the high-level Session API for pub/sub.
///
/// Publishes a message using `session.put()` and then subscribes
/// via a second session to receive it back, verifying the Session API.
#[tokio::test]
#[ignore]
async fn test_session_pubsub() {
    use zenoh_ros2_nostd::session::{Session, SessionConfig};

    let zenohd = ensure_zenohd();

    const KEY_EXPR: &str = "0/session_api_test/std_msgs::msg::dds_::String_/\
        RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
    const TEST_MSG: &str = "Hello from Session API!";

    let result = async {
        // ── Subscriber session (Session API) ────────────────────────────────
        let sub_tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let sub_link = FromTokio::new(sub_tcp);
        let sub_config = SessionConfig::new(ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x70]));
        let sub_session = Session::<_, 512, 8192>::open(sub_link, sub_config)
            .await
            .expect("Subscriber Session::open failed");

        sub_session
            .subscribe(KEY_EXPR)
            .await
            .expect("subscribe failed");
        eprintln!("Subscriber Session: subscribe OK");

        tokio::time::sleep(Duration::from_millis(200)).await;

        // ── Publisher session (Session API) ─────────────────────────────────
        let pub_tcp = TcpStream::connect(router_addr().as_str()).await.unwrap();
        let pub_link = FromTokio::new(pub_tcp);
        let pub_config = SessionConfig::new(ZenohId::from_bytes(&[0xCA, 0xFE, 0x00, 0x71]));
        let pub_session = Session::<_, 512, 8192>::open(pub_link, pub_config)
            .await
            .expect("Publisher Session::open failed");

        let mut cdr_buf = [0u8; 64];
        let cdr_len = build_cdr_string(&mut cdr_buf, TEST_MSG);
        pub_session
            .put(KEY_EXPR, &cdr_buf[..cdr_len])
            .await
            .expect("session.put failed");
        eprintln!("Publisher Session: put OK");

        // ── Receive on subscriber session ───────────────────────────────────
        let mut rx_buf = [0u8; 8192];
        let mut received = false;
        for attempt in 0..5 {
            match tokio::time::timeout(Duration::from_secs(1), sub_session.recv_once(&mut rx_buf))
                .await
            {
                Ok(Ok(Some((key, payload)))) => {
                    eprintln!("recv_once: key='{}' payload_len={}", key, payload.len());
                    if let Some(s) = decode_cdr_string(payload) {
                        eprintln!("  decoded: \"{}\"", s);
                        if s == TEST_MSG {
                            received = true;
                            break;
                        }
                        // Different message (e.g. from another concurrent test) — keep waiting
                    }
                }
                Ok(Ok(None)) => eprintln!("attempt {}: non-data frame", attempt),
                Ok(Err(e)) => panic!("recv_once error: {:?}", e),
                Err(_) => eprintln!("attempt {}: timeout", attempt),
            }
        }

        pub_session.close().await.expect("pub session.close failed");
        sub_session.close().await.expect("sub session.close failed");

        assert!(received, "No message received via Session API");
        eprintln!("Session pub/sub test PASSED");
    }
    .await;

    cleanup_zenohd(zenohd);
    result
}
