//! WebAssembly integration tests for zenoh-ros2-nostd.
//!
//! These tests verify that the library's pure-logic components work correctly
//! inside a `wasm32-unknown-unknown` environment, validating no_std
//! compatibility without requiring embedded hardware.
//!
//! # How it works
//!
//! [`wasm-bindgen-test`] is used as the test harness:
//! - On **`wasm32-unknown-unknown`** the tests run inside Node.js (see
//!   [`wasm_bindgen_test_configure!`] below) or a headless browser, driven by
//!   `wasm-bindgen-test-runner`.
//! - On the **host** (`cargo test` without `--target`), `#[wasm_bindgen_test]`
//!   automatically becomes `#[test]`, so the same test functions also run as
//!   ordinary unit tests.  This is useful during development.
//!
//! # Prerequisites (WASM-only execution)
//!
//! ```sh
//! # Install the wasm-bindgen CLI (provides wasm-bindgen-test-runner)
//! cargo install wasm-bindgen-cli
//!
//! # Install Node.js (used as the WASM runtime by the runner)
//! # https://nodejs.org
//!
//! # Add the WASM target to your Rust toolchain
//! rustup target add wasm32-unknown-unknown
//! ```
//!
//! # Running WASM tests
//!
//! ```sh
//! # Disable defmt (embedded logging) and enable no-logging for WASM
//! cargo test --test test_wasm \
//!       --target wasm32-unknown-unknown \
//!       --no-default-features
//! ```
//!
//! The `.cargo/config.toml` in this repository configures
//! `wasm-bindgen-test-runner` as the test runner for `wasm32-unknown-unknown`,
//! so the above command handles everything automatically.
//!
//! # Running on the host
//!
//! ```sh
//! # Runs as ordinary #[test] functions — no WASM tooling required
//! cargo test --test test_wasm --no-default-features
//! ```

use wasm_bindgen_test::*;

// Configure the WASM test runner to use Node.js.
// This is a no-op when compiling for non-wasm32 targets.
wasm_bindgen_test_configure!(run_in_node_experimental);

use zenoh_ros2_nostd::buf::pool::BufferPool;
use zenoh_ros2_nostd::cdr::{
    deserialize_from_buf, deserialize_with_header, serialize_to_buf, serialize_with_header,
    CDR_LE_HEADER,
};
use zenoh_ros2_nostd::ros2::keyexpr::TopicKeyExpr;
use zenoh_ros2_nostd::transport::codec::{decode_vbyte, encode_vbyte};

// ── CDR serialization ──────────────────────────────────────────────────────

/// Serialize a `u8` and round-trip it back.
#[wasm_bindgen_test]
fn cdr_roundtrip_u8() {
    let mut buf = [0u8; 16];
    let n = serialize_to_buf(&mut buf, &0x42u8).unwrap();
    assert_eq!(n, 1);
    assert_eq!(buf[0], 0x42);
    let (v, consumed): (u8, _) = deserialize_from_buf(&buf[..n]).unwrap();
    assert_eq!(consumed, 1);
    assert_eq!(v, 0x42);
}

/// Serialize a `u32` and verify little-endian byte order.
#[wasm_bindgen_test]
fn cdr_roundtrip_u32_little_endian() {
    let mut buf = [0u8; 16];
    let n = serialize_to_buf(&mut buf, &0x12345678u32).unwrap();
    assert_eq!(n, 4);
    assert_eq!(&buf[..4], &[0x78, 0x56, 0x34, 0x12]); // LE
    let (v, _): (u32, _) = deserialize_from_buf(&buf[..n]).unwrap();
    assert_eq!(v, 0x12345678u32);
}

/// Serialize a `bool` value.
#[wasm_bindgen_test]
fn cdr_roundtrip_bool() {
    let mut buf = [0u8; 4];
    let n = serialize_to_buf(&mut buf, &true).unwrap();
    assert_eq!(n, 1);
    assert_eq!(buf[0], 1);
    let (v, _): (bool, _) = deserialize_from_buf(&buf[..n]).unwrap();
    assert!(v);
}

/// Verify the 4-byte CDR LE encapsulation header is written correctly.
#[wasm_bindgen_test]
fn cdr_encapsulation_header() {
    let mut buf = [0u8; 32];
    let n = serialize_with_header(&mut buf, &42u32).unwrap();
    // First 4 bytes must be the CDR LE header
    assert_eq!(&buf[..4], &CDR_LE_HEADER);
    // Round-trip via deserialize_with_header
    let (v, consumed): (u32, _) = deserialize_with_header(&buf[..n]).unwrap();
    assert_eq!(consumed, n);
    assert_eq!(v, 42u32);
}

/// Reject a buffer whose encapsulation tag is wrong.
#[wasm_bindgen_test]
fn cdr_header_rejects_invalid_encapsulation() {
    let bad = [0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00];
    let result: Result<(u32, _), _> = deserialize_with_header(&bad);
    assert!(result.is_err());
}

/// `u64` round-trip: tests multi-word primitives.
#[wasm_bindgen_test]
fn cdr_roundtrip_u64() {
    let val: u64 = 0xDEAD_BEEF_CAFE_1234;
    let mut buf = [0u8; 16];
    let n = serialize_to_buf(&mut buf, &val).unwrap();
    assert_eq!(n, 8);
    let (v, _): (u64, _) = deserialize_from_buf(&buf[..n]).unwrap();
    assert_eq!(v, val);
}

// ── Buffer pool ────────────────────────────────────────────────────────────

/// Acquire and release a single buffer from a pool.
#[wasm_bindgen_test]
fn buf_pool_acquire_and_release() {
    let mut pool: BufferPool<128, 4> = BufferPool::new();
    assert_eq!(pool.available(), 4);
    let b = pool.acquire().unwrap();
    assert_eq!(pool.available(), 3);
    pool.release(b).unwrap();
    assert_eq!(pool.available(), 4);
}

/// Exhaust all buffers and verify `acquire` returns `None`.
#[wasm_bindgen_test]
fn buf_pool_exhaustion_returns_none() {
    let mut pool: BufferPool<64, 2> = BufferPool::new();
    let b0 = pool.acquire().unwrap();
    let b1 = pool.acquire().unwrap();
    assert!(pool.acquire().is_none());
    pool.release(b0).unwrap();
    pool.release(b1).unwrap();
    assert_eq!(pool.available(), 2);
}

// ── VByte codec ────────────────────────────────────────────────────────────

/// Single-byte VByte encoding (values < 128).
#[wasm_bindgen_test]
fn vbyte_single_byte_encoding() {
    let mut buf = [0u8; 16];
    let n = encode_vbyte(&mut buf, 42).unwrap();
    assert_eq!(n, 1);
    let (v, consumed) = decode_vbyte(&buf[..n]).unwrap();
    assert_eq!(v, 42);
    assert_eq!(consumed, 1);
}

/// Multi-byte VByte encoding (values >= 128).
#[wasm_bindgen_test]
fn vbyte_multibyte_encoding() {
    let mut buf = [0u8; 16];
    let n = encode_vbyte(&mut buf, 300).unwrap();
    assert_eq!(n, 2);
    let (v, _) = decode_vbyte(&buf[..n]).unwrap();
    assert_eq!(v, 300);
}

/// Large value VByte round-trip.
#[wasm_bindgen_test]
fn vbyte_large_value_roundtrip() {
    let mut buf = [0u8; 16];
    let n = encode_vbyte(&mut buf, 1_000_000).unwrap();
    let (v, consumed) = decode_vbyte(&buf[..n]).unwrap();
    assert_eq!(v, 1_000_000);
    assert_eq!(consumed, n);
}

// ── ROS2 key expressions ───────────────────────────────────────────────────

/// Basic topic key expression with default domain ID.
#[wasm_bindgen_test]
fn keyexpr_basic_topic() {
    let ke = TopicKeyExpr::new(
        0,
        "chatter",
        "std_msgs::msg::String",
        "RIHS01_abc123",
    );
    let s = ke.to_key_expr().unwrap();
    assert_eq!(s.as_str(), "0/chatter/std_msgs::msg::String/RIHS01_abc123");
}

/// Leading slash on topic name is stripped.
#[wasm_bindgen_test]
fn keyexpr_strips_leading_slash() {
    let ke = TopicKeyExpr::new(
        0,
        "/cmd_vel",
        "geometry_msgs::msg::Twist",
        "RIHS01_xyz",
    );
    let s = ke.to_key_expr().unwrap();
    assert_eq!(s.as_str(), "0/cmd_vel/geometry_msgs::msg::Twist/RIHS01_xyz");
}

/// Non-zero domain ID is included in the key expression.
#[wasm_bindgen_test]
fn keyexpr_nonzero_domain_id() {
    let ke = TopicKeyExpr::new(
        42,
        "scan",
        "sensor_msgs::msg::LaserScan",
        "RIHS01_hash",
    );
    let s = ke.to_key_expr().unwrap();
    assert_eq!(s.as_str(), "42/scan/sensor_msgs::msg::LaserScan/RIHS01_hash");
}
