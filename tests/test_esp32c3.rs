//! Embedded integration tests for Seeed Studio XIAO ESP32C3.
//!
//! Uses [embedded-test] to run tests directly on the hardware via probe-rs
//! over the built-in USB JTAG interface.
//!
//! # Prerequisites
//!
//! ```sh
//! # Install probe-rs (flash + run tool)
//! cargo install probe-rs-tools
//!
//! # Add the ESP32-C3 RISC-V target to your Rust toolchain
//! rustup target add riscv32imc-unknown-none-elf
//!
//! # On Linux, add yourself to the 'dialout' group for USB access
//! sudo usermod -aG dialout $USER
//! ```
//!
//! # Running on XIAO ESP32C3
//!
//! Connect the XIAO ESP32C3 via USB.  probe-rs uses the built-in JTAG, so
//! no external debugger is required.
//!
//! ```sh
//! cargo test --test test_esp32c3 \
//!       --target riscv32imc-unknown-none-elf \
//!       --no-default-features --features defmt
//! ```
//!
//! Test results are streamed over RTT (Real-Time Transfer) and printed to the
//! terminal by probe-rs.
//!
//! # Non-embedded targets
//!
//! When compiled for the host (e.g. during `cargo test`), this binary prints
//! a usage hint and exits immediately.  All actual assertions only execute on
//! the embedded target.

// ── Attributes ─────────────────────────────────────────────────────────────

// Bare-metal embedded target: opt out of std and the standard entry point.
// (cfg_attr is used so that host builds remain ordinary std binaries.)
#![cfg_attr(all(target_arch = "riscv32", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "riscv32", target_os = "none"), no_main)]

// ── Embedded-target-only imports ───────────────────────────────────────────

/// Forward defmt log frames via RTT so probe-rs can display them.
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
use defmt_rtt as _;

/// Report panics as defmt messages and halt; probe-rs marks the run failed.
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
use panic_probe as _;

// ── Host stub ──────────────────────────────────────────────────────────────

/// On non-embedded hosts the binary does nothing — tests only run on hardware.
///
/// `harness = false` means Cargo calls this `main()` directly when you run
/// `cargo test` on the host.
#[cfg(not(all(target_arch = "riscv32", target_os = "none")))]
fn main() {
    eprintln!(
        "XIAO ESP32C3 embedded tests — run on hardware with:\n  \
         cargo test --test test_esp32c3 \\\n        \
               --target riscv32imc-unknown-none-elf \\\n        \
               --no-default-features --features defmt"
    );
}

// ── Embedded test suite ────────────────────────────────────────────────────

/// Tests that run on `riscv32imc-unknown-none-elf` (ESP32-C3) via embedded-test.
///
/// Each `#[test]` function is independent; embedded-test prints a pass/fail
/// result for each one via defmt/RTT.
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
#[embedded_test::tests]
mod tests {
    use zenoh_ros2_nostd::buf::pool::BufferPool;
    use zenoh_ros2_nostd::cdr::{
        deserialize_from_buf, deserialize_with_header, serialize_to_buf, serialize_with_header,
        CDR_LE_HEADER,
    };
    use zenoh_ros2_nostd::ros2::keyexpr::TopicKeyExpr;
    use zenoh_ros2_nostd::transport::codec::{decode_vbyte, encode_vbyte};

    /// Initialize the ESP32-C3 clocks and peripherals before the test suite runs.
    ///
    /// Calling `esp_hal::init` is required before accessing any peripheral;
    /// for pure-logic tests we still call it to put the chip in a well-defined
    /// state (correct clock speed, etc.).
    #[init]
    fn init() {
        let _ = esp_hal::init(esp_hal::Config::default());
    }

    // ── CDR serialization ──────────────────────────────────────────────────

    /// Serialize a `u8` and round-trip it back.
    #[test]
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
    #[test]
    fn cdr_roundtrip_u32_little_endian() {
        let mut buf = [0u8; 16];
        let n = serialize_to_buf(&mut buf, &0x12345678u32).unwrap();
        assert_eq!(n, 4);
        assert_eq!(&buf[..4], &[0x78, 0x56, 0x34, 0x12]); // LE
        let (v, _): (u32, _) = deserialize_from_buf(&buf[..n]).unwrap();
        assert_eq!(v, 0x12345678u32);
    }

    /// Serialize a `bool` value.
    #[test]
    fn cdr_roundtrip_bool() {
        let mut buf = [0u8; 4];
        let n = serialize_to_buf(&mut buf, &true).unwrap();
        assert_eq!(n, 1);
        assert_eq!(buf[0], 1);
        let (v, _): (bool, _) = deserialize_from_buf(&buf[..n]).unwrap();
        assert!(v);
    }

    /// Verify the 4-byte CDR LE encapsulation header is written correctly.
    #[test]
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
    #[test]
    fn cdr_header_rejects_invalid_encapsulation() {
        let bad = [0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00];
        let result: Result<(u32, _), _> = deserialize_with_header(&bad);
        assert!(result.is_err());
    }

    /// `u64` round-trip: tests multi-word primitives.
    #[test]
    fn cdr_roundtrip_u64() {
        let val: u64 = 0xDEAD_BEEF_CAFE_1234;
        let mut buf = [0u8; 16];
        let n = serialize_to_buf(&mut buf, &val).unwrap();
        assert_eq!(n, 8);
        let (v, _): (u64, _) = deserialize_from_buf(&buf[..n]).unwrap();
        assert_eq!(v, val);
    }

    // ── Buffer pool ────────────────────────────────────────────────────────

    /// Acquire and release a single buffer from a pool.
    #[test]
    fn buf_pool_acquire_and_release() {
        let mut pool: BufferPool<128, 4> = BufferPool::new();
        assert_eq!(pool.available(), 4);
        let b = pool.acquire().unwrap();
        assert_eq!(pool.available(), 3);
        pool.release(b).unwrap();
        assert_eq!(pool.available(), 4);
    }

    /// Exhaust all buffers and verify `acquire` returns `None`.
    #[test]
    fn buf_pool_exhaustion_returns_none() {
        let mut pool: BufferPool<64, 2> = BufferPool::new();
        let b0 = pool.acquire().unwrap();
        let b1 = pool.acquire().unwrap();
        assert!(pool.acquire().is_none());
        pool.release(b0).unwrap();
        pool.release(b1).unwrap();
        assert_eq!(pool.available(), 2);
    }

    // ── VByte codec ────────────────────────────────────────────────────────

    /// Single-byte VByte encoding (values < 128).
    #[test]
    fn vbyte_single_byte_encoding() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 42).unwrap();
        assert_eq!(n, 1);
        let (v, consumed) = decode_vbyte(&buf[..n]).unwrap();
        assert_eq!(v, 42);
        assert_eq!(consumed, 1);
    }

    /// Multi-byte VByte encoding (values >= 128).
    #[test]
    fn vbyte_multibyte_encoding() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 300).unwrap();
        assert_eq!(n, 2);
        let (v, _) = decode_vbyte(&buf[..n]).unwrap();
        assert_eq!(v, 300);
    }

    /// Large value VByte round-trip.
    #[test]
    fn vbyte_large_value_roundtrip() {
        let mut buf = [0u8; 16];
        let n = encode_vbyte(&mut buf, 1_000_000).unwrap();
        let (v, consumed) = decode_vbyte(&buf[..n]).unwrap();
        assert_eq!(v, 1_000_000);
        assert_eq!(consumed, n);
    }

    // ── ROS2 key expressions ───────────────────────────────────────────────

    /// Basic topic key expression with default domain ID.
    #[test]
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
    #[test]
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
    #[test]
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
}
