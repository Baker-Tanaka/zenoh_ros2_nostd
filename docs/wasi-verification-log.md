# WASI Examples — Verification Log

Verified: 2026-04-13  
Environment: Ubuntu (GitHub Actions runner), Rust 1.94.1, Zenoh Router 1.3.4 (Docker)

---

## 1. Build Verification

### wasi_turtlebot3

```console
$ cd examples/wasi_turtlebot3

$ cargo build --target wasm32-wasip1 --features wasi
   Compiling wasi-turtlebot3 v0.1.0
    Finished `dev` profile [optimized + debuginfo]

$ cargo build
   Compiling wasi-turtlebot3 v0.1.0
    Finished `dev` profile [optimized + debuginfo]
```

### wasi_chatter_class

```console
$ cd examples/wasi_chatter_class

$ cargo build --target wasm32-wasip1 --features wasi
   Compiling wasi-chatter-class v0.1.0
    Finished `dev` profile [optimized + debuginfo]

$ cargo build
   Compiling wasi-chatter-class v0.1.0
    Finished `dev` profile [optimized + debuginfo]
```

### WASM Binary Sizes

| Binary | Size |
|--------|------|
| `wasi-turtlebot3.wasm` | 2.1 MB |
| `wasi-chatter-class.wasm` | 2.6 MB |

---

## 2. Topic Communication — wasi_turtlebot3

Zenoh Router (`eclipse/zenoh:1.3.4`) on `localhost:7447`.  
Subscriber: `verify_sub.py` (Python Zenoh client).

### Publisher Output

```
[chatter-pub] connecting to localhost:7447
[chatter-pub] connected (lease=10000ms)
[chatter-pub] declared: 0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
[chatter-pub] # 1 published: "Hello from zenoh-ros2-nostd! [0]"
[chatter-pub] # 2 published: "Hello from zenoh-ros2-nostd! [1]"
[chatter-pub] # 3 published: "Hello from zenoh-ros2-nostd! [2]"
[chatter-pub] # 4 published: "Hello from zenoh-ros2-nostd! [3]"
[chatter-pub] # 5 published: "Hello from zenoh-ros2-nostd! [4]"
[chatter-pub] # 6 published: "Hello from zenoh-ros2-nostd! [5]"
[chatter-pub] # 7 published: "Hello from zenoh-ros2-nostd! [6]"
[chatter-pub] # 8 published: "Hello from zenoh-ros2-nostd! [7]"
[chatter-pub] # 9 published: "Hello from zenoh-ros2-nostd! [8]"
[chatter-pub] #10 published: "Hello from zenoh-ros2-nostd! [9]"
[chatter-pub] #11 published: "Hello from zenoh-ros2-nostd! [10]"
[chatter-pub] #12 published: "Hello from zenoh-ros2-nostd! [11]"
[chatter-pub] #13 published: "Hello from zenoh-ros2-nostd! [12]"
[chatter-pub] #14 published: "Hello from zenoh-ros2-nostd! [13]"
[chatter-pub] #15 published: "Hello from zenoh-ros2-nostd! [14]"
[chatter-pub] #16 published: "Hello from zenoh-ros2-nostd! [15]"
[chatter-pub] #17 published: "Hello from zenoh-ros2-nostd! [16]"
[chatter-pub] #18 published: "Hello from zenoh-ros2-nostd! [17]"
[chatter-pub] #19 published: "Hello from zenoh-ros2-nostd! [18]"
[chatter-pub] #20 published: "Hello from zenoh-ros2-nostd! [19]"
[chatter-pub] done — published 20 messages to /chatter
```

### Subscriber Output (`verify_sub.py`)

```
[verify-sub] connected to localhost:7447 (ZID: 1a88f63f67608174890c37fbcaaae4b8)
[verify-sub] subscribed to /chatter — waiting 30s ...
  [  1] /chatter: "Hello from zenoh-ros2-nostd! [0]"
  [  2] /chatter: "Hello from zenoh-ros2-nostd! [1]"
  [  3] /chatter: "Hello from zenoh-ros2-nostd! [2]"
  [  4] /chatter: "Hello from zenoh-ros2-nostd! [3]"
  [  5] /chatter: "Hello from zenoh-ros2-nostd! [4]"
  [  6] /chatter: "Hello from zenoh-ros2-nostd! [5]"
  [  7] /chatter: "Hello from zenoh-ros2-nostd! [6]"
  [  8] /chatter: "Hello from zenoh-ros2-nostd! [7]"
  [  9] /chatter: "Hello from zenoh-ros2-nostd! [8]"
  [ 10] /chatter: "Hello from zenoh-ros2-nostd! [9]"
  [ 11] /chatter: "Hello from zenoh-ros2-nostd! [10]"
  [ 12] /chatter: "Hello from zenoh-ros2-nostd! [11]"
  [ 13] /chatter: "Hello from zenoh-ros2-nostd! [12]"
  [ 14] /chatter: "Hello from zenoh-ros2-nostd! [13]"
  [ 15] /chatter: "Hello from zenoh-ros2-nostd! [14]"
  [ 16] /chatter: "Hello from zenoh-ros2-nostd! [15]"
  [ 17] /chatter: "Hello from zenoh-ros2-nostd! [16]"
  [ 18] /chatter: "Hello from zenoh-ros2-nostd! [17]"
  [ 19] /chatter: "Hello from zenoh-ros2-nostd! [18]"
  [ 20] /chatter: "Hello from zenoh-ros2-nostd! [19]"

✅ Received 20 message(s) from /chatter
```

**Result: ✅ PASS** — 20/20 messages received correctly.

---

## 3. Topic Communication — wasi_chatter_class

Same Zenoh Router setup. Subscriber: `verify_sub.py`.

### Publisher Output (NodeCallbacks trait pattern)

```
[chatter-class] connecting to localhost:7447
[chatter-class] connected (lease=10000ms)
[chatter-class] publisher registered for /chatter
[chatter-class] subscribed to /chatter
[chatter-class] timer registered (500ms)
[chatter-class] spinning...
[chatter-class] tx #1
[chatter-class] tx #2
[chatter-class] tx #3
[chatter-class] tx #4
[chatter-class] tx #5
[chatter-class] tx #6
[chatter-class] tx #7
[chatter-class] tx #8
[chatter-class] tx #9
[chatter-class] tx #10
[chatter-class] tx #11
[chatter-class] tx #12
[chatter-class] tx #13
[chatter-class] tx #14
[chatter-class] tx #15
[chatter-class] tx #16
[chatter-class] tx #17
[chatter-class] tx #18
[chatter-class] tx #19
[chatter-class] tx #20
```

### Subscriber Output (`verify_sub.py`)

```
[verify-sub] connected to localhost:7447 (ZID: 5f6d9da103262acdd7aba5a7c0d874be)
[verify-sub] subscribed to /chatter — waiting 30s ...
  [  1] /chatter: "Hello from ChatterNode! [1]"
  [  2] /chatter: "Hello from ChatterNode! [2]"
  [  3] /chatter: "Hello from ChatterNode! [3]"
  [  4] /chatter: "Hello from ChatterNode! [4]"
  [  5] /chatter: "Hello from ChatterNode! [5]"
  [  6] /chatter: "Hello from ChatterNode! [6]"
  [  7] /chatter: "Hello from ChatterNode! [7]"
  [  8] /chatter: "Hello from ChatterNode! [8]"
  [  9] /chatter: "Hello from ChatterNode! [9]"
  [ 10] /chatter: "Hello from ChatterNode! [10]"
  [ 11] /chatter: "Hello from ChatterNode! [11]"
  [ 12] /chatter: "Hello from ChatterNode! [12]"
  [ 13] /chatter: "Hello from ChatterNode! [13]"
  [ 14] /chatter: "Hello from ChatterNode! [14]"
  [ 15] /chatter: "Hello from ChatterNode! [15]"
  [ 16] /chatter: "Hello from ChatterNode! [16]"
  [ 17] /chatter: "Hello from ChatterNode! [17]"
  [ 18] /chatter: "Hello from ChatterNode! [18]"
  [ 19] /chatter: "Hello from ChatterNode! [19]"
  [ 20] /chatter: "Hello from ChatterNode! [20]"

✅ Received 20 message(s) from /chatter
```

**Result: ✅ PASS** — 20/20 messages received correctly.

---

## 4. Unit Tests & Integration Tests

```console
$ cargo test --no-default-features --features log
running 109 tests
...
test result: ok. 109 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out

$ ZENOH_ROUTER_ADDR=localhost:7447 cargo test --test integration_test --no-default-features --features log -- --ignored
running 12 tests
test test_action_send_goal_request ... ok
test test_close_session ... ok
test test_declare_keyexpr ... ok
test test_declare_liveliness_token ... ok
test test_fragment_reassembly_large_message ... ok
test test_handshake_with_zenohd ... ok
test test_keepalive_after_handshake ... ok
test test_multi_topic_pub_sub ... ok
test test_publish_chatter_topic ... ok
test test_publish_with_rmw_attachment ... ok
test test_service_request_send ... ok
test test_subscribe_receive_loopback ... ok

test result: ok. 12 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

**Result: ✅ PASS** — 109 unit tests + 12 integration tests all passed.

---

## 5. Issues Found & Fixed

| # | Issue | Fix |
|---|-------|-----|
| 1 | `wasi_turtlebot3/Cargo.toml` missing `[workspace]` table — cargo fails with workspace detection error | Added `[workspace]` table |
| 2 | `src/wasi/time.rs` — dead code (unused `monotonic_ns()`, `CLOCKID_MONOTONIC`, `clock_time_get`) causing 3 warnings on WASM builds | Removed unused `time` module |
| 3 | `src/cdr/mod.rs` — unused `StdString` struct in test module causing dead-code warning on host builds | Removed unused struct |

---

## Summary

| Item | Status |
|------|--------|
| WASM build (`wasm32-wasip1`) | ✅ |
| Native build (x86_64) | ✅ |
| wasi_turtlebot3 topic communication | ✅ 20/20 |
| wasi_chatter_class topic communication | ✅ 20/20 |
| Unit tests (109) | ✅ |
| Integration tests (12) | ✅ |
| Compiler warnings | ✅ Zero |
