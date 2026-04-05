---
description: "Use when writing or editing code for ESP32-C3 targets using esp-hal, esp-rtos, esp-radio, embassy. Covers initialization order, WiFi usage, RTT logging, Cargo configuration, and common pitfalls."
applyTo: "examples/esp32c3_wifi/**"
---
# ESP32-C3 Rust Embedded Guidelines

## Crate Versions (verified working together)

```toml
embassy-executor = "0.9"         # Do NOT use arch-* features — esp-rtos provides the executor
embassy-time     = "0.5"
esp-alloc        = "0.9"
esp-bootloader-esp-idf = { version = "0.4", features = ["esp32c3"] }
esp-hal          = { version = "1.0", features = ["esp32c3", "unstable"] }
esp-rtos         = { version = "0.2", features = ["esp32c3", "esp-radio", "embassy"] }
esp-radio        = { version = "0.17", features = ["esp32c3", "wifi"] }
rtt-target       = "0.6"
```

## Required top-level declarations

Every binary must have these in this order:

```rust
#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();   // REQUIRED: embeds app descriptor for probe-rs / bootloader

extern crate alloc;  // only if heap (alloc::string::String etc.) is used
```

`esp_app_desc!()` is **mandatory**. Without it, probe-rs refuses to flash the binary
("Failed to format as esp-idf binary").

## Initialization Order (MUST follow this sequence)

```rust
#[esp_rtos::main]
async fn main(_spawner: Spawner) -> ! {
    // 1. RTT first — so the panic handler can print
    let channels = rtt_init! { up: { 0: { size: 2048, name: "Terminal" } } };
    rtt_target::set_print_channel(channels.up.0);

    // 2. HAL
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // 3. Heap — MUST come before esp_radio::init()
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // 4. esp-rtos scheduler — MUST come before any await or esp-radio call
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // 5. esp-radio (after heap + rtos)
    let radio = esp_radio::init().expect("radio init");
    // ...
}
```

Breaking this order causes hard-to-debug panics or silent hangs.

## Embassy Executor

- Mark main with `#[esp_rtos::main]` — this creates a thread-mode Embassy executor
- **Do NOT** add `arch-xtensa`, `arch-riscv32`, or any other `arch-*` feature to `embassy-executor`
- To spawn additional tasks, use `#[embassy_executor::task]` and the `Spawner` passed to main

```rust
#[embassy_executor::task]
async fn my_task() { ... }

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    spawner.spawn(my_task()).ok();
    ...
}
```

## WiFi (WifiController API)

### Initialization

```rust
let radio = esp_radio::init().expect("Failed to init esp-radio");
let (mut wifi, _interfaces) = esp_radio::wifi::new(
    &radio,
    peripherals.WIFI,
    Config::default(),
).expect("WiFi init");

let client_config = ClientConfig::default()
    .with_ssid(String::from(ssid))      // max 32 chars
    .with_password(String::from(pass)); // max 64 chars; empty = open network
wifi.set_config(&ModeConfig::Client(client_config)).expect("set config");
wifi.start_async().await.expect("WiFi start");
```

### Connection loop — ALWAYS use with_timeout

`connect_async()` waits for `StaConnected` OR `StaDisconnected` events from the firmware.
If neither event fires (AP out of range, wrong credentials sometimes), it **hangs forever**.
Always wrap with a timeout:

```rust
use embassy_time::with_timeout;

loop {
    match with_timeout(Duration::from_secs(15), wifi.connect_async()).await {
        Ok(Ok(())) => {
            // connected
            wifi.wait_for_event(WifiEvent::StaDisconnected).await;
        }
        Ok(Err(e)) => { /* auth failure, disconnected error */ }
        Err(_)     => { /* timeout — AP unreachable or credentials wrong */ }
    }
    Timer::after(Duration::from_secs(5)).await; // backoff before retry
}
```

### ClientConfig fields of note

| Field | Default | Note |
|---|---|---|
| `auth_method` | `Wpa2Personal` | Change for WPA3 or open networks |
| `channel` | `None` | Set to avoid roaming |
| `scan_method` | `Fast` | Use `AllChannels` with `failure_retry_cnt` |
| `beacon_timeout` | 6 | Must be 6–31 |

### Interfaces struct

`wifi::new()` returns `(WifiController, Interfaces)`.
`Interfaces` contains `sta: WifiDevice` and `ap: WifiDevice` which implement
`embassy_net_driver::Driver` for use with `embassy-net`.
When not using embassy-net, assign to `_interfaces` to drop it.

## RTT Logging

```rust
use rtt_target::{rprintln, rprint, rtt_init};

let channels = rtt_init! {
    up: { 0: { size: 2048, name: "Terminal" } }
    // Add down channel only if reading input from host:
    // down: { 0: { size: 256, name: "Terminal" } }
};
rtt_target::set_print_channel(channels.up.0);
```

- `rprintln!` / `rprint!` for output
- `channels.down.0.read(&mut buf)` for non-blocking input from host
- RTT buffer size 2048 is sufficient for debug output; increase if lines are dropped

## Panic Handler

```rust
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    rprintln!("\n\n!!! PANIC !!!\n{}\n", info);
    loop { core::hint::spin_loop(); }
}
```

Must be defined exactly once per binary. RTT must already be initialized for the message to appear.

## Credentials at Compile Time (build.rs pattern)

Sensitive values (SSID, password) are embedded via `build.rs` → `env!()`.
Never hard-code them in source files.

```rust
// main.rs
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
```

```rust
// build.rs — parses wifi_config.json and emits env vars
println!("cargo:rerun-if-changed=wifi_config.json");
println!("cargo:rustc-env=WIFI_SSID={ssid}");
println!("cargo:rustc-env=WIFI_PASSWORD={password}");
```

```json
// wifi_config.json (git-ignored)
{ "ssid": "MyNetwork", "password": "secret" }
```

## Build Profiles

```toml
[profile.dev]
opt-level = 2       # REQUIRED: WiFi firmware is unstable at opt-level 0/1

[profile.release]
lto = "fat"
opt-level = 3
codegen-units = 1
debug = true        # Keep debug info for probe-rs stack traces
```

## Common Pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| `Failed to format as esp-idf binary` | `esp_app_desc!()` missing | Add macro after `#![no_main]` |
| Hang at `connect_async()` | No WiFi event fires (AP OOR or firmware issue) | Wrap with `with_timeout()` |
| Panic at `esp_radio::init()` | Heap not initialized | Call `heap_allocator!()` before `init()` |
| Panic at first `await` | `esp_rtos::start()` not called | Call it before any `.await` |
| WiFi stack crashes at low opt-level | Undefined behavior in C WiFi code | Use `opt-level = 2` minimum in dev profile |
| RTT output gibberish | Clock not configured | Ensure `esp_hal::init()` is called first |
