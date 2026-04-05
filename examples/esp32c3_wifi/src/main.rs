//! ESP32-C3 Wi-Fi connectivity demo.
//!
//! SSID and password are read from `wifi_config.json` at compile time and
//! embedded as string constants — no interactive input required at runtime.
//!
//! Architecture:
//! - `wifi_task`: owns `WifiController`, manages start/connect/reconnect loop
//! - `net_task`:  runs the embassy-net stack (packet I/O loop)
//! - `main`:      initialises hardware, spawns tasks, waits for DHCP lease
//!
//! # Hardware
//! - ESP32-C3 (any revision)
//! - probe-rs compatible JTAG probe connected to JTAG pins:
//!   GPIO4 = TCK, GPIO5 = TDI, GPIO6 = TDO, GPIO7 = TMS
//!
//! # Setup
//! ```sh
//! cp wifi_config.json.example wifi_config.json
//! # Edit wifi_config.json and fill in your SSID and password.
//! ```
//!
//! # Build & run
//! ```sh
//! cd examples/esp32c3_wifi
//! cargo run --release
//! ```

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

extern crate alloc;

use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{with_timeout, Duration, Timer};
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock, interrupt::software::SoftwareInterruptControl, ram, rng::Rng,
    timer::timg::TimerGroup,
};
use esp_radio::{
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
    Controller,
};
use rtt_target::{rprintln, rtt_init};
use static_cell::StaticCell;

// ---------------------------------------------------------------------------
// Compile-time Wi-Fi credentials (embedded by build.rs from wifi_config.json)
// ---------------------------------------------------------------------------
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");

// ---------------------------------------------------------------------------
// Static storage — required so that tasks can hold 'static references
// ---------------------------------------------------------------------------
static RADIO: StaticCell<Controller<'static>> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

// ---------------------------------------------------------------------------
// Panic handler
// ---------------------------------------------------------------------------
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    rprintln!("\n\n!!! PANIC !!!\n{}\n", info);
    loop {
        core::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// wifi_task — owns WifiController; starts WiFi and connects/reconnects
//
// Pattern follows the official esp-radio 0.17 embassy_dhcp example:
//   - set_config + start_async done here (not in main)
//   - check is_started() before start to support reconnects
//   - check sta_state() to skip re-connect if already up
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    loop {
        // If already connected, just wait for a disconnect event.
        if matches!(esp_radio::wifi::sta_state(), WifiStaState::Connected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            rprintln!("[wifi] Disconnected — will retry in 5 s.");
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }

        // Start the WiFi driver if not already started (first run or after deinit).
        if !matches!(controller.is_started(), Ok(true)) {
            let mode_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(WIFI_SSID.into())
                    .with_password(WIFI_PASSWORD.into()),
            );
            controller
                .set_config(&mode_config)
                .expect("set wifi config");
            rprintln!("[wifi] Starting WiFi...");
            controller.start_async().await.expect("wifi start");
            rprintln!("[wifi] WiFi started.");
        }

        rprintln!("[wifi] Connecting to \"{}\" ...", WIFI_SSID);
        match with_timeout(Duration::from_secs(15), controller.connect_async()).await {
            Ok(Ok(_)) => {
                rprintln!("[wifi] Connected!");
            }
            Ok(Err(e)) => {
                rprintln!("[wifi] Connect error: {:?} — retrying in 5 s.", e);
                Timer::after(Duration::from_secs(5)).await;
            }
            Err(_) => {
                rprintln!("[wifi] Timeout — AP unreachable or wrong credentials. Retrying in 5 s.");
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// net_task — runs the embassy-net packet I/O loop; never returns
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // 1. RTT first — panic handler needs it
    let channels = rtt_init! {
        up: { 0: { size: 4096, name: "Terminal" } }
    };
    rtt_target::set_print_channel(channels.up.0);

    rprintln!("\r\n");
    rprintln!("=========================");
    rprintln!("  ESP32-C3  Wi-Fi  Demo  ");
    rprintln!("=========================");
    rprintln!("SSID    : {}", WIFI_SSID);
    rprintln!("Password: {} chars", WIFI_PASSWORD.len());

    // 2. HAL
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // 3. Heap — two regions required for WiFi stability:
    //    a) reclaimed bootloader RAM (used internally by the WiFi C firmware via malloc_internal)
    //    b) regular DRAM for application allocations
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    // 4. esp-rtos scheduler — must come before any .await or esp-radio call
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // 5. esp-radio + WiFi driver
    //    Store Controller in a StaticCell so WifiController gets a 'static lifetime,
    //    which is required for spawning tasks.
    let radio = RADIO.init(esp_radio::init().expect("radio init"));
    let (controller, interfaces) =
        esp_radio::wifi::new(radio, peripherals.WIFI, Default::default()).expect("wifi init");

    // 6. embassy-net stack with DHCP
    let seed = (Rng::new().random() as u64) << 32 | Rng::new().random() as u64;
    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    // 7. Spawn background tasks
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(wifi_task(controller)).ok();

    // 8. Wait for link-up, then for a DHCP lease
    rprintln!("[net] Waiting for link up...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    rprintln!("[net] Waiting for DHCP address...");
    loop {
        if let Some(cfg) = stack.config_v4() {
            rprintln!("[net] IP  : {}", cfg.address);
            if let Some(gw) = cfg.gateway {
                rprintln!("[net] GW  : {}", gw);
            }
            break;
        } else {
            rprintln!("[net] No IP yet...");
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // Application loop — replace with real work (e.g. Zenoh TCP session using `stack`)
    loop {
        Timer::after(Duration::from_secs(30)).await;
        match stack.config_v4() {
            Some(cfg) => rprintln!("[app] Still up — IP: {}", cfg.address),
            None => rprintln!("[app] No IP (reconnecting ...)"),
        }
    }
}
