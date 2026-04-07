//! ESP32-C3 ROS2 topic pub/sub demo using zenoh-ros2-nostd.
//!
//! Demonstrates the full pipeline:
//! ```text
//! WiFi → DHCP → TCP → Zenoh session → ROS2 /chatter pub/sub
//! ```
//!
//! ## Application architecture
//! ```text
//! wifi_task   — WiFi connect / reconnect loop
//! net_task    — embassy-net packet I/O driver
//! zenoh_task  — NodeBuilder::open(socket) → node.spin() → reconnect
//! app_task    — CHATTER_PUB.send(&msg) every 5 s; CHATTER_SUB.try_recv()
//! ```
//!
//! ## Transport independence
//! Replace the TCP socket creation in `zenoh_task` with any transport.
//! [`NodeBuilder::open`] accepts `T: Read + Write` — the session logic is
//! completely transport-agnostic.
//!
//! ## Setup
//! ```sh
//! cp wifi_config.json.example wifi_config.json
//! # Fill in: ssid, password, router_addr (e.g. "192.168.1.1:7447")
//! cargo run --release
//! ```

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

extern crate alloc;

mod config;

use config::AppConfig;
use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Timer, with_timeout};
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock, interrupt::software::SoftwareInterruptControl, ram, rng::Rng,
    timer::timg::TimerGroup,
};
use esp_radio::{
    Controller,
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
};
use heapless::String;
use rtt_target::{rprintln, rtt_init};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use zenoh_ros2_nostd::{
    ros2::{
        Publisher, Subscription, TopicKeyExpr,
        publisher::PublisherDrain,
        subscription::SubscriptionDispatch,
    },
    session::ReconnectPolicy,
};

// ── ROS2 topic definition ────────────────────────────────────────────────────

/// Key expression for `std_msgs/String` on `/chatter` (rmw_zenoh_cpp convention).
const CHATTER_TOPIC: TopicKeyExpr = TopicKeyExpr::new(
    0, // ROS_DOMAIN_ID
    "chatter",
    "std_msgs::msg::dds_::String_",
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
);

/// `std_msgs/String` message type.
#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String<128>,
}

/// CDR buffer capacity for `StringMsg` (4-byte header + 4-byte length + 128-byte data + null).
const CDR_BUF_CAP: usize = 144;

// ── Static publisher and subscriber ─────────────────────────────────────────
//
// These live for the entire program lifetime.  Any task can call `send` /
// `try_recv`; the Zenoh task drains / dispatches them via `Node::spin`.

static CHATTER_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();

// ── Static HAL storage ───────────────────────────────────────────────────────

static RADIO: StaticCell<Controller<'static>> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

// ── Panic handler ────────────────────────────────────────────────────────────

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    rprintln!("\n\n!!! PANIC !!!\n{}\n", info);
    loop {
        core::hint::spin_loop();
    }
}

// ── Tasks ────────────────────────────────────────────────────────────────────

/// WiFi management: start driver, connect, reconnect on disconnect.
#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    let cfg = AppConfig::new();
    loop {
        if matches!(esp_radio::wifi::sta_state(), WifiStaState::Connected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            rprintln!("[wifi] Disconnected — retrying in 5 s.");
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }
        if !matches!(controller.is_started(), Ok(true)) {
            controller
                .set_config(&ModeConfig::Client(
                    ClientConfig::default()
                        .with_ssid(cfg.wifi_ssid.into())
                        .with_password(cfg.wifi_password.into()),
                ))
                .expect("wifi config");
            rprintln!("[wifi] Starting...");
            controller.start_async().await.expect("wifi start");
        }
        rprintln!("[wifi] Connecting to \"{}\"...", cfg.wifi_ssid);
        match with_timeout(Duration::from_secs(15), controller.connect_async()).await {
            Ok(Ok(_)) => rprintln!("[wifi] Connected."),
            Ok(Err(e)) => {
                rprintln!("[wifi] Error: {:?} — retrying in 5 s.", e);
                Timer::after(Duration::from_secs(5)).await;
            }
            Err(_) => {
                rprintln!("[wifi] Timeout — retrying in 5 s.");
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

/// Drives the embassy-net packet I/O loop.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

/// Zenoh session lifecycle.
///
/// 1. Wait for DHCP.
/// 2. Create a TCP socket and connect to the Zenoh router.
/// 3. Build a [`Node`](NodeBuilder) and open it with the socket.
/// 4. Register publishers, declare subscribers.
/// 5. Run [`Node::spin`] until the connection drops.
/// 6. Exponential back-off, then go to 1.
///
/// To use Ethernet instead of WiFi, replace the `TcpSocket::new` /
/// `socket.connect` section with an Ethernet socket.
/// Steps 3–6 are transport-agnostic and remain unchanged.
#[embassy_executor::task]
async fn zenoh_task(stack: Stack<'static>) {
    let cfg = AppConfig::new();
    let mut reconnect = ReconnectPolicy::default_policy();

    loop {
        wait_for_dhcp(stack).await;

        // ── Create TCP transport ─────────────────────────────────────────────
        // Replace this block to use Ethernet, USB CDC, or any other transport.
        // Everything below (NodeBuilder::open → spin) remains unchanged.
        let mut tcp_rx = [0u8; 4096];
        let mut tcp_tx = [0u8; 4096];
        let mut socket = TcpSocket::new(stack, &mut tcp_rx, &mut tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(30)));

        rprintln!("[zenoh] TCP connecting...");
        match with_timeout(
            Duration::from_secs(10),
            socket.connect(cfg.zenoh.router_endpoint()),
        )
        .await
        {
            Ok(Ok(())) => rprintln!("[zenoh] TCP connected."),
            Ok(Err(e)) => {
                rprintln!("[zenoh] TCP error: {:?}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
            Err(_) => {
                rprintln!("[zenoh] TCP timeout.");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        // ── Build Node: handshake + configure pub/sub ────────────────────────
        let mut node = match cfg
            .zenoh
            .session
            .node_builder()
            .name("esp32_node")
            .open(socket)
            .await
        {
            Ok(n) => n,
            Err(e) => {
                rprintln!("[zenoh] Handshake failed: {:?}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
        };

        node.register_publisher(&CHATTER_PUB as &'static dyn PublisherDrain);

        if let Err(e) = node
            .subscribe(CHATTER_TOPIC, &CHATTER_SUB as &'static dyn SubscriptionDispatch)
            .await
        {
            rprintln!("[zenoh] Subscribe failed: {:?}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        reconnect.reset();
        rprintln!("[zenoh] Node '{}' ready.", node.node_name());

        // ── Run session loop (returns only on connection drop) ────────────────
        let mut rx_buf = [0u8; 4096];
        node.spin(&mut rx_buf).await;

        rprintln!("[zenoh] Session ended — reconnecting.");
        reconnect.wait_and_advance().await;
    }
}

/// Application logic: publish a counter message every 5 s; echo received messages.
#[embassy_executor::task]
async fn app_task() {
    let mut counter: u32 = 0;
    loop {
        // ── Publish ─────────────────────────────────────────────────────────
        let mut data: String<128> = String::new();
        let _ = core::fmt::write(
            &mut data,
            core::format_args!("Hello from MCU! count={}", counter),
        );
        counter += 1;

        match CHATTER_PUB.send(&StringMsg { data }).await {
            Ok(()) => {}
            Err(e) => rprintln!("[app] Send error: {:?}", e),
        }

        // ── Receive ─────────────────────────────────────────────────────────
        while let Some(result) = CHATTER_SUB.try_recv() {
            match result {
                Ok(m) => rprintln!("[app] Received: \"{}\"", m.data.as_str()),
                Err(e) => rprintln!("[app] Deserialize error: {:?}", e),
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

// ── Entry point ──────────────────────────────────────────────────────────────

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // RTT must be first so the panic handler can print.
    let channels = rtt_init! { up: { 0: { size: 4096, name: "Terminal" } } };
    rtt_target::set_print_channel(channels.up.0);

    let cfg = AppConfig::new();
    rprintln!("\r\n=== ESP32-C3  zenoh-ros2-nostd ===");
    rprintln!("SSID  : {}", cfg.wifi_ssid);

    // HAL initialisation (CPU clock, peripherals).
    let peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // Heap — two regions required for WiFi firmware stability.
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    // esp-rtos scheduler — must precede any `.await` or esp-radio call.
    esp_rtos::start(
        TimerGroup::new(peripherals.TIMG0).timer0,
        SoftwareInterruptControl::new(peripherals.SW_INTERRUPT).software_interrupt0,
    );

    // WiFi driver.
    let radio = RADIO.init(esp_radio::init().expect("radio init"));
    let (controller, interfaces) =
        esp_radio::wifi::new(radio, peripherals.WIFI, Default::default()).expect("wifi init");

    // embassy-net stack with DHCP.
    let seed = (Rng::new().random() as u64) << 32 | Rng::new().random() as u64;
    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    // Infrastructure tasks.
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(wifi_task(controller)).ok();

    // Wait for network.
    rprintln!("[net] Waiting for link...");
    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(500)).await;
    }
    wait_for_dhcp(stack).await;

    // Application tasks.
    spawner.spawn(zenoh_task(stack)).ok();
    spawner.spawn(app_task()).ok();

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Poll until embassy-net has an assigned IPv4 address and log it.
async fn wait_for_dhcp(stack: Stack<'_>) {
    while stack.config_v4().is_none() {
        Timer::after(Duration::from_millis(500)).await;
    }
    if let Some(cfg) = stack.config_v4() {
        rprintln!("[net] IP: {}  GW: {:?}", cfg.address, cfg.gateway);
    }
}
