//! ESP32-C3 ROS2 topic pub/sub demo using zenoh-ros2-nostd.
//!
//! This example demonstrates the full pipeline:
//! WiFi → DHCP → TCP → Zenoh handshake → ROS2 topic publish & subscribe.
//!
//! **Topics**
//! - Publishes `std_msgs/String` to `/chatter` every 5 seconds.
//! - Subscribes to `/chatter` and echoes received messages via RTT.
//!
//! **Architecture**
//! ```text
//! wifi_task     — manages WiFi start/connect/reconnect
//! net_task      — drives the embassy-net packet I/O loop
//! zenoh_task    — manages zenoh session lifecycle:
//!                   TCP connect → handshake → declare subs
//!                   → select { rx_loop | tx_loop } → reconnect
//! app_task      — application logic:
//!                   periodically publishes sensor data via PUB_CHANNEL
//!                   reads received commands from CHATTER_SUB
//! ```
//!
//! **Inter-task communication**
//! - `PUB_CHANNEL` (`Channel<..., CdrBuf, 4>`): app_task → zenoh_task (CDR-encoded payload)
//! - `CHATTER_SUB` (`TopicSubscriber<StringMsg, 256, 4>`): zenoh_task → app_task
//!
//! **Setup**
//! ```sh
//! cp wifi_config.json.example wifi_config.json
//! # Fill in ssid, password, and router_addr (e.g. "192.168.1.1:7447")
//! cargo run --release
//! ```
//!
//! On the ROS2 side, start the zenoh router and verify with:
//! ```sh
//! ros2 topic echo /chatter std_msgs/msg/String
//! ros2 topic pub /chatter std_msgs/msg/String 'data: "hello from ROS2"'
//! ```

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::{DhcpConfig, IpEndpoint, Runner, Stack, StackResources, tcp::TcpSocket};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use esp_alloc as _;
use esp_hal::{
    clock::CpuClock, interrupt::software::SoftwareInterruptControl, ram, rng::Rng,
    timer::timg::TimerGroup,
};
use esp_radio::{
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent, WifiStaState},
    Controller,
};
use heapless::String;
use rtt_target::{rprintln, rtt_init};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use zenoh_ros2_nostd::{
    cdr,
    ros2::{keyexpr::TopicKeyExpr, topic_subscriber::TopicSubscriber},
    session::ReconnectPolicy,
    transport::{codec, frame, handshake, protocol::ZenohId},
};

// ---------------------------------------------------------------------------
// Compile-time credentials (from wifi_config.json via build.rs)
// ---------------------------------------------------------------------------
const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PASSWORD: &str = env!("WIFI_PASSWORD");
/// Zenoh router TCP address: "a.b.c.d:port"
const ZENOH_ROUTER_ADDR: &str = env!("ZENOH_ROUTER_ADDR");

// ---------------------------------------------------------------------------
// ROS2 topic definitions
// ---------------------------------------------------------------------------

/// Key expression for std_msgs/String on /chatter (rmw_zenoh_cpp DDS convention).
const CHATTER_TOPIC: TopicKeyExpr = TopicKeyExpr::new(
    0, // ROS_DOMAIN_ID
    "chatter",
    "std_msgs::msg::dds_::String_",
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
);

/// Zenoh key expression ID for /chatter (used when declaring the subscriber).
const CHATTER_KEY_ID: u16 = 1;
#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String<128>,
}

/// Maximum CDR-encoded size for a StringMsg (header 4 + len 4 + data 128 + null 1 = 137).
const CDR_BUF_SIZE: usize = 144;

/// Type alias for CDR payload buffer.
type CdrBuf = heapless::Vec<u8, CDR_BUF_SIZE>;

// ---------------------------------------------------------------------------
// Inter-task channels
// ---------------------------------------------------------------------------

/// app_task → zenoh_task: CDR-encoded payloads to publish on /chatter.
static PUB_CHANNEL: Channel<CriticalSectionRawMutex, CdrBuf, 4> = Channel::new();

/// zenoh_task → app_task: received /chatter messages.
static CHATTER_SUB: TopicSubscriber<StringMsg, CDR_BUF_SIZE, 4> = TopicSubscriber::new();

// ---------------------------------------------------------------------------
// Static storage
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
// wifi_task
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    loop {
        if matches!(esp_radio::wifi::sta_state(), WifiStaState::Connected) {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            rprintln!("[wifi] Disconnected — retrying in 5 s.");
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let mode_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(WIFI_SSID.into())
                    .with_password(WIFI_PASSWORD.into()),
            );
            controller.set_config(&mode_config).expect("wifi config");
            rprintln!("[wifi] Starting WiFi...");
            controller.start_async().await.expect("wifi start");
            rprintln!("[wifi] WiFi started.");
        }

        rprintln!("[wifi] Connecting to \"{}\" ...", WIFI_SSID);
        match with_timeout(Duration::from_secs(15), controller.connect_async()).await {
            Ok(Ok(_)) => rprintln!("[wifi] Connected!"),
            Ok(Err(e)) => {
                rprintln!("[wifi] Connect error: {:?} — retrying in 5 s.", e);
                Timer::after(Duration::from_secs(5)).await;
            }
            Err(_) => {
                rprintln!("[wifi] Timeout — retrying in 5 s.");
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// net_task
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

// ---------------------------------------------------------------------------
// zenoh_task
//
// Manages the full zenoh session lifecycle:
//   1. Wait for DHCP
//   2. TCP connect to router
//   3. Zenoh handshake
//   4. Declare key expression + subscriber
//   5. Split socket → concurrent RX dispatch + TX (keepalive / pub)
//   6. On error: exponential back-off reconnect
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn zenoh_task(stack: Stack<'static>) {
    // Fixed ZenohId for this device — change per device to avoid collisions.
    let our_zid = ZenohId::from_bytes(&[0xBA, 0xBE, 0xCA, 0xFE, 0x00, 0x01, 0x02, 0x03]);

    let mut reconnect = ReconnectPolicy::default_policy();

    // Parse router address once at boot (runtime parsing; no allocation needed).
    let (router_ip, router_port) = match parse_router_addr(ZENOH_ROUTER_ADDR) {
        Some(addr) => addr,
        None => {
            rprintln!("[zenoh] Invalid ZENOH_ROUTER_ADDR: {}", ZENOH_ROUTER_ADDR);
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    };

    rprintln!("[zenoh] Router: {}", ZENOH_ROUTER_ADDR);

    loop {
        // ---- 1. Wait for DHCP -----------------------------------------------
        loop {
            if stack.config_v4().is_some() {
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        // ---- 2. TCP connect -------------------------------------------------
        let mut tcp_rx_buf = [0u8; 4096];
        let mut tcp_tx_buf = [0u8; 4096];
        let mut socket = TcpSocket::new(stack, &mut tcp_rx_buf, &mut tcp_tx_buf);
        socket.set_timeout(Some(Duration::from_secs(30)));

        let endpoint = IpEndpoint::from((router_ip, router_port));
        rprintln!("[zenoh] TCP connecting...");
        match with_timeout(Duration::from_secs(10), socket.connect(endpoint)).await {
            Ok(Ok(())) => rprintln!("[zenoh] TCP connected."),
            Ok(Err(e)) => {
                rprintln!("[zenoh] TCP connect error: {:?}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
            Err(_) => {
                rprintln!("[zenoh] TCP connect timeout.");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        // ---- 3. Zenoh handshake --------------------------------------------
        let mut hs_tx = [0u8; 512];
        let mut hs_rx = [0u8; 4096];
        let hs = match handshake::client_handshake(&mut socket, &our_zid, &mut hs_tx, &mut hs_rx)
            .await
        {
            Ok(hs) => {
                rprintln!("[zenoh] Handshake OK, lease={}ms", hs.lease_ms);
                hs
            }
            Err(e) => {
                rprintln!("[zenoh] Handshake failed: {:?}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
        };

        let lease_ms = hs.lease_ms.max(2_000); // at least 2 s

        // ---- 4. Declare subscriber ------------------------------------------
        // Declare key expression + subscriber for /chatter before splitting.
        let chatter_ke = CHATTER_TOPIC.to_key_expr().unwrap();
        let mut sn = 0u64;
        let mut decl_buf = [0u8; 512];

        // DeclareKeyExpr
        {
            let mut pos = 0;
            pos += codec::encode_frame_header(&mut decl_buf[pos..], sn, true).unwrap();
            sn += 1;
            pos += codec::encode_declare_keyexpr(
                &mut decl_buf[pos..],
                CHATTER_KEY_ID,
                chatter_ke.as_str(),
            )
            .unwrap();
            if frame::write_frame(&mut socket, &decl_buf[..pos])
                .await
                .is_err()
            {
                rprintln!("[zenoh] Declare key expr failed.");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        // DeclareSubscriber referencing the declared key
        {
            let mut pos = 0;
            pos += codec::encode_frame_header(&mut decl_buf[pos..], sn, true).unwrap();
            sn += 1;
            pos += codec::encode_declare_subscriber_mapped(
                &mut decl_buf[pos..],
                CHATTER_KEY_ID as u32,
                CHATTER_KEY_ID,
            )
            .unwrap();
            if frame::write_frame(&mut socket, &decl_buf[..pos])
                .await
                .is_err()
            {
                rprintln!("[zenoh] Declare subscriber failed.");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        reconnect.reset();
        rprintln!("[zenoh] Session ready. Listening on /chatter.");

        // ---- 5. Split socket → concurrent RX + TX --------------------------
        let (mut reader, mut writer) = socket.split();
        let mut rx_buf = [0u8; 4096];
        let mut tx_buf = [0u8; 512];
        let mut pub_sn = 0i64; // publisher-level sequence number (separate from session sn)

        // RX loop: read frames and dispatch to CHATTER_SUB
        // TX loop: keepalive + publish from PUB_CHANNEL
        // select terminates when either loop returns (i.e., on connection error)
        select(
            rx_loop(&mut reader, &mut rx_buf, chatter_ke.as_str()),
            tx_loop(&mut writer, &mut tx_buf, &mut sn, &mut pub_sn, lease_ms, our_zid),
        )
        .await;

        rprintln!("[zenoh] Session ended — reconnecting...");
        reconnect.wait_and_advance().await;
    }
}

/// Continuously read zenoh frames and dispatch matching Put messages to CHATTER_SUB.
///
/// Returns when a connection error occurs; the caller should then reconnect.
async fn rx_loop<R: embedded_io_async::Read>(
    reader: &mut R,
    rx_buf: &mut [u8],
    sub_key: &str,
) {
    use zenoh_ros2_nostd::transport::codec::{
        TransportMsgKind, decode_frame_header, decode_push_put, parse_transport_msg_kind,
    };

    loop {
        let n = match frame::read_frame(reader, rx_buf).await {
            Ok(n) => n,
            Err(_) => return,
        };
        if n == 0 {
            continue;
        }

        let msg_buf = &rx_buf[..n];
        match parse_transport_msg_kind(msg_buf[0]) {
            TransportMsgKind::Frame => {
                if let Ok((_, _, body_pos)) = decode_frame_header(msg_buf) {
                    let body = &msg_buf[body_pos..];
                    if let Ok(Some((put, _))) = decode_push_put(body) {
                        // Dispatch by matching the inline key suffix or the declared key ID.
                        // CHATTER_KEY_ID == 1 is what we declared for /chatter above.
                        let matches_key = put.key_suffix.contains(sub_key);
                        let matches_scope = put.scope == CHATTER_KEY_ID as u64;
                        if matches_key || matches_scope {
                            CHATTER_SUB.inner().push(put.payload);
                        }
                    }
                }
            }
            TransportMsgKind::KeepAlive => {}
            TransportMsgKind::Close => {
                return;
            }
            _ => {}
        }
    }
}

/// Send keepalives on a timer and drain PUB_CHANNEL to the wire.
///
/// Uses `encode_push_put_with_attachment` so messages include the rmw_zenoh_cpp
/// publisher attachment (sequence number, timestamp, GID).
///
/// Returns when a write error occurs; the caller should then reconnect.
async fn tx_loop<W: embedded_io_async::Write>(
    writer: &mut W,
    tx_buf: &mut [u8],
    sn: &mut u64,
    pub_sn: &mut i64,
    lease_ms: u64,
    gid: ZenohId,
) {
    let keepalive_dur = Duration::from_millis(lease_ms / 2);
    let chatter_ke = CHATTER_TOPIC.to_key_expr().unwrap();

    loop {
        match with_timeout(keepalive_dur, PUB_CHANNEL.receive()).await {
            Ok(cdr_payload) => {
                // Encode frame header + Push/Put with the rmw_zenoh_cpp attachment
                let mut pos = match codec::encode_frame_header(tx_buf, *sn, true) {
                    Ok(n) => n,
                    Err(_) => return,
                };
                *sn = sn.wrapping_add(1);

                let seq = *pub_sn;
                *pub_sn = pub_sn.wrapping_add(1);
                let timestamp_ns =
                    (Instant::now().as_micros() as i64).saturating_mul(1000);

                pos += match codec::encode_push_put_with_attachment(
                    &mut tx_buf[pos..],
                    chatter_ke.as_str(),
                    &cdr_payload,
                    seq,
                    timestamp_ns,
                    &gid,
                ) {
                    Ok(n) => n,
                    Err(_) => return,
                };

                if frame::write_frame(writer, &tx_buf[..pos]).await.is_err() {
                    return;
                }
                rprintln!("[zenoh] Published {} CDR bytes to /chatter.", cdr_payload.len());
            }
            Err(_timeout) => {
                // Keepalive due
                let n = match codec::encode_keepalive(tx_buf) {
                    Ok(n) => n,
                    Err(_) => return,
                };
                if frame::write_frame(writer, &tx_buf[..n]).await.is_err() {
                    return;
                }
                rprintln!("[zenoh] Keepalive sent.");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// app_task
//
// Application logic: publishes StringMsg every 5 s, and echoes received msgs.
// ---------------------------------------------------------------------------
#[embassy_executor::task]
async fn app_task() {
    let mut counter: u32 = 0;

    loop {
        // ---- Publish /chatter every 5 seconds ------------------------------
        let mut msg_str: String<128> = String::new();
        let _ = core::fmt::write(
            &mut msg_str,
            core::format_args!("Hello from MCU! count={}", counter),
        );
        counter += 1;

        let msg = StringMsg { data: msg_str };

        let mut cdr_buf = [0u8; CDR_BUF_SIZE];
        match cdr::serialize_with_header(&mut cdr_buf, &msg) {
            Ok(n) => {
                let mut payload: CdrBuf = heapless::Vec::new();
                let _ = payload.extend_from_slice(&cdr_buf[..n]);
                let _ = PUB_CHANNEL.try_send(payload);
                rprintln!("[app] Queued: \"{}\"", msg.data.as_str());
            }
            Err(e) => {
                rprintln!("[app] CDR encode error: {:?}", e);
            }
        }

        // ---- Try to receive any pending /chatter messages ------------------
        while let Some(result) = CHATTER_SUB.try_recv() {
            match result {
                Ok(m) => rprintln!("[app] Received /chatter: \"{}\"", m.data.as_str()),
                Err(e) => rprintln!("[app] Deserialize error: {:?}", e),
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // 1. RTT — must be first so panic handler can print
    let channels = rtt_init! {
        up: { 0: { size: 4096, name: "Terminal" } }
    };
    rtt_target::set_print_channel(channels.up.0);

    rprintln!("\r\n");
    rprintln!("=================================");
    rprintln!("  ESP32-C3  zenoh-ros2-nostd     ");
    rprintln!("=================================");
    rprintln!("SSID  : {}", WIFI_SSID);
    rprintln!("Router: {}", ZENOH_ROUTER_ADDR);

    // 2. HAL
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // 3. Heap — two regions for WiFi stack stability
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    // 4. esp-rtos scheduler
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // 5. esp-radio + WiFi driver
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

    // 8. Wait for link-up + DHCP
    rprintln!("[net] Waiting for link up...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    rprintln!("[net] Waiting for DHCP...");
    loop {
        if let Some(cfg) = stack.config_v4() {
            rprintln!("[net] IP  : {}", cfg.address);
            if let Some(gw) = cfg.gateway {
                rprintln!("[net] GW  : {}", gw);
            }
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    // 9. Spawn zenoh + app tasks
    spawner.spawn(zenoh_task(stack)).ok();
    spawner.spawn(app_task()).ok();

    // Keep main task alive
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Parse a `"a.b.c.d:port"` address string into `(Ipv4Address, u16)`.
///
/// Returns `None` if the format is invalid.
fn parse_router_addr(addr: &str) -> Option<(embassy_net::Ipv4Address, u16)> {
    // Find the last ':' to split IP from port
    let colon = addr.rfind(':')?;
    let ip_str = &addr[..colon];
    let port_str = &addr[colon + 1..];

    let port: u16 = parse_u16(port_str)?;

    let mut octets = [0u8; 4];
    let mut idx = 0;
    for part in ip_str.split('.') {
        if idx >= 4 {
            return None;
        }
        octets[idx] = parse_u8(part)?;
        idx += 1;
    }
    if idx != 4 {
        return None;
    }

    Some((embassy_net::Ipv4Address::from(octets), port))
}

/// Parse a decimal `u8` from a string slice (no allocation, no `std`).
fn parse_u8(s: &str) -> Option<u8> {
    if s.is_empty() || s.len() > 3 {
        return None;
    }
    let mut val: u16 = 0;
    for b in s.bytes() {
        if b < b'0' || b > b'9' {
            return None;
        }
        val = val * 10 + (b - b'0') as u16;
        if val > 255 {
            return None;
        }
    }
    Some(val as u8)
}

/// Parse a decimal `u16` from a string slice (no allocation, no `std`).
fn parse_u16(s: &str) -> Option<u16> {
    if s.is_empty() || s.len() > 5 {
        return None;
    }
    let mut val: u32 = 0;
    for b in s.bytes() {
        if b < b'0' || b > b'9' {
            return None;
        }
        val = val * 10 + (b - b'0') as u32;
        if val > 65535 {
            return None;
        }
    }
    Some(val as u16)
}
