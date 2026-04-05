//! ESP32-C3 ROS2 topic pub/sub demo using zenoh-ros2-nostd.
//!
//! Connects to a WiFi AP, establishes a Zenoh session with the configured
//! router, then publishes `std_msgs/String` on `/chatter` every 5 seconds
//! and echoes any received `/chatter` messages via RTT.
//!
//! ## Setup
//! ```sh
//! cp wifi_config.json.example wifi_config.json
//! # Fill in: ssid, password, router_addr (e.g. "192.168.1.1:7447")
//! cargo run --release
//! ```
//!
//! ## Switching from WiFi to Ethernet
//! Replace [`connect_tcp`] with your Ethernet TCP socket.
//! [`run_zenoh_session`] accepts any `Read + Write` split and is completely
//! transport-agnostic; the rest of the code is unchanged.
//!
//! ## Task architecture
//! ```text
//! wifi_task   — WiFi connect / reconnect loop
//! net_task    — embassy-net packet I/O driver
//! zenoh_task  — TCP connect → Zenoh session → reconnect with back-off
//! app_task    — publish counter message every 5 s; echo received messages
//! ```

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

extern crate alloc;

mod config;

use config::{AppConfig, ZenohConfig};
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::{DhcpConfig, Runner, Stack, StackResources, tcp::TcpSocket};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_io_async::{Error as IoError, ErrorKind, ErrorType, Read, Write};
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
    cdr,
    error::Error,
    ros2::{keyexpr::TopicKeyExpr, topic_subscriber::TopicSubscriber},
    session::ReconnectPolicy,
    transport::{codec, frame, handshake, protocol::ZenohId},
};

// ── ROS2 topic definitions ──────────────────────────────────────────────────────

/// Zenoh key ID assigned to the /chatter key expression on declaration.
const CHATTER_KEY_ID: u16 = 1;

/// /chatter topic key expression (rmw_zenoh_cpp DDS convention).
const CHATTER_TOPIC: TopicKeyExpr = TopicKeyExpr::new(
    0, // ROS_DOMAIN_ID
    "chatter",
    "std_msgs::msg::dds_::String_",
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
);

/// `std_msgs/String` message type for embedded targets (data up to 128 bytes).
#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String<128>,
}

/// CDR serialization buffer capacity for `StringMsg`.
/// 4 (encap header) + 4 (length) + 128 (data) + 1 (null) = 137 → rounded to 144.
const CDR_BUF_CAP: usize = 144;
type CdrBuf = heapless::Vec<u8, CDR_BUF_CAP>;

// ── Inter-task channels ─────────────────────────────────────────────────────────

/// `app_task` → `zenoh_task`: CDR-encoded payloads to publish on /chatter.
static PUB_CHANNEL: Channel<CriticalSectionRawMutex, CdrBuf, 4> = Channel::new();

/// `zenoh_task` → `app_task`: received /chatter messages.
static CHATTER_SUB: TopicSubscriber<StringMsg, CDR_BUF_CAP, 4> = TopicSubscriber::new();

// ── Static HAL storage ──────────────────────────────────────────────────────────

static RADIO: StaticCell<Controller<'static>> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

// ── Panic handler ───────────────────────────────────────────────────────────────

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    rprintln!("\n\n!!! PANIC !!!\n{}\n", info);
    loop {
        core::hint::spin_loop();
    }
}

// ── Tasks ───────────────────────────────────────────────────────────────────────

/// WiFi management: start driver, connect, and reconnect on disconnect.
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

/// Zenoh session lifecycle: TCP connect → run session → exponential-backoff reconnect.
///
/// TCP socket creation is isolated in [`connect_tcp`].  The session logic lives in
/// [`run_zenoh_session`] and is transport-agnostic — swap [`connect_tcp`] to use
/// Ethernet without touching any other code.
#[embassy_executor::task]
async fn zenoh_task(stack: Stack<'static>) {
    let cfg = AppConfig::new();
    let mut reconnect = ReconnectPolicy::default_policy();

    loop {
        wait_for_dhcp(stack).await;

        // ── Create transport ────────────────────────────────────────────────
        // Replace `connect_tcp` here to use Ethernet instead of WiFi.
        let mut tcp_rx = [0u8; 4096];
        let mut tcp_tx = [0u8; 4096];
        let Ok(mut socket) = connect_tcp(stack, &cfg.zenoh, &mut tcp_rx, &mut tcp_tx).await else {
            rprintln!("[zenoh] TCP connect failed.");
            reconnect.wait_and_advance().await;
            continue;
        };

        // ── Run Zenoh session (transport-agnostic) ──────────────────────────
        let (mut reader, mut writer) = socket.split();
        run_zenoh_session(&mut reader, &mut writer, &cfg.zenoh).await;

        rprintln!("[zenoh] Session ended — reconnecting.");
        reconnect.reset();
        reconnect.wait_and_advance().await;
    }
}

/// Application logic: publishes a counter message every 5 s; echoes received messages.
#[embassy_executor::task]
async fn app_task() {
    let mut counter: u32 = 0;
    loop {
        queue_counter_message(&mut counter);
        drain_received_messages();
        Timer::after(Duration::from_secs(5)).await;
    }
}

// ── Entry point ─────────────────────────────────────────────────────────────────

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // RTT must be first so the panic handler can print.
    let channels = rtt_init! { up: { 0: { size: 4096, name: "Terminal" } } };
    rtt_target::set_print_channel(channels.up.0);

    let cfg = AppConfig::new();
    rprintln!("\r\n=== ESP32-C3  zenoh-ros2-nostd ===");
    rprintln!("SSID  : {}", cfg.wifi_ssid);

    // HAL initialisation.
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

    // Wait for network before spawning application tasks.
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

// ── Network helpers ─────────────────────────────────────────────────────────────

/// Poll until embassy-net has an assigned IPv4 address and log it.
async fn wait_for_dhcp(stack: Stack<'_>) {
    while stack.config_v4().is_none() {
        Timer::after(Duration::from_millis(500)).await;
    }
    if let Some(cfg) = stack.config_v4() {
        rprintln!("[net] IP: {}  GW: {:?}", cfg.address, cfg.gateway);
    }
}

/// Create and connect a TCP socket to the Zenoh router (WiFi transport).
///
/// **To use Ethernet instead:** replace this function with one that creates an
/// Ethernet `TcpSocket` — everything else ([`run_zenoh_session`] and the tasks)
/// remains unchanged.
///
/// Returns `Ok(socket)` on success or `Err` on timeout / connection failure.
async fn connect_tcp<'d>(
    stack: Stack<'d>,
    cfg: &ZenohConfig,
    rx_buf: &'d mut [u8],
    tx_buf: &'d mut [u8],
) -> Result<TcpSocket<'d>, Error> {
    let mut socket = TcpSocket::new(stack, rx_buf, tx_buf);
    socket.set_timeout(Some(Duration::from_secs(30)));

    rprintln!("[zenoh] TCP connecting...");
    with_timeout(Duration::from_secs(10), socket.connect(cfg.router_endpoint()))
        .await
        .map_err(|_| Error::Timeout)?
        .map_err(|_| Error::Io)?;

    rprintln!("[zenoh] TCP connected.");
    Ok(socket)
}

// ── Zenoh session ───────────────────────────────────────────────────────────────

/// Run a complete Zenoh session: handshake → declare subscriber → concurrent I/O.
///
/// Accepts **separate reader and writer** so it works with any transport:
/// WiFi `TcpSocket::split()`, Ethernet reader/writer, or a mock I/O pair for
/// testing.  Returns when the connection drops or a protocol error occurs.
///
/// # Transport independence
///
/// The caller is responsible for creating and connecting the transport.
/// This function only speaks the Zenoh protocol over whatever `R` and `W`
/// are provided — it has no knowledge of WiFi, Ethernet, or any hardware.
async fn run_zenoh_session<R, W>(reader: &mut R, writer: &mut W, cfg: &ZenohConfig)
where
    R: Read,
    W: Write,
{
    let mut hs_tx = [0u8; 512];
    let mut hs_rx = [0u8; 4096];

    // ── Zenoh handshake ─────────────────────────────────────────────────────
    // The handshake needs a single Read+Write; use a temporary shim that joins
    // the separate reader and writer.  The shim is dropped after this block so
    // `reader` and `writer` become available again for the concurrent I/O loops.
    let hs = {
        let mut link = ReadWriter { reader: &mut *reader, writer: &mut *writer };
        handshake::client_handshake(&mut link, &cfg.session.zid, &mut hs_tx, &mut hs_rx).await
    };

    let hs = match hs {
        Ok(h) => {
            rprintln!("[zenoh] Handshake OK — lease {}ms.", h.lease_ms);
            h
        }
        Err(e) => {
            rprintln!("[zenoh] Handshake failed: {:?}", e);
            return;
        }
    };

    // ── Declare /chatter subscriber ─────────────────────────────────────────
    let chatter_ke = CHATTER_TOPIC.to_key_expr().unwrap();
    let mut session_sn = 0u64;
    if let Err(e) =
        declare_chatter_subscriber(writer, chatter_ke.as_str(), &mut session_sn).await
    {
        rprintln!("[zenoh] Subscribe declaration failed: {:?}", e);
        return;
    }
    rprintln!("[zenoh] Subscribed to /chatter.  Session ready.");

    // ── Concurrent RX + TX ──────────────────────────────────────────────────
    let lease_ms = hs.lease_ms.max(2_000);
    let mut pub_sn = 0i64;
    select(
        rx_frame_loop(reader, chatter_ke.as_str()),
        tx_frame_loop(
            writer,
            &mut session_sn,
            &mut pub_sn,
            &cfg.session.zid,
            lease_ms,
        ),
    )
    .await;
}

// ── Zenoh protocol helpers ──────────────────────────────────────────────────────

/// Send `DeclareKeyExpr` + `DeclareSubscriber` for the /chatter topic.
async fn declare_chatter_subscriber<W: Write>(
    writer: &mut W,
    key_expr: &str,
    sn: &mut u64,
) -> Result<(), Error> {
    let mut buf = [0u8; 512];

    // DeclareKeyExpr: assign CHATTER_KEY_ID to the key expression string.
    let n = {
        let mut pos = codec::encode_frame_header(&mut buf, *sn, true)
            .map_err(Error::Transport)?;
        *sn += 1;
        pos += codec::encode_declare_keyexpr(&mut buf[pos..], CHATTER_KEY_ID, key_expr)
            .map_err(Error::Transport)?;
        pos
    };
    frame::write_frame(writer, &buf[..n])
        .await
        .map_err(Error::Transport)?;

    // DeclareSubscriber: reference the key by its assigned ID.
    let n = {
        let mut pos = codec::encode_frame_header(&mut buf, *sn, true)
            .map_err(Error::Transport)?;
        *sn += 1;
        pos += codec::encode_declare_subscriber_mapped(
            &mut buf[pos..],
            CHATTER_KEY_ID as u32,
            CHATTER_KEY_ID,
        )
        .map_err(Error::Transport)?;
        pos
    };
    frame::write_frame(writer, &buf[..n])
        .await
        .map_err(Error::Transport)?;

    Ok(())
}

/// Receive Zenoh frames and dispatch matching Put payloads to [`CHATTER_SUB`].
/// Returns when the connection drops.
async fn rx_frame_loop<R: Read>(reader: &mut R, sub_key: &str) {
    use zenoh_ros2_nostd::transport::codec::{
        TransportMsgKind, decode_frame_header, decode_push_put, parse_transport_msg_kind,
    };
    let mut rx_buf = [0u8; 4096];

    loop {
        let n = match frame::read_frame(reader, &mut rx_buf).await {
            Ok(n) => n,
            Err(_) => return,
        };
        if n == 0 {
            continue;
        }

        let buf = &rx_buf[..n];
        match parse_transport_msg_kind(buf[0]) {
            TransportMsgKind::Frame => {
                if let Ok((_, _, body_pos)) = decode_frame_header(buf) {
                    if let Ok(Some((put, _))) = decode_push_put(&buf[body_pos..]) {
                        let by_suffix = put.key_suffix.contains(sub_key);
                        let by_scope = put.scope == CHATTER_KEY_ID as u64;
                        if by_suffix || by_scope {
                            CHATTER_SUB.inner().push(put.payload);
                        }
                    }
                }
            }
            TransportMsgKind::Close => return,
            _ => {}
        }
    }
}

/// Send keepalives on a timer and publish payloads from [`PUB_CHANNEL`].
/// Returns when a write error occurs.
async fn tx_frame_loop<W: Write>(
    writer: &mut W,
    session_sn: &mut u64,
    pub_sn: &mut i64,
    gid: &ZenohId,
    lease_ms: u64,
) {
    let keepalive_dur = Duration::from_millis(lease_ms / 2);
    let chatter_ke = CHATTER_TOPIC.to_key_expr().unwrap();
    let mut tx_buf = [0u8; 512];

    loop {
        match with_timeout(keepalive_dur, PUB_CHANNEL.receive()).await {
            Ok(cdr_payload) => {
                let ts = (Instant::now().as_micros() as i64).saturating_mul(1000);
                let seq = *pub_sn;
                *pub_sn = pub_sn.wrapping_add(1);

                let pos = match encode_pub_frame(
                    &mut tx_buf,
                    session_sn,
                    chatter_ke.as_str(),
                    &cdr_payload,
                    seq,
                    ts,
                    gid,
                ) {
                    Ok(n) => n,
                    Err(_) => return,
                };
                if frame::write_frame(writer, &tx_buf[..pos]).await.is_err() {
                    return;
                }
                rprintln!("[zenoh] Published {} B to /chatter.", cdr_payload.len());
            }
            Err(_timeout) => {
                let n = match codec::encode_keepalive(&mut tx_buf) {
                    Ok(n) => n,
                    Err(_) => return,
                };
                if frame::write_frame(writer, &tx_buf[..n]).await.is_err() {
                    return;
                }
            }
        }
    }
}

/// Encode a Frame + Push/Put-with-attachment into `buf`, advancing `sn`.
fn encode_pub_frame(
    buf: &mut [u8],
    sn: &mut u64,
    key_expr: &str,
    payload: &[u8],
    seq: i64,
    ts: i64,
    gid: &ZenohId,
) -> Result<usize, zenoh_ros2_nostd::error::TransportError> {
    let mut pos = codec::encode_frame_header(buf, *sn, true)?;
    *sn = sn.wrapping_add(1);
    pos += codec::encode_push_put_with_attachment(
        &mut buf[pos..],
        key_expr,
        payload,
        seq,
        ts,
        gid,
    )?;
    Ok(pos)
}

// ── Application helpers ─────────────────────────────────────────────────────────

/// Encode a counter message and queue it for publishing on /chatter.
fn queue_counter_message(counter: &mut u32) {
    let mut text: String<128> = String::new();
    let _ = core::fmt::write(
        &mut text,
        core::format_args!("Hello from MCU! count={}", counter),
    );
    *counter += 1;

    let msg = StringMsg { data: text };
    let mut raw = [0u8; CDR_BUF_CAP];
    match cdr::serialize_with_header(&mut raw, &msg) {
        Ok(n) => {
            let mut payload: CdrBuf = heapless::Vec::new();
            let _ = payload.extend_from_slice(&raw[..n]);
            let _ = PUB_CHANNEL.try_send(payload);
            rprintln!("[app] Queued: \"{}\"", msg.data.as_str());
        }
        Err(e) => rprintln!("[app] CDR encode error: {:?}", e),
    }
}

/// Drain all pending messages from [`CHATTER_SUB`] and echo them via RTT.
fn drain_received_messages() {
    while let Some(result) = CHATTER_SUB.try_recv() {
        match result {
            Ok(m) => rprintln!("[app] Received /chatter: \"{}\"", m.data.as_str()),
            Err(e) => rprintln!("[app] Deserialize error: {:?}", e),
        }
    }
}

// ── ReadWriter shim ─────────────────────────────────────────────────────────────

/// Joins a separate `reader: &mut R` and `writer: &mut W` into a single
/// `Read + Write` type, needed by the Zenoh handshake function.
///
/// The shim converts each side's error to [`ErrorKind`] via `map_err` so both
/// halves share a common `ErrorType`.  It is a lightweight temporary; drop it
/// after the handshake to regain independent access to `reader` and `writer`.
struct ReadWriter<'r, 'w, R: Read, W: Write> {
    reader: &'r mut R,
    writer: &'w mut W,
}

impl<R: Read, W: Write> ErrorType for ReadWriter<'_, '_, R, W> {
    type Error = ErrorKind;
}

impl<R: Read, W: Write> Read for ReadWriter<'_, '_, R, W> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
        self.reader.read(buf).await.map_err(|e| e.kind())
    }
}

impl<R: Read, W: Write> Write for ReadWriter<'_, '_, R, W> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
        self.writer.write(buf).await.map_err(|e| e.kind())
    }

    async fn flush(&mut self) -> Result<(), ErrorKind> {
        self.writer.flush().await.map_err(|e| e.kind())
    }
}
