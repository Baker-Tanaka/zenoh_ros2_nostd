//! RP2040 + W5500 Ethernet  ROS2 topic pub/sub demo.
//!
//! Connects to a Zenoh router over W5500 Ethernet, then publishes
//! `std_msgs/String` on `/chatter` every 5 seconds and echoes any
//! received `/chatter` messages via defmt RTT.
//!
//! ## Hardware (baker link.Dev)
//!
//! ```text
//! RP2040 GPIO         │ W5500 pin
//! ────────────────────┼──────────────────
//! GP16 (SPI0 RX/MISO) │ MISO
//! GP17 (GP output)    │ SCS  (chip select, active-low)
//! GP18 (SPI0 SCK)     │ SCLK
//! GP19 (SPI0 TX/MOSI) │ MOSI
//! GP20 (GP input)     │ INTn (active-low, pull-up on RP2040 side)
//! GP21 (GP output)    │ RSTn (active-low; hold HIGH for normal op)
//! ```
//!
//! Adjust pin constants below if your board wires the W5500 differently.
//!
//! ## Network topology
//!
//! ```text
//! RP2040+W5500 ──Ethernet──► Zenoh router (zenohd / rmw_zenohd :7447)
//!                              ▲
//!              Docker ROS2 ────┘   (ros2 topic echo /chatter)
//! ```
//!
//! ## Setup
//!
//! ```sh
//! cp config.json.example config.json
//! # Set router_addr to IP:port of your Zenoh router, e.g. "192.168.1.1:7447"
//! cargo run --release
//! ```
//!
//! ## Task architecture
//!
//! ```text
//! ethernet_task  — drives W5500 SPI packet I/O
//! net_task       — embassy-net stack runner
//! zenoh_task     — NodeBuilder::open(socket) → node.spin() → reconnect
//! app_task       — CHATTER_PUB.send(&msg) every 5 s; CHATTER_SUB.try_recv()
//! ```

#![no_std]
#![no_main]

mod config;

use config::AppConfig;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::{DhcpConfig, Runner as NetRunner, Stack, StackResources, tcp::TcpSocket};
use embassy_rp::{bind_interrupts, dma, peripherals::{DMA_CH0, DMA_CH1, SPI0}};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_net_wiznet::chip::W5500;
use embassy_net_wiznet::{Device as WiznetDevice, Runner as WiznetRunner, State as WiznetState};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use heapless::String;
use panic_probe as _;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use zenoh_ros2_nostd::ros2::{
    Publisher, Subscription, TopicKeyExpr,
    publisher::PublisherDrain,
    subscription::SubscriptionDispatch,
};
use zenoh_ros2_nostd::session::ReconnectPolicy;

// ── Interrupt bindings ───────────────────────────────────────────────────────

// DMA_IRQ_0 handles all DMA channels — required for async SPI.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

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

/// CDR buffer capacity for `StringMsg` (4 B header + 4 B length + 128 B data + 1 B null).
const CDR_BUF_CAP: usize = 144;

// ── Static publisher and subscriber ─────────────────────────────────────────

static CHATTER_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();

// ── Static storage for embassy ───────────────────────────────────────────────

static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
static WIZNET_STATE: StaticCell<WiznetState<8, 8>> = StaticCell::new();

// ── Type aliases to avoid long names in task signatures ──────────────────────

/// Async SPI bus on SPI0 with DMA.
type MySpi = Spi<'static, SPI0, Async>;
/// SPI bus wrapped with chip-select for use as an async SpiDevice.
type MySpiDevice = ExclusiveDevice<MySpi, Output<'static>, NoDelay>;
/// W5500 embassy runner type.
type MyWiznetRunner =
    WiznetRunner<'static, W5500, MySpiDevice, Input<'static>, Output<'static>>;

// ── Entry point ──────────────────────────────────────────────────────────────

/// Embassy entry point on RP2040.
///
/// Initialises peripherals, brings up the W5500 and networking stack, then
/// spawns the application tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // ── W5500 SPI peripheral (async + DMA) ──────────────────────────────────
    // W5500 supports up to 33.3 MHz at 3.3 V.  10 MHz is a safe starting point.
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000;

    // Async SPI uses DMA for non-blocking transfers.
    // Irqs provides the DMA interrupt handler binding required by embassy-rp 0.10.
    let spi: MySpi = Spi::new(
        p.SPI0,
        p.PIN_18,   // SCK
        p.PIN_19,   // MOSI (TX)
        p.PIN_16,   // MISO (RX)
        p.DMA_CH0,
        p.DMA_CH1,
        Irqs,       // DMA interrupt binding
        spi_cfg,
    );

    let cs: Output<'static> = Output::new(p.PIN_17, Level::High);
    let int: Input<'static> = Input::new(p.PIN_20, Pull::Up);
    let rst: Output<'static> = Output::new(p.PIN_21, Level::High);

    // Wrap SPI bus + CS into an async SpiDevice.
    // `async` feature of embedded-hal-bus enables the embedded_hal_async::spi::SpiDevice impl.
    let spi_device: MySpiDevice = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Use a locally-administered unicast MAC.  Replace with a unique value per board.
    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

    let wiznet_state = WIZNET_STATE.init(WiznetState::<8, 8>::new());

    // W5500 async init — async fn, must be .await-ed.
    // Returns Result<(Device, Runner), _> — expect at startup is appropriate.
    let (net_device, wiznet_runner) = embassy_net_wiznet::new(
        mac_addr,
        wiznet_state,
        spi_device,
        int,
        rst,
    )
    .await
    .expect("W5500 init failed");

    // Seed from a fixed constant.  Replace with hardware RNG (e.g. rosc + timer)
    // for production to avoid MAC / sequence-number collisions across reboots.
    let seed: u64 = 0x1234_5678_9abc_def0;

    let (stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    // In embassy-executor 0.10, task functions return Result<SpawnToken, SpawnError>.
    // spawner.spawn() takes a SpawnToken, so unwrap the Result first.
    spawner.spawn(ethernet_task(wiznet_runner).expect("spawn ethernet_task"));
    spawner.spawn(net_task(net_runner).expect("spawn net_task"));
    spawner.spawn(zenoh_task(stack).expect("spawn zenoh_task"));
    spawner.spawn(app_task().expect("spawn app_task"));
}

// ── Tasks ────────────────────────────────────────────────────────────────────

/// Drives W5500 SPI packet I/O — must run concurrently with net_task.
#[embassy_executor::task]
async fn ethernet_task(runner: MyWiznetRunner) {
    runner.run().await
}

/// embassy-net packet processing loop.
#[embassy_executor::task]
async fn net_task(mut runner: NetRunner<'static, WiznetDevice<'static>>) {
    runner.run().await
}

/// Zenoh session lifecycle.
///
/// 1. Wait for DHCP.
/// 2. Open a TCP connection to the Zenoh router.
/// 3. Build a `Node` via `NodeBuilder::open(socket)` — transport-agnostic.
/// 4. Register publishers and declare subscribers.
/// 5. Run `node.spin()` until the connection drops.
/// 6. Exponential back-off, then retry from step 1.
///
/// **Transport independence**: replace only the TCP socket creation block
/// (before `node_builder().open(socket)`) to use WiFi, USB CDC, or any
/// `Read + Write` transport.  Steps 3–6 remain unchanged.
#[embassy_executor::task]
async fn zenoh_task(stack: Stack<'static>) {
    let cfg = AppConfig::new();
    let mut reconnect = ReconnectPolicy::default_policy();

    loop {
        // ── Wait for DHCP ────────────────────────────────────────────────────
        wait_for_dhcp(stack).await;

        // ── TCP connect to Zenoh router ──────────────────────────────────────
        let mut tcp_rx = [0u8; 4096];
        let mut tcp_tx = [0u8; 4096];
        let mut socket = TcpSocket::new(stack, &mut tcp_rx, &mut tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(30)));

        info!("[zenoh] TCP connecting...");
        match with_timeout(
            Duration::from_secs(10),
            socket.connect(cfg.zenoh.router_endpoint()),
        )
        .await
        {
            Ok(Ok(())) => info!("[zenoh] TCP connected."),
            Ok(Err(_)) => {
                error!("[zenoh] TCP connect failed.");
                reconnect.wait_and_advance().await;
                continue;
            }
            Err(_) => {
                warn!("[zenoh] TCP connect timeout.");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        // ── Build Node: Zenoh handshake + configure pub/sub ──────────────────
        let mut node = match cfg
            .zenoh
            .session
            .node_builder()
            .name("rp2040_node")
            .open(socket)
            .await
        {
            Ok(n) => n,
            Err(e) => {
                error!("[zenoh] Handshake failed: {}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
        };

        node.register_publisher(&CHATTER_PUB as &'static dyn PublisherDrain);

        if let Err(e) = node
            .subscribe(CHATTER_TOPIC, &CHATTER_SUB as &'static dyn SubscriptionDispatch)
            .await
        {
            error!("[zenoh] Subscribe failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        reconnect.reset();
        info!("[zenoh] Node '{}' ready.", node.node_name());

        // ── Session loop — returns only when the connection drops ────────────
        let mut rx_buf = [0u8; 4096];
        node.spin(&mut rx_buf).await;

        warn!("[zenoh] Session ended — reconnecting.");
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
            core::format_args!("Hello from RP2040! count={}", counter),
        );
        counter += 1;

        match CHATTER_PUB.send(&StringMsg { data }).await {
            Ok(()) => {}
            Err(e) => error!("[app] Publish error: {}", e),
        }

        // ── Receive ─────────────────────────────────────────────────────────
        while let Some(result) = CHATTER_SUB.try_recv() {
            match result {
                Ok(m) => info!("[app] /chatter: {=[u8]}", m.data.as_bytes()),
                Err(e) => warn!("[app] Deserialize error: {}", e),
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Poll until embassy-net has an assigned IPv4 address.
async fn wait_for_dhcp(stack: Stack<'_>) {
    while stack.config_v4().is_none() {
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("[net] DHCP acquired — node is online.");
}
