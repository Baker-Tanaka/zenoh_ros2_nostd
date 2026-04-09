//! RP2040 + W5500 Ethernet  ROS2 topic pub/sub demo.
//!
//! Connects to a Zenoh router over W5500 Ethernet, then publishes
//! `std_msgs/String` on `/chatter` every 5 seconds and echoes any
//! received `/chatter` messages via defmt RTT.
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
use embassy_net::{tcp::TcpSocket, DhcpConfig, Runner as NetRunner, Stack, StackResources};
use embassy_net_wiznet::chip::W5500;
use embassy_net_wiznet::{Device as WiznetDevice, Runner as WiznetRunner, State as WiznetState};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::{
    bind_interrupts, dma,
    peripherals::{DMA_CH0, DMA_CH1, SPI0},
};
use embassy_time::{with_timeout, Duration, Timer};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use heapless::String;
use panic_probe as _;
use portable_atomic::{AtomicU32, Ordering};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use zenoh_ros2_nostd::cdr::cdr_cap_for_string;
use zenoh_ros2_nostd::ros2::{msg, Publisher, ReconnectPolicy, Subscription};

// DMA_IRQ_0 handles all DMA channels — required for async SPI.
bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

/// Key expression for `std_msgs/String` on `/chatter`.
///
/// Built from the verified `msg::std_msgs::String` constants — no hand-typed
/// type hash or type name; typos are caught at compile time.
const CHATTER_TOPIC: zenoh_ros2_nostd::ros2::TopicKeyExpr = msg::std_msgs::String::CHATTER;

/// `std_msgs/String` message type.
#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String<128>,
}

/// CDR buffer capacity for `StringMsg` computed from the string field size.
///
/// `cdr_cap_for_string(128)` = 4 (CDR header) + 4 (length) + 128 (data) + 1 (null) = 137.
/// The constant eliminates manual byte-count arithmetic and is updated automatically
/// if the string size changes.
const CDR_BUF_CAP: usize = cdr_cap_for_string(128);

static CHATTER_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();

/// Number of panics recorded since power-on (RAM counter — resets on power-off).
///
/// This counter is **not** incremented automatically by `panic-probe` (which
/// provides the `#[panic_handler]` and calls `cortex_m::asm::udf()` directly).
/// To make use of it, replace `panic-probe` with a custom panic handler:
///
/// ```rust,ignore
/// // Custom panic handler (remove panic-probe from Cargo.toml first):
/// #[panic_handler]
/// fn panic_handler(info: &core::panic::PanicInfo) -> ! {
///     PANIC_COUNT.fetch_add(1, Ordering::Relaxed);
///     // (Optional) write to RP2040 watchdog scratch register for persistence across resets.
///     defmt::error!("PANIC: {}", defmt::Debug2Format(info));
///     cortex_m::asm::udf()
/// }
/// ```
///
/// Even without a custom handler, the startup log warns if `PANIC_COUNT > 0`,
/// providing visibility when probe-rs **is** attached.
static PANIC_COUNT: AtomicU32 = AtomicU32::new(0);

/// Async SPI bus on SPI0 with DMA.
type MySpi = Spi<'static, SPI0, Async>;
/// SPI bus wrapped with chip-select for use as an async SpiDevice.
type MySpiDevice = ExclusiveDevice<MySpi, Output<'static>, NoDelay>;
/// W5500 embassy runner type.
type MyWiznetRunner = WiznetRunner<'static, W5500, MySpiDevice, Input<'static>, Output<'static>>;

/// Embassy entry point on RP2040.
///
/// Initialises peripherals, brings up the W5500 and networking stack, then
/// spawns the application tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    static WIZNET_STATE: StaticCell<WiznetState<8, 8>> = StaticCell::new();

    let p = embassy_rp::init(Default::default());

    // Report the panic count from the previous run.  A non-zero count means
    // the firmware hit a `panic!` (or assertion) and reset.  When probe-rs is
    // not attached the panic message is lost; this counter provides a visible
    // signal that something went wrong.
    let panics = PANIC_COUNT.load(Ordering::Relaxed);
    if panics > 0 {
        warn!(
            "[main] ⚠️  {} panic(s) recorded since last power-on.",
            panics
        );
    } else {
        info!("[main] Starting — no panics recorded.");
    }

    // ── W5500 SPI peripheral (async + DMA) ──────────────────────────────────
    // W5500 supports up to 33.3 MHz at 3.3 V.  10 MHz is a safe starting point.
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000;

    // Async SPI uses DMA for non-blocking transfers.
    // Irqs provides the DMA interrupt handler binding required by embassy-rp 0.10.
    let spi: MySpi = Spi::new(
        p.SPI0, p.PIN_18, // SCK
        p.PIN_19, // MOSI (TX)
        p.PIN_16, // MISO (RX)
        p.DMA_CH0, p.DMA_CH1, Irqs, // DMA interrupt binding
        spi_cfg,
    );

    let cs: Output<'static> = Output::new(p.PIN_17, Level::High);
    let int: Input<'static> = Input::new(p.PIN_15, Pull::Up);
    let rst: Output<'static> = Output::new(p.PIN_14, Level::High);

    // Wrap SPI bus + CS into an async SpiDevice.
    // `async` feature of embedded-hal-bus enables the embedded_hal_async::spi::SpiDevice impl.
    let spi_device: MySpiDevice = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // ⚠️ Use a unique MAC per board!  Identical MACs on the same network cause
    // ARP conflicts and intermittent connectivity.  Derive from RP2040's unique ID
    // (accessible via the QSPI `FLASH_RUID_CMD` or the 64-bit UID at address 0x40130084).
    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01]; // REPLACE with board-unique value

    let wiznet_state = WIZNET_STATE.init(WiznetState::<8, 8>::new());

    // W5500 async init — async fn, must be .await-ed.
    // Returns Result<(Device, Runner), _> — expect at startup is appropriate.
    let (net_device, wiznet_runner) =
        embassy_net_wiznet::new(mac_addr, wiznet_state, spi_device, int, rst)
            .await
            .expect("W5500 init failed");

    // ⚠️ Fixed seed — predictable TCP sequence numbers and port choices.
    // For production, derive entropy from RP2040's ring oscillator (ROSC) or
    // chip UID to prevent collisions when multiple identical firmware images run.
    let seed: u64 = 0x1234_5678_9abc_def0; // REPLACE with hardware-derived entropy

    let (stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(ethernet_task(wiznet_runner).expect("spawn ethernet_task"));
    spawner.spawn(net_task(net_runner).expect("spawn net_task"));
    spawner.spawn(zenoh_task(stack).expect("spawn zenoh_task"));
    spawner.spawn(app_task().expect("spawn app_task"));
}

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
    static TCP_RX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
    static TCP_TX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
    static ZENOH_RX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();

    let cfg = AppConfig::new();
    let mut reconnect = ReconnectPolicy::default_policy();
    let builder = cfg.zenoh.session.node_builder().name("rp2040_node");

    let tcp_rx = TCP_RX_BUF.init([0u8; 4096]);
    let tcp_tx = TCP_TX_BUF.init([0u8; 4096]);
    let rx_buf = ZENOH_RX_BUF.init([0u8; 4096]);

    loop {
        wait_for_dhcp(stack).await;

        let mut socket = TcpSocket::new(stack, tcp_rx, tcp_tx);
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

        let mut node = match builder.open(socket).await {
            Ok(n) => n,
            Err(e) => {
                error!("[zenoh] Handshake failed: {}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
        };

        node.register_publisher(CHATTER_PUB.as_drain());

        // Clear stale messages from the previous session before re-subscribing.
        // Without this, messages received before the disconnect would be delivered
        // to the application after reconnect — out-of-context and potentially stale.
        CHATTER_SUB.clear();

        if let Err(e) = node
            .subscribe(CHATTER_TOPIC, CHATTER_SUB.as_dispatch())
            .await
        {
            error!("[zenoh] Subscribe failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        info!("[zenoh] Node '{}' ready.", node.node_name());

        node.spin_and_backoff(rx_buf, &mut reconnect).await;
        warn!(
            "[zenoh] Session ended — reconnecting (attempt #{}).",
            reconnect.attempt()
        );
    }
}

/// Application logic: publish a counter message every 5 s; echo received messages.
///
/// Publish failures are counted and logged.  A non-zero drop count at runtime
/// indicates that the zenoh_task is not draining the publisher queue fast enough,
/// or that the Zenoh session is disconnected.  The `send()` API blocks when the
/// queue is full, so in normal operation `drop_count` stays at 0.
#[embassy_executor::task]
async fn app_task() {
    let mut counter: u32 = 0;
    let mut drop_count: u32 = 0;
    loop {
        let mut data: String<128> = String::new();
        let _ = core::fmt::write(
            &mut data,
            core::format_args!("Hello from RP2040! count={}", counter),
        );
        counter += 1;

        match CHATTER_PUB.send(&StringMsg { data }).await {
            Ok(()) => {}
            Err(e) => {
                drop_count += 1;
                // Log every failure.  `send()` awaits until there is queue space,
                // so this error only fires on CDR serialization failure (which
                // should not happen for well-typed messages).  If you see drops
                // here, check that CDR_BUF_CAP is large enough for the message.
                error!("[app] Publish error (total drops={}): {}", drop_count, e);
            }
        }

        while let Some(result) = CHATTER_SUB.try_recv() {
            match result {
                Ok(m) => info!("[app] /chatter: {=[u8]}", m.data.as_bytes()),
                Err(e) => warn!("[app] Deserialize error: {}", e),
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

/// Poll until embassy-net has an assigned IPv4 address.
async fn wait_for_dhcp(stack: Stack<'_>) {
    while stack.config_v4().is_none() {
        Timer::after(Duration::from_millis(500)).await;
    }
    info!("[net] DHCP acquired — node is online.");
}
