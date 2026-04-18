//! Baker link.dev (RP2040) + WIZ630io (W6300) — ROS2 pub/sub over Zenoh.
//!
//! WIZ630io uses W6300. `embassy-net-wiznet` v0.3 supports W6300 in Single SPI mode.

#![no_std]
#![no_main]

mod config;

use config::AppConfig;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::{tcp::TcpSocket, DhcpConfig, Runner as NetRunner, Stack, StackResources};
use embassy_net_wiznet::chip::W6300;
use embassy_net_wiznet::{Device as WiznetDevice, Runner as WiznetRunner, State as WiznetState};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Async, Config as SpiConfig, Phase, Polarity, Spi};
use embassy_rp::{
    bind_interrupts, dma,
    peripherals::{DMA_CH0, DMA_CH1, SPI0},
};
use embassy_time::{with_timeout, Duration, Timer};
use embedded_hal_async::spi::SpiDevice as _;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use heapless::String;
use panic_probe as _;
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;
use zenoh_ros2_nostd::cdr::cdr_cap_for_string;
use zenoh_ros2_nostd::prelude::*;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::CHATTER;
const CDR_BUF_CAP: usize = cdr_cap_for_string(128);

#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String<128>,
}

static CHATTER_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();

type MySpi = Spi<'static, SPI0, Async>;
type MySpiDevice = ExclusiveDevice<MySpi, Output<'static>, NoDelay>;
type MyWiznetRunner = WiznetRunner<'static, W6300, MySpiDevice, Input<'static>, Output<'static>>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    static WIZNET_STATE: StaticCell<WiznetState<4, 4>> = StaticCell::new();

    let p = embassy_rp::init(Default::default());

    defmt::trace!("RTT init");
    Timer::after(Duration::from_millis(500)).await;
    info!("[main] start");

    // SPI0: Mode 3, 1 MHz
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 1_000_000;
    spi_cfg.polarity = Polarity::IdleHigh;
    spi_cfg.phase = Phase::CaptureOnSecondTransition;

    let spi: MySpi = Spi::new(
        p.SPI0, p.PIN_18, // SCK
        p.PIN_19, // MOSI
        p.PIN_16, // MISO
        p.DMA_CH0, p.DMA_CH1, Irqs, spi_cfg,
    );

    let cs = Output::new(p.PIN_17, Level::High);
    let int = Input::new(p.PIN_15, Pull::Up);
    let mut rst = Output::new(p.PIN_14, Level::High);

    // Hardware reset: assert 10 ms, release, wait 150 ms
    rst.set_low();
    Timer::after(Duration::from_millis(10)).await;
    rst.set_high();
    Timer::after(Duration::from_millis(150)).await;

    let mut spi_dev: MySpiDevice = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // SPI smoke test: read W6300 CIDR2 (Common Register 0x0004).
    // W6300 SPI frame: [instruction, addr_hi, addr_lo, dummy, data_read]
    //   instruction = 0x00 (Common block, Read)
    //   address     = 0x0004 (CIDR2 — Minor Chip ID)
    //   Expected data_read (buf[4]) = 0x11 for W6300.
    {
        let mut buf = [0x00u8, 0x00, 0x04, 0x00, 0x00];
        match spi_dev.transfer_in_place(&mut buf).await {
            Ok(()) => {
                let version = buf[4];
                if version == 0x11 {
                    info!("[spi] W6300 CIDR2 = {:#04x} — SPI OK", version);
                } else {
                    error!(
                        "[spi] W6300 CIDR2 = {:#04x}, expected 0x11 — check chip/wiring",
                        version
                    );
                }
            }
            Err(_) => {
                error!("[spi] SPI transfer_in_place failed — check wiring");
            }
        }
    }

    let mac_addr = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
    let wiznet_state = WIZNET_STATE.init(WiznetState::<4, 4>::new());

    let (net_device, wiznet_runner) =
        embassy_net_wiznet::new(mac_addr, wiznet_state, spi_dev, int, rst)
            .await
            .unwrap();

    let (stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        STACK_RESOURCES.init(StackResources::new()),
        0x1234_5678_9abc_def0,
    );

    spawner.spawn(ethernet_task(wiznet_runner).unwrap());
    spawner.spawn(net_task(net_runner).unwrap());
    spawner.spawn(zenoh_task(stack).unwrap());
    spawner.spawn(app_task().unwrap());
}

#[embassy_executor::task]
async fn ethernet_task(runner: MyWiznetRunner) {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner<'static, WiznetDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn zenoh_task(stack: Stack<'static>) {
    static TCP_RX: StaticCell<[u8; 4096]> = StaticCell::new();
    static TCP_TX: StaticCell<[u8; 4096]> = StaticCell::new();

    let cfg = AppConfig::new();
    let mut reconnect = ReconnectPolicy::default_policy();
    let tcp_rx = TCP_RX.init([0u8; 4096]);
    let tcp_tx = TCP_TX.init([0u8; 4096]);

    loop {
        while stack.config_v4().is_none() {
            Timer::after(Duration::from_millis(500)).await;
        }
        info!("[net] DHCP OK");

        let mut socket = TcpSocket::new(stack, tcp_rx, tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(30)));

        match with_timeout(
            Duration::from_secs(10),
            socket.connect(cfg.zenoh.router_endpoint()),
        )
        .await
        {
            Ok(Ok(())) => info!("[zenoh] connected"),
            Ok(Err(_)) => {
                error!("[zenoh] connect failed");
                reconnect.wait_and_advance().await;
                continue;
            }
            Err(_) => {
                warn!("[zenoh] connect timeout");
                reconnect.wait_and_advance().await;
                continue;
            }
        }

        let mut node = match NodeBuilder::new("bakerlink_node")
            .zid(cfg.zenoh.session.zid)
            .domain_id(cfg.zenoh.session.domain_id)
            .build(socket)
            .await
        {
            Ok(n) => n,
            Err(e) => {
                error!("[zenoh] handshake failed: {}", e);
                reconnect.wait_and_advance().await;
                continue;
            }
        };

        if let Err(e) = node.register_static_publisher(&CHATTER_PUB).await {
            error!("[zenoh] publisher reg failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        CHATTER_SUB.clear();

        if let Err(e) = node
            .subscribe_with_dispatch(CHATTER_TOPIC, &CHATTER_SUB)
            .await
        {
            error!("[zenoh] subscribe failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        info!("[zenoh] node ready");
        node.spin_and_backoff(&mut reconnect).await;
        warn!("[zenoh] disconnected, reconnecting...");
    }
}

#[embassy_executor::task]
async fn app_task() {
    let mut counter: u32 = 0;
    loop {
        let mut data: String<128> = String::new();
        let _ = core::fmt::write(
            &mut data,
            core::format_args!("Hello from Baker link.dev! count={}", counter),
        );
        counter += 1;

        if let Err(e) = CHATTER_PUB.send(&StringMsg { data }).await {
            error!("[app] publish error: {}", e);
        }

        while let Some(result) = CHATTER_SUB.try_recv() {
            match result {
                Ok(m) => info!("[app] /chatter: {=[u8]}", m.data.as_bytes()),
                Err(e) => warn!("[app] deserialize error: {}", e),
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}
