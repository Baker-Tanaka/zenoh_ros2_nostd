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
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::{Async, Config as SpiConfig, Phase, Polarity, Spi};
use embassy_rp::{
    bind_interrupts, dma,
    peripherals::{ADC, ADC_TEMP_SENSOR, DMA_CH0, DMA_CH1, SPI0},
    Peri,
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
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
});

const STATUS_TOPIC: TopicKeyExpr = msg::std_msgs::String::topic(0, "baker_link/status");
const TEMP_TOPIC: TopicKeyExpr = msg::std_msgs::Float32Type::topic(0, "baker_link/cpu_temp");
const ADC_RAW_TOPIC: TopicKeyExpr = msg::std_msgs::Int32Type::topic(0, "baker_link/cpu_temp_raw");
const ROSOUT_TOPIC: TopicKeyExpr = msg::rcl_interfaces::LogType::ROSOUT;
const CDR_BUF_CAP: usize = cdr_cap_for_string(128);

#[derive(Serialize, Deserialize, Debug)]
struct StringMsg {
    data: String<128>,
}

static STATUS_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(STATUS_TOPIC);
static STATUS_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();
static TEMP_PUB: Publisher<msg::std_msgs::Float32Msg, 8, 4> = Publisher::new(TEMP_TOPIC);
static ADC_RAW_PUB: Publisher<msg::std_msgs::Int32Msg, 8, 4> = Publisher::new(ADC_RAW_TOPIC);
static ROSOUT_SUB: Subscription<msg::rcl_interfaces::Log, { msg::rcl_interfaces::LOG_CDR_CAP }, 4> =
    Subscription::new();

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
    spawner.spawn(app_task(p.ADC, p.ADC_TEMP_SENSOR).unwrap());
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

    let router_ep = cfg.zenoh.router_endpoint();
    info!(
        "[net] router target = {}.{}.{}.{}:{}",
        cfg.zenoh.router_ip[0],
        cfg.zenoh.router_ip[1],
        cfg.zenoh.router_ip[2],
        cfg.zenoh.router_ip[3],
        cfg.zenoh.router_port,
    );

    loop {
        while stack.config_v4().is_none() {
            Timer::after(Duration::from_millis(500)).await;
        }
        if let Some(ip_cfg) = stack.config_v4() {
            let addr = ip_cfg.address.address().octets();
            let gw = ip_cfg.gateway.map(|g| g.octets()).unwrap_or([0, 0, 0, 0]);
            info!(
                "[net] DHCP OK — IP {}.{}.{}.{} GW {}.{}.{}.{}",
                addr[0], addr[1], addr[2], addr[3], gw[0], gw[1], gw[2], gw[3],
            );
        }

        let mut socket = TcpSocket::new(stack, tcp_rx, tcp_tx);
        socket.set_timeout(Some(Duration::from_secs(30)));

        match with_timeout(Duration::from_secs(10), socket.connect(router_ep)).await {
            Ok(Ok(())) => info!("[zenoh] connected"),
            Ok(Err(e)) => {
                match e {
                    embassy_net::tcp::ConnectError::InvalidState => {
                        error!("[zenoh] connect failed: InvalidState")
                    }
                    embassy_net::tcp::ConnectError::ConnectionReset => {
                        error!("[zenoh] connect failed: ConnectionReset (RST)")
                    }
                    embassy_net::tcp::ConnectError::TimedOut => {
                        error!("[zenoh] connect failed: TimedOut")
                    }
                    embassy_net::tcp::ConnectError::NoRoute => {
                        error!("[zenoh] connect failed: NoRoute")
                    }
                }
                reconnect.wait_and_advance().await;
                continue;
            }
            Err(_) => {
                warn!("[zenoh] connect timeout — is router reachable?");
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

        if let Err(e) = node.create_publisher(&STATUS_PUB).await {
            error!("[zenoh] publisher reg failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        if let Err(e) = node.create_publisher(&TEMP_PUB).await {
            error!("[zenoh] publisher reg failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        if let Err(e) = node.create_publisher(&ADC_RAW_PUB).await {
            error!("[zenoh] publisher reg failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        STATUS_SUB.clear();

        if let Err(e) = node.create_subscription(STATUS_TOPIC, &STATUS_SUB).await {
            error!("[zenoh] subscribe failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        ROSOUT_SUB.clear();

        if let Err(e) = node
            .create_subscription(ROSOUT_TOPIC, &ROSOUT_SUB)
            .await
        {
            error!("[zenoh] rosout subscribe failed: {}", e);
            reconnect.wait_and_advance().await;
            continue;
        }

        info!("[zenoh] node ready");
        node.spin_and_backoff(&mut reconnect).await;
        warn!("[zenoh] disconnected, reconnecting...");
    }
}

#[embassy_executor::task]
async fn app_task(adc_peri: Peri<'static, ADC>, temp_sensor_peri: Peri<'static, ADC_TEMP_SENSOR>) {
    let mut adc = Adc::new(adc_peri, Irqs, AdcConfig::default());
    let mut temp_ch = Channel::new_temp_sensor(temp_sensor_peri);
    let mut counter: u32 = 0;
    loop {
        let mut data: String<128> = String::new();
        let _ = core::fmt::write(
            &mut data,
            core::format_args!("Baker link.dev heartbeat count={}", counter),
        );
        counter += 1;

        if let Err(e) = STATUS_PUB.send(&StringMsg { data }).await {
            error!("[app] status publish error: {}", e);
        }

        // Read RP2040 internal temperature sensor (ADC channel 4).
        // Formula: T(°C) = 27 - (V_adc - 0.706) / 0.001721, V_adc = raw * 3.3 / 4096
        let raw = adc.read(&mut temp_ch).await.unwrap_or(0);
        let temp_c = 27.0f32 - (raw as f32 * 3.3 / 4096.0 - 0.706) / 0.001721;

        if let Err(e) = TEMP_PUB
            .send(&msg::std_msgs::Float32Msg { data: temp_c })
            .await
        {
            error!("[app] cpu_temp publish error: {}", e);
        }

        if let Err(e) = ADC_RAW_PUB
            .send(&msg::std_msgs::Int32Msg { data: raw as i32 })
            .await
        {
            error!("[app] cpu_temp_raw publish error: {}", e);
        }

        info!("[app] cpu_temp={} raw={}", temp_c, raw);

        while let Some(result) = STATUS_SUB.try_recv() {
            match result {
                Ok(m) => info!("[app] /baker_link/status: {=str}", m.data.as_str()),
                Err(e) => warn!("[app] status deserialize error: {}", e),
            }
        }

        while let Some(result) = ROSOUT_SUB.try_recv() {
            match result {
                Ok(m) => info!("[rosout] [{=str}] {=str}", m.name.as_str(), m.msg.as_str()),
                Err(e) => warn!("[app] rosout deserialize error: {}", e),
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}
