# W5100S-EVB-Pico2 — ROS2 pub/sub over Ethernet

[W5100S-EVB-Pico2](https://docs.wiznet.io/Product/iEthernet/W5100S/w5100s-evb-pico2) (RP2350 + W5100S) の SPI Ethernet で Docker 上の ROS2 (`rmw_zenoh_cpp`) と `/chatter` トピックを pub/sub するサンプルです。

## クイックスタート

### ターミナル 1: Python 検証サブスクライバ（dev container 内）

```sh
cd examples/w5100s_evb_pico2
python3 verify_sub.py
```

### ターミナル 2: ファームウェア書き込み（ホスト側）

```sh
cd examples/w5100s_evb_pico2
cargo run --release
```

### 期待される出力

**verify_sub.py:**
```
[PY-SUB] Subscribed to /chatter — waiting for messages…
[  1] /chatter: "Hello from Pico2! count=0"
[  2] /chatter: "Hello from Pico2! count=1"
[  3] /chatter: "Hello from Pico2! count=2"
```

**probe-rs RTT（ホスト側）:**
```
[main] Starting — no panics recorded.
[net]  DHCP acquired — node is online.
[zenoh] TCP connected.
[zenoh] Node 'pico2_node' ready.
```

## ネットワーク構成

```text
W5100S-EVB-Pico2 ──Ethernet──► ホスト PC (:7447 port forward)
                                   │
                              Docker network
                                   │
                     ┌─────────────┴──────────────┐
                     │                             │
               Zenoh router              ROS2 Node (listener)
               (rmw_zenohd :7447)        (rmw_zenoh_cpp)
```

- デバイスはホスト PC の **LAN IP** + ポート 7447 に接続
- Docker compose が `7447:7447` でポートフォワード
- `config.json` の `router_addr` にホスト PC の LAN IP を設定

## Hardware

| RP2350 GPIO          | W5100S ピン | 用途                  |
|----------------------|------------|-----------------------|
| GP16 (SPI0 RX/MISO) | MISO       | SPI データ入力        |
| GP17 (output)        | SCS        | チップセレクト        |
| GP18 (SPI0 SCK)      | SCLK       | SPI クロック          |
| GP19 (SPI0 TX/MOSI)  | MOSI       | SPI データ出力        |
| GP20 (output)        | RSTn       | リセット (active-low) |
| GP21 (input)         | INTn       | 割り込み (active-low) |

## セットアップ

### 1. Docker で ROS2 + Zenoh ルーターを起動

```sh
cd <project_root>
docker compose up -d
```

### 2. 設定ファイルを作成

```sh
cd examples/w5100s_evb_pico2
cp config.json.example config.json
```

`config.json` の `router_addr` を **ホスト PC の LAN IP** に設定:

```json
{
  "router_addr": "192.168.1.100:7447"
}
```

> **確認方法**: ホスト上で `ip addr show` (Linux) / `ipconfig` (Windows) / `ifconfig` (macOS) で Ethernet/WiFi インタフェースの IPv4 アドレスを確認。

### 3. probe-rs でビルド＋書き込み（ホスト側）

```sh
cargo run --release
```

> ホストに [probe-rs](https://probe.rs/docs/getting-started/installation/) と
> `thumbv8m.main-none-eabihf` ターゲットが必要:
> ```sh
> curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
> rustup target add thumbv8m.main-none-eabihf
> ```

### 4. 動作確認

dev container 内で Python サブスクライバを実行して、デバイスからのメッセージを確認:

```sh
python3 verify_sub.py
```

または ROS2 コマンドで確認:

```sh
# ros2-node コンテナに入って
docker exec -it ros2-node bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic echo /chatter std_msgs/msg/String
```

## Notes

- `config.json` は `router_addr` のみ設定すれば動作します。
- ルーター到達性確認は `nc -vz <router_ip> 7447` でも確認できます。
- 実機での複数台運用時は、`src/main.rs` の MAC と ZID をボード固有値に変更してください。
