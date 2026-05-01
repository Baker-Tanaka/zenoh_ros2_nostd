# Baker link. Dev + WIZ630io — ROS2 pub/sub over Ethernet

[Baker link. Dev](https://github.com/Baker-link-Lab) (RP2040) と [WIZ630io](https://www.wiznet.io/) (W5500) を SPI 接続し、Docker 上の ROS2 (`rmw_zenoh_cpp`) と `/chatter` トピックを pub/sub するサンプルです。

## クイックスタート

### ターミナル 1: Python 検証サブスクライバ（dev container 内）

```sh
cd examples/bakerlink_wiz630io
python3 verify_sub.py
```

### ターミナル 2: ファームウェア書き込み（ホスト側）

```sh
cd examples/bakerlink_wiz630io
cargo run --release
```

### 期待される出力

**verify_sub.py:**
```
[PY-SUB] Subscribed to /chatter — waiting for messages…
[  1] /chatter: "Hello from Baker link. Dev! count=0"
[  2] /chatter: "Hello from Baker link. Dev! count=1"
[  3] /chatter: "Hello from Baker link. Dev! count=2"
```

**probe-rs RTT（ホスト側）:**
```
[main] Starting — no panics recorded.
[net]  DHCP acquired — node is online.
[zenoh] TCP connected.
[zenoh] Node 'bakerlink_node' ready.
```

## ネットワーク構成

```text
Baker link. Dev + WIZ630io ──Ethernet──► ホスト PC (:7447 port forward)
                                            │
                                       Docker network
                                            │
                              ┌─────────────┴──────────────┐
                              │                            │
                        Zenoh router              ROS2 Node (listener)
                        (rmw_zenohd :7447)        (rmw_zenoh_cpp)
```

- デバイスはホスト PC の **LAN IP** + ポート 7447 に接続
- Docker compose が `7447:7447` でポートフォワード
- `config.json` の `router_addr` にホスト PC の LAN IP を設定

## 配線図

### Baker link. Dev (RP2040) ↔ WIZ630io (W5500) SPI 接続

```text
  Baker link. Dev (RP2040)              WIZ630io (W5500)
 ┌─────────────────────┐             ┌──────────────────┐
 │                     │             │                  │
 │   GP16 (SPI0 MISO) ─┼─────────────┼─ MISO            │
 │   GP17 (GPIO out)  ─┼─────────────┼─ SCSn            │
 │   GP18 (SPI0 SCK)  ─┼─────────────┼─ SCLK            │
 │   GP19 (SPI0 MOSI) ─┼─────────────┼─ MOSI            │
 │   GP14 (GPIO out)  ─┼─────────────┼─ RSTn            │
 │   GP15 (GPIO in)   ─┼─────────────┼─ INTn            │
 │                     │             │                  │
 │   3V3 (OUT)        ─┼─────────────┼─ 3.3V            │
 │   GND              ─┼─────────────┼─ GND             │
 │                     │             │                  │
 └─────────────────────┘             └──────────────────┘
                                       │  RJ45 ├──► LAN
```

### ピンアサイン表

| RP2040 GPIO         | WIZ630io ピン | 用途                  |
| ------------------- | ------------- | --------------------- |
| GP16 (SPI0 RX/MISO) | MISO          | SPI データ入力        |
| GP17 (output)       | SCSn          | チップセレクト        |
| GP18 (SPI0 SCK)     | SCLK          | SPI クロック          |
| GP19 (SPI0 TX/MOSI) | MOSI          | SPI データ出力        |
| GP14 (output)       | RSTn          | リセット (active-low) |
| GP15 (input)        | INTn          | 割り込み (active-low) |
| 3V3 (OUT)           | 3.3V          | 電源                  |
| GND                 | GND           | グランド              |

![](https://docs.wiznet.io/img/products/wiz630io/WIZ630io_pin_out_2.png)

![](https://www.baker-link.com/wp-content/uploads/2024/11/PinAssign_NoRunPinExternalDebug-1024x587.png)

### 注意事項

- **電源**: WIZ630io は 3.3V 動作。Baker link. Dev の 3V3 出力ピンから給電
- **SPI クロック**: 10 MHz（安全なデフォルト値。W5500 は最大 80 MHz 対応）
- **プルアップ**: INTn は内部プルアップ有効化済み（ファームウェア設定）
- **MAC アドレス**: `src/main.rs` 内の `mac_addr` をボードごとにユニークな値に変更すること

## セットアップ

### 1. Docker で ROS2 + Zenoh ルーターを起動

```sh
cd <project_root>
docker compose up -d
```

### 2. 設定ファイルを作成

```sh
cd examples/bakerlink_wiz630io
```

`config.json` の `router_addr` を **ホスト PC の LAN IP** に設定:

```json
{
  "router_addr": "192.168.1.100:7447"
}
```

> `localhost` は使えません — デバイスから見たルーターのIPアドレスを指定してください。

### 3. ファームウェア書き込み

Baker link. Dev を USB で接続し、もう一台の Baker link. Dev（デバッガ側）経由で書き込みます:

```sh
cargo run --release
```

### 4. 動作確認

```sh
# dev container 内で
python3 verify_sub.py

# または ROS2 CLI で
ros2 topic echo /chatter std_msgs/msg/String
```

## ハードウェア仕様

### Baker link. Dev

- **MCU**: RP2040 (Cortex-M0+, 133 MHz dual-core)
- **Flash**: 2 MB (外部 QSPI)
- **RAM**: 264 KB SRAM
- **デバッガ**: 内蔵 CMSIS-DAP (probe-rs 対応)
- **詳細**: [Baker-link-Lab/baker-link-dev](https://github.com/Baker-link-Lab/baker-link-dev)

### WIZ630io

- **チップ**: W5500 (ハードウェア TCP/IP スタック内蔵)
- **インターフェース**: SPI (最大 80 MHz)
- **Ethernet**: 10/100 Mbps、RJ45 コネクタ・マグネティクス内蔵
- **ソケット**: 8 個の独立したハードウェアソケット
- **動作電圧**: 3.3V

## トラブルシューティング

| 症状                   | 対処法                                                              |
| ---------------------- | ------------------------------------------------------------------- |
| RTT 出力が表示されない | probe-rs のバージョンを確認。500ms の初期化待機で解決することが多い |
| DHCP が取得できない    | Ethernet ケーブルの接続を確認。WIZ630io の LED が点灯しているか確認 |
| TCP 接続がタイムアウト | `config.json` の `router_addr` が正しいか確認                       |
| パブリッシュが失敗する | Zenoh ルーターが起動しているか確認 (`docker compose ps`)            |
| W5500 init failed      | SPI 配線を確認。GP20 (RSTn) が正しく接続されているか確認            |


## Windows11 ファイヤーウォール設定
Zenoh Routerを起動しているホストマシンがWindows11の場合は、以下のコマンドを実行してファイヤーウォールの設定をしてください。
```powershell
New-NetFirewallRule -DisplayName "Zenoh Router 7447" `
    -Direction Inbound `
    -Protocol TCP `
    -LocalPort 7447 `
    -Action Allow
```
