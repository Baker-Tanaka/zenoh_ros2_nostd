# esp32c3-wifi example

ESP32-C3 Wi-Fi connectivity demo using embassy + esp-radio.

## Wi-Fi credentials setup

SSID とパスワードはビルド時に `wifi_config.json` から読み込まれ、バイナリに埋め込まれます。
このファイルは git 管理外です。

### 1. 設定ファイルを作成する

```sh
cp wifi_config.json.example wifi_config.json
```

### 2. `wifi_config.json` を編集する

```json
{
  "ssid": "YourNetworkName",
  "password": "YourPassword"
}
```

| フィールド | 型     | 説明                               |
| ---------- | ------ | ---------------------------------- |
| `ssid`     | string | 接続先 Wi-Fi ネットワーク名 (SSID) |
| `password` | string | Wi-Fi パスワード (空文字列も可)    |

> **注意**: `wifi_config.json` には認証情報が含まれます。`.gitignore` によって git 管理から除外されていますが、リポジトリに誤ってコミットしないよう注意してください。

## Build & run

probe-rs JTAG プローブを接続した状態で:

```sh
cd examples/esp32c3_wifi
cargo run --release
```

RTT ターミナルに接続状態が表示されます。切断時は 5 秒後に自動リトライします。

## Hardware

- ESP32-C3 (any revision)
- probe-rs 対応 JTAG プローブ (GPIO4=TCK, GPIO5=TDI, GPIO6=TDO, GPIO7=TMS)
