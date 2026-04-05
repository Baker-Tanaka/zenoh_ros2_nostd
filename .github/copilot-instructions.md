# Project Guidelines — zenoh-ros2-nostd

## Overview

組み込みMCU向け `no_std` ROS2トピックpub/subライブラリ。Zenohプロトコル v8 上でCDRシリアライゼーションを用い、`rmw_zenoh_cpp` が動作するROS2ノードと通信する。

## ROS2 + rmw_zenoh_cpp 通信アーキテクチャ

本クレートは、ROSの**RMW (ROS Middleware)** としてZenohを使う `rmw_zenoh_cpp` と互換性を持つ形で通信する。

### ネットワークトポロジー

```
┌──────────────┐       ┌──────────────────┐       ┌──────────────┐
│ MCU (Client) │──TCP──│ Zenoh Router     │──TCP──│ ROS2 Node    │
│ this crate   │       │ zenohd :7447     │       │ rmw_zenoh_cpp│
│ WhatAmI:     │       │                  │       │ mode: peer   │
│   Client     │       │ discovery +      │       │ connect:     │
│ connect:     │       │ gossip scouting  │       │  localhost:  │
│  router:7447 │       │                  │       │    7447      │
└──────────────┘       └──────────────────┘       └──────────────┘
```

- **Zenoh Router** (`zenohd` / `rmw_zenohd`): ポート7447でリッスン。ディスカバリのハブとして機能
- **rmw_zenoh_cpp セッション**: デフォルトで **peer** モード。`tcp/localhost:7447` でルーターに接続、gossip scoutingによるP2P接続
- **本クレート (MCU)**: **client** モード。ルーター経由で全データを送受信（MCUはP2P不可のため）
- **データフロー**: MCU→Router→ROS2 peer / ROS2 peer→Router→MCU

### rmw_zenoh_cpp の仕様 (design.md 準拠)

- **シリアライゼーション**: CDR (Common Data Representation)
- **Key Expression**: `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`
  - type_name は **DDS 慣例**: `std_msgs::msg::dds_::String_` (`dds_::` プレフィックス + `_` サフィックス)
- **Liveliness Token**: `@ros2_lv/<domain_id>/<session_id>/<node_id>/<entity_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>/<mangled_topic>/<type_name>/<type_hash>/<qos>`
  - mangled名: `/` → `%` に置換 (例: `/chatter` → `%chatter`)
  - enclave未設定時は `%`
- **Publisher Attachment** (put操作に付与):
  - 8 bytes: sequence number (int64_t, LE)
  - 8 bytes: timestamp (ns since UNIX EPOCH, int64_t, LE)
  - 1 byte: GID length (常に16)
  - 16 bytes: publisher GID
- **QoS liveliness エンコーディング**: `<reliability><durability>,<depth>:<deadline>:<lifespan>:<liveliness>,<lease_duration>`
  - 例: `::,10:,:,:,,` (system_default reliability/durability, depth=10)

## Language & Communication

- ユーザーへの応答は**日本語**で行うこと
- コード中のコメント・ドキュメントは英語（`//!` / `///` スタイル）
- コミットメッセージは英語

## Architecture

```
src/
├── buf/        # Static buffer pool (heapless)
├── cdr/        # CDR LE serialization (custom serde impl)
├── transport/  # Zenoh protocol v8 wire format, framing, handshake
├── session/    # Session state machine, pub/sub handles, reconnect
├── ros2/       # ROS2 adaptation: key expressions, QoS, Node, TopicPub/Sub
├── error.rs    # Hierarchical error types
└── logging.rs  # defmt/log macro abstraction
```

- 各モジュールは `mod.rs` + 個別ファイルで構成し、`pub use` で再エクスポート
- レイヤー依存: `ros2` → `session` → `transport` → `buf`/`cdr`

## Code Style

- `#![no_std]` — ヒープ割り当て禁止（`alloc` featureが明示的に有効な場合を除く）
- `heapless` コレクション（`Vec<u8, N>`, `String<N>`）を使い、サイズはconst genericsで指定
- async関数は `embedded-io-async` + `embassy-sync`/`embassy-time` を使用
- エラー型は `#[derive(Debug, Clone, Copy, PartialEq, Eq)]` + `Display` + `defmt::Format`
- ロギングは `ros2_trace!`, `ros2_debug!`, `ros2_info!`, `ros2_warn!`, `ros2_error!` マクロを使用（`println!` や `defmt::info!` を直接使わない）

## Build & Test

```sh
# ホストでのテスト（defmt無効）
cargo test --no-default-features

# WASMテスト（no_std環境）
cargo test --target wasm32-wasip1 --no-default-features

# no_std ビルド確認（Cortex-M0）
cargo check --target thumbv6m-none-eabi

# リリースビルド
cargo build --release --target thumbv6m-none-eabi

# Docker統合テスト（ROS2 rmw_zenoh_cpp との通信確認）
docker compose up -d
cargo test --test integration_test --no-default-features -- --ignored
docker compose down
```

## Conventions

- **Feature gating**: `defmt`（デフォルト）と `log` は排他。`#[cfg(feature = "defmt")]` で条件分岐
- **Protocol constants**: `pub mod transport_id { ... }` 形式でネストされたモジュール内に定義
- **Key expression format**: `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>` (rmw_zenoh_cpp互換)
  - `type_name` はDDS慣例: `pkg::msg::dds_::TypeName_`
- **CDR**: Little Endian固定、encapsulation header `[0x00, 0x01, 0x00, 0x00]`
- **テストは `#[cfg(test)] mod tests` で各ファイル末尾に配置**

## ESP32-C3 実装 (examples/esp32c3_wifi)

ESP32-C3ターゲットのコードを書く際は `.github/instructions/esp32c3-rust.instructions.md` を参照すること。主な要点:

- `esp_bootloader_esp_idf::esp_app_desc!()` を `#![no_main]` の直後に必ず記述（probe-rs書き込みに必須）
- 初期化順序: RTT → HAL → Heap → `esp_rtos::start()` → esp-radio の順を守る
- `embassy-executor` に `arch-*` featureを付けない（`esp-rtos`がexecutorを提供）
- `connect_async()` は常に `embassy_time::with_timeout()` でラップする（WiFiイベントが来ない場合ハング）
- ビルドプロファイルの `dev` でも `opt-level = 2` 必須（WiFiスタックが不安定になるため）
- Wi-Fi認証情報は `build.rs` + `wifi_config.json` パターンでコンパイル時に埋め込む（ソースへの直書き禁止）
