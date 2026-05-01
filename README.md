# zenoh-ros2-nostd

[![no_std](https://img.shields.io/badge/no__std-compatible-brightgreen)](https://doc.rust-lang.org/reference/names/preludes.html#the-no_std-attribute)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![Development Status](https://img.shields.io/badge/status-early%20development-orange)](https://github.com/Baker-Tanaka/zenoh_ros2_nostd)

> ⚠️ **開発中 (Early Development)**
>
> このライブラリは現在積極的に開発中です。API は予告なく破壊的変更される可能性があります。
> プロダクション環境での使用は推奨しません。フィードバックや Issue の報告は歓迎します。

組み込みMCU向け `no_std` ROS2トピック pub/sub ライブラリ。
Zenoh プロトコル v9（zenoh 1.x）上で CDR シリアライゼーションを用い、標準の ROS2 ノード（`rmw_zenoh_cpp`）と通信する。

## Features

- **`#![no_std]`** — ヒープ割り当て不要、`heapless` 固定サイズコレクションのみ
- **rclpy ライク SDK** — コールバック方式 / トレイト方式の2スタイル ([設計詳細](docs/DESIGN.md))
- **`#[derive(RosMessage)]`** — proc-macro でメッセージ型メタデータを自動生成
- **Embassy ネイティブ** — `embedded-io-async` / `embassy-sync` / `embassy-time` で非同期
- **rmw_zenoh_cpp 互換** — キー式・Liveliness トークンが標準 ROS2 zenoh RMW と相互運用
- **CDR シリアライズ** — serde ベースの no_std CDR LE 実装
- **MCU 非依存** — `embedded-io-async` Read/Write トレイト抽象によりどのチップでも可
- **WASI 対応** — `wasm32-wasip1/wasip2` で Gazebo シミュレーションと TCP 通信
- **オフライン耐性** — 指数バックオフ自動再接続
- **defmt / log 選択式ロギング**

## Quick Start

### SDK API

```rust
use zenoh_ros2_nostd::prelude::*;

#[derive(Serialize, Deserialize)]
struct StringMsg {
    data: heapless::String<128>,
}

// Static publisher & subscription (no_std requires static placement)
const CHATTER_TOPIC: TopicKeyExpr = TopicKeyExpr::new(
    0, "chatter",
    "std_msgs::msg::dds_::String_",
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
);
static CHATTER_PUB: Publisher<StringMsg, 137, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, 137, 4> = Subscription::new();

async fn run(transport: impl embedded_io_async::Read + embedded_io_async::Write) {
    let mut node = NodeBuilder::new("talker")
        .zid(ZenohId::from_bytes(&[0x01; 8]))
        .domain_id(0)
        .build(transport)
        .await
        .unwrap();

    // Register publisher — returns a handle usable from any task
    let _pub_handle = node.register_static_publisher(&CHATTER_PUB).await.unwrap();

    // Subscribe — declares on the router and returns a handle
    let _sub_handle = node.subscribe_with_dispatch(CHATTER_TOPIC, &CHATTER_SUB)
        .await
        .unwrap();

    // Drive the session (publish drain, receive, keepalive)
    node.spin().await;
}
```

### Derive Macro (feature = "derive")

`#[derive(RosMessage)]` を使うと型名・ハッシュを属性で宣言できます:

```rust
use zenoh_ros2_nostd::prelude::*;

#[derive(Serialize, Deserialize, RosMessage)]
#[ros_message(
    type_name = "geometry_msgs::msg::dds_::Twist_",
    type_hash = "RIHS01_9b0e20a73f3b74c00f80ed1f26a4c7568a63aeec6e79ba04b64bcd5b7f49a2a5",
)]
struct Twist {
    linear: Vector3,
    angular: Vector3,
}

// TopicKeyExpr を自動生成
const CMD_VEL: TopicKeyExpr = Twist::topic(0, "cmd_vel");
static CMD_PUB: Publisher<Twist, 52, 4> = Publisher::new(CMD_VEL);
```

### 内部 API (ros2 レイヤー)

> 以下は `sdk/` モジュールが内部的に使用する低レベル API です。
> `#[doc(hidden)]` で隠蔽されており、直接使用は非推奨です。

```rust
use zenoh_ros2_nostd::ros2::{Node, TopicKeyExpr, Qos};
use zenoh_ros2_nostd::transport::protocol::ZenohId;
use serde::{Serialize, Deserialize};

// ROS2 メッセージ定義
#[derive(Serialize, Deserialize)]
struct Twist {
    linear: Vector3,
    angular: Vector3,
}

#[derive(Serialize, Deserialize)]
struct Vector3 { x: f64, y: f64, z: f64 }

// キー式定義
const TWIST_KE: TopicKeyExpr = TopicKeyExpr::new(
    0, "cmd_vel",
    "geometry_msgs::msg::dds_::Twist_",
    "RIHS01_...",
);

// Publisher / Subscription は static 配置
static TWIST_PUB: zenoh_ros2_nostd::ros2::Publisher<Twist, 256, 4> =
    zenoh_ros2_nostd::ros2::Publisher::new(TWIST_KE);
```

## Examples

| Example                                              | ターゲット                         | 状態         | 説明                                                                         |
| ---------------------------------------------------- | ---------------------------------- | ------------ | ---------------------------------------------------------------------------- |
| [`bakerlink_wiz630io`](examples/bakerlink_wiz630io/) | RP2040 + WIZ630io (W5500) Ethernet | ✅ アクティブ | Baker link. Dev + WIZ630io で Docker 上の ROS2 と pub/sub                    |
| [`wasi_turtlebot3`](examples/wasi_turtlebot3/)       | wasm32-wasip1                      | ✅ アクティブ | wasmtime で Gazebo turtlebot3 と cmd_vel 通信 ([ガイド](docs/wasi-guide.md)) |
| [`wasi_chatter_class`](examples/wasi_chatter_class/) | wasm32-wasip1                      | ✅ アクティブ | NodeCallbacks トレイト方式のクラスベース pub/sub デモ                        |
| [`esp32c3_wifi`](examples/esp32c3_wifi/)             | ESP32-C3 WiFi                      | 🗄️ アーカイブ | ESP32-C3 WiFi 接続デモ（現在未動作）                                         |

### ⚠️ esp32c3_wifi — アーカイブ

> このサンプルは現在**動作しない**ことが確認されています。
> `esp-radio` / `esp-rtos` 依存のバージョン互換性問題により、接続が不安定です。
> 参考実装として残してありますが、積極的なメンテナンスは行われていません。
> RP2040 + WIZ630io の [`bakerlink_wiz630io`](examples/bakerlink_wiz630io/) サンプルを代わりにご利用ください。

## Architecture

```
src/
├── sdk/        # PUBLIC: rclpy-like high-level API (Node, PublisherHandle, etc.)
├── ros2/       # internal: ROS2 adaptation (key expressions, QoS, liveliness)
├── session/    # internal: Session state machine, reconnect
├── transport/  # internal: Zenoh v9 wire protocol
├── buf/        # internal: Static buffer pool (heapless)
├── cdr/        # internal: CDR LE serialization (serde)
├── wasi/       # internal: WASI socket/time adapters (#[cfg(feature = "wasi")])
├── error.rs    # Hierarchical error types
├── logging.rs  # defmt/log macro abstraction
└── prelude.rs  # convenience re-exports
```

### レイヤー構成

```
┌─────────────────────────────────┐
│  sdk/    (Node, PublisherHandle) │  ← 唯一の公開 API (rclpy-like)
├─────────────────────────────────┤
│  ros2/   (KeyExpr, QoS, Msg)    │  ← 内部: ROS2 プロトコル
├─────────────────────────────────┤
│  session/  (Session, Reconnect) │  ← 内部: セッション管理
├─────────────────────────────────┤
│  transport/  (Handshake, Frame) │  ← 内部: Zenoh v9 プロトコル
├─────────────────────────────────┤
│  buf/ + cdr/                    │  ← 内部: バッファ・シリアライゼーション
└─────────────────────────────────┘
         │
    embedded-io-async (Read + Write)
         │
   ┌─────┴──────────────┐
   │ TCP Socket (MCU)   │  WASI socket (wasm)  │  tokio (host test)
   └────────────────────┘
```

## Design Specification

### Zenoh Protocol v9 (zenoh 1.x)

本クレートは Zenoh プロトコル v9 のクライアントモード最小サブセットを自前実装する。

| メッセージ        | ID   | 用途                             |
| ----------------- | ---- | -------------------------------- |
| InitSyn / InitAck | 0x01 | セッション開始                   |
| OpenSyn / OpenAck | 0x02 | セッション確立                   |
| Close             | 0x03 | 切断                             |
| KeepAlive         | 0x04 | 生存確認                         |
| Frame             | 0x05 | ネットワークメッセージのコンテナ |

- **ヘッダ形式**: `0bFFFIIIII` (3 フラグビット + 5 メッセージ ID ビット)
- **可変長整数**: unsigned LEB128 (VByte)
- **TCP フレーミング**: `[u16 LE length][payload]`
- **ハンドシェイク**: InitSyn → InitAck → OpenSyn → OpenAck の 4 ステップ
- **Init flags バイト**: `((zid_len - 1) << 4) | whatami_2bit`
    - `whatami_2bit`: Router=`0b00`, Peer=`0b01`, Client=`0b10`
    - ZenohId は flags で長さを表し、本体は raw bytes で続く

### CDR Serialization

- OMG CDR Little Endian 固定
- Encapsulation header: `[0x00, 0x01, 0x00, 0x00]`
- アライメント: プリミティブは自身のサイズ境界にアライン (u32 → 4byte, f64 → 8byte)
- 文字列: `[u32 length (null含む)][UTF-8 bytes][null]`

### rmw_zenoh_cpp Key Expression Format

```
<domain_id>/<topic_name>/<type_name>/<type_hash>
```

例:
- `0/chatter/std_msgs::msg::dds_::String_/RIHS01_...`
- `0/cmd_vel/geometry_msgs::msg::dds_::Twist_/RIHS01_...`

### Liveliness Token Format

```
@ros2_lv/<domain_id>/<session_id>/<node_id>/<entity_id>/<entity_kind>/<mangled_enclave>/<mangled_namespace>/<node_name>/<mangled_topic>/<type_name>/<type_hash>/<qos>
```

- mangled 名は `/` を `%` に置換（例: `/chatter` → `%chatter`）
- enclave 未設定時は `%`

### オフライン耐性

1. **自動再接続**: `ReconnectPolicy` — 指数バックオフ (1s → 2s → 4s → ... → 30s max)
2. **KeepAlive**: リース時間の 1/4 間隔で送信、タイムアウト検知
3. **将来**: Publisher ローカルバッファリング (接続回復時フラッシュ)

### 高速化設計

- **静的バッファプール**: `BufferPool<N, COUNT>` でヒープ回避
- **const generics**: TX/RX バッファサイズを型レベルで指定
- **ゼロアロケーション publish**: CDR → Frame → TCP を一本のバッファで処理

## Build & Test

```sh
# ホストテスト (defmt 無効)
cargo test --no-default-features

# no_std ビルド確認 (Cortex-M0)
cargo check --target thumbv6m-none-eabi

# WASM no_std テスト
cargo test --target wasm32-wasip1 --no-default-features

# リリースビルド
cargo build --release --target thumbv6m-none-eabi

# zenohd 実機統合テスト (手元の zenohd を使用)
cargo test --no-default-features --test integration_test -- --ignored --test-threads=1
```

### テスト環境

| ターゲット           | 用途                                       |
| -------------------- | ------------------------------------------ |
| `x86_64` (host)      | 高速ユニットテスト                         |
| `thumbv6m-none-eabi` | no_std ビルド検証                          |
| `wasm32-wasip1`      | no_std ランタイムテスト (ハードウェア不要) |

## Copilot Prompts (開発者向け)

本プロジェクトには VS Code Copilot 用のカスタマイズファイルが含まれています:

### Instructions (自動適用)

| ファイル                                                  | 対象               | 内容                                 |
| --------------------------------------------------------- | ------------------ | ------------------------------------ |
| `.github/copilot-instructions.md`                         | 全体               | プロジェクトガイドライン・日本語応答 |
| `.github/instructions/rust-nostd.instructions.md`         | `**/*.rs`          | no_std 制約・heapless・embassy async |
| `.github/instructions/transport-protocol.instructions.md` | `src/transport/**` | Zenoh プロトコル仕様                 |
| `.github/instructions/ros2-layer.instructions.md`         | `src/ros2/**`      | rmw_zenoh_cpp キー式・CDR・QoS       |

### Prompts (チャットで `/` → 選択)

| コマンド            | 用途                                                    |
| ------------------- | ------------------------------------------------------- |
| `/new-ros2-msg`     | 新しい ROS2 メッセージ型を生成 (serde + CDR テスト付き) |
| `/build-check`      | ホスト・no_std・テストの一括確認                        |
| `/add-tests`        | 指定モジュールへのユニットテスト追加                    |
| `/new-protocol-msg` | Zenoh プロトコルメッセージの codec 追加                 |

## Roadmap

> 詳細な設計ドキュメント: [docs/DESIGN.md](docs/DESIGN.md)

### v0.1 — Foundation ✅

- [x] プロジェクト構造・Cargo.toml
- [x] defmt/log マクロ抽象
- [x] 階層的エラー型
- [x] CDR LE Serializer / Deserializer (serde)
- [x] 静的バッファプール
- [x] Zenoh v9 プロトコル型定義・定数
- [x] VByte / LEB128 コーデック
- [x] TCP フレーミング (長さプレフィックス)
- [x] クライアントハンドシェイク (Init → Open)
- [x] KeepAlive マネージャ
- [x] セッション状態マシン
- [x] Publisher / Subscriber ハンドル
- [x] 自動再接続ポリシー (指数バックオフ)
- [x] rmw_zenoh_cpp 互換キー式生成
- [x] QoS プロファイル
- [x] Node 抽象 (NodeBuilder → static Pub/Sub → spin)
- [x] Typed TopicPublisher / TopicSubscriber
- [x] Liveliness トークン生成
- [x] WASM (wasm32-wasip1) no_std テスト環境
- [x] 89 ユニットテスト通過 (host)
- [x] 10 統合テスト通過 (zenohd handshake/keepalive/declare/publish/close/cross-session-sub/rmw-attachment/liveliness-token/multi-topic/fragment-reassembly)

### v0.2 — Receive Path & Integration ✅

- [x] Frame 受信ループ (Frame → Push/Put → Subscriber dispatch)
- [x] Declare Subscriber → Router 登録
- [x] DeclareKeyExpr → ローカル ID 割り当て
- [x] Baker link. Dev + WIZ630io Embassy example (RP2040 + W5500 Ethernet)
- [x] Baker link. Dev + WIZ630io example を新 SDK API に移行
- [x] `rmw_zenoh_cpp` ROS2 ノードとの双方向 pub/sub 通信テスト (cross-session subscribe + rmw attachment)

### v0.3 — SDK Redesign (rclpy-like API) 🔄

高レベル SDK に刷新し、内部レイヤーを隠蔽化する。

- [x] `sdk/` モジュール新設 — `Node::builder("name").build(transport)`
- [x] トレイト実装方式: `impl NodeCallbacks for MyNode { on_message, on_timer }`
- [x] `PublisherHandle<M>` — `&'static Publisher` ラッパー (Copy + Clone)
- [x] `SubscriptionHandle<M>` — チャネルベース (recv / try_recv)
- [x] `prelude.rs` — `use zenoh_ros2_nostd::prelude::*` で一括インポート
- [x] 内部モジュール `#[doc(hidden)]` による隠蔽化
- [x] タイマーコールバック (`register_timer` + `spin_with_callbacks`)
- [x] ZenohId 自動生成 (名前ベースハッシュ; MCU は明示指定推奨)
- [x] spin loop (publisher drain, frame dispatch, keepalive, timer fire)
- [ ] コールバック関数方式: `node.create_subscription("topic", callback)` — 将来検討 (no_std closure 格納制約)
- [ ] `PublisherHandle<M>` owned ハンドル (static 不要) — 将来検討 (no_std alloc 制約)

### v0.4 — WASI Target & Gazebo Integration ✅

WASI ターゲットで Gazebo シミュレーションと通信する。

- [x] `wasi` feature フラグ追加
- [x] `wasi/` モジュール構造 (mod.rs, socket.rs, time.rs)
- [x] `WasiTcpStream` — WASI socket → `embedded-io-async` adapter 実装
- [x] WASI time driver shim (`wasi::clocks::monotonic_clock`) 実装
- [x] `wasm32-wasip1` ビルド + テスト通過
- [x] `geometry_msgs/Twist` + `Vector3` CDR シリアライズ対応
- [x] Gazebo turtlebot3 デモ (`examples/wasi_turtlebot3/` — cmd_vel publish)
- [x] wasmtime での実行手順ドキュメント ([docs/wasi-guide.md](docs/wasi-guide.md))

### v0.5 — Resilience & Performance ✅

- [x] Publisher オフラインバッファリング (retry slot で送信失敗メッセージを保持)
- [x] Fragment メッセージ対応 (大ペイロード再組立)
- [x] バッチ送信 (複数メッセージを 1 Frame に束ねて TCP 書き込み削減)
- [x] ゼロコピーデシリアライズ (CDR デシリアライザが `visit_borrowed_str`/`visit_borrowed_bytes` を使用)
- [x] `cargo size` フットプリント最適化 (FragmentAssembler 64→16 KiB; BSS -50%)

### v0.6 — Full ROS2 Interop

- [x] Liveliness token 宣言 (graph discovery)
- [x] QoS ネゴシエーション (互換性チェック)
- [x] 双方向 `rmw_zenoh_cpp` 通信テスト (pub + sub)
- [x] 複数トピック同時 pub/sub

### v0.7 — Service Client ✅

- [x] `rmw_zenoh_cpp` Service 通信プロトコル解析（Request/Response/ResponseFinal）
- [x] Request/Reply ワイヤ形式実装（`encode_request_query`, `decode_response`）
- [x] `node.call_service::<Req, Resp>()` API + `ServiceNoReply` エラー型
- [x] Liveliness トークン SC (ServiceClient) / SS (ServiceServer) 対応
- [x] 統合テスト: zenohd 経由 Request+Query 送信確認

### v0.8 — Action Client ✅

- [x] `rmw_zenoh_cpp` Action 通信プロトコル解析 (3 Service + 2 Topic 分解)
- [x] `action_msgs` 共通型: `GoalId`, `Stamp`, `GoalInfo`, `SendGoalResponse`, `GetResultRequest`, `CancelGoalRequest`
- [x] `ActionKeyExprs` — 5つのサブエンティティ用キー式ヘルパー (CancelGoal/Status は標準型自動適用)
- [x] `node.send_goal()` / `node.get_result()` / `node.cancel_goal()` API
- [x] `node.subscribe_feedback()` / `node.subscribe_status()` — アクションフィードバック購読
- [x] Goal ステータス定数 (`STATUS_UNKNOWN` 〜 `STATUS_ABORTED`)
- [x] CDR ラウンドトリップテスト (GoalId, SendGoalResponse, GetResultRequest, CancelGoalRequest)
- [x] 統合テスト: zenohd 経由 SendGoal Request 送信確認

### v0.9 — Release Preparation ✅

- [x] crates.io 公開準備（`Cargo.toml` メタデータ、LICENSE、CHANGELOG、CONTRIBUTING）
- [x] CI 検証導線の追加（fmt / clippy / test / target check）
- [x] examples の簡素化（`bakerlink_wiz630io` / `wasi_turtlebot3` の最短実行導線）
- [x] `cargo publish --dry-run` で公開可能性を検証
- [ ] crates.io への実公開（このフェーズでは実施しない）

### v1.0 — Production Ready 🔄

- [x] Proc-macro: `#[derive(RosMessage)]` による自動メッセージ型生成
- [x] `RosMessage` トレイト定義 (TYPE_NAME, TYPE_HASH, topic())
- [x] `zenoh-ros2-nostd-derive` クレート新設 (proc-macro)
- [x] DDS型名 / RIHS01ハッシュのコンパイル時バリデーション
- [x] ドキュメント・API リファレンス (crate-level doc, SDK doc, derive doc)
- [x] crates.io 公開準備 (ワークスペース構成、derive クレートメタデータ)
- [ ] `.msg` / `.idl` からの型ハッシュ (RIHS01) 自動生成 — 将来検討
- [ ] TLS / 認証サポート — 将来検討
- [ ] crates.io 公開（準備完了、実公開は別途実施）

## Dependencies

| クレート                  | バージョン | 用途                                                 |
| ------------------------- | ---------- | ---------------------------------------------------- |
| `serde`                   | 1.0        | CDR シリアライズ (no_std, derive)                    |
| `heapless`                | 0.8        | 固定サイズコレクション (serde 対応)                  |
| `embedded-io-async`       | 0.6        | 非同期 Read/Write トレイト                           |
| `embassy-sync`            | 0.8        | Mutex, Channel (no_std 非同期)                       |
| `embassy-time`            | 0.5        | Timer, Duration (no_std 時間)                        |
| `defmt`                   | 1.0        | 組み込みログ (optional)                              |
| `log`                     | 0.4        | std ログ (optional)                                  |
| `zenoh-ros2-nostd-derive` | 0.9        | `#[derive(RosMessage)]` (optional, `derive` feature) |

## License

Apache-2.0


## test

```bash
cargo test --no-default-features --test integration_test -- --ignored
```
