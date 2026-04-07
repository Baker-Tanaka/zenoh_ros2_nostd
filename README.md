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
- **Embassy ネイティブ** — `embedded-io-async` / `embassy-sync` / `embassy-time` で非同期
- **rmw_zenoh_cpp 互換** — キー式・Liveliness トークンが標準 ROS2 zenoh RMW と相互運用
- **CDR シリアライズ** — serde ベースの no_std CDR LE 実装
- **MCU 非依存** — `embedded-io-async` Read/Write トレイト抽象によりどのチップでも可
- **オフライン耐性** — 指数バックオフ自動再接続
- **defmt / log 選択式ロギング**

## Quick Start

```rust
use zenoh_ros2_nostd::ros2::{Node, TopicKeyExpr, Qos};
use zenoh_ros2_nostd::session::{Session, SessionConfig};
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

async fn run(tcp_socket: &mut impl embedded_io_async::ReadWrite) {
    let zid = ZenohId::from_bytes(&[0x01; 8]);
    let config = SessionConfig::new(zid);
    let session = Session::<_, 512, 4096>::open(tcp_socket, config).await.unwrap();

    let node = Node::new(&session, "", "mcu_node");
    let mut cdr_buf = [0u8; 256];
    let mut publisher = node.create_publisher(&TWIST_KE, &mut cdr_buf);

    let msg = Twist {
        linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
        angular: Vector3 { x: 0.0, y: 0.0, z: 0.5 },
    };
    publisher.publish(&msg).await.unwrap();
}
```

## Examples

| Example | ターゲット | 状態 | 説明 |
|---|---|---|---|
| [`rp2040_w5500`](examples/rp2040_w5500/) | RP2040 + W5500 Ethernet | ✅ アクティブ | baker link.Dev + W5500 で Docker 上の ROS2 と pub/sub |
| [`esp32c3_wifi`](examples/esp32c3_wifi/) | ESP32-C3 WiFi | 🗄️ アーカイブ | ESP32-C3 WiFi 接続デモ（現在未動作） |

### ⚠️ esp32c3_wifi — アーカイブ

> このサンプルは現在**動作しない**ことが確認されています。
> `esp-radio` / `esp-rtos` 依存のバージョン互換性問題により、接続が不安定です。
> 参考実装として残してありますが、積極的なメンテナンスは行われていません。
> RP2040 + W5500 の [`rp2040_w5500`](examples/rp2040_w5500/) サンプルを代わりにご利用ください。

## Architecture

```
src/
├── buf/        # 静的バッファプール (heapless)
├── cdr/        # CDR LE シリアライゼーション (serde)
├── transport/  # Zenoh protocol v9 ワイヤ形式・フレーミング・ハンドシェイク
├── session/    # セッション状態マシン・Pub/Sub ハンドル・再接続
├── ros2/       # ROS2 適応層: キー式・QoS・Node・TopicPub/Sub
├── error.rs    # 階層的エラー型
└── logging.rs  # defmt/log マクロ抽象
```

### レイヤー構成

```
┌─────────────────────────────────┐
│  ros2/   (Node, TopicPub/Sub)   │  ← ユーザー API
├─────────────────────────────────┤
│  session/  (Session, Reconnect) │  ← セッション管理
├─────────────────────────────────┤
│  transport/  (Handshake, Frame) │  ← Zenoh v9 プロトコル
├─────────────────────────────────┤
│  buf/ + cdr/                    │  ← バッファ・シリアライゼーション
└─────────────────────────────────┘
         │
    embedded-io-async (Read + Write)
         │
   ┌─────┴──────┐
   │ TCP Socket │  (embassy-net, WiFi, Ethernet, etc.)
   └────────────┘
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

### v0.1 — Foundation (current)

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
- [x] Node 抽象
- [x] Typed TopicPublisher / TopicSubscriber
- [x] Liveliness トークン生成
- [x] WASM (wasm32-wasip1) no_std テスト環境
- [x] 59 ユニットテスト通過 (host)
- [x] 5 統合テスト通過 (zenohd handshake/keepalive/declare/publish/close)

### v0.2 — Receive Path & Integration

- [ ] Frame 受信ループ (Frame → Push/Put → Subscriber dispatch)
- [ ] Declare Subscriber → Router 登録
- [ ] DeclareKeyExpr → 双方向 ID マッピング
- [ ] Embassy example (RP2040 + WiFi/Ethernet)
- [ ] `rmw_zenoh_cpp` ROS2 ノードとの pub 通信テスト

### v0.3 — Resilience & Performance

- [ ] Publisher オフラインバッファリング (heapless::Deque)
- [ ] Fragment メッセージ対応 (大ペイロード分割)
- [ ] バッチ送信 (複数メッセージを 1 Frame に束ねる)
- [ ] ゼロコピーデシリアライズ最適化
- [ ] `cargo size` フットプリント最適化

### v0.4 — Full ROS2 Interop

- [ ] Liveliness token 宣言 (graph discovery)
- [ ] QoS ネゴシエーション (互換性チェック)
- [ ] 双方向 `rmw_zenoh_cpp` 通信テスト (pub + sub)
- [ ] 複数トピック同時 pub/sub

### v1.0 — Production Ready

- [ ] ROS2 Service client/server
- [ ] Proc-macro: `#[derive(RosMessage)]` による自動メッセージ型生成
- [ ] `.msg` / `.idl` からの型ハッシュ (RIHS01) 自動生成
- [ ] TLS / 認証サポート
- [ ] ドキュメント・API リファレンス
- [ ] crates.io 公開

## Dependencies

| クレート            | バージョン | 用途                                |
| ------------------- | ---------- | ----------------------------------- |
| `serde`             | 1.0        | CDR シリアライズ (no_std, derive)   |
| `heapless`          | 0.8        | 固定サイズコレクション (serde 対応) |
| `embedded-io-async` | 0.6        | 非同期 Read/Write トレイト          |
| `embassy-sync`      | 0.8        | Mutex, Channel (no_std 非同期)      |
| `embassy-time`      | 0.5        | Timer, Duration (no_std 時間)       |
| `defmt`             | 1.0        | 組み込みログ (optional)             |
| `log`               | 0.4        | std ログ (optional)                 |

## License

Apache-2.0


## test

```bash
cargo test --no-default-features --test integration_test -- --ignored
```
