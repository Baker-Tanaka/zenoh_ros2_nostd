# wasi_turtlebot3 — Chatter Publisher Example

`zenoh-ros2-nostd` から Zenoh Router 経由で `/chatter` トピックに `std_msgs/String` を CDR パブリッシュするサンプル。

## クイックスタート（開発コンテナ内）

ターミナルを **2つ** 開きます。

### ターミナル 1: サブスクライバー（受信確認）

```sh
cd examples/wasi_turtlebot3
python3 verify_sub.py
```

```
[verify-sub] connected to zenoh-router:7447 (ZID: ...)
[verify-sub] subscribed to /chatter — waiting 30s ...
```

### ターミナル 2: パブリッシャー（送信）

```sh
cd examples/wasi_turtlebot3
cargo run
```

### 期待される出力

**パブリッシャー側:**
```
[chatter-pub] connecting to zenoh-router:7447
[chatter-pub] connected (lease=60000ms)
[chatter-pub] declared: 0/chatter/std_msgs::msg::dds_::String_/RIHS01_...
[chatter-pub] # 1 published: "Hello from zenoh-ros2-nostd! [0]"
[chatter-pub] # 2 published: "Hello from zenoh-ros2-nostd! [1]"
...
[chatter-pub] done — published 20 messages to /chatter
```

**サブスクライバー側:**
```
  [  1] /chatter: "Hello from zenoh-ros2-nostd! [0]"
  [  2] /chatter: "Hello from zenoh-ros2-nostd! [1]"
...
✅ Received 20 message(s) from /chatter
```

## データフロー

```
┌────────────────────┐   TCP    ┌──────────────┐   TCP   ┌──────────────────┐
│ cargo run          │────────→ │ Zenoh Router │←──────→ │ verify_sub.py    │
│ (publisher)        │          │ :7447        │         │ (subscriber)     │
│                    │          │              │         │                  │
│ CDR String         │          │ ルーティング  │         │ CDR → UTF-8 表示 │
│ → Push+Put Frame   │          │              │         │                  │
└────────────────────┘          └──────────────┘         └──────────────────┘
```

## ビルドモード

| モード | コマンド | 用途 |
|--------|---------|------|
| **Host (native)** | `cargo run` | 開発コンテナからすぐ実行。推奨 |
| **WASI** | `cargo build --target wasm32-wasip1 --features wasi` | wasmtime 実行用 .wasm 生成 |

### Host モード

開発コンテナ内で直接実行可能。環境変数 `ZENOH_ROUTER_ADDR` (デフォルト: `zenoh-router:7447`) でルーターアドレスを指定。

```sh
cargo run                                        # デフォルト
ZENOH_ROUTER_ADDR=localhost:7447 cargo run       # ローカルルーター
```

### WASI モード

`wasmtime` + `socat` が必要（開発コンテナには未インストール）。

```sh
cargo build --target wasm32-wasip1 --features wasi
socat EXEC:"wasmtime run target/wasm32-wasip1/debug/wasi-turtlebot3.wasm",fdin=3,fdout=3 \
      TCP:localhost:7447
```

WASI の詳細は [docs/wasi-guide.md](../../docs/wasi-guide.md) を参照。

## ファイル構成

| ファイル | 説明 |
|---------|------|
| `src/main.rs` | パブリッシャー本体（Host/WASI 両対応） |
| `verify_sub.py` | Python Zenoh サブスクライバー（受信確認用） |
| `Cargo.toml` | ビルド設定 |

## 仕組み

1. Zenoh Router に TCP 接続 → InitSyn/InitAck/OpenSyn/OpenAck ハンドシェイク
2. `/chatter` の Key Expression を Declare
3. 20 回ループ: 各メッセージを CDR LE シリアライズ → Push+Put フレーム送信
4. 5 メッセージごとに KeepAlive 送信
5. Close フレームで切断

## トラブルシューティング

| 症状 | 原因 | 対処 |
|------|------|------|
| `TCP connect failed` | ルーター未起動 | `docker compose` で zenoh-router が起動しているか確認 |
| `handshake failed: Io` | ネットワーク不通 | `ZENOH_ROUTER_ADDR` が正しいか確認 |
| サブスクライバーに何も表示されない | タイミング | サブスクライバーを**先に**起動してからパブリッシャーを実行 |
| `(raw N bytes)` と表示 | CDR ヘッダ不一致 | Key Expression のハッシュが一致しているか確認 |
