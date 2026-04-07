# rp2040-w5500 — ROS2 pub/sub over W5500 Ethernet

baker link.Dev (RP2040) + W5500 SPI Ethernet で Docker 上の ROS2 (`rmw_zenoh_cpp`) と
`/chatter` トピックを pub/sub するサンプルです。

## Hardware

| RP2040 GPIO         | W5500 ピン | 用途             |
|---------------------|-----------|-----------------|
| GP16 (SPI0 RX/MISO) | MISO      | SPI データ入力   |
| GP17 (GP output)    | SCS       | チップセレクト   |
| GP18 (SPI0 SCK)     | SCLK      | SPI クロック     |
| GP19 (SPI0 TX/MOSI) | MOSI      | SPI データ出力   |
| GP20 (GP input)     | INTn      | 割り込み (active-low) |
| GP21 (GP output)    | RSTn      | リセット (active-low) |

ピン番号は `src/main.rs` 内の `Spi::new(...)` および `Output::new(...)` / `Input::new(...)` の引数で変更できます。

## ネットワーク構成

```text
baker link.Dev + W5500 ──Ethernet──► Zenoh router (zenohd :7447)
                                       ▲
                    Docker ROS2 ────────┘
```

## セットアップ

### 1. Docker で ROS2 + Zenoh ルーターを起動

プロジェクトルートの `docker-compose.yml` を使用:

```sh
cd <repo_root>
docker compose up -d
```

### 2. 設定ファイルを作成

```sh
cp config.json.example config.json
# router_addr を編集: "192.168.1.xxx:7447"
```

### 3. ビルドして書き込む

```sh
cargo run --release
```

### 4. ROS2 側で確認

```sh
# メッセージ受信確認
ros2 topic echo /chatter std_msgs/msg/String

# メッセージ送信確認
ros2 topic pub /chatter std_msgs/msg/String 'data: "Hello from ROS2"'
```

## タスクアーキテクチャ

```text
main()         — RP2040 / W5500 初期化、embassy executor 起動
  ├─ ethernet_task  — W5500 SPI パケット I/O (embassy-net-wiznet Runner)
  ├─ net_task        — embassy-net TCP/IP スタック
  ├─ zenoh_task      — DHCP待機 → TCP接続 → NodeBuilder::open() → spin() → 再接続
  └─ app_task        — 5秒毎に /chatter をパブリッシュ、受信メッセージをログ出力
```

## SDK の使い方

```rust
// 1. static Publisher / Subscription を定義（const fn, no heap）
static CHATTER_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();

// 2. NodeBuilder でビルダー設定 → open(transport) でセッション確立
let mut node = cfg.zenoh.session
    .node_builder()
    .name("rp2040_node")
    .open(socket)           // T: Read + Write — WiFi, Ethernet, USB CDC など何でも可
    .await?;

// 3. 登録と宣言
node.register_publisher(&CHATTER_PUB as &'static dyn PublisherDrain);
node.subscribe(CHATTER_TOPIC, &CHATTER_SUB as &'static dyn SubscriptionDispatch).await?;

// 4. セッション駆動（接続断まで返らない）
node.spin(&mut rx_buf).await;

// 別タスクから送受信
CHATTER_PUB.send(&msg).await?;
while let Some(r) = CHATTER_SUB.try_recv() { ... }
```

---

## ⚠️ Code Review — `src/main.rs`

このサンプルの `main.rs` に対するレビューです。SDK 設計の観点から改善すべき点と、組み込みシステムとしてのバグリスクを指摘します。

### SDK 設計コンセプト: 可読性・使いやすさの問題

#### 1. `&'static dyn Trait` キャストの非直感性

```rust
// ユーザーが書く必要があるコード
node.register_publisher(&CHATTER_PUB as &'static dyn PublisherDrain);
node.subscribe(CHATTER_TOPIC, &CHATTER_SUB as &'static dyn SubscriptionDispatch).await?;
```

`as &'static dyn PublisherDrain` は `static` 変数に対してのみ有効な操作で、
なぜ `&'static dyn Trait` が必要かをユーザーは理解しにくい。
**改善案**: `Publisher::token()` や `Subscription::handle()` などのメソッドで
ユーザーから `dyn Trait` を隠蔽する。

#### 2. `CDR_BUF_CAP` の手動計算が難しく間違えやすい

```rust
const CDR_BUF_CAP: usize = 144;  // 4 + 4 + 128 + 1 = 137... なぜ 144?
```

メッセージ型のアライメントやヌル終端を含む正確な CDR バッファサイズを
ユーザーが手動で計算するのは困難。過小に設定するとパニックではなくデータ欠損が発生する。
**改善案**: `Publisher::<StringMsg, N>` が提供する `fn suggested_cdr_cap<M: Serialize>() -> usize`
のような計算ヘルパー、または `const MIN_CDR_CAP` 付きのメッセージトレイトが望ましい。

#### 3. `TopicKeyExpr` の type_hash が手入力

```rust
const CHATTER_TOPIC: TopicKeyExpr = TopicKeyExpr::new(
    0, "chatter",
    "std_msgs::msg::dds_::String_",
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18", // 手入力
);
```

RIHS01 ハッシュのタイポは実行時まで検出できない（ROS2 側でトピックが発見されない）。
**改善案**: `std_msgs::msg::String_::KEY_EXPR` のような定義済み定数の提供。

#### 4. 再接続後のサブスクライバー状態

`node.spin()` が返ると Node は消費される。再接続後に `subscribe()` を再呼び出しているが、
`CHATTER_SUB` のキューには切断前の古いメッセージが残留している可能性がある。
**改善案**: `Subscription::clear()` を提供し、再接続時に古いメッセージをフラッシュできるようにする。

---

### 組み込みシステムとしてのバグリスク

#### 5. TCP バッファのスタック配置（スタックオーバーフロー危険）

```rust
async fn zenoh_task(stack: Stack<'static>) {
    loop {
        let mut tcp_rx = [0u8; 4096];   // ← ループ毎にスタックに 4 KB!
        let mut tcp_tx = [0u8; 4096];   // ← 合計 8 KB + rx_buf 4 KB = 12 KB/iteration
        ...
        let mut rx_buf = [0u8; 4096];   // ← さらに 4 KB
```

RP2040 の SRAM は 264 KB だが、embassy の非同期スタックは 4–16 KB 程度。
ループボディ内で 12 KB をスタックに置くと他タスクのスタックと衝突する危険がある。
**修正**: これらの配列を `static` か `StaticCell<[u8; N]>` で確保する。

```rust
static TCP_RX: StaticCell<[u8; 4096]> = StaticCell::new();
static TCP_TX: StaticCell<[u8; 4096]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; 4096]> = StaticCell::new();
```

ただし `TcpSocket` は借用ライフタイムのため、`StaticCell` による `'static` バッファが必要。

#### 6. ランダムシードが固定値（MAC アドレス衝突）

```rust
let seed: u64 = 0x1234_5678_9abc_def0;  // 全デバイスで同一
```

複数のデバイスが同じシードを持つと、ARP や TCP 初期シーケンス番号が衝突する可能性がある。
RP2040 には `ROSC` (リングオシレータ) や `UID` (チップ固有 ID) があるため、これを使って
entropy を得るべき。

#### 7. `panic!` がサイレントに再起動する可能性

`expect("W5500 init failed")` などのパニックは `panic-probe` が defmt 出力し probe-rs で
確認できるが、probe-rs なしの量産環境では再起動ループになるだけでデバッグ情報が消える。
**対策**: watchdog タイマーと組み合わせ、パニック回数をフラッシュに記録するリカバリ戦略を検討する。

#### 8. `app_task` のパブリッシュ失敗が無視される

```rust
match CHATTER_PUB.send(&msg).await {
    Ok(()) => {}
    Err(e) => error!("[app] Publish error: {}", e),  // ログだけ; 再試行なし
}
```

`Zenoh セッション未確立時` に `send()` がキューフル `Err` を返しても、
ログを出力して次の 5 秒待機に進む。メッセージは黙って失われる。
**改善案**: エラーカウンタをトラッキングし、一定数を超えたらシステムリセットを検討する。
または `send()` を再試行するリトライロジックを追加する。
