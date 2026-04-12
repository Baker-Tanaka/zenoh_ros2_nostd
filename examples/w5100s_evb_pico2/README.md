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
main()         — RP2350 / W5100S 初期化、embassy executor 起動
  ├─ ethernet_task  — W5100S SPI パケット I/O (embassy-net-wiznet Runner)
  ├─ net_task        — embassy-net TCP/IP スタック
  ├─ zenoh_task      — DHCP待機 → TCP接続 → NodeBuilder::open() → spin() → 再接続
  └─ app_task        — 5秒毎に /chatter をパブリッシュ、受信メッセージをログ出力
```

## SDK の使い方

```rust
// 1. ros2::msg の定義済み定数でトピックを定義（type_hash の手入力不要）
const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::CHATTER;

// 2. CDR バッファサイズを cdr_cap_for_string() で計算（手動計算不要）
const CDR_BUF_CAP: usize = cdr_cap_for_string(128); // = 137

// 3. static Publisher / Subscription を定義（const fn, no heap）
static CHATTER_PUB: Publisher<StringMsg, CDR_BUF_CAP, 4> = Publisher::new(CHATTER_TOPIC);
static CHATTER_SUB: Subscription<StringMsg, CDR_BUF_CAP, 4> = Subscription::new();

// 4. NodeBuilder でビルダー設定 → open(transport) でセッション確立
let mut node = cfg.zenoh.session
    .node_builder()
    .name("pico2_node")
    .open(socket)           // T: Read + Write — WiFi, Ethernet, USB CDC など何でも可
    .await?;

// 5. 登録と宣言（as_drain() / as_dispatch() で dyn Trait キャスト不要）
node.register_publisher(CHATTER_PUB.as_drain());
CHATTER_SUB.clear(); // 再接続時: 前セッションの古いメッセージをフラッシュ
node.subscribe(CHATTER_TOPIC, CHATTER_SUB.as_dispatch()).await?;

// 6. セッション駆動（接続断まで返らない）
node.spin(&mut rx_buf).await;

// 別タスクから送受信
CHATTER_PUB.send(&msg).await?;
while let Some(r) = CHATTER_SUB.try_recv() { ... }
```

---

## ⚠️ Code Review — `src/main.rs`

このサンプルの `main.rs` に対するレビューです。SDK 設計の観点から改善すべき点と、組み込みシステムとしてのバグリスクを指摘します。

> **凡例**: ✅ 解決済み / ⚠️ 既知の課題（今後の改善項目）

### SDK 設計コンセプト: 可読性・使いやすさの問題

#### 1. `&'static dyn Trait` キャストの非直感性 ✅ 解決済み

以前のコード:
```rust
node.register_publisher(&CHATTER_PUB as &'static dyn PublisherDrain);
node.subscribe(CHATTER_TOPIC, &CHATTER_SUB as &'static dyn SubscriptionDispatch).await?;
```

`Publisher::as_drain()` / `Subscription::as_dispatch()` メソッドを追加して隠蔽:
```rust
node.register_publisher(CHATTER_PUB.as_drain());
node.subscribe(CHATTER_TOPIC, CHATTER_SUB.as_dispatch()).await?;
```

#### 2. `CDR_BUF_CAP` の手動計算が難しく間違えやすい ✅ 解決済み

以前のコード:
```rust
const CDR_BUF_CAP: usize = 144;  // 4 + 4 + 128 + 1 = 137... なぜ 144?
```

`cdr::cdr_cap_for_string()` ヘルパーで自動計算:
```rust
const CDR_BUF_CAP: usize = cdr_cap_for_string(128); // = 137 (4+4+128+1)
```

文字列サイズを変更すれば CAP も自動的に追従する。

#### 3. `TopicKeyExpr` の type_hash が手入力 ✅ 解決済み

以前のコード:
```rust
const CHATTER_TOPIC: TopicKeyExpr = TopicKeyExpr::new(
    0, "chatter", "std_msgs::msg::dds_::String_",
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18",
);
```

`ros2::msg` モジュールの定義済み定数を使用:
```rust
const CHATTER_TOPIC: TopicKeyExpr = msg::std_msgs::String::CHATTER;
// または
const MY_TOPIC: TopicKeyExpr = msg::std_msgs::String::topic(0, "my_topic");
```

タイポはコンパイル時に検出される。

#### 4. 再接続後のサブスクライバー状態 ✅ 解決済み

`Subscription::clear()` を追加して再接続前に古いメッセージをフラッシュ:
```rust
CHATTER_SUB.clear(); // 前セッションの古いメッセージを破棄
node.subscribe(CHATTER_TOPIC, CHATTER_SUB.as_dispatch()).await?;
```

---

### 組み込みシステムとしてのバグリスク

#### 5. TCP バッファのスタック配置（スタックオーバーフロー危険）✅ 解決済み

`StaticCell<[u8; 4096]>` statics に移動済み。詳細は main.rs の `TCP_RX_BUF` / `TCP_TX_BUF` / `ZENOH_RX_BUF` を参照。

#### 6. ランダムシードが固定値（MAC アドレス衝突）⚠️ 既知の課題

```rust
let seed: u64 = 0x1234_5678_9abc_def0; // REPLACE with hardware-derived entropy
```

複数のデバイスが同じシードを持つと ARP や TCP 初期シーケンス番号が衝突する可能性がある。
RP2040 の `ROSC` (リングオシレータ) や `UID` (チップ固有 ID) を使って entropy を得ることを推奨。

#### 7. `panic!` がサイレントに再起動する可能性 ⚠️ 既知の課題 (部分対応)

`panic-probe` は probe-rs 接続時に defmt で詳細を出力する。スタンドアロン環境では
再起動ループになるだけ。`PANIC_COUNT` 静的カウンターを追加し、起動時に前回のパニック数を
ログ出力することで兆候を検出できる（ただし RAM カウンターのため電源断で消える）。

**完全対応策**: watchdog スクラッチレジスタにパニック回数を書き込み、フラッシュに永続化する。

#### 8. `app_task` のパブリッシュ失敗が無視される ⚠️ 既知の課題 (部分対応)

ドロップカウンターを追加してトラッキングを改善:
```rust
match CHATTER_PUB.send(&msg).await {
    Ok(()) => {}
    Err(e) => {
        drop_count += 1;
        error!("[app] Publish error (total drops={}): {}", drop_count, e);
    }
}
```

セッション未確立時にメッセージが失われることは変わらないため、
将来は再試行ロジックや接続待機キューの実装が望ましい。
