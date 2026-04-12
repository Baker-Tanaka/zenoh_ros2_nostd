# wasi_chatter_class — NodeCallbacks トレイト方式デモ

SDK の `NodeCallbacks` トレイトを実装するパターン（rclpy のクラス継承に相当）で `/chatter` トピックの pub/sub を行うサンプルです。

## 概要

`ChatterNode` 構造体が `NodeCallbacks` を実装し:

- **`on_timer`**: 500ms ごとに `std_msgs/String` メッセージを publish
- **`on_message`**: 受信した `/chatter` メッセージを表示

## 実行方法

### Host（ネイティブ）— docker compose 環境

```sh
# zenoh router が起動している前提
cd examples/wasi_chatter_class
cargo run
```

### WASI モード

```sh
cd examples/wasi_chatter_class
cargo build --target wasm32-wasip1 --features wasi
socat EXEC:"wasmtime run target/wasm32-wasip1/debug/wasi-chatter-class.wasm",fdin=3,fdout=3 \
      TCP:localhost:7447
```

### ROS2 側で確認

```sh
# 別ターミナルで
ros2 topic echo /chatter std_msgs/msg/String
```

## コードの要点

```rust
// NodeCallbacks トレイトを実装
struct ChatterNode {
    pub_handle: Option<PublisherHandle<StringMsg, 137, 4>>,
    count: u32,
}

impl NodeCallbacks for ChatterNode {
    fn on_message(&mut self, topic: &str, payload: &[u8]) {
        // 受信メッセージを処理
    }
    fn on_timer(&mut self) {
        // タイマーで定期 publish
        self.pub_handle.as_ref().unwrap().try_publish(&msg).unwrap();
    }
}

// Node を構築してスピン
let (mut node, mut cbs) = Node::builder("chatter_class_node")
    .build_with_callbacks(transport, ChatterNode::new(20))
    .await?;

node.register_static_publisher(&CHATTER_PUB).await?;
node.subscribe_with_dispatch(CHATTER_TOPIC, &CHATTER_SUB).await?;
node.register_timer(Duration::from_millis(500));
node.spin_with_callbacks(&mut cbs).await;
```

## 比較: 2つのスタイル

| | ハンドルベース (`wasi_turtlebot3`) | トレイトベース (このサンプル) |
|---|---|---|
| パターン | `spin()` + ハンドル recv | `spin_with_callbacks()` + `on_*` |
| メッセージ受信 | `sub_handle.recv().await` | `on_message(topic, payload)` |
| タイマー | 手動ループ | `register_timer()` + `on_timer()` |
| 状態管理 | 外部変数 | 構造体フィールド |
| 適用場面 | 複数タスク並行 | シングルタスク・状態集約 |
