# WASI Guide — zenoh-ros2-nostd on wasmtime

## Overview

`zenoh-ros2-nostd` は `wasm32-wasip1` ターゲットでビルドでき、
[wasmtime](https://wasmtime.dev/) 上で実行することで、
Gazebo シミュレーション環境の ROS2 ノードと TCP/Zenoh 経由で通信できる。

## Prerequisites

| Tool | Version | Install |
|------|---------|---------|
| Rust | nightly | `rustup default nightly` |
| wasm32-wasip1 target | — | `rustup target add wasm32-wasip1` |
| wasmtime | ≥ 18.0 | https://wasmtime.dev/ |
| Zenoh router | 1.x | `docker pull eclipse/zenoh` or `cargo install zenoh` |

## Build

```sh
cd examples/wasi_turtlebot3
cargo build --target wasm32-wasip1 --release
```

成果物: `target/wasm32-wasip1/release/wasi-turtlebot3.wasm`

## Network Setup

WASI Preview 1 にはソケット `connect()` API がないため、
ホスト側で TCP 接続を確立し、ファイルディスクリプタ (fd 3) として WASM モジュールに渡す。

### 方法 1: socat + wasmtime (推奨)

```sh
# 1. Zenoh router を起動
docker run --rm -p 7447:7447 eclipse/zenoh

# 2. socat で TCP 接続を fd 3 として渡す
socat EXEC:"wasmtime run --mapdir /::/ target/wasm32-wasip1/release/wasi-turtlebot3.wasm",fdin=3,fdout=3 \
      TCP:localhost:7447
```

### 方法 2: カスタムホストプログラム

wasmtime の Rust API を使って、TCP 接続済みソケットを fd 3 として WASM インスタンスに渡す:

```rust
use wasmtime::*;
use wasmtime_wasi::preview1::WasiP1Ctx;
use std::net::TcpStream;

fn main() -> anyhow::Result<()> {
    let engine = Engine::default();
    let module = Module::from_file(&engine, "wasi-turtlebot3.wasm")?;

    // TCP connect to Zenoh router
    let tcp = TcpStream::connect("127.0.0.1:7447")?;

    let mut linker = Linker::new(&engine);
    wasmtime_wasi::preview1::add_to_linker_sync(&mut linker, |cx| cx)?;

    let mut wasi = wasmtime_wasi::WasiCtxBuilder::new();
    wasi.inherit_stdio();
    // Inject pre-connected TCP socket as fd 3
    let tcp_stream = wasmtime_wasi::TcpStream::from_std(tcp);
    wasi.preopened_socket(3, tcp_stream)?;

    let mut store = Store::new(&engine, wasi.build_p1());
    linker.module(&mut store, "", &module)?;
    linker.get_default(&mut store, "")?.typed::<(), ()>(&store)?.call(&mut store, ())?;
    Ok(())
}
```

> **注意**: wasmtime の WASI API は頻繁に変更されるため、
> 上記コードはコンセプト例として参考にしてください。

## Docker Compose で ROS2 + Gazebo と統合

```yaml
# docker-compose.yml
services:
  zenoh-router:
    image: eclipse/zenoh:latest
    ports:
      - "7447:7447"
    command: ["-l", "tcp/0.0.0.0:7447"]

  ros2-gazebo:
    image: ros:jazzy
    depends_on:
      - zenoh-router
    environment:
      - RMW_IMPLEMENTATION=rmw_zenoh_cpp
      - ZENOH_ROUTER_CONFIG_URI=tcp/zenoh-router:7447
    command: >
      bash -c "
        source /opt/ros/jazzy/setup.bash &&
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
      "
```

```sh
# 1. ROS2 + Zenoh 環境を起動
docker compose up -d

# 2. WASM モジュールを実行 (socat 方式)
socat EXEC:"wasmtime run target/wasm32-wasip1/release/wasi-turtlebot3.wasm",fdin=3,fdout=3 \
      TCP:localhost:7447

# 3. Gazebo 上の turtlebot3 が動き出す
```

## Architecture

```
┌──────────────────────┐   fd 3   ┌──────────────────┐   TCP  ┌──────────────┐
│ wasmtime             │══════════│ socat / host     │────────│ zenohd       │
│  wasi-turtlebot3.wasm│          │ TCP relay        │        │ :7447        │
│  WasiTcpStream(fd=3) │          │                  │        │              │
└──────────────────────┘          └──────────────────┘        └──────┬───────┘
                                                                     │
                                                              ┌──────┴───────┐
                                                              │ ROS2 Node    │
                                                              │ rmw_zenoh_cpp│
                                                              │ turtlebot3   │
                                                              └──────────────┘
```

## Data flow

1. `main()` → `WasiTcpStream::from_raw_fd(3)` でソケットをラップ
2. `client_handshake()` → InitSyn/InitAck/OpenSyn/OpenAck で Zenoh セッション確立
3. `encode_declare_keyexpr()` → `/cmd_vel` のキー式を登録
4. ループ: `Twist` を CDR シリアライズ → `encode_push_put_with_attachment()` → `write_frame()`
5. `encode_close()` で切断

## Troubleshooting

| 症状 | 原因 | 対処 |
|------|------|------|
| `handshake failed: Io` | TCP 接続できない | Zenoh router が起動しているか確認 |
| `handshake failed: Protocol` | プロトコルバージョン不一致 | zenoh 1.x (protocol v9) を使用 |
| WASM が即終了 | fd 3 が渡されていない | socat の `fdin=3,fdout=3` を確認 |
| Gazebo のロボットが動かない | キー式不一致 | `ros2 topic list` で `/cmd_vel` の型ハッシュを確認 |
