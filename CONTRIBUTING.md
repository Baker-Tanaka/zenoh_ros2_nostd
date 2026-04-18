# Contributing

## Development Setup

```sh
cargo fmt -- --check
cargo check
cargo test --no-default-features
```

## Extended Validation

```sh
cargo check --target thumbv6m-none-eabi
cargo test --target wasm32-wasip1 --no-default-features --no-run
```

## Examples

```sh
cd examples/wasi_turtlebot3 && cargo run
cd examples/bakerlink_wiz630io && cargo build
```

## Pre-release Check (No Publish)

```sh
cargo publish --dry-run
```

## Pull Request Guidelines

- Keep changes small and focused.
- Preserve `no_std` behavior unless feature-gated.
- Add or update tests when behavior changes.
- Update docs when command flow or public behavior changes.
