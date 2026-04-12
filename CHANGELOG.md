# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2026-04-12

### Added
- `RosMessage` trait (`src/ros2/message_trait.rs`): provides `TYPE_NAME`, `TYPE_HASH`, and `topic()` for message types.
- `zenoh-ros2-nostd-derive` proc-macro crate (`derive/`): `#[derive(RosMessage)]` generates `RosMessage` impl from `#[ros_message(type_name = "...", type_hash = "...")]` attributes.
- `derive` feature flag: enables `#[derive(RosMessage)]` re-export from the main crate.
- `RosMessage` impl for `geometry_msgs::Twist` via the trait (in addition to existing `TwistType` metadata struct).
- Comprehensive crate-level documentation with architecture diagram and usage examples.
- Derive macro validation: compile-time checks for DDS type name convention (`dds_::` prefix, `_` suffix) and RIHS01 hash prefix.
- Derive macro tests: unit + integration tests for the proc-macro.

### Changed
- `Cargo.toml`: workspace structure added (`[workspace] members = [".", "derive"]`).
- `prelude.rs`: now re-exports `RosMessage` trait and `#[derive(RosMessage)]` (when `derive` feature enabled).
- `lib.rs`: added `__private` module for proc-macro path resolution.
- Version bumped to 1.0.0.

## [0.9.0] - 2026-04-12

### Added
- crates.io publication preparation assets: `LICENSE`, `CHANGELOG.md`, `CONTRIBUTING.md`.
- CI workflows for lint and test matrix: `fmt`, `clippy`, host tests, and cross-target checks.

### Changed
- Crate metadata in `Cargo.toml` updated for publishing readiness.
- README roadmap extended with a dedicated v0.9 release preparation phase.
- Example docs simplified for fast start paths:
  - `examples/w5100s_evb_pico2/README.md`
  - `examples/wasi_turtlebot3/README.md`

### Notes
- crates.io release itself is intentionally deferred.
- Readiness is validated via `cargo publish --dry-run` only.
