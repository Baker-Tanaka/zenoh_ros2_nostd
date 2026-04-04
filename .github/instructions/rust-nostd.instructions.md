---
description: "Use when writing or editing Rust source files in this no_std embedded crate. Covers heapless collections, embassy async, error handling, feature gating, and module conventions."
applyTo: "**/*.rs"
---
# Rust no_std Coding Rules

## Allocation
- Never use `Vec`, `String`, `Box`, or `HashMap` from `std`/`alloc` unless behind `#[cfg(feature = "alloc")]`
- Use `heapless::Vec<T, N>` and `heapless::String<N>` with explicit const generic capacity
- Prefer stack buffers (`[u8; N]`) for scratch space

## Async
- All async I/O uses `embedded_io_async::{Read, Write}` traits — **not** `tokio` or `std::io`
- Synchronization: `embassy_sync::mutex::Mutex<CriticalSectionRawMutex, T>` or `embassy_sync::channel::Channel`
- Timers: `embassy_time::{Duration, Instant, Timer}` — never `std::time`

## Error Handling
- Custom error enums derive `#[derive(Debug, Clone, Copy, PartialEq, Eq)]`
- Implement `core::fmt::Display` for all error types
- Gate `defmt::Format` impl behind `#[cfg(feature = "defmt")]`
- Convert between layers with `From` trait — e.g., `impl From<TransportError> for Error`
- Use `?` operator with `.map_err()` for cross-layer error conversion

## Logging
- Use only `ros2_trace!`, `ros2_debug!`, `ros2_info!`, `ros2_warn!`, `ros2_error!` from `crate::logging`
- Never use `println!`, `eprintln!`, `dbg!`, `defmt::info!`, or `log::info!` directly

## Feature Gating
```rust
// Correct pattern for defmt conditional compilation
#[cfg(feature = "defmt")]
impl defmt::Format for MyType {
    fn format(&self, fmt: defmt::Formatter) { ... }
}
```
- `defmt` and `log` features are mutually exclusive
- Default feature is `defmt`
- Host tests run with `--no-default-features`

## Module Structure
- Each module directory: `mod.rs` (re-exports) + individual files
- Public types re-exported in `mod.rs` via `pub use submodule::TypeName;`
- Tests at the bottom of each file inside `#[cfg(test)] mod tests { ... }`

## Generics & Type Parameters
- Buffer sizes as const generics: `const N: usize`
- Transport links as trait bounds: `T: Read + Write`
- Lifetimes named descriptively: `'de` for deserializer, `'a` for borrowed refs

## Documentation
- Module-level: `//!` doc comments explaining purpose
- Public items: `///` doc comments with parameter descriptions
- Write documentation in English
