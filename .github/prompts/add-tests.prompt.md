---
description: "Add unit tests for a module: CDR roundtrip, codec encode/decode, buffer management, or integration scenarios"
agent: "agent"
---
# Add Tests

Add comprehensive unit tests for the specified module or function. Follow these conventions:

1. Place tests in `#[cfg(test)] mod tests { ... }` at the bottom of the file
2. Import with `use super::*;`
3. Test naming: `test_<feature>_<scenario>` (e.g., `test_vbyte_multibyte`)
4. Include:
   - Happy path
   - Edge cases (empty input, max values, boundary conditions)
   - Error cases (buffer overflow, invalid encoding)
   - Roundtrip tests (encode → decode → compare) for codec functions
5. Use `assert_eq!`, `assert!`, `assert!(matches!(..))` — no `unwrap()` in assertions where `assert_eq!` is clearer
6. Run `cargo test --no-default-features` to verify all tests pass
