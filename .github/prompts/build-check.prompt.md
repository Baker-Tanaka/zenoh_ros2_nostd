---
description: "Run a comprehensive build verification: host check, no_std cross-compile, and unit tests"
agent: "agent"
---
# Build Verification

Run the following checks in order and report results:

1. **Host check**: `cargo check`
2. **no_std check**: `cargo check --target thumbv6m-none-eabi`
3. **Host tests**: `cargo test --no-default-features`
4. **Clippy** (if available): `cargo clippy --no-default-features -- -W warnings`

For each step, report:
- PASS / FAIL
- Number of warnings (list them if < 10)
- Error details if FAIL

If any step fails, diagnose the root cause and suggest a fix.
