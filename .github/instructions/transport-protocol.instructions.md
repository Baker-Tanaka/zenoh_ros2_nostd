---
description: "Use when working on Zenoh protocol transport messages, wire encoding, VByte codec, TCP framing, handshake, or KeepAlive logic. Covers Zenoh protocol v8 specifics."
applyTo: "src/transport/**"
---
# Zenoh Transport Protocol Conventions

## Protocol Version
- This crate implements **Zenoh protocol v8** (compatible with zenoh router 1.x)
- Protocol version constant: `PROTO_VERSION = 8`

## Message Header Format
- Transport header byte: `0bZZZIIIII` — upper bits are flags, lower 5 bits are message ID
- Network header byte: same format with different flag meanings

## Transport Message IDs
| Message | ID | Direction |
|---------|-----|-----------|
| INIT | 0x01 | client ↔ router |
| OPEN | 0x02 | client ↔ router |
| CLOSE | 0x03 | either |
| KEEP_ALIVE | 0x04 | either |
| FRAME | 0x05 | either (wraps network messages) |
| FRAGMENT | 0x06 | either |

## VByte Encoding
- All variable-length integers use unsigned LEB128 (VByte)
- `encode_vbyte()` / `decode_vbyte()` in `codec.rs`
- Used for: sequence numbers, lengths, resource IDs, lease values

## Encoding Functions
- Each message type has `encode_*` and `decode_*` functions in `codec.rs`
- Functions take `&mut [u8]` buffer and return `Result<usize, TransportError>` (bytes written)
- Decode functions return `Result<(T, usize), TransportError>` — value + bytes consumed

## TCP Framing
- Length-prefixed: `[u16 BE length][payload bytes]`
- Maximum batch size: 8192 bytes (configurable)
- Uses `embedded_io_async::{Read, Write}` for I/O — never raw socket APIs

## Handshake Sequence
1. Client sends `InitSyn` (version, WhatAmI::Client, ZenohId)
2. Router replies `InitAck` (version, WhatAmI, ZenohId, cookie)
3. Client sends `OpenSyn` (lease, initial_sn, cookie echo)
4. Router replies `OpenAck` (lease, initial_sn) — session open

## Constants Organization
- Group related constants in nested modules: `pub mod transport_id { ... }`, `pub mod frame_flag { ... }`
- Flag constants as `pub const NAME: u8 = 1 << N;`
