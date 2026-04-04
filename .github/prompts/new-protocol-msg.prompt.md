---
description: "Generate a new transport protocol message encoder/decoder pair following the zenoh v8 wire format conventions"
agent: "agent"
---
# New Protocol Message Codec

Add a new zenoh protocol message encoder and decoder to `src/transport/codec.rs`. Follow existing patterns:

1. **Encoder**: `pub fn encode_<message>(buf: &mut [u8], ...) -> Result<usize, TransportError>`
   - Return number of bytes written
   - Check buffer space before each write
   - Use `encode_vbyte()` for variable-length integers
   - Use `encode_slice()` for byte arrays

2. **Decoder**: `pub fn decode_<message>(buf: &[u8]) -> Result<(T, usize), TransportError>`
   - Return parsed value and bytes consumed
   - Validate header byte (message ID + flags)
   - Parse fields in wire order

3. **Tests**: Add roundtrip test in `mod tests` block

4. If a new message type struct is needed, define it in `src/transport/protocol.rs` with appropriate flag constants in a nested module.

Reference the Zenoh protocol v8 specification for header format: `0bFFFIIIII` (3 flag bits, 5 message ID bits).
