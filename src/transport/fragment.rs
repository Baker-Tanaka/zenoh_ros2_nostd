//! Fragment reassembly for incoming Zenoh messages.
//!
//! When a message exceeds the TCP batch size, the router sends it as
//! multiple `Fragment` transport messages.  The first N-1 carry the
//! `M` (more) flag, and the last one does not.
//!
//! [`FragmentAssembler`] collects fragment payloads into a static buffer
//! and yields the complete network message once the final fragment arrives.

use crate::error::TransportError;

/// Reassembles fragmented Zenoh messages into a contiguous buffer.
///
/// Only one fragmented message can be in-flight at a time (MCU constraint).
/// If a new sequence starts while one is already active, the old one is discarded.
///
/// # Type parameter
///
/// - `CAP`: maximum reassembled message size in bytes.
pub struct FragmentAssembler<const CAP: usize> {
    buf: [u8; CAP],
    len: usize,
    active: bool,
}

impl<const CAP: usize> FragmentAssembler<CAP> {
    /// Create a new, idle assembler.
    pub const fn new() -> Self {
        Self {
            buf: [0u8; CAP],
            len: 0,
            active: false,
        }
    }

    /// Feed a fragment payload into the assembler.
    ///
    /// - `more`: `true` if more fragments follow (`M` flag set).
    /// - `payload`: the raw fragment body (after the Fragment header).
    ///
    /// Returns `Ok(Some(assembled_slice))` when the final fragment completes
    /// the message, `Ok(None)` for intermediate fragments, or
    /// `Err(FrameTooLarge)` if the reassembled data would exceed `CAP`.
    pub fn feed<'a>(
        &'a mut self,
        more: bool,
        payload: &[u8],
    ) -> Result<Option<&'a [u8]>, TransportError> {
        if !self.active {
            // Start of a new fragmented message (or single non-M fragment).
            self.len = 0;
            self.active = true;
        }

        let new_len = self.len + payload.len();
        if new_len > CAP {
            // Discard — too large for our buffer.
            self.reset();
            return Err(TransportError::FrameTooLarge);
        }

        self.buf[self.len..new_len].copy_from_slice(payload);
        self.len = new_len;

        if more {
            // Intermediate fragment — wait for more.
            Ok(None)
        } else {
            // Final fragment — yield the assembled message.
            self.active = false;
            Ok(Some(&self.buf[..self.len]))
        }
    }

    /// Discard any in-progress reassembly.
    pub fn reset(&mut self) {
        self.len = 0;
        self.active = false;
    }

    /// Whether a fragmented message is currently being assembled.
    pub fn is_active(&self) -> bool {
        self.active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_fragment_no_more() {
        let mut asm = FragmentAssembler::<256>::new();
        let payload = b"hello world";
        let result = asm.feed(false, payload).unwrap();
        assert_eq!(result, Some(payload.as_slice()));
        assert!(!asm.is_active());
    }

    #[test]
    fn test_two_fragments() {
        let mut asm = FragmentAssembler::<256>::new();
        assert_eq!(asm.feed(true, b"hello ").unwrap(), None);
        assert!(asm.is_active());

        let result = asm.feed(false, b"world").unwrap();
        assert_eq!(result, Some(b"hello world".as_slice()));
        assert!(!asm.is_active());
    }

    #[test]
    fn test_three_fragments() {
        let mut asm = FragmentAssembler::<256>::new();
        assert_eq!(asm.feed(true, b"aaa").unwrap(), None);
        assert_eq!(asm.feed(true, b"bbb").unwrap(), None);
        let result = asm.feed(false, b"ccc").unwrap();
        assert_eq!(result, Some(b"aaabbbccc".as_slice()));
    }

    #[test]
    fn test_overflow_returns_error() {
        let mut asm = FragmentAssembler::<8>::new();
        assert_eq!(asm.feed(true, b"1234").unwrap(), None);
        let err = asm.feed(false, b"56789");
        assert_eq!(err, Err(TransportError::FrameTooLarge));
        assert!(!asm.is_active());
    }

    #[test]
    fn test_reset_clears_state() {
        let mut asm = FragmentAssembler::<256>::new();
        assert_eq!(asm.feed(true, b"partial").unwrap(), None);
        assert!(asm.is_active());
        asm.reset();
        assert!(!asm.is_active());

        // New sequence works after reset.
        let result = asm.feed(false, b"fresh").unwrap();
        assert_eq!(result, Some(b"fresh".as_slice()));
    }

    #[test]
    fn test_new_sequence_replaces_old() {
        let mut asm = FragmentAssembler::<256>::new();
        // Start first sequence.
        assert_eq!(asm.feed(true, b"old_").unwrap(), None);
        // Simulate: the old sequence is abandoned (e.g. packet loss).
        // Manually reset to start new sequence — in real code the caller
        // detects SN discontinuity and calls reset().
        asm.reset();

        // New sequence.
        assert_eq!(asm.feed(true, b"new_").unwrap(), None);
        let result = asm.feed(false, b"data").unwrap();
        assert_eq!(result, Some(b"new_data".as_slice()));
    }
}
