//! Transport link abstraction over `embedded-io-async`.

use embedded_io_async::{Read, Write};

/// A bidirectional link (TCP socket or similar).
///
/// This is a thin wrapper trait to allow passing read + write halves
/// together or separately.
pub trait TransportLink: Read + Write {}

/// Blanket impl: anything that is Read + Write is a TransportLink.
impl<T: Read + Write> TransportLink for T {}
