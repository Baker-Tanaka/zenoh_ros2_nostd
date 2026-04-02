//! CDR error types.

use core::fmt;

/// Errors that can occur during CDR serialization/deserialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdrError {
    /// Output buffer is too small.
    BufferOverflow,
    /// Input buffer ended unexpectedly.
    BufferUnderflow,
    /// Boolean value was not 0 or 1.
    InvalidBool,
    /// String data was not valid UTF-8.
    InvalidUtf8,
    /// String too long for the target heapless::String capacity.
    StringCapacityExceeded,
    /// Unsupported CDR encapsulation identifier.
    UnsupportedEncapsulation,
    /// Generic serialization error (from serde).
    SerializeError,
    /// Generic deserialization error (from serde).
    DeserializeError,
}

impl fmt::Display for CdrError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BufferOverflow => write!(f, "buffer overflow"),
            Self::BufferUnderflow => write!(f, "buffer underflow"),
            Self::InvalidBool => write!(f, "invalid bool value"),
            Self::InvalidUtf8 => write!(f, "invalid UTF-8"),
            Self::StringCapacityExceeded => write!(f, "string capacity exceeded"),
            Self::UnsupportedEncapsulation => write!(f, "unsupported CDR encapsulation"),
            Self::SerializeError => write!(f, "serialization error"),
            Self::DeserializeError => write!(f, "deserialization error"),
        }
    }
}

impl serde::ser::Error for CdrError {
    fn custom<T: fmt::Display>(_msg: T) -> Self {
        Self::SerializeError
    }
}

impl serde::de::Error for CdrError {
    fn custom<T: fmt::Display>(_msg: T) -> Self {
        Self::DeserializeError
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for CdrError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::BufferOverflow => defmt::write!(f, "buffer overflow"),
            Self::BufferUnderflow => defmt::write!(f, "buffer underflow"),
            Self::InvalidBool => defmt::write!(f, "invalid bool"),
            Self::InvalidUtf8 => defmt::write!(f, "invalid UTF-8"),
            Self::StringCapacityExceeded => defmt::write!(f, "string capacity exceeded"),
            Self::UnsupportedEncapsulation => defmt::write!(f, "unsupported encapsulation"),
            Self::SerializeError => defmt::write!(f, "serialize error"),
            Self::DeserializeError => defmt::write!(f, "deserialize error"),
        }
    }
}
