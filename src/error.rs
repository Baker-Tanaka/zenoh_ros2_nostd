//! Unified error types for the crate.

use core::fmt;

/// Top-level crate error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// CDR serialization / deserialization error.
    Cdr(crate::cdr::CdrError),
    /// Transport-layer error.
    Transport(TransportError),
    /// Session-layer error.
    Session(SessionError),
    /// I/O error (read/write failure).
    Io,
    /// Buffer is full — cannot enqueue.
    BufferFull,
    /// Not connected to zenoh router.
    NotConnected,
    /// Operation timed out.
    Timeout,
    /// Invalid argument supplied.
    InvalidArgument,
    /// Service call received no reply (ResponseFinal without Reply).
    ServiceNoReply,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Cdr(e) => write!(f, "CDR: {}", e),
            Self::Transport(e) => write!(f, "Transport: {}", e),
            Self::Session(e) => write!(f, "Session: {}", e),
            Self::Io => write!(f, "I/O error"),
            Self::BufferFull => write!(f, "buffer full"),
            Self::NotConnected => write!(f, "not connected"),
            Self::Timeout => write!(f, "timeout"),
            Self::InvalidArgument => write!(f, "invalid argument"),
            Self::ServiceNoReply => write!(f, "service: no reply received"),
        }
    }
}

impl From<crate::cdr::CdrError> for Error {
    fn from(e: crate::cdr::CdrError) -> Self {
        Self::Cdr(e)
    }
}

impl From<TransportError> for Error {
    fn from(e: TransportError) -> Self {
        Self::Transport(e)
    }
}

impl From<SessionError> for Error {
    fn from(e: SessionError) -> Self {
        Self::Session(e)
    }
}

/// Transport-layer errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    /// TCP/UDP I/O failure.
    Io,
    /// Unexpected message received during handshake.
    HandshakeFailed,
    /// Protocol version mismatch.
    VersionMismatch,
    /// Frame too large for buffer.
    FrameTooLarge,
    /// Invalid wire encoding.
    InvalidEncoding,
    /// Connection closed by remote.
    ConnectionClosed,
    /// KeepAlive timeout — peer unresponsive.
    KeepAliveTimeout,
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io => write!(f, "I/O error"),
            Self::HandshakeFailed => write!(f, "handshake failed"),
            Self::VersionMismatch => write!(f, "version mismatch"),
            Self::FrameTooLarge => write!(f, "frame too large"),
            Self::InvalidEncoding => write!(f, "invalid encoding"),
            Self::ConnectionClosed => write!(f, "connection closed"),
            Self::KeepAliveTimeout => write!(f, "keepalive timeout"),
        }
    }
}

/// Session-layer errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionError {
    /// Maximum number of resources reached.
    ResourceLimitReached,
    /// Maximum number of publishers reached.
    PublisherLimitReached,
    /// Maximum number of subscribers reached.
    SubscriberLimitReached,
    /// Resource ID not found.
    ResourceNotFound,
    /// Session is closing or already closed.
    Closed,
}

impl fmt::Display for SessionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ResourceLimitReached => write!(f, "resource limit reached"),
            Self::PublisherLimitReached => write!(f, "publisher limit reached"),
            Self::SubscriberLimitReached => write!(f, "subscriber limit reached"),
            Self::ResourceNotFound => write!(f, "resource not found"),
            Self::Closed => write!(f, "session closed"),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Cdr(e) => defmt::write!(f, "CDR: {}", e),
            Self::Transport(e) => defmt::write!(f, "Transport: {}", e),
            Self::Session(e) => defmt::write!(f, "Session: {}", e),
            Self::Io => defmt::write!(f, "I/O error"),
            Self::BufferFull => defmt::write!(f, "buffer full"),
            Self::NotConnected => defmt::write!(f, "not connected"),
            Self::Timeout => defmt::write!(f, "timeout"),
            Self::InvalidArgument => defmt::write!(f, "invalid argument"),
            Self::ServiceNoReply => defmt::write!(f, "service: no reply received"),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for TransportError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::Io => defmt::write!(f, "I/O error"),
            Self::HandshakeFailed => defmt::write!(f, "handshake failed"),
            Self::VersionMismatch => defmt::write!(f, "version mismatch"),
            Self::FrameTooLarge => defmt::write!(f, "frame too large"),
            Self::InvalidEncoding => defmt::write!(f, "invalid encoding"),
            Self::ConnectionClosed => defmt::write!(f, "connection closed"),
            Self::KeepAliveTimeout => defmt::write!(f, "keepalive timeout"),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for SessionError {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Self::ResourceLimitReached => defmt::write!(f, "resource limit reached"),
            Self::PublisherLimitReached => defmt::write!(f, "publisher limit reached"),
            Self::SubscriberLimitReached => defmt::write!(f, "subscriber limit reached"),
            Self::ResourceNotFound => defmt::write!(f, "resource not found"),
            Self::Closed => defmt::write!(f, "session closed"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::CdrError;
    extern crate alloc;
    use alloc::format;

    #[test]
    fn test_from_transport_error() {
        let te = TransportError::Io;
        let e: Error = te.into();
        assert_eq!(e, Error::Transport(TransportError::Io));
    }

    #[test]
    fn test_from_session_error() {
        let se = SessionError::Closed;
        let e: Error = se.into();
        assert_eq!(e, Error::Session(SessionError::Closed));
    }

    #[test]
    fn test_from_cdr_error() {
        let ce = CdrError::BufferOverflow;
        let e: Error = ce.into();
        assert_eq!(e, Error::Cdr(CdrError::BufferOverflow));
    }

    #[test]
    fn test_error_display() {
        let e = Error::NotConnected;
        let s = format!("{}", e);
        assert_eq!(s, "not connected");
    }

    #[test]
    fn test_transport_error_display() {
        let e = TransportError::FrameTooLarge;
        let s = format!("{}", e);
        assert_eq!(s, "frame too large");
    }

    #[test]
    fn test_session_error_display() {
        let e = SessionError::ResourceNotFound;
        let s = format!("{}", e);
        assert_eq!(s, "resource not found");
    }

    #[test]
    fn test_error_clone_copy() {
        let e1 = Error::Timeout;
        let e2 = e1; // Copy
        let e3 = e1.clone(); // Clone
        assert_eq!(e1, e2);
        assert_eq!(e1, e3);
    }
}
