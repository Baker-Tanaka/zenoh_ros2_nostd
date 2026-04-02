//! KeepAlive management — periodic sending and timeout detection.

use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;

use super::codec;
use super::frame;
use crate::error::TransportError;

/// KeepAlive manager for maintaining the zenoh session.
pub struct KeepAliveManager {
    /// Interval between KeepAlive sends.
    interval: Duration,
    /// Timeout before considering the peer dead (typically = lease time).
    timeout: Duration,
    /// Last time we received any message from the peer.
    last_rx: Instant,
}

impl KeepAliveManager {
    /// Create a new KeepAlive manager.
    ///
    /// - `lease_ms`: the lease time negotiated during handshake.
    ///   KeepAlive messages are sent at 1/4 of the lease interval.
    pub fn new(lease_ms: u64) -> Self {
        let lease = Duration::from_millis(lease_ms);
        Self {
            interval: lease / 4,
            timeout: lease,
            last_rx: Instant::now(),
        }
    }

    /// Record that a message was received from the peer.
    pub fn mark_rx(&mut self) {
        self.last_rx = Instant::now();
    }

    /// Check if the peer has timed out (no messages within the lease period).
    pub fn is_timed_out(&self) -> bool {
        self.last_rx.elapsed() > self.timeout
    }

    /// Wait until the next KeepAlive should be sent.
    pub async fn wait_next_tick(&self) {
        Timer::after(self.interval).await;
    }

    /// Send a KeepAlive message.
    pub async fn send<W: Write>(
        &self,
        writer: &mut W,
        tx_buf: &mut [u8],
    ) -> Result<(), TransportError> {
        let n = codec::encode_keepalive(tx_buf)?;
        frame::write_frame(writer, &tx_buf[..n]).await
    }
}
