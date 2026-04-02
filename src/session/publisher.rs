//! Publisher handle for sending data through the session.

use embedded_io_async::{Read, Write};

use super::session::Session;
use crate::error::Error;

/// A publisher bound to a specific key expression.
///
/// Created via the ROS2 node API. Holds a reference to the session
/// and the key expression to publish on.
pub struct Publisher<'a, T: Read + Write, const TX_BUF: usize, const RX_BUF: usize> {
    session: &'a Session<T, TX_BUF, RX_BUF>,
    key_expr: &'a str,
}

impl<'a, T: Read + Write, const TX_BUF: usize, const RX_BUF: usize>
    Publisher<'a, T, TX_BUF, RX_BUF>
{
    /// Create a new publisher for the given key expression.
    pub fn new(session: &'a Session<T, TX_BUF, RX_BUF>, key_expr: &'a str) -> Self {
        Self { session, key_expr }
    }

    /// Publish raw bytes.
    pub async fn put(&self, payload: &[u8]) -> Result<(), Error> {
        self.session.put(self.key_expr, payload).await
    }

    /// Get the key expression this publisher is bound to.
    pub fn key_expr(&self) -> &str {
        self.key_expr
    }
}
