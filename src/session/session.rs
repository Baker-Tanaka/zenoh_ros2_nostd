//! Core session state machine.
//!
//! The session manages the zenoh transport connection, handles
//! declaration of key expressions, and routes incoming data to
//! subscribers.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_io_async::{Read, Write};

use crate::error::Error;
use crate::transport::{codec, frame, handshake, protocol::*};

/// Session state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    /// Not connected.
    Disconnected,
    /// Handshake in progress.
    Connecting,
    /// Session is open and operational.
    Open,
    /// Closing gracefully.
    Closing,
}

/// Configuration for creating a session.
pub struct SessionConfig {
    /// Our zenoh ID (should be unique per device).
    pub zid: ZenohId,
    /// ROS2 domain ID (default: 0).
    pub domain_id: u32,
}

impl SessionConfig {
    /// Create a new config with the given ZenohId.
    pub fn new(zid: ZenohId) -> Self {
        Self { zid, domain_id: 0 }
    }
}

/// The zenoh session.
///
/// Generic over:
/// - `T`: the transport link (Read + Write)
/// - `TX_BUF`: transmit buffer size
/// - `RX_BUF`: receive buffer size
pub struct Session<T: Read + Write, const TX_BUF: usize = 512, const RX_BUF: usize = 4096> {
    link: Mutex<CriticalSectionRawMutex, T>,
    config: SessionConfig,
    state: Mutex<CriticalSectionRawMutex, SessionState>,
    /// Next sequence number for reliable frames.
    sn_reliable: Mutex<CriticalSectionRawMutex, u64>,
    /// Next sequence number for best-effort frames.
    #[allow(dead_code)]
    sn_best_effort: Mutex<CriticalSectionRawMutex, u64>,
    /// Next key expression ID for declarations.
    next_key_id: Mutex<CriticalSectionRawMutex, u16>,
    /// Negotiated lease time in ms.
    lease_ms: u64,
}

impl<T: Read + Write, const TX_BUF: usize, const RX_BUF: usize> Session<T, TX_BUF, RX_BUF> {
    /// Open a session by performing the handshake over the given link.
    pub async fn open(link: T, config: SessionConfig) -> Result<Self, Error> {
        let session = Self {
            link: Mutex::new(link),
            config,
            state: Mutex::new(SessionState::Disconnected),
            sn_reliable: Mutex::new(0),
            sn_best_effort: Mutex::new(0),
            next_key_id: Mutex::new(1),
            lease_ms: 0,
        };

        session.do_handshake().await?;

        Ok(session)
    }

    /// Perform the transport handshake.
    async fn do_handshake(&self) -> Result<handshake::HandshakeResult, Error> {
        *self.state.lock().await = SessionState::Connecting;

        let mut tx_buf = [0u8; TX_BUF];
        let mut rx_buf = [0u8; RX_BUF];

        let result = {
            let mut link = self.link.lock().await;
            handshake::client_handshake(&mut *link, &self.config.zid, &mut tx_buf, &mut rx_buf)
                .await
                .map_err(Error::Transport)?
        };

        *self.state.lock().await = SessionState::Open;

        ros2_info!("session open, router={:?}", result.router_zid);

        Ok(result)
    }

    /// Get the current session state.
    pub async fn state(&self) -> SessionState {
        *self.state.lock().await
    }

    /// Declare a key expression and get its numeric ID.
    ///
    /// Key expressions declared this way can be referenced by ID in
    /// subsequent publish/subscribe operations for efficiency.
    pub async fn declare_key_expr(&self, key_expr: &str) -> Result<u16, Error> {
        let key_id = {
            let mut id = self.next_key_id.lock().await;
            let current = *id;
            *id = id.wrapping_add(1);
            current
        };

        let mut tx_buf = [0u8; TX_BUF];

        // Encode declare keyexpr inside a frame
        let mut pos = 0;
        let sn = self.next_sn_reliable().await;
        pos +=
            codec::encode_frame_header(&mut tx_buf[pos..], sn, true).map_err(Error::Transport)?;
        pos += codec::encode_declare_keyexpr(&mut tx_buf[pos..], key_id, key_expr)
            .map_err(Error::Transport)?;

        let mut link = self.link.lock().await;
        frame::write_frame(&mut *link, &tx_buf[..pos])
            .await
            .map_err(Error::Transport)?;

        ros2_debug!("declared keyexpr id={} expr={}", key_id, key_expr);

        Ok(key_id)
    }

    /// Publish raw payload with rmw_zenoh_cpp publisher attachment.
    ///
    /// The attachment includes the sequence number, timestamp, and publisher GID,
    /// which allows `rmw_zenoh_cpp` subscribers to identify the source and track ordering.
    ///
    /// - `seq_num`: monotonically increasing sequence number per publisher.
    /// - `timestamp_ns`: nanoseconds since UNIX epoch; pass `0` if RTC is unavailable.
    /// - `gid`: publisher identity (typically the session's ZenohId).
    pub async fn put_with_attachment(
        &self,
        key_expr: &str,
        payload: &[u8],
        seq_num: i64,
        timestamp_ns: i64,
        gid: &ZenohId,
    ) -> Result<(), Error> {
        let mut tx_buf = [0u8; TX_BUF];

        let mut pos = 0;
        let sn = self.next_sn_reliable().await;
        pos +=
            codec::encode_frame_header(&mut tx_buf[pos..], sn, true).map_err(Error::Transport)?;
        pos += codec::encode_push_put_with_attachment(
            &mut tx_buf[pos..],
            key_expr,
            payload,
            seq_num,
            timestamp_ns,
            gid,
        )
        .map_err(Error::Transport)?;

        let mut link = self.link.lock().await;
        frame::write_frame(&mut *link, &tx_buf[..pos])
            .await
            .map_err(Error::Transport)?;

        Ok(())
    }

    /// Publish raw payload to a key expression (inline, not pre-declared).
    pub async fn put(&self, key_expr: &str, payload: &[u8]) -> Result<(), Error> {
        let mut tx_buf = [0u8; TX_BUF];

        let mut pos = 0;
        let sn = self.next_sn_reliable().await;
        pos +=
            codec::encode_frame_header(&mut tx_buf[pos..], sn, true).map_err(Error::Transport)?;
        // Encoding ID 0 = zenoh's application/octet-stream
        pos += codec::encode_push_put(&mut tx_buf[pos..], key_expr, 0, payload)
            .map_err(Error::Transport)?;

        let mut link = self.link.lock().await;
        frame::write_frame(&mut *link, &tx_buf[..pos])
            .await
            .map_err(Error::Transport)?;

        Ok(())
    }

    /// Send a KeepAlive message.
    pub async fn keepalive(&self) -> Result<(), Error> {
        let mut tx_buf = [0u8; 8];
        let mut link = self.link.lock().await;
        let n = codec::encode_keepalive(&mut tx_buf).map_err(Error::Transport)?;
        frame::write_frame(&mut *link, &tx_buf[..n])
            .await
            .map_err(Error::Transport)?;
        Ok(())
    }

    /// Declare a subscriber for a key expression.
    ///
    /// First declares the key expression with a numeric ID, then
    /// sends a DeclareSubscriber referencing that ID. Returns the
    /// key ID  assigned to the key expression.
    pub async fn subscribe(&self, key_expr: &str) -> Result<u16, Error> {
        let key_id = self.declare_key_expr(key_expr).await?;

        let sub_id = key_id as u32;
        let mut tx_buf = [0u8; TX_BUF];
        let mut pos = 0;
        let sn = self.next_sn_reliable().await;
        pos +=
            codec::encode_frame_header(&mut tx_buf[pos..], sn, true).map_err(Error::Transport)?;
        pos += codec::encode_declare_subscriber_mapped(&mut tx_buf[pos..], sub_id, key_id)
            .map_err(Error::Transport)?;

        let mut link = self.link.lock().await;
        frame::write_frame(&mut *link, &tx_buf[..pos])
            .await
            .map_err(Error::Transport)?;

        ros2_debug!("subscribed key_id={} expr={}", key_id, key_expr);
        Ok(key_id)
    }

    /// Read one incoming frame and return the first Push+Put payload found.
    ///
    /// Returns `Some((key_suffix, payload_slice))` if a PUSH/PUT was received.
    /// Returns `None` for KeepAlive or Declare frames (caller should call again).
    /// The payload is written into `rx_buf`; the returned slices borrow from it.
    pub async fn recv_once<'a>(
        &self,
        rx_buf: &'a mut [u8],
    ) -> Result<Option<(&'a str, &'a [u8])>, Error> {
        let n = {
            let mut link = self.link.lock().await;
            frame::read_frame(&mut *link, rx_buf)
                .await
                .map_err(Error::Transport)?
        };

        let msg_buf = &rx_buf[..n];

        // Parse the transport message
        let header = msg_buf[0];
        match codec::parse_transport_msg_kind(header) {
            codec::TransportMsgKind::Frame => {
                let (_, _, body_pos) =
                    codec::decode_frame_header(msg_buf).map_err(Error::Transport)?;
                let body = &msg_buf[body_pos..];

                match codec::decode_push_put(body).map_err(Error::Transport)? {
                    Some((put, _)) => Ok(Some((put.key_suffix, put.payload))),
                    None => Ok(None),
                }
            }
            codec::TransportMsgKind::KeepAlive => Ok(None),
            codec::TransportMsgKind::Close => Err(Error::Transport(
                crate::error::TransportError::ConnectionClosed,
            )),
            _ => Ok(None),
        }
    }

    /// Close the session gracefully.
    pub async fn close(&self) -> Result<(), Error> {
        *self.state.lock().await = SessionState::Closing;

        let mut tx_buf = [0u8; 8];
        let n =
            codec::encode_close(&mut tx_buf, close_reason::GENERIC).map_err(Error::Transport)?;

        let mut link = self.link.lock().await;
        frame::write_frame(&mut *link, &tx_buf[..n])
            .await
            .map_err(Error::Transport)?;

        *self.state.lock().await = SessionState::Disconnected;

        ros2_info!("session closed");
        Ok(())
    }

    /// Get the lease time for KeepAlive scheduling.
    pub fn lease_ms(&self) -> u64 {
        self.lease_ms
    }

    /// Get the session's ZenohId.
    pub fn zid(&self) -> &ZenohId {
        &self.config.zid
    }

    /// Get the domain ID.
    pub fn domain_id(&self) -> u32 {
        self.config.domain_id
    }

    async fn next_sn_reliable(&self) -> u64 {
        let mut sn = self.sn_reliable.lock().await;
        let current = *sn;
        *sn = sn.wrapping_add(1);
        current
    }
}
