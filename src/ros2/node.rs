//! ROS2 Node — builder, connection management, and session spin loop.
//!
//! # Overview
//!
//! ```text
//! NodeBuilder::new(zid)    ← configure identity & parameters
//!     .name("talker")
//!     .domain_id(0)
//!     .open(transport)     ← pass any T: Read+Write (WiFi, Ethernet, …)
//!     .await?              ↓
//!                       Node<T>  ← owns the session
//!                          │
//!                 register_publisher(&STATIC_PUB)
//!                 subscribe(TOPIC, &STATIC_SUB).await?
//!                          │
//!                       spin(&mut rx_buf).await  ← drives RX + TX + keepalive
//! ```
//!
//! # Transport independence
//!
//! [`NodeBuilder::open`] accepts **any** `T: Read + Write`.  Swap WiFi for
//! Ethernet (or any other transport) without touching Node, Publisher, or
//! Subscription code.

use embassy_time::{Duration, with_timeout};
use embedded_io_async::{Read, Write};
use heapless::Vec;

use super::keyexpr::TopicKeyExpr;
use super::publisher::PublisherDrain;
use super::subscription::SubscriptionDispatch;
use crate::error::Error;
use crate::transport::{codec, frame, handshake, protocol::ZenohId};

// ── Internal frame buffer sizes ───────────────────────────────────────────────

/// Frame buffer size for TX (keepalive + encoded Put frames).
///
/// Must be large enough to hold:
/// frame header (~10 B) + Push/Put header (~10 B) + key expression (~130 B) +
/// CDR payload + rmw attachment (33 B).
///
/// 1024 B covers typical messages; increase if you have larger payloads.
const TX_FRAME_BUF: usize = 1024;

/// Scratch buffer for draining publisher CDR payloads before encoding.
///
/// Must be ≥ `CDR_CAP` of any registered publisher.
/// 512 B covers `std_msgs/String` up to ~500 bytes of text.
const CDR_DRAIN_BUF: usize = 512;

/// Minimum accepted lease time in milliseconds.
///
/// Some routers may advertise very short leases; this floor ensures
/// keepalives are not sent more frequently than every second.
const MIN_LEASE_MS: u64 = 2_000;
const MAX_PUBS: usize = 4;

/// Maximum subscriptions per Node.
const MAX_SUBS: usize = 4;

// ── NodeBuilder ───────────────────────────────────────────────────────────────

/// Builder for a [`Node`].
///
/// Set identity and parameters, then call [`open`](Self::open) with a
/// connected transport to create the node.
///
/// # Example
/// ```rust,ignore
/// let mut node = NodeBuilder::new(MY_ZID)
///     .name("talker")
///     .domain_id(0)
///     .open(socket)
///     .await?;
/// ```
pub struct NodeBuilder {
    node_name: &'static str,
    namespace: &'static str,
    domain_id: u32,
    zid: ZenohId,
}

impl NodeBuilder {
    /// Create a builder with the given device Zenoh ID.
    ///
    /// The ZID **must be unique** across all devices in the network.
    /// Use the device MAC address or a hard-coded constant per device.
    pub const fn new(zid: ZenohId) -> Self {
        Self {
            node_name: "node",
            namespace: "",
            domain_id: 0,
            zid,
        }
    }

    /// Set the ROS2 node name (e.g., `"talker"`).
    pub const fn name(mut self, name: &'static str) -> Self {
        self.node_name = name;
        self
    }

    /// Set the node namespace (e.g., `""` for root, `"/robot1"` for namespaced).
    pub const fn namespace(mut self, ns: &'static str) -> Self {
        self.namespace = ns;
        self
    }

    /// Set the ROS2 domain ID (default: `0`).  Must match `ROS_DOMAIN_ID`.
    pub const fn domain_id(mut self, id: u32) -> Self {
        self.domain_id = id;
        self
    }

    /// Perform the Zenoh handshake over `transport` and create the [`Node`].
    ///
    /// `transport` is any already-connected `T: Read + Write` — e.g. a WiFi
    /// TCP socket, an Ethernet socket, or any byte stream.  The SDK never
    /// references the concrete transport type; swap freely without changing
    /// any other code.
    ///
    /// Returns `Err` if the handshake fails (wrong router version, I/O error,
    /// etc.).  The caller should retry after a back-off delay.
    pub async fn open<T: Read + Write>(self, mut transport: T) -> Result<Node<T>, Error> {
        let mut hs_tx = [0u8; 512];
        let mut hs_rx = [0u8; 4096];
        let hs =
            handshake::client_handshake(&mut transport, &self.zid, &mut hs_tx, &mut hs_rx)
                .await
                .map_err(Error::Transport)?;

        let lease_ms = hs.lease_ms.max(MIN_LEASE_MS);

        ros2_info!(
            "node '{}': connected (lease={}ms, router={:?})",
            self.node_name,
            lease_ms,
            hs.router_zid
        );

        Ok(Node {
            node_name: self.node_name,
            namespace: self.namespace,
            transport,
            sn: 0,
            key_id_gen: 1,
            gid: self.zid,
            lease_ms,
            publishers: Vec::new(),
            subscribers: Vec::new(),
        })
    }
}

// ── Node ──────────────────────────────────────────────────────────────────────

/// An active ROS2 node.
///
/// Created by [`NodeBuilder::open`].  Owns the transport and drives the full
/// Zenoh session: serialization, declaration, keepalive, and dispatch.
///
/// **Single-task**: the node is not `Send`/`Sync` by default (it owns `T`).
/// Keep it inside the Zenoh task; use static [`Publisher`](super::Publisher) /
/// [`Subscription`](super::Subscription) channels to communicate with other
/// tasks.
pub struct Node<T: Read + Write> {
    node_name: &'static str,
    namespace: &'static str,
    transport: T,
    /// Session-level sequence number (reliable channel).
    sn: u64,
    /// Next key expression ID to assign.
    key_id_gen: u16,
    /// This node's ZenohId — used as publisher GID in the attachment.
    gid: ZenohId,
    /// Keepalive lease negotiated during handshake (milliseconds).
    lease_ms: u64,
    /// Registered static publisher drains.
    publishers: Vec<&'static dyn PublisherDrain, MAX_PUBS>,
    /// Registered subscriptions: `(key_id, dispatch)`.
    subscribers: Vec<(u16, &'static dyn SubscriptionDispatch), MAX_SUBS>,
}

impl<T: Read + Write> Node<T> {
    // ── Setup methods ─────────────────────────────────────────────────────────

    /// Register a static [`Publisher`](super::Publisher) with this node.
    ///
    /// After registration, [`spin`](Self::spin) will drain the publisher's
    /// internal queue and transmit enqueued messages automatically.
    ///
    /// This call does **not** send any wire messages — it only registers the
    /// publisher for draining.
    pub fn register_publisher(&mut self, publisher: &'static dyn PublisherDrain) {
        let _ = self.publishers.push(publisher);
    }

    /// Declare a topic subscription on the router.
    ///
    /// Sends `DeclareKeyExpr` + `DeclareSubscriber` to the router so that
    /// incoming publications on `topic` are forwarded to this session.
    ///
    /// Received payloads are pushed into `sub`'s internal queue; call
    /// [`Subscription::try_recv`](super::Subscription::try_recv) (or `recv`)
    /// from any task to read them.
    ///
    /// Returns the key ID assigned to `topic` (useful for diagnostics).
    pub async fn subscribe(
        &mut self,
        topic: TopicKeyExpr,
        sub: &'static dyn SubscriptionDispatch,
    ) -> Result<u16, Error> {
        let ke = topic.to_key_expr().map_err(|_| Error::InvalidArgument)?;
        let key_id = self.declare_key_expr(ke.as_str()).await?;
        self.declare_subscriber(key_id).await?;
        let _ = self.subscribers.push((key_id, sub));
        ros2_info!(
            "node '{}': subscribed to {} (key_id={})",
            self.node_name,
            ke.as_str(),
            key_id
        );
        Ok(key_id)
    }

    // ── Session loop ──────────────────────────────────────────────────────────

    /// Drive the session: drain publisher queues, receive and dispatch incoming
    /// frames, and send keepalives.
    ///
    /// **Returns only when the connection drops** (I/O error, remote close, or
    /// protocol error).  After it returns, the caller should reconnect via
    /// [`NodeBuilder`].
    ///
    /// `rx_buf` is the receive staging buffer.  It must be large enough for
    /// the largest expected incoming Zenoh frame (4096 B is safe).
    ///
    /// # Session loop behaviour
    /// 1. Drain all registered publisher queues (non-blocking TX).
    /// 2. Wait up to `lease_ms / 2` for an incoming frame.
    /// 3. If a frame arrives: dispatch to matching subscribers.
    /// 4. If the timer fires: drain publishers again, then send a keepalive.
    pub async fn spin(&mut self, rx_buf: &mut [u8]) {
        // Send a keepalive at half the lease interval to ensure the peer
        // receives it before the lease expires, even under moderate network jitter.
        let keepalive_interval = Duration::from_millis(self.lease_ms / 2);

        loop {
            // ── 1. Drain pending publisher queues (TX, non-blocking) ──────────
            if self.drain_publishers().await.is_err() {
                return;
            }

            // ── 2. Wait for next incoming frame (or keepalive timeout) ────────
            match with_timeout(keepalive_interval, frame::read_frame(&mut self.transport, rx_buf))
                .await
            {
                Ok(Ok(n)) => {
                    if self.dispatch_frame(&rx_buf[..n]).is_err() {
                        return;
                    }
                }
                Ok(Err(_)) => {
                    // I/O error — session is dead
                    return;
                }
                Err(_timeout) => {
                    // Keepalive window: drain publishers once more, then ping
                    if self.drain_publishers().await.is_err() {
                        return;
                    }
                    if self.send_keepalive().await.is_err() {
                        return;
                    }
                }
            }
        }
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /// Node name.
    pub fn node_name(&self) -> &str {
        self.node_name
    }

    /// Node namespace.
    pub fn namespace(&self) -> &str {
        self.namespace
    }

    /// Negotiated keepalive lease in milliseconds.
    pub fn lease_ms(&self) -> u64 {
        self.lease_ms
    }

    // ── Internal protocol helpers ─────────────────────────────────────────────

    async fn declare_key_expr(&mut self, key_expr: &str) -> Result<u16, Error> {
        let key_id = self.key_id_gen;
        self.key_id_gen = self.key_id_gen.wrapping_add(1);

        let mut buf = [0u8; 512];
        let mut pos =
            codec::encode_frame_header(&mut buf, self.sn, true).map_err(Error::Transport)?;
        self.sn = self.sn.wrapping_add(1);
        pos +=
            codec::encode_declare_keyexpr(&mut buf[pos..], key_id, key_expr)
                .map_err(Error::Transport)?;
        frame::write_frame(&mut self.transport, &buf[..pos])
            .await
            .map_err(Error::Transport)?;
        Ok(key_id)
    }

    async fn declare_subscriber(&mut self, key_id: u16) -> Result<(), Error> {
        let mut buf = [0u8; 128];
        let mut pos =
            codec::encode_frame_header(&mut buf, self.sn, true).map_err(Error::Transport)?;
        self.sn = self.sn.wrapping_add(1);
        pos +=
            codec::encode_declare_subscriber_mapped(&mut buf[pos..], key_id as u32, key_id)
                .map_err(Error::Transport)?;
        frame::write_frame(&mut self.transport, &buf[..pos])
            .await
            .map_err(Error::Transport)?;
        Ok(())
    }

    async fn drain_publishers(&mut self) -> Result<(), Error> {
        let mut cdr_scratch = [0u8; CDR_DRAIN_BUF];
        let mut frame_buf = [0u8; TX_FRAME_BUF];

        for i in 0..self.publishers.len() {
            let drain = self.publishers[i];
            // try_drain_into is non-blocking; loop until the queue is empty
            while let Some((n, seq, ts)) = drain.try_drain_into(&mut cdr_scratch) {
                let ke = drain
                    .topic_ke()
                    .to_key_expr()
                    .map_err(|_| Error::InvalidArgument)?;

                let mut pos = codec::encode_frame_header(&mut frame_buf, self.sn, true)
                    .map_err(Error::Transport)?;
                self.sn = self.sn.wrapping_add(1);

                pos += codec::encode_push_put_with_attachment(
                    &mut frame_buf[pos..],
                    ke.as_str(),
                    &cdr_scratch[..n],
                    seq,
                    ts,
                    &self.gid,
                )
                .map_err(Error::Transport)?;

                frame::write_frame(&mut self.transport, &frame_buf[..pos])
                    .await
                    .map_err(Error::Transport)?;
            }
        }
        Ok(())
    }

    async fn send_keepalive(&mut self) -> Result<(), Error> {
        let mut buf = [0u8; 8];
        let n = codec::encode_keepalive(&mut buf).map_err(Error::Transport)?;
        frame::write_frame(&mut self.transport, &buf[..n])
            .await
            .map_err(Error::Transport)?;
        Ok(())
    }

    fn dispatch_frame(&self, frame_buf: &[u8]) -> Result<(), Error> {
        use crate::transport::codec::{
            TransportMsgKind, decode_frame_header, decode_push_put, parse_transport_msg_kind,
        };

        if frame_buf.is_empty() {
            return Ok(());
        }

        match parse_transport_msg_kind(frame_buf[0]) {
            TransportMsgKind::Frame => {
                if let Ok((_, _, body_pos)) = decode_frame_header(frame_buf) {
                    if let Ok(Some((put, _))) = decode_push_put(&frame_buf[body_pos..]) {
                        let scope_id = put.scope as u16;
                        for (key_id, dispatch) in &self.subscribers {
                            if scope_id == *key_id {
                                dispatch.push_raw(put.payload);
                                break;
                            }
                        }
                    }
                }
                Ok(())
            }
            TransportMsgKind::Close => Err(Error::Transport(
                crate::error::TransportError::ConnectionClosed,
            )),
            _ => Ok(()),
        }
    }
}

