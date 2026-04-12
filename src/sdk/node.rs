//! rclpy-like Node — builder, publisher/subscription registration, and spin loop.
//!
//! # Overview
//!
//! ```text
//! Node::builder("talker")                   ← configure name & parameters
//!     .namespace("/robot1")
//!     .domain_id(0)
//!     .zid(my_zid)                          ← optional (auto-generated from name)
//!     .build(transport)                     ← any T: Read + Write
//!     .await?
//!         ↓
//!     Node<T>                               ← owns transport + internal node
//!         │
//!     register_static_publisher(&STATIC_PUB).await? → PublisherHandle<M>
//!     subscribe_with_dispatch(topic, &STATIC_SUB)  → SubscriptionHandle<M>
//!     register_timer(period)                       → TimerHandle
//!         │
//!     spin().await                          ← drives everything internally
//! ```

use embassy_time::{with_timeout, Duration, Instant};
use embedded_io_async::{Read, Write};
use heapless::Vec;
use serde::{Deserialize, Serialize};

use super::publisher::PublisherHandle;
use super::subscription::SubscriptionHandle;
use super::timer::TimerHandle;
use super::traits::NodeCallbacks;
use crate::error::Error;
use crate::ros2::keyexpr::{ActionKeyExprs, TopicKeyExpr};
use crate::ros2::liveliness::{self, EntityType};
use crate::ros2::publisher::{Publisher, PublisherDrain};
use crate::ros2::qos::Qos;
use crate::ros2::subscription::{Subscription, SubscriptionDispatch};
use crate::session::reconnect::ReconnectPolicy;
use crate::transport::fragment::FragmentAssembler;
use crate::transport::{codec, frame, handshake, protocol::ZenohId};

// ── Constants ─────────────────────────────────────────────────────────────────

/// Frame buffer size for TX.
const TX_FRAME_BUF: usize = 1024;

/// Scratch buffer for draining publisher CDR payloads.
const CDR_DRAIN_BUF: usize = 512;

/// Internal receive buffer size.
const RX_BUF_SIZE: usize = 4096;

/// Minimum accepted lease time in milliseconds.
const MIN_LEASE_MS: u64 = 2_000;

/// Maximum publishers per node.
const MAX_PUBS: usize = 4;

/// Maximum subscriptions per node.
const MAX_SUBS: usize = 4;

/// Maximum timers per node.
const MAX_TIMERS: usize = 4;

/// Fragment reassembly buffer size.
///
/// Must be large enough to hold the largest reassembled message.
/// Default 16 KiB handles common ROS2 messages (Twist, String, Odometry, etc.).
/// Increase for very large messages (PointCloud2, Image) if RAM permits.
const FRAGMENT_BUF_SIZE: usize = 16384;

/// Timer entry in the node.
struct TimerEntry {
    period: Duration,
    next_fire: Instant,
}

// ── NodeBuilder ───────────────────────────────────────────────────────────────

/// Builder for creating a [`Node`].
///
/// # Example
///
/// ```rust,ignore
/// let mut node = Node::builder("talker")
///     .domain_id(0)
///     .build(socket)
///     .await?;
/// ```
pub struct NodeBuilder {
    node_name: &'static str,
    namespace: &'static str,
    domain_id: u32,
    zid: Option<ZenohId>,
}

impl NodeBuilder {
    /// Create a new builder with the given node name.
    ///
    /// This is the primary entry point when you don't have a `Node<T>` yet:
    ///
    /// ```rust,ignore
    /// let mut node = NodeBuilder::new("talker")
    ///     .domain_id(0)
    ///     .build(socket)
    ///     .await?;
    /// ```
    pub const fn new(name: &'static str) -> Self {
        Self {
            node_name: name,
            namespace: "",
            domain_id: 0,
            zid: None,
        }
    }

    /// Set the ROS2 node name.
    pub const fn name(mut self, name: &'static str) -> Self {
        self.node_name = name;
        self
    }

    /// Set the node namespace (e.g., `"/robot1"`).
    pub const fn namespace(mut self, ns: &'static str) -> Self {
        self.namespace = ns;
        self
    }

    /// Set the ROS2 domain ID (default: `0`).
    pub const fn domain_id(mut self, id: u32) -> Self {
        self.domain_id = id;
        self
    }

    /// Set the Zenoh node ID explicitly.
    ///
    /// If not set, a default ID is generated from the node name.
    /// For production MCU use, always set a unique ZID (e.g., from MAC address).
    pub const fn zid(mut self, zid: ZenohId) -> Self {
        self.zid = Some(zid);
        self
    }

    /// Perform the Zenoh handshake and create the [`Node`].
    ///
    /// `transport` is any `T: Read + Write` — WiFi socket, Ethernet, WASI TCP, etc.
    pub async fn build<T: Read + Write>(self, mut transport: T) -> Result<Node<T>, Error> {
        let zid = self
            .zid
            .unwrap_or_else(|| Self::default_zid(self.node_name));

        let mut hs_tx = [0u8; 512];
        let mut hs_rx = [0u8; 4096];
        let hs = handshake::client_handshake(&mut transport, &zid, &mut hs_tx, &mut hs_rx)
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
            domain_id: self.domain_id,
            transport,
            sn: 0,
            key_id_gen: 1,
            entity_id_gen: 1,
            request_id_gen: 1,
            gid: zid,
            lease_ms,
            publishers: Vec::new(),
            subscribers: Vec::new(),
            timers: Vec::new(),
        })
    }

    /// Build a node together with a callbacks struct for the trait-based style.
    ///
    /// Register publishers, subscriptions and timers on the returned
    /// `Node`, then call [`Node::spin_with_callbacks`].
    pub async fn build_with_callbacks<T: Read + Write, C: NodeCallbacks>(
        self,
        transport: T,
        callbacks: C,
    ) -> Result<(Node<T>, C), Error> {
        let node = self.build(transport).await?;
        Ok((node, callbacks))
    }

    /// Generate a deterministic ZenohId from the node name.
    fn default_zid(name: &str) -> ZenohId {
        // Simple hash: sum of bytes, spread across 8 bytes
        let mut bytes = [0u8; 8];
        for (i, b) in name.bytes().enumerate() {
            bytes[i % 8] ^= b;
        }
        ZenohId::from_bytes(&bytes)
    }
}

// ── Node ──────────────────────────────────────────────────────────────────────

/// An active ROS2 node with rclpy-like ergonomics.
///
/// Created by [`Node::builder`].  Owns the transport and drives the full
/// Zenoh session: publish, subscribe, keepalive, and timer dispatch.
///
/// # Usage styles
///
/// **Callback style** — register handlers directly:
/// ```rust,ignore
/// let pub_h = node.register_static_publisher(&MY_PUB);
/// node.subscribe_with_dispatch(topic, &MY_SUB).await?;
/// node.spin().await;
/// ```
///
/// **Trait style** — implement `NodeCallbacks`:
/// ```rust,ignore
/// let (node, cbs) = Node::builder("name")
///     .build_with_callbacks(transport, my_callbacks)
///     .await?;
/// node.spin_with_callbacks(&mut cbs).await;
/// ```
pub struct Node<T: Read + Write> {
    node_name: &'static str,
    namespace: &'static str,
    domain_id: u32,
    transport: T,
    /// Session-level sequence number (reliable channel).
    sn: u64,
    /// Next key expression ID to assign.
    key_id_gen: u16,
    /// Next entity ID for liveliness tokens.
    entity_id_gen: u32,
    /// Next request ID for service calls.
    request_id_gen: u32,
    /// This node's ZenohId.
    gid: ZenohId,
    /// Keepalive lease negotiated during handshake (ms).
    lease_ms: u64,
    /// Registered publisher drains.
    publishers: Vec<&'static dyn PublisherDrain, MAX_PUBS>,
    /// Registered subscriptions: `(key_id, dispatch)`.
    subscribers: Vec<(u16, &'static dyn SubscriptionDispatch), MAX_SUBS>,
    /// Registered timers.
    timers: Vec<TimerEntry, MAX_TIMERS>,
}

impl<T: Read + Write> Node<T> {
    /// Create a [`NodeBuilder`] with the given node name.
    ///
    /// This is the primary entry point for creating nodes.
    ///
    /// ```rust,ignore
    /// let node = Node::builder("talker")
    ///     .domain_id(0)
    ///     .build(transport)
    ///     .await?;
    /// ```
    pub const fn builder(name: &'static str) -> NodeBuilder {
        NodeBuilder {
            node_name: name,
            namespace: "",
            domain_id: 0,
            zid: None,
        }
    }

    // ── Publisher registration ─────────────────────────────────────────────

    /// Register a static publisher and return a handle.
    ///
    /// The publisher must be a `static` item.  The returned handle can be
    /// used from any task to `publish()` messages.
    ///
    /// Sends a liveliness token to the router for rmw_zenoh_cpp graph discovery.
    ///
    /// ```rust,ignore
    /// static MY_PUB: Publisher<Twist, 256, 4> = Publisher::new(CMD_VEL_TOPIC);
    /// let handle = node.register_static_publisher(&MY_PUB).await?;
    /// handle.publish(&twist).await?;
    /// ```
    pub async fn register_static_publisher<
        M: Serialize,
        const CDR_CAP: usize,
        const QUEUE: usize,
    >(
        &mut self,
        publisher: &'static Publisher<M, CDR_CAP, QUEUE>,
    ) -> Result<PublisherHandle<M, CDR_CAP, QUEUE>, Error> {
        let topic = publisher.as_drain().topic_ke();
        self.declare_liveliness(EntityType::Publisher, topic)
            .await?;
        let _ = self.publishers.push(publisher.as_drain());
        Ok(PublisherHandle::new(publisher))
    }

    // ── Subscription registration ─────────────────────────────────────────

    /// Declare a topic subscription on the router and register the dispatch.
    ///
    /// Sends `DeclareKeyExpr` + `DeclareSubscriber` to the router.
    /// Incoming messages are pushed into `sub`'s internal queue.
    ///
    /// ```rust,ignore
    /// static MY_SUB: Subscription<StringMsg, 256, 4> = Subscription::new();
    /// let handle = node.subscribe_with_dispatch(CHATTER_TOPIC, &MY_SUB).await?;
    /// let msg = handle.recv().await?;
    /// ```
    pub async fn subscribe_with_dispatch<
        M: for<'de> Deserialize<'de>,
        const MSG_SIZE: usize,
        const QUEUE: usize,
    >(
        &mut self,
        topic: TopicKeyExpr,
        sub: &'static Subscription<M, MSG_SIZE, QUEUE>,
    ) -> Result<SubscriptionHandle<M, MSG_SIZE, QUEUE>, Error> {
        let ke = topic.to_key_expr().map_err(|_| Error::InvalidArgument)?;
        let key_id = self.declare_key_expr(ke.as_str()).await?;
        self.declare_subscriber(key_id).await?;
        self.declare_liveliness(EntityType::Subscriber, &topic)
            .await?;
        let _ = self.subscribers.push((key_id, sub.as_dispatch()));
        ros2_info!(
            "node '{}': subscribed to {} (key_id={})",
            self.node_name,
            ke.as_str(),
            key_id
        );
        Ok(SubscriptionHandle::new(sub))
    }

    // ── Timer registration ────────────────────────────────────────────────

    /// Register a periodic timer.
    ///
    /// The timer fires during `spin()`. Use the returned handle to identify it.
    pub fn register_timer(&mut self, period: Duration) -> TimerHandle {
        let id = self.timers.len() as u8;
        let _ = self.timers.push(TimerEntry {
            period,
            next_fire: Instant::now() + period,
        });
        TimerHandle { period, id }
    }

    // ── Service client ────────────────────────────────────────────────────

    /// Call a ROS2 service and wait for the response.
    ///
    /// Sends a Zenoh Request+Query (rmw_zenoh_cpp compatible) and waits for
    /// the matching Response+Reply.  While waiting, incoming Push messages
    /// are dispatched to subscribers normally.
    ///
    /// `CDR_REQ` / `CDR_RESP` are the CDR buffer sizes for the request and
    /// response payloads.  If the CDR-serialized request exceeds `CDR_REQ`
    /// or the response exceeds `CDR_RESP`, the call returns an error.
    ///
    /// `timeout` is the maximum time to wait for a reply.
    ///
    /// ```rust,ignore
    /// use serde::{Serialize, Deserialize};
    ///
    /// #[derive(Serialize)]
    /// struct AddTwoIntsReq { a: i64, b: i64 }
    ///
    /// #[derive(Deserialize)]
    /// struct AddTwoIntsResp { sum: i64 }
    ///
    /// let resp: AddTwoIntsResp = node.call_service::<_, _, 64, 64>(
    ///     &SERVICE_KE,
    ///     &AddTwoIntsReq { a: 1, b: 2 },
    ///     Duration::from_secs(5),
    /// ).await?;
    /// ```
    pub async fn call_service<
        Req: Serialize,
        Resp: for<'de> Deserialize<'de>,
        const CDR_REQ: usize,
        const CDR_RESP: usize,
    >(
        &mut self,
        service_ke: &TopicKeyExpr,
        request: &Req,
        timeout: Duration,
    ) -> Result<Resp, Error> {
        use crate::transport::codec::{decode_response, ResponseMsg};

        // Build key expression
        let ke = service_ke
            .to_key_expr()
            .map_err(|_| Error::InvalidArgument)?;

        // CDR-serialize the request
        let mut cdr_buf = [0u8; CDR_REQ];
        let cdr_len =
            crate::cdr::serialize_with_header(&mut cdr_buf, request).map_err(Error::Cdr)?;

        // Allocate request ID
        let request_id = self.request_id_gen;
        self.request_id_gen = self.request_id_gen.wrapping_add(1);

        // Declare liveliness for the service client (SC entity type)
        self.declare_liveliness(EntityType::ServiceClient, service_ke)
            .await?;

        // Encode Frame + Request + Query
        let mut frame_buf = [0u8; TX_FRAME_BUF];
        let mut pos =
            codec::encode_frame_header(&mut frame_buf, self.sn, true).map_err(Error::Transport)?;
        self.sn = self.sn.wrapping_add(1);

        pos += codec::encode_request_query(
            &mut frame_buf[pos..],
            request_id,
            ke.as_str(),
            &cdr_buf[..cdr_len],
            request_id as i64, // seq_num
            0,                 // no RTC timestamp
            &self.gid,
        )
        .map_err(Error::Transport)?;

        // Send the request
        frame::write_frame(&mut self.transport, &frame_buf[..pos])
            .await
            .map_err(Error::Transport)?;

        ros2_debug!(
            "node '{}': service call #{} to {}",
            self.node_name,
            request_id,
            ke.as_str()
        );

        // Wait for matching Response with timeout.
        // While waiting, dispatch incoming Push messages to subscribers.
        let mut rx_buf = [0u8; RX_BUF_SIZE];
        let mut frag_asm = FragmentAssembler::<FRAGMENT_BUF_SIZE>::new();

        loop {
            match with_timeout(timeout, frame::read_frame(&mut self.transport, &mut rx_buf)).await {
                Ok(Ok(n)) => {
                    if rx_buf[0] & 0x1F == crate::transport::protocol::transport_id::FRAME {
                        if let Ok((_, _, body_pos)) = codec::decode_frame_header(&rx_buf[..n]) {
                            // Try as Response
                            if let Ok(Some((msg, _))) = decode_response(&rx_buf[body_pos..n]) {
                                match msg {
                                    ResponseMsg::Reply(reply) if reply.request_id == request_id => {
                                        let (resp, _) =
                                            crate::cdr::deserialize_with_header::<Resp>(
                                                reply.payload,
                                            )
                                            .map_err(Error::Cdr)?;
                                        return Ok(resp);
                                    }
                                    ResponseMsg::Final(rid) if rid == request_id => {
                                        return Err(Error::ServiceNoReply);
                                    }
                                    _ => {} // not our response — continue
                                }
                            }

                            // Try as Push for subscriber dispatch
                            let _ = self.dispatch_frame(&rx_buf[..n], &mut frag_asm);
                        }
                    } else if rx_buf[0] & 0x1F == crate::transport::protocol::transport_id::CLOSE {
                        return Err(Error::Transport(
                            crate::error::TransportError::ConnectionClosed,
                        ));
                    }
                }
                Ok(Err(e)) => return Err(Error::Transport(e)),
                Err(_) => return Err(Error::Timeout),
            }
        }
    }

    // ── Action client ─────────────────────────────────────────────────────

    /// Send an action goal and wait for acceptance.
    ///
    /// `GoalReq` is the action-specific SendGoal request (typically contains
    /// [`GoalId`](crate::ros2::msg::action_msgs::GoalId) + user Goal).
    /// Returns the standard [`SendGoalResponse`] indicating acceptance.
    ///
    /// ```rust,ignore
    /// use zenoh_ros2_nostd::ros2::msg::action_msgs::*;
    ///
    /// #[derive(Serialize)]
    /// struct NavSendGoalReq { goal_id: GoalId, goal: MyGoal }
    ///
    /// let resp = node.send_goal::<_, 128>(
    ///     &action_ke.send_goal,
    ///     &NavSendGoalReq { goal_id, goal },
    ///     Duration::from_secs(5),
    /// ).await?;
    /// assert!(resp.accepted);
    /// ```
    pub async fn send_goal<GoalReq: Serialize, const CDR_REQ: usize>(
        &mut self,
        send_goal_ke: &TopicKeyExpr,
        request: &GoalReq,
        timeout: Duration,
    ) -> Result<crate::ros2::msg::action_msgs::SendGoalResponse, Error> {
        self.call_service::<
            GoalReq,
            crate::ros2::msg::action_msgs::SendGoalResponse,
            CDR_REQ,
            { crate::ros2::msg::action_msgs::SEND_GOAL_RESP_CDR_CAP },
        >(send_goal_ke, request, timeout)
        .await
    }

    /// Request the result of a completed action goal.
    ///
    /// Sends a [`GetResultRequest`](crate::ros2::msg::action_msgs::GetResultRequest)
    /// (just the goal ID) and deserializes the action-specific result.
    ///
    /// `Resp` is the user-defined GetResult response (typically
    /// `int8 status` + action Result).  `CDR_RESP` is its maximum CDR buffer
    /// size.
    ///
    /// ```rust,ignore
    /// #[derive(Deserialize)]
    /// struct NavGetResultResp { status: i8, /* result fields */ }
    ///
    /// let result: NavGetResultResp = node.get_result::<_, 64>(
    ///     &action_ke.get_result,
    ///     &goal_id,
    ///     Duration::from_secs(30),
    /// ).await?;
    /// ```
    pub async fn get_result<Resp: for<'de> Deserialize<'de>, const CDR_RESP: usize>(
        &mut self,
        get_result_ke: &TopicKeyExpr,
        goal_id: &crate::ros2::msg::action_msgs::GoalId,
        timeout: Duration,
    ) -> Result<Resp, Error> {
        let req = crate::ros2::msg::action_msgs::GetResultRequest { goal_id: *goal_id };
        self.call_service::<
            crate::ros2::msg::action_msgs::GetResultRequest,
            Resp,
            { crate::ros2::msg::action_msgs::GET_RESULT_REQ_CDR_CAP },
            CDR_RESP,
        >(get_result_ke, &req, timeout)
        .await
    }

    /// Cancel an active action goal.
    ///
    /// Returns the cancel return code (see
    /// [`cancel_goal`](crate::ros2::msg::action_msgs::cancel_goal) constants).
    ///
    /// `CancelResp` is the user-defined cancel response type.  For simple
    /// use, define a struct with just `return_code: i8`.  For full
    /// compatibility, include a `goals_canceling: heapless::Vec<GoalInfo, N>`.
    ///
    /// ```rust,ignore
    /// #[derive(Deserialize)]
    /// struct SimpleCancelResp { return_code: i8 }
    ///
    /// let resp: SimpleCancelResp = node.cancel_goal::<_, 64>(
    ///     &action_ke.cancel_goal,
    ///     &goal_id,
    ///     Stamp::ZERO,
    ///     Duration::from_secs(5),
    /// ).await?;
    /// assert_eq!(resp.return_code, cancel_goal::ERROR_NONE);
    /// ```
    pub async fn cancel_goal<CancelResp: for<'de> Deserialize<'de>, const CDR_RESP: usize>(
        &mut self,
        cancel_goal_ke: &TopicKeyExpr,
        goal_id: &crate::ros2::msg::action_msgs::GoalId,
        stamp: crate::ros2::msg::action_msgs::Stamp,
        timeout: Duration,
    ) -> Result<CancelResp, Error> {
        let req = crate::ros2::msg::action_msgs::CancelGoalRequest {
            goal_info: crate::ros2::msg::action_msgs::GoalInfo {
                goal_id: *goal_id,
                stamp,
            },
        };
        self.call_service::<
            crate::ros2::msg::action_msgs::CancelGoalRequest,
            CancelResp,
            { crate::ros2::msg::action_msgs::CANCEL_GOAL_REQ_CDR_CAP },
            CDR_RESP,
        >(cancel_goal_ke, &req, timeout)
        .await
    }

    /// Subscribe to action feedback messages.
    ///
    /// Convenience wrapper around [`subscribe_with_dispatch`](Self::subscribe_with_dispatch)
    /// using the feedback key expression from [`ActionKeyExprs`].
    ///
    /// ```rust,ignore
    /// static FB_SUB: Subscription<NavFeedback, 256, 4> = Subscription::new();
    /// let fb_handle = node.subscribe_feedback(&action_ke, &FB_SUB).await?;
    /// ```
    pub async fn subscribe_feedback<
        M: for<'de> Deserialize<'de>,
        const MSG_SIZE: usize,
        const QUEUE: usize,
    >(
        &mut self,
        action_ke: &ActionKeyExprs,
        sub: &'static Subscription<M, MSG_SIZE, QUEUE>,
    ) -> Result<SubscriptionHandle<M, MSG_SIZE, QUEUE>, Error> {
        self.subscribe_with_dispatch(action_ke.feedback, sub).await
    }

    /// Subscribe to action goal status updates.
    ///
    /// Convenience wrapper around [`subscribe_with_dispatch`](Self::subscribe_with_dispatch)
    /// using the status key expression from [`ActionKeyExprs`].
    pub async fn subscribe_status<
        M: for<'de> Deserialize<'de>,
        const MSG_SIZE: usize,
        const QUEUE: usize,
    >(
        &mut self,
        action_ke: &ActionKeyExprs,
        sub: &'static Subscription<M, MSG_SIZE, QUEUE>,
    ) -> Result<SubscriptionHandle<M, MSG_SIZE, QUEUE>, Error> {
        self.subscribe_with_dispatch(action_ke.status, sub).await
    }

    // ── Spin loop ─────────────────────────────────────────────────────────

    /// Drive the session: drain publishers, receive frames, send keepalives.
    ///
    /// **Returns only when the connection drops.** Re-create the node to
    /// reconnect.
    ///
    /// Unlike the internal `ros2::Node::spin`, this method manages its own
    /// receive buffer internally.
    pub async fn spin(&mut self) {
        let keepalive_interval = Duration::from_millis(self.lease_ms / 2);
        let mut rx_buf = [0u8; RX_BUF_SIZE];
        let mut frag_asm = FragmentAssembler::<FRAGMENT_BUF_SIZE>::new();

        loop {
            if self.drain_publishers().await.is_err() {
                return;
            }

            match with_timeout(
                keepalive_interval,
                frame::read_frame(&mut self.transport, &mut rx_buf),
            )
            .await
            {
                Ok(Ok(n)) => {
                    if self.dispatch_frame(&rx_buf[..n], &mut frag_asm).is_err() {
                        return;
                    }
                }
                Ok(Err(_)) => return,
                Err(_timeout) => {
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

    /// Spin with [`NodeCallbacks`] trait dispatch.
    ///
    /// In addition to the normal spin behavior, this also:
    /// - Fires `on_message` when a subscribed topic receives data.
    /// - Fires `on_timer` when registered timers expire.
    pub async fn spin_with_callbacks<C: NodeCallbacks>(&mut self, callbacks: &mut C) {
        let keepalive_interval = Duration::from_millis(self.lease_ms / 2);
        let mut rx_buf = [0u8; RX_BUF_SIZE];
        let mut frag_asm = FragmentAssembler::<FRAGMENT_BUF_SIZE>::new();

        loop {
            if self.drain_publishers().await.is_err() {
                return;
            }

            // Check timers
            self.fire_timers(callbacks);

            let timeout = self.next_deadline(keepalive_interval);

            match with_timeout(timeout, frame::read_frame(&mut self.transport, &mut rx_buf)).await {
                Ok(Ok(n)) => {
                    if self
                        .dispatch_frame_with_callbacks(&rx_buf[..n], &mut frag_asm, callbacks)
                        .is_err()
                    {
                        return;
                    }
                }
                Ok(Err(_)) => return,
                Err(_timeout) => {
                    self.fire_timers(callbacks);
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

    /// Spin with exponential backoff on disconnect.
    pub async fn spin_and_backoff(&mut self, policy: &mut ReconnectPolicy) {
        policy.reset();
        self.spin().await;
        policy.wait_and_advance().await;
    }

    // ── Getters ───────────────────────────────────────────────────────────

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

    /// The node's ZenohId.
    pub fn zid(&self) -> &ZenohId {
        &self.gid
    }

    // ── Internal protocol helpers ─────────────────────────────────────────

    async fn declare_key_expr(&mut self, key_expr: &str) -> Result<u16, Error> {
        let key_id = self.key_id_gen;
        self.key_id_gen = self.key_id_gen.wrapping_add(1);

        let mut buf = [0u8; 512];
        let mut pos =
            codec::encode_frame_header(&mut buf, self.sn, true).map_err(Error::Transport)?;
        self.sn = self.sn.wrapping_add(1);
        pos += codec::encode_declare_keyexpr(&mut buf[pos..], key_id, key_expr)
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
        pos += codec::encode_declare_subscriber_mapped(&mut buf[pos..], key_id as u32, key_id)
            .map_err(Error::Transport)?;
        frame::write_frame(&mut self.transport, &buf[..pos])
            .await
            .map_err(Error::Transport)?;
        Ok(())
    }

    async fn declare_liveliness(
        &mut self,
        entity_type: EntityType,
        topic: &TopicKeyExpr,
    ) -> Result<(), Error> {
        let entity_id = self.entity_id_gen;
        self.entity_id_gen += 1;

        let token = liveliness::build_liveliness_token(
            self.domain_id,
            &self.gid,
            0,
            entity_id,
            entity_type,
            self.namespace,
            self.node_name,
            topic.topic_name,
            topic.type_name,
            topic.type_hash,
            &Qos::DEFAULT,
        )
        .map_err(|_| Error::InvalidArgument)?;

        let mut buf = [0u8; 768];
        let mut pos =
            codec::encode_frame_header(&mut buf, self.sn, true).map_err(Error::Transport)?;
        self.sn = self.sn.wrapping_add(1);
        pos += codec::encode_declare_token(&mut buf[pos..], entity_id, token.as_str())
            .map_err(Error::Transport)?;
        frame::write_frame(&mut self.transport, &buf[..pos])
            .await
            .map_err(Error::Transport)?;

        ros2_debug!(
            "node '{}': declared liveliness ({}, entity_id={})",
            self.node_name,
            entity_type.as_str(),
            entity_id
        );
        Ok(())
    }

    /// Drain all queued publisher messages, batching multiple Push+Put
    /// messages into a single Zenoh Frame when they fit.
    ///
    /// Each publisher's queue is drained separately (one Frame per publisher
    /// burst), and each Frame may contain multiple messages to reduce TCP
    /// write overhead.  On write failure the most recent message is stashed
    /// for retry after reconnection.
    async fn drain_publishers(&mut self) -> Result<(), Error> {
        let mut cdr_scratch = [0u8; CDR_DRAIN_BUF];
        let mut frame_buf = [0u8; TX_FRAME_BUF];

        for i in 0..self.publishers.len() {
            let drain = self.publishers[i];

            // Compute key expression once per publisher (avoids repeated
            // heapless::String<256> formatting in the inner loop).
            let ke = drain
                .topic_ke()
                .to_key_expr()
                .map_err(|_| Error::InvalidArgument)?;

            let mut pos = 0usize;
            let mut batch_started = false;
            // Track the last drained message for stash_retry on write failure.
            let mut last_n = 0usize;
            let mut last_seq: i64 = 0;
            let mut last_ts: i64 = 0;

            while let Some((n, seq, ts)) = drain.try_drain_into(&mut cdr_scratch) {
                // Conservative estimate of the encoded Push+Put size.
                let msg_est = ke.as_str().len() + n + 64;

                // Flush current batch if adding this message would overflow.
                if batch_started && pos + msg_est > TX_FRAME_BUF {
                    if let Err(e) = frame::write_frame(&mut self.transport, &frame_buf[..pos]).await
                    {
                        drain.stash_retry(&cdr_scratch[..n], seq, ts);
                        return Err(Error::Transport(e));
                    }
                    pos = 0;
                    batch_started = false;
                }

                if !batch_started {
                    pos = codec::encode_frame_header(&mut frame_buf, self.sn, true)
                        .map_err(Error::Transport)?;
                    self.sn = self.sn.wrapping_add(1);
                    batch_started = true;
                }

                pos += codec::encode_push_put_with_attachment(
                    &mut frame_buf[pos..],
                    ke.as_str(),
                    &cdr_scratch[..n],
                    seq,
                    ts,
                    &self.gid,
                )
                .map_err(Error::Transport)?;

                last_n = n;
                last_seq = seq;
                last_ts = ts;
            }

            // Flush remaining batch for this publisher.
            if batch_started {
                if let Err(e) = frame::write_frame(&mut self.transport, &frame_buf[..pos]).await {
                    drain.stash_retry(&cdr_scratch[..last_n], last_seq, last_ts);
                    return Err(Error::Transport(e));
                }
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

    fn dispatch_frame(
        &self,
        frame_buf: &[u8],
        frag_asm: &mut FragmentAssembler<FRAGMENT_BUF_SIZE>,
    ) -> Result<(), Error> {
        use crate::transport::codec::{
            decode_fragment_header, decode_frame_header, decode_push_put, parse_transport_msg_kind,
            TransportMsgKind,
        };

        if frame_buf.is_empty() {
            return Ok(());
        }

        match parse_transport_msg_kind(frame_buf[0]) {
            TransportMsgKind::Frame => {
                if let Ok((_, _, body_pos)) = decode_frame_header(frame_buf) {
                    if let Ok(Some((put, _))) = decode_push_put(&frame_buf[body_pos..]) {
                        Self::route_to_subscribers(&self.subscribers, &put);
                    }
                }
                Ok(())
            }
            TransportMsgKind::Fragment => {
                if let Ok((_, _, more, body_pos)) = decode_fragment_header(frame_buf) {
                    match frag_asm.feed(more, &frame_buf[body_pos..]) {
                        Ok(Some(assembled)) => {
                            if let Ok(Some((put, _))) = decode_push_put(assembled) {
                                Self::route_to_subscribers(&self.subscribers, &put);
                            }
                        }
                        Ok(None) => {} // intermediate fragment
                        Err(_) => {
                            ros2_warn!("fragment reassembly failed (too large)");
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

    fn route_to_subscribers(
        subscribers: &[(u16, &'static dyn SubscriptionDispatch)],
        put: &codec::IncomingPut<'_>,
    ) {
        let scope_id = put.scope as u16;
        for (key_id, dispatch) in subscribers {
            if scope_id == *key_id {
                dispatch.push_raw(put.payload);
                break;
            }
        }
    }

    fn dispatch_frame_with_callbacks<C: NodeCallbacks>(
        &self,
        frame_buf: &[u8],
        frag_asm: &mut FragmentAssembler<FRAGMENT_BUF_SIZE>,
        callbacks: &mut C,
    ) -> Result<(), Error> {
        use crate::transport::codec::{
            decode_fragment_header, decode_frame_header, decode_push_put, parse_transport_msg_kind,
            TransportMsgKind,
        };

        if frame_buf.is_empty() {
            return Ok(());
        }

        match parse_transport_msg_kind(frame_buf[0]) {
            TransportMsgKind::Frame => {
                if let Ok((_, _, body_pos)) = decode_frame_header(frame_buf) {
                    if let Ok(Some((put, _))) = decode_push_put(&frame_buf[body_pos..]) {
                        Self::route_to_subscribers(&self.subscribers, &put);
                        callbacks.on_message(put.key_suffix, put.payload);
                    }
                }
                Ok(())
            }
            TransportMsgKind::Fragment => {
                if let Ok((_, _, more, body_pos)) = decode_fragment_header(frame_buf) {
                    match frag_asm.feed(more, &frame_buf[body_pos..]) {
                        Ok(Some(assembled)) => {
                            if let Ok(Some((put, _))) = decode_push_put(assembled) {
                                Self::route_to_subscribers(&self.subscribers, &put);
                                callbacks.on_message(put.key_suffix, put.payload);
                            }
                        }
                        Ok(None) => {}
                        Err(_) => {
                            ros2_warn!("fragment reassembly failed (too large)");
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

    fn fire_timers<C: NodeCallbacks>(&mut self, callbacks: &mut C) {
        let now = Instant::now();
        for timer in self.timers.iter_mut() {
            if now >= timer.next_fire {
                callbacks.on_timer();
                timer.next_fire = now + timer.period;
            }
        }
    }

    fn next_deadline(&self, keepalive_interval: Duration) -> Duration {
        let now = Instant::now();
        let mut min = keepalive_interval;
        for timer in &self.timers {
            if timer.next_fire > now {
                let remaining = timer.next_fire - now;
                if remaining < min {
                    min = remaining;
                }
            } else {
                // Already overdue — fire immediately
                return Duration::from_millis(0);
            }
        }
        min
    }
}
