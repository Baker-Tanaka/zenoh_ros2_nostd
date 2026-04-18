//! Zenoh client-mode handshake: InitSyn → InitAck → OpenSyn → OpenAck.

use embedded_io_async::{Read, Write};

use super::codec;
use super::frame;
use super::protocol::*;
use crate::error::TransportError;

/// Default lease time offered by the client (10 seconds).
const DEFAULT_LEASE_MS: u64 = 10_000;

/// Result of a successful handshake.
#[derive(Debug)]
pub struct HandshakeResult {
    /// Router's zenoh ID.
    pub router_zid: ZenohId,
    /// Lease time the router requires us to respect (router's announced lease).
    pub lease_ms: u64,
    /// Lease time we proposed to the router (our keepalive obligation).
    pub our_lease_ms: u64,
    /// Initial sequence number for this end.
    pub initial_sn: u64,
    /// Router's initial sequence number.
    pub router_initial_sn: u64,
}

/// Perform the client-mode handshake over a transport link.
///
/// 1. Send InitSyn
/// 2. Receive InitAck (get cookie)
/// 3. Send OpenSyn (echo cookie)
/// 4. Receive OpenAck (session open)
pub async fn client_handshake<T: Read + Write>(
    link: &mut T,
    our_zid: &ZenohId,
    tx_buf: &mut [u8],
    rx_buf: &mut [u8],
) -> Result<HandshakeResult, TransportError> {
    // 1. InitSyn
    let init_syn = InitSyn {
        version: PROTO_VERSION,
        whatami: WhatAmI::Client,
        zid: *our_zid,
        batch_size: None,
    };

    let n = codec::encode_init_syn(tx_buf, &init_syn)?;
    frame::write_frame(link, &tx_buf[..n]).await?;

    ros2_debug!("handshake: InitSyn sent");

    // 2. InitAck
    let n = frame::read_frame(link, rx_buf).await?;
    let (init_ack, _) = codec::decode_init_ack(&rx_buf[..n])?;

    if init_ack.version != PROTO_VERSION {
        return Err(TransportError::VersionMismatch);
    }

    ros2_debug!("handshake: InitAck received, router zid={:?}", init_ack.zid);

    // 3. OpenSyn
    let open_syn = OpenSyn {
        lease_ms: DEFAULT_LEASE_MS,
        initial_sn: 0,
        cookie: init_ack.cookie,
    };

    let n = codec::encode_open_syn(tx_buf, &open_syn)?;
    frame::write_frame(link, &tx_buf[..n]).await?;

    ros2_debug!("handshake: OpenSyn sent");

    // 4. OpenAck
    let n = frame::read_frame(link, rx_buf).await?;
    let (open_ack, _) = codec::decode_open_ack(&rx_buf[..n])?;

    ros2_debug!("handshake: OpenAck received, lease={}ms", open_ack.lease_ms);

    Ok(HandshakeResult {
        router_zid: init_ack.zid,
        lease_ms: open_ack.lease_ms,
        our_lease_ms: DEFAULT_LEASE_MS,
        initial_sn: 0,
        router_initial_sn: open_ack.initial_sn,
    })
}
