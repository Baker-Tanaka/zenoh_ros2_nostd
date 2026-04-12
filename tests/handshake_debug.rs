//! Minimal handshake debug test — run with zenohd already listening.
//!
//! cargo test --no-default-features --test handshake_debug -- --ignored --nocapture

#![cfg(not(target_arch = "wasm32"))]

use embedded_io_adapters::tokio_1::FromTokio;
use std::time::Duration;
use tokio::net::TcpStream;
use zenoh_ros2_nostd::transport::{codec, frame, protocol::*};

#[tokio::test]
#[ignore]
async fn debug_handshake_step_by_step() {
    // Connect
    let tcp = TcpStream::connect("127.0.0.1:7447")
        .await
        .expect("Failed to connect to zenohd at 127.0.0.1:7447");
    eprintln!("[OK] TCP connected");

    let mut link = FromTokio::new(tcp);

    let our_zid = ZenohId::from_bytes(&[0xDE, 0xAD, 0xBE, 0xEF]);
    let mut tx_buf = [0u8; 512];
    let mut rx_buf = [0u8; 8192];

    // Step 1: Encode InitSyn
    let init_syn = InitSyn {
        version: PROTO_VERSION,
        whatami: WhatAmI::Client,
        zid: our_zid,
        batch_size: None,
    };
    let n = codec::encode_init_syn(&mut tx_buf, &init_syn).expect("encode InitSyn");
    eprintln!(
        "[OK] InitSyn encoded: {} bytes, hex={}",
        n,
        hex(&tx_buf[..n])
    );

    // Step 2: Write frame
    frame::write_frame(&mut link, &tx_buf[..n])
        .await
        .expect("write InitSyn frame");
    eprintln!("[OK] InitSyn frame sent (with 2-byte length prefix)");

    // Step 3: Read InitAck frame
    eprintln!("[..] Waiting for InitAck...");
    let read_result = tokio::time::timeout(
        Duration::from_secs(5),
        frame::read_frame(&mut link, &mut rx_buf),
    )
    .await;

    match read_result {
        Ok(Ok(n)) => {
            eprintln!("[OK] InitAck frame received: {} bytes", n);
            eprintln!("     hex: {}", hex(&rx_buf[..n.min(64)]));

            // Try to decode
            match codec::decode_init_ack(&rx_buf[..n]) {
                Ok((ack, consumed)) => {
                    eprintln!(
                        "[OK] InitAck decoded: version={}, whatami={:?}, zid={:?}",
                        ack.version, ack.whatami, ack.zid
                    );
                    eprintln!(
                        "     cookie len={}, batch_size={:?}",
                        ack.cookie.len(),
                        ack.batch_size
                    );
                    eprintln!("     consumed {} of {} bytes", consumed, n);

                    // Step 4: Send OpenSyn
                    let open_syn = OpenSyn {
                        lease_ms: 10_000,
                        initial_sn: 0,
                        cookie: ack.cookie,
                    };
                    let n2 =
                        codec::encode_open_syn(&mut tx_buf, &open_syn).expect("encode OpenSyn");
                    eprintln!("[OK] OpenSyn encoded: {} bytes", n2);

                    frame::write_frame(&mut link, &tx_buf[..n2])
                        .await
                        .expect("write OpenSyn frame");
                    eprintln!("[OK] OpenSyn frame sent");

                    // Step 5: Read OpenAck
                    eprintln!("[..] Waiting for OpenAck...");
                    let ack_result = tokio::time::timeout(
                        Duration::from_secs(5),
                        frame::read_frame(&mut link, &mut rx_buf),
                    )
                    .await;

                    match ack_result {
                        Ok(Ok(n3)) => {
                            eprintln!("[OK] OpenAck frame received: {} bytes", n3);
                            eprintln!("     hex: {}", hex(&rx_buf[..n3.min(64)]));

                            match codec::decode_open_ack(&rx_buf[..n3]) {
                                Ok((open_ack, _)) => {
                                    eprintln!(
                                        "[OK] OpenAck decoded: lease={}ms, initial_sn={}",
                                        open_ack.lease_ms, open_ack.initial_sn
                                    );
                                    eprintln!("[SUCCESS] Handshake complete!");
                                }
                                Err(e) => {
                                    eprintln!("[FAIL] OpenAck decode error: {:?}", e);
                                    eprintln!("  raw bytes: {}", hex(&rx_buf[..n3]));
                                }
                            }
                        }
                        Ok(Err(e)) => eprintln!("[FAIL] OpenAck read error: {:?}", e),
                        Err(_) => eprintln!("[FAIL] OpenAck read timeout"),
                    }
                }
                Err(e) => {
                    eprintln!("[FAIL] InitAck decode error: {:?}", e);
                    eprintln!("  first 32 bytes: {}", hex(&rx_buf[..n.min(32)]));
                }
            }
        }
        Ok(Err(e)) => eprintln!("[FAIL] InitAck read error: {:?}", e),
        Err(_) => eprintln!("[FAIL] InitAck read timeout (5s)"),
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{:02x}", b))
        .collect::<Vec<_>>()
        .join(" ")
}
