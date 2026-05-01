#!/usr/bin/env python3
"""Subscribe to /chatter via Zenoh and print received CDR String messages.

Usage (dev container):
    python3 verify_sub.py                   # default: zenoh-router:7447
    python3 verify_sub.py localhost:7447    # custom address
    ZENOH_ROUTER_ADDR=host:7447 python3 verify_sub.py

Run this first, then flash the Baker link. Dev via `cargo run --release`
on the host.  Messages should appear within ~10 s (DHCP + handshake).
"""

import os
import struct
import sys
import time

import zenoh

CHATTER_KEY = (
    "0/chatter/std_msgs::msg::dds_::String_/"
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
)

LISTEN_SECONDS = 120  # longer than WASI example — device needs DHCP + handshake


def decode_cdr_string(data: bytes) -> str:
    """Decode a CDR LE std_msgs/String from raw bytes."""
    if len(data) < 9 or data[0:4] != b"\x00\x01\x00\x00":
        return f"(raw {len(data)} bytes)"
    str_len = struct.unpack("<I", data[4:8])[0]
    return data[8 : 8 + str_len - 1].decode("utf-8", errors="replace")


def main():
    addr = (
        sys.argv[1]
        if len(sys.argv) > 1
        else os.environ.get("ZENOH_ROUTER_ADDR", "zenoh-router:7447")
    )

    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["tcp/{addr}"]')

    session = zenoh.open(conf)
    print(f"[PY-SUB] Connected to {addr} (ZID: {session.zid()})")

    count = 0

    def on_sample(sample):
        nonlocal count
        count += 1
        text = decode_cdr_string(sample.payload.to_bytes())
        print(f"  [{count:>3}] /chatter: \"{text}\"")

    sub = session.declare_subscriber(CHATTER_KEY, on_sample)
    print(f"[PY-SUB] Subscribed to /chatter — waiting for messages…")
    print(f"[PY-SUB] (flash the device with `cargo run --release` on the host)")
    print(f"[PY-SUB] Ctrl+C to stop  |  auto-exit after {LISTEN_SECONDS}s\n")

    try:
        time.sleep(LISTEN_SECONDS)
    except KeyboardInterrupt:
        print()

    sub.undeclare()
    session.close()

    if count > 0:
        print(f"\n✅ Received {count} message(s) from /chatter")
    else:
        print(f"\n⚠️  No messages received.")
        print(f"    - Is the device powered on and connected to the network?")
        print(f"    - Does config.json have the correct router_addr?")
        print(f"    - Is the Zenoh router running? (docker compose up -d)")


if __name__ == "__main__":
    main()
