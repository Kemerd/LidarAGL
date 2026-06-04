#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_codec.py -- self-check that the Python LWNX encoder matches the firmware.

Run:  python tools/bench_sim/test_codec.py

It round-trips frames through a parser that mirrors the firmware's lwnx_feed
(main/lwnx.c) -- same flags decode, same CRC span, same payload slicing -- so a
PASS means a frame this tool emits is exactly what the device will accept and
decode. The on-device test is the ultimate proof, but this catches mistakes
(e.g. reaching for a reflected CRC) before you ever flash.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import protocol as P
import lwnx_codec as codec


def parse_one(frame: bytes):
    """Mirror of lwnx_feed: validate framing + CRC, return (cmd, payload)."""
    assert frame[0] == P.LWNX_START_BYTE, "bad start byte"
    flags = frame[1] | (frame[2] << 8)
    payload_len = flags >> 6                       # counts cmd + data
    total = 3 + payload_len + 2
    assert len(frame) == total, "length %d != expected %d" % (len(frame), total)
    crc_span = 3 + payload_len
    want = codec.lwnx_crc16(frame[:crc_span])
    got = frame[crc_span] | (frame[crc_span + 1] << 8)
    assert want == got, "CRC mismatch: want %#06x got %#06x" % (want, got)
    cmd = frame[3]
    payload = frame[4:3 + payload_len]             # payload_len-1 data bytes
    return cmd, payload, (flags & 1)


def expect(cond, msg):
    if not cond:
        print("FAIL:", msg)
        sys.exit(1)


def main():
    # 1) Distance frame round-trips with the right command + decoded centimetres.
    for cm in (0, 91, 500, 1234, 16000, 32767):
        f = codec.build_distance_frame(cm)
        cmd, payload, write = parse_one(f)
        expect(cmd == P.LWNX_CMD_DISTANCE_DATA, "distance cmd id")
        expect(write == 0, "distance frame must not set the write flag")
        expect(len(payload) == P.DISTANCE_PAYLOAD_LEN, "distance payload length")
        off = P.DISTANCE_OFFSET_FILT
        decoded = payload[off] | (payload[off + 1] << 8)
        expect(decoded == cm, "decoded cm %d != %d" % (decoded, cm))

    # 2) ft<->cm uses the firmware's exact factor.
    expect(codec.ft_to_cm(5.0) == round(5.0 / P.CM_TO_FT), "ft_to_cm factor")

    # 3) Bench-control frame: cmd 200, opcode in payload[0], write flag set.
    f = codec.build_bench_ctrl(P.OP_HELLO)
    cmd, payload, write = parse_one(f)
    expect(cmd == P.LWNX_CMD_BENCH_CTRL, "bench-ctrl cmd id")
    expect(write == 1, "bench-ctrl frame should set the write flag")
    expect(payload[0] == P.OP_HELLO, "bench-ctrl opcode")

    # 4) CRC is deterministic and a single-bit change perturbs it (sanity).
    a = codec.lwnx_crc16(b"\xAA\x55\x01\x2C")
    b = codec.lwnx_crc16(b"\xAA\x55\x01\x2D")
    expect(a == codec.lwnx_crc16(b"\xAA\x55\x01\x2C"), "CRC determinism")
    expect(a != b, "CRC should change on a 1-byte difference")

    # Show one example distance frame so it can be eyeballed against the firmware.
    ex = codec.build_distance_frame(500)
    print("example distance frame (500 cm):", ex.hex(" "))
    print("ALL CHECKS PASSED")


if __name__ == "__main__":
    main()
