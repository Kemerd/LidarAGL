#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lwnx_codec.py -- byte-exact Python encoder for the LightWare LWNX protocol.

This is a faithful port of the firmware's framing so the bytes we put on the
wire are INDISTINGUISHABLE from the real SF30/C sensor's output. The firmware
validates every frame with its own CRC, so any mismatch here would simply be
dropped on the device -- the whole point of the bench is that it is NOT dropped.

Mirrors (must stay in lock-step):
    lwnx_crc16() ... main/lwnx.c:21-36   (MSB-first poly-0x1021, init 0)
    lwnx_build() ... main/lwnx.c:38-68   (frame layout + flags + CRC span)

Frame on the wire (little-endian flags & CRC):
    [0]        0xAA                      start byte
    [1..2]     flags  = (payloadLen<<6) | write     (u16 LE)
    [3]        command id                (counted inside payloadLen)
    [4..]      payload data              (payloadLen - 1 bytes)
    [last 2]   CRC-16                    (u16 LE) over bytes [0 .. last payload]

where payloadLen counts the command-id byte plus the data bytes.
"""

import protocol as P


def lwnx_crc16(data: bytes) -> int:
    """CRC-16 -- exact port of LightWare's lwnxCreateCrc (main/lwnx.c:21-36).

    Init 0, processed MSB-first. This is the *unrolled* poly-0x1021 (CCITT)
    update used by the sensor. It is NOT the reflected (LSB-first) variant, so
    Python's binascii.crc_hqx / any "standard" CRC-CCITT will NOT match -- this
    hand-rolled loop is the only thing that reproduces the sensor's checksum.
    Every intermediate is masked to 16 bits to mimic the C uint16_t wrap.
    """
    crc = 0
    for b in data:
        code = (crc >> 8) & 0xFFFF       # high byte of the running CRC
        code ^= b
        code ^= (code >> 4)
        crc = (crc << 8) & 0xFFFF        # shift the CRC up a byte (16-bit wrap)
        crc ^= code
        code = (code << 5) & 0xFFFF
        crc ^= code
        code = (code << 7) & 0xFFFF
        crc ^= code
        crc &= 0xFFFF
    return crc & 0xFFFF


def lwnx_build(cmd: int, payload: bytes = b"", write: bool = False) -> bytes:
    """Build a complete LWNX frame -- exact port of lwnx_build (main/lwnx.c:38-68).

    `payload` is the data AFTER the command id (NOT counting it). Returns the
    full framed bytes ready to write to the serial port.
    """
    # payloadLen counts the command-id byte plus the data bytes (lwnx.c:42).
    payload_len = len(payload) + 1

    # flags = (payloadLen << 6) | write, stored little-endian (lwnx.c:51).
    flags = ((payload_len << 6) | (1 if write else 0)) & 0xFFFF

    frame = bytearray()
    frame.append(P.LWNX_START_BYTE)            # [0]
    frame.append(flags & 0xFF)                 # [1] flags low
    frame.append((flags >> 8) & 0xFF)          # [2] flags high
    frame.append(cmd & 0xFF)                    # [3] command id
    frame += payload                           # [4..] data

    # CRC covers everything up to and including the last payload byte (lwnx.c:62).
    crc = lwnx_crc16(bytes(frame))
    frame.append(crc & 0xFF)                    # CRC low
    frame.append((crc >> 8) & 0xFF)             # CRC high
    return bytes(frame)


# --- High-level builders the bench actually uses -----------------------------

def ft_to_cm(ft: float) -> int:
    """Feet -> centimetres, the exact inverse of the firmware's CM_TO_FT factor."""
    if ft < 0.0:
        ft = 0.0
    return int(round(ft / P.CM_TO_FT))


def build_distance_frame(dist_cm: int) -> bytes:
    """A realistic SF30/C distance frame (cmd 44) carrying `dist_cm`.

    The firmware only reads the int16 LE centimetre value at offset 6
    (firstReturnFiltered); the rest of the ~20-byte payload is left zero, which
    is harmless. `dist_cm` is clamped into the int16 range the sensor uses.
    """
    cm = int(dist_cm)
    if cm < 0:
        cm = 0
    if cm > 32767:
        cm = 32767                              # int16 ceiling (well past max range)

    payload = bytearray(P.DISTANCE_PAYLOAD_LEN)
    off = P.DISTANCE_OFFSET_FILT
    payload[off]     = cm & 0xFF                 # int16 LE low
    payload[off + 1] = (cm >> 8) & 0xFF          # int16 LE high
    return lwnx_build(P.LWNX_CMD_DISTANCE_DATA, bytes(payload), write=False)


def build_lost_signal_frame() -> bytes:
    """A distance frame carrying the sensor's lost-signal sentinel (16000 cm)."""
    return build_distance_frame(P.SF30_LOST_SIGNAL_CM)


def build_bench_ctrl(opcode: int, arg: bytes = b"") -> bytes:
    """A host->device bench-control frame (cmd 200) carrying an opcode + args."""
    return lwnx_build(P.LWNX_CMD_BENCH_CTRL, bytes([opcode & 0xFF]) + arg, write=True)
