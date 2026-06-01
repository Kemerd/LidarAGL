#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wav_to_pcm.py -- convert a WAV file (any rate/width/channels) to the headerless
raw 16 kHz mono signed-16-bit-LE PCM format the LidarAGL firmware embeds.

Use this if you record/produce your own callout clips instead of using the
SAPI5 TTS generator (gen_clips.py).

Usage:
    python tools/wav_to_pcm.py fifty.wav
    python tools/wav_to_pcm.py fifty.wav -o assets/clips/fifty.pcm
    python tools/wav_to_pcm.py *.wav            # batch (shell-expanded)

Windows-console-safe: forces UTF-8 stdout.
"""

import argparse
import os
import struct
import sys
import wave

try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

SAMPLE_RATE = 16000          # MUST match config.h SAMPLE_RATE
AMPLITUDE   = 22000          # peak s16 (~ -3.5 dBFS)

HERE     = os.path.dirname(os.path.abspath(__file__))
ROOT     = os.path.dirname(HERE)
CLIP_DIR = os.path.join(ROOT, "assets", "clips")


def wav_to_samples(wav_path):
    """Decode a WAV to s16 mono @ 16 kHz, returning a list of int samples."""
    with wave.open(wav_path, "rb") as w:
        nch     = w.getnchannels()
        width   = w.getsampwidth()
        rate    = w.getframerate()
        nframes = w.getnframes()
        raw     = w.readframes(nframes)

    if width == 2:
        ints = list(struct.unpack("<%dh" % (len(raw) // 2), raw))
        scale = 32768.0
    elif width == 1:
        ints = [b - 128 for b in raw]          # 8-bit WAV is unsigned
        scale = 128.0
    else:
        raise ValueError(f"unsupported WAV sample width: {width} bytes")

    # Downmix to mono.
    if nch > 1:
        mono = [sum(ints[i:i + nch]) / nch for i in range(0, len(ints), nch)]
    else:
        mono = ints
    mono = [x / scale for x in mono]

    # Linear resample to 16 kHz.
    if rate != SAMPLE_RATE and len(mono) > 1:
        ratio = SAMPLE_RATE / rate
        out_n = int(len(mono) * ratio)
        out = []
        for i in range(out_n):
            src = i / ratio
            i0 = int(src)
            i1 = min(i0 + 1, len(mono) - 1)
            frac = src - i0
            out.append(mono[i0] * (1 - frac) + mono[i1] * frac)
        mono = out

    # Normalise to the target peak so trim is consistent across recordings.
    peak = max((abs(x) for x in mono), default=0.0)
    g = (1.0 / peak) if peak > 0 else 0.0
    return [int(x * g * AMPLITUDE) for x in mono]


def write_pcm(samples, out_path):
    data = bytearray()
    for s in samples:
        if s >  32767: s =  32767
        if s < -32768: s = -32768
        data += struct.pack("<h", int(s))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(data)
    print(f"  wrote {out_path}  ({len(samples)} samples, "
          f"{len(samples)/SAMPLE_RATE:.2f}s)")


def main():
    ap = argparse.ArgumentParser(description="WAV -> raw 16k mono s16le PCM.")
    ap.add_argument("inputs", nargs="+", help="input WAV file(s)")
    ap.add_argument("-o", "--output", default=None,
                    help="output .pcm path (single input only; "
                         "default: assets/clips/<stem>.pcm)")
    args = ap.parse_args()

    if args.output and len(args.inputs) > 1:
        print("ERROR: -o/--output only valid with a single input.", file=sys.stderr)
        return 1

    for wav_path in args.inputs:
        if not os.path.exists(wav_path):
            print(f"  [!] missing: {wav_path}", file=sys.stderr)
            continue
        samples = wav_to_samples(wav_path)
        if args.output:
            out_path = args.output
        else:
            stem = os.path.splitext(os.path.basename(wav_path))[0]
            out_path = os.path.join(CLIP_DIR, stem + ".pcm")
        write_pcm(samples, out_path)

    return 0


if __name__ == "__main__":
    sys.exit(main())
