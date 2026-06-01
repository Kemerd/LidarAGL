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
import math
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

# --- Silence trimming ---------------------------------------------------------
# Recorded/TTS clips have inconsistent leading/trailing dead air, which would
# make the callouts fire with ragged timing. We detect the speech region by
# amplitude (relative to the clip's own peak), trim to it with a little padding
# so consonants aren't clipped, and apply short raised-cosine fades so the cut
# edges don't click.
TRIM_THRESH   = 0.02         # speech starts where |sample| exceeds this * peak
TRIM_PAD_MS   = 30           # keep this much audio either side of the speech
TRIM_FADE_MS  = 8            # raised-cosine fade in/out at the trimmed edges

HERE     = os.path.dirname(os.path.abspath(__file__))
ROOT     = os.path.dirname(HERE)
CLIP_DIR = os.path.join(ROOT, "assets", "clips")


def trim_silence(mono):
    """Trim leading/trailing silence from a float mono signal, with pad + fades.

    `mono` is a list of floats (any scale). Returns the trimmed list. The
    threshold is relative to the clip's own peak so it adapts to recording level.
    """
    n = len(mono)
    if n == 0:
        return mono

    peak = max((abs(x) for x in mono), default=0.0)
    if peak <= 0.0:
        return mono                      # pure silence -> leave as-is

    thresh = TRIM_THRESH * peak

    # First and last samples that exceed the speech threshold.
    first = 0
    while first < n and abs(mono[first]) < thresh:
        first += 1
    if first == n:
        return mono                      # never crossed threshold -> all quiet

    last = n - 1
    while last > first and abs(mono[last]) < thresh:
        last -= 1

    # Pad either side so we don't bite into soft consonants.
    pad = int(TRIM_PAD_MS / 1000.0 * SAMPLE_RATE)
    start = max(0, first - pad)
    end   = min(n, last + pad + 1)
    clip  = mono[start:end]

    # Raised-cosine fades at the new edges (click-free).
    fade = int(TRIM_FADE_MS / 1000.0 * SAMPLE_RATE)
    fade = min(fade, len(clip) // 2)
    for i in range(fade):
        env = 0.5 - 0.5 * math.cos(math.pi * i / fade)
        clip[i]            *= env
        clip[-1 - i]       *= env

    return clip


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

    # Cut leading/trailing silence so every callout starts tight and on time.
    mono = trim_silence(mono)

    # Normalise to the target peak so loudness is consistent across recordings.
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
