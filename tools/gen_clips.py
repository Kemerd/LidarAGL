#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_clips.py -- generate the LidarAGL voice callout clips + calibration chirp.

Produces headerless raw 16 kHz mono signed-16-bit-LE PCM files in
assets/clips/, which the firmware embeds via EMBED_FILES.

Real speech is synthesized with an OFFLINE TTS engine (pyttsx3 -> Windows SAPI5
by default). This script will NOT silently fabricate fake speech: if no TTS
engine is available it refuses, unless you explicitly pass --placeholder-tones,
which writes clearly-labelled distinct BEEP patterns (NOT speech) for bench
testing only, with a loud warning.

Usage (run it yourself; the firmware build just embeds whatever .pcm exist):
    python tools/gen_clips.py                 # real TTS speech
    python tools/gen_clips.py --voice 1       # pick a different SAPI voice
    python tools/gen_clips.py --placeholder-tones   # beeps, NOT speech
    python tools/gen_clips.py --list-voices

Windows-console-safe: forces UTF-8 stdout so it runs cleanly in cmd/PowerShell.
"""

import argparse
import math
import os
import struct
import sys
import wave

# --- Windows console / UTF-8 safety -----------------------------------------
# Reconfigure stdout/stderr to UTF-8 so emoji/box-drawing and any non-ASCII voice
# names don't blow up on the legacy Windows code page.
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

SAMPLE_RATE = 16000          # MUST match config.h SAMPLE_RATE
AMPLITUDE   = 22000          # peak s16 amplitude (~ -3.5 dBFS), leaves headroom

# The phrases, keyed by the output filename stem the firmware expects.
CLIPS = {
    "ten":           "ten",
    "twenty":        "twenty",
    "thirty":        "thirty",
    "forty":         "forty",
    "fifty":         "fifty",
    "one_hundred":   "one hundred",
    "two_hundred":   "two hundred",
    "three_hundred": "three hundred",
    "four_hundred":  "four hundred",
    "five_hundred":  "five hundred",
    "six_hundred":   "six hundred",
}

# Repo paths (this file lives in tools/).
HERE     = os.path.dirname(os.path.abspath(__file__))
ROOT     = os.path.dirname(HERE)
CLIP_DIR = os.path.join(ROOT, "assets", "clips")


def ensure_clip_dir():
    """Create assets/clips/ if it doesn't exist yet."""
    os.makedirs(CLIP_DIR, exist_ok=True)


def write_pcm(stem, samples):
    """Write a list of signed-16-bit samples as headerless raw little-endian PCM."""
    path = os.path.join(CLIP_DIR, stem + ".pcm")
    # Clamp + pack to s16le.
    data = bytearray()
    for s in samples:
        if s >  32767: s =  32767
        if s < -32768: s = -32768
        data += struct.pack("<h", int(s))
    with open(path, "wb") as f:
        f.write(data)
    print(f"  wrote {stem}.pcm  ({len(samples)} samples, "
          f"{len(samples)/SAMPLE_RATE:.2f}s)")


def wav_bytes_to_samples_16k_mono(wav_path):
    """Read a WAV (any rate/width/channels), return s16 mono @ 16 kHz samples.

    Simple, dependency-free resampler: decode to mono float, then linear-resample
    to SAMPLE_RATE. Good enough for short speech tokens.
    """
    with wave.open(wav_path, "rb") as w:
        nch    = w.getnchannels()
        width  = w.getsampwidth()
        rate   = w.getframerate()
        nframes = w.getnframes()
        raw = w.readframes(nframes)

    # Decode to float mono in [-1, 1].
    if width == 2:
        fmt = "<%dh" % (len(raw) // 2)
        ints = struct.unpack(fmt, raw)
        scale = 32768.0
    elif width == 1:
        # 8-bit WAV is unsigned.
        ints = [b - 128 for b in raw]
        scale = 128.0
    else:
        raise ValueError(f"unsupported WAV sample width: {width} bytes")

    if nch > 1:
        mono = [sum(ints[i:i + nch]) / nch for i in range(0, len(ints), nch)]
    else:
        mono = list(ints)
    mono = [x / scale for x in mono]

    # Linear resample to SAMPLE_RATE.
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

    return [int(x * AMPLITUDE) for x in mono]


# --- Real TTS path (pyttsx3 -> SAPI5 on Windows) ----------------------------

def tts_engine(voice_index):
    """Create a pyttsx3 engine, or return None if unavailable."""
    try:
        import pyttsx3
    except ImportError:
        return None
    eng = pyttsx3.init()
    eng.setProperty("rate", 150)       # a touch slower for clarity in the cockpit
    voices = eng.getProperty("voices")
    if voice_index is not None and 0 <= voice_index < len(voices):
        eng.setProperty("voice", voices[voice_index].id)
    return eng


def list_voices():
    eng = tts_engine(None)
    if eng is None:
        print("pyttsx3 not installed; cannot list voices.")
        return
    for i, v in enumerate(eng.getProperty("voices")):
        print(f"  [{i}] {v.name}  ({v.id})")


def gen_tts(voice_index):
    """Synthesize each phrase to a temp WAV, then convert to raw 16k mono PCM."""
    eng = tts_engine(voice_index)
    if eng is None:
        return False

    import tempfile
    print("Using pyttsx3/SAPI5 TTS:")
    name = None
    try:
        name = eng.getProperty("voice")
    except Exception:
        pass
    print(f"  voice id: {name}")

    for stem, phrase in CLIPS.items():
        with tempfile.TemporaryDirectory() as td:
            wav_path = os.path.join(td, stem + ".wav")
            eng.save_to_file(phrase, wav_path)
            eng.runAndWait()
            if not os.path.exists(wav_path):
                print(f"  [!] TTS produced no file for '{phrase}'", file=sys.stderr)
                return False
            samples = wav_bytes_to_samples_16k_mono(wav_path)
        write_pcm(stem, samples)

    # The calibration-error chirp is always synthesized (it's a tone, not speech).
    write_chirp()
    return True


# --- Chirp + placeholder tones ----------------------------------------------

def sine(freq, dur_s, gain=1.0):
    """Generate a gain-shaped sine with short raised-cosine fades (click-free)."""
    n = int(dur_s * SAMPLE_RATE)
    fade = int(0.01 * SAMPLE_RATE)     # 10 ms fades
    out = []
    for i in range(n):
        env = 1.0
        if i < fade:
            env = 0.5 - 0.5 * math.cos(math.pi * i / fade)
        elif i > n - fade:
            env = 0.5 - 0.5 * math.cos(math.pi * (n - i) / fade)
        out.append(AMPLITUDE * gain * env * math.sin(2 * math.pi * freq * i / SAMPLE_RATE))
    return out


def write_chirp():
    """Calibration-error chirp: a distinctive falling two-tone 'uh-oh' pattern."""
    samples = []
    samples += sine(880, 0.18, 0.8)    # high
    samples += [0] * int(0.04 * SAMPLE_RATE)
    samples += sine(440, 0.28, 0.8)    # low (falling = 'problem')
    write_pcm("calibration_error", samples)   # matches the firmware clip symbol


def gen_placeholders():
    """Distinct beep patterns per number -- NOT speech. Bench testing only."""
    print("=" * 70)
    print("WARNING: writing PLACEHOLDER BEEPS, *not* spoken numbers.")
    print("         These are for bench testing only. Generate real speech with")
    print("         'python tools/gen_clips.py' (requires: pip install pyttsx3).")
    print("=" * 70)

    # Map each clip to a count of short beeps at a base pitch, so they are
    # audibly distinguishable on the bench (e.g. 'fifty' = 5 beeps).
    pattern = {
        "ten":           (1, 700),
        "twenty":        (2, 700),
        "thirty":        (3, 700),
        "forty":         (4, 700),
        "fifty":         (5, 700),
        "one_hundred":   (1, 1100),
        "two_hundred":   (2, 1100),
        "three_hundred": (3, 1100),
        "four_hundred":  (4, 1100),
        "five_hundred":  (5, 1100),
        "six_hundred":   (6, 1100),
    }
    for stem, (count, freq) in pattern.items():
        samples = []
        for _ in range(count):
            samples += sine(freq, 0.08, 0.7)
            samples += [0] * int(0.05 * SAMPLE_RATE)
        write_pcm(stem, samples)

    write_chirp()


def main():
    ap = argparse.ArgumentParser(description="Generate LidarAGL voice clips.")
    ap.add_argument("--voice", type=int, default=None,
                    help="SAPI voice index (see --list-voices)")
    ap.add_argument("--list-voices", action="store_true",
                    help="list available TTS voices and exit")
    ap.add_argument("--placeholder-tones", action="store_true",
                    help="write distinct BEEPS instead of speech (bench only)")
    args = ap.parse_args()

    if args.list_voices:
        list_voices()
        return 0

    ensure_clip_dir()
    print(f"Output directory: {CLIP_DIR}")

    if args.placeholder_tones:
        gen_placeholders()
        return 0

    if gen_tts(args.voice):
        print("Done -- real speech clips generated.")
        return 0

    # No TTS engine: refuse rather than ship fake speech.
    print("=" * 70, file=sys.stderr)
    print("ERROR: no offline TTS engine found (pyttsx3 not installed).",
          file=sys.stderr)
    print("Install it:   pip install pyttsx3", file=sys.stderr)
    print("On Windows it drives the built-in SAPI5 voices (no internet needed).",
          file=sys.stderr)
    print("Or, for bench BEEPS (not speech):  "
          "python tools/gen_clips.py --placeholder-tones", file=sys.stderr)
    print("=" * 70, file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
