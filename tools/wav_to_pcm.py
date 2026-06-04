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

    # Convert everything (numbers + chirp + the boot-config-menu prompts):
    python tools/wav_to_pcm.py assets/original_audio/*.wav assets/original_audio/*.pcm

Config-menu prompts the firmware expects (drop the WAVs in original_audio/ and
convert; the build globs assets/clips/ and embeds whatever exists):
    chirp.wav                 -> chirp.pcm                  (short menu chirp)
    config_mode.wav           -> config_mode.pcm            ("config mode, memory cleared")
    mono.wav / stereo.wav     -> mono.pcm / stereo.pcm      (channel pieces)
    callouts_and_tone.wav     -> callouts_and_tone.pcm      (stream pieces)
    callouts_only.wav         -> callouts_only.pcm
    tone_only.wav             -> tone_only.pcm
    callout_start_altitude.wav-> callout_start_altitude.pcm ("callout start altitude")
The start-altitude numbers reuse the existing number clips (200..10).

Note: a source file that ends in .pcm but is actually a RIFF/WAVE file (renamed)
is detected by its header and decoded normally — so you can pass *.pcm too. A
genuinely headerless .pcm is skipped (it's already in the target format).

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

# --- Consonant softening (de-esser + plosive tamer) --------------------------
# Recorded/synth voice tends to spit on the hard "T" in "two/twenty/thirty" (a
# fast broadband plosive burst) and hiss on any "S" (sibilance — a narrow band
# of high-frequency energy). Because the firmware just plays the baked PCM back,
# we tame those consonants HERE, offline, so the embedded clip is already smooth
# and the runtime pays zero CPU and the tone stays untouched.
#
# How it works: isolate a high band with a 12 dB/oct high-pass, follow its
# envelope with an instant attack / slow release (so it pounces on each plosive
# and sibilant), and — only while that band is hot — subtract a scaled slice of
# it back out of the original. Vowels and body are left alone; just the brittle
# top edge of the offending consonants gets ducked.
#
# Tuned to MEDIUM by default. Presets (swap these four numbers to taste):
#     gentle : THRESH 0.45  RATIO 3.0  FLOOR 0.55  RELEASE 40
#     medium : THRESH 0.33  RATIO 4.0  FLOOR 0.40  RELEASE 35   <-- current
#     strong : THRESH 0.22  RATIO 6.0  FLOOR 0.28  RELEASE 28
DEESS_ENABLE      = True      # master switch for the whole stage
DEESS_SPLIT_HZ    = 4500.0    # corner of the sibilant/plosive high band (Hz)
DEESS_THRESH_FRAC = 0.33      # duck above this fraction of the band's own peak
DEESS_RATIO       = 4.0       # compression ratio applied to the high band
DEESS_FLOOR_GAIN  = 0.40      # most we ever attenuate the band (~ -8 dB)
DEESS_RELEASE_MS  = 35.0      # how quickly the duck recovers after a consonant

HERE     = os.path.dirname(os.path.abspath(__file__))
ROOT     = os.path.dirname(HERE)
CLIP_DIR = os.path.join(ROOT, "assets", "clips")

# --- Numeric-master -> spelled-clip name bridge ------------------------------
# The voice masters are named by HEIGHT (e.g. 600.wav) because that reads cleanly
# in original_audio/ and matches how the web emulator loads them. The FIRMWARE,
# though, globs SPELLED-OUT clip symbols (_binary_six_hundred_pcm, derived from
# six_hundred.pcm), so a numeric stem must be translated to its spelled clip name
# on the way out. Inputs not in this map (stereo.wav, chirp.wav, ...) keep their
# own stem unchanged — only the number clips need the rename.
NUMBER_NAMES = {
    "10": "ten",          "20": "twenty",        "30": "thirty",
    "40": "forty",        "50": "fifty",         "100": "one_hundred",
    "200": "two_hundred", "300": "three_hundred","400": "four_hundred",
    "500": "five_hundred","600": "six_hundred",
}


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


def soften_consonants(mono):
    """De-ess sibilants and soften plosive "T" bursts in a float mono signal.

    `mono` is a list of floats in roughly [-1, 1]. Returns a new list the same
    length with the harsh top edge of "S"/"T"/"TH" consonants ducked, the rest
    of the voice untouched. Self-calibrating: the threshold is a fraction of the
    clip's OWN high-band peak, so it adapts to recording level automatically.

    Pure-Python DSP (no numpy) to match the rest of this tool.
    """
    if not DEESS_ENABLE:
        return mono

    n = len(mono)
    if n < 4:
        return mono                       # too short to filter meaningfully

    # --- 1) Isolate the sibilant/plosive high band ---------------------------
    # Two cascaded one-pole high-passes give a 12 dB/oct split — steep enough to
    # grab "S"/"T" energy without dragging vowel formants up with it. Each pole
    # is a one-pole LOW-pass subtracted from its input (a complementary HPF).
    a = 1.0 - math.exp(-2.0 * math.pi * DEESS_SPLIT_HZ / SAMPLE_RATE)
    lp1 = 0.0
    lp2 = 0.0
    hb = [0.0] * n                        # the isolated high band, per sample
    for i, x in enumerate(mono):
        lp1 += a * (x - lp1)              # 1st pole: low part
        h1   = x - lp1                    #           high part
        lp2 += a * (h1 - lp2)             # 2nd pole on the high part
        hb[i] = h1 - lp2                  # 12 dB/oct high band

    # --- 2) Self-calibrating threshold ---------------------------------------
    hb_peak = max((abs(v) for v in hb), default=0.0)
    if hb_peak <= 0.0:
        return mono                       # no high-frequency content -> nothing to do
    thresh = DEESS_THRESH_FRAC * hb_peak
    if thresh <= 0.0:
        return mono

    # --- 3) Envelope follower + ratio gain on the band -----------------------
    # Instant attack (env jumps to any louder sample) so a plosive can't sneak
    # through before the duck engages; exponential release so the gain glides
    # back smoothly with no zipper/pumping artefacts.
    rel = 1.0 - math.exp(-1.0 / (DEESS_RELEASE_MS * 0.001 * SAMPLE_RATE))
    gain_exp = (1.0 / DEESS_RATIO) - 1.0  # < 0 => the louder the band, the more we cut

    env = 0.0
    out = [0.0] * n
    for i in range(n):
        mag = abs(hb[i])
        if mag > env:
            env = mag                     # instant attack
        else:
            env += rel * (mag - env)      # slow release

        if env > thresh:
            over = env / thresh           # how far over threshold we are
            g = over ** gain_exp          # ratio gain (1.0 at threshold, falling)
            if g < DEESS_FLOOR_GAIN:
                g = DEESS_FLOOR_GAIN      # clamp the deepest cut
        else:
            g = 1.0                       # below threshold -> band passes clean

        # Subtract the ducked *portion* of the high band from the original. At
        # g == 1 this is a no-op; as g falls we notch out the brittle top edge.
        out[i] = mono[i] - (1.0 - g) * hb[i]

    return out


def is_riff_wav(path):
    """True if the file actually begins with a RIFF/WAVE header.

    Some source clips arrive with a .pcm extension but are really WAV files
    (e.g. exported then renamed). We sniff the magic bytes so those still get
    decoded properly instead of being treated as headerless raw PCM.
    """
    try:
        with open(path, "rb") as f:
            head = f.read(12)
        return head[:4] == b"RIFF" and head[8:12] == b"WAVE"
    except OSError:
        return False


def wav_to_samples(wav_path):
    """Decode a WAV to s16 mono @ 16 kHz, returning a list of int samples."""
    with wave.open(wav_path, "rb") as w:
        nch     = w.getnchannels()
        width   = w.getsampwidth()
        rate    = w.getframerate()
        nframes = w.getnframes()
        raw     = w.readframes(nframes)

    if width == 2:
        # 16-bit signed little-endian.
        ints = list(struct.unpack("<%dh" % (len(raw) // 2), raw))
        scale = 32768.0
    elif width == 1:
        # 8-bit WAV is unsigned (0..255, centred on 128).
        ints = [b - 128 for b in raw]
        scale = 128.0
    elif width == 3:
        # 24-bit signed little-endian. There's no struct code for 3-byte ints,
        # so assemble each sample from its 3 bytes and sign-extend the top bit.
        ints = []
        for i in range(0, len(raw) - 2, 3):
            v = raw[i] | (raw[i + 1] << 8) | (raw[i + 2] << 16)
            if v & 0x800000:                   # negative -> sign-extend to 32-bit
                v -= 0x1000000
            ints.append(v)
        scale = 8388608.0                      # 2^23
    elif width == 4:
        # 32-bit. Could be int OR float (WAVE_FORMAT_IEEE_FLOAT); wave can't tell
        # us the format tag, so sniff it: float PCM lives in roughly [-1, 1], int
        # PCM spans the full +/-2^31 range. Decode as int, and if every value is
        # tiny relative to the int range, re-read it as float.
        ints = list(struct.unpack("<%di" % (len(raw) // 4), raw))
        peak = max((abs(x) for x in ints), default=0)
        if peak <= 0x7FFFFF:                   # never uses the high bytes -> float
            floats = struct.unpack("<%df" % (len(raw) // 4), raw)
            ints   = [int(x * 2147483648.0) for x in floats]
        scale = 2147483648.0                   # 2^31
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

    # Soften the harsh "T" plosives and "S" sibilance before trimming/normalising,
    # while the signal is still float in ~[-1, 1] (the natural scale for the DSP).
    mono = soften_consonants(mono)

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
        # Skip anything that isn't actually a WAV (covers .pcm files that are
        # already headerless raw PCM, and rejects junk early). A .pcm input that
        # is secretly RIFF/WAVE is detected and decoded normally.
        if not is_riff_wav(wav_path):
            print(f"  [skip] not a WAV (no RIFF header): {wav_path}",
                  file=sys.stderr)
            continue
        samples = wav_to_samples(wav_path)
        if args.output:
            out_path = args.output
        else:
            stem = os.path.splitext(os.path.basename(wav_path))[0]
            stem = NUMBER_NAMES.get(stem, stem)   # numeric height -> spelled clip name
            out_path = os.path.join(CLIP_DIR, stem + ".pcm")
        write_pcm(samples, out_path)

    return 0


if __name__ == "__main__":
    sys.exit(main())
