# LidarAGL

**Advisory AGL height-callout + flare-tone box for an experimental aircraft.**

ESP32-S3 reads slant range from a belly-mounted **LightWare SF30/C** (default) or
**SF30/D** micro-lidar, converts it to AGL in feet, and drives an aircraft audio
panel through a **PCM5102A** I2S DAC with two simultaneous streams:

1. **Voice callouts** — spoken altitude numbers at discrete heights on the way down.
2. **A continuous "presence" tone** — pitch *ascends* and the level swells in (in
   dB) as the ground approaches, giving a hands-off "ground coming up" cue through
   the flare.

> ⚠️ **This is advisory only. It is not certified equipment and must never replace
> visual flare judgment.** A wrong or late callout is worse than silence — the
> firmware prioritizes correct, on-time numbers over features.

Built for a Glasair III (fast experimental single, ~95 mph over the fence, short
flare window). WiFi/Bluetooth are **off by design** (no EMI into the avionics).

---

## Sensor profiles — SF30/C vs SF30/D

The box **auto-detects** which LightWare unit is fitted at boot (it asks the sensor
its product name over the LWNX binary protocol) and loads the matching profile. If
it can't positively identify the unit, it **falls back to SF30/C** (the default;
`DEFAULT_SENSOR_MODEL` in `config.h`).

| | **SF30/C** (default) | **SF30/D** |
|---|---|---|
| Usable range | ~100 m / **328 ft** | ~200 m / **656 ft** |
| Callout ladder (ft AGL) | **200**, 100, 50, 40, 30, 20, 10 | **500, 400, 300**, 200, 100, 50, 40, 30, 20, 10 |
| Low-power cruise cutover | **250 ft** | **500 ft** |

**Why the SF30/C ladder starts at 200, not 300:** the SF30/C tops out around 328 ft,
so a 300 ft callout would sit inside the sensor's noisy upper margin and be too
erroneous. Starting at 200 gives the sensor headroom to breathe and keeps every
spoken number trustworthy. The SF30/D, with twice the range, adds the
**500 / 400 / 300** high-altitude callouts on top of the same flare-band set.

The callout ladders, cruise altitude, and max range live in
`main/sensor_profile.c` — everything else (pins, dB levels, tone band, timing) is
in `main/config.h`.

---

## Build & flash (you run these)

Prerequisites: **ESP-IDF v5.x** installed and `IDF_PATH` set.

```sh
# 1. Generate the voice clips first (see "Voice clips" below).
python tools/gen_clips.py

# 2. Build & flash.
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Wiring is in **[WIRING.md](WIRING.md)** — ESP32-S3 ↔ SF30 (UART), ESP32-S3 ↔
PCM5102A (I2S), the reset button, the isolation transformer, and power.

### Bring-up tip
For first bring-up you can switch the sensor parser to the legacy ASCII stream by
setting `#define SF30C_MODE SF30C_ASCII` in `config.h` — handy to confirm the UART
and ranging before exercising the LWNX binary path. Production uses `SF30C_BINARY`
(also required for sensor auto-detect).

---

## Voice clips

The spoken clips are **not** shipped — you generate them (the build embeds whatever
exists). The firmware skips any clip that's missing, so a partial set never breaks
the build or the tone.

```sh
# Real speech via offline Windows SAPI5 TTS (needs: pip install pyttsx3):
python tools/gen_clips.py
python tools/gen_clips.py --list-voices      # pick a voice
python tools/gen_clips.py --voice 1

# Bench BEEPS instead of speech (clearly labelled, opt-in only):
python tools/gen_clips.py --placeholder-tones

# Convert your own recordings to the embed format:
python tools/wav_to_pcm.py myfifty.wav       # -> assets/clips/myfifty.pcm
```

Clips are raw **16 kHz mono signed-16-bit-LE PCM** (headerless `.pcm`) in
`assets/clips/`. `main/CMakeLists.txt` globs that directory and embeds the files via
`EMBED_FILES`; `callouts.c` resolves each one with a **weak** symbol so a missing
clip is simply absent rather than a link error.

A distinctive **calibration-error chirp** (`calib_error.pcm`) is also generated and
played at boot if the box can't establish a ground reference (see below).

---

## How it decides what to do

### Ground reference (no manual calibration)
The box has no calibration knobs. It **learns** its ground/mount offset from the last
~10 on-ground readings (captured over ~1 s at boot) and stores them in NVS — **one
write per boot**, so flash wear is negligible. `AGL = measured_range − ground_avg`.

- A boot reading more than **10 ft** above the learned ground is treated as
  **airborne** (a lidar doesn't drift that far sitting on the ground): the box keeps
  the stored ground reference and computes AGL from the live reading. This is how it
  recovers gracefully from an **in-flight reboot** (brownout/glitch) instead of
  waking up thinking it's parked.
- If there's no usable reference (first ever boot while airborne, or a wiped buffer),
  it falls back to a **3 ft** emergency ground offset **and chirps a
  calibration-error tone** so you know.
- The **reset button** (held at power-on) wipes the learned ground and reboots —
  use it on install / re-install.

### State machine & callouts
- **Silent climb-out:** no takeoff callouts. Descent callouts arm only after the
  aircraft climbs through **100 ft** (`ARM_FT`).
- Each callout is an **edge-trigger on the way down**, fired once. It re-arms only
  after climbing back **20 ft above** that height (`REARM_MARGIN_FT`) — so hovering
  near a threshold doesn't machine-gun, and a **go-around** re-enables the numbers.
- A callout fires only on a genuine **downward crossing** (was above, now at/below),
  so an in-flight reboot never blurts every number at once.
- Climb/descent direction comes from the smoothed range trend (no IMU/baro) with a
  dead-band so noise can't flip it.

### Presence tone (perceptual)
- **Pitch ascends** as AGL falls (~600 Hz at 100 ft → ~1800 Hz near the ground),
  clamped to 500–3000 Hz (cuts cockpit noise, survives ANR headsets).
- **Volume is scheduled in dB**, not linear amplitude: silent above 100 ft, fading in
  at a barely-audible floor at 100 ft, reaching full presence by 50 ft and holding
  through the flare to the ground.
- The tone **ducks ~4 dB** while a voice number plays, so numbers are never masked.
- Every gain change and tone start/stop is raised-cosine ramped — no clicks, no
  startle. No alerting chirp before the numbers (it slows pilot response).

### Power
- WiFi/BT compiled out. In GROUND/CRUISE the tone is silent, the I2S channel is
  paused, and the MCU drops into automatic light-sleep between slow watch-polls. The
  instant a reading at/below cruise altitude appears (descent beginning), it spins
  back up to fast polling and resumes the audio.

---

## Project layout

```
main/
  config.h          all profile-independent tunables (pins, dB, tone band, timing)
  sensor_profile.*  the SF30/C & SF30/D profiles (callout ladders, cruise, range)
  lwnx.*            LightWare LWNX binary protocol (build/parse + hand-rolled CRC)
  sf30c.*           UART driver, ASCII+binary parse, cm→ft, EMA, autodetect, sensor task
  robust.*          median/MAD outlier filter (pure)
  boot_buffer.*     NVS ground-reference store, in-flight-reboot recovery, reset button
  state_machine.*   states, arming, edge-trigger callouts, hysteresis, poll profile (pure)
  audio_math.*      pitch map, dB schedule, gain, envelopes (pure)
  audio.*           i2s_std engine: NCO tone, dB volume, ducking, clip mixing, suspend/resume
  callouts.*        callout enum + embedded-clip manifest (graceful on missing clips)
  app_main.c        boot sequence, task spawning, the logic loop
assets/clips/       generated raw PCM voice clips (+ calib chirp)
tools/              gen_clips.py (TTS), wav_to_pcm.py (WAV→PCM)
test/               host unit tests (no hardware) for the pure modules
```

---

## Tests (no hardware required)

The safety-relevant logic (outlier filter, the whole callout/arming/hysteresis state
machine over **both** profiles, the perceptual audio math, and the LWNX framing/CRC)
is exercised on the host:

```sh
cd test
make            # builds + runs all four suites

# No 'make'? Run one suite directly, e.g.:
gcc -I../main -DUNIT_TEST ../main/robust.c ../main/sensor_profile.c \
    ../main/state_machine.c ../main/audio_math.c ../main/lwnx.c \
    test_state_machine.c -lm -o run_sm && ./run_sm
```

The pure modules include no `esp_*` / `driver/*` / `freertos/*` headers, which is why
they build and run on a desktop compiler.
