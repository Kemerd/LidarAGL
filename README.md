# LidarAGL — by [Novabox](https://novabox.works/)

**Advisory AGL height-callout + flare-tone box for experimental aircraft.**

An ESP32-S3 reads slant range from a belly-mounted **LightWare SF30/C** (default) or
**SF30/D** micro-lidar, converts it to height above ground (AGL) in feet, and drives
your aircraft audio panel through a line-level DAC with two simultaneous streams:

1. **Voice callouts** — spoken altitude numbers at discrete heights on the way down
   ("two hundred… one hundred… fifty… forty…").
2. **A continuous "presence" tone** — pitch *ascends* and the level swells in (in dB)
   as the ground approaches, a hands-off "ground coming up" cue right through the flare.

Designed around a Glasair III (fast experimental single, ~95 mph over the fence,
short flare window). WiFi/Bluetooth are **off by design** — no EMI into the avionics.

> ⚠️ **Advisory only. Not certified equipment. Never a substitute for visual flare
> judgment.** A wrong or late callout is worse than silence — the firmware
> prioritizes correct, on-time numbers over features.

> 🛒 **Novabox project.** Files and build kits available at
> **[novabox.works](https://novabox.works/)**. A complete unit (including the smart
> threaded housing with acrylic lens — see [Enclosure](#enclosure-3d-printed-housing))
> is available for purchase at **$599.95** for 100m range unit or $699.95 for the 200m range unit.

---

## Table of contents
- [What you need (Bill of Materials)](#what-you-need-bill-of-materials)
- [GPIO pinout & wiring](#gpio-pinout--wiring)
- [Power](#power)
- [Sensor profiles — SF30/C vs SF30/D](#sensor-profiles--sf30c-vs-sf30d)
- [Software setup & flashing](#software-setup--flashing)
- [Voice clips](#voice-clips)
- [How it works](#how-it-works)
- [Enclosure (3D-printed housing)](#enclosure-3d-printed-housing)
- [Project layout](#project-layout)
- [Tests](#tests)

---

## What you need (Bill of Materials)

| # | Component | Role | Notes |
|---|-----------|------|-------|
| 1 | **ESP32-S3** devkit or module (≥8 MB flash, PSRAM helpful) | The MCU / brain | e.g. ESP32-S3-DevKitC-1. Native USB used for flashing + logging. |
| 2 | **LightWare SF30/C** (100 m) **or SF30/D** (200 m) micro-lidar | Slant-range rangefinder | SF30/C is the default. The box auto-detects which is fitted. |
| 3 | **PCM5102A** I2S DAC breakout | Digital audio → line level | The common purple/black breakout. **SCK jumper must go to GND** (internal PLL). |
| 4 | **600:600 Ω audio isolation transformer** | Ground-loop isolation into the panel | 1:1 line transformer. Breaks ground loops + blocks switching noise from the bus. |
| 5 | **Level-trim potentiometer** (~10 kΩ) | Master output level | Sets absolute loudness into the panel. Firmware only shapes the *perceptual* ramp. |
| 6 | **Momentary push button** | Reset / re-learn ground reference | Wipes the learned ground offset and reboots. Active-low, uses internal pull-up. |
| 7 | **Clean regulated power** (5 V DC-DC + low-noise LDO) | Supply | The SF30 gets its **own regulated rail**; share only ground with the S3. |
| 8 | Hookup wire, JST/Dupont connectors | Interconnect | — |
| 9 | **Acrylic lens disc** + Novabox threaded housing | Enclosure | See [Enclosure](#enclosure-3d-printed-housing). Available at novabox.works. |

**Interfaces used:** UART (sensor), I2S (DAC), one GPIO (reset button), USB
(flash/log). All sensor & DAC logic is 3.3 V.

---

## GPIO pinout & wiring

> All GPIO numbers are **defaults defined in [`main/config.h`](main/config.h)** — if
> they conflict on your specific devkit, change them there (not in the wiring). A
> standalone copy of this map with extra notes is in **[WIRING.md](WIRING.md)**.

### Block diagram

```
  SF30/C or SF30/D ──UART (3.3V TTL)──► ESP32-S3 ──I2S──► PCM5102A ──► trim pot
   (belly lidar)                         (MCU)             (DAC)          │
                                                                         ▼
                                                          600:600 Ω isolation xfmr
                                                                         │
                                                                         ▼
                                                          Aircraft audio panel
                                                          (AUX / ALERT input)
```

### 1) SF30 lidar → ESP32-S3 — UART1, 115200 8N1

| SF30 pin | wire to | ESP32-S3 GPIO | `config.h` |
|----------|:-------:|---------------|------------|
| **TX**   |  →      | **GPIO 17** (UART RX) | `PIN_SF30C_RX` |
| **RX**   |  →      | **GPIO 18** (UART TX) | `PIN_SF30C_TX` |
| **GND**  |  →      | common GND            | — |
| **V+**   |  →      | **own regulated rail** (NOT the S3 3V3) | — |

> Note the crossover: sensor **TX** → S3 **RX**, sensor **RX** → S3 **TX**.
> Give the SF30 its own clean rail — it draws real current and is noisy on its
> supply. Share **ground only** with the S3.

### 2) PCM5102A DAC → ESP32-S3 — I2S

| PCM5102A pin | wire to | ESP32-S3 GPIO | `config.h` |
|--------------|:-------:|---------------|------------|
| **BCK**      |  →      | **GPIO 5**    | `PIN_I2S_BCK` |
| **LRCK** (WS/LCK) | →  | **GPIO 6**    | `PIN_I2S_LRCK` |
| **DIN**      |  →      | **GPIO 7** (I2S DOUT from S3) | `PIN_I2S_DIN` |
| **SCK**      |  →      | **GND**       | — (forces internal PLL; no MCLK) |
| **VIN**      |  →      | 3V3           | — |
| **GND**      |  →      | common GND    | — |

> **SCK → GND is required.** It forces the PCM5102A to run from its internal PLL, so
> the S3 emits **no MCLK** (firmware sets `mclk = I2S_GPIO_UNUSED`). Leave the onboard
> jumpers FLT / DEMP / XSMT / FMT at their board defaults (I2S format, normal latency).

### 3) Reset button → ESP32-S3

| Button | wire to | ESP32-S3 GPIO | `config.h` |
|--------|:-------:|---------------|------------|
| one leg  | →     | **GPIO 4**    | `PIN_RESET_BTN` |
| other leg| →     | GND           | — |

> Active-low with the S3's internal pull-up (no external resistor). **Held at
> power-on** → the box wipes the stored ground readings and reboots. Use it on
> install / re-install so the ground reference re-learns fresh.

### 4) Analog output → audio panel

```
PCM5102A LOUT/ROUT ─► level-trim pot ─► 600:600 Ω isolation transformer ─► panel AUX/ALERT
```

- Mono: tie L+R, or use a single channel. Most breakouts AC-couple their output.
- Set **absolute loudness with the trim pot** — the firmware only shapes the dB ramp.

### ⚠️ GPIO cautions
- Avoid the **strapping pins (0, 45, 46)** and the **native-USB pins (19, 20)** — the
  console logs over USB-Serial-JTAG.
- Any other free GPIOs work for UART/I2S/button; if you remap, do it in `config.h`.

---

## Power

- Feed the S3 + DAC from a **clean regulated supply**: DC-DC down to 5 V, then a
  **low-noise LDO** to the DAC analog rail.
- The firmware assumes it may **brown out / reboot in flight**. That's handled by the
  in-flight-reboot recovery (it reconstructs roughly where you are instead of waking
  up thinking it's parked — see [How it works](#how-it-works)).

---

## Sensor profiles — SF30/C vs SF30/D

The box **auto-detects** the fitted sensor at boot (it asks the unit its product name
over the LWNX binary protocol) and loads the matching profile. If it can't positively
identify the unit, it **falls back to SF30/C** (`DEFAULT_SENSOR_MODEL` in `config.h`).

| | **SF30/C** (default) | **SF30/D** |
|---|---|---|
| Usable range | ~100 m / **328 ft** | ~200 m / **656 ft** |
| Callout ladder (ft AGL) | **200**, 100, 50, 40, 30, 20, 10 | **500, 400, 300**, 200, 100, 50, 40, 30, 20, 10 |
| Low-power cruise cutover | **250 ft** | **500 ft** |

**Why the SF30/C ladder starts at 200, not 300:** the SF30/C tops out near 328 ft, so a
300 ft callout would sit inside the sensor's noisy upper margin and be too erroneous.
Starting at 200 gives the sensor headroom to breathe and keeps every spoken number
trustworthy. The SF30/D, with twice the range, adds the **500 / 400 / 300** callouts on
top of the same flare-band set.

Callout ladders, cruise altitude, and max range live in
[`main/sensor_profile.c`](main/sensor_profile.c); everything else (pins, dB levels,
tone band, timing) is in [`main/config.h`](main/config.h).

---

## Software setup & flashing

Prerequisites: **ESP-IDF v5.x** installed and `IDF_PATH` set, plus **Python 3** for the
clip tools.

```sh
# 0. Clone, then from the project root:

# 1. Generate the voice clips (see the next section). Either convert your own WAVs:
python tools/wav_to_pcm.py assets/original_audio/*.wav     # -> assets/clips/*.pcm
#    ...or synthesize them with TTS:
python tools/gen_clips.py                                  # needs: pip install pyttsx3

# 2. Build & flash.
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

**First bring-up tip:** to confirm the UART and ranging before exercising the binary
protocol, set `#define SF30C_MODE SF30C_ASCII` in `config.h` — this uses the legacy
ASCII distance stream and just prints range. Production uses `SF30C_BINARY` (also
required for sensor auto-detect).

---

## Voice clips

The spoken clips are raw **16 kHz mono signed-16-bit-LE PCM** (headerless `.pcm`) in
`assets/clips/`. The build **globs that folder and embeds whatever exists**; any missing
clip is skipped gracefully by the firmware (the tone keeps running).

You have two ways to make them:

**A) Convert your own / AI-generated WAVs (recommended):**
```sh
python tools/wav_to_pcm.py assets/original_audio/*.wav
```
`wav_to_pcm.py` resamples to 16 kHz, downmixes to mono, **trims leading/trailing
silence** (with a small pad + click-free fades), normalizes the level, and strips the
header. Filenames map to the firmware clip names — see
[`tools/voice_prompts.txt`](tools/voice_prompts.txt) for the exact phrase→filename list
and a ready-made prompt you can paste into any TTS/voice generator.

**B) Synthesize with offline Windows SAPI5 TTS:**
```sh
pip install pyttsx3
python tools/gen_clips.py                 # real speech
python tools/gen_clips.py --list-voices   # pick a voice, then --voice N
python tools/gen_clips.py --placeholder-tones   # bench BEEPS, clearly labelled
```

A distinctive **calibration-error chirp** (`calib_error.pcm`) is also generated and
plays at boot if the box can't establish a ground reference.

---

## How it works

### Ground reference (no manual calibration)
The box has no calibration knobs. It **learns** its ground/mount offset from the last
~10 on-ground readings (captured over ~1 s at boot) and stores them in NVS — **one write
per boot**, so flash wear is negligible. `AGL = measured_range − ground_avg`.

- A boot reading more than **10 ft** above the learned ground is treated as **airborne**
  (a lidar doesn't drift that far on the ground): the box keeps the stored reference and
  computes AGL from the live reading. This is how it recovers gracefully from an
  **in-flight reboot** instead of waking up thinking it's parked.
- No usable reference (first-ever boot while airborne, or a wiped buffer) → it falls
  back to a **3 ft** emergency offset **and chirps a calibration-error tone** so you know.
- The **reset button** (held at power-on) wipes the learned ground and reboots.

### State machine & callouts
- **Silent climb-out:** no takeoff callouts. Descent callouts arm only after climbing
  through **100 ft** (`ARM_FT`).
- Each callout is a one-shot **edge-trigger on the way down**. It re-arms only after
  climbing **20 ft above** that height (`REARM_MARGIN_FT`) — so hovering near a threshold
  doesn't machine-gun, and a **go-around** re-enables the numbers.
- A callout fires only on a genuine **downward crossing**, so an in-flight reboot never
  blurts every number at once.
- Climb/descent direction comes from the smoothed range trend (no IMU/baro) with a
  dead-band so noise can't flip it.

### Presence tone (perceptual)
- **Pitch ascends** as AGL falls (~600 Hz at 100 ft → ~1800 Hz near the ground), clamped
  to 500–3000 Hz (cuts cockpit noise, survives ANR headsets).
- **Volume scheduled in dB**, not linear amplitude: silent above 100 ft, fading in at a
  barely-audible floor at 100 ft, full presence by 50 ft and held through the flare.
- The tone **ducks ~4 dB** while a number plays, so numbers are never masked. No alerting
  chirp before the numbers (it slows pilot response).
- Every gain change and tone start/stop is raised-cosine ramped — no clicks, no startle.

### Power
- WiFi/BT compiled out. In GROUND/CRUISE the tone is silent, the I2S channel is paused,
  and the MCU drops into automatic light-sleep between slow watch-polls. The instant a
  reading at/below cruise altitude appears, it spins back up to fast polling and resumes
  audio.

---

## Enclosure (3D-printed housing)

> 🚧 **Placeholder — full housing guide coming.**

The production unit ships in a **Novabox smart threaded housing**: the body has an
integral thread so it acts as its own compression fitting — you cut a hole, drop in the
**acrylic lens**, and thread the housing down to clamp everything into place (no separate
hardware, clean optical window for the lidar/indicator).

This section will be filled in with:
- [ ] STL / STEP files and print settings (material, walls, infill, orientation)
- [ ] Acrylic lens spec (diameter, thickness, cut template)
- [ ] Cut-hole dimensions and the thread/compression assembly steps
- [ ] Mounting guidance (belly location, sensor aim, strain relief)
- [ ] BOM additions (gasket/o-ring, fasteners if any)

Print files and ready-made units are at **[novabox.works](https://novabox.works/)**
(complete unit **$599.95**).

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
assets/
  clips/            generated raw PCM voice clips (+ calib chirp) — embedded by the build
  original_audio/   source WAV masters (re-convert with tools/wav_to_pcm.py)
tools/
  wav_to_pcm.py     WAV → 16k mono PCM, with silence trim + normalize
  gen_clips.py      offline SAPI5 TTS clip generator
  voice_prompts.txt phrase→filename list + a paste-ready TTS prompt
test/               host unit tests (no hardware) for the pure modules
WIRING.md           standalone wiring reference
```

---

## Tests

The safety-relevant logic (outlier filter, the whole callout/arming/hysteresis state
machine over **both** profiles, the perceptual audio math, and the LWNX framing/CRC) is
exercised on the host — no hardware required:

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

---

*LidarAGL is an open project by [Novabox](https://novabox.works/). Advisory use only.*
