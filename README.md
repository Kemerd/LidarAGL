# LidarAGL — open source, by [Novabox.Works](https://novabox.works/)

**Advisory AGL height-callout + flare-tone box for experimental aircraft.**

An ESP32-S3 reads slant range from a belly-mounted **LightWare SF30/C** (default) or
**SF30/D** micro-lidar, converts it to height above ground (AGL) in feet, and drives
your aircraft audio panel through a line-level DAC with two simultaneous streams:

1. **Voice callouts** — spoken altitude numbers at discrete heights on the way down
   ("two hundred… one hundred… fifty… forty…").
2. **A continuous "presence" tone** — pitch *ascends* all the way down (100→0 ft) as a
   hands-off "ground coming up" cue right through the flare. The level only swells in
   over 100→50 ft (a gentle fade-in so it doesn't appear jarringly); from 50 ft to the
   ground the *perceived* loudness is held constant — urgency is carried by the rising
   pitch, never by getting louder.

Designed around a Glasair III (fast experimental single, ~95 mph over the fence,
short flare window). WiFi/Bluetooth are **off by design** — no EMI into the avionics.

> ⚠️ **Advisory only. Not certified equipment. Never a substitute for visual flare
> judgment.** A wrong or late callout is worse than silence — the firmware
> prioritizes correct, on-time numbers over features.

> 🛒 **Novabox project.** Files and build kits available at
> **[novabox.works](https://novabox.works/)**. A complete unit (including the smart
> threaded housing with acrylic lens — see [Enclosure](#enclosure-3d-printed-housing))
> is available for purchase at **$599.95** (100 m / SF30/C) or **$699.95**
> (200 m / SF30/D).

---

## Table of contents
- [What you need (Bill of Materials)](#what-you-need-bill-of-materials)
- [GPIO pinout & wiring](#gpio-pinout--wiring)
- [Power](#power)
- [Sensor profiles — SF30/C vs SF30/D](#sensor-profiles--sf30c-vs-sf30d)
- [Configuring the unit](#configuring-the-unit)
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
| 3 | **PCM5102A** I2S DAC breakout | Digital audio → **line level** (stereo) | The common purple/black breakout. **SCK jumper must go to GND** (internal PLL). Output goes straight to the panel — it's line level, so the panel sets volume (no trim pot needed). |
| 4 | **Momentary push button** | Config button (set audio modes + re-learn ground) | Hold at power-on for the config menu; wipes the learned ground + saved config. Active-low, internal pull-up. Mount in the cockpit on a harness conductor. |
| 5 | **Clean regulated 5 V supply** | Supply | A single clean 5 V rail feeds the S3, the DAC, **and** the SF30. See [Power](#power). |
| 6 | **Multi-conductor shielded cable** (≥6 cores) | Cockpit harness | Carries +, −, L, R, panel audio-LO reference, and the config button. See [Cockpit harness](#cockpit-harness-6-conductor-shielded). |
| 7 | **Acrylic lens disc** (55 mm ideal, or 2″ + gasket) + Novabox threaded housing | Enclosure | One 2.5″ hole, no screws — compression fit. See [Enclosure](#enclosure-3d-printed-housing). Available at Novabox.Works. |

**Interfaces used:** UART (sensor), I2S (DAC, **stereo**), one GPIO (config button),
USB (flash/log). All sensor & DAC logic is 3.3 V.

---

## GPIO pinout & wiring

> All GPIO numbers are **defaults defined in [`main/config.h`](main/config.h)** — if
> they conflict on your specific devkit, change them there (not in the wiring). A
> standalone copy of this map with extra notes is in **[WIRING.md](WIRING.md)**.

### Block diagram

```
  SF30/C or SF30/D ──UART (3.3V TTL)──► ESP32-S3 ──I2S──► PCM5102A ── L / R / LO ──►
   (belly lidar)                         (MCU)             (DAC)                     │
                                                                                     ▼
                                                                       GMA 245 audio panel
                                                                       (stereo aux input)
```

### 1) SF30 lidar → ESP32-S3 — UART1, 115200 8N1

| SF30 pin | wire to | ESP32-S3 GPIO | `config.h` |
|----------|:-------:|---------------|------------|
| **TX**   |  →      | **GPIO 8** (UART RX) | `PIN_SF30C_RX` |
| **RX**   |  →      | **GPIO 9** (UART TX) | `PIN_SF30C_TX` |
| **GND**  |  →      | common GND            | — |
| **V+**   |  →      | the shared clean 5 V rail | — |

> Note the crossover: sensor **TX** → S3 **RX**, sensor **RX** → S3 **TX**.
> In an aircraft you'll run everything from one regulated 5 V rail (a separate
> sensor rail isn't practical in the panel). The SF30 draws real current and is
> noisy on its supply, so feed it through a small **local LC/ferrite + bulk cap**
> right at the sensor to keep its switching hash off the shared rail.

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

### 3) Config button → ESP32-S3

| Button | wire to | ESP32-S3 GPIO | `config.h` |
|--------|:-------:|---------------|------------|
| one leg  | →     | **GPIO 4**    | `PIN_CONFIG_BTN` |
| other leg| →     | GND           | — |

> Active-low with the S3's internal pull-up (no external resistor). **Held at
> power-on** → the box enters the **config menu**: it wipes the stored ground
> readings *and* the saved audio config, then lets you tap to pick the audio mode
> and callout start altitude (see [Configuring the unit](#configuring-the-unit)).
> Use it on install / re-install so the ground reference re-learns fresh. In the
> aircraft install this button lives in the **cockpit**, reached over harness
> conductor 6 (see [Cockpit harness](#cockpit-harness-6-conductor-shielded)).

### 4) Analog output → audio panel (GMA 245)

The reference install feeds a **Garmin GMA 245**'s stereo unbalanced **aux input**.
That input is built to take a consumer/line-level stereo source directly and
provides its own **audio LO** (signal-ground reference), so the standalone
600:600 Ω isolation transformer of the original bench design is **not required** —
the panel's input stage does the level/impedance work, and referencing both audio
channels to the panel's LO keeps the audio return off the noisy power ground.

```
PCM5102A LOUT ─┐
PCM5102A ROUT ─┼─► GMA 245 aux  (L, R, audio-LO ref)
   panel LO  ◄──┘
```

- **Line level — no trim pot.** The DAC output is line level and goes **straight to
  the panel**; the GMA 245 sets the volume, so there's no master-level pot in the
  chain. The firmware only shapes the *perceptual* dB ramp.
- **Stereo by default.** The firmware emits L/R and gently pans the streams to
  opposite sides (voice right, tone left — see [How it works](#presence-tone-perceptual)).
  Mono and stereo are a **runtime** choice — pick it from the
  [config menu](#configuring-the-unit); the I2S hardware
  always drives both channels, so mono just sends the same signal to L and R
  (safe even if the panel is wired stereo). `DEFAULT_AUDIO_MODE` in `config.h` sets
  the post-wipe default.
- **Audio LO is the reference, not power ground.** Land L and R against the GMA's
  **audio LO** conductor — that's what isolates the audio return from the supply.

#### Cockpit harness (6-conductor, shielded)

Run a single shielded multi-core from the box to the cockpit. Six cores cover
power, stereo audio + its reference, and the remote button:

| # | Conductor | From → to |
|---|-----------|-----------|
| 1 | **+** (5 V) | clean 5 V rail → box supply |
| 2 | **−** (GND) | power ground → box ground |
| 3 | **L** | DAC LOUT → GMA 245 aux **L** (line level) |
| 4 | **R** | DAC ROUT → GMA 245 aux **R** (line level) |
| 5 | **LO** | GMA 245 **audio-LO** reference ← box audio return |
| 6 | **BTN** | cockpit config button → **GPIO 4** (`PIN_CONFIG_BTN`) |

> **Shield:** bond the cable shield to chassis at **one end only — the panel
> side** — so it drains noise without forming a ground loop. Never bond both ends.
>
> **Config button:** the §3 config button, relocated into the cockpit on
> conductor 6 (active-low to the box's GND via the S3's internal pull-up). Held at
> power-on it opens the config menu (wipes ground + saved config, then tap/double-
> tap to set the audio mode + start altitude — see
> [Configuring the unit](#configuring-the-unit)).
>
> **Ground-loop fallback:** a GMA aux input + single-end shield is normally quiet.
> If you ever hear supply-correlated whine (alternator buzz / switching hash) in
> the channel, drop a small **inline aux ground-loop isolator** (a compact dual
> 1:1 transformer barrel) into the L/R lines — same fix as the old transformer,
> far smaller.

### ⚠️ GPIO cautions
- **Mini-board pin budget.** The reference build targets a **mini ESP32-S3** that only
  breaks out **GPIO 1–13** (plus 5 V, GND, 3V3, and the USB TX/RX), so every default
  pin lives in that range: UART **8/9**, I2S **5/6/7**, config button **4**.
- Avoid the **strapping pins (0, 3, 45, 46)** and the **native-USB pins (19, 20)** — the
  console logs over USB-Serial-JTAG.
- Any other free GPIOs work for UART/I2S/button; if you remap, do it in `config.h`.

---

## Power

- One **clean regulated 5 V rail** feeds the whole box — the S3, the DAC, and the
  SF30 — over harness conductors **+ / −**. A separate sensor rail isn't practical
  in an aircraft panel, so instead of isolating supplies, isolate *noise*:
  - A **low-noise LDO** off the 5 V for the DAC analog rail.
  - A **local LC/ferrite + bulk cap** at the SF30 so its switching draw and supply
    noise stay local instead of riding back onto the shared rail.
  - Keep the **audio return on the panel's audio LO**, not power ground (see
    [Analog output](#4-analog-output--audio-panel-gma-245)), so supply hash can't
    couple into the audio.
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
| Callout ladder (ft AGL) | **300**, 200, 100, 50, 40, 30, 20, 10 | **600, 500, 400, 300**, 200, 100, 50, 40, 30, 20, 10 |
| Low-power cruise cutover | **318 ft** | **605 ft** |

**Why the SF30/C ladder tops at 300, not 328:** the SF30/C tops out near 328 ft, so a
callout right at the ceiling would sit inside the sensor's noisy upper margin and be too
erroneous. Topping the ladder at 300 gives the sensor headroom to breathe and keeps every
spoken number trustworthy. The SF30/D, with twice the range, adds the **600 / 500 / 400**
high-altitude callouts on top of the same set.

Callout ladders, cruise altitude, and max range live in
[`main/sensor_profile.c`](main/sensor_profile.c); everything else (pins, dB levels,
tone band, timing) is in [`main/config.h`](main/config.h).

---

## Configuring the unit

There are **no settings to flash and no app** — everything is set from the
cockpit **config button** (the single button on harness conductor 6). All audio
behaviour is chosen here at boot, stored in NVS, and kept across power cycles
until you change it again.

### Defaults (out of the box / after a wipe)
| Setting | Default | Notes |
|---|---|---|
| **Audio mode** | **Stereo, Callouts & Tone** | `DEFAULT_AUDIO_MODE` in `config.h` |
| **Callout start altitude** | **Profile top** — 300 ft (SF30/C) / 600 ft (SF30/D) | the highest number that speaks |
| **Tone volume** | **0 dB** (no change) | trim on the presence tone, ±6 dB |
| **Voice volume** | **0 dB** (no cut) | cut-only trim on the callouts, 0…−6 dB |
| **Check-gear altitude** | **OFF** | descent "check gear" reminder; off until you set it |
| **Positive rate** | **OFF** | takeoff climb callout; off until you enable it |
| Ground reference | learned at every boot | not a menu item — see [Ground reference](#ground-reference-no-manual-calibration) |

### Entering config mode
**Hold the config button while powering the unit on.** You'll hear a **chirp**,
then *"Config mode, memory cleared."* Entering config mode **always wipes** the
learned ground reference **and** the saved audio/start-altitude config first — so
a hold-at-boot is also your clean-slate / re-install reset.

> Do this **on the ground**, on the install surface, so the ground reference
> re-learns fresh when it reboots.

### The config menu
The button has two gestures: a **single tap** to cycle, a **double-tap** to
confirm (or just **wait ~5 s** to confirm the current choice). You walk the levels
in order; each confirm advances to the next.

**Level 1 — Audio mode.** Each option is spoken as you tap to it:

| Taps to | Spoken | What you get |
|---|---|---|
| 1 | "Mono · Callouts and Tone" | both streams, same signal to L+R |
| 2 | "Stereo · Callouts and Tone" | both streams, voice→right / tone→left |
| 3 | "Mono · Callouts Only" | numbers only, no tone |
| 4 | "Mono · Tone Only" | tone only, no numbers |

Double-tap (or wait) to confirm → **chirp**.

**Level 2 — Callout start altitude.** *Skipped automatically if you picked
Tone Only* (no callouts to gate). Otherwise you'll hear *"Callout start
altitude"* then the current value. Tap to step **down** the ladder; it's the
**highest number that will speak** — everything above it is silenced (the tone is
unaffected):

- **SF30/C:** 300 → 200 → 100 → 50 → 40 → 30 → 20 → 10 (wraps back to 300)
- **SF30/D:** 600 → 500 → 400 → 300 → 200 → 100 → 50 → 40 → 30 → 20 → 10

Double-tap (or wait) to confirm → **chirp**.

> **Example:** pick *Stereo, Callouts & Tone*, then set start altitude to **100** on
> an SF30/C → you'll hear "one hundred, fifty, forty…" down to ten, but never
> "two hundred", with the stereo presence tone the whole way down.

**Level 3 — Tone volume.** Always runs (the tone sounds in every mode). You'll hear
*"Volume Adjustment · Tone Only"*, then each tap steps the tone trim and **previews it
live** as a short "mini-flare" (the real presence tone sweeping 20→10 ft with the "20"
and "10" callouts ducking it). It cuts **or** boosts in 2 dB steps and wraps around:
`0 → −2 → −4 → −6 → +6 → +4 → +2 → 0 …`. Double-tap (or wait) to confirm → **chirp**.

**Level 4 — Voice volume.** *Skipped in Tone Only* (no callouts to trim). You'll hear
*"Volume Adjustment · Callouts Only"*; each tap previews the same mini-flare with the
voice at the chosen level. Cut-only, in 2 dB steps, wrapping back to no-cut:
`0 → −2 → −4 → −6 → 0 …`. Double-tap (or wait) to confirm → **chirp**.

The last two levels are **optional callouts**, both **off by default** and both
*skipped in Tone Only* (they are spoken callouts).

**Level 5 — "Check Gear" altitude.** *Off by default.* You'll hear *"check gear"* then the
current value. Tap to cycle **OFF → highest → … → lowest → OFF**; double-tap (or wait)
to confirm. When set, the box speaks the altitude number **then "check gear"** as you
descend through it — e.g. *"two hundred … check gear"* — and the reminder always
speaks even if it sits above your start-altitude cap (it's a deliberate, independent
call):

- **SF30/C:** OFF → 200 → 100
- **SF30/D:** OFF → 500 → 400 → 300 → 200 → 100

**Level 6 — "Positive Rate" callout.** *Off by default.* A takeoff "positive rate of climb"
reminder. You'll hear *"positive rate"* then the current setting; tap to toggle
**ON / OFF**, double-tap (or wait) to confirm. When ON, the box says *"positive rate"*
once after each takeoff (and touch-and-go) — but only after a **confirmed** climb: it
arms once the aircraft has settled in the flare region (held below 10 ft long enough
for the tone's fade-out to finish, so a bounce never arms it), then fires after the
climb rate holds **≥ 100 fpm for a continuous 2 s** above 10 ft. That confirmation
window is what rejects a bounce, a flare balloon, or sensor jitter.

After confirming this last level the unit **reboots** into normal operation.

### Wiring for stereo
Mono vs stereo is a runtime choice and the DAC **always drives both channels**, so
the unit works either way — but to actually *hear* stereo separation you must run
**both** audio conductors to a stereo input:

| Want | Wire |
|---|---|
| **Stereo** (separation) | DAC **LOUT → L**, **ROUT → R**, both referenced to the panel **audio LO** (a stereo aux, e.g. GMA 245) |
| **Mono** | tie **LOUT + ROUT** together (or use one), into one input + LO |

See [Analog output → audio panel (GMA 245)](#4-analog-output--audio-panel-gma-245)
and the [cockpit harness](#cockpit-harness-6-conductor-shielded). In *Mono* mode
the firmware sends the identical signal to both pins, so a stereo-wired unit set
to mono still works perfectly — the lean just isn't there.

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

**Config-menu clips.** Beyond the altitude numbers + `calibration_error.pcm`, the
[config menu](#configuring-the-unit) uses these files (drop them in `assets/clips/` —
the build picks them up automatically). To save flash the spoken options are
**composed from pieces** (a channel piece + a stream piece) rather than one clip per
phrase, and the start-altitude reuses the existing number clips:

| File | Spoken phrase |
|------|---------------|
| `chirp.pcm` | short chirp (entry / each confirm) |
| `config_mode.pcm` | "Config mode, memory cleared" |
| `mono.pcm` / `stereo.pcm` | "Mono" / "Stereo" (channel piece) |
| `callouts_and_tone.pcm` | "Callouts and Tone" (stream piece) |
| `callouts_only.pcm` | "Callouts Only" (stream piece) |
| `tone_only.pcm` | "Tone Only" (stream piece) |
| `callout_start_altitude.pcm` | "Callout Start Altitude" |
| `volume_adjustment.pcm` | "Volume Adjustment" |
| `check_gear.pcm` | "Check gear" (descent reminder + its menu title) |
| `positive_rate.pcm` | "Positive rate" (climb callout + its menu title) |
| `off.pcm` | "Off" (the OFF choice in the check-gear / positive-rate menus) |

The converter sniffs the header, so a `.pcm` file that is secretly a renamed WAV
still converts — you can pass `assets/original_audio/*.pcm` alongside the WAVs.

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

A distinctive **calibration-error chirp** (`calibration_error.pcm`) is also generated and
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
- The **config button** (held at power-on) opens config mode, which wipes the learned
  ground (see [Configuring the unit](#configuring-the-unit)).

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
- **Volume scheduled in dB**, not linear amplitude: silent above 100 ft, then a gentle
  fade-in from a barely-audible floor at 100 ft up to full presence by 50 ft — the
  fade-in is purely so the tone doesn't appear jarringly. Below 50 ft the level is held
  **perceptually constant** all the way to the ground; it deliberately does **not** get
  louder near the ground (that would be an extra stress cue). Urgency is carried by the
  rising **pitch**, not by loudness.
- **Equal-loudness correction (ISO 226).** The ear is more sensitive to higher pitches,
  so as the tone sweeps up it would *sound* louder even at a constant electrical level.
  The firmware applies a frequency-dependent correction (from the ISO 226 ~60 phon
  equal-loudness contour) that flattens this tilt, so the perceived loudness stays steady
  through the flare while the pitch keeps climbing. Toggle with `EQUAL_LOUDNESS_CORRECTION`
  in `config.h` to A/B it on the bench.
- The tone **ducks ~4 dB** while a number plays, so numbers are never masked. No alerting
  chirp before the numbers (it slows pilot response).
- Every gain change and tone start/stop is raised-cosine ramped — no clicks, no startle.
- **Gentle stereo separation (default on).** Into a stereo panel (e.g. the GMA 245 aux)
  the firmware pans the two streams to opposite sides by a small, **equal-power** amount
  (`STEREO_PAN`, ~15% — most of each stream stays centered). The **voice leans right**
  and the **tone leans left**: the right ear has a documented advantage for processing
  speech (right ear → left, language-dominant hemisphere), so the spoken numbers get the
  speech-favoured ear while the tone takes the other — easier to parse the two at once
  without either dominating. The pan never changes a stream's loudness, and it's purely a
  separation cue (no inter-channel time delay, so callout onset stays crisp). Mono is a
  runtime option in the [config menu](#configuring-the-unit).

### Power
- WiFi/BT compiled out. In GROUND/CRUISE the tone is silent, the I2S channel is paused,
  and the MCU drops into automatic light-sleep between slow watch-polls. The instant a
  reading at/below cruise altitude appears, it spins back up to fast polling and resumes
  audio.

---

## Enclosure (3D-printed housing)

> 🚧 **Placeholder — full housing guide coming.**

The production unit ships in a **Novabox smart threaded housing**: the body has an
integral thread so it acts as its own compression fitting — you thread the housing down
to clamp it into place (no separate hardware, clean optical window for the lidar).

### Mounting — one hole, no screws
The housing is **belly- or wing-mountable** and installs with **zero fasteners**:

- **Drill a single 2.5″ hole** in the skin at your chosen mount point.
- Drop the unit in and thread it down — the **integral thread acts as a compression
  fitting**, clamping the housing to the skin against its built-in flange/lens.
- **Overall height: ~70 mm**, so confirm you have that much clearance behind the
  skin (and a clear, unobstructed view of the ground for the lidar).

That's the whole install: **one 2.5″ hole, hand-tighten, done** — no screw holes to
drill, no backing plate, no separate hardware.

### Acrylic lens
- **Ideal: a 55 mm acrylic disc** — sized to seat cleanly in the threaded housing.
- **Alternative: a standard 2″ (50.8 mm) disc** works too, sealed with a **gasket**
  to take up the difference and keep the optical window weather-tight.

This section will be filled in with:
- [ ] STL / STEP files and print settings (material, walls, infill, orientation)
- [ ] Acrylic lens spec (thickness, cut template) — 55 mm ideal / 2″ + gasket
- [ ] Exact thread pitch / torque + gasket/o-ring spec
- [ ] Mounting guidance (belly vs wing location, sensor aim, strain relief)

Print files and ready-made units are at **[Novabox.Works](https://novabox.works/)**
(complete unit **$599.95** for 100 m / SF30/C, **$699.95** for 200 m / SF30/D).

---

## Project layout

```
main/
  config.h          all profile-independent tunables (pins, dB, tone band, timing)
  sensor_profile.*  the SF30/C & SF30/D profiles (callout ladders, cruise, range)
  lwnx.*            LightWare LWNX binary protocol (build/parse + hand-rolled CRC)
  sf30c.*           UART driver, ASCII+binary parse, cm→ft, EMA, autodetect, sensor task
  robust.*          median/MAD outlier filter (pure)
  boot_buffer.*     NVS ground-reference + audio-config store, in-flight-reboot recovery, reset/config button
  state_machine.*   states, arming, edge-trigger callouts, hysteresis, poll profile (pure)
  audio_math.*      pitch map, dB schedule, gain, envelopes (pure)
  audio.*           i2s_std engine: NCO tone, dB volume, ducking, clip mixing, runtime mono/stereo pan, suspend/resume
  callouts.*        callout enum + embedded-clip manifest (graceful on missing clips); config-menu clips
  app_main.c        boot sequence (incl. the hold-at-boot config menu), task spawning, the logic loop
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

*LidarAGL is an open-source project by [Novabox.Works](https://novabox.works/). Advisory use only.*
