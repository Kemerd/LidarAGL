# LidarAGL — Firmware Implementation Spec

**Project:** LidarAGL (Novabox)
**Target:** ESP32-S3 + LightWare SF30C micro-lidar + PCM5102A I2S DAC → aircraft audio panel
**Language:** C / C++ on **ESP-IDF v5.x** (FreeRTOS). **Not** Arduino-core, **not** MicroPython.
**Aircraft context:** Glasair III (fast experimental piston single, ~95 mph over the fence, short flare window). This is an **advisory** AGL height-callout + flare-tone box. It is **not** certified equipment and must never be a substitute for visual flare judgment.

> **For the implementer (Claude Code):** Build this as a clean ESP-IDF project (`idf.py` build system, `CMakeLists.txt`, `sdkconfig.defaults`). Favor readable C/C++ with small, testable modules. All tunables go in a single `config.h` as `#define`s or `constexpr`. Include a host-buildable unit-test target for the pure-logic modules (outlier filter, state machine, tone/volume mapping) so they can be tested without hardware. Comment the *why*, not just the *what*, especially in the state machine and the audio engine.

---

## 1. System Overview

LidarAGL reads slant range from a belly-mounted SF30C, converts it to AGL, and produces two simultaneous audio outputs into an aircraft audio panel via a PCM5102A line-level DAC:

1. **Voice callouts** — spoken altitude numbers at discrete heights (200, 100, 50, 40, 30, 20, 10 ft).
2. **A continuous "presence" tone** — an *ascending*-pitch tone that swells in from barely-audible at ~100 ft to full presence by ~50 ft and through the flare band, giving a continuous "ground approaching" cue.

The system is **fully autonomous: no user calibration, no buttons, no config at runtime.** All behavior is driven by a power-aware state machine plus a boot-time sanity filter.

### Design principles (do not violate)
- **Two audio streams maximum** (tone + voice). Do **not** add a third concurrent encoded stream. Sink-rate-as-cadence was deliberately rejected.
- **Voice and tone must not collide:** duck the tone ~3–6 dB during each spoken token; never play a number at the tone's loud peak.
- **No alerting chirp before voice tokens** (it slows response — Simpson & Williams 1980). Numbers stand alone.
- **Volume is scheduled in dB (perceptual), never linear amplitude.**
- **Tone pitch ASCENDS as AGL decreases** (auditory looming bias).
- **Power saving is a first-class requirement** (see §6).

---

## 2. Hardware & Wiring

### 2.1 Bill of materials (firmware-relevant)
| Component | Role | Interface |
|---|---|---|
| ESP32-S3 (devkit or module, ≥8 MB flash, PSRAM helpful) | MCU | — |
| LightWare SF30C | Lidar rangefinder | **UART** (primary) |
| PCM5102A breakout | I2S DAC → line level | I2S |
| Audio isolation transformer 600:600 Ω | ground-loop isolation into panel | analog |

### 2.2 Pin map (all GPIO assignments live in `config.h` — these are defaults)
```
SF30C  (UART1)
  SF30C TX  -> ESP32-S3 GPIO17  (UART RX)
  SF30C RX  -> ESP32-S3 GPIO18  (UART TX)
  SF30C GND -> common GND
  SF30C V+  -> own regulated rail (NOT from the S3 3V3)

PCM5102A (I2S)
  BCK  -> GPIO5
  LRCK -> GPIO6   (a.k.a. WS / LCK)
  DIN  -> GPIO7   (I2S DOUT from S3)
  SCK  -> GND     (forces PCM5102A internal PLL; no MCLK needed)
  VIN  -> 3V3
  GND  -> common GND
  (board config pins FLT/DEMP/XSMT/FMT set by onboard jumpers: I2S format, normal latency)

Analog out
  PCM5102A LOUT/ROUT -> level trim -> 600:600 isolation transformer -> audio panel AUX/ALERT input
  (mono: tie L+R or use one channel; output AC-coupled on most breakouts)
```
**GPIO cautions for the implementer:** avoid strapping pins (0, 45, 46) and the native-USB pins (19, 20) if USB-serial-JTAG is used for logging. Any free GPIOs are fine for I2S/UART — keep the defaults above unless they conflict on the chosen devkit.

### 2.3 Power
Run the S3 + DAC off a clean regulated supply (DC-DC to 5 V then low-noise LDO to the DAC analog rail). Firmware should assume it may **brown out / reboot in flight** — see §5.

---

## 3. SF30C Interface (UART)

- **UART1**, default **115200 8N1** (make baud a `#define`; SF30C baud is configurable).
- Implement a **continuous-streaming reader**: the sensor streams distance; firmware parses the latest range.
- Support **two parse modes** behind a `#define SF30C_MODE`:
  - `SF30C_ASCII` — simple text-line distance, `atof`-style parse (use for bring-up).
  - `SF30C_BINARY` — LightWare binary protocol (flag/length/payload/checksum). Preferred for production; lets firmware set update rate at runtime.
- Sensor reads run in a dedicated **sensor task** pinned to core 0.
- Convert slant range → range in feet immediately (`#define USE_FEET 1`). All higher logic works in **feet**.
- Maintain a **latest-value** handoff to the logic task using a length-1 queue with `xQueueOverwrite` so the consumer always sees the freshest sample (no backlog lag — critical when a callout must fire on time).

### 3.1 Light filtering
Apply a light exponential moving average (EMA, `alpha` in `config.h`, default ~0.3) to the range for the *voice/state* path. The tone path may use slightly heavier smoothing to avoid pitch warble (see §7). Do **not** over-filter the state path or callouts lag.

---

## 4. Altitude / Zeroing Model (read this — it is NOT classic calibration)

The SF30C measures **slant range to whatever is under the belly**. On the ground that is just the fixed **mount height** above the surface (a small constant, e.g. ~2–3 ft). It does not drift. Therefore:

- **There is no runtime calibration and none is wanted.** AGL ≈ measured range − mount offset. `MOUNT_OFFSET_FT` is a `#define` (default e.g. 2.5 ft) the builder sets once for their install. Firmware does **not** try to "learn" it during normal operation.
- The boot buffer (§5) is **not** a ground-reference calibration. It is a **reboot sanity filter** so the box recovers gracefully from an **in-flight reboot** instead of waking up assuming it's on the ground.

---

## 5. Boot-Reading Sanity Buffer (in-flight-reboot protection)

**Purpose:** If the box reboots *in flight* (brownout, power glitch, manual cycle), it must not wake up in GROUND state and do something dumb (e.g. arm/fire callouts wrongly, or sit dead because it thinks it's parked). The boot buffer lets it reconstruct roughly where it is and resume in the correct state.

### 5.1 Storage
- Use **ESP-IDF NVS** (key-value, wear-leveled).
- Store a **rolling buffer of the last N boot readings** (`#define BOOT_BUFFER_N 10`). Each entry: `{ float agl_ft; uint32_t epoch_or_uptime_marker; }` (timestamp optional — uptime/boot counter is fine; no RTC required).
- **Exactly ONE NVS write per boot** (write the new entry once, early, after the first clean reading is obtained). **No continuous writes.** This keeps flash wear negligible.

### 5.2 Boot sequence
1. On power-up, configure clocks/peripherals, **disable WiFi & BT** (§8), start sensor task.
2. Acquire the **first clean live reading(s)** from the SF30C (median of the first few valid samples; reject obvious garbage immediately).
3. Read back the stored `BOOT_BUFFER_N` previous boot readings from NVS.
4. **Filter** the combined set {stored readings + current reading} (§5.3).
5. From the surviving values, decide the **initial state** (§6.2): if the recent/current readings indicate we are high (e.g. > ARM_FT), boot directly into a flying state (ARMED/CRUISE) rather than GROUND. If they indicate near-ground and stable, boot into GROUND.
6. **Append the current reading** to the rolling buffer and write NVS **once**.

### 5.3 Outlier filter (fast, integer/float C — no heavy stats libs)
Given the small set (≤11 values), use a robust, cheap method:
1. **Hard rejects first:** discard `inf`, `NaN`, negatives, and anything **> MAX_VALID_FT** (`#define MAX_VALID_FT 50.0f` — readings above this at *boot* are treated as not-useful-for-zeroing junk per the user's spec; note this cap is for the **boot sanity filter only**, NOT for in-flight ranging, which must read full range to ≥300 ft).
2. **Robust outlier test on survivors:** compute the **median** and the **MAD** (median absolute deviation). Reject any value where `|x − median| > MAD_K * MAD` (`#define MAD_K 3.0f`). MAD is preferred over mean/stdev because it is robust to the very fliers we're removing and is cheap for small N (just two median passes). If MAD == 0 (all equal), skip the MAD test.
3. **Average the survivors** (mean of the inliers) and blend with the current reading to form the working estimate used for the initial-state decision.

> Implementer: write this as a pure function `float robust_estimate(const float* vals, size_t n, float current, bool* ok)` with a host unit test. Keep it allocation-free.

---

## 6. State Machine (the brain — power + behavior)

All thresholds in `config.h`. Heights in **feet AGL**.

### 6.1 Thresholds (defaults)
```
ARM_FT          100.0   // must climb through this before any descent callouts are armed
CRUISE_FT       300.0   // at/above this, polling slows or sleeps
TONE_START_FT   100.0   // tone becomes (barely) audible here
TONE_FULL_FT     50.0   // tone reaches full presence by here (and stays full below)
FLARE_BAND_HI    35.0   // top of flare "full attention" band
FLARE_BAND_LO    20.0   // bottom of flare band
MAX_VALID_FT     50.0   // boot-sanity junk cap ONLY (see 5.3)
CALLOUT_FT[]   {200,100,50,40,30,20,10}
REARM_MARGIN_FT  20.0   // hysteresis: re-arm a callout once you climb this far back above it
```

### 6.2 States
```
GROUND      — on the ground, stable low reading. Polling SLOW. No callouts. No tone.
CLIMB       — reading increasing from ground; callouts NOT armed yet.
ARMED       — aircraft has climbed through ARM_FT (100 ft). Descent callouts now enabled.
CRUISE      — at/above CRUISE_FT (300 ft). Polling SLOWEST / sensor low-power. Still "armed" in principle.
DESCENT     — descending while ARMED/CRUISE: fast polling, fire callouts + run tone.
FLARE       — within/below flare band: tone at full presence (handled by audio engine, not a hard state jump).
```

### 6.3 Transition rules (the important behavior)
- **No takeoff callouts.** From GROUND → CLIMB as range rises, but callouts stay **disarmed** until range exceeds **ARM_FT (100 ft)**, at which point → **ARMED**. This guarantees the climb-out after takeoff is silent.
- Once **ARMED**, descent through each `CALLOUT_FT` fires that callout **once** (edge-trigger on the way down).
- **Hysteresis / re-arm:** each callout is a one-shot on descent; it re-arms only after the aircraft climbs back above `callout_height + REARM_MARGIN_FT`. This is what lets you do a go-around and get the callouts again on the next descent, without machine-gunning when hovering near a threshold.
- **≥ CRUISE_FT (300 ft):** enter low-power polling (§6.4). The moment a reading at/below ~300 ft is seen again (descent beginning), ramp polling back up and resume DESCENT behavior.
- **Climb/descent direction** is derived purely from the **range trend** (sign of smoothed dRange/dt), since there is no IMU/baro. Use a small dead-band so noise doesn't flip direction.

### 6.4 Polling / power policy (tie to §8)
| State | Sensor poll rate | MCU |
|---|---|---|
| GROUND | very slow (e.g. 1–2 Hz) or sensor idle, MCU light-sleep between polls | light-sleep allowed |
| CLIMB | moderate (e.g. 10 Hz) | active |
| ARMED (below cruise) | moderate–fast | active |
| CRUISE (≥300 ft) | slowest, or sensor placed in its low-power/standby; MCU light-sleep between checks | light-sleep allowed |
| DESCENT / near flare | **fast** (e.g. 20–50 Hz) for crisp callout timing | active, no sleep |

- "Slow/off above 300 ft": acceptable to drop to a low watch-rate (e.g. 1–2 Hz) just enough to notice descent crossing back below 300, OR use the SF30C's own low-power mode if exposed. As soon as one reading at/below CRUISE_FT appears, immediately spin back up to fast polling.
- Implement poll-rate changes as a single function `set_poll_profile(state)` so the policy is in one place.

---

## 7. Audio Engine (PCM5102A via I2S)

Runs as a dedicated **audio task** on core 1, owning the I2S peripheral. It continuously generates the output buffer; the logic task feeds it (a) the current AGL (for tone pitch + volume) and (b) discrete callout requests (queue of token IDs).

### 7.1 I2S setup
- ESP-IDF v5 **`i2s_std`** driver, TX channel, 16-bit, mono (or stereo with duplicated mono), sample rate `#define SAMPLE_RATE 16000` (16 kHz is plenty for voice + tone and saves storage).
- PCM5102A uses internal PLL (SCK→GND), so **no MCLK output required** from the S3.
- Keep one fixed sample rate for everything so the channel is never reconfigured mid-flight (no clicks).

### 7.2 Presence tone — generation
- **NCO (numerically controlled oscillator):** phase accumulator + sine lookup table. `sample = amp * sin(phase); phase += 2π * f / SAMPLE_RATE;`
- **Pitch mapping (ASCENDING as AGL decreases):**
  - Active range from `TONE_START_FT` (100 ft) down to touchdown.
  - `f` rises as AGL falls. Defaults: `F_AT_TONE_START ≈ 600 Hz` at 100 ft → `F_AT_GROUND ≈ 1800 Hz` near 0 ft. Keep energy in **500–3000 Hz** (cuts cockpit noise, survives ANR headsets, ear most sensitive there).
  - Map smoothly (linear in Hz, or linear in log-f for a more musical glide — make it a `#define TONE_LOG_SWEEP`).
- **Sine, not square** (square is harsh/fatiguing). Use a reasonably sized LUT (e.g. 1024 entries) with interpolation.
- **Smooth the pitch:** drive the tone from a more heavily smoothed AGL than the callouts use, and/or slew-limit `f`, so lidar jitter doesn't make the pitch warble. A little lag here is fine — trend matters, not absolute precision.

### 7.3 Presence tone — volume (PERCEPTUAL, dB-scheduled)
- **Do NOT ramp linear amplitude.** Schedule the **level in dB** vs AGL, then convert to a linear gain (`gain = 10^(dB/20)`).
- Curve:
  - Above `TONE_START_FT` (100 ft): tone **off / silent**.
  - At 100 ft: fade in at a **barely-audible** floor (a few dB above expected masked threshold).
  - 100 → 50 ft: ramp **level in dB** smoothly up so it reaches **full presence by `TONE_FULL_FT` (50 ft)**.
  - 50 ft → flare band → ground: **hold full presence.**
- Expose `TONE_FLOOR_DB` and `TONE_FULL_DB` as `#define`s (relative to full-scale output). Builder calibrates absolute level via the analog trim pot to suit their panel/headset; firmware just provides a clean perceptual ramp shape. Rule of thumb to document: **+10 dB ≈ perceived "twice as loud."**
- **Onset/offset envelopes:** apply ≥30–50 ms raised-cosine ramps to any gain change and to tone start/stop. **Never** hard-gate (avoids clicks and avoids any startle from abrupt onset). Since the tone is already present and merely swelling, startle risk is inherently low — keep it that way.

### 7.4 Voice callouts
- Pre-stored clips: "two hundred", "one hundred", "fifty", "forty", "thirty", "twenty", "ten". Store as **raw 16 kHz mono PCM** (strip WAV headers) in flash (embed via `EMBED_FILES` in CMake, or a SPIFFS/LittleFS partition). Keep an enum `callout_id_t`.
- Audio task pends on `calloutQueue`; on receiving an ID, mixes/sequences that clip into the output.
- **Tone ducking:** while a voice clip plays, attenuate the tone by `#define VOICE_DUCK_DB 4.0f` (≈3–6 dB) and restore with a smooth ramp after. Numbers must never be masked by the tone peak.
- **No pre-token alert chirp.**
- Callouts are fired by the **logic task** (edge-trigger + hysteresis, §6.3), not by the audio task. Audio task just renders what it's told.

### 7.5 Mixing
- Final output each frame = `clip_sample (if playing) + tone_sample * tone_gain * duck_gain`, soft-clipped/limited to avoid overflow. Keep headroom; document the limiter.

---

## 8. WiFi / Bluetooth — DISABLED

- **Disable both** at the project level and at runtime:
  - In `sdkconfig.defaults`: do not start WiFi/BT; where practical disable the stacks in menuconfig so they aren't initialized.
  - At boot, ensure radios are off (`esp_wifi_stop()`/deinit not started; controller not enabled). Net effect: radios never powered → no draw, no EMI into the audio/avionics.
- Document a `README` note that these can be re-enabled later (future telemetry/config) but are off for now by design.

---

## 9. Task / Concurrency Architecture (FreeRTOS)

```
Core 0:
  sensor_task   — UART read + parse + EMA -> xQueueOverwrite(latest range)
Core 1:
  logic_task    — peek latest range; run state machine; derive trend;
                  fire callouts (edge+hysteresis); set poll profile;
                  publish AGL + tone params to audio engine
  audio_task    — owns i2s_std; NCO tone + clip playback + ducking + mixing
```
- Use a length-1 overwrite queue for range, a small queue for callout IDs, and a lightweight shared struct (mutex or atomics) for "current AGL / tone params."
- `logic_task` decision rate ~50 Hz in active states; slower in GROUND/CRUISE to allow sleep.
- Keep all timing/threshold/pin constants in `config.h`.

---

## 10. Deliverables (what to produce)

1. ESP-IDF project skeleton: `CMakeLists.txt`, `sdkconfig.defaults`, `main/` with modules:
   - `config.h` — all tunables (pins, thresholds, freqs, dB levels, baud, sample rate).
   - `sf30c.c/.h` — UART driver + ASCII/binary parse, EMA.
   - `boot_buffer.c/.h` — NVS rolling buffer, robust_estimate(), one-write-per-boot.
   - `state_machine.c/.h` — states, transitions, trend detection, hysteresis, poll profiles.
   - `audio.c/.h` — i2s_std setup, NCO tone, dB-volume schedule, clip playback, ducking, mixing.
   - `callouts/` — embedded PCM clips + manifest.
   - `app_main.c` — init, disable radios, spawn tasks.
2. **Host-buildable unit tests** (plain C, no hardware) for: `robust_estimate` (outlier filter), state-machine transitions + hysteresis, AGL→pitch and AGL→dB→gain mappings.
3. A short `WIRING.md` (can reuse §2) and a `README` stub (builder fills in later).
4. Inline comments explaining the *why* for: ascending-pitch choice, dB-scheduled volume, no-pre-chirp, edge-trigger+hysteresis, one-write-per-boot, no-takeoff-callout arming.

---

## 11. Tunables Quick-Reference (put in `config.h`)
```c
// ---- Pins ----
#define PIN_SF30C_RX   17
#define PIN_SF30C_TX   18
#define PIN_I2S_BCK     5
#define PIN_I2S_LRCK    6
#define PIN_I2S_DIN     7

// ---- SF30C ----
#define SF30C_BAUD      115200
#define SF30C_MODE      SF30C_ASCII   // or SF30C_BINARY
#define USE_FEET        1
#define RANGE_EMA_ALPHA 0.30f
#define MOUNT_OFFSET_FT 2.5f          // builder sets for their install

// ---- Boot sanity buffer ----
#define BOOT_BUFFER_N   10
#define MAX_VALID_FT    50.0f         // boot-sanity junk cap ONLY
#define MAD_K           3.0f

// ---- State machine (ft AGL) ----
#define ARM_FT          100.0f
#define CRUISE_FT       300.0f
#define REARM_MARGIN_FT  20.0f
// CALLOUT_FT[] = {200,100,50,40,30,20,10}

// ---- Tone ----
#define SAMPLE_RATE     16000
#define TONE_START_FT   100.0f
#define TONE_FULL_FT     50.0f
#define FLARE_BAND_HI    35.0f
#define FLARE_BAND_LO    20.0f
#define F_AT_TONE_START 600.0f        // Hz at 100 ft (low/quiet)
#define F_AT_GROUND    1800.0f        // Hz near 0 ft (high) -- ASCENDING
#define TONE_LOG_SWEEP   1
#define TONE_FLOOR_DB  -40.0f         // barely audible (rel. full scale)
#define TONE_FULL_DB    -6.0f         // full presence (rel. full scale)
#define GAIN_RAMP_MS     40           // raised-cosine envelope time
#define VOICE_DUCK_DB    4.0f
```

---

## 12. Notes / Guardrails for the implementer
- This is **advisory** firmware for an experimental aircraft; keep the audio path simple, deterministic, and click-free. A wrong/late callout is worse than silence (SYNCALL finding) — prioritize correct, on-time numbers over features.
- Lidar can lose returns over water / very dark or wet surfaces — handle "no valid return" gracefully (hold last good, don't emit garbage callouts, don't crash the tone).
- Keep everything in **feet** internally for consistency with the spec; clearly mark the one place range is converted.
- Don't introduce continuous NVS writes anywhere — one write per boot is the whole persistence contract.
- All magic numbers belong in `config.h`. No hard-coded thresholds scattered in logic.
