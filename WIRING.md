# LidarAGL — Wiring

> Advisory AGL callout box. **Not certified.** Never a substitute for visual flare judgment.

This is the physical wiring for the ESP32-S3 + LightWare SF30/C (or /D) + PCM5102A
DAC build. All GPIO numbers are defaults defined in `main/config.h` — change them
there if your devkit conflicts, don't hand-edit the wiring assumptions in code.

---

## 1. Block diagram

```
  SF30/C or SF30/D ──UART (3.3V TTL)──► ESP32-S3 ──I2S──► PCM5102A ── L / R / LO ──►
   (belly lidar)                         (MCU)             (DAC)            │
                                                                           │
                                                                           ▼
                                                            GMA 245 audio panel
                                                            (stereo aux input)
```

The reference install feeds a **GMA 245** stereo aux input, which accepts a line-level
source directly and provides its own **audio LO** (signal-ground) reference — so the
standalone isolation transformer isn't required. Referencing the audio return to the
panel's LO (not power ground) keeps DAC/MCU switching noise off the audio. If you ever
hear supply-correlated whine, add a small inline aux ground-loop isolator on L/R.

---

## 2. Pin map (defaults — `config.h`)

### SF30 lidar → ESP32-S3 (UART1, 460800 8N1)

> **Baud must match `SF30C_BAUD` in `config.h` (currently 460800).** Set the
> Serial port baud to the same value in LightWare Studio, with Output type set to
> **"Distance over Serial"** — the SF30/C's legacy 2-byte stream, not the LWNX
> binary protocol. A mismatch here reads as a dead sensor: the box chirps at
> boot and stays silent.

The SF30/C exits on a **7-conductor cable**; the wire **colors are fixed by
LightWare** (Table 4 of the SF30/C manual). We only use four of them — TXD, RXD,
GND, VIN — and **cap off the other three** (Alarm, Sync, Analog) unused.

| SF30 pin | Wire color | SF30 function | → | ESP32-S3 | config.h |
|----------|-----------|---------------|---|----------|----------|
| **3** | 🟡 Yellow | **TXD** (serial transmit) | → | GPIO **8** (UART RX) | `PIN_SF30C_RX` |
| **4** | 🟠 Orange | **RXD** (serial receive) | → | GPIO **9** (UART TX) | `PIN_SF30C_TX` |
| **6** | ⚫ Black | **GND** (supply −, power/logic) | → | common GND | — |
| **7** | 🔴 Red | **VIN** (+5 V supply +) | → | shared clean 5 V rail | — |
| 1 | 🟢 Green | Alarm output | — | *unused — cap off* | — |
| 2 | ⚪ White | Sync output | — | *unused — cap off* | — |
| 5 | 🔵 Blue | Analog | — | *unused — cap off* | — |

> **Cross TX↔RX:** the sensor's **TXD (yellow)** lands on the S3's **RX** and the
> sensor's **RXD (orange)** lands on the S3's **TX** — receive listens to the
> other end's transmit. UART lines are 3.3 V TTL (5 V tolerant on the SF30 side).
>
> One clean 5 V rail feeds everything (a separate sensor rail isn't practical in
> the panel). The SF30 draws real current and is noisy on its supply, so add a
> **local LC/ferrite + bulk cap** at the sensor to keep its hash off the shared
> rail.

> ### ⚠️ Land the sensor on the pads numbered **`8`** and **`9`** — NOT the pins marked `TX`/`RX`
>
> This one cost real bench hours. The firmware reads the SF30 on **UART1 (GPIO 8 / 9)**.
> The header pins silk-screened **`TX`/`RX`** on most ESP32-S3 boards are a *different*
> UART — **UART0 / the USB-console (GPIO 43/44)** — which the firmware does **not** use
> for the sensor. Wire the SF30 to `TX`/`RX` and GPIO 8 sits floating: you'll get
> `raw UART1 drain: n=0` (zero bytes) plus occasional random "readings" from line noise.
>
> - 🟡 **Yellow (sensor TXD)** → the pad printed **`8`** / `IO8`  (the S3's RX)
> - 🟠 **Orange (sensor RXD)** → the pad printed **`9`** / `IO9`  (the S3's TX)
>
> If you must use other pads, change `PIN_SF30C_RX` / `PIN_SF30C_TX` in `config.h` to
> match — but never use the USB-Serial-JTAG pins (19/20) or the strapping pins.

### PCM5102A DAC → ESP32-S3 (I2S)

| PCM5102A pin | → | ESP32-S3 | config.h |
|--------------|---|----------|----------|
| **BCK** | → | GPIO **5** | `PIN_I2S_BCK` |
| **LRCK** (a.k.a. WS / LCK) | → | GPIO **6** | `PIN_I2S_LRCK` |
| **DIN** | → | GPIO **7** (I2S DOUT from S3) | `PIN_I2S_DIN` |
| **SCK** | → | **GND** | — |
| **VIN** | → | 3V3 | — |
| **GND** | → | common GND | — |

> **SCK → GND is important.** It forces the PCM5102A to run from its internal
> PLL, so the S3 emits **no MCLK** (firmware sets `mclk = I2S_GPIO_UNUSED`).
>
> **XSMT must be pulled HIGH or the DAC stays MUTED.** This bites hard: the
> common purple **GY-PCM5102A** clone ships with XSMT **low (muted)**, so you get
> perfect BCK/LRCK/DIN clocks + data and *dead silence* — every wire looks right.
> XSMT is an active-low soft-mute with an internal pull-DOWN, so it needs an
> external pull-up. On the back of the board the four solder jumpers are
> `H1L..H4L` = **FLT / DEMP / XSMT / FMT** (High/Low select). Fix it by bridging
> **jumper 3 (XSMT) to H**, *or* wiring the **XSMT pin ("3") → A3V3** (the analog
> 3.3 V pin between pin 4 and G). Verify with a meter: **XSMT should read ~3.3 V**
> (a stuck ~0.5 V reading = still muted). The other three jumpers (FLT / DEMP /
> FMT) are correct at their **Low** defaults — that's I2S format, normal latency.

### Config button (audio modes + wipe learned ground reference)

| Button | → | ESP32-S3 | config.h |
|--------|---|----------|----------|
| momentary, one side | → | GPIO **4** | `PIN_CONFIG_BTN` |
| other side | → | GND | — |

> Active-low with the S3's internal pull-up (no external resistor needed).
> Held at power-on → the box enters the **config menu**: it wipes the stored ground
> readings *and* the saved audio config, then a single **tap** cycles the option and
> a **double-tap** (or ~5 s of silence) confirms — first the audio mode (mono/stereo,
> callouts/tone), then the callout start altitude. It reboots on commit. Use it after
> an install / re-install so the ground reference re-learns fresh. In the aircraft the
> button is **cockpit-mounted** on harness conductor 6.

### Analog output → GMA 245 (stereo aux)

The PCM5102A's analog output is the 4-pin header silkscreened
**`LOUT · AGND · ROUT · AGND`**. The two **AGND** pins are the **same net** (the
DAC's analog ground) — they're doubled up only so each channel has a return pin
next to it. This AGND is the **audio return**, and it is a *separate net* from the
digital `GND` / `DGND` on the I2S side of the board.

```
PCM5102A LOUT ───────────────────► GMA 245 aux L      (conductor 3)
PCM5102A ROUT ───────────────────► GMA 245 aux R      (conductor 4)
PCM5102A AGND ─┐  (both AGND pins,
PCM5102A AGND ─┴── same net) ─────► GMA 245 audio LO  (conductor 5)
```

- **Line level, no trim pot** — the DAC puts out ~2.1 Vrms (TI datasheet typ.),
  which is line level, so it goes straight to the GMA 245 aux, which sets the
  volume. The firmware only shapes the perceptual dB ramp.
- **Stereo by default** — firmware emits L/R and gently pans the streams apart
  (voice right, tone left). Mono/stereo is a **runtime** choice from the config
  menu (hold the button at power-on); the I2S hardware always drives both
  channels, so mono just duplicates the signal to L and R (safe if wired mono).
- Land L and R against the GMA's **audio LO** reference (not power ground) — that
  isolation is what replaces the old transformer. **Tie the audio LO to BOTH
  AGND pins** on the `LOUT/ROUT` header; do **not** jump AGND to the board's
  digital `GND`/`DGND` — bridging analog return to power ground reintroduces the
  ground loop (the supply-whine path) this isolation exists to avoid.

### Cockpit harness (6-conductor, shielded)

| # | Conductor | From → to |
|---|-----------|-----------|
| 1 | **+** (5 V) | clean 5 V rail → box |
| 2 | **−** (GND) | power ground → box |
| 3 | **L** | DAC LOUT → GMA 245 aux L (line level) |
| 4 | **R** | DAC ROUT → GMA 245 aux R (line level) |
| 5 | **LO** | GMA 245 audio-LO ref ← box audio return |
| 6 | **BTN** | cockpit config button → GPIO 4 |

> Bond the cable **shield at one end only — the panel side** — so it drains noise
> without forming a ground loop. The button on conductor 6 is the config button,
> just relocated to the cockpit (active-low to box GND).

---

## 3. Power

- One **clean regulated 5 V rail** feeds the S3, DAC, and SF30 over the harness
  **+ / −** conductors. Isolate noise rather than supplies: a **low-noise LDO** for
  the DAC analog rail, a **local LC/ferrite + bulk cap** at the SF30, and keep the
  audio return on the panel's **audio LO** (not power ground).
- The firmware assumes it may **brown out / reboot in flight**. That's handled by
  the in-flight-reboot recovery in `boot_buffer.c` (it reconstructs roughly where
  you are instead of waking up assuming it's parked).

---

## 4. GPIO cautions

- **Mini ESP32-S3 board:** only GPIO **1–13** are broken out (plus 5V/GND/3V3 and
  USB TX/RX). All defaults fit: UART **8/9**, I2S **5/6/7**, config button **4**.
- Avoid the strapping pins (**0, 3, 45, 46**) and the native-USB pins (**19, 20**) —
  logging uses the USB-Serial-JTAG console.
- Any other free GPIOs are fine for I2S/UART/button; keep the defaults above
  unless they conflict on your specific devkit, and change them in `config.h`.
