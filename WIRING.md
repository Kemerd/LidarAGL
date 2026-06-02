# LidarAGL — Wiring

> Advisory AGL callout box. **Not certified.** Never a substitute for visual flare judgment.

This is the physical wiring for the ESP32-S3 + LightWare SF30/C (or /D) + PCM5102A
DAC build. All GPIO numbers are defaults defined in `main/config.h` — change them
there if your devkit conflicts, don't hand-edit the wiring assumptions in code.

---

## 1. Block diagram

```
  SF30/C or SF30/D ──UART (3.3V TTL)──► ESP32-S3 ──I2S──► PCM5102A ──► level trim
   (belly lidar)                         (MCU)             (DAC)            │
                                                                      L / R / LO
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

### SF30 lidar → ESP32-S3 (UART1, 115200 8N1)

| SF30 pin | → | ESP32-S3 | config.h |
|----------|---|----------|----------|
| SF30 **TX** | → | GPIO **17** (UART RX) | `PIN_SF30C_RX` |
| SF30 **RX** | → | GPIO **18** (UART TX) | `PIN_SF30C_TX` |
| SF30 **GND** | → | common GND | — |
| SF30 **V+** | → | shared clean 5 V rail | — |

> One clean 5 V rail feeds everything (a separate sensor rail isn't practical in
> the panel). The SF30 draws real current and is noisy on its supply, so add a
> **local LC/ferrite + bulk cap** at the sensor to keep its hash off the shared
> rail. UART lines are 3.3 V TTL (5 V tolerant on the SF30 side).

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
> Onboard jumpers FLT / DEMP / XSMT / FMT: leave at the board defaults
> (I2S format, normal latency).

### Reset button (wipe learned ground reference)

| Button | → | ESP32-S3 | config.h |
|--------|---|----------|----------|
| momentary, one side | → | GPIO **4** | `PIN_RESET_BTN` |
| other side | → | GND | — |

> Active-low with the S3's internal pull-up (no external resistor needed).
> Held at power-on → the box enters the **boot config menu**: it wipes the stored
> ground readings *and* the saved audio config, then you tap to pick an audio mode
> (mono/stereo, callouts/tone). It commits ~5 s after your last tap and reboots.
> Use it after an install / re-install so the ground reference re-learns fresh.
> In the aircraft the button is **cockpit-mounted** on harness conductor 6.

### Analog output → GMA 245 (stereo aux)

```
PCM5102A LOUT ─► trim pot ─┐
PCM5102A ROUT ─► trim pot ─┼─► GMA 245 aux  (L, R, audio-LO ref)
              panel LO  ◄──┘
```

- **Stereo by default** — firmware emits L/R and gently pans the streams apart
  (voice right, tone left). Mono/stereo is a **runtime** choice from the boot
  config menu (hold the button at power-on); the I2S hardware always drives both
  channels, so mono just duplicates the signal to L and R (safe if wired mono).
- Land L and R against the GMA's **audio LO** reference (not power ground) — that
  isolation is what replaces the old transformer.
- Set absolute loudness with the **trim pot** — the firmware only shapes the
  perceptual ramp (the tone fades in by dB; the pot sets the master level).

### Cockpit harness (6-conductor, shielded)

| # | Conductor | From → to |
|---|-----------|-----------|
| 1 | **+** (5 V) | clean 5 V rail → box |
| 2 | **−** (GND) | power ground → box |
| 3 | **L** | DAC LOUT (via trim) → GMA 245 aux L |
| 4 | **R** | DAC ROUT (via trim) → GMA 245 aux R |
| 5 | **LO** | GMA 245 audio-LO ref ← box audio return |
| 6 | **BTN** | cockpit reset button → GPIO 4 |

> Bond the cable **shield at one end only — the panel side** — so it drains noise
> without forming a ground loop. The button on conductor 6 is the same reset /
> re-learn button, just relocated to the cockpit (active-low to box GND).

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

- Avoid the strapping pins (**0, 45, 46**) and the native-USB pins (**19, 20**) —
  logging uses the USB-Serial-JTAG console.
- Any other free GPIOs are fine for I2S/UART/button; keep the defaults above
  unless they conflict on your specific devkit, and change them in `config.h`.
