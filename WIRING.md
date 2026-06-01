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
                                                                           ▼
                                                            600:600 Ω isolation xfmr
                                                                           │
                                                                           ▼
                                                            Aircraft audio panel
                                                            (AUX / ALERT input)
```

Two analog reasons for the isolation transformer: it breaks the ground loop between
the box and the panel, and it keeps any DAC/MCU switching noise off the avionics bus.

---

## 2. Pin map (defaults — `config.h`)

### SF30 lidar → ESP32-S3 (UART1, 115200 8N1)

| SF30 pin | → | ESP32-S3 | config.h |
|----------|---|----------|----------|
| SF30 **TX** | → | GPIO **17** (UART RX) | `PIN_SF30C_RX` |
| SF30 **RX** | → | GPIO **18** (UART TX) | `PIN_SF30C_TX` |
| SF30 **GND** | → | common GND | — |
| SF30 **V+** | → | **own regulated rail** (NOT the S3 3V3) | — |

> The SF30 draws real current and is noisy on its supply — give it its own
> regulated rail and only share **ground** with the S3. UART lines are 3.3 V TTL
> (5 V tolerant on the SF30 side).

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
> Held at power-on → the box wipes the stored ground readings and reboots.
> Use it after an install / re-install so the ground reference re-learns fresh.

### Analog output

```
PCM5102A LOUT/ROUT ─► level-trim pot ─► 600:600 Ω isolation transformer ─► panel AUX/ALERT
```

- Mono: tie L+R, or use a single channel. Most breakouts AC-couple the output.
- Set absolute loudness with the **trim pot** — the firmware only shapes the
  perceptual ramp (the tone fades in by dB; the pot sets the master level).

---

## 3. Power

- Feed the S3 + DAC from a **clean regulated supply**: DC-DC down to 5 V, then a
  **low-noise LDO** to the DAC analog rail.
- The firmware assumes it may **brown out / reboot in flight**. That's handled by
  the in-flight-reboot recovery in `boot_buffer.c` (it reconstructs roughly where
  you are instead of waking up assuming it's parked).

---

## 4. GPIO cautions

- Avoid the strapping pins (**0, 45, 46**) and the native-USB pins (**19, 20**) —
  logging uses the USB-Serial-JTAG console.
- Any other free GPIOs are fine for I2S/UART/button; keep the defaults above
  unless they conflict on your specific devkit, and change them in `config.h`.
