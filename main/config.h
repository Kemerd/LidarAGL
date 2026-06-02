/**
 * @file    config.h
 * @brief   Single source of truth for every LidarAGL tunable.
 *
 * @details Per the spec, ALL pins, thresholds, frequencies, dB levels, baud
 *          rates and timing live here as `#define`s / `constexpr`-style macros.
 *          No magic numbers are scattered through the logic modules.
 *
 *          The file is split into two clearly fenced regions:
 *
 *            1. PURE-LOGIC-SAFE CONSTANTS  — plain numeric `#define`s used by the
 *               host-testable modules (robust, state_machine, audio_math, lwnx,
 *               sensor_profile). These pull in NO ESP-IDF headers, so the test/
 *               build can `#include "config.h"` on a desktop compiler.
 *
 *            2. HARDWARE-ONLY CONSTANTS    — GPIO numbers and peripheral knobs.
 *               These are only meaningful in the firmware build and are guarded
 *               so the host test build (which defines UNIT_TEST) skips them.
 *
 *          Per-sensor values (callout list, cruise altitude, max range) do NOT
 *          live here — they belong to the active `sensor_profile_t` so the box
 *          can adapt to whichever LightWare unit is fitted. See sensor_profile.h.
 */
#ifndef LIDARAGL_CONFIG_H
#define LIDARAGL_CONFIG_H

/* ===========================================================================
 *  REGION 1 — PURE-LOGIC-SAFE CONSTANTS  (no ESP headers; host-test friendly)
 * ===========================================================================*/

/* ---- Sensor model identifiers ------------------------------------------- */
/*  Mirror of the enum in sensor_profile.h, expressed as plain ints so this
 *  header has zero dependencies. The autodetect falls back to whichever model
 *  DEFAULT_SENSOR_MODEL names when it cannot positively identify the sensor.   */
#define SENSOR_MODEL_SF30C   0
#define SENSOR_MODEL_SF30D   1
#define DEFAULT_SENSOR_MODEL SENSOR_MODEL_SF30C   /* SF30/C is the default unit */

/* ---- SF30 serial / parse ------------------------------------------------- */
#define SF30C_BAUD        115200      /* LightWare default; 8N1                 */
#define SF30C_ASCII       0           /* legacy 2-byte stream — bring-up        */
#define SF30C_BINARY      1           /* LWNX framed protocol — production      */
#define SF30C_MODE        SF30C_BINARY
#define USE_FEET          1           /* all higher logic works in feet         */

/*  Conversion factor applied in exactly ONE place (sf30c.c). The sensor reports
 *  centimetres on both the ASCII and the LWNX path.                            */
#define CM_TO_FT          0.0328084f

/*  Lost-signal sentinel the SF30 emits when it gets no return (e.g. over water
 *  or a very dark/wet surface). Value is in centimetres on both paths.         */
#define SF30_LOST_SIGNAL_CM   16000

/* ---- Range filtering ----------------------------------------------------- */
/*  Light EMA on the state/voice path so callouts stay crisp; a heavier EMA on
 *  the tone path so lidar jitter doesn't make the pitch warble.                */
#define RANGE_EMA_ALPHA   0.30f       /* state/voice path (responsive)          */
#define TONE_EMA_ALPHA    0.12f       /* tone path (smoother, a little laggy)   */

/* ---- Ground reference / boot buffer (see boot_buffer.c, spec §5) --------- */
/*  The buffer stores the last N on-ground readings; their mean is the learned
 *  ground/mount offset. AGL = measured_range - ground_ref.                     */
#define BOOT_BUFFER_N            10    /* stored ground readings per boot        */
#define GROUND_FILL_SAMPLES      100   /* raw samples taken to pick the N from   */
#define GROUND_FILL_MS           1000  /* spread the fill over ~1 second         */

/*  A fresh reading more than GROUND_DEV_FT above the learned ground mean is
 *  treated as AIRBORNE: it does NOT update the ground reference and is used
 *  directly to compute AGL. A lidar does not drift this much sitting on the
 *  ground, so anything beyond it cannot be the ground.                         */
#define GROUND_DEV_FT            10.0f

/*  Hard junk cap used only while selecting ground-fill samples: a *ground*
 *  reading above this is obvious garbage and is dropped before averaging. This
 *  is NOT an in-flight range limit (in flight we read the sensor's full range). */
#define MAX_VALID_FT             50.0f

/*  Robust (MAD) outlier rejection strength for the ground-fill set.            */
#define MAD_K                    3.0f

/*  Emergency ground offset when no usable reference can be established
 *  (empty buffer AND the first reading looks airborne). The box proceeds on
 *  this value and chirps a calibration-error tone so the pilot is aware.       */
#define MOUNT_OFFSET_FALLBACK_FT 3.0f

/* ---- State machine (ft AGL) --------------------------------------------- */
/*  ARM_FT is intentionally global (same for both sensors): the climb-out after
 *  takeoff is silent until the aircraft has climbed through it, which arms the
 *  descent callouts. CRUISE_FT and the callout list are per-PROFILE, not here. */
#define ARM_FT            100.0f       /* arm descent callouts above this        */
#define REARM_MARGIN_FT   20.0f        /* climb this far back above a callout    */
                                       /* height to re-arm it (go-around).       */

/*  Direction (climb vs descent) comes purely from the smoothed range trend
 *  since there is no IMU/baro. A small dead-band stops noise flipping the sign. */
#define TREND_DEADBAND_FPS 0.5f        /* |dAGL/dt| below this == "level"        */

/* ---- Per-state poll cadence (ms between sensor reads) -------------------- */
/*  One place defines the power/latency policy; set_poll_profile() maps state
 *  to one of these. Fast in DESCENT for crisp callout timing; slow (sleep-
 *  friendly) in GROUND/CRUISE.                                                  */
#define POLL_MS_GROUND    750         /* ~1.3 Hz watch rate, light-sleep         */
#define POLL_MS_CLIMB     100         /* ~10 Hz                                  */
#define POLL_MS_ARMED     50          /* ~20 Hz                                  */
#define POLL_MS_CRUISE    500         /* ~2 Hz watch for descent, light-sleep    */
#define POLL_MS_DESCENT   25          /* ~40 Hz, crisp callouts                  */

/* ---- Audio: sample rate & tone band (fixed for both sensors) ------------- */
/*  The flare physics of the Glasair III do not change with which sensor is
 *  fitted, so the tone swell band is fixed here; only the high-altitude callout
 *  ceiling differs per profile.                                                */
#define SAMPLE_RATE       16000       /* plenty for voice + tone, saves flash    */
#define AUDIO_FRAME_LEN   128         /* samples generated per render block      */

/* ---- Audio: channel layout & runtime config defaults -------------------- */
/*  The PCM5102A is a true stereo DAC. The I2S peripheral ALWAYS runs in stereo
 *  (both L and R driven) so the unit works no matter how the panel is wired;
 *  whether we actually SEPARATE the two streams is a RUNTIME choice set by the
 *  boot config menu (hold the button at power-on — see app_main.c) and stored in
 *  NVS. These defines only seed the DEFAULT used after a config wipe.
 *
 *  Audio modes (AUDIO_MODE_*): selected by tapping through the boot config menu.
 *    MONO_BOTH    - callouts + tone, same signal to L and R (safe if mono-wired)
 *    STEREO_BOTH  - callouts + tone, gently panned apart (voice right, tone left)
 *    MONO_CALLOUTS- callouts only, mono
 *    MONO_TONE    - tone only, mono
 *
 *  STEREO_PAN is the gentle equal-power separation used in STEREO_BOTH: most of
 *  each stream stays common to both channels, only this fraction leans aside.
 *  Into a MONO panel input the two channels sum and the lean cancels to plain
 *  mono — so the pan only buys you anything when both L and R reach your ears.  */
#define AUDIO_MODE_MONO_BOTH     0
#define AUDIO_MODE_STEREO_BOTH   1
#define AUDIO_MODE_MONO_CALLOUTS 2
#define AUDIO_MODE_MONO_TONE     3
#define AUDIO_MODE_COUNT         4

#define DEFAULT_AUDIO_MODE       AUDIO_MODE_STEREO_BOTH  /* used after a wipe     */
#define STEREO_PAN               0.15f   /* equal-power lean, ~85% common/~15% aside */

#define TONE_START_FT     100.0f      /* tone becomes barely audible here        */
#define TONE_FULL_FT      50.0f       /* tone reaches full presence by here      */
#define FLARE_BAND_HI     35.0f       /* top of flare full-attention band        */
#define FLARE_BAND_LO     20.0f       /* bottom of flare band                    */

/* ---- Audio: ascending pitch map ----------------------------------------- */
/*  Pitch ASCENDS as AGL falls (auditory "looming" bias). Energy kept in the
 *  500–3000 Hz band: cuts cockpit noise, survives ANR headsets, and is where
 *  the ear is most sensitive.                                                   */
#define F_AT_TONE_START   600.0f      /* Hz at 100 ft  (low / quiet)             */
#define F_AT_GROUND       1800.0f     /* Hz near 0 ft  (high) — ASCENDING        */
#define F_CLAMP_LO        500.0f      /* never below this                        */
#define F_CLAMP_HI        3000.0f     /* never above this                        */
#define TONE_LOG_SWEEP    1           /* 1 = musical log glide, 0 = linear Hz     */

/* ---- Audio: dB-scheduled volume (perceptual, never linear amplitude) ----- */
/*  Levels are relative to full-scale output. The builder sets the ABSOLUTE
 *  level with the analog trim pot; firmware only provides the ramp shape.
 *  Rule of thumb: +10 dB ~ perceived "twice as loud".                          */
#define TONE_FLOOR_DB     -40.0f      /* barely audible at 100 ft                */
#define TONE_FULL_DB      -6.0f       /* full presence at/below 50 ft            */
#define VOICE_DUCK_DB     4.0f        /* duck the tone this much under a callout  */
#define GAIN_RAMP_MS      40          /* raised-cosine envelope time (>=30–50ms)  */

/* ---- Equal-loudness (ISO 226) correction --------------------------------- */
/*  The tone's pitch ascends as the ground nears. The ear is more sensitive to
 *  higher pitches, so WITHOUT correction the tone would sound progressively
 *  louder through the flare even at a constant electrical level — an unwanted
 *  stress cue. Urgency is meant to be carried by PITCH, with perceived loudness
 *  held constant once the tone has faded in. This flag flattens the ear's
 *  frequency tilt (relative to 1 kHz) using the ISO 226 ~60 phon contour, so
 *  equal scheduled dB sounds equally loud across the whole sweep.
 *  Set to 0 to A/B the raw (uncorrected) behaviour on the bench.               */
#define EQUAL_LOUDNESS_CORRECTION 1

/* ===========================================================================
 *  REGION 2 — HARDWARE-ONLY CONSTANTS  (firmware build only)
 * ===========================================================================*/
#ifndef UNIT_TEST

/* ---- GPIO pin map (defaults; see WIRING.md) ------------------------------ */
/*  Avoid strapping pins (0, 45, 46) and the native-USB pins (19, 20) since the
 *  console uses USB-Serial-JTAG for logging.                                   */

/*  SF30 on UART1. (Sensor TX -> S3 RX, Sensor RX -> S3 TX.)                    */
#define PIN_SF30C_RX      17          /* S3 UART RX  <- SF30 TX                  */
#define PIN_SF30C_TX      18          /* S3 UART TX  -> SF30 RX                  */
#define SF30C_UART_NUM    UART_NUM_1

/*  PCM5102A on I2S. SCK is tied to GND on the board (internal PLL) so the S3
 *  emits NO MCLK; mclk is set to I2S_GPIO_UNUSED in audio.c.                   */
#define PIN_I2S_BCK       5           /* bit clock                              */
#define PIN_I2S_LRCK      6           /* word select / LRCK                     */
#define PIN_I2S_DIN       7           /* data out (S3 -> DAC DIN)               */

/*  Reset button: momentary to GND, internal pull-up. Wipes the NVS ground
 *  buffer and reboots (install / reinstall). Active-low.                       */
#define PIN_RESET_BTN     4
#define RESET_BTN_ACTIVE_LEVEL 0      /* pressed == logic low                   */

/* ---- UART driver buffer sizing ------------------------------------------ */
#define SF30C_UART_RX_BUF 1024
#define SF30C_UART_TX_BUF 256

/* ---- NVS namespace / keys ------------------------------------------------ */
#define NVS_NAMESPACE     "lidaragl"
#define NVS_KEY_GROUNDBUF "groundbuf"  /* blob: BOOT_BUFFER_N x boot_entry_t     */
#define NVS_KEY_AUDIOCFG  "audiocfg"   /* u8: selected AUDIO_MODE_* (config menu) */
#define NVS_KEY_STARTALT  "startalt"   /* u16: callout start-altitude cap in ft   */

/* ---- FreeRTOS task stacks (BYTES in ESP-IDF) & priorities ---------------- */
#define SENSOR_TASK_STACK 3072
#define LOGIC_TASK_STACK  4096
#define AUDIO_TASK_STACK  4096
#define SENSOR_TASK_PRIO  6
#define LOGIC_TASK_PRIO   5
#define AUDIO_TASK_PRIO   7           /* audio is the most timing-critical       */
#define SENSOR_TASK_CORE  0
#define LOGIC_TASK_CORE   1
#define AUDIO_TASK_CORE   1

/* ---- Power management frequency envelope -------------------------------- */
#define PM_MAX_FREQ_MHZ   240
#define PM_MIN_FREQ_MHZ   40

#endif /* !UNIT_TEST */

#endif /* LIDARAGL_CONFIG_H */
