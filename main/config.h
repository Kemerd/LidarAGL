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
/*  MUST equal the sensor's "Serial port baud rate" in LightWare Studio (currently
 *  set to 460800 on our unit — confirm in Studio → Communication). A mismatch yields
 *  n=0 (big error) or a CE-heavy garbage stream decoding to ~328 ft (~2x error).
 *  The S3 UART handles any of these rates; just keep BOTH sides equal.             */
#define SF30C_BAUD        460800      /* match Studio's Serial port baud; 8N1     */
#define SF30C_ASCII       0           /* legacy 2-byte high/low stream          */
#define SF30C_BINARY      1           /* LWNX framed protocol                   */
/*  Our SF30/C (and its shipping firmware) ONLY offers "Distance over Serial" —
 *  the legacy 2-byte high/low stream + the '#' ASCII command set. It has NO
 *  "Full communication mode" (LWNX) option in LightWare Studio, so the REAL
 *  sensor is parsed as ASCII. The bench SIM stream stays LWNX regardless — that
 *  sim/real split lives in sf30c_read_latest_ft(). Set the sensor's serial baud
 *  to SF30C_BAUD and Output type to "Distance over Serial" in Studio (see README).*/
#define SF30C_MODE        SF30C_ASCII
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

/*  After a landing the box keeps its descent callouts armed so a quick bounce or
 *  go-around still gets the tone (you usually WANT it then). But once the aircraft
 *  has simply SAT on the ground for this long — a taxi-back, a full-stop, parked —
 *  we DISARM as if freshly rebooted: the next takeoff is silent until the aircraft
 *  climbs back through ARM_FT and re-arms naturally. The timer counts only
 *  continuous GROUND dwell and resets the moment we leave the ground, so a touch-
 *  and-go inside this window keeps the arming and the tone.                       */
#define GROUND_RESET_MS    30000u      /* 30 s parked -> disarm (re-arm via climb)*/

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
#define STEREO_PAN               0.90f   /* equal-power lean, ~90% common/~10% aside */

#define TONE_START_FT     100.0f      /* tone becomes barely audible here        */
#define TONE_FULL_FT      50.0f       /* full presence by here @100ft start; the
                                       * full-presence point scales with the start,
                                       * so the 200 ft option reaches full at 100 ft
                                       * (swell runs 200->100, not 200->50)       */
#define FLARE_BAND_HI     35.0f       /* top of flare full-attention band        */
#define FLARE_BAND_LO     20.0f       /* bottom of flare band                    */

/*  TONE_START_FT above is the DEFAULT altitude at which the presence tone begins
 *  its fade-in (and where the pitch sweep starts). It is pilot-selectable in the
 *  boot config menu's LAST level: the default 100 ft, or a higher 200 ft for an
 *  earlier, gentler swell on a longer final. Only these two are offered. The chosen
 *  value is stored in NVS (NVS_KEY_TONESTART) and threaded into the PURE tone
 *  schedule at boot via audio_set_tone_start(), so agl_to_pitch_hz() and
 *  agl_to_tone_db() anchor their pitch + dB fade-in on whichever the pilot picked.  */
#define TONE_START_FT_HIGH    200.0f  /* the alternate (earlier) tone-start option */
#define DEFAULT_TONE_START_FT TONE_START_FT  /* used until the pilot picks 200 ft   */

/* ---- Audio: ascending pitch map ----------------------------------------- */
/*  Pitch ASCENDS as AGL falls (auditory "looming" bias). Energy kept in the
 *  500–3000 Hz band: cuts cockpit noise, survives ANR headsets, and is where
 *  the ear is most sensitive.                                                   */
#define F_AT_TONE_START   600.0f      /* Hz at 100 ft  (low / quiet)             */
#define F_AT_GROUND       1400.0f     /* Hz near 0 ft  (high) — ASCENDING        */
#define F_CLAMP_LO        500.0f      /* never below this                        */
#define F_CLAMP_HI        3000.0f     /* never above this                        */
#define TONE_LOG_SWEEP    1           /* 1 = musical log glide, 0 = linear Hz     */

/* ---- Audio: timbre shaping (warmth + anti-harshness) --------------------- */
/*  A PURE sine has exactly one frequency and zero overtones, so it can sound
 *  thin, sterile and "ice-pick" sharp — like a hearing-test beep — especially
 *  sitting on the ear's most-sensitive shelf near the top of the sweep. Three
 *  cheap, complementary moves round the tone off without losing the
 *  fundamental's noise cut-through (all per-sample, all in audio.c):
 *
 *    1) A small 2nd-harmonic mix. An EVEN-order overtone (one octave up) reads
 *       as "warm / organ-like" rather than buzzy; it adds body so the tone is
 *       a voice, not a test tone. Kept low so the fundamental still dominates
 *       (that is the part that survives cockpit noise and ANR headsets).
 *
 *    2) A 1-pole low-pass on the final mix. It does NOTHING to the pure sine
 *       itself, but it tames the small ODD harmonics the tanh() soft-clip
 *       generates and softens the 2nd-harmonic's own edge, so the top end is
 *       silk instead of glass. fc sits above the whole sweep + 2nd harmonic.
 *
 *    3) Conditional soft-clip — see TONE_SOFTCLIP_ONLY_WITH_VOICE below.       */
#define TONE_HARMONIC2_LVL  0.18f     /* 2nd-harmonic mix (0 = pure sine)        */
#define MIX_LPF_FC_HZ       4200.0f   /* 1-pole LPF corner on the L/R mix bus    */

/*  The solo presence tone is scheduled to peak around -6 dBFS, so on its OWN it
 *  never needs limiting — running it through tanh() only ADDS odd-harmonic edge
 *  for no benefit. We therefore soft-clip ONLY when a voice clip is also playing
 *  (the one time tone+voice can sum past full scale); a solo tone passes through
 *  linearly and clean. Set 0 to always soft-clip (the old behaviour).           */
#define TONE_SOFTCLIP_ONLY_WITH_VOICE 1

/* ---- Audio: dB-scheduled volume (perceptual, never linear amplitude) ----- */
/*  Levels are relative to full-scale output. The builder sets the ABSOLUTE
 *  level with the analog trim pot; firmware only provides the ramp shape.
 *  Rule of thumb: +10 dB ~ perceived "twice as loud".                          */
#define TONE_FLOOR_DB     -40.0f      /* barely audible at 100 ft                */
#define TONE_FULL_DB      -16.0f       /* full presence at/below 50 ft            */
#define VOICE_DUCK_DB     4.0f        /* duck the tone this much under a callout  */
#define GAIN_RAMP_MS      40          /* raised-cosine envelope time (>=30–50ms)  */

/*  Global output headroom — reserved AT THE OUTPUT, on top of the pilot's config
 *  volume offset. The equal-loudness correction (audio_math.c) BOOSTS the tone by
 *  up to +3.18 dB around 1.3–1.6 kHz (the flare's pitch band) to keep perceived
 *  loudness flat as the pitch climbs. That is a real gain INCREASE that eats into
 *  full-scale headroom, and in MONO — where tone + voice sum into one channel at
 *  full weight instead of being panned apart like stereo — the mix can reach or
 *  cross 0 dBFS. Rather than rely on the limiter catching it, we simply reserve
 *  this much fixed headroom on every output sample so the boost can never reach
 *  full scale in the first place (gain-stage, don't clamp). Sized to just cover the
 *  eql peak; the absolute level is irrelevant since the analog trim/panel sets
 *  loudness, so this costs nothing. Applied to flight AND config-menu audio so the
 *  volume preview plays at the same net level the running box will.               */
#define OUTPUT_HEADROOM_DB  -3.2f

/*  Output makeup gain — a deliberate loudness LIFT applied to the final mix (tone +
 *  voice together), on top of the headroom above and the pilot's volume offset. The
 *  render meter showed the hottest real sample sitting near -7 dBFS, i.e. ~7 dB of
 *  unused headroom, so we spend some of it here to drive the line output harder by
 *  default. The hard limiter in f32_to_s16() remains the safety net for the rare
 *  worst-case peak, and OUTPUT_HEADROOM_DB still reserves margin for the
 *  equal-loudness boost; this just decides how much of the remaining room to use.   */
#define OUTPUT_MAKEUP_DB    6.0f

/* ---- Audio: voice-sidechain duck (a real compressor, keyed off the voice) -- */
/*  The tone ducks UNDER the voice the way a broadcast "voice over music" ducker
 *  does: instead of slamming to a fixed attenuation the instant a clip starts
 *  (which clips the leading edge, since the tone drops before the word is even
 *  audible), we DETECT the voice's actual loudness and duck PROPORTIONALLY to it.
 *
 *  How it works, per sample (all in audio.c):
 *    1) A one-pole peak FOLLOWER tracks the rectified voice |sample| — this is the
 *       sidechain envelope, "how loud is the voice right now". It rises with the
 *       DUCK_ATTACK_MS time constant and falls with DUCK_RELEASE_MS, so it leads
 *       neither the syllable's onset nor lingers past its tail.
 *    2) Above DUCK_THRESHOLD the envelope opens the duck; the duck depth scales
 *       from 0 dB (voice at/below threshold) to the full VOICE_DUCK_DB (voice at
 *       or above DUCK_KNEE_LEVEL). Between those it interpolates — a soft knee.
 *
 *  Because the duck now FOLLOWS the voice amplitude, a word that fades in eases
 *  the tone down WITH it (no clippy pre-duck), and the tone breathes back as the
 *  word trails off. Cost is trivial: one fabsf + one multiply-add per sample.     */
#define DUCK_ATTACK_MS    6.0f        /* follower rise: how fast the duck opens    */
#define DUCK_RELEASE_MS   90.0f       /* follower fall: gentle recovery, no pump   */
#define DUCK_THRESHOLD    0.04f       /* |voice| below this == "silent", no duck   */
#define DUCK_KNEE_LEVEL   0.50f       /* |voice| at/above this == full VOICE_DUCK_DB*/

/* ---- Audio: baseline tone trim when callouts are enabled ----------------- */
/*  Even with the voice ducking the tone during a callout, the steady presence
 *  tone is loud enough that the spoken numbers can FEEL quiet by contrast — a
 *  loudness illusion, not an actual level problem. So whenever the active mode
 *  plays callouts we hold the presence tone a constant TONE_TRIM_WITH_VOICE_DB
 *  below its schedule for the WHOLE descent (not just under a word), giving the
 *  voice a touch more room across the board. It stacks with VOICE_DUCK_DB during
 *  an actual callout. In a tone-only mode there are no callouts to clear, so the
 *  trim is not applied and the tone runs at its full scheduled level.            */
#define TONE_TRIM_WITH_VOICE_DB  -1.0f  /* steady tone cut while callouts are on   */

/* ---- Audio: independent tone + voice volume offsets (boot config menu) ----- */
/*  Two pilot-set trims, layered ON TOP of the builder's analog pot, chosen in the
 *  boot config menu and stored in NVS. Unlike the old single master, these adjust
 *  the presence TONE and the voice CALLOUTS independently, so the pilot can dial
 *  the tone-vs-voice balance to taste:
 *
 *    - TONE volume  : cut OR boost, TONE_VOLUME_DB_MIN .. TONE_VOLUME_DB_MAX.
 *    - VOICE volume : cut ONLY,     VOICE_VOLUME_DB_MIN .. 0 dB (never boosted,
 *                     since the clips already sit near full scale).
 *
 *  Both step in 2 dB increments (a comfortably-noticeable click) and default to
 *  0 dB (no change) until the pilot moves them. The tone offset is stored SIGNED
 *  in NVS (it can go negative or positive); the voice offset is stored as a small
 *  NON-NEGATIVE cut magnitude (e.g. 4 == -4 dB) so it packs into a u8 cleanly.   */
#define TONE_VOLUME_DB_MIN     -6.0f  /* deepest tone cut the menu offers          */
#define TONE_VOLUME_DB_MAX      6.0f  /* loudest tone boost the menu offers        */
#define TONE_VOLUME_DB_STEP     2.0f  /* +6,+4,+2,0,-2,-4,-6 dB                     */
#define DEFAULT_TONE_VOLUME_DB  0.0f  /* no change until the pilot moves it        */

#define VOICE_VOLUME_DB_MIN    -6.0f  /* deepest voice cut the menu offers         */
#define VOICE_VOLUME_DB_STEP    2.0f  /* 0,-2,-4,-6 dB                             */
#define DEFAULT_VOICE_VOLUME_DB 0.0f  /* no cut until the pilot lowers it          */

/*  The volume menu previews each step as a short "mini-flare": the REAL presence
 *  tone sweeps DOWN the 20->10 ft band (pitch rising on the actual schedule) in the
 *  BACKGROUND while the "20" and "10" callouts speak over it and DUCK it — the exact
 *  sidechain that flies. So the pilot judges the tone-vs-voice BALANCE the way the
 *  box truly sounds, not as a static beep. Both tone and voice carry the live tone/
 *  voice offsets, so each step auditions the chosen balance.                       */
#define VOLUME_PREVIEW_SWEEP_FROM_FT 20.0f  /* sweep starts here ("20" speaks)      */
#define VOLUME_PREVIEW_SWEEP_TO_FT   10.0f  /* ...descends to here ("10" speaks)    */
#define VOLUME_PREVIEW_SWEEP_MS      2500   /* quick 2.5 s mini-flare               */

/*  Legacy fixed-burst preview parameters, still used for the 1 kHz reference tone
 *  in any non-sweep context. 1 kHz is the equal-loudness reference.               */
#define VOLUME_PREVIEW_HZ      1000.0f
#define VOLUME_PREVIEW_MS      350    /* length of each preview tone burst (ms)    */
#define VOLUME_PREVIEW_DB      -6.0f  /* preview tone level (== TONE_FULL_DB)      */

/* ---- Audio: flare fade-out (distraction guard under the flare) ----------- */
/*  Once the aircraft settles below FLARE_FADE_FT the presence tone is no longer
 *  giving the pilot useful new information (the last callout — "ten" — has
 *  already fired) and would only distract during the flare. So we FADE THE TONE
 *  OUT over FLARE_FADE_OUT_MS to silence. Climbing back above FLARE_FADE_FT
 *  (a bounce or go-around) RE-ARMS the tone by fading it quickly back UP over
 *  FLARE_FADE_IN_MS, after which the slow fade-out re-starts if we sink again.
 *  The asymmetry — slow out, fast in — is deliberate: leaving the flare must
 *  restore the cue promptly, while entering it must be gentle and unobtrusive.
 *  Implemented as a slew-limited multiplier in audio.c, so a re-cross part-way
 *  through the fade simply reverses from wherever the envelope currently sits.  */
#define FLARE_FADE_FT       10.0f     /* below this the tone fades out           */
#define FLARE_FADE_OUT_MS   3000      /* full->silent fade-out time              */
#define FLARE_FADE_IN_MS    100       /* silent->full quick restore on re-cross  */

/* ---- Variometer "blip" cadence (sink/climb-rate feature) ----------------- */
/*  A varioshore-style glider behaviour folded into the presence tone. The boot
 *  menu picks ONE active direction (mutually exclusive, see audio_set_vario_enable):
 *    - SINK mode : the below-100 ft tone blips faster the faster you SINK; level or
 *                  climbing => a steady CONSTANT tone.
 *    - CLIMB mode: the inverse — blips on the way UP, constant when level/sinking.
 *    - OFF       : steady tone always (the default; the gate is held open).
 *
 *  The cadence is modelled on the open-source BlueFlyVario, whose blip rate is
 *  PROPORTIONAL to vertical speed (bpm ~= 200 x m/s): it starts beeping at the 0.2 m/s
 *  onset and the period bottoms out at ~0.1 s (~600 bpm) by ~3 m/s, with its
 *  speedMultiplier just stretching WHERE that cap lands. We take that curve, FLIP it to
 *  descent, and STRETCH it ~2x so the 600 bpm cap lands at 1000 fpm (instead of
 *  BlueFly's ~590) -- gentler for a landing aid than a thermal vario. The pitch is NOT
 *  touched here: it stays purely a function of ALTITUDE (agl_to_pitch_hz). Per blip:
 *      if rate_fpm < VARIO_ONSET_FPM  -> CONSTANT tone (no chopping)
 *      bpm        = min(VARIO_BPM_MAX, rate_fpm * VARIO_BPM_MAX / VARIO_FULL_FPM)
 *      period_ms  = 60000 / bpm
 *      silence_ms = period_ms / (1 + VARIO_BEEP_FACTOR)   (beep = the rest)
 *  rate_fpm is the magnitude in the active direction only (the other way reads 0 =>
 *  constant tone). Onset 40 fpm == BlueFly's 0.2 m/s; the 1000 fpm / 600 bpm cap is the
 *  stretched top end. Retune by moving those three numbers.
 *
 *  The current beep/silence ALWAYS plays out at its committed length — durations are
 *  recomputed only at a phase boundary, never mid-blip. To stop the cadence LURCHING
 *  when the rate suddenly shifts, the rate that feeds the mapping is first run through
 *  a one-pole follower (VARIO_RATE_SMOOTH_MS) so a sudden change eases in over a few
 *  blips instead of snapping in one. Smaller = snappier, larger = a smoother glide.   */
#define VARIO_ONSET_FPM       40.0f   /* below this: CONSTANT tone (BlueFly 0.2 m/s)*/
#define VARIO_FULL_FPM        1000.0f /* rate at which the cadence reaches its cap  */
#define VARIO_BPM_MAX         600.0f  /* cap blip rate (beats/min) at VARIO_FULL_FPM*/
#define VARIO_BEEP_FACTOR     0.5f    /* beep:silence ratio within each period    */
#define VARIO_EDGE_MS         4.0f    /* raised-cosine-ish gate ramp (anti-click) */
#define VARIO_RATE_SMOOTH_MS  450.0f  /* one-pole time const that eases cadence    */
                                      /* changes when the rate suddenly shifts     */

/* ---- "Positive rate" climb callout (takeoff / touch-and-go) -------------- */
/*  A spoken "positive rate" reminder on the way UP, modelled on the standard
 *  "positive rate, gear up" cockpit call. It is DISABLED by default (a config-
 *  menu toggle enables it) and is deliberately NOT a single AGL crossing: a lone
 *  sample at some height would fire on a bounce, a flare balloon, or sensor
 *  jitter. Instead we CONFIRM a genuine sustained climb:
 *
 *    1) ARM the detector only once the aircraft has SETTLED in the flare /
 *       touch-and-go region — held at or below POSRATE_ARM_FT (intentionally the
 *       same 10 ft as FLARE_FADE_FT) continuously long enough for the tone's
 *       flare FADE-OUT to have finished, i.e. for FLARE_FADE_OUT_MS. Tying the
 *       arm gate to the fade-out completion is the bounce guard the user asked
 *       for: a bounce — a brief dip below 10 ft that pops back up before the fade
 *       completes — never arms it. This still re-arms on EVERY landing/touch, so
 *       a true touch-and-go re-arms for the next departure.
 *    2) Once armed, the callout fires only after the aircraft has climbed back
 *       ABOVE POSRATE_ARM_FT and the smoothed climb rate has held at or above
 *       POSRATE_MIN_FPS CONTINUOUSLY for POSRATE_SUSTAIN_MS. The sustain window
 *       is the part that rejects noise — we are confirming that the climb is real
 *       and persistent, not measuring a specific climb performance. 100 fpm is a
 *       floor comfortably above lidar jitter; a real Glasair III climb of many
 *       hundreds of fpm clears it almost instantly.
 *    3) One-shot: after firing it disarms until the aircraft has settled back in
 *       the arm region (i.e. the next landing / touch completes its fade-out).    */
#define POSRATE_ARM_FT      10.0f     /* arm at/below this AGL (matches FLARE_FADE_FT) */
#define POSRATE_MIN_FPS     1.667f    /* 100 fpm climb floor (100/60 ft/s)        */
#define POSRATE_SUSTAIN_MS  2000u     /* rate must hold this long, continuously   */

/* ---- "Check gear" descent callout ---------------------------------------- */
/*  At a pilot-configurable descent altitude the box speaks the altitude number
 *  then "check gear" (e.g. "two hundred ... check gear"). The altitude is chosen
 *  in the config menu from a per-profile list (see sensor_profile.c's
 *  gear_check_opts) or turned OFF. It is OFF (disabled) by default — the stored
 *  NVS value is 0 for OFF, or the chosen altitude in feet (see NVS_KEY_GEARCHK). */

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
/*  Target board is a MINI ESP32-S3 that only breaks out GPIO 1..13 (plus 5V,
 *  GND, 3V3 and the USB TX/RX), so every assignment below lives in 1..13.
 *  Avoid strapping pins (0, 3, 45, 46) and the native-USB pins (19, 20) — the
 *  console uses USB-Serial-JTAG for logging. GPIO 3 is skipped (strapping).     */

/*  SF30 on UART1. (Sensor TX -> S3 RX, Sensor RX -> S3 TX.) Pins 8/9 are clean
 *  low GPIOs on the mini board (not strapping, not USB).                        */
#define PIN_SF30C_RX      8           /* S3 UART RX  <- SF30 TX                  */
#define PIN_SF30C_TX      9           /* S3 UART TX  -> SF30 RX                  */
#define SF30C_UART_NUM    UART_NUM_1

/*  PCM5102A on I2S. SCK is tied to GND on the board (internal PLL) so the S3
 *  emits NO MCLK; mclk is set to I2S_GPIO_UNUSED in audio.c.                   */
#define PIN_I2S_BCK       5           /* bit clock                              */
#define PIN_I2S_LRCK      6           /* word select / LRCK                     */
#define PIN_I2S_DIN       7           /* data out (S3 -> DAC DIN)               */

/*  Config button: momentary to GND, internal pull-up. Held at power-on it opens
 *  the boot config menu (wipes the NVS ground buffer + saved audio config, then
 *  tap/double-tap to pick the audio mode + callout start altitude). Active-low. */
#define PIN_CONFIG_BTN    4
#define CONFIG_BTN_ACTIVE_LEVEL 0     /* pressed == logic low                   */

/* ---- UART driver buffer sizing ------------------------------------------ */
#define SF30C_UART_RX_BUF 1024
#define SF30C_UART_TX_BUF 256

/* ---- NVS namespace / keys ------------------------------------------------ */
#define NVS_NAMESPACE     "lidaragl"
#define NVS_KEY_GROUNDBUF "groundbuf"  /* blob: BOOT_BUFFER_N x boot_entry_t     */
#define NVS_KEY_AUDIOCFG  "audiocfg"   /* u8: selected AUDIO_MODE_* (config menu) */
#define NVS_KEY_STARTALT  "startalt"   /* u16: callout start-altitude cap in ft   */
#define NVS_KEY_VOLOFS    "volofs"     /* u8: voice volume CUT magnitude in dB     */
#define NVS_KEY_TONEVOL   "tonevol"    /* i8: tone volume offset in dB (signed)    */
#define NVS_KEY_GEARCHK   "gearchk"    /* u16: gear-check altitude in ft (0 == OFF)*/
#define NVS_KEY_POSRATE   "posrate"    /* u8: positive-rate callout enable (0/1)   */
#define NVS_KEY_SINKRATE  "sinkrate"   /* u8: sink-rate vario-blip enable (0/1)    */
#define NVS_KEY_CLIMBRATE "climbrate"  /* u8: climb-rate vario-blip enable (0/1)   */
#define NVS_KEY_TONESTART "tonestart"  /* u16: tone-start altitude in ft (100/200) */

/* ---- Bench HIL simulation (USB-Serial-JTAG side-door) -------------------- */
/*  A developer can bench-test THIS firmware with headphones and NO LiDAR fitted
 *  by streaming byte-accurate SF30/C LWNX frames into the native USB-Serial-JTAG
 *  from the tools/bench_sim Python app. A runtime flag (NEVER persisted — so a
 *  unit can't accidentally ship in sim mode) swaps the sensor read path from
 *  UART1 to the USB-Serial-JTAG; everything downstream is the real decode chain.
 *
 *  At boot we briefly listen for a bench "hello": if a USB host is on the bus we
 *  wait up to SIM_ATTACH_WINDOW_MS for it, but if nothing is plugged in (no USB
 *  SOF) we bail after SIM_ATTACH_GRACE_MS so a normal/flight boot is barely
 *  delayed and the console is left exactly as it was.                           */
#define SIM_USJ_RX_BUF        1024   /* USJ driver RX ring (>= one frame @ ~78 Hz) */
#define SIM_USJ_TX_BUF        256    /* unused by us, but the driver requires > 0  */
#define SIM_POLL_MS           20     /* sim: drain USB RX at LEAST this often, no
                                      * matter the (slow, e.g. 750 ms GROUND) state
                                      * poll cadence — otherwise the RX ring fills
                                      * between reads and drops frames, including a
                                      * one-shot control frame like reboot-to-config */
#define SIM_ATTACH_GRACE_MS   800    /* fast-bail a normal boot when no USB host    */
#define SIM_ATTACH_WINDOW_MS  4000   /* max wait for a hello while a host IS present*/
/*  Why 800/4000: entering sim requires a reboot, and resetting a native-USB chip
 *  drops the COM port for ~1 s while it re-enumerates. The host bench reconnects
 *  and resumes its hello spam inside this window. The grace covers USB enumeration
 *  so a genuinely unplugged (flight) boot still bails quickly.                   */

/*  Custom LWNX command id for host->device bench control. Reuses the LWNX frame
 *  + CRC so the device validates it with the same parser; the value sits safely
 *  outside LightWare's well-known set (0/29/30/44/76, see lwnx.h).              */
#define LWNX_CMD_BENCH_CTRL   200u
/*  Bench-control opcodes (carried in payload[0]). Mirrored byte-for-byte in
 *  tools/bench_sim/protocol.py.                                                  */
#define OP_HELLO          0x01u   /* attach: enter sim mode this boot             */
#define OP_BENCH_REAL     0x02u   /* attach: REAL sensor + scaled-AGL debug boot  */
#define OP_ENTER_CONFIG   0x10u   /* attach + run the boot config menu this boot  */
#define OP_REBOOT         0x11u   /* esp_restart(); the host re-attaches after    */
#define OP_MENU_NEXT      0x20u   /* config menu: single-tap (cycle a selection)  */
#define OP_MENU_CONFIRM   0x21u   /* config menu: double-tap (confirm a selection)*/

/* ---- Bench REAL-sensor scaled debug mode (OP_BENCH_REAL) ----------------- */
/*  A second bench path that is the OPPOSITE of sim mode: instead of feeding
 *  fabricated frames over USB, it keeps the REAL LiDAR on UART1 and just (a)
 *  scales the post-ground AGL up so a close bench target exercises the full
 *  flight-altitude callout ladder, and (b) periodically logs the raw sensor
 *  reading so you can confirm the sensor is actually streaming.
 *
 *  The scaling is applied to AGL (range MINUS the learned ground), never to the
 *  raw range — so the boot ground-fill, the MAX_VALID_FT sanity check, and the
 *  stored NVS ground all keep working in honest small feet and nothing leaks
 *  between this mode and a normal boot. Each foot of real range above ground
 *  becomes BENCH_SCALE_GAIN feet of AGL, so a few feet of hand travel sweeps the
 *  whole 0..400 ft band (boot distance == 0 ft AGL, +4 ft == ~400 ft AGL at the
 *  x100 default). This is only the COMPILE-TIME DEFAULT: the bench tool may send
 *  its own gain in the OP_BENCH_REAL attach frame, which wins for that boot (see
 *  bench_attach_detected / s_bench_scale_gain in app_main.c).
 *  Runtime-only and never persisted, exactly like sim mode.                     */
#define BENCH_SCALE_GAIN     100.0f  /* default AGL feet produced per real foot above ground */
#define BENCH_TAPE_LOG_MS    100     /* fast, compact "bench: -> N ft" value cadence  */
                                     /* (the host altitude tape tracks THIS line)     */
#define BENCH_DEBUG_LOG_MS   750     /* slower verbose raw/ground/agl breakdown line  */

/* ---- FreeRTOS task stacks (BYTES in ESP-IDF) & priorities ---------------- */
#define SENSOR_TASK_STACK 3072
#define LOGIC_TASK_STACK  4096
#define AUDIO_TASK_STACK  4096
#define SENSOR_TASK_PRIO  6
#define LOGIC_TASK_PRIO   5
#define AUDIO_TASK_PRIO   7           /* audio is the most timing-critical       */
#define SENSOR_TASK_CORE  0
#define LOGIC_TASK_CORE   0           /* with sensor; audio gets core 1 to itself */
#define AUDIO_TASK_CORE   1           /* real-time render alone on its own core    */

/* ---- Power management frequency envelope -------------------------------- */
#define PM_MAX_FREQ_MHZ   240
#define PM_MIN_FREQ_MHZ   40

#endif /* !UNIT_TEST */

#endif /* LIDARAGL_CONFIG_H */
