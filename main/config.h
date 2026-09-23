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

/* ---- Firmware version ---------------------------------------------------- */
/*  The single source of truth for the build's human-readable version string.
 *  Bump this on every release that ships to a unit. It is logged in the boot
 *  banner (app_main.c) so a `idf.py monitor` / bench capture immediately shows
 *  which firmware a box is running, and it lives here in the pure-logic region
 *  so the host tests + the WASM emulator can read the same constant without
 *  pulling in any ESP-IDF headers.
 *
 *  DEMO builds carry an unmissable "-DEMO" suffix. The -DDEMO_MODE flag is
 *  STICKY in a build directory's CMake cache (see main/CMakeLists.txt), so a
 *  plain `idf.py build` can silently keep producing demo images — and a demo
 *  image on the AIRCRAFT self-arms its AGL scaling with no USB attach. The
 *  boot banner must therefore be a POSITIVE discriminator between the two
 *  images, not leave identification to the easily-missed demo chirp.           */
#ifndef DEMO_MODE
#define DEMO_MODE 0                 /* compiled OUT unless the build defines it */
#endif
#if DEMO_MODE
#define FIRMWARE_VERSION "v1.64-DEMO"
#else
#define FIRMWARE_VERSION "v1.64"
#endif

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

/* ---- Range tracking (per-sample Kalman tracker — see range_filter.c) ------ */
/*  The checksum-free SF30/C stream can decode a corrupted byte to any plausible
 *  distance in 0..537 ft; the protective window can reflect a "0 ft" return;
 *  and the sensor emits erroneous distances before it reports lost signal. The
 *  v1.64 tracker processes every raw sample with its own timestamp:
 *  validity gates -> a two-state Kalman filter behind an innovation gate ->
 *  M-of-N candidate tracks for (re)acquisition -> explicit track states.
 *
 *  HISTORY, kept so the lessons survive the rewrite. The v1.59-v1.63 design was
 *  built around per-POLL batches: a median-of-drain vote, a Hampel window
 *  (HAMPEL_WIN 7, K 3, MAD floor 0.35 ft) with a slew allowance capped at
 *  60 ft, poll-count re-acquisition (3 agreeing polls + 8 raw samples), a
 *  12-poll track break, a drain-coherence test, and an EMA later replaced by an
 *  alpha-beta smoother (tau 0.12 s). Every one was right at one poll cadence
 *  and wrong at another: a 1-2 sample drain has no median immunity; "12 polls"
 *  is 0.3 s at DESCENT and 6 s at CRUISE; a per-poll band capped the descent
 *  rate the filter could follow, which silenced whole approaches (Sept 2026
 *  flight: frozen at the ceiling pin, grass returning from ~250 ft). Working
 *  per SAMPLE makes the cadence irrelevant to every decision below.          */

/*  Drain buffer: the samples of one poll, in arrival order, newest kept. 64
 *  covers the slowest poll (750 ms x 78 Hz = ~58 samples).                    */
#define RANGE_DRAIN_MEDIAN_N   64

/*  The airframe's physical |d(range)/dt| bound. Gear down at idle the Glasair
 *  III sinks 3500-4000 fpm (58-67 ft/s) and ~2000 fpm approaches trip TAWS
 *  "SINK RATE" routinely; an old 60 ft/s bound made the filter reject real
 *  descents as impossible. 100 ft/s = 6000 fpm clamps the tracked rate and
 *  sizes the candidate's first step and the downward-reachability test.        */
#define RANGE_MAX_SLEW_FPS     100.0f /* physical |d(range)/dt| bound (6000 fpm)*/

/* ---- Main track: Kalman filter + innovation gate ------------------------- */
/*  State [range, rate]; constant-velocity model driven by white-noise
 *  acceleration (Kalata 1984; the standard radar/altimeter tracker). A sample is
 *  accepted when its innovation nu = z - x_pred satisfies nu^2 <= g^2 * S with
 *  S = P00 + R: a statistical, per-sample test (the PX4 EKF2 rangefinder gate
 *  uses g = 5). It replaces the median, Hampel, coherence and slew-allowance
 *  layers at once, and because P grows with time the gate WIDENS by itself
 *  across a gap and tightens again once the track is re-established.
 *
 *  sigma_a is the manoeuvre the model must follow without losing the target:
 *  a round-out from a 4000 fpm descent to a flare is ~10-20 ft/s^2.
 *  sigma_m is the FLOOR of the measurement noise (the SF30's +/-5 cm plus a
 *  little surface texture); the actual R is estimated online from the accepted
 *  innovations and bounded by RF_SIGMA_MEAS_MAX_FT, so the gate adapts to real
 *  grass/tarmac texture without ever opening wide enough to admit junk.       */
#define RF_SIGMA_ACCEL_FPS2    15.0f  /* white-noise acceleration (ft/s^2)      */
#define RF_SIGMA_MEAS_FT       0.25f  /* measurement noise floor (ft)           */
#define RF_SIGMA_MEAS_MAX_FT   2.0f   /* adaptive measurement noise ceiling (ft)*/
#define RF_R_ADAPT_ALPHA       0.02f  /* innovation-variance follower gain      */
#define RF_GATE_SIGMA          5.0f   /* innovation gate, in sigmas (PX4: 5)    */

/*  Sample-and-hold repeats. The SF30/C's measurement rate (#R) and serial
 *  output rate (#U) are SEPARATE settings (LightWare Studio): with the output
 *  faster than the measurement, the wire repeats each reading until the next
 *  one — a staircase. A per-sample tracker that believed every repeat would
 *  see "no motion" for the whole plateau and then a jump, and at a 4000 fpm
 *  approach with 4x repeats (3.4 ft steps) the innovation gate rejects the
 *  jumps and the track is lost. An EXACT repeat of the previous raw sample
 *  carries no new information about a moving target, so within
 *  RF_REPEAT_HOLD_S of the value first appearing it only lets time pass; each
 *  new value is then tracked at its true arrival time. After that a repeat is
 *  a genuinely steady reading and counts normally, so a parked aircraft with
 *  a noiseless return still locks and stays locked. 0.1 s covers any
 *  plausible hold (an internal rate down to 10 Hz) and is a third of the
 *  coast limit, so skipping can never expire a live track.                   */
#define RF_REPEAT_HOLD_S       0.10f  /* repeats within this are time only (s)  */

/*  COAST: no accepted sample for up to this long (a dark patch, a wet spot) and
 *  the track keeps predicting from its measured rate, so the tone and callouts
 *  keep moving on time — at 4000 fpm a 0.3 s freeze would be ~20 ft of callout
 *  lateness. Beyond it the track is LOST and the output holds.               */
#define RF_COAST_MAX_S         0.3f   /* longest predicted-only stretch (s)     */

/* ---- Candidate track: M-of-N initiation --------------------------------- */
/*  Lock-on, re-entry into range, and a genuine terrain step all go through a
 *  separate candidate track built from samples the main gate rejects (radar
 *  M-of-N track initiation). The candidate is a Kalman track of its own,
 *  seeded at rest with a velocity prior of RANGE_MAX_SLEW_FPS / RF_GATE_SIGMA
 *  (so its first gate admits any motion the airframe can make and nothing
 *  more), and every later sample must pass the same innovation gate against
 *  its constant-velocity PREDICTION — so a real descent at any rate confirms,
 *  while scattered junk cannot line up. (v1.59-v1.63 used a fixed 8 ft
 *  poll-to-poll band here: ~5% of full-span junk fell inside it; the
 *  statistical gate admits well under 1%.) Confirmation needs:
 *    - RF_CONFIRM_SAMPLES members spanning at least RF_CONFIRM_MIN_S, and
 *    - members >= RF_CONFIRM_MEMBER_FRAC of EVERY sample since its first
 *      member: disagreeing returns, junk and no-returns alike. A genuine
 *      surface moves ALL the returns; a bimodal minority cannot steal the
 *      track (every sample the main track accepts is a miss), and a blind,
 *      junk-dominated stream cannot line up half its samples on one line.
 *  A no-member stretch longer than RF_CANDIDATE_GAP_S ends a candidate: its
 *  members must be consecutive, or a blind junk stream could collect scattered
 *  "members" over many seconds (a phantom "100" at 450 ft, found in the sweep). */
#define RF_CONFIRM_SAMPLES     8u     /* members needed to confirm (M)          */
#define RF_CONFIRM_MIN_S       0.10f  /* ...spanning at least this long (s)     */
#define RF_CONFIRM_MEMBER_FRAC 0.5f   /* members / all samples in the span      */
#define RF_CANDIDATE_GAP_S     0.15f  /* no-member gap that ends a candidate    */

/*  Switching onto a confirmed candidate while the main track is alive:
 *    - A level within RANGE_MAX_SLEW_FPS * span + RANGE_REACQUIRE_JUMP_SLACK_FT
 *      of the main track's prediction, capped at RANGE_REACQUIRE_JUMP_CAP_FT,
 *      switches CONTINUOUSLY on the ordinary evidence (a manoeuvre the model
 *      lagged, a small terrain step). With confirmation needing >= 0.1 s the
 *      cap is what binds: 35 ft, whatever the wait.
 *    - Anything else needs RF_BREAK_S of agreement and switches as a TRACK
 *      BREAK (the consumer re-anchors silently instead of walking the ladder
 *      across the gap). A self-consistent stuck byte pattern satisfies every
 *      consistency test for free; physical reachability is the one thing it
 *      cannot fake (a 110 ft teleport on final once spoke "one hundred" at
 *      186 ft). A fixed cap, not a time-grown allowance: an allowance that
 *      grows with waiting lets patience substitute for evidence (v1.63 let a
 *      continuous jump grow to 60 ft; UPWARD jumps were unrestricted — both
 *      now need the same persistence, so a junk burst is coasted through
 *      instead of yanking the tone up and back).
 *  From LOST:
 *    - A DESCENDING candidate (sinking faster than RANGE_REENTRY_SINK_FPS) is
 *      the approach coming back into range: confirmed on the ordinary
 *      evidence, always as a break (a re-established track is a new track —
 *      the radar-altimeter convention).
 *    - A stationary candidate within the continuous reach of the HELD value is
 *      the same surface coming back (a dropout, a garbage burst ending while
 *      parked): resumed continuously on the ordinary evidence.
 *    - Any other stationary candidate — the shape of a stuck byte pattern —
 *      needs RF_BREAK_S. Breaks are silent, so this latency costs nothing
 *      audible, while every stuck pattern shorter than coast + RF_BREAK_S
 *      (1.3 s) never moves the output at all. (v1.63 needed 1.5 s at the slow
 *      cadences but only 12 polls = 0.3 s at the fast one; the time rule is
 *      now the same at every cadence.)
 *  A break is a RE-ENTRY for the late-rung window (see CALLOUT_LATE_TOL_*)
 *  only when the candidate is descending, the aircraft could physically have
 *  flown from its last MEASURED position to the new level in the elapsed time,
 *  and the old track was lost to BLINDNESS — no-return samples, not rejected
 *  returns. Only then were the rungs in the gap genuinely passed while the
 *  laser saw nothing; tree tops, a building or a stuck pattern on final are
 *  rejected returns, and must never spend a rung the aircraft is still above. */
#define RANGE_REACQUIRE_JUMP_SLACK_FT 25.0f /* headroom over the physical reach  */
#define RANGE_REACQUIRE_JUMP_CAP_FT   35.0f /* never a continuous jump beyond    */
#define RF_BREAK_S                    1.00f /* agreement for an unreachable jump */
#define RANGE_REENTRY_SINK_FPS        5.0f  /* descending re-entry threshold     */

/* ---- Tracking verdict: the box's REAL power/latency signal ---------------- */
/*  Whether the box may relax (slow the poll, suspend audio, light-sleep) is a
 *  question about the SENSOR — "can we see the ground?" — not about the
 *  aircraft. Going dark needs this many consecutive drains with nothing usable
 *  in them, while ONE usable return restores tracking immediately: slow to
 *  conclude blindness costs a little power; slow to notice recovery costs
 *  callouts on an approach.                                                    */
#define RANGE_NOTRACK_POLLS      8u   /* useless drains before "sensor is dark" */

/*  Sliding lost-signal vote over the last RANGE_LOST_VOTE_SAMPLES raw samples
 *  (~0.4 s at 78 Hz). "The stream is real" — most recent samples are returns —
 *  gates the fast re-acquire poll, and a lost-dominated stream is evidence of
 *  flying out the top of the range. Must be <= 32 (one bit per sample).       */
#define RANGE_LOST_VOTE_SAMPLES  32u  /* sliding sample window for the vote     */

/*  Ceiling on the elapsed time handed to the filter: a gap far larger than any
 *  real poll interval is a timekeeping artefact (a stalled task, a torn clock
 *  read), not information about the aircraft.                                 */
#define SENSOR_MAX_DT_S          5.0f /* clamp on the filter's elapsed-time input*/

/*  Sensor-ceiling sanity: a return farther than the active profile's
 *  max_range_ft plus this margin is physically impossible (the SF30/C cannot
 *  see 400 ft) and is treated as a no-return sample at the gate.             */
#define RANGE_MAX_MARGIN_FT    15.0f

/* ---- Out-of-range-ABOVE detection (the "silent descent" fix) ------------- */
/*  The SF30 reports no return for TWO opposite reasons: the ground is BEYOND its
 *  reach (we climbed out of range), or the surface returns nothing (water, a
 *  dark wet runway). Holding is right only for the second. If the track was
 *  already within RANGE_CEILING_NEAR_FT of the ceiling when it was lost and the
 *  stream then went lost-signal, the aircraft flew out the TOP: the output is
 *  PINNED at the ceiling, so the descent back into range reads as downward
 *  motion rather than a frozen mid-air value (a frozen 318 ft once kept the box
 *  in ST_CRUISE with no exit). RANGE_CEILING_CONFIRM_POLLS keeps one ragged
 *  drain at the noisy top of the range from flipping the verdict.            */
#define RANGE_CEILING_NEAR_FT      40.0f  /* last track within this of the ceiling*/
#define RANGE_CEILING_CONFIRM_POLLS 2u    /* consecutive lost drains to confirm   */

/*  Tone-path EMA (audio side, unchanged by the robust-filter work).            */
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

/*  The DOWNWARD counterpart, and deliberately a different number. GROUND_DEV_FT
 *  bounds how far ABOVE the learned ground a boot can be and still count as
 *  parked; 10 ft is reasonable there because the sky is unbounded. Downward it
 *  is meaningless: the ground sits only ~MOUNT_OFFSET_FALLBACK_FT beneath the
 *  sensor, so a reading cannot be 10 ft BELOW it without going negative — reuse
 *  would make the low-side test unreachable.
 *
 *  A reading materially closer than the learned ground means something is IN
 *  THE BEAM (a mechanic under the wing, a tow bar, a chock, a puddle returning
 *  specularly) or the aircraft is on JACKS. Both used to be clamped to
 *  "AGL = 0, perfectly healthy" and — because the persist gate keys off
 *  !airborne && !calib_error — WRITTEN TO NVS as the new ground, silently
 *  offsetting every future flight. Sized to pass strut compression, surface
 *  variation and sensor noise, while catching a solid object in the beam.      */
#define GROUND_BELOW_DEV_FT      1.5f

/*  Hard junk cap used only while selecting ground-fill samples: a *ground*
 *  reading above this is obvious garbage and is dropped before averaging. This
 *  is NOT an in-flight range limit (in flight we read the sensor's full range). */
#define MAX_VALID_FT             50.0f

/*  Robust (MAD) outlier rejection strength for the ground-fill set.            */
#define MAD_K                    3.0f

/*  Minimum ground-fill samples before the fill may be PERSISTED to NVS as the
 *  learned reference. The gate used to be "more than zero", and a single sample
 *  passes that — but robust_mean over one element cannot reject anything (the
 *  median IS the sample and the MAD is zero, so it is trivially an inlier).
 *  One corrupt-but-plausible reading from a marginal connector could therefore
 *  become the persisted ground for every subsequent flight: not a bad altitude
 *  once, but a permanently offset one until someone recalibrates. A boot that
 *  cannot collect this many samples still FLIES on its resolved reference — we
 *  simply decline to write a conclusion we cannot support.                     */
#define GROUND_PERSIST_MIN_SAMPLES 5u

/*  Emergency ground offset when no usable reference can be established
 *  (empty buffer AND the first reading looks airborne). The box proceeds on
 *  this value and chirps a calibration-error tone so the pilot is aware.       */
#define MOUNT_OFFSET_FALLBACK_FT 3.0f

/*  Upper bound on a reading that may be ADOPTED as the ground reference on a
 *  first boot (empty NVS buffer — a new box, or one the config menu just
 *  wiped). This is a statement about the MOUNT, not about valid ranges: the
 *  sensor looks down from the airframe, so a genuine parked reading is the
 *  mount height plus strut travel and surface variation — a few feet.
 *
 *  The band used to be MOUNT_OFFSET_FALLBACK_FT + GROUND_DEV_FT = 13 ft, which
 *  is over four times a real mount and sits squarely in the altitudes a bounce
 *  or a low pass occupies. A wiped box that power-glitched at 12 ft AGL adopted
 *  12.4 ft as its ground with NO calibration error raised: silent, seeded
 *  disarmed for the landing, and persisted to NVS for every later flight.
 *  Anything above this now falls to the emergency offset, which chirps the
 *  pilot and refuses to persist — the honest response to "I cannot tell where
 *  the ground is". Sized to pass any plausible installation while excluding
 *  altitudes an aircraft can actually be at.                                   */
#define MOUNT_GROUND_MAX_FT      6.0f

/* ---- State machine (ft AGL) --------------------------------------------- */
/*  ARM_FT is intentionally global (same for both sensors): the climb-out after
 *  takeoff is silent until the aircraft has climbed through it, which arms the
 *  descent callouts. CRUISE_FT and the callout list are per-PROFILE, not here.
 *
 *  DEMO builds lower the gate to half the demo sweep: at the x50 default gain a
 *  24-inch rig throw tops out at exactly 100 ft scaled, so the flight gate of
 *  100 ft could NEVER be climbed through and the box would stay disarmed (and
 *  silent) forever. Arming at 50 ft (~12 real inches) lets a full hoist arm the
 *  ladder while the dwell still rejects one-poll spikes. (The fallback guard is
 *  needed here because this line sits above the documented DEMO_MODE section.)  */
#ifndef DEMO_MODE
#define DEMO_MODE         0            /* compiled OUT unless the build defines it */
#endif
#if DEMO_MODE
#define ARM_FT            50.0f        /* demo rig: arm at half the scaled sweep */
#else
#define ARM_FT            100.0f       /* arm descent callouts above this        */
#endif
#define REARM_MARGIN_FT   20.0f        /* climb this far back above a callout    */
                                       /* height to re-arm it (go-around).       */

/*  Arming persistence. A single sample above ARM_FT used to latch the ENTIRE
 *  callout ladder instantly — which is precisely how one garbage range spike
 *  turned into a spoken phantom descent on the taxiway. Arming (and per-callout
 *  RE-arming after a go-around) now requires the height to HOLD continuously,
 *  mirroring the sustained-confirmation pattern the positive-rate detector has
 *  always used. A real climb-out spends many seconds above ARM_FT so the dwell
 *  is imperceptible; a one-poll spike can never satisfy it — the state machine
 *  banks dwell only from the SECOND consecutive in-band decision, because the
 *  elapsed time handed to the first was spent BELOW the gate. (Crediting that
 *  first interval is what once let two 750 ms GROUND polls — or one poll after
 *  a dropped read — arm the whole ladder; see rearm_above/sm_step.)
 *  REARM_SUSTAIN_MS is shorter than ARM_DWELL_MS: it only re-enables single
 *  already-proven callouts on a go-around, where a >=400 ms hold above
 *  height+REARM_MARGIN_FT is trivially true for any genuine climb.             */
#define ARM_DWELL_MS      1500u        /* AGL > ARM_FT this long -> armed        */

/*  CALLOUT LEAD. A number is only useful if the pilot HEARS it at that height.
 *  Between the laser crossing a rung and the word being recognisable sit: half
 *  a poll window, the logic tick, the audio task's pickup, the I2S DMA queue
 *  (~90 ms: 6 x 240 frames at 16 kHz) and the word's own onset before it is
 *  intelligible. That is ~0.2 s — 13 ft at a 4000 fpm gear-down Glasair
 *  approach. Each rung therefore fires when the altitude PREDICTED one lead
 *  time ahead (AGL + sink x CALLOUT_LEAD_S) crosses it, the way 1970s
 *  altitude-callout annunciators did (US4093938 anticipates up to 24 ft to
 *  cover message start-up) and the way Airbus trigger heights sit above their
 *  nominal values. The sink rate is the range tracker's Kalman velocity:
 *  lag-free at a steady descent, and carried over from the confirmed
 *  candidate after a re-acquire, so a re-entry snap cannot spike it.
 *
 *  Only a DESCENT leads; the lead is capped at CALLOUT_LEAD_MAX_FT; and the
 *  go-around re-arm hysteresis stays on the MEASURED altitude, so a lead can
 *  move a callout earlier but can never make one speak twice. The flight
 *  recorder logs raw sample times and the moment each clip starts, so the
 *  real end-to-end latency can be measured and this constant tuned.         */
#define CALLOUT_LEAD_S       0.20f     /* measured-latency target (s)            */
#define CALLOUT_LEAD_MAX_FT  20.0f     /* never anticipate a rung by more       */

/*  Late-rung window (Everett, 2026-09-22). When the laser comes back into
 *  range on a DESCENDING approach — grass returning from ~250 ft, a dark wet
 *  patch ending — the range filter re-establishes the track as a break, and
 *  the rungs between the held value and the new level were passed while
 *  blind. EGPWS practice is "bypassed thresholds are not announced"; a number
 *  heard only slightly late is still useful, so the LOWEST passed rung is
 *  spoken if the word would be heard no more than this far below it:
 *    - rungs at/above CALLOUT_LATE_TOL_SPLIT_FT (100/200/300): 25 ft,
 *    - the closely spaced low rungs (50..10): 5 ft, half their spacing, so a
 *      late number can never be mistaken for the next one down.
 *  Farther below, the rungs are skipped (and spent). Never applied to a break
 *  that is not a physically flyable descending re-entry (a stuck pattern, a
 *  teleport): those re-anchor silently and leave the ladder untouched.       */
#define CALLOUT_LATE_TOL_HI_FT     25.0f  /* window for the 100+ rungs (ft)      */
#define CALLOUT_LATE_TOL_LO_FT      5.0f  /* window for the 10-50 rungs (ft)     */
#define CALLOUT_LATE_TOL_SPLIT_FT 100.0f  /* rungs >= this use the HI window     */
#define REARM_SUSTAIN_MS  400u         /* above h+margin this long -> re-armed   */

/*  The "on the ground" AGL band shared by the pre-arm GROUND classification and
 *  the parked/taxi-back detector below (was a scattered magic 2.0f).            */
#define GROUND_BAND_FT    2.0f         /* at/below this counts as on the ground  */

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

/*  Stale-sample watchdog for the logic task. Decisions are now made ONCE PER
 *  FRESH SENSOR SAMPLE (so the vertical-rate trend divides a real range change
 *  by the real elapsed time instead of attributing a whole 750 ms poll step to
 *  one 20 ms tick — the aliasing that made the taxi-back disarm unreachable).
 *  If the sensor goes silent, this timeout forces a decision tick on the held
 *  value anyway so dwell timers (parked disarm, posrate) keep accruing.         */
#define SAMPLE_STALE_MS   2000u        /* force a decision tick after this long  */

/*  No-data tone mute (fail-safe; enforced in logic_task). The range filter
 *  deliberately HOLDS its last-good output through a lost-signal stretch (over
 *  water, a dark wet runway — the sensor's own documented no-return cases) so
 *  momentary dropouts never twitch the audio. But the presence tone encodes
 *  ALTITUDE as PITCH: held long enough, that hold becomes an actively
 *  misleading "altitude steady" cue while the aircraft may in fact still be
 *  descending. The logic task therefore tracks wall-clock time since the last
 *  FRESH sample the filter marked VALID and force-mutes the tone once the data
 *  is older than this bound; the first live sample restores it instantly.
 *  Silence == "no data" — an honest cue; a frozen pitch is a wrong one.
 *  Sized well clear of the active-state poll gaps (ARMED 50 ms / DESCENT
 *  25 ms); GROUND/CRUISE never sound the tone, so their slow polls don't care. */
#define LOST_SIGNAL_MUTE_MS 1000u      /* data older than this mutes the tone    */

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

/*  Sanity ceiling applied to the AGL handed to the audio engine. Everything
 *  downstream of audio_set_params() is a one-pole follower or a slew limiter,
 *  and each is a NaN trap with no recovery — a single non-finite value poisons
 *  the carried state permanently and ultimately reaches the oscillator's LUT
 *  index, where a float->int cast of a NaN is undefined behaviour and reads the
 *  table out of bounds. That kills the audio task and with it every remaining
 *  callout of the flight. Non-finite input is therefore REJECTED at the
 *  boundary and finite input is clamped to this envelope, which is generous
 *  enough to never touch a legitimate reading (well above the SF30/D's 656 ft
 *  reach, and above any DEMO_MODE scaled altitude worth sounding).             */
#define AUDIO_MAX_AGL_FT  1000.0f     /* clamp on the AGL fed to the audio task  */

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

/* ---- DEMO-MODE build (show-floor pulley rig; compile-time opt-in) --------- */
/*  A standalone show-floor variant of the bench real-sensor mode above: the
 *  REAL LiDAR stays on UART1 and the post-ground AGL is multiplied up, but with
 *  NO laptop / USB attach handshake — the scaling arms itself at every boot from
 *  a gain saved in NVS. Built for demoing the box on a physical pulley rig: a
 *  couple of feet of target travel sweeps the entire callout ladder, tone swell,
 *  flare fade and all, exactly as the real decode chain would fly it.
 *
 *  The whole feature is fenced by DEMO_MODE so ordinary flight firmware carries
 *  NONE of it — no extra menu level, no NVS helpers, no scaling branch; zero
 *  flash cost on a non-demo unit. Build a demo image with:
 *
 *      idf.py -DDEMO_MODE=1 build     (the CMake cache remembers the flag, so
 *      idf.py -DDEMO_MODE=0 build      later plain builds stay demo; set =0 or
 *                                      fullclean to return to flight firmware)
 *
 *  The multiplier is RUNTIME-configurable on the unit itself: LEVEL 9 of the
 *  boot config menu (present only in demo builds) cycles x50 / x100 / x200 /
 *  x400 / OFF — every gain is speakable with the existing number clips, and the
 *  level announces itself with a DOUBLE chirp since no "demo" voice clip exists.
 *  The choice persists in NVS; OFF (stored 0) makes the demo image behave
 *  exactly like flight firmware. Each real foot above the learned ground reads
 *  as <gain> feet of AGL, so at the x50 default 2 ft of pulley travel == 100 ft
 *  of demo altitude — the full callout ladder rides the whole rig throw. A
 *  fresh/wiped demo unit boots straight into the default — no menu visit
 *  required before the show.                                                    */
#ifndef DEMO_MODE
#define DEMO_MODE            0       /* compiled OUT unless the build defines it */
#endif
#define DEMO_GAIN_DEFAULT    50.0f   /* fresh demo unit: 2 ft real == 100 ft demo */
#define NVS_KEY_DEMOGAIN     "demogain"  /* u16: demo AGL gain (0 == OFF)         */

/*  Show-floor feature defaults. A booth unit should demonstrate the full voice
 *  repertoire out of the box — "check gear", "positive rate" and the sink-rate
 *  vario blip — without anyone crawling through the config menu first. In DEMO
 *  builds the NVS loaders therefore treat an ABSENT key as ON (flight builds
 *  keep their safety-first OFF defaults); an explicit menu choice still wins
 *  either way, because a stored value is always honoured over the default.
 *  Gear-check defaults to 200 ft (the flight menu's top SF30/C option). Note the
 *  standard x50 sweep tops out at 100 ft, so demoing the gear call needs a
 *  taller hoist or a bigger gain from LEVEL 9 — that's the booth's call.         */
#define DEMO_GEAR_CHECK_DEFAULT_FT 200.0f /* demo: gear reminder on the 200 call  */

/*  Demo "parked" watchdog band, in REAL feet. The state machine's own 30 s
 *  ground-dwell disarm evaluates its parked test on the SCALED AGL, where its
 *  2 ft stillness band shrinks to millimetres of real travel at demo gains —
 *  below the SF30's 1 cm quantization, so it can never hold and the box would
 *  stay armed (and blip the tone on noise) forever between demo runs. The demo
 *  build therefore runs its OWN parked detector in honest real feet: once the
 *  target has rested within this band of the learned ground for GROUND_RESET_MS
 *  (the same 30 s as flight), the state machine is disarmed exactly as a
 *  taxi-back would — silent until the next hoist climbs back through ARM_FT.   */
#define DEMO_PARKED_BAND_FT  0.25f   /* within ~3 in of ground == "target at rest" */

/*  Demo boot ground-fill "farthest cluster" guard, in REAL feet. On the show
 *  rig the fill samples are EMA-flat and bit-identical, so robust_mean's MAD
 *  collapses to 0 and its outlier rejection degrades to keep-everything — a
 *  hand or body crossing the beam during the ~1 s fill would then be AVERAGED
 *  into the ground reference at full weight and poison the whole session (demo
 *  boots deliberately never persist or re-load ground, see app_main). But an
 *  intrusion can only ever sit NEARER than the surface the rig ranges against,
 *  so the demo fill keeps ONLY samples within this band of the FARTHEST valid
 *  reading: the true resting surface survives, the intrusion is discarded.     */
#define DEMO_GROUND_CLUSTER_FT 0.30f /* fill samples within this of the farthest  */

/*  Minimum parked-cluster samples before the watchdog may RE-ANCHOR the ground
 *  (the disarm itself is not gated). At the logic task's ~20 ms demo tick this
 *  is ~1 s of agreeing readings. Guards a knife-edge: if a beam intrusion ends
 *  within a tick or two of the 30 s latch, the farthest-cluster mean has just
 *  restarted mid-EMA-ramp and would re-anchor off a handful of transitional
 *  samples; too-small clusters instead keep the previous (known-good) ground.  */
#define DEMO_REANCHOR_MIN_N  50u     /* ~1 s of cluster samples to move ground    */

/* ---- Flight recorder (black box — see flightlog.h) ----------------------- */
/*  Two flights came back silent and neither could be diagnosed from evidence,
 *  only reconstructed. The recorder writes every raw SF30 sample, the logic
 *  task's decisions, audio-path events and every console line into the
 *  'flightlog' partition (partitions.csv) as a ring that keeps the most recent
 *  session(s). Pull it with tools/flightlog/flightlog.py.
 *
 *  Budget: ~2.2 MB partition. Typical rates are ~150-250 B/s parked or cruising
 *  (the RLE makes a blind sensor almost free) and ~550 B/s on an approach, so
 *  the ring holds on the order of 1.5-3 hours of powered time.               */
#define FLOG_PARTITION_LABEL   "flightlog" /* must match partitions.csv        */
#define FLOG_RING_BYTES        16384u  /* RAM buffer (power of two): ~30 s of  */
                                       /* approach data if flash must wait     */
#define FLOG_STAGE_BYTES       1024u   /* batched page write: a few ms freeze  */
#define FLOG_WRITER_PERIOD_MS  100u    /* writer wake cadence                  */
#define FLOG_FLUSH_MS          1000u   /* max age of unwritten bytes: at most  */
                                       /* ~1 s is lost when the master goes off*/
#define FLOG_RUNWAY_SECTORS    48u     /* pre-erased sectors kept ready (~6 min*/
                                       /* of approach) so no erase under audio */
#define FLOG_TEXT_MAX          200u    /* longest captured console line        */
#define FLOG_RAW_EMIT_MS       500u    /* RAW record cadence (on a drain edge) */
#define FLOG_DECISION_MIN_MS   100u    /* decision-record floor: 10 Hz, plus   */
                                       /* every tick where anything discrete   */
                                       /* changed (state, arming, fire, tone)  */
#define FLOG_TASK_STACK        4096
#define FLOG_TASK_PRIO         1       /* below every flight task              */
#define FLOG_TASK_CORE         0       /* with sensor/logic; audio keeps core 1*/

/* ---- FreeRTOS task stacks (BYTES in ESP-IDF) & priorities ---------------- */
/*  The sensor task grew from 3072: the flight recorder now frames RAW records
 *  (~0.5 KB of stack) on this task, and the ESP_LOG capture hook formats a
 *  private copy of each console line on whichever task logs it.               */
#define SENSOR_TASK_STACK 4096
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

/* ---- Light-sleep master switch (compile-time) ---------------------------- */
/*  Automatic light-sleep saves power in GROUND/CRUISE, where the box is silent
 *  anyway. It also costs us: sleeping gates the UART clock (the documented
 *  source of the wake-edge framing garbage sf30c.c drains around), tears the
 *  I2S channel down via audio_suspend(), and relaxes the poll to 500 ms — so
 *  every millisecond of latency in noticing a descent is paid back on final.
 *
 *  It is left ENABLED. Sleep was the prime suspect for the silent approach that
 *  motivated this switch, but it turned out to be the accomplice rather than the
 *  culprit: the real defect was that ST_CRUISE became a ONE-WAY DOOR. Climbing
 *  out of the sensor's range froze the published value; a frozen value has zero
 *  trend; and leaving CRUISE requires a descending trend — so the box could
 *  ENTER the sleep state but never leave it, for the rest of the flight. With
 *  the ceiling handling in range_filter.c (see RANGE_CEILING_NEAR_FT) the
 *  descent is visible again and CRUISE exits normally, which is what makes the
 *  power saving safe to keep.
 *
 *  The switch stays because it is the one knob that removes sleep from the
 *  variables entirely if a future flight is still unexplained. Build with
 *  -DSLEEP_MODE_ENABLE=0 and esp_pm_configure() runs with light_sleep_enable =
 *  false (DFS still scales the CPU frequency, which is free) while the logic
 *  task never requests an audio suspend, so the I2S channel stays up for the
 *  whole flight.
 *
 *  Independently of this switch, the logic task now also refuses to enter a
 *  sleep state on data it knows is STALE — a held value is precisely when we
 *  are least entitled to assume nothing is happening below us.                 */
#ifndef SLEEP_MODE_ENABLE
#define SLEEP_MODE_ENABLE 1           /* 1 = allow light-sleep in GROUND/CRUISE */
#endif

#endif /* !UNIT_TEST */

#endif /* LIDARAGL_CONFIG_H */
