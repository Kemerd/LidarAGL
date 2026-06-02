/**
 * sim.js — LidarAGL web emulator front-end.
 *
 * Responsibilities, top to bottom:
 *   1. Load the WASM module (the REAL firmware logic compiled by build_wasm).
 *   2. cwrap its flat ABI (sim_glue.c) into convenient JS functions.
 *   3. Run a requestAnimationFrame loop that:
 *        - reads the user's target AGL (drag / slider),
 *        - steps the firmware state machine with a real dt,
 *        - reads back the decision (state, fired callout, tone),
 *        - drives WebAudio (presence tone + callout clips),
 *        - paints the altitude tape and telemetry.
 *
 * No firmware logic is reimplemented here. Every altitude decision and every
 * tone value comes from the compiled C via the sim_* exports.
 */

import createLidarSim from './dist/lidar_sim.js';

/* =============================================================================
 *  Constants
 * ===========================================================================*/

// Where the real callout recordings live, relative to index.html. We serve from
// the repo root so ../assets reaches the firmware's audio masters.
const AUDIO_BASE = '../assets/original_audio';

// Human-readable names for the sm_state_t enum (must match the C order:
// ST_GROUND=0, ST_CLIMB=1, ST_ARMED=2, ST_CRUISE=3, ST_DESCENT=4).
const STATE_NAMES = ['GROUND', 'CLIMB', 'ARMED', 'CRUISE', 'DESCENT'];

// Accent colour per state, mirroring the cockpit meaning (CSS values).
const STATE_COLORS = {
  GROUND:  '#9aa0ad',
  CLIMB:   '#0a84ff',
  ARMED:   '#30d158',
  CRUISE:  '#64d2ff',
  DESCENT: '#ff9f0a',
};

// The full union of callout heights across both profiles, so we can preload
// every WAV once regardless of which profile is active.
const ALL_CALLOUT_FT = [500, 400, 300, 200, 100, 50, 40, 30, 20, 10];

// Human-facing labels for the AUDIO_MODE_* indices (config.h). The INDEX is the
// source of truth (the flags come from the WASM glue per mode); this is only the
// UI copy. Order matches AUDIO_MODE_MONO_BOTH..MONO_TONE = 0..3.
const AUDIO_MODE_META = [
  { title: 'Mono · both',     desc: 'Callouts + tone, same signal to both ears.' },
  { title: 'Stereo · both',   desc: 'Callouts lean right, tone leans left (gentle pan).' },
  { title: 'Callouts only',   desc: 'Spoken heights only — no presence tone.' },
  { title: 'Tone only',       desc: 'Presence tone only — no spoken callouts.' },
];

// Smoothing factor for the *visual* aircraft glyph only. The state machine is
// always fed the RAW target AGL; this just makes the on-screen motion glide.
const DISPLAY_EASE = 0.18;

/* -----------------------------------------------------------------------------
 *  Simulated-approach physics
 * ---------------------------------------------------------------------------
 *  A scripted ILS approach so you can hear the firmware's natural callout cadence
 *  without hand-flying the slider. The numbers below are real-world textbook, not
 *  invented: a light aircraft tracking the glideslope at 85 kt and a standard 3°
 *  ILS path, then a conventional exponential flare to a soft touchdown.
 *
 *  All this code does is move targetAgl over time; the REAL firmware still makes
 *  every callout / tone / flare-fade decision off that altitude, exactly as if a
 *  human were dragging the aircraft down the tape.
 * ------------------------------------------------------------------------------ */

// Groundspeed → feet per second. 1 knot = 1.68781 ft/s; 85 kt ≈ 143.5 ft/s.
const APPROACH_KNOTS   = 85;
const KT_TO_FTS        = 1.68781;
const APPROACH_GS_FTS  = APPROACH_KNOTS * KT_TO_FTS;

// Standard ILS glideslope. Descent rate on the slope = groundspeed · tan(γ).
// 143.5 · tan(3°) ≈ 7.52 ft/s ≈ 451 ft/min — the classic "rule-of-thumb" sink
// for this speed/angle, so the callout spacing lands right where a pilot expects.
const GLIDESLOPE_DEG   = 3.0;
const GLIDESLOPE_SINK  = APPROACH_GS_FTS * Math.tan(GLIDESLOPE_DEG * Math.PI / 180);

// Flare model. At FLARE_START_FT the pilot arrests the glideslope sink and lets
// it decay exponentially toward a gentle touchdown sink, giving the float-and-
// settle feel of a real flare. tau is the time-constant of that decay (s); a
// larger tau = a longer, lazier float.
const FLARE_START_FT   = 20.0;   // height AGL where the flare begins
const FLARE_TOUCHDOWN_SINK = 1.5; // residual sink at the wheels (ft/s, gentle)
const FLARE_TAU        = 1.6;    // exponential time-constant of the flare (s)

// Where the scripted approach begins, in feet above the highest callout-ceiling
// the sensor can see. We start a touch above the profile's CRUISE band so the run
// opens in CRUISE, breaks into DESCENT through the ladder, and ends on the ground.
const APPROACH_START_MARGIN_FT = 15.0;

// A brief hold once the wheels are down before the run auto-clears, so the final
// "10" callout and the flare-fade tail can finish playing.
const APPROACH_HOLD_FT = 0.3;    // treat at/below this as "on the ground"

/* =============================================================================
 *  Module-level state
 * ===========================================================================*/

let M = null;            // the Emscripten Module
let api = {};            // cwrap'd sim_* functions

// Simulator inputs / derived values.
let targetAgl  = 0;      // where the user wants the aircraft (raw -> sm_step)
let displayAgl = 0;      // eased copy, for smooth drawing
let lastTs     = 0;      // previous rAF timestamp (ms)
let profileIdx = 0;      // 0 = SF30/C, 1 = SF30/D
let cfg        = null;   // cached profile + constant readouts (see readConfig)
let lastFiredFt = null;  // most recent callout height fired (for the readout)

// Scripted-approach runner. Null when idle; an object { sink } while flying. The
// sink (ft/s) is integrated into targetAgl each frame and morphs from the glide-
// slope rate into the exponential flare as the aircraft nears the ground.
let approach = null;

// Flare-fade multiplier, mirroring audio.c's s_flare_fade. 1 = tone fully
// present, 0 = faded out under the flare. Slewed every frame with the firmware's
// asymmetric rates: slow toward 0 below flareFadeFt, fast toward 1 above it.
let flareFade  = 1;

// --- Boot config (mirrors the firmware's hold-at-boot NVS menu) --------------
// audioMode is one of AUDIO_MODE_* (0..3). startAltFt is the callout ceiling: no
// callout ABOVE this height fires, exactly like app_main.c's s_start_alt_ft cap
// (the tone is unaffected). Both default to the firmware defaults at boot and are
// changed only by committing the in-sim config menu. Resolved flags are cached so
// the audio frame doesn't re-cross the WASM boundary every tick.
let audioMode   = 1;        // set to api.defaultMode() once the module is up
let startAltFt  = Infinity; // "no cap" until a profile is loaded (then = top callout)
let modeStereo  = true;     // resolved api.modeStereo(audioMode)
let modeCallouts = true;    // resolved api.modeCallouts(audioMode)
let modeTone    = true;     // resolved api.modeTone(audioMode)

// Master volume offset (dB, <= 0). The pilot's trim layered on the analog pot,
// attenuating the WHOLE mix (tone + voice) exactly like app_main.c's saved offset
// driving audio_set_master_db(). 0 dB == no cut; default seeded from the firmware.
let volumeDb    = 0;

// WebAudio graph.
let audioCtx     = null; // AudioContext (created on first gesture)
let osc          = null; // continuous OscillatorNode for the presence tone
let osc2         = null; // 2nd-harmonic oscillator (warmth) — runs at 2x osc
let osc2Gain     = null; // fixed gain on osc2 == TONE_HARMONIC2_LVL / (1 + lvl)
let toneGainNode = null; // GainNode — 0 == silent
let tonePanNode  = null; // StereoPannerNode — leans the tone left in stereo mode
let mixLpfNode   = null; // BiquadFilter (lowpass) — anti-harshness mix-bus LPF
let masterGainNode = null; // GainNode — the pilot's master volume offset (tone+voice)
let audioUnlocked = false;
const calloutBuffers = new Map(); // height(ft) -> decoded AudioBuffer

// Voice-duck envelope, mirroring audio.c's s_duck_cur. While a callout plays the
// tone ducks by VOICE_DUCK_DB with a fast ATTACK and a slow RELEASE; between
// callouts it sits at 1.0 (no duck). We slew it per frame by dt with the firmware's
// asymmetric rates so the first syllable is never masked yet the tone eases back
// without a pump. duckActiveUntil holds the audioCtx time a fired callout ends.
let duckCur        = 1;   // current duck multiplier (1 = un-ducked)
let duckActiveUntil = 0;  // audioCtx.currentTime by which the active callout ends

// Canvas / layout.
let canvas, ctx, dpr = 1;
let topFt = 300;         // top of the visible tape in feet (set per profile)

// Drag bookkeeping.
let dragging = false;

/* =============================================================================
 *  Boot
 * ===========================================================================*/

async function main() {
  // 1. Instantiate the WASM module. createLidarSim() returns a Promise<Module>.
  M = await createLidarSim();

  // 2. Wrap the flat ABI. cwrap(name, returnType, [argTypes]) gives a plain fn.
  const num = 'number';
  api = {
    setProfile:    M.cwrap('sim_set_profile',    null,  [num]),
    init:          M.cwrap('sim_init',           null,  [num]),
    initialState:  M.cwrap('sim_initial_state',  num,   [num, num]),
    step:          M.cwrap('sim_step',           null,  [num, num]),

    state:         M.cwrap('sim_state',          num,   []),
    firedHeightFt: M.cwrap('sim_fired_height_ft',num,   []),
    pollMs:        M.cwrap('sim_poll_ms',        num,   []),
    toneAgl:       M.cwrap('sim_tone_agl',       num,   []),
    toneActive:    M.cwrap('sim_tone_active',    num,   []),

    nCallouts:     M.cwrap('sim_n_callouts',     num,   []),
    calloutFt:     M.cwrap('sim_callout_ft',     num,   [num]),
    cruiseFt:      M.cwrap('sim_cruise_ft',      num,   []),
    maxRangeFt:    M.cwrap('sim_max_range_ft',   num,   []),

    pitchHz:       M.cwrap('sim_pitch_hz',       num,   [num]),
    toneDb:        M.cwrap('sim_tone_db',        num,   [num]),
    eqlDb:         M.cwrap('sim_eql_db',         num,   [num]),
    toneGain:      M.cwrap('sim_tone_gain',      num,   [num]),

    armFt:         M.cwrap('sim_arm_ft',         num,   []),
    toneStartFt:   M.cwrap('sim_tone_start_ft',  num,   []),
    toneFullFt:    M.cwrap('sim_tone_full_ft',   num,   []),
    flareHiFt:     M.cwrap('sim_flare_hi_ft',    num,   []),
    flareLoFt:     M.cwrap('sim_flare_lo_ft',    num,   []),

    flareFadeFt:     M.cwrap('sim_flare_fade_ft',     num, []),
    flareFadeOutMs:  M.cwrap('sim_flare_fade_out_ms', num, []),
    flareFadeInMs:   M.cwrap('sim_flare_fade_in_ms',  num, []),

    // Boot config menu — audio mode flags + stereo lean (see sim_glue.c).
    audioModeCount: M.cwrap('sim_audio_mode_count',   num, []),
    defaultMode:    M.cwrap('sim_default_audio_mode', num, []),
    stereoPan:      M.cwrap('sim_stereo_pan',         num, []),
    // Timbre shaping (warmth + anti-harshness), mirrored from audio.c (see config.h).
    toneHarmonic2:  M.cwrap('sim_tone_harmonic2',     num, []),
    mixLpfFc:       M.cwrap('sim_mix_lpf_fc',         num, []),
    modeStereo:     M.cwrap('sim_mode_stereo',        num, [num]),
    modeCallouts:   M.cwrap('sim_mode_callouts',      num, [num]),
    modeTone:       M.cwrap('sim_mode_tone',          num, [num]),

    // Master volume offset (boot config menu) — range/step/default + the SAME
    // db_to_gain() the firmware uses, so the master node matches the DAC path.
    volMinDb:       M.cwrap('sim_volume_offset_db_min',  num, []),
    volStepDb:      M.cwrap('sim_volume_offset_db_step', num, []),
    volDefaultDb:   M.cwrap('sim_default_volume_offset_db', num, []),
    masterGain:     M.cwrap('sim_master_gain',        num, [num]),

    // Volume-preview tone parameters ("tone .. number .. tone").
    volPreviewHz:   M.cwrap('sim_volume_preview_hz',  num, []),
    volPreviewMs:   M.cwrap('sim_volume_preview_ms',  num, []),
    volPreviewDb:   M.cwrap('sim_volume_preview_db',  num, []),

    // Voice-duck envelope + baseline tone trim, mirrored from audio.c.
    voiceDuckDb:    M.cwrap('sim_voice_duck_db',      num, []),
    duckAttackMs:   M.cwrap('sim_duck_attack_ms',     num, []),
    duckReleaseMs:  M.cwrap('sim_duck_release_ms',    num, []),
    toneTrimDb:     M.cwrap('sim_tone_trim_with_voice_db', num, []),

    // Ground-dwell disarm timeout (informational; the real SM enforces it).
    groundResetMs:  M.cwrap('sim_ground_reset_ms',    num, []),
  };

  // 3. Start the machine on the default profile, parked on the ground.
  //    Seed the boot config from the firmware's own defaults, then resolve the
  //    mode flags so the very first audio frame already honours them.
  audioMode = api.defaultMode();
  volumeDb  = api.volDefaultDb();      // master offset default (0 dB == no cut)
  applyAudioMode(audioMode);
  applyVolume(volumeDb);
  api.setProfile(profileIdx);
  api.init(0 /* ST_GROUND */);
  readConfig();

  // 4. Wire up the DOM and canvas, then start the loop.
  setupDom();
  setupCanvas();
  document.getElementById('loading').classList.add('hidden');
  document.getElementById('stage').classList.add('ready');
  requestAnimationFrame(frame);
}

/**
 * Cache the active profile's ladder + the firmware constants we visualise.
 * Re-run whenever the profile changes so the tape redraws against real data.
 */
function readConfig() {
  const callouts = [];
  const n = api.nCallouts();
  for (let i = 0; i < n; i++) callouts.push(api.calloutFt(i));

  cfg = {
    callouts,                       // descending heights, e.g. [200,100,...,10]
    cruiseFt:  api.cruiseFt(),
    maxRangeFt: api.maxRangeFt(),
    armFt:     api.armFt(),
    toneStart: api.toneStartFt(),
    toneFull:  api.toneFullFt(),
    flareHi:   api.flareHiFt(),
    flareLo:   api.flareLoFt(),

    // Flare fade-out: threshold + the two asymmetric ramp times (ms), straight
    // from the firmware so the emulated envelope matches the hardware timing.
    flareFadeFt:    api.flareFadeFt(),
    flareFadeOutMs: api.flareFadeOutMs(),
    flareFadeInMs:  api.flareFadeInMs(),
  };

  // Visible tape top: a little headroom above the highest callout / cruise so
  // the climb-out and CRUISE band are both on-screen.
  topFt = Math.max(cfg.callouts[0], cfg.cruiseFt) + 60;

  // Keep the target within the new range.
  targetAgl = clamp(targetAgl, 0, topFt);

  // Resolve the start-altitude cap against THIS profile's ladder, mirroring the
  // firmware's config_load_start_alt(top-callout) default. If the cap is still
  // unset (Infinity) OR sits above the new ladder's ceiling, snap it to the top
  // callout (= "no suppression"); a valid lower pick made in the menu survives a
  // profile switch as long as it's still on the ladder.
  const top = cfg.callouts[0];
  if (!isFinite(startAltFt) || startAltFt > top) {
    startAltFt = top;
  }
}

/* =============================================================================
 *  Audio mode (boot config) resolution
 * ===========================================================================*/

/**
 * Resolve one AUDIO_MODE_* index into the three behaviour flags the firmware
 * derives in audio_config_from_mode(), caching them so the audio frame never
 * re-crosses the WASM boundary. Also nudges the live tone panner so a mode change
 * takes effect immediately (no need to rebuild the graph).
 *
 * @param {number} mode  One of AUDIO_MODE_* (0..3).
 */
function applyAudioMode(mode) {
  audioMode    = mode;
  modeStereo   = api.modeStereo(mode)   === 1;
  modeCallouts = api.modeCallouts(mode) === 1;
  modeTone     = api.modeTone(mode)     === 1;

  // If the graph already exists, re-lean the tone panner now. The gain itself is
  // driven each frame and respects modeTone there, so nothing else to do here.
  if (tonePanNode && audioCtx) {
    tonePanNode.pan.setTargetAtTime(
      modeStereo ? -api.stereoPan() : 0, audioCtx.currentTime, 0.02);
  }
}

/**
 * Apply the pilot's master volume offset (dB) to the whole mix — the sim analogue
 * of app_main.c calling audio_set_master_db(). The offset is converted to a linear
 * gain by the SAME firmware db_to_gain() (via api.masterGain) and driven into the
 * master node so it trims the presence tone AND the voice callouts equally, just
 * like the DAC path. Clamped to attenuation only (never boosts past the schedule).
 *
 * @param {number} db  Offset in dB (<= 0).
 */
function applyVolume(db) {
  volumeDb = Math.min(0, db);
  if (masterGainNode && audioCtx) {
    // Short glide so a live change during the menu preview never clicks.
    masterGainNode.gain.setTargetAtTime(
      api.masterGain(volumeDb), audioCtx.currentTime, 0.02);
  }
}

/* =============================================================================
 *  Boot config menu (Calibrate button)
 *
 *  A faithful in-sim version of the firmware's hold-at-boot menu: choose an audio
 *  mode and the start-altitude cap, then Commit. Commit applies the picks and
 *  recalibrates the box to ground (0 ft), the sim analogue of the real menu
 *  rebooting to a clean ground reference.
 *
 *  PENDING picks live in these two vars while the sheet is open so Cancel can
 *  discard them without touching the live audio. Commit copies them across.
 * ===========================================================================*/

let pendingMode   = 1;   // mode highlighted in the open sheet (not yet applied)
let pendingCapFt  = 0;   // start-alt cap highlighted in the open sheet
let pendingVolDb  = 0;   // master volume offset highlighted in the open sheet

/** Open the config sheet, seeding the controls from the current live config. */
function openConfigMenu() {
  pendingMode  = audioMode;
  // Seed the cap from the live value, but never above this profile's ceiling.
  const top = cfg.callouts[0];
  pendingCapFt = isFinite(startAltFt) ? Math.min(startAltFt, top) : top;
  pendingVolDb = volumeDb;          // seed the master offset from the live value

  buildModeList();
  buildCapControl();
  buildVolumeControl();

  document.getElementById('configBackdrop').classList.add('open');
}

/** Close the sheet without applying anything. */
function closeConfigMenu() {
  document.getElementById('configBackdrop').classList.remove('open');
}

/** (Re)render the audio-mode option rows, marking the pending pick selected. */
function buildModeList() {
  const list = document.getElementById('modeList');
  list.innerHTML = '';

  const n = api.audioModeCount();
  for (let m = 0; m < n; m++) {
    const meta = AUDIO_MODE_META[m] ?? { title: `Mode ${m}`, desc: '' };

    const row = document.createElement('div');
    row.className = 'mode-opt' + (m === pendingMode ? ' sel' : '');
    row.innerHTML =
      `<div class="mo-check"></div>` +
      `<div class="mo-text"><div class="mo-title">${meta.title}</div>` +
      `<div class="mo-desc">${meta.desc}</div></div>`;

    // Selecting a mode updates the pending pick and re-skins the rows. The cap
    // control disables itself when the chosen mode plays no callouts (tone-only),
    // since a callout ceiling is meaningless with the voice muted.
    row.addEventListener('click', () => {
      pendingMode = m;
      buildModeList();
      refreshCapEnabled();
    });

    list.appendChild(row);
  }
  refreshCapEnabled();
}

/** Build the start-alt range against the active profile's callout ladder. */
function buildCapControl() {
  const range = document.getElementById('capRange');
  const ladder = cfg.callouts;                 // descending, e.g. [200,...,10]
  const top = ladder[0];

  // The slider walks discrete callout heights (ascending), so every stop is a
  // real ceiling. We store the chosen FT, not the index, to stay ladder-agnostic.
  const asc = [...ladder].sort((a, b) => a - b);
  range.min = 0;
  range.max = asc.length - 1;
  range.step = 1;

  // Find the slider index whose height is the pending cap (default = top).
  let idx = asc.indexOf(pendingCapFt);
  if (idx < 0) idx = asc.length - 1;           // fall back to the top callout
  range.value = idx;

  // Keep the ascending ladder around for the input handler.
  range._ascLadder = asc;
  updateCapLabel(asc[idx]);

  range.oninput = () => {
    const ft = range._ascLadder[parseInt(range.value, 10)];
    pendingCapFt = ft;
    updateCapLabel(ft);
  };

  refreshCapEnabled();
}

/** Show the cap value, flagging "no cap" when it sits at the profile top. */
function updateCapLabel(ft) {
  const top = cfg.callouts[0];
  const el = document.getElementById('capVal');
  el.textContent = ft >= top ? `${ft} ft · all` : `${ft} ft`;
}

/** Grey out the cap when the pending mode mutes callouts (cap is moot then). */
function refreshCapEnabled() {
  const capRow = document.getElementById('capRow');
  const calloutsOn = api.modeCallouts(pendingMode) === 1;
  capRow.classList.toggle('disabled', !calloutsOn);
}

/**
 * Apply the pending picks and recalibrate to ground. This is the sim analogue of
 * the firmware committing the menu and rebooting: the audio mode + cap take
 * effect, the state machine restarts cleanly on the ground, and the aircraft
 * snaps to 0 ft so the box is "zeroed" for the next approach.
 */
function commitConfigMenu() {
  applyAudioMode(pendingMode);
  startAltFt = pendingCapFt;

  calibrateToGround();          // zero the box (shared with the Calibrate path)
  closeConfigMenu();
}

/**
 * Recalibrate the box to ground: park at 0 ft, restart the state machine clean,
 * snap the glyph (no glide), clear the last-callout readout, and re-arm the flare
 * fade. Used by Commit and reusable elsewhere.
 */
function calibrateToGround() {
  cancelApproach();            // a zeroing always supersedes a running approach
  targetAgl  = 0;
  displayAgl = 0;              // snap the glyph instantly — this is a zeroing
  api.init(0 /* ST_GROUND */);
  lastFiredFt = null;
  flareFade = 1;
  duckCur = 1;                 // un-duck (no callout in flight after a zeroing)
  duckActiveUntil = 0;
}

/* =============================================================================
 *  Simulated approach
 *
 *  Kicks off a scripted, physics-driven landing: park just above CRUISE, then let
 *  the runner in stepApproach() walk targetAgl down the 3° glideslope and through
 *  the flare. The firmware does all the rest. Any manual input (drag, Reset, Jump
 *  to cruise, profile change, config commit) cancels the run via cancelApproach().
 * ===========================================================================*/

/**
 * Begin a simulated approach. Snaps the aircraft to the start altitude (just above
 * the profile's CRUISE band so the run opens high and clean), restarts the state
 * machine there, and arms the runner. Unlocks audio so you actually hear it.
 */
function startApproach() {
  unlockAudio();                          // first-class user gesture: enable sound

  // Start a hair above CRUISE so the firmware boots into CRUISE and then breaks
  // into DESCENT as we sink through the ladder — the full state arc on one run.
  const startFt = cfg.cruiseFt + APPROACH_START_MARGIN_FT;

  targetAgl  = startFt;
  displayAgl = startFt;                   // snap the glyph; the run glides from here
  api.init(0 /* ST_GROUND */);            // clean restart...
  api.step(startFt, 0.001);               // ...then seat the machine at altitude
  lastFiredFt = null;
  flareFade   = 1;                        // re-arm the flare fade for the descent

  approach = { sink: GLIDESLOPE_SINK };   // open on the glideslope sink rate
  updateApproachButton();
}

/** Stop a running approach and hand control back to the user. Safe if idle. */
function cancelApproach() {
  if (!approach) return;
  approach = null;
  updateApproachButton();
}

/**
 * Advance the scripted approach by dt seconds, driving targetAgl. Above the flare
 * we hold the constant glideslope sink; at/below FLARE_START_FT we relax the sink
 * exponentially toward a soft touchdown rate, reproducing the float-and-settle of
 * a real flare. When the wheels are effectively down we park at 0 and end the run.
 *
 * @param {number} dt  Frame time in seconds (already clamped by frame()).
 */
function stepApproach(dt) {
  if (!approach) return;

  // Choose the target sink rate for THIS height. On the glideslope it's constant;
  // inside the flare it eases from the current sink toward the touchdown sink with
  // a first-order (exponential) approach — the standard flare feel.
  let targetSink;
  if (targetAgl > FLARE_START_FT) {
    targetSink = GLIDESLOPE_SINK;
  } else {
    targetSink = FLARE_TOUCHDOWN_SINK;
  }

  // First-order relaxation of the live sink toward the target. The 1 - e^(-dt/tau)
  // factor makes the transition frame-rate independent and smoothly continuous as
  // we cross into the flare, so there's no kink in the descent.
  const k = 1 - Math.exp(-dt / FLARE_TAU);
  approach.sink += (targetSink - approach.sink) * k;

  // Integrate altitude. Sink is positive-down, so subtract it from the target AGL.
  targetAgl = Math.max(0, targetAgl - approach.sink * dt);

  // Touchdown: once we're on the ground, hold at 0 and let the run finish. We keep
  // approach alive for one settle frame so the final low callout + flare-fade tail
  // can play, then clear it on the next pass.
  if (targetAgl <= APPROACH_HOLD_FT) {
    targetAgl = 0;
    cancelApproach();
  }
}

/** Reflect the run state on the Fly-approach button (label + cancel affordance). */
function updateApproachButton() {
  const btn = document.getElementById('approachBtn');
  if (!btn) return;
  btn.textContent = approach ? 'Cancel approach' : 'Fly approach · 85 kt ILS';
  btn.classList.toggle('secondary', !!approach);
}

/* =============================================================================
 *  WebAudio
 * ===========================================================================*/

/**
 * Build the continuous tone graph once. The oscillator runs forever; silence is
 * just gain == 0. This mirrors the firmware, where the audio task always renders
 * and the tone's dB schedule decides audibility.
 */
function setupAudioGraph() {
  if (audioCtx) return;
  audioCtx = new (window.AudioContext || window.webkitAudioContext)();

  // Fundamental + a small 2nd harmonic, mirroring audio.c::tone_sample(): an
  // EVEN overtone one octave up adds "warmth" so the tone reads as a voice, not
  // a sterile test beep. Both feed the same gain node so the dB schedule, pan
  // and gating treat them as one stream. The pair is normalised by (1 + level)
  // exactly like the firmware, so adding body never changes the peak loudness.
  const h2 = api.toneHarmonic2();           // TONE_HARMONIC2_LVL
  const norm = 1 / (1 + h2);

  osc = audioCtx.createOscillator();
  osc.type = 'sine';                 // pure fundamental, like the firmware NCO
  osc.frequency.value = 600;         // F_AT_TONE_START; updated every frame

  osc2 = audioCtx.createOscillator();
  osc2.type = 'sine';                // 2nd harmonic, also a pure sine
  osc2.frequency.value = 1200;       // 2 x F_AT_TONE_START; tracked every frame
  osc2Gain = audioCtx.createGain();
  osc2Gain.gain.value = h2 * norm;   // matches the lvl/(1+lvl) weight in C

  toneGainNode = audioCtx.createGain();
  toneGainNode.gain.value = 0;       // start silent

  // Stereo lean for the tone. The firmware pans the TONE to the LEFT by
  // STEREO_PAN in STEREO_BOTH and centres it in every mono mode; a StereoPanner
  // reproduces that. pan = -STEREO_PAN (left) when stereo, 0 (centre) otherwise.
  tonePanNode = audioCtx.createStereoPanner();
  tonePanNode.pan.value = modeStereo ? -api.stereoPan() : 0;

  // Anti-harshness mix-bus LPF, mirroring the firmware's 1-pole filter: rounds
  // the high end of the sweep so it is silk, not glass. A gentle 1st-order
  // (12 dB/oct would be Q-peaky; the default Q ~0.707 lowpass is the closest
  // WebAudio analogue to the firmware's one-pole).
  mixLpfNode = audioCtx.createBiquadFilter();
  mixLpfNode.type = 'lowpass';
  mixLpfNode.frequency.value = api.mixLpfFc();   // MIX_LPF_FC_HZ

  // Master volume offset node — the pilot's trim layered on the analog pot. The
  // firmware applies it LAST, to the whole mix (tone AND voice), so in the graph
  // BOTH the tone chain and every callout source feed THIS node, and only this
  // node reaches the destination. Mirrors audio_set_master_db()/s_master_gain.
  masterGainNode = audioCtx.createGain();
  masterGainNode.gain.value = api.masterGain(volumeDb);
  masterGainNode.connect(audioCtx.destination);

  // Fundamental feeds the gain node at the normalised weight; the harmonic feeds
  // it through osc2Gain. Then: gain -> pan -> LPF -> master -> out (LPF is
  // post-pan so it shapes the final mix exactly like audio.c filters the L/R
  // after panning; master is post-LPF, the true master trim).
  const fundGain = audioCtx.createGain();
  fundGain.gain.value = norm;        // fundamental weight (1/(1+lvl))
  osc.connect(fundGain).connect(toneGainNode);
  osc2.connect(osc2Gain).connect(toneGainNode);
  toneGainNode.connect(tonePanNode).connect(mixLpfNode).connect(masterGainNode);
  osc.start();
  osc2.start();
}

/**
 * Decode every callout WAV once into an AudioBuffer keyed by its height. A
 * missing file is tolerated (skipped + logged) so a partial asset set still runs.
 */
async function loadCallouts() {
  await Promise.all(ALL_CALLOUT_FT.map(async (ft) => {
    try {
      const res = await fetch(`${AUDIO_BASE}/${ft}.wav`);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const buf = await audioCtx.decodeAudioData(await res.arrayBuffer());
      calloutBuffers.set(ft, buf);
    } catch (e) {
      console.warn(`callout ${ft}.wav unavailable:`, e.message);
    }
  }));
}

/** Fire a one-shot callout clip at the given height (no-op if not loaded). */
function playCallout(ft) {
  const buf = calloutBuffers.get(ft);
  if (!buf || !audioCtx) return;

  // Callouts are gated by the configured audio mode: MONO_TONE silences voice.
  if (!modeCallouts) return;

  const src = audioCtx.createBufferSource();
  src.buffer = buf;

  // Voice leans RIGHT in stereo (mirror of the tone's left lean); centred in mono.
  const pan = audioCtx.createStereoPanner();
  pan.pan.value = modeStereo ? +api.stereoPan() : 0;

  // Through the master node (post-pan) so the pilot's volume offset trims the
  // voice exactly as it does the tone — the firmware applies master to both.
  src.connect(pan).connect(masterGainNode);
  src.start();

  // Open the duck window for this clip's duration so the frame loop ducks the
  // tone under it (fast attack now, slow release once it ends), mirroring the
  // firmware's s_duck_cur envelope. buf.duration is the real clip length.
  duckActiveUntil = audioCtx.currentTime + buf.duration;
}

/**
 * Unlock audio on the first user gesture (autoplay policy). Idempotent: safe to
 * call from the Start button AND from the first drag.
 */
async function unlockAudio() {
  if (audioUnlocked) return;
  setupAudioGraph();
  await audioCtx.resume();
  await loadCallouts();
  audioUnlocked = true;
  document.getElementById('audioGate').classList.add('hidden');
}

/* =============================================================================
 *  Main loop
 * ===========================================================================*/

function frame(ts) {
  // dt in seconds. Clamp the upper bound exactly like the firmware logic_task
  // guards against a stalled tick — a backgrounded tab must not inject a huge
  // dt that would poison the trend (vertical-rate) estimate inside sm_step.
  let dt = (ts - lastTs) / 1000;
  lastTs = ts;
  if (!(dt > 0)) dt = 0.001;        // first frame / clock weirdness
  if (dt > 0.1) dt = 0.1;           // ~100 ms ceiling

  // Drive the scripted approach (if any) BEFORE stepping the machine, so the
  // firmware sees the freshly-integrated glideslope/flare altitude this frame.
  stepApproach(dt);

  // Step the REAL state machine with the raw target altitude.
  api.step(targetAgl, dt);

  // Read the decision back out.
  const st       = api.state();
  const firedFt  = api.firedHeightFt();
  const toneAgl  = api.toneAgl();
  const toneOn   = api.toneActive() === 1;
  const pollMs   = api.pollMs();

  // --- Audio: callout + tone ------------------------------------------------
  // Suppress any callout ABOVE the configured start-altitude cap, exactly like
  // app_main.c (the tone is unaffected). With the cap at the profile top this
  // never suppresses. The readout still reflects the last AUDIBLE callout only.
  if (firedFt >= 0 && firedFt <= startAltFt) {
    lastFiredFt = firedFt;
    if (audioUnlocked) playCallout(firedFt);
  }

  // --- Flare fade: mirror audio.c's s_flare_fade ----------------------------
  // Below flareFadeFt the tone fades OUT (target 0) over flareFadeOutMs; at/above
  // it restores (target 1) over the quick flareFadeInMs. We slew per frame by
  // dt/rampSeconds, the frame-stepped equivalent of the firmware's per-sample
  // slew, so a bounce back above the threshold reverses the envelope at once.
  // When the tone is off entirely (climbed away) we hold it re-armed at full,
  // matching the firmware re-arming s_flare_fade on light-sleep suspend.
  const fadeTarget = (toneOn && toneAgl < cfg.flareFadeFt) ? 0 : 1;
  const fadeMs     = fadeTarget < flareFade ? cfg.flareFadeOutMs
                                            : cfg.flareFadeInMs;
  const fadeStep   = fadeMs > 0 ? dt / (fadeMs / 1000) : 1;
  flareFade += clamp(fadeTarget - flareFade, -fadeStep, fadeStep);

  // --- Voice duck: mirror audio.c's asymmetric s_duck_cur envelope ----------
  // While a callout is sounding (we tracked its end in duckActiveUntil) the tone
  // ducks toward db_to_gain(-VOICE_DUCK_DB); otherwise it releases back to 1.0.
  // The ATTACK (fast) and RELEASE (slow) use the firmware's own ms constants,
  // slewed per frame by dt — the frame-stepped equivalent of the per-sample slew
  // — so the first syllable is cleared instantly yet the tone eases back without
  // a pump. Only meaningful when callouts can play; otherwise it stays at 1.0.
  if (audioUnlocked) {
    const ducking   = modeCallouts && audioCtx.currentTime < duckActiveUntil;
    const duckTarget = ducking ? api.masterGain(-api.voiceDuckDb()) : 1; // db->gain
    const duckMs    = duckTarget < duckCur ? api.duckAttackMs()
                                           : api.duckReleaseMs();
    const duckStep  = duckMs > 0 ? dt / (duckMs / 1000) : 1;
    duckCur += clamp(duckTarget - duckCur, -duckStep, duckStep);
  }

  if (audioUnlocked) {
    const now = audioCtx.currentTime;
    // The tone only sounds when the configured mode enables it (MONO_CALLOUTS
    // silences it entirely). Gate here so the scheduled gain never reaches the
    // node in a tone-disabled mode, mirroring audio.c's tone_enabled flag.
    if (toneOn && modeTone) {
      // Drive the oscillator from the SAME math the firmware uses. setTargetAtTime
      // gives a short, click-free glide analogous to the raised-cosine envelope.
      const fHz = api.pitchHz(toneAgl);
      osc.frequency.setTargetAtTime(fHz, now, 0.01);
      // Keep the 2nd-harmonic oscillator locked to 2x the fundamental as it glides.
      osc2.frequency.setTargetAtTime(fHz * 2, now, 0.01);

      // Steady baseline tone trim while callouts are enabled (TONE_TRIM_WITH_VOICE
      // _DB): holds the tone a touch down for the whole descent so the voice reads
      // clearer over it — exactly audio.c's s_tone_trim (1.0 / no trim otherwise).
      const toneTrim = modeCallouts ? api.masterGain(api.toneTrimDb()) : 1;

      // Final tone gain = scheduled gain · flare fade · voice duck · baseline trim,
      // the same stack of multipliers the firmware applies per sample (the master
      // volume offset lives on masterGainNode, downstream of this node).
      toneGainNode.gain.setTargetAtTime(
        api.toneGain(toneAgl) * flareFade * duckCur * toneTrim, now, 0.01);
    } else {
      // Ramp to silence rather than cutting — no click.
      toneGainNode.gain.setTargetAtTime(0, now, 0.03);
    }
  }

  // --- Visuals: ease the glyph toward target, then paint --------------------
  // During a scripted approach the target moves smoothly already, so track it
  // tightly (a hard ease would trail a fast descent); otherwise keep the gentle
  // glide that makes manual drags feel fluid.
  const ease = approach ? 0.6 : DISPLAY_EASE;
  displayAgl += (targetAgl - displayAgl) * ease;
  draw(st, toneAgl, toneOn, pollMs);
  updateTelemetry(st, toneAgl, toneOn, pollMs);

  requestAnimationFrame(frame);
}

/* =============================================================================
 *  Telemetry panel
 * ===========================================================================*/

function updateTelemetry(st, toneAgl, toneOn, pollMs) {
  const name = STATE_NAMES[st] ?? '—';
  const pill = document.getElementById('statePill');
  const dot  = document.getElementById('stateDot');

  document.getElementById('stateName').textContent = name;
  pill.style.color = STATE_COLORS[name] ?? 'var(--text)';
  // The dot lights green whenever the box is "armed-and-watching" (any flying
  // state). GROUND/CLIMB leave it dim.
  dot.classList.toggle('on', name === 'ARMED' || name === 'CRUISE' || name === 'DESCENT');

  document.getElementById('aglVal').textContent = `${Math.round(targetAgl)} ft`;

  // Trend: infer a friendly arrow from display vs target (purely cosmetic).
  const dv = targetAgl - displayAgl;
  const trend = Math.abs(dv) < 0.3 ? 'level'
              : dv > 0 ? '▲ climbing' : '▼ descending';
  document.getElementById('trendVal').textContent = trend;

  document.getElementById('toneVal').textContent = toneOn ? 'active' : 'off';
  document.getElementById('toneVal').style.color = toneOn ? 'var(--good)' : 'var(--text-dim)';

  // Show pitch/level only while the tone is meaningful (at/below TONE_START).
  if (toneOn) {
    document.getElementById('hzVal').textContent = `${Math.round(api.pitchHz(toneAgl))} Hz`;
    document.getElementById('dbVal').textContent = `${api.toneDb(toneAgl).toFixed(1)} dB`;

    // Equal-loudness (Fletcher-Munson) correction, shown SEPARATELY from the
    // scheduled level so you can watch the ISO 226 flattening track the pitch
    // sweep. Force an explicit sign: + boosts (ear less sensitive here), - cuts
    // (ear more sensitive). The signed default for non-negative is '+'.
    const eql = api.eqlDb(toneAgl);
    const sign = eql >= 0 ? '+' : '−';        // U+2212 minus for crisp typography
    document.getElementById('eqlVal').textContent = `${sign}${Math.abs(eql).toFixed(2)} dB`;
  } else {
    document.getElementById('hzVal').textContent = '— Hz';
    document.getElementById('dbVal').textContent = '— dB';
    document.getElementById('eqlVal').textContent = '— dB';
  }

  document.getElementById('pollVal').textContent = `${pollMs} ms`;
  document.getElementById('calloutVal').textContent =
    lastFiredFt == null ? '—' : `${Math.round(lastFiredFt)} ft`;

  // Reflect the live boot-config picks so a Commit visibly changes the panel.
  const modeMeta = AUDIO_MODE_META[audioMode];
  document.getElementById('modeVal').textContent = modeMeta ? modeMeta.title : `mode ${audioMode}`;

  // Cap: "all" when it sits at (or above) the profile top, else the height.
  const capTop = cfg.callouts[0];
  document.getElementById('capValTelem').textContent =
    (!isFinite(startAltFt) || startAltFt >= capTop) ? 'all' : `${Math.round(startAltFt)} ft`;
}

/* =============================================================================
 *  Canvas: the altitude tape
 * ===========================================================================*/

function setupCanvas() {
  canvas = document.getElementById('tape');
  ctx = canvas.getContext('2d');
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  // Pointer-driven flying: map the pointer's Y on the tape back to feet.
  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  window.addEventListener('pointerup', onPointerUp);
}

function resizeCanvas() {
  dpr = Math.max(1, window.devicePixelRatio || 1);
  const r = canvas.getBoundingClientRect();
  // Back the canvas with a DPR-scaled bitmap so the tape is retina-crisp.
  canvas.width  = Math.round(r.width  * dpr);
  canvas.height = Math.round(r.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // draw in CSS pixels
}

// Map an altitude (ft) to a Y pixel on the tape (0 ft at bottom, topFt at top).
function ftToY(ft, h) {
  const pad = 24;                          // top/bottom breathing room
  const usable = h - pad * 2;
  const t = clamp(ft / topFt, 0, 1);       // 0..1 up the tape
  return pad + (1 - t) * usable;
}
// Inverse: a Y pixel back to altitude (for dragging).
function yToFt(y, h) {
  const pad = 24;
  const usable = h - pad * 2;
  const t = clamp((y - pad) / usable, 0, 1);
  return (1 - t) * topFt;
}

function draw(st, toneAgl, toneOn) {
  const w = canvas.width / dpr;
  const h = canvas.height / dpr;
  ctx.clearRect(0, 0, w, h);

  // Tape gutter where the altitude line + ticks live (left third); the aircraft
  // and bands span the full width.
  const tapeX = Math.min(64, w * 0.18);

  // --- Shaded altitude bands (drawn bottom-most) ----------------------------
  // Tone swell band: TONE_START_FT -> ground, faint blue that deepens lower.
  fillBand(0, cfg.toneStart, h, w, 'rgba(10,132,255,0.06)');
  // Full-presence band: at/below TONE_FULL_FT.
  fillBand(0, cfg.toneFull, h, w, 'rgba(10,132,255,0.05)');
  // Flare full-attention band: a warmer accent at the business end.
  fillBand(cfg.flareLo, cfg.flareHi, h, w, 'rgba(255,159,10,0.10)');

  // --- The vertical altitude line ------------------------------------------
  ctx.strokeStyle = 'rgba(255,255,255,0.18)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(tapeX, ftToY(topFt, h));
  ctx.lineTo(tapeX, ftToY(0, h));
  ctx.stroke();

  // Ground line.
  drawHLine(ftToY(0, h), w, 'rgba(255,255,255,0.35)', 1.5);
  label(tapeX + 10, ftToY(0, h) - 6, 'GROUND', 'rgba(255,255,255,0.45)', 11);

  // --- Callout tick marks + labels -----------------------------------------
  ctx.font = '600 12px -apple-system, system-ui, sans-serif';
  for (const ft of cfg.callouts) {
    const y = ftToY(ft, h);
    // A short tick on the line plus the number to its left.
    ctx.strokeStyle = 'rgba(255,255,255,0.30)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(tapeX - 7, y);
    ctx.lineTo(tapeX + 7, y);
    ctx.stroke();

    ctx.fillStyle = 'rgba(245,246,250,0.78)';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${ft}`, tapeX - 12, y);
  }

  // --- ARM line (dashed) and CRUISE line -----------------------------------
  drawDashed(ftToY(cfg.armFt, h), w, 'rgba(48,209,88,0.55)');
  label(w - 8, ftToY(cfg.armFt, h) - 6, `ARM ${cfg.armFt}`, 'rgba(48,209,88,0.8)', 11, 'right');

  drawHLine(ftToY(cfg.cruiseFt, h), w, 'rgba(100,210,255,0.45)', 1);
  label(w - 8, ftToY(cfg.cruiseFt, h) - 6, `CRUISE ${cfg.cruiseFt}`, 'rgba(100,210,255,0.85)', 11, 'right');

  // --- The aircraft glyph ---------------------------------------------------
  const acY = ftToY(displayAgl, h);
  drawAircraft(tapeX, acY, STATE_COLORS[STATE_NAMES[st]] ?? '#fff', toneOn);

  // Altitude readout floating by the glyph.
  ctx.fillStyle = '#f5f6fa';
  ctx.font = '700 16px -apple-system, system-ui, sans-serif';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.fillText(`${Math.round(targetAgl)} ft`, tapeX + 22, acY);
}

/* ---- Canvas drawing helpers --------------------------------------------- */

function fillBand(loFt, hiFt, h, w, color) {
  const yLo = ftToY(loFt, h);
  const yHi = ftToY(hiFt, h);
  ctx.fillStyle = color;
  ctx.fillRect(0, yHi, w, yLo - yHi);
}

function drawHLine(y, w, color, lw) {
  ctx.strokeStyle = color; ctx.lineWidth = lw;
  ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
}

function drawDashed(y, w, color) {
  ctx.save();
  ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.setLineDash([6, 5]);
  ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  ctx.restore();
}

function label(x, y, text, color, size, align = 'left') {
  ctx.fillStyle = color;
  ctx.font = `600 ${size}px -apple-system, system-ui, sans-serif`;
  ctx.textAlign = align;
  ctx.textBaseline = 'alphabetic';
  ctx.fillText(text, x, y);
}

// A simple side-view aircraft chevron; pulses a soft halo while the tone sounds.
function drawAircraft(x, y, color, toneOn) {
  if (toneOn) {
    ctx.save();
    ctx.fillStyle = 'rgba(255,159,10,0.18)';
    ctx.beginPath();
    ctx.arc(x, y, 16, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }
  ctx.save();
  ctx.translate(x, y);
  ctx.fillStyle = color;
  ctx.beginPath();
  // Little nose-down chevron pointing right.
  ctx.moveTo(-9, -7);
  ctx.lineTo(9, 0);
  ctx.lineTo(-9, 7);
  ctx.lineTo(-5, 0);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

/* =============================================================================
 *  Input: dragging the aircraft on the tape
 * ===========================================================================*/

function onPointerDown(e) {
  dragging = true;
  canvas.classList.add('dragging');
  canvas.setPointerCapture?.(e.pointerId);
  cancelApproach();              // grabbing the aircraft takes over from the script
  unlockAudio();                 // first interaction also unlocks sound
  setTargetFromPointer(e);
}
function onPointerMove(e) {
  if (!dragging) return;
  setTargetFromPointer(e);
}
function onPointerUp() {
  dragging = false;
  canvas.classList.remove('dragging');
}
function setTargetFromPointer(e) {
  const r = canvas.getBoundingClientRect();
  const y = e.clientY - r.top;
  targetAgl = clamp(yToFt(y, r.height), 0, topFt);
}

/* =============================================================================
 *  DOM wiring
 * ===========================================================================*/

function setupDom() {
  document.getElementById('startBtn').addEventListener('click', unlockAudio);

  document.getElementById('profileSel').addEventListener('change', (e) => {
    cancelApproach();              // ladder is changing out from under the run
    profileIdx = parseInt(e.target.value, 10) || 0;
    api.setProfile(profileIdx);
    api.init(0 /* ST_GROUND */);   // clean restart under the new ladder
    lastFiredFt = null;
    flareFade = 1;                 // re-arm the flare fade for the fresh run
    readConfig();
  });

  document.getElementById('resetBtn').addEventListener('click', () => {
    cancelApproach();
    targetAgl = 0;
    api.init(0 /* ST_GROUND */);
    lastFiredFt = null;
    flareFade = 1;                 // re-arm the flare fade
  });

  document.getElementById('cruiseBtn').addEventListener('click', () => {
    cancelApproach();
    // Snap the aircraft up to just above cruise so you can demo a full approach.
    targetAgl = cfg.cruiseFt + 20;
    displayAgl = targetAgl;        // jump the glyph too, no long glide
  });

  // Fly approach: a one-tap scripted 85 kt / 3° ILS descent + flare. Toggles to
  // a cancel control while running so a second tap hands control back.
  document.getElementById('approachBtn').addEventListener('click', () => {
    if (approach) cancelApproach();
    else          startApproach();
  });

  // --- Config menu (Calibrate) --------------------------------------------
  document.getElementById('calibrateBtn').addEventListener('click', openConfigMenu);
  document.getElementById('cfgCancelBtn').addEventListener('click', closeConfigMenu);
  document.getElementById('cfgCommitBtn').addEventListener('click', commitConfigMenu);

  // Click the dimmed backdrop (but not the sheet itself) to dismiss.
  document.getElementById('configBackdrop').addEventListener('click', (e) => {
    if (e.target.id === 'configBackdrop') closeConfigMenu();
  });

  // Esc closes the sheet too, matching native sheet behaviour.
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') closeConfigMenu();
  });
}

/* =============================================================================
 *  Utilities
 * ===========================================================================*/

function clamp(v, lo, hi) { return v < lo ? lo : v > hi ? hi : v; }

// Go.
main().catch((err) => {
  console.error(err);
  const el = document.getElementById('loading');
  if (el) { el.textContent = 'Failed to load firmware — check the console.'; }
});
