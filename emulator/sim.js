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
const ALL_CALLOUT_FT = [600, 500, 400, 300, 200, 100, 50, 40, 30, 20, 10];

// Spoken config-menu prompt pieces, mirroring the firmware's config_clip_piece
// set (see callouts.c). Keyed by the same names the firmware uses; the value is
// the WAV filename in original_audio/. A piece with no WAV master yet (mono,
// config_mode are only .pcm) is simply skipped when spoken — exactly the
// graceful "absent clip" handling the firmware does for unembedded clips.
const CONFIG_PIECE_WAV = {
  config_mode:        'config_mode.wav',          // (no master yet -> skipped)
  chirp:              'chirp.wav',
  mono:               'mono.wav',                  // (no master yet -> skipped)
  stereo:             'stereo.wav',
  callouts_and_tone:  'callouts_and_tone.wav',
  callouts_only:      'callouts_only.wav',
  tone_only:          'tone_only.wav',
  start_alt:          'callout_start_altitude.wav',
  volume_adjustment:  'volume_adjustment.wav',
};

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

// Where the scripted approach begins, in feet AGL. A fixed default start height so
// the run opens high, breaks into DESCENT through the ladder, and ends on the
// ground. 342 ft sits just above the SF30/C cruise band (305 ft), so that profile
// still opens in CRUISE; on the longer-range SF30/D it opens armed-and-descending.
const APPROACH_START_FT = 342.0;

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

// Independent tone + voice volume offsets (dB). The pilot's two trims layered on
// the analog pot: the TONE offset can cut OR boost (audio_set_tone_db), the VOICE
// offset is cut-only (audio_set_voice_db). 0 dB == no change; defaults seeded from
// the firmware. Each drives its own WebAudio gain node so the sim balances the two
// streams exactly like the box.
let toneVolDb   = 0;
let voiceVolDb  = 0;

// WebAudio graph.
let audioCtx     = null; // AudioContext (created on first gesture)
let osc          = null; // continuous OscillatorNode for the presence tone
let osc2         = null; // 2nd-harmonic oscillator (warmth) — runs at 2x osc
let osc2Gain     = null; // fixed gain on osc2 == TONE_HARMONIC2_LVL / (1 + lvl)
let toneGainNode = null; // GainNode — 0 == silent
let blipGainNode = null; // GainNode — the vario blip gate (1 == open, 0 == silence)
let tonePanNode  = null; // StereoPannerNode — leans the tone left in stereo mode
let mixLpfNode   = null; // BiquadFilter (lowpass) — anti-harshness mix-bus LPF
let toneVolNode  = null; // GainNode — the pilot's TONE volume offset (cut or boost)
let voiceVolNode = null; // GainNode — the pilot's VOICE volume offset (cut only)
let audioUnlocked = false;
const calloutBuffers = new Map(); // height(ft) -> decoded AudioBuffer
const calloutEnv     = new Map(); // height(ft) -> Float32Array sidechain envelope
const configBuffers  = new Map(); // piece name -> decoded AudioBuffer (menu prompts)

// Voice-SIDECHAIN duck, mirroring audio.c's compressor. The firmware ducks the
// tone in proportion to the voice's ACTUAL loudness (a one-pole follower over the
// clip samples), not a fixed level — so the leading edge of a word eases the tone
// down WITH the syllable instead of clipping it. We reproduce this exactly: each
// callout's envelope is precomputed with the SAME follower (precomputeEnv), and
// while it plays we read env[playbackPosition] and map it through the same soft
// knee to a duck multiplier. activeVoice tracks the currently-sounding clip.
let duckCur     = 1;      // current duck multiplier (1 = un-ducked), for readout
let activeVoice = null;   // { env, startTime, durSamples } of the playing callout, or null

// Vario "blip", mirroring audio.c's blip_advance_gate. The two enables are the
// firmware's LEVEL 7/8 toggles (NVS sink/climb-rate). The gate is a slow square
// wave with asymmetric, rate-driven half-periods: blipInBeep says which half we
// are in and blipNextFlip is the audioCtx time of the next flip. With sink off the
// gate is held open (1.0) so the tone is the steady sound it has always been.
let sinkRateOn  = false;  // LEVEL 7 sink mode: blips track descent (mutually excl.)
let climbRateOn = false;  // LEVEL 7 climb mode: blips track climb (the inverse)
let blipInBeep  = false;  // current half (false => next flip opens on a beep)
let blipNextFlip = 0;     // audioCtx time of the next beep<->silence flip
let blipRateFps = 0;      // smoothed rate driving the cadence (eased toward vertFps)

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
    eqlDbHz:       M.cwrap('sim_eql_db_hz',      num,   [num]),
    toneGain:      M.cwrap('sim_tone_gain',      num,   [num]),

    armFt:         M.cwrap('sim_arm_ft',         num,   []),
    toneStartFt:   M.cwrap('sim_tone_start_ft',  num,   []),
    toneFullFt:    M.cwrap('sim_tone_full_ft',   num,   []),
    flareHiFt:     M.cwrap('sim_flare_hi_ft',    num,   []),
    flareLoFt:     M.cwrap('sim_flare_lo_ft',    num,   []),

    flareFadeFt:     M.cwrap('sim_flare_fade_ft',     num, []),
    flareFadeOutMs:  M.cwrap('sim_flare_fade_out_ms', num, []),
    flareFadeInMs:   M.cwrap('sim_flare_fade_in_ms',  num, []),

    // Vario "blip": the live vertical rate + the cadence tunables, so the JS blip
    // scheduler chops the tone with exactly the firmware's mapping (bpm linear in
    // vertical rate, anchored to two measured points).
    vertFps:         M.cwrap('sim_vert_fps',          num, []),
    varioOnsetFpm:   M.cwrap('sim_vario_onset_fpm',   num, []),
    varioFullFpm:    M.cwrap('sim_vario_full_fpm',    num, []),
    varioBpmMax:     M.cwrap('sim_vario_bpm_max',     num, []),
    varioBeepFactor: M.cwrap('sim_vario_beep_factor', num, []),
    varioEdgeMs:     M.cwrap('sim_vario_edge_ms',     num, []),
    varioRateSmoothMs:M.cwrap('sim_vario_rate_smooth_ms', num, []),

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

    // Independent tone + voice volume offsets (boot config menu) — ranges/steps/
    // defaults + the SAME clamped db_to_gain() the firmware uses, so each WebAudio
    // volume node matches the DAC path.
    toneVolMinDb:    M.cwrap('sim_tone_volume_db_min',    num, []),
    toneVolMaxDb:    M.cwrap('sim_tone_volume_db_max',    num, []),
    toneVolStepDb:   M.cwrap('sim_tone_volume_db_step',   num, []),
    toneVolDefaultDb:M.cwrap('sim_default_tone_volume_db',num, []),
    toneVolGain:     M.cwrap('sim_tone_volume_gain',      num, [num]),
    voiceVolMinDb:   M.cwrap('sim_voice_volume_db_min',   num, []),
    voiceVolStepDb:  M.cwrap('sim_voice_volume_db_step',  num, []),
    voiceVolDefaultDb:M.cwrap('sim_default_voice_volume_db',num, []),
    voiceVolGain:    M.cwrap('sim_voice_volume_gain',     num, [num]),
    dbToGain:        M.cwrap('sim_db_to_gain',            num, [num]),

    // Volume-preview tone parameters ("tone .. number .. tone").
    volPreviewHz:   M.cwrap('sim_volume_preview_hz',  num, []),
    volPreviewMs:   M.cwrap('sim_volume_preview_ms',  num, []),
    volPreviewDb:   M.cwrap('sim_volume_preview_db',  num, []),

    // Voice-duck envelope + baseline tone trim, mirrored from audio.c.
    voiceDuckDb:    M.cwrap('sim_voice_duck_db',      num, []),
    duckAttackMs:   M.cwrap('sim_duck_attack_ms',     num, []),
    duckReleaseMs:  M.cwrap('sim_duck_release_ms',    num, []),
    duckThreshold:  M.cwrap('sim_duck_threshold',     num, []),
    duckKneeLevel:  M.cwrap('sim_duck_knee_level',    num, []),
    duckFloor:      M.cwrap('sim_duck_floor',         num, []),
    toneTrimDb:     M.cwrap('sim_tone_trim_with_voice_db', num, []),

    // Ground-dwell disarm timeout (informational; the real SM enforces it).
    groundResetMs:  M.cwrap('sim_ground_reset_ms',    num, []),
  };

  // 3. Start the machine on the default profile, parked on the ground.
  //    Seed the boot config from the firmware's own defaults, then resolve the
  //    mode flags so the very first audio frame already honours them.
  audioMode = api.defaultMode();
  toneVolDb  = api.toneVolDefaultDb();   // tone offset default (0 dB == no change)
  voiceVolDb = api.voiceVolDefaultDb();  // voice offset default (0 dB == no cut)
  applyAudioMode(audioMode);
  applyToneVolume(toneVolDb);
  applyVoiceVolume(voiceVolDb);
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

    // Vario blip cadence tunables (mirror config.h VARIO_*). The JS scheduler turns
    // the live vertical rate/accel into beep/silence lengths with this same mapping.
    varioOnsetFpm:   api.varioOnsetFpm(),
    varioFullFpm:    api.varioFullFpm(),
    varioBpmMax:     api.varioBpmMax(),
    varioBeepFactor: api.varioBeepFactor(),
    varioEdgeMs:     api.varioEdgeMs(),
    varioRateSmoothMs: api.varioRateSmoothMs(),
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
 * Apply the pilot's TONE volume offset (dB) — the sim analogue of app_main.c calling
 * audio_set_tone_db(). Converted to a linear gain by the SAME firmware-side clamp +
 * db_to_gain() (via api.toneVolGain) and driven into the tone-path node, so it trims
 * (or boosts) ONLY the presence tone, just like the DAC path.
 *
 * @param {number} db  Tone offset in dB (may be negative or positive).
 */
function applyToneVolume(db) {
  // Clamp to the firmware's tone range so the readout/state never drift past it.
  toneVolDb = Math.max(api.toneVolMinDb(), Math.min(api.toneVolMaxDb(), db));
  if (toneVolNode && audioCtx) {
    // Short glide so a live change during the menu preview never clicks.
    toneVolNode.gain.setTargetAtTime(
      api.toneVolGain(toneVolDb), audioCtx.currentTime, 0.02);
  }
}

/**
 * Apply the pilot's VOICE volume offset (dB) — the sim analogue of app_main.c calling
 * audio_set_voice_db(). Cut-only (clamped to <= 0), converted to a linear gain by the
 * SAME firmware-side db_to_gain() (via api.voiceVolGain) and driven into the voice-path
 * node, so it trims ONLY the spoken callouts.
 *
 * @param {number} db  Voice offset in dB (<= 0).
 */
function applyVoiceVolume(db) {
  voiceVolDb = Math.max(api.voiceVolMinDb(), Math.min(0, db));
  if (voiceVolNode && audioCtx) {
    voiceVolNode.gain.setTargetAtTime(
      api.voiceVolGain(voiceVolDb), audioCtx.currentTime, 0.02);
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

let pendingMode      = 1;   // mode highlighted in the open sheet (not yet applied)
let pendingCapFt     = 0;   // start-alt cap highlighted in the open sheet
let pendingToneVolDb = 0;   // tone volume offset highlighted in the open sheet
let pendingVoiceVolDb= 0;   // voice volume offset highlighted in the open sheet

/** Open the config sheet, seeding the controls from the current live config. */
function openConfigMenu() {
  pendingMode  = audioMode;
  // Seed the cap from the live value, but never above this profile's ceiling.
  const top = cfg.callouts[0];
  pendingCapFt = isFinite(startAltFt) ? Math.min(startAltFt, top) : top;
  pendingToneVolDb  = toneVolDb;    // seed both offsets from the live values
  pendingVoiceVolDb = voiceVolDb;

  buildModeList();
  buildCapControl();
  buildToneVolumeControl();
  buildVoiceVolumeControl();

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
 * Build the TONE-volume range against the firmware's tone range/step (LEVEL 3 of
 * run_config_menu, "Volume Adjustment, Tone Only"). The slider walks the full cut-
 * or-boost span (TONE_VOLUME_DB_MIN..MAX) in TONE_VOLUME_DB_STEP increments; moving
 * it applies the offset LIVE and auditions the mini-flare balance preview.
 */
function buildToneVolumeControl() {
  const range  = document.getElementById('toneVolRange');
  const minDb  = api.toneVolMinDb();    // e.g. -6
  const maxDb  = api.toneVolMaxDb();    // e.g. +6
  const stepDb = api.toneVolStepDb();   // e.g. 2
  const nSteps = Math.round((maxDb - minDb) / stepDb);   // e.g. 6 -> 7 stops

  // The slider counts steps from the MIN up to the MAX; value maps linearly to dB.
  range.min = 0;
  range.max = nSteps;
  range.step = 1;
  range.value = Math.round((pendingToneVolDb - minDb) / stepDb);
  updateToneVolLabel(pendingToneVolDb);

  range.oninput = () => {
    pendingToneVolDb = minDb + parseInt(range.value, 10) * stepDb;
    updateToneVolLabel(pendingToneVolDb);
    unlockAudio().then(() => {
      applyToneVolume(pendingToneVolDb);
      previewBalance();
    });
  };
}

/**
 * Build the VOICE-volume range against the firmware's voice range/step (LEVEL 4,
 * "Volume Adjustment, Callouts Only"). Cut-only: the slider walks 0 dB down to
 * VOICE_VOLUME_DB_MIN in VOICE_VOLUME_DB_STEP increments.
 */
function buildVoiceVolumeControl() {
  const range  = document.getElementById('voiceVolRange');
  const minDb  = api.voiceVolMinDb();   // e.g. -6
  const stepDb = api.voiceVolStepDb();  // e.g. 2
  const nSteps = Math.round(-minDb / stepDb);   // e.g. 3 -> 4 stops

  // The slider counts CUT steps (0..nSteps); 0 == no cut (0 dB), nSteps == min.
  range.min = 0;
  range.max = nSteps;
  range.step = 1;
  range.value = Math.round(-pendingVoiceVolDb / stepDb);
  updateVoiceVolLabel(pendingVoiceVolDb);

  range.oninput = () => {
    pendingVoiceVolDb = -parseInt(range.value, 10) * stepDb;
    updateVoiceVolLabel(pendingVoiceVolDb);
    unlockAudio().then(() => {
      applyVoiceVolume(pendingVoiceVolDb);
      previewBalance();
    });
  };
}

/** Show the chosen TONE offset, flagging 0 dB as "flat" and signing the value. */
function updateToneVolLabel(db) {
  const el = document.getElementById('toneVolVal');
  el.textContent = db === 0 ? '0 dB · flat' : `${db > 0 ? '+' : ''}${db} dB`;
}

/** Show the chosen VOICE offset, flagging 0 dB as "full". */
function updateVoiceVolLabel(db) {
  const el = document.getElementById('voiceVolVal');
  el.textContent = db >= 0 ? '0 dB · full' : `${db} dB`;
}

// Mini-flare preview parameters — mirror config.h VOLUME_PREVIEW_SWEEP_*. The tone
// sweeps DOWN this band while the "20" and "10" callouts duck it.
const PREVIEW_SWEEP_FROM_FT = 20;
const PREVIEW_SWEEP_TO_FT   = 10;
const PREVIEW_SWEEP_S       = 2.5;

/**
 * Preview the current tone/voice BALANCE by ear: the "mini-flare", mirroring
 * audio_play_volume_preview_blocking(). The REAL presence tone sweeps down the
 * 20->10 ft band (pitch + level on the firmware schedule, through toneVolNode) in
 * the background while the "20" then "10" callouts speak over it (through
 * voiceVolNode) and DUCK it — so the pilot hears the actual balance. The SAME
 * preview serves both volume steps; whichever offset is live shifts the balance.
 * No-op until audio is unlocked.
 */
function previewBalance() {
  if (!audioUnlocked || !audioCtx) return;

  const t0  = audioCtx.currentTime + 0.03;
  const dur = PREVIEW_SWEEP_S;

  // Swept tone: a dedicated oscillator gliding pitch on the real schedule, through a
  // duck gain node, into the tone-volume node (so the live tone offset applies). The
  // level node rides the scheduled tone dB across the band (unclamped db_to_gain).
  const f0   = api.pitchHz(PREVIEW_SWEEP_FROM_FT);
  const f1   = api.pitchHz(PREVIEW_SWEEP_TO_FT);
  const lvl0 = api.dbToGain(api.toneDb(PREVIEW_SWEEP_FROM_FT) + api.eqlDb(PREVIEW_SWEEP_FROM_FT));
  const lvl1 = api.dbToGain(api.toneDb(PREVIEW_SWEEP_TO_FT)   + api.eqlDb(PREVIEW_SWEEP_TO_FT));

  const osc = audioCtx.createOscillator();
  osc.type = 'sine';
  osc.frequency.setValueAtTime(f0, t0);
  osc.frequency.linearRampToValueAtTime(f1, t0 + dur);

  const lvlGain = audioCtx.createGain();
  lvlGain.gain.setValueAtTime(0.0001, t0);
  lvlGain.gain.linearRampToValueAtTime(lvl0, t0 + 0.05);
  lvlGain.gain.linearRampToValueAtTime(lvl1, t0 + dur - 0.08);
  lvlGain.gain.linearRampToValueAtTime(0.0001, t0 + dur);

  const duckGain = audioCtx.createGain();
  duckGain.gain.value = 1;

  osc.connect(lvlGain).connect(duckGain).connect(toneVolNode);
  osc.start(t0);
  osc.stop(t0 + dur + 0.05);

  // The two callouts at their heights — each plays through the voice node AND ducks
  // the swept tone above via duckGain, the audible equivalent of duck_step().
  scheduleDuckedCallout(20, t0 + dur * 0.12, duckGain);
  scheduleDuckedCallout(10, t0 + dur * 0.58, duckGain);
}

/**
 * Play one callout at a scheduled time (through the voice-volume node) and dip the
 * background tone's duck gain while it sounds: down to the firmware's duck floor over
 * the attack, recovering over the release. A coarse but faithful stand-in for the
 * per-sample sidechain the flight loop (and the firmware preview) run.
 *
 * @param {number} ft        Callout height whose WAV to play.
 * @param {number} when      audioCtx time to start.
 * @param {GainNode} duckGain  The background tone's duck node to dip.
 */
function scheduleDuckedCallout(ft, when, duckGain) {
  const buf = calloutBuffers.get(ft);
  if (!buf) return;

  const src = audioCtx.createBufferSource();
  src.buffer = buf;
  src.connect(voiceVolNode);
  src.start(when);

  const floor = api.duckFloor();
  const atk   = Math.max(0.005, api.duckAttackMs()  / 1000);
  const rel   = Math.max(0.02,  api.duckReleaseMs() / 1000);
  duckGain.gain.setTargetAtTime(floor, when, atk);            // dip under the voice
  duckGain.gain.setTargetAtTime(1.0, when + buf.duration, rel); // breathe back up
}

/**
 * Schedule a fixed-frequency sine burst through the tone-volume node, returning the
 * audioCtx time it ENDS. Mirrors audio_play_tone_blocking(): the burst level is
 * db + equal-loudness at this frequency, with short raised-cosine-ish fades (a
 * linear ramp is a close, click-free analogue) top and tail.
 *
 * @param {number} hz    Tone frequency.
 * @param {number} ms    Burst length in ms.
 * @param {number} db    Burst level in dB before equal-loudness + master.
 * @param {number} when  audioCtx time to start at.
 * @returns {number}     audioCtx time the burst ends.
 */
function scheduleTone(hz, ms, db, when) {
  const dur = ms / 1000;
  const o = audioCtx.createOscillator();
  o.type = 'sine';
  o.frequency.value = hz;

  // Level = scheduled dB + equal-loudness flattening at this pitch (the firmware
  // folds equal_loudness_db into the preview gain), then -> linear gain. This is
  // the burst's INTRINSIC level (unclamped db_to_gain); the pilot's tone offset is
  // applied separately by toneVolNode downstream.
  const g = audioCtx.createGain();
  const peak = api.dbToGain(db + api.eqlDbHz(hz));
  const fade = Math.min(0.004, dur / 3);   // few-ms click-free edges
  g.gain.setValueAtTime(0.0001, when);
  g.gain.linearRampToValueAtTime(peak, when + fade);
  g.gain.setValueAtTime(peak, when + dur - fade);
  g.gain.linearRampToValueAtTime(0.0001, when + dur);

  o.connect(g).connect(toneVolNode);   // through the tone-volume node -> offset applies
  o.start(when);
  o.stop(when + dur + 0.01);
  return when + dur;
}

/**
 * Schedule the spoken callout for a height at a specific time, returning the time
 * it ends. Like playCallout() but time-anchored (for the volume preview sequence)
 * and NOT gated by modeCallouts — the preview always voices its number so the
 * pilot can judge voice-vs-tone balance even while auditioning a tone-only setup.
 *
 * @param {number} ft    Callout height whose WAV to play.
 * @param {number} when  audioCtx time to start at.
 * @returns {number}     audioCtx time it ends (start + buffer duration).
 */
function scheduleCalloutAt(ft, when) {
  const buf = calloutBuffers.get(ft);
  if (!buf) return when;                   // missing clip -> no gap added

  const src = audioCtx.createBufferSource();
  src.buffer = buf;
  // Centre the preview number (pan is a balance cue, not relevant to a level audit).
  // Through the VOICE node so the live voice offset trims it like the running box.
  src.connect(voiceVolNode);
  src.start(when);
  return when + buf.duration;
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
  applyToneVolume(pendingToneVolDb);    // independent tone + voice offsets
  applyVoiceVolume(pendingVoiceVolDb);

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
  activeVoice = null;          // drop any in-flight sidechain follower
}

/* =============================================================================
 *  Manual config-menu simulation (single button)
 *
 *  A faithful re-enactment of the firmware's hold-at-boot run_config_menu()
 *  (app_main.c), driven by ONE button exactly as the box's single config button
 *  is: a TAP cycles the current option, a DOUBLE-TAP confirms it (the firmware
 *  also auto-confirms after CONFIG_COMMIT_MS of silence — we keep the double-tap
 *  as the explicit confirm here). The level order, the spoken prompts, the
 *  preview tones, and the chirps are the SAME as the firmware's; we cannot run
 *  app_main.c itself in WASM (it pulls in ESP-IDF/I2S), so this JS walks the
 *  identical sequence and commits into the SAME live config vars + persists by
 *  recalibrating to ground, just like the real menu reboots on commit.
 *
 *  Implemented as an explicit step list so the flow reads top-to-bottom like the
 *  firmware function: enter -> LEVEL 1 (mode) -> LEVEL 2 (start-alt, skipped for
 *  tone-only) -> LEVEL 3 (volume) -> commit. A small interpreter advances on each
 *  tap/double-tap and narrates the current selection in the popup.
 * ===========================================================================*/

const DTAP_MS = 350;     // two taps within this window == a double-tap (confirm)

let manualOpen   = false;   // is the manual menu modal showing?
let manualLevel  = 0;       // 0 = mode, 1 = start-alt, 2 = tone-vol, 3 = voice-vol
let manualLastTap = 0;      // performance.now() of the last tap (double-tap detect)
let manualTapTimer = null;  // pending single-tap resolution timer

// Working picks for the in-progress walk (committed only at the end), seeded from
// the live config when the menu opens — same as the firmware starting from its
// saved/default values.
let manMode  = 1;
let manCapIdx = 0;          // index into the ascending callout ladder
let manToneVolDb  = 0;      // tone volume offset (cut or boost)
let manVoiceVolDb = 0;      // voice volume offset (cut only)

/** Open the manual menu: reset to LEVEL 1 and speak the entry + first option. */
function openManualMenu() {
  unlockAudio().then(() => {
    manualOpen  = true;
    manualLevel = 0;
    manMode     = audioMode;
    manToneVolDb  = toneVolDb;
    manVoiceVolDb = voiceVolDb;

    // Cap index seeded from the live cap against the ascending ladder.
    const asc = [...cfg.callouts].sort((a, b) => a - b);
    const liveCap = isFinite(startAltFt) ? startAltFt : asc[asc.length - 1];
    manCapIdx = Math.max(0, asc.indexOf(liveCap));
    if (manCapIdx < 0) manCapIdx = asc.length - 1;

    document.getElementById('manualBackdrop').classList.add('open');

    // Entry: chirp + "config mode" (config_mode has no WAV master -> skipped),
    // then announce the starting mode, mirroring run_config_menu()'s opening.
    let when = audioCtx.currentTime + 0.05;
    when = schedulePieceAt('chirp', when) + 0.08;
    when = schedulePieceAt('config_mode', when) + 0.12;
    announceModeManual(when);
    renderManual();
  });
}

/** Close the manual menu without committing. */
function closeManualMenu() {
  manualOpen = false;
  if (manualTapTimer) { clearTimeout(manualTapTimer); manualTapTimer = null; }
  document.getElementById('manualBackdrop').classList.remove('open');
}

/** Speak an audio mode as channel + stream pieces (mirrors announce_mode()). */
function announceModeManual(startWhen) {
  let when = startWhen ?? (audioCtx.currentTime + 0.05);
  const channel = api.modeStereo(manMode) === 1 ? 'stereo' : 'mono';
  let stream = 'callouts_and_tone';
  if (api.modeCallouts(manMode) === 0)      stream = 'tone_only';
  else if (api.modeTone(manMode) === 0)     stream = 'callouts_only';
  when = schedulePieceAt(channel, when) + 0.12;   // small gap, like the firmware
  schedulePieceAt(stream, when);
}

/** Speak a callout height for the start-alt level via its number WAV. */
function announceCapManual() {
  const asc = [...cfg.callouts].sort((a, b) => a - b);
  scheduleCalloutAt(asc[manCapIdx], audioCtx.currentTime + 0.05);
}

/** A single tap: cycle the current level's option (+ announce/preview it). */
function manualTap() {
  if (!manualOpen) return;
  switch (manualLevel) {
    case 0:   // audio mode
      manMode = (manMode + 1) % api.audioModeCount();
      applyAudioMode(manMode);          // apply live so prompts honour the mode
      announceModeManual();
      break;
    case 1: { // start altitude (only reached when callouts are enabled)
      const asc = [...cfg.callouts].sort((a, b) => a - b);
      // Firmware steps DOWN the descending ladder; on the ascending slider that is
      // a step toward index 0, wrapping to the top.
      manCapIdx = (manCapIdx - 1 + asc.length) % asc.length;
      announceCapManual();
      break;
    }
    case 2:   // tone volume (cut OR boost) — step DOWN, wrap from below MIN to MAX
      manToneVolDb -= api.toneVolStepDb();
      if (manToneVolDb < api.toneVolMinDb() - 0.001) manToneVolDb = api.toneVolMaxDb();
      applyToneVolume(manToneVolDb);    // apply live so the preview is at-level
      previewBalance();
      break;
    case 3:   // voice volume (cut only) — step DOWN, wrap from below MIN to 0
      manVoiceVolDb -= api.voiceVolStepDb();
      if (manVoiceVolDb < api.voiceVolMinDb() - 0.001) manVoiceVolDb = 0;
      applyVoiceVolume(manVoiceVolDb);
      previewBalance();
      break;
  }
  renderManual();
}

/** A double-tap: confirm the current level and advance (mirrors TAP_DOUBLE). */
function manualConfirm() {
  if (!manualOpen) return;
  schedulePieceAt('chirp', audioCtx.currentTime + 0.03);   // confirm chirp

  if (manualLevel === 0) {
    // Committed the mode. LEVEL 2 is skipped entirely for a tone-only mode (no
    // callouts to gate), exactly like the firmware.
    applyAudioMode(manMode);
    if (api.modeCallouts(manMode) === 1) {
      manualLevel = 1;
      // Announce "Callout Start Altitude" + the starting (top) height.
      let when = schedulePieceAt('start_alt', audioCtx.currentTime + 0.12) + 0.12;
      const asc = [...cfg.callouts].sort((a, b) => a - b);
      scheduleCalloutAt(asc[manCapIdx], when);
    } else {
      enterToneVolumeLevel();
    }
  } else if (manualLevel === 1) {
    enterToneVolumeLevel();
  } else if (manualLevel === 2) {
    // Tone volume committed. The voice-volume level is skipped for a tone-only mode
    // (no callouts to trim), exactly like the firmware.
    if (api.modeCallouts(manMode) === 1) {
      enterVoiceVolumeLevel();
    } else {
      commitManualMenu();
      return;
    }
  } else if (manualLevel === 3) {
    commitManualMenu();
    return;
  }
  renderManual();
}

/** Advance into LEVEL 3 (tone volume): announce "Volume Adjustment, Tone Only". */
function enterToneVolumeLevel() {
  manualLevel = 2;
  manToneVolDb = 0;                     // firmware starts each volume level at 0 dB
  applyToneVolume(manToneVolDb);
  let when = schedulePieceAt('volume_adjustment', audioCtx.currentTime + 0.12) + 0.12;
  when = schedulePieceAt('tone_only', when) + 0.12;
  // Preview the mini-flare after the announcement finishes.
  setTimeout(() => { if (manualOpen) previewBalance(); },
             Math.max(0, (when - audioCtx.currentTime) * 1000));
}

/** Advance into LEVEL 4 (voice volume): announce "Volume Adjustment, Callouts Only". */
function enterVoiceVolumeLevel() {
  manualLevel = 3;
  manVoiceVolDb = 0;                    // firmware starts each volume level at 0 dB
  applyVoiceVolume(manVoiceVolDb);
  let when = schedulePieceAt('volume_adjustment', audioCtx.currentTime + 0.12) + 0.12;
  when = schedulePieceAt('callouts_only', when) + 0.12;
  setTimeout(() => { if (manualOpen) previewBalance(); },
             Math.max(0, (when - audioCtx.currentTime) * 1000));
}

/** Commit the manual walk: apply all picks + recalibrate to ground, like reboot. */
function commitManualMenu() {
  applyAudioMode(manMode);
  const asc = [...cfg.callouts].sort((a, b) => a - b);
  startAltFt = api.modeCallouts(manMode) === 1 ? asc[manCapIdx] : cfg.callouts[0];
  applyToneVolume(manToneVolDb);
  applyVoiceVolume(manVoiceVolDb);
  calibrateToGround();
  closeManualMenu();
}

/** Route a button press through tap / double-tap detection (single button). */
function manualButtonPress() {
  const now = performance.now();
  if (now - manualLastTap < DTAP_MS) {
    // Second press inside the window -> double-tap (confirm). Cancel the pending
    // single-tap so it doesn't also cycle.
    if (manualTapTimer) { clearTimeout(manualTapTimer); manualTapTimer = null; }
    manualLastTap = 0;
    manualConfirm();
  } else {
    // First press: wait DTAP_MS to see if a second arrives; if not, it's a tap.
    manualLastTap = now;
    manualTapTimer = setTimeout(() => {
      manualTapTimer = null;
      manualLastTap = 0;
      manualTap();
    }, DTAP_MS);
  }
}

/** Paint the manual menu's status text from the current level + working picks. */
function renderManual() {
  const levelEl = document.getElementById('manualLevel');
  const valueEl = document.getElementById('manualValue');
  const LEVELS = ['Audio mode', 'Start-altitude cap', 'Tone volume', 'Voice volume'];
  levelEl.textContent = `Step ${manualLevel + 1} of ${LEVELS.length} · ${LEVELS[manualLevel] ?? '—'}`;

  let v = '—';
  if (manualLevel === 0) {
    v = AUDIO_MODE_META[manMode]?.title ?? `mode ${manMode}`;
  } else if (manualLevel === 1) {
    const asc = [...cfg.callouts].sort((a, b) => a - b);
    const ft = asc[manCapIdx];
    v = ft >= cfg.callouts[0] ? `${ft} ft · all` : `${ft} ft`;
  } else if (manualLevel === 2) {
    v = manToneVolDb === 0 ? '0 dB · flat'
                           : `${manToneVolDb > 0 ? '+' : ''}${manToneVolDb} dB`;
  } else if (manualLevel === 3) {
    v = manVoiceVolDb >= 0 ? '0 dB · full' : `${manVoiceVolDb} dB`;
  }
  valueEl.textContent = v;
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

  // Begin at the fixed default ILS start height and sink down the ladder from there.
  const startFt = APPROACH_START_FT;

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

  // Vario blip gate — sits right after the tone gain, before the pan, exactly where
  // audio.c multiplies s_blip_gate into the tone before panning. 1.0 == open (the
  // steady tone); the frame loop chops it to 0 between blips when sink-rate is on.
  blipGainNode = audioCtx.createGain();
  blipGainNode.gain.value = 1;

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

  // Two independent volume nodes, mirroring the firmware's split trims: the TONE
  // node sits at the END of the tone chain (audio_set_tone_db, may cut OR boost);
  // the VOICE node is fed by every callout source (audio_set_voice_db, cut only).
  // Each reaches the destination directly — there is no single master node anymore.
  toneVolNode = audioCtx.createGain();
  toneVolNode.gain.value = api.toneVolGain(toneVolDb);
  toneVolNode.connect(audioCtx.destination);

  voiceVolNode = audioCtx.createGain();
  voiceVolNode.gain.value = api.voiceVolGain(voiceVolDb);
  voiceVolNode.connect(audioCtx.destination);

  // Fundamental feeds the gain node at the normalised weight; the harmonic feeds
  // it through osc2Gain. Then: gain -> pan -> LPF -> tone-volume -> out (LPF is
  // post-pan so it shapes the final mix exactly like audio.c filters the L/R after
  // panning; the tone-volume node is post-LPF, the pilot's tone trim).
  const fundGain = audioCtx.createGain();
  fundGain.gain.value = norm;        // fundamental weight (1/(1+lvl))
  osc.connect(fundGain).connect(toneGainNode);
  osc2.connect(osc2Gain).connect(toneGainNode);
  toneGainNode.connect(blipGainNode).connect(tonePanNode)
              .connect(mixLpfNode).connect(toneVolNode);
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
      // Precompute this clip's sidechain envelope with the SAME one-pole follower
      // the firmware runs over the PCM, so the emulated duck tracks the identical
      // voice contour the DAC would (see precomputeEnv / audio.c).
      calloutEnv.set(ft, precomputeEnv(buf));
    } catch (e) {
      console.warn(`callout ${ft}.wav unavailable:`, e.message);
    }
  }));

  // Also decode the spoken config-menu prompt pieces (mono/config_mode have no
  // WAV master yet and are silently skipped, like an absent firmware clip).
  await Promise.all(Object.entries(CONFIG_PIECE_WAV).map(async ([name, file]) => {
    try {
      const res = await fetch(`${AUDIO_BASE}/${file}`);
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const buf = await audioCtx.decodeAudioData(await res.arrayBuffer());
      configBuffers.set(name, buf);
    } catch (e) {
      console.warn(`config piece ${file} unavailable:`, e.message);
    }
  }));
}

/**
 * Schedule a config-menu prompt piece at a time, returning when it ends. Plays
 * through the master node so the live volume offset trims it (the menu prompts
 * go through the same DAC path on hardware). A missing piece adds no gap.
 *
 * @param {string} name  Key into CONFIG_PIECE_WAV / configBuffers.
 * @param {number} when  audioCtx time to start at.
 * @returns {number}     audioCtx time it ends.
 */
function schedulePieceAt(name, when) {
  const buf = configBuffers.get(name);
  if (!buf || !audioCtx) return when;
  const src = audioCtx.createBufferSource();
  src.buffer = buf;
  // Menu prompts are voice clips -> through the voice-volume node.
  src.connect(voiceVolNode);
  src.start(when);
  return when + buf.duration;
}

/**
 * Precompute a callout's sidechain envelope — the exact one-pole peak follower the
 * firmware runs over the voice PCM (audio.c), so the emulated duck tracks the same
 * loudness contour the DAC produces. The follower's attack/release TIME CONSTANTS
 * are the firmware's DUCK_ATTACK_MS / DUCK_RELEASE_MS; we derive the per-sample
 * coefficients at THIS buffer's sample rate (so the constants mean the same wall-
 * clock time regardless of the browser's decode rate). Channel 0 is sufficient —
 * the clips are mono.
 *
 * @param {AudioBuffer} buf  Decoded callout buffer.
 * @returns {Float32Array}   Per-sample voice envelope (same length as the buffer).
 */
function precomputeEnv(buf) {
  const x  = buf.getChannelData(0);
  const fs = buf.sampleRate;
  // a = 1 - exp(-1/(tau_seconds * fs)) — identical form to audio_init().
  const atkA = 1 - Math.exp(-1 / ((api.duckAttackMs()  / 1000) * fs));
  const relA = 1 - Math.exp(-1 / ((api.duckReleaseMs() / 1000) * fs));
  const env = new Float32Array(x.length);
  let e = 0;
  for (let i = 0; i < x.length; i++) {
    const rect = Math.abs(x[i]);
    e += (rect > e ? atkA : relA) * (rect - e);
    env[i] = e;
  }
  return env;
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

  // Through the voice-volume node (post-pan) so the pilot's voice offset trims the
  // callouts, exactly like audio_set_voice_db() on the box.
  src.connect(pan).connect(voiceVolNode);
  src.start();

  // Register this clip as the active sidechain source: the frame loop reads its
  // precomputed envelope at the live playback position to derive the duck, exactly
  // like the firmware follows the voice it is mixing. Tracked by start time so we
  // can index env[(now - startTime) * fs]. A new callout supersedes the old one.
  const env = calloutEnv.get(ft);
  if (env) {
    activeVoice = {
      env,
      fs: buf.sampleRate,
      startTime: audioCtx.currentTime,
      endTime: audioCtx.currentTime + buf.duration,
    };
  }
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

/**
 * The vertical-rate MAGNITUDE (fpm) in the active vario direction, from the smoothed
 * cadence rate (blipRateFps). Sink mode counts descent only, climb mode counts climb
 * only, and the inactive direction reads 0 (=> constant tone). Mirrors audio.c.
 * @returns {number} fpm magnitude (>= 0)
 */
function varioRateFpm() {
  const vFpm = blipRateFps * 60;
  if (sinkRateOn)  return vFpm < 0 ? -vFpm : 0;
  if (climbRateOn) return vFpm > 0 ?  vFpm : 0;
  return 0;
}

/**
 * Blip rate (beats/min) for a vertical-rate magnitude — BlueFlyVario-style: PROPORTIONAL
 * to the rate, capped at varioBpmMax once it reaches varioFullFpm. Mirrors vario_bpm().
 * @returns {number} bpm, or 0 to mean "below onset => CONSTANT tone".
 */
function varioBpm(rateFpm) {
  if (rateFpm < cfg.varioOnsetFpm) return 0;          // constant tone below onset
  let bpm = rateFpm * (cfg.varioBpmMax / cfg.varioFullFpm);
  if (bpm > cfg.varioBpmMax) bpm = cfg.varioBpmMax;   // cap at the top
  return bpm;
}

/**
 * Beep / silence half-period lengths (ms) for a vertical-rate magnitude, mirroring
 * audio.c's vario_bpm() + blip_split(). Returns null when the tone should be CONSTANT.
 * @returns {{beepMs:number, silenceMs:number}|null}
 */
function varioBlip(rateFpm) {
  const bpm = varioBpm(rateFpm);
  if (bpm <= 0) return null;                  // constant tone
  const periodMs  = 60000 / bpm;
  const silenceMs = periodMs / (1 + cfg.varioBeepFactor);
  const beepMs    = periodMs - silenceMs;
  return { beepMs, silenceMs };
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

  // --- Voice SIDECHAIN duck: mirror audio.c's compressor --------------------
  // The duck tracks the voice's ACTUAL loudness, not a fixed window. We read the
  // active callout's precomputed envelope (the SAME one-pole follower the firmware
  // runs over the PCM) at the live playback position, then map it through the same
  // soft knee: silent (<= threshold) leaves the tone at 1.0; full voice (>= knee)
  // pulls it to the duck floor (db_to_gain(-VOICE_DUCK_DB)); between, linear. So
  // the leading edge of a word eases the tone down WITH the syllable — no clip.
  if (audioUnlocked) {
    let duck = 1;
    if (modeCallouts && activeVoice) {
      const now = audioCtx.currentTime;
      if (now >= activeVoice.endTime) {
        activeVoice = null;                 // clip done; the follower tail is in env
      } else {
        const idx = Math.floor((now - activeVoice.startTime) * activeVoice.fs);
        const e = (idx >= 0 && idx < activeVoice.env.length)
                  ? activeVoice.env[idx] : 0;
        const thr  = api.duckThreshold();
        const knee = api.duckKneeLevel();
        const floor = api.duckFloor();
        if (e > thr) {
          let t = (e - thr) / (knee - thr);
          if (t > 1) t = 1;
          duck = 1 + t * (floor - 1);       // 1.0 -> floor, matching audio.c
        }
      }
    }
    duckCur = duck;
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
      const toneTrim = modeCallouts ? api.dbToGain(api.toneTrimDb()) : 1;

      // Final tone gain = scheduled gain · flare fade · voice duck · baseline trim,
      // the same stack of multipliers the firmware applies per sample (the pilot's
      // tone volume offset lives on toneVolNode, downstream of this node).
      toneGainNode.gain.setTargetAtTime(
        api.toneGain(toneAgl) * flareFade * duckCur * toneTrim, now, 0.01);
    } else {
      // Ramp to silence rather than cutting — no click.
      toneGainNode.gain.setTargetAtTime(0, now, 0.03);
    }
  }

  // --- Vario blip gate: mirror audio.c's blip_advance_gate ------------------
  // A slow square wave on blipGainNode with rate-driven half-periods, scheduled on
  // the AudioContext clock (decoupled from the frame rate). With no direction active,
  // below onset, or the tone silent, the gate is held OPEN (constant tone) and the
  // phase primed so the next blip opens on a beep — exactly the firmware's gate.
  if (audioUnlocked && blipGainNode) {
    const now  = audioCtx.currentTime;
    const edge = Math.max(cfg.varioEdgeMs / 1000, 0.001);   // VARIO_EDGE_MS

    // Ease the cadence rate toward the live rate (one-pole over VARIO_RATE_SMOOTH_MS)
    // so a sudden shift glides in over a few blips instead of snapping — mirrors
    // audio.c's s_blip_rate_fps follower. Per-frame alpha from dt keeps it framerate-
    // independent.
    const tau = Math.max(cfg.varioRateSmoothMs / 1000, 1e-3);
    blipRateFps += (1 - Math.exp(-dt / tau)) * (api.vertFps() - blipRateFps);

    // Directional rate -> blip timing, or null for a constant tone (below onset /
    // inactive direction). Only chop while the tone is actually audible in this mode.
    const blip = (toneOn && modeTone) ? varioBlip(varioRateFpm()) : null;

    if (blip) {
      // Durations recompute ONLY at a flip, so each beep/silence plays out fully.
      if (now >= blipNextFlip) {
        blipInBeep = !blipInBeep;                            // flip into the next half
        const halfMs = blipInBeep ? blip.beepMs : blip.silenceMs;
        blipNextFlip = now + halfMs / 1000;
        blipGainNode.gain.setTargetAtTime(blipInBeep ? 1 : 0, now, edge);
      }
    } else {
      blipGainNode.gain.setTargetAtTime(1, now, edge);      // held open (steady tone)
      blipInBeep   = false;
      blipNextFlip = 0;
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

  // Vario blip readout: show the active mode + the live cadence (bpm) or "steady".
  let varioTxt = 'off';
  if (sinkRateOn || climbRateOn) {
    const mode = sinkRateOn ? 'sink' : 'climb';
    const blip = varioBlip(varioRateFpm());
    if (!toneOn)      varioTxt = `${mode} · armed`;
    else if (!blip)   varioTxt = `${mode} · steady`;
    else              varioTxt = `${mode} · ${Math.round(60000 / (blip.beepMs + blip.silenceMs))} bpm`;
  }
  document.getElementById('varioVal').textContent = varioTxt;

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

  // Independent tone + voice offsets (0 dB == flat/full). Shown compactly as
  // "T <tone> · V <voice>".
  const tStr = toneVolDb  === 0 ? '0' : `${toneVolDb > 0 ? '+' : ''}${toneVolDb}`;
  const vStr = voiceVolDb === 0 ? '0' : `${voiceVolDb}`;
  document.getElementById('volValTelem').textContent = `T ${tStr} · V ${vStr} dB`;
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

  // Vario blip direction (firmware LEVEL 7): one mutually-exclusive choice. Pure JS
  // state the gate reads every frame; resetting blipNextFlip makes a switch start a
  // fresh blip on a beep at once. Off => both false => steady tone.
  const onVario = (mode) => {
    sinkRateOn   = (mode === 'sink');
    climbRateOn  = (mode === 'climb');
    blipInBeep   = false;
    blipNextFlip = 0;
  };
  document.getElementById('varioOff').addEventListener('change',   () => onVario('off'));
  document.getElementById('varioSink').addEventListener('change',  () => onVario('sink'));
  document.getElementById('varioClimb').addEventListener('change', () => onVario('climb'));

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

  // --- Manual config menu (single-button, drives the firmware sequence) ----
  document.getElementById('manualBtn').addEventListener('click', openManualMenu);
  document.getElementById('manualDoneBtn').addEventListener('click', closeManualMenu);
  // The single config button: tap to cycle, double-tap to confirm/advance.
  document.getElementById('manualTapBtn').addEventListener('click', manualButtonPress);
  document.getElementById('manualBackdrop').addEventListener('click', (e) => {
    if (e.target.id === 'manualBackdrop') closeManualMenu();
  });

  // Esc closes whichever sheet is open, matching native sheet behaviour.
  window.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { closeConfigMenu(); closeManualMenu(); }
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
