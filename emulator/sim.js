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

// Smoothing factor for the *visual* aircraft glyph only. The state machine is
// always fed the RAW target AGL; this just makes the on-screen motion glide.
const DISPLAY_EASE = 0.18;

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

// WebAudio graph.
let audioCtx     = null; // AudioContext (created on first gesture)
let osc          = null; // continuous OscillatorNode for the presence tone
let toneGainNode = null; // GainNode — 0 == silent
let audioUnlocked = false;
const calloutBuffers = new Map(); // height(ft) -> decoded AudioBuffer

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
    toneGain:      M.cwrap('sim_tone_gain',      num,   [num]),

    armFt:         M.cwrap('sim_arm_ft',         num,   []),
    toneStartFt:   M.cwrap('sim_tone_start_ft',  num,   []),
    toneFullFt:    M.cwrap('sim_tone_full_ft',   num,   []),
    flareHiFt:     M.cwrap('sim_flare_hi_ft',    num,   []),
    flareLoFt:     M.cwrap('sim_flare_lo_ft',    num,   []),
  };

  // 3. Start the machine on the default profile, parked on the ground.
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
  };

  // Visible tape top: a little headroom above the highest callout / cruise so
  // the climb-out and CRUISE band are both on-screen.
  topFt = Math.max(cfg.callouts[0], cfg.cruiseFt) + 60;

  // Keep the target within the new range.
  targetAgl = clamp(targetAgl, 0, topFt);
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

  osc = audioCtx.createOscillator();
  osc.type = 'sine';                 // pure presence tone, like the firmware NCO
  osc.frequency.value = 600;         // F_AT_TONE_START; updated every frame

  toneGainNode = audioCtx.createGain();
  toneGainNode.gain.value = 0;       // start silent

  osc.connect(toneGainNode).connect(audioCtx.destination);
  osc.start();
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
  const src = audioCtx.createBufferSource();
  src.buffer = buf;
  src.connect(audioCtx.destination);
  src.start();
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

  // Step the REAL state machine with the raw target altitude.
  api.step(targetAgl, dt);

  // Read the decision back out.
  const st       = api.state();
  const firedFt  = api.firedHeightFt();
  const toneAgl  = api.toneAgl();
  const toneOn   = api.toneActive() === 1;
  const pollMs   = api.pollMs();

  // --- Audio: callout + tone ------------------------------------------------
  if (firedFt >= 0) {
    lastFiredFt = firedFt;
    if (audioUnlocked) playCallout(firedFt);
  }

  if (audioUnlocked) {
    const now = audioCtx.currentTime;
    if (toneOn) {
      // Drive the oscillator from the SAME math the firmware uses. setTargetAtTime
      // gives a short, click-free glide analogous to the raised-cosine envelope.
      osc.frequency.setTargetAtTime(api.pitchHz(toneAgl), now, 0.01);
      toneGainNode.gain.setTargetAtTime(api.toneGain(toneAgl), now, 0.01);
    } else {
      // Ramp to silence rather than cutting — no click.
      toneGainNode.gain.setTargetAtTime(0, now, 0.03);
    }
  }

  // --- Visuals: ease the glyph toward target, then paint --------------------
  displayAgl += (targetAgl - displayAgl) * DISPLAY_EASE;
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
  } else {
    document.getElementById('hzVal').textContent = '— Hz';
    document.getElementById('dbVal').textContent = '— dB';
  }

  document.getElementById('pollVal').textContent = `${pollMs} ms`;
  document.getElementById('calloutVal').textContent =
    lastFiredFt == null ? '—' : `${Math.round(lastFiredFt)} ft`;
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
    profileIdx = parseInt(e.target.value, 10) || 0;
    api.setProfile(profileIdx);
    api.init(0 /* ST_GROUND */);   // clean restart under the new ladder
    lastFiredFt = null;
    readConfig();
  });

  document.getElementById('resetBtn').addEventListener('click', () => {
    targetAgl = 0;
    api.init(0 /* ST_GROUND */);
    lastFiredFt = null;
  });

  document.getElementById('cruiseBtn').addEventListener('click', () => {
    // Snap the aircraft up to just above cruise so you can demo a full approach.
    targetAgl = cfg.cruiseFt + 20;
    displayAgl = targetAgl;        // jump the glyph too, no long glide
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
