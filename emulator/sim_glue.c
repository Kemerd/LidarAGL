/**
 * @file    sim_glue.c
 * @brief   Flat, JS-friendly ABI over the REAL LidarAGL firmware logic.
 *
 * @details This shim is the only new C in the web emulator. It compiles the
 *          firmware's PURE modules UNCHANGED (state_machine.c, audio_math.c,
 *          sensor_profile.c) and exposes their behaviour through a set of plain
 *          scalar functions that JavaScript can call via Emscripten's cwrap.
 *
 *          Why a shim at all?
 *            sm_step() takes and returns STRUCT POINTERS (sm_ctx_t / sm_out_t).
 *            Marshalling structs across the WASM/JS boundary is fiddly and
 *            error-prone. Instead we keep one carried context and one output
 *            struct as file-static state inside the WASM module, and expose:
 *              - "command" functions that mutate that state (sim_step, sim_init)
 *              - "getter" functions that read the last sm_out_t back out as
 *                simple ints/floats.
 *            JS therefore never touches a C struct — it just calls sim_step()
 *            then reads sim_state(), sim_fired_height_ft(), etc.
 *
 *          Compiled with -DUNIT_TEST (exactly like the host unit tests), so
 *          config.h fences out its REGION 2 (ESP-IDF GPIO block) and this stays
 *          a hardware-free translation unit. Mirrors test/Makefile's PURE set.
 *
 *          NOTE: callouts.c is deliberately NOT part of this build. Its only
 *          pure piece (callout_id_for_ft) shares a translation unit with the
 *          EMBED_FILES "_binary_*_pcm" weak externs that do not exist under
 *          Emscripten. We don't need it anyway: the callout WAV filenames are
 *          numeric (10.wav .. 500.wav) and come straight from the profile's
 *          callout heights, so JS maps index -> height -> "<height>.wav".
 */

#include "state_machine.h"   /* sm_ctx_t, sm_out_t, sm_step, sm_init, ...        */
#include "sensor_profile.h"  /* sensor_profile_t, SF30C_PROFILE, SF30D_PROFILE   */
#include "audio_math.h"      /* agl_to_pitch_hz, agl_to_tone_db, db_to_gain, ... */
#include "config.h"          /* ARM_FT, TONE_START_FT, FLARE_BAND_*, ...         */

#include <emscripten.h>      /* EMSCRIPTEN_KEEPALIVE                              */

/* ===========================================================================
 *  Module-static simulator state.
 *
 *  Exactly the same three things the firmware's logic_task carries: the running
 *  state-machine context, the most recent decision output, and a pointer to the
 *  active sensor profile. One instance, owned by the WASM module, driven from JS.
 * ===========================================================================*/

static sm_ctx_t                g_ctx;                  /* carried SM context     */
static sm_out_t                g_out;                  /* last sm_step() output  */
static const sensor_profile_t *g_profile = &SF30C_PROFILE;  /* active profile    */

/*  Current pilot-chosen tone-start altitude (ft). Mirrors app_main's s_tone_start_ft:
 *  the height at/below which the presence tone is allowed to sound and the anchor of
 *  the pitch + dB fade-in. Seeded to the compile-time default; sim_set_tone_start()
 *  updates BOTH this (so the getter/tape reflect it) and the two firmware paths it
 *  drives — the SM tone gate (g_ctx.tone_start_ft) and the audio math schedule.      */
static float s_tone_start_ft = TONE_START_FT;

/* ===========================================================================
 *  Lifecycle / profile selection
 * ===========================================================================*/

/**
 * @brief Select the active sensor profile (0 = SF30/C, anything else = SF30/D).
 *
 * @details Switching the profile changes the callout ladder and cruise_ft that
 *          every subsequent sim_step() reasons against. The caller should follow
 *          this with sim_init() to start the machine cleanly under the new
 *          profile (the JS app does exactly that on the profile dropdown).
 */
EMSCRIPTEN_KEEPALIVE
void sim_set_profile(int idx)
{
    g_profile = (idx == 1) ? &SF30D_PROFILE : &SF30C_PROFILE;
}

/**
 * @brief Initialise the carried context into a given starting state.
 * @param initial  An sm_state_t value as an int (ST_GROUND=0 .. ST_DESCENT=4).
 */
EMSCRIPTEN_KEEPALIVE
void sim_init(int initial)
{
    sm_init(&g_ctx, (sm_state_t)initial);
}

/**
 * @brief Expose the firmware's boot-state chooser for completeness.
 *
 * @details Lets the UI ask "if I rebooted at this AGL, where would the machine
 *          wake up?" — handy for demonstrating in-flight-reboot recovery without
 *          re-implementing the rule in JS. Returns an sm_state_t as an int.
 */
EMSCRIPTEN_KEEPALIVE
int sim_initial_state(float boot_agl, int ok)
{
    return (int)sm_initial_state(boot_agl, ok != 0, g_profile);
}

/**
 * @brief Advance the REAL state machine by one tick.
 *
 * @param agl_ft  Current AGL in feet (the simulated aircraft's height).
 * @param dt_s    Seconds since the previous step (drives the trend estimate).
 *
 * @details This is the heart of the emulator: it calls the firmware's own
 *          sm_step() verbatim, storing the decision in g_out for the getters
 *          below to read. No logic is duplicated here.
 */
EMSCRIPTEN_KEEPALIVE
void sim_step(float agl_ft, float dt_s)
{
    sm_step(&g_ctx, agl_ft, dt_s, g_profile, &g_out);
}

/* ===========================================================================
 *  Decision getters — read the last sim_step() result.
 * ===========================================================================*/

/** @brief Current behaviour state as an int (sm_state_t). */
EMSCRIPTEN_KEEPALIVE
int sim_state(void) { return (int)g_out.state; }

/** @brief Index of the callout fired this tick (-1 if none), into the profile. */
EMSCRIPTEN_KEEPALIVE
int sim_fired_index(void) { return g_out.fired_callout; }

/**
 * @brief Height in feet of the callout fired this tick, or -1 if none.
 *
 * @details Convenience for the audio layer: fired_callout is an INDEX, but the
 *          WAV files are named by height (e.g. "50.wav"). We resolve it here so
 *          JS gets a number it can turn straight into a filename.
 */
EMSCRIPTEN_KEEPALIVE
float sim_fired_height_ft(void)
{
    int i = g_out.fired_callout;
    if (i < 0 || (size_t)i >= g_profile->n_callouts) {
        return -1.0f;
    }
    return g_profile->callouts[i];
}

/** @brief Concrete poll period (ms) the firmware would apply for this state. */
EMSCRIPTEN_KEEPALIVE
int sim_poll_ms(void) { return (int)poll_profile_to_ms(g_out.poll); }

/** @brief AGL (ft) the audio engine should use for the presence tone. */
EMSCRIPTEN_KEEPALIVE
float sim_tone_agl(void) { return g_out.tone_agl; }

/** @brief 1 if the presence tone should sound this tick, else 0. */
EMSCRIPTEN_KEEPALIVE
int sim_tone_active(void) { return g_out.tone_active ? 1 : 0; }

/** @brief Smoothed vertical rate (ft/s, +up) for the vario blip cadence. */
EMSCRIPTEN_KEEPALIVE
float sim_vert_fps(void) { return g_out.vert_fps; }

/**
 * @brief 1 the ONE tick the state machine confirms a sustained post-liftoff climb.
 *
 * @details Surfaces sm_out_t.fired_positive_rate — the firmware speaks the spoken
 *          "positive rate" reminder on this edge when the pilot has enabled the
 *          feature (LEVEL 6). The emulator gates it on its own posRateOn flag in JS,
 *          exactly like app_main's s_posrate_enabled, and plays the clip on the edge.
 */
EMSCRIPTEN_KEEPALIVE
int sim_fired_positive_rate(void) { return g_out.fired_positive_rate ? 1 : 0; }

/* ---- Vario blip tunables (mirror config.h so the JS cadence matches firmware) -- */
EMSCRIPTEN_KEEPALIVE float sim_vario_onset_fpm(void)     { return VARIO_ONSET_FPM; }
EMSCRIPTEN_KEEPALIVE float sim_vario_full_fpm(void)      { return VARIO_FULL_FPM; }
EMSCRIPTEN_KEEPALIVE float sim_vario_bpm_max(void)       { return VARIO_BPM_MAX; }
EMSCRIPTEN_KEEPALIVE float sim_vario_beep_factor(void)   { return VARIO_BEEP_FACTOR; }
EMSCRIPTEN_KEEPALIVE float sim_vario_edge_ms(void)       { return VARIO_EDGE_MS; }
EMSCRIPTEN_KEEPALIVE float sim_vario_rate_smooth_ms(void){ return VARIO_RATE_SMOOTH_MS; }

/* ===========================================================================
 *  Profile introspection — lets JS draw the altitude tape from real data.
 * ===========================================================================*/

/** @brief Number of callouts in the active profile's ladder. */
EMSCRIPTEN_KEEPALIVE
int sim_n_callouts(void) { return (int)g_profile->n_callouts; }

/** @brief Height (ft) of callout @p i in the active profile, or -1 if OOB. */
EMSCRIPTEN_KEEPALIVE
float sim_callout_ft(int i)
{
    if (i < 0 || (size_t)i >= g_profile->n_callouts) {
        return -1.0f;
    }
    return g_profile->callouts[i];
}

/** @brief Number of "check gear" altitude options the active profile offers. */
EMSCRIPTEN_KEEPALIVE
int sim_n_gear_check_opts(void) { return (int)g_profile->n_gear_check_opts; }

/**
 * @brief Height (ft) of gear-check option @p i in the active profile, or -1 if OOB.
 *
 * @details The config menu's LEVEL 5 cycles OFF -> these heights -> OFF (descending,
 *          highest first), mirroring run_config_menu(). JS reads the list straight
 *          from the profile so the sim offers the SAME altitudes the box does
 *          (SF30/C: 200/100; SF30/D: 500/400/300/200/100).
 */
EMSCRIPTEN_KEEPALIVE
float sim_gear_check_opt(int i)
{
    if (i < 0 || (size_t)i >= g_profile->n_gear_check_opts) {
        return -1.0f;
    }
    return g_profile->gear_check_opts[i];
}

/** @brief At/above this AGL (ft) the active profile enters low-power CRUISE. */
EMSCRIPTEN_KEEPALIVE
float sim_cruise_ft(void) { return g_profile->cruise_ft; }

/** @brief Nominal usable ceiling (ft) of the active profile's sensor. */
EMSCRIPTEN_KEEPALIVE
float sim_max_range_ft(void) { return g_profile->max_range_ft; }

/* ===========================================================================
 *  Real audio math — the SAME functions that drive the firmware's tone, so the
 *  browser's WebAudio oscillator hears exactly what the pilot would.
 * ===========================================================================*/

/** @brief Presence-tone frequency (Hz) for an AGL — ascends as AGL falls. */
EMSCRIPTEN_KEEPALIVE
float sim_pitch_hz(float agl_ft) { return agl_to_pitch_hz(agl_ft); }

/** @brief Scheduled presence-tone level (dB) for an AGL (readout only). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_db(float agl_ft) { return agl_to_tone_db(agl_ft); }

/**
 * @brief Equal-loudness (Fletcher-Munson) correction in dB at this AGL's pitch.
 *
 * @details Readout-only surface of the SAME ISO 226 term that sim_tone_gain()
 *          folds into the final gain. Resolves the AGL to its current sweep pitch
 *          first, then returns the dB the firmware ADDS at that frequency so equal
 *          scheduled level sounds equally loud (negative = ear more sensitive, so
 *          attenuate; positive = less sensitive, so boost). Lets the UI display
 *          the flattening separately from the scheduled level. Returns 0 when
 *          EQUAL_LOUDNESS_CORRECTION is disabled in config.h.
 */
EMSCRIPTEN_KEEPALIVE
float sim_eql_db(float agl_ft) { return equal_loudness_db(agl_to_pitch_hz(agl_ft)); }

/**
 * @brief Equal-loudness correction (dB) at a RAW frequency (not via AGL).
 *
 * @details The volume-preview tone is a fixed 1 kHz burst, not a swept AGL pitch,
 *          so the emulator needs the ISO-226 flattening AT a frequency directly —
 *          the same equal_loudness_db(freq_hz) the firmware folds into
 *          audio_play_tone_blocking(). Exposing it keeps the preview's level
 *          identical to the hardware's instead of re-deriving the curve in JS.
 */
EMSCRIPTEN_KEEPALIVE
float sim_eql_db_hz(float freq_hz) { return equal_loudness_db(freq_hz); }

/**
 * @brief Final linear gain for the tone at a given AGL.
 *
 * @details Combines the scheduled dB level with the equal-loudness correction at
 *          the tone's current pitch, then converts to a linear gain — exactly the
 *          chain the firmware's audio_task runs per sample. JS sets the WebAudio
 *          GainNode to this value each frame, so the perceived loudness curve
 *          matches the hardware (urgency carried by pitch, loudness held flat).
 */
EMSCRIPTEN_KEEPALIVE
float sim_tone_gain(float agl_ft)
{
    float hz = agl_to_pitch_hz(agl_ft);
    float db = agl_to_tone_db(agl_ft) + equal_loudness_db(hz);
    return db_to_gain(db);
}

/* ===========================================================================
 *  Config constants — re-exported so the UI never hard-codes a magic number.
 *  Every value here is the single source of truth from config.h REGION 1.
 * ===========================================================================*/

/** @brief AGL (ft) above which descent callouts arm (silent climb-out below). */
EMSCRIPTEN_KEEPALIVE
float sim_arm_ft(void) { return ARM_FT; }

/** @brief AGL (ft) at which the presence tone begins to fade in (the live, pilot-
 *         chosen value — default TONE_START_FT or the higher TONE_START_FT_HIGH). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_start_ft(void) { return s_tone_start_ft; }

/** @brief The alternate (earlier/higher) tone-start option the menu offers (200 ft). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_start_high_ft(void) { return TONE_START_FT_HIGH; }

/**
 * @brief Set the pilot's tone-start altitude — the sim analogue of app_main applying
 *        config_load_tone_start() at boot (LEVEL 8 of run_config_menu).
 *
 * @details Threads the chosen altitude into BOTH firmware paths exactly as the box:
 *            1) the audio math fade-in schedule (audio_math_set_tone_start), so the
 *               pitch + dB anchor on it, and
 *            2) the state machine's tone gate (g_ctx.tone_start_ft), so the tone is
 *               allowed to sound from this height down.
 *          Also caches it for sim_tone_start_ft() so the readout + tape band track it.
 *          Must be called AFTER sim_init() each time, since sm_init() re-seeds the
 *          gate to the compile-time default (the JS side re-applies after every init).
 *
 * @param ft  Tone-start altitude in feet (TONE_START_FT or TONE_START_FT_HIGH).
 */
EMSCRIPTEN_KEEPALIVE
void sim_set_tone_start(float ft)
{
    s_tone_start_ft     = ft;
    g_ctx.tone_start_ft = ft;
    audio_math_set_tone_start(ft);
}

/** @brief AGL (ft) at/below which the tone reaches full presence. */
EMSCRIPTEN_KEEPALIVE
float sim_tone_full_ft(void) { return TONE_FULL_FT; }

/** @brief Top of the flare full-attention band (ft). */
EMSCRIPTEN_KEEPALIVE
float sim_flare_hi_ft(void) { return FLARE_BAND_HI; }

/** @brief Bottom of the flare full-attention band (ft). */
EMSCRIPTEN_KEEPALIVE
float sim_flare_lo_ft(void) { return FLARE_BAND_LO; }

/* ---- Flare fade-out (distraction guard) — see audio.c render loop --------- */
/*  The firmware's audio_task fades the presence tone out below FLARE_FADE_FT and
 *  quickly back in on a climb-back-up. The emulator's WebAudio path re-creates
 *  that envelope in JS; these getters hand it the SAME constants so the timing
 *  matches the hardware exactly and no magic number is duplicated in sim.js.    */

/** @brief AGL (ft) below which the presence tone fades out under the flare. */
EMSCRIPTEN_KEEPALIVE
float sim_flare_fade_ft(void) { return FLARE_FADE_FT; }

/** @brief Slow fade-OUT time (ms): tone full -> silent under the flare. */
EMSCRIPTEN_KEEPALIVE
float sim_flare_fade_out_ms(void) { return FLARE_FADE_OUT_MS; }

/** @brief Quick fade-IN time (ms): tone silent -> full on a climb-back-up. */
EMSCRIPTEN_KEEPALIVE
float sim_flare_fade_in_ms(void) { return FLARE_FADE_IN_MS; }

/* ===========================================================================
 *  Boot config menu — audio mode + start-altitude cap.
 *
 *  The real firmware enters a hold-at-boot menu (app_main.c run_config_menu) that
 *  selects one of AUDIO_MODE_* and a start-altitude cap, persists them to NVS, and
 *  reboots. The emulator can't run audio.c (it pulls in I2S/ESP-IDF hardware and
 *  is therefore NOT in the PURE WASM build), so we surface the SAME decision data
 *  here from config.h's pure region and re-encode audio_config_from_mode()'s
 *  truth table 1:1. JS reads these to drive its WebAudio graph, so the sim plays
 *  exactly what the configured firmware would: tone-only, callouts-only, mono, or
 *  gently panned stereo.
 *
 *  IMPORTANT: this switch MUST stay in lockstep with audio.c::audio_config_from_mode.
 *  If a mode's flags change there, change them here too.
 * ------------------------------------------------------------------------- */

/** @brief Number of selectable audio modes (AUDIO_MODE_COUNT). */
EMSCRIPTEN_KEEPALIVE
int sim_audio_mode_count(void) { return AUDIO_MODE_COUNT; }

/** @brief Default audio mode applied after a config wipe (DEFAULT_AUDIO_MODE). */
EMSCRIPTEN_KEEPALIVE
int sim_default_audio_mode(void) { return DEFAULT_AUDIO_MODE; }

/** @brief Equal-power stereo lean fraction used in STEREO_BOTH (STEREO_PAN). */
EMSCRIPTEN_KEEPALIVE
float sim_stereo_pan(void) { return STEREO_PAN; }

/** @brief 2nd-harmonic mix level for tone warmth (TONE_HARMONIC2_LVL). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_harmonic2(void) { return TONE_HARMONIC2_LVL; }

/** @brief Corner frequency (Hz) of the anti-harshness mix LPF (MIX_LPF_FC_HZ). */
EMSCRIPTEN_KEEPALIVE
float sim_mix_lpf_fc(void) { return MIX_LPF_FC_HZ; }

/* ---- Independent tone + voice volume offsets (boot config menu) ------------ */
/*  The pilot's two trims, layered on the analog pot: the TONE offset (cut OR boost)
 *  and the VOICE offset (cut only). These getters hand the SAME ranges/steps/
 *  defaults the firmware config menu cycles so the emulator's two volume controls
 *  and audio graph match the hardware exactly. The *_gain helpers convert a chosen
 *  offset to the linear gain the matching WebAudio node should use, clamping to the
 *  same limits audio_set_tone_db / audio_set_voice_db enforce on the box.         */

/** @brief Deepest TONE cut the menu offers, in dB (TONE_VOLUME_DB_MIN). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_volume_db_min(void) { return TONE_VOLUME_DB_MIN; }

/** @brief Loudest TONE boost the menu offers, in dB (TONE_VOLUME_DB_MAX). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_volume_db_max(void) { return TONE_VOLUME_DB_MAX; }

/** @brief Step between TONE volume settings, in dB (TONE_VOLUME_DB_STEP). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_volume_db_step(void) { return TONE_VOLUME_DB_STEP; }

/** @brief Default TONE offset until the pilot moves it (0 dB). */
EMSCRIPTEN_KEEPALIVE
float sim_default_tone_volume_db(void) { return DEFAULT_TONE_VOLUME_DB; }

/** @brief Deepest VOICE cut the menu offers, in dB (VOICE_VOLUME_DB_MIN). */
EMSCRIPTEN_KEEPALIVE
float sim_voice_volume_db_min(void) { return VOICE_VOLUME_DB_MIN; }

/** @brief Step between VOICE volume settings, in dB (VOICE_VOLUME_DB_STEP). */
EMSCRIPTEN_KEEPALIVE
float sim_voice_volume_db_step(void) { return VOICE_VOLUME_DB_STEP; }

/** @brief Default VOICE offset until the pilot lowers it (0 dB). */
EMSCRIPTEN_KEEPALIVE
float sim_default_voice_volume_db(void) { return DEFAULT_VOICE_VOLUME_DB; }

/** @brief Linear TONE gain for an offset dB — clamped + db_to_gain()'d exactly like
 *         audio_set_tone_db(), so the sim's tone node tracks the hardware 1:1. */
EMSCRIPTEN_KEEPALIVE
float sim_tone_volume_gain(float db)
{
    if (db < TONE_VOLUME_DB_MIN) db = TONE_VOLUME_DB_MIN;
    if (db > TONE_VOLUME_DB_MAX) db = TONE_VOLUME_DB_MAX;
    return db_to_gain(db);
}

/** @brief Linear VOICE gain for an offset dB — clamped (cut-only) + db_to_gain()'d
 *         exactly like audio_set_voice_db(), so the voice node matches the box. */
EMSCRIPTEN_KEEPALIVE
float sim_voice_volume_gain(float db)
{
    if (db > 0.0f) db = 0.0f;
    if (db < VOICE_VOLUME_DB_MIN) db = VOICE_VOLUME_DB_MIN;
    return db_to_gain(db);
}

/** @brief Generic dB -> linear gain (the firmware's db_to_gain, UNCLAMPED). Used by
 *         the sim for the preview-burst level and the steady tone trim, where the
 *         value is an intrinsic level rather than a clamped pilot offset. */
EMSCRIPTEN_KEEPALIVE
float sim_db_to_gain(float db) { return db_to_gain(db); }

/* ---- Volume-preview tone (config menu) ------------------------------------ */
/*  The "tone .. number .. tone" preview the volume menu plays at each step. The
 *  sim re-creates it with a short WebAudio oscillator burst at exactly these
 *  parameters so the preview the user hears matches the firmware's.             */

/** @brief Preview tone frequency (Hz) — the 1 kHz equal-loudness reference. */
EMSCRIPTEN_KEEPALIVE
float sim_volume_preview_hz(void) { return VOLUME_PREVIEW_HZ; }

/** @brief Preview tone burst length (ms). */
EMSCRIPTEN_KEEPALIVE
float sim_volume_preview_ms(void) { return VOLUME_PREVIEW_MS; }

/** @brief Preview tone level (dB) before the master offset (VOLUME_PREVIEW_DB). */
EMSCRIPTEN_KEEPALIVE
float sim_volume_preview_db(void) { return VOLUME_PREVIEW_DB; }

/* ---- Voice-duck envelope + baseline tone trim ----------------------------- */
/*  The firmware ducks the tone under a callout with an ASYMMETRIC envelope (fast
 *  attack, slow release) and, whenever callouts are enabled, holds the tone a
 *  constant TONE_TRIM_WITH_VOICE_DB down so the voice reads clearer. The emulator
 *  mirrors both; these getters hand it the SAME constants.                       */

/** @brief Per-callout tone duck depth (dB), VOICE_DUCK_DB (positive magnitude). */
EMSCRIPTEN_KEEPALIVE
float sim_voice_duck_db(void) { return VOICE_DUCK_DB; }

/** @brief Duck ATTACK time (ms): tone -> ducked, fast (DUCK_ATTACK_MS). */
EMSCRIPTEN_KEEPALIVE
float sim_duck_attack_ms(void) { return DUCK_ATTACK_MS; }

/** @brief Duck RELEASE time (ms): follower fall, gentle (DUCK_RELEASE_MS). */
EMSCRIPTEN_KEEPALIVE
float sim_duck_release_ms(void) { return DUCK_RELEASE_MS; }

/** @brief Sidechain threshold: |voice| below this == silent, no duck (DUCK_THRESHOLD). */
EMSCRIPTEN_KEEPALIVE
float sim_duck_threshold(void) { return DUCK_THRESHOLD; }

/** @brief Sidechain knee level: |voice| at/above this == full duck (DUCK_KNEE_LEVEL). */
EMSCRIPTEN_KEEPALIVE
float sim_duck_knee_level(void) { return DUCK_KNEE_LEVEL; }

/** @brief Full-duck linear gain floor, db_to_gain(-VOICE_DUCK_DB). */
EMSCRIPTEN_KEEPALIVE
float sim_duck_floor(void) { return db_to_gain(-VOICE_DUCK_DB); }

/** @brief Steady tone trim (dB) held while callouts are enabled
 *         (TONE_TRIM_WITH_VOICE_DB; negative == a cut). */
EMSCRIPTEN_KEEPALIVE
float sim_tone_trim_with_voice_db(void) { return TONE_TRIM_WITH_VOICE_DB; }

/* ---- Ground-dwell disarm (taxi-back reset) -------------------------------- */
/*  After GROUND_RESET_MS continuously parked the firmware disarms as if rebooted.
 *  The sim's state machine is the REAL one (state_machine.c), so this already
 *  happens inside sim_step(); this getter is exposed only so the UI can show /
 *  explain the timeout without hard-coding it.                                   */

/** @brief Continuous-on-ground time (ms) after which the box disarms. */
EMSCRIPTEN_KEEPALIVE
float sim_ground_reset_ms(void) { return GROUND_RESET_MS; }

/**
 * @brief 1 if @p mode pans the two streams apart (stereo), 0 if it duplicates L=R.
 * @details Mirrors audio_config_from_mode().stereo for the given AUDIO_MODE_*.
 */
EMSCRIPTEN_KEEPALIVE
int sim_mode_stereo(int mode)
{
    return (mode == AUDIO_MODE_STEREO_BOTH) ? 1 : 0;
}

/**
 * @brief 1 if @p mode plays the spoken callouts, else 0.
 * @details Mirrors audio_config_from_mode().callouts_enabled. Out-of-range falls
 *          back to the default mode's flag (never silences the box on bad input).
 */
EMSCRIPTEN_KEEPALIVE
int sim_mode_callouts(int mode)
{
    switch (mode) {
        case AUDIO_MODE_MONO_BOTH:     return 1;
        case AUDIO_MODE_STEREO_BOTH:   return 1;
        case AUDIO_MODE_MONO_CALLOUTS: return 1;
        case AUDIO_MODE_MONO_TONE:     return 0;
        default:                       return sim_mode_callouts(DEFAULT_AUDIO_MODE);
    }
}

/**
 * @brief 1 if @p mode sounds the presence tone, else 0.
 * @details Mirrors audio_config_from_mode().tone_enabled. Out-of-range falls back
 *          to the default mode's flag.
 */
EMSCRIPTEN_KEEPALIVE
int sim_mode_tone(int mode)
{
    switch (mode) {
        case AUDIO_MODE_MONO_BOTH:     return 1;
        case AUDIO_MODE_STEREO_BOTH:   return 1;
        case AUDIO_MODE_MONO_CALLOUTS: return 0;
        case AUDIO_MODE_MONO_TONE:     return 1;
        default:                       return sim_mode_tone(DEFAULT_AUDIO_MODE);
    }
}
