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

/** @brief AGL (ft) at which the presence tone begins to fade in. */
EMSCRIPTEN_KEEPALIVE
float sim_tone_start_ft(void) { return TONE_START_FT; }

/** @brief AGL (ft) at/below which the tone reaches full presence. */
EMSCRIPTEN_KEEPALIVE
float sim_tone_full_ft(void) { return TONE_FULL_FT; }

/** @brief Top of the flare full-attention band (ft). */
EMSCRIPTEN_KEEPALIVE
float sim_flare_hi_ft(void) { return FLARE_BAND_HI; }

/** @brief Bottom of the flare full-attention band (ft). */
EMSCRIPTEN_KEEPALIVE
float sim_flare_lo_ft(void) { return FLARE_BAND_LO; }
