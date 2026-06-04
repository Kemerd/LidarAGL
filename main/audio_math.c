/**
 * @file    audio_math.c
 * @brief   Implementation of the pure perceptual audio math.
 *
 * @details PURE: <math.h> + config.h pure region. No hardware, no state.
 */

#include "audio_math.h"
#include "config.h"

#include <math.h>
#include <stddef.h>

/* M_PI is not in the C standard (it's POSIX/_USE_MATH_DEFINES), so define it
 * locally to stay portable across MinGW, MSVC and the ESP-IDF toolchain.       */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* A "silent" dB floor well below the audible/maskable threshold. Returned above
 * TONE_START_FT so the audio engine can treat the tone as effectively off
 * without a separate branch (db_to_gain of this is ~1e-4).                     */
#define TONE_SILENT_DB (-80.0f)

/* The tone-start altitude (ft) the pitch + dB schedules anchor on. This is the
 * ONE configured value the otherwise-stateless math holds: it defaults to the
 * compile-time TONE_START_FT and is overwritten ONCE at boot, before the render
 * task runs, by audio_set_tone_start() (which the pilot drives via the config
 * menu). The functions stay pure/deterministic for a given anchor — host tests
 * and the emulator never touch the setter, so they evaluate at the 100 ft default
 * exactly as before. See TONE_START_FT_HIGH / NVS_KEY_TONESTART in config.h.      */
static float s_tone_start_ft = TONE_START_FT;

/* The altitude (ft) at which the dB swell reaches FULL presence. It tracks the
 * tone-start above so the fade-in always occupies the SAME proportion of the
 * descent (its top half), instead of dragging down to a fixed floor:
 *   - 100 ft start -> full at 50 ft (TONE_FULL_FT, exactly as before)
 *   - 200 ft start -> full at 100 ft (the swell runs 200 -> 100, not 200 -> 50)
 * Refreshed alongside s_tone_start_ft by audio_math_set_tone_start().           */
static float s_tone_full_ft = TONE_FULL_FT;

/* Clamp helper. */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void audio_math_set_tone_start(float ft)
{
    /* Guard against a nonsensical value silencing/inverting the schedule: a
     * tone-start at or below TONE_FULL_FT would leave no fade-in band at all, so
     * fall back to the default if asked for something below it.                  */
    s_tone_start_ft = (ft > TONE_FULL_FT) ? ft : TONE_START_FT;

    /* Keep the fade-in band a constant FRACTION of the descent: the full-presence
     * altitude scales with the chosen start, so a 200 ft start swells in over
     * 200 -> 100 ft rather than the over-long 200 -> 50 ft. At the 100 ft default
     * this is exactly TONE_FULL_FT (50 ft), leaving that mode byte-identical.     */
    s_tone_full_ft = TONE_FULL_FT * (s_tone_start_ft / TONE_START_FT);
}

float agl_to_pitch_hz(float agl_ft)
{
    /* Constrain to the active band. Above the tone-start height we still return
     * the start frequency (the engine keeps it silent up there via the dB
     * schedule), which keeps the function continuous and warble-free at the
     * fade-in boundary.                                                         */
    float a = clampf(agl_ft, 0.0f, s_tone_start_ft);

    /* t goes 0 at the top of the band (the tone-start altitude) to 1 at the ground. */
    float t = (s_tone_start_ft - a) / s_tone_start_ft;

    float f;
#if TONE_LOG_SWEEP
    /* Exponential (musical) glide: equal ratios per equal AGL step. */
    f = F_AT_TONE_START * powf(F_AT_GROUND / F_AT_TONE_START, t);
#else
    /* Linear-in-Hz glide. */
    f = F_AT_TONE_START + t * (F_AT_GROUND - F_AT_TONE_START);
#endif

    /* Keep all energy in the cockpit-friendly, ANR-surviving band. */
    return clampf(f, F_CLAMP_LO, F_CLAMP_HI);
}

float agl_to_tone_db(float agl_ft)
{
    /* Off above the swell band. */
    if (agl_ft > s_tone_start_ft) {
        return TONE_SILENT_DB;
    }

    /* Full presence at and below the (scaled) full altitude — through flare to
     * the ground. For the 200 ft start this is 100 ft, so the tone is already at
     * full presence by 100 ft and simply holds the rest of the way down.        */
    if (agl_ft <= s_tone_full_ft) {
        return TONE_FULL_DB;
    }

    /* Between the tone-start altitude and the full altitude: ramp the LEVEL IN dB
     * linearly. frac = 0 at the start (floor) -> 1 at s_tone_full_ft (full).     */
    float span = s_tone_start_ft - s_tone_full_ft;      /* 50 ft @100 / 100 @200 */
    float frac = (s_tone_start_ft - agl_ft) / span;     /* 0..1 */
    frac = clampf(frac, 0.0f, 1.0f);

    return TONE_FLOOR_DB + frac * (TONE_FULL_DB - TONE_FLOOR_DB);
}

/* ---------------------------------------------------------------------------
 *  Equal-loudness flattening table (ISO 226:2003, ~60 phon).
 *
 *  Each entry is { frequency Hz, correction dB }. The correction is the dB to
 *  ADD to the scheduled level so equal scheduled dB sounds equally loud across
 *  the sweep, referenced to 1 kHz (so 1 kHz == 0). Values were computed directly
 *  from the ISO 226 inverse formula at 60 phon for our 500–3000 Hz working band:
 *  where the ear is MORE sensitive the correction is NEGATIVE (attenuate); where
 *  LESS sensitive it is POSITIVE (boost). The dip near 1250–1600 Hz is genuine
 *  to the standard (ear-canal resonance behaviour), not a fudge.
 * ------------------------------------------------------------------------- */
typedef struct { float hz; float corr_db; } eql_point_t;

static const eql_point_t EQL_TABLE[] = {
    {  500.0f, +2.04f },
    {  630.0f, +0.80f },
    {  800.0f, -0.12f },
    { 1000.0f,  0.00f },
    { 1250.0f, +2.14f },
    { 1600.0f, +3.18f },
    { 2000.0f, -0.05f },
    { 2500.0f, -2.76f },
    { 3150.0f, -3.59f },
};
#define EQL_N (sizeof(EQL_TABLE) / sizeof(EQL_TABLE[0]))

float equal_loudness_db(float freq_hz)
{
#if EQUAL_LOUDNESS_CORRECTION
    /* Clamp to the table ends (our tone is already clamped to 500–3000 Hz). */
    if (freq_hz <= EQL_TABLE[0].hz) {
        return EQL_TABLE[0].corr_db;
    }
    if (freq_hz >= EQL_TABLE[EQL_N - 1].hz) {
        return EQL_TABLE[EQL_N - 1].corr_db;
    }
    /* Linear interpolation between the two bracketing points. */
    for (size_t i = 0; i + 1 < EQL_N; ++i) {
        float f0 = EQL_TABLE[i].hz;
        float f1 = EQL_TABLE[i + 1].hz;
        if (freq_hz >= f0 && freq_hz <= f1) {
            float t = (freq_hz - f0) / (f1 - f0);
            return EQL_TABLE[i].corr_db +
                   t * (EQL_TABLE[i + 1].corr_db - EQL_TABLE[i].corr_db);
        }
    }
    return 0.0f;   /* unreachable */
#else
    (void)freq_hz;
    return 0.0f;   /* correction disabled: no change */
#endif
}

float db_to_gain(float db)
{
    return powf(10.0f, db / 20.0f);
}

float raised_cosine(float t01)
{
    float t = clampf(t01, 0.0f, 1.0f);
    return 0.5f - 0.5f * cosf((float)M_PI * t);
}

float slew_limit(float cur, float target, float max_delta)
{
    float d = target - cur;
    if (d >  max_delta) d =  max_delta;
    if (d < -max_delta) d = -max_delta;
    return cur + d;
}
