/**
 * @file    audio_math.c
 * @brief   Implementation of the pure perceptual audio math.
 *
 * @details PURE: <math.h> + config.h pure region. No hardware, no state.
 */

#include "audio_math.h"
#include "config.h"

#include <math.h>

/* M_PI is not in the C standard (it's POSIX/_USE_MATH_DEFINES), so define it
 * locally to stay portable across MinGW, MSVC and the ESP-IDF toolchain.       */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* A "silent" dB floor well below the audible/maskable threshold. Returned above
 * TONE_START_FT so the audio engine can treat the tone as effectively off
 * without a separate branch (db_to_gain of this is ~1e-4).                     */
#define TONE_SILENT_DB (-80.0f)

/* Clamp helper. */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float agl_to_pitch_hz(float agl_ft)
{
    /* Constrain to the active band. Above the tone-start height we still return
     * the start frequency (the engine keeps it silent up there via the dB
     * schedule), which keeps the function continuous and warble-free at the
     * fade-in boundary.                                                         */
    float a = clampf(agl_ft, 0.0f, TONE_START_FT);

    /* t goes 0 at the top of the band (100 ft) to 1 at the ground. */
    float t = (TONE_START_FT - a) / TONE_START_FT;

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
    if (agl_ft > TONE_START_FT) {
        return TONE_SILENT_DB;
    }

    /* Full presence at and below TONE_FULL_FT (through flare to the ground). */
    if (agl_ft <= TONE_FULL_FT) {
        return TONE_FULL_DB;
    }

    /* Between TONE_START_FT and TONE_FULL_FT: ramp the LEVEL IN dB linearly.
     * frac = 0 at TONE_START_FT (floor) -> 1 at TONE_FULL_FT (full).           */
    float span = TONE_START_FT - TONE_FULL_FT;          /* e.g. 50 ft */
    float frac = (TONE_START_FT - agl_ft) / span;       /* 0..1 */
    frac = clampf(frac, 0.0f, 1.0f);

    return TONE_FLOOR_DB + frac * (TONE_FULL_DB - TONE_FLOOR_DB);
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
