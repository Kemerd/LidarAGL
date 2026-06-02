/**
 * @file    test_audio_math.c
 * @brief   Host unit tests for the perceptual audio math.
 */

#include "test_util.h"
#include "audio_math.h"
#include "config.h"

#include <math.h>
#include <stdbool.h>

TEST_GLOBALS

int main(void)
{
    printf("== audio_math ==\n");

    /* --- Pitch: ascends as AGL falls -------------------------------------- */
    {
        /* Endpoints land on the configured frequencies (within clamp band). */
        ASSERT_NEAR(agl_to_pitch_hz(TONE_START_FT), F_AT_TONE_START, 1.0f,
                    "pitch at 100 ft == F_AT_TONE_START");
        ASSERT_NEAR(agl_to_pitch_hz(0.0f), F_AT_GROUND, 1.0f,
                    "pitch at 0 ft == F_AT_GROUND");

        /* Monotonic increase as AGL decreases from 100 -> 0. */
        bool ascending = true;
        float prev = agl_to_pitch_hz(100.0f);
        for (float a = 99.0f; a >= 0.0f; a -= 1.0f) {
            float f = agl_to_pitch_hz(a);
            if (f < prev - 0.001f) {     /* must not go DOWN as AGL falls */
                ascending = false;
            }
            prev = f;
        }
        ASSERT_TRUE(ascending, "pitch is monotonically ascending as AGL falls");

        /* Stays inside the 500-3000 Hz band everywhere. */
        bool in_band = true;
        for (float a = -10.0f; a <= 150.0f; a += 1.0f) {
            float f = agl_to_pitch_hz(a);
            if (f < F_CLAMP_LO - 0.5f || f > F_CLAMP_HI + 0.5f) {
                in_band = false;
            }
        }
        ASSERT_TRUE(in_band, "pitch clamped to 500-3000 Hz band");
    }

    /* --- dB schedule ------------------------------------------------------ */
    {
        /* Silent above the swell band. */
        ASSERT_TRUE(agl_to_tone_db(150.0f) <= TONE_FLOOR_DB - 10.0f,
                    "tone effectively silent above 100 ft");

        /* Floor at the fade-in height. */
        ASSERT_NEAR(agl_to_tone_db(TONE_START_FT), TONE_FLOOR_DB, 0.01f,
                    "dB == FLOOR at 100 ft");

        /* Full presence reached by TONE_FULL_FT and held below it. */
        ASSERT_NEAR(agl_to_tone_db(TONE_FULL_FT), TONE_FULL_DB, 0.01f,
                    "dB == FULL at 50 ft");
        ASSERT_NEAR(agl_to_tone_db(10.0f), TONE_FULL_DB, 0.01f,
                    "dB held at FULL below 50 ft");
        ASSERT_NEAR(agl_to_tone_db(0.0f), TONE_FULL_DB, 0.01f,
                    "dB held at FULL on the ground");

        /* Monotonic rise (louder) as AGL falls through the ramp. */
        bool louder = true;
        float prev = agl_to_tone_db(100.0f);
        for (float a = 99.0f; a >= 50.0f; a -= 1.0f) {
            float db = agl_to_tone_db(a);
            if (db < prev - 0.001f) {
                louder = false;
            }
            prev = db;
        }
        ASSERT_TRUE(louder, "dB rises monotonically from 100 to 50 ft");

        /* Midpoint sanity: at 75 ft we should be roughly halfway in dB. */
        float mid = agl_to_tone_db(75.0f);
        float halfway = TONE_FLOOR_DB + 0.5f * (TONE_FULL_DB - TONE_FLOOR_DB);
        ASSERT_NEAR(mid, halfway, 0.5f, "dB ~halfway at 75 ft");
    }

    /* --- equal-loudness correction (ISO 226) ------------------------------ */
    {
        /* 1 kHz is the reference: zero correction. */
        ASSERT_NEAR(equal_loudness_db(1000.0f), 0.0f, 0.05f,
                    "equal-loudness is 0 dB at 1 kHz");

        /* Where the ear is LESS sensitive (low freqs in band) -> positive boost. */
        ASSERT_TRUE(equal_loudness_db(500.0f) > 1.0f,
                    "500 Hz gets a positive (boost) correction");

        /* Where the ear is MORE sensitive (~2.5-3 kHz) -> negative (attenuate). */
        ASSERT_TRUE(equal_loudness_db(2500.0f) < -1.0f,
                    "2500 Hz gets a negative (attenuate) correction");

        /* Interpolation between table points is monotonic on a known segment
         * (2000 Hz -> 2500 Hz both attenuate increasingly). */
        ASSERT_TRUE(equal_loudness_db(2500.0f) < equal_loudness_db(2000.0f),
                    "correction interpolates between table points");

        /* Clamps below/above the table without blowing up. */
        ASSERT_NEAR(equal_loudness_db(400.0f), equal_loudness_db(500.0f), 0.001f,
                    "below-band freq clamps to first point");
        ASSERT_NEAR(equal_loudness_db(5000.0f), equal_loudness_db(3150.0f), 0.001f,
                    "above-band freq clamps to last point");
    }

    /* --- THE key behaviour: perceived loudness is FLAT through the flare ---- */
    {
        /* Independent model of the ear's sensitivity (positive = MORE sensitive
         * vs 1 kHz), from the same ISO 226 @ 60 phon data the firmware's
         * correction is built to cancel. This is the INVERSE of the correction
         * table, computed here independently so the test isn't tautological. */
        struct { float hz, sens; } EAR[] = {
            {500.f,-2.04f},{630.f,-0.80f},{800.f,0.12f},{1000.f,0.0f},
            {1250.f,-2.14f},{1600.f,-3.18f},{2000.f,0.05f},{2500.f,2.76f},{3150.f,3.59f},
        };
        const int EN = (int)(sizeof EAR / sizeof EAR[0]);
        /* interpolate the independent ear-sensitivity model */
        /* (local lambda-free helper) */
        #define EAR_SENS(F) ({ float _f=(F); float _r; \
            if (_f<=EAR[0].hz) _r=EAR[0].sens; \
            else if (_f>=EAR[EN-1].hz) _r=EAR[EN-1].sens; \
            else { _r=0; for(int _i=0;_i+1<EN;++_i){ if(_f>=EAR[_i].hz&&_f<=EAR[_i+1].hz){ \
                float _t=(_f-EAR[_i].hz)/(EAR[_i+1].hz-EAR[_i].hz); \
                _r=EAR[_i].sens+_t*(EAR[_i+1].sens-EAR[_i].sens); break; } } } _r; })

        /* UNCORRECTED perceived loudness (scheduled + ear sensitivity) WANDERS
         * through the flare as the pitch sweeps across the ear's uneven
         * sensitivity — confirm the problem (loudness not constant) exists. */
        float ulo = 1e9f, uhi = -1e9f;
        for (float agl = 50.0f; agl >= 0.0f; agl -= 2.0f) {
            float f = agl_to_pitch_hz(agl);
            float perceived = agl_to_tone_db(agl) + EAR_SENS(f);
            if (perceived < ulo) ulo = perceived;
            if (perceived > uhi) uhi = perceived;
        }
        ASSERT_TRUE((uhi - ulo) > 1.0f,
                    "uncorrected: perceived loudness wanders >1 dB through the flare (the problem)");

        /* CORRECTED perceived loudness (scheduled + firmware correction + ear)
         * stays flat: the correction cancels the ear tilt, leaving the constant
         * scheduled level. Check the spread across the whole flare band. */
        float lo = 1e9f, hi = -1e9f;
        for (float agl = 50.0f; agl >= 0.0f; agl -= 2.0f) {
            float f = agl_to_pitch_hz(agl);
            float perceived = agl_to_tone_db(agl) + equal_loudness_db(f) + EAR_SENS(f);
            if (perceived < lo) lo = perceived;
            if (perceived > hi) hi = perceived;
        }
        ASSERT_TRUE((hi - lo) < 0.2f,
                    "corrected: perceived loudness is FLAT through the flare (<0.2 dB)");

        #undef EAR_SENS
    }

    /* --- db_to_gain ------------------------------------------------------- */
    {
        ASSERT_NEAR(db_to_gain(0.0f),   1.0f,   1e-4, "0 dB -> gain 1.0");
        ASSERT_NEAR(db_to_gain(-6.0f),  0.5012f, 1e-3, "-6 dB -> ~0.501");
        ASSERT_NEAR(db_to_gain(-20.0f), 0.1f,   1e-4, "-20 dB -> 0.1");
        ASSERT_NEAR(db_to_gain(-40.0f), 0.01f,  1e-4, "-40 dB -> 0.01");
    }

    /* --- raised-cosine envelope ------------------------------------------- */
    {
        ASSERT_NEAR(raised_cosine(0.0f), 0.0f, 1e-5, "envelope(0) == 0");
        ASSERT_NEAR(raised_cosine(1.0f), 1.0f, 1e-5, "envelope(1) == 1");
        ASSERT_NEAR(raised_cosine(0.5f), 0.5f, 1e-5, "envelope(0.5) == 0.5");

        bool monotone = true;
        float prev = raised_cosine(0.0f);
        for (float t = 0.0f; t <= 1.0f; t += 0.05f) {
            float v = raised_cosine(t);
            if (v < prev - 1e-4f) {
                monotone = false;
            }
            prev = v;
        }
        ASSERT_TRUE(monotone, "envelope is monotonically increasing");

        /* Clamps outside [0,1]. */
        ASSERT_NEAR(raised_cosine(-0.5f), 0.0f, 1e-5, "envelope clamps below 0");
        ASSERT_NEAR(raised_cosine(2.0f),  1.0f, 1e-5, "envelope clamps above 1");
    }

    /* --- slew limiter ----------------------------------------------------- */
    {
        ASSERT_NEAR(slew_limit(0.0f, 10.0f, 2.0f), 2.0f, 1e-5, "slew up clamps to +max");
        ASSERT_NEAR(slew_limit(0.0f, -10.0f, 2.0f), -2.0f, 1e-5, "slew down clamps to -max");
        ASSERT_NEAR(slew_limit(5.0f, 5.3f, 2.0f), 5.3f, 1e-5, "slew within max passes through");
    }

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
