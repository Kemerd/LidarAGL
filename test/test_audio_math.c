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
