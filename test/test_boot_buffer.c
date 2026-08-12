/**
 * @file    test_boot_buffer.c
 * @brief   Host unit tests for the ground-reference resolver.
 *
 * @details boot_buffer_resolve() decides three things that the entire rest of
 *          the box depends on: WHERE THE GROUND IS, whether the aircraft was
 *          airborne when it booted, and whether the calibration can be trusted
 *          at all. Every altitude the pilot ever hears is measured against its
 *          answer, and one of its outputs (calib_error) gates whether that
 *          answer is written to NVS and reused on every subsequent flight.
 *
 *          It is pure logic, yet it shipped with no coverage — the file's NVS
 *          and GPIO includes made the whole translation unit firmware-only.
 *          These tests exist because the failures it can produce are silent by
 *          construction: a wrong ground reference does not crash, it simply
 *          offsets every callout, and the pilot has no way to tell.
 *
 *          The cases below are the physical situations an aircraft actually
 *          gets into on a ramp — someone under the sensor, the aircraft on
 *          jacks, a power glitch after a bounce, a dead sensor — not abstract
 *          input fuzzing.
 */

#include "test_util.h"
#include "boot_buffer.h"
#include "config.h"

#include <math.h>

TEST_GLOBALS

/* Fill a stored-entry set with one repeated ground reading. */
static void fill_stored(boot_entry_t *e, size_t n, float ft)
{
    for (size_t i = 0; i < n; ++i) {
        e[i].range_ft = ft;
        e[i].marker   = 0u;
    }
}

/* ---------------------------------------------------------------------------
 *  A normal, healthy parked boot.
 * ------------------------------------------------------------------------- */
static void test_normal_parked_boot(void)
{
    boot_entry_t stored[BOOT_BUFFER_N];
    boot_result_t r;

    fill_stored(stored, BOOT_BUFFER_N, 3.0f);
    boot_buffer_resolve(stored, BOOT_BUFFER_N, 3.1f, true, &r);

    ASSERT_TRUE(!r.airborne,    "parked boot is not airborne");
    ASSERT_TRUE(!r.calib_error, "parked boot raises no calibration error");
    ASSERT_NEAR(r.ground_ref_ft, 3.0f, 0.2f, "parked boot keeps the learned ground");
    ASSERT_NEAR(r.boot_agl_ft,   0.1f, 0.2f, "parked boot reports ~0 ft AGL");
}

/* ---------------------------------------------------------------------------
 *  In-flight reboot: the stored ground must be KEPT, and the box must know it
 *  is airborne so the caller can seed the ladder hot instead of parked.
 * ------------------------------------------------------------------------- */
static void test_inflight_reboot(void)
{
    boot_entry_t stored[BOOT_BUFFER_N];
    boot_result_t r;

    fill_stored(stored, BOOT_BUFFER_N, 3.0f);
    boot_buffer_resolve(stored, BOOT_BUFFER_N, 150.0f, true, &r);

    ASSERT_TRUE(r.airborne,     "in-flight reboot is detected as airborne");
    ASSERT_TRUE(!r.calib_error, "in-flight reboot is not a calibration error");
    ASSERT_NEAR(r.ground_ref_ft, 3.0f, 0.2f,
                "in-flight reboot KEEPS the learned ground (never overwrites)");
    ASSERT_NEAR(r.boot_agl_ft, 147.0f, 0.5f,
                "in-flight reboot reports AGL above the learned ground");
}

/* ---------------------------------------------------------------------------
 *  REGRESSION: an obstruction under the sensor must not be adopted as ground.
 *
 *  A mechanic's shoulder, a tow bar, a chock, or a puddle giving a specular
 *  return all read CLOSER than the surface the box learned on. The old test
 *  looked only at the high side and then cleared calib_error unconditionally,
 *  so this clamped to agl = 0 and reported a perfectly healthy parked boot —
 *  whereupon the caller's persist gate (which keys off !airborne && !calib_error)
 *  WROTE THE OBSTRUCTED READING TO NVS as the new learned ground.
 *
 *  That is the worst failure this file can produce: it is not confined to the
 *  current flight, it silently offsets every future one until someone
 *  recalibrates on a clean surface.
 * ------------------------------------------------------------------------- */
static void test_obstruction_under_sensor(void)
{
    boot_entry_t stored[BOOT_BUFFER_N];
    boot_result_t r;

    fill_stored(stored, BOOT_BUFFER_N, 3.0f);

    /* Something solid sits well inside the learned ground. */
    boot_buffer_resolve(stored, BOOT_BUFFER_N, 0.4f, true, &r);
    ASSERT_TRUE(r.calib_error,
                "obstruction far BELOW the learned ground raises calib_error");
    ASSERT_TRUE(!r.airborne, "an obstruction is not 'airborne'");

    /* A few inches of surface variation is normal and must stay clean. */
    boot_buffer_resolve(stored, BOOT_BUFFER_N, 2.6f, true, &r);
    ASSERT_TRUE(!r.calib_error,
                "ordinary surface variation does NOT raise calib_error");
}

/* ---------------------------------------------------------------------------
 *  Sanity on the boundary itself, from both directions.
 * ------------------------------------------------------------------------- */
static void test_disagreement_is_two_sided(void)
{
    boot_entry_t stored[BOOT_BUFFER_N];
    boot_result_t r;
    fill_stored(stored, BOOT_BUFFER_N, 10.0f);

    /*  The low side uses GROUND_BELOW_DEV_FT, NOT GROUND_DEV_FT — the ground is
     *  only a mount-height away, so the downward tolerance must be scaled to
     *  the mount or the test can never fire.                                   */
    boot_buffer_resolve(stored, BOOT_BUFFER_N,
                        10.0f - (GROUND_BELOW_DEV_FT * 0.5f), true, &r);
    ASSERT_TRUE(!r.calib_error, "just inside the low band is trusted");

    /* Just outside on the low side: flagged. */
    boot_buffer_resolve(stored, BOOT_BUFFER_N,
                        10.0f - (GROUND_BELOW_DEV_FT + 1.0f), true, &r);
    ASSERT_TRUE(r.calib_error, "just outside the low band is flagged");

    /* Just outside on the HIGH side is airborne, not a calibration error —
     * that is a real flight condition, not a broken reference.              */
    boot_buffer_resolve(stored, BOOT_BUFFER_N, 10.0f + (GROUND_DEV_FT + 1.0f),
                        true, &r);
    ASSERT_TRUE(r.airborne,     "just outside the high band is airborne");
    ASSERT_TRUE(!r.calib_error, "the high side is airborne, NOT a calib error");
}

/* ---------------------------------------------------------------------------
 *  REGRESSION: first boot (empty NVS) must not adopt an airborne reading.
 *
 *  The config menu wipes the ground buffer, so "empty stored set" is reachable
 *  on a unit that has flown for years. The adoption band was
 *  MOUNT_OFFSET_FALLBACK_FT + GROUND_DEV_FT = 13 ft — over four times a real
 *  mount and squarely inside the altitudes a bounce or a low pass occupies. A
 *  power glitch at 12 ft AGL therefore adopted 12.4 ft as "ground" with
 *  calib_error CLEAR: no chirp, the ladder seeded disarmed for the whole
 *  landing, and the airborne value persisted for later flights.
 * ------------------------------------------------------------------------- */
static void test_first_boot_band(void)
{
    boot_result_t r;

    /* A plausible mount reading is adopted silently — the normal install case. */
    boot_buffer_resolve(NULL, 0, 3.0f, true, &r);
    ASSERT_TRUE(!r.calib_error, "first boot adopts a plausible mount reading");
    ASSERT_NEAR(r.ground_ref_ft, 3.0f, 0.01f,
                "first boot uses the reading as the ground");

    /* An airborne reading must NOT become the ground. */
    boot_buffer_resolve(NULL, 0, 12.0f, true, &r);
    ASSERT_TRUE(r.calib_error,
                "first boot at 12 ft raises calib_error (was silently adopted)");
    ASSERT_NEAR(r.ground_ref_ft, MOUNT_OFFSET_FALLBACK_FT, 0.01f,
                "first boot at 12 ft falls back to the emergency offset");

    /* Right at the boundary, both sides. */
    boot_buffer_resolve(NULL, 0, MOUNT_GROUND_MAX_FT - 0.5f, true, &r);
    ASSERT_TRUE(!r.calib_error, "just inside the mount band is adopted");
    boot_buffer_resolve(NULL, 0, MOUNT_GROUND_MAX_FT + 0.5f, true, &r);
    ASSERT_TRUE(r.calib_error,  "just outside the mount band is flagged");
}

/* ---------------------------------------------------------------------------
 *  A totally silent sensor must not produce a confident ground.
 * ------------------------------------------------------------------------- */
static void test_dead_sensor_boot(void)
{
    boot_result_t r;

    /* No stored reference AND no current reading: nothing to go on. */
    boot_buffer_resolve(NULL, 0, 0.0f, false, &r);
    ASSERT_TRUE(r.calib_error,
                "dead sensor + empty NVS raises calib_error (chirps the pilot)");
    ASSERT_NEAR(r.ground_ref_ft, MOUNT_OFFSET_FALLBACK_FT, 0.01f,
                "dead sensor falls back to the emergency offset");
}

/* ---------------------------------------------------------------------------
 *  NaN must never reach the outputs.
 *
 *  Every decision in the resolver is a floating-point ordering test, and every
 *  such test against NaN is FALSE — so a NaN takes whichever branch the code
 *  falls through to, and the `boot_agl < 0` clamp is provably ineffective
 *  against it. A NaN ground reference would make every subsequent AGL NaN,
 *  which downstream poisons the audio followers permanently.
 * ------------------------------------------------------------------------- */
static void test_nan_current_reading(void)
{
    boot_entry_t stored[BOOT_BUFFER_N];
    boot_result_t r;

    fill_stored(stored, BOOT_BUFFER_N, 3.0f);
    boot_buffer_resolve(stored, BOOT_BUFFER_N, NAN, true, &r);
    ASSERT_TRUE(isfinite(r.ground_ref_ft),
                "NaN current -> ground_ref stays finite");
    ASSERT_TRUE(isfinite(r.boot_agl_ft),
                "NaN current -> boot_agl stays finite");
    ASSERT_NEAR(r.ground_ref_ft, 3.0f, 0.2f,
                "NaN current -> the learned ground is kept unchanged");

    /* Same with no stored reference to fall back on. */
    boot_buffer_resolve(NULL, 0, NAN, true, &r);
    ASSERT_TRUE(isfinite(r.ground_ref_ft) && isfinite(r.boot_agl_ft),
                "NaN on a first boot still yields finite outputs");
    ASSERT_TRUE(r.calib_error, "NaN on a first boot raises calib_error");

    /* Infinity is the same class of input. */
    boot_buffer_resolve(stored, BOOT_BUFFER_N, INFINITY, true, &r);
    ASSERT_TRUE(isfinite(r.ground_ref_ft) && isfinite(r.boot_agl_ft),
                "infinite current -> outputs stay finite");
}

/* ---------------------------------------------------------------------------
 *  A corrupt stored set must not become the ground.
 * ------------------------------------------------------------------------- */
static void test_corrupt_stored_entries(void)
{
    boot_entry_t stored[BOOT_BUFFER_N];
    boot_result_t r;

    /* Stored values that are physically impossible as a ground reading. */
    for (size_t i = 0; i < BOOT_BUFFER_N; ++i) {
        stored[i].range_ft = 400.0f;    /* far beyond any mount */
        stored[i].marker   = 0u;
    }
    boot_buffer_resolve(stored, BOOT_BUFFER_N, 3.0f, true, &r);
    ASSERT_TRUE(isfinite(r.ground_ref_ft),
                "implausible stored set -> ground_ref stays finite");
    ASSERT_TRUE(r.ground_ref_ft >= 0.0f,
                "implausible stored set -> ground_ref stays non-negative");
}

int main(void)
{
    printf("== boot_buffer_resolve ==\n");
    test_normal_parked_boot();
    test_inflight_reboot();
    test_obstruction_under_sensor();
    test_disagreement_is_two_sided();
    test_first_boot_band();
    test_dead_sensor_boot();
    test_nan_current_reading();
    test_corrupt_stored_entries();

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
