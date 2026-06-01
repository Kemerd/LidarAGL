/**
 * @file    test_robust.c
 * @brief   Host unit tests for the median/MAD boot-buffer outlier filter.
 */

#include "test_util.h"
#include "robust.h"
#include "config.h"
#include <math.h>

TEST_GLOBALS

int main(void)
{
    printf("== robust_estimate ==\n");

    bool ok;

    /* --- median_inplace sanity -------------------------------------------- */
    {
        float odd[5]  = { 5, 1, 3, 2, 4 };
        ASSERT_NEAR(median_inplace(odd, 5), 3.0f, 1e-6, "median of 5 -> 3");

        float even[4] = { 10, 2, 8, 4 };
        ASSERT_NEAR(median_inplace(even, 4), 6.0f, 1e-6, "median of 4 -> (4+8)/2");
    }

    /* --- Clean ground cluster: estimate sits in the cluster --------------- */
    {
        float g[3] = { 9.0f, 10.0f, 9.4f };
        float est = robust_estimate(g, 3, 9.2f, &ok);
        ASSERT_TRUE(ok, "clean cluster produces an estimate");
        /* inlier mean ~9.467 blended 50/50 with current 9.2 -> ~9.33 */
        ASSERT_NEAR(est, 9.33f, 0.2f, "clean cluster estimate near ground");
    }

    /* --- A single gross flier is rejected by MAD -------------------------- */
    {
        /* Four tight ground reads + one 49 ft flier (under the 50 ft hard cap
         * so it must be caught by MAD, not the hard-junk pass).               */
        float g[5] = { 9.0f, 9.2f, 9.1f, 8.9f, 49.0f };
        float est = robust_estimate(g, 5, 9.05f, &ok);
        ASSERT_TRUE(ok, "flier set still produces an estimate");
        ASSERT_TRUE(est < 12.0f, "MAD rejects the 49 ft flier");
    }

    /* --- Hard rejects: inf / NaN / negative / > MAX_VALID_FT --------------- */
    {
        float g[4] = { 9.0f, 9.1f, 9.2f, 9.05f };
        /* current is way above the ground cap -> treated as hard junk, but the
         * stored cluster still yields an estimate (no blend with current).     */
        float est = robust_estimate(g, 4, 999.0f, &ok);
        ASSERT_TRUE(ok, "junk current still leaves stored estimate");
        ASSERT_NEAR(est, 9.0875f, 0.3f, "estimate from stored cluster only");
    }
    {
        float bad[3] = { INFINITY, NAN, -5.0f };
        float est = robust_estimate(bad, 3, NAN, &ok);
        ASSERT_TRUE(!ok, "all-junk set yields ok=false");
        ASSERT_NEAR(est, 0.0f, 1e-6, "all-junk estimate is 0");
        (void)est;
    }

    /* --- MAD == 0 path: all survivors identical --------------------------- */
    {
        float g[4] = { 7.0f, 7.0f, 7.0f, 7.0f };
        float est = robust_estimate(g, 4, 7.0f, &ok);
        ASSERT_TRUE(ok, "all-equal set produces an estimate");
        ASSERT_NEAR(est, 7.0f, 1e-5, "all-equal estimate is that value");
    }

    /* --- Allocation-free at the maximum buffer size ----------------------- */
    {
        float g[BOOT_BUFFER_N];
        for (int i = 0; i < BOOT_BUFFER_N; ++i) {
            g[i] = 9.0f + 0.01f * (float)i;
        }
        float est = robust_estimate(g, BOOT_BUFFER_N, 9.05f, &ok);
        ASSERT_TRUE(ok, "full buffer handled without crash");
        ASSERT_NEAR(est, 9.05f, 0.2f, "full buffer estimate near cluster");
    }

    /* --- Empty stored set, only a current reading ------------------------- */
    {
        float est = robust_estimate(NULL, 0, 12.5f, &ok);
        ASSERT_TRUE(ok, "current-only produces an estimate");
        ASSERT_NEAR(est, 12.5f, 1e-5, "current-only estimate equals current");
    }

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
