/**
 * @file    robust.c
 * @brief   Implementation of the median/MAD boot-buffer outlier filter.
 *
 * @details PURE: includes only the C standard library and config.h's
 *          pure-logic-safe region (MAX_VALID_FT, MAD_K, BOOT_BUFFER_N).
 */

#include "robust.h"
#include "config.h"

#include <math.h>
#include <string.h>

/*  The internal scratch is sized for the worst case: every stored reading plus
 *  the current one. Keeping it on the stack here makes the whole module
 *  allocation-free, which matters because it runs during early boot.           */
#define ROBUST_MAX_VALS (BOOT_BUFFER_N + 1)

/* ---------------------------------------------------------------------------
 *  Small insertion sort. For N <= 11 this is faster and far simpler than any
 *  qsort()/heapsort, with no function-pointer overhead and no recursion.
 * ------------------------------------------------------------------------- */
static void insertion_sort(float *a, size_t n)
{
    for (size_t i = 1; i < n; ++i) {
        float key = a[i];
        size_t j = i;
        /* Shift larger elements right until 'key' finds its slot. */
        while (j > 0 && a[j - 1] > key) {
            a[j] = a[j - 1];
            --j;
        }
        a[j] = key;
    }
}

float median_inplace(float *scratch, size_t n)
{
    if (n == 0) {
        return 0.0f;
    }
    insertion_sort(scratch, n);

    /* Odd count -> middle element. Even count -> mean of the two middles. */
    if (n & 1u) {
        return scratch[n / 2];
    }
    return 0.5f * (scratch[n / 2 - 1] + scratch[n / 2]);
}

/* ---------------------------------------------------------------------------
 *  A reading is "hard junk" if it is non-finite, negative, or implausibly high
 *  for a GROUND reading. The MAX_VALID_FT cap is a ground-zeroing junk guard,
 *  not an in-flight range limit.
 * ------------------------------------------------------------------------- */
static bool is_hard_junk(float v)
{
    if (!isfinite(v)) {
        return true;            /* inf / NaN                                    */
    }
    if (v < 0.0f) {
        return true;            /* negative distance is nonsense                */
    }
    if (v > MAX_VALID_FT) {
        return true;            /* too high to be a ground reading              */
    }
    return false;
}

float robust_estimate(const float *vals, size_t n, float current, bool *ok)
{
    /* Defensive clamp so a caller passing more than we budgeted can't overflow
     * the fixed scratch — we simply consider only the first ROBUST_MAX_VALS-1. */
    if (n > BOOT_BUFFER_N) {
        n = BOOT_BUFFER_N;
    }

    float surv[ROBUST_MAX_VALS];        /* survivors of the hard-junk pass      */
    size_t m = 0;

    /* --- Pass 1: hard rejects on the stored set + the current reading ------ */
    for (size_t i = 0; i < n; ++i) {
        if (!is_hard_junk(vals[i])) {
            surv[m++] = vals[i];
        }
    }
    bool current_ok = !is_hard_junk(current);
    if (current_ok) {
        surv[m++] = current;
    }

    /* Nothing survived -> no usable estimate. */
    if (m == 0) {
        if (ok) {
            *ok = false;
        }
        return 0.0f;
    }

    /* --- Pass 2: MAD-based robust rejection -------------------------------- */
    /*  median_inplace sorts 'surv', so take the median first, then compute the
     *  absolute deviations into a second scratch and median THOSE for the MAD.  */
    float sorted[ROBUST_MAX_VALS];
    memcpy(sorted, surv, m * sizeof(float));
    float med = median_inplace(sorted, m);

    float devs[ROBUST_MAX_VALS];
    for (size_t i = 0; i < m; ++i) {
        devs[i] = fabsf(surv[i] - med);
    }
    float mad = median_inplace(devs, m);

    /* --- Pass 3: average the inliers --------------------------------------- */
    float sum = 0.0f;
    size_t inliers = 0;
    for (size_t i = 0; i < m; ++i) {
        /* When MAD == 0 (all survivors equal) the test is meaningless — keep
         * every survivor rather than divide-by-nothing.                        */
        if (mad == 0.0f || fabsf(surv[i] - med) <= MAD_K * mad) {
            sum += surv[i];
            ++inliers;
        }
    }

    /* MAD math can in pathological cases reject everything; fall back to the
     * median so we always return SOMETHING usable when m > 0.                  */
    float estimate = (inliers > 0) ? (sum / (float)inliers) : med;

    /* Blend the inlier mean with the current reading when current is valid, so
     * the freshest sample pulls the estimate toward "now" without letting a
     * single noisy current reading dominate the historical ground set.         */
    if (current_ok) {
        estimate = 0.5f * estimate + 0.5f * current;
    }

    if (ok) {
        *ok = true;
    }
    return estimate;
}
