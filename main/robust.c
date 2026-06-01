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

/* ---------------------------------------------------------------------------
 *  Core: run the hard-junk + MAD pipeline over 'surv[0..m)' and return the
 *  inlier mean. Caller has already done the hard-junk pass into 'surv'. Returns
 *  false via *ok when m == 0.
 * ------------------------------------------------------------------------- */
static float inlier_mean(const float *surv, size_t m, bool *ok)
{
    if (m == 0) {
        if (ok) {
            *ok = false;
        }
        return 0.0f;
    }

    /* MAD: median first (on a copy, since median_inplace sorts), then the
     * median of the absolute deviations.                                      */
    float sorted[ROBUST_MAX_VALS];
    memcpy(sorted, surv, m * sizeof(float));
    float med = median_inplace(sorted, m);

    float devs[ROBUST_MAX_VALS];
    for (size_t i = 0; i < m; ++i) {
        devs[i] = fabsf(surv[i] - med);
    }
    float mad = median_inplace(devs, m);

    /* Average the inliers. When MAD == 0 (all equal) the test is meaningless,
     * so keep everything.                                                     */
    float sum = 0.0f;
    size_t inliers = 0;
    for (size_t i = 0; i < m; ++i) {
        if (mad == 0.0f || fabsf(surv[i] - med) <= MAD_K * mad) {
            sum += surv[i];
            ++inliers;
        }
    }

    if (ok) {
        *ok = true;
    }
    /* Pathological all-rejected case falls back to the median. */
    return (inliers > 0) ? (sum / (float)inliers) : med;
}

/* Hard-junk filter from a source array into 'surv'; returns survivor count. */
static size_t collect_survivors(const float *vals, size_t n, float *surv)
{
    size_t m = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!is_hard_junk(vals[i])) {
            surv[m++] = vals[i];
        }
    }
    return m;
}

float robust_mean(const float *vals, size_t n, bool *ok)
{
    if (n > BOOT_BUFFER_N) {
        n = BOOT_BUFFER_N;
    }
    float surv[ROBUST_MAX_VALS];
    size_t m = collect_survivors(vals, n, surv);
    return inlier_mean(surv, m, ok);
}

float robust_estimate(const float *vals, size_t n, float current, bool *ok)
{
    /* Defensive clamp so a caller passing more than we budgeted can't overflow
     * the fixed scratch.                                                       */
    if (n > BOOT_BUFFER_N) {
        n = BOOT_BUFFER_N;
    }

    float surv[ROBUST_MAX_VALS];
    size_t m = collect_survivors(vals, n, surv);

    bool current_ok = !is_hard_junk(current);
    if (current_ok) {
        surv[m++] = current;
    }

    bool got = false;
    float estimate = inlier_mean(surv, m, &got);
    if (!got) {
        if (ok) {
            *ok = false;
        }
        return 0.0f;
    }

    /* Blend with the current reading so the freshest sample pulls the estimate
     * toward "now" without a single noisy reading dominating the set.          */
    if (current_ok) {
        estimate = 0.5f * estimate + 0.5f * current;
    }

    if (ok) {
        *ok = true;
    }
    return estimate;
}
