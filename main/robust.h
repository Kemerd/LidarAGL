/**
 * @file    robust.h
 * @brief   Cheap, robust outlier rejection for the small boot-buffer set.
 *
 * @details The boot sanity filter (spec §5.3) must reject gross fliers from a
 *          tiny set of readings (<= BOOT_BUFFER_N + 1) without any heavy stats
 *          libraries and without heap allocation — it runs early in boot. We use
 *          the median + MAD (median absolute deviation), which is robust to the
 *          very outliers we are removing and is just two cheap median passes.
 *
 *          PURE module: no ESP-IDF headers, host-testable.
 */
#ifndef LIDARAGL_ROBUST_H
#define LIDARAGL_ROBUST_H

#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Sort @p scratch ascending in place and return its median.
 *
 * @param scratch  Array to sort (CONTENTS ARE MODIFIED). Must hold @p n floats.
 * @param n        Number of valid entries (>= 1).
 * @return         Median value (for even @p n, the mean of the two middles).
 */
float median_inplace(float *scratch, size_t n);

/**
 * @brief Robustly estimate a single value from a noisy set + a current reading.
 *
 * @details Pipeline (allocation-free; uses a fixed internal scratch):
 *            1. Hard reject inf / NaN / negatives / values > MAX_VALID_FT.
 *            2. On the survivors, reject any point more than MAD_K * MAD from
 *               the median (skipped when MAD == 0, i.e. all-equal survivors).
 *            3. Return the mean of the inliers, blended with @p current.
 *
 * @param vals     Stored historical readings (may be NULL when @p n == 0).
 * @param n        Number of entries in @p vals (0 .. BOOT_BUFFER_N).
 * @param current  The freshly acquired reading to blend in.
 * @param[out] ok  Set true if a usable estimate was produced, false if every
 *                 value (including @p current) was rejected as junk.
 * @return         The robust estimate when @p *ok is true; 0.0f otherwise.
 */
float robust_estimate(const float *vals, size_t n, float current, bool *ok);

#endif /* LIDARAGL_ROBUST_H */
