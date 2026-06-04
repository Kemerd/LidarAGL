/**
 * @file    sensor_profile.c
 * @brief   The two built-in sensor profiles and the selectors that pick one.
 *
 * @details PURE module — no ESP-IDF dependencies, fully host-testable. The
 *          callout tables encode the user's headline requirement directly:
 *          SF30/C starts at 200 ft; SF30/D prepends 500/400/300 ft.
 */

#include "sensor_profile.h"
#include <string.h>
#include <ctype.h>

/* ---------------------------------------------------------------------------
 *  SF30/C callout ladder (descending).
 *
 *  Highest callout is 305 ft on purpose: the SF30/C ceiling is ~328 ft, so a
 *  328 ft callout would sit inside the sensor's noisy upper margin. Starting at
 *  305 gives the sensor headroom and keeps every spoken number trustworthy.
 * ------------------------------------------------------------------------- */
static const float SF30C_CALLOUTS[] = {
    300.0f, 200.0f, 100.0f, 50.0f, 40.0f, 30.0f, 20.0f, 10.0f
};

/* ---------------------------------------------------------------------------
 *  SF30/D callout ladder (descending).
 *
 *  The SF30/D reaches ~656 ft, so it adds the 600/500/400/300 ft high-altitude
 *  callouts requested for the long-range unit, then shares the same flare-band
 *  numbers as the SF30/C below 200 ft.
 * ------------------------------------------------------------------------- */
static const float SF30D_CALLOUTS[] = {
    600.0f, 500.0f, 400.0f, 300.0f, 200.0f, 100.0f, 50.0f, 40.0f, 30.0f, 20.0f, 10.0f
};

/* ---- The immutable profiles --------------------------------------------- */

const sensor_profile_t SF30C_PROFILE = {
    .model        = SENSOR_SF30C,
    .name         = "SF30/C",
    .max_range_ft = 328.0f,                 /* 100 m                          */
    .callouts     = SF30C_CALLOUTS,
    .n_callouts   = sizeof(SF30C_CALLOUTS) / sizeof(SF30C_CALLOUTS[0]),
    .cruise_ft    = 305.0f,                 /* margin below the 328 ft ceiling */
};

const sensor_profile_t SF30D_PROFILE = {
    .model        = SENSOR_SF30D,
    .name         = "SF30/D",
    .max_range_ft = 656.0f,                 /* 200 m                          */
    .callouts     = SF30D_CALLOUTS,
    .n_callouts   = sizeof(SF30D_CALLOUTS) / sizeof(SF30D_CALLOUTS[0]),
    .cruise_ft    = 605.0f,                 /* sleep above the top callout     */
};

/* ---- Selectors ----------------------------------------------------------- */

const sensor_profile_t *sensor_profile_for_model(sensor_model_t m)
{
    /* Default to SF30/C for any unknown value so the caller always gets a
     * usable profile back — there is no failure path here.                    */
    return (m == SENSOR_SF30D) ? &SF30D_PROFILE : &SF30C_PROFILE;
}

/**
 * @brief Case-insensitive substring search (host/firmware portable).
 *
 * The C library's strcasestr() is not available everywhere (MSVC, some libcs),
 * so we roll a tiny one. @p hay/@p needle are NUL-terminated; returns nonzero
 * if @p needle occurs in @p hay ignoring case.
 */
static int contains_ci(const char *hay, const char *needle)
{
    if (!hay || !needle) {
        return 0;
    }
    for (; *hay; ++hay) {
        const char *h = hay;
        const char *n = needle;
        /* Walk both while characters match case-insensitively. */
        while (*h && *n &&
               tolower((unsigned char)*h) == tolower((unsigned char)*n)) {
            ++h;
            ++n;
        }
        if (*n == '\0') {
            return 1;           /* reached end of needle -> full match          */
        }
    }
    return 0;
}

const sensor_profile_t *sensor_profile_from_name(const char *product_name)
{
    /* Positive identification of the long-range unit wins; everything else —
     * including a NULL/empty name or a legacy SF30/C — maps to the SF30/C
     * default so the box is never left without a profile.                     */
    if (contains_ci(product_name, "SF30/D") ||
        contains_ci(product_name, "SF30D")) {
        return &SF30D_PROFILE;
    }
    return &SF30C_PROFILE;
}
