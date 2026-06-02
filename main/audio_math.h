/**
 * @file    audio_math.h
 * @brief   Pure perceptual math for the presence tone and envelopes.
 *
 * @details All of the "why it sounds right" math lives here, isolated from the
 *          I2S hardware so it can be unit-tested on the host:
 *
 *            - Pitch ASCENDS as AGL falls (auditory looming bias).
 *            - Volume is scheduled in dB (perceptual), then converted to a
 *              linear gain — we never ramp linear amplitude directly.
 *            - Gain changes and tone start/stop use a raised-cosine envelope so
 *              there are no clicks and no startle from abrupt onsets.
 *
 *          PURE module: standard library + config.h pure region only.
 */
#ifndef LIDARAGL_AUDIO_MATH_H
#define LIDARAGL_AUDIO_MATH_H

/**
 * @brief Map AGL to the presence-tone frequency (Hz), ascending as AGL falls.
 *
 * @details Active from TONE_START_FT down to the ground. At TONE_START_FT the
 *          tone sits at F_AT_TONE_START; near 0 ft it reaches F_AT_GROUND. With
 *          TONE_LOG_SWEEP the glide is exponential (musical / perceptually
 *          even); otherwise it is linear in Hz. The result is clamped to the
 *          500–3000 Hz band where the ear is sensitive and ANR headsets pass.
 *
 * @param agl_ft  Current AGL in feet (clamped internally to [0, TONE_START_FT]).
 * @return        Tone frequency in Hz.
 */
float agl_to_pitch_hz(float agl_ft);

/**
 * @brief Schedule the presence-tone level in dB (relative to full scale).
 *
 * @details Above TONE_START_FT the tone is silent (returns a very low floor).
 *          From TONE_START_FT down to TONE_FULL_FT the level ramps linearly in
 *          dB from TONE_FLOOR_DB up to TONE_FULL_DB. At/below TONE_FULL_FT it
 *          holds TONE_FULL_DB (full presence through the flare band to the
 *          ground).
 *
 * @param agl_ft  Current AGL in feet.
 * @return        Level in dB relative to full scale (<= 0).
 */
float agl_to_tone_db(float agl_ft);

/**
 * @brief Equal-loudness correction (dB) to flatten the ear's frequency tilt.
 *
 * @details The human ear is more sensitive to some frequencies than others
 *          (the ISO 226 equal-loudness contours / Fletcher-Munson curves). Our
 *          presence tone deliberately SWEEPS its pitch up as the ground nears,
 *          which — if uncorrected — makes the tone sound progressively louder
 *          even when its electrical level is held constant, because the rising
 *          pitch climbs into the ear's more-sensitive band. That extra
 *          loudness reads as a stress cue we do NOT want: urgency is carried by
 *          the PITCH, while loudness is meant to stay perceptually constant once
 *          the tone has faded in.
 *
 *          This returns the dB to ADD to the scheduled level so that equal
 *          scheduled dB sounds equally loud across the sweep. It is a RELATIVE
 *          flattening (referenced to 1 kHz), based on a compact interpolation of
 *          the ISO 226 contour at a typical cockpit listening level (~60 phon).
 *          It is not absolute calibration — the trim pot still sets master level.
 *
 *          Where the ear is MORE sensitive (higher freqs) this returns a
 *          NEGATIVE value (attenuate); where it is LESS sensitive (lower freqs)
 *          it returns a POSITIVE value (boost). Gated by
 *          EQUAL_LOUDNESS_CORRECTION in config.h.
 *
 * @param freq_hz  The current tone frequency.
 * @return         dB to add to the scheduled level (0.0 if correction disabled).
 */
float equal_loudness_db(float freq_hz);

/**
 * @brief Convert a dB level (relative to full scale) to a linear gain.
 * @param db  Level in dB (<= 0 for attenuation).
 * @return    Linear gain, 10^(db/20).
 */
float db_to_gain(float db);

/**
 * @brief Raised-cosine envelope shaper for click-free ramps.
 * @param t01  Normalised ramp position in [0, 1].
 * @return     Smoothstep-like value 0.5 - 0.5*cos(pi*t), 0 at t=0, 1 at t=1.
 */
float raised_cosine(float t01);

/**
 * @brief Move @p cur toward @p target by at most @p max_delta (slew limiter).
 * @param cur        Current value.
 * @param target     Desired value.
 * @param max_delta  Maximum change permitted this step (>= 0).
 * @return           The new, slew-limited value.
 */
float slew_limit(float cur, float target, float max_delta);

#endif /* LIDARAGL_AUDIO_MATH_H */
