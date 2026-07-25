/**
 * @file    callouts.c
 * @brief   Embedded-clip manifest with graceful handling of missing clips.
 *
 * @details Each clip is embedded by EMBED_FILES, which emits the symbols
 *          _binary_<name>_pcm_start / _end for files that EXIST. Because we glob
 *          the clip directory, a clip may be absent — referencing a missing
 *          symbol would fail the link. To stay robust we declare every symbol
 *          WEAK: an absent clip's start/end resolve to NULL, and the manifest
 *          reports it as not-present so the audio engine simply skips it.
 *
 *          (The host test build does not compile this file — the EMBED symbols
 *          only exist in the firmware link.)
 */

#include "callouts.h"
#include <math.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 *  Weak declarations of the embedded-clip boundary symbols.
 *
 *  ESP-IDF names them from the file path with non-alphanumerics turned into
 *  underscores, e.g. assets/clips/fifty.pcm -> _binary_fifty_pcm_start/_end.
 *  The __attribute__((weak)) means: if the file wasn't embedded, the symbol is
 *  NULL rather than an unresolved-reference link error.
 * ------------------------------------------------------------------------- */
#define DECLARE_CLIP(sym)                                                      \
    extern const uint8_t sym##_start[] __attribute__((weak));                  \
    extern const uint8_t sym##_end[]   __attribute__((weak))

DECLARE_CLIP(_binary_ten_pcm);
DECLARE_CLIP(_binary_twenty_pcm);
DECLARE_CLIP(_binary_thirty_pcm);
DECLARE_CLIP(_binary_forty_pcm);
DECLARE_CLIP(_binary_fifty_pcm);
DECLARE_CLIP(_binary_one_hundred_pcm);
DECLARE_CLIP(_binary_two_hundred_pcm);
DECLARE_CLIP(_binary_three_hundred_pcm);
DECLARE_CLIP(_binary_four_hundred_pcm);
DECLARE_CLIP(_binary_five_hundred_pcm);
DECLARE_CLIP(_binary_six_hundred_pcm);
/*  Non-numeric spoken callouts. */
DECLARE_CLIP(_binary_check_gear_pcm);
DECLARE_CLIP(_binary_positive_rate_pcm);
/*  Vario config-menu page titles (sink_rate.pcm / climb_rate.pcm). */
DECLARE_CLIP(_binary_sink_rate_pcm);
DECLARE_CLIP(_binary_climb_rate_pcm);
DECLARE_CLIP(_binary_calibration_error_pcm);
/*  Spoken instruction played right after the calibration-error chirp: the chirp
 *  grabs attention, this voice line tells the pilot what to do.                 */
DECLARE_CLIP(_binary_please_reset_unit_on_the_ground_pcm);

/*  Boot config-menu prompts (see config_clip_* accessors). We compose phrases
 *  from small pieces to save flash. Filenames in assets/clips/ map to these
 *  symbols: chirp.pcm, config_mode.pcm, mono.pcm, stereo.pcm,
 *  callouts_and_tone.pcm, callouts_only.pcm, tone_only.pcm,
 *  callout_start_altitude.pcm, volume_adjustment.pcm, off.pcm. Start-altitude
 *  and gear-check numbers reuse the CO_* clips; the gear-check and positive-rate
 *  page titles reuse check_gear.pcm / positive_rate.pcm via callout_clip().     */
DECLARE_CLIP(_binary_chirp_pcm);
DECLARE_CLIP(_binary_config_mode_pcm);
DECLARE_CLIP(_binary_mono_pcm);
DECLARE_CLIP(_binary_stereo_pcm);
DECLARE_CLIP(_binary_callouts_and_tone_pcm);
DECLARE_CLIP(_binary_callouts_only_pcm);
DECLARE_CLIP(_binary_tone_only_pcm);
DECLARE_CLIP(_binary_callout_start_altitude_pcm);
DECLARE_CLIP(_binary_volume_adjustment_pcm);
DECLARE_CLIP(_binary_off_pcm);
DECLARE_CLIP(_binary_on_pcm);

/* Compute the byte length of an embedded clip, or 0 if it is absent. */
#define CLIP_LEN(sym) ((sym##_start && sym##_end) \
                       ? (size_t)(sym##_end - sym##_start) : (size_t)0)

/* Build a clip_t for an embedded symbol pair + a label. Resolved at runtime so
 * weak-NULL symbols yield an absent clip.                                       */
static clip_t make_clip(const uint8_t *start, const uint8_t *end, const char *name)
{
    clip_t c;
    if (start && end && end > start) {
        c.pcm       = start;
        c.len_bytes = (size_t)(end - start);
    } else {
        c.pcm       = NULL;
        c.len_bytes = 0;
    }
    c.name = name;
    return c;
}

/* ---------------------------------------------------------------------------
 *  Manifest: one row per callout id, in enum order. Rebuilt lazily into a
 *  static cache on first access (the symbols are link-time constants, so this
 *  is computed once and reused).
 * ------------------------------------------------------------------------- */
static clip_t s_clips[CO_COUNT];
static clip_t s_chirp;                        /* calibration-error chirp          */
static clip_t s_calib_voice;                  /* "please reset unit on the ground"*/
static clip_t s_cfg_enter;                    /* "config mode, memory cleared"    */
static clip_t s_cfg_chirp;                    /* short menu chirp (entry/confirm) */
static clip_t s_cfg_pieces[CFG_PIECE_COUNT];  /* composable spoken pieces         */
static bool   s_built = false;

static void build_manifest(void)
{
    s_clips[CO_TEN]           = make_clip(_binary_ten_pcm_start,         _binary_ten_pcm_end,         "ten");
    s_clips[CO_TWENTY]        = make_clip(_binary_twenty_pcm_start,      _binary_twenty_pcm_end,      "twenty");
    s_clips[CO_THIRTY]        = make_clip(_binary_thirty_pcm_start,      _binary_thirty_pcm_end,      "thirty");
    s_clips[CO_FORTY]         = make_clip(_binary_forty_pcm_start,       _binary_forty_pcm_end,       "forty");
    s_clips[CO_FIFTY]         = make_clip(_binary_fifty_pcm_start,       _binary_fifty_pcm_end,       "fifty");
    s_clips[CO_ONE_HUNDRED]   = make_clip(_binary_one_hundred_pcm_start, _binary_one_hundred_pcm_end, "one hundred");
    s_clips[CO_TWO_HUNDRED]   = make_clip(_binary_two_hundred_pcm_start, _binary_two_hundred_pcm_end, "two hundred");
    s_clips[CO_THREE_HUNDRED] = make_clip(_binary_three_hundred_pcm_start, _binary_three_hundred_pcm_end, "three hundred");
    s_clips[CO_FOUR_HUNDRED]  = make_clip(_binary_four_hundred_pcm_start,  _binary_four_hundred_pcm_end,  "four hundred");
    s_clips[CO_FIVE_HUNDRED]  = make_clip(_binary_five_hundred_pcm_start,  _binary_five_hundred_pcm_end,  "five hundred");
    s_clips[CO_SIX_HUNDRED]   = make_clip(_binary_six_hundred_pcm_start,   _binary_six_hundred_pcm_end,   "six hundred");
    s_clips[CO_CHECK_GEAR]    = make_clip(_binary_check_gear_pcm_start,    _binary_check_gear_pcm_end,    "check gear");
    s_clips[CO_POSITIVE_RATE] = make_clip(_binary_positive_rate_pcm_start, _binary_positive_rate_pcm_end, "positive rate");
    s_clips[CO_SINK_RATE]     = make_clip(_binary_sink_rate_pcm_start,    _binary_sink_rate_pcm_end,     "sink rate");
    s_clips[CO_CLIMB_RATE]    = make_clip(_binary_climb_rate_pcm_start,   _binary_climb_rate_pcm_end,    "climb rate");

    s_chirp       = make_clip(_binary_calibration_error_pcm_start, _binary_calibration_error_pcm_end, "calib error");
    s_calib_voice = make_clip(_binary_please_reset_unit_on_the_ground_pcm_start,
                              _binary_please_reset_unit_on_the_ground_pcm_end, "please reset on ground");

    /* Boot config-menu prompts + composable pieces. */
    s_cfg_enter = make_clip(_binary_config_mode_pcm_start, _binary_config_mode_pcm_end, "config mode");
    s_cfg_chirp = make_clip(_binary_chirp_pcm_start,       _binary_chirp_pcm_end,       "config chirp");

    s_cfg_pieces[CFG_PIECE_MONO]              = make_clip(_binary_mono_pcm_start,                   _binary_mono_pcm_end,                   "mono");
    s_cfg_pieces[CFG_PIECE_STEREO]            = make_clip(_binary_stereo_pcm_start,                 _binary_stereo_pcm_end,                 "stereo");
    s_cfg_pieces[CFG_PIECE_CALLOUTS_AND_TONE] = make_clip(_binary_callouts_and_tone_pcm_start,      _binary_callouts_and_tone_pcm_end,      "callouts and tone");
    s_cfg_pieces[CFG_PIECE_CALLOUTS_ONLY]     = make_clip(_binary_callouts_only_pcm_start,          _binary_callouts_only_pcm_end,          "callouts only");
    s_cfg_pieces[CFG_PIECE_TONE_ONLY]         = make_clip(_binary_tone_only_pcm_start,              _binary_tone_only_pcm_end,              "tone only");
    s_cfg_pieces[CFG_PIECE_START_ALT]         = make_clip(_binary_callout_start_altitude_pcm_start, _binary_callout_start_altitude_pcm_end, "callout start altitude");
    s_cfg_pieces[CFG_PIECE_VOLUME_ADJ]        = make_clip(_binary_volume_adjustment_pcm_start,      _binary_volume_adjustment_pcm_end,      "volume adjustment");
    s_cfg_pieces[CFG_PIECE_OFF]               = make_clip(_binary_off_pcm_start,                    _binary_off_pcm_end,                    "off");
    s_cfg_pieces[CFG_PIECE_ON]                = make_clip(_binary_on_pcm_start,                     _binary_on_pcm_end,                     "on");

    s_built = true;
}

const clip_t *callout_clip(callout_id_t id)
{
    if (!s_built) {
        build_manifest();
    }
    if (id < 0 || id >= CO_COUNT) {
        /* Out-of-range id: return an EMPTY clip, never the chirp. The chirp is
         * the calibration/sensor-failure ALARM — sounding it for what is purely
         * an internal lookup slip would tell the pilot the LiDAR had failed. A
         * silent skip is the honest response to a bug that has no audio.        */
        static const clip_t s_none = { NULL, 0, "invalid-callout-id" };
        return &s_none;
    }
    return &s_clips[id];
}

const clip_t *callout_chirp(void)
{
    if (!s_built) {
        build_manifest();
    }
    return &s_chirp;
}

const clip_t *callout_calib_voice(void)
{
    if (!s_built) {
        build_manifest();
    }
    return &s_calib_voice;
}

const clip_t *config_clip_enter(void)
{
    if (!s_built) {
        build_manifest();
    }
    return &s_cfg_enter;
}

const clip_t *config_clip_chirp(void)
{
    if (!s_built) {
        build_manifest();
    }
    return &s_cfg_chirp;
}

const clip_t *config_clip_piece(config_piece_t piece)
{
    if (!s_built) {
        build_manifest();
    }
    if (piece < 0 || piece >= CFG_PIECE_COUNT) {
        return &s_cfg_chirp;   /* harmless absent-safe fallback */
    }
    return &s_cfg_pieces[piece];
}

callout_id_t callout_id_for_ft(float ft)
{
    /* Round to the nearest integer foot to be robust against tiny float noise
     * in the callout table values.                                            */
    int h = (int)(ft + 0.5f);
    switch (h) {
        case 10:  return CO_TEN;
        case 20:  return CO_TWENTY;
        case 30:  return CO_THIRTY;
        case 40:  return CO_FORTY;
        case 50:  return CO_FIFTY;
        case 100: return CO_ONE_HUNDRED;
        case 200: return CO_TWO_HUNDRED;
        case 300: return CO_THREE_HUNDRED;
        case 400: return CO_FOUR_HUNDRED;
        case 500: return CO_FIVE_HUNDRED;
        case 600: return CO_SIX_HUNDRED;
        default:  return CO_COUNT;     /* no clip for this height */
    }
}
