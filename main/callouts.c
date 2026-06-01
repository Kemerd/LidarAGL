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
DECLARE_CLIP(_binary_calib_error_pcm);

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
static clip_t s_chirp;
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

    s_chirp = make_clip(_binary_calib_error_pcm_start, _binary_calib_error_pcm_end, "calib error");

    s_built = true;
}

const clip_t *callout_clip(callout_id_t id)
{
    if (!s_built) {
        build_manifest();
    }
    if (id < 0 || id >= CO_COUNT) {
        return &s_chirp;   /* harmless absent fallback */
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
        default:  return CO_COUNT;     /* no clip for this height */
    }
}
