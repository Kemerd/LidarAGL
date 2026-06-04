/**
 * @file    callouts.h
 * @brief   Voice-callout ids, embedded-clip manifest, and AGL->id mapping.
 *
 * @details The spoken clips are raw 16 kHz mono s16le PCM embedded into flash
 *          via EMBED_FILES (see main/CMakeLists.txt). Because the clip set is
 *          GLOBBED, some clips may be absent at build time; the manifest reports
 *          {NULL,0} for those so the audio engine skips them gracefully.
 *
 *          A separate "calibration error" chirp is emitted at boot when no
 *          ground reference could be established.
 */
#ifndef LIDARAGL_CALLOUTS_H
#define LIDARAGL_CALLOUTS_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Callout clip identifiers.
 *
 * @note Ordered low-to-high altitude. The SF30/C tops out at 300 ft, so it never
 *       requests the 400/500/600 ids (they're not in its profile); the SF30/D,
 *       with its longer range, requests all of them up to 600 ft.
 */
typedef enum {
    CO_TEN = 0,
    CO_TWENTY,
    CO_THIRTY,
    CO_FORTY,
    CO_FIFTY,
    CO_ONE_HUNDRED,
    CO_TWO_HUNDRED,
    CO_THREE_HUNDRED,
    CO_FOUR_HUNDRED,
    CO_FIVE_HUNDRED,
    CO_SIX_HUNDRED,
    /*  Non-numeric spoken callouts (no altitude maps to them via
     *  callout_id_for_ft). CO_CHECK_GEAR is queued right AFTER the altitude
     *  number at the gear-check altitude to say "... check gear"; CO_POSITIVE_RATE
     *  is the standalone takeoff climb call.                                      */
    CO_CHECK_GEAR,
    CO_POSITIVE_RATE,
    CO_COUNT
} callout_id_t;

/** A resolved embedded clip. pcm == NULL / len_bytes == 0 means "not present". */
typedef struct {
    const uint8_t *pcm;        /**< Pointer to embedded raw s16le PCM, or NULL.  */
    size_t         len_bytes;  /**< Length in bytes (0 if absent).               */
    const char    *name;       /**< Human/log label, e.g. "fifty".               */
} clip_t;

/**
 * @brief Composable config-menu prompt pieces.
 *
 * @details To save flash we don't store one clip per full phrase. Instead the
 *          menu plays a CHANNEL piece ("Mono" / "Stereo") followed by a STREAM
 *          piece ("Callouts and Tone" / "Callouts Only" / "Tone Only"), with a
 *          short gap between. The number clips (CO_*) voice the start-altitude.
 */
typedef enum {
    CFG_PIECE_MONO = 0,          /**< "Mono"                                     */
    CFG_PIECE_STEREO,            /**< "Stereo"                                   */
    CFG_PIECE_CALLOUTS_AND_TONE, /**< "Callouts and Tone"                        */
    CFG_PIECE_CALLOUTS_ONLY,     /**< "Callouts Only"                            */
    CFG_PIECE_TONE_ONLY,         /**< "Tone Only"                                */
    CFG_PIECE_START_ALT,         /**< "Callout Start Altitude"                   */
    CFG_PIECE_VOLUME_ADJ,        /**< "Volume Adjustment"                        */
    CFG_PIECE_OFF,               /**< "Off" — disabled choice in the new menus   */
    CFG_PIECE_COUNT
} config_piece_t;

/**
 * @brief Resolve a callout id to its embedded clip.
 * @param id  Callout id.
 * @return    Pointer to a (possibly absent) clip descriptor; never NULL itself.
 */
const clip_t *callout_clip(callout_id_t id);

/**
 * @brief The calibration-error chirp clip (also embedded; may be absent).
 * @return Pointer to the chirp descriptor; never NULL itself.
 */
const clip_t *callout_chirp(void);

/**
 * @brief The spoken calibration-error instruction ("please reset unit on the
 *        ground"), played right after the chirp on a calibration failure.
 * @return Pointer to the descriptor; never NULL itself (may be an absent clip).
 */
const clip_t *callout_calib_voice(void);

/**
 * @brief The "config mode, memory cleared" prompt played when config mode opens.
 * @return Pointer to the descriptor; never NULL itself (may be an absent clip).
 */
const clip_t *config_clip_enter(void);

/**
 * @brief The short chirp played at config entry, on each confirm, and on commit.
 * @return Pointer to the descriptor; never NULL itself (may be an absent clip).
 */
const clip_t *config_clip_chirp(void);

/**
 * @brief One composable config-menu prompt piece (see config_piece_t).
 * @param piece  Which piece to fetch.
 * @return       Pointer to the descriptor; never NULL itself (may be absent).
 */
const clip_t *config_clip_piece(config_piece_t piece);

/**
 * @brief Map a callout HEIGHT in feet to its clip id.
 * @param ft  Callout height (10,20,30,40,50,100,200,300,400,500,600).
 * @return    The matching id, or CO_COUNT if no clip corresponds.
 */
callout_id_t callout_id_for_ft(float ft);

#endif /* LIDARAGL_CALLOUTS_H */
