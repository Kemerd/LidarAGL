/**
 * @file    range_filter.c
 * @brief   Per-sample Kalman range tracker with explicit track states (see
 *          range_filter.h for the architecture and its history).
 *
 * @details PURE: C standard library + config.h's pure region only. Everything
 *          here is deterministic in its inputs, allocation-free, bounded in
 *          time (O(samples) per finalize, ~100 flops each), and exercised by
 *          test/test_range_filter.c and the whole-flight sorties in
 *          test/test_flight.c.
 *
 *          Processing order for ONE raw sample (rf_finalize walks the drain):
 *
 *            advance clocks ──> candidate gap timeout ──> COAST expiry (LOST)
 *                │
 *                ├─ no-return? ──> lost vote, candidate void count, done
 *                │
 *                ├─ main track alive and the sample passes its innovation
 *                │  gate? ──> Kalman update, TRACK, done
 *                │
 *                └─ otherwise ──> offered to the candidate (seed / join /
 *                                 miss) ──> confirmation rules ──> adopt
 *
 *          Every decision is a function of per-sample time, never of how many
 *          polls happened, so the poll cadence changes latency and nothing else.
 */

#include "range_filter.h"

#include <math.h>
#include <string.h>

/* ===========================================================================
 *  SF30 legacy 2-byte pair decoder.
 * ===========================================================================*/

void sf30_ascii_reset(sf30_ascii_ctx_t *c)
{
    if (c == NULL) {
        return;
    }
    c->have_high = false;
    c->high      = 0;
}

bool sf30_ascii_feed(sf30_ascii_ctx_t *c, uint8_t b, int *out_cm)
{
    if (c == NULL || out_cm == NULL) {
        return false;
    }
    /* Bit 7 set -> this is a HIGH byte: latch it (a repeated high byte simply
     * re-latches, which is the stream's own resync mechanism).                 */
    if (b & 0x80) {
        c->high      = (uint8_t)(b & 0x7F);
        c->have_high = true;
        return false;
    }
    /* A LOW byte with no preceding high byte is an orphan (we joined the
     * stream mid-pair, or its high byte was lost) — drop it and stay synced.   */
    if (!c->have_high) {
        return false;
    }
    c->have_high = false;
    *out_cm = ((int)c->high << 7) | (int)(b & 0x7F);
    return true;
}

/* ===========================================================================
 *  Derived constants.
 * ===========================================================================*/

/*  Discrete white-noise-acceleration process noise level (ft^2/s^4). */
#define RF_Q            (RF_SIGMA_ACCEL_FPS2 * RF_SIGMA_ACCEL_FPS2)

/*  Bounds of the adaptive measurement-noise variance (ft^2). */
#define RF_R_MIN        (RF_SIGMA_MEAS_FT * RF_SIGMA_MEAS_FT)
#define RF_R_MAX        (RF_SIGMA_MEAS_MAX_FT * RF_SIGMA_MEAS_MAX_FT)

/*  Squared innovation gate (in units of the innovation variance S). */
#define RF_GATE_SQ      (RF_GATE_SIGMA * RF_GATE_SIGMA)

/*  Huber clip on the innovation fed to the noise follower: 3 sigma. One
 *  sample at the edge of the 5-sigma gate would otherwise move the estimate
 *  by alpha * 25 S and let a handful of gate-edge junk samples inflate the
 *  gate that admitted them.                                                    */
#define RF_R_HUBER_SQ   9.0f

/*  A fresh candidate's velocity prior: 1 sigma = RANGE_MAX_SLEW_FPS divided by
 *  the gate, so its very first gate admits exactly the motion the airframe can
 *  make in the elapsed time — no more — whatever the aircraft is doing.        */
#define RF_CAND_SIGMA_V (RANGE_MAX_SLEW_FPS / RF_GATE_SIGMA)

/*  Covariance sanity bounds. The floor keeps S strictly positive and the gate
 *  meaningful; the ceilings stop a pathological input from growing P until
 *  the float arithmetic stops being arithmetic.                                */
#define RF_P_FLOOR      1.0e-6f
#define RF_P00_MAX      1.0e6f
#define RF_P11_MAX      (RANGE_MAX_SLEW_FPS * RANGE_MAX_SLEW_FPS)

/*  Clocks saturate here: ~11 days, far past any threshold they are compared
 *  with, and far below the point where adding 13 ms stops changing a float.    */
#define RF_CLOCK_MAX_S  1.0e6f

/* ===========================================================================
 *  Small helpers.
 * ===========================================================================*/

/*  cm -> ft. The SINGLE place this conversion happens (kept here, not in the
 *  driver, so the whole numeric path is host-testable).                        */
static inline float cm_to_ft(float cm)
{
    return cm * CM_TO_FT;
}

/*  Saturating clock advance: a bounded, never-NaN elapsed time. */
static inline float clock_add(float t, float h)
{
    float s = t + h;
    if (!(s >= 0.0f)) {
        return 0.0f;                     /* NaN or negative: restart honestly   */
    }
    return (s > RF_CLOCK_MAX_S) ? RF_CLOCK_MAX_S : s;
}

/*  Clamp a rate into the airframe's physical bound. */
static inline float clamp_rate(float v)
{
    if (!isfinite(v)) {
        return 0.0f;
    }
    if (v >  RANGE_MAX_SLEW_FPS) return  RANGE_MAX_SLEW_FPS;
    if (v < -RANGE_MAX_SLEW_FPS) return -RANGE_MAX_SLEW_FPS;
    return v;
}

/*  Record one raw sample's verdict in the sliding lost-signal history (bit 0 =
 *  newest): 1 = no return, 0 = a return that passed the validity gates.       */
static inline void lost_hist_push(range_filter_t *f, bool lost)
{
    f->lost_bits = (f->lost_bits << 1) | (lost ? 1u : 0u);
    if (f->lost_bits_n < RANGE_LOST_VOTE_SAMPLES) {
        ++f->lost_bits_n;
    }
}

/*  Saturating counter increment. */
static inline void count_up(uint32_t *c)
{
    if (*c < UINT32_MAX) {
        ++*c;
    }
}

/* ===========================================================================
 *  The two-state Kalman filter (shared by the main track and the candidate).
 * ---------------------------------------------------------------------------
 *  Model (Bar-Shalom, discrete white-noise acceleration):
 *
 *      state   s = [x, v]'            x: range (ft), v: range rate (ft/s)
 *      F       = [[1, h], [0, 1]]     h: time since the state's epoch
 *      Q       = q * [[h^4/4, h^3/2],
 *                     [h^3/2, h^2  ]] q: sigma_a^2
 *      H       = [1, 0],  R = r_var   (adaptive, bounded)
 *
 *  The 2x2 algebra is written out by hand: it is exact, allocation-free, and
 *  small enough to verify line by line.
 * ===========================================================================*/

/*  Seed an estimate on a single measurement: position known to the noise of
 *  one sample, velocity unknown to sigma_v, no correlation.                    */
static void kf_seed(rf_kf_t *k, float z, float r_var, float sigma_v)
{
    if (!(r_var >= RF_R_MIN)) r_var = RF_R_MIN;    /* also catches NaN        */
    if (r_var > RF_R_MAX)     r_var = RF_R_MAX;

    k->x     = z;
    k->v     = 0.0f;
    k->p00   = r_var;
    k->p01   = 0.0f;
    k->p11   = sigma_v * sigma_v;
    k->r_var = r_var;
}

/*  Return a copy of @p k predicted @p h seconds ahead. @p k is NOT modified:
 *  the tracker only commits a prediction together with the measurement that
 *  justified it, so time without an accepted sample never shrinks P.          */
static rf_kf_t kf_predicted(const rf_kf_t *k, float h)
{
    rf_kf_t o = *k;
    if (!(h > 0.0f)) {
        return o;                                   /* no time passed / bogus h */
    }
    float h2 = h * h;
    float h3 = h2 * h;
    float h4 = h2 * h2;

    o.x   = k->x + k->v * h;
    o.p00 = k->p00 + 2.0f * h * k->p01 + h2 * k->p11 + RF_Q * h4 * 0.25f;
    o.p01 = k->p01 + h * k->p11 + RF_Q * h3 * 0.5f;
    o.p11 = k->p11 + RF_Q * h2;
    return o;
}

/*  The innovation gate: does measurement @p z belong to the predicted estimate
 *  @p kp? Accept when nu^2 <= g^2 * S, S = P00 + R. Outputs nu and S for the
 *  update. Any non-finite quantity fails the gate (never "accepts a NaN").     */
static bool kf_gate(const rf_kf_t *kp, float z, float *nu_out, float *s_out)
{
    float s  = kp->p00 + kp->r_var;
    float nu = z - kp->x;
    *nu_out = nu;
    *s_out  = s;
    if (!isfinite(nu) || !isfinite(s) || !(s > 0.0f)) {
        return false;
    }
    return (nu * nu) <= RF_GATE_SQ * s;
}

/*  Covariance hygiene: finite, symmetric positive-definite, bounded. Returns
 *  false if the estimate is beyond repair (the caller re-seeds).               */
static bool kf_sanitize(rf_kf_t *k)
{
    if (!isfinite(k->x) || !isfinite(k->v) || !isfinite(k->p00) ||
        !isfinite(k->p01) || !isfinite(k->p11) || !isfinite(k->r_var)) {
        return false;
    }
    /* Physical velocity bound (the model has no other notion of one). */
    k->v = clamp_rate(k->v);

    /* Diagonal: strictly positive, bounded. */
    if (k->p00 < RF_P_FLOOR) k->p00 = RF_P_FLOOR;
    if (k->p00 > RF_P00_MAX) k->p00 = RF_P00_MAX;
    if (k->p11 < RF_P_FLOOR) k->p11 = RF_P_FLOOR;
    if (k->p11 > RF_P11_MAX) k->p11 = RF_P11_MAX;

    /* Off-diagonal: |p01| < sqrt(p00 * p11) keeps the determinant positive.
     * Rounding in the update can push it a hair past the bound; pull it in.  */
    float lim = 0.999f * sqrtf(k->p00 * k->p11);
    if (k->p01 >  lim) k->p01 =  lim;
    if (k->p01 < -lim) k->p01 = -lim;

    /* Measurement noise stays inside its configured band. */
    if (k->r_var < RF_R_MIN) k->r_var = RF_R_MIN;
    if (k->r_var > RF_R_MAX) k->r_var = RF_R_MAX;
    return true;
}

/*  Measurement update of a PREDICTED estimate with innovation @p nu (already
 *  gated) and innovation variance @p s, followed by the measurement-noise
 *  follower. Returns false if the result is not a usable estimate.
 *
 *    K   = [P00, P01]' / S
 *    s  += K * nu
 *    P   = (I - K H) P     ->  P00' = P00 R/S,  P01' = P01 R/S,
 *                              P11' = P11 - P01^2/S
 *
 *  Noise follower: E[nu^2] = P00_pred + R, so nu^2 - P00_pred is an unbiased
 *  one-sample estimate of R. It is Huber-clipped at 3 sigma and averaged with
 *  gain RF_R_ADAPT_ALPHA, then bounded to [sigma_m, sigma_max]. This lets the
 *  gate open for genuinely textured surfaces (grass, high speed over terrain)
 *  without ever becoming wide enough to admit junk.                           */
static bool kf_update(rf_kf_t *kp, float nu, float s)
{
    float p00 = kp->p00;
    float p01 = kp->p01;
    float k0  = p00 / s;
    float k1  = p01 / s;
    float ros = kp->r_var / s;                     /* R / S, in (0, 1]        */

    kp->x  += k0 * nu;
    kp->v  += k1 * nu;
    kp->p00 = p00 * ros;
    kp->p01 = p01 * ros;
    kp->p11 = kp->p11 - (p01 * p01) / s;

    float nu2 = nu * nu;
    if (nu2 > RF_R_HUBER_SQ * s) {
        nu2 = RF_R_HUBER_SQ * s;
    }
    kp->r_var += RF_R_ADAPT_ALPHA * ((nu2 - p00) - kp->r_var);

    return kf_sanitize(kp);
}

/* ===========================================================================
 *  Candidate track.
 * ===========================================================================*/

/*  Forget the candidate. */
static void cand_drop(range_filter_t *f)
{
    f->c_n      = 0;
    f->c_miss   = 0;
    f->c_void   = 0;
    f->c_age_s  = 0.0f;
    f->c_span_s = 0.0f;
}

/*  Start a new candidate on sample @p z. Its noise prior is the main track's
 *  latest surface estimate — the best knowledge of what this sensor's returns
 *  look like right now.                                                        */
static void cand_seed(range_filter_t *f, float z)
{
    kf_seed(&f->cand, z, f->trk.r_var, RF_CAND_SIGMA_V);
    f->c_n      = 1;
    f->c_miss   = 0;
    f->c_void   = 0;
    f->c_age_s  = 0.0f;
    f->c_span_s = 0.0f;
}

/* ===========================================================================
 *  Track state transitions.
 * ===========================================================================*/

/*  The main track has gone RF_COAST_MAX_S without an accepted sample: it is
 *  LOST. The published value freezes where the coast left it (the track's own
 *  prediction at the coast limit), and carries no motion from here on.        */
static void rf_go_lost(range_filter_t *f)
{
    /*  Why was it lost? Blindness (the coast window was mostly no-returns: the
     *  surface went dark, or we flew out of range) or rejection (returns kept
     *  arriving but none fit: a terrain step, tree tops, a stuck pattern).
     *  Only blindness can hide rungs the aircraft genuinely passed.         */
    f->lost_blind = (f->unacc_void >= f->unacc_real);

    float coast = f->since_accept_s;
    if (coast > RF_COAST_MAX_S) {
        coast = RF_COAST_MAX_S;
    }
    float held = f->trk.x + f->trk.v * coast;
    if (isfinite(held)) {
        f->out_ft = held;
    }
    f->out_rate_fps = 0.0f;
    f->state        = RF_LOST;
}

/*  Per-drain scratch, reported back to rf_finalize(). */
typedef struct {
    uint32_t n_real;     /**< Samples that passed the validity gates.          */
    uint32_t n_void;     /**< No-return samples (sentinel, lens, beyond, NaN). */
    bool     accepted;   /**< The main track accepted (or adopted) a sample.   */
} rf_drain_stats_t;

/*  Adopt the confirmed candidate as the main track.
 *
 *  @p brk      the switch is discontinuous (report a track break);
 *  @p reentry  ...and it is a flyable descending re-entry (see
 *              rf_break_reentry()).                                            */
static void rf_adopt(range_filter_t *f, bool brk, bool reentry,
                     rf_drain_stats_t *st)
{
    f->trk            = f->cand;               /* incl. its noise estimate     */
    f->trk.v          = clamp_rate(f->trk.v);
    f->since_accept_s = f->c_age_s;            /* 0: confirmed on a member     */
    f->unacc_void     = 0;
    f->unacc_real     = 0;
    f->state          = RF_TRACK;
    f->have_out       = true;

    /* A confirmed track is a measurement again: the out-of-range-above
     * inference no longer applies.                                            */
    f->above_ceiling = false;
    f->ceiling_polls = 0;

    if (brk) {
        /*  Two breaks inside one drain are pathological, but if it happens the
         *  re-entry verdict must hold for BOTH or the late-rung window stays
         *  shut: speaking a rung is the one irreversible thing downstream.  */
        f->break_reentry = f->track_break ? (f->break_reentry && reentry)
                                          : reentry;
        f->track_break   = true;
    }

    cand_drop(f);
    st->accepted = true;
}

/*  Has the candidate earned the track? Applies the M-of-N evidence and the
 *  switch rules (config.h, "Switching onto a confirmed candidate"). Adopts it
 *  if so.                                                                      */
static void rf_try_confirm(range_filter_t *f, rf_drain_stats_t *st)
{
    /* --- Evidence: M members over a minimum span ---------------------------- */
    if (f->c_n < RF_CONFIRM_SAMPLES || f->c_span_s < RF_CONFIRM_MIN_S) {
        return;
    }

    /* --- ...that make up enough of EVERY sample since it began --------------
     *  Junk scattered across the band lands on one straight line a few percent
     *  of the time; a surface the laser genuinely sees supplies most of the
     *  stream. Misses (incl. main-track accepts) and no-returns all count.   */
    float total = (float)f->c_n + (float)f->c_miss + (float)f->c_void;
    if ((float)f->c_n < RF_CONFIRM_MEMBER_FRAC * total) {
        return;
    }

    bool brk     = false;
    bool reentry = false;

    switch (f->state) {
        case RF_SEARCH:
            /* First lock: there is nothing to break from. */
            break;

        case RF_LOST: {
            bool descending = f->cand.v < -RANGE_REENTRY_SINK_FPS;
            if (descending) {
                /*  The approach coming back into range: a NEW track (always a
                 *  break), taken on the ordinary evidence.                   */
                brk = true;

                /*  Re-entry (the late-rung window may speak a passed rung)
                 *  only if the track was lost to BLINDNESS and the aircraft
                 *  could physically have flown from its last MEASURED
                 *  position to the new level in the time that passed. A long
                 *  blind stretch makes almost anything reachable — that is the
                 *  genuine approach — while a 110 ft "descent" 0.3 s after
                 *  tracking at 186 ft is not, and stays silent.             */
                float drop  = f->trk.x - f->cand.x;         /* >0: closer now  */
                float reach = RANGE_MAX_SLEW_FPS * f->since_accept_s +
                              RANGE_REACQUIRE_JUMP_SLACK_FT;
                reentry = f->lost_blind && (drop <= reach); /* NaN -> false    */
            } else {
                /*  Stationary (or climbing). Near the HELD value it is the
                 *  same surface coming back — resume continuously. Anything
                 *  else is the shape of a stuck byte pattern: it must persist
                 *  RF_BREAK_S, and then re-anchors silently.                 */
                float jump  = fabsf(f->out_ft - f->cand.x);
                float reach = RANGE_MAX_SLEW_FPS * f->c_span_s +
                              RANGE_REACQUIRE_JUMP_SLACK_FT;
                if (reach > RANGE_REACQUIRE_JUMP_CAP_FT) {
                    reach = RANGE_REACQUIRE_JUMP_CAP_FT;
                }
                if (!(jump <= reach)) {                      /* NaN -> break   */
                    if (f->c_span_s < RF_BREAK_S) {
                        return;
                    }
                    brk = true;
                }
            }
            break;
        }

        case RF_TRACK:
        case RF_COAST:
        default: {
            /*  Competing with a live track. Reachable from the main track's own
             *  prediction -> a continuous hand-over (a manoeuvre the model
             *  lagged, a small terrain step). Otherwise the level must persist
             *  RF_BREAK_S and is reported as a break, never as flown motion.  */
            float pred  = f->trk.x + f->trk.v * f->since_accept_s;
            float jump  = fabsf(pred - f->cand.x);
            float reach = RANGE_MAX_SLEW_FPS * f->c_span_s +
                          RANGE_REACQUIRE_JUMP_SLACK_FT;
            if (reach > RANGE_REACQUIRE_JUMP_CAP_FT) {
                reach = RANGE_REACQUIRE_JUMP_CAP_FT;
            }
            if (!(jump <= reach)) {                          /* NaN -> break   */
                if (f->c_span_s < RF_BREAK_S) {
                    return;
                }
                brk = true;
            }
            break;
        }
    }

    rf_adopt(f, brk, reentry, st);
}

/* ===========================================================================
 *  Per-sample processing.
 * ===========================================================================*/

/*  Advance every clock by @p h seconds and apply the time-driven transitions:
 *  a candidate with no member for RF_CANDIDATE_GAP_S is dropped, and a live
 *  track with no accepted sample for RF_COAST_MAX_S is LOST.                   */
static void rf_advance(range_filter_t *f, float h)
{
    f->since_accept_s = clock_add(f->since_accept_s, h);

    if (f->c_n > 0) {
        f->c_age_s  = clock_add(f->c_age_s,  h);
        f->c_span_s = clock_add(f->c_span_s, h);
        if (f->c_age_s > RF_CANDIDATE_GAP_S) {
            cand_drop(f);                /* members must be consecutive        */
        }
    }

    if ((f->state == RF_TRACK || f->state == RF_COAST) &&
        f->since_accept_s > RF_COAST_MAX_S) {
        rf_go_lost(f);
    }
}

/*  Process ONE drain slot, @p h seconds after the previous one. @p z is the
 *  range in feet, or NaN for a no-return slot.                                 */
static void rf_process_sample(range_filter_t *f, float h, float z,
                              rf_drain_stats_t *st)
{
    rf_advance(f, h);

    /* --- No return: evidence of "nothing seen", never a range -------------- */
    bool no_return = !isfinite(z);
    lost_hist_push(f, no_return);
    if (no_return) {
        /* A no-return ends any repeat run: the next value is a new reading. */
        f->last_z   = NAN;
        f->repeat_s = 0.0f;
        count_up(&st->n_void);
        count_up(&f->unacc_void);
        if (f->c_n > 0) {
            count_up(&f->c_void);
        }
        return;
    }
    count_up(&st->n_real);

    /* --- Sample-and-hold repeats (see RF_REPEAT_HOLD_S) ----------------------
     *  An EXACT repeat of the previous raw sample, within the hold window of
     *  the value first appearing, is the sensor re-sending its last reading:
     *  no new information, so it only lets time pass. (It has already counted
     *  as a return for the lost vote and the tracking verdict above.)        */
    if (z == f->last_z) {
        f->repeat_s = clock_add(f->repeat_s, h);
        if (f->repeat_s < RF_REPEAT_HOLD_S) {
            return;
        }
    } else {
        f->last_z   = z;
        f->repeat_s = 0.0f;
    }

    /* --- Main track: the innovation gate ------------------------------------ */
    if (f->state == RF_TRACK || f->state == RF_COAST) {
        rf_kf_t kp = kf_predicted(&f->trk, f->since_accept_s);
        float   nu, s;
        if (kf_gate(&kp, z, &nu, &s)) {
            if (kf_update(&kp, nu, s)) {
                f->trk = kp;
            } else {
                /* Numerically beyond repair (never seen; defensive): restart
                 * the estimate on this sample rather than carry a NaN on.    */
                kf_seed(&f->trk, z, f->trk.r_var, RF_CAND_SIGMA_V);
            }
            f->since_accept_s = 0.0f;
            f->unacc_void     = 0;
            f->unacc_real     = 0;
            f->state          = RF_TRACK;
            st->accepted      = true;

            /*  A return that fits the main track is evidence AGAINST any
             *  competing candidate. Once the competition outnumbers its
             *  members the candidate is a minority and is dropped.          */
            if (f->c_n > 0) {
                count_up(&f->c_miss);
                if (f->c_miss > f->c_n) {
                    cand_drop(f);
                }
            }
            return;
        }
    }

    /* --- Rejected by (or no) main track: offered to the candidate ----------- */
    count_up(&f->unacc_real);
    if (f->c_n == 0) {
        cand_seed(f, z);
        return;
    }

    rf_kf_t cp = kf_predicted(&f->cand, f->c_age_s);
    float   nu, s;
    if (kf_gate(&cp, z, &nu, &s)) {
        if (!kf_update(&cp, nu, s)) {
            cand_seed(f, z);                 /* defensive: never carry a NaN   */
            return;
        }
        f->cand    = cp;
        f->c_age_s = 0.0f;
        count_up(&f->c_n);
        rf_try_confirm(f, st);
    } else {
        /*  A miss. Once misses outnumber members the candidate is a poorer
         *  hypothesis than a fresh one, so the slot is re-seeded on this
         *  sample (a junk-seeded candidate yields to the real stream within
         *  a couple of samples, and vice versa).                            */
        count_up(&f->c_miss);
        if (f->c_miss > f->c_n) {
            cand_seed(f, z);
        }
    }
}

/* ===========================================================================
 *  Public API.
 * ===========================================================================*/

void rf_init(range_filter_t *f, float max_range_ft)
{
    if (f == NULL) {
        return;
    }
    memset(f, 0, sizeof *f);
    f->max_range_ft = max_range_ft;
    f->state        = RF_SEARCH;
    f->trk.r_var    = RF_R_MIN;
    f->cand.r_var   = RF_R_MIN;
    f->last_z       = NAN;               /* no previous sample to repeat       */
}

void rf_set_max_range(range_filter_t *f, float max_range_ft)
{
    if (f == NULL) {
        return;
    }
    f->max_range_ft = max_range_ft;
}

void rf_set_min_range(range_filter_t *f, float min_range_ft)
{
    if (f == NULL) {
        return;
    }
    /* A non-finite or negative floor disables the gate rather than poisoning it. */
    f->min_range_ft = (min_range_ft > 0.0f) ? min_range_ft : 0.0f;
}

bool rf_track_broken(const range_filter_t *f)
{
    return (f != NULL) && f->track_break;
}

bool rf_break_reentry(const range_filter_t *f)
{
    return (f != NULL) && f->track_break && f->break_reentry;
}

float rf_rate_fps(const range_filter_t *f)
{
    return (f != NULL) ? f->out_rate_fps : 0.0f;
}

rf_state_t rf_track_state(const range_filter_t *f)
{
    return (f != NULL) ? f->state : RF_SEARCH;
}

bool rf_reacquiring(const range_filter_t *f)
{
    /*  No live track, and the recent stream is mostly real returns: the ground
     *  is coming back into view and a new track is (about to be) confirming. */
    return (f != NULL) &&
           (f->state == RF_LOST || f->state == RF_SEARCH) &&
           f->stream_real;
}

bool rf_tracking(const range_filter_t *f)
{
    if (f == NULL) {
        return false;
    }
    /*  Before the first lock there is nothing to track, but we must NOT report
     *  "not tracking" and invite the caller to relax — a box that has never seen
     *  the ground yet is exactly when we want it awake and looking.             */
    return f->tracking || !f->have_out;
}

void rf_push_cm(range_filter_t *f, float cm)
{
    if (f == NULL) {
        return;
    }

    /* --- Validity gates: a gated sample is a NO-RETURN slot ------------------
     *  Every sample occupies a slot either way, so the per-sample timestamps
     *  rf_finalize() assigns stay correct.                                     */
    float slot = NAN;

    /*  The lost-signal check is a BAND, not an equality: the sensor's 16000 cm
     *  sentinel is one bit-flip away from 16001..16383 cm (525..537 ft), all of
     *  which are beyond any real return.                                       */
    if (cm >= 0.0f && cm < (float)SF30_LOST_SIGNAL_CM) {       /* NaN fails   */
        float ft = cm_to_ft(cm);
        /*  Closer than the ground can physically be: the lens (or a raindrop or
         *  bug on it) reflecting, not terrain. Evidence of NO ground return, so
         *  it votes with the sentinels — never read as "0 ft".                */
        bool below_floor = ft < f->min_range_ft;
        /*  Beyond the fitted sensor's ceiling (+margin) is physically
         *  impossible — the SF30/C cannot see 400 ft, so such a value is
         *  corruption by definition. (Inert for the SF30/D.)                  */
        bool beyond_ceiling = f->max_range_ft > 0.0f &&
                              ft > f->max_range_ft + RANGE_MAX_MARGIN_FT;
        if (!below_floor && !beyond_ceiling) {
            slot = ft;
        }
    }

    /* --- Ring: arrival order, the newest RANGE_DRAIN_MEDIAN_N kept ---------- */
    f->drain[f->drain_head] = slot;
    f->drain_head = (f->drain_head + 1u) % RANGE_DRAIN_MEDIAN_N;
    if (f->drain_n < RANGE_DRAIN_MEDIAN_N) {
        ++f->drain_n;
    }
    count_up(&f->drain_total);
}

void rf_drain_abort(range_filter_t *f)
{
    if (f == NULL) {
        return;
    }
    /* Hardware said these bytes are untrustworthy (framing/parity/overflow):
     * throw the WHOLE drain away. Its interval still elapses at the next
     * finalize (dt is wall-clock), so every clock stays honest.               */
    f->drain_n     = 0;
    f->drain_head  = 0;
    f->drain_total = 0;
}

bool rf_finalize(range_filter_t *f, float dt_s, float *range_ft, bool *fresh_valid)
{
    /* Defensive: outputs are always written, whatever the inputs. */
    float dummy_ft;
    bool  dummy_fresh;
    if (range_ft == NULL)    range_ft    = &dummy_ft;
    if (fresh_valid == NULL) fresh_valid = &dummy_fresh;
    *range_ft    = 0.0f;
    *fresh_valid = false;
    if (f == NULL) {
        return false;
    }

    /* Guard the elapsed time: never zero/negative/NaN, never absurdly large. */
    if (!(dt_s > 0.0f)) {
        dt_s = 0.001f;
    }
    if (dt_s > SENSOR_MAX_DT_S) {
        dt_s = SENSOR_MAX_DT_S;
    }

    /*  The break flags describe THIS finalize only. */
    f->track_break   = false;
    f->break_reentry = false;

    rf_drain_stats_t st = { 0u, 0u, false };

    /* --- Walk the drain, one timestamped sample at a time --------------------
     *  The sensor streams at a fixed rate, so the drain's samples are spread
     *  uniformly across the interval since the last finalize: sample k of n
     *  lands at t_prev + (k+1) * dt/n, and the newest one at "now". Samples the
     *  ring overflowed away still take up their time (it simply elapses before
     *  the oldest stored one).                                                 */
    uint32_t total = f->drain_total;
    size_t   n     = f->drain_n;
    if (total == 0u || n == 0u) {
        rf_advance(f, dt_s);                     /* a silent interval          */
    } else {
        if ((uint32_t)n > total) {
            total = (uint32_t)n;                 /* defensive consistency      */
        }
        float  step  = dt_s / (float)total;
        size_t start = (f->drain_head + RANGE_DRAIN_MEDIAN_N - n) %
                       RANGE_DRAIN_MEDIAN_N;
        float  h     = step * (float)(total - (uint32_t)n + 1u);
        for (size_t k = 0; k < n; ++k) {
            size_t idx = (start + k) % RANGE_DRAIN_MEDIAN_N;
            rf_process_sample(f, h, f->drain[idx], &st);
            h = step;
        }
    }

    /* The drain is consumed by this call no matter what. */
    f->drain_n     = 0;
    f->drain_head  = 0;
    f->drain_total = 0;

    /* --- Tracking verdict ------------------------------------------------------
     *  "Did this drain contain ANY usable return?" — the honest, directly
     *  observed question the power policy keys off. Deliberately NOT "was a
     *  sample accepted": a return the gate rejects still proves the sensor sees
     *  something, and the noisy, ambiguous stretch is exactly where the box must
     *  stay awake. Slow to go dark, instant to wake (see RANGE_NOTRACK_POLLS). */
    if (st.n_real > 0u) {
        f->notrack_polls = 0;
        f->tracking      = true;
    } else {
        count_up(&f->notrack_polls);
        if (f->notrack_polls >= (uint32_t)RANGE_NOTRACK_POLLS) {
            f->tracking = false;
        }
    }

    /* --- Sliding lost-signal vote over the last RANGE_LOST_VOTE_SAMPLES ------ */
    uint32_t win_n    = f->lost_bits_n;
    uint32_t win_mask = (win_n >= 32u) ? 0xFFFFFFFFu : ((1u << win_n) - 1u);
    uint32_t win_lost = (uint32_t)__builtin_popcount(f->lost_bits & win_mask);
    bool     full     = (win_n >= RANGE_LOST_VOTE_SAMPLES);
    bool     window_lost = full && (2u * win_lost > win_n);
    f->stream_real = full && !window_lost;

    /* --- Out-of-range ABOVE: pin at the ceiling --------------------------------
     *  "No return" has two opposite causes and holding is right for only one.
     *  If the track was lost within RANGE_CEILING_NEAR_FT of the ceiling and the
     *  stream then says "nothing there", the aircraft flew out the TOP: publish
     *  the ceiling, so the descent back into range is downward motion rather
     *  than a frozen mid-air number (the v1.62-era one-way door into CRUISE).
     *  Only a drain that actually CARRIED no-return evidence counts — a wholly
     *  EMPTY drain means the sensor said nothing at all (dead, unplugged, wrong
     *  baud) and must never be read as an altitude claim.                      */
    if (f->state == RF_LOST) {
        bool near_ceiling  = f->max_range_ft > 0.0f &&
                             f->out_ft >= f->max_range_ft - RANGE_CEILING_NEAR_FT;
        bool lost_evidence = (st.n_void > 0u) ||
                             (window_lost && st.n_real > 0u);
        if (near_ceiling && lost_evidence) {
            count_up(&f->ceiling_polls);
            if (f->ceiling_polls >= RANGE_CEILING_CONFIRM_POLLS) {
                f->above_ceiling = true;
                f->out_ft        = f->max_range_ft;
            }
        } else {
            f->ceiling_polls = 0;
            f->above_ceiling = false;
        }
    } else {
        f->ceiling_polls = 0;
        f->above_ceiling = false;
    }

    /* --- Publish --------------------------------------------------------------- */
    if (f->state == RF_TRACK || f->state == RF_COAST) {
        /*  COAST = this poll was predicted, not measured. */
        f->state = st.accepted ? RF_TRACK : RF_COAST;

        /*  The estimate is kept at its last accepted sample; publish it
         *  predicted to NOW so a trailing run of rejected samples does not
         *  make the published range lag the aircraft.                       */
        float now_ft = f->trk.x + f->trk.v * f->since_accept_s;
        if (isfinite(now_ft)) {
            f->out_ft = now_ft;
        }
        f->out_rate_fps = clamp_rate(f->trk.v);
    } else {
        f->out_rate_fps = 0.0f;                  /* an inference carries none  */
    }

    *range_ft    = f->out_ft;
    *fresh_valid = st.accepted;
    return f->have_out;
}
