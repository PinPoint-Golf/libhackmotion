/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wrist/clock.h — the device→host clock mapping, and an honest account of
 * how wrong it is.
 *
 * THE LIVE STREAM IS NOT THE DATA.  IT IS THE CLOCK REFERENCE FOR THE DATA.
 * (spec §7.6, api-request §2.14.)  A 250 ms downswing is 6 samples at the 25 Hz
 * live rate and ~200 in the device's internal buffer, so every sample a
 * consumer analyses comes from a history retrieval — and history records carry
 * no usable arrival time at all.  The only anchor is the sample index, and the
 * only thing that can map it is a fit maintained live, while the data was being
 * recorded.
 *
 * A wr_clock_snapshot is a self-contained immutable value (api-request C9).  It
 * is carried WITH a block of samples, never queried from a live device object
 * afterwards, so that re-analysing a capture a year later reproduces the same
 * alignment it produced on the day.
 */
#ifndef WRIST_CLOCK_H
#define WRIST_CLOCK_H

#include "wrist/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Rate constants (spec §6.5)                                                */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ ≈799.2 Hz, NEVER A ROUND 800.
 *
 * Measured on two independent host clocks: 799.19 Hz (238 s, phone) and
 * 799.32 Hz (14.9 s, laptop).  A second unit measured 799.47-799.55 Hz — about
 * 400 ppm away.  The constant moves between sessions and between devices.
 *
 * Assuming 800 costs ≈1,000 ppm — 1 ms per second of streaming, one-directional
 * (mapped times run EARLY): 45 ms after 45 s, which is 11 frames at 240 fps.
 * And it does something worse: the lower-envelope estimator SILENTLY
 * DEGENERATES.  With the rate wrong, hostArrival − index/rate drifts
 * monotonically instead of hovering, so its minimum always lands on one END of
 * the session — in practice the first frame, the one most exposed to
 * connection-setup jitter.  The estimator collapses from a robust fit over
 * thousands of frames to a single worst-case point while still reporting
 * small-looking residuals.  The library detects this and raises
 * WR_CLOCK_DEGENERATE.
 *
 * This value is a SEED for the first few frames of a stream and a sanity bound.
 * It is never the answer.
 */
#define WR_NOMINAL_SAMPLE_RATE_HZ   799.2
#define WR_RATE_PLAUSIBLE_MIN_HZ    795.0
#define WR_RATE_PLAUSIBLE_MAX_HZ    803.0

/* ≈64,068 ticks/s, range 64,025-64,088 across four sessions — 0.1 % (§6.4).
 * Fine for wrap arithmetic and sizing; fitted from data where timing matters. */
#define WR_NOMINAL_TICKS_PER_SAMPLE 80.166
#define WR_TICK_MODULUS             65536u

/*
 * ⚠ THE MEASURED SYSTEMATIC, in MICROseconds of error per second of session.
 * (2.2 ms/s is 2200 µs/s.  The unit is easy to get wrong by three orders and
 * the mistake is invisible, so the number is spelled out both ways below.)
 *
 * Spec §10, validated against five struck golf balls with a microphone
 * reference on the same host clock: the mapping drifts at 2.2 ms per second of
 * session, consistent across all five, R² = 1.000.  Roughly two thirds of that
 * is NOT accounted for by the reference used to measure it, and three candidate
 * causes were eliminated by direct measurement.
 *
 * A 240 fps camera frame is 4.17 ms, so this is gone in under two seconds of
 * session.  RESIDUALS CANNOT SEE IT — they measure link jitter AROUND the fit,
 * not whether the fit is right.
 *
 * ⚠⚠ THIS IS THE LEAST TRANSFERABLE NUMBER IN THE SPECIFICATION.  Everything
 * else in §10 is a property of the device.  This one is a property of a whole
 * measurement chain — sensor, link, host clock, sound card, room — measured
 * ONCE, on ONE rig, from ONE host, against a reference whose own calibration
 * §10 and §12 disagree about (759 ppm slow vs −219 ppm eliminated; the
 * arithmetic behind "≈1.4 ms/s unexplained" uses the first, and the second
 * would leave ≈2.0).  It is a starting value, not a device constant.
 *
 * ⚠ AND IT IS A SYSTEMATIC, NOT A RANDOM ERROR.  R² = 1.000 against session
 * time means it is perfectly predictable, and a predictable term is CORRECTED,
 * not budgeted.  Folding it into an error bar would say "we do not know where
 * this sample is to within 1.3 seconds" when the truth is "we know where it is,
 * up to a slope somebody measured once and nobody has applied".
 *
 * So the library reports it SEPARATELY from precision — see wr_clock_error —
 * and a consumer that measures its own rig feeds both halves back through
 * wr_session_set_clock_correction().
 */
#define WR_ACCURACY_DRIFT_US_PER_S_DEFAULT 2200.0   /* = 2.2 ms per second */

/* ------------------------------------------------------------------------ */
/* Snapshot                                                                  */
/* ------------------------------------------------------------------------ */
#define WR_PROVENANCE_MAX 64

/*
 * ⚠ TIME TO FIRST FIT: ONE LIVE FRAME.  There is no dead zone at the start of a
 * stream, and a consumer does not need to budget for dropped samples.
 *
 * WR_CLOCK_HAS_FIT is set from the very first live frame.  With one observation
 * the rate cannot be separated from the offset, so the fit uses the SEEDED rate
 * — the connection-pooled one if an earlier stream measured it, otherwise the
 * nominal 799.2 Hz — fits only the offset, and raises WR_CLOCK_SHORT_BASELINE.
 * Every sample from that first frame onward therefore carries a `host_time_us`.
 *
 * What the 2 s figure below actually gates is when SHORT_BASELINE CLEARS and
 * the rate becomes independently fitted rather than seeded.  It is bounded by
 * SPAN, not by observation count: two observations two seconds apart suffice,
 * and two thousand within one second do not.  The difference between the two
 * regimes is visible in wr_clock_error_at() — SHORT_BASELINE adds a
 * residual_max_us term — rather than hidden behind a boolean.
 *
 * ⚠ And note what does NOT depend on the fit at all: `sample_index` and each
 * unit's `device_time_us` are derived from the frame alone (§10.2), so they are
 * populated on every sample regardless of fit state — no fit, short baseline,
 * degenerate or stale.  Analysis anchored on device time rather than host time
 * never needs to wait for anything.
 */
#define WR_CLOCK_RATE_FIT_MIN_SPAN_US ((wr_time_us)2 * 1000 * 1000)

typedef enum wr_clock_flag {
    WR_CLOCK_HAS_FIT       = 1u << 0, /* a usable fit exists                        */
    /*
     * ⚠ The published rate IS the carried-over one — seeded, not fitted from
     * this stretch's own observations (§10).  It clears the moment the stretch
     * has enough baseline to fit its own.
     *
     * "Earlier streams" was too narrow twice over: design §6.1.1 re-anchors at
     * every history PULL, so the stretch after a retrieval also starts on a
     * carried rate, inside a single never-restarted stream.  It was also raised
     * on the joint-fit path, where the pooled value is only a starting point for
     * the hull scan — a consumer was told its rate came from elsewhere when it
     * had just been measured here (implementation-review I11).
     */
    WR_CLOCK_RATE_POOLED   = 1u << 1,
    WR_CLOCK_DEGENERATE    = 1u << 2, /* ⚠ envelope support collapsed to one end     */
    WR_CLOCK_RATE_IMPLAUSIBLE = 1u << 3, /* fitted rate outside the plausible band   */
    WR_CLOCK_STALE         = 1u << 4, /* no live observation for a long time         */
    WR_CLOCK_BLIND         = 1u << 5, /* a retrieval is in flight; live is suspended */
    WR_CLOCK_EXTERNAL_CORRECTION = 1u << 6, /* a consumer-supplied ppm is in force   */
    WR_CLOCK_SHORT_BASELINE = 1u << 7 /* span too short to separate rate from offset */
} wr_clock_flag;

/*
 * The mapping is:
 *
 *     host_us(index) = anchor_host_us + (index − anchor_index) * slope_us_per_index
 *
 * anchor_* is a point ON the fitted line, not an observation.  Storing the
 * mapping this way keeps the arithmetic well conditioned regardless of the
 * host clock's epoch.
 */
typedef struct wr_clock_snapshot {
    uint64_t   stream_id;
    uint32_t   flags;              /* wr_clock_flag */
    int32_t    observations;       /* live frames contributing to this fit */

    uint32_t   anchor_index;
    uint32_t   reserved0;
    wr_time_us anchor_host_us;
    double     slope_us_per_index; /* after any external correction */

    double     fitted_rate_hz;     /* 1e6 / slope, after correction — ~799.2, NOT 800 */
    double     raw_fitted_rate_hz; /* before correction */
    double     external_ppm;       /* consumer-supplied, 0 if none */

    wr_time_us offset_us;          /* host_us(0); the lower-envelope estimate, reported */
    wr_time_us span_us;            /* how long a baseline the fit rests on */
    wr_time_us last_observation_us;/* stale mappings must be visible as stale */
    uint32_t   first_index;        /* observed index span, for extrapolation accounting */
    uint32_t   last_index;

    /*
     * PRECISION — the evidence behind the fit (api-request B3).  These measure
     * link jitter around the fit and nothing else.  ⚠ Treat a widening spread
     * as a LINK-HEALTH signal: BLE at range delays notifications and nothing in
     * the protocol reports it, so the fit degrades quietly while every frame
     * still parses (§10).
     *
     * ⚠ WHILE `span_us` IS 0 THESE CARRY THE CONNECTION'S LAST MEASUREMENT
     * rather than reading zero.  A fit resting on a single instant has no
     * spread to measure, and a zero there is an absence of evidence rather than
     * an absence of error — the shape that switched off the alignment gate once
     * per pull (implementation-review I2).  `span_us == 0` is the marker; it is
     * the state design §6.1.1's re-anchor leaves behind until the next live
     * frame after a retrieval.
     */
    uint32_t   residual_median_us;
    uint32_t   residual_p90_us;
    uint32_t   residual_max_us;

    /* ACCURACY — what precision cannot cover.  See the constant above. */
    double     accuracy_drift_us_per_s;

    /* Where any external correction came from; empty if none.  Carried into
     * the capture's provenance so a re-analysis reproduces the alignment. */
    char       provenance[WR_PROVENANCE_MAX];
} wr_clock_snapshot;

/*
 * Map a device sample index to host time.  Pure; depends only on the snapshot.
 * Returns WR_TIME_UNKNOWN if the snapshot carries no fit.
 */
WR_API wr_time_us wr_clock_to_host_us(const wr_clock_snapshot *fit, uint32_t sample_index);

/*
 * The inverse, for sizing an `a1` request from a host-time window.
 * Returns WR_ERR_NO_FIT with no fit, WR_ERR_INVALID_ARG if the result would
 * fall outside the unwrapped index space.
 */
WR_API wr_status wr_clock_index_for_host_us(const wr_clock_snapshot *fit,
                                            wr_time_us host_us,
                                            uint32_t *out_index);

/*
 * ⚠ PRECISION AND ACCURACY ARE DIFFERENT NUMBERS AND THEY DO NOT BELONG IN ONE
 * FIGURE (api-request B2).  Both are reported, AT THIS SAMPLE — not for the
 * batch, because a single batch-level figure cannot express a fit that was poor
 * early in a session and good later.
 *
 *   precision_us   what the residuals can actually see:
 *                    residual_p90_us                     link jitter
 *                  + extrapolation                       beyond the observations,
 *                                                        which is what a retrieval
 *                                                        blind span produces (B15)
 *                  inflated when the fit is degenerate or its baseline too short,
 *                  and FLOORED at one sample period while `span_us` is 0.
 *
 *                  ⚠ That floor is not conservatism, it is the difference
 *                  between a measurement and an assertion.  With the evidence
 *                  resting on a single instant every residual is 0 BY
 *                  CONSTRUCTION — one point cannot disagree with a line through
 *                  it — so every term above evaluates to zero and this reported
 *                  perfect accuracy from one observation, which made
 *                  wr_clock_meets_budget() accept any budget at all.  Design
 *                  §6.1.1 re-anchors the fit at every history pull, so that
 *                  state recurs once per RETRIEVAL rather than once per stream.
 *                  The residual figures carry the connection's last real
 *                  measurement across a re-anchor for the same reason the rate
 *                  is pooled — jitter is a property of the link — and the floor
 *                  covers the first fit of a connection, where there is none.
 *
 *   systematic_us  accuracy_drift_us_per_s × seconds-into-the-session.
 *                  ⚠ Perfectly predictable (R² = 1.000), measured once on one
 *                  rig, and NOT corrected for.  It grows without bound across a
 *                  session: at ten minutes it is 1.32 s.
 *
 *   total_us       precision + systematic.  The honest total, for provenance.
 *
 * ⚠ GATE ON precision_us, NOT ON total_us.  A consumer refusing a pull whose
 * alignment is worse than a camera frame is asking about the part of the error
 * that varies with the link and the fit.  Gating on the total would refuse every
 * pull after the first second of any session, which disables the feature rather
 * than informing it.  wr_clock_meets_budget() gates on precision for exactly
 * this reason.  Record total_us in the capture's provenance regardless.
 *
 * All three saturate rather than overflow.  UINT32_MAX means "no fit; do not
 * align" — refusing rather than guessing.
 */
typedef struct wr_clock_error {
    uint32_t precision_us;
    uint32_t systematic_us;
    uint32_t total_us;
} wr_clock_error;

WR_API wr_clock_error wr_clock_error_at(const wr_clock_snapshot *fit, uint32_t sample_index);

/* Convenience accessors for the two halves.  Same numbers, one at a time. */
WR_API uint32_t wr_clock_precision_us(const wr_clock_snapshot *fit, uint32_t sample_index);
WR_API uint32_t wr_clock_uncertainty_us(const wr_clock_snapshot *fit, uint32_t sample_index);

/* True if aligning this sample to within `budget_us` is defensible.  ⚠ Compares
 * against precision_us — see above.  A consumer that must refuse a pull whose
 * alignment is worse than one camera frame, and record the refusal in the
 * capture's provenance, asks this. */
WR_API bool wr_clock_meets_budget(const wr_clock_snapshot *fit,
                                  uint32_t sample_index,
                                  uint32_t budget_us);

/* ------------------------------------------------------------------------ */
/* Feeding a measured correction back                                        */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THERE ARE TWO KNOBS, THEY COME OUT OF ONE MEASUREMENT, AND WHICH ONES YOU
 * ARE SETTING IS STATED EXPLICITLY.
 *
 * `rate_ppm` changes slope_us_per_index — the rate the fit uses.
 * `residual_drift_us_per_s` changes accuracy_drift_us_per_s — the systematic
 * that remains AFTER the rate correction has been applied.
 *
 * `fields` says which of the two this call is setting.  It is REQUIRED and a
 * zero `fields` is an error, which makes all three failure shapes unreachable:
 *
 *   - Setting only the rate and finding the systematic unchanged looks like the
 *     call did nothing.  Now you either set both or you said you did not.
 *   - ⚠ Setting only the rate and silently ZEROING the systematic is worse,
 *     because it is optimistic: the capture would claim an accuracy it had not
 *     earned, with a provenance string describing a measurement that covered
 *     half of it.  `= {0}` is the idiom for every other struct in this API, so
 *     a value-range sentinel would be reached for by accident here.
 *   - And `= {0}` on its own no longer quietly does nothing: it is rejected.
 *
 * ⚠ Note the deliberate asymmetry with wr_session_policy, where 0 means "use
 * the default": here 0 in a field you have FLAGGED means a measured zero, and a
 * field you have not flagged is untouched.  The names differ
 * (`accuracy_drift_us_per_s` vs `residual_drift_us_per_s`) to keep the two
 * apart, and neither ever means "guess".
 *
 * Worked example.  A consumer with a 240 fps camera and a microphone on the
 * same monotonic clock measures its own rig and finds the mapping runs 350 ppm
 * fast and, once that is taken out, drifts by a further 400 µs per second:
 *
 *     wr_clock_correction c = {0};
 *     c.fields                  = WR_CORRECTION_RATE | WR_CORRECTION_DRIFT;
 *     c.rate_ppm                = -350.0;
 *     c.residual_drift_us_per_s =  400.0;
 *     snprintf(c.provenance, sizeof c.provenance, "bay 3 camera+mic 2026-08-20");
 *     wr_session_set_clock_correction(session, &c);
 *
 * A consumer that has measured only the rate sets only WR_CORRECTION_RATE, and
 * the systematic keeps whatever value the policy gave it.
 *
 * Everything set here is carried in every subsequent wr_clock_snapshot, and
 * therefore into every history block, so a re-analysis a year later reproduces
 * the same alignment (api-request C9).
 */
typedef enum wr_correction_field {
    WR_CORRECTION_RATE  = 1u << 0, /* apply rate_ppm                */
    WR_CORRECTION_DRIFT = 1u << 1  /* apply residual_drift_us_per_s */
} wr_correction_field;

typedef struct wr_clock_correction {
    uint32_t fields;                 /* bitmask; 0 → WR_ERR_INVALID_ARG              */
    uint32_t reserved;
    double   rate_ppm;               /* rate_effective = rate_fitted × (1 + ppm/1e6) */
    double   residual_drift_us_per_s;/* µs of error per second of session            */
    char     provenance[WR_PROVENANCE_MAX];
} wr_clock_correction;

/* ------------------------------------------------------------------------ */
/* Host time → index                                                         */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THE TWO RANGE TYPES HAVE DIFFERENT CONVENTIONS, AND THIS IS WHERE AN
 * OFF-BY-ONE IS BORN.  wr_time_range is HALF-OPEN [start, end); wr_index_range
 * is INCLUSIVE [first, last].  This function is the one blessed conversion:
 * it yields every sample whose mapped host time falls inside the window, and
 * nothing else.
 *
 * Use it rather than calling wr_clock_index_for_host_us() twice.
 *
 * Returns WR_ERR_NO_FIT with no fit, WR_ERR_INVALID_ARG if the window is empty,
 * inverted, or maps outside the unwrapped index space.
 */
WR_API wr_status wr_clock_index_range_for_time(const wr_clock_snapshot *fit,
                                               wr_time_range window,
                                               wr_index_range *out);

WR_API const char *wr_clock_flag_name(wr_clock_flag flag);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WRIST_CLOCK_H */
