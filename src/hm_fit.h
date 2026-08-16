/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_fit.h — internal.  The device→host clock fit.
 *
 * ⚠ FIT THE LOWER ENVELOPE, NOT LEAST SQUARES (spec §10).  BLE delay is
 * one-sided — a notification can be late, never early — so ordinary least
 * squares biases the offset by the MEAN link delay, which reaches the user as
 * every event landing consistently late.
 *
 * ⚠ FIT THE RATE TOO.  Hard-coding 800 Hz costs ≈1 ms per second AND silently
 * degenerates this estimator: with the rate wrong, hostArrival − index/rate
 * drifts monotonically instead of hovering, so its minimum always lands on one
 * END of the session — in practice the first frame, the one most exposed to
 * connection-setup jitter — collapsing a robust fit over thousands of frames
 * onto a single worst-case point while still reporting small residuals.
 * hm_fit detects that and raises HM_CLOCK_DEGENERATE.
 *
 * ── The estimator ──────────────────────────────────────────────────────────
 * Minimise Σ(hᵢ − line(iᵢ)) subject to line(iᵢ) ≤ hᵢ for all i: the highest line
 * that still lies under every observation, which is the maximum-likelihood
 * answer for one-sided delay.  Writing the line as slope s through a point
 * (i₀,h₀) the objective becomes
 *
 *     cost(s, i₀, h₀) = (Σh − n·h₀) − s·(Σi − n·i₀)
 *
 * which is LINEAR in s given running sums.  The feasible optimum is attained on
 * an edge of the lower convex hull of the observations, so the whole fit is:
 * maintain the hull incrementally (O(1) amortised, points arrive with i
 * ascending), then scan its edges in O(hull).  Exact, not iterative, and
 * bounded in both time and memory.
 *
 * ── Rate pooling (spec §10) ────────────────────────────────────────────────
 * The counter resets at every stream start but the crystal does not, so the
 * correct decomposition is a PER-SESSION OFFSET WITH A RATE POOLED ACROSS THE
 * WHOLE CONNECTION.  hm_fit_begin_stream() folds the finished stream's slope
 * into a span-weighted pooled estimate and seeds the next stream with it.
 */
#ifndef HM_FIT_H
#define HM_FIT_H

#include "hackmotion/types.h"
#include "hackmotion/clock.h"

#define HM_FIT_HULL_MAX   96u
#define HM_FIT_RESID_MAX 256u

/* Below this baseline the rate cannot be separated from the offset, and trying
 * produces a wild slope from a few milliseconds of jitter. */
#define HM_FIT_MIN_SPAN_US ((hm_time_us)2 * 1000 * 1000)

/* No live observation for this long and the fit is stale rather than current;
 * a retrieval blind span (api-request B15) is the common cause. */
#define HM_FIT_STALE_AFTER_US ((hm_time_us)3 * 1000 * 1000)

typedef struct hm_fit_point {
    int64_t i; /* sample index, relative to the stream origin */
    int64_t h; /* host arrival, µs, relative to the stream origin */
} hm_fit_point;

typedef struct hm_fit {
    /* Origin — everything is stored relative to it so the arithmetic stays
     * well conditioned whatever epoch the caller's clock uses. */
    bool       have_origin;
    uint32_t   origin_index;
    hm_time_us origin_host_us;
    uint64_t   stream_id;

    /* Lower convex hull of the observations. */
    hm_fit_point hull[HM_FIT_HULL_MAX];
    size_t       hull_n;
    uint32_t     hull_dropped;      /* vertices lost to capacity */
    int64_t      hull_drop_error_us;/* worst upward bias that dropping introduced */

    /* Running sums over ALL observations. */
    int64_t sum_i;
    int64_t sum_h;
    int64_t n;

    /* Recent observations, for the residual distribution. */
    hm_fit_point resid[HM_FIT_RESID_MAX];
    size_t       resid_n;
    size_t       resid_head;

    int64_t    first_i, last_i;
    hm_time_us last_host_us;

    /* Current fit. */
    bool     have_fit;
    double   slope;          /* µs per index, BEFORE external correction */
    int64_t  anchor_i;
    int64_t  anchor_h;
    uint32_t flags;
    uint32_t residual_median_us;
    uint32_t residual_p90_us;
    uint32_t residual_max_us;
    bool     dirty;

    /* Connection-scoped pooled rate. */
    double  pooled_slope;
    double  pooled_weight;   /* span in µs, summed */

    /*
     * ⚠ CONNECTION-SCOPED LINK JITTER, and it survives a re-anchor for the same
     * reason the pooled rate does: it is a property of the LINK, not of the
     * stretch (spec §10 — BLE at range delays notifications and nothing in the
     * protocol reports it).
     *
     * Design §6.1.1 re-anchors at every history pull, which wipes the residual
     * ring.  Until the new stretch has a baseline of its own, every residual is
     * 0 BY CONSTRUCTION — one point cannot disagree with a line through it — and
     * publishing that zero made hm_clock_meets_budget() accept any budget at all
     * (implementation-review I2).  A spread of zero out of one observation is an
     * absence of evidence, not an absence of error, so the last real measurement
     * stands in until this stretch has one.
     */
    uint32_t carry_p90_us;
    uint32_t carry_max_us;

    /* Consumer-supplied correction and its provenance. */
    double  external_ppm;
    char    provenance[HM_PROVENANCE_MAX];

    double  accuracy_drift_us_per_s;

    /* Test hook — see hm_fit_force_slope(). */
    bool   slope_forced;
    double forced_slope;
} hm_fit;

void hm_fit_init(hm_fit *f, double accuracy_drift_us_per_s);

/* Fold the finished stream's rate into the pooled estimate and start a new
 * index space.  Two connection-scoped things survive — the pooled RATE and the
 * carried link JITTER (see carry_p90_us) — and nothing else does. */
void hm_fit_begin_stream(hm_fit *f, uint64_t stream_id);

/* One live frame.  History records must NOT be fed here: their arrival
 * timestamps carry no information (spec §10, §10.1). */
void hm_fit_observe(hm_fit *f, uint32_t sample_index, hm_time_us host_recv_us);

/* Mark the fit blind — a retrieval is in flight and live delivery is suspended
 * for its duration, so uncertainty must not stay flat across it. */
void hm_fit_set_blind(hm_fit *f, bool blind);

/*
 * ⚠ Sets BOTH knobs, because they come out of one measurement.  `rate_ppm`
 * moves the slope; `residual_drift_us_per_s` moves the systematic that remains
 * after the slope correction.  A consumer that could set only the first would
 * feed back a measurement and see its reported uncertainty unchanged.
 */
hm_status hm_fit_set_correction(hm_fit *f, const hm_clock_correction *correction);

/*
 * ⚠ TEST HOOK, and the only way to reach the failure §10 warns about.
 *
 * Pins the slope instead of fitting it, which is what hard-coding 800 Hz does.
 * The fit then reports HM_CLOCK_DEGENERATE, because the pinned rate disagrees
 * with the empirical slope across the session — the observable symptom of an
 * estimator that has collapsed onto one end-of-session point while still
 * reporting small residuals.  Pass 0 to release the pin.
 *
 * Nothing in the shipping session calls this.
 */
void hm_fit_force_slope(hm_fit *f, double slope_us_per_index);

/* Recompute if needed and publish.  `now_us` only drives the STALE flag. */
void hm_fit_snapshot(hm_fit *f, hm_time_us now_us, hm_clock_snapshot *out);

#endif /* HM_FIT_H */
