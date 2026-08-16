/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
#include "hm_fit.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ========================================================================= */
/* Pure snapshot functions (public API)                                      */
/* ========================================================================= */

hm_time_us hm_clock_to_host_us(const hm_clock_snapshot *fit, uint32_t sample_index)
{
    double di;
    if (fit == NULL || (fit->flags & (uint32_t)HM_CLOCK_HAS_FIT) == 0u) {
        return HM_TIME_UNKNOWN;
    }
    di = (double)sample_index - (double)fit->anchor_index;
    return fit->anchor_host_us + (hm_time_us)llround(di * fit->slope_us_per_index);
}

hm_status hm_clock_index_for_host_us(const hm_clock_snapshot *fit, hm_time_us host_us,
                                     uint32_t *out_index)
{
    double di;
    double idx;

    if (fit == NULL || out_index == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    if ((fit->flags & (uint32_t)HM_CLOCK_HAS_FIT) == 0u ||
        !(fit->slope_us_per_index > 0.0)) {
        return HM_ERR_NO_FIT;
    }
    di = ((double)host_us - (double)fit->anchor_host_us) / fit->slope_us_per_index;
    idx = (double)fit->anchor_index + di;
    if (!(idx >= 0.0) || idx > 4294967295.0) {
        return HM_ERR_INVALID_ARG;
    }
    *out_index = (uint32_t)llround(idx);
    return HM_OK;
}

static double saturate_u32(double v)
{
    if (!(v >= 0.0)) {
        return 0.0;
    }
    if (v >= 4294967295.0) {
        return 4294967295.0;
    }
    return v;
}

hm_clock_error hm_clock_error_at(const hm_clock_snapshot *fit, uint32_t sample_index)
{
    hm_clock_error e;
    double precision;
    double systematic;
    double dt_s;
    double gap_indices = 0.0;

    if (fit == NULL || (fit->flags & (uint32_t)HM_CLOCK_HAS_FIT) == 0u) {
        e.precision_us = UINT32_MAX;
        e.systematic_us = UINT32_MAX;
        e.total_us = UINT32_MAX;
        return e; /* no fit: do not align */
    }

    /* ---- PRECISION: what the residuals can actually see ------------------ */
    precision = (double)fit->residual_p90_us;

    /* Beyond the observations the rate's own uncertainty propagates.  A
     * retrieval blind span (api-request B15) puts every sample here. */
    if (sample_index > fit->last_index) {
        gap_indices = (double)(sample_index - fit->last_index);
    } else if (sample_index < fit->first_index) {
        gap_indices = (double)(fit->first_index - sample_index);
    }
    if (gap_indices > 0.0) {
        double gap_us = gap_indices * fit->slope_us_per_index;
        precision += (double)fit->residual_max_us;
        if (fit->span_us > 0) {
            precision += gap_us * ((double)fit->residual_max_us / (double)fit->span_us);
        }
    }
    if ((fit->flags & (uint32_t)HM_CLOCK_DEGENERATE) != 0u) {
        /* The offset rests on a single worst-case point.  There is no honest
         * small number to report here. */
        precision = precision * 2.0 + (double)fit->residual_max_us;
    }
    if ((fit->flags & (uint32_t)HM_CLOCK_SHORT_BASELINE) != 0u) {
        precision += (double)fit->residual_max_us;
    }
    /*
     * ⚠ THE FLOOR WHEN NOTHING HAS EVER BEEN MEASURED (implementation-review I2).
     *
     * Every term above is a multiple of a residual, and with the evidence resting
     * on a single instant every residual is 0 by construction — so this used to
     * return precision_us = 0, and hm_clock_meets_budget() accepted ANY budget.
     * AR B2's refuse-rather-than-misalign gate, silently off.
     *
     * The connection's own measured jitter stands in where there is one
     * (hm_fit.carry_*); this covers the first fit of a connection, where there is
     * not.  One sample period is the honest number: with one jittered arrival and
     * a SEEDED rate, the anchor cannot be placed to better than the interval
     * between the samples that were not seen.  ≈1251 µs at 799.2 Hz — about a
     * third of a 240 fps camera frame, so a consumer asking for frame-accurate
     * alignment still gets it, and one asking for sub-millisecond is refused
     * until the fit has earned it.
     */
    if (fit->span_us == 0 && fit->slope_us_per_index > 0.0 &&
        precision < fit->slope_us_per_index) {
        precision = fit->slope_us_per_index;
    }

    /* ---- SYSTEMATIC: §10's measured, uncorrected drift ------------------- */
    /*
     * ⚠ Deliberately NOT added to precision.  R² = 1.000 against session time
     * makes this predictable rather than uncertain, and it grows without bound:
     * folded into one figure it refuses every pull after the first second of a
     * session, which disables the refuse-rather-than-misalign path instead of
     * informing it.  Reported, carried into provenance, and gated on separately.
     */
    dt_s = ((double)sample_index - (double)fit->first_index) * fit->slope_us_per_index / 1e6;
    if (dt_s < 0.0) {
        dt_s = -dt_s;
    }
    systematic = fit->accuracy_drift_us_per_s * dt_s;

    e.precision_us = (uint32_t)saturate_u32(precision);
    e.systematic_us = (uint32_t)saturate_u32(systematic);
    e.total_us = (uint32_t)saturate_u32(precision + systematic);
    return e;
}

uint32_t hm_clock_precision_us(const hm_clock_snapshot *fit, uint32_t sample_index)
{
    return hm_clock_error_at(fit, sample_index).precision_us;
}

uint32_t hm_clock_uncertainty_us(const hm_clock_snapshot *fit, uint32_t sample_index)
{
    return hm_clock_error_at(fit, sample_index).total_us;
}

bool hm_clock_meets_budget(const hm_clock_snapshot *fit, uint32_t sample_index,
                           uint32_t budget_us)
{
    /* ⚠ precision, not total — see hm_clock_error. */
    return hm_clock_error_at(fit, sample_index).precision_us <= budget_us;
}

hm_status hm_clock_index_range_for_time(const hm_clock_snapshot *fit, hm_time_range window,
                                        hm_index_range *out)
{
    double first_exact;
    double last_exact;
    double first_i;
    double last_i;

    if (fit == NULL || out == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    if (window.start_us >= window.end_us) {
        return HM_ERR_INVALID_ARG; /* half-open and empty */
    }
    if ((fit->flags & (uint32_t)HM_CLOCK_HAS_FIT) == 0u ||
        !(fit->slope_us_per_index > 0.0)) {
        return HM_ERR_NO_FIT;
    }

    /*
     * hm_time_range is half-open, hm_index_range is inclusive.  The conversion
     * is therefore not symmetric, and writing it once here is the whole point:
     *   first = smallest index whose host time is >= start   → ceil
     *   last  = largest  index whose host time is <  end     → ceil(end) − 1
     */
    first_exact = (double)fit->anchor_index +
                  ((double)(window.start_us - fit->anchor_host_us) / fit->slope_us_per_index);
    last_exact = (double)fit->anchor_index +
                 ((double)(window.end_us - fit->anchor_host_us) / fit->slope_us_per_index);

    first_i = ceil(first_exact);
    last_i = ceil(last_exact) - 1.0;

    if (last_i < first_i) {
        return HM_ERR_INVALID_ARG; /* the window falls between two samples */
    }
    if (first_i < 0.0 || last_i > 4294967295.0) {
        return HM_ERR_INVALID_ARG;
    }

    out->first = (uint32_t)first_i;
    out->last = (uint32_t)last_i;
    return HM_OK;
}

/* ========================================================================= */
/* The fitter                                                                */
/* ========================================================================= */

static double nominal_slope(void)
{
    return 1e6 / HM_NOMINAL_SAMPLE_RATE_HZ;
}

void hm_fit_init(hm_fit *f, double accuracy_drift_us_per_s)
{
    memset(f, 0, sizeof(*f));
    f->slope = nominal_slope();
    f->accuracy_drift_us_per_s =
        (accuracy_drift_us_per_s > 0.0) ? accuracy_drift_us_per_s
                                        : HM_ACCURACY_DRIFT_US_PER_S_DEFAULT;
    f->dirty = true;
}

void hm_fit_begin_stream(hm_fit *f, uint64_t stream_id)
{
    /* Fold the finished stream's rate into the pooled estimate before throwing
     * the index space away.  Only a fit with a real baseline and no degeneracy
     * earns a vote. */
    if (f->have_fit && f->n >= 64 && (f->flags & (uint32_t)HM_CLOCK_DEGENERATE) == 0u &&
        (f->flags & (uint32_t)HM_CLOCK_SHORT_BASELINE) == 0u) {
        double span = (double)(f->last_host_us - f->origin_host_us);
        if (span > 0.0) {
            double w = f->pooled_weight + span;
            f->pooled_slope = (f->pooled_slope * f->pooled_weight + f->slope * span) / w;
            f->pooled_weight = w;
        }
    }

    f->have_origin = false;
    f->stream_id = stream_id;
    f->hull_n = 0;
    f->hull_dropped = 0;
    f->hull_drop_error_us = 0;
    f->sum_i = 0;
    f->sum_h = 0;
    f->n = 0;
    f->resid_n = 0;
    f->resid_head = 0;
    f->first_i = 0;
    f->last_i = 0;
    f->last_host_us = 0;
    f->have_fit = false;
    f->flags = 0;
    f->residual_median_us = 0;
    f->residual_p90_us = 0;
    f->residual_max_us = 0;
    f->slope = (f->pooled_weight > 0.0) ? f->pooled_slope : nominal_slope();
    f->dirty = true;
}

static int64_t cross_of(hm_fit_point o, hm_fit_point a, hm_fit_point b)
{
    return (a.i - o.i) * (b.h - o.h) - (a.h - o.h) * (b.i - o.i);
}

/* Vertical distance from H[j] up to the chord H[j-1]→H[j+1].  Non-negative for
 * a lower hull; this is exactly the bias that dropping H[j] would introduce. */
static int64_t chord_deviation(const hm_fit_point *H, size_t j)
{
    double di = (double)(H[j + 1].i - H[j - 1].i);
    double chord_h;
    if (di == 0.0) {
        return 0;
    }
    chord_h = (double)H[j - 1].h +
              ((double)(H[j].i - H[j - 1].i) / di) * (double)(H[j + 1].h - H[j - 1].h);
    return (int64_t)llround(chord_h - (double)H[j].h);
}

static void hull_make_room(hm_fit *f)
{
    size_t best = 1;
    int64_t best_dev = INT64_MAX;

    if (f->hull_n < HM_FIT_HULL_MAX) {
        return;
    }
    /*
     * Drop the interior vertex whose removal changes the chain least, keeping
     * the two extremes — and therefore the baseline the rate rests on.  A
     * subset of points in convex position is still in convex position, so the
     * chain stays a valid hull; the cost is that a candidate line may now sit
     * up to `hull_drop_error_us` above one discarded observation.  That bias is
     * accumulated and folded into the reported residual maximum rather than
     * quietly ignored.
     */
    for (size_t j = 1; j + 1 < f->hull_n; ++j) {
        int64_t dev = chord_deviation(f->hull, j);
        if (dev < 0) {
            dev = -dev;
        }
        if (dev < best_dev) {
            best_dev = dev;
            best = j;
        }
    }
    if (best_dev != INT64_MAX && best_dev > f->hull_drop_error_us) {
        f->hull_drop_error_us = best_dev;
    }
    memmove(&f->hull[best], &f->hull[best + 1], (f->hull_n - best - 1) * sizeof(f->hull[0]));
    f->hull_n--;
    f->hull_dropped++;
}

void hm_fit_observe(hm_fit *f, uint32_t sample_index, hm_time_us host_recv_us)
{
    hm_fit_point p;

    if (!f->have_origin) {
        f->have_origin = true;
        f->origin_index = sample_index;
        f->origin_host_us = host_recv_us;
        f->first_i = 0;
    }

    p.i = (int64_t)sample_index - (int64_t)f->origin_index;
    p.h = host_recv_us - f->origin_host_us;

    /* Points must arrive with i non-decreasing for the incremental hull.  A
     * genuinely out-of-order live frame is discarded from the fit (it is still
     * delivered to the consumer); the index unwrapper reports the anomaly. */
    if (f->n > 0 && p.i < f->last_i) {
        return;
    }

    f->sum_i += p.i;
    f->sum_h += p.h;
    f->n += 1;
    f->last_i = p.i;
    f->last_host_us = host_recv_us;

    f->resid[f->resid_head] = p;
    f->resid_head = (f->resid_head + 1u) % HM_FIT_RESID_MAX;
    if (f->resid_n < HM_FIT_RESID_MAX) {
        f->resid_n++;
    }

    while (f->hull_n >= 2u &&
           cross_of(f->hull[f->hull_n - 2u], f->hull[f->hull_n - 1u], p) <= 0) {
        f->hull_n--;
    }
    hull_make_room(f);
    f->hull[f->hull_n++] = p;

    f->dirty = true;
}

void hm_fit_set_blind(hm_fit *f, bool blind)
{
    if (blind) {
        f->flags |= (uint32_t)HM_CLOCK_BLIND;
    } else {
        f->flags &= ~(uint32_t)HM_CLOCK_BLIND;
    }
}

hm_status hm_fit_set_correction(hm_fit *f, const hm_clock_correction *c)
{
    if (f == NULL || c == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    /*
     * ⚠ An empty `fields` is rejected rather than treated as a no-op.  `= {0}`
     * is the idiom for every other struct in this API, so it is what a consumer
     * reaches for by accident — and the two things it could silently mean here
     * (do nothing / wipe a term nobody measured) are both wrong, the second
     * optimistically so.  Making it an error is the only reading that cannot
     * mislead.
     */
    if ((c->fields & (uint32_t)(HM_CORRECTION_RATE | HM_CORRECTION_DRIFT)) == 0u) {
        return HM_ERR_INVALID_ARG;
    }
    if ((c->fields & (uint32_t)HM_CORRECTION_RATE) != 0u &&
        !(c->rate_ppm > -100000.0 && c->rate_ppm < 100000.0)) {
        return HM_ERR_INVALID_ARG;
    }
    if ((c->fields & (uint32_t)HM_CORRECTION_DRIFT) != 0u &&
        !(c->residual_drift_us_per_s >= 0.0 && c->residual_drift_us_per_s < 1e9)) {
        return HM_ERR_INVALID_ARG;
    }

    if ((c->fields & (uint32_t)HM_CORRECTION_RATE) != 0u) {
        f->external_ppm = c->rate_ppm;
    }
    if ((c->fields & (uint32_t)HM_CORRECTION_DRIFT) != 0u) {
        /* A flagged zero is a MEASURED zero and is honoured as one. */
        f->accuracy_drift_us_per_s = c->residual_drift_us_per_s;
    }

    memset(f->provenance, 0, sizeof(f->provenance));
    {
        size_t n = strlen(c->provenance);
        if (n > sizeof(f->provenance) - 1u) {
            n = sizeof(f->provenance) - 1u;
        }
        memcpy(f->provenance, c->provenance, n);
    }
    f->dirty = true;
    return HM_OK;
}

/* --- residual statistics ------------------------------------------------- */

static int cmp_i64(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a;
    int64_t y = *(const int64_t *)b;
    return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

static void residual_stats(hm_fit *f)
{
    int64_t buf[HM_FIT_RESID_MAX];
    size_t n = f->resid_n;

    f->residual_median_us = 0;
    f->residual_p90_us = 0;
    f->residual_max_us = 0;

    /*
     * ⚠ THE EVIDENCE IS A SINGLE INSTANT, so there is no spread to measure and
     * a zero here would be a perfect-accuracy claim from one data point
     * (implementation-review I2).  Stand the last real measurement of THIS
     * LINK in instead — §6.1.1 re-anchors at every pull, so this state recurs
     * once per retrieval and the link has not changed across it.
     *
     * `span_us == 0` rather than `n < 2`: two observations sharing a host
     * timestamp are the same degenerate case wearing a different count.
     */
    if (n == 0u || f->last_host_us == f->origin_host_us) {
        f->residual_median_us = f->carry_p90_us;
        f->residual_p90_us = f->carry_p90_us;
        f->residual_max_us = f->carry_max_us;
        return;
    }

    for (size_t k = 0; k < n; ++k) {
        size_t idx = (f->resid_head + HM_FIT_RESID_MAX - n + k) % HM_FIT_RESID_MAX;
        double line = (double)f->anchor_h + f->slope * (double)(f->resid[idx].i - f->anchor_i);
        int64_t r = (int64_t)llround((double)f->resid[idx].h - line);
        buf[k] = (r < 0) ? 0 : r;
    }
    qsort(buf, n, sizeof(buf[0]), cmp_i64);

    f->residual_median_us = (uint32_t)saturate_u32((double)buf[n / 2u]);
    f->residual_p90_us = (uint32_t)saturate_u32((double)buf[(n * 9u) / 10u]);
    f->residual_max_us = (uint32_t)saturate_u32((double)buf[n - 1u]);

    /* Fold in whatever bias hull simplification could have introduced. */
    if (f->hull_drop_error_us > 0) {
        double m = (double)f->residual_max_us + (double)f->hull_drop_error_us;
        f->residual_max_us = (uint32_t)saturate_u32(m);
    }

    /* This stretch measured the link; remember it for the next re-anchor. */
    f->carry_p90_us = f->residual_p90_us;
    f->carry_max_us = f->residual_max_us;
}

/* Highest feasible line at a FIXED slope: the anchor is the hull vertex
 * minimising h − s·i, which is exact because that minimum is always attained on
 * the lower hull. */
static void anchor_at_slope(hm_fit *f, double slope, size_t *out_vertex)
{
    size_t best = 0;
    double best_v = 0.0;

    for (size_t k = 0; k < f->hull_n; ++k) {
        double v = (double)f->hull[k].h - slope * (double)f->hull[k].i;
        if (k == 0u || v < best_v) {
            best_v = v;
            best = k;
        }
    }
    f->anchor_i = f->hull[best].i;
    f->anchor_h = f->hull[best].h;
    if (out_vertex != NULL) {
        *out_vertex = best;
    }
}

/*
 * ⚠ THE GUARD AGAINST §10's SILENT DEGENERATION.
 *
 * When the slope is not fitted but pinned — the shape of hard-coding 800 Hz —
 * `h − s·i` drifts monotonically instead of hovering, so the lower-envelope
 * minimum always lands on one END of the session.  The estimator collapses onto
 * a single worst-case point while still reporting small residuals, which is
 * what makes the failure invisible.
 *
 * The observable symptom is that the pinned rate disagrees with the empirical
 * slope across the whole baseline.  799.2 against a round 800 is ~1,000 ppm;
 * the threshold below is a fifth of that, well outside the ~0.1 % per-device
 * spread of §6.4 that is legitimate.
 */
#define HM_FIT_PINNED_RATE_TOLERANCE_PPM 200.0

static void check_pinned_slope(hm_fit *f, double slope)
{
    double di;
    double empirical;
    double err_ppm;

    if (f->n < 64 || f->hull_n < 2u) {
        return;
    }
    di = (double)(f->last_i - f->first_i);
    if (!(di > 0.0)) {
        return;
    }
    empirical = (double)(f->last_host_us - f->origin_host_us) / di;
    err_ppm = ((empirical - slope) / slope) * 1e6;
    if (err_ppm < 0.0) {
        err_ppm = -err_ppm;
    }
    if (err_ppm > HM_FIT_PINNED_RATE_TOLERANCE_PPM) {
        f->flags |= (uint32_t)HM_CLOCK_DEGENERATE;
    }
}

void hm_fit_force_slope(hm_fit *f, double slope_us_per_index)
{
    if (f == NULL) {
        return;
    }
    f->slope_forced = (slope_us_per_index > 0.0);
    f->forced_slope = slope_us_per_index;
    f->dirty = true;
}

/*
 * ⚠ HM_CLOCK_RATE_POOLED SAYS THE PUBLISHED SLOPE *IS* THE CARRIED-OVER ONE, so
 * it is raised only on the paths that actually publish the seed
 * (implementation-review I11).
 *
 * It used to be set whenever `pooled_weight > 0` — including on the joint-fit
 * path below, where the pooled value is only a starting point for the hull scan
 * and the slope that comes out is fitted from this stretch's own observations.
 * A consumer was told its rate came from somewhere else when it had just been
 * measured here.
 *
 * ⚠ And design §6.1.1 reuses hm_fit_begin_stream() as the per-PULL re-anchor, so
 * `pooled_weight` becomes non-zero after the first retrieval of any connection —
 * which made the flag permanent inside a single, never-restarted stream.  That
 * part is honest under this rule: after a pull the next stretch genuinely does
 * start on a carried rate, and it clears as soon as that stretch fits its own.
 */
static void flag_seeded_rate(hm_fit *f)
{
    if (f->pooled_weight > 0.0) {
        f->flags |= (uint32_t)HM_CLOCK_RATE_POOLED;
    }
}

static void refit(hm_fit *f)
{
    double span_us;
    double seed = (f->pooled_weight > 0.0) ? f->pooled_slope : nominal_slope();

    f->dirty = false;
    f->flags &= (uint32_t)HM_CLOCK_BLIND; /* everything else is recomputed */

    if (f->n == 0 || f->hull_n == 0u) {
        f->have_fit = false;
        return;
    }

    if (f->slope_forced) {
        f->slope = f->forced_slope;
        anchor_at_slope(f, f->slope, NULL);
        f->have_fit = true;
        f->flags |= (uint32_t)HM_CLOCK_HAS_FIT;
        check_pinned_slope(f, f->slope);
        residual_stats(f);
        return;
    }
    span_us = (double)(f->last_host_us - f->origin_host_us);

    if (f->n < 2 || span_us < (double)HM_FIT_MIN_SPAN_US) {
        /*
         * Too short a baseline to separate rate from offset: a few milliseconds
         * of jitter over a second of data produces a wild slope.  Use the
         * seeded rate — pooled if this connection has already measured one,
         * otherwise nominal — and fit only the offset.
         */
        f->slope = seed;
        anchor_at_slope(f, f->slope, NULL);
        f->have_fit = true;
        f->flags |= (uint32_t)(HM_CLOCK_HAS_FIT | HM_CLOCK_SHORT_BASELINE);
        flag_seeded_rate(f);
        residual_stats(f);
        return;
    }

    /*
     * The joint fit.  cost(s) is linear in s given the running sums, and the
     * optimum lies on a lower-hull edge, so one scan settles it exactly.
     */
    {
        double best_cost = 0.0;
        double best_slope = seed;
        size_t best_edge = 0;
        bool have_best = false;

        for (size_t k = 0; k + 1u < f->hull_n; ++k) {
            double di = (double)(f->hull[k + 1u].i - f->hull[k].i);
            double s;
            double cost;
            if (di <= 0.0) {
                continue;
            }
            s = (double)(f->hull[k + 1u].h - f->hull[k].h) / di;
            cost = ((double)f->sum_h - (double)f->n * (double)f->hull[k].h) -
                   s * ((double)f->sum_i - (double)f->n * (double)f->hull[k].i);
            if (!have_best || cost < best_cost) {
                have_best = true;
                best_cost = cost;
                best_slope = s;
                best_edge = k;
            }
        }

        if (!have_best) {
            f->slope = seed;
            anchor_at_slope(f, f->slope, NULL);
            f->flags |= (uint32_t)HM_CLOCK_SHORT_BASELINE;
            flag_seeded_rate(f);
        } else {
            double rate = 1e6 / best_slope;
            if (!(rate > HM_RATE_PLAUSIBLE_MIN_HZ && rate < HM_RATE_PLAUSIBLE_MAX_HZ)) {
                /* Outside anything this crystal could do.  Keep the seeded rate
                 * and say so — an implausible rate is a symptom, not a value to
                 * publish. */
                f->flags |= (uint32_t)HM_CLOCK_RATE_IMPLAUSIBLE;
                f->slope = seed;
                anchor_at_slope(f, f->slope, NULL);
                check_pinned_slope(f, f->slope);
                flag_seeded_rate(f);
            } else {
                double total_span_i = (double)(f->last_i - f->first_i);
                double edge_span_i =
                    (double)(f->hull[best_edge + 1u].i - f->hull[best_edge].i);
                f->slope = best_slope;
                f->anchor_i = f->hull[best_edge].i;
                f->anchor_h = f->hull[best_edge].h;

                /*
                 * ⚠ Degeneracy: if the whole fit rests on two nearly adjacent
                 * frames, it is not a fit over thousands of observations — it
                 * is two points that happened to be early, and it will report
                 * small residuals while being arbitrarily wrong.
                 */
                if (f->n >= 64 && total_span_i > 0.0 &&
                    edge_span_i < 0.02 * total_span_i) {
                    f->flags |= (uint32_t)HM_CLOCK_DEGENERATE;
                }
            }
        }
        f->have_fit = true;
        f->flags |= (uint32_t)HM_CLOCK_HAS_FIT;
        residual_stats(f);
    }
}

void hm_fit_snapshot(hm_fit *f, hm_time_us now_us, hm_clock_snapshot *out)
{
    double corrected_slope;

    memset(out, 0, sizeof(*out));
    if (f == NULL) {
        return;
    }
    if (f->dirty) {
        refit(f);
    }

    out->stream_id = f->stream_id;
    out->accuracy_drift_us_per_s = f->accuracy_drift_us_per_s;
    out->observations = (f->n > INT32_MAX) ? INT32_MAX : (int32_t)f->n;
    out->external_ppm = f->external_ppm;
    memcpy(out->provenance, f->provenance, sizeof(out->provenance));

    if (!f->have_fit) {
        out->flags = f->flags & ~(uint32_t)HM_CLOCK_HAS_FIT;
        out->raw_fitted_rate_hz = 0.0;
        out->fitted_rate_hz = 0.0;
        return;
    }

    /* rate_effective = rate_fitted × (1 + ppm/1e6)  ⇒  slope divides. */
    corrected_slope = f->slope / (1.0 + f->external_ppm * 1e-6);

    out->flags = f->flags;
    if (f->external_ppm != 0.0) {
        out->flags |= (uint32_t)HM_CLOCK_EXTERNAL_CORRECTION;
    }
    if (now_us != HM_TIME_UNKNOWN && f->last_host_us != 0 &&
        (now_us - f->last_host_us) > HM_FIT_STALE_AFTER_US) {
        out->flags |= (uint32_t)HM_CLOCK_STALE;
    }

    out->anchor_index = (uint32_t)((int64_t)f->origin_index + f->anchor_i);
    out->anchor_host_us = f->origin_host_us + f->anchor_h;
    out->slope_us_per_index = corrected_slope;
    out->raw_fitted_rate_hz = 1e6 / f->slope;
    out->fitted_rate_hz = 1e6 / corrected_slope;

    out->first_index = (uint32_t)((int64_t)f->origin_index + f->first_i);
    out->last_index = (uint32_t)((int64_t)f->origin_index + f->last_i);
    out->span_us = f->last_host_us - f->origin_host_us;
    out->last_observation_us = f->last_host_us;
    out->offset_us =
        out->anchor_host_us - (hm_time_us)llround((double)out->anchor_index * corrected_slope);

    out->residual_median_us = f->residual_median_us;
    out->residual_p90_us = f->residual_p90_us;
    out->residual_max_us = f->residual_max_us;
}
