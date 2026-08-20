/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "hm_unwrap.h"

#include <string.h>
#include <math.h>

void hm_index_unwrapper_reset(hm_index_unwrapper *u)
{
    memset(u, 0, sizeof(*u));
}

uint32_t hm_index_unwrap(hm_index_unwrapper *u, uint16_t raw, bool *out_suspect)
{
    uint16_t step;

    if (out_suspect != NULL) {
        *out_suspect = false;
    }
    if (!u->have) {
        u->have = true;
        u->last_raw = raw;
        u->unwrapped = raw;
        return u->unwrapped;
    }

    step = (uint16_t)(raw - u->last_raw);
    if (step >= HM_INDEX_REGRESSION_LIMIT) {
        u->regressions++;
        if (out_suspect != NULL) {
            *out_suspect = true;
        }
    }
    u->unwrapped += step;
    u->last_raw = raw;
    return u->unwrapped;
}

/* ------------------------------------------------------------------------ */

void hm_tick_unwrapper_reset(hm_tick_unwrapper *u)
{
    memset(u, 0, sizeof(*u));
    u->ratio = HM_NOMINAL_TICKS_PER_SAMPLE;
    u->worst_margin = (uint32_t)(HM_TICK_MODULUS / 2u);
}

int64_t hm_tick_unwrap(hm_tick_unwrapper *u, uint32_t index, uint16_t raw, uint32_t *out_margin)
{
    int64_t predicted;
    int64_t residue;
    int64_t wraps;
    int64_t unwrapped;
    int64_t error;
    uint32_t margin;

    if (!u->have_anchor) {
        u->have_anchor = true;
        u->anchor_index = index;
        u->anchor_raw = raw;
        u->last_unwrapped = 0;
        u->last_index = index;
        u->fit_first_index = index;
        u->fit_first_ticks = 0;
        u->fit_last_index = index;
        u->fit_last_ticks = 0;
        if (out_margin != NULL) {
            *out_margin = (uint32_t)(HM_TICK_MODULUS / 2u);
        }
        return 0;
    }

    /*
     * Predict from the index, then pick the wrap count that lands nearest.  The
     * ratio starts at the nominal value, which §10.2 shows is good for 8.4 % of
     * the wrap budget across four minutes — ample to bootstrap the refit below.
     */
    predicted = (int64_t)llround(((double)index - (double)u->anchor_index) * u->ratio);
    residue = (int64_t)(uint16_t)(raw - u->anchor_raw);
    wraps = (int64_t)llround(((double)(predicted - residue)) / (double)HM_TICK_MODULUS);
    unwrapped = residue + wraps * (int64_t)HM_TICK_MODULUS;

    error = unwrapped - predicted;
    if (error < 0) {
        error = -error;
    }
    margin = (error >= (int64_t)(HM_TICK_MODULUS / 2u))
                 ? 0u
                 : (uint32_t)((int64_t)(HM_TICK_MODULUS / 2u) - error);
    if (margin < u->worst_margin) {
        u->worst_margin = margin;
    }
    if (out_margin != NULL) {
        *out_margin = margin;
    }

    u->last_unwrapped = unwrapped;
    u->last_index = index;

    /*
     * Extend the fit window, then refit once the baseline is long enough.  The
     * ratio is refitted on every DOUBLING of the baseline, which converges fast
     * and then stops costing anything.
     *
     * ⚠ THE GATE COMPARES AGAINST THE SPAN AT THE LAST REFIT, and it has to
     * (implementation-review I10).  Deriving `needed` from the same two
     * endpoints as `span` made the test `span >= span/2`, which is true for
     * every value of span — so the comment above described a doubling while the
     * code refitted on every record, at a division per record per unit through a
     * full-rate replay.  A gate that cannot fail is not a gate.
     */
    if (index > u->fit_last_index) {
        u->fit_last_index = index;
        u->fit_last_ticks = unwrapped;

        {
            uint32_t span = u->fit_last_index - u->fit_first_index;
            uint64_t needed = u->ratio_fitted ? (uint64_t)u->fit_span_at_refit * 2u
                                              : (uint64_t)HM_TICK_RATIO_FIT_INDICES;
            if (span >= HM_TICK_RATIO_FIT_INDICES && (uint64_t)span >= needed) {
                double dt = (double)(u->fit_last_ticks - u->fit_first_ticks);
                double di = (double)span;
                double r = dt / di;
                /* Reject anything the crystal could not plausibly produce; the
                 * 0.1 % spread of §6.4 is three orders inside this band. */
                if (r > 40.0 && r < 160.0) {
                    u->ratio = r;
                    u->ratio_fitted = true;
                    u->fit_span_at_refit = span;
                }
            }
        }
    }
    return unwrapped;
}

double hm_tick_rate_hz(const hm_tick_unwrapper *u, double sample_rate_hz)
{
    return u->ratio * sample_rate_hz;
}

hm_time_us hm_ticks_to_us(int64_t ticks, double tick_rate_hz)
{
    if (!(tick_rate_hz > 0.0)) {
        return HM_TIME_UNKNOWN;
    }
    return (hm_time_us)llround(((double)ticks / tick_rate_hz) * 1e6);
}

int32_t hm_tick_skew(uint16_t palm_raw, uint16_t lower_arm_raw)
{
    uint16_t d = (uint16_t)(palm_raw - lower_arm_raw);
    if (d >= (uint16_t)(HM_TICK_MODULUS / 2u)) {
        return (int32_t)d - (int32_t)HM_TICK_MODULUS;
    }
    return (int32_t)d;
}
