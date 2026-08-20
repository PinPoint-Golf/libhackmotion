/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_unwrap.h — internal.  Turning the device's two wrapping counters into
 * monotonic device time, with no host clock involved.
 *
 * ⚠ THERE IS ONE CORRECT ALGORITHM HERE AND IT IS ENCODED ONCE (api-request
 * §2.1).  The frame carries two clocks that solve each other's problem
 * (spec §10.2):
 *
 *              resolution   wraps every   weakness
 *   index      1.251 ms     82.0 s        coarse
 *   ticks      15.6 µs      1.023 s       wrap ambiguous across any gap > 1.02 s
 *
 * Since ticks ≈ index × 80.166, an unwrapped index PREDICTS the tick value and
 * the wrap count falls out.  Verified over a 238 s session — a ratio fitted on
 * the first 5 s predicted the tick counter across 2.9 index wraps and ~233 tick
 * wraps with a worst-case error of 2,745 ticks against the ±32,768 needed to
 * pick the wrong one: 8.4 % of budget, zero failures on either unit.
 *
 * So device time is fully self-describing from the frame alone, and IT WORKS
 * IDENTICALLY FOR HISTORY, where arrival times carry nothing at all.
 */
#ifndef WR_UNWRAP_H
#define WR_UNWRAP_H

#include "wrist/types.h"
#include "wrist/clock.h"

/* ------------------------------------------------------------------------ */
/* Sample index                                                              */
/* ------------------------------------------------------------------------ */
typedef struct wr_index_unwrapper {
    bool     have;
    uint16_t last_raw;
    uint32_t unwrapped;
    uint32_t regressions;   /* implausibly large forward steps — see below */
} wr_index_unwrapper;

void wr_index_unwrapper_reset(wr_index_unwrapper *u);

/*
 * The counter only ever increases within a stream (§6.5), so the unsigned
 * difference is the true step and wraps take care of themselves — correct for
 * any gap shorter than the full 82.0 s wrap period.
 *
 * A step above WR_INDEX_REGRESSION_LIMIT is almost certainly a reordered or
 * corrupt frame rather than a 60-second gap; it is counted and reported, and
 * the sample is still delivered so a consumer can see it.
 */
#define WR_INDEX_REGRESSION_LIMIT 49152u

uint32_t wr_index_unwrap(wr_index_unwrapper *u, uint16_t raw, bool *out_suspect);

/* ------------------------------------------------------------------------ */
/* Tick counter                                                              */
/* ------------------------------------------------------------------------ */
/* The ratio is refitted once the baseline is at least this many indices — 5 s
 * at the internal rate, which §10.2 verified is enough for a whole session. */
#define WR_TICK_RATIO_FIT_INDICES 4000u

typedef struct wr_tick_unwrapper {
    bool     have_anchor;
    uint32_t anchor_index;
    uint16_t anchor_raw;

    double   ratio;          /* ticks per index; seeded from the nominal 80.166 */
    bool     ratio_fitted;

    int64_t  last_unwrapped; /* ticks since the anchor */
    uint32_t last_index;

    /* Endpoints of the current ratio fit. */
    uint32_t fit_first_index;
    int64_t  fit_first_ticks;
    uint32_t fit_last_index;
    int64_t  fit_last_ticks;
    /* ⚠ The baseline the ratio was LAST fitted over, which is what makes "refit
     * on every doubling" a gate rather than a description.  Without it the test
     * was `span >= span/2` — true for every value of span — so the ratio refitted
     * on every record once fitted (implementation-review I10). */
    uint32_t fit_span_at_refit;

    /* The smallest wrap-decision margin seen, in ticks.  ±32,768 is the budget;
     * the session warns if this ever falls below a fifth of it. */
    uint32_t worst_margin;
} wr_tick_unwrapper;

void wr_tick_unwrapper_reset(wr_tick_unwrapper *u);

/*
 * Returns ticks since the stream's first observed sample for this unit.
 * `out_margin`, if non-NULL, receives how far the decision was from ambiguous
 * (32768 = perfect, 0 = a coin toss).
 */
int64_t wr_tick_unwrap(wr_tick_unwrapper *u, uint32_t index, uint16_t raw,
                       uint32_t *out_margin);

/* Ticks per second implied by the current ratio and a sample rate. */
double wr_tick_rate_hz(const wr_tick_unwrapper *u, double sample_rate_hz);

/* Convert an unwrapped tick count to microseconds. */
wr_time_us wr_ticks_to_us(int64_t ticks, double tick_rate_hz);

/*
 * ⚠ palm − lower_arm, in ticks, resolved without either unwrapper.
 *
 * The two counters are independent free-running MCU timers whose difference is
 * a small stable number — 59 ticks (0.92 ms), identical median across the first
 * and second halves of a 238 s session (§10.3).  Interpreting the raw
 * difference as signed modulo 65536 recovers it exactly, and does so for the
 * very first record of a stream, before any anchor exists.
 */
int32_t wr_tick_skew(uint16_t palm_raw, uint16_t lower_arm_raw);

#endif /* WR_UNWRAP_H */
