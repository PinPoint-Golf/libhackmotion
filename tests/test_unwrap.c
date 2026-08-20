/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_unwrap.c — the counters, and §10.2's claim that device time is fully
 * self-describing from the frame alone.
 */
#include "wr_test.h"
#include "wr_unwrap.h"

/* §6.5 — the sample counter wraps every 82.0 s and only ever increases. */
WR_TEST(unwrap_index_crosses_the_82_second_wrap)
{
    wr_index_unwrapper u;
    bool suspect = false;
    wr_index_unwrapper_reset(&u);

    WR_ASSERT_EQ(wr_index_unwrap(&u, 65500, &suspect), 65500);
    WR_ASSERT(!suspect);
    WR_ASSERT_EQ(wr_index_unwrap(&u, 65532, &suspect), 65532);   /* +32, the 25 Hz step */
    WR_ASSERT_EQ(wr_index_unwrap(&u, 28, &suspect), 65564);      /* wrapped */
    WR_ASSERT(!suspect);
    WR_ASSERT_EQ(wr_index_unwrap(&u, 60, &suspect), 65596);
}

/* §6.6 — the live rate is a continuum, not a two-state switch: steps of 32, 8
 * and anything from 1 upward all appear, in every session containing motion. */
WR_TEST(unwrap_index_handles_the_whole_step_distribution)
{
    static const uint16_t steps[] = {32, 32, 8, 8, 1, 1, 1, 1, 1, 1, 1, 1, 12, 27, 32, 4};
    wr_index_unwrapper u;
    uint32_t raw = 65000;
    uint32_t expected = 65000;
    wr_index_unwrapper_reset(&u);
    WR_ASSERT_EQ(wr_index_unwrap(&u, (uint16_t)raw, NULL), expected);

    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i) {
        raw = (raw + steps[i]) & 0xffffu;
        expected += steps[i];
        WR_ASSERT_EQ(wr_index_unwrap(&u, (uint16_t)raw, NULL), expected);
    }
}

WR_TEST(unwrap_index_flags_an_implausible_backward_jump)
{
    wr_index_unwrapper u;
    bool suspect = false;
    wr_index_unwrapper_reset(&u);
    (void)wr_index_unwrap(&u, 1000, &suspect);
    (void)wr_index_unwrap(&u, 500, &suspect); /* looks like +65036 */
    WR_ASSERT(suspect);
    WR_ASSERT_EQ(u.regressions, 1);
}

/*
 * ⚠ §10.2, the headline claim: a ratio fitted on the first 5 s predicts the
 * tick counter across a 238 s session — 2.9 index wraps, ~233 tick wraps — with
 * a worst-case error of 2,745 ticks against the ±32,768 needed to pick the
 * wrong wrap.  This reproduces that at the same scale.
 */
WR_TEST(unwrap_ticks_resolve_across_a_238_second_session)
{
    const double ratio = 80.166;      /* §6.5, measured over 6,063 steps */
    const uint32_t total_indices = 190000u; /* 238 s at 799.2 Hz */
    wr_tick_unwrapper u;
    uint32_t index = 0;
    uint32_t worst = 32768u;
    int steps = 0;

    wr_tick_unwrapper_reset(&u);

    while (index < total_indices) {
        int64_t true_ticks = (int64_t)((double)index * ratio);
        uint16_t raw = (uint16_t)(true_ticks & 0xffff);
        uint32_t margin = 0;
        int64_t got = wr_tick_unwrap(&u, index, raw, &margin);

        /* The unwrapped value must equal the truth, not merely be close. */
        WR_ASSERT_EQ(got, true_ticks);
        if (margin < worst) {
            worst = margin;
        }
        steps++;
        index += (steps % 7 == 0) ? 8u : 32u; /* the two dominant live steps */
    }

    WR_ASSERT(steps > 6000);
    /* §10.2 reports 8.4 % of budget consumed.  Anything under half is safe. */
    WR_ASSERT_MSG(worst > 16384u, "wrap decision margin fell below half of budget");
    WR_ASSERT(u.ratio_fitted);
    WR_ASSERT_NEAR(u.ratio, ratio, 0.01);
}

/*
 * ⚠⚠ "REFIT ON EVERY DOUBLING" HAS TO BE A GATE, NOT A DESCRIPTION.
 *
 * implementation-review I10.  `needed` was derived from the same two endpoints
 * as `span`, so the test read `span >= span/2` — true for every value of span.
 * Once fitted, the ratio refitted on every record: a division per record per
 * unit through a full-rate replay, under a comment promising it "stops costing
 * anything".  A gate that cannot fail is not a gate, and this asserts it can.
 */
WR_TEST(unwrap_ticks_refit_the_ratio_on_a_doubling_and_not_on_every_record)
{
    const double   ratio = 80.166;
    wr_tick_unwrapper u;
    uint32_t       index = 0;
    double         after_first;
    uint32_t       span_at_first;

    wr_tick_unwrapper_reset(&u);

    /* Just past the first fit's baseline. */
    while (index <= WR_TICK_RATIO_FIT_INDICES) {
        int64_t true_ticks = (int64_t)((double)index * ratio);
        (void)wr_tick_unwrap(&u, index, (uint16_t)(true_ticks & 0xffff), NULL);
        index += 8u;
    }
    WR_ASSERT(u.ratio_fitted);
    after_first = u.ratio;
    span_at_first = u.fit_span_at_refit;
    WR_ASSERT(span_at_first >= WR_TICK_RATIO_FIT_INDICES);

    /*
     * ⚠ Now feed a stretch at a DIFFERENT ratio, still short of a doubling.  If
     * the gate is real the published ratio cannot move; if it is a tautology
     * every one of these records refits and it moves immediately.
     */
    while (index < span_at_first * 2u - 8u) {
        int64_t true_ticks = (int64_t)((double)index * (ratio * 1.02));
        (void)wr_tick_unwrap(&u, index, (uint16_t)(true_ticks & 0xffff), NULL);
        index += 8u;
        WR_ASSERT_MSG(u.ratio == after_first,
                      "the ratio may not be refitted before the baseline doubles");
        WR_ASSERT_EQ(u.fit_span_at_refit, span_at_first);
    }

    /* And past the doubling it does move, so the gate is a delay and not a stop. */
    while (u.fit_span_at_refit == span_at_first && index < span_at_first * 4u) {
        int64_t true_ticks = (int64_t)((double)index * (ratio * 1.02));
        (void)wr_tick_unwrap(&u, index, (uint16_t)(true_ticks & 0xffff), NULL);
        index += 8u;
    }
    WR_ASSERT_MSG(u.fit_span_at_refit >= span_at_first * 2u,
                  "and it must refit once the baseline HAS doubled");
}

/*
 * ⚠ §10.2 again, but the case that actually matters: a BLE gap longer than the
 * tick counter's 1.023 s wrap period.  A client that tried to resolve the wrap
 * from arrival times would be stuck here; resolving from the index is not.
 */
WR_TEST(unwrap_ticks_survive_a_gap_longer_than_the_tick_wrap)
{
    const double ratio = 80.166;
    wr_tick_unwrapper u;
    wr_tick_unwrapper_reset(&u);

    for (uint32_t i = 0; i < 8000u; i += 32u) {
        int64_t t = (int64_t)((double)i * ratio);
        (void)wr_tick_unwrap(&u, i, (uint16_t)(t & 0xffff), NULL);
    }
    {
        /* A 5-second hole: nearly five full tick wraps. */
        uint32_t after = 8000u + 4000u;
        int64_t truth = (int64_t)((double)after * ratio);
        uint32_t margin = 0;
        int64_t got = wr_tick_unwrap(&u, after, (uint16_t)(truth & 0xffff), &margin);
        WR_ASSERT_EQ(got, truth);
        WR_ASSERT(margin > 16384u);
    }
}

/*
 * §10.3 — the two blocks' counters differ by a stable 59 ticks (0.92 ms).
 * Recovering it must work on the very first record, before any anchor exists,
 * and must survive either counter wrapping independently.
 */
WR_TEST(unwrap_inter_unit_skew_is_recovered_modulo_the_wrap)
{
    WR_ASSERT_EQ(wr_tick_skew(4719, 4660), 59);
    WR_ASSERT_EQ(wr_tick_skew(4660, 4719), -59);
    /* palm wrapped, arm did not */
    WR_ASSERT_EQ(wr_tick_skew(30, 65507), 59);
    /* arm wrapped, palm did not */
    WR_ASSERT_EQ(wr_tick_skew(65507, 30), -59);
    WR_ASSERT_EQ(wr_tick_skew(0, 0), 0);
}

/* §6.4 — ≈64,068 ticks/s follows from 799.19 Hz × 80.166 ticks/sample. */
WR_TEST(unwrap_tick_rate_matches_section_6_4)
{
    wr_tick_unwrapper u;
    wr_tick_unwrapper_reset(&u);
    WR_ASSERT_NEAR(wr_tick_rate_hz(&u, 799.19), 64068.0, 5.0);

    /* 59 ticks at that rate is the 0.92 ms of §10.3. */
    WR_ASSERT_NEAR(wr_ticks_to_us(59, 64068.0), 921.0, 2.0);
}

WR_TEST(unwrap_ticks_to_us_is_defensive)
{
    WR_ASSERT_EQ(wr_ticks_to_us(1000, 0.0), WR_TIME_UNKNOWN);
    WR_ASSERT_EQ(wr_ticks_to_us(1000, -1.0), WR_TIME_UNKNOWN);
}

WR_TEST_MAIN()
