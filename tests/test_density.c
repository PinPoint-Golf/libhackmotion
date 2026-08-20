/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_density.c — ACHIEVED DENSITY, api-request C4's other half.
 *
 * ⚠ THE TEST THIS REPLACES PASSED OVER A FIELD THAT COULD ONLY EVER RETURN 1.0.
 *
 * `coverage_density_separates_reach_from_completeness` built its input from a
 * coverage set plus a sample count chosen independently of it — a shape the
 * session cannot produce, because gather_record() increments both on the same
 * line.  So it agreed with the bug (implementation-review I1), while seven real
 * pulls at rates from 99.9 Hz to 728 Hz all reported density 1.000 and that was
 * published as a positive finding.
 *
 * These are built from ASCENDING SAMPLE ARRAYS — what a block actually carries —
 * and every case here is a shape the device has been measured producing.
 */
#include "wr_test.h"
#include "wrist/history.h"

#include <string.h>

#define MAX_SAMPLES 1024

static size_t fill_step(wr_sample *out, uint32_t first, uint32_t last, uint32_t step)
{
    size_t n = 0;
    for (uint32_t i = first; i <= last && n < MAX_SAMPLES; i += step) {
        memset(&out[n], 0, sizeof(out[n]));
        out[n].sample_index = i;
        n++;
    }
    return n;
}

/*
 * §7.3's two regimes, which are the two ends of this field's working range.
 * Across 25 retrievals and 17,739 measured steps none exceeded 8 and none was 0.
 */
WR_TEST(density_reads_step_one_as_the_full_internal_rate)
{
    wr_sample s[MAX_SAMPLES];
    size_t    n = fill_step(s, 1000u, 1399u, 1u);

    WR_ASSERT_EQ(n, 400);
    WR_ASSERT_NEAR(wr_sample_step_density(s, n), 1.0, 1e-12);
}

WR_TEST(density_reads_the_at_rest_floor_as_one_eighth)
{
    wr_sample s[MAX_SAMPLES];
    size_t    n = fill_step(s, 1000u, 1399u, 8u);

    WR_ASSERT_EQ(n, 50);
    /* ⚠ §7.3's own figure for step 8, and the shape a still wrist returns.  It
     * is the buffer working correctly, not a delivery failure. */
    WR_ASSERT_NEAR(wr_sample_step_density(s, n), 0.125, 1e-12);
}

/*
 * ⚠ THE DISTINCTION C4 IS CRITICAL FOR, on two shapes the session really
 * produces: a SHORT reply (contiguous, narrower than asked) and a HOLED one at
 * the motion-adaptive floor.  Both cover an eighth of the same window, so
 * `coverage_fraction` cannot tell them apart — and at 12.5 % coverage that
 * distinction decides whether a metric computed at impact exists at all.
 */
WR_TEST(density_separates_a_short_reply_from_one_at_the_hundred_hertz_floor)
{
    wr_sample short_reply[MAX_SAMPLES];
    wr_sample at_rest[MAX_SAMPLES];
    size_t    n_short = fill_step(short_reply, 1000u, 1049u, 1u); /* dense over 1/8 */
    size_t    n_rest = fill_step(at_rest, 1000u, 1399u, 8u);      /* 1/8-dense over all */

    /* Identical sample counts over the identical 400-index window. */
    WR_ASSERT_EQ(n_short, 50);
    WR_ASSERT_EQ(n_rest, 50);

    WR_ASSERT_NEAR(wr_sample_step_density(short_reply, n_short), 1.0, 1e-12);
    WR_ASSERT_NEAR(wr_sample_step_density(at_rest, n_rest), 0.125, 1e-12);
}

/*
 * ⚠ AND WHY IT IS A SPACING RATHER THAN AN AVERAGE RATE.  §7.5's measured 58 %
 * reply is two dense runs with one 168-index hole between them.  Averaged over
 * its span that reads 0.58 — the same number a uniformly half-dense reply gives,
 * which is exactly the pair C4 exists to separate.  The hole is `largest_gap_us`
 * and `gaps[]`; the spacing is this.
 */
WR_TEST(density_is_the_spacing_and_not_the_average_rate)
{
    wr_sample two_runs[MAX_SAMPLES];
    wr_sample uniform[MAX_SAMPLES];
    size_t    a = fill_step(two_runs, 7000u, 7115u, 1u);
    size_t    b;

    a += fill_step(&two_runs[a], 7284u, 7399u, 1u);
    WR_ASSERT_EQ(a, 232);
    WR_ASSERT_NEAR(wr_sample_step_density(two_runs, a), 1.0, 1e-12);

    b = fill_step(uniform, 7000u, 7399u, 2u);
    WR_ASSERT_EQ(b, 200);
    WR_ASSERT_NEAR(wr_sample_step_density(uniform, b), 0.5, 1e-12);
}

/*
 * ⚠ The UPPER median: with an even number of gaps, the sparser of the two middle
 * values.  Over-claiming density invites a consumer to resample onto a uniform
 * 800 Hz grid (§7.6), which invents samples that were never taken.
 */
WR_TEST(density_takes_the_sparser_of_two_middle_gaps)
{
    wr_sample s[4];
    memset(s, 0, sizeof(s));
    s[0].sample_index = 100u;
    s[1].sample_index = 101u; /* gap 1 */
    s[2].sample_index = 109u; /* gap 8 */

    WR_ASSERT_NEAR(wr_sample_step_density(s, 3), 0.125, 1e-12);
}

/*
 * ⚠ 0.0 IS "NOT MEASURABLE" AND NEVER "EMPTY", and a field that returns a number
 * over evidence it does not have is what this whole finding was about.
 */
WR_TEST(density_refuses_rather_than_guessing)
{
    wr_sample s[MAX_SAMPLES];
    size_t    n;

    memset(s, 0, sizeof(s));
    s[0].sample_index = 100u;

    WR_ASSERT_NEAR(wr_sample_step_density(NULL, 10), 0.0, 1e-12);
    WR_ASSERT_NEAR(wr_sample_step_density(s, 0), 0.0, 1e-12);
    /* One sample cannot show a spacing. */
    WR_ASSERT_NEAR(wr_sample_step_density(s, 1), 0.0, 1e-12);

    /* Not strictly ascending in device index (api-request C3). */
    s[1].sample_index = 100u;
    WR_ASSERT_NEAR(wr_sample_step_density(s, 2), 0.0, 1e-12);
    s[1].sample_index = 99u;
    WR_ASSERT_NEAR(wr_sample_step_density(s, 2), 0.0, 1e-12);

    /* An index the frame never carried is not a spacing of zero. */
    s[1].sample_index = 101u;
    s[1].flags = (uint16_t)WR_SAMPLE_INDEX_MISSING;
    WR_ASSERT_NEAR(wr_sample_step_density(s, 2), 0.0, 1e-12);

    /* Wider than the widest spacing this can characterise: fewer than one
     * sample in WR_DENSITY_STEP_MAX indices. */
    n = fill_step(s, 0u, 100000u, WR_DENSITY_STEP_MAX + 1u);
    WR_ASSERT(n > 2);
    WR_ASSERT_NEAR(wr_sample_step_density(s, n), 0.0, 1e-12);

    /* ...and exactly at the limit it still answers. */
    n = fill_step(s, 0u, 100000u, WR_DENSITY_STEP_MAX);
    WR_ASSERT(n > 2);
    WR_ASSERT_NEAR(wr_sample_step_density(s, n), 1.0 / (double)WR_DENSITY_STEP_MAX, 1e-12);
}

/*
 * ⚠ One wide hole does not move the spacing, and that is the point: the swing
 * pulls in `swings.wrwire` read 1.000 with 34-43 gaps apiece because the
 * SPACING really was step 1.  A reader who takes that for the old pinned-to-1.0
 * bug should compare it with the still-wrist pull beside it, which reads 0.125.
 */
WR_TEST(density_is_unmoved_by_a_single_wide_hole_which_the_gap_list_carries)
{
    wr_sample s[MAX_SAMPLES];
    size_t    n = fill_step(s, 1000u, 1199u, 1u);

    n += fill_step(&s[n], 1600u, 1799u, 1u);
    WR_ASSERT_EQ(n, 400);
    WR_ASSERT_NEAR(wr_sample_step_density(s, n), 1.0, 1e-12);
}

WR_TEST_MAIN()
