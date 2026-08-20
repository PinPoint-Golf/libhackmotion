/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_coverage.c — the device's main failure mode, made detectable.
 *
 * ⚠ §7.1, §7.3: an over-wide history request comes back HOLED, spanning most of
 * the requested range at 33-58 % coverage, with no error and no indication.
 * api-request C4 is Critical because intervals-plus-density is the ONLY way a
 * consumer can tell that apart from a short pull.
 *
 * This file is the INTERVALS half — reach.  The density half is measured over
 * the delivered samples rather than over an interval set, and lives in
 * test_density.c; wr_coverage.c's own comment says why it cannot live here.
 */
#include "wr_test.h"
#include "wrist/coverage.h"

#define STORAGE 64

WR_TEST(coverage_merges_a_contiguous_ascending_delivery)
{
    wr_index_range storage[STORAGE];
    wr_coverage cov;
    wr_index_range window = {100, 199};

    wr_coverage_init(&cov, storage, STORAGE);
    for (uint32_t i = 100; i <= 199; ++i) {
        WR_ASSERT_EQ(wr_coverage_add(&cov, i), WR_OK);
    }
    WR_ASSERT_EQ(cov.count, 1);
    WR_ASSERT_EQ(cov.ranges[0].first, 100);
    WR_ASSERT_EQ(cov.ranges[0].last, 199);
    WR_ASSERT_EQ(cov.indices, 100);
    WR_ASSERT_NEAR(wr_coverage_fraction(&cov, window), 1.0, 1e-12);
    WR_ASSERT_EQ(wr_coverage_largest_gap(&cov, window), 0);
    WR_ASSERT(!cov.overflowed);
}

/*
 * ⚠ THE DISTINCTION C4 EXISTS FOR.  A count cannot tell "dense over half the
 * range" from "half-dense over all of it", and at 33-58 % coverage that decides
 * whether a metric computed at impact exists.
 */
WR_TEST(coverage_distinguishes_dense_over_half_from_half_dense_over_all)
{
    wr_index_range storage_a[STORAGE], storage_b[512];
    wr_coverage dense_half, half_dense_all;
    wr_index_range window = {0, 999};

    wr_coverage_init(&dense_half, storage_a, STORAGE);
    wr_coverage_init(&half_dense_all, storage_b, 512);

    for (uint32_t i = 0; i < 500u; ++i) {
        (void)wr_coverage_add(&dense_half, i);
    }
    for (uint32_t i = 0; i < 1000u; i += 2u) {
        (void)wr_coverage_add(&half_dense_all, i);
    }

    /* Identical sample counts. */
    WR_ASSERT_EQ(dense_half.indices, 500);
    WR_ASSERT_EQ(half_dense_all.indices, 500);

    /* Completely different swings. */
    WR_ASSERT_NEAR(wr_coverage_fraction(&dense_half, window), 0.5, 1e-12);
    WR_ASSERT_NEAR(wr_coverage_fraction(&half_dense_all, window), 0.5, 1e-12);
    WR_ASSERT_EQ(wr_coverage_largest_gap(&dense_half, window), 500);
    WR_ASSERT_EQ(wr_coverage_largest_gap(&half_dense_all, window), 1);
    WR_ASSERT_EQ(dense_half.count, 1);
    WR_ASSERT_EQ(half_dense_all.count, 500);
}

/* §7.5's measured numbers: 58 % mid-stream, 33 % stopped, both holed. */
WR_TEST(coverage_reports_the_measured_holed_deliveries)
{
    wr_index_range storage[512];
    wr_coverage cov;
    wr_index_range window = {0, 7999};

    wr_coverage_init(&cov, storage, 512);
    /* 58 % coverage spread across the whole span, as a mid-stream pull gave:
     * 80 runs of 58 consecutive indices with a 42-index hole after each. */
    for (uint32_t i = 0; i < 8000u; ++i) {
        if (i % 100u < 58u) {
            (void)wr_coverage_add(&cov, i);
        }
    }
    WR_ASSERT_NEAR(wr_coverage_fraction(&cov, window), 0.58, 1e-9);
    WR_ASSERT_EQ(cov.count, 80);
    WR_ASSERT_EQ(wr_coverage_largest_gap(&cov, window), 42);
    WR_ASSERT(!cov.overflowed);
    WR_ASSERT_MSG(wr_coverage_largest_gap(&cov, window) > 0,
                  "a holed delivery must not look complete");
}

WR_TEST(coverage_enumerates_gaps_within_a_window)
{
    wr_index_range storage[STORAGE];
    wr_index_range gaps[8];
    wr_coverage cov;
    wr_index_range window = {0, 99};
    size_t n;

    wr_coverage_init(&cov, storage, STORAGE);
    {
        wr_index_range a = {10, 19};
        wr_index_range b = {50, 59};
        (void)wr_coverage_add_range(&cov, a);
        (void)wr_coverage_add_range(&cov, b);
    }

    n = wr_coverage_gaps(&cov, window, gaps, 8);
    WR_ASSERT_EQ(n, 3);
    WR_ASSERT_EQ(gaps[0].first, 0);
    WR_ASSERT_EQ(gaps[0].last, 9);
    WR_ASSERT_EQ(gaps[1].first, 20);
    WR_ASSERT_EQ(gaps[1].last, 49);
    WR_ASSERT_EQ(gaps[2].first, 60);
    WR_ASSERT_EQ(gaps[2].last, 99);
    WR_ASSERT_EQ(wr_coverage_largest_gap(&cov, window), 40);
}

WR_TEST(coverage_absorbs_out_of_order_and_overlapping_insertions)
{
    wr_index_range storage[STORAGE];
    wr_coverage cov;

    wr_coverage_init(&cov, storage, STORAGE);
    {
        wr_index_range a = {50, 59};
        wr_index_range b = {10, 19};
        wr_index_range c = {20, 49}; /* bridges the two */
        wr_index_range d = {15, 25}; /* overlaps both, adds nothing */
        (void)wr_coverage_add_range(&cov, a);
        (void)wr_coverage_add_range(&cov, b);
        WR_ASSERT_EQ(cov.count, 2);
        (void)wr_coverage_add_range(&cov, c);
        WR_ASSERT_EQ(cov.count, 1);
        WR_ASSERT_EQ(cov.indices, 50);
        (void)wr_coverage_add_range(&cov, d);
        WR_ASSERT_EQ(cov.count, 1);
        WR_ASSERT_EQ(cov.indices, 50); /* ⚠ C3: no double counting */
    }
    WR_ASSERT(wr_coverage_contains(&cov, 10));
    WR_ASSERT(wr_coverage_contains(&cov, 59));
    WR_ASSERT(!wr_coverage_contains(&cov, 9));
    WR_ASSERT(!wr_coverage_contains(&cov, 60));
}

/* Adjacent ranges merge; a one-index hole does not. */
WR_TEST(coverage_merges_adjacent_but_not_separated_ranges)
{
    wr_index_range storage[STORAGE];
    wr_coverage cov;
    wr_index_range a = {10, 19};
    wr_index_range adjacent = {20, 29};
    wr_index_range gapped = {31, 40};

    wr_coverage_init(&cov, storage, STORAGE);
    (void)wr_coverage_add_range(&cov, a);
    (void)wr_coverage_add_range(&cov, adjacent);
    WR_ASSERT_EQ(cov.count, 1);
    (void)wr_coverage_add_range(&cov, gapped);
    WR_ASSERT_EQ(cov.count, 2);
    WR_ASSERT_EQ(cov.indices, 30);
}

/*
 * On overflow the set stays a valid SUPERSET and the index total stays exact,
 * but the gap list becomes optimistic — so `overflowed` has to reach the
 * consumer rather than being swallowed.
 */
WR_TEST(coverage_overflow_is_reported_not_silent)
{
    wr_index_range storage[4];
    wr_coverage cov;
    wr_coverage_init(&cov, storage, 4);

    for (uint32_t i = 0; i < 40u; i += 2u) {
        (void)wr_coverage_add(&cov, i);
    }
    WR_ASSERT(cov.overflowed);
    WR_ASSERT(cov.dropped > 0);
    WR_ASSERT(cov.count <= 4);
    WR_ASSERT_EQ(cov.bounds.first, 0);
    WR_ASSERT_EQ(cov.bounds.last, 38);
    /* The union is a superset, so the reported coverage is never an underclaim
     * of what arrived — the optimism is in the gaps, which is why it is flagged. */
    WR_ASSERT(wr_coverage_indices_in(&cov, cov.bounds) >= 20);
}

WR_TEST(coverage_rejects_inverted_ranges)
{
    wr_index_range storage[STORAGE];
    wr_coverage cov;
    wr_index_range bad = {100, 50};
    wr_index_range window = {100, 50};

    wr_coverage_init(&cov, storage, STORAGE);
    WR_ASSERT_EQ(wr_coverage_add_range(&cov, bad), WR_ERR_INVALID_ARG);
    WR_ASSERT_NEAR(wr_coverage_fraction(&cov, window), 0.0, 1e-12);
    WR_ASSERT_EQ(wr_coverage_indices_in(&cov, window), 0);
}

WR_TEST(coverage_handles_the_empty_set)
{
    wr_index_range storage[STORAGE];
    wr_index_range gaps[4];
    wr_coverage cov;
    wr_index_range window = {0, 99};

    wr_coverage_init(&cov, storage, STORAGE);
    WR_ASSERT_EQ(wr_coverage_gaps(&cov, window, gaps, 4), 1);
    WR_ASSERT_EQ(gaps[0].first, 0);
    WR_ASSERT_EQ(gaps[0].last, 99);
    WR_ASSERT_EQ(wr_coverage_largest_gap(&cov, window), 100);
}

WR_TEST_MAIN()
