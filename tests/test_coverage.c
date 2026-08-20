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
 * test_density.c; hm_coverage.c's own comment says why it cannot live here.
 */
#include "hm_test.h"
#include "hackmotion/coverage.h"

#define STORAGE 64

HM_TEST(coverage_merges_a_contiguous_ascending_delivery)
{
    hm_index_range storage[STORAGE];
    hm_coverage cov;
    hm_index_range window = {100, 199};

    hm_coverage_init(&cov, storage, STORAGE);
    for (uint32_t i = 100; i <= 199; ++i) {
        HM_ASSERT_EQ(hm_coverage_add(&cov, i), HM_OK);
    }
    HM_ASSERT_EQ(cov.count, 1);
    HM_ASSERT_EQ(cov.ranges[0].first, 100);
    HM_ASSERT_EQ(cov.ranges[0].last, 199);
    HM_ASSERT_EQ(cov.indices, 100);
    HM_ASSERT_NEAR(hm_coverage_fraction(&cov, window), 1.0, 1e-12);
    HM_ASSERT_EQ(hm_coverage_largest_gap(&cov, window), 0);
    HM_ASSERT(!cov.overflowed);
}

/*
 * ⚠ THE DISTINCTION C4 EXISTS FOR.  A count cannot tell "dense over half the
 * range" from "half-dense over all of it", and at 33-58 % coverage that decides
 * whether a metric computed at impact exists.
 */
HM_TEST(coverage_distinguishes_dense_over_half_from_half_dense_over_all)
{
    hm_index_range storage_a[STORAGE], storage_b[512];
    hm_coverage dense_half, half_dense_all;
    hm_index_range window = {0, 999};

    hm_coverage_init(&dense_half, storage_a, STORAGE);
    hm_coverage_init(&half_dense_all, storage_b, 512);

    for (uint32_t i = 0; i < 500u; ++i) {
        (void)hm_coverage_add(&dense_half, i);
    }
    for (uint32_t i = 0; i < 1000u; i += 2u) {
        (void)hm_coverage_add(&half_dense_all, i);
    }

    /* Identical sample counts. */
    HM_ASSERT_EQ(dense_half.indices, 500);
    HM_ASSERT_EQ(half_dense_all.indices, 500);

    /* Completely different swings. */
    HM_ASSERT_NEAR(hm_coverage_fraction(&dense_half, window), 0.5, 1e-12);
    HM_ASSERT_NEAR(hm_coverage_fraction(&half_dense_all, window), 0.5, 1e-12);
    HM_ASSERT_EQ(hm_coverage_largest_gap(&dense_half, window), 500);
    HM_ASSERT_EQ(hm_coverage_largest_gap(&half_dense_all, window), 1);
    HM_ASSERT_EQ(dense_half.count, 1);
    HM_ASSERT_EQ(half_dense_all.count, 500);
}

/* §7.5's measured numbers: 58 % mid-stream, 33 % stopped, both holed. */
HM_TEST(coverage_reports_the_measured_holed_deliveries)
{
    hm_index_range storage[512];
    hm_coverage cov;
    hm_index_range window = {0, 7999};

    hm_coverage_init(&cov, storage, 512);
    /* 58 % coverage spread across the whole span, as a mid-stream pull gave:
     * 80 runs of 58 consecutive indices with a 42-index hole after each. */
    for (uint32_t i = 0; i < 8000u; ++i) {
        if (i % 100u < 58u) {
            (void)hm_coverage_add(&cov, i);
        }
    }
    HM_ASSERT_NEAR(hm_coverage_fraction(&cov, window), 0.58, 1e-9);
    HM_ASSERT_EQ(cov.count, 80);
    HM_ASSERT_EQ(hm_coverage_largest_gap(&cov, window), 42);
    HM_ASSERT(!cov.overflowed);
    HM_ASSERT_MSG(hm_coverage_largest_gap(&cov, window) > 0,
                  "a holed delivery must not look complete");
}

HM_TEST(coverage_enumerates_gaps_within_a_window)
{
    hm_index_range storage[STORAGE];
    hm_index_range gaps[8];
    hm_coverage cov;
    hm_index_range window = {0, 99};
    size_t n;

    hm_coverage_init(&cov, storage, STORAGE);
    {
        hm_index_range a = {10, 19};
        hm_index_range b = {50, 59};
        (void)hm_coverage_add_range(&cov, a);
        (void)hm_coverage_add_range(&cov, b);
    }

    n = hm_coverage_gaps(&cov, window, gaps, 8);
    HM_ASSERT_EQ(n, 3);
    HM_ASSERT_EQ(gaps[0].first, 0);
    HM_ASSERT_EQ(gaps[0].last, 9);
    HM_ASSERT_EQ(gaps[1].first, 20);
    HM_ASSERT_EQ(gaps[1].last, 49);
    HM_ASSERT_EQ(gaps[2].first, 60);
    HM_ASSERT_EQ(gaps[2].last, 99);
    HM_ASSERT_EQ(hm_coverage_largest_gap(&cov, window), 40);
}

HM_TEST(coverage_absorbs_out_of_order_and_overlapping_insertions)
{
    hm_index_range storage[STORAGE];
    hm_coverage cov;

    hm_coverage_init(&cov, storage, STORAGE);
    {
        hm_index_range a = {50, 59};
        hm_index_range b = {10, 19};
        hm_index_range c = {20, 49}; /* bridges the two */
        hm_index_range d = {15, 25}; /* overlaps both, adds nothing */
        (void)hm_coverage_add_range(&cov, a);
        (void)hm_coverage_add_range(&cov, b);
        HM_ASSERT_EQ(cov.count, 2);
        (void)hm_coverage_add_range(&cov, c);
        HM_ASSERT_EQ(cov.count, 1);
        HM_ASSERT_EQ(cov.indices, 50);
        (void)hm_coverage_add_range(&cov, d);
        HM_ASSERT_EQ(cov.count, 1);
        HM_ASSERT_EQ(cov.indices, 50); /* ⚠ C3: no double counting */
    }
    HM_ASSERT(hm_coverage_contains(&cov, 10));
    HM_ASSERT(hm_coverage_contains(&cov, 59));
    HM_ASSERT(!hm_coverage_contains(&cov, 9));
    HM_ASSERT(!hm_coverage_contains(&cov, 60));
}

/* Adjacent ranges merge; a one-index hole does not. */
HM_TEST(coverage_merges_adjacent_but_not_separated_ranges)
{
    hm_index_range storage[STORAGE];
    hm_coverage cov;
    hm_index_range a = {10, 19};
    hm_index_range adjacent = {20, 29};
    hm_index_range gapped = {31, 40};

    hm_coverage_init(&cov, storage, STORAGE);
    (void)hm_coverage_add_range(&cov, a);
    (void)hm_coverage_add_range(&cov, adjacent);
    HM_ASSERT_EQ(cov.count, 1);
    (void)hm_coverage_add_range(&cov, gapped);
    HM_ASSERT_EQ(cov.count, 2);
    HM_ASSERT_EQ(cov.indices, 30);
}

/*
 * On overflow the set stays a valid SUPERSET and the index total stays exact,
 * but the gap list becomes optimistic — so `overflowed` has to reach the
 * consumer rather than being swallowed.
 */
HM_TEST(coverage_overflow_is_reported_not_silent)
{
    hm_index_range storage[4];
    hm_coverage cov;
    hm_coverage_init(&cov, storage, 4);

    for (uint32_t i = 0; i < 40u; i += 2u) {
        (void)hm_coverage_add(&cov, i);
    }
    HM_ASSERT(cov.overflowed);
    HM_ASSERT(cov.dropped > 0);
    HM_ASSERT(cov.count <= 4);
    HM_ASSERT_EQ(cov.bounds.first, 0);
    HM_ASSERT_EQ(cov.bounds.last, 38);
    /* The union is a superset, so the reported coverage is never an underclaim
     * of what arrived — the optimism is in the gaps, which is why it is flagged. */
    HM_ASSERT(hm_coverage_indices_in(&cov, cov.bounds) >= 20);
}

HM_TEST(coverage_rejects_inverted_ranges)
{
    hm_index_range storage[STORAGE];
    hm_coverage cov;
    hm_index_range bad = {100, 50};
    hm_index_range window = {100, 50};

    hm_coverage_init(&cov, storage, STORAGE);
    HM_ASSERT_EQ(hm_coverage_add_range(&cov, bad), HM_ERR_INVALID_ARG);
    HM_ASSERT_NEAR(hm_coverage_fraction(&cov, window), 0.0, 1e-12);
    HM_ASSERT_EQ(hm_coverage_indices_in(&cov, window), 0);
}

HM_TEST(coverage_handles_the_empty_set)
{
    hm_index_range storage[STORAGE];
    hm_index_range gaps[4];
    hm_coverage cov;
    hm_index_range window = {0, 99};

    hm_coverage_init(&cov, storage, STORAGE);
    HM_ASSERT_EQ(hm_coverage_gaps(&cov, window, gaps, 4), 1);
    HM_ASSERT_EQ(gaps[0].first, 0);
    HM_ASSERT_EQ(gaps[0].last, 99);
    HM_ASSERT_EQ(hm_coverage_largest_gap(&cov, window), 100);
}

HM_TEST_MAIN()
