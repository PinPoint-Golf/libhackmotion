/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/coverage.h — what actually arrived, as intervals.
 *
 * ⚠ THE DEVICE DOES NOT CLAMP AN OVER-WIDE HISTORY REQUEST — IT HOLES ONE.
 *
 * Spec §7.1 and §7.3: asking for more than the buffer holds returns a HOLED set
 * spanning most of the requested range at 33-58 % coverage, with no error and
 * no indication.  That is materially worse than clamping, because a contiguous
 * short block is at least obviously short.  api-request §2.15 C4 promotes this
 * to Critical: reporting coverage as INTERVALS AND ACHIEVED DENSITY is the only
 * way a consumer can detect the device's main failure mode.
 *
 * A COUNT CANNOT DISTINGUISH "dense over half the range" FROM "half-dense over
 * all of it", and at 33-58 % coverage that distinction decides whether a metric
 * computed at impact exists at all.  So this library never reports a count on
 * its own.
 *
 * All storage is caller-provided; nothing here allocates.
 */
#ifndef HACKMOTION_COVERAGE_H
#define HACKMOTION_COVERAGE_H

#include "hackmotion/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A sorted, disjoint, non-adjacent set of inclusive index ranges.
 *
 * Ranges are kept merged: adding index 5 to a set holding [1,4] yields [1,5],
 * not two entries.  Insertion in ascending order — the order records arrive in
 * — is O(1).  Out-of-order insertion is supported and is O(count).
 */
typedef struct hm_coverage {
    hm_index_range *ranges;    /* caller-owned storage */
    size_t          capacity;
    size_t          count;
    uint64_t        indices;   /* total distinct indices held */
    uint32_t        dropped;   /* intervals lost to capacity — see `overflowed` */
    bool            overflowed;/* ⚠ the interval list is APPROXIMATE from here on */
    bool            has_bounds;
    hm_index_range  bounds;    /* min..max seen, exact even after overflow */
} hm_coverage;

HM_API void hm_coverage_init(hm_coverage *cov, hm_index_range *storage, size_t capacity);
HM_API void hm_coverage_reset(hm_coverage *cov);

/* HM_OK, or HM_ERR_BUFFER_TOO_SMALL once `overflowed` is set.  On overflow the
 * two nearest neighbouring intervals are coalesced across their gap, so the set
 * stays a valid superset of the truth and `indices` stays exact — but the gap
 * list becomes optimistic, which is why `overflowed` must be surfaced. */
HM_API hm_status hm_coverage_add(hm_coverage *cov, uint32_t index);
HM_API hm_status hm_coverage_add_range(hm_coverage *cov, hm_index_range range);

HM_API bool hm_coverage_contains(const hm_coverage *cov, uint32_t index);

/* Distinct indices held within `window`, and the fraction of `window` they
 * cover, in [0,1].  A window of zero width yields 0.0. */
HM_API uint64_t hm_coverage_indices_in(const hm_coverage *cov, hm_index_range window);
HM_API double   hm_coverage_fraction(const hm_coverage *cov, hm_index_range window);

/*
 * ⚠ ACHIEVED DENSITY IS NOT HERE, AND THAT IS DELIBERATE.  C4's other half is
 * hm_sample_step_density() in history.h, measured over the DELIVERED SAMPLES.
 *
 * It cannot be computed from an interval set.  A reply at §7.3's 100 Hz floor is
 * one interval per delivered index, so it is exactly the shape that overflows
 * the storage below — and overflow coalesces two intervals ACROSS THEIR GAP,
 * erasing the very step a density would be measuring.  The at-rest regime is
 * both the one that overflows and the one density matters most in.
 *
 * A version of this function used to live here and divided the delivered sample
 * count by `indices`.  The session increments both on the same line, so it
 * returned 1.0 for every reply that ever arrived — a constant wearing a
 * measurement's clothes, published as a positive finding across seven real
 * pulls (implementation-review I1).
 */

/* The complement of the covered set within `window`, ascending.  Returns the
 * number of gaps written; if the return value equals `max` there may be more. */
HM_API size_t hm_coverage_gaps(const hm_coverage *cov,
                               hm_index_range window,
                               hm_index_range *out,
                               size_t max);

/* The largest single gap inside `window`, in indices.  0 when complete.  A
 * consumer gating an analysis stage cares about this more than about the total:
 * one 400-sample hole at impact is fatal where the same count scattered is not. */
HM_API uint32_t hm_coverage_largest_gap(const hm_coverage *cov, hm_index_range window);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_COVERAGE_H */
