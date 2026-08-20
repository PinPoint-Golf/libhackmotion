/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "hackmotion/coverage.h"

#include <string.h>

void hm_coverage_init(hm_coverage *cov, hm_index_range *storage, size_t capacity)
{
    memset(cov, 0, sizeof(*cov));
    cov->ranges = storage;
    cov->capacity = capacity;
}

void hm_coverage_reset(hm_coverage *cov)
{
    hm_index_range *storage = cov->ranges;
    size_t capacity = cov->capacity;
    memset(cov, 0, sizeof(*cov));
    cov->ranges = storage;
    cov->capacity = capacity;
}

static uint64_t range_width(hm_index_range r)
{
    return (uint64_t)r.last - (uint64_t)r.first + 1u;
}

/* Merge the pair of neighbouring intervals separated by the smallest gap.  Used
 * only on overflow: the set stays a valid SUPERSET of the truth and `indices`
 * stays exact, but the gap list becomes optimistic — which is why `overflowed`
 * has to reach the consumer. */
static void coalesce_closest(hm_coverage *cov)
{
    size_t best = 0;
    uint64_t best_gap = UINT64_MAX;

    if (cov->count < 2u) {
        return;
    }
    for (size_t i = 0; i + 1u < cov->count; ++i) {
        uint64_t gap = (uint64_t)cov->ranges[i + 1u].first - (uint64_t)cov->ranges[i].last;
        if (gap < best_gap) {
            best_gap = gap;
            best = i;
        }
    }
    cov->ranges[best].last = cov->ranges[best + 1u].last;
    memmove(&cov->ranges[best + 1u], &cov->ranges[best + 2u],
            (cov->count - best - 2u) * sizeof(cov->ranges[0]));
    cov->count--;
    cov->dropped++;
    cov->overflowed = true;
}

hm_status hm_coverage_add(hm_coverage *cov, uint32_t index)
{
    hm_index_range r;
    r.first = index;
    r.last = index;
    return hm_coverage_add_range(cov, r);
}

hm_status hm_coverage_add_range(hm_coverage *cov, hm_index_range range)
{
    size_t pos;
    size_t merge_end;
    uint64_t before;

    if (cov == NULL || cov->ranges == NULL || cov->capacity == 0u) {
        return HM_ERR_INVALID_ARG;
    }
    if (range.first > range.last) {
        return HM_ERR_INVALID_ARG;
    }

    if (!cov->has_bounds) {
        cov->has_bounds = true;
        cov->bounds = range;
    } else {
        if (range.first < cov->bounds.first) {
            cov->bounds.first = range.first;
        }
        if (range.last > cov->bounds.last) {
            cov->bounds.last = range.last;
        }
    }

    /* Fast path: extends or is inside the final interval.  This is the shape
     * history delivery actually has — index step 1, ascending. */
    if (cov->count > 0u) {
        hm_index_range *tail = &cov->ranges[cov->count - 1u];
        if (range.first >= tail->first && range.first <= (uint64_t)tail->last + 1u) {
            if (range.last > tail->last) {
                cov->indices += (uint64_t)range.last - (uint64_t)tail->last;
                tail->last = range.last;
            }
            return HM_OK;
        }
        if (range.first > (uint64_t)tail->last + 1u) {
            if (cov->count == cov->capacity) {
                coalesce_closest(cov);
                if (cov->count == cov->capacity) {
                    return HM_ERR_BUFFER_TOO_SMALL;
                }
            }
            cov->ranges[cov->count++] = range;
            cov->indices += range_width(range);
            return HM_OK;
        }
    } else {
        cov->ranges[cov->count++] = range;
        cov->indices = range_width(range);
        return HM_OK;
    }

    /* Slow path: an out-of-order insertion.  Find the position, splice, then
     * absorb every interval the new one now touches. */
    pos = 0;
    while (pos < cov->count && (uint64_t)cov->ranges[pos].last + 1u < (uint64_t)range.first) {
        pos++;
    }

    merge_end = pos;
    while (merge_end < cov->count &&
           (uint64_t)cov->ranges[merge_end].first <= (uint64_t)range.last + 1u) {
        merge_end++;
    }

    if (merge_end > pos) {
        hm_index_range merged = range;
        for (size_t i = pos; i < merge_end; ++i) {
            if (cov->ranges[i].first < merged.first) {
                merged.first = cov->ranges[i].first;
            }
            if (cov->ranges[i].last > merged.last) {
                merged.last = cov->ranges[i].last;
            }
        }
        before = 0;
        for (size_t i = pos; i < merge_end; ++i) {
            before += range_width(cov->ranges[i]);
        }
        cov->ranges[pos] = merged;
        if (merge_end > pos + 1u) {
            memmove(&cov->ranges[pos + 1u], &cov->ranges[merge_end],
                    (cov->count - merge_end) * sizeof(cov->ranges[0]));
            cov->count -= (merge_end - pos - 1u);
        }
        cov->indices += range_width(merged) - before;
        return HM_OK;
    }

    if (cov->count == cov->capacity) {
        coalesce_closest(cov);
        if (cov->count == cov->capacity) {
            return HM_ERR_BUFFER_TOO_SMALL;
        }
    }
    memmove(&cov->ranges[pos + 1u], &cov->ranges[pos],
            (cov->count - pos) * sizeof(cov->ranges[0]));
    cov->ranges[pos] = range;
    cov->count++;
    cov->indices += range_width(range);
    return HM_OK;
}

bool hm_coverage_contains(const hm_coverage *cov, uint32_t index)
{
    for (size_t i = 0; i < cov->count; ++i) {
        if (index >= cov->ranges[i].first && index <= cov->ranges[i].last) {
            return true;
        }
        if (cov->ranges[i].first > index) {
            break;
        }
    }
    return false;
}

uint64_t hm_coverage_indices_in(const hm_coverage *cov, hm_index_range window)
{
    uint64_t total = 0;

    if (window.first > window.last) {
        return 0;
    }
    for (size_t i = 0; i < cov->count; ++i) {
        uint32_t lo = cov->ranges[i].first > window.first ? cov->ranges[i].first : window.first;
        uint32_t hi = cov->ranges[i].last < window.last ? cov->ranges[i].last : window.last;
        if (lo <= hi) {
            total += (uint64_t)hi - (uint64_t)lo + 1u;
        }
    }
    return total;
}

double hm_coverage_fraction(const hm_coverage *cov, hm_index_range window)
{
    uint64_t width;

    if (window.first > window.last) {
        return 0.0;
    }
    width = (uint64_t)window.last - (uint64_t)window.first + 1u;
    return (double)hm_coverage_indices_in(cov, window) / (double)width;
}

size_t hm_coverage_gaps(const hm_coverage *cov, hm_index_range window, hm_index_range *out,
                        size_t max)
{
    size_t written = 0;
    uint64_t cursor;

    if (window.first > window.last || out == NULL || max == 0u) {
        return 0;
    }
    cursor = window.first;

    for (size_t i = 0; i < cov->count && written < max; ++i) {
        uint32_t lo = cov->ranges[i].first;
        uint32_t hi = cov->ranges[i].last;
        if (hi < window.first) {
            continue;
        }
        if (lo > window.last) {
            break;
        }
        if ((uint64_t)lo > cursor) {
            out[written].first = (uint32_t)cursor;
            out[written].last = lo - 1u;
            written++;
        }
        if ((uint64_t)hi + 1u > cursor) {
            cursor = (uint64_t)hi + 1u;
        }
    }
    if (written < max && cursor <= (uint64_t)window.last) {
        out[written].first = (uint32_t)cursor;
        out[written].last = window.last;
        written++;
    }
    return written;
}

uint32_t hm_coverage_largest_gap(const hm_coverage *cov, hm_index_range window)
{
    uint64_t cursor;
    uint64_t largest = 0;

    if (window.first > window.last) {
        return 0;
    }
    cursor = window.first;

    for (size_t i = 0; i < cov->count; ++i) {
        uint32_t lo = cov->ranges[i].first;
        uint32_t hi = cov->ranges[i].last;
        if (hi < window.first) {
            continue;
        }
        if (lo > window.last) {
            break;
        }
        if ((uint64_t)lo > cursor) {
            uint64_t gap = (uint64_t)lo - cursor;
            if (gap > largest) {
                largest = gap;
            }
        }
        if ((uint64_t)hi + 1u > cursor) {
            cursor = (uint64_t)hi + 1u;
        }
    }
    if (cursor <= (uint64_t)window.last) {
        uint64_t gap = (uint64_t)window.last - cursor + 1u;
        if (gap > largest) {
            largest = gap;
        }
    }
    return (largest > UINT32_MAX) ? UINT32_MAX : (uint32_t)largest;
}
