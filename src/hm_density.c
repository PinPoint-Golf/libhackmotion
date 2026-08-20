/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_density.c — ACHIEVED DENSITY, and it is a SPACING rather than a RATE.
 *
 * ⚠ THE OBVIOUS DEFINITION IS A FREQUENCY WEARING DIFFERENT UNITS, AND THE BLOCK
 * ALREADY PUBLISHES THE FREQUENCY.
 *
 * "Samples per unit of span" is exactly `achieved_hz` divided by the device's
 * full internal rate, and `achieved_hz` sits in the next field of the same
 * struct (history.h).  Worse, on real hardware `coverage_fraction` carries that
 * same value too: §7.1 measured the device HOLING an over-wide request evenly
 * across the whole range it was asked for rather than clamping it, so "how much
 * of the window arrived" and "how fast what arrived was sampled" come out equal.
 * Three fields, one number, and no way to answer C4.
 *
 * So the measure here is the TYPICAL GAP between consecutive delivered indices,
 * reported as its reciprocal:
 *
 *     density = 1 / median(delivered index step)
 *
 * §7.3's two regimes land on the two ends of that scale by construction: step 1
 * is the full ≈799.2 Hz and reads 1.0, step 8 is the ≈100 Hz a still wrist
 * returns and reads 0.125.  Across 25 retrievals and 17,739 measured steps none
 * exceeded 8 and none was 0, so on this device the field's whole working range
 * is [0.125, 1.0] and anything below it is a delivery failure rather than
 * motion.
 *
 * ⚠ AND THE MEDIAN IS WHY IT ANSWERS C4 WHERE AN AVERAGE CANNOT.  §7.5's
 * measured 58 % reply is two dense runs with one 168-index hole between them.
 * Averaged over its span that reads 0.58, indistinguishable from a reply that
 * was uniformly half-dense — which is the very pair api-request C4 calls
 * Critical to tell apart, because at impact one of them has the samples and the
 * other does not.  The median reads 1.0 for the first and 0.5 for the second.
 * The hole is reported by `largest_gap_us` and `gaps[]`, which is where a hole
 * belongs.
 *
 * ⚠ MEASURED OVER THE SAMPLES, NEVER OVER THE COVERAGE SET.  A reply at §7.3's
 * floor is one interval per delivered index (history.h), which is precisely the
 * shape that exhausts `hm_coverage`'s caller-provided storage — and on overflow
 * two intervals are coalesced ACROSS THEIR GAP, erasing the step this function
 * exists to measure.  The at-rest regime is both the one that overflows and the
 * one density matters most in, so the delivered samples are the only input that
 * stays exact.
 *
 * ⚠ `hm_report`'s `history_modal_step` (record.h) is a neighbour, not a copy:
 * it is the MODE over a whole recording, computed off the wire by the offline
 * reconciler, where this is the MEDIAN over one block, computed in the core.
 * They answer different questions at different scopes and are expected to
 * differ; neither is derived from the other.
 */
#include "hackmotion/history.h"

#include <string.h>

/*
 * ⚠ The upper median: with an even number of gaps this takes the SPARSER of the
 * two middle values.  Over-claiming density is the direction that invites a
 * consumer to resample onto a uniform 800 Hz grid (§7.6), which invents samples
 * that were never taken.
 */
double hm_sample_step_density(const hm_sample *samples, size_t count)
{
    uint32_t hist[HM_DENSITY_STEP_MAX + 1u];
    uint64_t seen = 0u;
    uint64_t steps;
    uint64_t target;
    size_t   i;

    if (samples == NULL || count < 2u) {
        return 0.0; /* one sample cannot show a spacing */
    }

    memset(hist, 0, sizeof(hist));
    steps = (uint64_t)count - 1u;

    for (i = 1u; i < count; ++i) {
        uint64_t step;

        /* Strictly ascending in device index is the block's own guarantee
         * (api-request C3).  A caller who has re-ordered or merged by timestamp
         * gets "not measurable" rather than a number computed over nonsense. */
        if ((samples[i].flags & (uint16_t)HM_SAMPLE_INDEX_MISSING) != 0u ||
            (samples[i - 1u].flags & (uint16_t)HM_SAMPLE_INDEX_MISSING) != 0u ||
            samples[i].sample_index <= samples[i - 1u].sample_index) {
            return 0.0;
        }
        step = (uint64_t)samples[i].sample_index - (uint64_t)samples[i - 1u].sample_index;
        if (step <= (uint64_t)HM_DENSITY_STEP_MAX) {
            hist[step]++;
        }
        /* Steps beyond the histogram are counted only by their absence from it:
         * they sort above every bucket, so they can only push the median off the
         * end, which is reported as "not measurable" below. */
    }

    target = steps / 2u; /* 0-indexed position of the upper median */
    for (uint64_t step = 1u; step <= (uint64_t)HM_DENSITY_STEP_MAX; ++step) {
        seen += hist[step];
        if (seen > target) {
            return 1.0 / (double)step;
        }
    }
    /* More than half the gaps are wider than HM_DENSITY_STEP_MAX — fewer than
     * one sample in 256 indices.  `sample_count` and `delivered[]` say what
     * arrived; this field will not invent a spacing for it. */
    return 0.0;
}
