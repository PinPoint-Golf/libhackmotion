/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_overlap.h — internal.  The live-vs-history agreement check.
 *
 * ⚠ WHY THIS EXISTS.  A consumer stitching a shot's lane as
 * [live prefix] + [retrieved 800 Hz span] + [live suffix] is relying on history
 * being a strict SUPERSET of live over the same span — same index, same values.
 *
 * §6.5 calls the live stream "a decimated view of the internal rate", which
 * implies it, but the specification never asserts that the VALUES are identical
 * for a given index.  If they are not, a stitched lane has a discontinuity at
 * the seam that looks exactly like a real wrist movement.
 *
 * The library is the only layer that ever holds both halves: a mid-stream `a1`
 * (§7.5) covers a span the live stream already delivered, so the comparison is
 * available inside every gather, for free.  Counting it turns an assumption
 * into a STANDING measurement — on every capture, on hardware nobody here has
 * seen.  Always zero, and the stitching contract is proven in the field rather
 * than argued; ever non-zero, and everyone finds out at once instead of chasing
 * a phantom movement at a seam.
 */
#ifndef WR_OVERLAP_H
#define WR_OVERLAP_H

#include "wrist/types.h"
#include "wrist/sample.h"
#include "wrist/session.h"

/*
 * A 64-bit FNV-1a over a record's RAW counts — both units' quaternion,
 * acceleration, gyroscope and tick fields, in wire order.
 *
 * Raw only.  The scaled floats are derived from these under a configuration
 * that could differ between the two decodes, and the question being asked is
 * whether the DEVICE sent the same values, not whether we scaled them the same
 * way.  The record header index is excluded because it is the key.
 */
uint64_t wr_sample_raw_digest(const wr_sample *sample);

typedef struct wr_overlap {
    wr_live_digest *ring;
    size_t          capacity;
    size_t          count;
    size_t          head;      /* oldest entry; entries ascend in index from here */
    uint64_t        evicted;   /* live entries dropped before a pull could use them */
} wr_overlap;

void wr_overlap_init(wr_overlap *o, wr_live_digest *storage, size_t capacity);
void wr_overlap_reset(wr_overlap *o);

/* Record a live sample.  Indices must be non-decreasing, which they are within
 * a stream (§6.5).  Oldest is evicted when full. */
void wr_overlap_note_live(wr_overlap *o, uint32_t sample_index, uint64_t digest);

typedef enum wr_overlap_result {
    WR_OVERLAP_ABSENT   = 0, /* this index was never seen live — no evidence     */
    WR_OVERLAP_AGREES   = 1, /* seen live, and the values match                  */
    WR_OVERLAP_DIFFERS  = 2  /* ⚠ seen live, and the values DIFFER               */
} wr_overlap_result;

wr_overlap_result wr_overlap_check(const wr_overlap *o, uint32_t sample_index,
                                   uint64_t digest);

#endif /* WR_OVERLAP_H */
