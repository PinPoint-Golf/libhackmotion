/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "wr_overlap.h"

#include <string.h>

#define FNV64_OFFSET 1469598103934665603ULL
#define FNV64_PRIME  1099511628211ULL

static void fnv_u16(uint64_t *h, uint16_t v)
{
    *h = (*h ^ (uint64_t)(v & 0xffu)) * FNV64_PRIME;
    *h = (*h ^ (uint64_t)((v >> 8) & 0xffu)) * FNV64_PRIME;
}

static void fnv_unit(uint64_t *h, const wr_unit_sample *u)
{
    for (int i = 0; i < 4; ++i) {
        fnv_u16(h, (uint16_t)u->q_world_to_body_raw[i]);
    }
    for (int i = 0; i < 3; ++i) {
        fnv_u16(h, (uint16_t)u->linear_accel_raw[i]);
    }
    for (int i = 0; i < 3; ++i) {
        fnv_u16(h, (uint16_t)u->gyro_raw[i]);
    }
    fnv_u16(h, u->ticks_raw);
}

uint64_t wr_sample_raw_digest(const wr_sample *sample)
{
    uint64_t h = FNV64_OFFSET;
    if (sample == NULL) {
        return 0;
    }
    fnv_unit(&h, &sample->lower_arm);
    fnv_unit(&h, &sample->palm);
    return h;
}

void wr_overlap_init(wr_overlap *o, wr_live_digest *storage, size_t capacity)
{
    memset(o, 0, sizeof(*o));
    o->ring = storage;
    o->capacity = (storage != NULL) ? capacity : 0u;
}

void wr_overlap_reset(wr_overlap *o)
{
    wr_live_digest *storage = o->ring;
    size_t capacity = o->capacity;
    memset(o, 0, sizeof(*o));
    o->ring = storage;
    o->capacity = capacity;
}

static wr_live_digest *slot(const wr_overlap *o, size_t logical)
{
    return &o->ring[(o->head + logical) % o->capacity];
}

void wr_overlap_note_live(wr_overlap *o, uint32_t sample_index, uint64_t digest)
{
    wr_live_digest *e;

    if (o->ring == NULL || o->capacity == 0u) {
        return; /* no storage supplied: the check is simply not performed */
    }

    /* Within a stream the index only increases (§6.5).  A repeat of the newest
     * index overwrites rather than duplicating, so a retried decode cannot
     * inflate the sample count. */
    if (o->count > 0u) {
        wr_live_digest *newest = slot(o, o->count - 1u);
        if (sample_index == newest->sample_index) {
            newest->digest = digest;
            return;
        }
        if (sample_index < newest->sample_index) {
            return; /* out of order: not evidence of anything */
        }
    }

    if (o->count == o->capacity) {
        o->head = (o->head + 1u) % o->capacity;
        o->count--;
        o->evicted++;
    }
    e = slot(o, o->count);
    e->sample_index = sample_index;
    e->reserved = 0u;
    e->digest = digest;
    o->count++;
}

wr_overlap_result wr_overlap_check(const wr_overlap *o, uint32_t sample_index, uint64_t digest)
{
    size_t lo = 0;
    size_t hi;

    if (o->ring == NULL || o->count == 0u) {
        return WR_OVERLAP_ABSENT;
    }

    /* Entries ascend in index from `head`, so a binary search over the logical
     * order works and keeps a full-rate pull's worth of lookups cheap. */
    hi = o->count;
    while (lo < hi) {
        size_t mid = lo + (hi - lo) / 2u;
        const wr_live_digest *e = slot(o, mid);
        if (e->sample_index == sample_index) {
            return (e->digest == digest) ? WR_OVERLAP_AGREES : WR_OVERLAP_DIFFERS;
        }
        if (e->sample_index < sample_index) {
            lo = mid + 1u;
        } else {
            hi = mid;
        }
    }
    return WR_OVERLAP_ABSENT;
}
