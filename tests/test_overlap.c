/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_overlap.c — the live-vs-history agreement check.
 *
 * ⚠ What this defends: a consumer stitching [live prefix] + [retrieved 800 Hz
 * span] + [live suffix] into one ascending lane is assuming history is a strict
 * SUPERSET of live over the same span — same index, same values.  §6.5 implies
 * it; the specification never asserts it.  If it is false, the seam looks like a
 * real wrist movement.
 */
#include "wr_test.h"
#include "wr_wire.h"

#include "wr_codec.h"
#include "wr_overlap.h"

static wr_sample make(uint32_t index, int16_t gyro_x, uint16_t ticks)
{
    uint8_t buf[64];
    wr_wire_block arm = wr_wire_identity_block(ticks);
    wr_wire_block palm = wr_wire_identity_block((uint16_t)(ticks + 59u));
    wr_stream_config cfg = wr_stream_config_default();
    wr_decoded d;
    size_t n;

    arm.gyro[0] = gyro_x;
    n = wr_wire_notification1(buf, (uint16_t)index, &arm, &palm);
    (void)wr_codec_decode(buf, n, cfg, &d);
    d.u.frame.sample[0].sample_index = index;
    return d.u.frame.sample[0];
}

/* The digest must see every raw field, and nothing derived. */
WR_TEST(overlap_digest_covers_the_raw_counts)
{
    wr_sample a = make(100, 500, 1000);
    wr_sample b = make(100, 500, 1000);
    wr_sample gyro_differs = make(100, 501, 1000);
    wr_sample ticks_differ = make(100, 500, 1001);

    WR_ASSERT_EQ(wr_sample_raw_digest(&a), wr_sample_raw_digest(&b));
    WR_ASSERT(wr_sample_raw_digest(&a) != wr_sample_raw_digest(&gyro_differs));
    WR_ASSERT(wr_sample_raw_digest(&a) != wr_sample_raw_digest(&ticks_differ));

    /* Quaternion and acceleration too — one bit anywhere must show. */
    b.lower_arm.q_world_to_body_raw[2] += 1;
    WR_ASSERT(wr_sample_raw_digest(&a) != wr_sample_raw_digest(&b));
    b = a;
    b.palm.linear_accel_raw[1] += 1;
    WR_ASSERT(wr_sample_raw_digest(&a) != wr_sample_raw_digest(&b));

    /*
     * ⚠ But NOT the derived floats or the host-side fields.  The question is
     * whether the DEVICE sent the same values, not whether we scaled or
     * timestamped them the same way.
     */
    b = a;
    b.host_time_us = 12345;
    b.host_recv_us = 999;
    b.uncertainty_us = 7;
    b.calibration = WR_CAL_CALIBRATED;
    b.lower_arm.gyro_dps[0] = -1.0f;
    WR_ASSERT_EQ(wr_sample_raw_digest(&a), wr_sample_raw_digest(&b));

    WR_ASSERT_EQ(wr_sample_raw_digest(NULL), 0);
}

/* The happy path: history repeats what live delivered, and says so. */
WR_TEST(overlap_counts_agreement_over_a_pull)
{
    wr_live_digest storage[256];
    wr_overlap o;
    uint32_t samples = 0, mismatches = 0;

    wr_overlap_init(&o, storage, 256);

    /* Live at the 25 Hz mode: index step 32. */
    for (uint32_t i = 0; i < 200u; ++i) {
        wr_sample s = make(1000u + i * 32u, (int16_t)(i * 3), (uint16_t)(2000u + i));
        wr_overlap_note_live(&o, s.sample_index, wr_sample_raw_digest(&s));
    }

    /* A pull covering the same span at index step 1.  Only the indices live
     * also saw can be compared; the rest are new data, which is the point. */
    for (uint32_t idx = 1000u; idx < 1000u + 200u * 32u; ++idx) {
        wr_sample s;
        wr_overlap_result r;
        if ((idx - 1000u) % 32u == 0u) {
            uint32_t i = (idx - 1000u) / 32u;
            s = make(idx, (int16_t)(i * 3), (uint16_t)(2000u + i)); /* identical */
        } else {
            s = make(idx, 7, 4242); /* an index live never delivered */
        }
        r = wr_overlap_check(&o, idx, wr_sample_raw_digest(&s));
        if (r != WR_OVERLAP_ABSENT) {
            samples++;
            if (r == WR_OVERLAP_DIFFERS) {
                mismatches++;
            }
        }
    }

    WR_ASSERT_EQ(samples, 200);
    WR_ASSERT_MSG(mismatches == 0, "history repeating live must register as agreement");
}

/* ⚠ And the failure it exists to catch: same index, different values. */
WR_TEST(overlap_detects_a_value_disagreement)
{
    wr_live_digest storage[64];
    wr_overlap o;
    wr_sample live = make(500, 1234, 3000);
    wr_sample same = make(500, 1234, 3000);
    wr_sample different = make(500, 1235, 3000);

    wr_overlap_init(&o, storage, 64);
    wr_overlap_note_live(&o, live.sample_index, wr_sample_raw_digest(&live));

    WR_ASSERT_EQ(wr_overlap_check(&o, 500, wr_sample_raw_digest(&same)), WR_OVERLAP_AGREES);
    WR_ASSERT_EQ(wr_overlap_check(&o, 500, wr_sample_raw_digest(&different)),
                 WR_OVERLAP_DIFFERS);
    /* An index live never saw is not evidence of anything. */
    WR_ASSERT_EQ(wr_overlap_check(&o, 501, wr_sample_raw_digest(&same)), WR_OVERLAP_ABSENT);
}

/*
 * ⚠ No storage means the check is NOT PERFORMED, and must read as no evidence
 * rather than as agreement.  A zero mismatch count out of zero samples is the
 * shape of a check that silently stopped running.
 */
WR_TEST(overlap_without_storage_reports_no_evidence)
{
    wr_overlap o;
    wr_sample s = make(100, 1, 2);

    wr_overlap_init(&o, NULL, 0);
    wr_overlap_note_live(&o, s.sample_index, wr_sample_raw_digest(&s));
    WR_ASSERT_EQ(o.count, 0);
    WR_ASSERT_EQ(wr_overlap_check(&o, 100, wr_sample_raw_digest(&s)), WR_OVERLAP_ABSENT);
}

/* The ring drops oldest and counts what it dropped, so a pull reaching further
 * back than the ring covers is visibly under-checked rather than silently so. */
WR_TEST(overlap_ring_evicts_oldest_and_counts_it)
{
    wr_live_digest storage[8];
    wr_overlap o;

    wr_overlap_init(&o, storage, 8);
    for (uint32_t i = 0; i < 20u; ++i) {
        wr_sample s = make(i * 32u, (int16_t)i, (uint16_t)i);
        wr_overlap_note_live(&o, s.sample_index, wr_sample_raw_digest(&s));
    }
    WR_ASSERT_EQ(o.count, 8);
    WR_ASSERT_EQ(o.evicted, 12);

    /* The newest eight are still checkable; the evicted ones read as absent. */
    {
        wr_sample newest = make(19u * 32u, 19, 19);
        wr_sample gone = make(0, 0, 0);
        WR_ASSERT_EQ(wr_overlap_check(&o, newest.sample_index,
                                      wr_sample_raw_digest(&newest)),
                     WR_OVERLAP_AGREES);
        WR_ASSERT_EQ(wr_overlap_check(&o, 0, wr_sample_raw_digest(&gone)), WR_OVERLAP_ABSENT);
    }
}

/* Live indices ascend within a stream (§6.5); a repeat overwrites and an
 * out-of-order arrival is ignored rather than corrupting the ordering the
 * lookup depends on. */
WR_TEST(overlap_handles_repeats_and_out_of_order_arrivals)
{
    wr_live_digest storage[16];
    wr_overlap o;
    wr_sample first = make(100, 1, 10);
    wr_sample second = make(132, 2, 11);
    wr_sample repeat = make(132, 99, 11);
    wr_sample stale = make(64, 3, 12);

    wr_overlap_init(&o, storage, 16);
    wr_overlap_note_live(&o, 100, wr_sample_raw_digest(&first));
    wr_overlap_note_live(&o, 132, wr_sample_raw_digest(&second));
    WR_ASSERT_EQ(o.count, 2);

    wr_overlap_note_live(&o, 132, wr_sample_raw_digest(&repeat)); /* overwrite */
    WR_ASSERT_EQ(o.count, 2);
    WR_ASSERT_EQ(wr_overlap_check(&o, 132, wr_sample_raw_digest(&repeat)), WR_OVERLAP_AGREES);

    wr_overlap_note_live(&o, 64, wr_sample_raw_digest(&stale)); /* ignored */
    WR_ASSERT_EQ(o.count, 2);
    WR_ASSERT_EQ(wr_overlap_check(&o, 64, wr_sample_raw_digest(&stale)), WR_OVERLAP_ABSENT);
}

/* A full-rate pull performs thousands of lookups; the search must not be linear
 * in the ring size for each one. */
WR_TEST(overlap_lookup_scales_to_a_full_rate_pull)
{
    static wr_live_digest storage[1024];
    wr_overlap o;
    uint32_t found = 0;

    wr_overlap_init(&o, storage, 1024);
    for (uint32_t i = 0; i < 1024u; ++i) {
        wr_overlap_note_live(&o, i * 8u, (uint64_t)i * 2654435761u);
    }
    for (uint32_t idx = 0; idx < 8192u; ++idx) {
        if (wr_overlap_check(&o, idx, (uint64_t)(idx / 8u) * 2654435761u) == WR_OVERLAP_AGREES) {
            found++;
        }
    }
    WR_ASSERT_EQ(found, 1024);
    WR_ASSERT_EQ(wr_overlap_check(&o, 1024u * 8u, 0), WR_OVERLAP_ABSENT);
}

WR_TEST(overlap_reset_keeps_storage_and_clears_evidence)
{
    wr_live_digest storage[8];
    wr_overlap o;
    wr_sample s = make(100, 1, 2);

    wr_overlap_init(&o, storage, 8);
    wr_overlap_note_live(&o, 100, wr_sample_raw_digest(&s));
    wr_overlap_reset(&o);

    WR_ASSERT_EQ(o.count, 0);
    WR_ASSERT_EQ(o.evicted, 0);
    WR_ASSERT_EQ(o.capacity, 8);
    /* ⚠ A new stream is a new index space (§6.5), so evidence must not survive
     * one — an index from the old stream must never match the new. */
    WR_ASSERT_EQ(wr_overlap_check(&o, 100, wr_sample_raw_digest(&s)), WR_OVERLAP_ABSENT);
}

WR_TEST_MAIN()
