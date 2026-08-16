/* SPDX-License-Identifier: LGPL-2.1-or-later */
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
#include "hm_test.h"
#include "hm_wire.h"

#include "hm_codec.h"
#include "hm_overlap.h"

static hm_sample make(uint32_t index, int16_t gyro_x, uint16_t ticks)
{
    uint8_t buf[64];
    hm_wire_block arm = hm_wire_identity_block(ticks);
    hm_wire_block palm = hm_wire_identity_block((uint16_t)(ticks + 59u));
    hm_stream_config cfg = hm_stream_config_default();
    hm_decoded d;
    size_t n;

    arm.gyro[0] = gyro_x;
    n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
    (void)hm_codec_decode(buf, n, cfg, &d);
    d.u.frame.sample[0].sample_index = index;
    return d.u.frame.sample[0];
}

/* The digest must see every raw field, and nothing derived. */
HM_TEST(overlap_digest_covers_the_raw_counts)
{
    hm_sample a = make(100, 500, 1000);
    hm_sample b = make(100, 500, 1000);
    hm_sample gyro_differs = make(100, 501, 1000);
    hm_sample ticks_differ = make(100, 500, 1001);

    HM_ASSERT_EQ(hm_sample_raw_digest(&a), hm_sample_raw_digest(&b));
    HM_ASSERT(hm_sample_raw_digest(&a) != hm_sample_raw_digest(&gyro_differs));
    HM_ASSERT(hm_sample_raw_digest(&a) != hm_sample_raw_digest(&ticks_differ));

    /* Quaternion and acceleration too — one bit anywhere must show. */
    b.lower_arm.q_world_to_body_raw[2] += 1;
    HM_ASSERT(hm_sample_raw_digest(&a) != hm_sample_raw_digest(&b));
    b = a;
    b.palm.linear_accel_raw[1] += 1;
    HM_ASSERT(hm_sample_raw_digest(&a) != hm_sample_raw_digest(&b));

    /*
     * ⚠ But NOT the derived floats or the host-side fields.  The question is
     * whether the DEVICE sent the same values, not whether we scaled or
     * timestamped them the same way.
     */
    b = a;
    b.host_time_us = 12345;
    b.host_recv_us = 999;
    b.uncertainty_us = 7;
    b.calibration = HM_CAL_CALIBRATED;
    b.lower_arm.gyro_dps[0] = -1.0f;
    HM_ASSERT_EQ(hm_sample_raw_digest(&a), hm_sample_raw_digest(&b));

    HM_ASSERT_EQ(hm_sample_raw_digest(NULL), 0);
}

/* The happy path: history repeats what live delivered, and says so. */
HM_TEST(overlap_counts_agreement_over_a_pull)
{
    hm_live_digest storage[256];
    hm_overlap o;
    uint32_t samples = 0, mismatches = 0;

    hm_overlap_init(&o, storage, 256);

    /* Live at the 25 Hz mode: index step 32. */
    for (uint32_t i = 0; i < 200u; ++i) {
        hm_sample s = make(1000u + i * 32u, (int16_t)(i * 3), (uint16_t)(2000u + i));
        hm_overlap_note_live(&o, s.sample_index, hm_sample_raw_digest(&s));
    }

    /* A pull covering the same span at index step 1.  Only the indices live
     * also saw can be compared; the rest are new data, which is the point. */
    for (uint32_t idx = 1000u; idx < 1000u + 200u * 32u; ++idx) {
        hm_sample s;
        hm_overlap_result r;
        if ((idx - 1000u) % 32u == 0u) {
            uint32_t i = (idx - 1000u) / 32u;
            s = make(idx, (int16_t)(i * 3), (uint16_t)(2000u + i)); /* identical */
        } else {
            s = make(idx, 7, 4242); /* an index live never delivered */
        }
        r = hm_overlap_check(&o, idx, hm_sample_raw_digest(&s));
        if (r != HM_OVERLAP_ABSENT) {
            samples++;
            if (r == HM_OVERLAP_DIFFERS) {
                mismatches++;
            }
        }
    }

    HM_ASSERT_EQ(samples, 200);
    HM_ASSERT_MSG(mismatches == 0, "history repeating live must register as agreement");
}

/* ⚠ And the failure it exists to catch: same index, different values. */
HM_TEST(overlap_detects_a_value_disagreement)
{
    hm_live_digest storage[64];
    hm_overlap o;
    hm_sample live = make(500, 1234, 3000);
    hm_sample same = make(500, 1234, 3000);
    hm_sample different = make(500, 1235, 3000);

    hm_overlap_init(&o, storage, 64);
    hm_overlap_note_live(&o, live.sample_index, hm_sample_raw_digest(&live));

    HM_ASSERT_EQ(hm_overlap_check(&o, 500, hm_sample_raw_digest(&same)), HM_OVERLAP_AGREES);
    HM_ASSERT_EQ(hm_overlap_check(&o, 500, hm_sample_raw_digest(&different)),
                 HM_OVERLAP_DIFFERS);
    /* An index live never saw is not evidence of anything. */
    HM_ASSERT_EQ(hm_overlap_check(&o, 501, hm_sample_raw_digest(&same)), HM_OVERLAP_ABSENT);
}

/*
 * ⚠ No storage means the check is NOT PERFORMED, and must read as no evidence
 * rather than as agreement.  A zero mismatch count out of zero samples is the
 * shape of a check that silently stopped running.
 */
HM_TEST(overlap_without_storage_reports_no_evidence)
{
    hm_overlap o;
    hm_sample s = make(100, 1, 2);

    hm_overlap_init(&o, NULL, 0);
    hm_overlap_note_live(&o, s.sample_index, hm_sample_raw_digest(&s));
    HM_ASSERT_EQ(o.count, 0);
    HM_ASSERT_EQ(hm_overlap_check(&o, 100, hm_sample_raw_digest(&s)), HM_OVERLAP_ABSENT);
}

/* The ring drops oldest and counts what it dropped, so a pull reaching further
 * back than the ring covers is visibly under-checked rather than silently so. */
HM_TEST(overlap_ring_evicts_oldest_and_counts_it)
{
    hm_live_digest storage[8];
    hm_overlap o;

    hm_overlap_init(&o, storage, 8);
    for (uint32_t i = 0; i < 20u; ++i) {
        hm_sample s = make(i * 32u, (int16_t)i, (uint16_t)i);
        hm_overlap_note_live(&o, s.sample_index, hm_sample_raw_digest(&s));
    }
    HM_ASSERT_EQ(o.count, 8);
    HM_ASSERT_EQ(o.evicted, 12);

    /* The newest eight are still checkable; the evicted ones read as absent. */
    {
        hm_sample newest = make(19u * 32u, 19, 19);
        hm_sample gone = make(0, 0, 0);
        HM_ASSERT_EQ(hm_overlap_check(&o, newest.sample_index,
                                      hm_sample_raw_digest(&newest)),
                     HM_OVERLAP_AGREES);
        HM_ASSERT_EQ(hm_overlap_check(&o, 0, hm_sample_raw_digest(&gone)), HM_OVERLAP_ABSENT);
    }
}

/* Live indices ascend within a stream (§6.5); a repeat overwrites and an
 * out-of-order arrival is ignored rather than corrupting the ordering the
 * lookup depends on. */
HM_TEST(overlap_handles_repeats_and_out_of_order_arrivals)
{
    hm_live_digest storage[16];
    hm_overlap o;
    hm_sample first = make(100, 1, 10);
    hm_sample second = make(132, 2, 11);
    hm_sample repeat = make(132, 99, 11);
    hm_sample stale = make(64, 3, 12);

    hm_overlap_init(&o, storage, 16);
    hm_overlap_note_live(&o, 100, hm_sample_raw_digest(&first));
    hm_overlap_note_live(&o, 132, hm_sample_raw_digest(&second));
    HM_ASSERT_EQ(o.count, 2);

    hm_overlap_note_live(&o, 132, hm_sample_raw_digest(&repeat)); /* overwrite */
    HM_ASSERT_EQ(o.count, 2);
    HM_ASSERT_EQ(hm_overlap_check(&o, 132, hm_sample_raw_digest(&repeat)), HM_OVERLAP_AGREES);

    hm_overlap_note_live(&o, 64, hm_sample_raw_digest(&stale)); /* ignored */
    HM_ASSERT_EQ(o.count, 2);
    HM_ASSERT_EQ(hm_overlap_check(&o, 64, hm_sample_raw_digest(&stale)), HM_OVERLAP_ABSENT);
}

/* A full-rate pull performs thousands of lookups; the search must not be linear
 * in the ring size for each one. */
HM_TEST(overlap_lookup_scales_to_a_full_rate_pull)
{
    static hm_live_digest storage[1024];
    hm_overlap o;
    uint32_t found = 0;

    hm_overlap_init(&o, storage, 1024);
    for (uint32_t i = 0; i < 1024u; ++i) {
        hm_overlap_note_live(&o, i * 8u, (uint64_t)i * 2654435761u);
    }
    for (uint32_t idx = 0; idx < 8192u; ++idx) {
        if (hm_overlap_check(&o, idx, (uint64_t)(idx / 8u) * 2654435761u) == HM_OVERLAP_AGREES) {
            found++;
        }
    }
    HM_ASSERT_EQ(found, 1024);
    HM_ASSERT_EQ(hm_overlap_check(&o, 1024u * 8u, 0), HM_OVERLAP_ABSENT);
}

HM_TEST(overlap_reset_keeps_storage_and_clears_evidence)
{
    hm_live_digest storage[8];
    hm_overlap o;
    hm_sample s = make(100, 1, 2);

    hm_overlap_init(&o, storage, 8);
    hm_overlap_note_live(&o, 100, hm_sample_raw_digest(&s));
    hm_overlap_reset(&o);

    HM_ASSERT_EQ(o.count, 0);
    HM_ASSERT_EQ(o.evicted, 0);
    HM_ASSERT_EQ(o.capacity, 8);
    /* ⚠ A new stream is a new index space (§6.5), so evidence must not survive
     * one — an index from the old stream must never match the new. */
    HM_ASSERT_EQ(hm_overlap_check(&o, 100, hm_sample_raw_digest(&s)), HM_OVERLAP_ABSENT);
}

HM_TEST_MAIN()
