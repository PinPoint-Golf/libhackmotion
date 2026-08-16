/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_record.c — the `.hmwire` container and the reconciliation that reads a
 * capture back against the specification.
 *
 * Two families of claim are pinned here.
 *
 *   The CONTAINER must return exactly what it was given, and must refuse a file
 *   it cannot trust rather than clamping it into a fixed-size buffer.  A wire
 *   recording is the artefact that outlives the theory; a
 *   container that quietly altered it would defeat the only reason it exists.
 *
 *   The RECONCILIATION must say "no evidence" where it has none.  A zero
 *   mismatch count out of zero samples reads as success, and that is the single
 *   failure mode this whole phase is built against — so the empty-capture case
 *   is a test, not a comment.
 */
#include "hm_test.h"
#include "hm_wire.h"

#include "hackmotion/record.h"

#include <math.h>

#define TMP_PATH "test_record_tmp.hmwire"

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */
static hm_wire_chunk make_chunk(uint8_t direction, uint32_t sequence, hm_time_us t,
                                const uint8_t *data, size_t length)
{
    hm_wire_chunk c;
    memset(&c, 0, sizeof(c));
    c.direction = direction;
    c.sequence = sequence;
    c.host_time_us = t;
    c.length = (uint16_t)length;
    if (length > 0u) {
        memcpy(c.data, data, length);
    }
    return c;
}

/* The internal rate a synthetic session is generated at.  ⚠ Deliberately NOT
 * 799.2 and emphatically not 800: the test has to show the fit RECOVERING a
 * rate, which a test generated at the library's own seed could not. */
#define SYNTH_RATE_HZ 799.45

/* §6.5's measured ticks-per-sample. */
#define SYNTH_TICK_RATIO 80.166

/* §10.3's palm − lower_arm. */
#define SYNTH_SKEW_TICKS 59

typedef struct synth_options {
    int      records;          /* live records to emit                         */
    uint32_t step;             /* index step between them                      */
    int      dense_run;        /* how many step-1 records to splice in mid-way  */
    int16_t  arm_accel;        /* raw counts                                    */
    int16_t  palm_accel;
    int16_t  gyro;             /* raw counts on both units                      */
    bool     bringup;          /* emit §9.1's vendor sequence first             */
    hm_time_us keepalive_period_us; /* 0 → no keepalive writes                  */
    uint8_t  header_config_bits;    /* 0 → 0x7e; what the FILE HEADER claims     */
    /* ⚠ Deterministic ± jitter on the palm's tick counter, to give the
     * relative-rate fit a real residual spread.  Without one, every synthetic
     * capture has an impossibly precise standard error and the "this capture
     * cannot test the claim" path is unreachable. */
    int      palm_tick_jitter;
} synth_options;

static synth_options synth_defaults(void)
{
    synth_options o;
    memset(&o, 0, sizeof(o));
    o.records = 400;
    o.step = 32u;             /* §6.5's nominal 25 Hz decimation */
    o.dense_run = 0;
    o.arm_accel = 3000;       /* 29.4 m/s² */
    o.palm_accel = 8000;      /* 78.4 m/s² — §6.4's palm-at-larger-radius */
    o.gyro = 8000;            /* 1000 °/s under 0x7e's /8 divisor */
    o.bringup = true;
    o.keepalive_period_us = 0;
    return o;
}

static uint16_t ticks_for(uint32_t index, int offset)
{
    double t = (double)index * SYNTH_TICK_RATIO + (double)offset;
    return (uint16_t)((uint64_t)(t + 0.5) & 0xffffu);
}

/* Reproducible, so a failure is reproducible.  Range [-j, +j]. */
static int tick_jitter(uint32_t index, int j)
{
    uint32_t h;
    if (j <= 0) {
        return 0;
    }
    h = index * 2654435761u;
    h ^= h >> 15;
    return (int)(h % (uint32_t)(2 * j + 1)) - j;
}

/*
 * Writes a synthetic session to TMP_PATH.  Host times advance at SYNTH_RATE_HZ
 * so the clock fit has a real rate to recover.
 */
static void write_synthetic(const synth_options *o)
{
    hm_recording_info info = hm_recording_info_default();
    hm_recorder *rec = NULL;
    uint32_t seq = 0u;
    uint32_t index = 0u;
    hm_time_us t0 = 1000000;
    hm_time_us last_keepalive = t0;
    uint8_t buf[128];
    size_t n;

    memcpy(info.device_id, "synthetic", sizeof("synthetic"));
    if (o->header_config_bits != 0u) {
        info.config_bits = o->header_config_bits;
    }
    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, &info, &rec), HM_OK);

    if (o->bringup) {
        /* §9.1, verbatim.  Only step 2 is required of a client; the rest is the
         * vendor app's behaviour, kept because reproducing it is a useful
         * bring-up test (design §10.5). */
        static const uint8_t seqbytes[] = {0x80u, 0x81u, 0x84u, 0x81u,
                                           0x86u, 0x86u, 0x86u, 0x85u};
        for (size_t i = 0; i < sizeof(seqbytes); ++i) {
            hm_wire_chunk c =
                make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t0 + (hm_time_us)i * 1000,
                           &seqbytes[i], 1u);
            HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        }
    }

    {
        uint8_t start[3] = {0xa0u, 0x01u, HM_CONFIG_OBSERVED_DEFAULT};
        hm_wire_chunk c = make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t0 + 20000, start, 3u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    t0 += 20000 + 60000; /* §6.1: the first frame arrives within 50-80 ms */

    for (int i = 0; i < o->records; ++i) {
        hm_wire_block arm = hm_wire_identity_block(ticks_for(index, 0));
        hm_wire_block palm = hm_wire_identity_block(
            ticks_for(index, SYNTH_SKEW_TICKS + tick_jitter(index, o->palm_tick_jitter)));
        hm_time_us t;
        hm_wire_chunk c;

        arm.accel[0] = o->arm_accel;
        palm.accel[0] = o->palm_accel;
        arm.gyro[0] = o->gyro;
        palm.gyro[0] = o->gyro;

        n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
        t = t0 + (hm_time_us)((double)index * 1e6 / SYNTH_RATE_HZ + 0.5);

        if (o->keepalive_period_us > 0 && t - last_keepalive >= o->keepalive_period_us) {
            uint8_t poll = 0x81u;
            hm_wire_chunk k = make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t, &poll, 1u);
            HM_ASSERT_EQ(hm_recorder_write(rec, &k, 1u), HM_OK);
            last_keepalive = t;
        }

        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, buf, n);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);

        /* §6.6: bursts reach index step 1 — the full internal rate — and they
         * appear in every session containing motion. */
        if (o->dense_run > 0 && i == o->records / 2) {
            for (int d = 0; d < o->dense_run; ++d) {
                index += 1u;
                arm.ticks = ticks_for(index, 0);
                palm.ticks = ticks_for(
                    index, SYNTH_SKEW_TICKS + tick_jitter(index, o->palm_tick_jitter));
                n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
                t = t0 + (hm_time_us)((double)index * 1e6 / SYNTH_RATE_HZ + 0.5);
                c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, buf, n);
                HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
            }
        }
        index += o->step;
    }

    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);
}

static void reconcile_file(hm_reconcile_report *out)
{
    hm_replay *rp = NULL;
    hm_reconciler *rc = NULL;
    hm_wire_chunk c;
    hm_status st;

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_EQ(hm_reconcile_begin(hm_replay_info(rp), &rc), HM_OK);
    while ((st = hm_replay_next(rp, &c)) == HM_OK) {
        hm_reconcile_observe(rc, &c);
    }
    HM_ASSERT_EQ(st, HM_DONE);
    hm_reconcile_finish(rc, out);
    hm_reconcile_free(rc);
    hm_replay_close(rp);
}

/* ------------------------------------------------------------------------ */
/* The container                                                             */
/* ------------------------------------------------------------------------ */
HM_TEST(record_round_trips_every_chunk_field_byte_exactly)
{
    static const size_t lengths[] = {0u, 1u, 47u, 93u, 255u, HM_WIRE_CHUNK_MAX};
    hm_recorder *rec = NULL;
    hm_replay *rp = NULL;
    hm_wire_chunk written[6];
    hm_wire_chunk read_back;
    hm_recording_info info = hm_recording_info_default();

    for (size_t i = 0; i < 6u; ++i) {
        uint8_t payload[HM_WIRE_CHUNK_MAX];
        for (size_t b = 0; b < lengths[i]; ++b) {
            payload[b] = (uint8_t)(b * 7u + i);
        }
        /* ⚠ A negative host time is legal: the epoch is arbitrary and only
         * differences matter (types.h).  A container that assumed unsigned
         * would work on one host and corrupt a capture on another. */
        written[i] = make_chunk((uint8_t)(i % 3u), (uint32_t)(1000u + i),
                                (hm_time_us)-500000 + (hm_time_us)i * 1000, payload,
                                lengths[i]);
        written[i].flags = (uint8_t)((i == 2u) ? HM_WIRE_REDACTED : 0u);
    }

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, &info, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_write(rec, written, 6u), HM_OK);
    HM_ASSERT_EQ(hm_recorder_chunks(rec), 6u);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    for (size_t i = 0; i < 6u; ++i) {
        HM_ASSERT_EQ(hm_replay_next(rp, &read_back), HM_OK);
        HM_ASSERT_EQ(read_back.length, written[i].length);
        HM_ASSERT_EQ(read_back.direction, written[i].direction);
        HM_ASSERT_EQ(read_back.flags, written[i].flags);
        HM_ASSERT_EQ(read_back.sequence, written[i].sequence);
        HM_ASSERT_EQ(read_back.host_time_us, written[i].host_time_us);
        HM_ASSERT(memcmp(read_back.data, written[i].data, written[i].length) == 0);
    }
    HM_ASSERT_EQ(hm_replay_next(rp, &read_back), HM_DONE);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

/*
 * ⚠ The sequence number is the field design §5.6's sketch did not have.  It is
 * here because HM_WIRE_LOST says chunks were dropped but not HOW MANY: a reader
 * renumbering from its own ordinal would turn a lossy recording into a
 * complete-looking one, and nothing downstream would ever say so.
 */
HM_TEST(record_preserves_a_sequence_gap_rather_than_renumbering_it)
{
    hm_recorder *rec = NULL;
    hm_replay *rp = NULL;
    uint8_t byte = 0x90u;
    hm_wire_chunk c[2];
    hm_wire_chunk back;

    c[0] = make_chunk(HM_WIRE_DEVICE_TO_HOST, 10u, 1000, &byte, 1u);
    c[1] = make_chunk(HM_WIRE_DEVICE_TO_HOST, 900u, 2000, &byte, 1u);
    c[1].flags = HM_WIRE_LOST;

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_write(rec, c, 2u), HM_OK);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_EQ(hm_replay_next(rp, &back), HM_OK);
    HM_ASSERT_EQ(back.sequence, 10u);
    HM_ASSERT_EQ(hm_replay_next(rp, &back), HM_OK);
    HM_ASSERT_EQ(back.sequence, 900u);
    HM_ASSERT_EQ(back.flags & HM_WIRE_LOST, HM_WIRE_LOST);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

HM_TEST(record_header_round_trips_and_says_which_clock_was_used)
{
    hm_recorder *rec = NULL;
    hm_replay *rp = NULL;
    hm_recording_info info = hm_recording_info_default();

    memcpy(info.device_id, "wG3-under-test", sizeof("wG3-under-test"));
    info.config_bits = 0x5eu; /* ⚠ bit 5 clear: 20-byte blocks, no tick counter */
    info.identifiers_recorded = true;

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, &info, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_STR(hm_replay_info(rp)->device_id, "wG3-under-test");
    HM_ASSERT_EQ(hm_replay_info(rp)->config_bits, 0x5eu);
    HM_ASSERT_EQ(hm_replay_info(rp)->config_legacy, 0u);
    HM_ASSERT_EQ(hm_replay_info(rp)->layout_version, HM_SAMPLE_LAYOUT_VERSION);
    HM_ASSERT_EQ(hm_replay_info(rp)->identifiers_recorded, true);
    HM_ASSERT_STR(hm_replay_info(rp)->clock, HM_RECORD_CLOCK_MONOTONIC);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

/* The legacy `82` start takes no configuration byte at all, so it cannot be
 * spelled as a value of `config` (config.h) and gets its own word. */
HM_TEST(record_header_distinguishes_the_legacy_start_from_a_config_byte)
{
    hm_recorder *rec = NULL;
    hm_replay *rp = NULL;
    hm_recording_info info = hm_recording_info_default();

    info.config_legacy = 1u;
    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, &info, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);
    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_EQ(hm_replay_info(rp)->config_legacy, 1u);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

HM_TEST(record_refuses_a_file_that_is_not_a_recording)
{
    FILE *fp = fopen(TMP_PATH, "wb");
    hm_replay *rp = NULL;

    HM_ASSERT(fp != NULL);
    fputs("not a recording\n\n", fp);
    fclose(fp);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_ERR_MALFORMED);
    HM_ASSERT(rp == NULL);
    remove(TMP_PATH);
}

/* §5.1's rule for unknown message ids applies to unknown header keys for the
 * same reason: a recording written by a later version must stay readable up to
 * the part this version understands. */
HM_TEST(record_ignores_a_header_key_it_does_not_know)
{
    FILE *fp = fopen(TMP_PATH, "wb");
    hm_replay *rp = NULL;

    HM_ASSERT(fp != NULL);
    fputs(HM_RECORD_MAGIC "\n"
          "device_id=future\n"
          "config=0x7e\n"
          "layout_version=1\n"
          "clock=monotonic_us\n"
          "byte_order=little\n"
          "something_invented_in_2027=42\n"
          "\n",
          fp);
    fclose(fp);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_STR(hm_replay_info(rp)->device_id, "future");
    hm_replay_close(rp);
    remove(TMP_PATH);
}

/*
 * ⚠ The one place a corrupt or hostile file meets a fixed-size buffer.
 * CONTRIBUTING.md invites fuzzing exactly here, and the answer is a refusal
 * rather than a clamp: a clamp would keep decoding a file whose framing is
 * already known to be wrong.
 */
HM_TEST(record_refuses_a_length_above_the_chunk_maximum_rather_than_clamping)
{
    FILE *fp = fopen(TMP_PATH, "wb");
    hm_replay *rp = NULL;
    hm_wire_chunk c;
    uint8_t entry[HM_RECORD_ENTRY_HEADER];

    HM_ASSERT(fp != NULL);
    fputs(HM_RECORD_MAGIC "\nconfig=0x7e\nbyte_order=little\n\n", fp);
    memset(entry, 0, sizeof(entry));
    entry[0] = 0x01u; /* length = 0x00010001, far above HM_WIRE_CHUNK_MAX */
    entry[2] = 0x01u;
    fwrite(entry, 1, sizeof(entry), fp);
    fclose(fp);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_EQ(hm_replay_next(rp, &c), HM_ERR_MALFORMED);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

HM_TEST(record_reports_a_file_that_ends_mid_chunk)
{
    hm_recorder *rec = NULL;
    hm_replay *rp = NULL;
    hm_wire_chunk c;
    uint8_t payload[40];
    long size;
    FILE *fp;

    memset(payload, 0xa5, sizeof(payload));
    c = make_chunk(HM_WIRE_DEVICE_TO_HOST, 1u, 1000, payload, sizeof(payload));
    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    /* Chop the last ten bytes off — a capture killed mid-write. */
    fp = fopen(TMP_PATH, "rb");
    HM_ASSERT(fp != NULL);
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fclose(fp);
    {
        uint8_t *all = (uint8_t *)malloc((size_t)size);
        HM_ASSERT(all != NULL);
        fp = fopen(TMP_PATH, "rb");
        HM_ASSERT(fread(all, 1, (size_t)size, fp) == (size_t)size);
        fclose(fp);
        fp = fopen(TMP_PATH, "wb");
        fwrite(all, 1, (size_t)size - 10u, fp);
        fclose(fp);
        free(all);
    }

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_EQ(hm_replay_next(rp, &c), HM_ERR_TRUNCATED);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

HM_TEST(record_refuses_a_chunk_longer_than_the_wire_maximum)
{
    hm_recorder *rec = NULL;
    hm_wire_chunk c;
    uint8_t payload[1] = {0x90u};

    c = make_chunk(HM_WIRE_DEVICE_TO_HOST, 1u, 0, payload, 1u);
    c.length = (uint16_t)(HM_WIRE_CHUNK_MAX + 1u);
    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_ERR_INVALID_ARG);
    HM_ASSERT_EQ(hm_recorder_chunks(rec), 0u);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);
    remove(TMP_PATH);
}

/*
 * The whole reason for recording bytes rather than samples: a recording read
 * back must decode to exactly what the decoder saw live, so a later decode fix
 * can be applied to an old capture (design §5.6, api-request §2.9).
 */
HM_TEST(record_replays_a_frame_that_decodes_identically_to_the_original_bytes)
{
    hm_recorder *rec = NULL;
    hm_replay *rp = NULL;
    hm_wire_block arm = hm_wire_identity_block(1234u);
    hm_wire_block palm = hm_wire_identity_block(1293u);
    uint8_t buf[128];
    size_t n;
    hm_wire_chunk c, back;

    arm.gyro[1] = -4096;
    palm.accel[2] = 999;
    n = hm_wire_notification1(buf, 0x0040u, &arm, &palm);

    c = make_chunk(HM_WIRE_DEVICE_TO_HOST, 1u, 42, buf, n);
    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    HM_ASSERT_EQ(hm_replay_open(TMP_PATH, &rp), HM_OK);
    HM_ASSERT_EQ(hm_replay_next(rp, &back), HM_OK);
    HM_ASSERT_EQ(back.length, n);
    HM_ASSERT(memcmp(back.data, buf, n) == 0);
    hm_replay_close(rp);
    remove(TMP_PATH);
}

/* ------------------------------------------------------------------------ */
/* The reconciliation                                                        */
/* ------------------------------------------------------------------------ */
/*
 * ⚠⚠ THE HEADLINE TEST OF THIS PHASE.
 *
 * "No evidence" is not "agreement".  A capture with nothing in it must not
 * report zero mismatches, zero suspect norms and zero disagreements as though
 * the specification had been confirmed — every verdict must come back
 * HM_CHECK_NO_EVIDENCE and the unmeasured count must be non-zero.
 */
HM_TEST(reconcile_reports_no_evidence_rather_than_agreement_on_an_empty_capture)
{
    hm_recorder *rec = NULL;
    hm_reconcile_report rep;

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.chunks, 0u);
    HM_ASSERT_EQ(hm_reconcile_disagreements(&rep), 0);
    /* ⚠ Every claim, and the COUNT is pinned deliberately: adding a check
     * without deciding what it reports on an empty capture is how a claim ends
     * up silently defaulting to "match". */
    HM_ASSERT_EQ(hm_reconcile_unmeasured(&rep), 11);
    HM_ASSERT_EQ(rep.verdict_frame_length, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_quat_norm, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_rate, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_tick_ratio, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_skew, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_bursts, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_palm_is_second_block, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_bringup, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_keepalive, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_history_rate, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT_EQ(rep.verdict_retrieval_continuity, HM_CHECK_NO_EVIDENCE);
    /* ⚠ And the counts that would otherwise read as clean. */
    HM_ASSERT_EQ(rep.quat_norm[HM_UNIT_LOWER_ARM].n, 0u);
    HM_ASSERT_EQ(rep.skew_stored, 0u);
    remove(TMP_PATH);
}

/*
 * §6.5: the rate must be FITTED, never assumed.  The synthetic session runs at
 * 799.45 Hz — neither the library's 799.2 seed nor the round 800 — so a fit
 * that recovered a constant instead of a measurement would fail here.
 */
HM_TEST(reconcile_fits_the_sample_rate_rather_than_adopting_a_constant)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    write_synthetic(&o);
    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.verdict_rate, HM_CHECK_MATCH);
    HM_ASSERT(rep.fit.observations > 300);
    HM_ASSERT_NEAR(rep.fitted_rate_hz, SYNTH_RATE_HZ, 0.05);
    /* ⚠ And it is not the seed and not the round number. */
    HM_ASSERT(fabs(rep.fitted_rate_hz - HM_NOMINAL_SAMPLE_RATE_HZ) > 0.1);
    HM_ASSERT(rep.ppm_vs_800 < -600.0);
    HM_ASSERT((rep.fit.flags & HM_CLOCK_SHORT_BASELINE) == 0u);
    HM_ASSERT((rep.fit.flags & HM_CLOCK_DEGENERATE) == 0u);
    remove(TMP_PATH);
}

/* §6.5's ratio and §6.4's derived tick rate, both recovered from the frames. */
HM_TEST(reconcile_recovers_the_tick_ratio_and_the_derived_tick_rate)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    write_synthetic(&o);
    reconcile_file(&rep);

    HM_ASSERT(rep.tick_ratio_fitted[HM_UNIT_LOWER_ARM]);
    HM_ASSERT(rep.tick_ratio_fitted[HM_UNIT_PALM]);
    HM_ASSERT_NEAR(rep.ticks_per_index[HM_UNIT_LOWER_ARM], SYNTH_TICK_RATIO, 0.01);
    HM_ASSERT_NEAR(rep.ticks_per_index[HM_UNIT_PALM], SYNTH_TICK_RATIO, 0.01);
    /* §6.4: ≈64,068 ticks/s, four sessions spanning 64,025-64,088. */
    HM_ASSERT_NEAR(rep.tick_rate_hz, 64068.0, 100.0);
    HM_ASSERT_EQ(rep.verdict_tick_ratio, HM_CHECK_MATCH);
    /* ⚠ Every point in the stream, not two endpoints — see the test below. */
    HM_ASSERT_EQ(rep.tick_fit_n, rep.records_live);
    HM_ASSERT(rep.rel_rate_measured);
    HM_ASSERT_NEAR(rep.rel_rate_ppm, 0.0, 2.0);
    remove(TMP_PATH);
}

/*
 * ⚠⚠ THE FIRST REAL CAPTURE REPORTED THIS CLAIM AS A VIOLATION, AND IT WAS NOT.
 *
 * The reconciliation used hm_tick_unwrapper's own `ratio` as the measurement.
 * That is a two-endpoint fit refitted on doubling, and it exists to predict a
 * tick value well enough to pick the right wrap out of a ±32,768 budget — a job
 * a 350 ppm error still does perfectly.  Read as a precision instrument it put
 * the two units 17.21 ppm apart on a capture where a least-squares fit over all
 * 1,572 records puts them 0.43 ± 1.26 ppm apart, and the report accused the
 * device of breaking §6.5.
 *
 * The report contradicted itself and nobody would have had to notice: the skew
 * medians on the very next line were stable at 59 → 58 ticks, which bounds the
 * relative rate at about 1 ppm all by itself.
 *
 * So: the measurement is a least-squares slope of (palm − arm) against index,
 * which is zero iff the two counters run at one rate, and it carries a standard
 * error because a ppm figure without one cannot test a claim stated in ppm.
 */
HM_TEST(reconcile_measures_the_tick_ratio_itself_not_the_unwrappers_working_estimate)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    o.records = 1200;
    write_synthetic(&o);
    reconcile_file(&rep);

    /* The fit uses every live record, not two endpoints. */
    HM_ASSERT_EQ(rep.tick_fit_n, 1200u);
    HM_ASSERT(rep.rel_rate_measured);
    /* Both counters advance at exactly one rate here, so the slope is zero and
     * the estimate must land on it well inside §6.5's claim. */
    HM_ASSERT(fabs(rep.rel_rate_ppm) < HM_ONE_RATE_CLAIM_PPM);
    HM_ASSERT(rep.rel_rate_ppm_sigma < HM_ONE_RATE_CLAIM_PPM);
    HM_ASSERT_EQ(rep.verdict_tick_ratio, HM_CHECK_MATCH);
    remove(TMP_PATH);
}

/*
 * ⚠ A capture whose own noise exceeds the claim cannot test the claim.
 *
 * §6.5's 2 ppm came from a 238 s session.  The standard error on the same
 * measurement scales with the baseline — ±1.26 ppm over the 30 s bench capture
 * — so a short or noisy one could not have DETECTED a violation, and reporting
 * "match" from it would be claiming a confirmation that was never on offer.
 * That is the same error as reading zero mismatches out of zero samples as
 * agreement, and it gets the same answer: no evidence.
 */
HM_TEST(reconcile_refuses_to_confirm_two_ppm_from_a_capture_that_cannot_resolve_it)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    o.records = 60;           /* a short baseline ... */
    o.palm_tick_jitter = 6;   /* ... and a real residual spread on the difference */
    write_synthetic(&o);
    reconcile_file(&rep);

    HM_ASSERT(rep.rel_rate_measured);
    HM_ASSERT(rep.rel_rate_ppm_sigma > HM_ONE_RATE_CLAIM_PPM);
    /* ⚠ Not MATCH, and not DIFFERS.  The capture simply cannot say. */
    HM_ASSERT_EQ(rep.verdict_tick_ratio, HM_CHECK_NO_EVIDENCE);
    HM_ASSERT(hm_reconcile_unmeasured(&rep) > 0);
    remove(TMP_PATH);
}

/*
 * ⚠ And it must still be able to CATCH a real violation — a check that only
 * ever says "no evidence" is not a check.  Two counters genuinely running at
 * different rates show up as a non-zero slope well clear of its own sigma.
 */
HM_TEST(reconcile_still_catches_two_counters_that_genuinely_drift_apart)
{
    hm_recording_info info = hm_recording_info_default();
    hm_recorder *rec = NULL;
    hm_reconcile_report rep;
    uint8_t buf[128];
    uint32_t seq = 0u, index = 0u;
    hm_time_us t0 = 1000000;
    size_t n;
    hm_wire_chunk c;

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, &info, &rec), HM_OK);
    {
        uint8_t start[3] = {0xa0u, 0x01u, HM_CONFIG_OBSERVED_DEFAULT};
        c = make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t0, start, 3u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    for (int i = 0; i < 1200; ++i) {
        /* The palm counter runs 100 ppm fast — 50× §6.5's claim, and still a
         * number no per-sample inspection would ever show you. */
        double palm_ticks = (double)index * SYNTH_TICK_RATIO * 1.0001 + SYNTH_SKEW_TICKS;
        hm_wire_block arm = hm_wire_identity_block(ticks_for(index, 0));
        hm_wire_block palm =
            hm_wire_identity_block((uint16_t)((uint64_t)(palm_ticks + 0.5) & 0xffffu));
        hm_time_us t = t0 + (hm_time_us)((double)index * 1e6 / SYNTH_RATE_HZ + 0.5);

        n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, buf, n);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        index += 32u;
    }
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    reconcile_file(&rep);
    HM_ASSERT(rep.rel_rate_measured);
    HM_ASSERT_NEAR(rep.rel_rate_ppm, 100.0, 5.0);
    HM_ASSERT_EQ(rep.verdict_tick_ratio, HM_CHECK_DIFFERS);
    /* ⚠ And the skew must show it too — the two halves can no longer agree,
     * which is the cross-check that exposed the estimator bug in the first
     * place.  A drift the ratio check caught and the skew check missed would
     * mean one of them had stopped working. */
    HM_ASSERT_EQ(rep.verdict_skew, HM_CHECK_DIFFERS);
    remove(TMP_PATH);
}

/*
 * §10.3's skew.  ⚠ The claim under test is STABILITY across the session, which
 * is why the reconciler keeps the values rather than reducing them to a running
 * median: you cannot split a session you did not keep.
 */
HM_TEST(reconcile_recovers_the_inter_unit_skew_and_checks_it_across_both_halves)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    write_synthetic(&o);
    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.skew_stored, rep.skew_total);
    HM_ASSERT(rep.skew_stored > 300u);
    HM_ASSERT_NEAR(rep.skew_median_ticks, (double)SYNTH_SKEW_TICKS, 1.0);
    HM_ASSERT_NEAR(rep.skew_median_first_half, rep.skew_median_second_half, 1.0);
    HM_ASSERT_EQ(rep.verdict_skew, HM_CHECK_MATCH);
    /* 59 ticks at ≈64,068 ticks/s is 0.92 ms. */
    HM_ASSERT_NEAR(rep.skew_median_us, 921.0, 20.0);
    remove(TMP_PATH);
}

/*
 * §6.3's physical check.  ⚠ A negative mean means wire block 0 and block 1 are
 * swapped, which produces a plausible but MIRRORED wrist angle that every
 * ordinary plausibility check passes.  The reconciliation must call it.
 */
HM_TEST(reconcile_detects_two_unit_blocks_that_have_been_swapped)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.verdict_palm_is_second_block, HM_CHECK_MATCH);
    HM_ASSERT(rep.accel_palm_minus_arm_fast.n > 300u);
    /* §6.4 measured 31-51 m/s² more across five golf swings. */
    HM_ASSERT_NEAR(rep.accel_palm_minus_arm_fast.mean, 49.0, 1.0);

    /* Now the same session with the blocks the wrong way round. */
    o.arm_accel = 8000;
    o.palm_accel = 3000;
    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.verdict_palm_is_second_block, HM_CHECK_DIFFERS);
    HM_ASSERT(rep.accel_palm_minus_arm_fast.mean < 0.0);
    remove(TMP_PATH);
}

/* ⚠ And it must report NO EVIDENCE, not agreement, when nothing in the capture
 * rotated fast enough to separate the two units. */
HM_TEST(reconcile_will_not_identify_the_palm_from_a_capture_with_no_motion)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    o.gyro = 0;
    write_synthetic(&o);
    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.accel_palm_minus_arm_fast.n, 0u);
    HM_ASSERT_EQ(rep.verdict_palm_is_second_block, HM_CHECK_NO_EVIDENCE);
    remove(TMP_PATH);
}

/*
 * §6.6: 25 and 100 Hz are two strong modes of a continuum, not a two-state
 * switch, and dense steps reach index step 1.  A capture with no motion is pure
 * +32 and is §6.6's own stationary row — which confirms nothing and
 * contradicts nothing.
 */
HM_TEST(reconcile_separates_a_dense_burst_from_a_stationary_session)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.step[HM_STEP_DENSE], 0u);
    HM_ASSERT(rep.step[HM_STEP_32] > 300u);
    HM_ASSERT_EQ(rep.verdict_bursts, HM_CHECK_NOTE);

    o.dense_run = 8; /* §6.6 measured a run of eight consecutive steps of 1 */
    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.step[HM_STEP_DENSE], 8u);
    HM_ASSERT_EQ(rep.verdict_bursts, HM_CHECK_MATCH);
    HM_ASSERT(rep.dense_fraction > 0.0);
    remove(TMP_PATH);
}

/*
 * ⚠ THE BRACKET.  Live and history 0x90 frames are byte-identical (§10.1) and
 * the `a1 02` … `a1 01` bracket is the only discriminator.  History records
 * must be counted separately AND must never reach the clock fit: their arrival
 * times carry no information, and ~4,000 bulk arrivals fed into the fit would
 * wreck the rate while every frame still parsed.
 */
HM_TEST(reconcile_keeps_bracketed_history_records_out_of_the_clock_fit)
{
    hm_recorder *rec = NULL;
    hm_reconcile_report rep;
    uint8_t buf[128];
    uint32_t seq = 0u;
    hm_time_us t = 1000000;
    size_t n;
    hm_wire_chunk c;
    int i;

    /* A short live stream, then a bracket full of records that all arrive at
     * once — the shape a real retrieval has (§7.3: ~260 notifications/s). */
    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    {
        uint8_t start[3] = {0xa0u, 0x01u, HM_CONFIG_OBSERVED_DEFAULT};
        c = make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t, start, 3u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    for (i = 0; i < 200; ++i) {
        hm_wire_block arm = hm_wire_identity_block(ticks_for((uint32_t)i * 32u, 0));
        hm_wire_block palm = hm_wire_identity_block(ticks_for((uint32_t)i * 32u, SYNTH_SKEW_TICKS));
        n = hm_wire_notification1(buf, (uint16_t)(i * 32), &arm, &palm);
        t = 1000000 + (hm_time_us)((double)(i * 32) * 1e6 / SYNTH_RATE_HZ + 0.5);
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, buf, n);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }

    {
        /* ⚠ §7.2's LEADING `a1 01`, which closes a PREVIOUS retrieval.  Reading
         * it as our closing marker is one of the four bracket confusions in
         * implementation-notes §5. */
        uint8_t lead[2] = {0xa1u, 0x01u};
        uint8_t open[2] = {0xa1u, 0x02u};
        uint8_t close[2] = {0xa1u, 0x01u};
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, lead, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, open, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);

        for (i = 0; i < 500; ++i) {
            /* Indices BEHIND live, all arriving in a few milliseconds. */
            hm_wire_block arm = hm_wire_identity_block(ticks_for((uint32_t)i, 0));
            hm_wire_block palm = hm_wire_identity_block(ticks_for((uint32_t)i, SYNTH_SKEW_TICKS));
            n = hm_wire_notification1(buf, (uint16_t)i, &arm, &palm);
            c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t + i * 4, buf, n);
            HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        }
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t + 2000, close, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.records_live, 200u);
    HM_ASSERT_EQ(rep.records_history, 500u);
    /* ⚠ The fit saw the 200 live frames and NONE of the 500 history ones. */
    HM_ASSERT_EQ(rep.fit.observations, 200);
    HM_ASSERT_EQ(rep.brackets_opened, 1u);
    HM_ASSERT_EQ(rep.brackets_closed, 1u);
    /* The leading `a1 01` was not counted as closing anything of ours. */
    HM_ASSERT_NEAR(rep.fitted_rate_hz, SYNTH_RATE_HZ, 0.5);
    /* And the history records still contributed their skew, which needs no
     * unwrapper and works from the very first record of a stream (§10.3). */
    HM_ASSERT_EQ(rep.skew_stored, 700u);

    remove(TMP_PATH);
}

/*
 * ⚠ The configuration comes from the wire, not from the file header.  Two of
 * these bits change the WIRE FORMAT (§6.2), so a decoder told the wrong one
 * produces a differently-shaped payload silently misparsed.
 */
HM_TEST(reconcile_takes_the_configuration_from_the_recorded_start_command)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    /* ⚠ A header that claims 0x5e — 20-byte blocks, no tick counter — while the
     * wire carries `a0 01 7e`.  Believing the header would size every record
     * four bytes short and misparse the entire capture. */
    o.header_config_bits = 0x5eu;
    o.records = 60;
    write_synthetic(&o);
    reconcile_file(&rep);

    HM_ASSERT(rep.config_from_stream);
    HM_ASSERT_EQ(rep.config.bits, HM_CONFIG_OBSERVED_DEFAULT);
    HM_ASSERT_EQ(rep.expected_len_one_record, 47u);
    HM_ASSERT_EQ(rep.expected_len_two_records, 93u);
    HM_ASSERT_EQ(rep.notif_other_len, 0u);
    HM_ASSERT_EQ(rep.verdict_frame_length, HM_CHECK_MATCH);
    remove(TMP_PATH);
}

/* §9.1's sequence, and §9.2's mandatory keepalive. */
HM_TEST(reconcile_reads_the_bringup_sequence_and_the_keepalive_gap)
{
    synth_options o = synth_defaults();
    hm_reconcile_report rep;

    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.bringup_len, 8u);
    HM_ASSERT(rep.bringup_matches_vendor);
    HM_ASSERT_EQ(rep.verdict_bringup, HM_CHECK_MATCH);
    HM_ASSERT_EQ(rep.status_polls, 2u); /* §9.1 sends `81` twice */
    /* ⚠ The bring-up bytes only; the start command that follows is not part of
     * §9.1's sequence and must not be appended to it. */
    HM_ASSERT_EQ(rep.bringup[7], 0x85u);

    /* A 50-minute session polling `0x81` every 30 s, which is what §9.2
     * measured surviving 7 minutes where a silent connection died at 5.0. */
    o.bringup = false;
    o.records = 3000;
    o.step = 800u; /* ~1 s of internal samples per record */
    o.keepalive_period_us = 30 * 1000 * 1000;
    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.verdict_keepalive, HM_CHECK_MATCH);
    HM_ASSERT(rep.status_polls > 90u);

    /* ⚠ The same session polling every 60 s.  §9.2 is measured, not advisory:
     * the vendor app polls every 30 s and the device drops a connection that
     * has not been written to for 5.0 minutes — an active stream does NOT
     * prevent it. */
    o.keepalive_period_us = 60 * 1000 * 1000;
    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.verdict_keepalive, HM_CHECK_DIFFERS);
    HM_ASSERT(rep.max_host_write_gap_us > 33 * 1000 * 1000);

    /* ⚠ And one write is not evidence of a keepalive, in either direction. */
    o.keepalive_period_us = 0;
    o.records = 20;
    write_synthetic(&o);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.host_writes, 1u);
    HM_ASSERT_EQ(rep.verdict_keepalive, HM_CHECK_NO_EVIDENCE);
    remove(TMP_PATH);
}

/* ------------------------------------------------------------------------ */
/* §7.3 — the motion-adaptive buffer                                         */
/* ------------------------------------------------------------------------ */
/*
 * Writes a capture whose only retrieval returns records at `step`, each
 * carrying `gyro_raw` counts of angular rate on both units.
 */
static void write_retrieval(uint32_t step, int16_t gyro_raw, int records)
{
    hm_recorder *rec = NULL;
    uint8_t buf[128];
    uint32_t seq = 0u, index = 1000u;
    hm_time_us t = 1000000;
    hm_wire_chunk c;
    size_t n;

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    {
        uint8_t start[3] = {0xa0u, 0x01u, HM_CONFIG_OBSERVED_DEFAULT};
        c = make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t, start, 3u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    {
        uint8_t open[2] = {0xa1u, 0x02u};
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, open, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    for (int i = 0; i < records; ++i) {
        hm_wire_block arm = hm_wire_identity_block(ticks_for(index, 0));
        hm_wire_block palm = hm_wire_identity_block(ticks_for(index, SYNTH_SKEW_TICKS));
        arm.gyro[0] = gyro_raw;
        palm.gyro[0] = gyro_raw;
        n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t + i * 4, buf, n);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        index += step;
    }
    {
        uint8_t close[2] = {0xa1u, 0x01u};
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t + 5000, close, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);
}

/*
 * ⚠⚠ THE MISTAKE THIS PROJECT ACTUALLY MADE, KEPT AS A TEST.
 *
 * The first retrieval ever performed here came back as 503 records at a uniform
 * index step of 8 over a 4,000-index window — 12.5% density — and it read like
 * a fault in the device.  It was the device working perfectly: §7.3's buffer is
 * MOTION-ADAPTIVE, holding ~100 Hz while the wrist is still and reaching the
 * full ≈799.2 Hz only in fast motion, and the wrist had been still.  Median
 * angular rate over that retrieval was 0.7 °/s.
 *
 * Two hypotheses were drawn from it — a per-request record cap, and a buffer
 * that only ever holds 100 Hz — and BOTH were wrong.  The experiment proposed
 * to settle it swept the request WIDTH, which is the variable that does not
 * matter; run at a desk it returns ~100 Hz at every width and reads as proof
 * that the design's premise was gone.
 *
 * So a stationary retrieval must report NO EVIDENCE.  Not a match, which would
 * certify a full-rate path nobody exercised; not a failure, which would blame
 * the device for the bench.
 */
HM_TEST(reconcile_will_not_certify_the_full_rate_path_from_a_stationary_retrieval)
{
    hm_reconcile_report rep;

    /* 8000 raw / 8 = 1000 °/s would be fast; this is 1.0 °/s — a desk. */
    write_retrieval(8u, 8, 400);
    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.records_history, 400u);
    HM_ASSERT_EQ(rep.history_modal_step, 8u);
    HM_ASSERT(!rep.history_exercised_full_rate);
    HM_ASSERT_EQ(rep.verdict_history_rate, HM_CHECK_NO_EVIDENCE);
    /* ⚠ And it must not be quietly folded into the disagreement count. */
    HM_ASSERT_EQ(hm_reconcile_disagreements(&rep), 0);
    HM_ASSERT(hm_reconcile_unmeasured(&rep) > 0);
    remove(TMP_PATH);
}

/* The other half: a retrieval over real motion returns step 1, and THAT is
 * what demonstrates the full-rate path §7.6's whole premise rests on. */
HM_TEST(reconcile_certifies_the_full_rate_path_from_a_retrieval_over_fast_motion)
{
    hm_reconcile_report rep;

    /* 8000 counts / 8 = 1000 °/s, in §7.3's step-1 band of 780-850 and above. */
    write_retrieval(1u, 8000, 400);
    reconcile_file(&rep);

    HM_ASSERT_EQ(rep.history_modal_step, 1u);
    HM_ASSERT(rep.history_exercised_full_rate);
    HM_ASSERT(rep.history_peak_gyro_dps > 900.0);
    HM_ASSERT_EQ(rep.verdict_history_rate, HM_CHECK_MATCH);
    remove(TMP_PATH);
}

/* §7.3's two falsifiable claims: across 17,739 measured steps none exceeded 8
 * and none was 0.  A step of 32 in a retrieval would break the first. */
HM_TEST(reconcile_flags_a_history_step_above_the_documented_floor)
{
    hm_reconcile_report rep;

    write_retrieval(32u, 8000, 400);
    reconcile_file(&rep);

    HM_ASSERT(rep.history_step_count[9] > 0u);
    HM_ASSERT_EQ(rep.verdict_history_rate, HM_CHECK_DIFFERS);
    HM_ASSERT(hm_reconcile_disagreements(&rep) > 0);
    remove(TMP_PATH);
}

/*
 * ⚠⚠ §7.5 SAYS A MID-STREAM RETRIEVAL COSTS NO RECORDING GAP.  IT DOES.
 *
 * Measured across six retrievals on hardware: the sample counter advanced by
 * 4-35 indices while the MCU tick timer advanced ~22,400 — about 350 ms, the
 * bracket's own duration.  Excluding the six live pairs that straddle a bracket
 * moved ticks-per-index from 83.3988 to 80.1407 against §6.5's ~80.14, so those
 * six pairs carried the entire distortion.
 *
 * The consequence is not cosmetic and this test exists so it cannot be
 * forgotten: the index→host-time mapping gains a TIME OFFSET at every pull, so
 * one line per stream is wrong across a retrieval, and the clock fit reports
 * itself DEGENERATE with 1.7 s residuals rather than misaligning quietly.
 */
HM_TEST(reconcile_detects_the_sample_counter_stalling_across_a_retrieval)
{
    hm_recorder *rec = NULL;
    hm_reconcile_report rep;
    uint8_t buf[128];
    uint32_t seq = 0u, index = 0u;
    hm_time_us t = 1000000;
    hm_wire_chunk c;
    size_t n;
    int i;

    HM_ASSERT_EQ(hm_recorder_open(TMP_PATH, NULL, &rec), HM_OK);
    {
        uint8_t start[3] = {0xa0u, 0x01u, HM_CONFIG_OBSERVED_DEFAULT};
        c = make_chunk(HM_WIRE_HOST_TO_DEVICE, seq++, t, start, 3u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    /* 200 live records, then a bracket, then 200 more.  ⚠ Across the bracket
     * the TICKS advance by 350 ms worth and the INDEX does not — which is what
     * the device did. */
    for (i = 0; i < 200; ++i) {
        hm_wire_block arm = hm_wire_identity_block(ticks_for(index, 0));
        hm_wire_block palm = hm_wire_identity_block(ticks_for(index, SYNTH_SKEW_TICKS));
        n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
        t = 1000000 + (hm_time_us)((double)index * 1e6 / SYNTH_RATE_HZ + 0.5);
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, buf, n);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        index += 32u;
    }
    {
        uint8_t open_m[2] = {0xa1u, 0x02u};
        uint8_t close_m[2] = {0xa1u, 0x01u};
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, open_m, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        for (i = 0; i < 100; ++i) {
            hm_wire_block a2 = hm_wire_identity_block(ticks_for((uint32_t)i, 0));
            hm_wire_block p2 = hm_wire_identity_block(ticks_for((uint32_t)i, SYNTH_SKEW_TICKS));
            n = hm_wire_notification1(buf, (uint16_t)i, &a2, &p2);
            c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t + i, buf, n);
            HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        }
        t += 350000;
        c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, close_m, 2u);
        HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
    }
    /* ⚠ The stall: 350 ms of ticks, ~22,400 at 64,068 ticks/s, and the index
     * moves on by one ordinary live step instead of ~280. */
    {
        uint32_t tick_base = (uint32_t)(80.166 * (double)index) + 22424u;
        for (i = 0; i < 200; ++i) {
            uint32_t off = (uint32_t)(80.166 * 32.0 * (double)i);
            hm_wire_block arm = hm_wire_identity_block((uint16_t)((tick_base + off) & 0xffffu));
            hm_wire_block palm =
                hm_wire_identity_block((uint16_t)((tick_base + off + SYNTH_SKEW_TICKS) & 0xffffu));
            index += 32u;
            n = hm_wire_notification1(buf, (uint16_t)index, &arm, &palm);
            t += 40000;
            c = make_chunk(HM_WIRE_DEVICE_TO_HOST, seq++, t, buf, n);
            HM_ASSERT_EQ(hm_recorder_write(rec, &c, 1u), HM_OK);
        }
    }
    HM_ASSERT_EQ(hm_recorder_close(rec), HM_OK);

    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.retrievals_measured, 1u);
    HM_ASSERT_EQ(rep.retrievals_stalled, 1u);
    HM_ASSERT_EQ(rep.verdict_retrieval_continuity, HM_CHECK_DIFFERS);
    /*
     * ⚠ THE CLAIM IS THAT THE GAP EQUALS THE PULL, so the fraction is what is
     * asserted.  The absolute figure carries the one live step that measures it
     * (32 indices, 40 ms) and the synthetic's own tick rounding; pinning it to
     * the millisecond would be pinning the fixture, not the property.
     */
    HM_ASSERT_NEAR(rep.retrieval_stall_fraction.mean, 1.0, 0.3);
    HM_ASSERT(rep.retrieval_stall_ms.mean > 250.0);
    HM_ASSERT(rep.retrieval_indices_lost.mean > 200.0);
    /* Cumulative, because that is the figure that reaches a wrap boundary.
     * ⚠ One real stall measured ~23,500 ticks — 72% of the ±32,768 that decides
     * a wrap, where a pull-free gap uses 12 — so a single pull spends most of
     * the budget and two in one live-frame gap exceed it (§10.2). */
    HM_ASSERT(rep.stall_total_ticks > 15000.0);
    HM_ASSERT_NEAR(rep.stall_worst_ticks, rep.stall_total_ticks, 1.0);
    remove(TMP_PATH);
}

/* The other side: a retrieval that does NOT stall must not be reported as one,
 * or the check would fire on every capture and stop meaning anything. */
HM_TEST(reconcile_does_not_invent_a_stall_where_the_counter_kept_running)
{
    hm_reconcile_report rep;

    /* write_retrieval() brackets records whose ticks and indices stay in step. */
    write_retrieval(1u, 8000, 200);
    reconcile_file(&rep);
    HM_ASSERT_EQ(rep.retrievals_stalled, 0u);
    HM_ASSERT(rep.verdict_retrieval_continuity != HM_CHECK_DIFFERS);
    remove(TMP_PATH);
}

HM_TEST_MAIN()
