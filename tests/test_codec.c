/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_codec.c — the wire decoder against hand-computed golden vectors.
 *
 * Everything here traces to a numbered section of docs/specification.md, and
 * the test names say which.  A failure should point at the paragraph it
 * contradicts.
 */
#include "hm_test.h"
#include "hm_wire.h"
#include "fixtures/hm_fixtures.h"

#include "hm_codec.h"
#include "hackmotion/quat.h"
#include "hackmotion/event.h"

/* §1: byte order is NOT uniform across this protocol; the stream is big-endian. */
HM_TEST(codec_stream_fields_are_big_endian)
{
    hm_decoded d;
    hm_status st = hm_codec_decode(HM_FIX_FRAME_7E, sizeof(HM_FIX_FRAME_7E),
                                   hm_stream_config_default(), &d);
    HM_ASSERT_EQ(st, HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_FRAME);
    HM_ASSERT_EQ(d.u.frame.count, 1);
    HM_ASSERT_EQ(d.consumed, 47);

    HM_ASSERT_EQ(d.u.frame.sample[0].sample_index_raw, 0x0123);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.q_world_to_body_raw[0], 16384);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.linear_accel_raw[0], 100);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.linear_accel_raw[1], -200);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.gyro_raw[0], 800);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.gyro_raw[2], -1600);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.ticks_raw, 0x1234);
    HM_ASSERT_EQ(d.u.frame.sample[0].palm.ticks_raw, 0x126F);
}

/* §6.4: quaternion /16384, accel × 0.0098 m/s², gyro /8 with config bit 6. */
HM_TEST(codec_scales_match_section_6_4)
{
    hm_decoded d;
    const hm_unit_sample *arm;
    (void)hm_codec_decode(HM_FIX_FRAME_7E, sizeof(HM_FIX_FRAME_7E),
                          hm_stream_config_default(), &d);
    arm = &d.u.frame.sample[0].lower_arm;

    HM_ASSERT_NEAR(arm->q_world_to_body[0], 1.0f, 1e-6);
    HM_ASSERT_NEAR(arm->linear_accel_mps2[0], 0.98f, 1e-5);
    HM_ASSERT_NEAR(arm->linear_accel_mps2[1], -1.96f, 1e-5);
    HM_ASSERT_NEAR(arm->gyro_dps[0], 100.0f, 1e-4);   /* 800 / 8 */
    HM_ASSERT_NEAR(arm->gyro_dps[2], -200.0f, 1e-4);  /* -1600 / 8 */
}

/*
 * ⚠ §6.2: config bit 6 halves the gyro scale.  A recording decoded under the
 * wrong divisor is silently wrong — which is why api-request §2.3 insists the
 * raw counts and the configuration byte travel with the data.
 */
HM_TEST(codec_gyro_divisor_follows_config_bit_6)
{
    hm_decoded d;
    hm_stream_config cfg = hm_stream_config_nonstandard(0x3e, "test: bit 6 clear");

    /* 0x3e keeps bit 5 (ticks) so the layout is unchanged; only the scale moves. */
    (void)hm_codec_decode(HM_FIX_FRAME_7E, sizeof(HM_FIX_FRAME_7E), cfg, &d);
    HM_ASSERT_EQ(hm_stream_config_gyro_divisor(cfg), 16);
    HM_ASSERT_NEAR(d.u.frame.sample[0].lower_arm.gyro_dps[0], 50.0f, 1e-4);
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.gyro_raw[0], 800); /* raw is unchanged */
    HM_ASSERT(d.u.frame.sample[0].flags & HM_SAMPLE_NONSTANDARD_CONFIG);
}

/* ⚠ §6.2: bit 5 changes the BLOCK SIZE, not just the content. */
HM_TEST(codec_config_bit_5_changes_the_wire_format)
{
    hm_stream_config with = hm_stream_config_default();          /* 0x7e */
    hm_stream_config without = hm_stream_config_nonstandard(0x5e, "test: 5e");

    HM_ASSERT_EQ(hm_stream_config_block_size(with), 22);
    HM_ASSERT_EQ(hm_stream_config_record_size(with), 46);
    HM_ASSERT(hm_stream_config_has_ticks(with));

    HM_ASSERT_EQ(hm_stream_config_block_size(without), 20);
    HM_ASSERT_EQ(hm_stream_config_record_size(without), 42);
    HM_ASSERT(!hm_stream_config_has_ticks(without));
}

/* §6.3: the FIRST block is the lower arm, the SECOND is the palm. */
HM_TEST(codec_block_order_is_lower_arm_then_palm)
{
    hm_decoded d;
    (void)hm_codec_decode(HM_FIX_FRAME_7E, sizeof(HM_FIX_FRAME_7E),
                          hm_stream_config_default(), &d);

    /* The fixture gives the arm w=1 and the palm a 90° rotation; if the blocks
     * were swapped both of these would fail. */
    HM_ASSERT_NEAR(d.u.frame.sample[0].lower_arm.q_world_to_body[0], 1.0f, 1e-6);
    HM_ASSERT_NEAR(d.u.frame.sample[0].palm.q_world_to_body[0], 0.70709f, 1e-4);
    HM_ASSERT_EQ(hm_sample_unit(&d.u.frame.sample[0], HM_UNIT_LOWER_ARM),
                 &d.u.frame.sample[0].lower_arm);
    HM_ASSERT_EQ(hm_sample_unit(&d.u.frame.sample[0], HM_UNIT_PALM),
                 &d.u.frame.sample[0].palm);
}

/* §6.3: a notification carries one OR two records — 47 or 93 bytes. */
HM_TEST(codec_accepts_two_record_notifications)
{
    uint8_t buf[128];
    hm_wire_block a = hm_wire_identity_block(1000);
    hm_wire_block b = hm_wire_identity_block(1059);
    hm_decoded d;
    size_t n = hm_wire_notification2(buf, 100, &a, &b, 108, &a, &b);

    HM_ASSERT_EQ(n, 93);
    HM_ASSERT_EQ(hm_codec_decode(buf, n, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT_EQ(d.u.frame.count, 2);
    HM_ASSERT_EQ(d.consumed, 93);
    HM_ASSERT_EQ(d.u.frame.sample[0].sample_index_raw, 100);
    HM_ASSERT_EQ(d.u.frame.sample[1].sample_index_raw, 108); /* the +8 100 Hz step */
}

/*
 * ⚠ THE RECORD COUNT COMES FROM THE LENGTH, NOT FROM THE CONTENT.
 *
 * One call, one notification (§2.4's MTU floor guarantees the 93-byte maximum
 * message fits in one).  §6.4's norm check is reported as evidence that the
 * decode is ALIGNED, and never used to decide a frame boundary — a heuristic
 * boundary would leave 46 bytes of a record in the buffer whose first byte is a
 * sample counter that can perfectly well be 0x90.
 */
HM_TEST(codec_flags_a_misaligned_record_without_dropping_it)
{
    uint8_t buf[128];
    hm_wire_block a = hm_wire_identity_block(1000);
    hm_decoded d;
    size_t n = hm_wire_notification2(buf, 100, &a, &a, 108, &a, &a);

    /* Corrupt the second record's first quaternion component. */
    buf[1 + 46 + 2] = 0x00;
    buf[1 + 46 + 3] = 0x10;

    HM_ASSERT_EQ(hm_codec_decode(buf, n, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT_MSG(d.u.frame.count == 2, "the length says two records, so there are two");
    HM_ASSERT_EQ(d.consumed, 93);
    HM_ASSERT(d.warnings & (1u << HM_WARN_QUAT_NORM));
    HM_ASSERT(d.u.frame.sample[1].flags & HM_SAMPLE_QUAT_NORM_SUSPECT);
    HM_ASSERT(!(d.u.frame.sample[0].flags & HM_SAMPLE_QUAT_NORM_SUSPECT));
}

/*
 * ⚠ A payload that is not a whole number of records is the signature of a
 * transport that coalesced two notifications.  §3 gives the protocol no length
 * field, no sequence number and no checksum, so nothing could resynchronise
 * after it — the only safe response is to be loud immediately.
 */
HM_TEST(codec_warns_on_a_payload_that_is_not_a_whole_number_of_records)
{
    uint8_t buf[256];
    hm_wire_block a = hm_wire_identity_block(1000);
    hm_decoded d;
    size_t n = hm_wire_notification1(buf, 100, &a, &a);

    /* Simulate a coalescing transport: append a second notification. */
    n += hm_wire_notification1(buf + n, 132, &a, &a);
    HM_ASSERT_EQ(n, 94);

    HM_ASSERT_EQ(hm_codec_decode(buf, n, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT_MSG(d.warnings & (1u << HM_WARN_TRAILING_BYTES),
                  "a coalesced buffer must be reported, never absorbed");

    /* And a fixed-length message with anything after it says so too. */
    {
        uint8_t over[16];
        memcpy(over, HM_FIX_VERSIONS, sizeof(HM_FIX_VERSIONS));
        memset(over + sizeof(HM_FIX_VERSIONS), 0x00, 4);
        HM_ASSERT_EQ(hm_codec_decode(over, sizeof(HM_FIX_VERSIONS) + 4,
                                     hm_stream_config_default(), &d),
                     HM_OK);
        HM_ASSERT_EQ(d.kind, HM_MSGK_VERSIONS);
        HM_ASSERT(d.warnings & (1u << HM_WARN_TRAILING_BYTES));
    }
}

/* §6.3 says one or two records.  More is decoded and reported, not truncated. */
HM_TEST(codec_reports_more_records_than_the_specification_describes)
{
    uint8_t buf[256];
    hm_wire_block a = hm_wire_identity_block(1000);
    hm_decoded d;
    size_t o = 1;
    buf[0] = 0x90;
    for (uint16_t k = 0; k < 3u; ++k) {
        o += hm_wire_write_record(buf + o, (uint16_t)(100 + 8 * k), &a, &a, 1);
    }
    HM_ASSERT_EQ(o, 139);

    HM_ASSERT_EQ(hm_codec_decode(buf, o, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT_EQ(d.u.frame.count, 3);
    HM_ASSERT_EQ(d.u.frame.count_present, 3);
    HM_ASSERT_EQ(d.consumed, o);
    HM_ASSERT(d.warnings & (1u << HM_WARN_UNEXPECTED_RECORD_COUNT));
    HM_ASSERT(!(d.warnings & (1u << HM_WARN_TRAILING_BYTES)));
}

/*
 * ⚠⚠ AND ABOVE THE ARRAY'S OWN SIZE THE TRUNCATION MUST NOT BE SILENT.
 *
 * implementation-review I13.  hm_codec.h promises the oversized array exists "so
 * that a firmware sending more is DECODED and reported rather than silently
 * truncated" — a promise a fixed array of four cannot keep for five.  It dropped
 * the extras AND computed `consumed` from the truncated count, so a 5-record
 * notification reported 185 bytes interpreted out of 231 and the missing 46 went
 * unmentioned.  The warning fired; nothing said the array had overflowed.
 *
 * Two numbers now state the contract: how many were decoded, and how many the
 * LENGTH says are there.  `consumed` follows the length, so the notification is
 * always fully accounted for.
 */
HM_TEST(codec_says_how_many_records_it_dropped_rather_than_truncating_quietly)
{
    uint8_t buf[512];
    hm_wire_block a = hm_wire_identity_block(1000);
    hm_decoded d;
    size_t o = 1;
    buf[0] = 0x90;
    for (uint16_t k = 0; k < 5u; ++k) {
        o += hm_wire_write_record(buf + o, (uint16_t)(100 + 8 * k), &a, &a, 1);
    }

    HM_ASSERT_EQ(hm_codec_decode(buf, o, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT_EQ(d.u.frame.count, HM_CODEC_MAX_RECORDS);
    HM_ASSERT_MSG(d.u.frame.count_present == 5u,
                  "the length says five, and that is what the notification held");
    HM_ASSERT_MSG(d.consumed == o,
                  "every byte of the notification was interpreted, whatever the "
                  "array could hold");
    HM_ASSERT(d.warnings & (1u << HM_WARN_UNEXPECTED_RECORD_COUNT));
    HM_ASSERT(!(d.warnings & (1u << HM_WARN_TRAILING_BYTES)));
}

/*
 * ⚠ The norm tolerance must be tight enough to catch a one-byte misalignment.
 * §6.4 measures |q| = 16384.7 ± 0.41 over 6,064 records, so a correct frame has
 * enormous margin and a shifted one has none.
 */
HM_TEST(codec_norm_tolerance_catches_a_one_byte_shift)
{
    uint8_t buf[128];
    hm_wire_block a = hm_wire_identity_block(1000);
    hm_decoded d;
    size_t n;

    /* A realistic, fully populated quaternion rather than the identity. */
    a.q[0] = 14189;
    a.q[1] = 8192;
    a.q[2] = 0;
    a.q[3] = 0;
    n = hm_wire_notification2(buf, 100, &a, &a, 108, &a, &a);

    HM_ASSERT_EQ(hm_codec_decode(buf, n, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT_MSG(!(d.warnings & (1u << HM_WARN_QUAT_NORM)),
                  "a correctly located frame must never trip the check");

    /* Now decode the same bytes one byte late, as a stream framer would after a
     * mis-sized message.  This is the corruption the datagram contract removes. */
    {
        hm_stream_config cfg = hm_stream_config_default();
        HM_ASSERT_EQ(hm_codec_decode_record(buf + 2, n - 2, &cfg, &d.u.frame.sample[0]), HM_OK);
    }
    HM_ASSERT_MSG(d.u.frame.sample[0].flags & HM_SAMPLE_QUAT_NORM_SUSPECT,
                  "a one-byte shift must be visible in the norm");
    HM_ASSERT(HM_QUAT_NORM_TOLERANCE <= 64.0f);
}

/*
 * ⚠ §6.4: int16 fields SATURATE rather than wrap.  Clipped data looks like a
 * plausible waveform, and nothing in the protocol reports it — a sharp hand
 * shake reaches 83 % of full scale where a struck golf swing reaches 53-58 %.
 */
HM_TEST(codec_counts_pinned_samples)
{
    uint8_t buf[64];
    hm_wire_block arm = hm_wire_identity_block(1000);
    hm_wire_block palm = hm_wire_identity_block(1059);
    hm_decoded d;
    hm_pinned_counts counts;
    size_t n;

    arm.gyro[1] = 32767;   /* pinned positive */
    palm.accel[2] = -32767; /* pinned negative */
    n = hm_wire_notification1(buf, 7, &arm, &palm);

    HM_ASSERT_EQ(hm_codec_decode(buf, n, hm_stream_config_default(), &d), HM_OK);
    HM_ASSERT(d.u.frame.sample[0].flags & HM_SAMPLE_PINNED);

    hm_pinned_counts_reset(&counts);
    hm_pinned_counts_add(&counts, &d.u.frame.sample[0]);
    HM_ASSERT_EQ(counts.n[HM_UNIT_LOWER_ARM][HM_CH_GYRO_Y], 1);
    HM_ASSERT_EQ(counts.n[HM_UNIT_PALM][HM_CH_ACCEL_Z], 1);
    HM_ASSERT_EQ(counts.n[HM_UNIT_LOWER_ARM][HM_CH_ACCEL_Z], 0);
    HM_ASSERT_EQ(counts.total, 2);
}

/*
 * ⚠ §6.3.1: the legacy 0x7f layout has NO record header and NO tick counter,
 * so it carries no device clock whatsoever and history retrieval is impossible
 * against it.  Samples decoded from it must say so.
 */
HM_TEST(codec_legacy_frame_is_flagged_not_time_alignable)
{
    hm_decoded d;
    hm_status st = hm_codec_decode(HM_FIX_FRAME_LEGACY, sizeof(HM_FIX_FRAME_LEGACY),
                                   hm_stream_config_legacy(), &d);
    HM_ASSERT_EQ(st, HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_LEGACY_FRAME);
    HM_ASSERT_EQ(d.consumed, 43);
    HM_ASSERT(d.u.frame.sample[0].flags & HM_SAMPLE_NOT_TIME_ALIGNABLE);
    HM_ASSERT(d.u.frame.sample[0].flags & HM_SAMPLE_INDEX_MISSING);
    HM_ASSERT(d.u.frame.sample[0].flags & HM_SAMPLE_TICKS_MISSING);
    HM_ASSERT(d.warnings & (1u << HM_WARN_LEGACY_STREAM));

    /* Blocks start at offset 0, so the quaternion is the first thing there. */
    HM_ASSERT_EQ(d.u.frame.sample[0].lower_arm.q_world_to_body_raw[0], 16384);
    /* Nominal divisor 16 for the legacy path: 800 → 50 °/s, not 100. */
    HM_ASSERT_NEAR(d.u.frame.sample[0].lower_arm.gyro_dps[0], 50.0f, 1e-4);
}

/* §5.2 — versions.  Protocol version gates features, so it must be exact. */
HM_TEST(codec_decodes_versions)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_VERSIONS, sizeof(HM_FIX_VERSIONS),
                                 hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_VERSIONS);
    HM_ASSERT_EQ(d.u.versions.hardware_major, 4);
    HM_ASSERT_EQ(d.u.versions.hardware_minor, 1);
    HM_ASSERT_EQ(d.u.versions.protocol_major, 4);
    HM_ASSERT_EQ(d.u.versions.protocol_minor, 0);
    HM_ASSERT_EQ(d.u.versions.firmware_major, 4);
    HM_ASSERT_EQ(d.u.versions.firmware_minor, 8);
    HM_ASSERT_EQ(d.u.versions.product_id, 0x14);
}

/* §5.3 — battery, then a u16be that is explicitly NOT millivolts. */
HM_TEST(codec_decodes_status)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_STATUS, sizeof(HM_FIX_STATUS),
                                 hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.u.status.percent, 100);
    HM_ASSERT_EQ(d.u.status.undecoded, 2226);
}

/* §5.4 — sensor count 2 is why every record carries two blocks. */
HM_TEST(codec_decodes_sensor_map)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_SENSOR_MAP, sizeof(HM_FIX_SENSOR_MAP),
                                 hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.u.sensor_map.sensor_count, 2);
}

/* §4 — the MAC arrives as 12 ASCII hex characters with no separators. */
HM_TEST(codec_decodes_identifiers)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_MAC, sizeof(HM_FIX_MAC), hm_stream_config_default(),
                                 &d),
                 HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_MAC);
    HM_ASSERT_STR(d.u.text.text, "01:23:45:67:89:AB");

    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_SERIAL, sizeof(HM_FIX_SERIAL),
                                 hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_SERIAL);
    HM_ASSERT_STR(d.u.text.text, "WG3-00042");
}

/* §7.2 — the bracket.  02 = start (the acceptance test), 01 = end. */
HM_TEST(codec_decodes_history_bracket)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_HIST_START, 2, hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.u.history.marker, 0x02);
    HM_ASSERT(d.u.history.valid);

    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_HIST_END, 2, hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.u.history.marker, 0x01);
    HM_ASSERT(d.u.history.valid);

    {
        /* "unknown magic cookie" — any other value is rejected. */
        const uint8_t bogus[2] = {0xA1, 0x07};
        HM_ASSERT_EQ(hm_codec_decode(bogus, 2, hm_stream_config_default(), &d), HM_OK);
        HM_ASSERT(!d.u.history.valid);
    }
}

/* §7.2 — d0 03 is the only error the history command produces, across seven
 * distinct causes.  The codec must not pretend to know which. */
HM_TEST(codec_decodes_device_error)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_DEVICE_ERROR, 2, hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_DEVICE_ERROR);
    HM_ASSERT_EQ(d.u.device_error.code, 0x03);
}

/* §8.2 — the 64-byte payload is carried verbatim and deliberately undecoded. */
HM_TEST(codec_decodes_calibration_result)
{
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_CALIBRATION, sizeof(HM_FIX_CALIBRATION),
                                 hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_CALIBRATION_RESULT);
    HM_ASSERT_EQ(d.consumed, 65);
    HM_ASSERT(!d.u.calibration.is_status);
    HM_ASSERT_EQ(d.u.calibration.payload[0], 0x01);
    HM_ASSERT_EQ(d.u.calibration.payload[63], 0x40);
    HM_ASSERT_EQ(d.warnings, 0u);

    /* Past the long form's 65 bytes there is nothing to interpret: the eight
     * quaternions are still decoded and the remainder is reported rather than
     * absorbed, exactly as for any other over-long notification. */
    {
        uint8_t over[70];
        memset(over, 0xEE, sizeof(over));
        memcpy(over, HM_FIX_CALIBRATION, sizeof(HM_FIX_CALIBRATION));
        HM_ASSERT_EQ(hm_codec_decode(over, sizeof(over), hm_stream_config_default(), &d), HM_OK);
        HM_ASSERT_EQ(d.kind, HM_MSGK_CALIBRATION_RESULT);
        HM_ASSERT_EQ(d.consumed, 65);
        HM_ASSERT_EQ(d.u.calibration.payload[63], 0x40);
        HM_ASSERT(d.warnings & (1u << HM_WARN_TRAILING_BYTES));
    }
}

/*
 * ⚠ §8.2 — `0x94` HAS TWO FORMS AND THE DEVICE SPLITS THEM ON TOTAL LENGTH.
 * A short one is not a truncated long one, and decoding it as such would drop
 * the only form that carries a status byte at all.
 */
HM_TEST(codec_decodes_the_short_form_of_the_calibration_result)
{
    hm_decoded d;

    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_CALIBRATION_STATUS, sizeof(HM_FIX_CALIBRATION_STATUS),
                                 hm_stream_config_default(), &d),
                 HM_OK);
    HM_ASSERT_EQ(d.kind, HM_MSGK_CALIBRATION_RESULT);
    HM_ASSERT(d.u.calibration.is_status);
    HM_ASSERT_EQ(d.u.calibration.status, 0x01);
    HM_ASSERT_EQ(d.consumed, 2);
    /* The quaternion buffer is not left holding something that looks decoded. */
    HM_ASSERT_EQ(d.u.calibration.payload[0], 0x00);
    HM_ASSERT_EQ(d.warnings, 0u);

    /* The whole short range decodes, up to the device's own 63-byte ceiling —
     * and `consumed` claims only the byte that was read, never the rest. */
    {
        uint8_t msg[63];
        memset(msg, 0xEE, sizeof(msg));
        msg[0] = 0x94;
        msg[1] = 0x7F;
        for (size_t n = 2; n <= sizeof(msg); ++n) {
            HM_ASSERT_EQ(hm_codec_decode(msg, n, hm_stream_config_default(), &d), HM_OK);
            HM_ASSERT(d.u.calibration.is_status);
            HM_ASSERT_EQ(d.u.calibration.status, 0x7F);
            HM_ASSERT_EQ(d.consumed, 2);
        }
    }

    /* The id on its own has no status byte in it. */
    {
        const uint8_t bare[1] = {0x94};
        HM_ASSERT_EQ(hm_codec_decode(bare, sizeof(bare), hm_stream_config_default(), &d),
                     HM_ERR_TRUNCATED);
    }

    /*
     * ⚠ 64 BYTES IS NEITHER FORM.  Above the short form's ceiling, so it is a
     * long form one byte shy of its eighth quaternion — truncated, and not to
     * be salvaged as seven quaternions and a guess.
     */
    {
        uint8_t msg[64];
        memset(msg, 0, sizeof(msg));
        msg[0] = 0x94;
        HM_ASSERT_EQ(hm_codec_decode(msg, sizeof(msg), hm_stream_config_default(), &d),
                     HM_ERR_TRUNCATED);
    }
}

/* §5.1 — anything not in the table is logged and ignored, NOT an error. */
HM_TEST(codec_reports_unknown_ids_without_treating_them_as_failures)
{
    const uint8_t unknown[4] = {0x77, 0x01, 0x02, 0x03};
    hm_decoded d;
    HM_ASSERT_EQ(hm_codec_decode(unknown, sizeof(unknown), hm_stream_config_default(), &d),
                 HM_ERR_UNKNOWN_MESSAGE);
    HM_ASSERT_EQ(d.kind, HM_MSGK_UNKNOWN);
    HM_ASSERT_EQ(d.message_id, 0x77);
    /* An unknown id has no implied length, so nothing after it can be located. */
    HM_ASSERT_EQ(d.consumed, 4);
}

HM_TEST(codec_reports_truncation_rather_than_reading_past_the_buffer)
{
    hm_decoded d;
    for (size_t n = 1; n < sizeof(HM_FIX_FRAME_7E); ++n) {
        HM_ASSERT_EQ(hm_codec_decode(HM_FIX_FRAME_7E, n, hm_stream_config_default(), &d),
                     HM_ERR_TRUNCATED);
    }
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_VERSIONS, 4, hm_stream_config_default(), &d),
                 HM_ERR_TRUNCATED);
    HM_ASSERT_EQ(hm_codec_decode(HM_FIX_FRAME_7E, 0, hm_stream_config_default(), &d),
                 HM_ERR_TRUNCATED);
}

/* Every prefix of every fixture must be handled without a crash or a read past
 * the end.  Run under ASan this is a cheap standing fuzz of the decoder. */
HM_TEST(codec_survives_every_prefix_of_every_fixture)
{
    const uint8_t *fixtures[] = {HM_FIX_FRAME_7E,  HM_FIX_FRAME_LEGACY, HM_FIX_VERSIONS,
                                 HM_FIX_STATUS,    HM_FIX_SENSOR_MAP,   HM_FIX_MAC,
                                 HM_FIX_SERIAL,    HM_FIX_CALIBRATION,  HM_FIX_START_ACK,
                                 HM_FIX_STOP_ACK,  HM_FIX_BUTTON,       HM_FIX_DEVICE_ERROR};
    const size_t sizes[] = {sizeof(HM_FIX_FRAME_7E),  sizeof(HM_FIX_FRAME_LEGACY),
                            sizeof(HM_FIX_VERSIONS),  sizeof(HM_FIX_STATUS),
                            sizeof(HM_FIX_SENSOR_MAP), sizeof(HM_FIX_MAC),
                            sizeof(HM_FIX_SERIAL),    sizeof(HM_FIX_CALIBRATION),
                            sizeof(HM_FIX_START_ACK), sizeof(HM_FIX_STOP_ACK),
                            sizeof(HM_FIX_BUTTON),    sizeof(HM_FIX_DEVICE_ERROR)};
    hm_decoded d;
    int handled = 0;

    for (size_t f = 0; f < sizeof(sizes) / sizeof(sizes[0]); ++f) {
        for (size_t n = 0; n <= sizes[f]; ++n) {
            hm_status st = hm_codec_decode(fixtures[f], n, hm_stream_config_default(), &d);
            HM_ASSERT(st == HM_OK || st == HM_ERR_TRUNCATED || st == HM_ERR_UNKNOWN_MESSAGE);
            if (st == HM_OK) {
                HM_ASSERT(d.consumed <= n);
            }
            handled++;
        }
    }
    HM_ASSERT(handled > 200);
}

HM_TEST_MAIN()
