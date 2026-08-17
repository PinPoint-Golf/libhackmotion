/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
#include "hm_codec.h"
#include "hackmotion/event.h"
#include "hackmotion/quat.h"

#include <string.h>
#include <math.h>
#include <stdio.h>

#define WARN(bit) (1u << (unsigned)(bit))

/* ------------------------------------------------------------------------ */
/* Scaling                                                                   */
/* ------------------------------------------------------------------------ */
void hm_codec_scale_unit(hm_unit_sample *unit, const hm_stream_config *cfg)
{
    const float gyro_div = (float)hm_stream_config_gyro_divisor(*cfg);
    uint8_t mask = 0;

    for (int i = 0; i < 4; ++i) {
        unit->q_world_to_body[i] = (float)unit->q_world_to_body_raw[i] / HM_QUAT_SCALE;
    }
    for (int i = 0; i < 3; ++i) {
        int16_t a = unit->linear_accel_raw[i];
        int16_t g = unit->gyro_raw[i];
        unit->linear_accel_mps2[i] = (float)a * HM_ACCEL_LSB_MPS2;
        unit->gyro_dps[i] = (float)g / gyro_div;

        /*
         * ⚠ int16 fields SATURATE rather than wrap, so clipped data still looks
         * like a plausible waveform (§6.4).  Nothing in the protocol reports
         * it, so the library counts it.
         */
        if (a >= HM_PIN_THRESHOLD || a <= -HM_PIN_THRESHOLD) {
            mask |= (uint8_t)(1u << (unsigned)(HM_CH_ACCEL_X + i));
        }
        if (g >= HM_PIN_THRESHOLD || g <= -HM_PIN_THRESHOLD) {
            mask |= (uint8_t)(1u << (unsigned)(HM_CH_GYRO_X + i));
        }
    }
    unit->pinned_mask = mask;
    unit->device_time_us = HM_TIME_UNKNOWN;
}

/* Reads one block.  `block_size` is 22, 20 (0x90 layouts) or 21 (legacy). */
static void decode_block(const uint8_t *p, size_t block_size, const hm_stream_config *cfg,
                         hm_unit_sample *unit)
{
    memset(unit, 0, sizeof(*unit));

    for (int i = 0; i < 4; ++i) {
        unit->q_world_to_body_raw[i] = hm_be16s(p + 2 * i);
    }
    for (int i = 0; i < 3; ++i) {
        unit->linear_accel_raw[i] = hm_be16s(p + 8 + 2 * i);
        unit->gyro_raw[i] = hm_be16s(p + 14 + 2 * i);
    }

    if (block_size == 22u) {
        unit->ticks_raw = hm_be16(p + 20);
        unit->has_ticks = 1u;
    } else {
        unit->ticks_raw = 0u;
        unit->has_ticks = 0u;
    }
    /* block_size 21: the trailing byte read 00 in every frame of a 981-frame
     * session (§6.3.1).  It is consumed and not interpreted. */

    hm_codec_scale_unit(unit, cfg);
}

static bool quat_norm_ok(const hm_unit_sample *unit)
{
    float n = hm_quat_raw_norm(unit->q_world_to_body_raw);
    float d = n - HM_QUAT_NORM_NOMINAL;
    if (d < 0.0f) {
        d = -d;
    }
    return d <= HM_QUAT_NORM_TOLERANCE;
}

hm_status hm_codec_decode_record(const uint8_t *data, size_t length,
                                 const hm_stream_config *cfg, hm_sample *out)
{
    const size_t block = hm_stream_config_block_size(*cfg);
    const size_t record = hm_stream_config_record_size(*cfg);
    const bool legacy = hm_stream_config_is_legacy(*cfg);
    size_t offset;

    if (data == NULL || out == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    if (length < record) {
        return HM_ERR_TRUNCATED;
    }

    memset(out, 0, sizeof(*out));
    out->config_bits = cfg->bits;
    out->skew_us = 0;
    out->host_time_us = HM_TIME_UNKNOWN;
    out->host_recv_us = HM_TIME_UNKNOWN;
    out->uncertainty_us = UINT32_MAX;

    if (legacy) {
        /* No record header: blocks start at offset 0 (§6.3.1). */
        offset = 0;
        out->sample_index_raw = 0;
        out->flags |= (uint16_t)(HM_SAMPLE_INDEX_MISSING | HM_SAMPLE_NOT_TIME_ALIGNABLE |
                                 HM_SAMPLE_TICKS_MISSING | HM_SAMPLE_NONSTANDARD_CONFIG);
    } else {
        out->sample_index_raw = hm_be16(data);
        offset = 2;
        if (!hm_stream_config_has_ticks(*cfg)) {
            out->flags |= (uint16_t)HM_SAMPLE_TICKS_MISSING;
        }
        if (!hm_stream_config_is_observed_default(*cfg)) {
            out->flags |= (uint16_t)HM_SAMPLE_NONSTANDARD_CONFIG;
        }
    }

    /* ⚠ The FIRST block is the lower-arm unit, the SECOND is the palm.  Fixed
     * by the cable, not by convention (§6.3). */
    decode_block(data + offset, block, cfg, &out->lower_arm);
    decode_block(data + offset + block, block, cfg, &out->palm);

    if (out->lower_arm.pinned_mask != 0u || out->palm.pinned_mask != 0u) {
        out->flags |= (uint16_t)HM_SAMPLE_PINNED;
    }
    if (!quat_norm_ok(&out->lower_arm) || !quat_norm_ok(&out->palm)) {
        out->flags |= (uint16_t)HM_SAMPLE_QUAT_NORM_SUSPECT;
    }
    return HM_OK;
}

/* ------------------------------------------------------------------------ */
/* Message dispatch                                                          */
/* ------------------------------------------------------------------------ */

/* Payload length implied by the message id, or -1 when it is not fixed. */
static int fixed_payload_length(uint8_t id)
{
    switch (id) {
        case HM_MSG_ID_VERSIONS:       return 7;
        case HM_MSG_ID_STATUS:         return 3;
        case HM_MSG_ID_STREAM_STARTED: return 1;
        case HM_MSG_ID_STREAM_STOPPED: return 1;
        case HM_MSG_ID_MAC:            return 12;
        case HM_MSG_ID_SERIAL:         return 9;
        case HM_MSG_ID_START_ACK:      return 1;
        case HM_MSG_ID_HISTORY_MARK:   return 1;
        case HM_MSG_ID_CAL_ACK:        return 1;
        case HM_MSG_ID_DEVICE_ERROR:   return 1;
        case HM_MSG_ID_BUTTON:         return 1;
        default:                       return -1;
    }
}

static void copy_text(char *dst, size_t dst_size, const uint8_t *src, size_t n)
{
    size_t i;
    size_t limit = (n < dst_size - 1u) ? n : dst_size - 1u;
    for (i = 0; i < limit; ++i) {
        /* Only printable ASCII survives; the device has never sent anything
         * else here, and a log line is not the place to find out otherwise. */
        dst[i] = (src[i] >= 0x20 && src[i] < 0x7f) ? (char)src[i] : '?';
    }
    dst[i] = '\0';
}

static void format_mac(char *dst, size_t dst_size, const uint8_t *src)
{
    /* 12 ASCII hex characters, no separators (§4).  Rendered colon-separated so
     * it matches what every platform's device registry shows. */
    if (dst_size < 18u) {
        if (dst_size > 0u) {
            dst[0] = '\0';
        }
        return;
    }
    (void)snprintf(dst, dst_size, "%c%c:%c%c:%c%c:%c%c:%c%c:%c%c", src[0], src[1], src[2],
                   src[3], src[4], src[5], src[6], src[7], src[8], src[9], src[10], src[11]);
}

/*
 * Decode the run of records in a 0x90 / 0x7f notification.  The count comes
 * from the LENGTH, never from the content — see hm_codec.h for why a heuristic
 * would be a silent-corruption hazard rather than a convenience.
 */
static hm_status decode_frame(const uint8_t *data, size_t length, const hm_stream_config *cfg,
                              hm_decoded *out)
{
    const size_t record = hm_stream_config_record_size(*cfg);
    const size_t avail = length - 1u;
    const size_t present = avail / record;
    size_t count = present;

    if (count == 0u) {
        return HM_ERR_TRUNCATED;
    }
    if (avail % record != 0u) {
        /* ⚠ The signature of a transport that coalesced two notifications, or
         * of a message this decoder does not understand.  Loud on purpose. */
        out->warnings |= WARN(HM_WARN_TRAILING_BYTES);
    }
    if (count > HM_CODEC_SPEC_MAX_RECORDS) {
        out->warnings |= WARN(HM_WARN_UNEXPECTED_RECORD_COUNT);
    }
    if (count > HM_CODEC_MAX_RECORDS) {
        count = HM_CODEC_MAX_RECORDS;
    }

    for (size_t i = 0; i < count; ++i) {
        hm_status st = hm_codec_decode_record(data + 1u + i * record, avail - i * record, cfg,
                                              &out->u.frame.sample[i]);
        if (st != HM_OK) {
            return st;
        }
        /*
         * §6.4's |q| = 16384.7 ± 0.41 is a check that the decode is ALIGNED.
         * It is reported, never acted on: framing is the length's job, and a
         * structural check that silently changed the frame boundary would be
         * the very heuristic this design removed.
         */
        if ((out->u.frame.sample[i].flags & (uint16_t)HM_SAMPLE_QUAT_NORM_SUSPECT) != 0u) {
            out->warnings |= WARN(HM_WARN_QUAT_NORM);
        }
    }
    out->u.frame.count = count;
    /*
     * ⚠ `present`, NOT `count`: the notification's own length is what was
     * interpreted, whatever the fixed array could hold (implementation-review
     * I13).  Computing this from the truncated count under-reported a 5-record
     * frame by 46 bytes and left them unaccounted for — a silent truncation
     * reported as a clean decode, under a header comment promising the oversized
     * array exists so that a firmware sending more is decoded and reported
     * rather than silently truncated.
     *
     * `count` and `count_present` differing IS the report, and it travels beside
     * HM_WARN_UNEXPECTED_RECORD_COUNT rather than instead of it: a warning says
     * "more than §6.3 describes", these two say how many were kept.
     */
    out->u.frame.count_present = present;
    out->consumed = 1u + present * record;
    return HM_OK;
}

/*
 * 0x84 — the sensor map.  ⚠ THE SENSOR COUNT IS THE REPLY'S LENGTH, NOT A BYTE
 * IN IT (§5.4), which is why this is length-driven rather than a fixed two.
 *
 * `84 02 01` carries two payload bytes, so two sensors.  The leading `02` is
 * the first sensor's LOCATION CODE — 0.26 m from the joint, the lower arm — and
 * it equals the sensor count on this device by pure coincidence.  A parser that
 * reads byte 0 as the count is correct here and wrong on any device whose first
 * sensor sits somewhere else.
 *
 * Zero payload bytes would be zero sensors, which no device can be; it is
 * TRUNCATED rather than a valid empty map.
 */
static hm_status decode_sensor_map(const uint8_t *data, size_t length, hm_decoded *out)
{
    size_t n = length - 1u;
    size_t i;

    if (n == 0u) {
        return HM_ERR_TRUNCATED;
    }
    out->kind = HM_MSGK_SENSOR_MAP;
    out->u.sensor_map.count = (uint8_t)((n > 255u) ? 255u : n);
    for (i = 0; i < n && i < HM_SENSOR_LOCATION_MAX; ++i) {
        out->u.sensor_map.location[i] = data[1 + i];
    }
    /* Every payload byte is a location code, so the whole notification is
     * accounted for however many there are — no trailing-bytes case exists. */
    out->consumed = length;
    return HM_OK;
}

#define CAL_SHORT_FORM_MAX 63u
#define CAL_LONG_FORM_LEN  65u

/*
 * 0x94 — the calibration result.  ⚠ IT IS NOT A FIXED-LENGTH MESSAGE: §8.2
 * gives it TWO FORMS and the device splits them on the notification's own
 * total length, which is why it is decoded here rather than through
 * fixed_payload_length().
 *
 *   long form    65 bytes — the id and eight quaternions.  The full result,
 *                and the only form ever captured.
 *   short form   63 bytes or fewer — the id and a status byte.  ⚠ The status
 *                byte exists ONLY here; the long form carries no verdict at
 *                all (§8.2).
 *
 * ⚠ 64 BYTES IS NEITHER, and that is the device's own arithmetic rather than a
 * choice made here: the split is on total length, 64 is above the short form's
 * ceiling, so a 64-byte notification is a long form one byte shy of its eighth
 * quaternion.  Truncated, and reported as such rather than decoded as seven
 * quaternions and a guess.
 *
 * The 64 payload bytes themselves stay undecoded, deliberately: the device
 * applies the transform itself, so a client never needs the values (§8.2,
 * api-request §2.6).  They are carried verbatim into the wire log.
 */
static hm_status decode_calibration(const uint8_t *data, size_t length, hm_decoded *out)
{
    out->kind = HM_MSGK_CALIBRATION_RESULT;

    if (length > CAL_SHORT_FORM_MAX) {
        if (length < CAL_LONG_FORM_LEN) {
            return HM_ERR_TRUNCATED;
        }
        if (length > CAL_LONG_FORM_LEN) {
            out->warnings |= WARN(HM_WARN_TRAILING_BYTES);
        }
        memcpy(out->u.calibration.payload, data + 1, 64u);
        out->consumed = CAL_LONG_FORM_LEN;
        return HM_OK;
    }

    if (length < 2u) {
        return HM_ERR_TRUNCATED;
    }
    out->u.calibration.is_status = true;
    out->u.calibration.status = data[1];
    /*
     * ⚠ No HM_WARN_TRAILING_BYTES here even when the notification is longer
     * than two bytes.  What is known about the short form is that its second
     * byte is a status code; its full length is not known, and warning about
     * bytes 2..n would assert a length this library cannot support.  `consumed`
     * says exactly what was interpreted, which is what it is for.
     */
    out->consumed = 2u;
    return HM_OK;
}

hm_status hm_codec_decode(const uint8_t *data, size_t length, hm_stream_config cfg,
                          hm_decoded *out)
{
    uint8_t id;
    int fixed;

    if (data == NULL || out == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    if (length == 0) {
        return HM_ERR_TRUNCATED;
    }

    id = data[0];
    out->message_id = id;

    /* --- Stream frames ------------------------------------------------- */
    if (id == HM_MSG_ID_FRAME) {
        hm_stream_config frame_cfg = cfg;
        if (hm_stream_config_is_legacy(frame_cfg)) {
            /* A 0x90 cannot come from a legacy start; decode it under the
             * default layout rather than mis-sizing the blocks. */
            frame_cfg = hm_stream_config_default();
        }
        out->kind = HM_MSGK_FRAME;
        return decode_frame(data, length, &frame_cfg, out);
    }

    if (id == HM_MSG_ID_LEGACY_FRAME) {
        hm_status st;
        {
            hm_stream_config legacy = hm_stream_config_legacy();
            out->kind = HM_MSGK_LEGACY_FRAME;
            st = decode_frame(data, length, &legacy, out);
        }
        if (st == HM_OK) {
            out->warnings |= WARN(HM_WARN_LEGACY_STREAM);
        }
        return st;
    }

    /* --- The two messages whose LENGTH carries meaning ------------------- */
    if (id == HM_MSG_ID_SENSOR_MAP) {
        return decode_sensor_map(data, length, out);
    }

    if (id == HM_MSG_ID_CALIBRATION) {
        return decode_calibration(data, length, out);
    }

    /* --- Fixed-length messages ------------------------------------------ */
    fixed = fixed_payload_length(id);
    if (fixed < 0) {
        /*
         * §5.1: anything not in the table is logged and ignored rather than
         * treated as an error.  The whole notification is discarded, which
         * under datagram framing loses nothing else.
         */
        out->kind = HM_MSGK_UNKNOWN;
        out->consumed = length;
        return HM_ERR_UNKNOWN_MESSAGE;
    }
    if (length - 1u < (size_t)fixed) {
        return HM_ERR_TRUNCATED;
    }
    if (length - 1u > (size_t)fixed) {
        out->warnings |= WARN(HM_WARN_TRAILING_BYTES);
    }
    out->consumed = 1u + (size_t)fixed;

    switch (id) {
        case HM_MSG_ID_VERSIONS:
            out->kind = HM_MSGK_VERSIONS;
            out->u.versions.hardware_major = data[1];
            out->u.versions.hardware_minor = data[2];
            out->u.versions.protocol_major = data[3];
            out->u.versions.protocol_minor = data[4];
            out->u.versions.firmware_major = data[5];
            out->u.versions.firmware_minor = data[6];
            out->u.versions.product_id = data[7];
            break;

        case HM_MSG_ID_STATUS:
            out->kind = HM_MSGK_STATUS;
            out->u.status.percent = data[1];
            out->u.status.undecoded = hm_be16(data + 2); /* NOT millivolts */
            break;

        case HM_MSG_ID_STREAM_STARTED:
            out->kind = HM_MSGK_STREAM_STARTED;
            break;

        case HM_MSG_ID_STREAM_STOPPED:
            out->kind = HM_MSGK_STREAM_STOPPED;
            break;

        case HM_MSG_ID_MAC:
            out->kind = HM_MSGK_MAC;
            format_mac(out->u.text.text, sizeof(out->u.text.text), data + 1);
            break;

        case HM_MSG_ID_SERIAL:
            out->kind = HM_MSGK_SERIAL;
            copy_text(out->u.text.text, sizeof(out->u.text.text), data + 1, 9u);
            break;

        case HM_MSG_ID_START_ACK:
            /* ⚠ The app ignores this and so must a client: stream startup is
             * keyed off the first 0x90 frame, not off an acknowledgement. */
            out->kind = HM_MSGK_START_ACK;
            break;

        case HM_MSG_ID_HISTORY_MARK:
            out->kind = HM_MSGK_HISTORY_MARK;
            out->u.history.marker = data[1];
            out->u.history.valid =
                (data[1] == HM_HISTORY_MARK_START || data[1] == HM_HISTORY_MARK_END);
            break;

        case HM_MSG_ID_CAL_ACK:
            out->kind = HM_MSGK_CAL_ACK;
            break;

        case HM_MSG_ID_DEVICE_ERROR:
            out->kind = HM_MSGK_DEVICE_ERROR;
            out->u.device_error.code = data[1];
            break;

        case HM_MSG_ID_BUTTON:
            out->kind = HM_MSGK_BUTTON;
            out->u.button.payload = data[1];
            break;

        default:
            out->kind = HM_MSGK_IGNORED;
            break;
    }
    return HM_OK;
}
