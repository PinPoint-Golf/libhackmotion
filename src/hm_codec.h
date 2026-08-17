/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_codec.h — internal.  Wire bytes in, decoded messages out.
 *
 * This layer is pure: no state beyond what the caller passes, no allocation, no
 * clock, no I/O.  It is the layer the fuzzers and the golden vectors target.
 *
 * ⚠ Fuzzing the DECODER is encouraged.  Fuzzing the device is not — the command
 * space must never be swept, because ⛔ `f0` reaches firmware-update mode
 * through the ordinary data characteristic (spec §4.1, §12).  Those are
 * different activities and only one of them is safe.
 */
#ifndef HM_CODEC_H
#define HM_CODEC_H

#include "hackmotion/types.h"
#include "hackmotion/config.h"
#include "hackmotion/sample.h"

/* Message ids the device can send.  Spec §5.1. */
typedef enum hm_msg_id {
    HM_MSG_ID_LEGACY_FRAME  = 0x7f,
    HM_MSG_ID_VERSIONS      = 0x80,
    HM_MSG_ID_STATUS        = 0x81,
    HM_MSG_ID_STREAM_STARTED= 0x82,
    HM_MSG_ID_STREAM_STOPPED= 0x83,
    HM_MSG_ID_SENSOR_MAP    = 0x84,
    HM_MSG_ID_MAC           = 0x85,
    HM_MSG_ID_SERIAL        = 0x86,
    HM_MSG_ID_FRAME         = 0x90,
    HM_MSG_ID_CALIBRATION   = 0x94,
    HM_MSG_ID_START_ACK     = 0xa0,
    HM_MSG_ID_HISTORY_MARK  = 0xa1,
    HM_MSG_ID_CAL_ACK       = 0xa2,
    HM_MSG_ID_DEVICE_ERROR  = 0xd0,
    HM_MSG_ID_BUTTON        = 0xfb
} hm_msg_id;

/* The `a1` payload byte discriminates the bracket.  Any other value is
 * rejected — the vendor app calls it an "unknown magic cookie". */
#define HM_HISTORY_MARK_END   0x01
#define HM_HISTORY_MARK_START 0x02

typedef enum hm_msg_kind {
    HM_MSGK_UNKNOWN = 0,
    HM_MSGK_FRAME,             /* 0x90 — one or two records                   */
    HM_MSGK_LEGACY_FRAME,      /* 0x7f — one record, no header, no ticks      */
    HM_MSGK_VERSIONS,
    HM_MSGK_STATUS,
    HM_MSGK_STREAM_STARTED,
    HM_MSGK_STREAM_STOPPED,
    HM_MSGK_SENSOR_MAP,
    HM_MSGK_MAC,
    HM_MSGK_SERIAL,
    HM_MSGK_CALIBRATION_RESULT,
    HM_MSGK_START_ACK,
    HM_MSGK_HISTORY_MARK,
    HM_MSGK_CAL_ACK,
    HM_MSGK_DEVICE_ERROR,
    HM_MSGK_BUTTON,
    HM_MSGK_IGNORED            /* in §5.1, nothing for a client to do with it */
} hm_msg_kind;

/*
 * §6.3 says a notification carries one or two records.  The array is larger so
 * that a firmware sending more is DECODED and reported rather than silently
 * truncated; anything above two raises HM_WARN_UNEXPECTED_RECORD_COUNT.
 *
 * ⚠ A FIXED ARRAY CANNOT KEEP THAT PROMISE ABOVE ITS OWN SIZE, and pretending
 * otherwise is how a truncation becomes silent (implementation-review I13).  So
 * the contract is stated in two numbers instead of one: `count` is how many were
 * DECODED, `count_present` is how many the notification's LENGTH says are there.
 * They differ only above HM_CODEC_MAX_RECORDS, `consumed` always follows
 * `count_present` so the notification is fully accounted for, and the warning
 * fires from two.  A consumer that reads only `count` sees fewer records; one
 * that compares them sees that it did.
 */
#define HM_CODEC_MAX_RECORDS 4u
#define HM_CODEC_SPEC_MAX_RECORDS 2u

typedef struct hm_decoded {
    hm_msg_kind kind;
    uint8_t     message_id;
    size_t      consumed;     /* bytes of the notification that were interpreted */
    uint32_t    warnings;     /* bitmask of (1u << hm_warning_code) */

    union {
        struct {
            size_t    count;         /* decoded — at most HM_CODEC_MAX_RECORDS */
            size_t    count_present; /* ⚠ what the LENGTH says; see above       */
            hm_sample sample[HM_CODEC_MAX_RECORDS]; /* raw + scaled, no host time */
        } frame;
        struct {
            uint8_t hardware_major, hardware_minor;
            uint8_t protocol_major, protocol_minor;
            uint8_t firmware_major, firmware_minor;
            uint8_t product_id;
        } versions;
        struct {
            uint8_t  percent;
            uint16_t undecoded;
        } status;
        struct {
            uint8_t sensor_count;
            uint8_t undecoded;
        } sensor_map;
        struct {
            char text[20];           /* MAC (colon-formatted) or serial, NUL-terminated */
        } text;
        struct {
            uint8_t marker;          /* HM_HISTORY_MARK_START / _END */
            bool    valid;
        } history;
        struct {
            uint8_t code;            /* only 0x03 has ever been seen */
        } device_error;
        struct {
            uint8_t payload;         /* 0x01 */
        } button;
        struct {
            /*
             * ⚠ TWO FORMS, discriminated by the notification's LENGTH (§8.2).
             * `is_status` says which one arrived.
             *
             *   long form   65 bytes: `payload` holds the eight quaternions.
             *   short form  63 bytes or fewer: `status` holds the one status
             *               byte, and `payload` is zeroed.
             *
             * ⚠ The status byte exists only on the SHORT form.  The long form
             * carries no accept/reject anywhere in it, which is why arrival of
             * a `0x94` is not a verdict (§8.2) and the session's own presence
             * measurement is the only thing that decides.
             *
             * The long form's 64 bytes are still not decoded here, and that is
             * a decision rather than a gap: the device applies the transform
             * itself, so a client never needs the values (design §5.6, §7.4).
             * They are carried verbatim into the wire log, where anything that
             * wants them can re-decode them.  All eight are now identified:
             *
             *   q1  palm, rotation about z     q2  palm, mount quaternion
             *   q3  arm,  rotation about z     q4  arm,  mount quaternion
             *   q5  palm at pose 0             q6  palm at pose 1
             *   q7  arm  at pose 0             q8  arm  at pose 1
             *
             * — the four applied state quaternions, then the two poses they
             * were derived from.  q1 and q3 have x = y = 0 EXACTLY, being pure
             * rotations about z by construction, which is a free structural
             * check on the payload.
             *
             * ⚠ PALM FIRST, in both halves — reversed from a stream record,
             * which puts the lower arm first (§6.3).
             *
             * ⚠ AND A DECODER MUST NORMALISE RATHER THAN SCALE.  Each
             * quaternion is four i16be in w-x-y-z order, but the fixed-point
             * scale is implicit: firmware 4.5 populates only the HIGH BYTE of
             * each component, so |q| lands ~200 counts off 16384 and each
             * quaternion is good to about a degree, while 4.8 uses all 16 bits.
             * Dividing by 16384 and range-checking the norm rejects real device
             * output; normalising does not care.  §8.2 has the evidence.
             */
            bool    is_status;
            uint8_t status;
            uint8_t payload[64];
        } calibration;
    } u;
} hm_decoded;

/*
 * ⚠ ONE CALL, ONE NOTIFICATION.  `data`/`length` is a whole ATT notification
 * payload exactly as the transport delivered it — NOT a byte stream.
 *
 * This is a deliberate contract, not an assumption of convenience:
 *
 *  - GATT notifications are ATT PDUs and every stack preserves the boundary:
 *    one QByteArray per characteristicChanged, one D-Bus signal or socket read
 *    on BlueZ, one didUpdateValueForCharacteristic on CoreBluetooth.
 *  - HM_MIN_ATT_MTU is 96 precisely so the 93-byte maximum message fits in one
 *    notification (§2.4).  Having enforced that floor, the library is entitled
 *    to rely on it.
 *  - ⚠ And the alternative is unimplementable.  §3: no length field, no
 *    sequence number, no checksum.  A byte stream with those three absences
 *    cannot be resynchronised after any loss, because there is nothing to
 *    synchronise TO.  Stream framing here is not merely unnecessary; its
 *    failure mode is to decode mid-record data as a frame, silently, with
 *    everything downstream parsing.
 *
 * The record count therefore comes from the LENGTH — (length − 1) / record_size
 * — and never from a content heuristic.  A payload that is not a whole number
 * of records raises HM_WARN_TRAILING_BYTES, which is the signature of a
 * coalescing transport and is meant to be loud rather than absorbed.
 *
 * Returns HM_OK, HM_ERR_TRUNCATED if the notification is shorter than the
 * message its id implies, or HM_ERR_UNKNOWN_MESSAGE (logged and ignored per
 * §5.1, never an error condition for the session).
 *
 * `cfg` selects the stream layout for 0x90 frames.  The decoder does NOT
 * distinguish live from history: they are byte-identical and only the
 * `a1 02` … `a1 01` bracket separates them (§10.1), which is protocol state the
 * session owns.
 */
hm_status hm_codec_decode(const uint8_t *data, size_t length, hm_stream_config cfg,
                          hm_decoded *out);

/*
 * Decode one record's worth of bytes into a sample.  Exposed for the golden
 * vectors and the fuzz target.
 *
 * ⚠ `cfg` is by POINTER here and by VALUE on hm_codec_decode() above, and that
 * is a decision rather than an oversight.  hm_stream_config grew from 4 to 68
 * bytes when it took on the justification string, and these two functions run
 * per record and per unit — up to 1,600 calls per second through a full-rate
 * pull.  hm_codec_decode() runs once per notification, where the copy is
 * irrelevant and the by-value ergonomics are worth more.
 */
hm_status hm_codec_decode_record(const uint8_t *data, size_t length,
                                 const hm_stream_config *cfg, hm_sample *out);

/* Scale raw counts into the float mirrors and set the pinned mask. */
void hm_codec_scale_unit(hm_unit_sample *unit, const hm_stream_config *cfg);

/* Big-endian readers, shared with the command encoder's tests. */
static inline uint16_t hm_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline int16_t hm_be16s(const uint8_t *p)
{
    return (int16_t)hm_be16(p);
}

#endif /* HM_CODEC_H */
