/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wrist/sample.h — the one sample type, live and history alike.
 *
 * There is a single POD layout with a version stamp (api-request §2.15 C11).
 * A live sample and a history sample differ in how they were obtained and in
 * which fields are meaningful, not in their shape — so a consumer writes one
 * conversion loop, and a recording made today is still decodable when the
 * struct grows.
 */
#ifndef WRIST_SAMPLE_H
#define WRIST_SAMPLE_H

#include "wrist/types.h"
#include "wrist/config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bumped whenever a field's meaning changes or the struct grows.  Carried on
 * every wr_sample_block so a persisted capture stays readable. */
#define WR_SAMPLE_LAYOUT_VERSION 1u

/* ------------------------------------------------------------------------ */
/* The two units                                                             */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THE ORDER IS FIXED BY THE CABLE, NOT BY CONVENTION.  Spec §6.3: the two
 * units are one BLE peripheral joined by a cable; the FIRST wire block is the
 * lower-arm unit and the SECOND is the palm unit.
 *
 * A consumer that swaps them produces a plausible-looking wrist angle that is
 * simply MIRRORED — a silent bug that every plausibility check passes.  That is
 * why this API names the units and does not hand out unit[0] / unit[1].
 *
 * There is a physical check, and it is in the test suite rather than in a
 * comment: under rotation the palm unit sits at the larger radius and reads a
 * consistently higher linear-acceleration magnitude — 31-51 m/s² more across
 * five golf swings (§6.4).
 */
typedef enum wr_unit {
    WR_UNIT_LOWER_ARM = 0,
    WR_UNIT_PALM      = 1,
    WR_UNIT_COUNT     = 2
} wr_unit;

/* Channels that can saturate.  Quaternion components cannot: ±2.0 is
 * representable in Q14 and a unit quaternion never exceeds ±1 (§6.4). */
typedef enum wr_channel {
    WR_CH_ACCEL_X = 0,
    WR_CH_ACCEL_Y,
    WR_CH_ACCEL_Z,
    WR_CH_GYRO_X,
    WR_CH_GYRO_Y,
    WR_CH_GYRO_Z,
    WR_CHANNEL_COUNT
} wr_channel;

/* ------------------------------------------------------------------------ */
/* Scales (spec §6.4)                                                        */
/* ------------------------------------------------------------------------ */
#define WR_QUAT_SCALE            16384.0f  /* Q14 */
#define WR_QUAT_NORM_NOMINAL     16384.7f  /* measured over 6,064 records, ±0.41 */
/*
 * The cheapest structural check that a frame has been located correctly.
 *
 * 64 counts is 0.4 % — over 150× the measured ±0.41 spread, so a correctly
 * located frame can never trip it, and still tight enough that a decode
 * misaligned by even one byte will.  It was 512 while this check also decided
 * frame boundaries; freed from that duty by the one-call-one-notification
 * contract, it can be as tight as the evidence supports.
 */
#define WR_QUAT_NORM_TOLERANCE      64.0f
#define WR_ACCEL_LSB_MPS2         0.0098f  /* 1 LSB = 1 mg */
#define WR_ACCEL_FULL_SCALE_MPS2   321.0f  /* ±32.8 g */
#define WR_PIN_THRESHOLD           32767   /* |count| >= this is a pinned sample */

/* ------------------------------------------------------------------------ */
/* One unit's reading                                                        */
/* ------------------------------------------------------------------------ */
typedef struct wr_unit_sample {
    /*
     * ⚠ THE STREAMED QUATERNION MAPS WORLD → BODY.  Spec §6.7.  This is the
     * CONJUGATE of the convention most IMU code assumes, and it determines
     * every composition order downstream:
     *
     *     q_rel = q_palm ⊗ q_arm*        NOT  q_arm* ⊗ q_palm
     *     r     = q(t+1) ⊗ q(t)*         NOT  q(t)*  ⊗ q(t+1)
     *
     * Getting it backwards leaves the wrist ANGLE correct — 2·acos|q_a·q_b| is
     * convention-blind — while mirroring every decomposed component.  So the
     * obvious validation cannot catch it; verify against a known single-axis
     * motion instead.  The name of this field is the warning.
     *
     * Order is w, x, y, z.
     *
     * ⚠⚠ THE HAZARD IS HERE, NOT IN wr_quat_relative().  Reading these two
     * fields and feeding them into relative-rotation code you already have is
     * the reachable mistake, and it reaches past every warning this library
     * carries — because it never calls the function that carries them.
     *
     *   - wr_quat_relative() is the blessed path.  Use it.
     *   - If your existing code composes the other way round — `q_arm* ⊗ q_palm`,
     *     the inverse-first order, which plenty of IMU code uses — it is NOT
     *     wrong.  It is the same rotation expressed in the other frame, and the
     *     two are related by
     *
     *         q_rel_inverse_first = q_arm* ⊗ q_rel_wrist ⊗ q_arm
     *
     *     Pick one and convert; do not mix them.  ⚠ The ANGLE is identical
     *     under both (§6.7), so a consumer can hold a perfectly plausible wrist
     *     display with every decomposed component sign-inverted, and nothing
     *     will say so.  test_quat.c pins this identity so the formula above
     *     cannot rot.
     */
    int16_t  q_world_to_body_raw[4];

    /*
     * ⚠ GRAVITY-REMOVED linear acceleration, not a raw accelerometer reading.
     * It reads ≈0 at rest.  The name is deliberate: anything called `accel`
     * will be misread by someone (api-request §2.3).
     */
    int16_t  linear_accel_raw[3];

    int16_t  gyro_raw[3];         /* sensor frame; shares the quaternion's axes exactly */
    uint16_t ticks_raw;           /* 15.6 µs/tick, wraps every 1.023 s.  0 if !has_ticks */
    uint8_t  has_ticks;
    uint8_t  pinned_mask;         /* bit per wr_channel that hit WR_PIN_THRESHOLD */

    /* Scaled forms.  The RAW counts above are authoritative: they stay correct
     * under any configuration, and a recording read back under the wrong gyro
     * divisor is silently wrong (api-request §2.3, B12). */
    float    q_world_to_body[4];
    float    linear_accel_mps2[3];
    float    gyro_dps[3];

    /*
     * Unwrapped device time for THIS unit, microseconds since the start of the
     * stream, derived from the tick counter with the wrap resolved by the
     * sample index (§10.2 — no host clock involved, works identically for
     * history).  WR_TIME_UNKNOWN only when the configuration carries no ticks.
     *
     * ⚠ THIS DOES NOT DEPEND ON THE CLOCK FIT.  It is populated whether the fit
     * is absent, short-baselined, degenerate or stale, because it is derived
     * from the frame alone.  Analysis anchored on device time — comparing two
     * instruments at matched events rather than on a shared clock — needs
     * nothing from the host mapping and never waits for it.
     */
    wr_time_us device_time_us;
} wr_unit_sample;

/* ------------------------------------------------------------------------ */
/* Sample flags and provenance                                               */
/* ------------------------------------------------------------------------ */
typedef enum wr_sample_source {
    WR_SOURCE_LIVE    = 0,  /* arrival is real time; these pairs ARE the clock fit */
    WR_SOURCE_HISTORY = 1,  /* arrival is meaningless; host time comes from the fit */
    WR_SOURCE_REPLAY  = 2   /* decoded from a wire recording */
} wr_sample_source;

/*
 * ⚠ Per-sample, not device-level.  api-request §2.4.
 *
 * Pre-calibration quaternions are perfectly valid geometry and completely
 * meaningless anatomically — §8.1 puts the raw mounting offset at 11-15° at a
 * straight wrist.  The transform is applied ON-DEVICE and is not recoverable
 * later, so if the recording does not carry this flag the mistake is permanent
 * and invisible.
 *
 * ⚠ UNKNOWN IS NOT A HOPEFUL CALIBRATED.  It is what this field reads from the
 * moment `0x94` says the device re-referenced its own stream until a presence
 * check has been measured and passed — the transform is applied, and whether it
 * took is exactly what nothing on the wire reports (§8.2).  Skipping
 * wr_calibration_confirm_reference_pose() leaves every later sample here.
 *
 * ⚠ NO SAMPLE EVER CARRIES WR_CAL_LOST, and that is a decision rather than an
 * omission.  §8.3 measures calibration being destroyed by a PLAIN DISCONNECT
 * (0.70° before dropping the link, 18.80° after, strap untouched) — but no
 * sample can be captured between the loss and the notice, so a per-sample
 * `LOST` would label nothing.  Link-down drives this straight to UNCALIBRATED.
 *
 * The value exists for `wr_calibration_span` on a history block (history.h),
 * where it does work UNCALIBRATED cannot: `state_at_start = CALIBRATED` with
 * `state_at_end = LOST` distinguishes "never calibrated" from "calibrated, and
 * then it went away inside this block".
 */
typedef enum wr_calibration_state {
    WR_CAL_UNKNOWN      = 0,
    WR_CAL_UNCALIBRATED = 1,
    WR_CAL_CALIBRATED   = 2,
    WR_CAL_LOST         = 3
} wr_calibration_state;

typedef enum wr_sample_flag {
    WR_SAMPLE_PINNED               = 1u << 0, /* some channel saturated — silently */
    WR_SAMPLE_NOT_TIME_ALIGNABLE   = 1u << 1, /* stream carries no device clock (§6.3.1) */
    WR_SAMPLE_NONSTANDARD_CONFIG   = 1u << 2, /* not 0x7e — relative-angle cancellation unmeasured */
    WR_SAMPLE_HOST_TIME_EXTRAPOLATED = 1u << 3, /* mapped beyond the fit's observations */
    WR_SAMPLE_NO_FIT               = 1u << 4, /* host_time_us is WR_TIME_UNKNOWN */
    WR_SAMPLE_QUAT_NORM_SUSPECT    = 1u << 5, /* |q| outside tolerance — decode may be misaligned */
    WR_SAMPLE_TICKS_MISSING        = 1u << 6, /* no per-unit device time available */
    WR_SAMPLE_INDEX_MISSING        = 1u << 7  /* legacy 0x7f layout: no record header */
} wr_sample_flag;

/* ------------------------------------------------------------------------ */
/* A sample                                                                  */
/* ------------------------------------------------------------------------ */
typedef struct wr_sample {
    uint64_t   stream_id;        /* which `a0 01 7e` this index space belongs to (B4) */
    uint32_t   sample_index;     /* UNWRAPPED record header index; monotonic within a stream */
    uint16_t   sample_index_raw; /* as it appeared on the wire */
    uint8_t    source;           /* wr_sample_source */
    uint8_t    calibration;      /* wr_calibration_state, at the instant of capture */

    uint16_t   flags;            /* wr_sample_flag */
    uint8_t    config_bits;      /* the `a0 01 <cfg>` byte that produced this sample */
    uint8_t    reserved0;

    /*
     * ⚠ palm − lower_arm, microseconds.  §10.3 measures a STABLE 59 ticks
     * (0.92 ms) whose physical meaning is unresolved — real sampling skew or
     * arbitrary phase between two free-running counters cannot be told apart
     * from the counters alone, and it CANNOT be settled by a shared impulse.
     *
     * It is worth ~0.9° in the relative angle at 1,000 °/s, which is the
     * primary output.  Being stable it is subtractable as a constant — but it
     * is carried explicitly rather than silently pairing the two blocks as
     * simultaneous.
     *
     * ⚠ THE TWO BLOCKS *ARE* ONE SAMPLE, and this field is not evidence against
     * that.  §6.3: a record carries ONE header, read once and applying to every
     * block in it, so the two units share a sample index by construction.  What
     * they do not share is a tick counter — two free-running MCU timers, hence
     * this number.  One sample, two clocks.
     *
     * ⚠ THIS IS THIS RECORD'S DIFFERENCE, AND ONE OF THEM IS NOT THE SKEW.
     * Single-record readings are dominated by ±½-sample pairing jitter and
     * scatter widely — 89 and 99 ticks on two consecutive records of one
     * capture, against a session median of 59 (§10.3).  The stable 0.92 ms is a
     * SESSION-LEVEL figure and only appears after aggregating.
     *
     * The per-record value is carried anyway, because it is what was measured
     * and aggregating it is the consumer's choice to make — but a client that
     * subtracts one sample's `skew_us` from that sample is subtracting mostly
     * jitter, and should take a median over a run instead.
     */
    int32_t    skew_us;

    /*
     * Host time, mapped through the clock fit.  For live samples this is the
     * fit applied to the index, NOT the arrival instant — arrival is what the
     * fit is built from, and is one-sidedly late.
     */
    wr_time_us host_time_us;

    /* Arrival instant as the transport stamped it.  WR_TIME_UNKNOWN for
     * history records: bulk arrival timestamps carry no information (§10). */
    wr_time_us host_recv_us;

    /*
     * ⚠ TWO NUMBERS, NOT ONE (api-request B2).  See wr_clock_error.
     *   precision_us   link jitter + extrapolation — GATE ON THIS
     *   uncertainty_us precision + §10's measured, uncorrected systematic —
     *                  the honest total, for the capture's provenance
     * Gating on the total would refuse every pull after the first second of a
     * session; recording only the precision would understate the error by three
     * orders of magnitude by the end of one.  Both travel with the sample.
     */
    uint32_t   precision_us;
    uint32_t   uncertainty_us;

    wr_unit_sample lower_arm;    /* wire block 0 */
    wr_unit_sample palm;         /* wire block 1 */
} wr_sample;

/* Index access for generic code and for language bindings that want a loop.
 * Returns NULL for an out-of-range unit. */
WR_API const wr_unit_sample *wr_sample_unit(const wr_sample *s, wr_unit unit);

/* Human-readable unit name: "lower_arm" / "palm".  Never NULL. */
WR_API const char *wr_unit_name(wr_unit unit);

/*
 * ⚠ THERE IS DELIBERATELY NO AGGREGATE ACROSS THE TWO UNITS.
 *
 * Spec §6.4: they sit 3-8 cm apart on the hand and forearm, varying with the
 * wearer, so under rotation they are at different radii and their linear
 * accelerations differ by roughly ω²r.  Through a golf swing at ~2,100 °/s the
 * palm unit read 31-51 m/s² MORE than the lower-arm unit, consistently, across
 * five swings — several g of legitimate difference.
 *
 * Do not average them.  Do not treat a disagreement as a fault.  There is no
 * "device acceleration", so this library does not offer one, and any quantity
 * a consumer computes from linear acceleration must name the unit it came from.
 *
 * The quaternions are unaffected — rotation is the same about any point on a
 * rigid segment — which is exactly why the wrist metrics are built from
 * orientation rather than from acceleration.
 */

/* ------------------------------------------------------------------------ */
/* Pinned-sample accounting (api-request §2.3)                               */
/* ------------------------------------------------------------------------ */
/*
 * int16 fields SATURATE rather than wrap, so clipped data still looks like a
 * plausible waveform — a flattened peak, not an obvious fault — and nothing in
 * the protocol reports it.  A deliberate flick of the wrist reaches 83 % of
 * full scale where a struck golf swing reaches 53-58 %, because the sensor is
 * on the wrist and not the clubhead: the worst case is whatever the SENSOR is
 * subjected to, including someone waving it about before a session starts.
 */
typedef struct wr_pinned_counts {
    uint32_t n[WR_UNIT_COUNT][WR_CHANNEL_COUNT];
    uint32_t total;
} wr_pinned_counts;

WR_API const char *wr_channel_name(wr_channel ch);
WR_API void wr_pinned_counts_reset(wr_pinned_counts *counts);
WR_API void wr_pinned_counts_add(wr_pinned_counts *counts, const wr_sample *s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WRIST_SAMPLE_H */
