/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/quat.h — the minimum quaternion algebra, and the one place in this
 * library where the composition order is written down.
 *
 * ⚠ THIS IS NOT ANATOMICAL DECOMPOSITION AND NEVER WILL BE.
 *
 * Flexion/extension, radial/ulnar deviation and rotation belong to the
 * application: they need an axis convention different consumers will disagree
 * about, and deviation additionally depends on which hand the sensor is worn on
 * — which the device never asks and this library has no way to know (spec §0,
 * api-request §1, §3).  Two quaternions in, two quaternions out.
 *
 * What IS here is the relative rotation and its angle, because those are decode
 * facts rather than analysis choices: the convention is a property of the wire
 * format, getting it wrong corrupts every consumer identically, and the angle
 * is what the calibration PRESENCE check is built from.
 */
#ifndef HACKMOTION_QUAT_H
#define HACKMOTION_QUAT_H

#include "hackmotion/types.h"
#include "hackmotion/sample.h"

#ifdef __cplusplus
extern "C" {
#endif

/* All quaternions here are (w, x, y, z), and every one that comes off the wire
 * maps WORLD → BODY (spec §6.7). */

HM_API void hm_quat_conjugate(const float q[4], float out[4]);
HM_API void hm_quat_multiply(const float a[4], const float b[4], float out[4]);
HM_API void hm_quat_normalise(float q[4]);
HM_API float hm_quat_dot(const float a[4], const float b[4]);

/*
 * The relative rotation of the palm with respect to the lower arm:
 *
 *     q_rel = q_palm ⊗ q_arm*          NOT  q_arm* ⊗ q_palm
 *
 * ⚠ THE OBVIOUS VALIDATION CANNOT CATCH A REVERSED ORDER.  The wrist ANGLE is
 * convention-blind, so a client that checks its decode by confirming the angle
 * looks sensible passes with the order reversed and every decomposed component
 * sign-flipped.  Verify against a KNOWN SINGLE-AXIS MOTION instead: a pure
 * rotation about one anatomical axis should move one decomposed component and
 * leave the others near zero, and a reversed order shows as inverted signs
 * rather than wrong magnitudes.  The test suite ships exactly that fixture.
 */
HM_API void hm_quat_relative(const hm_sample *sample, float out_q_rel[4]);

/*
 * The angle of the relative rotation, in degrees: 2·acos|q_palm · q_arm|.
 *
 * ⚠ Convention-blind by construction, which is why it is fit for the
 * calibration PRESENCE check (uncalibrated sits at 15-19°, applied at 0.4-3.8°
 * — an order of magnitude apart) and unfit for anything that needs a sign.
 * ⚠ And it is NOT a calibration quality score: §8.2 measures it scoring a
 * no-raise calibration BEST.  See hm_calibration_presence_event.
 */
HM_API float hm_relative_angle_deg(const hm_sample *sample);

/*
 * The angular rate implied by two consecutive samples of ONE unit, °/s, with
 * the axis in the body frame.  The world→body convention fixes the order:
 *
 *     r = q(t+1) ⊗ q(t)*        ω_body = −2·atan2(|r.v|, r.w)/dt · unit(r.v)
 *
 * `dt_us` must come from the device clock — never from arrival times, which BLE
 * bunching makes unusable (§6.6).  Returns 0 and leaves `out_axis` zeroed when
 * dt_us <= 0.
 */
HM_API float hm_angular_rate_dps(const float q_prev[4], const float q_next[4],
                                 hm_time_us dt_us, float out_axis[3]);

/* |q| in raw wire counts, for the structural check of §6.4: a correctly located
 * frame has |q| = 16384.7 ± 0.41 over thousands of records, and the norm is the
 * cheapest evidence that the decode is aligned. */
HM_API float hm_quat_raw_norm(const int16_t q_raw[4]);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_QUAT_H */
