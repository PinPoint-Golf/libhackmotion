/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_quat.c — the quaternion convention, which api-request §2.2 singled out
 * because "the obvious validation cannot catch it".
 */
#include "hm_test.h"
#include "hm_wire.h"
#include "fixtures/hm_fixtures.h"

#include "hm_codec.h"
#include "hackmotion/quat.h"

static hm_sample make_sample(const int16_t arm_q[4], const int16_t palm_q[4])
{
    uint8_t buf[64];
    hm_wire_block arm = hm_wire_identity_block(1000);
    hm_wire_block palm = hm_wire_identity_block(1059);
    hm_decoded d;
    size_t n;

    for (int i = 0; i < 4; ++i) {
        arm.q[i] = arm_q[i];
        palm.q[i] = palm_q[i];
    }
    n = hm_wire_notification1(buf, 42, &arm, &palm);
    (void)hm_codec_decode(buf, n, hm_stream_config_default(), &d);
    return d.u.frame.sample[0];
}

/*
 * ⚠ THE TEST api-request §2.2 ASKED FOR, and §6.7 says is the only one that
 * works: a known single-axis motion.
 *
 *   q_arm  = 180° about y,  q_palm = q_rel ⊗ q_arm with q_rel = +60° about x
 *   correct  q_palm ⊗ q_arm*  →  (cos30, +sin30, 0, 0)
 *   reversed q_arm* ⊗ q_palm  →  (cos30, −sin30, 0, 0)
 *
 * Same magnitude, inverted sign — exactly as the specification describes.
 */
HM_TEST(quat_relative_uses_palm_times_arm_conjugate)
{
    const int16_t arm_q[4] = HM_FIX_AXIS_ARM_Q;
    const int16_t palm_q[4] = HM_FIX_AXIS_PALM_Q;
    hm_sample s = make_sample(arm_q, palm_q);
    float rel[4];
    float reversed[4];
    float arm_conj[4];

    hm_quat_relative(&s, rel);

    HM_ASSERT_NEAR(rel[0], 0.86603f, 1e-3);
    HM_ASSERT_NEAR(rel[1], HM_FIX_AXIS_EXPECTED_REL_X, 1e-3);
    HM_ASSERT_NEAR(rel[2], 0.0f, 1e-3);
    HM_ASSERT_NEAR(rel[3], 0.0f, 1e-3);
    HM_ASSERT_MSG(rel[1] > 0.0f, "reversed composition order would make this negative");

    /* And the reverse really does differ, so the test has teeth. */
    hm_quat_conjugate(s.lower_arm.q_world_to_body, arm_conj);
    hm_quat_multiply(arm_conj, s.palm.q_world_to_body, reversed);
    HM_ASSERT_NEAR(reversed[1], -HM_FIX_AXIS_EXPECTED_REL_X, 1e-3);
}

/*
 * ⚠ AND THE REASON THE ABOVE IS NECESSARY: the wrist angle is convention-blind,
 * so a decode validated by "the angle looks sensible" passes with the order
 * reversed and every decomposed component sign-flipped.
 */
HM_TEST(quat_angle_cannot_detect_a_reversed_composition_order)
{
    const int16_t arm_q[4] = HM_FIX_AXIS_ARM_Q;
    const int16_t palm_q[4] = HM_FIX_AXIS_PALM_Q;
    hm_sample s = make_sample(arm_q, palm_q);
    float rel[4], reversed[4], arm_conj[4];
    float angle_correct, angle_reversed;

    hm_quat_relative(&s, rel);
    hm_quat_conjugate(s.lower_arm.q_world_to_body, arm_conj);
    hm_quat_multiply(arm_conj, s.palm.q_world_to_body, reversed);

    angle_correct = 2.0f * acosf(fabsf(rel[0])) * (180.0f / 3.14159265f);
    angle_reversed = 2.0f * acosf(fabsf(reversed[0])) * (180.0f / 3.14159265f);

    HM_ASSERT_NEAR(angle_correct, HM_FIX_AXIS_EXPECTED_ANGLE_DEG, 0.1);
    HM_ASSERT_NEAR(angle_reversed, HM_FIX_AXIS_EXPECTED_ANGLE_DEG, 0.1);
    HM_ASSERT_MSG(fabsf(angle_correct - angle_reversed) < 0.01f,
                  "if these differed, the angle check would suffice and it does not");
    HM_ASSERT_NEAR(hm_relative_angle_deg(&s), HM_FIX_AXIS_EXPECTED_ANGLE_DEG, 0.1);
}

/*
 * ⚠ R15 — the identity a consumer needs if its own code composes the other way.
 *
 * The hazard is one level above hm_quat_relative(): a consumer reads the two
 * q_world_to_body fields and feeds them into relative-rotation code it already
 * has.  Plenty of that code uses the inverse-first order, `q_arm* ⊗ q_palm`,
 * which is NOT wrong — it is the same rotation in the other frame. sample.h
 * documents the conversion, and this test is what stops that formula rotting.
 */
HM_TEST(quat_documented_conversion_between_the_two_composition_orders_holds)
{
    const int16_t arm_q[4] = HM_FIX_AXIS_ARM_Q;
    const int16_t palm_q[4] = HM_FIX_AXIS_PALM_Q;
    hm_sample s = make_sample(arm_q, palm_q);
    float arm_conj[4], ours[4], theirs[4], converted[4], tmp[4];

    hm_quat_relative(&s, ours);                                   /* q_palm ⊗ q_arm* */
    hm_quat_conjugate(s.lower_arm.q_world_to_body, arm_conj);
    hm_quat_multiply(arm_conj, s.palm.q_world_to_body, theirs);   /* q_arm* ⊗ q_palm */

    /* sample.h claims:  q_rel_inverse_first = q_arm* ⊗ q_rel_hackmotion ⊗ q_arm */
    hm_quat_multiply(arm_conj, ours, tmp);
    hm_quat_multiply(tmp, s.lower_arm.q_world_to_body, converted);

    for (int k = 0; k < 4; ++k) {
        HM_ASSERT_NEAR(converted[k], theirs[k], 1e-4);
    }

    /* ⚠ And the reason the identity is needed at all: the two differ in the
     * vector part while the ANGLE is identical, so nothing downstream catches a
     * consumer that mixed them. */
    HM_ASSERT_MSG(fabsf(ours[1] - theirs[1]) > 0.5f, "the two orders really do differ");
    HM_ASSERT_NEAR(fabsf(ours[0]), fabsf(theirs[0]), 1e-5);
}

/*
 * §8.2 — the presence check.  Uncalibrated sits at 15-19°, applied at 0.4-3.8°:
 * an order of magnitude, which is why it detects "never happened or was lost".
 * ⚠ It is NOT a quality score — §8.2 measures a no-raise calibration scoring
 * BEST — and this library never ranks on it.
 */
HM_TEST(quat_relative_angle_separates_calibrated_from_uncalibrated)
{
    const int16_t identity[4] = {16384, 0, 0, 0};
    /* ~0.7° apart: a calibration that took effect. */
    const int16_t tiny[4] = {16384, 100, 0, 0};
    /* ~15° apart: the uncalibrated mounting offset of §8.1. */
    const int16_t offset[4] = {16244, 2138, 0, 0};

    hm_sample calibrated = make_sample(identity, tiny);
    hm_sample uncalibrated = make_sample(identity, offset);

    float a_cal = hm_relative_angle_deg(&calibrated);
    float a_unc = hm_relative_angle_deg(&uncalibrated);

    HM_ASSERT(a_cal < 3.0f);
    HM_ASSERT(a_unc > 12.0f);
    HM_ASSERT(a_unc < 20.0f);
    HM_ASSERT_MSG(a_unc > a_cal * 5.0f, "the presence gap must stay an order of magnitude");
}

/* §6.4 — |q| = 16384.7 ± 0.41 over thousands of records is the structural check
 * that a frame has been located correctly. */
HM_TEST(quat_raw_norm_matches_the_structural_check)
{
    const int16_t identity[4] = {16384, 0, 0, 0};
    const int16_t diagonal[4] = {11585, 11585, 0, 0};
    HM_ASSERT_NEAR(hm_quat_raw_norm(identity), 16384.0f, 0.5);
    HM_ASSERT_NEAR(hm_quat_raw_norm(diagonal), 16384.0f, 1.0);
}

/*
 * §6.7 — the angular rate composition order is fixed by the same convention:
 * r = q(t+1) ⊗ q(t)*, and ω = −2·atan2(|r.v|, r.w)/dt.
 */
HM_TEST(quat_angular_rate_uses_the_world_to_body_order)
{
    /* 1° about +x over one sample period at 799.2 Hz → 799.2 °/s. */
    const float prev[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float next[4];
    float axis[3];
    float rate;
    const float half = 0.5f * 1.0f * 3.14159265f / 180.0f;

    next[0] = cosf(half);
    next[1] = sinf(half);
    next[2] = 0.0f;
    next[3] = 0.0f;

    rate = hm_angular_rate_dps(prev, next, 1251, axis);
    HM_ASSERT_NEAR(fabsf(rate), 799.0f, 5.0);
    HM_ASSERT_NEAR(fabsf(axis[0]), 1.0f, 1e-3);
    HM_ASSERT_NEAR(axis[1], 0.0f, 1e-3);
    HM_ASSERT_NEAR(axis[2], 0.0f, 1e-3);

    /* A non-positive dt must not divide by zero or invent a rate. */
    HM_ASSERT_NEAR(hm_angular_rate_dps(prev, next, 0, axis), 0.0f, 1e-9);
    HM_ASSERT_NEAR(axis[0], 0.0f, 1e-9);
}

HM_TEST(quat_algebra_basics)
{
    const float a[4] = {0.0f, 1.0f, 0.0f, 0.0f}; /* 180° about x */
    const float b[4] = {0.0f, 0.0f, 1.0f, 0.0f}; /* 180° about y */
    float conj[4];
    float prod[4];
    float ident[4] = {2.0f, 0.0f, 0.0f, 0.0f};

    hm_quat_conjugate(a, conj);
    HM_ASSERT_NEAR(conj[0], 0.0f, 1e-9);
    HM_ASSERT_NEAR(conj[1], -1.0f, 1e-9);

    hm_quat_multiply(a, b, prod);
    /* x then y composes to 180° about z (up to sign). */
    HM_ASSERT_NEAR(fabsf(prod[3]), 1.0f, 1e-6);
    HM_ASSERT_NEAR(prod[0], 0.0f, 1e-6);

    hm_quat_normalise(ident);
    HM_ASSERT_NEAR(ident[0], 1.0f, 1e-9);
    HM_ASSERT_NEAR(hm_quat_dot(a, a), 1.0f, 1e-9);
}

HM_TEST(quat_null_arguments_are_safe)
{
    float out[4] = {9, 9, 9, 9};
    hm_quat_relative(NULL, out);
    HM_ASSERT_NEAR(out[0], 9.0f, 1e-9); /* untouched, not crashed */
    HM_ASSERT(isnan(hm_relative_angle_deg(NULL)));
    HM_ASSERT_EQ(hm_sample_unit(NULL, HM_UNIT_PALM), NULL);
}

HM_TEST_MAIN()
