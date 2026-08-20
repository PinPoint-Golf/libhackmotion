/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_presence.c — the calibration presence check and the reference-pose
 * anchor it produces.
 *
 * ⚠ Everything here is a PRESENCE test.  §8.2 measures this figure inverting —
 * a no-raise calibration scored 0.70° against 1.96° for the correct routine —
 * so nothing in this file ranks two attempts or produces a score, and a test
 * that did would be asserting the wrong contract.
 */
#include "wr_test.h"
#include "wr_wire.h"
#include "fixtures/wr_fixtures.h"

#include "wr_codec.h"
#include "wr_presence.h"
#include "wrist/quat.h"

/*
 * Build a sample whose palm is rotated `deg` about x relative to the arm.
 *
 * ⚠ The arm is given a GENERAL orientation rather than the identity, because
 * the identity is a quantisation trap the real device never presents.  With the
 * arm at identity the relative angle depends on the palm's `w` alone, and one
 * Q14 LSB of `w` near 1.0 is 1.27° — so the fixture could not express the
 * sub-degree angles §8.2 measures.  With both quaternions general, all four
 * components carry the difference and the resolution is ~0.007°.  On the wire
 * both units always hold absolute world→body attitudes, so this is the
 * realistic case and the identity was the artificial one.
 */
static void quat_to_raw(const float q[4], int16_t out[4])
{
    for (int i = 0; i < 4; ++i) {
        out[i] = (int16_t)lrintf(q[i] * 16384.0f);
    }
}

/* Angular distance between two unit quaternions, in degrees. */
static float angle_between(const float a[4], const float b[4])
{
    float d = wr_quat_dot(a, b);
    if (d < 0.0f) {
        d = -d;
    }
    if (d > 1.0f) {
        d = 1.0f;
    }
    return 2.0f * acosf(d) * (180.0f / 3.14159265f);
}

/*
 * Build a sample where the arm is rotated `wobble_deg` about world z — a
 * whole-arm movement that carries BOTH units together, so the relative rotation
 * is unchanged and only an absolute measurement can see it.
 */
static wr_sample sample_wobbled(float rel_deg, float wobble_deg, uint32_t index,
                                uint16_t arm_ticks, uint16_t palm_ticks)
{
    uint8_t buf[64];
    wr_wire_block arm = wr_wire_identity_block(arm_ticks);
    wr_wire_block palm = wr_wire_identity_block(palm_ticks);
    wr_stream_config cfg = wr_stream_config_default();
    wr_decoded d;
    float q_base[4], q_wob[4], q_arm[4], q_rel[4], q_palm[4];
    float s = 0.267261f;
    float a = 30.0f * 0.5f * 3.14159265f / 180.0f;
    float hw = wobble_deg * 0.5f * 3.14159265f / 180.0f;
    float hr = rel_deg * 0.5f * 3.14159265f / 180.0f;
    size_t n;

    q_base[0] = cosf(a);
    q_base[1] = sinf(a) * s;
    q_base[2] = sinf(a) * s * 2.0f;
    q_base[3] = sinf(a) * s * 3.0f;
    wr_quat_normalise(q_base);

    q_wob[0] = cosf(hw);
    q_wob[1] = 0.0f;
    q_wob[2] = 0.0f;
    q_wob[3] = sinf(hw);

    /* world→body, so a world-frame rotation post-multiplies. */
    wr_quat_multiply(q_base, q_wob, q_arm);
    wr_quat_normalise(q_arm);

    q_rel[0] = cosf(hr);
    q_rel[1] = sinf(hr);
    q_rel[2] = 0.0f;
    q_rel[3] = 0.0f;
    wr_quat_multiply(q_rel, q_arm, q_palm);
    wr_quat_normalise(q_palm);

    quat_to_raw(q_arm, arm.q);
    quat_to_raw(q_palm, palm.q);
    n = wr_wire_notification1(buf, (uint16_t)index, &arm, &palm);
    (void)wr_codec_decode(buf, n, cfg, &d);
    d.u.frame.sample[0].sample_index = index;
    d.u.frame.sample[0].skew_us = 921;
    return d.u.frame.sample[0];
}

/* As sample_at(), but with the relative rotation about a chosen axis. */
static wr_sample sample_about(float deg, char axis, uint32_t index, uint16_t arm_ticks,
                              uint16_t palm_ticks)
{
    uint8_t buf[64];
    wr_wire_block arm = wr_wire_identity_block(arm_ticks);
    wr_wire_block palm = wr_wire_identity_block(palm_ticks);
    wr_stream_config cfg = wr_stream_config_default();
    wr_decoded d;
    float q_arm[4], q_rel[4], q_palm[4];
    float s = 0.267261f;
    float a = 30.0f * 0.5f * 3.14159265f / 180.0f;
    float half = deg * 0.5f * 3.14159265f / 180.0f;
    size_t n;

    q_arm[0] = cosf(a);
    q_arm[1] = sinf(a) * s;
    q_arm[2] = sinf(a) * s * 2.0f;
    q_arm[3] = sinf(a) * s * 3.0f;
    wr_quat_normalise(q_arm);

    q_rel[0] = cosf(half);
    q_rel[1] = (axis == 'x') ? sinf(half) : 0.0f;
    q_rel[2] = (axis == 'y') ? sinf(half) : 0.0f;
    q_rel[3] = (axis == 'z') ? sinf(half) : 0.0f;

    wr_quat_multiply(q_rel, q_arm, q_palm);
    wr_quat_normalise(q_palm);

    quat_to_raw(q_arm, arm.q);
    quat_to_raw(q_palm, palm.q);
    n = wr_wire_notification1(buf, (uint16_t)index, &arm, &palm);
    (void)wr_codec_decode(buf, n, cfg, &d);
    d.u.frame.sample[0].sample_index = index;
    d.u.frame.sample[0].skew_us = 921;
    return d.u.frame.sample[0];
}

static wr_sample sample_at(float deg, uint32_t index, uint16_t arm_ticks, uint16_t palm_ticks)
{
    uint8_t buf[64];
    wr_wire_block arm = wr_wire_identity_block(arm_ticks);
    wr_wire_block palm = wr_wire_identity_block(palm_ticks);
    wr_stream_config cfg = wr_stream_config_default();
    wr_decoded d;
    float half = deg * 0.5f * 3.14159265f / 180.0f;
    float q_arm[4], q_rel[4], q_palm[4];
    float s = 0.267261f; /* (1,2,3)/|(1,2,3)| */
    float a = 30.0f * 0.5f * 3.14159265f / 180.0f;
    size_t n;

    /* A generic arm attitude: 30° about (1,2,3). */
    q_arm[0] = cosf(a);
    q_arm[1] = sinf(a) * s;
    q_arm[2] = sinf(a) * s * 2.0f;
    q_arm[3] = sinf(a) * s * 3.0f;
    wr_quat_normalise(q_arm);

    q_rel[0] = cosf(half);
    q_rel[1] = sinf(half);
    q_rel[2] = 0.0f;
    q_rel[3] = 0.0f;

    /* q_rel = q_palm ⊗ q_arm*  ⇒  q_palm = q_rel ⊗ q_arm  (§6.7). */
    wr_quat_multiply(q_rel, q_arm, q_palm);
    wr_quat_normalise(q_palm);

    quat_to_raw(q_arm, arm.q);
    quat_to_raw(q_palm, palm.q);

    n = wr_wire_notification1(buf, (uint16_t)index, &arm, &palm);
    (void)wr_codec_decode(buf, n, cfg, &d);
    d.u.frame.sample[0].sample_index = index;
    d.u.frame.sample[0].skew_us = 921;
    return d.u.frame.sample[0];
}

/* §8.2's three populations, an order of magnitude apart. */
WR_TEST(presence_classifies_the_measured_populations)
{
    /* Calibration applied: 0.36° / 0.79°, and 3.73-3.80° holding the pose. */
    WR_ASSERT_EQ(wr_presence_classify(0.36f), WR_CAL_CALIBRATED);
    WR_ASSERT_EQ(wr_presence_classify(0.79f), WR_CAL_CALIBRATED);
    WR_ASSERT_EQ(wr_presence_classify(3.80f), WR_CAL_CALIBRATED);

    /* Uncalibrated, or after a power cycle: 14.36-15.76°.
     * After a plain disconnect: 18.80°. */
    WR_ASSERT_EQ(wr_presence_classify(15.01f), WR_CAL_UNCALIBRATED);
    WR_ASSERT_EQ(wr_presence_classify(18.80f), WR_CAL_UNCALIBRATED);

    /* ⚠ Between the two populations is evidence of NEITHER, and must not be
     * guessed either way — guessing is how a presence check becomes a score. */
    WR_ASSERT_EQ(wr_presence_classify(8.0f), WR_CAL_UNKNOWN);
    WR_ASSERT_EQ(wr_presence_classify(NAN), WR_CAL_UNKNOWN);

    /* The thresholds sit inside the gap, with margin on both sides. */
    WR_ASSERT(WR_PRESENCE_CALIBRATED_MAX_DEG > 3.80f);
    WR_ASSERT(WR_PRESENCE_ABSENT_MIN_DEG < 14.36f);
}

/*
 * ⚠ R14.  The device applies calibration in ITS anatomical frame and the
 * 64-byte result is deliberately undecoded, so a consumer with its own
 * convention must solve the constant rotation between the two.  The pair at a
 * pose the application declared known IS that anchor — and the library already
 * collected it to compute the angle.
 */
WR_TEST(presence_keeps_the_reference_pose_anchor)
{
    wr_sample run[5];
    wr_calibration_presence_event ev;
    size_t chosen = (size_t)-1;

    run[0] = sample_at(12.6f, 100, 1000, 1059);
    run[1] = sample_at(12.9f, 108, 1642, 1701);
    run[2] = sample_at(12.7f, 116, 2284, 2343);
    run[3] = sample_at(13.1f, 124, 2926, 2985);
    run[4] = sample_at(12.5f, 132, 3568, 3627);

    WR_ASSERT_EQ(wr_presence_select_reference(run, 5, &ev), WR_OK);
    WR_ASSERT_EQ(ev.samples_used, 5);
    WR_ASSERT_NEAR(ev.relative_angle_deg, 12.76f, 0.3);

    /* ⚠ A REAL measured record, not a synthetic average: the anchor is one of
     * the samples that arrived, and it carries that record's index and skew. */
    for (size_t i = 0; i < 5; ++i) {
        if (ev.sample_index == run[i].sample_index) {
            chosen = i;
        }
    }
    WR_ASSERT_MSG(chosen != (size_t)-1, "the anchor must be one of the measured records");
    WR_ASSERT_EQ(ev.skew_us, 921);

    /* The pair reproduces the reported angle exactly, because it is the pair
     * the angle was computed from — an averaged pair could not promise this. */
    {
        wr_sample rebuilt;
        memset(&rebuilt, 0, sizeof(rebuilt));
        for (int k = 0; k < 4; ++k) {
            rebuilt.lower_arm.q_world_to_body[k] = ev.q_lower_arm[k];
            rebuilt.palm.q_world_to_body[k] = ev.q_palm[k];
        }
        WR_ASSERT_NEAR(wr_relative_angle_deg(&rebuilt), ev.relative_angle_deg, 1e-4);
    }

    /* The quaternions are the ones that were measured, verbatim. */
    WR_ASSERT_NEAR(ev.q_lower_arm[0], run[chosen].lower_arm.q_world_to_body[0], 1e-9);
    WR_ASSERT_NEAR(ev.q_palm[1], run[chosen].palm.q_world_to_body[1], 1e-9);

    /* And it is the record nearest the run's centre, not an arbitrary one. */
    WR_ASSERT_MSG(chosen != 3 && chosen != 4, "the medoid must not be an extreme of the run");
}

/*
 * ⚠ THE QUANTISATION FLOOR, and why the estimator is a medoid of ROTATIONS
 * rather than a median of ANGLES.
 *
 * Q14 quaternions put ~1.2e-4 of noise on the dot product, and
 * dθ = 2·δdot / sin(θ/2) blows that up near zero: a true 0.36° can read
 * anywhere in ±4.45°, including exactly 0.0.  A median of per-sample angles
 * inherits that, because acos rectifies the noise and biases the result high.
 *
 * The presence DECISION is untouched — the populations are ≤3.80° and ≥14.36°
 * against thresholds of 6° and 10° — but the reported figure at the calibrated
 * end is at the floor, which is a second, independent reason never to rank two
 * calibrations on it.
 */
WR_TEST(presence_survives_the_sub_degree_quantisation_floor)
{
    wr_sample run[16];
    wr_calibration_presence_event ev;

    for (uint32_t i = 0; i < 16u; ++i) {
        run[i] = sample_at(0.7f, 100u + i * 8u, (uint16_t)(1000u + i * 642u),
                           (uint16_t)(1059u + i * 642u));
    }
    WR_ASSERT_EQ(wr_presence_select_reference(run, 16, &ev), WR_OK);

    /* The number itself is untrustworthy at this scale — the test does not
     * pretend otherwise, and asserts only what the specification relies on. */
    WR_ASSERT_MSG(ev.state == WR_CAL_CALIBRATED,
                  "a sub-degree pose must still classify as calibrated");
    WR_ASSERT(ev.relative_angle_deg < WR_PRESENCE_CALIBRATED_MAX_DEG);

    /* And the anchor is still a real, usable pair regardless of the noise. */
    WR_ASSERT(ev.sample_index >= 100u);
    {
        float n2 = 0.0f;
        for (int k = 0; k < 4; ++k) {
            n2 += ev.q_palm[k] * ev.q_palm[k];
        }
        WR_ASSERT_NEAR(sqrtf(n2), 1.0f, 1e-3);
    }
}

/*
 * ⚠ R21 — THIS TEST PINS THE CHOICE, NOT THE OUTCOME.
 *
 * Every other presence test passes identically under a median-of-per-sample-
 * ANGLES implementation, which is the biased estimator wr_presence.c exists to
 * reject.  So a maintainer could revert to it and nothing would fail.
 *
 * This run separates the two.  Four rotations cluster about +x at 9.0, 9.5,
 * 10.5 and 11.0°; a fifth is 10.0° about +z — a different axis entirely.
 *
 *   median of ANGLES    → the 10.0° outlier, because its magnitude is central
 *   medoid of ROTATIONS → one of the x-cluster, because the mean rotation is
 *                         dominated by them and the outlier is ~14° away
 *
 * A median of angles cannot see the axis at all.  That is the whole point.
 */
WR_TEST(presence_uses_the_rotation_medoid_not_the_angle_median)
{
    wr_sample run[5];
    wr_calibration_presence_event ev;
    float angles[5];
    size_t angle_median_i = 0;

    run[0] = sample_about(9.0f, 'x', 100, 1000, 1059);
    run[1] = sample_about(9.5f, 'x', 108, 1642, 1701);
    run[2] = sample_about(10.0f, 'z', 116, 2284, 2343); /* the odd axis out */
    run[3] = sample_about(10.5f, 'x', 124, 2926, 2985);
    run[4] = sample_about(11.0f, 'x', 132, 3568, 3627);

    /* What a median of per-sample angles would have chosen. */
    for (size_t i = 0; i < 5; ++i) {
        angles[i] = wr_relative_angle_deg(&run[i]);
    }
    for (size_t i = 0; i < 5; ++i) {
        size_t below = 0;
        for (size_t j = 0; j < 5; ++j) {
            if (angles[j] < angles[i]) {
                below++;
            }
        }
        if (below == 2u) {
            angle_median_i = i;
        }
    }
    WR_ASSERT_MSG(angle_median_i == 2, "the fixture must make the angle median the outlier");

    WR_ASSERT_EQ(wr_presence_select_reference(run, 5, &ev), WR_OK);
    WR_ASSERT_MSG(ev.sample_index != run[2].sample_index,
                  "a rotation medoid must reject an outlier the angle median accepts");

    /*
     * And it must have chosen from the dominant axis cluster.  Which member is
     * not the interesting part — averaging rotations about differing axes
     * shrinks the mean's angle, so the nearest cluster member is the lowest —
     * and it is an artefact of a deliberately pathological fixture.  A real
     * held pose has every relative rotation about the same axis, where no such
     * shrinkage occurs.  What matters is that the choice saw the AXIS at all,
     * which a median of scalar angles cannot.
     */
    {
        wr_sample chosen;
        float rel[4];
        size_t i;
        for (i = 0; i < 5; ++i) {
            if (run[i].sample_index == ev.sample_index) {
                chosen = run[i];
            }
        }
        wr_quat_relative(&chosen, rel);
        WR_ASSERT_MSG(fabsf(rel[1]) > 0.05f, "the medoid must be one of the x-axis cluster");
        WR_ASSERT_MSG(fabsf(rel[3]) < 0.01f, "and not the z-axis outlier");
    }
}

/*
 * ⚠ R22 — the averaged pose, and the wobble that says whether to trust it.
 *
 * A person holding a declared pose still moves 0.5-2°, against ~0.007° of Q14
 * quantisation, and the medoid cannot remove it: the medoid is chosen on the
 * RELATIVE rotation, which is blind to a whole-arm movement that carries both
 * units together.  So the run below wobbles both units together — the relative
 * rotation is constant throughout, and only the absolute mean can see it.
 */
WR_TEST(presence_averages_the_absolute_pose_and_reports_the_wobble)
{
    wr_sample run[9];
    wr_calibration_presence_event ev;
    float rel_spread = 0.0f;
    float first_angle;

    /* A steady 12° wrist angle, while the whole arm swings ±1.5° about z. */
    for (uint32_t i = 0; i < 9u; ++i) {
        float wobble = ((float)i - 4.0f) * 0.375f; /* −1.5 … +1.5 degrees */
        run[i] = sample_wobbled(12.0f, wobble, 100u + i * 8u, (uint16_t)(1000u + i * 642u),
                                (uint16_t)(1059u + i * 642u));
    }

    WR_ASSERT_EQ(wr_presence_select_reference(run, 9, &ev), WR_OK);

    /* ⚠ The relative rotation is constant, so the medoid selection cannot see
     * the wobble at all — which is exactly why the absolute mean is needed. */
    first_angle = wr_relative_angle_deg(&run[0]);
    for (size_t i = 1; i < 9; ++i) {
        float d = fabsf(wr_relative_angle_deg(&run[i]) - first_angle);
        if (d > rel_spread) {
            rel_spread = d;
        }
    }
    WR_ASSERT_MSG(rel_spread < 0.1f, "the fixture's wobble must be common-mode");

    /* The spread names the wobble the medoid is blind to. */
    WR_ASSERT_NEAR(ev.pose_spread_deg[WR_UNIT_LOWER_ARM], 1.5, 0.2);
    WR_ASSERT_NEAR(ev.pose_spread_deg[WR_UNIT_PALM], 1.5, 0.2);

    /* The means are unit quaternions sitting at the centre of the run, closer
     * to it than the medoid record is. */
    {
        float n_arm = 0.0f;
        float d_mean, d_medoid;
        for (int k = 0; k < 4; ++k) {
            n_arm += ev.q_lower_arm_mean[k] * ev.q_lower_arm_mean[k];
        }
        WR_ASSERT_NEAR(sqrtf(n_arm), 1.0f, 1e-5);

        /* Distance from the run's true centre (sample 4, zero wobble). */
        d_mean = angle_between(ev.q_lower_arm_mean, run[4].lower_arm.q_world_to_body);
        d_medoid = angle_between(ev.q_lower_arm, run[4].lower_arm.q_world_to_body);
        WR_ASSERT_MSG(d_mean <= d_medoid + 1e-3f,
                      "the averaged pose must be at least as central as the medoid record");
        WR_ASSERT(d_mean < 0.5f);
    }

    /* And the medoid pair is still a real record with a real skew. */
    WR_ASSERT_EQ(ev.skew_us, 921);
    WR_ASSERT(ev.sample_index >= 100u);
}

/* The far end of the gap is well resolved, which is what the check relies on. */
WR_TEST(presence_resolves_the_uncalibrated_population_precisely)
{
    wr_sample run[8];
    wr_calibration_presence_event ev;

    for (uint32_t i = 0; i < 8u; ++i) {
        run[i] = sample_at(15.01f, 200u + i * 8u, (uint16_t)(500u + i * 642u),
                           (uint16_t)(559u + i * 642u));
    }
    WR_ASSERT_EQ(wr_presence_select_reference(run, 8, &ev), WR_OK);
    WR_ASSERT_NEAR(ev.relative_angle_deg, 15.01f, 0.2);
    WR_ASSERT_EQ(ev.state, WR_CAL_UNCALIBRATED);
}

/*
 * An uncalibrated run is classified as such and still yields its anchor — a
 * consumer wants the pair even when the verdict is "this did not take".
 *
 * ⚠ This case also pins a bug worth not repeating: the medoid must be chosen by
 * ANGULAR distance, which means normalising each relative rotation first.
 * Decoded quaternions are unit only to ~4e-5 (|q| = 16384.7 against a divisor
 * of 16384), and at 15° that magnitude wander is larger than the dot-product
 * difference a 0.7° separation produces — so comparing them un-normalised
 * ranks by rounding error and picks an extreme of the run.
 */
WR_TEST(presence_reports_an_uncalibrated_run_with_its_anchor)
{
    wr_sample run[3];
    wr_calibration_presence_event ev;

    run[0] = sample_at(14.4f, 10, 1000, 1059);
    run[1] = sample_at(15.0f, 18, 1642, 1701);
    run[2] = sample_at(15.8f, 26, 2284, 2343);

    WR_ASSERT_EQ(wr_presence_select_reference(run, 3, &ev), WR_OK);
    WR_ASSERT_NEAR(ev.relative_angle_deg, 15.0f, 0.5);
    WR_ASSERT_EQ(ev.state, WR_CAL_UNCALIBRATED);
    WR_ASSERT_EQ(ev.sample_index, 18);
    WR_ASSERT(ev.q_palm[1] != 0.0f);
}

WR_TEST(presence_selection_is_defensive)
{
    wr_sample one = sample_at(1.0f, 5, 100, 159);
    wr_calibration_presence_event ev;

    /* A single sample is a legitimate run of one. */
    WR_ASSERT_EQ(wr_presence_select_reference(&one, 1, &ev), WR_OK);
    WR_ASSERT_EQ(ev.samples_used, 1);
    WR_ASSERT_EQ(ev.sample_index, 5);

    WR_ASSERT_EQ(wr_presence_select_reference(NULL, 3, &ev), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_presence_select_reference(&one, 0, &ev), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_presence_select_reference(&one, 1, NULL), WR_ERR_INVALID_ARG);
}

/* An over-long run is capped rather than overrunning the working arrays. */
WR_TEST(presence_caps_an_overlong_run)
{
    wr_sample run[WR_PRESENCE_MAX_SAMPLES + 16u];
    wr_calibration_presence_event ev;

    for (uint32_t i = 0; i < WR_PRESENCE_MAX_SAMPLES + 16u; ++i) {
        run[i] = sample_at(1.0f + (float)i * 0.01f, i, (uint16_t)(1000u + i),
                           (uint16_t)(1059u + i));
    }
    WR_ASSERT_EQ(wr_presence_select_reference(run, WR_PRESENCE_MAX_SAMPLES + 16u, &ev), WR_OK);
    WR_ASSERT_EQ(ev.samples_used, WR_PRESENCE_MAX_SAMPLES);
    WR_ASSERT(ev.sample_index < WR_PRESENCE_MAX_SAMPLES);
}

WR_TEST_MAIN()
