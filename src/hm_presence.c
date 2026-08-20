/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_presence.c — the calibration PRESENCE check, and the reference-pose anchor
 * it produces as a by-product.
 *
 * ⚠ PRESENCE, NOT QUALITY.  §8.2 measures the reference-pose relative angle
 * INVERTING: a calibration with no raise at all — carrying no axis information
 * whatsoever — scored 0.70° against 1.96° for the correct routine.  Nothing here
 * ranks two attempts and nothing here returns a score.  The one sound use is
 * catching "calibration never happened, or was lost", where the gap is an order
 * of magnitude.
 */
#include "hm_presence.h"
#include "hackmotion/quat.h"

#include <string.h>
#include <math.h>

hm_calibration_state hm_presence_classify(float angle_deg)
{
    if (!(angle_deg == angle_deg)) { /* NaN */
        return HM_CAL_UNKNOWN;
    }
    if (angle_deg < HM_PRESENCE_CALIBRATED_MAX_DEG) {
        return HM_CAL_CALIBRATED;
    }
    if (angle_deg > HM_PRESENCE_ABSENT_MIN_DEG) {
        return HM_CAL_UNCALIBRATED;
    }
    /* ⚠ Indeterminate.  Between the two populations §8.2 measured, so it is
     * evidence of neither.  The caller keeps whatever state it had and warns —
     * guessing here is how a presence check becomes a quality score. */
    return HM_CAL_UNKNOWN;
}

/*
 * ⚠ WHY THIS IS A MEDOID AND NOT A MEDIAN OF ANGLES.
 *
 * The relative angle is 2·acos|q_palm · q_arm|, and near zero that is violently
 * sensitive to quantisation: the quaternions arrive in Q14, so a half-LSB on
 * each of four components moves the dot product by ~1.2e-4, and
 * dθ = 2·δdot / sin(θ/2) turns that into
 *
 *     true 0.36°  →  ±4.45°      true 2.88°  →  ±0.56°
 *     true 0.72°  →  ±2.23°      true 11.5°  →  ±0.14°
 *
 * So a SINGLE sample's angle is worthless at the calibrated end — it can read
 * exactly 0.0 — and a median of per-sample angles inherits that, because acos
 * rectifies the noise and biases the result high.
 *
 * ⚠ THIS DOES NOT THREATEN THE PRESENCE DECISION.  The populations §8.2
 * measured are ≤3.80° applied and ≥14.36° absent, against thresholds of 6° and
 * 10°; the worst-case noise at 3.80° is ±0.42°.  The order-of-magnitude gap
 * swallows it entirely.
 *
 * ⚠ BUT IT IS A SECOND, INDEPENDENT REASON NEVER TO SCORE ON THIS FIGURE.  §8.2
 * shows a ranking would prefer the worst calibration on offer; the arithmetic
 * above shows that at the calibrated end it would be ranking quantisation noise
 * as well.  Two different arguments, same conclusion.
 *
 * So: average the relative ROTATIONS (where the noise is not rectified and does
 * average down), then return the real measured record nearest that average.
 * The anchor stays a pair that came off the wire together, its skew is a real
 * measurement, and its angle is the one reported.
 */
/*
 * Sign-aligned, normalised mean of one unit's absolute rotations over the run,
 * plus the largest angular deviation from it — the wobble, which is the
 * evidence for whether the mean is worth anything.
 *
 * Two passes over the samples rather than a scratch array: this runs once per
 * calibration and a 2 KB stack buffer for something used twice is not a trade
 * worth making.
 */
static void mean_of_unit(const hm_sample *samples, size_t n, hm_unit unit, float out_mean[4],
                         float *out_spread_deg)
{
    float sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float first[4];
    float worst = 0.0f;
    size_t used = 0;

    for (size_t i = 0; i < n; ++i) {
        const hm_unit_sample *u = hm_sample_unit(&samples[i], unit);
        float q[4];
        if (u == NULL) {
            continue;
        }
        for (int k = 0; k < 4; ++k) {
            q[k] = u->q_world_to_body[k];
        }
        /* ⚠ Normalise before comparing or summing: §6.4's |q| = 16384.7 against
         * a Q14 divisor of 16384 leaves these unit only to ~4e-5, which is
         * larger than the angular signal at small separations. */
        hm_quat_normalise(q);
        if (used == 0u) {
            for (int k = 0; k < 4; ++k) {
                first[k] = q[k];
            }
        } else if (hm_quat_dot(q, first) < 0.0f) {
            for (int k = 0; k < 4; ++k) {
                q[k] = -q[k];
            }
        }
        for (int k = 0; k < 4; ++k) {
            sum[k] += q[k];
        }
        used++;
    }

    if (used == 0u) {
        for (int k = 0; k < 4; ++k) {
            out_mean[k] = 0.0f;
        }
        *out_spread_deg = 0.0f;
        return;
    }

    hm_quat_normalise(sum);
    for (int k = 0; k < 4; ++k) {
        out_mean[k] = sum[k];
    }

    for (size_t i = 0; i < n; ++i) {
        const hm_unit_sample *u = hm_sample_unit(&samples[i], unit);
        float q[4];
        float d;
        float deg;
        if (u == NULL) {
            continue;
        }
        for (int k = 0; k < 4; ++k) {
            q[k] = u->q_world_to_body[k];
        }
        hm_quat_normalise(q);
        d = hm_quat_dot(q, sum);
        if (d < 0.0f) {
            d = -d;
        }
        if (d > 1.0f) {
            d = 1.0f;
        }
        deg = 2.0f * acosf(d) * (180.0f / 3.14159265358979323846f);
        if (deg > worst) {
            worst = deg;
        }
    }
    *out_spread_deg = worst;
}

static void mean_absolute(const hm_sample *samples, size_t n,
                          hm_calibration_presence_event *out)
{
    mean_of_unit(samples, n, HM_UNIT_LOWER_ARM, out->q_lower_arm_mean,
                 &out->pose_spread_deg[HM_UNIT_LOWER_ARM]);
    mean_of_unit(samples, n, HM_UNIT_PALM, out->q_palm_mean,
                 &out->pose_spread_deg[HM_UNIT_PALM]);
}

hm_status hm_presence_select_reference(const hm_sample *samples, size_t count,
                                       hm_calibration_presence_event *out)
{
    float rel[HM_PRESENCE_MAX_SAMPLES][4];
    float mean[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float best_dot = -1.0f;
    size_t n;
    size_t pick_i = 0;
    const hm_sample *pick;

    if (samples == NULL || out == NULL || count == 0u) {
        return HM_ERR_INVALID_ARG;
    }
    n = (count < HM_PRESENCE_MAX_SAMPLES) ? count : HM_PRESENCE_MAX_SAMPLES;

    for (size_t i = 0; i < n; ++i) {
        hm_quat_relative(&samples[i], rel[i]);
        if (!(rel[i][0] == rel[i][0])) {
            return HM_ERR_MALFORMED; /* a NaN quaternion has no average to take */
        }
        /*
         * ⚠ NORMALISE FIRST.  §6.4 measures |q| = 16384.7 on the wire against a
         * Q14 divisor of 16384, so decoded quaternions are unit to about 4e-5 —
         * NOT exactly unit.  A dot product between un-normalised quaternions is
         * dominated by that magnitude variation rather than by the angular
         * difference it is meant to measure: at 15° a 0.7° separation moves the
         * dot by 1.8e-5, which is smaller than the 3.3e-5 the magnitudes wander
         * by.  Comparing them un-normalised silently ranks by rounding error.
         */
        hm_quat_normalise(rel[i]);
        /* q and −q are the same rotation; align signs before summing or the
         * average collapses toward zero. */
        if (i > 0u && hm_quat_dot(rel[i], rel[0]) < 0.0f) {
            for (int k = 0; k < 4; ++k) {
                rel[i][k] = -rel[i][k];
            }
        }
        for (int k = 0; k < 4; ++k) {
            mean[k] += rel[i][k];
        }
    }
    hm_quat_normalise(mean);
    if (!(mean[0] == mean[0])) {
        return HM_ERR_MALFORMED;
    }

    /* The measured record nearest the average rotation. */
    for (size_t i = 0; i < n; ++i) {
        float d = hm_quat_dot(rel[i], mean);
        if (d < 0.0f) {
            d = -d;
        }
        if (d > best_dot) {
            best_dot = d;
            pick_i = i;
        }
    }
    pick = &samples[pick_i];

    memset(out, 0, sizeof(*out));
    out->relative_angle_deg = hm_relative_angle_deg(pick);
    for (int k = 0; k < 4; ++k) {
        out->q_lower_arm[k] = pick->lower_arm.q_world_to_body[k];
        out->q_palm[k] = pick->palm.q_world_to_body[k];
    }
    out->sample_index = pick->sample_index;
    out->skew_us = pick->skew_us;
    out->state = (uint8_t)hm_presence_classify(out->relative_angle_deg);
    out->samples_used = (uint8_t)n;

    /*
     * ⚠ And the AVERAGED ABSOLUTE pose, which is a different question.
     *
     * The medoid answers "which real record best represents the run", and it is
     * selected on the RELATIVE rotation — which is blind to a whole-arm wobble
     * carrying both units together, exactly the motion that contaminates an
     * absolute pose.  A person holding a declared pose still moves 0.5-2°,
     * against ~0.007° of Q14 quantisation, so for a frame-reconciliation solve
     * the average over the run is one to two orders better than any single
     * record however centrally chosen.
     *
     * Same machinery, applied per unit to the absolute rotations.
     */
    mean_absolute(samples, n, out);
    return HM_OK;
}
