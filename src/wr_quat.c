/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "wrist/quat.h"

#include <math.h>
#include <string.h>

void wr_quat_conjugate(const float q[4], float out[4])
{
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

void wr_quat_multiply(const float a[4], const float b[4], float out[4])
{
    float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

float wr_quat_dot(const float a[4], const float b[4])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

void wr_quat_normalise(float q[4])
{
    float n = sqrtf(wr_quat_dot(q, q));
    if (n > 0.0f) {
        q[0] /= n;
        q[1] /= n;
        q[2] /= n;
        q[3] /= n;
    }
}

void wr_quat_relative(const wr_sample *sample, float out_q_rel[4])
{
    float arm_conj[4];

    if (sample == NULL || out_q_rel == NULL) {
        return;
    }
    /*
     * ⚠ THE ONLY PLACE THIS ORDER IS WRITTEN.  Spec §6.7: the streamed
     * quaternion maps world → body, so
     *
     *     q_rel = q_palm ⊗ q_arm*      and NOT   q_arm* ⊗ q_palm.
     *
     * Reversing it leaves the wrist ANGLE correct and mirrors every decomposed
     * component, which is why no angle-based check can catch the mistake.
     */
    wr_quat_conjugate(sample->lower_arm.q_world_to_body, arm_conj);
    wr_quat_multiply(sample->palm.q_world_to_body, arm_conj, out_q_rel);
}

float wr_relative_angle_deg(const wr_sample *sample)
{
    float d;

    if (sample == NULL) {
        return NAN;
    }
    d = wr_quat_dot(sample->palm.q_world_to_body, sample->lower_arm.q_world_to_body);
    if (d < 0.0f) {
        d = -d;
    }
    if (d > 1.0f) {
        d = 1.0f;
    }
    return 2.0f * acosf(d) * (180.0f / 3.14159265358979323846f);
}

float wr_angular_rate_dps(const float q_prev[4], const float q_next[4], wr_time_us dt_us,
                          float out_axis[3])
{
    float conj[4];
    float r[4];
    float vnorm;
    float angle;
    float rate;

    if (out_axis != NULL) {
        out_axis[0] = out_axis[1] = out_axis[2] = 0.0f;
    }
    if (q_prev == NULL || q_next == NULL || dt_us <= 0) {
        return 0.0f;
    }

    /* r = q(t+1) ⊗ q(t)*   — the world→body convention fixes this order too. */
    wr_quat_conjugate(q_prev, conj);
    wr_quat_multiply(q_next, conj, r);

    vnorm = sqrtf(r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
    angle = 2.0f * atan2f(vnorm, r[0]);
    if (angle > 3.14159265358979323846f) {
        angle -= 2.0f * 3.14159265358979323846f;
    }
    rate = -angle * (180.0f / 3.14159265358979323846f) / ((float)dt_us * 1e-6f);

    if (out_axis != NULL && vnorm > 1e-9f) {
        out_axis[0] = r[1] / vnorm;
        out_axis[1] = r[2] / vnorm;
        out_axis[2] = r[3] / vnorm;
    }
    return rate;
}

float wr_quat_raw_norm(const int16_t q_raw[4])
{
    double acc = 0.0;
    for (int i = 0; i < 4; ++i) {
        acc += (double)q_raw[i] * (double)q_raw[i];
    }
    return (float)sqrt(acc);
}

const wr_unit_sample *wr_sample_unit(const wr_sample *s, wr_unit unit)
{
    if (s == NULL) {
        return NULL;
    }
    switch (unit) {
        case WR_UNIT_LOWER_ARM: return &s->lower_arm;
        case WR_UNIT_PALM:      return &s->palm;
        case WR_UNIT_COUNT:     break;
    }
    return NULL;
}

void wr_pinned_counts_reset(wr_pinned_counts *counts)
{
    memset(counts, 0, sizeof(*counts));
}

void wr_pinned_counts_add(wr_pinned_counts *counts, const wr_sample *s)
{
    static const wr_unit units[WR_UNIT_COUNT] = {WR_UNIT_LOWER_ARM, WR_UNIT_PALM};

    if (counts == NULL || s == NULL) {
        return;
    }
    for (size_t u = 0; u < (size_t)WR_UNIT_COUNT; ++u) {
        const wr_unit_sample *us = wr_sample_unit(s, units[u]);
        if (us == NULL) {
            continue;
        }
        for (size_t c = 0; c < (size_t)WR_CHANNEL_COUNT; ++c) {
            if ((us->pinned_mask & (uint8_t)(1u << c)) != 0u) {
                counts->n[u][c]++;
                counts->total++;
            }
        }
    }
}
