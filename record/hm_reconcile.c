/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_reconcile.c — replay a `.hmwire` recording against the specification and
 * report, claim by claim, what the device actually did.
 *
 * ⚠ WHY THIS EXISTS.  Every number in this library came from a document and
 * nothing here had met a sensor.  One capture validates the frame
 * layout, the scales, the ≈799.2 Hz rate, the 80.166 tick ratio, the 59-tick
 * skew and the burst distribution — all of which the session, calibration and
 * history phases assume.  Finding a disagreement after building on them costs
 * three phases; finding it first costs an afternoon.
 *
 * ⚠ TWO RULES SHAPE EVERY ACCUMULATOR BELOW, AND BOTH COST US SOMETHING ONCE.
 *
 *   "No evidence" is not "agreement".  A zero mismatch count out of zero
 *   samples reads as success.  So every estimate here carries its own n, every
 *   verdict has a distinct HM_CHECK_NO_EVIDENCE value, and the printer refuses
 *   to state a figure without the count behind it.
 *
 *   Precision and accuracy are different numbers.  The rate check reports the
 *   fit's residual spread AND the fit's own degeneracy flags, because §10's
 *   silent failure is an estimator that collapses onto one point while still
 *   reporting small residuals.
 *
 * This file re-implements exactly one piece of session logic — the
 * `a1 02` … `a1 01` bracket — because live and history 0x90 frames are
 * byte-identical (§10.1) and history arrival times carry no information.  That
 * duplication is deliberate and is the same discriminator implementation-notes
 * §5 specifies for hm_session.c; getting it wrong here would feed ~4,000 bulk
 * arrivals into the clock fit and quietly wreck the rate.
 */
#include "hackmotion/record.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "hm_codec.h"
#include "hm_unwrap.h"
#include "hm_fit.h"

/*
 * ⚠ The threshold for §6.3's physical check on which block is which.
 *
 * The two units differ by roughly ω²r and sit 3-8 cm apart, so the difference
 * only separates from noise once the rotation is fast.  At 500 °/s (8.7 rad/s)
 * and r = 5 cm that is ~3.8 m/s²; at §6.4's measured swing peak of 2,100 °/s it
 * is ~67 m/s², consistent with the 31-51 m/s² the specification measured.  Well
 * above hand-waving, well below a swing, so a capture containing any deliberate
 * motion gets an answer.
 */
#define HM_FAST_GYRO_DPS 500.0

/*
 * ⚠ §7.3's table: index step 1 (the full ≈799.2 Hz) corresponds to a median
 * angular rate of 780-850 °/s, step 2 to ~370 °/s.  A retrieval only
 * demonstrates the full-rate path where BOTH halves hold — a dense step AND
 * fast motion — so this is the rate above which a step of 1 or 2 counts as
 * evidence rather than as coincidence.
 */
#define HM_FULL_RATE_GYRO_DPS 300.0

/* §9.1's vendor bring-up, verbatim.  Only step 2 is required of a client; the
 * rest is recorded because reproducing it is a useful bring-up test. */
static const uint8_t k_vendor_bringup[] = {0x80u, 0x81u, 0x84u, 0x81u,
                                           0x86u, 0x86u, 0x86u, 0x85u};

typedef struct lin_fit {
    uint64_t n;
    double   mx, my;     /* running means            */
    double   sxx, sxy, syy; /* centred sums of squares */
} lin_fit;

struct hm_reconciler {
    hm_reconcile_report rep;
    bool                finished;

    hm_stream_config cfg;

    /* Protocol state.  ⚠ One discriminator, written in two places only. */
    bool bracket_open;

    /* Per-stream live decode. */
    hm_index_unwrapper idx;
    bool               have_prev_index;
    uint32_t           prev_index;
    hm_tick_unwrapper  tick[HM_UNIT_COUNT];
    hm_fit             fit;
    uint64_t           stream_id;

    /* ⚠ The reconciler's OWN ratio fit, because the unwrapper's is a
     * wrap-resolution aid — see hm_reconcile_report.ticks_per_index. */
    lin_fit ratio_fit[HM_UNIT_COUNT];
    lin_fit rel_fit;   /* (palm − arm) against index; slope zero ⇒ one rate */

    /* Keepalive and bring-up. */
    bool       have_last_write;
    hm_time_us last_write_us;
    bool       stream_ever_started;

    /* §6.1 start latency. */
    bool       awaiting_first_frame;
    hm_time_us start_write_us;

    bool       have_first_time;

    /* ⚠ Delivery bunching, which is what a fabricated arrival time looks like. */
    hm_time_us last_frame_arrival_us;
    bool       have_frame_arrival;
    uint32_t   frames_this_arrival;

    /* ⚠ §7.3's motion-adaptive buffer: the step between consecutive HISTORY
     * records is only interpretable next to the angular rate it spans. */
    bool     have_hist_prev;
    uint16_t hist_prev_raw;
    uint32_t hist_run;     /* consecutive step-1 history records */
    uint32_t live_run;     /* ... and live */

    /* ⚠ §7.5's "no recording gap" claim, measured across each bracket from the
     * live frames either side.  Raw ticks, so the measurement does not lean on
     * the wrap resolution whose margin it is reporting. */
    bool     have_live_mark;
    uint32_t live_mark_index;
    uint16_t live_mark_ticks_raw;
    bool       awaiting_post_bracket;
    uint32_t   pre_bracket_index;
    uint16_t   pre_bracket_ticks_raw;
    hm_time_us bracket_open_us;
    hm_time_us bracket_close_us;

    /* ⚠ Kept rather than reduced to a running median: §10.3's claim is that the
     * median is IDENTICAL across the first and second halves of the session,
     * and you cannot split a session you did not keep. */
    int16_t *skew;
};

/* ------------------------------------------------------------------------ */
/* hm_stat — Welford, so a long capture does not lose the spread             */
/* ------------------------------------------------------------------------ */
void hm_stat_reset(hm_stat *stat)
{
    if (stat == NULL) {
        return;
    }
    memset(stat, 0, sizeof(*stat));
}

void hm_stat_add(hm_stat *stat, double value)
{
    double delta;

    if (stat == NULL) {
        return;
    }
    if (stat->n == 0u) {
        stat->min = value;
        stat->max = value;
    } else {
        if (value < stat->min) {
            stat->min = value;
        }
        if (value > stat->max) {
            stat->max = value;
        }
    }
    stat->n++;
    delta = value - stat->mean;
    stat->mean += delta / (double)stat->n;
    stat->m2 += delta * (value - stat->mean);
}

double hm_stat_sd(const hm_stat *stat)
{
    if (stat == NULL || stat->n < 2u) {
        return 0.0;
    }
    return sqrt(stat->m2 / (double)(stat->n - 1u));
}

/* ------------------------------------------------------------------------ */
/* A streaming least-squares line, with its standard error                   */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ Mean-centred updates, not raw Σx / Σxy.  Over a ten-minute session the
 * index reaches ~480,000 and the tick count ~38 million, and the textbook
 * (nΣxy − ΣxΣy) form differences two numbers around 10^17 in a type with 16
 * digits — the answer would be dominated by cancellation exactly when the
 * baseline is long enough to be worth having.
 *
 * `sigma` matters as much as `slope`: a ppm figure with no standard error
 * cannot test a claim stated in ppm, and reporting one without the other is
 * how a 30 s capture comes to look like it confirmed a 238 s measurement.
 */
static void lin_add(lin_fit *f, double x, double y)
{
    double dx, dy;
    f->n++;
    dx = x - f->mx;
    dy = y - f->my;
    f->mx += dx / (double)f->n;
    f->my += dy / (double)f->n;
    f->sxx += dx * (x - f->mx);
    f->sxy += dx * (y - f->my);
    f->syy += dy * (y - f->my);
}

/* Returns false when the line is not determined — fewer than three points, or
 * every point at the same x.  ⚠ Not "slope 0"; not determined. */
static bool lin_slope(const lin_fit *f, double *out_slope, double *out_sigma)
{
    double slope, rss, s2;

    if (f->n < 3u || f->sxx <= 0.0) {
        return false;
    }
    slope = f->sxy / f->sxx;
    rss = f->syy - slope * f->sxy;
    if (rss < 0.0) {
        rss = 0.0; /* rounding only */
    }
    s2 = rss / (double)(f->n - 2u);
    *out_slope = slope;
    *out_sigma = sqrt(s2 / f->sxx);
    return true;
}

const char *hm_check_verdict_name(hm_check_verdict verdict)
{
    switch (verdict) {
        case HM_CHECK_NO_EVIDENCE: return "no evidence";
        case HM_CHECK_MATCH:       return "match";
        case HM_CHECK_DIFFERS:     return "DIFFERS";
        case HM_CHECK_NOTE:        return "note";
        default:                   return "?";
    }
}

const char *hm_step_bucket_name(hm_step_bucket bucket)
{
    switch (bucket) {
        case HM_STEP_NONPOSITIVE:   return "<=0";
        case HM_STEP_DENSE:         return "1-7 (dense)";
        case HM_STEP_8:             return "8 (~100 Hz)";
        case HM_STEP_9_31:          return "9-31";
        case HM_STEP_32:            return "32 (~25 Hz)";
        case HM_STEP_ABOVE_32:      return ">32 (gap)";
        case HM_STEP_BUCKET_COUNT:  return "?";
        default:                    return "?";
    }
}

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------ */
static void begin_stream(hm_reconciler *r)
{
    hm_index_unwrapper_reset(&r->idx);
    hm_tick_unwrapper_reset(&r->tick[HM_UNIT_LOWER_ARM]);
    hm_tick_unwrapper_reset(&r->tick[HM_UNIT_PALM]);
    r->have_prev_index = false;
    r->prev_index = 0u;
    /* ⚠ The index space restarts at every stream start (§6.5), so a fit
     * carried across one would draw a line through two unrelated origins. */
    memset(r->ratio_fit, 0, sizeof(r->ratio_fit));
    memset(&r->rel_fit, 0, sizeof(r->rel_fit));
    r->stream_id++;
    /* ⚠ The counter resets at every start but the crystal does not, so the fit
     * keeps its pooled rate across streams and drops everything else (hm_fit.h). */
    hm_fit_begin_stream(&r->fit, r->stream_id);
}

hm_status hm_reconcile_begin(const hm_recording_info *info, hm_reconciler **out_reconciler)
{
    hm_reconciler *r;

    if (out_reconciler == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    *out_reconciler = NULL;

    r = (hm_reconciler *)calloc(1, sizeof(*r));
    if (r == NULL) {
        return HM_ERR_NO_MEMORY;
    }
    r->skew = (int16_t *)calloc(HM_RECONCILE_SKEW_MAX, sizeof(int16_t));
    if (r->skew == NULL) {
        free(r);
        return HM_ERR_NO_MEMORY;
    }

    r->rep.info = (info != NULL) ? *info : hm_recording_info_default();
    r->rep.first_us = HM_TIME_UNKNOWN;
    r->rep.last_us = HM_TIME_UNKNOWN;
    r->rep.stream_start_latency_us = HM_TIME_UNKNOWN;
    r->rep.fast_gyro_threshold_dps = HM_FAST_GYRO_DPS;

    /*
     * The header's `config=` is a claim about the recording; the recorded
     * `a0 01 <cfg>` write is the byte that produced the frames.  Start from the
     * claim and replace it the moment the wire says otherwise.
     */
    if (r->rep.info.config_legacy) {
        r->cfg = hm_stream_config_legacy();
    } else if (r->rep.info.config_bits == HM_CONFIG_OBSERVED_DEFAULT) {
        r->cfg = hm_stream_config_default();
    } else {
        r->cfg = hm_stream_config_nonstandard(r->rep.info.config_bits,
                                              "from the recording's file header");
    }
    r->rep.config_from_stream = false;

    hm_fit_init(&r->fit, HM_ACCURACY_DRIFT_US_PER_S_DEFAULT);
    begin_stream(r);
    r->stream_id = 0u; /* begin_stream() incremented it; no stream has started */

    *out_reconciler = r;
    return HM_OK;
}

void hm_reconcile_free(hm_reconciler *reconciler)
{
    if (reconciler == NULL) {
        return;
    }
    free(reconciler->skew);
    free(reconciler);
}

/* ------------------------------------------------------------------------ */
/* Host → device                                                             */
/* ------------------------------------------------------------------------ */
static void observe_host_write(hm_reconciler *r, const hm_wire_chunk *c)
{
    uint8_t b0;

    r->rep.host_writes++;

    /* ⚠ §9.2: what resets the device's 5-minute idle shutdown is a host→device
     * WRITE, not motion and not an active stream.  So the gap that matters is
     * between writes of any kind, not between `0x81` polls. */
    if (r->have_last_write && c->host_time_us >= r->last_write_us) {
        hm_time_us gap = c->host_time_us - r->last_write_us;
        if (gap > r->rep.max_host_write_gap_us) {
            r->rep.max_host_write_gap_us = gap;
        }
    }
    r->have_last_write = true;
    r->last_write_us = c->host_time_us;

    if (c->length == 0u) {
        return;
    }
    b0 = c->data[0];

    if (b0 == 0x81u) {
        r->rep.status_polls++;
    }

    if (b0 == 0xa0u && c->length >= 3u) {
        uint8_t bits = c->data[2];
        if (bits == HM_CONFIG_OBSERVED_DEFAULT) {
            r->cfg = hm_stream_config_default();
        } else {
            /* ⚠ Two of these bits change the WIRE FORMAT, not just the content
             * (§6.2), so the decoder below must be told rather than assume. */
            r->cfg = hm_stream_config_nonstandard(bits, "observed on the wire in this capture");
        }
        r->rep.config_from_stream = true;
        r->rep.stream_starts++;
        r->stream_ever_started = true;
        r->awaiting_first_frame = true;
        r->start_write_us = c->host_time_us;
        begin_stream(r);
    } else if (b0 == 0x82u) {
        r->cfg = hm_stream_config_legacy();
        r->rep.config_from_stream = true;
        r->rep.stream_starts++;
        r->stream_ever_started = true;
        r->awaiting_first_frame = true;
        r->start_write_us = c->host_time_us;
        begin_stream(r);
    }

    /*
     * §9.1's sequence is whatever the host sent BEFORE it started a stream —
     * the start command itself is not part of it, which is why this runs after
     * the branches above rather than before them.
     */
    if (!r->stream_ever_started && r->rep.bringup_len < sizeof(r->rep.bringup)) {
        r->rep.bringup[r->rep.bringup_len++] = b0;
    }
}

/* ------------------------------------------------------------------------ */
/* One decoded record                                                        */
/* ------------------------------------------------------------------------ */
static double quat_norm_raw(const int16_t q[4])
{
    double acc = 0.0;
    for (int i = 0; i < 4; ++i) {
        acc += (double)q[i] * (double)q[i];
    }
    return sqrt(acc);
}

static double vec3_mag(const float v[3])
{
    return sqrt((double)v[0] * (double)v[0] + (double)v[1] * (double)v[1] +
                (double)v[2] * (double)v[2]);
}

static void bucket_step(hm_reconciler *r, int64_t step)
{
    hm_step_bucket b;

    if (step <= 0) {
        b = HM_STEP_NONPOSITIVE;
    } else if (step <= 7) {
        b = HM_STEP_DENSE;
    } else if (step == 8) {
        b = HM_STEP_8;
    } else if (step <= 31) {
        b = HM_STEP_9_31;
    } else if (step == 32) {
        b = HM_STEP_32;
    } else {
        b = HM_STEP_ABOVE_32;
    }
    r->rep.step[b]++;
    r->rep.steps_total++;
    /* A run of adjacent samples is the evidence; a count of "dense" is not. */
    r->live_run = (step == 1) ? r->live_run + 1u : 0u;
    if (r->live_run + 1u > r->rep.live_max_adjacent_run) {
        r->rep.live_max_adjacent_run = r->live_run + 1u;
    }
}

static void observe_record(hm_reconciler *r, const hm_sample *s, hm_time_us recv_us)
{
    const hm_unit_sample *unit[HM_UNIT_COUNT];
    double gyro_mag[HM_UNIT_COUNT];
    double accel_mag[HM_UNIT_COUNT];
    uint32_t unwrapped = 0u;
    bool suspect = false;
    int32_t skew = 0;
    bool have_skew = false;

    unit[HM_UNIT_LOWER_ARM] = &s->lower_arm;
    unit[HM_UNIT_PALM] = &s->palm;

    for (int u = 0; u < HM_UNIT_COUNT; ++u) {
        double norm = quat_norm_raw(unit[u]->q_world_to_body_raw);
        hm_stat_add(&r->rep.quat_norm[u], norm);
        if (fabs(norm - (double)HM_QUAT_NORM_NOMINAL) > (double)HM_QUAT_NORM_TOLERANCE) {
            r->rep.quat_norm_suspect++;
        }
        accel_mag[u] = vec3_mag(unit[u]->linear_accel_mps2);
        gyro_mag[u] = vec3_mag(unit[u]->gyro_dps);
        hm_stat_add(&r->rep.accel_mag_mps2[u], accel_mag[u]);
        hm_stat_add(&r->rep.gyro_mag_dps[u], gyro_mag[u]);
        if (gyro_mag[u] > r->rep.gyro_peak_dps[u]) {
            r->rep.gyro_peak_dps[u] = gyro_mag[u];
        }
    }

    hm_pinned_counts_add(&r->rep.pinned, s);

    /*
     * ⚠ §6.3's physical check on which block is which.  Under rotation the palm
     * sits at the larger radius and reads a consistently HIGHER magnitude —
     * 31-51 m/s² more across five golf swings.  A negative mean here means the
     * two blocks are swapped, which produces a plausible but MIRRORED wrist
     * angle that every ordinary plausibility check passes.
     */
    if (gyro_mag[HM_UNIT_LOWER_ARM] > HM_FAST_GYRO_DPS ||
        gyro_mag[HM_UNIT_PALM] > HM_FAST_GYRO_DPS) {
        hm_stat_add(&r->rep.accel_palm_minus_arm_fast,
                    accel_mag[HM_UNIT_PALM] - accel_mag[HM_UNIT_LOWER_ARM]);
    }

    /*
     * ⚠ §10.3's skew is resolved from the RAW counters with no unwrapper at
     * all, so it works for the very first record of a stream and for history
     * records alike.  Both halves of the evidence are wanted here.
     */
    if (s->lower_arm.has_ticks && s->palm.has_ticks) {
        skew = hm_tick_skew(s->palm.ticks_raw, s->lower_arm.ticks_raw);
        have_skew = true;
        hm_stat_add(&r->rep.skew_ticks, (double)skew);
        r->rep.skew_total++;
        if (r->rep.skew_stored < HM_RECONCILE_SKEW_MAX) {
            r->skew[r->rep.skew_stored++] = (int16_t)skew;
        }
    }

    if (r->bracket_open) {
        /*
         * ⚠ History.  Arrival carries no information (§10, §10.1), the index
         * runs BEHIND live and would read as a near-full-modulus forward step,
         * and feeding either into the fit or the live unwrapper is the mistake
         * implementation-notes §5 lists four ways to make.  Counted, and
         * nothing else.
         */
        r->rep.records_history++;
        {
            double omega = (gyro_mag[HM_UNIT_LOWER_ARM] > gyro_mag[HM_UNIT_PALM])
                               ? gyro_mag[HM_UNIT_LOWER_ARM]
                               : gyro_mag[HM_UNIT_PALM];
            hm_stat_add(&r->rep.history_gyro_dps, omega);
            if (omega > r->rep.history_peak_gyro_dps) {
                r->rep.history_peak_gyro_dps = omega;
            }
            if (r->have_hist_prev) {
                /* Records arrive in ascending index order within a bracket, and
                 * history never touches the live unwrapper (§10.1). */
                uint16_t step = (uint16_t)(s->sample_index_raw - r->hist_prev_raw);
                size_t slot = (step <= 8u) ? (size_t)step : 9u;
                r->rep.history_step_count[slot]++;
                /* ⚠ §7.3: the full rate exists only where the wrist moved fast,
                 * so BOTH halves are required before the path counts as tested. */
                if (step <= 2u && omega > HM_FULL_RATE_GYRO_DPS) {
                    r->rep.history_exercised_full_rate = true;
                }
                r->hist_run = (step == 1u) ? r->hist_run + 1u : 0u;
                if (r->hist_run + 1u > r->rep.history_max_adjacent_run) {
                    r->rep.history_max_adjacent_run = r->hist_run + 1u;
                }
            }
            r->hist_prev_raw = s->sample_index_raw;
            r->have_hist_prev = true;
        }
        return;
    }

    r->rep.records_live++;

    /* ⚠ A run of frames arriving <2 ms apart is a backlog being dispatched,
     * not a device behaviour.  §10 warns that BLE bunching corrupts any rate
     * measured from arrival times; this is the same warning applied to any
     * ANALYSIS keyed on them. */
    if (r->have_frame_arrival && (recv_us - r->last_frame_arrival_us) < 2000) {
        r->frames_this_arrival++;
    } else {
        r->frames_this_arrival = 1u;
    }
    r->last_frame_arrival_us = recv_us;
    r->have_frame_arrival = true;
    if (r->frames_this_arrival > r->rep.max_frame_burst) {
        r->rep.max_frame_burst = r->frames_this_arrival;
    }

    if ((s->flags & HM_SAMPLE_INDEX_MISSING) != 0u) {
        /* Legacy 0x7f: no record header, so there is no index to unwrap, no
         * step to bucket and nothing the clock fit can anchor on (§6.3.1). */
        return;
    }

    unwrapped = hm_index_unwrap(&r->idx, s->sample_index_raw, &suspect);
    if (suspect) {
        r->rep.index_regressions++;
    }
    if (r->have_prev_index) {
        int64_t step = (int64_t)unwrapped - (int64_t)r->prev_index;
        bucket_step(r, step);
        if (step == 1) {
            double omega = (gyro_mag[HM_UNIT_LOWER_ARM] > gyro_mag[HM_UNIT_PALM])
                               ? gyro_mag[HM_UNIT_LOWER_ARM]
                               : gyro_mag[HM_UNIT_PALM];
            if (omega > r->rep.live_adjacent_peak_gyro_dps) {
                r->rep.live_adjacent_peak_gyro_dps = omega;
            }
        }
    }
    r->have_prev_index = true;
    r->prev_index = unwrapped;

    for (int u = 0; u < HM_UNIT_COUNT; ++u) {
        if (unit[u]->has_ticks) {
            uint32_t margin = 0u;
            int64_t ticks = hm_tick_unwrap(&r->tick[u], unwrapped, unit[u]->ticks_raw, &margin);
            /* ⚠ 0 is the worst possible margin, so "never measured" cannot be
             * spelled as 0.  A separate flag, not a sentinel. */
            if (!r->rep.wrap_margin_measured[u] || margin < r->rep.worst_wrap_margin[u]) {
                r->rep.worst_wrap_margin[u] = margin;
                r->rep.wrap_margin_measured[u] = true;
            }
            /* The measurement, as opposed to the unwrapper's working estimate. */
            lin_add(&r->ratio_fit[u], (double)unwrapped, (double)ticks);
        }
    }
    if (have_skew) {
        /* ⚠ The one-rate test, on the DIFFERENCE.  Zero slope ⇔ one rate, and
         * every common-mode term is gone before the fit rather than after it. */
        lin_add(&r->rel_fit, (double)unwrapped, (double)skew);
    }

    if (r->awaiting_post_bracket && r->have_live_mark) {
        /*
         * §7.5 says issuing `a1` in place removes the recording window a
         * stop-and-restart would cost.  This is that claim, measured: how far
         * the SAMPLE COUNTER moved against how far the MCU TIMER moved, over
         * the same interval.
         *
         * ⚠ Judged as a FRACTION OF THE PULL, not as an absolute shortfall.
         * The gap equals the pull at any size — 90-99% of it across a 16× range
         * — so a fixed threshold calls a 25 ms pull healthy and undercounts.
         */
        uint32_t d_index = unwrapped - r->pre_bracket_index;
        uint32_t d_ticks = (uint32_t)(uint16_t)(s->lower_arm.ticks_raw - r->pre_bracket_ticks_raw);
        double expected = (double)d_ticks / HM_NOMINAL_TICKS_PER_SAMPLE;
        double lost_samples = expected - (double)d_index;
        double lost_ms = lost_samples / HM_NOMINAL_SAMPLE_RATE_HZ * 1000.0;
        double pull_ms = (double)(r->bracket_close_us - r->bracket_open_us) / 1000.0;
        double fraction = (pull_ms > 0.0) ? lost_ms / pull_ms : 0.0;

        r->rep.retrievals_measured++;
        if (lost_samples > 0.0 && fraction >= 0.5) {
            r->rep.retrievals_stalled++;
            hm_stat_add(&r->rep.retrieval_stall_ms, lost_ms);
            hm_stat_add(&r->rep.retrieval_indices_lost, lost_samples);
            hm_stat_add(&r->rep.retrieval_stall_fraction, fraction);
            r->rep.stall_total_ms += lost_ms;
            r->rep.stall_total_ticks += lost_samples * HM_NOMINAL_TICKS_PER_SAMPLE;
            if (lost_samples * HM_NOMINAL_TICKS_PER_SAMPLE > r->rep.stall_worst_ticks) {
                r->rep.stall_worst_ticks = lost_samples * HM_NOMINAL_TICKS_PER_SAMPLE;
            }
        }
        r->awaiting_post_bracket = false;
    }
    r->have_live_mark = true;
    r->live_mark_index = unwrapped;
    r->live_mark_ticks_raw = s->lower_arm.ticks_raw;

    /* ⚠ Live frames only.  This is the one call site, exactly as the session's
     * own rule requires (implementation-notes §5). */
    hm_fit_observe(&r->fit, unwrapped, recv_us);

    if (r->awaiting_first_frame) {
        r->rep.stream_start_latency_us = recv_us - r->start_write_us;
        r->awaiting_first_frame = false;
    }
}

/* ------------------------------------------------------------------------ */
/* Device → host                                                             */
/* ------------------------------------------------------------------------ */
static void observe_notification(hm_reconciler *r, const hm_wire_chunk *c)
{
    hm_decoded dec;
    hm_status st;

    r->rep.device_notifications++;
    if (c->length == 0u) {
        r->rep.decode_errors++;
        return;
    }
    r->rep.message_count[c->data[0]]++;

    st = hm_codec_decode(c->data, c->length, r->cfg, &dec);
    r->rep.codec_warnings |= dec.warnings;

    if (st == HM_ERR_UNKNOWN_MESSAGE) {
        /* §5.1: log and ignore.  Not an error condition — and the point of
         * counting them is that the id range is sparse and this table is scoped
         * to what a client uses, not to every byte the device could emit. */
        r->rep.unknown_messages++;
        return;
    }
    if (st < HM_OK) {
        r->rep.decode_errors++;
        return;
    }

    switch (dec.kind) {
        case HM_MSGK_FRAME:
        case HM_MSGK_LEGACY_FRAME: {
            size_t one = 1u + hm_stream_config_record_size(r->cfg);
            size_t two = 1u + 2u * hm_stream_config_record_size(r->cfg);
            if ((size_t)c->length == one) {
                r->rep.notif_one_record++;
            } else if ((size_t)c->length == two) {
                r->rep.notif_two_records++;
            } else {
                r->rep.notif_other_len++;
            }
            for (size_t i = 0; i < dec.u.frame.count; ++i) {
                observe_record(r, &dec.u.frame.sample[i], c->host_time_us);
            }
            break;
        }
        case HM_MSGK_HISTORY_MARK:
            if (!dec.u.history.valid) {
                break;
            }
            if (dec.u.history.marker == HM_HISTORY_MARK_START) {
                r->rep.brackets_opened++;
                r->bracket_open = true;
                r->have_hist_prev = false; /* steps do not span two retrievals */
                r->hist_run = 0u;
                if (r->have_live_mark) {
                    r->pre_bracket_index = r->live_mark_index;
                    r->pre_bracket_ticks_raw = r->live_mark_ticks_raw;
                    r->awaiting_post_bracket = true;
                    r->bracket_open_us = c->host_time_us;
                }
            } else if (r->bracket_open) {
                r->rep.brackets_closed++;
                r->bracket_open = false;
                r->bracket_close_us = c->host_time_us;
            }
            /*
             * ⚠ An `a1 01` with no bracket open is §7.2's LEADING marker, which
             * closes a PREVIOUS retrieval.  Counting it as our close is one of
             * the four bracket confusions in implementation-notes §5, and it
             * would let the next retrieval's records through as live.
             */
            break;
        case HM_MSGK_DEVICE_ERROR:
            r->rep.device_errors++;
            break;
        case HM_MSGK_BUTTON:
            r->rep.buttons++;
            break;
        case HM_MSGK_UNKNOWN:
        case HM_MSGK_VERSIONS:
        case HM_MSGK_STATUS:
        case HM_MSGK_STREAM_STARTED:
        case HM_MSGK_STREAM_STOPPED:
        case HM_MSGK_SENSOR_MAP:
        case HM_MSGK_MAC:
        case HM_MSGK_SERIAL:
        case HM_MSGK_CALIBRATION_RESULT:
        case HM_MSGK_START_ACK:
        case HM_MSGK_CAL_ACK:
        case HM_MSGK_IGNORED:
            break;
    }
}

/* ------------------------------------------------------------------------ */
/* The stream of chunks                                                      */
/* ------------------------------------------------------------------------ */
void hm_reconcile_observe(hm_reconciler *r, const hm_wire_chunk *c)
{
    if (r == NULL || c == NULL || r->finished) {
        return;
    }

    r->rep.chunks++;
    if ((c->flags & HM_WIRE_LOST) != 0u) {
        /* ⚠ Chunks were dropped before this one.  Everything downstream — the
         * step histogram, the keepalive gap, the record census — is measured
         * over a recording with holes in it, and a report that did not say so
         * would read as complete. */
        r->rep.chunks_after_loss++;
    }
    if ((c->flags & HM_WIRE_REDACTED) != 0u) {
        r->rep.redacted_chunks++;
    }

    if (!r->have_first_time) {
        r->rep.first_us = c->host_time_us;
        r->have_first_time = true;
    }
    r->rep.last_us = c->host_time_us;

    switch ((hm_wire_direction)c->direction) {
        case HM_WIRE_HOST_TO_DEVICE:
            observe_host_write(r, c);
            break;
        case HM_WIRE_DEVICE_TO_HOST:
            observe_notification(r, c);
            break;
        case HM_WIRE_META:
            r->rep.meta_chunks++;
            break;
        default:
            r->rep.meta_chunks++;
            break;
    }
}

/* ------------------------------------------------------------------------ */
/* Verdicts                                                                  */
/* ------------------------------------------------------------------------ */
static int cmp_i16(const void *a, const void *b)
{
    int16_t x = *(const int16_t *)a;
    int16_t y = *(const int16_t *)b;
    return (x < y) ? -1 : ((x > y) ? 1 : 0);
}

/* Median of a copy; `values` is not disturbed.
 *
 * ⚠ NAN is a FLOAT constant and every target here is a double, so each use is
 * cast explicitly.  clang's -Wdouble-promotion rejects the implicit widening
 * (gcc does not, which is why this only surfaced in CI). */
static double median_of(const int16_t *values, size_t n, int16_t *scratch)
{
    if (n == 0u) {
        return (double)NAN;
    }
    memcpy(scratch, values, n * sizeof(int16_t));
    qsort(scratch, n, sizeof(int16_t), cmp_i16);
    if ((n % 2u) == 1u) {
        return (double)scratch[n / 2u];
    }
    return 0.5 * ((double)scratch[n / 2u - 1u] + (double)scratch[n / 2u]);
}

static void finish_skew(hm_reconciler *r)
{
    size_t n = (size_t)r->rep.skew_stored;
    int16_t *scratch;

    r->rep.skew_median_ticks = (double)NAN;
    r->rep.skew_median_first_half = (double)NAN;
    r->rep.skew_median_second_half = (double)NAN;
    r->rep.skew_median_us = (double)NAN;
    if (n == 0u) {
        return;
    }
    scratch = (int16_t *)malloc(n * sizeof(int16_t));
    if (scratch == NULL) {
        return;
    }
    r->rep.skew_median_ticks = median_of(r->skew, n, scratch);
    if (n >= 2u) {
        size_t half = n / 2u;
        r->rep.skew_median_first_half = median_of(r->skew, half, scratch);
        r->rep.skew_median_second_half = median_of(r->skew + half, n - half, scratch);
    }
    free(scratch);

    if (r->rep.tick_rate_hz > 0.0) {
        r->rep.skew_median_us = r->rep.skew_median_ticks * 1e6 / r->rep.tick_rate_hz;
    }
}

static void decide_verdicts(hm_reconcile_report *rep)
{
    /* --- §6.3 framing ---------------------------------------------------- */
    if (rep->notif_one_record + rep->notif_two_records + rep->notif_other_len == 0u) {
        rep->verdict_frame_length = HM_CHECK_NO_EVIDENCE;
    } else if (rep->notif_other_len == 0u) {
        rep->verdict_frame_length = HM_CHECK_MATCH;
    } else {
        rep->verdict_frame_length = HM_CHECK_DIFFERS;
    }

    /* --- §6.4 the quaternion norm ----------------------------------------- */
    if (rep->quat_norm[HM_UNIT_LOWER_ARM].n == 0u) {
        rep->verdict_quat_norm = HM_CHECK_NO_EVIDENCE;
    } else {
        bool ok = (rep->quat_norm_suspect == 0u);
        for (int u = 0; u < HM_UNIT_COUNT; ++u) {
            /* §6.4 measured 16384.7 ± 0.41 under `7e` and 16383.2 ± 0.31 under
             * the legacy path, so the band has to admit both. */
            if (fabs(rep->quat_norm[u].mean - (double)HM_QUAT_NORM_NOMINAL) > 4.0) {
                ok = false;
            }
        }
        rep->verdict_quat_norm = ok ? HM_CHECK_MATCH : HM_CHECK_DIFFERS;
    }

    /* --- §6.5 the rate ---------------------------------------------------- */
    if (rep->fit.observations <= 0 || (rep->fit.flags & HM_CLOCK_HAS_FIT) == 0u) {
        rep->verdict_rate = HM_CHECK_NO_EVIDENCE;
    } else if ((rep->fit.flags & HM_CLOCK_SHORT_BASELINE) != 0u) {
        /* ⚠ Below HM_FIT_MIN_SPAN_US the rate cannot be separated from the
         * offset at all.  That is not a small answer, it is no answer. */
        rep->verdict_rate = HM_CHECK_NO_EVIDENCE;
    } else if ((rep->fit.flags & HM_CLOCK_DEGENERATE) != 0u) {
        rep->verdict_rate = HM_CHECK_DIFFERS;
    } else if (rep->fitted_rate_hz >= HM_RATE_PLAUSIBLE_MIN_HZ &&
               rep->fitted_rate_hz <= HM_RATE_PLAUSIBLE_MAX_HZ) {
        rep->verdict_rate = HM_CHECK_MATCH;
    } else {
        rep->verdict_rate = HM_CHECK_DIFFERS;
    }

    /* --- §6.5 ticks per sample -------------------------------------------- */
    if (!rep->tick_ratio_fitted[HM_UNIT_LOWER_ARM] || !rep->tick_ratio_fitted[HM_UNIT_PALM] ||
        !rep->rel_rate_measured) {
        rep->verdict_tick_ratio = HM_CHECK_NO_EVIDENCE;
    } else if (rep->rel_rate_ppm_sigma > HM_ONE_RATE_CLAIM_PPM) {
        /*
         * ⚠ THE CAPTURE CANNOT TEST THE CLAIM.  §6.5's 2 ppm came from a 238 s
         * session; the standard error on this figure scales with the baseline,
         * and at 30 s it is ±1.26 ppm.  A shorter capture cannot separate 2 ppm
         * from 20, and printing "0.4 ppm — match" from a measurement that could
         * not have detected a violation is precisely the failure this report
         * exists to refuse.  Not evidence.
         */
        rep->verdict_tick_ratio = HM_CHECK_NO_EVIDENCE;
    } else {
        bool ok = true;
        for (int u = 0; u < HM_UNIT_COUNT; ++u) {
            /* §6.4's four sessions span 64,025-64,088 ticks/s, ~0.1 %; 0.5 % is
             * five times that and still nowhere near a mis-decode.  §6.5's own
             * two sessions gave 80.166 and 80.136, so the band must admit both. */
            if (fabs(rep->ticks_per_index[u] - HM_NOMINAL_TICKS_PER_SAMPLE) >
                0.005 * HM_NOMINAL_TICKS_PER_SAMPLE) {
                ok = false;
            }
        }
        /*
         * §6.5: "Both blocks of a record agree on it to within 2 ppm."
         * ⚠ Judged against the measurement's own uncertainty: a difference is a
         * disagreement only once it is 2σ clear of the claim.  Otherwise a
         * capture reports a violation whenever its own noise exceeds 2 ppm,
         * which is what a 17.21 ppm reading from a wrap-resolution estimator
         * did on the first real capture.
         */
        if (fabs(rep->rel_rate_ppm) - 2.0 * rep->rel_rate_ppm_sigma > HM_ONE_RATE_CLAIM_PPM) {
            ok = false;
        }
        rep->verdict_tick_ratio = ok ? HM_CHECK_MATCH : HM_CHECK_DIFFERS;
    }

    /* --- §10.3 the skew ---------------------------------------------------- */
    if (rep->skew_stored < 2u) {
        rep->verdict_skew = HM_CHECK_NO_EVIDENCE;
    } else {
        /*
         * ⚠ The claim under test is STABILITY, not the value 59.  §10.3 says
         * the difference is "a small stable number" whose physical meaning is
         * unresolved and which cannot be settled by a shared impulse — so a
         * different unit reading a different constant is expected, and a
         * constant that MOVES within one session is the finding.
         */
        bool stable = fabs(rep->skew_median_first_half - rep->skew_median_second_half) <= 2.0;
        bool small = fabs(rep->skew_median_ticks) <= 500.0;
        rep->verdict_skew = (stable && small) ? HM_CHECK_MATCH : HM_CHECK_DIFFERS;
    }

    /* --- §6.6 the live rate is a continuum -------------------------------- */
    if (rep->steps_total == 0u) {
        rep->verdict_bursts = HM_CHECK_NO_EVIDENCE;
    } else if (rep->step[HM_STEP_DENSE] > 0u) {
        rep->verdict_bursts = HM_CHECK_MATCH;
    } else if (rep->step[HM_STEP_8] == 0u && rep->step[HM_STEP_9_31] == 0u) {
        /* Pure +32 — §6.6's own stationary session, the one row of its table
         * with no dense steps at all.  Nothing was moving, so nothing is
         * confirmed and nothing is contradicted. */
        rep->verdict_bursts = HM_CHECK_NOTE;
    } else {
        /* Motion happened and produced no dense step.  §6.6 says dense steps
         * appear in EVERY session containing motion. */
        rep->verdict_bursts = HM_CHECK_DIFFERS;
    }

    /* --- §6.3 which block is the palm -------------------------------------- */
    if (rep->accel_palm_minus_arm_fast.n == 0u) {
        rep->verdict_palm_is_second_block = HM_CHECK_NO_EVIDENCE;
    } else if (rep->accel_palm_minus_arm_fast.mean > 0.0) {
        rep->verdict_palm_is_second_block = HM_CHECK_MATCH;
    } else {
        rep->verdict_palm_is_second_block = HM_CHECK_DIFFERS;
    }

    /* --- §9.1 bring-up ------------------------------------------------------ */
    if (rep->bringup_len == 0u) {
        rep->verdict_bringup = HM_CHECK_NO_EVIDENCE;
    } else if (rep->bringup_matches_vendor) {
        rep->verdict_bringup = HM_CHECK_MATCH;
    } else {
        bool asked_versions = false;
        for (size_t i = 0; i < rep->bringup_len; ++i) {
            if (rep->bringup[i] == 0x80u) {
                asked_versions = true;
            }
        }
        /* §9.1: "Only step 2 is required."  A different sequence is a note; no
         * `80` at all means the protocol version that gates features (§6.2,
         * §7.1) was never read. */
        rep->verdict_bringup = asked_versions ? HM_CHECK_NOTE : HM_CHECK_DIFFERS;
    }

    /* --- §9.2 the keepalive -------------------------------------------------- */
    if (rep->host_writes < 2u) {
        rep->verdict_keepalive = HM_CHECK_NO_EVIDENCE;
    } else if (rep->max_host_write_gap_us <= (hm_time_us)33 * 1000 * 1000) {
        rep->verdict_keepalive = HM_CHECK_MATCH;
    } else {
        rep->verdict_keepalive = HM_CHECK_DIFFERS;
    }

    /* --- §7.5 does a mid-stream retrieval cost a recording gap? ---------------- */
    if (rep->retrievals_measured == 0u) {
        rep->verdict_retrieval_continuity = HM_CHECK_NO_EVIDENCE;
    } else if (rep->retrievals_stalled == 0u) {
        rep->verdict_retrieval_continuity = HM_CHECK_MATCH;
    } else {
        rep->verdict_retrieval_continuity = HM_CHECK_DIFFERS;
    }

    /* --- §7.3 the motion-adaptive buffer -------------------------------------- */
    if (rep->records_history < 2u) {
        rep->verdict_history_rate = HM_CHECK_NO_EVIDENCE;
    } else if (rep->history_step_count[0] > 0u || rep->history_step_count[9] > 0u) {
        /*
         * §7.3's two falsifiable claims, and the only ones a single retrieval
         * can contradict: across 17,739 measured steps NO step exceeded 8 and
         * NONE was 0.  Step 8 is a hard floor, step 1 a hard ceiling.
         */
        rep->verdict_history_rate = HM_CHECK_DIFFERS;
    } else if (!rep->history_exercised_full_rate) {
        /*
         * ⚠ THE CAPTURE DID NOT TEST THE THING THAT MATTERS.  A retrieval over
         * a still wrist comes back evenly at one-in-eight — correct behaviour,
         * and indistinguishable from a broken full-rate path.  Reporting that
         * as a match would certify a path nobody exercised; reporting it as a
         * failure would blame the device for the bench.  Neither: no evidence.
         */
        rep->verdict_history_rate = HM_CHECK_NO_EVIDENCE;
    } else {
        rep->verdict_history_rate = HM_CHECK_MATCH;
    }
}

void hm_reconcile_finish(hm_reconciler *r, hm_reconcile_report *out_report)
{
    if (r == NULL) {
        return;
    }
    if (!r->finished) {
        r->finished = true;

        r->rep.config = r->cfg;
        r->rep.expected_len_one_record = 1u + hm_stream_config_record_size(r->cfg);
        r->rep.expected_len_two_records = 1u + 2u * hm_stream_config_record_size(r->cfg);

        if (r->have_first_time && r->rep.last_us >= r->rep.first_us) {
            r->rep.duration_us = r->rep.last_us - r->rep.first_us;
        }

        hm_fit_snapshot(&r->fit, r->rep.last_us, &r->rep.fit);
        if (r->rep.fit.observations > 0 && r->rep.fit.fitted_rate_hz > 0.0) {
            r->rep.fitted_rate_hz = r->rep.fit.fitted_rate_hz;
            r->rep.ppm_vs_nominal = (r->rep.fitted_rate_hz - HM_NOMINAL_SAMPLE_RATE_HZ) /
                                    HM_NOMINAL_SAMPLE_RATE_HZ * 1e6;
            /* ⚠ The number the specification tells a client not to use: 800 Hz
             * costs ≈1,000 ppm, one-directional, 45 ms after 45 s (§6.5). */
            r->rep.ppm_vs_800 = (r->rep.fitted_rate_hz - 800.0) / 800.0 * 1e6;
        }

        for (int u = 0; u < HM_UNIT_COUNT; ++u) {
            double slope = 0.0, sigma = 0.0;
            r->rep.tick_ratio_fitted[u] = lin_slope(&r->ratio_fit[u], &slope, &sigma);
            r->rep.ticks_per_index[u] = r->rep.tick_ratio_fitted[u] ? slope : 0.0;
        }
        r->rep.tick_fit_n = r->ratio_fit[HM_UNIT_LOWER_ARM].n;
        if (r->rep.tick_ratio_fitted[HM_UNIT_LOWER_ARM] &&
            r->rep.tick_ratio_fitted[HM_UNIT_PALM]) {
            double mean = 0.5 * (r->rep.ticks_per_index[HM_UNIT_LOWER_ARM] +
                                 r->rep.ticks_per_index[HM_UNIT_PALM]);
            double slope = 0.0, sigma = 0.0;

            /* §6.4: the tick rate is not measured directly, it FOLLOWS from the
             * sample rate and the ticks-per-sample ratio.  Reported the same
             * way it was derived. */
            r->rep.tick_rate_hz =
                mean * ((r->rep.fitted_rate_hz > 0.0) ? r->rep.fitted_rate_hz
                                                      : HM_NOMINAL_SAMPLE_RATE_HZ);

            if (mean > 0.0 && lin_slope(&r->rel_fit, &slope, &sigma)) {
                r->rep.rel_rate_ppm = slope / mean * 1e6;
                r->rep.rel_rate_ppm_sigma = sigma / mean * 1e6;
                r->rep.rel_rate_measured = true;
            }
        }

        {
            uint64_t best = 0u;
            for (size_t k = 1; k < 10u; ++k) {
                if (r->rep.history_step_count[k] > best) {
                    best = r->rep.history_step_count[k];
                    r->rep.history_modal_step = (uint32_t)k;
                }
            }
        }

        finish_skew(r);

        if (r->rep.steps_total > 0u) {
            r->rep.dense_fraction =
                (double)r->rep.step[HM_STEP_DENSE] / (double)r->rep.steps_total;
        }

        {
            double full_scale = hm_stream_config_gyro_full_scale_dps(r->cfg);
            double peak = r->rep.gyro_peak_dps[HM_UNIT_LOWER_ARM];
            if (r->rep.gyro_peak_dps[HM_UNIT_PALM] > peak) {
                peak = r->rep.gyro_peak_dps[HM_UNIT_PALM];
            }
            if (full_scale > 0.0) {
                r->rep.gyro_peak_fraction_of_full_scale = peak / full_scale;
            }
        }

        r->rep.bringup_matches_vendor =
            (r->rep.bringup_len == sizeof(k_vendor_bringup)) &&
            (memcmp(r->rep.bringup, k_vendor_bringup, sizeof(k_vendor_bringup)) == 0);

        decide_verdicts(&r->rep);
    }
    if (out_report != NULL) {
        *out_report = r->rep;
    }
}

int hm_reconcile_disagreements(const hm_reconcile_report *rep)
{
    int n = 0;
    const hm_check_verdict v[] = {
        rep->verdict_frame_length, rep->verdict_quat_norm,
        rep->verdict_rate,         rep->verdict_tick_ratio,
        rep->verdict_skew,         rep->verdict_bursts,
        rep->verdict_palm_is_second_block, rep->verdict_bringup,
        rep->verdict_keepalive,    rep->verdict_history_rate,
        rep->verdict_retrieval_continuity};
    for (size_t i = 0; i < sizeof(v) / sizeof(v[0]); ++i) {
        if (v[i] == HM_CHECK_DIFFERS) {
            n++;
        }
    }
    return n;
}

int hm_reconcile_unmeasured(const hm_reconcile_report *rep)
{
    int n = 0;
    const hm_check_verdict v[] = {
        rep->verdict_frame_length, rep->verdict_quat_norm,
        rep->verdict_rate,         rep->verdict_tick_ratio,
        rep->verdict_skew,         rep->verdict_bursts,
        rep->verdict_palm_is_second_block, rep->verdict_bringup,
        rep->verdict_keepalive,    rep->verdict_history_rate,
        rep->verdict_retrieval_continuity};
    for (size_t i = 0; i < sizeof(v) / sizeof(v[0]); ++i) {
        if (v[i] == HM_CHECK_NO_EVIDENCE) {
            n++;
        }
    }
    return n;
}
