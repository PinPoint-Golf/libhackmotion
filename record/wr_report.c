/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_report.c — printing a reconciliation report.
 *
 * ⚠ THE ONE RULE THIS FILE ENFORCES: never print an estimate without the thing
 * that says what it is worth.  A mean without its n, a mismatch count without
 * the sample count, a fitted rate without the fit's own degeneracy flags — each
 * of those reads as a result when it may be nothing at all.  So every line
 * below carries its count, and a quantity with no observations prints
 * "no evidence" rather than a zero that would read as agreement.
 */
#include "wrist/record.h"

#include <string.h>
#include <math.h>

/* Column widths chosen so the claim, the expectation and the measurement line
 * up in a terminal; the section reference is part of the line, not a footnote,
 * because a reader disagreeing with a verdict must be able to go and read it. */
#define CLAIM_W 34
#define CONT_W  58 /* 2 + [11] + 2 + 6 + 1 + CLAIM_W + 1 */

/*
 * ⚠ printf's field width counts BYTES.  Half the labels here carry a §, a ± or
 * a ⚠, each of which is two or three bytes and one column, so %-6s pads "§6.3"
 * and "§10.3" to different widths and the table walks.  Display width is the
 * count of bytes that are not UTF-8 continuation bytes.
 */
static size_t display_width(const char *s)
{
    const unsigned char *p;
    size_t w = 0;
    for (p = (const unsigned char *)s; *p != '\0'; ++p) {
        if ((*p & 0xc0u) != 0x80u) {
            w++;
        }
    }
    return w;
}

static void pad_to(FILE *out, const char *s, size_t width)
{
    size_t w = display_width(s);
    fputs(s, out);
    while (w < width) {
        fputc(' ', out);
        w++;
    }
}

static void rule(FILE *out, const char *title)
{
    size_t w = display_width(title);
    fprintf(out, "\n%s\n", title);
    for (size_t i = 0; i < w; ++i) {
        fputc('-', out);
    }
    fputc('\n', out);
}

static void claim(FILE *out, wr_check_verdict v, const char *section, const char *name)
{
    fputs("  [", out);
    pad_to(out, wr_check_verdict_name(v), 11u);
    fputs("] ", out);
    pad_to(out, section, 6u);
    fputc(' ', out);
    pad_to(out, name, CLAIM_W);
    fputc(' ', out);
}

/* Continuation line, aligned under the measurement column. */
static void cont(FILE *out)
{
    fprintf(out, "%*s", CONT_W, "");
}

static void print_stat(FILE *out, const wr_stat *s, const char *units)
{
    if (s->n == 0u) {
        fprintf(out, "no evidence (n=0)");
        return;
    }
    /* ⚠ Six significant figures, not four: a quaternion norm is 16384.7 and the
     * whole point of the check is the first decimal place. */
    fprintf(out, "%.6g ± %.3g %s  [n=%llu, %.6g … %.6g]", s->mean, wr_stat_sd(s), units,
            (unsigned long long)s->n, s->min, s->max);
}

static void print_flags(FILE *out, uint32_t flags)
{
    static const wr_clock_flag all[] = {
        WR_CLOCK_HAS_FIT,          WR_CLOCK_RATE_POOLED, WR_CLOCK_DEGENERATE,
        WR_CLOCK_RATE_IMPLAUSIBLE, WR_CLOCK_STALE,       WR_CLOCK_BLIND,
        WR_CLOCK_EXTERNAL_CORRECTION, WR_CLOCK_SHORT_BASELINE};
    bool any = false;
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) {
        if ((flags & (uint32_t)all[i]) != 0u) {
            fprintf(out, "%s%s", any ? "," : "", wr_clock_flag_name(all[i]));
            any = true;
        }
    }
    if (!any) {
        fputs("none", out);
    }
}

static void print_warnings(FILE *out, uint32_t bits)
{
    bool any = false;
    for (int c = 1; c < WR_WARN_CODE_COUNT; ++c) {
        if ((bits & (1u << (unsigned)c)) != 0u) {
            fprintf(out, "%s%s", any ? ", " : "", wr_warning_code_name((wr_warning_code)c));
            any = true;
        }
    }
    if (!any) {
        fputs("none", out);
    }
}

static double seconds(wr_time_us us)
{
    return (double)us / 1e6;
}

void wr_reconcile_print(const wr_reconcile_report *rep, FILE *out)
{
    char cfgtext[WR_CONFIG_DESCRIBE_SIZE];

    if (rep == NULL || out == NULL) {
        return;
    }

    fprintf(out, "wrwire reconciliation — a capture read back against docs/specification.md\n");

    rule(out, "Recording");
    fprintf(out, "  device_id            %s\n",
            rep->info.device_id[0] ? rep->info.device_id : "(none)");
    fprintf(out, "  host clock           %s\n", rep->info.clock);
    fprintf(out, "  sample layout        v%u\n", (unsigned)rep->info.layout_version);
    fprintf(out, "  identifiers          %s\n",
            rep->info.identifiers_recorded ? "RECORDED (0x85/0x86 in the clear)"
                                           : "redacted");
    (void)wr_stream_config_describe(rep->config, cfgtext, sizeof(cfgtext));
    fprintf(out, "  configuration        %s  (%s)\n", cfgtext,
            rep->config_from_stream ? "from the recorded a0/82 write"
                                    : "⚠ from the file header — no start command in this capture");
    fprintf(out, "  duration             %.3f s\n", seconds(rep->duration_us));
    fprintf(out, "  chunks               %llu total = %llu host→device + %llu device→host + "
                 "%llu meta\n",
            (unsigned long long)rep->chunks, (unsigned long long)rep->host_writes,
            (unsigned long long)rep->device_notifications,
            (unsigned long long)rep->meta_chunks);
    if (rep->chunks_after_loss > 0u) {
        fprintf(out, "  ⚠ LOSSY              %llu chunk(s) followed a drop — every count below "
                     "is over a recording with holes in it\n",
                (unsigned long long)rep->chunks_after_loss);
    }
    if (rep->redacted_chunks > 0u) {
        fprintf(out, "  redacted             %llu chunk(s) had identifiers removed\n",
                (unsigned long long)rep->redacted_chunks);
    }

    rule(out, "Message census (device → host, spec §5.1)");
    for (int id = 0; id < 256; ++id) {
        if (rep->message_count[id] != 0u) {
            fprintf(out, "  0x%02x  %10llu\n", id, (unsigned long long)rep->message_count[id]);
        }
    }
    fprintf(out, "  unknown ids          %llu  (§5.1: log and ignore, never an error)\n",
            (unsigned long long)rep->unknown_messages);
    fprintf(out, "  decode errors        %llu\n", (unsigned long long)rep->decode_errors);
    fprintf(out, "  codec warnings       ");
    print_warnings(out, rep->codec_warnings);
    fputc('\n', out);

    rule(out, "Claims");

    /* --- §6.3 ------------------------------------------------------------- */
    claim(out, rep->verdict_frame_length, "§6.3", "notification length");
    if (rep->notif_one_record + rep->notif_two_records + rep->notif_other_len == 0u) {
        fprintf(out, "no frames in this capture\n");
    } else {
        fprintf(out, "%zu B ×%llu, %zu B ×%llu, other ×%llu\n", rep->expected_len_one_record,
                (unsigned long long)rep->notif_one_record, rep->expected_len_two_records,
                (unsigned long long)rep->notif_two_records,
                (unsigned long long)rep->notif_other_len);
    }
    cont(out);
    fprintf(out, "records: %llu live, %llu history (bracketed)\n", (unsigned long long)rep->records_live, (unsigned long long)rep->records_history);

    /* --- §6.4 quaternion norm --------------------------------------------- */
    claim(out, rep->verdict_quat_norm, "§6.4", "|q| = 16384.7 ± 0.41");
    if (rep->quat_norm[WR_UNIT_LOWER_ARM].n == 0u) {
        fprintf(out, "no evidence (n=0)\n");
    } else {
        fprintf(out, "%llu outside tolerance\n", (unsigned long long)rep->quat_norm_suspect);
        for (int u = 0; u < WR_UNIT_COUNT; ++u) {
            cont(out);
            fprintf(out, "%-9s ", wr_unit_name((wr_unit)u));
            print_stat(out, &rep->quat_norm[u], "counts");
            fputc('\n', out);
        }
    }

    /* --- §6.5 rate --------------------------------------------------------- */
    claim(out, rep->verdict_rate, "§6.5", "internal rate ≈799.2 Hz, not 800");
    if (rep->verdict_rate == WR_CHECK_NO_EVIDENCE && rep->fitted_rate_hz == 0.0) {
        fprintf(out, "no usable fit (observations=%d)\n", rep->fit.observations);
    } else {
        fprintf(out, "%.3f Hz  [n=%d live frames, span %.1f s]\n", rep->fitted_rate_hz,
                rep->fit.observations, seconds(rep->fit.span_us));
        if (rep->stream_starts > 1u) {
            /* ⚠ The counter resets at every start but the crystal does not
             * (§10), so the fit is per stream while records_live counts them
             * all.  Saying so is the difference between an estimate and an
             * estimate somebody can trust. */
            cont(out);
            fprintf(out, "⚠ %u stream starts in this capture — this fit covers the LAST "
                         "stream only\n",
                    rep->stream_starts);
        }
        cont(out);
        fprintf(out, "%+.1f ppm vs 799.2   %+.1f ppm vs 800  "
                     "(⚠ the round number costs ≈1 ms/s, one-directional)\n",
                rep->ppm_vs_nominal, rep->ppm_vs_800);
        /* ⚠ Precision, reported next to the estimate it qualifies — a fit that
         * has collapsed onto one point still reports small residuals (§10). */
        cont(out);
        fprintf(out, "residuals: median %u µs, p90 %u µs, max %u µs; flags: ",
                rep->fit.residual_median_us, rep->fit.residual_p90_us,
                rep->fit.residual_max_us);
        print_flags(out, rep->fit.flags);
        fputc('\n', out);
    }

    /* --- §6.5 / §6.4 ticks -------------------------------------------------- */
    claim(out, rep->verdict_tick_ratio, "§6.5", "80.166 ticks/sample, units ≤2 ppm");
    if (!rep->tick_ratio_fitted[WR_UNIT_LOWER_ARM] || !rep->tick_ratio_fitted[WR_UNIT_PALM]) {
        fprintf(out, "no evidence — too few live records to fit a line\n");
    } else {
        fprintf(out, "lower_arm %.4f, palm %.4f  [least squares, n=%llu]\n",
                rep->ticks_per_index[WR_UNIT_LOWER_ARM], rep->ticks_per_index[WR_UNIT_PALM],
                (unsigned long long)rep->tick_fit_n);
        cont(out);
        fprintf(out, "→ %.0f ticks/s (§6.4 expects ≈64,068; four sessions span "
                     "64,025-64,088)\n",
                rep->tick_rate_hz);
        cont(out);
        if (!rep->rel_rate_measured) {
            fputs("relative rate palm−arm: no evidence\n", out);
        } else {
            /* ⚠ The estimate and what it is worth, on one line, because a ppm
             * figure with no standard error cannot test a claim stated in ppm. */
            fprintf(out, "relative rate palm−arm: %+.2f ± %.2f ppm  (§6.5 claims ≤%.0f)\n",
                    rep->rel_rate_ppm, rep->rel_rate_ppm_sigma, WR_ONE_RATE_CLAIM_PPM);
            if (rep->rel_rate_ppm_sigma > WR_ONE_RATE_CLAIM_PPM) {
                cont(out);
                fprintf(out, "⚠ this capture resolves only ±%.2f ppm, so it CANNOT test a "
                             "%.0f ppm claim — stream for longer\n",
                        rep->rel_rate_ppm_sigma, WR_ONE_RATE_CLAIM_PPM);
            }
        }
    }
    cont(out);
    /* ⚠ 0 is a coin toss, so an unmeasured margin must not print as 0. */
    fputs("worst wrap margin: ", out);
    for (int u = 0; u < WR_UNIT_COUNT; ++u) {
        fprintf(out, "%s%s ", (u == 0) ? "" : ", ", wr_unit_name((wr_unit)u));
        if (rep->wrap_margin_measured[u]) {
            fprintf(out, "%u", rep->worst_wrap_margin[u]);
        } else {
            fputs("no evidence", out);
        }
    }
    fputs(" of ±32768 (§10.2 measured 8.4% of budget used)\n", out);

    /* --- §10.3 skew --------------------------------------------------------- */
    claim(out, rep->verdict_skew, "§10.3", "skew palm−arm: small and stable");
    if (rep->skew_stored < 2u) {
        fprintf(out, "no evidence (n=%llu)\n", (unsigned long long)rep->skew_stored);
    } else {
        fprintf(out, "median %.1f ticks (§10.3 measured 59)\n", rep->skew_median_ticks);
        cont(out);
        fprintf(out, "first half %.1f, second half %.1f  [n=%llu",
                rep->skew_median_first_half, rep->skew_median_second_half,
                (unsigned long long)rep->skew_stored);
        if (rep->skew_stored < rep->skew_total) {
            /* ⚠ A cap nobody reports reads as coverage. */
            fprintf(out, " of %llu — ⚠ STORE TRUNCATED, medians cover the first %llu only",
                    (unsigned long long)rep->skew_total, (unsigned long long)rep->skew_stored);
        }
        fprintf(out, "]\n");
        if (!isnan(rep->skew_median_us)) {
            cont(out);
            fprintf(out, "= %.3f ms — worth ~0.9° in the relative angle at 1,000 °/s\n",
                    rep->skew_median_us / 1000.0);
        }
    }

    /* --- §6.6 bursts --------------------------------------------------------- */
    claim(out, rep->verdict_bursts, "§6.6", "100 Hz is NOT the live ceiling");
    if (rep->steps_total == 0u) {
        fprintf(out, "no index steps in this capture\n");
    } else {
        fprintf(out, "%.2f%% dense  [n=%llu steps]\n", rep->dense_fraction * 100.0,
                (unsigned long long)rep->steps_total);
        for (int b = 0; b < WR_STEP_BUCKET_COUNT; ++b) {
            cont(out);
            fprintf(out, "%-12s %8llu\n", wr_step_bucket_name((wr_step_bucket)b), (unsigned long long)rep->step[b]);
        }
        cont(out);
        if (rep->live_max_adjacent_run > 1u) {
            /* ⚠ The run, not the count.  A "dense" bucket of 1-7 cannot tell a
             * scattered step-3 from a sustained burst at the full internal
             * rate, and only the latter proves 800 Hz was ever delivered. */
            fprintf(out, "longest run of ADJACENT samples: %u records (~%.0f ms at 799.2 Hz), "
                         "peak |ω| %.0f °/s\n",
                    rep->live_max_adjacent_run,
                    1000.0 * rep->live_max_adjacent_run / 799.2,
                    rep->live_adjacent_peak_gyro_dps);
        } else {
            fputs("longest run of ADJACENT samples: none — the full internal rate was "
                  "never delivered live\n", out);
        }
        if (rep->index_regressions > 0u) {
            cont(out);
            fprintf(out, "⚠ %u implausible forward step(s) — reordered or corrupt "
                         "frames\n",
                    rep->index_regressions);
        }
    }

    /* --- §6.3 palm identification --------------------------------------------- */
    claim(out, rep->verdict_palm_is_second_block, "§6.3", "block 1 is the palm (larger |a|)");
    if (rep->accel_palm_minus_arm_fast.n == 0u) {
        fprintf(out, "no evidence — nothing in this capture exceeded %.0f °/s\n",
                rep->fast_gyro_threshold_dps);
    } else {
        fprintf(out, "palm − arm = ");
        print_stat(out, &rep->accel_palm_minus_arm_fast, "m/s²");
        fputc('\n', out);
        cont(out);
        fprintf(out, "§6.4 measured 31-51 m/s² more at ~2,100 °/s. ⚠ A NEGATIVE mean "
                     "means the blocks are swapped\n");
    }

    /* --- §6.4 headroom ---------------------------------------------------------- */
    claim(out, WR_CHECK_NOTE, "§6.4", "gyro headroom (clipping is silent)");
    fprintf(out, "peak |ω| %.0f °/s = %.0f%% of full scale\n",
            (rep->gyro_peak_dps[WR_UNIT_PALM] > rep->gyro_peak_dps[WR_UNIT_LOWER_ARM])
                ? rep->gyro_peak_dps[WR_UNIT_PALM]
                : rep->gyro_peak_dps[WR_UNIT_LOWER_ARM],
            rep->gyro_peak_fraction_of_full_scale * 100.0);
    cont(out);
    fprintf(out, "pinned samples: %u total", rep->pinned.total);
    if (rep->pinned.total > 0u) {
        fputs(" —", out);
        for (int u = 0; u < WR_UNIT_COUNT; ++u) {
            for (int ch = 0; ch < WR_CHANNEL_COUNT; ++ch) {
                if (rep->pinned.n[u][ch] > 0u) {
                    fprintf(out, " %s.%s=%u", wr_unit_name((wr_unit)u),
                            wr_channel_name((wr_channel)ch), rep->pinned.n[u][ch]);
                }
            }
        }
    }
    fputc('\n', out);
    for (int u = 0; u < WR_UNIT_COUNT; ++u) {
        cont(out);
        fprintf(out, "%-9s |a| ", wr_unit_name((wr_unit)u));
        print_stat(out, &rep->accel_mag_mps2[u], "m/s²");
        fputc('\n', out);
    }

    /* --- §9.1 bring-up ------------------------------------------------------------ */
    claim(out, rep->verdict_bringup, "§9.1", "vendor bring-up sequence");
    if (rep->bringup_len == 0u) {
        fprintf(out, "no host writes before the first stream start\n");
    } else {
        for (size_t i = 0; i < rep->bringup_len; ++i) {
            fprintf(out, "%s%02x", (i == 0u) ? "" : " ", rep->bringup[i]);
        }
        fprintf(out, "   (§9.1: 80 81 84 81 86 86 86 85; only step 2 is required)\n");
    }

    /* --- §9.2 keepalive --------------------------------------------------------- */
    claim(out, rep->verdict_keepalive, "§9.2", "a host write at least every 30 s");
    if (rep->host_writes < 2u) {
        fprintf(out, "no evidence (n=%llu writes)\n", (unsigned long long)rep->host_writes);
    } else {
        fprintf(out, "largest gap %.1f s  [n=%llu writes, %llu were 0x81]\n",
                seconds(rep->max_host_write_gap_us), (unsigned long long)rep->host_writes,
                (unsigned long long)rep->status_polls);
        if (rep->max_host_write_gap_us > (wr_time_us)300 * 1000 * 1000) {
            cont(out);
            fprintf(out, "⚠ past the device's 5.0 min idle shutdown — an active stream "
                         "does NOT prevent it\n");
        }
    }

    /* --- §7.3 the motion-adaptive buffer ------------------------------------------ */
    claim(out, rep->verdict_history_rate, "§7.3", "buffer rate tracks the motion");
    if (rep->records_history < 2u) {
        fprintf(out, "no retrieval in this capture\n");
    } else {
        fprintf(out, "modal step %u (~%.0f Hz), |ω| over the retrieval ",
                rep->history_modal_step,
                rep->history_modal_step ? 799.2 / (double)rep->history_modal_step : 0.0);
        print_stat(out, &rep->history_gyro_dps, "°/s");
        fputc('\n', out);
        cont(out);
        fputs("step: ", out);
        for (size_t k = 1; k < 10u; ++k) {
            if (rep->history_step_count[k] > 0u) {
                if (k == 9u) {
                    fprintf(out, ">8×%llu ", (unsigned long long)rep->history_step_count[k]);
                } else {
                    fprintf(out, "%zu×%llu ", k,
                            (unsigned long long)rep->history_step_count[k]);
                }
            }
        }
        fprintf(out, " (§7.3: step 8 ≈100 Hz at 0.4-28 °/s, step 1 ≈799 Hz at 780-850 °/s)\n");
        cont(out);
        if (rep->history_max_adjacent_run > 1u) {
            fprintf(out, "longest run of ADJACENT samples: %u records (~%.0f ms of full-rate "
                         "replay)\n",
                    rep->history_max_adjacent_run,
                    1000.0 * rep->history_max_adjacent_run / 799.2);
        } else {
            fputs("longest run of ADJACENT samples: none — this retrieval never returned "
                  "the full internal rate\n", out);
        }
        if (rep->history_step_count[9] > 0u || rep->history_step_count[0] > 0u) {
            cont(out);
            fputs("⚠ §7.3 measured 17,739 steps with none above 8 and none at 0\n", out);
        }
        if (!rep->history_exercised_full_rate) {
            /* ⚠ The distinction the first retrieval in this project got wrong. */
            cont(out);
            fprintf(out, "⚠ peak |ω| in the retrieval was %.0f °/s, so this capture did NOT "
                         "exercise the full-rate path.\n",
                    rep->history_peak_gyro_dps);
            cont(out);
            fputs("  An even one-in-eight over a still wrist is CORRECT and is "
                  "indistinguishable from a broken path.\n", out);
        }
    }

    /* --- §7.5 the recording gap a mid-stream retrieval was supposed to remove --- */
    claim(out, rep->verdict_retrieval_continuity, "§7.5",
          "a mid-stream pull costs no gap");
    if (rep->retrievals_measured == 0u) {
        fputs("no retrieval with live frames on both sides\n", out);
    } else if (rep->retrievals_stalled == 0u) {
        fprintf(out, "the sample counter kept advancing across all %u retrieval(s)\n",
                rep->retrievals_measured);
    } else {
        fprintf(out, "⚠ the sample counter STALLED in %u of %u retrieval(s)\n",
                rep->retrievals_stalled, rep->retrievals_measured);
        cont(out);
        fprintf(out, "recording lost per pull ");
        print_stat(out, &rep->retrieval_stall_ms, "ms");
        fputc('\n', out);
        cont(out);
        /* ⚠ The fraction is the claim: the gap equals the PULL, at any size. */
        fprintf(out, "as a fraction of the pull ");
        print_stat(out, &rep->retrieval_stall_fraction, "");
        fputc('\n', out);
        cont(out);
        fprintf(out, "cumulative: %.0f ms of recording never taken, %.0f ticks\n",
                rep->stall_total_ms, rep->stall_total_ticks);
        cont(out);
        /* ⚠ §10.2: one stall is ~72%% of the ±32,768 that picks the right wrap,
         * where a pull-free gap uses 12 ticks.  Two pulls in one live-frame gap
         * already exceed it. */
        fprintf(out, "worst single stall %.0f ticks = %.0f%% of the ±32768 wrap budget "
                     "(a pull-free gap uses ~12)\n",
                rep->stall_worst_ticks, 100.0 * rep->stall_worst_ticks / 32768.0);
        cont(out);
        fputs("§7.5: \"issuing `a1` in place removes that window entirely\" — it does not.\n",
              out);
        cont(out);
        fputs("⚠ §10: the mapping is PIECEWISE — one rate per connection, one offset per "
              "stretch between pulls.\n", out);
        cont(out);
        /* ⚠ Both framings, because they differ by 2× and the wrong one mis-sizes
         * a client: the modulus is 65,536 ticks, the decision budget ±32,768. */
        fprintf(out, "  A fit anchored once at stream start is out by %.0f ticks = %.2f wraps "
                     "of 65536 = %.2f× the ±32768 decision budget.\n",
                rep->stall_total_ticks, rep->stall_total_ticks / 65536.0,
                rep->stall_total_ticks / 32768.0);
    }

    /* --- §6.1 / §7 --------------------------------------------------------------- */
    rule(out, "Session");
    fprintf(out, "  stream starts        %u\n", rep->stream_starts);
    if (rep->stream_start_latency_us == WR_TIME_UNKNOWN) {
        fprintf(out, "  first-frame latency  not measured (no start command in this capture)\n");
    } else {
        fprintf(out, "  first-frame latency  %.1f ms  (§6.1 measured 50-80 ms)\n",
                (double)rep->stream_start_latency_us / 1000.0);
        if (rep->stream_start_latency_us > (wr_time_us)1000 * 1000 ||
            rep->max_frame_burst >= 16u) {
            /* ⚠ Paid for once: an analysis keyed on arrival time, run inside a
             * window whose arrival times were already known to be fabricated,
             * returned two calibration poses the wrong way round. */
            fprintf(out, "  ⚠ ARRIVAL TIMES SUSPECT   longest burst %u frames arriving "
                         "<2 ms apart — a backlog being dispatched, not the device.\n",
                    rep->max_frame_burst);
            fputs("                       Anything keyed on WHEN a frame arrived must "
                  "use device time (the sample index) instead.\n", out);
        }
    }
    fprintf(out, "  history brackets     %u opened, %u closed\n", rep->brackets_opened,
            rep->brackets_closed);
    if (rep->brackets_opened != rep->brackets_closed) {
        fprintf(out, "  ⚠ unbalanced         a bracket that never closed leaves the device "
                     "replaying and suppresses host writes\n");
    }
    fprintf(out, "  device errors (0xd0) %llu\n", (unsigned long long)rep->device_errors);
    fprintf(out, "  button (0xfb)        %llu  (§9.4: the count per press is NOT reliable)\n",
            (unsigned long long)rep->buttons);

    rule(out, "Summary");
    fprintf(out, "  %d claim(s) DIFFER from the specification\n",
            wr_reconcile_disagreements(rep));
    fprintf(out, "  %d claim(s) have NO EVIDENCE in this capture — ⚠ that is not agreement\n",
            wr_reconcile_unmeasured(rep));
}
