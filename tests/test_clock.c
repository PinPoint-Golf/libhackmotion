/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_clock.c — the device→host mapping.
 *
 * This is the file that decides whether a swing can be placed on a camera
 * timeline, so the tests are written against the specification's numbers rather
 * than against round ones: 799.19 Hz, one-sided BLE delay, 2.2 ms/s of measured
 * drift, and a 4.17 ms camera frame at 240 fps.
 */
#include "hm_test.h"
#include "hm_fit.h"
#include "hm_unwrap.h"

#define TRUE_RATE_HZ   799.19                  /* §6.5, phone session, 238 s   */
#define TRUE_SLOPE_US  (1e6 / TRUE_RATE_HZ)    /* ≈1251.267 µs per index       */
#define T0             ((hm_time_us)1700000000000000LL) /* arbitrary epoch      */

/* Deterministic one-sided link delay.  BLE notifications can be late, never
 * early (§10), and bunching makes the distribution lumpy rather than gaussian. */
static hm_time_us link_delay(uint32_t i)
{
    if (i % 7u == 0u) {
        return 0; /* the occasional undelayed frame is what the envelope finds */
    }
    return (hm_time_us)(500 + (int)((i * 7919u) % 15000u));
}

static hm_time_us true_host_us(uint32_t index)
{
    return T0 + (hm_time_us)llround((double)index * TRUE_SLOPE_US);
}

/* Feed a session's worth of live frames at the two dominant live steps. */
static uint32_t feed_session(hm_fit *f, uint32_t records, bool with_delay)
{
    uint32_t index = 0;
    for (uint32_t k = 0; k < records; ++k) {
        hm_time_us h = true_host_us(index) + (with_delay ? link_delay(index) : 0);
        hm_fit_observe(f, index, h);
        index += (k % 7u == 6u) ? 8u : 32u;
    }
    return index;
}

/* ------------------------------------------------------------------------ */

/*
 * ⚠ §6.5, §10 — the rate is FITTED, and it is ≈799.2, not 800.  A round 800
 * costs ≈1,000 ppm: 1 ms per second of streaming, one-directional.
 */
HM_TEST(clock_fits_the_rate_rather_than_assuming_800)
{
    hm_fit f;
    hm_clock_snapshot s;
    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 6000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    HM_ASSERT(s.flags & HM_CLOCK_HAS_FIT);
    HM_ASSERT(!(s.flags & HM_CLOCK_SHORT_BASELINE));
    HM_ASSERT(!(s.flags & HM_CLOCK_RATE_IMPLAUSIBLE));

    /* Within 100 ppm of the truth — a tenth of the error a round 800 would
     * introduce, and inside the ~0.1 % per-device spread of §6.4. */
    HM_ASSERT_NEAR(s.fitted_rate_hz, TRUE_RATE_HZ, TRUE_RATE_HZ * 100e-6);
    HM_ASSERT(s.fitted_rate_hz < 800.0);
    HM_ASSERT(s.observations > 5000);
    HM_ASSERT(s.span_us > 200 * 1000 * 1000); /* a 238 s session */
}

/*
 * ⚠ §10 — FIT THE LOWER ENVELOPE, NOT LEAST SQUARES.  Least squares biases the
 * offset by the MEAN link delay, which reaches the user as every swing landing
 * consistently late.  This test computes both and shows the difference.
 */
HM_TEST(clock_lower_envelope_beats_least_squares_on_one_sided_delay)
{
    hm_fit f;
    hm_clock_snapshot s;
    double sum_i = 0, sum_h = 0, sum_ii = 0, sum_ih = 0, n = 0;
    double ls_slope, ls_offset;
    double envelope_error, ls_error;
    uint32_t index = 0;
    double mean_delay = 0;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);

    for (uint32_t k = 0; k < 4000u; ++k) {
        hm_time_us d = link_delay(index);
        hm_time_us h = true_host_us(index) + d;
        double hi = (double)(h - T0);
        hm_fit_observe(&f, index, h);
        sum_i += index;
        sum_h += hi;
        sum_ii += (double)index * (double)index;
        sum_ih += (double)index * hi;
        mean_delay += (double)d;
        n += 1;
        index += (k % 7u == 6u) ? 8u : 32u;
    }
    mean_delay /= n;

    ls_slope = (n * sum_ih - sum_i * sum_h) / (n * sum_ii - sum_i * sum_i);
    ls_offset = (sum_h - ls_slope * sum_i) / n;

    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    envelope_error = fabs((double)(s.offset_us - T0));
    ls_error = fabs(ls_offset);

    /* The envelope lands on the true line; least squares lands a mean delay
     * above it — about 6 ms here, which is 1.5 camera frames at 240 fps. */
    HM_ASSERT(mean_delay > 4000.0);
    HM_ASSERT_MSG(envelope_error < 2000.0, "lower-envelope offset should sit on the true line");
    HM_ASSERT_MSG(ls_error > mean_delay * 0.5,
                  "least squares should be biased by roughly the mean delay");
    HM_ASSERT_MSG(envelope_error < ls_error * 0.5, "envelope must beat least squares");
}

/*
 * ⚠ §10's second, subtler failure: with the rate pinned wrong, the estimator
 * SILENTLY DEGENERATES — its support collapses onto one end of the session
 * while it still reports small residuals.  The library must notice.
 */
HM_TEST(clock_flags_the_degenerate_estimator_when_the_rate_is_pinned_to_800)
{
    hm_fit f;
    hm_clock_snapshot pinned, fitted;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);

    /* This is what hard-coding a round 800 Hz does. */
    hm_fit_force_slope(&f, 1e6 / 800.0);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &pinned);
    HM_ASSERT_MSG(pinned.flags & HM_CLOCK_DEGENERATE,
                  "a pinned 800 Hz must be reported as degenerate, not published quietly");

    /* And the joint fit over the same data is not degenerate. */
    hm_fit_force_slope(&f, 0.0);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &fitted);
    HM_ASSERT(!(fitted.flags & HM_CLOCK_DEGENERATE));

    /* ⚠ The degenerate fit's residuals still look small — which is exactly why
     * residual spread cannot be the only thing a library reports. */
    HM_ASSERT_MSG(hm_clock_uncertainty_us(&pinned, pinned.last_index) >
                      hm_clock_uncertainty_us(&fitted, fitted.last_index),
                  "uncertainty must reflect degeneracy even when residuals do not");
}

/* A pinned rate that happens to be right is not degenerate. */
HM_TEST(clock_pinning_the_correct_rate_is_not_flagged)
{
    hm_fit f;
    hm_clock_snapshot s;
    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_force_slope(&f, TRUE_SLOPE_US);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT(!(s.flags & HM_CLOCK_DEGENERATE));
}

/*
 * ⚠⚠ A ONE-OBSERVATION FIT MUST NOT ACCEPT A BUDGET IT CANNOT SUPPORT.
 *
 * implementation-review I2: every additive term in hm_clock_error_at() is a
 * multiple of residual_p90_us or residual_max_us, and with one observation both
 * are 0 by construction — a line through one point has no residual.  So
 * precision_us was 0 and hm_clock_meets_budget() returned true for EVERY budget,
 * which is api-request B2's refuse-rather-than-misalign gate silently off.
 *
 * ⚠ This asserts the CHOICE, not the number.  What matters is that a budget
 * finer than the evidence is REFUSED and a generous one is still accepted — a
 * floor that refused everything would be just as broken, and would look like a
 * fix.
 */
HM_TEST(clock_a_fit_on_one_instant_refuses_a_budget_it_cannot_support)
{
    hm_fit f;
    hm_clock_snapshot s;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    hm_fit_observe(&f, 0, true_host_us(0) + 500);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    HM_ASSERT(s.flags & HM_CLOCK_HAS_FIT);
    HM_ASSERT_EQ(s.span_us, 0);
    HM_ASSERT_MSG(!hm_clock_meets_budget(&s, 0, 100u),
                  "100 us from one observation is a claim, not a measurement");
    HM_ASSERT_MSG(!hm_clock_meets_budget(&s, 0, 1000u),
                  "nor is a millisecond: the anchor rests on one jittered arrival");

    /* ⚠ And it is a floor, not a refusal.  One sample period is ≈1251 µs — under
     * a third of a 240 fps camera frame — so frame-accurate alignment, which is
     * what a consumer actually asks for, still passes from the first frame. */
    HM_ASSERT_MSG(hm_clock_meets_budget(&s, 0, 4170u),
                  "a 240 fps camera frame must still be servable from frame one");
    HM_ASSERT_NEAR((double)hm_clock_precision_us(&s, 0), TRUE_SLOPE_US, 2.0);
}

/*
 * ⚠⚠ AND THE STATE RECURS ONCE PER PULL, WHICH IS WHAT MADE I2 ROUTINE.
 *
 * Design §6.1.1 re-anchors the fit at every history retrieval — hm_fit_begin_
 * stream() — because the sample counter stalls for the pull's duration.  That
 * wipes the residual ring, so without the carry a well-measured link went back
 * to claiming perfect accuracy after every single pull.
 *
 * Link jitter is a property of the LINK, not of the stretch between two pulls,
 * so it survives the re-anchor exactly as the pooled rate does.
 */
HM_TEST(clock_re_anchoring_at_a_pull_does_not_reset_the_measured_link_jitter)
{
    hm_fit f;
    hm_clock_snapshot before, after;
    uint32_t next;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    next = feed_session(&f, 4000u, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &before);

    HM_ASSERT(before.residual_p90_us > 0u);
    HM_ASSERT(!hm_clock_meets_budget(&before, next, 100u));

    /* The pull happens.  §6.1.1: one rate per connection, one offset per
     * stretch between pulls. */
    hm_fit_begin_stream(&f, 1);
    hm_fit_observe(&f, next, true_host_us(next) + 500);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &after);

    HM_ASSERT_EQ(after.observations, 1);
    HM_ASSERT_EQ(after.span_us, 0);
    HM_ASSERT_MSG(after.residual_p90_us >= before.residual_p90_us,
                  "a re-anchor learns nothing new about the link, so it may not "
                  "report a better one");
    HM_ASSERT_MSG(!hm_clock_meets_budget(&after, next, 100u),
                  "a budget refused before the pull must not be accepted after it");
}

/*
 * ⚠ R16 — TIME TO FIRST FIT IS ONE LIVE FRAME.  There is no dead zone at the
 * start of a stream and a consumer need not budget for dropped samples: a ring
 * that requires a host timestamp on every write can take the very first one.
 */
HM_TEST(clock_produces_a_usable_fit_from_the_first_live_frame)
{
    hm_fit f;
    hm_clock_snapshot s;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);

    /* Nothing yet: refuse rather than guess. */
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT(!(s.flags & HM_CLOCK_HAS_FIT));

    /* One frame is enough to map. */
    hm_fit_observe(&f, 0, true_host_us(0) + 500);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    HM_ASSERT_MSG(s.flags & HM_CLOCK_HAS_FIT, "one live frame must yield a usable fit");
    HM_ASSERT_MSG(s.flags & HM_CLOCK_SHORT_BASELINE,
                  "and must say the rate is seeded rather than measured");
    HM_ASSERT_EQ(s.observations, 1);
    HM_ASSERT(hm_clock_to_host_us(&s, 0) != HM_TIME_UNKNOWN);
    HM_ASSERT_NEAR(s.fitted_rate_hz, HM_NOMINAL_SAMPLE_RATE_HZ, 0.001);
    /*
     * ⚠ Its uncertainty is finite and honest — and "honest" needs a LOWER bound
     * as well as an upper one.  This line asserted only `< UINT32_MAX` while the
     * answer was 0, which is how implementation-review I2 survived: every term
     * in hm_clock_error_at() is a multiple of a residual, and one observation
     * has no residuals.  A perfect-accuracy claim from one data point.
     */
    HM_ASSERT(hm_clock_precision_us(&s, 0) < UINT32_MAX);
    HM_ASSERT_MSG(hm_clock_precision_us(&s, 0) > 0u,
                  "a fit resting on one instant must not claim perfect accuracy");

    /* SHORT_BASELINE clears on SPAN, not on count: thousands of observations
     * inside one second still cannot separate rate from offset. */
    {
        uint32_t idx = 0;
        for (uint32_t k = 0; k < 800u; ++k) { /* ~1 s at index step 1 */
            hm_fit_observe(&f, idx, true_host_us(idx) + link_delay(idx));
            idx += 1u;
        }
        hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
        HM_ASSERT(s.observations > 500);
        HM_ASSERT_MSG(s.flags & HM_CLOCK_SHORT_BASELINE,
                      "span, not count, is what gates an independently fitted rate");
        HM_ASSERT(s.span_us < HM_CLOCK_RATE_FIT_MIN_SPAN_US);
    }
}

/*
 * ⚠ R16, second half — device time never depends on the fit.  Analysis anchored
 * on matched events rather than a shared clock needs nothing from the host
 * mapping, and must not have to wait for it.
 */
HM_TEST(clock_device_time_is_independent_of_fit_state)
{
    /* The unwrapper is fed indices and raw ticks and knows nothing about a fit,
     * a host clock or a snapshot — which is the property being asserted. */
    hm_tick_unwrapper u;
    hm_index_unwrapper iu;
    hm_tick_unwrapper_reset(&u);
    hm_index_unwrapper_reset(&iu);

    for (uint32_t k = 0; k < 200u; ++k) {
        uint32_t raw = (k * 32u) & 0xffffu;
        uint32_t idx = hm_index_unwrap(&iu, (uint16_t)raw, NULL);
        int64_t t = (int64_t)((double)idx * HM_NOMINAL_TICKS_PER_SAMPLE);
        int64_t got = hm_tick_unwrap(&u, idx, (uint16_t)(t & 0xffff), NULL);
        HM_ASSERT_EQ(got, t);
        HM_ASSERT_EQ(idx, k * 32u);
    }
    HM_ASSERT(hm_ticks_to_us(u.last_unwrapped, 64068.0) > 0);
}

/*
 * A second of data cannot separate rate from offset — trying produces a wild
 * slope out of a few milliseconds of jitter.  Say so rather than publish it.
 */
HM_TEST(clock_short_baseline_is_flagged_and_falls_back_to_the_seed)
{
    hm_fit f;
    hm_clock_snapshot s;
    uint32_t index = 0;
    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);

    for (uint32_t k = 0; k < 20u; ++k) { /* ~0.8 s */
        hm_fit_observe(&f, index, true_host_us(index) + link_delay(index));
        index += 32u;
    }
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT(s.flags & HM_CLOCK_HAS_FIT);
    HM_ASSERT(s.flags & HM_CLOCK_SHORT_BASELINE);
    HM_ASSERT_NEAR(s.fitted_rate_hz, HM_NOMINAL_SAMPLE_RATE_HZ, 0.001);
}

/*
 * §10 — "a per-session offset with a rate pooled across the whole connection",
 * because the counter resets at every stream start and the crystal does not.
 */
HM_TEST(clock_pools_the_rate_across_streams_but_not_the_offset)
{
    hm_fit f;
    hm_clock_snapshot first, second;
    uint32_t index;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &first);
    HM_ASSERT(!(first.flags & HM_CLOCK_RATE_POOLED));

    /* A new stream: the index space resets, the crystal does not. */
    hm_fit_begin_stream(&f, 2);
    index = 0;
    for (uint32_t k = 0; k < 20u; ++k) { /* deliberately too short to fit a rate */
        hm_fit_observe(&f, index, T0 + 500000000 + (hm_time_us)llround(index * TRUE_SLOPE_US));
        index += 32u;
    }
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &second);

    HM_ASSERT_EQ(second.stream_id, 2);
    HM_ASSERT_MSG(second.flags & HM_CLOCK_RATE_POOLED, "the rate must survive a stream restart");
    HM_ASSERT_NEAR(second.fitted_rate_hz, first.fitted_rate_hz, 0.05);
    /* But the offset is per-stream: it must not have been carried over. */
    HM_ASSERT(second.offset_us != first.offset_us);
    HM_ASSERT_EQ(second.observations, 20);

    /*
     * ⚠ AND THE FLAG CLEARS ONCE THIS STREAM HAS FITTED ITS OWN RATE
     * (implementation-review I11).  It was raised wherever a pooled value merely
     * EXISTED — including on the joint-fit path, where the pooled slope is only
     * a starting point for the hull scan and the published one is measured here.
     * `clock.h` calls it "carried over", so a consumer reading it on a rate this
     * stream just measured is told something untrue about where it came from.
     */
    {
        hm_clock_snapshot third;
        while (index < 4000u * 32u) {
            hm_fit_observe(&f, index,
                           T0 + 500000000 + (hm_time_us)llround(index * TRUE_SLOPE_US) +
                               link_delay(index));
            index += 32u;
        }
        hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &third);
        HM_ASSERT(!(third.flags & HM_CLOCK_SHORT_BASELINE));
        HM_ASSERT_MSG(!(third.flags & HM_CLOCK_RATE_POOLED),
                      "a rate fitted from this stream's own observations was not "
                      "carried over from anywhere");
    }
}

/*
 * api-request §2.1 — let a consumer feed back a correction it measured on its
 * own rig, and carry the provenance so a re-analysis reproduces it (C9).
 *
 * ⚠ BOTH KNOBS MOVE TOGETHER.  A consumer that could set only the rate would
 * feed back a measurement, see its reported uncertainty unchanged, and
 * reasonably conclude the call did nothing.
 */
HM_TEST(clock_correction_moves_both_the_rate_and_the_systematic)
{
    hm_fit f;
    hm_clock_snapshot before, after;
    hm_clock_correction c;
    uint32_t idx_60s;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &before);

    memset(&c, 0, sizeof(c));
    c.fields = HM_CORRECTION_RATE | HM_CORRECTION_DRIFT;
    c.rate_ppm = 500.0;
    c.residual_drift_us_per_s = 120.0; /* measured on the consumer's own rig */
    memcpy(c.provenance, "camera+mic rig A, 2026-08-14", 29);

    HM_ASSERT_EQ(hm_fit_set_correction(&f, &c), HM_OK);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &after);

    HM_ASSERT(after.flags & HM_CLOCK_EXTERNAL_CORRECTION);
    HM_ASSERT_NEAR(after.external_ppm, 500.0, 1e-9);
    HM_ASSERT_NEAR(after.raw_fitted_rate_hz, before.fitted_rate_hz, 1e-6);
    HM_ASSERT_NEAR(after.fitted_rate_hz, before.fitted_rate_hz * (1.0 + 500e-6),
                   before.fitted_rate_hz * 1e-9);
    HM_ASSERT_STR(after.provenance, "camera+mic rig A, 2026-08-14");

    /* The systematic moved too — 2200 µs/s down to 120. */
    HM_ASSERT_NEAR(after.accuracy_drift_us_per_s, 120.0, 1e-9);
    idx_60s = after.first_index + (uint32_t)(TRUE_RATE_HZ * 60.0);
    HM_ASSERT_NEAR((double)hm_clock_error_at(&after, idx_60s).systematic_us, 7200.0, 200.0);
    HM_ASSERT_MSG(hm_clock_error_at(&after, idx_60s).systematic_us <
                      hm_clock_error_at(&before, idx_60s).systematic_us / 10u,
                  "correcting the rig must reduce the reported systematic");

    /* Setting only the rate leaves the systematic exactly where it was. */
    memset(&c, 0, sizeof(c));
    c.fields = HM_CORRECTION_RATE;
    c.rate_ppm = 250.0;
    HM_ASSERT_EQ(hm_fit_set_correction(&f, &c), HM_OK);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &after);
    HM_ASSERT_NEAR(after.external_ppm, 250.0, 1e-9);
    HM_ASSERT_MSG(fabs(after.accuracy_drift_us_per_s - 120.0) < 1e-9,
                  "a rate-only correction must not touch the systematic");

    c.rate_ppm = 1e9;
    HM_ASSERT_EQ(hm_fit_set_correction(&f, &c), HM_ERR_INVALID_ARG);
    HM_ASSERT_EQ(hm_fit_set_correction(&f, NULL), HM_ERR_INVALID_ARG);
}

/*
 * ⚠ R6.  `= {0}` is the idiom for every other struct in this API, so it is what
 * a consumer reaches for by accident — and a zeroed correction must NOT silently
 * wipe a systematic nobody measured.  That failure is optimistic: the capture
 * would claim an accuracy it had not earned, with a provenance string describing
 * a measurement that covered half of it.
 */
HM_TEST(clock_a_zeroed_correction_is_rejected_rather_than_obeyed)
{
    hm_fit f;
    hm_clock_snapshot s;
    hm_clock_correction c;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 2000, true);

    memset(&c, 0, sizeof(c));
    HM_ASSERT_MSG(hm_fit_set_correction(&f, &c) == HM_ERR_INVALID_ARG,
                  "an empty `fields` must be an error, not a wipe and not a no-op");

    /* The natural half-done call: rate set, fields forgotten. */
    c.rate_ppm = -350.0;
    HM_ASSERT_EQ(hm_fit_set_correction(&f, &c), HM_ERR_INVALID_ARG);

    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT_MSG(fabs(s.accuracy_drift_us_per_s - HM_ACCURACY_DRIFT_US_PER_S_DEFAULT) < 1e-9,
                  "a rejected correction must leave the systematic untouched");
    HM_ASSERT_NEAR(s.external_ppm, 0.0, 1e-9);
    HM_ASSERT(!(s.flags & HM_CLOCK_EXTERNAL_CORRECTION));

    /* And "I measured it and it is zero" is expressible, explicitly. */
    c.fields = HM_CORRECTION_DRIFT;
    c.residual_drift_us_per_s = 0.0;
    HM_ASSERT_EQ(hm_fit_set_correction(&f, &c), HM_OK);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT_NEAR(s.accuracy_drift_us_per_s, 0.0, 1e-9);
    HM_ASSERT_EQ(hm_clock_error_at(&s, s.last_index).systematic_us, 0);

    /* A negative drift is nonsense and is refused rather than reinterpreted. */
    c.residual_drift_us_per_s = -1.0;
    HM_ASSERT_EQ(hm_fit_set_correction(&f, &c), HM_ERR_INVALID_ARG);
}

/*
 * ⚠ api-request B2, and the design review's blocking finding R1.
 *
 * The error has two parts and they must NOT be one number:
 *   precision   link jitter + extrapolation.  Bounded, and what a consumer
 *               gates on.
 *   systematic  §10's 2.2 ms per second of session.  Perfectly predictable
 *               (R² = 1.000), measured once on one rig, and unbounded across a
 *               session — 1.32 s at ten minutes.
 *
 * Folded together, `alignment_budget_us` refuses every pull after the first
 * second of any session, which disables the refuse-rather-than-misalign path
 * instead of informing it.
 */
HM_TEST(clock_reports_precision_and_the_systematic_separately)
{
    hm_fit f;
    hm_clock_snapshot s;
    uint32_t idx_1s, idx_60s;
    hm_clock_error e_start, e_1s, e_60s;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 6000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    /* ⚠ 2.2 ms per second, expressed in µs/s.  Getting this unit wrong by three
     * orders of magnitude is silent, so the test states the number twice. */
    HM_ASSERT_NEAR(s.accuracy_drift_us_per_s, 2200.0, 1e-9);
    HM_ASSERT_NEAR(s.accuracy_drift_us_per_s, HM_ACCURACY_DRIFT_US_PER_S_DEFAULT, 1e-9);

    idx_1s = s.first_index + (uint32_t)TRUE_RATE_HZ;
    idx_60s = s.first_index + (uint32_t)(TRUE_RATE_HZ * 60.0);
    e_start = hm_clock_error_at(&s, s.first_index);
    e_1s = hm_clock_error_at(&s, idx_1s);
    e_60s = hm_clock_error_at(&s, idx_60s);

    /* Precision is flat inside the observed span: it is a property of the link
     * and the fit, not of how long the session has been running. */
    HM_ASSERT_EQ(e_start.precision_us, s.residual_p90_us);
    HM_ASSERT_EQ(e_1s.precision_us, e_start.precision_us);
    HM_ASSERT_EQ(e_60s.precision_us, e_start.precision_us);

    /* The systematic grows without bound.  60 s × 2.2 ms/s = 132 ms. */
    HM_ASSERT_EQ(e_start.systematic_us, 0);
    HM_ASSERT_NEAR((double)e_1s.systematic_us, 2200.0, 50.0);
    HM_ASSERT_NEAR((double)e_60s.systematic_us, 132000.0, 2000.0);

    /* And the total is still reported, for the capture's provenance. */
    HM_ASSERT_EQ(e_60s.total_us, e_60s.precision_us + e_60s.systematic_us);
    HM_ASSERT_EQ(hm_clock_uncertainty_us(&s, idx_60s), e_60s.total_us);
    HM_ASSERT_EQ(hm_clock_precision_us(&s, idx_60s), e_60s.precision_us);
}

/*
 * ⚠ R1's reproduction, kept as a regression test.  With a healthy link the gate
 * must stay usable for the whole of a ten-minute lesson.
 */
HM_TEST(clock_alignment_gate_survives_a_ten_minute_session)
{
    hm_clock_snapshot f;
    memset(&f, 0, sizeof(f));
    f.flags = HM_CLOCK_HAS_FIT;
    f.slope_us_per_index = 1e6 / 799.2;
    f.first_index = 0;
    f.last_index = 4000000;
    f.span_us = 5000000;
    f.residual_median_us = 1200;
    f.residual_p90_us = 3000; /* a healthy link */
    f.residual_max_us = 9000;
    f.accuracy_drift_us_per_s = HM_ACCURACY_DRIFT_US_PER_S_DEFAULT;

    for (double t = 0.5; t <= 600.0; t *= 4.0) {
        uint32_t idx = (uint32_t)(t * 799.2);
        HM_ASSERT_MSG(hm_clock_meets_budget(&f, idx, 4167u),
                      "a 3 ms link must meet a camera frame at any point in a session");
        HM_ASSERT_EQ(hm_clock_precision_us(&f, idx), 3000);
    }

    /* At ten minutes the honest TOTAL is still over a second, and still says so. */
    {
        uint32_t idx = (uint32_t)(600.0 * 799.2);
        HM_ASSERT(hm_clock_uncertainty_us(&f, idx) > 1000000u);
        HM_ASSERT_NEAR((double)hm_clock_error_at(&f, idx).systematic_us, 1320000.0, 5000.0);
    }

    /* A bad link still fails the gate — precision is doing real work. */
    f.residual_p90_us = 20000;
    HM_ASSERT(!hm_clock_meets_budget(&f, 799, 4167u));
}

/*
 * ⚠ hm_time_range is HALF-OPEN, hm_index_range is INCLUSIVE.  That mismatch is
 * where an off-by-one is born, so the conversion exists exactly once.
 */
HM_TEST(clock_index_range_conversion_respects_both_conventions)
{
    hm_clock_snapshot f;
    hm_index_range r;
    hm_time_range w;

    memset(&f, 0, sizeof(f));
    f.flags = HM_CLOCK_HAS_FIT;
    f.slope_us_per_index = 1000.0; /* 1 ms per sample, for exact arithmetic */
    f.anchor_index = 0;
    f.anchor_host_us = 0;
    f.first_index = 0;
    f.last_index = 1000000;
    f.span_us = 1000000;

    /* [0, 10000) covers samples 0..9 — ten samples, not eleven. */
    w.start_us = 0;
    w.end_us = 10000;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&f, w, &r), HM_OK);
    HM_ASSERT_EQ(r.first, 0);
    HM_ASSERT_MSG(r.last == 9, "the exclusive end must not pull in sample 10");

    /* A window that starts between two samples rounds up to the first inside. */
    w.start_us = 10500;
    w.end_us = 20000;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&f, w, &r), HM_OK);
    HM_ASSERT_EQ(r.first, 11);
    HM_ASSERT_EQ(r.last, 19);

    /* A window falling entirely between two samples yields nothing, and says so
     * rather than returning an inverted range. */
    w.start_us = 10200;
    w.end_us = 10800;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&f, w, &r), HM_ERR_INVALID_ARG);

    /* Empty and inverted windows are refused. */
    w.start_us = 5000;
    w.end_us = 5000;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&f, w, &r), HM_ERR_INVALID_ARG);
    w.end_us = 1000;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&f, w, &r), HM_ERR_INVALID_ARG);

    /* No fit means refuse, never guess. */
    f.flags = 0;
    w.start_us = 0;
    w.end_us = 10000;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&f, w, &r), HM_ERR_NO_FIT);
}

/*
 * A round trip over a real fit: every sample the conversion claims is inside the
 * window must map back into it, and the two just outside must not.
 */
HM_TEST(clock_index_range_round_trips_against_the_mapping)
{
    hm_fit f;
    hm_clock_snapshot s;
    hm_index_range r;
    hm_time_range w;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    w.start_us = hm_clock_to_host_us(&s, 10000) + 300;
    w.end_us = hm_clock_to_host_us(&s, 13600) - 300;
    HM_ASSERT_EQ(hm_clock_index_range_for_time(&s, w, &r), HM_OK);

    HM_ASSERT(hm_clock_to_host_us(&s, r.first) >= w.start_us);
    HM_ASSERT(hm_clock_to_host_us(&s, r.last) < w.end_us);
    HM_ASSERT(hm_clock_to_host_us(&s, r.first - 1u) < w.start_us);
    HM_ASSERT(hm_clock_to_host_us(&s, r.last + 1u) >= w.end_us);
}

/*
 * The precision term tracks link quality: §10 says to treat residual spread as
 * a link-health signal rather than averaging it away.  A clean link earns a
 * tight number at the fit anchor; a jittery one does not, with no other change.
 */
HM_TEST(clock_precision_term_tracks_link_health)
{
    hm_fit clean, noisy;
    hm_clock_snapshot cs, ns;

    hm_fit_init(&clean, 0.0);
    hm_fit_begin_stream(&clean, 1);
    (void)feed_session(&clean, 4000, false); /* no link delay at all */
    hm_fit_snapshot(&clean, HM_TIME_UNKNOWN, &cs);

    hm_fit_init(&noisy, 0.0);
    hm_fit_begin_stream(&noisy, 1);
    (void)feed_session(&noisy, 4000, true); /* up to ~15 ms of one-sided delay */
    hm_fit_snapshot(&noisy, HM_TIME_UNKNOWN, &ns);

    HM_ASSERT(cs.residual_p90_us < 100u);
    HM_ASSERT(ns.residual_p90_us > 5000u);
    HM_ASSERT_MSG(hm_clock_meets_budget(&cs, cs.first_index, 4167u),
                  "a clean link should meet a camera frame at the fit anchor");
    HM_ASSERT_MSG(!hm_clock_meets_budget(&ns, ns.first_index, 4167u),
                  "a 15 ms-jitter link should not, and must say so");
}

/*
 * api-request B15 — live delivery is suspended while a retrieval is in flight,
 * so the fit gets no new observations for roughly the width of the window.
 *
 * ⚠ THE PENALTY IS TRANSIENT BY DESIGN, and the test name says so because the
 * design document previously claimed more than the code delivers.  While the
 * pull is the most recent thing that happened its span lies beyond
 * `last_index` and is penalised as extrapolation.  Once live resumes,
 * `last_index` advances past it, the gap becomes INTERIOR, and the penalty goes
 * to zero — which is correct: a line supported by observations on both sides
 * interpolates across an interior gap perfectly well, and the device clock has
 * no discontinuity there.  What durably records the blind span is
 * HM_GAP_FIT_BLIND on the block.
 */
HM_TEST(clock_penalises_a_blind_span_while_it_is_the_edge_and_not_once_it_is_interior)
{
    hm_fit f;
    hm_clock_snapshot s;
    uint32_t last;
    uint32_t inside, just_after, well_after;
    uint32_t gap_start, gap_mid;
    uint32_t u_mid_during, u_mid_after;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    last = s.last_index;

    inside = last - 1000u;
    just_after = last + 100u;
    well_after = last + (uint32_t)(TRUE_RATE_HZ * 4.5); /* a 4.5 s pull */
    gap_start = last;
    gap_mid = last + (uint32_t)(TRUE_RATE_HZ * 2.25);

    /* During the pull, the blind span is the edge of the fit and is penalised. */
    HM_ASSERT(hm_clock_precision_us(&s, just_after) >
              hm_clock_precision_us(&s, inside) + 100u);
    HM_ASSERT(hm_clock_precision_us(&s, well_after) >
              hm_clock_precision_us(&s, just_after));
    u_mid_during = hm_clock_precision_us(&s, gap_mid);

    hm_fit_set_blind(&f, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT(s.flags & HM_CLOCK_BLIND);

    /* Live resumes on the far side of the gap. */
    hm_fit_set_blind(&f, false);
    {
        uint32_t idx = well_after;
        for (uint32_t k = 0; k < 500u; ++k) {
            hm_fit_observe(&f, idx, true_host_us(idx) + link_delay(idx));
            idx += 32u;
        }
    }
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    HM_ASSERT(!(s.flags & HM_CLOCK_BLIND));
    HM_ASSERT(s.last_index > well_after);

    /* ⚠ The gap is now interior, and carries no extrapolation penalty at all. */
    u_mid_after = hm_clock_precision_us(&s, gap_mid);
    HM_ASSERT_MSG(u_mid_after < u_mid_during,
                  "an interior gap must not keep an extrapolation penalty");
    HM_ASSERT_EQ(u_mid_after, hm_clock_precision_us(&s, gap_start));
    HM_ASSERT_EQ(u_mid_after, s.residual_p90_us);
}

HM_TEST(clock_maps_indices_to_host_time_and_back)
{
    hm_fit f;
    hm_clock_snapshot s;
    uint32_t round_trip = 0;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    for (uint32_t idx = 0; idx < 100000u; idx += 9973u) {
        hm_time_us h = hm_clock_to_host_us(&s, idx);
        HM_ASSERT_EQ(hm_clock_index_for_host_us(&s, h, &round_trip), HM_OK);
        HM_ASSERT(round_trip == idx || round_trip == idx + 1u || round_trip + 1u == idx);
        /* And it agrees with the truth to well inside a camera frame. */
        HM_ASSERT_NEAR((double)(h - true_host_us(idx)), 0.0, 4000.0);
    }
}

HM_TEST(clock_without_a_fit_refuses_rather_than_guesses)
{
    hm_fit f;
    hm_clock_snapshot s;
    uint32_t idx = 0;

    hm_fit_init(&f, 0.0);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    HM_ASSERT(!(s.flags & HM_CLOCK_HAS_FIT));
    HM_ASSERT_EQ(hm_clock_to_host_us(&s, 100), HM_TIME_UNKNOWN);
    HM_ASSERT_EQ(hm_clock_index_for_host_us(&s, T0, &idx), HM_ERR_NO_FIT);
    HM_ASSERT_EQ(hm_clock_uncertainty_us(&s, 100), UINT32_MAX);
    HM_ASSERT(!hm_clock_meets_budget(&s, 100, 1000000u));
}

HM_TEST(clock_marks_a_fit_stale_when_live_frames_stop)
{
    hm_fit f;
    hm_clock_snapshot s;
    hm_time_us last;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 4000, true);
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
    last = s.last_observation_us;

    hm_fit_snapshot(&f, last + 1000, &s);
    HM_ASSERT(!(s.flags & HM_CLOCK_STALE));
    hm_fit_snapshot(&f, last + HM_FIT_STALE_AFTER_US + 1, &s);
    HM_ASSERT(s.flags & HM_CLOCK_STALE);
}

/*
 * The hull is bounded, so a pathological session that puts every observation on
 * the lower hull must still terminate with a sane fit — and must account for
 * the bias that dropping vertices introduces rather than hiding it.
 */
HM_TEST(clock_bounded_hull_survives_an_all_vertices_session)
{
    hm_fit f;
    hm_clock_snapshot s;
    uint32_t index = 0;

    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);

    /* Monotonically decreasing delay makes the point set convex from below, so
     * every single observation is a hull vertex. */
    for (uint32_t k = 0; k < 3000u; ++k) {
        double frac = 1.0 - (double)k / 3000.0;
        hm_time_us d = (hm_time_us)(200000.0 * frac * frac);
        hm_fit_observe(&f, index, true_host_us(index) + d);
        index += 32u;
    }
    hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);

    HM_ASSERT(s.flags & HM_CLOCK_HAS_FIT);
    HM_ASSERT(f.hull_dropped > 0);
    HM_ASSERT(f.hull_n <= HM_FIT_HULL_MAX);
    HM_ASSERT(s.fitted_rate_hz > HM_RATE_PLAUSIBLE_MIN_HZ);
    HM_ASSERT(s.fitted_rate_hz < HM_RATE_PLAUSIBLE_MAX_HZ);
}

/* Out-of-order live frames are excluded from the fit rather than corrupting the
 * incremental hull; the consumer still receives the sample. */
HM_TEST(clock_ignores_out_of_order_observations)
{
    hm_fit f;
    hm_clock_snapshot s;
    hm_fit_init(&f, 0.0);
    hm_fit_begin_stream(&f, 1);
    (void)feed_session(&f, 1000, true);
    {
        int32_t before = 0;
        hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
        before = s.observations;
        hm_fit_observe(&f, 10, T0); /* far behind the head */
        hm_fit_snapshot(&f, HM_TIME_UNKNOWN, &s);
        HM_ASSERT_EQ(s.observations, before);
    }
}

HM_TEST_MAIN()
