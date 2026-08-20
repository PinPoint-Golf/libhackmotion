/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_defaults.c — the library's opinions, in one readable place.
 *
 * Every value here is either measured in docs/specification.md or recommended
 * in docs/api-request.md, and each carries the section that justifies it.  A
 * reviewer should be able to check this file against those two documents
 * without opening anything else.
 */
#include "hackmotion/session.h"
#include "hackmotion/history.h"

#include <string.h>

hm_session_policy hm_session_policy_default(void)
{
    hm_session_policy p;
    memset(&p, 0, sizeof(p));

    /* §9.2 — measured: a 30 s poll survived 7 minutes where a silent
     * connection, streaming or not, died at exactly 5.0. */
    p.keepalive_period_us = HM_KEEPALIVE_PERIOD_US;

    /* §8.2 — CLIENT policy.  The device returned and applied a result after
     * 15.6 s; it was the vendor's application that rejected it.  6 s is long
     * enough for an unhurried raise and short enough that the motion is still
     * one motion. */
    p.calibration_raise_limit_us = (hm_time_us)6 * 1000 * 1000;
    p.calibration_result_timeout_us = (hm_time_us)3 * 1000 * 1000;

    p.bringup_timeout_us = (hm_time_us)10 * 1000 * 1000;

    /* §6.1 — the first 0x90 arrives within 50-80 ms, so 3 s is two orders of
     * margin and anything near it means the start was not accepted. */
    p.stream_start_timeout_us = (hm_time_us)3 * 1000 * 1000;

    /* §9.2 — 300 s idle shutdown against a 30 s poll: at 120 s several polls
     * have been missed and the deadline is in view with time left to act. */
    p.keepalive_alarm_us = (hm_time_us)120 * 1000 * 1000;

    /* Matches the fit's own staleness threshold.  ⚠ Suppressed inside a history
     * bracket, where live delivery is suspended by design (§10.1). */
    p.live_gap_alarm_us = (hm_time_us)3 * 1000 * 1000;

    /* Accumulate rather than emit per sample: a burst of clipping would
     * otherwise drown the event queue exactly when it matters (§6.4). */
    p.pinned_report_period_us = (hm_time_us)1000 * 1000;

    /* §7.6 — 4.5 s against a ~7.5 s buffer leaves usable margin. */
    p.history_pre_roll_us = HM_DEFAULT_PRE_ROLL_US;
    p.history_post_roll_us = HM_DEFAULT_POST_ROLL_US;

    /* §10 — the measured drift against an external physical reference: 2.2 ms
     * per second of session, i.e. 2200 µs/s.  Not the residual spread, which
     * cannot see it. */
    p.accuracy_drift_us_per_s = HM_ACCURACY_DRIFT_US_PER_S_DEFAULT;

    /* Five camera frames at 240 fps: enough headroom that ordinary BLE jitter
     * does not cry wolf, tight enough to notice a link going bad. */
    p.residual_alarm_us = 20000u;
    p.clock_event_period_us = (hm_time_us)1000 * 1000;

    /* ⚠ Off: identifiers stay out of the wire log unless explicitly asked. */
    p.record_identifiers = false;

    return p;
}

hm_session_config hm_session_config_default(void)
{
    hm_session_config c;
    memset(&c, 0, sizeof(c));
    c.stream_config = hm_stream_config_default();
    c.policy = hm_session_policy_default();
    return c;
}

hm_history_request hm_history_request_around(const struct hm_session_policy *policy,
                                             hm_time_us event_host_us)
{
    hm_history_request r;
    hm_time_us pre = HM_DEFAULT_PRE_ROLL_US;
    hm_time_us post = HM_DEFAULT_POST_ROLL_US;

    memset(&r, 0, sizeof(r));

    /*
     * The policy is the point of the parameter: a field that nothing can read
     * is a field that does not exist.  NULL yields §7.6's recommended sizing.
     */
    if (policy != NULL) {
        if (policy->history_pre_roll_us > 0) {
            pre = policy->history_pre_roll_us;
        }
        if (policy->history_post_roll_us > 0) {
            post = policy->history_post_roll_us;
        }
    }

    r.window.start_us = event_host_us - pre;
    r.window.end_us = event_host_us + post;

    /*
     * §7.4 — retrieval takes about as long as the requested window spans, so a
     * 4.5 s window is ~4.5 s of pull.  Twice that plus a second of slack keeps
     * a stalled pull from becoming an unbounded wait without cutting off a
     * healthy one.
     */
    r.deadline_us = r.window.end_us + 2 * (pre + post) + (hm_time_us)1000 * 1000;

    r.refill_gaps = true;
    r.max_attempts = 3u;
    r.alignment_budget_us = 0u; /* no gate by default — the consumer opts in */
    return r;
}
