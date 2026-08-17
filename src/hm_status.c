/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
#include "hackmotion/types.h"
#include "hackmotion/event.h"
#include "hackmotion/history.h"
#include "hackmotion/sample.h"

#include <stdio.h>
#include <string.h>

const char *hm_status_str(hm_status status)
{
    switch (status) {
        case HM_OK:                   return "ok";
        case HM_PENDING:              return "pending";
        case HM_DONE:                 return "done";
        case HM_ERR_INVALID_ARG:      return "invalid argument";
        case HM_ERR_INVALID_STATE:    return "invalid state";
        case HM_ERR_NO_MEMORY:        return "out of memory";
        case HM_ERR_BUFFER_TOO_SMALL: return "buffer too small";
        case HM_ERR_NOT_SUPPORTED:    return "not supported";
        case HM_ERR_TRUNCATED:        return "truncated frame";
        case HM_ERR_MALFORMED:        return "malformed frame";
        case HM_ERR_UNKNOWN_MESSAGE:  return "unknown message id";
        case HM_ERR_NOT_ALLOWED:      return "command not on the allowlist";
        case HM_ERR_TIMEOUT:          return "timed out";
        case HM_ERR_CANCELLED:        return "cancelled";
        case HM_ERR_LINK_DOWN:        return "link down";
        case HM_ERR_MTU_TOO_SMALL:    return "negotiated MTU below 96";
        case HM_ERR_NO_STREAM:        return "no stream open";
        case HM_ERR_DEVICE_ERROR:     return "device reported an error";
        case HM_ERR_NO_FIT:           return "no clock fit yet";
        case HM_ERR_EVICTED:          return "range evicted from the device buffer";
        case HM_ERR_BUSY:             return "a retrieval is already in flight";
    }
    return "unknown status";
}

const char *hm_event_type_name(hm_event_type type)
{
    switch (type) {
        case HM_EV_NONE:                  return "none";
        case HM_EV_LINK_UP:               return "link_up";
        case HM_EV_LINK_DOWN:             return "link_down";
        case HM_EV_MTU_REJECTED:          return "mtu_rejected";
        case HM_EV_READY:                 return "ready";
        case HM_EV_DEVICE_INFO:           return "device_info";
        case HM_EV_BATTERY:               return "battery";
        case HM_EV_IDENTITY:              return "identity";
        case HM_EV_STREAM_STARTED:        return "stream_started";
        case HM_EV_STREAM_STOPPED:        return "stream_stopped";
        case HM_EV_STREAM_RESTARTED:      return "stream_restarted";
        case HM_EV_CALIBRATION_PHASE:     return "calibration_phase";
        case HM_EV_CALIBRATION_PRESENCE:  return "calibration_presence";
        case HM_EV_HISTORY_STARTED:       return "history_started";
        case HM_EV_HISTORY_PROGRESS:      return "history_progress";
        case HM_EV_HISTORY_READY:         return "history_ready";
        case HM_EV_HISTORY_BLIND_SPAN:    return "history_blind_span";
        case HM_EV_HISTORY_EVICTION_RISK: return "history_eviction_risk";
        case HM_EV_CLOCK_UPDATED:         return "clock_updated";
        case HM_EV_CLOCK_DEGRADED:        return "clock_degraded";
        case HM_EV_BUTTON:                return "button";
        case HM_EV_DEVICE_ERROR:          return "device_error";
        case HM_EV_UNKNOWN_MESSAGE:       return "unknown_message";
        case HM_EV_PINNED_SAMPLES:        return "pinned_samples";
        case HM_EV_WARNING:               return "warning";
        case HM_EVENT_TYPE_COUNT:         break;
    }
    return "invalid";
}

const char *hm_warning_code_name(hm_warning_code code)
{
    switch (code) {
        case HM_WARN_NONE:                return "none";
        case HM_WARN_SHORT_FRAME:         return "short_frame";
        case HM_WARN_TRAILING_BYTES:      return "trailing_bytes";
        case HM_WARN_UNEXPECTED_RECORD_COUNT: return "unexpected_record_count";
        case HM_WARN_QUAT_NORM:           return "quaternion_norm";
        case HM_WARN_INDEX_REGRESSION:    return "index_regression";
        case HM_WARN_TICK_PREDICTION_MARGIN: return "tick_prediction_margin";
        case HM_WARN_KEEPALIVE_LATE:      return "keepalive_late";
        case HM_WARN_LIVE_GAP:            return "live_gap";
        case HM_WARN_STREAM_START_TIMEOUT: return "stream_start_timeout";
        case HM_WARN_STREAM_STOP_TIMEOUT:  return "stream_stop_timeout";
        case HM_WARN_HISTORY_HOLED:       return "history_holed";
        case HM_WARN_HISTORY_SHORT:       return "history_short";
        case HM_WARN_HISTORY_OUT_OF_RANGE: return "history_out_of_range";
        case HM_WARN_HISTORY_DEPTH_CONFLICT: return "history_depth_conflict";
        case HM_WARN_HOST_CLOCK_REGRESSION: return "host_clock_regression";
        case HM_WARN_NONSTANDARD_CONFIG:  return "nonstandard_config";
        case HM_WARN_LEGACY_STREAM:       return "legacy_stream";
        case HM_WARN_MTU_UNKNOWN:         return "mtu_unknown";
        case HM_WARN_PRESENCE_NOT_MEASURED: return "presence_not_measured";
        case HM_WARN_CALIBRATION_INDETERMINATE: return "calibration_indeterminate";
        case HM_WARN_CALIBRATION_ABSENT:  return "calibration_absent";
        case HM_WARN_CALIBRATION_UNSOLICITED: return "calibration_unsolicited";
        case HM_WARN_CALIBRATION_STATUS_FORM: return "calibration_status_form";
        case HM_WARN_CODE_COUNT:          break;
    }
    return "invalid";
}

const char *hm_calibration_phase_name(hm_calibration_phase phase)
{
    switch (phase) {
        case HM_CALP_IDLE:             return "idle";
        case HM_CALP_AWAIT_HORIZONTAL: return "await_horizontal";
        case HM_CALP_MARKING_POSE0:    return "marking_pose0";
        case HM_CALP_OBSERVING_RAISE:  return "observing_raise";
        case HM_CALP_MARKING_POSE1:    return "marking_pose1";
        case HM_CALP_APPLYING:         return "applying";
        case HM_CALP_VERIFYING:        return "verifying";
        case HM_CALP_COMPLETE:         return "complete";
        case HM_CALP_ABORTED:          return "aborted";
        case HM_CALIBRATION_PHASE_COUNT: break;
    }
    return "invalid";
}

const char *hm_link_down_cause_name(hm_link_down_cause cause)
{
    switch (cause) {
        case HM_LINK_DOWN_UNKNOWN:             return "unknown";
        case HM_LINK_DOWN_LOCAL_REQUEST:       return "local_request";
        case HM_LINK_DOWN_SUPERVISION_TIMEOUT: return "supervision_timeout";
        case HM_LINK_DOWN_REMOTE_CLOSED:       return "remote_closed";
        case HM_LINK_DOWN_TRANSPORT_ERROR:     return "transport_error";
        case HM_LINK_DOWN_ADAPTER_GONE:        return "adapter_gone";
        case HM_LINK_DOWN_CONNECTION_TAKEN:    return "connection_taken";
    }
    return "invalid";
}

const char *hm_recovery_advice_name(hm_recovery_advice advice)
{
    switch (advice) {
        case HM_RECOVER_UNKNOWN:               return "unknown";
        case HM_RECOVER_RECONNECT_WITH_BACKOFF:return "reconnect_with_backoff";
        case HM_RECOVER_NEEDS_BUTTON_PRESS:    return "needs_button_press";
        case HM_RECOVER_NEEDS_OTHER_APP_CLOSED:return "needs_other_app_closed";
        case HM_RECOVER_DO_NOT_RETRY:          return "do_not_retry";
    }
    return "invalid";
}

const char *hm_history_status_name(hm_history_status status)
{
    switch (status) {
        case HM_HIST_COMPLETE:           return "complete";
        case HM_HIST_SHORT:              return "short";
        case HM_HIST_HOLED:              return "holed";
        case HM_HIST_TIMED_OUT:          return "timed_out";
        case HM_HIST_CANCELLED:          return "cancelled";
        case HM_HIST_REFUSED_ALIGNMENT:  return "refused_alignment";
        case HM_HIST_EVICTED:            return "evicted";
        case HM_HIST_NO_STREAM:          return "no_stream";
        case HM_HIST_LINK_LOST:          return "link_lost";
        case HM_HIST_NOT_ALIGNABLE:      return "not_alignable";
        case HM_HIST_ERROR:              return "error";
        case HM_HISTORY_STATUS_COUNT:    break;
    }
    return "invalid";
}

const char *hm_gap_kind_name(hm_gap_kind kind)
{
    switch (kind) {
        case HM_GAP_NOT_RECORDED:   return "not_recorded";
        case HM_GAP_NOT_DELIVERED:  return "not_delivered";
        case HM_GAP_FIT_BLIND:      return "fit_blind";
    }
    return "invalid";
}

const char *hm_unit_name(hm_unit unit)
{
    switch (unit) {
        case HM_UNIT_LOWER_ARM: return "lower_arm";
        case HM_UNIT_PALM:      return "palm";
        case HM_UNIT_COUNT:     break;
    }
    return "invalid";
}

const char *hm_channel_name(hm_channel ch)
{
    switch (ch) {
        case HM_CH_ACCEL_X: return "accel_x";
        case HM_CH_ACCEL_Y: return "accel_y";
        case HM_CH_ACCEL_Z: return "accel_z";
        case HM_CH_GYRO_X:  return "gyro_x";
        case HM_CH_GYRO_Y:  return "gyro_y";
        case HM_CH_GYRO_Z:  return "gyro_z";
        case HM_CHANNEL_COUNT: break;
    }
    return "invalid";
}

const char *hm_clock_flag_name(hm_clock_flag flag)
{
    switch (flag) {
        case HM_CLOCK_HAS_FIT:            return "has_fit";
        case HM_CLOCK_RATE_POOLED:        return "rate_pooled";
        case HM_CLOCK_DEGENERATE:         return "degenerate";
        case HM_CLOCK_RATE_IMPLAUSIBLE:   return "rate_implausible";
        case HM_CLOCK_STALE:              return "stale";
        case HM_CLOCK_BLIND:              return "blind";
        case HM_CLOCK_EXTERNAL_CORRECTION:return "external_correction";
        case HM_CLOCK_SHORT_BASELINE:     return "short_baseline";
    }
    return "invalid";
}

/* ------------------------------------------------------------------------ */
/* Event helpers                                                             */
/* ------------------------------------------------------------------------ */

bool hm_event_is_sensitive(const hm_event *ev)
{
    if (ev == NULL) {
        return false;
    }
    /*
     * api-request §2.13: exactly the events that carry a MAC or a serial.
     *
     * ⚠ HM_EV_DEVICE_INFO CARRIES BOTH, ON EVERY NORMAL BRING-UP, and it was
     * missing here (implementation-review I4).  It ships the whole
     * `hm_device_info` by value — `enter_ready()` does `ev.u.device_info =
     * s->info` — and `maybe_enter_ready()` only reaches READY once
     * BRINGUP_ALL_INFO is satisfied, which includes HM_INFO_MAC and
     * HM_INFO_SERIAL.  So the identifiers are in that payload by the time it is
     * emitted, whatever its name suggests.
     *
     * ⚠ It looked safe because hm_event_format() happens not to PRINT those
     * fields for this type, so the "format every event and assert nothing leaks"
     * test passed while the other documented mechanism — the one design §9.2
     * tells a consumer to filter their log sink, their telemetry and a binding's
     * serialisation on — was wrong.  Two safeguards, one tested.
     *
     * The rule for anything added here: ask what the PRODUCER copies into the
     * union, not what the formatter chooses to render.
     */
    return ev->type == (uint16_t)HM_EV_IDENTITY || ev->type == (uint16_t)HM_EV_DEVICE_INFO;
}

int hm_event_format(const hm_event *ev, char *out, size_t size, bool include_identifiers)
{
    if (ev == NULL || out == NULL || size == 0) {
        return -1;
    }

    const char *name = hm_event_type_name((hm_event_type)ev->type);

    switch ((hm_event_type)ev->type) {
        case HM_EV_IDENTITY:
            if (!include_identifiers) {
                return snprintf(out, size, "%s serial=<redacted> mac=<redacted>", name);
            }
            return snprintf(out, size, "%s serial=%s mac=%s", name,
                            ev->u.device_info.serial, ev->u.device_info.mac);

        case HM_EV_LINK_DOWN:
            return snprintf(out, size, "%s cause=%s advice=%s connected_for=%lldus",
                            name,
                            hm_link_down_cause_name((hm_link_down_cause)ev->u.link_down.cause),
                            hm_recovery_advice_name((hm_recovery_advice)ev->u.link_down.advice),
                            (long long)ev->u.link_down.connected_for_us);

        case HM_EV_MTU_REJECTED:
            return snprintf(out, size, "%s negotiated=%d required=%d", name, ev->u.mtu,
                            HM_MIN_ATT_MTU);

        case HM_EV_BATTERY:
            return snprintf(out, size, "%s percent=%u status=%u", name,
                            (unsigned)ev->u.battery.percent,
                            (unsigned)ev->u.battery.status_undecoded);

        case HM_EV_DEVICE_INFO:
            return snprintf(out, size, "%s hw=%u.%u proto=%u.%u fw=%u.%u product=%u sensors=%u",
                            name, ev->u.device_info.hardware_major,
                            ev->u.device_info.hardware_minor, ev->u.device_info.protocol_major,
                            ev->u.device_info.protocol_minor, ev->u.device_info.firmware_major,
                            ev->u.device_info.firmware_minor, ev->u.device_info.product_id,
                            ev->u.device_info.sensor_count);

        case HM_EV_CALIBRATION_PHASE:
            return snprintf(out, size, "%s phase=%s from=%s reason=%u", name,
                            hm_calibration_phase_name(
                                (hm_calibration_phase)ev->u.calibration_phase.phase),
                            hm_calibration_phase_name(
                                (hm_calibration_phase)ev->u.calibration_phase.previous_phase),
                            (unsigned)ev->u.calibration_phase.abort_reason);

        /* ⚠ The angle never appears without the run behind it: a mean without a
         * spread is an estimate without evidence, and this one is a PRESENCE
         * check that inverts under a deliberately bad calibration (§8.2). */
        case HM_EV_CALIBRATION_PRESENCE:
            return snprintf(out, size,
                            "%s angle=%.2fdeg n=%u spread=%.2f/%.2fdeg (presence, NOT quality)",
                            name, (double)ev->u.calibration_presence.relative_angle_deg,
                            (unsigned)ev->u.calibration_presence.samples_used,
                            (double)ev->u.calibration_presence.pose_spread_deg[0],
                            (double)ev->u.calibration_presence.pose_spread_deg[1]);

        case HM_EV_DEVICE_ERROR:
            return snprintf(out, size, "%s code=0x%02x (means nothing specific: 7 causes, "
                                       "1 code)", name, ev->u.device_error.code);

        case HM_EV_UNKNOWN_MESSAGE:
            return snprintf(out, size, "%s id=0x%02x len=%u (logged and ignored)", name,
                            ev->u.unknown_message.message_id,
                            (unsigned)ev->u.unknown_message.length);

        case HM_EV_WARNING:
            return snprintf(out, size, "%s %s detail=%d/%.6g", name,
                            hm_warning_code_name((hm_warning_code)ev->u.warning.code),
                            ev->u.warning.detail_i32, ev->u.warning.detail_f64);

        case HM_EV_CLOCK_UPDATED:
        case HM_EV_CLOCK_DEGRADED:
            return snprintf(out, size, "%s rate=%.4fHz n=%d p90=%uus max=%uus flags=0x%02x",
                            name, ev->u.clock.fitted_rate_hz, ev->u.clock.observations,
                            (unsigned)ev->u.clock.residual_p90_us,
                            (unsigned)ev->u.clock.residual_max_us,
                            (unsigned)ev->u.clock.flags);

        /* ⚠ The fraction never appears without the count beside it: coverage
         * alone cannot tell "dense over half the range" from "half-dense over
         * all of it", and at §7.3's 16-58 % that is the whole question. */
        case HM_EV_HISTORY_STARTED:
        case HM_EV_HISTORY_PROGRESS:
        case HM_EV_HISTORY_READY:
            return snprintf(out, size, "%s id=%llu records=%u coverage=%.3f elapsed=%lldus", name,
                            (unsigned long long)ev->u.history_progress.request_id,
                            (unsigned)ev->u.history_progress.records_delivered,
                            (double)ev->u.history_progress.fraction,
                            (long long)ev->u.history_progress.elapsed_us);

        /* ⚠ Live delivery was suspended here AND the device was not recording
         * behind the silence (§7.5) — the span is lost data, not just latency. */
        case HM_EV_HISTORY_BLIND_SPAN:
            return snprintf(out, size, "%s id=%llu span=%lldus (not recorded)", name,
                            (unsigned long long)ev->u.history_blind_span.request_id,
                            (long long)(ev->u.history_blind_span.span.end_us -
                                        ev->u.history_blind_span.span.start_us));

        /* ⚠ BOTH FIGURES OR NEITHER (§8.6).  A margin with no wait beside it
         * cannot be acted on, and a negative margin means the window's oldest
         * sample is estimated to be gone already. */
        case HM_EV_HISTORY_EVICTION_RISK:
            return snprintf(out, size, "%s id=%llu queued=%lldus evicted_in=%lldus (estimate)",
                            name,
                            (unsigned long long)ev->u.history_eviction_risk.request_id,
                            (long long)ev->u.history_eviction_risk.queued_for_us,
                            (long long)ev->u.history_eviction_risk.estimated_eviction_in_us);

        case HM_EV_NONE:
        case HM_EV_LINK_UP:
        case HM_EV_READY:
        case HM_EV_STREAM_STARTED:
        case HM_EV_STREAM_STOPPED:
        case HM_EV_STREAM_RESTARTED:
        case HM_EV_BUTTON:
        case HM_EV_PINNED_SAMPLES:
        case HM_EVENT_TYPE_COUNT:
        default:
            break;
    }
    return snprintf(out, size, "%s", name);
}
