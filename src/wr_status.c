/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "wrist/types.h"
#include "wrist/event.h"
#include "wrist/history.h"
#include "wrist/sample.h"

#include <stdio.h>
#include <string.h>

const char *wr_status_str(wr_status status)
{
    switch (status) {
        case WR_OK:                   return "ok";
        case WR_PENDING:              return "pending";
        case WR_DONE:                 return "done";
        case WR_ERR_INVALID_ARG:      return "invalid argument";
        case WR_ERR_INVALID_STATE:    return "invalid state";
        case WR_ERR_NO_MEMORY:        return "out of memory";
        case WR_ERR_BUFFER_TOO_SMALL: return "buffer too small";
        case WR_ERR_NOT_SUPPORTED:    return "not supported";
        case WR_ERR_TRUNCATED:        return "truncated frame";
        case WR_ERR_MALFORMED:        return "malformed frame";
        case WR_ERR_UNKNOWN_MESSAGE:  return "unknown message id";
        case WR_ERR_NOT_ALLOWED:      return "command not on the allowlist";
        case WR_ERR_TIMEOUT:          return "timed out";
        case WR_ERR_CANCELLED:        return "cancelled";
        case WR_ERR_LINK_DOWN:        return "link down";
        case WR_ERR_MTU_TOO_SMALL:    return "negotiated MTU below 96";
        case WR_ERR_NO_STREAM:        return "no stream open";
        case WR_ERR_DEVICE_ERROR:     return "device reported an error";
        case WR_ERR_NO_FIT:           return "no clock fit yet";
        case WR_ERR_EVICTED:          return "range evicted from the device buffer";
        case WR_ERR_BUSY:             return "a retrieval is already in flight";
    }
    return "unknown status";
}

const char *wr_event_type_name(wr_event_type type)
{
    switch (type) {
        case WR_EV_NONE:                  return "none";
        case WR_EV_LINK_UP:               return "link_up";
        case WR_EV_LINK_DOWN:             return "link_down";
        case WR_EV_MTU_REJECTED:          return "mtu_rejected";
        case WR_EV_READY:                 return "ready";
        case WR_EV_DEVICE_INFO:           return "device_info";
        case WR_EV_BATTERY:               return "battery";
        case WR_EV_IDENTITY:              return "identity";
        case WR_EV_STREAM_STARTED:        return "stream_started";
        case WR_EV_STREAM_STOPPED:        return "stream_stopped";
        case WR_EV_STREAM_RESTARTED:      return "stream_restarted";
        case WR_EV_CALIBRATION_PHASE:     return "calibration_phase";
        case WR_EV_CALIBRATION_PRESENCE:  return "calibration_presence";
        case WR_EV_HISTORY_STARTED:       return "history_started";
        case WR_EV_HISTORY_PROGRESS:      return "history_progress";
        case WR_EV_HISTORY_READY:         return "history_ready";
        case WR_EV_HISTORY_BLIND_SPAN:    return "history_blind_span";
        case WR_EV_HISTORY_EVICTION_RISK: return "history_eviction_risk";
        case WR_EV_CLOCK_UPDATED:         return "clock_updated";
        case WR_EV_CLOCK_DEGRADED:        return "clock_degraded";
        case WR_EV_BUTTON:                return "button";
        case WR_EV_DEVICE_ERROR:          return "device_error";
        case WR_EV_UNKNOWN_MESSAGE:       return "unknown_message";
        case WR_EV_PINNED_SAMPLES:        return "pinned_samples";
        case WR_EV_WARNING:               return "warning";
        case WR_EVENT_TYPE_COUNT:         break;
    }
    return "invalid";
}

const char *wr_warning_code_name(wr_warning_code code)
{
    switch (code) {
        case WR_WARN_NONE:                return "none";
        case WR_WARN_SHORT_FRAME:         return "short_frame";
        case WR_WARN_TRAILING_BYTES:      return "trailing_bytes";
        case WR_WARN_UNEXPECTED_RECORD_COUNT: return "unexpected_record_count";
        case WR_WARN_QUAT_NORM:           return "quaternion_norm";
        case WR_WARN_INDEX_REGRESSION:    return "index_regression";
        case WR_WARN_TICK_PREDICTION_MARGIN: return "tick_prediction_margin";
        case WR_WARN_KEEPALIVE_LATE:      return "keepalive_late";
        case WR_WARN_LIVE_GAP:            return "live_gap";
        case WR_WARN_STREAM_START_TIMEOUT: return "stream_start_timeout";
        case WR_WARN_STREAM_STOP_TIMEOUT:  return "stream_stop_timeout";
        case WR_WARN_HISTORY_HOLED:       return "history_holed";
        case WR_WARN_HISTORY_SHORT:       return "history_short";
        case WR_WARN_HISTORY_OUT_OF_RANGE: return "history_out_of_range";
        case WR_WARN_HISTORY_DEPTH_CONFLICT: return "history_depth_conflict";
        case WR_WARN_HOST_CLOCK_REGRESSION: return "host_clock_regression";
        case WR_WARN_NONSTANDARD_CONFIG:  return "nonstandard_config";
        case WR_WARN_LEGACY_STREAM:       return "legacy_stream";
        case WR_WARN_MTU_UNKNOWN:         return "mtu_unknown";
        case WR_WARN_PRESENCE_NOT_MEASURED: return "presence_not_measured";
        case WR_WARN_CALIBRATION_INDETERMINATE: return "calibration_indeterminate";
        case WR_WARN_CALIBRATION_ABSENT:  return "calibration_absent";
        case WR_WARN_CALIBRATION_UNSOLICITED: return "calibration_unsolicited";
        case WR_WARN_CALIBRATION_STATUS_FORM: return "calibration_status_form";
        case WR_WARN_SENSOR_COUNT_UNSUPPORTED: return "sensor_count_unsupported";
        case WR_WARN_UNVERIFIED_PRODUCT:  return "unverified_product";
        case WR_WARN_CODE_COUNT:          break;
    }
    return "invalid";
}

const char *wr_calibration_phase_name(wr_calibration_phase phase)
{
    switch (phase) {
        case WR_CALP_IDLE:             return "idle";
        case WR_CALP_AWAIT_HORIZONTAL: return "await_horizontal";
        case WR_CALP_MARKING_POSE0:    return "marking_pose0";
        case WR_CALP_OBSERVING_RAISE:  return "observing_raise";
        case WR_CALP_MARKING_POSE1:    return "marking_pose1";
        case WR_CALP_APPLYING:         return "applying";
        case WR_CALP_VERIFYING:        return "verifying";
        case WR_CALP_COMPLETE:         return "complete";
        case WR_CALP_ABORTED:          return "aborted";
        case WR_CALIBRATION_PHASE_COUNT: break;
    }
    return "invalid";
}

const char *wr_link_down_cause_name(wr_link_down_cause cause)
{
    switch (cause) {
        case WR_LINK_DOWN_UNKNOWN:             return "unknown";
        case WR_LINK_DOWN_LOCAL_REQUEST:       return "local_request";
        case WR_LINK_DOWN_SUPERVISION_TIMEOUT: return "supervision_timeout";
        case WR_LINK_DOWN_REMOTE_CLOSED:       return "remote_closed";
        case WR_LINK_DOWN_TRANSPORT_ERROR:     return "transport_error";
        case WR_LINK_DOWN_ADAPTER_GONE:        return "adapter_gone";
        case WR_LINK_DOWN_CONNECTION_TAKEN:    return "connection_taken";
    }
    return "invalid";
}

const char *wr_recovery_advice_name(wr_recovery_advice advice)
{
    switch (advice) {
        case WR_RECOVER_UNKNOWN:               return "unknown";
        case WR_RECOVER_RECONNECT_WITH_BACKOFF:return "reconnect_with_backoff";
        case WR_RECOVER_NEEDS_BUTTON_PRESS:    return "needs_button_press";
        case WR_RECOVER_NEEDS_OTHER_APP_CLOSED:return "needs_other_app_closed";
        case WR_RECOVER_DO_NOT_RETRY:          return "do_not_retry";
    }
    return "invalid";
}

const char *wr_history_status_name(wr_history_status status)
{
    switch (status) {
        case WR_HIST_COMPLETE:           return "complete";
        case WR_HIST_SHORT:              return "short";
        case WR_HIST_HOLED:              return "holed";
        case WR_HIST_TIMED_OUT:          return "timed_out";
        case WR_HIST_CANCELLED:          return "cancelled";
        case WR_HIST_REFUSED_ALIGNMENT:  return "refused_alignment";
        case WR_HIST_EVICTED:            return "evicted";
        case WR_HIST_NO_STREAM:          return "no_stream";
        case WR_HIST_LINK_LOST:          return "link_lost";
        case WR_HIST_NOT_ALIGNABLE:      return "not_alignable";
        case WR_HIST_ERROR:              return "error";
        case WR_HISTORY_STATUS_COUNT:    break;
    }
    return "invalid";
}

const char *wr_gap_kind_name(wr_gap_kind kind)
{
    switch (kind) {
        case WR_GAP_NOT_RECORDED:   return "not_recorded";
        case WR_GAP_NOT_DELIVERED:  return "not_delivered";
        case WR_GAP_FIT_BLIND:      return "fit_blind";
    }
    return "invalid";
}

const char *wr_unit_name(wr_unit unit)
{
    switch (unit) {
        case WR_UNIT_LOWER_ARM: return "lower_arm";
        case WR_UNIT_PALM:      return "palm";
        case WR_UNIT_COUNT:     break;
    }
    return "invalid";
}

const char *wr_channel_name(wr_channel ch)
{
    switch (ch) {
        case WR_CH_ACCEL_X: return "accel_x";
        case WR_CH_ACCEL_Y: return "accel_y";
        case WR_CH_ACCEL_Z: return "accel_z";
        case WR_CH_GYRO_X:  return "gyro_x";
        case WR_CH_GYRO_Y:  return "gyro_y";
        case WR_CH_GYRO_Z:  return "gyro_z";
        case WR_CHANNEL_COUNT: break;
    }
    return "invalid";
}

const char *wr_clock_flag_name(wr_clock_flag flag)
{
    switch (flag) {
        case WR_CLOCK_HAS_FIT:            return "has_fit";
        case WR_CLOCK_RATE_POOLED:        return "rate_pooled";
        case WR_CLOCK_DEGENERATE:         return "degenerate";
        case WR_CLOCK_RATE_IMPLAUSIBLE:   return "rate_implausible";
        case WR_CLOCK_STALE:              return "stale";
        case WR_CLOCK_BLIND:              return "blind";
        case WR_CLOCK_EXTERNAL_CORRECTION:return "external_correction";
        case WR_CLOCK_SHORT_BASELINE:     return "short_baseline";
    }
    return "invalid";
}

/* ------------------------------------------------------------------------ */
/* Event helpers                                                             */
/* ------------------------------------------------------------------------ */

bool wr_event_is_sensitive(const wr_event *ev)
{
    if (ev == NULL) {
        return false;
    }
    /*
     * api-request §2.13: exactly the events that carry a MAC or a serial.
     *
     * ⚠ WR_EV_DEVICE_INFO CARRIES BOTH, ON EVERY NORMAL BRING-UP, and it was
     * missing here (implementation-review I4).  It ships the whole
     * `wr_device_info` by value — `enter_ready()` does `ev.u.device_info =
     * s->info` — and `maybe_enter_ready()` only reaches READY once
     * BRINGUP_ALL_INFO is satisfied, which includes WR_INFO_MAC and
     * WR_INFO_SERIAL.  So the identifiers are in that payload by the time it is
     * emitted, whatever its name suggests.
     *
     * ⚠ It looked safe because wr_event_format() happens not to PRINT those
     * fields for this type, so the "format every event and assert nothing leaks"
     * test passed while the other documented mechanism — the one design §9.2
     * tells a consumer to filter their log sink, their telemetry and a binding's
     * serialisation on — was wrong.  Two safeguards, one tested.
     *
     * The rule for anything added here: ask what the PRODUCER copies into the
     * union, not what the formatter chooses to render.
     */
    return ev->type == (uint16_t)WR_EV_IDENTITY || ev->type == (uint16_t)WR_EV_DEVICE_INFO;
}

int wr_event_format(const wr_event *ev, char *out, size_t size, bool include_identifiers)
{
    if (ev == NULL || out == NULL || size == 0) {
        return -1;
    }

    const char *name = wr_event_type_name((wr_event_type)ev->type);

    switch ((wr_event_type)ev->type) {
        case WR_EV_IDENTITY:
            if (!include_identifiers) {
                return snprintf(out, size, "%s serial=<redacted> mac=<redacted>", name);
            }
            return snprintf(out, size, "%s serial=%s mac=%s", name,
                            ev->u.device_info.serial, ev->u.device_info.mac);

        case WR_EV_LINK_DOWN:
            return snprintf(out, size, "%s cause=%s advice=%s connected_for=%lldus",
                            name,
                            wr_link_down_cause_name((wr_link_down_cause)ev->u.link_down.cause),
                            wr_recovery_advice_name((wr_recovery_advice)ev->u.link_down.advice),
                            (long long)ev->u.link_down.connected_for_us);

        case WR_EV_MTU_REJECTED:
            return snprintf(out, size, "%s negotiated=%d required=%d", name, ev->u.mtu,
                            WR_MIN_ATT_MTU);

        case WR_EV_BATTERY:
            return snprintf(out, size, "%s percent=%u status=%u", name,
                            (unsigned)ev->u.battery.percent,
                            (unsigned)ev->u.battery.status_undecoded);

        case WR_EV_DEVICE_INFO:
            return snprintf(out, size, "%s hw=%u.%u proto=%u.%u fw=%u.%u product=%u sensors=%u",
                            name, ev->u.device_info.hardware_major,
                            ev->u.device_info.hardware_minor, ev->u.device_info.protocol_major,
                            ev->u.device_info.protocol_minor, ev->u.device_info.firmware_major,
                            ev->u.device_info.firmware_minor, ev->u.device_info.product_id,
                            ev->u.device_info.sensor_count);

        case WR_EV_CALIBRATION_PHASE:
            return snprintf(out, size, "%s phase=%s from=%s reason=%u", name,
                            wr_calibration_phase_name(
                                (wr_calibration_phase)ev->u.calibration_phase.phase),
                            wr_calibration_phase_name(
                                (wr_calibration_phase)ev->u.calibration_phase.previous_phase),
                            (unsigned)ev->u.calibration_phase.abort_reason);

        /* ⚠ The angle never appears without the run behind it: a mean without a
         * spread is an estimate without evidence, and this one is a PRESENCE
         * check that inverts under a deliberately bad calibration (§8.2). */
        case WR_EV_CALIBRATION_PRESENCE:
            return snprintf(out, size,
                            "%s angle=%.2fdeg n=%u spread=%.2f/%.2fdeg (presence, NOT quality)",
                            name, (double)ev->u.calibration_presence.relative_angle_deg,
                            (unsigned)ev->u.calibration_presence.samples_used,
                            (double)ev->u.calibration_presence.pose_spread_deg[0],
                            (double)ev->u.calibration_presence.pose_spread_deg[1]);

        case WR_EV_DEVICE_ERROR:
            return snprintf(out, size, "%s code=0x%02x (means nothing specific: 7 causes, "
                                       "1 code)", name, ev->u.device_error.code);

        case WR_EV_UNKNOWN_MESSAGE:
            return snprintf(out, size, "%s id=0x%02x len=%u (logged and ignored)", name,
                            ev->u.unknown_message.message_id,
                            (unsigned)ev->u.unknown_message.length);

        case WR_EV_WARNING:
            return snprintf(out, size, "%s %s detail=%d/%.6g", name,
                            wr_warning_code_name((wr_warning_code)ev->u.warning.code),
                            ev->u.warning.detail_i32, ev->u.warning.detail_f64);

        case WR_EV_CLOCK_UPDATED:
        case WR_EV_CLOCK_DEGRADED:
            return snprintf(out, size, "%s rate=%.4fHz n=%d p90=%uus max=%uus flags=0x%02x",
                            name, ev->u.clock.fitted_rate_hz, ev->u.clock.observations,
                            (unsigned)ev->u.clock.residual_p90_us,
                            (unsigned)ev->u.clock.residual_max_us,
                            (unsigned)ev->u.clock.flags);

        /* ⚠ The fraction never appears without the count beside it: coverage
         * alone cannot tell "dense over half the range" from "half-dense over
         * all of it", and at §7.3's 16-58 % that is the whole question. */
        case WR_EV_HISTORY_STARTED:
        case WR_EV_HISTORY_PROGRESS:
        case WR_EV_HISTORY_READY:
            return snprintf(out, size, "%s id=%llu records=%u coverage=%.3f elapsed=%lldus", name,
                            (unsigned long long)ev->u.history_progress.request_id,
                            (unsigned)ev->u.history_progress.records_delivered,
                            (double)ev->u.history_progress.fraction,
                            (long long)ev->u.history_progress.elapsed_us);

        /* ⚠ Live delivery was suspended here AND the device was not recording
         * behind the silence (§7.5) — the span is lost data, not just latency. */
        case WR_EV_HISTORY_BLIND_SPAN:
            return snprintf(out, size, "%s id=%llu span=%lldus (not recorded)", name,
                            (unsigned long long)ev->u.history_blind_span.request_id,
                            (long long)(ev->u.history_blind_span.span.end_us -
                                        ev->u.history_blind_span.span.start_us));

        /* ⚠ BOTH FIGURES OR NEITHER (§8.6).  A margin with no wait beside it
         * cannot be acted on, and a negative margin means the window's oldest
         * sample is estimated to be gone already. */
        case WR_EV_HISTORY_EVICTION_RISK:
            return snprintf(out, size, "%s id=%llu queued=%lldus evicted_in=%lldus (estimate)",
                            name,
                            (unsigned long long)ev->u.history_eviction_risk.request_id,
                            (long long)ev->u.history_eviction_risk.queued_for_us,
                            (long long)ev->u.history_eviction_risk.estimated_eviction_in_us);

        case WR_EV_NONE:
        case WR_EV_LINK_UP:
        case WR_EV_READY:
        case WR_EV_STREAM_STARTED:
        case WR_EV_STREAM_STOPPED:
        case WR_EV_STREAM_RESTARTED:
        case WR_EV_BUTTON:
        case WR_EV_PINNED_SAMPLES:
        case WR_EVENT_TYPE_COUNT:
        default:
            break;
    }
    return snprintf(out, size, "%s", name);
}
