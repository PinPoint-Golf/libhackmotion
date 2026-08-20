/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_abi_table.c — every public struct's size, and every field's offset.
 *
 * See wr_abi_table.h for why wr_abi_check() is not enough on its own.
 *
 * ⚠ ADDING A FIELD TO A PUBLIC STRUCT MEANS ADDING A ROW HERE, and four things
 * check that you did.  They are listed with what each one CANNOT see, because a
 * completeness claim nobody has bounded is the shape this project has a rule
 * about:
 *
 *   1. The tiling self-check below — rows must start at 0, never overlap and
 *      reach the end, with no gap wider than the struct's own alignment.
 *      ⚠ BLIND to a dropped row whose bytes are indistinguishable from padding:
 *      remove `wr_sample.skew_us` (int32 at 20, followed by an 8-aligned field)
 *      and the four bytes it leaves behind are exactly the padding that would
 *      have been there anyway.  Confirmed by doing it.  No offset-based check
 *      can see that, because the information is not in the layout.
 *   2. sizeof here against ctypes.sizeof in tests/test_python_abi.py, for EVERY
 *      struct in this table rather than the nine wr_abi_check() covers.  A field
 *      added to a header changes the struct's size, so this is what actually
 *      catches a forgotten row.  ⚠ Blind to a field carved out of an existing
 *      `reserved` array, which does not change the size.
 *   3. The field-name sets, compared in BOTH directions by the same test.  A row
 *      here with no ctypes field, or a ctypes field with no row here, fails.
 *   4. wr_abi_check() at binding load, for the nine structs it knows.
 *
 * The one shape all four miss: a field carved out of a `reserved` slot, updated
 * in the header and in neither this table nor the binding.  It is narrow — the
 * reserved arrays are themselves rows here, so shrinking one changes that row's
 * size and trips (2) — and it is written down rather than engineered around.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "wrist/wrist.h"
#include "wrist/record.h"

#include "wr_abi_table.h"

typedef struct abi_field {
    const char *name;
    size_t      offset;
    size_t      size;
} abi_field;

typedef struct abi_struct {
    const char      *name;
    size_t           size;
    size_t           align;
    const abi_field *fields;
    size_t           field_count;
} abi_struct;

/* ⚠ sizeof on a member expression of a null pointer never dereferences it —
 * the operand of sizeof is unevaluated.  This is the standard idiom and is what
 * lets one macro cover scalars, arrays, nested structs and the union alike. */
#define F(type, member) \
    { #member, offsetof(type, member), sizeof(((type *)0)->member) }

#define S(type, fields) \
    { #type, sizeof(type), _Alignof(type), (fields), sizeof(fields) / sizeof((fields)[0]) }

/* ------------------------------------------------------------------------ */
/* types.h                                                                   */
/* ------------------------------------------------------------------------ */
static const abi_field f_uuid[] = { F(wr_uuid, bytes) };

static const abi_field f_index_range[] = {
    F(wr_index_range, first), F(wr_index_range, last),
};

static const abi_field f_time_range[] = {
    F(wr_time_range, start_us), F(wr_time_range, end_us),
};

static const abi_field f_allocator[] = {
    F(wr_allocator, alloc), F(wr_allocator, free), F(wr_allocator, ctx),
};

/* ------------------------------------------------------------------------ */
/* config.h                                                                  */
/* ------------------------------------------------------------------------ */
static const abi_field f_stream_config[] = {
    F(wr_stream_config, bits),
    F(wr_stream_config, legacy),
    F(wr_stream_config, reserved),
    F(wr_stream_config, justification),
};

/* ------------------------------------------------------------------------ */
/* device.h                                                                  */
/* ------------------------------------------------------------------------ */
static const abi_field f_device_info[] = {
    F(wr_device_info, hardware_major),
    F(wr_device_info, hardware_minor),
    F(wr_device_info, protocol_major),
    F(wr_device_info, protocol_minor),
    F(wr_device_info, firmware_major),
    F(wr_device_info, firmware_minor),
    F(wr_device_info, product_id),
    F(wr_device_info, sensor_count),
    F(wr_device_info, sensor_location),
    F(wr_device_info, battery_percent),
    F(wr_device_info, status_undecoded),
    F(wr_device_info, mac),
    F(wr_device_info, serial),
    F(wr_device_info, valid),
};

/* ------------------------------------------------------------------------ */
/* sample.h                                                                  */
/* ------------------------------------------------------------------------ */
static const abi_field f_unit_sample[] = {
    F(wr_unit_sample, q_world_to_body_raw),
    F(wr_unit_sample, linear_accel_raw),
    F(wr_unit_sample, gyro_raw),
    F(wr_unit_sample, ticks_raw),
    F(wr_unit_sample, has_ticks),
    F(wr_unit_sample, pinned_mask),
    F(wr_unit_sample, q_world_to_body),
    F(wr_unit_sample, linear_accel_mps2),
    F(wr_unit_sample, gyro_dps),
    F(wr_unit_sample, device_time_us),
};

static const abi_field f_sample[] = {
    F(wr_sample, stream_id),
    F(wr_sample, sample_index),
    F(wr_sample, sample_index_raw),
    F(wr_sample, source),
    F(wr_sample, calibration),
    F(wr_sample, flags),
    F(wr_sample, config_bits),
    F(wr_sample, reserved0),
    F(wr_sample, skew_us),
    F(wr_sample, host_time_us),
    F(wr_sample, host_recv_us),
    F(wr_sample, precision_us),
    F(wr_sample, uncertainty_us),
    F(wr_sample, lower_arm),
    F(wr_sample, palm),
};

static const abi_field f_pinned_counts[] = {
    F(wr_pinned_counts, n), F(wr_pinned_counts, total),
};

/* ------------------------------------------------------------------------ */
/* clock.h                                                                   */
/* ------------------------------------------------------------------------ */
static const abi_field f_clock_snapshot[] = {
    F(wr_clock_snapshot, stream_id),
    F(wr_clock_snapshot, flags),
    F(wr_clock_snapshot, observations),
    F(wr_clock_snapshot, anchor_index),
    F(wr_clock_snapshot, reserved0),
    F(wr_clock_snapshot, anchor_host_us),
    F(wr_clock_snapshot, slope_us_per_index),
    F(wr_clock_snapshot, fitted_rate_hz),
    F(wr_clock_snapshot, raw_fitted_rate_hz),
    F(wr_clock_snapshot, external_ppm),
    F(wr_clock_snapshot, offset_us),
    F(wr_clock_snapshot, span_us),
    F(wr_clock_snapshot, last_observation_us),
    F(wr_clock_snapshot, first_index),
    F(wr_clock_snapshot, last_index),
    F(wr_clock_snapshot, residual_median_us),
    F(wr_clock_snapshot, residual_p90_us),
    F(wr_clock_snapshot, residual_max_us),
    F(wr_clock_snapshot, accuracy_drift_us_per_s),
    F(wr_clock_snapshot, provenance),
};

static const abi_field f_clock_error[] = {
    F(wr_clock_error, precision_us),
    F(wr_clock_error, systematic_us),
    F(wr_clock_error, total_us),
};

static const abi_field f_clock_correction[] = {
    F(wr_clock_correction, fields),
    F(wr_clock_correction, reserved),
    F(wr_clock_correction, rate_ppm),
    F(wr_clock_correction, residual_drift_us_per_s),
    F(wr_clock_correction, provenance),
};

/* ------------------------------------------------------------------------ */
/* event.h                                                                   */
/*                                                                           */
/* ⚠ The payload union is ONE row.  A binding reads the header fields and     */
/* renders the rest through wr_event_format(); the union's members are        */
/* deliberately not pinned here until something decodes them, because a table */
/* nobody compares against is not evidence.  `u`'s size is pinned, which is   */
/* the part wr_event's ABI reservation exists to hold still (event.h:527).    */
/* ------------------------------------------------------------------------ */
static const abi_field f_event[] = {
    F(wr_event, type),
    F(wr_event, reserved),
    F(wr_event, sequence),
    F(wr_event, host_time_us),
    F(wr_event, stream_id),
    F(wr_event, u),
};

/* ------------------------------------------------------------------------ */
/* history.h                                                                 */
/* ------------------------------------------------------------------------ */
static const abi_field f_history_request[] = {
    F(wr_history_request, window),
    F(wr_history_request, deadline_us),
    F(wr_history_request, refill_gaps),
    F(wr_history_request, max_attempts),
    F(wr_history_request, alignment_budget_us),
    F(wr_history_request, user_tag),
};

static const abi_field f_gap[] = {
    F(wr_gap, span), F(wr_gap, indices), F(wr_gap, kind), F(wr_gap, reserved),
};

static const abi_field f_calibration_span[] = {
    F(wr_calibration_span, state_at_start),
    F(wr_calibration_span, state_at_end),
    F(wr_calibration_span, spans_transition),
    F(wr_calibration_span, reserved),
    F(wr_calibration_span, presence_angle_deg),
};

static const abi_field f_history_block[] = {
    F(wr_history_block, layout_version),
    F(wr_history_block, sample_stride),
    F(wr_history_block, samples),
    F(wr_history_block, sample_count),
    F(wr_history_block, status),
    F(wr_history_block, attempts),
    F(wr_history_block, coverage_overflowed),
    F(wr_history_block, reserved0),
    F(wr_history_block, stream_id),
    F(wr_history_block, user_tag),
    F(wr_history_block, requested),
    F(wr_history_block, requested_indices),
    F(wr_history_block, delivered),
    F(wr_history_block, delivered_count),
    F(wr_history_block, delivered_indices),
    F(wr_history_block, gaps),
    F(wr_history_block, gap_count),
    F(wr_history_block, coverage_fraction),
    F(wr_history_block, density),
    F(wr_history_block, achieved_hz),
    F(wr_history_block, largest_gap_us),
    F(wr_history_block, reserved1),
    F(wr_history_block, live_overlap_samples),
    F(wr_history_block, live_overlap_mismatches),
    F(wr_history_block, self_recording_gap),
    F(wr_history_block, fit),
    F(wr_history_block, calibration),
    F(wr_history_block, config),
    F(wr_history_block, reserved2),
    F(wr_history_block, pinned),
    F(wr_history_block, requested_at_us),
    F(wr_history_block, completed_at_us),
};

/* ------------------------------------------------------------------------ */
/* session.h                                                                 */
/* ------------------------------------------------------------------------ */
static const abi_field f_write_request[] = {
    F(wr_write_request, data),
    F(wr_write_request, length),
    F(wr_write_request, without_response),
    F(wr_write_request, reserved),
};

static const abi_field f_wire_chunk[] = {
    F(wr_wire_chunk, host_time_us),
    F(wr_wire_chunk, length),
    F(wr_wire_chunk, direction),
    F(wr_wire_chunk, flags),
    F(wr_wire_chunk, sequence),
    F(wr_wire_chunk, data),
};

static const abi_field f_session_policy[] = {
    F(wr_session_policy, keepalive_period_us),
    F(wr_session_policy, calibration_raise_limit_us),
    F(wr_session_policy, calibration_result_timeout_us),
    F(wr_session_policy, bringup_timeout_us),
    F(wr_session_policy, stream_start_timeout_us),
    F(wr_session_policy, keepalive_alarm_us),
    F(wr_session_policy, live_gap_alarm_us),
    F(wr_session_policy, pinned_report_period_us),
    F(wr_session_policy, history_pre_roll_us),
    F(wr_session_policy, history_post_roll_us),
    F(wr_session_policy, accuracy_drift_us_per_s),
    F(wr_session_policy, residual_alarm_us),
    F(wr_session_policy, clock_event_period_us),
    F(wr_session_policy, record_identifiers),
};

static const abi_field f_live_digest[] = {
    F(wr_live_digest, sample_index),
    F(wr_live_digest, reserved),
    F(wr_live_digest, digest),
};

static const abi_field f_session_memory[] = {
    F(wr_session_memory, live_ring),
    F(wr_session_memory, live_ring_capacity),
    F(wr_session_memory, event_ring),
    F(wr_session_memory, event_ring_capacity),
    F(wr_session_memory, wire_ring),
    F(wr_session_memory, wire_ring_capacity),
    F(wr_session_memory, history_gather),
    F(wr_session_memory, history_gather_capacity),
    F(wr_session_memory, coverage_storage),
    F(wr_session_memory, coverage_capacity),
    F(wr_session_memory, digest_ring),
    F(wr_session_memory, digest_ring_capacity),
};

static const abi_field f_session_config[] = {
    F(wr_session_config, stream_config),
    F(wr_session_config, policy),
    F(wr_session_config, memory),
    F(wr_session_config, allocator),
    F(wr_session_config, device_id),
};

/* ------------------------------------------------------------------------ */
/* version.h                                                                 */
/* ------------------------------------------------------------------------ */
static const abi_field f_abi_sizes[] = {
    F(wr_abi_sizes, abi_version),
    F(wr_abi_sizes, sample),
    F(wr_abi_sizes, unit_sample),
    F(wr_abi_sizes, event),
    F(wr_abi_sizes, clock_snapshot),
    F(wr_abi_sizes, history_block),
    F(wr_abi_sizes, write_request),
    F(wr_abi_sizes, wire_chunk),
    F(wr_abi_sizes, session_config),
    F(wr_abi_sizes, sample_layout_version),
};

/* ------------------------------------------------------------------------ */
/* record.h — outside the core, and the binding loads it from the same object */
/* ------------------------------------------------------------------------ */
static const abi_field f_recording_info[] = {
    F(wr_recording_info, device_id),
    F(wr_recording_info, config_bits),
    F(wr_recording_info, config_legacy),
    F(wr_recording_info, reserved),
    F(wr_recording_info, layout_version),
    F(wr_recording_info, clock),
    F(wr_recording_info, identifiers_recorded),
};

/* ------------------------------------------------------------------------ */
/* Enumerators                                                               */
/*                                                                           */
/* ⚠ NOT A LUXURY BESIDE THE STRUCT TABLE — the same silent-wrong shape, one  */
/* register down.  A binding that transcribes an enum by hand and gets a      */
/* value wrong mislabels every value of that kind and reports nothing.  When  */
/* this table was written, the hand-written Python had WR_GAP_NOT_RECORDED    */
/* and WR_GAP_NOT_DELIVERED THE WRONG WAY ROUND and four of eleven history    */
/* statuses on the wrong numbers.  Both read as perfectly ordinary code.      */
/*                                                                           */
/* Only the enums a binding actually mirrors need a row.  wr_warning_code is  */
/* deliberately absent: the binding renders warnings through                  */
/* wr_warning_code_name() and keeps no copy of the list, so there is nothing  */
/* there to drift.                                                           */
/* ------------------------------------------------------------------------ */
typedef struct abi_enumerator {
    const char *name;
    long long   value;
} abi_enumerator;

typedef struct abi_enum {
    const char            *name;
    const abi_enumerator  *values;
    size_t                 count;
} abi_enum;

#define E(sym) { #sym, (long long)(sym) }
#define EN(name, values) { name, (values), sizeof(values) / sizeof((values)[0]) }

static const abi_enumerator e_status[] = {
    E(WR_OK), E(WR_PENDING), E(WR_DONE),
    E(WR_ERR_INVALID_ARG), E(WR_ERR_INVALID_STATE), E(WR_ERR_NO_MEMORY),
    E(WR_ERR_BUFFER_TOO_SMALL), E(WR_ERR_NOT_SUPPORTED), E(WR_ERR_TRUNCATED),
    E(WR_ERR_MALFORMED), E(WR_ERR_UNKNOWN_MESSAGE), E(WR_ERR_NOT_ALLOWED),
    E(WR_ERR_TIMEOUT), E(WR_ERR_CANCELLED), E(WR_ERR_LINK_DOWN),
    E(WR_ERR_MTU_TOO_SMALL), E(WR_ERR_NO_STREAM), E(WR_ERR_DEVICE_ERROR),
    E(WR_ERR_NO_FIT), E(WR_ERR_EVICTED), E(WR_ERR_BUSY),
};

/* ⚠ WR_UNIT_COUNT and WR_CHANNEL_COUNT are sentinels rather than units, and a
 * binding that turned them into members would offer a third unit nothing can
 * return.  They are omitted here so the comparison demands their absence. */
static const abi_enumerator e_unit[] = {
    E(WR_UNIT_LOWER_ARM), E(WR_UNIT_PALM),
};

static const abi_enumerator e_channel[] = {
    E(WR_CH_ACCEL_X), E(WR_CH_ACCEL_Y), E(WR_CH_ACCEL_Z),
    E(WR_CH_GYRO_X), E(WR_CH_GYRO_Y), E(WR_CH_GYRO_Z),
};

static const abi_enumerator e_sample_source[] = {
    E(WR_SOURCE_LIVE), E(WR_SOURCE_HISTORY), E(WR_SOURCE_REPLAY),
};

static const abi_enumerator e_calibration_state[] = {
    E(WR_CAL_UNKNOWN), E(WR_CAL_UNCALIBRATED),
    E(WR_CAL_CALIBRATED), E(WR_CAL_LOST),
};

static const abi_enumerator e_sample_flag[] = {
    E(WR_SAMPLE_PINNED), E(WR_SAMPLE_NOT_TIME_ALIGNABLE),
    E(WR_SAMPLE_NONSTANDARD_CONFIG), E(WR_SAMPLE_HOST_TIME_EXTRAPOLATED),
    E(WR_SAMPLE_NO_FIT), E(WR_SAMPLE_QUAT_NORM_SUSPECT),
    E(WR_SAMPLE_TICKS_MISSING), E(WR_SAMPLE_INDEX_MISSING),
};

static const abi_enumerator e_wire_direction[] = {
    E(WR_WIRE_HOST_TO_DEVICE), E(WR_WIRE_DEVICE_TO_HOST), E(WR_WIRE_META),
};

static const abi_enumerator e_wire_flag[] = {
    E(WR_WIRE_REDACTED), E(WR_WIRE_LOST),
};

static const abi_enumerator e_history_status[] = {
    E(WR_HIST_COMPLETE), E(WR_HIST_SHORT), E(WR_HIST_HOLED),
    E(WR_HIST_TIMED_OUT), E(WR_HIST_CANCELLED), E(WR_HIST_REFUSED_ALIGNMENT),
    E(WR_HIST_EVICTED), E(WR_HIST_NO_STREAM), E(WR_HIST_LINK_LOST),
    E(WR_HIST_NOT_ALIGNABLE), E(WR_HIST_ERROR),
};

static const abi_enumerator e_gap_kind[] = {
    E(WR_GAP_NOT_RECORDED), E(WR_GAP_NOT_DELIVERED), E(WR_GAP_FIT_BLIND),
};

static const abi_enumerator e_clock_flag[] = {
    E(WR_CLOCK_HAS_FIT), E(WR_CLOCK_RATE_POOLED), E(WR_CLOCK_DEGENERATE),
    E(WR_CLOCK_RATE_IMPLAUSIBLE), E(WR_CLOCK_STALE), E(WR_CLOCK_BLIND),
    E(WR_CLOCK_EXTERNAL_CORRECTION), E(WR_CLOCK_SHORT_BASELINE),
};

static const abi_enumerator e_link_down_cause[] = {
    E(WR_LINK_DOWN_UNKNOWN), E(WR_LINK_DOWN_LOCAL_REQUEST),
    E(WR_LINK_DOWN_SUPERVISION_TIMEOUT), E(WR_LINK_DOWN_REMOTE_CLOSED),
    E(WR_LINK_DOWN_TRANSPORT_ERROR), E(WR_LINK_DOWN_ADAPTER_GONE),
    E(WR_LINK_DOWN_CONNECTION_TAKEN),
};

static const abi_enumerator e_recovery_advice[] = {
    E(WR_RECOVER_UNKNOWN), E(WR_RECOVER_RECONNECT_WITH_BACKOFF),
    E(WR_RECOVER_NEEDS_BUTTON_PRESS), E(WR_RECOVER_NEEDS_OTHER_APP_CLOSED),
    E(WR_RECOVER_DO_NOT_RETRY),
};

static const abi_enumerator e_calibration_phase[] = {
    E(WR_CALP_IDLE), E(WR_CALP_AWAIT_HORIZONTAL), E(WR_CALP_MARKING_POSE0),
    E(WR_CALP_OBSERVING_RAISE), E(WR_CALP_MARKING_POSE1), E(WR_CALP_APPLYING),
    E(WR_CALP_VERIFYING), E(WR_CALP_COMPLETE), E(WR_CALP_ABORTED),
};

static const abi_enumerator e_calibration_abort_reason[] = {
    E(WR_CAL_ABORT_NONE), E(WR_CAL_ABORT_CALLER), E(WR_CAL_ABORT_RAISE_TOO_SLOW),
    E(WR_CAL_ABORT_STREAM_LOST), E(WR_CAL_ABORT_LINK_LOST),
    E(WR_CAL_ABORT_NO_RESULT),
};

static const abi_enumerator e_event_type[] = {
    E(WR_EV_NONE), E(WR_EV_LINK_UP), E(WR_EV_LINK_DOWN), E(WR_EV_MTU_REJECTED),
    E(WR_EV_READY), E(WR_EV_DEVICE_INFO), E(WR_EV_BATTERY), E(WR_EV_IDENTITY),
    E(WR_EV_STREAM_STARTED), E(WR_EV_STREAM_STOPPED), E(WR_EV_STREAM_RESTARTED),
    E(WR_EV_CALIBRATION_PHASE), E(WR_EV_CALIBRATION_PRESENCE),
    E(WR_EV_HISTORY_STARTED), E(WR_EV_HISTORY_PROGRESS), E(WR_EV_HISTORY_READY),
    E(WR_EV_HISTORY_BLIND_SPAN), E(WR_EV_HISTORY_EVICTION_RISK),
    E(WR_EV_CLOCK_UPDATED), E(WR_EV_CLOCK_DEGRADED), E(WR_EV_BUTTON),
    E(WR_EV_DEVICE_ERROR), E(WR_EV_UNKNOWN_MESSAGE), E(WR_EV_PINNED_SAMPLES),
    E(WR_EV_WARNING),
};

static const abi_enumerator e_correction_field[] = {
    E(WR_CORRECTION_RATE), E(WR_CORRECTION_DRIFT),
};

static const abi_enumerator e_device_info_field[] = {
    E(WR_INFO_VERSIONS), E(WR_INFO_SENSOR_MAP), E(WR_INFO_BATTERY),
    E(WR_INFO_MAC), E(WR_INFO_SERIAL),
};

static const abi_enumerator e_config_mask[] = {
    E(WR_CFG_ALT_HARDWARE_PATH), E(WR_CFG_NO_MAGNETOMETER),
    E(WR_CFG_UNDECODED_BIT3), E(WR_CFG_UNDECODED_BIT4),
    E(WR_CFG_TIMESTAMPS), E(WR_CFG_EXTENDED_GYRO),
};

static const abi_enum k_enums[] = {
    EN("wr_status", e_status),
    EN("wr_unit", e_unit),
    EN("wr_channel", e_channel),
    EN("wr_sample_source", e_sample_source),
    EN("wr_calibration_state", e_calibration_state),
    EN("wr_sample_flag", e_sample_flag),
    EN("wr_wire_direction", e_wire_direction),
    EN("wr_wire_flag", e_wire_flag),
    EN("wr_history_status", e_history_status),
    EN("wr_gap_kind", e_gap_kind),
    EN("wr_clock_flag", e_clock_flag),
    EN("wr_link_down_cause", e_link_down_cause),
    EN("wr_recovery_advice", e_recovery_advice),
    EN("wr_calibration_phase", e_calibration_phase),
    EN("wr_calibration_abort_reason", e_calibration_abort_reason),
    EN("wr_event_type", e_event_type),
    EN("wr_correction_field", e_correction_field),
    EN("wr_device_info_field", e_device_info_field),
    EN("wr_config_mask", e_config_mask),
};

static const abi_struct k_structs[] = {
    S(wr_uuid, f_uuid),
    S(wr_index_range, f_index_range),
    S(wr_time_range, f_time_range),
    S(wr_allocator, f_allocator),
    S(wr_stream_config, f_stream_config),
    S(wr_device_info, f_device_info),
    S(wr_unit_sample, f_unit_sample),
    S(wr_sample, f_sample),
    S(wr_pinned_counts, f_pinned_counts),
    S(wr_clock_snapshot, f_clock_snapshot),
    S(wr_clock_error, f_clock_error),
    S(wr_clock_correction, f_clock_correction),
    S(wr_event, f_event),
    S(wr_history_request, f_history_request),
    S(wr_gap, f_gap),
    S(wr_calibration_span, f_calibration_span),
    S(wr_history_block, f_history_block),
    S(wr_write_request, f_write_request),
    S(wr_wire_chunk, f_wire_chunk),
    S(wr_session_policy, f_session_policy),
    S(wr_live_digest, f_live_digest),
    S(wr_session_memory, f_session_memory),
    S(wr_session_config, f_session_config),
    S(wr_abi_sizes, f_abi_sizes),
    S(wr_recording_info, f_recording_info),
};

/* ------------------------------------------------------------------------ */
/* The self-check                                                            */
/* ------------------------------------------------------------------------ */
/*
 * The rows must TILE their struct.  Padding is the only thing allowed between
 * two fields, and padding before a field can never exceed the struct's own
 * alignment minus one — so a gap of `align` or more is a field missing from the
 * row above it.  The same bound applies to the tail.
 *
 * ⚠ THIS IS CHECK 1 OF THE FOUR AT THE TOP OF THIS FILE, AND IT IS THE WEAKEST.
 * It catches an appended field and a mis-sized row outright; it is blind to a
 * dropped row that hides inside padding.  The struct-size comparison in
 * tests/test_python_abi.py is the one that actually closes that, because a
 * field added to a header changes sizeof.  Do not read a clean run here as the
 * table being complete — read it as the table being self-consistent.
 */
static int selfcheck_one(const abi_struct *s, FILE *err)
{
    size_t end = 0u;
    size_t i;
    int    problems = 0;

    if (s->field_count == 0u) {
        fprintf(err, "abi: %s has no fields listed\n", s->name);
        return 1;
    }

    for (i = 0u; i < s->field_count; i++) {
        const abi_field *f = &s->fields[i];

        if (f->offset < end) {
            fprintf(err, "abi: %s.%s overlaps the field before it (offset %zu < %zu)\n",
                    s->name, f->name, f->offset, end);
            problems++;
        } else if (f->offset - end >= s->align) {
            fprintf(err,
                    "abi: %s has an unexplained %zu-byte hole before .%s — "
                    "a field is MISSING from tools/wr_abi_table.c\n",
                    s->name, f->offset - end, f->name);
            problems++;
        }
        end = f->offset + f->size;
    }

    if (end > s->size) {
        fprintf(err, "abi: %s runs %zu bytes past sizeof (%zu)\n",
                s->name, end - s->size, s->size);
        problems++;
    } else if (s->size - end >= s->align) {
        fprintf(err,
                "abi: %s has %zu bytes of unexplained tail past .%s — "
                "a field is MISSING from tools/wr_abi_table.c\n",
                s->name, s->size - end, s->fields[s->field_count - 1u].name);
        problems++;
    }

    return problems;
}

/* ------------------------------------------------------------------------ */
int wr_abi_print_json(FILE *out, FILE *err)
{
    wr_abi_sizes sizes;
    size_t       i;
    size_t       j;
    int          problems = 0;

    wr_abi_sizes_get(&sizes);

    fprintf(out, "{\n");
    fprintf(out, "  \"abi_version\": %u,\n", (unsigned)sizes.abi_version);
    fprintf(out, "  \"sample_layout_version\": %u,\n",
            (unsigned)sizes.sample_layout_version);
    fprintf(out, "  \"library_version\": \"%s\",\n", wr_version_string());
    fprintf(out, "  \"structs\": {\n");

    for (i = 0u; i < sizeof(k_structs) / sizeof(k_structs[0]); i++) {
        const abi_struct *s = &k_structs[i];

        problems += selfcheck_one(s, err);

        fprintf(out, "    \"%s\": {\n", s->name);
        fprintf(out, "      \"size\": %zu,\n", s->size);
        fprintf(out, "      \"align\": %zu,\n", s->align);
        fprintf(out, "      \"fields\": [\n");
        for (j = 0u; j < s->field_count; j++) {
            const abi_field *f = &s->fields[j];
            fprintf(out, "        {\"name\": \"%s\", \"offset\": %zu, \"size\": %zu}%s\n",
                    f->name, f->offset, f->size,
                    (j + 1u == s->field_count) ? "" : ",");
        }
        fprintf(out, "      ]\n");
        fprintf(out, "    }%s\n",
                (i + 1u == sizeof(k_structs) / sizeof(k_structs[0])) ? "" : ",");
    }

    fprintf(out, "  },\n");
    fprintf(out, "  \"enums\": {\n");

    for (i = 0u; i < sizeof(k_enums) / sizeof(k_enums[0]); i++) {
        const abi_enum *e = &k_enums[i];

        fprintf(out, "    \"%s\": {\n", e->name);
        for (j = 0u; j < e->count; j++) {
            fprintf(out, "      \"%s\": %lld%s\n", e->values[j].name, e->values[j].value,
                    (j + 1u == e->count) ? "" : ",");
        }
        fprintf(out, "    }%s\n",
                (i + 1u == sizeof(k_enums) / sizeof(k_enums[0])) ? "" : ",");
    }

    fprintf(out, "  }\n");
    fprintf(out, "}\n");

    return problems;
}
