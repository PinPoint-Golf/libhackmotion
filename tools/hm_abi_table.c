/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_abi_table.c — every public struct's size, and every field's offset.
 *
 * See hm_abi_table.h for why hm_abi_check() is not enough on its own.
 *
 * ⚠ ADDING A FIELD TO A PUBLIC STRUCT MEANS ADDING A ROW HERE, and four things
 * check that you did.  They are listed with what each one CANNOT see, because a
 * completeness claim nobody has bounded is the shape this project has a rule
 * about:
 *
 *   1. The tiling self-check below — rows must start at 0, never overlap and
 *      reach the end, with no gap wider than the struct's own alignment.
 *      ⚠ BLIND to a dropped row whose bytes are indistinguishable from padding:
 *      remove `hm_sample.skew_us` (int32 at 20, followed by an 8-aligned field)
 *      and the four bytes it leaves behind are exactly the padding that would
 *      have been there anyway.  Confirmed by doing it.  No offset-based check
 *      can see that, because the information is not in the layout.
 *   2. sizeof here against ctypes.sizeof in tests/test_python_abi.py, for EVERY
 *      struct in this table rather than the nine hm_abi_check() covers.  A field
 *      added to a header changes the struct's size, so this is what actually
 *      catches a forgotten row.  ⚠ Blind to a field carved out of an existing
 *      `reserved` array, which does not change the size.
 *   3. The field-name sets, compared in BOTH directions by the same test.  A row
 *      here with no ctypes field, or a ctypes field with no row here, fails.
 *   4. hm_abi_check() at binding load, for the nine structs it knows.
 *
 * The one shape all four miss: a field carved out of a `reserved` slot, updated
 * in the header and in neither this table nor the binding.  It is narrow — the
 * reserved arrays are themselves rows here, so shrinking one changes that row's
 * size and trips (2) — and it is written down rather than engineered around.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hackmotion/hackmotion.h"
#include "hackmotion/record.h"

#include "hm_abi_table.h"

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
static const abi_field f_uuid[] = { F(hm_uuid, bytes) };

static const abi_field f_index_range[] = {
    F(hm_index_range, first), F(hm_index_range, last),
};

static const abi_field f_time_range[] = {
    F(hm_time_range, start_us), F(hm_time_range, end_us),
};

static const abi_field f_allocator[] = {
    F(hm_allocator, alloc), F(hm_allocator, free), F(hm_allocator, ctx),
};

/* ------------------------------------------------------------------------ */
/* config.h                                                                  */
/* ------------------------------------------------------------------------ */
static const abi_field f_stream_config[] = {
    F(hm_stream_config, bits),
    F(hm_stream_config, legacy),
    F(hm_stream_config, reserved),
    F(hm_stream_config, justification),
};

/* ------------------------------------------------------------------------ */
/* device.h                                                                  */
/* ------------------------------------------------------------------------ */
static const abi_field f_device_info[] = {
    F(hm_device_info, hardware_major),
    F(hm_device_info, hardware_minor),
    F(hm_device_info, protocol_major),
    F(hm_device_info, protocol_minor),
    F(hm_device_info, firmware_major),
    F(hm_device_info, firmware_minor),
    F(hm_device_info, product_id),
    F(hm_device_info, sensor_count),
    F(hm_device_info, sensor_location),
    F(hm_device_info, battery_percent),
    F(hm_device_info, status_undecoded),
    F(hm_device_info, mac),
    F(hm_device_info, serial),
    F(hm_device_info, valid),
};

/* ------------------------------------------------------------------------ */
/* sample.h                                                                  */
/* ------------------------------------------------------------------------ */
static const abi_field f_unit_sample[] = {
    F(hm_unit_sample, q_world_to_body_raw),
    F(hm_unit_sample, linear_accel_raw),
    F(hm_unit_sample, gyro_raw),
    F(hm_unit_sample, ticks_raw),
    F(hm_unit_sample, has_ticks),
    F(hm_unit_sample, pinned_mask),
    F(hm_unit_sample, q_world_to_body),
    F(hm_unit_sample, linear_accel_mps2),
    F(hm_unit_sample, gyro_dps),
    F(hm_unit_sample, device_time_us),
};

static const abi_field f_sample[] = {
    F(hm_sample, stream_id),
    F(hm_sample, sample_index),
    F(hm_sample, sample_index_raw),
    F(hm_sample, source),
    F(hm_sample, calibration),
    F(hm_sample, flags),
    F(hm_sample, config_bits),
    F(hm_sample, reserved0),
    F(hm_sample, skew_us),
    F(hm_sample, host_time_us),
    F(hm_sample, host_recv_us),
    F(hm_sample, precision_us),
    F(hm_sample, uncertainty_us),
    F(hm_sample, lower_arm),
    F(hm_sample, palm),
};

static const abi_field f_pinned_counts[] = {
    F(hm_pinned_counts, n), F(hm_pinned_counts, total),
};

/* ------------------------------------------------------------------------ */
/* clock.h                                                                   */
/* ------------------------------------------------------------------------ */
static const abi_field f_clock_snapshot[] = {
    F(hm_clock_snapshot, stream_id),
    F(hm_clock_snapshot, flags),
    F(hm_clock_snapshot, observations),
    F(hm_clock_snapshot, anchor_index),
    F(hm_clock_snapshot, reserved0),
    F(hm_clock_snapshot, anchor_host_us),
    F(hm_clock_snapshot, slope_us_per_index),
    F(hm_clock_snapshot, fitted_rate_hz),
    F(hm_clock_snapshot, raw_fitted_rate_hz),
    F(hm_clock_snapshot, external_ppm),
    F(hm_clock_snapshot, offset_us),
    F(hm_clock_snapshot, span_us),
    F(hm_clock_snapshot, last_observation_us),
    F(hm_clock_snapshot, first_index),
    F(hm_clock_snapshot, last_index),
    F(hm_clock_snapshot, residual_median_us),
    F(hm_clock_snapshot, residual_p90_us),
    F(hm_clock_snapshot, residual_max_us),
    F(hm_clock_snapshot, accuracy_drift_us_per_s),
    F(hm_clock_snapshot, provenance),
};

static const abi_field f_clock_error[] = {
    F(hm_clock_error, precision_us),
    F(hm_clock_error, systematic_us),
    F(hm_clock_error, total_us),
};

static const abi_field f_clock_correction[] = {
    F(hm_clock_correction, fields),
    F(hm_clock_correction, reserved),
    F(hm_clock_correction, rate_ppm),
    F(hm_clock_correction, residual_drift_us_per_s),
    F(hm_clock_correction, provenance),
};

/* ------------------------------------------------------------------------ */
/* event.h                                                                   */
/*                                                                           */
/* ⚠ The payload union is ONE row.  A binding reads the header fields and     */
/* renders the rest through hm_event_format(); the union's members are        */
/* deliberately not pinned here until something decodes them, because a table */
/* nobody compares against is not evidence.  `u`'s size is pinned, which is   */
/* the part hm_event's ABI reservation exists to hold still (event.h:527).    */
/* ------------------------------------------------------------------------ */
static const abi_field f_event[] = {
    F(hm_event, type),
    F(hm_event, reserved),
    F(hm_event, sequence),
    F(hm_event, host_time_us),
    F(hm_event, stream_id),
    F(hm_event, u),
};

/* ------------------------------------------------------------------------ */
/* history.h                                                                 */
/* ------------------------------------------------------------------------ */
static const abi_field f_history_request[] = {
    F(hm_history_request, window),
    F(hm_history_request, deadline_us),
    F(hm_history_request, refill_gaps),
    F(hm_history_request, max_attempts),
    F(hm_history_request, alignment_budget_us),
    F(hm_history_request, user_tag),
};

static const abi_field f_gap[] = {
    F(hm_gap, span), F(hm_gap, indices), F(hm_gap, kind), F(hm_gap, reserved),
};

static const abi_field f_calibration_span[] = {
    F(hm_calibration_span, state_at_start),
    F(hm_calibration_span, state_at_end),
    F(hm_calibration_span, spans_transition),
    F(hm_calibration_span, reserved),
    F(hm_calibration_span, presence_angle_deg),
};

static const abi_field f_history_block[] = {
    F(hm_history_block, layout_version),
    F(hm_history_block, sample_stride),
    F(hm_history_block, samples),
    F(hm_history_block, sample_count),
    F(hm_history_block, status),
    F(hm_history_block, attempts),
    F(hm_history_block, coverage_overflowed),
    F(hm_history_block, reserved0),
    F(hm_history_block, stream_id),
    F(hm_history_block, user_tag),
    F(hm_history_block, requested),
    F(hm_history_block, requested_indices),
    F(hm_history_block, delivered),
    F(hm_history_block, delivered_count),
    F(hm_history_block, delivered_indices),
    F(hm_history_block, gaps),
    F(hm_history_block, gap_count),
    F(hm_history_block, coverage_fraction),
    F(hm_history_block, density),
    F(hm_history_block, achieved_hz),
    F(hm_history_block, largest_gap_us),
    F(hm_history_block, reserved1),
    F(hm_history_block, live_overlap_samples),
    F(hm_history_block, live_overlap_mismatches),
    F(hm_history_block, self_recording_gap),
    F(hm_history_block, fit),
    F(hm_history_block, calibration),
    F(hm_history_block, config),
    F(hm_history_block, reserved2),
    F(hm_history_block, pinned),
    F(hm_history_block, requested_at_us),
    F(hm_history_block, completed_at_us),
};

/* ------------------------------------------------------------------------ */
/* session.h                                                                 */
/* ------------------------------------------------------------------------ */
static const abi_field f_write_request[] = {
    F(hm_write_request, data),
    F(hm_write_request, length),
    F(hm_write_request, without_response),
    F(hm_write_request, reserved),
};

static const abi_field f_wire_chunk[] = {
    F(hm_wire_chunk, host_time_us),
    F(hm_wire_chunk, length),
    F(hm_wire_chunk, direction),
    F(hm_wire_chunk, flags),
    F(hm_wire_chunk, sequence),
    F(hm_wire_chunk, data),
};

static const abi_field f_session_policy[] = {
    F(hm_session_policy, keepalive_period_us),
    F(hm_session_policy, calibration_raise_limit_us),
    F(hm_session_policy, calibration_result_timeout_us),
    F(hm_session_policy, bringup_timeout_us),
    F(hm_session_policy, stream_start_timeout_us),
    F(hm_session_policy, keepalive_alarm_us),
    F(hm_session_policy, live_gap_alarm_us),
    F(hm_session_policy, pinned_report_period_us),
    F(hm_session_policy, history_pre_roll_us),
    F(hm_session_policy, history_post_roll_us),
    F(hm_session_policy, accuracy_drift_us_per_s),
    F(hm_session_policy, residual_alarm_us),
    F(hm_session_policy, clock_event_period_us),
    F(hm_session_policy, record_identifiers),
};

static const abi_field f_live_digest[] = {
    F(hm_live_digest, sample_index),
    F(hm_live_digest, reserved),
    F(hm_live_digest, digest),
};

static const abi_field f_session_memory[] = {
    F(hm_session_memory, live_ring),
    F(hm_session_memory, live_ring_capacity),
    F(hm_session_memory, event_ring),
    F(hm_session_memory, event_ring_capacity),
    F(hm_session_memory, wire_ring),
    F(hm_session_memory, wire_ring_capacity),
    F(hm_session_memory, history_gather),
    F(hm_session_memory, history_gather_capacity),
    F(hm_session_memory, coverage_storage),
    F(hm_session_memory, coverage_capacity),
    F(hm_session_memory, digest_ring),
    F(hm_session_memory, digest_ring_capacity),
};

static const abi_field f_session_config[] = {
    F(hm_session_config, stream_config),
    F(hm_session_config, policy),
    F(hm_session_config, memory),
    F(hm_session_config, allocator),
    F(hm_session_config, device_id),
};

/* ------------------------------------------------------------------------ */
/* version.h                                                                 */
/* ------------------------------------------------------------------------ */
static const abi_field f_abi_sizes[] = {
    F(hm_abi_sizes, abi_version),
    F(hm_abi_sizes, sample),
    F(hm_abi_sizes, unit_sample),
    F(hm_abi_sizes, event),
    F(hm_abi_sizes, clock_snapshot),
    F(hm_abi_sizes, history_block),
    F(hm_abi_sizes, write_request),
    F(hm_abi_sizes, wire_chunk),
    F(hm_abi_sizes, session_config),
    F(hm_abi_sizes, sample_layout_version),
};

/* ------------------------------------------------------------------------ */
/* record.h — outside the core, and the binding loads it from the same object */
/* ------------------------------------------------------------------------ */
static const abi_field f_recording_info[] = {
    F(hm_recording_info, device_id),
    F(hm_recording_info, config_bits),
    F(hm_recording_info, config_legacy),
    F(hm_recording_info, reserved),
    F(hm_recording_info, layout_version),
    F(hm_recording_info, clock),
    F(hm_recording_info, identifiers_recorded),
};

/* ------------------------------------------------------------------------ */
/* Enumerators                                                               */
/*                                                                           */
/* ⚠ NOT A LUXURY BESIDE THE STRUCT TABLE — the same silent-wrong shape, one  */
/* register down.  A binding that transcribes an enum by hand and gets a      */
/* value wrong mislabels every value of that kind and reports nothing.  When  */
/* this table was written, the hand-written Python had HM_GAP_NOT_RECORDED    */
/* and HM_GAP_NOT_DELIVERED THE WRONG WAY ROUND and four of eleven history    */
/* statuses on the wrong numbers.  Both read as perfectly ordinary code.      */
/*                                                                           */
/* Only the enums a binding actually mirrors need a row.  hm_warning_code is  */
/* deliberately absent: the binding renders warnings through                  */
/* hm_warning_code_name() and keeps no copy of the list, so there is nothing  */
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
    E(HM_OK), E(HM_PENDING), E(HM_DONE),
    E(HM_ERR_INVALID_ARG), E(HM_ERR_INVALID_STATE), E(HM_ERR_NO_MEMORY),
    E(HM_ERR_BUFFER_TOO_SMALL), E(HM_ERR_NOT_SUPPORTED), E(HM_ERR_TRUNCATED),
    E(HM_ERR_MALFORMED), E(HM_ERR_UNKNOWN_MESSAGE), E(HM_ERR_NOT_ALLOWED),
    E(HM_ERR_TIMEOUT), E(HM_ERR_CANCELLED), E(HM_ERR_LINK_DOWN),
    E(HM_ERR_MTU_TOO_SMALL), E(HM_ERR_NO_STREAM), E(HM_ERR_DEVICE_ERROR),
    E(HM_ERR_NO_FIT), E(HM_ERR_EVICTED), E(HM_ERR_BUSY),
};

/* ⚠ HM_UNIT_COUNT and HM_CHANNEL_COUNT are sentinels rather than units, and a
 * binding that turned them into members would offer a third unit nothing can
 * return.  They are omitted here so the comparison demands their absence. */
static const abi_enumerator e_unit[] = {
    E(HM_UNIT_LOWER_ARM), E(HM_UNIT_PALM),
};

static const abi_enumerator e_channel[] = {
    E(HM_CH_ACCEL_X), E(HM_CH_ACCEL_Y), E(HM_CH_ACCEL_Z),
    E(HM_CH_GYRO_X), E(HM_CH_GYRO_Y), E(HM_CH_GYRO_Z),
};

static const abi_enumerator e_sample_source[] = {
    E(HM_SOURCE_LIVE), E(HM_SOURCE_HISTORY), E(HM_SOURCE_REPLAY),
};

static const abi_enumerator e_calibration_state[] = {
    E(HM_CAL_UNKNOWN), E(HM_CAL_UNCALIBRATED),
    E(HM_CAL_CALIBRATED), E(HM_CAL_LOST),
};

static const abi_enumerator e_sample_flag[] = {
    E(HM_SAMPLE_PINNED), E(HM_SAMPLE_NOT_TIME_ALIGNABLE),
    E(HM_SAMPLE_NONSTANDARD_CONFIG), E(HM_SAMPLE_HOST_TIME_EXTRAPOLATED),
    E(HM_SAMPLE_NO_FIT), E(HM_SAMPLE_QUAT_NORM_SUSPECT),
    E(HM_SAMPLE_TICKS_MISSING), E(HM_SAMPLE_INDEX_MISSING),
};

static const abi_enumerator e_wire_direction[] = {
    E(HM_WIRE_HOST_TO_DEVICE), E(HM_WIRE_DEVICE_TO_HOST), E(HM_WIRE_META),
};

static const abi_enumerator e_wire_flag[] = {
    E(HM_WIRE_REDACTED), E(HM_WIRE_LOST),
};

static const abi_enumerator e_history_status[] = {
    E(HM_HIST_COMPLETE), E(HM_HIST_SHORT), E(HM_HIST_HOLED),
    E(HM_HIST_TIMED_OUT), E(HM_HIST_CANCELLED), E(HM_HIST_REFUSED_ALIGNMENT),
    E(HM_HIST_EVICTED), E(HM_HIST_NO_STREAM), E(HM_HIST_LINK_LOST),
    E(HM_HIST_NOT_ALIGNABLE), E(HM_HIST_ERROR),
};

static const abi_enumerator e_gap_kind[] = {
    E(HM_GAP_NOT_RECORDED), E(HM_GAP_NOT_DELIVERED), E(HM_GAP_FIT_BLIND),
};

static const abi_enumerator e_clock_flag[] = {
    E(HM_CLOCK_HAS_FIT), E(HM_CLOCK_RATE_POOLED), E(HM_CLOCK_DEGENERATE),
    E(HM_CLOCK_RATE_IMPLAUSIBLE), E(HM_CLOCK_STALE), E(HM_CLOCK_BLIND),
    E(HM_CLOCK_EXTERNAL_CORRECTION), E(HM_CLOCK_SHORT_BASELINE),
};

static const abi_enumerator e_link_down_cause[] = {
    E(HM_LINK_DOWN_UNKNOWN), E(HM_LINK_DOWN_LOCAL_REQUEST),
    E(HM_LINK_DOWN_SUPERVISION_TIMEOUT), E(HM_LINK_DOWN_REMOTE_CLOSED),
    E(HM_LINK_DOWN_TRANSPORT_ERROR), E(HM_LINK_DOWN_ADAPTER_GONE),
    E(HM_LINK_DOWN_CONNECTION_TAKEN),
};

static const abi_enumerator e_recovery_advice[] = {
    E(HM_RECOVER_UNKNOWN), E(HM_RECOVER_RECONNECT_WITH_BACKOFF),
    E(HM_RECOVER_NEEDS_BUTTON_PRESS), E(HM_RECOVER_NEEDS_OTHER_APP_CLOSED),
    E(HM_RECOVER_DO_NOT_RETRY),
};

static const abi_enumerator e_calibration_phase[] = {
    E(HM_CALP_IDLE), E(HM_CALP_AWAIT_HORIZONTAL), E(HM_CALP_MARKING_POSE0),
    E(HM_CALP_OBSERVING_RAISE), E(HM_CALP_MARKING_POSE1), E(HM_CALP_APPLYING),
    E(HM_CALP_VERIFYING), E(HM_CALP_COMPLETE), E(HM_CALP_ABORTED),
};

static const abi_enumerator e_calibration_abort_reason[] = {
    E(HM_CAL_ABORT_NONE), E(HM_CAL_ABORT_CALLER), E(HM_CAL_ABORT_RAISE_TOO_SLOW),
    E(HM_CAL_ABORT_STREAM_LOST), E(HM_CAL_ABORT_LINK_LOST),
    E(HM_CAL_ABORT_NO_RESULT),
};

static const abi_enumerator e_event_type[] = {
    E(HM_EV_NONE), E(HM_EV_LINK_UP), E(HM_EV_LINK_DOWN), E(HM_EV_MTU_REJECTED),
    E(HM_EV_READY), E(HM_EV_DEVICE_INFO), E(HM_EV_BATTERY), E(HM_EV_IDENTITY),
    E(HM_EV_STREAM_STARTED), E(HM_EV_STREAM_STOPPED), E(HM_EV_STREAM_RESTARTED),
    E(HM_EV_CALIBRATION_PHASE), E(HM_EV_CALIBRATION_PRESENCE),
    E(HM_EV_HISTORY_STARTED), E(HM_EV_HISTORY_PROGRESS), E(HM_EV_HISTORY_READY),
    E(HM_EV_HISTORY_BLIND_SPAN), E(HM_EV_HISTORY_EVICTION_RISK),
    E(HM_EV_CLOCK_UPDATED), E(HM_EV_CLOCK_DEGRADED), E(HM_EV_BUTTON),
    E(HM_EV_DEVICE_ERROR), E(HM_EV_UNKNOWN_MESSAGE), E(HM_EV_PINNED_SAMPLES),
    E(HM_EV_WARNING),
};

static const abi_enumerator e_correction_field[] = {
    E(HM_CORRECTION_RATE), E(HM_CORRECTION_DRIFT),
};

static const abi_enumerator e_device_info_field[] = {
    E(HM_INFO_VERSIONS), E(HM_INFO_SENSOR_MAP), E(HM_INFO_BATTERY),
    E(HM_INFO_MAC), E(HM_INFO_SERIAL),
};

static const abi_enumerator e_config_mask[] = {
    E(HM_CFG_ALT_HARDWARE_PATH), E(HM_CFG_NO_MAGNETOMETER),
    E(HM_CFG_UNDECODED_BIT3), E(HM_CFG_UNDECODED_BIT4),
    E(HM_CFG_TIMESTAMPS), E(HM_CFG_EXTENDED_GYRO),
};

static const abi_enum k_enums[] = {
    EN("hm_status", e_status),
    EN("hm_unit", e_unit),
    EN("hm_channel", e_channel),
    EN("hm_sample_source", e_sample_source),
    EN("hm_calibration_state", e_calibration_state),
    EN("hm_sample_flag", e_sample_flag),
    EN("hm_wire_direction", e_wire_direction),
    EN("hm_wire_flag", e_wire_flag),
    EN("hm_history_status", e_history_status),
    EN("hm_gap_kind", e_gap_kind),
    EN("hm_clock_flag", e_clock_flag),
    EN("hm_link_down_cause", e_link_down_cause),
    EN("hm_recovery_advice", e_recovery_advice),
    EN("hm_calibration_phase", e_calibration_phase),
    EN("hm_calibration_abort_reason", e_calibration_abort_reason),
    EN("hm_event_type", e_event_type),
    EN("hm_correction_field", e_correction_field),
    EN("hm_device_info_field", e_device_info_field),
    EN("hm_config_mask", e_config_mask),
};

static const abi_struct k_structs[] = {
    S(hm_uuid, f_uuid),
    S(hm_index_range, f_index_range),
    S(hm_time_range, f_time_range),
    S(hm_allocator, f_allocator),
    S(hm_stream_config, f_stream_config),
    S(hm_device_info, f_device_info),
    S(hm_unit_sample, f_unit_sample),
    S(hm_sample, f_sample),
    S(hm_pinned_counts, f_pinned_counts),
    S(hm_clock_snapshot, f_clock_snapshot),
    S(hm_clock_error, f_clock_error),
    S(hm_clock_correction, f_clock_correction),
    S(hm_event, f_event),
    S(hm_history_request, f_history_request),
    S(hm_gap, f_gap),
    S(hm_calibration_span, f_calibration_span),
    S(hm_history_block, f_history_block),
    S(hm_write_request, f_write_request),
    S(hm_wire_chunk, f_wire_chunk),
    S(hm_session_policy, f_session_policy),
    S(hm_live_digest, f_live_digest),
    S(hm_session_memory, f_session_memory),
    S(hm_session_config, f_session_config),
    S(hm_abi_sizes, f_abi_sizes),
    S(hm_recording_info, f_recording_info),
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
                    "a field is MISSING from tools/hm_abi_table.c\n",
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
                "a field is MISSING from tools/hm_abi_table.c\n",
                s->name, s->size - end, s->fields[s->field_count - 1u].name);
        problems++;
    }

    return problems;
}

/* ------------------------------------------------------------------------ */
int hm_abi_print_json(FILE *out, FILE *err)
{
    hm_abi_sizes sizes;
    size_t       i;
    size_t       j;
    int          problems = 0;

    hm_abi_sizes_get(&sizes);

    fprintf(out, "{\n");
    fprintf(out, "  \"abi_version\": %u,\n", (unsigned)sizes.abi_version);
    fprintf(out, "  \"sample_layout_version\": %u,\n",
            (unsigned)sizes.sample_layout_version);
    fprintf(out, "  \"library_version\": \"%s\",\n", hm_version_string());
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
