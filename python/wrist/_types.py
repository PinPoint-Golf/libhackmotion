# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""The public structs and enums, declared a second time in ctypes.

⚠ THIS FILE IS A SECOND COPY OF THE HEADERS AND IS TREATED AS ONE.

A field one slot out of place here decodes every sample into plausible nonsense
with no error anywhere — the failure shape that has escaped review in this
project three times.  So nothing here is trusted on inspection:
`tests/test_python_abi.py` compares every struct's size and every field's name
and offset against `wrwire abi`, which reads them out of the compiler.  Enum
values are pinned the same way, against the library's own name functions.

⚠ THE COMMENTS HERE ARE THE SHORT FORM.  Each one names the trap and points at
the header that carries the argument; the headers are not summarised, because a
summary that drifts is worse than a pointer.  Read `include/wrist/` before
changing a meaning.
"""

from __future__ import annotations

import enum
from ctypes import (
    Structure,
    Union,
    c_bool,
    c_char,
    c_double,
    c_float,
    c_int16,
    c_int32,
    c_int64,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
    POINTER,
)

# --- types.h ---------------------------------------------------------------
# ⚠ Microseconds on a clock THE CALLER OWNS and the library never reads.  It
# must be monotonic: a wall clock stepped by NTP corrupts a capture in a way
# that looks like a sensor fault.  Python's `time.monotonic_ns() // 1000`.
wr_time_us = c_int64

WR_TIME_UNKNOWN = -(2**63)  # INT64_MIN — structurally unavailable, e.g. a
#                             history record's arrival instant
WR_TIME_NEVER = 2**63 - 1  # INT64_MAX — the session has no pending deadline

# --- sizes that appear in struct layouts -----------------------------------
WR_MAX_COMMAND_LEN = 8
WR_WIRE_CHUNK_MAX = 256
WR_DEVICE_ID_MAX = 64
WR_SERIAL_MAX = 16
WR_MAC_STRING_MAX = 18
WR_SENSOR_LOCATION_MAX = 4
WR_PROVENANCE_MAX = 64
WR_CONFIG_JUSTIFICATION_MAX = 64
WR_RECORD_CLOCK_MAX = 32
WR_UNIT_COUNT = 2
WR_CHANNEL_COUNT = 6
WR_EVENT_PAYLOAD_MAX = 256

# --- recommended capacities (session.h) ------------------------------------
# ⚠ NOT sized from an assumed live rate.  There is no live rate ceiling: dense
# bursts reach the full internal rate in every session containing motion, so a
# ring sized from an assumed maximum is wrong exactly when it matters.  Size
# from how often you drain.
WR_LIVE_RING_RECOMMENDED = 2048
WR_EVENT_RING_RECOMMENDED = 256
WR_WIRE_RING_RECOMMENDED = 1024
WR_HISTORY_GATHER_RECOMMENDED = 12288
WR_COVERAGE_RECOMMENDED = 512
WR_DIGEST_RING_RECOMMENDED = 1024

WR_MIN_ATT_MTU = 96
WR_CONFIG_OBSERVED_DEFAULT = 0x7E


# ---------------------------------------------------------------------------
# Enums
#
# ⚠ Every one of these is pinned against the library's own name function in
# tests/test_python_abi.py.  A value that drifts lands on a different member or
# off the end, and the name comes back wrong or as the "unknown" fallback.
# ---------------------------------------------------------------------------
class Status(enum.IntEnum):
    """⚠ NON-NEGATIVE IS SUCCESS.  Test ``st < Status.OK``, never ``!=`` —
    PENDING and DONE are not errors."""

    OK = 0
    PENDING = 1
    DONE = 2

    ERR_INVALID_ARG = -1
    ERR_INVALID_STATE = -2
    ERR_NO_MEMORY = -3
    ERR_BUFFER_TOO_SMALL = -4
    ERR_NOT_SUPPORTED = -5
    ERR_TRUNCATED = -6
    ERR_MALFORMED = -7
    ERR_UNKNOWN_MESSAGE = -8
    ERR_NOT_ALLOWED = -9
    ERR_TIMEOUT = -10
    ERR_CANCELLED = -11
    ERR_LINK_DOWN = -12
    ERR_MTU_TOO_SMALL = -13
    ERR_NO_STREAM = -14
    ERR_DEVICE_ERROR = -15
    ERR_NO_FIT = -16
    ERR_EVICTED = -17
    ERR_BUSY = -18


class Unit(enum.IntEnum):
    """⚠ THE ORDER IS FIXED BY THE CABLE, NOT BY CONVENTION.  The first wire
    block is the lower-arm unit and the second is the palm unit.  Swapping them
    produces a wrist angle that is simply MIRRORED — a silent bug that every
    plausibility check passes."""

    LOWER_ARM = 0
    PALM = 1


class Channel(enum.IntEnum):
    ACCEL_X = 0
    ACCEL_Y = 1
    ACCEL_Z = 2
    GYRO_X = 3
    GYRO_Y = 4
    GYRO_Z = 5


class SampleSource(enum.IntEnum):
    LIVE = 0  # arrival is real time; these pairs ARE the clock fit
    HISTORY = 1  # arrival is meaningless; host time comes from the fit
    REPLAY = 2  # decoded from a wire recording


class CalibrationState(enum.IntEnum):
    """⚠ UNKNOWN IS NOT A HOPEFUL CALIBRATED.  The device applies its transform
    for every attempt, including ones an application would reject, so "we issued
    the markers" is not evidence that calibration took.  Only a passing presence
    measurement reaches CALIBRATED.

    ⚠ No sample ever carries LOST — it exists for a history block's calibration
    span, where CALIBRATED→LOST says the calibration went away inside the
    block."""

    UNKNOWN = 0
    UNCALIBRATED = 1
    CALIBRATED = 2
    LOST = 3


class SampleFlag(enum.IntFlag):
    PINNED = 1 << 0  # ⚠ a channel saturated, SILENTLY
    NOT_TIME_ALIGNABLE = 1 << 1
    NONSTANDARD_CONFIG = 1 << 2
    HOST_TIME_EXTRAPOLATED = 1 << 3
    NO_FIT = 1 << 4
    QUAT_NORM_SUSPECT = 1 << 5
    TICKS_MISSING = 1 << 6
    INDEX_MISSING = 1 << 7


class WireDirection(enum.IntEnum):
    HOST_TO_DEVICE = 0
    DEVICE_TO_HOST = 1
    META = 2


class WireFlag(enum.IntFlag):
    REDACTED = 1 << 0  # identifiers were removed before this chunk was queued
    LOST = 1 << 1  # chunks were dropped before this one


class HistoryStatus(enum.IntEnum):
    """⚠ On real hardware nothing ever comes back COMPLETE.  The device's buffer
    is motion-adaptive, so a retrieval over a still wrist is an even one-in-eight
    and every reply is HOLED without anything being wrong."""

    COMPLETE = 0
    SHORT = 1
    HOLED = 2
    TIMED_OUT = 3
    CANCELLED = 4
    REFUSED_ALIGNMENT = 5
    EVICTED = 6
    NO_STREAM = 7
    LINK_LOST = 8
    NOT_ALIGNABLE = 9
    ERROR = 10


class GapKind(enum.IntEnum):
    """⚠ The three kinds MAY OVERLAP.  Read them as three independent statements
    about one index axis, not as a partition of it.

    ⚠ NOT_RECORDED IS 0 AND NOT_DELIVERED IS 1, which is the opposite of the
    order the names suggest to anyone reading the gap list for the first time.
    They were transcribed the wrong way round when this file was written, and
    the enum table in `wrwire abi` is what said so — nothing about a mislabelled
    gap looks wrong on inspection."""

    NOT_RECORDED = 0
    NOT_DELIVERED = 1
    FIT_BLIND = 2


class ClockFlag(enum.IntFlag):
    HAS_FIT = 1 << 0
    RATE_POOLED = 1 << 1
    DEGENERATE = 1 << 2  # ⚠ envelope support collapsed to one end
    RATE_IMPLAUSIBLE = 1 << 3
    STALE = 1 << 4
    BLIND = 1 << 5  # a retrieval is in flight; live is suspended
    EXTERNAL_CORRECTION = 1 << 6
    SHORT_BASELINE = 1 << 7


class LinkDownCause(enum.IntEnum):
    """What a transport passes to ``on_link_down``.  ⚠ Pass a real
    classification: it drives the recovery advice a consumer shows a user, and
    ``CONNECTION_TAKEN`` in particular — the device accepts ONE connection and
    the vendor app wins the race — is a thing a person can act on, where a retry
    loop against it is pure waste."""

    UNKNOWN = 0
    LOCAL_REQUEST = 1
    SUPERVISION_TIMEOUT = 2
    REMOTE_CLOSED = 3
    TRANSPORT_ERROR = 4
    ADAPTER_GONE = 5
    CONNECTION_TAKEN = 6


class RecoveryAdvice(enum.IntEnum):
    """⚠ ``NEEDS_BUTTON_PRESS`` and ``DO_NOT_RETRY`` mean stop reconnecting.
    Only the user can fix the first, and the second is a power-off we asked
    for."""

    UNKNOWN = 0
    RECONNECT_WITH_BACKOFF = 1
    NEEDS_BUTTON_PRESS = 2
    NEEDS_OTHER_APP_CLOSED = 3
    DO_NOT_RETRY = 4


class CalibrationPhase(enum.IntEnum):
    """⚠ ``COMPLETE`` IS NOT "CALIBRATED".  The routine finishing says nothing
    about whether the transform took — check ``Session.calibration_state``.

    ⚠ At ``VERIFYING`` the transform is ALREADY APPLIED and no command reverses
    it, so aborting there skips the presence check rather than undoing
    anything."""

    IDLE = 0
    AWAIT_HORIZONTAL = 1
    MARKING_POSE0 = 2
    OBSERVING_RAISE = 3
    MARKING_POSE1 = 4
    APPLYING = 5
    VERIFYING = 6
    COMPLETE = 7
    ABORTED = 8


class CalibrationAbortReason(enum.IntEnum):
    NONE = 0
    CALLER = 1
    RAISE_TOO_SLOW = 2  # ⚠ CLIENT policy — the device imposes no deadline
    STREAM_LOST = 3
    LINK_LOST = 4
    NO_RESULT = 5


class EventType(enum.IntEnum):
    NONE = 0
    LINK_UP = 1
    LINK_DOWN = 2
    MTU_REJECTED = 3
    READY = 4
    DEVICE_INFO = 5
    BATTERY = 6
    IDENTITY = 7
    STREAM_STARTED = 8
    STREAM_STOPPED = 9
    STREAM_RESTARTED = 10
    CALIBRATION_PHASE = 11
    CALIBRATION_PRESENCE = 12
    HISTORY_STARTED = 13
    HISTORY_PROGRESS = 14
    HISTORY_READY = 15
    HISTORY_BLIND_SPAN = 16
    HISTORY_EVICTION_RISK = 17
    CLOCK_UPDATED = 18
    CLOCK_DEGRADED = 19
    BUTTON = 20
    DEVICE_ERROR = 21
    UNKNOWN_MESSAGE = 22
    PINNED_SAMPLES = 23
    WARNING = 24


class DeviceInfoField(enum.IntFlag):
    VERSIONS = 1 << 0
    SENSOR_MAP = 1 << 1
    BATTERY = 1 << 2
    MAC = 1 << 3
    SERIAL = 1 << 4


class ConfigMask(enum.IntFlag):
    """⚠ TIMESTAMPS changes the BLOCK SIZE and EXTENDED_GYRO changes the GYRO
    SCALE.  Two of these change the wire format, not just the content."""

    ALT_HARDWARE_PATH = 0x01
    NO_MAGNETOMETER = 0x06  # covers two bits, hence "mask" not "bit"
    UNDECODED_BIT3 = 0x08
    UNDECODED_BIT4 = 0x10
    TIMESTAMPS = 0x20
    EXTENDED_GYRO = 0x40


class CorrectionField(enum.IntFlag):
    RATE = 1 << 0
    DRIFT = 1 << 1


# ⚠ Python enum name -> (C enum name, the prefix the C enumerators carry).  The
# layout test compares member for member in both directions, so a member added
# on either side without the other fails.
#
# ⚠ wr_warning_code is DELIBERATELY ABSENT.  The binding renders warnings
# through wr_warning_code_name() and keeps no copy of that list, so there is no
# second copy of it to drift.  A Python enum added here that C does not dump,
# or dumped in C and mirrored here without an entry, fails the test.
PINNED_ENUMS = {
    Status: ("wr_status", "WR_"),
    Unit: ("wr_unit", "WR_UNIT_"),
    Channel: ("wr_channel", "WR_CH_"),
    SampleSource: ("wr_sample_source", "WR_SOURCE_"),
    CalibrationState: ("wr_calibration_state", "WR_CAL_"),
    SampleFlag: ("wr_sample_flag", "WR_SAMPLE_"),
    WireDirection: ("wr_wire_direction", "WR_WIRE_"),
    WireFlag: ("wr_wire_flag", "WR_WIRE_"),
    HistoryStatus: ("wr_history_status", "WR_HIST_"),
    GapKind: ("wr_gap_kind", "WR_GAP_"),
    ClockFlag: ("wr_clock_flag", "WR_CLOCK_"),
    LinkDownCause: ("wr_link_down_cause", "WR_LINK_DOWN_"),
    RecoveryAdvice: ("wr_recovery_advice", "WR_RECOVER_"),
    CalibrationPhase: ("wr_calibration_phase", "WR_CALP_"),
    CalibrationAbortReason: ("wr_calibration_abort_reason", "WR_CAL_ABORT_"),
    EventType: ("wr_event_type", "WR_EV_"),
    CorrectionField: ("wr_correction_field", "WR_CORRECTION_"),
    DeviceInfoField: ("wr_device_info_field", "WR_INFO_"),
    ConfigMask: ("wr_config_mask", "WR_CFG_"),
}


# ---------------------------------------------------------------------------
# Structs — types.h
# ---------------------------------------------------------------------------
class wr_uuid(Structure):
    _fields_ = [("bytes", c_uint8 * 16)]


class wr_index_range(Structure):
    """Inclusive on both ends; ``first == last`` is one sample."""

    _fields_ = [("first", c_uint32), ("last", c_uint32)]


class wr_time_range(Structure):
    """⚠ HALF-OPEN [start_us, end_us) — where ``wr_index_range`` is INCLUSIVE.
    That difference is where an off-by-one is born; never convert one to the
    other by hand."""

    _fields_ = [("start_us", wr_time_us), ("end_us", wr_time_us)]


class wr_allocator(Structure):
    """Left zeroed: the library then uses malloc/free, and makes exactly one
    allocation at create and one free at destroy."""

    _fields_ = [("alloc", c_void_p), ("free", c_void_p), ("ctx", c_void_p)]


# --- config.h --------------------------------------------------------------
class wr_stream_config(Structure):
    """⚠ Two of the configuration bits change the WIRE FORMAT, not just the
    content, and 0x7e is the only configuration under which the relative-angle
    cancellation was ever measured."""

    _fields_ = [
        ("bits", c_uint8),
        ("legacy", c_uint8),
        ("reserved", c_uint8 * 2),
        ("justification", c_char * WR_CONFIG_JUSTIFICATION_MAX),
    ]


# --- device.h --------------------------------------------------------------
class wr_device_info(Structure):
    """⚠ ``mac`` and ``serial`` name a specific unit and a specific owner.  Treat
    them as personal data; the library redacts them from its own formatting and
    from the wire log unless asked otherwise."""

    _fields_ = [
        ("hardware_major", c_uint8),
        ("hardware_minor", c_uint8),
        ("protocol_major", c_uint8),
        ("protocol_minor", c_uint8),
        ("firmware_major", c_uint8),
        ("firmware_minor", c_uint8),
        ("product_id", c_uint8),
        # ⚠ The count is the 0x84 reply's LENGTH, not a byte in it; the bytes
        # are per-sensor location codes (0 → 0.00 m, 1 → 0.10 m, 2 → 0.26 m).
        ("sensor_count", c_uint8),
        ("sensor_location", c_uint8 * WR_SENSOR_LOCATION_MAX),
        ("battery_percent", c_uint8),
        ("status_undecoded", c_uint16),
        ("mac", c_char * WR_MAC_STRING_MAX),
        ("serial", c_char * WR_SERIAL_MAX),
        ("valid", c_uint32),
    ]


# --- sample.h --------------------------------------------------------------
class wr_unit_sample(Structure):
    """One unit's reading.

    ⚠ ``q_world_to_body_raw`` MAPS WORLD → BODY, which is the CONJUGATE of the
    convention most IMU code assumes.  Feeding these into relative-rotation code
    you already have is the reachable mistake, and it reaches past every warning
    the library carries because it never calls the blessed function.  The angle
    is identical under both conventions, so the obvious validation cannot catch
    it.  Use ``wr_quat_relative()``.

    ⚠ ``linear_accel_raw`` is GRAVITY-REMOVED and reads ≈0 at rest.

    ⚠ The RAW counts are authoritative.  The scaled forms depend on the
    configuration, and a recording read back under the wrong gyro divisor is
    silently wrong by 2×."""

    _fields_ = [
        ("q_world_to_body_raw", c_int16 * 4),  # w, x, y, z
        ("linear_accel_raw", c_int16 * 3),
        ("gyro_raw", c_int16 * 3),
        ("ticks_raw", c_uint16),
        ("has_ticks", c_uint8),
        ("pinned_mask", c_uint8),
        ("q_world_to_body", c_float * 4),
        ("linear_accel_mps2", c_float * 3),
        ("gyro_dps", c_float * 3),
        ("device_time_us", wr_time_us),
    ]


class wr_sample(Structure):
    """One sample, live and history alike.

    ⚠ ``calibration`` is resolved from the instant the sample was CAPTURED, not
    from when it was handed over.  ⚠ ``precision_us`` is the number to gate on;
    ``uncertainty_us`` is the honest total for provenance.  Gating on the total
    refuses every pull after the first second of a session.

    ⚠ THERE IS DELIBERATELY NO AGGREGATE ACROSS THE TWO UNITS.  They sit 3-8 cm
    apart, so under rotation their linear accelerations differ by roughly ω²r —
    several g through a golf swing, legitimately.  Do not average them, and do
    not read a disagreement as a fault."""

    _fields_ = [
        ("stream_id", c_uint64),
        ("sample_index", c_uint32),
        ("sample_index_raw", c_uint16),
        ("source", c_uint8),  # SampleSource
        ("calibration", c_uint8),  # CalibrationState, at the instant of capture
        ("flags", c_uint16),  # SampleFlag
        ("config_bits", c_uint8),
        ("reserved0", c_uint8),
        ("skew_us", c_int32),  # palm − lower_arm
        ("host_time_us", wr_time_us),
        ("host_recv_us", wr_time_us),  # WR_TIME_UNKNOWN for history records
        ("precision_us", c_uint32),
        ("uncertainty_us", c_uint32),
        ("lower_arm", wr_unit_sample),  # wire block 0
        ("palm", wr_unit_sample),  # wire block 1
    ]


class wr_pinned_counts(Structure):
    _fields_ = [
        ("n", (c_uint32 * WR_CHANNEL_COUNT) * WR_UNIT_COUNT),
        ("total", c_uint32),
    ]


# --- clock.h ---------------------------------------------------------------
class wr_clock_snapshot(Structure):
    """⚠ WHILE ``span_us`` IS 0 THE RESIDUAL FIGURES CARRY THE CONNECTION'S LAST
    REAL MEASUREMENT rather than reading zero.  A fit resting on a single instant
    has no spread to measure — one point cannot disagree with a line through it —
    and a zero there is an absence of evidence, not an absence of error."""

    _fields_ = [
        ("stream_id", c_uint64),
        ("flags", c_uint32),  # ClockFlag
        ("observations", c_int32),
        ("anchor_index", c_uint32),
        ("reserved0", c_uint32),
        ("anchor_host_us", wr_time_us),
        ("slope_us_per_index", c_double),
        ("fitted_rate_hz", c_double),  # ~799.2, NOT 800
        ("raw_fitted_rate_hz", c_double),
        ("external_ppm", c_double),
        ("offset_us", wr_time_us),
        ("span_us", wr_time_us),
        ("last_observation_us", wr_time_us),
        ("first_index", c_uint32),
        ("last_index", c_uint32),
        ("residual_median_us", c_uint32),
        ("residual_p90_us", c_uint32),
        ("residual_max_us", c_uint32),
        ("accuracy_drift_us_per_s", c_double),
        ("provenance", c_char * WR_PROVENANCE_MAX),
    ]


class wr_clock_error(Structure):
    """⚠ PRECISION AND ACCURACY ARE DIFFERENT NUMBERS.  Gate on ``precision_us``;
    record ``total_us``.  Merging them once made the library's headline feature
    refuse every pull after one second of session."""

    _fields_ = [
        ("precision_us", c_uint32),
        ("systematic_us", c_uint32),
        ("total_us", c_uint32),
    ]


class wr_clock_correction(Structure):
    """⚠ ``fields`` is REQUIRED and a zero bitmask is an error.  A field you do
    not flag is left as it was, and a FLAGGED ZERO IS A MEASURED ZERO."""

    _fields_ = [
        ("fields", c_uint32),  # CorrectionField
        ("reserved", c_uint32),
        ("rate_ppm", c_double),
        ("residual_drift_us_per_s", c_double),
        ("provenance", c_char * WR_PROVENANCE_MAX),
    ]


# --- event.h ---------------------------------------------------------------
class wr_event_payload(Union):
    """⚠ THE PAYLOAD IS NOT DECODED HERE YET.  ``wr_event_format()`` renders any
    event to one line with identifiers redacted, which is what the binding uses.
    Adding a typed member means adding it to ``tools/wr_abi_table.c`` too, or it
    is a layout nothing compares.

    ``clock`` is declared because it is the largest real member and therefore
    what sets the union's alignment; ``reserved_abi`` is the deliberate ABI
    reservation that sets its size.  Both are genuine C members — a padding
    member invented to fix the alignment would be a lie in a file whose whole
    purpose is fidelity."""

    _fields_ = [
        ("clock", wr_clock_snapshot),
        ("mtu", c_int32),
        ("reserved_abi", c_uint8 * WR_EVENT_PAYLOAD_MAX),
    ]


class wr_event(Structure):
    _fields_ = [
        ("type", c_uint16),
        ("reserved", c_uint16),
        ("sequence", c_uint32),  # gaps mean the queue overflowed
        ("host_time_us", wr_time_us),
        ("stream_id", c_uint64),
        ("u", wr_event_payload),
    ]


# --- history.h -------------------------------------------------------------
class wr_history_request(Structure):
    """⚠ ``alignment_budget_us`` of 0 disables the QUALITY gate and keeps the
    pull — that is the intended way to ask for data with no alignment claim.  It
    does not make an unaddressable window addressable."""

    _fields_ = [
        ("window", wr_time_range),
        ("deadline_us", wr_time_us),
        ("refill_gaps", c_bool),
        ("max_attempts", c_uint16),  # 0 → library default (3)
        ("alignment_budget_us", c_uint32),
        ("user_tag", c_uint64),
    ]


class wr_gap(Structure):
    _fields_ = [
        ("span", wr_time_range),
        ("indices", wr_index_range),
        ("kind", c_uint8),  # GapKind
        ("reserved", c_uint8 * 7),
    ]


class wr_calibration_span(Structure):
    """⚠ THE ONE PLACE ``LOST`` IS EVER WRITTEN.  ``state_at_start = CALIBRATED``
    with ``state_at_end = LOST`` says the calibration went away INSIDE this
    block, which is a different claim from "there never was one"."""

    _fields_ = [
        ("state_at_start", c_uint8),
        ("state_at_end", c_uint8),
        ("spans_transition", c_uint8),
        ("reserved", c_uint8),
        ("presence_angle_deg", c_float),  # NaN if never taken
    ]


class wr_history_block(Structure):
    """⚠ THE BLOCK OWNS ITS POINTERS AND THEY DIE WITH IT.  ``samples``,
    ``delivered``, ``delivered_indices`` and ``gaps`` are library memory freed by
    ``wr_history_block_release()``.  Use ``session.HistoryBlock``, which is a
    context manager, rather than touching these directly.

    ⚠ THREE NUMBERS THAT ANSWER THREE DIFFERENT QUESTIONS, and none can stand in
    for another:

      coverage_fraction  how much of what you ASKED FOR arrived
      density            how closely spaced what arrived actually was
      achieved_hz        the AVERAGE rate across the span it reached

    ``density`` is a SPACING — 1 / the median gap between consecutive delivered
    indices.  1.0 is index step 1, the full rate; 0.125 is step 8, what a still
    wrist returns.  0.0 means NOT MEASURABLE, never "empty".  ⚠ It is not
    ``achieved_hz`` normalised: a reply that is two dense runs with one hole
    averages to half rate while its spacing really was step 1.

    ⚠ ``live_overlap_samples == 0`` means NO EVIDENCE, not agreement.  Never read
    the mismatch count without the sample count beside it."""

    _fields_ = [
        ("layout_version", c_uint32),
        ("sample_stride", c_uint32),
        ("samples", POINTER(wr_sample)),
        ("sample_count", c_size_t),
        ("status", c_uint8),  # HistoryStatus
        ("attempts", c_uint8),
        ("coverage_overflowed", c_uint8),  # ⚠ the gap list is then OPTIMISTIC
        ("reserved0", c_uint8 * 5),
        ("stream_id", c_uint64),
        ("user_tag", c_uint64),
        ("requested", wr_time_range),
        ("requested_indices", wr_index_range),
        ("delivered", POINTER(wr_time_range)),
        ("delivered_count", c_size_t),
        ("delivered_indices", POINTER(wr_index_range)),
        ("gaps", POINTER(wr_gap)),
        ("gap_count", c_size_t),
        ("coverage_fraction", c_double),
        ("density", c_double),
        ("achieved_hz", c_double),
        ("largest_gap_us", c_uint32),
        ("reserved1", c_uint32),
        ("live_overlap_samples", c_uint32),
        ("live_overlap_mismatches", c_uint32),
        ("self_recording_gap", wr_time_range),  # the hole this pull itself caused
        ("fit", wr_clock_snapshot),
        ("calibration", wr_calibration_span),
        ("config", wr_stream_config),
        ("reserved2", c_uint8 * 7),
        ("pinned", wr_pinned_counts),
        ("requested_at_us", wr_time_us),
        ("completed_at_us", wr_time_us),
    ]


# --- session.h -------------------------------------------------------------
class wr_write_request(Structure):
    """⛔ EVERY BYTE THE LIBRARY CAN EMIT IS ON A SHORT REVIEWABLE ALLOWLIST, and
    these are the only bytes a transport ever writes.  There is no sendRaw(),
    because `f0` reboots the device into firmware-update mode through the
    ORDINARY data characteristic."""

    _fields_ = [
        ("data", c_uint8 * WR_MAX_COMMAND_LEN),
        ("length", c_uint8),
        ("without_response", c_uint8),
        ("reserved", c_uint8 * 2),
    ]


class wr_wire_chunk(Structure):
    _fields_ = [
        ("host_time_us", wr_time_us),
        ("length", c_uint16),
        ("direction", c_uint8),  # WireDirection
        ("flags", c_uint8),  # WireFlag
        ("sequence", c_uint32),
        ("data", c_uint8 * WR_WIRE_CHUNK_MAX),
    ]


class wr_session_policy(Structure):
    """Every field: 0 selects the measured-good default.

    ⚠ ``keepalive_period_us`` is not disableable.  A silent connection drops at
    exactly 5.0 minutes and AN ACTIVE STREAM DOES NOT PREVENT IT — what resets
    the device's timer is a host→device write.

    ⚠ ``accuracy_drift_us_per_s`` is the one field where 0 means "use the
    default" while the same-named knob on ``wr_clock_correction`` treats a
    flagged 0 as a measured zero.  This one is a starting assumption; that one is
    a measurement."""

    _fields_ = [
        ("keepalive_period_us", wr_time_us),
        ("calibration_raise_limit_us", wr_time_us),
        ("calibration_result_timeout_us", wr_time_us),
        ("bringup_timeout_us", wr_time_us),
        ("stream_start_timeout_us", wr_time_us),
        ("keepalive_alarm_us", wr_time_us),
        ("live_gap_alarm_us", wr_time_us),
        ("pinned_report_period_us", wr_time_us),
        ("history_pre_roll_us", wr_time_us),
        ("history_post_roll_us", wr_time_us),
        ("accuracy_drift_us_per_s", c_double),
        ("residual_alarm_us", c_uint32),
        ("clock_event_period_us", wr_time_us),
        ("record_identifiers", c_bool),
    ]


class wr_live_digest(Structure):
    _fields_ = [
        ("sample_index", c_uint32),
        ("reserved", c_uint32),
        ("digest", c_uint64),
    ]


class wr_session_memory(Structure):
    """⚠ TWO SEPARATE QUESTIONS, TWO SEPARATE FIELDS.  The capacity says what you
    want (0 = off, or the recommended default — each field differs); the pointer
    says who owns it (NULL = the library allocates).

    ⚠ ``wire_ring_capacity`` and ``digest_ring_capacity`` default to OFF, and
    with the digest ring off a block reports ``live_overlap_samples = 0``, which
    means NO EVIDENCE rather than agreement.

    The binding leaves every pointer NULL and sets capacities only, so there is
    no Python object whose lifetime has to outlive the session."""

    _fields_ = [
        ("live_ring", c_void_p),
        ("live_ring_capacity", c_size_t),
        ("event_ring", c_void_p),
        ("event_ring_capacity", c_size_t),
        ("wire_ring", c_void_p),
        ("wire_ring_capacity", c_size_t),
        ("history_gather", c_void_p),
        ("history_gather_capacity", c_size_t),
        ("coverage_storage", c_void_p),
        ("coverage_capacity", c_size_t),
        ("digest_ring", c_void_p),
        ("digest_ring_capacity", c_size_t),
    ]


class wr_session_config(Structure):
    _fields_ = [
        ("stream_config", wr_stream_config),
        ("policy", wr_session_policy),
        ("memory", wr_session_memory),
        ("allocator", wr_allocator),
        ("device_id", c_char * WR_DEVICE_ID_MAX),  # ⚠ opaque label, never a MAC
    ]


# --- version.h -------------------------------------------------------------
class wr_abi_sizes(Structure):
    _fields_ = [
        ("abi_version", c_uint32),
        ("sample", c_uint32),
        ("unit_sample", c_uint32),
        ("event", c_uint32),
        ("clock_snapshot", c_uint32),
        ("history_block", c_uint32),
        ("write_request", c_uint32),
        ("wire_chunk", c_uint32),
        ("session_config", c_uint32),
        ("sample_layout_version", c_uint32),
    ]


# --- record.h --------------------------------------------------------------
class wr_recording_info(Structure):
    _fields_ = [
        ("device_id", c_char * WR_DEVICE_ID_MAX),
        ("config_bits", c_uint8),
        ("config_legacy", c_uint8),
        ("reserved", c_uint8 * 2),
        ("layout_version", c_uint32),
        ("clock", c_char * WR_RECORD_CLOCK_MAX),
        ("identifiers_recorded", c_bool),
    ]


# ⚠ The set the layout test walks.  A struct declared above and missing here is
# a struct nothing compares against the compiler, so keep them in step — the
# test asserts this list covers every Structure/Union defined in this module.
PINNED_STRUCTS = {
    "wr_uuid": wr_uuid,
    "wr_index_range": wr_index_range,
    "wr_time_range": wr_time_range,
    "wr_allocator": wr_allocator,
    "wr_stream_config": wr_stream_config,
    "wr_device_info": wr_device_info,
    "wr_unit_sample": wr_unit_sample,
    "wr_sample": wr_sample,
    "wr_pinned_counts": wr_pinned_counts,
    "wr_clock_snapshot": wr_clock_snapshot,
    "wr_clock_error": wr_clock_error,
    "wr_clock_correction": wr_clock_correction,
    "wr_event": wr_event,
    "wr_history_request": wr_history_request,
    "wr_gap": wr_gap,
    "wr_calibration_span": wr_calibration_span,
    "wr_history_block": wr_history_block,
    "wr_write_request": wr_write_request,
    "wr_wire_chunk": wr_wire_chunk,
    "wr_session_policy": wr_session_policy,
    "wr_live_digest": wr_live_digest,
    "wr_session_memory": wr_session_memory,
    "wr_session_config": wr_session_config,
    "wr_abi_sizes": wr_abi_sizes,
    "wr_recording_info": wr_recording_info,
}
