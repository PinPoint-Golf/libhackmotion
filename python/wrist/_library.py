# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""Finding, loading and declaring the shared object.

The library is ``libwrist_ffi`` — ONE shared object holding the sans-I/O
core and the `.wrwire` container, built by the ``wrist_ffi`` target.  It is
a separate target from ``wrist`` on purpose: the core must not open files
and a purity test enforces that, while a binding needs the container too.

⚠ EVERY PROTOTYPE IS DECLARED.  ctypes defaults an undeclared function to
returning ``int``, which silently truncates a pointer or a 64-bit time on a
64-bit host — a crash if you are lucky and a wrong timestamp if you are not.
So a function that is not in ``_PROTOTYPES`` below is not callable through this
module, deliberately.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import (
    CDLL,
    POINTER,
    c_bool,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_int16,
    c_int32,
    c_size_t,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path

from . import _types as T
from ._types import wr_time_us

__all__ = ["lib", "library_path", "WristError", "check", "LibraryNotFound"]


class LibraryNotFound(RuntimeError):
    """The shared object could not be located or loaded."""


class AbiMismatch(RuntimeError):
    """The loaded library was built from different headers than this binding.

    ⚠ Raised at import, which is the whole point: a binding that guesses wrong
    about a struct returns plausible numbers with no error anywhere."""


class WristError(RuntimeError):
    """A library call returned a negative ``wr_status``."""

    def __init__(self, status: int, what: str = ""):
        self.status = T.Status(status)
        self.detail = lib.wr_status_str(status).decode()
        super().__init__(f"{what or 'call'} failed: {self.detail} ({self.status.name})")


def check(status: int, what: str = "") -> int:
    """⚠ NEGATIVE IS FAILURE; ``PENDING`` and ``DONE`` ARE NOT ERRORS.

    Returns the status so a caller can distinguish OK from PENDING/DONE.
    """
    if status < T.Status.OK:
        raise WristError(status, what)
    return status


# ---------------------------------------------------------------------------
# Locating the shared object
# ---------------------------------------------------------------------------
def _candidate_names() -> list[str]:
    if sys.platform == "darwin":
        return ["libwrist_ffi.dylib"]
    if sys.platform == "win32":
        return ["wrist_ffi.dll", "libwrist_ffi.dll"]
    return ["libwrist_ffi.so", "libwrist_ffi.so.0"]


def _search_paths() -> list[Path]:
    """⚠ Ordered so an explicit choice always wins over a discovered one.

    ``WRIST_LIBRARY`` first, because pointing the binding at a specific
    build — a sanitizer build, a release build, one under test — is a thing
    somebody does deliberately and should never have to fight a search order for.
    """
    here = Path(__file__).resolve().parent
    repo = here.parent.parent  # python/wrist/ -> python/ -> repo root
    return [
        here,  # bundled beside the package (a wheel, one day)
        repo / "build" / "dev",
        repo / "build" / "rel",
        repo / "build" / "san",
    ]


def _load() -> tuple[CDLL, str]:
    explicit = os.environ.get("WRIST_LIBRARY")
    if explicit:
        path = Path(explicit)
        if not path.exists():
            raise LibraryNotFound(f"WRIST_LIBRARY={explicit} does not exist")
        return CDLL(str(path)), str(path)

    tried: list[str] = []
    for directory in _search_paths():
        for name in _candidate_names():
            path = directory / name
            tried.append(str(path))
            if path.exists():
                return CDLL(str(path)), str(path)

    # Last resort: let the platform loader look on its own search path.
    for name in _candidate_names():
        try:
            return CDLL(name), name
        except OSError:
            tried.append(f"{name} (loader search path)")

    raise LibraryNotFound(
        "cannot find libwrist_ffi.  Build it with\n"
        "    cmake -S . -B build/dev -DCMAKE_BUILD_TYPE=Debug\n"
        "    cmake --build build/dev -j\n"
        "or set WRIST_LIBRARY to its path.  Tried:\n  "
        + "\n  ".join(tried)
    )


lib, library_path = _load()

# ---------------------------------------------------------------------------
# Prototypes
#
# (restype, [argtypes]) per symbol.  A `None` restype is C `void`.
# ---------------------------------------------------------------------------
_session_p = c_void_p  # opaque wr_session *
_recorder_p = c_void_p  # opaque wr_recorder *
_replay_p = c_void_p  # opaque wr_replay *
_status = c_int  # wr_status is an enum: int-sized

_PROTOTYPES: dict[str, tuple[object, list]] = {
    # --- version.h ---------------------------------------------------------
    "wr_version_string": (c_char_p, []),
    "wr_abi_version": (c_uint32, []),
    "wr_abi_sizes_get": (None, [POINTER(T.wr_abi_sizes)]),
    "wr_abi_check": (_status, [POINTER(T.wr_abi_sizes)]),
    # --- types.h -----------------------------------------------------------
    "wr_status_str": (c_char_p, [_status]),
    "wr_uuid_parse": (_status, [c_char_p, POINTER(T.wr_uuid)]),
    "wr_uuid_format": (c_char_p, [POINTER(T.wr_uuid), c_char_p, c_size_t]),
    "wr_uuid_equal": (c_bool, [POINTER(T.wr_uuid), POINTER(T.wr_uuid)]),
    # --- config.h ----------------------------------------------------------
    "wr_stream_config_default": (T.wr_stream_config, []),
    "wr_stream_config_legacy": (T.wr_stream_config, []),
    "wr_stream_config_nonstandard": (T.wr_stream_config, [c_uint8, c_char_p]),
    "wr_stream_config_has_ticks": (c_bool, [T.wr_stream_config]),
    "wr_stream_config_block_size": (c_size_t, [T.wr_stream_config]),
    "wr_stream_config_record_size": (c_size_t, [T.wr_stream_config]),
    "wr_stream_config_gyro_divisor": (c_int, [T.wr_stream_config]),
    "wr_stream_config_gyro_full_scale_dps": (c_double, [T.wr_stream_config]),
    "wr_stream_config_is_time_alignable": (c_bool, [T.wr_stream_config]),
    "wr_stream_config_is_observed_default": (c_bool, [T.wr_stream_config]),
    "wr_stream_config_is_legacy": (c_bool, [T.wr_stream_config]),
    "wr_stream_config_describe": (c_char_p, [T.wr_stream_config, c_char_p, c_size_t]),
    # --- device.h ----------------------------------------------------------
    "wr_looks_like_sensor": (c_bool, [c_char_p, POINTER(T.wr_uuid), c_size_t]),
    # --- sample.h ----------------------------------------------------------
    "wr_sample_unit": (POINTER(T.wr_unit_sample), [POINTER(T.wr_sample), c_int]),
    "wr_unit_name": (c_char_p, [c_int]),
    "wr_channel_name": (c_char_p, [c_int]),
    "wr_pinned_counts_reset": (None, [POINTER(T.wr_pinned_counts)]),
    "wr_pinned_counts_add": (
        None,
        [POINTER(T.wr_pinned_counts), POINTER(T.wr_sample)],
    ),
    # --- quat.h ------------------------------------------------------------
    "wr_quat_conjugate": (None, [POINTER(c_float), POINTER(c_float)]),
    "wr_quat_multiply": (None, [POINTER(c_float), POINTER(c_float), POINTER(c_float)]),
    "wr_quat_normalise": (None, [POINTER(c_float)]),
    "wr_quat_dot": (c_float, [POINTER(c_float), POINTER(c_float)]),
    "wr_quat_relative": (None, [POINTER(T.wr_sample), POINTER(c_float)]),
    "wr_relative_angle_deg": (c_float, [POINTER(T.wr_sample)]),
    "wr_angular_rate_dps": (
        c_float,
        [POINTER(c_float), POINTER(c_float), wr_time_us, POINTER(c_float)],
    ),
    "wr_quat_raw_norm": (c_float, [POINTER(c_int16)]),
    # --- clock.h -----------------------------------------------------------
    "wr_clock_to_host_us": (wr_time_us, [POINTER(T.wr_clock_snapshot), c_uint32]),
    "wr_clock_index_for_host_us": (
        _status,
        [POINTER(T.wr_clock_snapshot), wr_time_us, POINTER(c_uint32)],
    ),
    "wr_clock_error_at": (
        T.wr_clock_error,
        [POINTER(T.wr_clock_snapshot), c_uint32],
    ),
    "wr_clock_precision_us": (c_uint32, [POINTER(T.wr_clock_snapshot), c_uint32]),
    "wr_clock_uncertainty_us": (c_uint32, [POINTER(T.wr_clock_snapshot), c_uint32]),
    "wr_clock_meets_budget": (
        c_bool,
        [POINTER(T.wr_clock_snapshot), c_uint32, c_uint32],
    ),
    "wr_clock_index_range_for_time": (
        _status,
        [POINTER(T.wr_clock_snapshot), T.wr_time_range, POINTER(T.wr_index_range)],
    ),
    "wr_clock_flag_name": (c_char_p, [c_int]),
    # --- event.h -----------------------------------------------------------
    "wr_event_type_name": (c_char_p, [c_int]),
    "wr_warning_code_name": (c_char_p, [c_int]),
    "wr_calibration_phase_name": (c_char_p, [c_int]),
    "wr_link_down_cause_name": (c_char_p, [c_int]),
    "wr_recovery_advice_name": (c_char_p, [c_int]),
    "wr_event_is_sensitive": (c_bool, [POINTER(T.wr_event)]),
    "wr_event_format": (c_int, [POINTER(T.wr_event), c_char_p, c_size_t, c_bool]),
    # --- history.h ---------------------------------------------------------
    "wr_history_request_around": (
        T.wr_history_request,
        [POINTER(T.wr_session_policy), wr_time_us],
    ),
    "wr_history_status_name": (c_char_p, [c_int]),
    "wr_gap_kind_name": (c_char_p, [c_int]),
    "wr_sample_step_density": (c_double, [POINTER(T.wr_sample), c_size_t]),
    "wr_history_reserve": (
        _status,
        [_session_p, POINTER(T.wr_history_request), POINTER(c_uint64)],
    ),
    "wr_history_collect": (
        _status,
        [_session_p, c_uint64, POINTER(POINTER(T.wr_history_block))],
    ),
    "wr_history_cancel": (_status, [_session_p, c_uint64]),
    "wr_history_block_release": (None, [POINTER(T.wr_history_block)]),
    "wr_history_pending": (c_size_t, [_session_p]),
    "wr_history_resident_range": (_status, [_session_p, POINTER(T.wr_time_range)]),
    "wr_history_coverage_available": (
        c_bool,
        [_session_p, wr_time_us, wr_time_us],
    ),
    # --- session.h ---------------------------------------------------------
    "wr_command_is_allowed": (c_bool, [c_uint8]),
    "wr_command_allowlist": (c_size_t, [POINTER(c_uint8), c_size_t]),
    "wr_session_policy_default": (T.wr_session_policy, []),
    "wr_session_config_default": (T.wr_session_config, []),
    "wr_session_create": (
        _status,
        [POINTER(T.wr_session_config), POINTER(_session_p)],
    ),
    "wr_session_close": (None, [_session_p]),
    "wr_session_destroy": (None, [_session_p]),
    "wr_session_on_link_up": (None, [_session_p, c_int32, wr_time_us]),
    "wr_session_on_link_down": (None, [_session_p, c_int, wr_time_us]),
    "wr_session_on_bytes": (
        None,
        [_session_p, POINTER(c_uint8), c_size_t, wr_time_us],
    ),
    "wr_session_on_advertising_seen": (None, [_session_p, wr_time_us]),
    "wr_session_next_due_us": (wr_time_us, [_session_p]),
    "wr_session_tick": (None, [_session_p, wr_time_us]),
    "wr_session_poll_writes": (
        c_size_t,
        [_session_p, POINTER(T.wr_write_request), c_size_t],
    ),
    "wr_session_poll_events": (c_size_t, [_session_p, POINTER(T.wr_event), c_size_t]),
    "wr_session_poll_live": (c_size_t, [_session_p, POINTER(T.wr_sample), c_size_t]),
    "wr_session_poll_wire": (
        c_size_t,
        [_session_p, POINTER(T.wr_wire_chunk), c_size_t],
    ),
    "wr_session_dropped_live": (c_uint64, [_session_p]),
    "wr_session_dropped_events": (c_uint64, [_session_p]),
    "wr_session_dropped_wire": (c_uint64, [_session_p]),
    "wr_session_start_stream": (_status, [_session_p]),
    "wr_session_stop_stream": (_status, [_session_p]),
    "wr_session_is_streaming": (c_bool, [_session_p]),
    "wr_session_stream_id": (c_uint64, [_session_p]),
    "wr_session_power_off": (_status, [_session_p]),
    "wr_session_clock": (_status, [_session_p, POINTER(T.wr_clock_snapshot)]),
    "wr_session_device_info": (_status, [_session_p, POINTER(T.wr_device_info)]),
    "wr_session_stream_config": (T.wr_stream_config, [_session_p]),
    "wr_session_set_clock_correction": (
        _status,
        [_session_p, POINTER(T.wr_clock_correction)],
    ),
    "wr_session_calibration_state": (c_int, [_session_p]),
    "wr_calibration_begin": (_status, [_session_p]),
    "wr_calibration_confirm_horizontal": (_status, [_session_p]),
    "wr_calibration_confirm_raise": (_status, [_session_p]),
    "wr_calibration_confirm_reference_pose": (_status, [_session_p]),
    "wr_calibration_abort": (_status, [_session_p]),
    "wr_calibration_current_phase": (c_int, [_session_p]),
    "wr_calibration_presence_angle_deg": (c_float, [_session_p]),
    # --- record.h ----------------------------------------------------------
    "wr_recording_info_default": (T.wr_recording_info, []),
    "wr_recorder_open": (
        _status,
        [c_char_p, POINTER(T.wr_recording_info), POINTER(_recorder_p)],
    ),
    "wr_recorder_write": (_status, [_recorder_p, POINTER(T.wr_wire_chunk), c_size_t]),
    "wr_recorder_chunks": (c_uint64, [_recorder_p]),
    "wr_recorder_bytes": (c_uint64, [_recorder_p]),
    "wr_recorder_close": (_status, [_recorder_p]),
    "wr_replay_open": (_status, [c_char_p, POINTER(_replay_p)]),
    "wr_replay_info": (POINTER(T.wr_recording_info), [_replay_p]),
    "wr_replay_next": (_status, [_replay_p, POINTER(T.wr_wire_chunk)]),
    "wr_replay_chunks_read": (c_uint64, [_replay_p]),
    "wr_replay_close": (None, [_replay_p]),
}


def _declare() -> None:
    missing: list[str] = []
    for name, (restype, argtypes) in _PROTOTYPES.items():
        try:
            fn = getattr(lib, name)
        except AttributeError:
            missing.append(name)
            continue
        fn.restype = restype
        fn.argtypes = argtypes
    if missing:
        raise LibraryNotFound(
            f"{library_path} is missing {len(missing)} expected symbol(s): "
            + ", ".join(missing[:8])
            + ("..." if len(missing) > 8 else "")
            + "\n⚠ A partial shared object is a wrong one — it was probably built "
            "without the record module.  Rebuild the `wrist_ffi` target."
        )


_declare()


# ---------------------------------------------------------------------------
# The load-time guard
# ---------------------------------------------------------------------------
def _abi_guard() -> None:
    """⚠ FAIL AT LOAD, NOT AT RANDOM.

    ``wr_abi_check()`` compares the struct sizes this binding believes in against
    the ones the library was built with.  It is the reason a binding compiled
    against one set of headers cannot quietly misread a shared object built from
    another.

    ⚠ It covers NINE structs and only their sizes.  Field offsets are pinned
    separately and at development time, by ``tests/test_python_abi.py`` against
    ``wrwire abi``.  Neither check subsumes the other: this one runs on a user's
    machine against whatever they installed, that one runs against the compiler.
    """
    expected = T.wr_abi_sizes(
        abi_version=lib.wr_abi_version(),
        sample=ctypes.sizeof(T.wr_sample),
        unit_sample=ctypes.sizeof(T.wr_unit_sample),
        event=ctypes.sizeof(T.wr_event),
        clock_snapshot=ctypes.sizeof(T.wr_clock_snapshot),
        history_block=ctypes.sizeof(T.wr_history_block),
        write_request=ctypes.sizeof(T.wr_write_request),
        wire_chunk=ctypes.sizeof(T.wr_wire_chunk),
        session_config=ctypes.sizeof(T.wr_session_config),
        sample_layout_version=0,  # filled from the library, then compared below
    )

    actual = T.wr_abi_sizes()
    lib.wr_abi_sizes_get(ctypes.byref(actual))
    expected.sample_layout_version = actual.sample_layout_version

    if lib.wr_abi_check(ctypes.byref(expected)) != T.Status.OK:
        differences = [
            f"{field}: binding {getattr(expected, field)} vs library {getattr(actual, field)}"
            for field, _ in T.wr_abi_sizes._fields_
            if getattr(expected, field) != getattr(actual, field)
        ]
        raise AbiMismatch(
            f"{library_path} was built from different headers than this binding.\n"
            + ("\n".join("  " + d for d in differences) or "  (sizes agree; ABI version differs)")
        )


_abi_guard()

VERSION = lib.wr_version_string().decode()
ABI_VERSION = lib.wr_abi_version()
