# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""Python bindings for libwrist — the sans-I/O library for HackMotion wrist sensors.

    import wrist

    with wrist.Session(device_id="bench") as s:
        s.on_link_up(mtu, now_us)
        s.start_stream()
        ...
        s.on_bytes(payload, arrival_us)      # one call, one notification
        s.tick(now_us)
        for w in s.poll_writes():            # the ONLY bytes that go out
            characteristic.write(w.data)

⚠ THE LIBRARY DOES NOT OWN A RADIO, A THREAD, A TIMER OR A CLOCK, and this
binding does not add any.  The host supplies `now_us` from a MONOTONIC clock —
``time.monotonic_ns() // 1000``.  A wall clock will be stepped by NTP and will
corrupt a capture in a way that looks like a sensor fault.

⚠ ONE THREAD.  Every call on a session must come from the same thread for the
whole life of that session; there are no locks, no atomics and no threads
anywhere in the library.  With asyncio that means the event loop thread —
``asyncio.to_thread`` is for blocking on the user, never for calling in here.

⚠ THE RADIO IS NOT IN HERE AND IS NOT IMPORTED.
``wrist.bleak_transport`` is an OPTIONAL reference transport over `bleak`;
importing this package does not import it, and `bleak` is not a dependency of
the binding.  api-request §2.0 is the reason and it is a blocker rather than a
preference: a consumer that already owns a cross-platform BLE stack cannot embed
a library that brings a second one contending for the same adapter.
``wrist.device`` is the other half of that split — the discovery and GATT
constants a host needs to run its OWN scanner, read straight out of the library.

⛔ SAFETY.  Command `f0` reboots the sensor into firmware-update mode through the
ORDINARY data characteristic, so avoiding the OTA service is not sufficient.
There is no way to send an arbitrary command through this binding: the only
bytes that exist came out of ``poll_writes()``, and the library composed them
against the allowlist in ``src/wr_command.c``.  Do not add one.
"""

from __future__ import annotations

from ._library import (
    ABI_VERSION,
    VERSION,
    AbiMismatch,
    WristError,
    LibraryNotFound,
    library_path,
)
from ._types import (
    WR_CONFIG_OBSERVED_DEFAULT,
    WR_DIGEST_RING_RECOMMENDED,
    WR_MIN_ATT_MTU,
    WR_TIME_NEVER,
    WR_TIME_UNKNOWN,
    WR_WIRE_RING_RECOMMENDED,
    CalibrationAbortReason,
    CalibrationPhase,
    CalibrationState,
    Channel,
    ClockFlag,
    ConfigMask,
    CorrectionField,
    DeviceInfoField,
    EventType,
    GapKind,
    HistoryStatus,
    LinkDownCause,
    RecoveryAdvice,
    SampleFlag,
    SampleSource,
    Status,
    Unit,
    WireDirection,
    WireFlag,
)
from .device import (
    UUID_DATA_CHARACTERISTIC,
    UUID_ISSC_PIPE_INERT,
    UUID_OTA_SERVICE_FORBIDDEN,
    UUID_TRANSPARENT_UART_SERVICE,
    looks_like_sensor,
    parse_uuid,
    uuid_str,
)
from .record import Recorder, Replay
from .session import (
    Event,
    Gap,
    HistoryBlock,
    Samples,
    Session,
    WriteRequest,
    clock_error_at,
    clock_to_host_us,
    command_allowlist,
    command_is_allowed,
    gap_kind_name,
    history_status_name,
    relative_angle_deg,
    request_around,
    warning_code_name,
)

__all__ = [
    "ABI_VERSION",
    "VERSION",
    "AbiMismatch",
    "CalibrationAbortReason",
    "CalibrationPhase",
    "CalibrationState",
    "Channel",
    "ClockFlag",
    "ConfigMask",
    "CorrectionField",
    "DeviceInfoField",
    "Event",
    "EventType",
    "Gap",
    "GapKind",
    "WR_CONFIG_OBSERVED_DEFAULT",
    "WR_DIGEST_RING_RECOMMENDED",
    "WR_MIN_ATT_MTU",
    "WR_TIME_NEVER",
    "WR_TIME_UNKNOWN",
    "WR_WIRE_RING_RECOMMENDED",
    "WristError",
    "HistoryBlock",
    "HistoryStatus",
    "LibraryNotFound",
    "LinkDownCause",
    "Recorder",
    "RecoveryAdvice",
    "Replay",
    "SampleFlag",
    "SampleSource",
    "Samples",
    "Session",
    "Status",
    "UUID_DATA_CHARACTERISTIC",
    "UUID_ISSC_PIPE_INERT",
    "UUID_OTA_SERVICE_FORBIDDEN",
    "UUID_TRANSPARENT_UART_SERVICE",
    "Unit",
    "WireDirection",
    "WireFlag",
    "WriteRequest",
    "clock_error_at",
    "clock_to_host_us",
    "command_allowlist",
    "command_is_allowed",
    "gap_kind_name",
    "history_status_name",
    "library_path",
    "looks_like_sensor",
    "parse_uuid",
    "relative_angle_deg",
    "request_around",
    "uuid_str",
    "warning_code_name",
]

__version__ = VERSION
