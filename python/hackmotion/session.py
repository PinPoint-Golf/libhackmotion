# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""The session, wrapped.

⚠ THE HOST OWNS THE RADIO, THE THREAD, THE TIMER AND THE CLOCK.  This module
adds none of them.  The loop a host runs is:

    on transport bytes ......... session.on_bytes(payload, arrival_us)
    on link up/down ............ session.on_link_up / .on_link_down
    arm one timer to ........... session.next_due_us
    when it fires .............. session.tick(now_us)
    then always ................ drain poll_writes / poll_events / poll_live
                                 / poll_wire, and write poll_writes' bytes to
                                 the characteristic

⚠ ONE THREAD FOR THE WHOLE LIFE OF A SESSION.  There are no locks, no atomics
and no threads in the library, so this is not advice.  Under asyncio that means
the event loop thread.

⚠ AND DO NOT BLOCK THAT THREAD.  `now_us` and the arrival stamps are the entire
foundation of the clock fit.  A blocking `input()` on the loop once fabricated
4.08 s of arrival times in this project's own capture harness, and that capture
still needs a relaxed stream-start bound to replay at all.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Iterator, Sequence

from . import _types as T
from ._library import HackMotionError, check, lib

__all__ = [
    "Event",
    "Gap",
    "HistoryBlock",
    "Samples",
    "Session",
    "WriteRequest",
    "clock_error_at",
    "clock_to_host_us",
    "command_allowlist",
    "command_is_allowed",
    "gap_kind_name",
    "history_status_name",
    "relative_angle_deg",
    "request_around",
    "warning_code_name",
]

_EVENT_TEXT_MAX = 512


# ---------------------------------------------------------------------------
# Values handed out
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class WriteRequest:
    """One command for the transport to write, and the ONLY source of bytes.

    ⛔ The library composed these against the allowlist in `src/hm_command.c`.
    A transport writes exactly this and never synthesises a command of its own,
    not even a harmless-looking one: `f0` reboots the sensor into
    firmware-update mode through the ordinary data characteristic."""

    data: bytes
    without_response: bool


@dataclass(frozen=True)
class Event:
    """⚠ `text` is rendered by the library with identifiers REDACTED unless you
    ask otherwise, and `sensitive` is true for exactly the events carrying a MAC
    or a serial.  A log sink can filter on it without having to think.

    The payload union is not decoded yet; `raw` is a copy of the C struct for a
    caller that wants to reach into it, and `text` is what the library says."""

    type: T.EventType | int
    sequence: int
    host_time_us: int
    stream_id: int
    text: str
    sensitive: bool
    raw: T.hm_event

    def format(self, include_identifiers: bool = False) -> str:
        return _format_event(self.raw, include_identifiers)


@dataclass(frozen=True)
class Gap:
    """⚠ The three kinds MAY OVERLAP — read them as three independent statements
    about one index axis, not as a partition of it."""

    kind: T.GapKind | int
    start_us: int
    end_us: int
    first_index: int
    last_index: int


def _format_event(raw: T.hm_event, include_identifiers: bool) -> str:
    buf = ctypes.create_string_buffer(_EVENT_TEXT_MAX)
    lib.hm_event_format(ctypes.byref(raw), buf, len(buf), include_identifiers)
    return buf.value.decode(errors="replace")


# ---------------------------------------------------------------------------
# Free functions — the library's own, so nothing has to reach into `_library`
# ---------------------------------------------------------------------------
def clock_to_host_us(fit: T.hm_clock_snapshot, sample_index: int) -> int:
    """Map a sample index to host time through a fit.

    ⚠ Use the fit a history block CARRIES, not the session's current one. The
    mapping is piecewise: the device's sample counter stalls for the duration of
    every retrieval, so the fit re-anchors at each one and a block is dated by
    the fit that was in force when its window closed. That is why the block
    carries its own — it stays reproducible a year later."""
    return lib.hm_clock_to_host_us(ctypes.byref(fit), sample_index)


def clock_error_at(fit: T.hm_clock_snapshot, sample_index: int) -> T.hm_clock_error:
    """⚠ Gate on `precision_us`, record `total_us`.  Gating on the total refuses
    every pull after the first second of a session."""
    return lib.hm_clock_error_at(ctypes.byref(fit), sample_index)


def history_status_name(status) -> str:
    return lib.hm_history_status_name(int(status)).decode()


def gap_kind_name(kind) -> str:
    return lib.hm_gap_kind_name(int(kind)).decode()


def warning_code_name(code) -> str:
    """⚠ The binding keeps NO COPY of the warning list — this asks the library.
    One fewer second copy to drift."""
    return lib.hm_warning_code_name(int(code)).decode()


def relative_angle_deg(sample: T.hm_sample) -> float:
    """The wrist angle, in degrees, from one sample.

    ⚠ The blessed path. The streamed quaternions map WORLD → BODY, the conjugate
    of the convention most IMU code assumes, so composing them with relative-
    rotation code you already have is the reachable mistake — and the ANGLE is
    identical under both orders, so the obvious validation cannot catch it."""
    return lib.hm_relative_angle_deg(ctypes.byref(sample))


def command_is_allowed(command_byte: int) -> bool:
    """⛔ Always false for 0xf0, which reboots the sensor into firmware-update
    mode through the ordinary data characteristic."""
    return bool(lib.hm_command_is_allowed(command_byte))


def command_allowlist() -> bytes:
    """⛔ Every command byte this library can ever send — for a reviewer, a log
    line, or a transport's own assertion. There is no way to send anything
    else."""
    buf = (ctypes.c_uint8 * 32)()
    n = lib.hm_command_allowlist(buf, len(buf))
    return bytes(buf[: min(n, len(buf))])


class Samples(Sequence):
    """A window onto an array of `hm_sample`.

    ⚠ WHEN THIS COMES FROM A HISTORY BLOCK IT POINTS AT LIBRARY MEMORY AND DIES
    WITH THE BLOCK.  `HistoryBlock` is a context manager for that reason; if you
    need the samples to outlive it, call `.copy()` (or `.numpy().copy()`) first.

    ⚠ AND THE GUARD BELOW IS THE ONLY THING THAT CATCHES IT, WHICH IS WHY IT IS
    HERE RATHER THAN IN A COMMENT.  A sanitizer does not help: AddressSanitizer
    instruments the code it compiled, and a ctypes read happens inside CPython,
    which it did not.  Measured — an escaped view read after release returns
    plausible sample data under `build/san` with no report at all.  So every
    access asks the owning block whether it is still alive and raises if it is
    not, turning a silent wrong answer into a loud one.

    Samples from `poll_live()` are backed by Python-owned memory and have no
    owner and no such constraint."""

    __slots__ = ("_array", "_count", "_owner")

    def __init__(self, array, count: int, owner=None):
        self._array = array
        self._count = count
        # Holds the memory's owner alive, AND is asked before every read.
        self._owner = owner

    def _alive(self):
        if self._owner is not None and getattr(self._owner, "_released", False):
            raise RuntimeError(
                "these samples belonged to a history block that has been "
                "released; the memory was the library's and is gone.  Copy what "
                "you need — `.copy()` or `.numpy().copy()` — before leaving the "
                "`with` statement."
            )

    def __len__(self) -> int:
        self._alive()
        return self._count

    def __getitem__(self, index):
        self._alive()
        if isinstance(index, slice):
            return [self._array[i] for i in range(*index.indices(self._count))]
        if index < 0:
            index += self._count
        if not 0 <= index < self._count:
            raise IndexError(index)
        return self._array[index]

    def __iter__(self) -> Iterator:
        # ⚠ Checked once at entry rather than per element: the realistic misuse
        # is iterating a view long after the `with` closed, not releasing a block
        # from inside its own loop.
        self._alive()
        for i in range(self._count):
            yield self._array[i]

    def copy(self) -> "Samples":
        """A detached copy that outlives whatever produced it."""
        self._alive()
        out = (T.hm_sample * self._count)()
        ctypes.memmove(out, self._array, ctypes.sizeof(T.hm_sample) * self._count)
        return Samples(out, self._count)

    def numpy(self):
        """A ZERO-COPY structured view, when numpy is installed.

        ⚠ Zero-copy means numpy holds the raw address and this class's guard can
        no longer see it.  `.copy()` before the block is released — an ndarray
        that outlives its block is the one route left to a silent wrong answer."""
        # ⚠ The guard runs BEFORE the import.  Otherwise, on a machine without
        # numpy, using a released view reports a missing dependency — an error
        # about the wrong thing, at exactly the moment it matters most.
        self._alive()

        import numpy as np

        if self._count == 0:
            return np.empty(0, dtype=np.dtype([("stream_id", "<u8")]))
        return np.ctypeslib.as_array(self._array)[: self._count]

    def step_density(self) -> float:
        """1 / the median gap between consecutive delivered indices.

        ⚠ A SPACING, NOT A RATE, and not `achieved_hz` normalised.  1.0 is index
        step 1 — the full internal rate; 0.125 is step 8, what a still wrist
        returns.  0.0 means NOT MEASURABLE, never "empty"."""
        self._alive()
        return lib.hm_sample_step_density(self._array, self._count)


# ---------------------------------------------------------------------------
# History
# ---------------------------------------------------------------------------
def request_around(
    event_host_us: int,
    policy: T.hm_session_policy | None = None,
    *,
    deadline_us: int | None = None,
    max_attempts: int = 0,
    refill_gaps: bool = True,
    alignment_budget_us: int = 0,
    user_tag: int = 0,
) -> T.hm_history_request:
    """A request sized the way the vendor recommends around an event instant.

    ⚠ Do NOT request the whole buffer: an over-wide ask comes back holed rather
    than clamped."""
    req = lib.hm_history_request_around(
        ctypes.byref(policy) if policy is not None else None, event_host_us
    )
    if deadline_us is not None:
        req.deadline_us = deadline_us
    req.max_attempts = max_attempts
    req.refill_gaps = refill_gaps
    req.alignment_budget_us = alignment_budget_us
    req.user_tag = user_tag
    return req


class HistoryBlock:
    """One retrieval's result — and ⚠ ONE OWNING VALUE WITH A LIFETIME.

    The samples, the delivered intervals and the gap list are library memory
    freed by `hm_history_block_release()`.  Use it as a context manager:

        with session.history_collect(request_id) as block:
            if block is not None:
                arr = block.samples.numpy().copy()   # ⚠ copy to outlive it

    ⚠ THREE NUMBERS THAT ANSWER THREE DIFFERENT QUESTIONS, and no one of them
    can stand in for another:

      coverage_fraction  how much of what you ASKED FOR arrived
      density            how closely spaced what arrived actually was
      achieved_hz        the AVERAGE rate across the span it reached

    ⚠ `live_overlap_samples == 0` means NO EVIDENCE, not agreement: either the
    pull covered a span live never reached, or no digest ring was supplied.
    """

    __slots__ = ("_ptr", "_released")

    def __init__(self, ptr):
        self._ptr = ptr
        self._released = False

    # --- lifetime ---------------------------------------------------------
    def __enter__(self) -> "HistoryBlock":
        return self

    def __exit__(self, *exc) -> None:
        self.release()

    def release(self) -> None:
        if not self._released and self._ptr:
            lib.hm_history_block_release(self._ptr)
            self._released = True
            self._ptr = None

    def __del__(self) -> None:
        # A safety net, not the mechanism.  Reaching this means a block was left
        # to the garbage collector, which is fine but unbounded — the library
        # holds the memory until then.
        try:
            self.release()
        except Exception:  # pragma: no cover - interpreter teardown
            pass

    @property
    def _b(self) -> T.hm_history_block:
        if self._released:
            raise RuntimeError(
                "this history block has been released; its samples were library "
                "memory and are gone.  Copy what you need before leaving the "
                "`with` statement."
            )
        return self._ptr.contents

    # --- content ----------------------------------------------------------
    @property
    def status(self) -> T.HistoryStatus:
        return T.HistoryStatus(self._b.status)

    @property
    def samples(self) -> Samples:
        block = self._b
        if block.sample_count == 0 or not block.samples:
            return Samples((T.hm_sample * 0)(), 0, self)
        array = (T.hm_sample * block.sample_count).from_address(
            ctypes.addressof(block.samples.contents)
        )
        return Samples(array, block.sample_count, self)

    @property
    def gaps(self) -> list[Gap]:
        block = self._b
        return [
            Gap(
                kind=T.GapKind(block.gaps[i].kind),
                start_us=block.gaps[i].span.start_us,
                end_us=block.gaps[i].span.end_us,
                first_index=block.gaps[i].indices.first,
                last_index=block.gaps[i].indices.last,
            )
            for i in range(block.gap_count)
        ]

    @property
    def delivered(self) -> list[tuple[int, int]]:
        block = self._b
        return [
            (block.delivered[i].start_us, block.delivered[i].end_us)
            for i in range(block.delivered_count)
        ]

    @property
    def delivered_indices(self) -> list[tuple[int, int]]:
        block = self._b
        return [
            (block.delivered_indices[i].first, block.delivered_indices[i].last)
            for i in range(block.delivered_count)
        ]

    def __getattr__(self, name: str):
        """Scalars pass straight through — `coverage_fraction`, `density`,
        `achieved_hz`, `attempts`, `fit`, `pinned`, and the rest."""
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return getattr(self._b, name)
        except AttributeError:
            raise AttributeError(
                f"hm_history_block has no field {name!r}"
            ) from None

    def summary(self) -> str:
        """One line, in the shape this project insists on: ⚠ never a derived
        number without the raw one it came from beside it."""
        b = self._b
        median_step = f"step {round(1.0 / b.density)}" if b.density > 0.0 else "step ?"
        return (
            f"{lib.hm_history_status_name(b.status).decode():<10} "
            f"n={b.sample_count:<6} coverage={b.coverage_fraction:.3f} "
            f"density={b.density:.3f} ({median_step}) "
            f"achieved={b.achieved_hz:7.1f} Hz "
            f"gaps={b.gap_count:<4} largest={b.largest_gap_us // 1000:5d} ms  "
            f"overlap {b.live_overlap_mismatches}/{b.live_overlap_samples}  "
            f"attempts={b.attempts}"
        )


# ---------------------------------------------------------------------------
# The session
# ---------------------------------------------------------------------------
class Session:
    """⚠ Capacities only; the library owns every buffer.

    The C API lets a caller supply the rings, which is what an embedded host
    wants.  This binding deliberately does not: a Python object whose lifetime
    has to outlive a C session is a use-after-free waiting to be written, and
    the library makes exactly one allocation at create and one free at destroy
    either way.

    ⚠ `wire_ring` and `digest_ring` default to OFF, as they do in C.  With the
    digest ring off, every history block reports `live_overlap_samples = 0` —
    which means NO EVIDENCE, not agreement.
    """

    __slots__ = ("_handle", "_closed")

    def __init__(
        self,
        device_id: str = "",
        *,
        live_ring: int = 0,  # 0 → recommended
        event_ring: int = 0,  # 0 → recommended
        history_gather: int = 0,  # 0 → recommended
        coverage: int = 0,  # 0 → recommended
        wire_ring: int = 0,  # ⚠ 0 → OFF
        digest_ring: int = 0,  # ⚠ 0 → OFF
        policy: dict | None = None,
        stream_config: T.hm_stream_config | None = None,
    ):
        config = lib.hm_session_config_default()
        config.device_id = device_id.encode()[: T.HM_DEVICE_ID_MAX - 1]
        config.memory.live_ring_capacity = live_ring
        config.memory.event_ring_capacity = event_ring
        config.memory.history_gather_capacity = history_gather
        config.memory.coverage_capacity = coverage
        config.memory.wire_ring_capacity = wire_ring
        config.memory.digest_ring_capacity = digest_ring
        if stream_config is not None:
            config.stream_config = stream_config
        for field, value in (policy or {}).items():
            if not hasattr(config.policy, field):
                raise TypeError(f"hm_session_policy has no field {field!r}")
            setattr(config.policy, field, value)

        handle = ctypes.c_void_p()
        check(
            lib.hm_session_create(ctypes.byref(config), ctypes.byref(handle)),
            "hm_session_create",
        )
        self._handle = handle
        self._closed = False

    # --- lifetime ---------------------------------------------------------
    def __enter__(self) -> "Session":
        return self

    def __exit__(self, *exc) -> None:
        self.destroy()

    def close(self) -> None:
        """⚠ CLOSE FINISHES WORK, IT DOES NOT DISCARD IT.  Every outstanding
        reservation materialises here as cancelled with whatever arrived, so
        DRAIN ONCE MORE afterwards if you want those blocks: `poll_events()`
        still yields their ready events and `history_collect()` still hands them
        over, until `destroy()`.

        Live samples and wire chunks ARE dropped — bulk data you chose to
        abandon by closing."""
        if not self._closed and self._handle:
            lib.hm_session_close(self._handle)
            self._closed = True

    def destroy(self) -> None:
        if self._handle:
            lib.hm_session_destroy(self._handle)
            self._handle = ctypes.c_void_p()
            self._closed = True

    def __del__(self) -> None:
        try:
            self.destroy()
        except Exception:  # pragma: no cover - interpreter teardown
            pass

    # --- transport inputs -------------------------------------------------
    def on_link_up(self, negotiated_mtu: int, now_us: int) -> None:
        """⚠ Pass 0 if the platform will not tell you the MTU — that is treated
        as "unknown, proceed" with a warning.  Passing a number the platform
        did not actually give you is how a perfectly good link gets refused;
        BlueZ reports the 23-byte ATT default until something asks it properly,
        and reading that as a measurement is an ABSENT answer dressed up as a
        bad one."""
        lib.hm_session_on_link_up(self._handle, negotiated_mtu, now_us)

    def on_link_down(self, cause: T.LinkDownCause, now_us: int) -> None:
        """⚠ ALWAYS INVALIDATES CALIBRATION, and that is not expressible
        otherwise: 0.70° before dropping a link and 18.80° at the same pose
        after reconnecting, strap untouched."""
        lib.hm_session_on_link_down(self._handle, int(cause), now_us)

    def on_bytes(self, data: bytes, host_recv_us: int) -> None:
        """⚠ ONE CALL, ONE NOTIFICATION.  `data` is a complete ATT notification
        payload exactly as the transport delivered it.  This is NOT a byte
        stream and the library does not reassemble one — the protocol has no
        length field, no sequence number and no checksum, so a coalescing
        transport would corrupt silently with nothing to resynchronise on."""
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        lib.hm_session_on_bytes(self._handle, buf, len(data), host_recv_us)

    def on_advertising_seen(self, now_us: int) -> None:
        lib.hm_session_on_advertising_seen(self._handle, now_us)

    # --- the clock the host owns -----------------------------------------
    @property
    def next_due_us(self) -> int:
        """Absolute host time of the next thing the session wants to do, or
        `HM_TIME_NEVER`.  ⚠ Re-read after every call into the session."""
        return lib.hm_session_next_due_us(self._handle)

    def tick(self, now_us: int) -> None:
        lib.hm_session_tick(self._handle, now_us)

    # --- outputs ----------------------------------------------------------
    def poll_writes(self, max_count: int = 8) -> list[WriteRequest]:
        """⚠ Returns NOTHING AT ALL while a history retrieval is in flight, and
        that is deliberate — see the write quiet period in session.h.  A
        transport that keeps its own queue moving during a retrieval puts a
        reply into a record stream that cannot be resynchronised."""
        out = (T.hm_write_request * max_count)()
        n = lib.hm_session_poll_writes(self._handle, out, max_count)
        return [
            WriteRequest(
                data=bytes(out[i].data[: out[i].length]),
                without_response=bool(out[i].without_response),
            )
            for i in range(n)
        ]

    def poll_events(self, max_count: int = 32) -> list[Event]:
        out = (T.hm_event * max_count)()
        n = lib.hm_session_poll_events(self._handle, out, max_count)
        events = []
        for i in range(n):
            raw = T.hm_event.from_buffer_copy(out[i])
            try:
                kind = T.EventType(raw.type)
            except ValueError:
                kind = raw.type
            events.append(
                Event(
                    type=kind,
                    sequence=raw.sequence,
                    host_time_us=raw.host_time_us,
                    stream_id=raw.stream_id,
                    text=_format_event(raw, False),
                    sensitive=bool(lib.hm_event_is_sensitive(ctypes.byref(raw))),
                    raw=raw,
                )
            )
        return events

    def poll_live(self, max_count: int = 256) -> Samples:
        """⚠ Drain often.  There is NO LIVE RATE CEILING — dense bursts reach
        the full internal rate in every session containing motion — so size
        this from how often you drain, never from an assumed rate.  Whatever
        you fail to drain shows up in `dropped_live`."""
        out = (T.hm_sample * max_count)()
        n = lib.hm_session_poll_live(self._handle, out, max_count)
        return Samples(out, n)

    def poll_wire(self, max_count: int = 64) -> list[T.hm_wire_chunk]:
        out = (T.hm_wire_chunk * max_count)()
        n = lib.hm_session_poll_wire(self._handle, out, max_count)
        return [T.hm_wire_chunk.from_buffer_copy(out[i]) for i in range(n)]

    @property
    def dropped_live(self) -> int:
        """⚠ Non-zero means the host is not keeping up and data has been LOST.
        Report it; never let it be silent."""
        return lib.hm_session_dropped_live(self._handle)

    @property
    def dropped_events(self) -> int:
        return lib.hm_session_dropped_events(self._handle)

    @property
    def dropped_wire(self) -> int:
        return lib.hm_session_dropped_wire(self._handle)

    # --- control ----------------------------------------------------------
    def start_stream(self) -> None:
        """⚠ ONE STREAM, OPENED ONCE, LEFT OPEN.  Calibration needs it open, the
        clock fit needs it continuous, and a retrieval works in place — so
        nothing ever requires stopping it.  A restart drops calibration back to
        UNKNOWN."""
        check(lib.hm_session_start_stream(self._handle), "start_stream")

    def stop_stream(self) -> None:
        check(lib.hm_session_stop_stream(self._handle), "stop_stream")

    def power_off(self) -> None:
        """⚠ The device then needs a PHYSICAL BUTTON PRESS to return, sends no
        acknowledgement, and the link stays up for ~9 s afterwards."""
        check(lib.hm_session_power_off(self._handle), "power_off")

    @property
    def is_streaming(self) -> bool:
        return bool(lib.hm_session_is_streaming(self._handle))

    @property
    def stream_id(self) -> int:
        return lib.hm_session_stream_id(self._handle)

    # --- queries ----------------------------------------------------------
    def clock(self) -> T.hm_clock_snapshot:
        """Raises `HackMotionError` with `ERR_NO_FIT` until a fit exists."""
        snap = T.hm_clock_snapshot()
        check(lib.hm_session_clock(self._handle, ctypes.byref(snap)), "session_clock")
        return snap

    def clock_or_none(self) -> T.hm_clock_snapshot | None:
        """The fit if there is one, else None — for a caller that treats "no fit
        yet" as ordinary rather than exceptional."""
        snap = T.hm_clock_snapshot()
        if lib.hm_session_clock(self._handle, ctypes.byref(snap)) < T.Status.OK:
            return None
        return snap

    def device_info(self) -> T.hm_device_info:
        info = T.hm_device_info()
        check(lib.hm_session_device_info(self._handle, ctypes.byref(info)), "device_info")
        return info

    def stream_config(self) -> T.hm_stream_config:
        return lib.hm_session_stream_config(self._handle)

    def set_clock_correction(self, correction: T.hm_clock_correction) -> None:
        """⚠ `correction.fields` is REQUIRED; a zero bitmask is an error, and a
        FLAGGED ZERO IS A MEASURED ZERO."""
        check(
            lib.hm_session_set_clock_correction(self._handle, ctypes.byref(correction)),
            "set_clock_correction",
        )

    # --- calibration ------------------------------------------------------
    @property
    def calibration_state(self) -> T.CalibrationState:
        """⚠ UNKNOWN is not a hopeful CALIBRATED.  Only a passing presence
        measurement reaches CALIBRATED — the device applies its transform for
        every attempt, including ones an application would reject."""
        return T.CalibrationState(lib.hm_session_calibration_state(self._handle))

    @property
    def calibration_phase(self) -> T.CalibrationPhase:
        return T.CalibrationPhase(lib.hm_calibration_current_phase(self._handle))

    @property
    def presence_angle_deg(self) -> float:
        """⚠ A PRESENCE CHECK, NOT A QUALITY SCORE.  It was measured ranking a
        no-raise calibration best.  NaN before any measurement."""
        return lib.hm_calibration_presence_angle_deg(self._handle)

    def calibration_begin(self) -> None:
        check(lib.hm_calibration_begin(self._handle), "calibration_begin")

    def calibration_confirm_horizontal(self) -> None:
        check(
            lib.hm_calibration_confirm_horizontal(self._handle),
            "calibration_confirm_horizontal",
        )

    def calibration_confirm_raise(self) -> None:
        check(lib.hm_calibration_confirm_raise(self._handle), "calibration_confirm_raise")

    def calibration_confirm_reference_pose(self) -> None:
        """⚠ Returns before the measurement exists — wait on the presence event,
        not on this call.  ⚠ SKIPPING IT LEAVES EVERY LATER SAMPLE AT UNKNOWN."""
        check(
            lib.hm_calibration_confirm_reference_pose(self._handle),
            "calibration_confirm_reference_pose",
        )

    def calibration_abort(self) -> None:
        """⚠ Always permitted, retrieval or not — it writes nothing.  But after
        the device has applied its transform this SKIPS THE PRESENCE CHECK
        rather than undoing a calibration; nothing reverses that."""
        check(lib.hm_calibration_abort(self._handle), "calibration_abort")

    # --- history ----------------------------------------------------------
    def history_reserve(self, request: T.hm_history_request) -> int:
        request_id = ctypes.c_uint64()
        check(
            lib.hm_history_reserve(
                self._handle, ctypes.byref(request), ctypes.byref(request_id)
            ),
            "history_reserve",
        )
        return request_id.value

    def history_collect(self, request_id: int) -> HistoryBlock | None:
        """The block once it exists, else None while it is still in flight.

        ⚠ The returned block owns library memory.  Use it as a context manager
        or call `.release()`."""
        ptr = ctypes.POINTER(T.hm_history_block)()
        status = lib.hm_history_collect(self._handle, request_id, ctypes.byref(ptr))
        if status == T.Status.PENDING:
            return None
        check(status, "history_collect")
        return HistoryBlock(ptr) if ptr else None

    def history_cancel(self, request_id: int) -> None:
        check(lib.hm_history_cancel(self._handle, request_id), "history_cancel")

    @property
    def history_pending(self) -> int:
        return lib.hm_history_pending(self._handle)

    def history_resident_range(self) -> tuple[T.Status, T.hm_time_range]:
        """⚠ THIS UNDER-CLAIMS ON PURPOSE.  It reports only what the device has
        actually served, and returns PENDING while the width is still an
        estimate.  The eviction warning runs on a deliberately more generous
        depth — reading one number into both is how a residency estimate quietly
        becomes a guarantee, and a consumer that skips a pull on a bad guarantee
        has lost the swing."""
        out = T.hm_time_range()
        status = lib.hm_history_resident_range(self._handle, ctypes.byref(out))
        check(status, "history_resident_range")
        return T.Status(status), out

    def history_coverage_available(self, start_us: int, end_us: int) -> bool:
        return bool(
            lib.hm_history_coverage_available(self._handle, start_us, end_us)
        )
