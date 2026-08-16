# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
"""A reference BLE transport over `bleak`, for Linux, macOS and Windows.

    import hackmotion as hm
    from hackmotion.bleak_transport import BleakTransport

    device = await BleakTransport.discover(on_armed=lambda: print("press it"))
    with hm.Session("bench") as session:
        async with BleakTransport(session, device) as link:
            session.start_stream()
            await link.flush()
            ...
        # leaving the `async with` tears the link down and classifies the drop

⚠ OPTIONAL, AND OUTSIDE THE CORE.  ``import hackmotion`` does not import this
module and never will: api-request §2.0 is the request that most shapes this
library's adoption, and it is a blocker rather than a preference — a consumer
that already owns a BLE stack cannot embed a library that brings a second one
contending for the same adapter.  So the radio lives out here, `bleak` is not a
dependency of the binding, and the purity gate over the core stays untouched.

=============================================================================
⚠ THE TRANSPORT IS A TRANSLATOR, NOT A SECOND STATE MACHINE
=============================================================================
Five arrows, and there is nothing else in this file that a reviewer has to
believe:

    discovery + connect  ->  session.on_link_up(mtu, now)
    each notification    ->  session.on_bytes(payload, now)   ONE CALL, ONE
                             NOTIFICATION, stamped inside the callback
    a pump               ->  session.tick(now), then drain ALL FOUR polls, and
                             honour session.next_due_us
    each poll_writes()   ->  write_gatt_char, and NOTHING ELSE ever
    disconnect           ->  session.on_link_down(cause, now), classified

Every deadline — the 30 s keepalive, the bring-up bound, the calibration bound,
the history deadline, the eviction estimate — lives in the session's own timer
table where it is tested on a synthetic clock.  This file owns none of them.

=============================================================================
⛔ SAFETY — the one thing this file must get right
=============================================================================
Command `f0` reboots the sensor into FIRMWARE-UPDATE MODE, and it reaches that
mode through the ORDINARY data characteristic — the same pipe every other
command uses.  Avoiding the OTA service is explicitly not sufficient.

⚠ THE GOOD NEWS HERE IS STRUCTURAL.  This module never composes a command.  The
only bytes that exist came out of ``poll_writes()``, and the library built them
against the allowlist in `src/hm_command.c`.  There is no ``send_raw()``, there
will not be one, and the assertion in ``_write()`` costs one call and is kept
anyway.  **Never sweep or fuzz the device's command space** — the known set was
recovered by reading the vendor's binary, which cannot prove the firmware
accepts nothing else.  Fuzzing the DECODER is a different activity and welcome.
"""

from __future__ import annotations

import asyncio
import time
import warnings
from dataclasses import dataclass
from typing import Any, Callable, Sequence

from . import _types as T
from ._library import lib
from .device import (
    UUID_DATA_CHARACTERISTIC,
    UUID_ISSC_PIPE_INERT,
    UUID_OTA_SERVICE_FORBIDDEN,
    looks_like_hackmotion,
    parse_uuid,
    uuid_str,
)
from .session import Event, Samples, Session, WriteRequest, command_is_allowed

__all__ = [
    "BleakTransport",
    "LinkClassification",
    "TransportError",
    "monotonic_us",
]

try:  # the module stays importable without a radio stack, so tests can load it
    from bleak import BleakClient, BleakScanner

    _BLEAK_ERROR: Exception | None = None
except ImportError as _exc:  # pragma: no cover - exercised on machines without bleak
    BleakClient = BleakScanner = None  # type: ignore[assignment]
    _BLEAK_ERROR = _exc


# ---------------------------------------------------------------------------
# The clock the host supplies
# ---------------------------------------------------------------------------
def monotonic_us() -> int:
    """⚠ MONOTONIC, NEVER A WALL CLOCK.

    An NTP step or a DST change mid-session corrupts a capture in a way that
    looks like a sensor fault (AR C8, `include/hackmotion/types.h`).  The epoch
    is arbitrary and never interpreted; only differences matter, and the session
    warns with `HM_WARN_HOST_CLOCK_REGRESSION` if this ever goes backwards.
    """
    return time.monotonic_ns() // 1000


# The library's own bounds, read from the library rather than transcribed here.
_POLICY = lib.hm_session_policy_default()

# ⚠ How long the link outlived the device's silence, and it is the whole of the
# supervision-timeout / remote-close discriminator.  See `classify_disconnect`.
LINK_QUIET_AMBIGUOUS_US = int(_POLICY.live_gap_alarm_us)

# A pump pass drains until nothing is left, because a drain can produce more
# work — a write's reply is not involved, but an event drained now can be
# followed by another the same tick queued.  Bounded so a library bug becomes a
# loud message rather than a hang.
_MAX_DRAIN_ROUNDS = 64

# ⚠ NOT A POLL PERIOD, AND NOT WHERE THE DEADLINES LIVE.  The session's own
# `next_due_us` is what the pump waits on; this is only a ceiling on that wait,
# so a HOST-side condition — "a swing happened, retrieve it" — gets looked at
# regularly even when the library has nothing due.  Raising it does not make the
# library miss a deadline; it makes the host's own logic coarser.
DEFAULT_MAX_IDLE_US = 100_000

# Spec §2.1's recommended discovery window: be generous, it costs nothing.  The
# sensor advertises for only a few seconds after a button press, so the cost of
# a short window is a user who is told the device does not work.
RECOMMENDED_SCAN_WINDOW_US = 90_000_000


class TransportError(RuntimeError):
    """A radio-level failure, carrying the classification it implies.

    ⚠ `cause` is what would be handed to ``session.on_link_down()``.  It is a
    real classification rather than UNKNOWN wherever the evidence supports one —
    ``CONNECTION_TAKEN`` in particular is a thing a user can act on, where a
    retry loop against it is pure waste."""

    def __init__(self, message: str, cause: T.LinkDownCause = T.LinkDownCause.UNKNOWN):
        super().__init__(message)
        self.cause = cause


@dataclass(frozen=True)
class LinkClassification:
    """A cause and the evidence behind it.

    ⚠ THE EVIDENCE IS NOT DECORATION.  This project's rule is that an estimate
    is reported together with the thing that says what it is worth — intervals
    and density, precision and systematic, a mean and its spread.  A bare
    `SUPERVISION_TIMEOUT` is an assertion; the same value beside "no device byte
    for 4.31 s before the drop" is a measurement a reader can disagree with."""

    cause: T.LinkDownCause
    evidence: str

    def __str__(self) -> str:
        return f"{self.cause.name} ({self.evidence})"


# ---------------------------------------------------------------------------
# The MTU
# ---------------------------------------------------------------------------
async def negotiated_mtu(client) -> int:
    """The ATT MTU, or 0 meaning UNKNOWN.  ⚠ Never a guess, and never a number
    the platform did not actually give us.

    bleak's BlueZ backend returns the ATT default of 23 until something asks
    BlueZ for the real value — its own docstring says "The BlueZ backend will
    always return 23 (the minimum MTU size)".  Reading that 23 as a measurement
    is the same mistake as reading a zero mismatch count out of zero samples as
    agreement, run in the other direction: an ABSENT measurement dressed up as a
    bad one.  It refused a perfectly good link on this bench.

    ``hm_session_on_link_up()`` already has the right word for this case — pass
    0 for "the platform will not tell you", which is treated as
    unknown-and-proceed with `HM_WARN_MTU_UNKNOWN` rather than as a failure —
    so 0 is what an unanswered question returns.

    ⚠ And an unverified MTU is not an unverifiable one.  The frames settle it: a
    47- or 93-byte notification cannot arrive over a 23-byte MTU, so the session
    measures from the data what the platform would not say (§2.4).  The bench
    prints the largest notification it saw for exactly this reason.

    ⚠ This is deliberately a SECOND copy of `tools/hm_capture.py`'s routine.
    That script is frozen so it stays the one path to the device that does not
    go through the library, and pointing both at the same sensor and comparing
    the bytes is what makes a session real.  A shared helper would delete the
    independence that check depends on.
    """
    # _acquire_mtu() is private, and it is what bleak's own mtu_size example
    # uses; there is no public route on this backend.
    backend = getattr(client, "_backend", client)
    acquire = getattr(backend, "_acquire_mtu", None)
    if acquire is not None:
        try:
            await acquire()
        except Exception:  # noqa: BLE001 - any failure at all means "unknown"
            pass

    with warnings.catch_warnings():
        # The "using default MTU" warning is the answer, not a problem.
        warnings.simplefilter("ignore")
        try:
            mtu = int(client.mtu_size or 0)
        except Exception:  # noqa: BLE001
            mtu = 0

    # ⚠ 23 is the ATT default and is therefore indistinguishable from "nobody
    # answered".  Treat it as unknown rather than as a refusal.
    return 0 if mtu <= 23 else mtu


# ---------------------------------------------------------------------------
# Connect-failure classification
# ---------------------------------------------------------------------------
#
# ⚠ A HEURISTIC OVER BLUEZ ERROR TEXT, AND IT SAYS SO.  Neither BlueZ's D-Bus
# API nor CoreBluetooth reports why a connection attempt failed in a form a
# program can branch on; the kernel's mgmt socket carries a reason code and
# nothing above it does.  So this matches strings, and every classification it
# makes travels with the string it matched.
#
# §2.2: the device accepts ONE connection and the vendor app wins the race if it
# is running.  When that happens the device is already connected to something
# else, so it has stopped advertising — the usual symptom is a discovery
# timeout, not a connect error, which is why `discover()` says both things.
_CONNECTION_TAKEN_SIGNS = (
    "br-connection-profile-unavailable",
    "le-connection-abort-by-local",
    "software caused connection abort",
    "connection refused",
    "already connected",
    "in progress",
)
_ADAPTER_GONE_SIGNS = (
    "no powered bluetooth adapters",
    "bluetooth device is turned off",
    "org.bluez.error.notready",
    "not found: /org/bluez",
    "adapter not found",
)


def _same_uuid(text: str, uuid: T.hm_uuid) -> bool:
    """⚠ Compared through the library's parser, never as strings.  Platforms
    differ in case and in whether they expand the 16-bit short form, and
    ``"413C3893-…" != "413c3893-…"`` is a comparison that silently never fires."""
    try:
        return uuid_str(parse_uuid(text)) == uuid_str(uuid)
    except Exception:  # noqa: BLE001 - an unparseable UUID is not a match
        return False


def classify_connect_failure(exc: BaseException) -> LinkClassification:
    """What a failed connect attempt implies, with the text it was read from."""
    text = f"{type(exc).__name__}: {exc}"
    lowered = text.lower()
    for sign in _ADAPTER_GONE_SIGNS:
        if sign in lowered:
            return LinkClassification(T.LinkDownCause.ADAPTER_GONE, text)
    for sign in _CONNECTION_TAKEN_SIGNS:
        if sign in lowered:
            return LinkClassification(T.LinkDownCause.CONNECTION_TAKEN, text)
    return LinkClassification(T.LinkDownCause.TRANSPORT_ERROR, text)


# ---------------------------------------------------------------------------
# The transport
# ---------------------------------------------------------------------------
class BleakTransport:
    """One link, one session, one thread.

    ⚠ ONE THREAD FOR THE WHOLE LIFE OF A SESSION, and under asyncio that is the
    event loop thread.  There are no locks, no atomics and no threads in the
    library, so this is a hard requirement rather than advice.  Every call this
    class makes into the session happens on the loop; `asyncio.to_thread` is for
    blocking on a user and never for calling in here.

    ⚠ AND DO NOT BLOCK THAT THREAD.  Arrival stamps are the entire foundation of
    the clock fit.  A bare `input()` on the loop once fabricated 4.08 s of
    arrival times in this project's own capture harness, and that recording still
    needs a relaxed stream-start bound to replay at all.
    """

    def __init__(
        self,
        session: Session,
        device: Any,
        *,
        adapter: str | None = None,
        now_us: Callable[[], int] = monotonic_us,
        on_event: Callable[[Sequence[Event]], None] | None = None,
        on_live: Callable[[Samples], None] | None = None,
        on_wire: Callable[[Sequence[T.hm_wire_chunk]], None] | None = None,
        max_idle_us: int = DEFAULT_MAX_IDLE_US,
        client: Any = None,
    ):
        if _BLEAK_ERROR is not None and client is None:
            raise TransportError(f"needs `bleak` (pip install bleak): {_BLEAK_ERROR}")

        self.session = session
        self.device = device
        self.now_us = now_us
        self.max_idle_us = max_idle_us

        self._adapter = adapter
        # `client` is the seam the loopback test drives: anything with bleak's
        # connect / disconnect / start_notify / write_gatt_char shape works, so
        # the pump, the drains, the allowlist assertion and the classification
        # are all exercised without a radio.
        self._client = client
        self._client_supplied = client is not None

        self._on_event = on_event
        self._on_live = on_live
        self._on_wire = on_wire

        self._data_char: Any = None
        self._pump_lock = asyncio.Lock()
        self._wake = asyncio.Event()
        self._pump_task: asyncio.Task | None = None

        self._link_up = False
        self._link_down_reported = False
        self._teardown: LinkClassification | None = None
        self._failure: BaseException | None = None
        self._last_device_byte_us = 0
        self._connected_at_us = 0

        self.disconnected = asyncio.Event()

        # --- what the host wants to see afterwards ------------------------
        self.notifications = 0
        self.writes = 0
        self.bytes_in = 0
        self.bytes_out = 0
        self.max_notification_len = 0
        #: What the platform said, or 0 for "it would not say" (§2.4).
        self.mtu = 0
        #: ⚠ Chunks and samples the session produced that NOBODY WAS DRAINING.
        #: A host that configured a wire ring and installed no sink has an absent
        #: recording, not an empty one, and the difference must never be silent.
        self.undelivered_wire_chunks = 0
        self.undelivered_live_samples = 0
        self.undelivered_events = 0
        #: The classification handed to ``on_link_down()``, once it has been.
        self.link_down: LinkClassification | None = None

    # --- discovery --------------------------------------------------------
    @staticmethod
    async def discover(
        *,
        address: str | None = None,
        adapter: str | None = None,
        timeout_us: int = RECOMMENDED_SCAN_WINDOW_US,
        on_armed: Callable[[], None] | None = None,
    ):
        """Scan for a wG3 and return the first one seen.

        ⚠ DISCOVERY IS A RACE (spec §2.1).  The sensor advertises for only a few
        seconds after a physical button press and then stops, so the scanner is
        armed BEFORE `on_armed` is called and the user is asked to press.  A
        scan-then-pick-from-a-list flow assumes the device is discoverable
        whenever you go looking; here it is not, and a scan started after the
        prompt routinely finds nothing — which reaches the user as "the sensor
        doesn't work".

        ⚠ If the sensor was asleep the first press only WAKES it and it does not
        advertise.  Press, pause, then press again.  That instruction is user
        guidance rather than a protocol step, which is why it belongs to
        `on_armed` — the caller owns the UI.

        The match is ``hm_looks_like_hackmotion()``, so the advertised name
        exists in exactly one place in this project.
        """
        if _BLEAK_ERROR is not None:
            raise TransportError(f"needs `bleak` (pip install bleak): {_BLEAK_ERROR}")
        if address:
            return address

        found: asyncio.Future = asyncio.get_running_loop().create_future()

        def seen(device, adv):
            name = adv.local_name or device.name or ""
            if found.done():
                return
            if looks_like_hackmotion(name, adv.service_uuids or ()):
                found.set_result(device)

        kwargs = {"adapter": adapter} if adapter else {}
        async with BleakScanner(detection_callback=seen, **kwargs):
            if on_armed is not None:
                on_armed()
            try:
                return await asyncio.wait_for(found, timeout=timeout_us / 1e6)
            except asyncio.TimeoutError:
                raise TransportError(
                    f"no HackMotion wG3 advertised in {timeout_us / 1e6:.0f} s.\n"
                    "  ⚠ Two different things look like this, and only one is a "
                    "fault:\n"
                    "   · the sensor is asleep — press, pause, press again "
                    "(§2.1); or\n"
                    "   · something else already holds it.  The device accepts "
                    "ONE connection\n"
                    "     and stops advertising once taken, so the vendor app "
                    "running on a\n"
                    "     phone or another machine looks exactly like this "
                    "(§2.2).",
                    T.LinkDownCause.CONNECTION_TAKEN,
                ) from None

    # --- lifetime ---------------------------------------------------------
    async def __aenter__(self) -> "BleakTransport":
        await self.connect()
        return self

    async def __aexit__(self, *exc) -> None:
        await self.close()

    async def connect(self) -> None:
        """Connect, measure the MTU, subscribe, and tell the session.

        ⚠ THE ORDER MATTERS.  Notifications are subscribed BEFORE
        ``on_link_up()``, because link-up is what starts bring-up and the first
        `80` reply can arrive before the next line of Python runs.  Subscribing
        afterwards drops it, and the session then waits out its bring-up bound
        for a reply that was delivered to nobody.
        """
        if self._client is None:
            kwargs = {"adapter": self._adapter} if self._adapter else {}
            self._client = BleakClient(
                self.device, disconnected_callback=self._on_disconnected, **kwargs
            )
            try:
                await self._client.connect()
            except Exception as exc:  # noqa: BLE001
                classification = classify_connect_failure(exc)
                raise TransportError(
                    f"could not connect: {classification.evidence}",
                    classification.cause,
                ) from exc
        elif hasattr(self._client, "set_disconnected_callback"):
            self._client.set_disconnected_callback(self._on_disconnected)

        self._connected_at_us = self.now_us()
        self._data_char = self._resolve_characteristic()
        self.mtu = await negotiated_mtu(self._client)

        # ⚠ SUBSCRIBE FIRST.  See the docstring: bring-up starts at on_link_up()
        # and the first `80` reply can beat the next line of Python.
        await self._client.start_notify(self._data_char, self._on_notify)

        self._link_up = True
        self.session.on_link_up(self.mtu, self.now_us())
        self._pump_task = asyncio.get_running_loop().create_task(self._run())
        await self.flush()

    def _resolve_characteristic(self):
        """⛔ REFUSE THE FIRMWARE-UPDATE PATH BY CONSTRUCTION, not by comment.

        AR §2.16 asked for exactly this: the OTA/DFU service is published so a
        transport can decline it structurally.  The inert ISSC pipe is published
        for the other half — it accepts writes and never replies, so a transport
        that enumerates characteristics and takes the first writable one picks a
        pipe that swallows the whole protocol in silence.

        ⚠ And the data characteristic does NOT sit under the service's UUID base
        (AR §2.0.5), so it is resolved on its OWN UUID and never derived from the
        service's by fragment substitution.  A transport that substitutes a
        fragment cannot express this device at all — that is a real finding from
        the consumer's own codebase, not a hypothetical.
        """
        wanted = uuid_str(UUID_DATA_CHARACTERISTIC)
        try:
            services = getattr(self._client, "services", None)
        except Exception:  # noqa: BLE001
            # bleak raises rather than returning None when service discovery has
            # not run.  ⚠ `getattr`'s default does not cover that — it only
            # swallows AttributeError — so the try is doing real work here.
            services = None
        if services is None or not hasattr(services, "get_characteristic"):
            # A client that does not enumerate — the loopback's fake, or a
            # backend mid-connect — still takes the UUID string itself.
            return wanted

        found = services.get_characteristic(wanted)
        if found is None:
            raise TransportError(
                f"the data characteristic {wanted} is not on this device.\n"
                "  ⚠ The protocol is one bidirectional characteristic (spec "
                "§2.3); there is nothing else to fall back to, and the ISSC pipe "
                "that looks like an alternative accepts writes and never replies.",
                T.LinkDownCause.TRANSPORT_ERROR,
            )

        # ⛔ Structural refusals, in the order they could bite.
        service_uuid = getattr(found, "service_uuid", None)
        if service_uuid and _same_uuid(service_uuid, UUID_OTA_SERVICE_FORBIDDEN):
            raise TransportError(
                "⛔ the data characteristic resolved inside the OTA/DFU service. "
                "A bad write there bricks the device (spec §2.3); refusing.",
                T.LinkDownCause.TRANSPORT_ERROR,
            )
        if _same_uuid(str(found.uuid), UUID_ISSC_PIPE_INERT):
            raise TransportError(
                "⛔ resolved the inert ISSC pipe, which accepts every write and "
                "never replies; refusing rather than swallowing the protocol.",
                T.LinkDownCause.TRANSPORT_ERROR,
            )
        return found

    async def close(self, cause: T.LinkDownCause = T.LinkDownCause.LOCAL_REQUEST) -> None:
        """Stop the pump, let the last writes out, disconnect, and report it.

        ⚠ ONE LAST PUMP BEFORE THE RADIO GOES.  ``hm_session_close()`` is not
        what happens here — that is the caller's — but a `83` the caller queued
        a moment ago is still sitting in `poll_writes()`, and tearing the link
        down first would leave the device streaming into nothing.

        ⚠ AND IT DOES NOT OVERWRITE A CLASSIFICATION THAT ALREADY EXISTS.  A
        link that dropped on its own a second ago did not drop because the host
        then asked it to, and relabelling it LOCAL_REQUEST would hide the one
        failure a user could have acted on.
        """
        if self._teardown is None and not self.disconnected.is_set():
            self._teardown = LinkClassification(cause, "the host asked for it")
        if self._pump_task is not None:
            task, self._pump_task = self._pump_task, None
            self._wake.set()
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        if self._client is not None and self._link_up and not self.disconnected.is_set():
            try:
                await self._pump_once()
            except Exception:  # noqa: BLE001 - the link may already be gone
                pass
            try:
                await self._client.stop_notify(self._data_char)
            except Exception:  # noqa: BLE001
                pass

        if self._client is not None and not self._client_supplied:
            try:
                await self._client.disconnect()
            except Exception:  # noqa: BLE001
                pass

        self._report_link_down()
        # ⚠ ONE MORE DRAIN, because `on_link_down()` PRODUCES.  It pushes the
        # LINK_DOWN event carrying the recovery advice, and a `link_down` chunk
        # that a recording wants as its last line.  Without this the caller's
        # sinks never see either, and the single most useful event of a failed
        # session is the one that goes missing.
        await self._deliver_after_link_down()

    # --- notifications ----------------------------------------------------
    def _on_notify(self, _characteristic, data) -> None:
        """⚠ SYNCHRONOUS, AND IT DOES EXACTLY THREE THINGS.

        Stamp, hand the payload to the session, wake the pump.  It never awaits
        and never writes, and both of those are load bearing:

        ⚠ IF THIS WERE A COROUTINE, ORDER WOULD NOT SURVIVE.  bleak schedules an
        async notification callback as a task; each task runs to its first
        `await` and then yields, so two notifications that await a write can
        reach ``on_bytes()`` in the wrong order.  The protocol has no length
        field, no sequence number and no checksum (§3), so there is nothing to
        resynchronise on and the corruption is silent.  A sync callback is
        called in arrival order by every backend and cannot reorder.

        ⚠ AND THE STAMP IS TAKEN HERE, not on the pump.  `host_recv_us` is the
        entire foundation of the clock fit; a stamp taken one scheduling hop
        later is the scheduler's latency measured as the device's.

        ⚠ ONE CALL, ONE NOTIFICATION.  The payload goes in exactly as the
        transport delivered it — never concatenated, never re-framed.
        """
        when = self.now_us()
        payload = bytes(data)
        try:
            self.session.on_bytes(payload, when)
        except BaseException as exc:  # noqa: BLE001
            # ⚠ Never swallowed.  bleak logs an exception out of a callback and
            # carries on, which would leave a session quietly missing frames; it
            # is stashed and re-raised on the pump instead.
            self._failure = exc
        self._last_device_byte_us = when
        self.notifications += 1
        self.bytes_in += len(payload)
        if len(payload) > self.max_notification_len:
            self.max_notification_len = len(payload)
        self._wake.set()

    def _on_disconnected(self, _client) -> None:
        """bleak's disconnect callback.  ⚠ Sync, so it only records and wakes."""
        self.disconnected.set()
        self._wake.set()

    # --- the pump ---------------------------------------------------------
    def wake(self) -> None:
        """Ask the pump to run now.  Safe from anywhere on the loop."""
        self._wake.set()

    async def flush(self) -> None:
        """Run one full pump pass and wait for it.

        Call it after any session command — `start_stream()`,
        `calibration_begin()`, `history_reserve()` — when you want the bytes
        actually on the wire before the next line.  The background pump would
        get there anyway; this makes *when* deterministic, which is what a
        prompt-driven routine like calibration needs.
        """
        await self._pump_once()

    async def _run(self) -> None:
        """The loop.  ⚠ It waits on `next_due_us`, it does not poll a period."""
        try:
            while not self.disconnected.is_set():
                await self._pump_once()
                if self.disconnected.is_set():
                    break

                # ⚠ RE-READ AFTER EVERY CALL INTO THE SESSION.  A drain can arm
                # a deadline — the keepalive, a history attempt, the eviction
                # estimate — so a value read before the drain is already stale.
                due = self.session.next_due_us
                now = self.now_us()
                if due >= T.HM_TIME_NEVER:
                    wait_us = self.max_idle_us
                else:
                    wait_us = min(max(due - now, 0), self.max_idle_us)
                try:
                    await asyncio.wait_for(self._wake.wait(), wait_us / 1e6)
                except asyncio.TimeoutError:
                    pass
                self._wake.clear()
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            self._failure = exc
            if self._teardown is None:
                # ⚠ Not overwritten: `_write()` already classified a write
                # failure with the text it read the classification out of, and
                # re-deriving it here from the wrapper would lose that.
                self._teardown = classify_connect_failure(exc)
            self.disconnected.set()

        # ⚠ REPORT THE DROP FROM HERE, not only from `close()`.  A link that goes
        # away while the caller is awaiting something else has to reach the
        # session promptly: link-down is what invalidates the calibration,
        # abandons every outstanding retrieval as LINK_LOST rather than letting
        # it time out, and produces the recovery advice a user acts on.
        if self.disconnected.is_set():
            self._report_link_down()
            await self._deliver_after_link_down()

    async def _pump_once(self) -> None:
        async with self._pump_lock:
            if self._failure is not None:
                failure, self._failure = self._failure, None
                raise failure
            self.session.tick(self.now_us())
            await self._drain()

    async def _drain(self) -> None:
        """Drain ALL FOUR polls until the session has nothing left to say.

        ⚠ NO WRITE QUEUE OF ITS OWN, AND THIS IS THE PLACE THAT WOULD GROW ONE.
        ``poll_writes()`` returns nothing at all while a history retrieval is in
        flight — a deliberate quiet period with four exits, not a stall — and a
        transport that "helpfully" kept its own queue moving during a retrieval
        would put a `81` reply into a record stream that has no length field, no
        sequence number and nothing to resynchronise on.  So this asks, writes
        what it is given, and asks again.
        """
        for _ in range(_MAX_DRAIN_ROUNDS):
            busy = False

            for write in self.session.poll_writes():
                await self._write(write)
                busy = True

            events = self.session.poll_events()
            if events:
                busy = True
                self._deliver_events(events)

            samples = self.session.poll_live()
            if len(samples):
                busy = True
                if self._on_live is not None:
                    self._on_live(samples)
                else:
                    self.undelivered_live_samples += len(samples)

            chunks = self.session.poll_wire()
            if chunks:
                busy = True
                if self._on_wire is not None:
                    self._on_wire(chunks)
                else:
                    # ⚠ An absent recording, not an empty one.  Counted so it
                    # cannot pass for "the session produced nothing".
                    self.undelivered_wire_chunks += len(chunks)

            if not busy:
                return

        # ⚠ Loud rather than silent.  Reaching here means a drain kept producing
        # work for 64 rounds, which is a library bug and not a busy session.
        raise TransportError(
            f"the session did not settle in {_MAX_DRAIN_ROUNDS} drain rounds",
            T.LinkDownCause.TRANSPORT_ERROR,
        )

    async def _deliver_after_link_down(self) -> None:
        """The three OUTPUT polls, once, with the radio already gone.

        ⚠ Deliberately not `_drain()`: `poll_writes()` is asked nothing here,
        because there is nowhere for the bytes to go and the session is entitled
        to be told the link is down before anything else happens.  The other
        three still carry work worth having — the LINK_DOWN event itself with its
        recovery advice, the last live samples, and the wire chunks that make the
        recording end where the link did rather than a beat early.
        """
        events = self.session.poll_events()
        if events:
            self._deliver_events(events)
        samples = self.session.poll_live()
        if len(samples) and self._on_live is not None:
            self._on_live(samples)
        chunks = self.session.poll_wire()
        if chunks and self._on_wire is not None:
            self._on_wire(chunks)

    def _deliver_events(self, events: Sequence[Event]) -> None:
        for event in events:
            if event.type == T.EventType.MTU_REJECTED:
                # ⚠ The session REFUSES to run below 96 — a hard floor, not
                # advice (§2.4) — so there is nothing to wait for.  The link is
                # torn down rather than left up looking healthy.
                self._teardown = LinkClassification(
                    T.LinkDownCause.LOCAL_REQUEST,
                    f"the session refused a negotiated MTU of "
                    f"{event.raw.u.mtu} (§2.4 needs ≥ 96)",
                )
                self.disconnected.set()
        if self._on_event is not None:
            self._on_event(events)
        else:
            self.undelivered_events += len(events)

    async def _write(self, write: WriteRequest) -> None:
        """⛔ THE ONLY PLACE BYTES LEAVE THIS PROCESS.

        `write.data` came out of ``poll_writes()``, which means the library
        composed it against the allowlist in `src/hm_command.c`.  The assertion
        below cannot fail through this path — and it is kept because it costs one
        call, and because the day somebody adds a second caller is the day it
        earns its keep.  ⛔ `f0` reboots the sensor into firmware-update mode
        through this same characteristic; there is no `send_raw()` and there will
        not be one.
        """
        if not write.data:
            raise TransportError("an empty write request", T.LinkDownCause.TRANSPORT_ERROR)
        if not command_is_allowed(write.data[0]):
            raise TransportError(
                f"⛔ refusing to write {write.data.hex()}: 0x{write.data[0]:02x} is "
                "not on the library's allowlist",
                T.LinkDownCause.TRANSPORT_ERROR,
            )
        try:
            await self._client.write_gatt_char(
                self._data_char, write.data, response=not write.without_response
            )
        except Exception as exc:  # noqa: BLE001
            classification = classify_connect_failure(exc)
            self._teardown = LinkClassification(
                classification.cause, f"a write failed: {classification.evidence}"
            )
            self.disconnected.set()
            raise TransportError(
                f"write of {write.data.hex()} failed: {classification.evidence}",
                classification.cause,
            ) from exc
        self.writes += 1
        self.bytes_out += len(write.data)

    # --- the way down -----------------------------------------------------
    def classify_disconnect(self) -> LinkClassification:
        """What happened to the link, and the evidence for saying so.

        ⚠ NOT UNKNOWN.  The library turns a cause into recovery advice a user
        acts on — reconnect with backoff, close the other application, press the
        button, stop retrying — and `UNKNOWN` throws that away at the one moment
        it is worth most.  Three of the four arms below are facts this transport
        holds directly; the fourth is a heuristic and is labelled as one.

        The heuristic is HOW LONG THE LINK OUTLIVED THE DEVICE'S SILENCE:

          · A supervision timeout drops the link a fixed window after the LAST
            RECEIVED PACKET — 420 ms under Linux's default LE connection
            parameters, a few seconds under any stack that negotiates its own.
            Nothing arrives during that window by definition.
          · A remote terminate ARRIVES AS A PACKET, so the link can outlive the
            silence for as long as the far end likes.

        So a link still up ``LINK_QUIET_AMBIGUOUS_US`` after the device went
        quiet was alive through that silence and the far end chose to go:
        REMOTE_CLOSED.  Below that the two are indistinguishable, and
        SUPERVISION_TIMEOUT is the one that claims less — it says the link went
        away and nothing about the far end's intent.

        ⚠ The bound is the library's own `policy.live_gap_alarm_us`, not a
        number invented here: it is already this project's answer to "frames
        should have arrived by now".

        ⚠ AND IT MATTERS AT EXACTLY ONE PLACE.  `hm_session.c` turns
        REMOTE_CLOSED after 10 s of quiet into NEEDS_BUTTON_PRESS — a device
        that went to sleep, which no amount of retrying can fix.  Every shorter
        quiet reaches RECONNECT_WITH_BACKOFF under either cause, so the choice
        is only ever visible where the evidence for it is strongest.
        """
        if self._teardown is not None:
            return self._teardown

        now = self.now_us()
        if self._last_device_byte_us == 0:
            connected_for = (now - self._connected_at_us) / 1e6
            return LinkClassification(
                T.LinkDownCause.REMOTE_CLOSED,
                f"the device never sent a byte in {connected_for:.2f} s of link",
            )

        quiet_us = now - self._last_device_byte_us
        if quiet_us >= LINK_QUIET_AMBIGUOUS_US:
            return LinkClassification(
                T.LinkDownCause.REMOTE_CLOSED,
                f"the link outlived {quiet_us / 1e6:.2f} s of device silence, "
                f"longer than the {LINK_QUIET_AMBIGUOUS_US / 1e6:.2f} s a "
                "supervision timeout would have waited",
            )
        return LinkClassification(
            T.LinkDownCause.SUPERVISION_TIMEOUT,
            f"the last device byte was {quiet_us / 1e6:.2f} s before the drop; "
            "a clean remote close and a short supervision timeout are "
            "indistinguishable here",
        )

    def _report_link_down(self) -> None:
        if not self._link_up or self._link_down_reported:
            return
        self._link_down_reported = True
        classification = self.classify_disconnect()
        self.link_down = classification
        self.session.on_link_down(classification.cause, self.now_us())

    async def wait_closed(self) -> None:
        """Block until the link goes down for any reason, then report it.

        The pump reports it too, so this is a place for a caller to WAIT rather
        than the mechanism — `_report_link_down()` runs exactly once either way.
        """
        await self.disconnected.wait()
        self._report_link_down()
