#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
"""test_python_transport.py — the bleak transport, driven by a real device's bytes.

⚠ AN UNTESTED TRANSPORT THAT LOOKS FINISHED IS WORSE THAN AN HONESTLY PARTIAL
ONE.  This file is what keeps `hackmotion/bleak_transport.py` from being the
second kind.  It replaces the radio and nothing else: the transport's own
notification callback, its own pump, its own drains, its own write path and its
own classification all run exactly as they do against a sensor, over the bytes
`swings.hmwire` recorded off one.

⚠ WHAT IT CANNOT SEE, said plainly.  The loopback cannot test bleak itself — the
BlueZ MTU trap, whether a backend really preserves ATT PDU boundaries, whether
a disconnect callback fires.  Those need a device, and this file's existence is
not a substitute for pointing the bench at one.

What it CAN see is everything between the callback and the session, which is
where a transport goes wrong:

  · one call, one notification — never coalesced, never split, never reordered
  · the only bytes that leave came out of poll_writes(), and every one of them
    is on the library's allowlist
  · ⛔ a command that is NOT on the allowlist is refused rather than written
  · NO WRITE QUEUE OF ITS OWN: nothing is written while a history bracket is
    open, which is the R7 hazard the quiet period exists to remove
  · a whole retrieval survives the drain path, at the density signature the C
    tool and the binding both produce
  · a link drop is CLASSIFIED, and each arm of the classification is reachable

Usage: test_python_transport.py <libhackmotion_ffi.so> [<fixture.hmwire>]
"""

import asyncio
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

failures = 0
checks = 0

# ⚠ THE TWO REGIMES, AND A CHECK THAT ONLY EVER SEES ONE HAS SEEN HALF THE
# DEVICE.  `swings.hmwire` is the 657-728 Hz regime — five pulls over golf
# swings and one over a wrist held still.  `session1.hmwire` is the 100.5 Hz
# one, a calibration choreography whose single pull is over a still wrist, and
# it sits at 173 s, PAST TWO INDEX WRAPS.
#
#     fixture            blocks, densest, sparsest
DENSITY_SIGNATURE = {
    "swings.hmwire": (6, 1.0, 0.125),
    "session1.hmwire": (1, 0.125, 0.125),
}


def check(condition, what):
    global failures, checks
    checks += 1
    print(f"[{' ok ' if condition else 'FAIL'}] {what}")
    if not condition:
        failures += 1


def note(what):
    print(f"[note] {what}")


# ---------------------------------------------------------------------------
# The radio, replaced
# ---------------------------------------------------------------------------
class Clock:
    """⚠ The RECORDING's clock, not the wall's.

    Arrival stamps are the whole foundation of the fit, so a replay that stamps
    with `time.monotonic_ns()` hands the session 173 s of session compressed
    into two — every frame apparently arriving at once, a rate fit of nonsense,
    and a stream-start bound blown in both directions.  The recorded host time
    is the only honest stamp here."""

    def __init__(self):
        self.now = 0

    def __call__(self) -> int:
        return self.now


class LoopbackClient:
    """Everything below the transport, and nothing above it.

    The shape is bleak's — `start_notify`, `write_gatt_char`, `disconnect`,
    `mtu_size` — so the transport under test is the real one and this is the
    only substitution.  ⚠ `services` is None on purpose: that is the branch
    where the transport falls back to the characteristic's UUID string, which is
    what every backend accepts, and it keeps this file from having to fake
    BlueZ's whole service model.
    """

    def __init__(self, mtu: int = 247):
        self.mtu_size = mtu
        self.services = None
        self.written: list[bytes] = []
        self.write_marks: list[bool] = []  # was a history bracket open?
        self.notify_cb = None
        self.disconnect_cb = None
        self.stopped_notify = False
        self.disconnected = False
        self.bracket_open = False

    # --- the bits bleak provides -----------------------------------------
    def set_disconnected_callback(self, cb):
        self.disconnect_cb = cb

    async def start_notify(self, _char, cb):
        self.notify_cb = cb

    async def stop_notify(self, _char):
        self.stopped_notify = True

    async def write_gatt_char(self, _char, data, response=True):
        self.written.append(bytes(data))
        # ⚠ Recorded at the instant of the write, because the assertion is about
        # WHEN it happened: a write that lands inside a bracket is the R7 hazard.
        self.write_marks.append(self.bracket_open)

    async def disconnect(self):
        self.disconnected = True

    # --- the test's own driving ------------------------------------------
    def deliver(self, payload: bytes) -> None:
        """One ATT notification, exactly as the recording holds it."""
        if payload[:2] == b"\xa1\x02":
            self.bracket_open = True
        self.notify_cb(None, bytearray(payload))
        if payload[:2] == b"\xa1\x01":
            self.bracket_open = False

    def drop(self) -> None:
        if self.disconnect_cb is not None:
            self.disconnect_cb(self)


class SpySession:
    """A real session with the transport's inputs recorded on the way in.

    ⚠ Wrapping rather than mocking: the session underneath is the shipping one,
    so a retrieval driven through here is a real retrieval.  The spy exists only
    so "one call, one notification" can be asserted on the CALLS rather than
    inferred from the result."""

    def __init__(self, session):
        self._session = session
        self.bytes_in: list[bytes] = []
        self.link_up: list[tuple[int, int]] = []
        self.link_down: list[int] = []

    def __getattr__(self, name):
        return getattr(self._session, name)

    def on_bytes(self, data, host_recv_us):
        self.bytes_in.append(bytes(data))
        self._session.on_bytes(data, host_recv_us)

    def on_link_up(self, mtu, now_us):
        self.link_up.append((mtu, now_us))
        self._session.on_link_up(mtu, now_us)

    def on_link_down(self, cause, now_us):
        self.link_down.append(int(cause))
        self._session.on_link_down(cause, now_us)


# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------
async def replay_through_transport(fixture: Path):
    import hackmotion as hm
    from hackmotion import _types as T
    from hackmotion.bleak_transport import BleakTransport

    clock = Clock()
    client = LoopbackClient(mtu=247)
    # ⚠ THE STREAM-START BOUND IS RELAXED, AND ONLY BECAUSE THIS IS A REPLAY.
    # The library's 3 s default is two orders of margin over the 50-80 ms a real
    # device takes.  `session1.hmwire`'s first frame arrived at 4.08 s — a
    # property of the RECORDER, whose harness blocked its own event loop, not of
    # the device.  Enforcing a live-session bound against a recording would
    # discard the only capture this project holds of a retrieval past two index
    # wraps, and would be enforcing it against the wrong party.  The bound is
    # tested where it belongs, in `tests/test_session.c`.
    # `tools/hm_replay_py.py` carries the same relaxation for the same reason.
    session = hm.Session(
        "loopback",
        digest_ring=hm.HM_DIGEST_RING_RECOMMENDED,
        policy={"stream_start_timeout_us": 15_000_000},
    )
    spy = SpySession(session)

    blocks = []
    pending = [0]
    events = []
    live = [0]
    errors: list[str] = []

    def on_event(evs):
        events.extend(evs)

    def on_live(samples):
        live[0] += len(samples)

    transport = BleakTransport(
        spy,
        "loopback",
        now_us=clock,
        client=client,
        on_event=on_event,
        on_live=on_live,
        # ⚠ 1 ms rather than 100 ms: the pump's idle wait is REAL time even when
        # the session's clock is synthetic, and the fixture has 21,000 chunks.
        max_idle_us=1_000,
    )

    def collect():
        if not pending[0]:
            return
        block = session.history_collect(pending[0])
        if block is None:
            return
        with block:
            pending[0] = 0
            blocks.append(
                {
                    "status": hm.history_status_name(block.status),
                    "n": block.sample_count,
                    "density": block.density,
                    "coverage": block.coverage_fraction,
                    "achieved_hz": block.achieved_hz,
                    "overlap": (block.live_overlap_mismatches,
                                block.live_overlap_samples),
                }
            )

    def reserve_like_the_recording(chunk):
        """The same reservation `tools/hm_replay_py.py` makes, for the same
        reason: the recording's own `a1` says where a retrieval happened, and a
        library-driven consumer asks for that span in HOST time."""
        collect()
        snapshot = session.clock_or_none()
        if snapshot is None or not (snapshot.flags & T.ClockFlag.HAS_FIT):
            return
        raw_last = (chunk.data[3] << 8) | chunk.data[4]
        delta = (raw_last - snapshot.last_index) & 0xFFFF
        if delta >= 0x8000:
            delta -= 0x10000
        last = (snapshot.last_index + delta) & 0xFFFFFFFF
        raw_first = (chunk.data[1] << 8) | chunk.data[2]
        first = (last - ((last - raw_first) & 0xFFFF)) & 0xFFFFFFFF
        # Clamped to the head the device had actually reached — a reservation
        # would rightly have held a window past it.
        last = min(last, snapshot.last_index)
        if first >= last:
            return

        request = T.hm_history_request()
        request.window.start_us = hm.clock_to_host_us(snapshot, first) - 1
        request.window.end_us = hm.clock_to_host_us(snapshot, last) + 1
        request.deadline_us = chunk.host_time_us + 60_000_000
        request.max_attempts = 1
        try:
            pending[0] = session.history_reserve(request)
        except hm.HackMotionError:
            pending[0] = 0

    # --- drive it ---------------------------------------------------------
    with hm.Replay(fixture) as replay:
        chunks = list(replay)

    if not chunks:
        return None

    clock.now = chunks[0].host_time_us
    await transport.connect()

    device_to_host = 0
    for chunk in chunks:
        clock.now = chunk.host_time_us
        if chunk.direction == T.WireDirection.META:
            text = bytes(chunk.data[: chunk.length])
            if text.startswith(b"stream_start"):
                try:
                    session.start_stream()
                except hm.HackMotionError as exc:
                    # ⚠ Recorded rather than raised, so a broken transport fails
                    # the CHECKS below by name instead of killing the harness
                    # with a traceback that says nothing about which property
                    # went.  A session refusing to stream here is itself a
                    # symptom — of bring-up never completing, which is what a
                    # coalescing or reordering callback produces.
                    errors.append(f"start_stream: {exc}")
        elif chunk.direction == T.WireDirection.HOST_TO_DEVICE:
            if chunk.length == 5 and chunk.data[0] == 0xA1:
                reserve_like_the_recording(chunk)
        else:
            device_to_host += 1
            # ⚠ THROUGH THE TRANSPORT'S OWN CALLBACK, which is the whole point
            # of this file.  Not session.on_bytes() — that would test the
            # binding, which `test_python_replay.py` already does.
            client.deliver(bytes(chunk.data[: chunk.length]))
        await transport.flush()
        collect()

    collect()
    await transport.close()

    return {
        "session": session,
        "spy": spy,
        "client": client,
        "transport": transport,
        "blocks": blocks,
        "events": events,
        "errors": errors,
        "live": live[0],
        "device_to_host": device_to_host,
        "payloads": [bytes(c.data[: c.length]) for c in chunks
                     if c.direction == T.WireDirection.DEVICE_TO_HOST],
    }


# ---------------------------------------------------------------------------
# The classification, on its own — a pure function with reachable arms
# ---------------------------------------------------------------------------
def check_classification():
    import hackmotion as hm  # noqa: F401
    from hackmotion import _types as T
    from hackmotion.bleak_transport import (
        LINK_QUIET_AMBIGUOUS_US,
        BleakTransport,
        LinkClassification,
        classify_connect_failure,
    )

    clock = Clock()

    def transport_at(quiet_us: int, last_byte: bool = True):
        t = BleakTransport.__new__(BleakTransport)
        t.now_us = clock
        t._teardown = None
        t._connected_at_us = 0
        t._last_device_byte_us = (clock.now - quiet_us) if last_byte else 0
        return t

    clock.now = 100_000_000

    # ⚠ Every arm below is reachable, and a change that collapses two of them
    # into UNKNOWN fails here rather than at a user's bench.
    short = transport_at(LINK_QUIET_AMBIGUOUS_US // 2)
    check(
        short.classify_disconnect().cause == T.LinkDownCause.SUPERVISION_TIMEOUT,
        "a drop that follows traffic closely reads SUPERVISION_TIMEOUT",
    )
    long = transport_at(LINK_QUIET_AMBIGUOUS_US * 4)
    classification = long.classify_disconnect()
    check(
        classification.cause == T.LinkDownCause.REMOTE_CLOSED,
        "a link that outlived a long silence reads REMOTE_CLOSED",
    )
    # ⚠ This is the arm the library turns into NEEDS_BUTTON_PRESS at 10 s of
    # quiet — a device that went to sleep, which no retry can fix.  Collapsing
    # it to SUPERVISION_TIMEOUT would make that advice unreachable.
    check(
        "s of device silence" in classification.evidence,
        "the classification carries the measurement it was made from",
    )
    silent = transport_at(0, last_byte=False)
    check(
        silent.classify_disconnect().cause == T.LinkDownCause.REMOTE_CLOSED,
        "a link that never carried a device byte is still classified",
    )
    asked = transport_at(0)
    asked._teardown = LinkClassification(T.LinkDownCause.LOCAL_REQUEST, "asked")
    check(
        asked.classify_disconnect().cause == T.LinkDownCause.LOCAL_REQUEST,
        "a teardown we asked for is never relabelled",
    )

    # ⚠ CONNECTION_TAKEN is the one a user can ACT on: the device accepts one
    # connection and the vendor app wins the race.  A retry loop against it is
    # pure waste, so it must not fall through to TRANSPORT_ERROR.
    taken = classify_connect_failure(
        Exception("org.bluez.Error.Failed: Software caused connection abort")
    )
    check(
        taken.cause == T.LinkDownCause.CONNECTION_TAKEN,
        "BlueZ's connection-abort text classifies as CONNECTION_TAKEN",
    )
    gone = classify_connect_failure(Exception("No powered Bluetooth adapters found"))
    check(
        gone.cause == T.LinkDownCause.ADAPTER_GONE,
        "a missing adapter classifies as ADAPTER_GONE",
    )
    other = classify_connect_failure(Exception("something nobody has seen"))
    check(
        other.cause == T.LinkDownCause.TRANSPORT_ERROR,
        "an unrecognised failure is TRANSPORT_ERROR, not a guessed CONNECTION_TAKEN",
    )


async def check_mtu_translation():
    """⚠ 23 IS NOT A MEASUREMENT.  BlueZ reports the ATT default until something
    asks it properly, and reading that as a negotiated value refuses a perfectly
    good link — an ABSENT answer dressed up as a bad one."""
    from hackmotion.bleak_transport import negotiated_mtu

    class Raising:
        @property
        def mtu_size(self):
            raise RuntimeError("the platform will not say")

    check(await negotiated_mtu(LoopbackClient(mtu=23)) == 0,
          "an MTU of 23 is reported as UNKNOWN (0), not as a refusal")
    check(await negotiated_mtu(LoopbackClient(mtu=247)) == 247,
          "a real negotiated MTU is passed through unchanged")
    check(await negotiated_mtu(Raising()) == 0,
          "a platform that raises rather than answers reports UNKNOWN")


async def check_allowlist_refusal():
    """⛔ The assertion that cannot fire through poll_writes() — confirmed by
    making it fire.  `f0` reboots the sensor into firmware-update mode through
    the ORDINARY data characteristic, and a transport that grew a second write
    path is exactly how it would reach the wire."""
    import hackmotion as hm
    from hackmotion.bleak_transport import BleakTransport, TransportError

    client = LoopbackClient()
    transport = BleakTransport.__new__(BleakTransport)
    transport._client = client
    transport._data_char = "loopback"
    transport._teardown = None
    transport.disconnected = asyncio.Event()
    transport.writes = 0
    transport.bytes_out = 0

    check(not hm.command_is_allowed(0xF0),
          "⛔ 0xf0 is not on the library's allowlist")
    try:
        await transport._write(hm.WriteRequest(data=b"\xf0", without_response=False))
        check(False, "⛔ a write of f0 was NOT refused — it reached the client")
    except TransportError:
        check(len(client.written) == 0,
              "⛔ a write of f0 is refused and nothing reaches the characteristic")

    # And the allowed case still goes through, or the check above proves nothing.
    await transport._write(hm.WriteRequest(data=b"\x81", without_response=False))
    check(client.written == [b"\x81"], "an allowlisted command is written unchanged")


# ---------------------------------------------------------------------------
def check_fixture(fixture: Path) -> int:
    """Every transport check that needs real bytes, over one recording."""
    import hackmotion as hm
    from hackmotion import _types as T

    print(f"\n--- {fixture.name} ---")
    result = asyncio.run(replay_through_transport(fixture))
    if result is None:
        print(f"⚠ NOTHING WAS CHECKED: {fixture.name} held no chunks",
              file=sys.stderr)
        return 3

    spy = result["spy"]
    client = result["client"]
    transport = result["transport"]

    # ⚠ Anything the drive loop could not do, named before the rest — otherwise a
    # session that refused to stream shows up only as a missing block.
    check(not result["errors"],
          "the session accepted every command the recording called for"
          + (f" — {'; '.join(result['errors'])}" if result["errors"] else ""))

    # --- one call, one notification --------------------------------------
    #
    # ⚠ THE ONE THING A TRANSPORT MUST NOT GET WRONG.  The protocol has no length
    # field, no sequence number and no checksum, so a coalescing or reordering
    # transport corrupts silently with nothing to resynchronise on.
    check(
        len(spy.bytes_in) == result["device_to_host"],
        f"one on_bytes() per notification — {len(spy.bytes_in)} calls for "
        f"{result['device_to_host']} notifications",
    )
    check(
        spy.bytes_in == result["payloads"],
        "every payload reached the session byte-identical and IN ORDER",
    )
    check(
        transport.notifications == result["device_to_host"],
        "the transport counted every notification it delivered",
    )

    # --- the link, both ends ---------------------------------------------
    check(len(spy.link_up) == 1 and spy.link_up[0][0] == 247,
          "on_link_up() was called once, with the MTU the platform gave")
    check(
        len(spy.link_down) == 1
        and spy.link_down[0] == int(T.LinkDownCause.LOCAL_REQUEST),
        "on_link_down() was called once, classified LOCAL_REQUEST for a teardown "
        "we asked for",
    )
    check(client.stopped_notify, "notifications were unsubscribed before the link went")
    # ⚠ `on_link_down()` PRODUCES — the event carrying the recovery advice, and
    # the chunk a recording wants as its last line.  A transport that reports the
    # drop and then stops draining loses the single most useful event of a failed
    # session, and the loss is silent.
    down = [e for e in result["events"] if e.type == T.EventType.LINK_DOWN]
    check(len(down) == 1,
          f"the LINK_DOWN event reached the sink after the drop ({len(down)})")
    if down:
        check("advice=" in down[0].text,
              f"...carrying its recovery advice — {down[0].text}")

    # --- ⛔ the bytes that left -------------------------------------------
    allowed = set(hm.command_allowlist())
    check(len(client.written) > 0,
          f"the transport wrote {len(client.written)} command(s)")
    check(
        all(w and w[0] in allowed for w in client.written),
        "⛔ every byte written is on the library's allowlist",
    )
    check(
        all(w[0] != 0xF0 for w in client.written),
        "⛔ 0xf0 never went out",
    )
    check(
        client.written[0][:1] == b"\x80",
        "bring-up opened with `80` — the protocol version, which gates features",
    )
    check(
        any(w[:2] == b"\xa0\x01" for w in client.written),
        "the stream was started with `a0 01`",
    )

    # --- ⚠ NO WRITE QUEUE OF ITS OWN --------------------------------------
    #
    # poll_writes() returns nothing at all while a history bracket is open, and a
    # transport that kept its own queue moving would put an unrelated reply into
    # a record stream that cannot be resynchronised.  This is the assertion that
    # a "helpful" refactor of the drain would break.
    inside = sum(1 for mark in client.write_marks if mark)
    check(inside == 0,
          f"nothing was written while a history bracket was open ({inside} writes "
          "inside a bracket)")

    # --- a whole retrieval, through the drain path ------------------------
    blocks = result["blocks"]
    for b in blocks:
        step = round(1.0 / b["density"]) if b["density"] > 0 else 0
        print(f"       {b['status']:<9} n={b['n']:<6} "
              f"coverage={b['coverage']:.3f} density={b['density']:.3f} "
              f"(step {step}) achieved={b['achieved_hz']:7.1f} Hz "
              f"overlap {b['overlap'][0]}/{b['overlap'][1]}")

    expected = DENSITY_SIGNATURE.get(fixture.name)
    if expected is None:
        # ⚠ A recording with nothing pinned about it is checked for shape and
        # nothing else — said out loud, because a silent pass over an unknown
        # fixture reads exactly like a verified one.
        note(f"{fixture.name} has no pinned density signature; only the "
             "transport-level checks above apply to it")
        check(len(blocks) > 0,
              f"a retrieval survived the transport's drain path — "
              f"{len(blocks)} block(s)")
    else:
        count, best_expected, worst_expected = expected
        check(len(blocks) == count,
              f"{count} retrieval(s) survived the transport's drain path "
              f"(got {len(blocks)})")
        if blocks:
            # ⚠ ASSERTED BY VALUE, and not because a magic number is nice to
            # have.  Equality with the C tool is what `test_python_replay.py`
            # pins; here there is no second run to compare against, so without
            # this the check would be satisfied by any block at all — including
            # one that reached nothing.  1.000 is index step 1, the full
            # ≈799 Hz, which is what a SWING returns; §7.3's still-wrist floor
            # is 0.125, an even one index in eight.
            #
            # ⚠ BOTH ENDS, because either alone cannot fail for the reason it
            # exists — and the two fixtures are the two regimes, which is why
            # this test runs over both.  A run where they stop separating has
            # had something undone.
            best = max(b["density"] for b in blocks)
            worst = min(b["density"] for b in blocks)
            check(abs(best - best_expected) < 1e-9,
                  f"densest pull {best:.4f} — step "
                  f"{round(1 / best) if best else '?'}, expected "
                  f"{best_expected:.4f}")
            check(abs(worst - worst_expected) < 1e-9,
                  f"sparsest pull {worst:.4f} — step "
                  f"{round(1 / worst) if worst else '?'}, expected "
                  f"{worst_expected:.4f}")

    # ⚠ The agreement between live and history, which is only evidence when the
    # sample count beside it is non-zero.
    total_samples = sum(b["overlap"][1] for b in blocks)
    total_mismatch = sum(b["overlap"][0] for b in blocks)
    if total_samples:
        check(total_mismatch == 0,
              f"live and history agreed over {total_samples} shared indices, "
              f"{total_mismatch} mismatched")
    else:
        note("live-vs-history: NO EVIDENCE — no block overlapped the live "
             "stream, which is not agreement")

    # --- the drains, and what they cost -----------------------------------
    #
    # ⚠ A ring that overflowed is data LOST, and the counters are the only thing
    # that says so.  Here they are also a statement about the pump: a drain that
    # ran often enough leaves them at zero, and one that did not cannot hide it.
    session = result["session"]
    drops = (session.dropped_live, session.dropped_events, session.dropped_wire)
    check(
        drops == (0, 0, 0),
        f"nothing was dropped — live {drops[0]}, events {drops[1]}, "
        f"wire {drops[2]}",
    )
    undelivered = (
        transport.undelivered_live_samples,
        transport.undelivered_events,
        transport.undelivered_wire_chunks,
    )
    check(
        undelivered == (0, 0, 0),
        f"every drained output reached a sink — {result['live']} live samples, "
        f"{len(result['events'])} events",
    )
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    os.environ["HACKMOTION_LIBRARY"] = sys.argv[1]

    fixtures = [Path(a) for a in sys.argv[2:]]
    if not fixtures:
        here = Path(__file__).resolve().parent / "fixtures"
        fixtures = [here / "swings.hmwire", here / "session1.hmwire"]

    missing = [f for f in fixtures if not f.is_file()]
    if missing:
        print("⚠ NOTHING WAS CHECKED: missing "
              + ", ".join(str(f) for f in missing), file=sys.stderr)
        return 3

    print("test_python_transport — the bleak transport over real device bytes")

    # Everything that needs no recording, first: the classification, the MTU
    # translation and ⛔ the refusal.
    check_classification()
    asyncio.run(check_mtu_translation())
    asyncio.run(check_allowlist_refusal())

    status = 0
    for fixture in fixtures:
        status = max(status, check_fixture(fixture))

    print(f"\n{checks - failures}/{checks} checks passed over "
          f"{len(fixtures)} recording(s)")
    if failures:
        return 1
    return status


if __name__ == "__main__":
    sys.exit(main())
