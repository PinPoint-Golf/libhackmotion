#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""hm_bench.py — the exemplar: a whole session against a real sensor.

    ./tools/hm_bench.py --out bench.hmwire --duration 120 --calibrate
    ./build/dev/hmwire verify bench.hmwire
    ./build/dev/hm_gather_replay bench.hmwire

⚠ THIS IS WHAT A CONSUMER COPIES, so every line of it is meant to be the thing
worth copying.  Connect, bring up, stream, optionally calibrate, retrieve each
swing WHILE IT IS STILL IN THE BUFFER, record the wire bytes through the
library's own writer, and report what the session made of it.

The radio is `hackmotion.bleak_transport`, which is a translator and not a
second state machine.  Everything with a deadline attached — the 30 s keepalive,
the bring-up bound, the calibration bound, the history deadline, the eviction
estimate — lives in the session's own timer table.  This file owns none of them,
and that is the point of the whole design (api-request §2.0).

=============================================================================
⚠ WHAT THIS FILE IS NOT
=============================================================================
It is NOT `tools/hm_capture.py`.  That script is FROZEN and deliberately does
not link the library: it is the only path to the device that composes its own
bytes, and pointing both at the same sensor and comparing the recordings is the
check that makes a transport session real rather than self-confirming.

=============================================================================
⛔ SAFETY
=============================================================================
Command `f0` reboots the sensor into FIRMWARE-UPDATE MODE through the ORDINARY
data characteristic, so avoiding the OTA service is not sufficient.

⚠ The protection here is STRUCTURAL, not vigilance: nothing in this file
composes a command.  Every byte that leaves came out of `poll_writes()`, and the
library built it against the allowlist in `src/hm_command.c`.  There is no
`--send` flag and there will not be one.  Never sweep or fuzz the command space;
fuzzing the DECODER is a different activity and is welcome.

exit: 0 the session ran and the recording verified, 1 something failed,
      2 usage or I/O error, 3 nothing was measured
"""

from __future__ import annotations

import argparse
import asyncio
import math
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

import hackmotion as hm  # noqa: E402
from hackmotion import _types as T  # noqa: E402
from hackmotion.bleak_transport import BleakTransport, TransportError, monotonic_us  # noqa: E402

# ⚠ EVENT-TRIGGERED RETRIEVAL, NOT DEFERRED, and the depth is why.  §7.3 puts
# the device's buffer at ~7.5 s.  A swing that is not retrieved inside that
# window is gone at any resolution, so a pull deferred to the end of a session
# can only ever reach the last few seconds — and over a wrist that has stopped
# moving it returns §7.3's 100 Hz floor and proves nothing.
#
# A struck golf swing peaks at 2,179-2,376 °/s and a deliberate wrist flick at
# 3,399 °/s (§6.4), against a few hundred for ordinary handling.  600 catches a
# swing without firing on someone picking the sensor up.
SWING_TRIGGER_DPS = 600.0
SWING_QUIET_DPS = 150.0

# Wait this long after the motion settles before asking, so the post-roll is in
# the buffer.  Still an order of magnitude inside the ~7.5 s depth.
SWING_POST_ROLL_US = 700_000

# ⚠ More than this queued at once and the oldest is at risk of eviction before
# it is served (§8.6) — the library says so with HM_EV_HISTORY_EVICTION_RISK,
# and this stops us manufacturing the situation.  A skipped swing is reported.
MAX_PENDING_PULLS = 2


def gyro_dps(sample) -> float:
    """Angular rate of the faster unit, °/s, from the library's own scaling.

    ⚠ Not re-derived from the raw counts here.  The gyro divisor is config bit 6
    and under a configuration with it clear the scale halves (§6.4); the library
    knows which configuration produced this sample and a bench tool does not.
    """
    best = 0.0
    for unit in (sample.lower_arm, sample.palm):
        g = unit.gyro_dps
        best = max(best, math.sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]))
    return best


async def prompt(text: str) -> None:
    """⚠ input() ON A THREAD, NEVER ON THE EVENT LOOP.

    A bare input() blocks the loop, so bleak dispatches no notifications until
    the user presses Enter — and every frame that arrived meanwhile is delivered
    in a burst and stamped with the time the loop resumed, not the time it
    arrived.  Measured in this project's own capture harness: 4.08 s of
    fabricated arrival times, and `session1.hmwire` still needs a relaxed
    stream-start bound to replay because of it.

    ⚠ `asyncio.to_thread` is for blocking on a user and for NOTHING ELSE.  Every
    call into the session must stay on the loop thread.
    """
    await asyncio.to_thread(input, text)


class Bench:
    def __init__(self, args, now_us=monotonic_us):
        self.args = args
        # ⚠ THE SAME CLOCK THE TRANSPORT STAMPS ARRIVALS WITH, and it has to be:
        # a sample's `host_time_us` comes out of the fit built from those stamps,
        # so the swing trigger below compares two readings of one clock.  Reading
        # `monotonic_us()` directly here would be right on hardware by accident
        # and wrong anywhere the host supplies its own — a replay, a simulator, a
        # test.  The duration budget is the one place a WALL reading is meant,
        # because that one is the operator's patience rather than the device's.
        self.now_us = now_us
        # ⚠ Both optional rings are ON here and both are OFF by default in the
        # library, deliberately:
        #   · the WIRE ring is the recording — with it off `poll_wire()` is
        #     always empty, which is an ABSENT recording rather than an empty
        #     session;
        #   · the DIGEST ring is the live-vs-history agreement check (§8.8) —
        #     with it off every block reports live_overlap_samples = 0, which
        #     means NO EVIDENCE and never agreement.
        self.session = hm.Session(
            args.device_id or "bench",
            wire_ring=hm.HM_WIRE_RING_RECOMMENDED,
            digest_ring=hm.HM_DIGEST_RING_RECOMMENDED,
        )
        self.recorder: hm.Recorder | None = None
        self.transport: BleakTransport | None = None

        self.ready = asyncio.Event()
        self.stream_started = asyncio.Event()

        # --- swing detection, over live samples ---------------------------
        self.peak: tuple[int, float] | None = None  # (host_us, |ω|)
        self.quiet_since_us: int | None = None
        self.live_samples = 0
        self.peak_dps = 0.0

        # --- what came back -----------------------------------------------
        self.pending: set[int] = set()
        self.blocks: list[dict] = []
        self.swings_seen = 0
        self.swings_skipped = 0
        self.warnings: dict[str, int] = {}
        self.pinned_events = 0

    # --- sinks the transport drains into ---------------------------------
    def on_wire(self, chunks) -> None:
        if self.recorder is not None:
            # ⚠ The library's own container writer, not a second one.  Anything
            # written here goes through `record/hm_record.c`, so there is no
            # second implementation of the format to keep in step.
            self.recorder.write(chunks)

    def on_event(self, events) -> None:
        for event in events:
            kind = event.type
            if kind == T.EventType.READY:
                self.ready.set()
            elif kind == T.EventType.STREAM_STARTED:
                self.stream_started.set()
            elif kind == T.EventType.HISTORY_READY:
                # ⚠ WHICH request is ready is in the event's payload union, and
                # the binding does not decode that union yet — `event.text`
                # renders it, `_types.py` does not type it.  So the pattern here
                # is the one that needs nothing from it: keep the ids we
                # reserved, and ask each.  `history_collect()` answers PENDING
                # for the ones that are not ready, which is not an error.
                self.collect_ready()
            elif kind == T.EventType.WARNING:
                # The library renders the code's name; the binding keeps no
                # second copy of the warning list.
                parts = event.text.split()
                name = parts[1] if len(parts) > 1 else "warning"
                self.warnings[name] = self.warnings.get(name, 0) + 1
                print(f"  ⚠ {event.text}")
                continue
            elif kind == T.EventType.PINNED_SAMPLES:
                # ⚠ Silent saturation at the sensor (§6.4).  The per-channel
                # counts are in the undecoded payload; that it HAPPENED is the
                # part a bench must not lose.
                self.pinned_events += 1
            elif kind in (
                T.EventType.DEVICE_ERROR,
                T.EventType.HISTORY_EVICTION_RISK,
                T.EventType.CLOCK_DEGRADED,
                T.EventType.MTU_REJECTED,
            ):
                print(f"  ⚠ {event.text}")
                continue

            if kind in (
                T.EventType.LINK_UP,
                T.EventType.LINK_DOWN,
                T.EventType.READY,
                T.EventType.DEVICE_INFO,
                T.EventType.BATTERY,
                T.EventType.STREAM_STARTED,
                T.EventType.STREAM_STOPPED,
                T.EventType.STREAM_RESTARTED,
                T.EventType.CALIBRATION_PHASE,
                T.EventType.CALIBRATION_PRESENCE,
                T.EventType.HISTORY_STARTED,
                T.EventType.HISTORY_BLIND_SPAN,
                T.EventType.BUTTON,
            ):
                # ⚠ `event.text` is rendered by the library with identifiers
                # REDACTED, and `event.sensitive` is true for exactly the events
                # carrying a MAC or a serial — so a log sink filters without
                # having to know which those are.
                print(f"  {event.text}")

    def on_live(self, samples) -> None:
        """⚠ THE TRIGGER RUNS HERE, on the samples as they arrive.

        Retrieval has to be decided from live data, because by the time a
        session ends the swing is already out of the buffer (§7.3).
        """
        for sample in samples:
            self.live_samples += 1
            if sample.host_time_us == T.HM_TIME_UNKNOWN:
                # No fit yet — a sample with no host time cannot anchor a window.
                continue
            omega = gyro_dps(sample)
            self.peak_dps = max(self.peak_dps, omega)

            if omega >= SWING_TRIGGER_DPS:
                if self.peak is None or omega > self.peak[1]:
                    self.peak = (sample.host_time_us, omega)
                self.quiet_since_us = None
            elif self.peak is not None and omega < SWING_QUIET_DPS:
                if self.quiet_since_us is None:
                    self.quiet_since_us = sample.host_time_us

        self.maybe_pull()

    # --- retrieval --------------------------------------------------------
    def maybe_pull(self) -> None:
        if self.peak is None or self.quiet_since_us is None:
            return
        if self.now_us() - self.quiet_since_us < SWING_POST_ROLL_US:
            return

        peak_us, peak_omega = self.peak
        self.peak = None
        self.quiet_since_us = None
        self.swings_seen += 1

        if len(self.pending) >= MAX_PENDING_PULLS:
            # ⚠ Said out loud.  A silently dropped swing is indistinguishable
            # from a session with no swings in it.
            self.swings_skipped += 1
            print(f"  swing #{self.swings_seen}: {len(self.pending)} pulls already "
                  "queued — NOT retrieved, it would risk eviction (§8.6)")
            return

        # ⚠ The vendor-recommended sizing (§7.6): 3 s pre-roll, 1.5 s post-roll.
        # Do NOT ask for the whole buffer — an over-wide request comes back
        # HOLED rather than clamped, at 33-58 % coverage.
        request = hm.request_around(peak_us, user_tag=self.swings_seen)
        # ⚠ ZERO DISABLES THE ALIGNMENT GATE AND KEEPS THE PULL, which is what a
        # bench wants: the block still carries every index, every device time,
        # the coverage and the fit with its flags saying what it is worth.  A
        # consumer aligning against a camera sets this to the frame period
        # instead (a 240 fps frame is 4,167 µs) and gets a refusal in the
        # provenance rather than a silently misaligned trace.
        request.alignment_budget_us = 0

        try:
            request_id = self.session.history_reserve(request)
        except hm.HackMotionError as exc:
            print(f"  swing #{self.swings_seen}: reserve refused: {exc.detail}")
            return
        self.pending.add(request_id)
        print(f"  swing #{self.swings_seen}: peak |ω| {peak_omega:.0f} °/s — "
              f"retrieving 3.0 s before to 1.5 s after, now (id={request_id})")
        if self.transport is not None:
            self.transport.wake()

    def collect_ready(self) -> None:
        """Ask every outstanding request whether it has finished.

        ⚠ `history_collect()` answers PENDING while a request is still in
        flight, which is a status and not an error — so asking one that is not
        ready costs nothing and the set stays the only bookkeeping needed.
        """
        for request_id in sorted(self.pending):
            block = self.session.history_collect(request_id)
            if block is None:
                continue
            self.pending.discard(request_id)
            # ⚠ The block owns library memory.  The `with` is what frees it;
            # anything that has to outlive it must be copied first.
            with block:
                self.report_block(block)
                self.blocks.append(
                    {
                        "status": hm.history_status_name(block.status),
                        "sample_count": block.sample_count,
                        "coverage_fraction": block.coverage_fraction,
                        "density": block.density,
                        "achieved_hz": block.achieved_hz,
                    }
                )

    def report_block(self, block) -> None:
        """⚠ EVERY DERIVED NUMBER WITH THE RAW ONE IT CAME FROM BESIDE IT.

        That is the house rule that turned `density` from a report into
        evidence: it printed 1.000 on every pull across two sessions and seven
        retrievals and was read as confirmation, when a number that never varies
        is a check that has stopped running.  The median step beside it is how
        you can tell 1.000-because-it-is-a-swing from 1.000-because-it-is-stuck.
        """
        requested = block.requested_indices
        width = requested.last - requested.first + 1
        delivered = sum(hi - lo + 1 for lo, hi in block.delivered_indices)
        step = round(1.0 / block.density) if block.density > 0.0 else 0
        span_s = 0.0
        if block.delivered_count:
            span_s = (block.delivered[-1][1] - block.delivered[0][0]) / 1e6

        print(f"    block  {hm.history_status_name(block.status)}  "
              f"n={block.sample_count}  attempts={block.attempts}  "
              f"tag={block.user_tag}")
        print(f"      coverage  {block.coverage_fraction:.3f}   "
              f"{delivered} of {width} requested indices arrived")
        if step:
            print(f"      density   {block.density:.3f}   median delivered index "
                  f"step {step}   (§7.3: step 1 ≈ 799 Hz, step 8 ≈ 100 Hz — a "
                  f"still wrist)")
        else:
            print("      density   NOT MEASURABLE — fewer than two delivered "
                  "indices, which is not the same as 0")
        print(f"      achieved  {block.achieved_hz:.1f} Hz averaged over "
              f"{span_s:.3f} s of delivered span, in {block.delivered_count} "
              f"interval(s)")
        print(f"      gaps      {block.gap_count}, largest "
              f"{block.largest_gap_us // 1000} ms"
              + ("   ⚠ the interval list OVERFLOWED and is a SUPERSET"
                 if block.coverage_overflowed else ""))

        # ⚠ 0 samples is NO EVIDENCE, not agreement — the two must never print
        # the same way.
        if block.live_overlap_samples:
            print(f"      overlap   {block.live_overlap_mismatches} mismatched out "
                  f"of {block.live_overlap_samples} live indices checked (§8.8)")
        else:
            print("      overlap   NO EVIDENCE — this pull covered a span the "
                  "live stream never reached")

        fit = block.fit
        if fit.flags & T.ClockFlag.HAS_FIT and block.sample_count:
            # ⚠ The block's OWN fit, not the session's current one.  The mapping
            # is piecewise: the device's sample counter stalls for the duration
            # of every retrieval, so the fit re-anchors at each one and a block
            # is dated by the fit in force when its window closed (§6.1.1).
            error = hm.clock_error_at(fit, block.samples[-1].sample_index)
            print(f"      clock     precision {error.precision_us} µs, "
                  f"systematic {error.systematic_us} µs, total {error.total_us} µs "
                  f"— ⚠ gate on precision, record the total")

        span = block.calibration
        print(f"      calibrated  {T.CalibrationState(span.state_at_start).name}"
              f" → {T.CalibrationState(span.state_at_end).name}")

    # --- the session ------------------------------------------------------
    async def await_phase(self, wanted: set, timeout_s: float) -> bool:
        """Wait for the calibration machine to reach one of `wanted`.

        ⚠ Polling the phase rather than a callback, because the library never
        invokes one — events are queued and the host drains them, which is what
        makes the teardown barrier "stop draining" (AR §2.0.3).
        """
        deadline = monotonic_us() + int(timeout_s * 1e6)
        while monotonic_us() < deadline:
            if self.session.calibration_phase in wanted:
                return True
            await asyncio.sleep(0.05)
        return self.session.calibration_phase in wanted

    async def calibrate(self) -> None:
        """§8.2's two-marker routine, driven by the library's phase machine.

        ⚠ The device observes a CONTINUOUS RAISE between the two markers, so the
        stream must already be running — it cannot be done from two static
        samples, which is why `calibration_begin()` refuses without a stream
        rather than waiting for one.

        ⚠ AND `0x94` IS NOT A VERDICT.  The device applies the transform for
        every attempt, including ones an application would reject.  Only the
        presence check at the end writes CALIBRATED, so skipping step 3 leaves
        every later sample at UNKNOWN — permanently and invisibly.
        """
        print("\n  CALIBRATION")
        try:
            self.session.calibration_begin()
            await self.transport.flush()

            await prompt("    1. Forearm horizontal, wrist straight, then press "
                         "Enter... ")
            self.session.calibration_confirm_horizontal()
            await self.transport.flush()

            print("    2. Now raise the forearm ~30° across the chest, smoothly.")
            await prompt("       Press Enter when the raise is complete... ")
            self.session.calibration_confirm_raise()
            await self.transport.flush()

            # ⚠ WAIT FOR THE DEVICE, THEN ASK THE USER.  The reference-pose call
            # is only legal at VERIFYING — the phase reached once `0x94` has
            # arrived and the transform is already applied.  Prompting first and
            # calling early gets HM_ERR_INVALID_STATE, and skipping the call
            # leaves every later sample at UNKNOWN.
            if not await self.await_phase({T.CalibrationPhase.VERIFYING}, 10.0):
                print(f"    ⚠ the device did not return a result — phase "
                      f"{self.session.calibration_phase.name}")
            else:
                await prompt("    3. Return to the reference pose and HOLD it, "
                             "then press Enter... ")
                self.session.calibration_confirm_reference_pose()
                await self.transport.flush()
                # ⚠ The call above returns BEFORE the measurement exists; the
                # presence event carries it.  Reading the state straight after
                # would report what was true before the check ran.
                await self.await_phase(
                    {T.CalibrationPhase.COMPLETE, T.CalibrationPhase.ABORTED}, 5.0
                )
        except hm.HackMotionError as exc:
            print(f"    ⚠ calibration refused: {exc.detail}")

        angle = self.session.presence_angle_deg
        state = self.session.calibration_state
        print(f"    phase {self.session.calibration_phase.name}, "
              f"state {state.name}, reference-pose angle "
              + ("not measured" if math.isnan(angle) else f"{angle:.2f}°"))
        if state != T.CalibrationState.CALIBRATED:
            # ⚠ UNKNOWN is not a hopeful CALIBRATED.
            print("    ⚠ NOT calibrated as far as this library is concerned. The "
                  "device may still have applied a transform — `0x94` is not a "
                  "verdict — but nothing measured that it took.")
        print("    ⚠ The angle is a PRESENCE check and never a score: §8.2 "
              "measured it ranking a no-raise calibration BEST.")

    async def run(self) -> int:
        args = self.args

        device = await BleakTransport.discover(
            address=args.address,
            adapter=args.adapter,
            timeout_us=int(args.scan * 1e6),
            on_armed=lambda: (
                print(f"  scanner armed, {args.scan:.0f} s window."),
                print("  ⚠ PRESS THE SENSOR'S BUTTON NOW."),
                print("    If the sensor was asleep the first press only WAKES it "
                      "and it does"),
                print("    not advertise — press, pause, then press again (§2.1)."),
            ),
        )
        print(f"  found {getattr(device, 'address', device)}")

        self.recorder = hm.Recorder(
            args.out,
            device_id=args.device_id or "bench",
            config_bits=self.session.stream_config().bits,
            record_identifiers=args.record_identifiers,
        )

        self.transport = BleakTransport(
            self.session,
            device,
            adapter=args.adapter,
            now_us=self.now_us,
            on_event=self.on_event,
            on_live=self.on_live,
            on_wire=self.on_wire,
        )

        try:
            await self.transport.connect()
            print("  connected (the device vibrates — §9.5)")
            if self.transport.mtu == 0:
                print("  ⚠ the platform will not report an ATT MTU — proceeding "
                      "UNVERIFIED.")
                print("    The frames settle it: a 47- or 93-byte notification "
                      "cannot arrive")
                print("    over a 23-byte MTU, and the summary below says what "
                      "actually turned up.")
            else:
                print(f"  ATT MTU {self.transport.mtu} (§2.4 needs ≥ "
                      f"{hm.HM_MIN_ATT_MTU})")

            try:
                await asyncio.wait_for(self.ready.wait(), 15.0)
            except asyncio.TimeoutError:
                print("  ⚠ bring-up did not reach READY — the session says why "
                      "above")
                return 1

            # ⚠ ONE STREAM, OPENED ONCE, LEFT OPEN.  Calibration needs it open,
            # the clock fit needs it continuous, and a retrieval works in place —
            # so nothing here ever stops it.  A restart drops calibration back to
            # UNKNOWN.
            try:
                self.session.start_stream()
            except hm.HackMotionError as exc:
                print(f"  ⚠ the session refused to start a stream: {exc.detail}")
                return 1
            await self.transport.flush()
            try:
                await asyncio.wait_for(self.stream_started.wait(), 5.0)
            except asyncio.TimeoutError:
                # ⚠ STREAM_STARTED is the first `90` FRAME, not the `a0 01`
                # acknowledgement — the device can accept the command and send
                # nothing, and that difference is the whole of §6.1's watchdog.
                print("  ⚠ no stream frames arrived")
                return 1

            if args.calibrate:
                await self.calibrate()

            print(f"\n  recording for {args.duration:.0f} s — Ctrl-C to stop early")
            print("  ⚠ swing the club.  Every fast motion is retrieved as it "
                  "happens; a swing")
            print("    that is not pulled inside the ~7.5 s buffer is gone at any "
                  "resolution (§7.3).")
            print("    Hold still for a while too — the still-wrist pull is the "
                  "control that")
            print("    separates a working full-rate path from a broken one.")

            deadline = monotonic_us() + int(args.duration * 1e6)
            try:
                while monotonic_us() < deadline and not self.transport.disconnected.is_set():
                    await asyncio.sleep(0.1)
                    # The transport's pump drives everything; these only give the
                    # host's own logic a look-in when no live samples are
                    # arriving — during a retrieval, live delivery is suspended.
                    self.maybe_pull()
                    self.collect_ready()
            except (asyncio.CancelledError, KeyboardInterrupt):
                # ⚠ Caught HERE rather than at the top: the teardown below is
                # what gets the `83` out, closes the recording and classifies
                # the drop.  A Ctrl-C that unwound past this point would leave a
                # device streaming into nothing and a half-written capture.
                print("\n  interrupted")

            if self.pending:
                print(f"  waiting for {len(self.pending)} retrieval(s) to finish...")
                for _ in range(200):
                    if not self.pending:
                        break
                    await asyncio.sleep(0.1)
                    self.collect_ready()

            # ⚠ NEITHER OF THESE MAY THROW PAST HERE.  The recording is closed in
            # the `finally` below, and a session that has already stopped
            # streaming — because the link went, because the stream watchdog
            # fired — refuses `stop_stream()` with HM_ERR_INVALID_STATE.  Letting
            # that escape would trade a tidy teardown for a truncated capture,
            # which is the more expensive of the two by a long way.
            if not self.transport.disconnected.is_set():
                try:
                    self.session.stop_stream()
                    await self.transport.flush()
                    await asyncio.sleep(0.3)
                except hm.HackMotionError as exc:
                    print(f"  ⚠ stop_stream: {exc.detail} — the stream had "
                          "already ended")
                if args.power_off:
                    # ⚠ §9.3: no acknowledgement arrives, the link stays up ~9 s,
                    # and the device then needs a PHYSICAL BUTTON PRESS to come
                    # back.  A teardown that reads the gap as failure and retries
                    # into it will be wrong.
                    try:
                        self.session.power_off()
                        await self.transport.flush()
                        print("  powered off — the device now needs a physical "
                              "button press (§9.3)")
                        await asyncio.sleep(1.0)
                    except hm.HackMotionError as exc:
                        print(f"  ⚠ power_off: {exc.detail}")
        except TransportError as exc:
            print(f"\n  ⚠ {exc}")
            print(f"    classification: {exc.cause.name}")
            return 1
        finally:
            if self.transport is not None:
                # One last pump, then the radio goes and the drop is classified.
                await self.transport.close()

            # ⚠ DRAIN THE WIRE RING BEFORE close(), NOT AFTER.  The stop barrier
            # CLEARS the live and wire rings by contract — bulk data the consumer
            # chose to abandon by closing — so a chunk still queued at that
            # instant is gone, including the `link_down` the line above just
            # produced.  The EVENT ring is deliberately not cleared, which is
            # what makes the drain below work.
            if self.recorder is not None:
                self.recorder.write(self.session.poll_wire())

            # ⚠ CLOSE FINISHES WORK, IT DOES NOT DISCARD IT.  Every outstanding
            # reservation materialises here as CANCELLED carrying whatever had
            # arrived, and stays collectable — so a session that ends mid-pull
            # still records what it got rather than losing it.
            self.session.close()
            self.session.poll_events()
            self.collect_ready()
            if self.recorder is not None:
                self.recorder.close()

        return 0

    # --- the report -------------------------------------------------------
    def report(self) -> None:
        t = self.transport
        print("\nsession\n-------")
        if t is not None:
            print(f"  notifications  {t.notifications} in, {t.writes} commands out "
                  f"({t.bytes_in} B / {t.bytes_out} B)")
            if t.max_notification_len:
                # ⚠ The MTU check the platform refused to answer, answered by the
                # data: an ATT notification is opcode + handle + value, so a
                # payload of N bytes proves the MTU was at least N + 3.
                implied = t.max_notification_len + 3
                verdict = ("met, measured not assumed"
                           if implied >= hm.HM_MIN_ATT_MTU
                           else "NOT shown — see below")
                print(f"  largest frame  {t.max_notification_len} B → the ATT MTU "
                      f"was at least {implied}; §2.4's floor of "
                      f"{hm.HM_MIN_ATT_MTU} is {verdict}")
                if implied < hm.HM_MIN_ATT_MTU:
                    print("    That is expected if no 93-byte two-record frame "
                          "occurred; it is a")
                    print("    TRUNCATION if `hmwire reconcile` also reports odd "
                          "frame lengths.")
        print(f"  live samples   {self.live_samples}, peak |ω| {self.peak_dps:.0f} °/s")

        # ⚠ THE THREE DROP COUNTERS, LOUDLY.  Non-zero means the host did not
        # keep up and data was LOST — never let this be silent.
        drops = (self.session.dropped_live, self.session.dropped_events,
                 self.session.dropped_wire)
        line = (f"  drops          live {drops[0]}, events {drops[1]}, "
                f"wire {drops[2]}")
        print(line + ("   ⚠ NON-ZERO: this host did not keep up and data was LOST"
                      if any(drops) else "   (none)"))
        if t is not None and (t.undelivered_wire_chunks or t.undelivered_events
                              or t.undelivered_live_samples):
            print(f"  ⚠ undrained    {t.undelivered_wire_chunks} wire chunks, "
                  f"{t.undelivered_events} events, {t.undelivered_live_samples} "
                  "live samples had no sink")
        if self.pinned_events:
            print(f"  ⚠ pinned       {self.pinned_events} report(s) of channels "
                  "saturating — silently, at the sensor (§6.4)")

        # ⚠ PYTHON IS A WORSE CLOCK THAN C, AND THIS IS THE NUMBER THAT SAYS BY
        # HOW MUCH.  Report it rather than assuming the lower-envelope fit
        # absorbed the event loop's jitter — it is built for one-sided delay and
        # is robust to it, which is not the same as being unaffected.
        print("\nclock\n-----")
        snapshot = self.session.clock_or_none()
        if snapshot is None or not (snapshot.flags & T.ClockFlag.HAS_FIT):
            print("  NO FIT — nothing here was time-aligned")
        else:
            error = hm.clock_error_at(snapshot, snapshot.last_index)
            print(f"  precision      {error.precision_us} µs   ⚠ THE NUMBER TO "
                  f"GATE ON, and what an asyncio loop costs you")
            print(f"  systematic     {error.systematic_us} µs   accumulated drift "
                  f"since the anchor")
            print(f"  total          {error.total_us} µs   ⚠ record it, never "
                  f"gate on it")
            print(f"  residuals      median {snapshot.residual_median_us} µs, "
                  f"p90 {snapshot.residual_p90_us} µs, "
                  f"max {snapshot.residual_max_us} µs")
            print(f"  fit            {snapshot.fitted_rate_hz:.3f} Hz over "
                  f"{snapshot.span_us / 1e6:.1f} s, {snapshot.observations} "
                  f"observations")
            flags = [f.name for f in T.ClockFlag if snapshot.flags & f]
            print(f"  flags          {' '.join(flags)}")

        print("\nretrieval\n---------")
        print(f"  swings seen    {self.swings_seen}"
              + (f", {self.swings_skipped} NOT retrieved" if self.swings_skipped else ""))
        if not self.blocks:
            print("  ⚠ NO BLOCKS.  Nothing about retrieval was measured here — "
                  "that is an")
            print("    absence of evidence, not a clean result.")
        else:
            tally: dict[str, int] = {}
            for b in self.blocks:
                tally[b["status"]] = tally.get(b["status"], 0) + 1
            print(f"  blocks         {len(self.blocks)}: "
                  + ", ".join(f"{n} {name}" for name, n in sorted(tally.items())))
            # ⚠ 0.0 IS "NOT MEASURABLE", NEVER "EMPTY" — a block with fewer than
            # two delivered indices has no spacing to report, and printing 0.000
            # beside real densities would read as the worst measurement rather
            # than as no measurement.
            measurable = [b["density"] for b in self.blocks if b["density"] > 0.0]
            if not measurable:
                print("  density        NO READING — no block held two delivered "
                      "indices to space")
            else:
                lo, hi = min(measurable), max(measurable)
                print(f"  density        {lo:.3f} (step {round(1 / lo)}) to "
                      f"{hi:.3f} (step {round(1 / hi)}) over {len(measurable)} "
                      f"of {len(self.blocks)} block(s)")
                if abs(hi - lo) < 1e-9:
                    # ⚠ A number that never varies is not a measurement; it is a
                    # check that has stopped running.  This project published one
                    # across seven real retrievals before noticing.
                    print("                 ⚠ every block read the same — with a "
                          "swing and a still wrist in the same session these "
                          "should SEPARATE (1.000 vs 0.125)")

        if self.warnings:
            print("\nwarnings\n--------")
            for name, count in sorted(self.warnings.items()):
                print(f"  {name:<34} {count}")

        if t is not None and t.link_down is not None:
            print(f"\nlink down\n---------\n  {t.link_down}")


def find_hmwire(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    for candidate in (ROOT / "build/dev/hmwire", ROOT / "build/rel/hmwire"):
        if candidate.is_file():
            return str(candidate)
    return shutil.which("hmwire")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="A whole HackMotion wrist-sensor session through libhackmotion.",
        epilog="Then: hmwire verify FILE && hm_gather_replay FILE",
    )
    ap.add_argument("--out", default="bench.hmwire", help="output .hmwire path")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds to stream")
    ap.add_argument("--scan", type=float, default=90.0,
                    help="discovery window; §2.1 says be generous, it costs nothing")
    ap.add_argument("--address", help="skip discovery and connect to this address")
    ap.add_argument("--adapter", help="e.g. hci0 (Linux)")
    ap.add_argument("--device-id", help="opaque label for the recording; ⚠ never a MAC")
    ap.add_argument("--calibrate", action="store_true",
                    help="run §8.2's two-marker routine with prompts")
    ap.add_argument("--power-off", action="store_true",
                    help="⚠ the device then needs a physical button press")
    ap.add_argument("--record-identifiers", action="store_true",
                    help="⚠ keep the MAC and serial replies in the recording")
    ap.add_argument("--hmwire", default=None, help="path to the built hmwire tool")
    args = ap.parse_args()

    print(f"hm_bench — libhackmotion {hm.VERSION}, abi {hm.ABI_VERSION}")
    print(f"  library  {hm.library_path}")
    # ⛔ Printed, because a reviewer should be able to see the whole set of bytes
    # this process can possibly emit without reading any code.
    print("  allowlist " + " ".join(f"{b:02x}" for b in hm.command_allowlist()))

    bench = Bench(args)
    try:
        status = asyncio.run(bench.run())
    except TransportError as exc:
        print(f"\nhm_bench: {exc}")
        return 1
    except KeyboardInterrupt:
        status = 0
    bench.report()

    hmwire = find_hmwire(args.hmwire)
    if hmwire is None:
        print("\n⚠ no `hmwire` tool found, so the recording was NOT verified.")
        return status or 3
    print(f"\nverifying {args.out}")
    if subprocess.run([hmwire, "verify", args.out]).returncode != 0:
        print("hm_bench: ⚠ the recording did not verify — see above")
        return 1
    replay = ROOT / "build/dev/hm_gather_replay"
    print(f"\nNext: {replay if replay.is_file() else 'hm_gather_replay'} {args.out}")

    if status == 0 and not bench.blocks:
        # ⚠ A run that measured nothing must not exit 0 looking like a clean one.
        return 3
    return status


if __name__ == "__main__":
    sys.exit(main())
