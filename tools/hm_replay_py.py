#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
"""hm_replay_py.py — tools/hm_gather_replay.c, through the Python binding.

    ./tools/hm_replay_py.py tests/fixtures/swings.hmwire

⚠ WHY A SECOND COPY OF A TOOL THAT ALREADY EXISTS.  This is the only thing that
checks the binding against real device bytes.  Every other Python test compares
declarations; this one drives the actual gather over a real capture and has to
arrive at the same numbers the C tool does — the same six blocks, the same
densities, the same step histogram.  A ctypes field at the wrong offset agrees
with the compiler about nothing here.

⚠ IT IS A TRANSLATION, NOT A REIMPLEMENTATION, and the fidelity is the point.
Every decision below is the C tool's decision, at the same place and for the
same reason.  Where it looks like it could be simplified, read the comment in
hm_gather_replay.c before simplifying it — several of these lines are load
bearing and none of them is obvious.

⚠ AND IT IS NOT A CAPTURE TOOL.  It reads a recording; it never touches a radio.

exit: 0 checks passed, 1 a check FAILED, 2 usage or I/O error,
      3 nothing was checked
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

import hackmotion as hm  # noqa: E402
from hackmotion import _types as T  # noqa: E402

# §7.3's own buckets: step 1 is a swing, step 8 is a still wrist, and nothing
# above 8 was seen in 17,739 measured steps.
STEP_NAMES = ["1  (≈799 Hz)", "2  (≈400 Hz)", "3-5", "6-7", "8  (≈100 Hz)", ">8 ⚠"]


def step_bucket(step: int) -> int:
    if step <= 1:
        return 0
    if step == 2:
        return 1
    if step <= 5:
        return 2
    if step <= 7:
        return 3
    if step == 8:
        return 4
    return 5


class Check:
    """⚠ A check that never ran and a check that passed are DIFFERENT ANSWERS.
    Every verdict carries how many observations it rests on, and zero of them is
    NO EVIDENCE rather than a pass."""

    def __init__(self, what: str):
        self.what = what
        self.observed = 0
        self.failed = 0

    def note(self, ok: bool) -> None:
        self.observed += 1
        if not ok:
            self.failed += 1

    def report(self) -> int:
        if self.observed == 0:
            print(f"  [ ---- ] {self.what:<50} NO EVIDENCE — nothing was checked")
            return 0
        verdict = "  ok  " if self.failed == 0 else " FAIL "
        print(
            f"  [{verdict}] {self.what:<50} {self.observed} checked, "
            f"{self.failed} failed"
        )
        return 1 if self.failed else 0


def unwrap_near(head: int, raw16: int) -> int:
    """⚠ The recording holds what went on the wire, and that is u16 big-endian.

    The session works in UNWRAPPED indices, so a recorded `a1` pair has to be
    lifted back before it can become a host-time window.  Lifted to the index
    NEAREST the head rather than at-or-below: swings.hmwire's sixth pull asks 200
    indices PAST the head, and rounding that down a whole wrap would hide the
    very thing the clamp reports."""
    delta = ((raw16 - head) & 0xFFFF)
    if delta >= 0x8000:
        delta -= 0x10000
    return (head + delta) & 0xFFFFFFFF


class Replayer:
    def __init__(self):
        # ⚠ The live-vs-history digest ring is OFF by default, and with it off a
        # block reports live_overlap_samples = 0 — which means NO EVIDENCE, not
        # agreement.  This run wants the evidence.
        #
        # ⚠ THE STREAM-START BOUND IS RELAXED HERE, AND ONLY WHERE A RECORDING IS
        # BEING REPLAYED — here and in tests/test_python_transport.py.  The library's
        # 3 s default is two orders of margin over the 50-80 ms a real device
        # takes.  session1.hmwire's first frame arrived at 4.08 s — a property of
        # the RECORDER, whose harness blocked its own event loop, not of the
        # device.  Enforcing a live-session bound against a recording would
        # discard the only capture this project holds of a retrieval past two
        # index wraps, and would be enforcing it against the wrong party.
        self.session = hm.Session(
            "replay",
            digest_ring=hm.HM_DIGEST_RING_RECOMMENDED,
            policy={"stream_start_timeout_us": 15_000_000},
        )
        self.pending_id = 0
        self.blocks: list[dict] = []
        self.steps = [0] * 6
        self.status_count: dict[int, int] = {}
        self.a1_seen = 0
        self.reserved = 0
        self.refused = 0

        self.dated = Check("every sample dated by its block's own fit")
        self.ascending = Check("samples ascending, strictly monotonic in index")
        self.history_src = Check("every sample marked history, with no arrival time")
        self.floor = Check("no delivered step above §7.3's floor of 8")
        self.overlap = Check("live-vs-history agreement (§8.8)")
        self.depth = Check("buffer depth MEASURED, not seeded (§8.5)")
        self.overlap_samples = 0
        self.overlap_mismatches = 0

        self.depth_owed = False
        self.depth_width = 0
        self.depth_measured = 0
        self.depth_seeded = 0
        self.worst_density: float | None = None

    # --- the same three drains the C tool performs ------------------------
    def drain(self) -> None:
        while self.session.poll_writes(8):
            pass
        while self.session.poll_events(16):
            pass
        while len(self.session.poll_live(64)):
            pass

    def collect_if_ready(self) -> None:
        if self.pending_id == 0:
            return
        block = self.session.history_collect(self.pending_id)
        if block is None:
            return
        with block:
            self.pending_id = 0
            self.depth_owed = True
            self.status_count[block.status] = self.status_count.get(block.status, 0) + 1

            print("  block   " + block.summary())
            if block.coverage_overflowed:
                print("          ⚠ coverage storage overflowed: the interval list "
                      "is a SUPERSET")

            if self.worst_density is None or block.density < self.worst_density:
                self.worst_density = block.density

            self.overlap_samples += block.live_overlap_samples
            self.overlap_mismatches += block.live_overlap_mismatches
            if block.live_overlap_samples > 0:
                # ⚠ ONE observation of the agreement, carrying its own sample
                # count — a zero mismatch count means nothing without it.
                self.overlap.note(block.live_overlap_mismatches == 0)

            samples = block.samples
            fit = block.fit
            previous = None
            for sample in samples:
                # ⚠ The block is internally reproducible: the fit it carries is
                # the one that dated every sample in it.
                self.dated.note(
                    sample.host_time_us
                    == hm.clock_to_host_us(fit, sample.sample_index)
                )
                # Bulk arrival timestamps carry no information at all.
                self.history_src.note(
                    sample.source == T.SampleSource.HISTORY
                    and sample.host_recv_us == T.HM_TIME_UNKNOWN
                )
                if previous is not None:
                    self.ascending.note(sample.sample_index > previous)
                    if sample.sample_index > previous:
                        step = sample.sample_index - previous
                        self.steps[step_bucket(step)] += 1
                        self.floor.note(step <= 8)
                previous = sample.sample_index

            self.blocks.append(
                {
                    "status": int(block.status),
                    "status_name": hm.history_status_name(block.status),
                    "sample_count": block.sample_count,
                    "coverage_fraction": block.coverage_fraction,
                    "density": block.density,
                    "achieved_hz": block.achieved_hz,
                    "largest_gap_us": block.largest_gap_us,
                    "gap_count": block.gap_count,
                    "delivered_count": block.delivered_count,
                    "live_overlap_samples": block.live_overlap_samples,
                    "live_overlap_mismatches": block.live_overlap_mismatches,
                    "attempts": block.attempts,
                    "coverage_overflowed": block.coverage_overflowed,
                    "requested_first": block.requested_indices.first,
                    "requested_last": block.requested_indices.last,
                    "first_index": samples[0].sample_index if len(samples) else 0,
                    "last_index": samples[-1].sample_index if len(samples) else 0,
                }
            )

    def depth_sample(self) -> None:
        """⚠ The reading cannot be taken at collection time: the fit re-anchors
        at every bracket close, so until the next live frame lands there is no
        mapping from the buffer's head to a host time and the query rightly
        refuses."""
        if not self.depth_owed:
            return
        try:
            status, span = self.session.history_resident_range()
        except hm.HackMotionError:
            # ⚠ NOT AN ANSWER, SO NOT AN OBSERVATION.  No fit means the bracket
            # just re-anchored; no stream means the capture ended with the pull.
            # Counting either as a failure would report "never checked" as
            # though it were "checked and wrong".
            return
        self.depth_owed = False
        # ⚠ PENDING is a FAILURE here rather than an absence: a pull happened, so
        # something should have been learned from it.
        self.depth.note(status == T.Status.OK)
        if status == T.Status.OK:
            self.depth_measured += 1
            self.depth_width = span.end_us - span.start_us
        else:
            self.depth_seeded += 1

    def on_recorded_a1(self, chunk) -> None:
        first = (chunk.data[1] << 8) | chunk.data[2]
        last = (chunk.data[3] << 8) | chunk.data[4]

        self.collect_if_ready()

        snapshot = self.session.clock_or_none()
        if snapshot is None or not (snapshot.flags & T.ClockFlag.HAS_FIT):
            snapshot = snapshot or T.hm_clock_snapshot()
            print(f"  a1 [{first}..{last}]  no fit (flags=0x{snapshot.flags:02x}, "
                  f"observations={snapshot.observations})")
            self.refused += 1
            return

        last = unwrap_near(snapshot.last_index, last)
        # `first` is lifted relative to `last`, not to the head, so a range that
        # straddles a wrap stays one ascending range.
        first = (last - ((last - first) & 0xFFFF)) & 0xFFFFFFFF

        # ⚠ Clamped to the head the device had actually reached — this models
        # what a library-driven consumer asks for, not what the recording harness
        # asked for.  swings.hmwire contains one pull whose `a1` ran 200 indices
        # PAST the head, and a reservation would rightly have held it.
        if last > snapshot.last_index:
            print(f"  a1 [{first}..{last}]  ⚠ asked {last - snapshot.last_index} "
                  f"indices past the head; clamped to {snapshot.last_index}")
            last = snapshot.last_index

        request = T.hm_history_request()
        # ⚠ The one-microsecond nudge at each end is the half-open/inclusive
        # boundary, pushed off the rounding edge rather than sat on it.
        request.window.start_us = hm.clock_to_host_us(snapshot, first) - 1
        request.window.end_us = hm.clock_to_host_us(snapshot, last) + 1
        request.deadline_us = chunk.host_time_us + 60_000_000
        request.max_attempts = 1

        try:
            self.pending_id = self.session.history_reserve(request)
        except hm.HackMotionError as exc:
            print(f"  a1 [{first}..{last}]  reserve refused: {exc.detail}")
            self.refused += 1
            return
        self.reserved += 1
        print(f"  a1 [{first}..{last}]  reserved id={self.pending_id}")
        self.drain()

    # --- the loop ---------------------------------------------------------
    def run(self, path: Path) -> None:
        with hm.Replay(path) as replay:
            for chunk in replay:
                if chunk.direction == T.WireDirection.META:
                    text = bytes(chunk.data[: chunk.length])
                    if text.startswith(b"link_up"):
                        self.session.on_link_up(247, chunk.host_time_us)
                    elif text.startswith(b"stream_start"):
                        self.session.start_stream()
                    self.drain()
                    continue
                if chunk.direction == T.WireDirection.HOST_TO_DEVICE:
                    self.session.tick(chunk.host_time_us)
                    self.drain()
                    if chunk.length == 5 and chunk.data[0] == 0xA1:
                        self.a1_seen += 1
                        self.on_recorded_a1(chunk)
                    continue
                self.session.on_bytes(
                    bytes(chunk.data[: chunk.length]), chunk.host_time_us
                )
                self.drain()
                self.collect_if_ready()
                self.depth_sample()
        self.collect_if_ready()
        self.depth_sample()

        if self.pending_id != 0:
            print(f"  ⚠ request {self.pending_id} was never answered — pending "
                  f"{self.session.history_pending} at end of capture")


def main() -> int:
    args = [a for a in sys.argv[1:]]
    json_path = None
    if len(args) == 3 and args[1] == "--json":
        json_path = Path(args[2])
        args = args[:1]
    if len(args) != 1 or args[0].startswith("-"):
        print(__doc__)
        return 2

    path = Path(args[0])
    if not path.is_file():
        # ⚠ NOT a silent success: a run that checked nothing has to say so in the
        # same breath, or it reads as a clean bill of health.
        print(f"hm_replay_py: {path} not found.", file=sys.stderr)
        print("⚠ NOTHING WAS CHECKED.  tests/fixtures/ holds the tracked captures.",
              file=sys.stderr)
        return 3

    print(f"hm_replay_py — {path}")
    print("the history gather, driven by a real device's own bytes, "
          "through the Python binding\n")

    replayer = Replayer()
    replayer.run(path)

    print("\nblocks\n------")
    if not replayer.blocks:
        print("  none — this capture contains no retrieval the session could serve")
    for status, count in sorted(replayer.status_count.items()):
        name = hm.history_status_name(status)
        print(f"  {name:<20} {count}")

    print("\ndelivered index steps (§7.3)\n---------------------------")
    for i, count in enumerate(replayer.steps):
        if count:
            print(f"  {STEP_NAMES[i]:<14} {count}")

    print("\nbuffer depth (§8.5)\n-------------------")
    if replayer.depth_measured == 0 and replayer.depth_seeded == 0:
        print("  no reading — no pull completed with a live frame after it")
    else:
        print(f"  readings       {replayer.depth_measured} measured, "
              f"{replayer.depth_seeded} still seeded")
    if replayer.depth_measured:
        print(f"  verified reach-back  {replayer.depth_width / 1e6:.3f} s   "
              f"⚠ a LOWER bound")

    print("\nverdicts\n--------")
    failures = 0
    failures += replayer.dated.report()
    failures += replayer.ascending.report()
    failures += replayer.history_src.report()
    failures += replayer.floor.report()
    failures += replayer.overlap.report()
    print(f"           {'...over':<50} {replayer.overlap_samples} indices, "
          f"{replayer.overlap_mismatches} mismatched")
    failures += replayer.depth.report()

    density = (
        f"{replayer.worst_density:.4f}" if replayer.worst_density is not None
        else "NO READING"
    )
    print(f"\n  recorded a1 {replayer.a1_seen}, reserved {replayer.reserved}, "
          f"refused {replayer.refused}; {len(replayer.blocks)} block(s); "
          f"worst density {density}")

    if json_path is not None:
        json_path.write_text(
            json.dumps(
                {
                    "blocks": replayer.blocks,
                    "steps": {str(i): n for i, n in enumerate(replayer.steps)},
                    "a1_seen": replayer.a1_seen,
                    "reserved": replayer.reserved,
                    "refused": replayer.refused,
                    "blocks_total": len(replayer.blocks),
                    "depth_measured": replayer.depth_measured,
                    "depth_seeded": replayer.depth_seeded,
                    "depth_width_us": replayer.depth_width,
                },
                indent=2,
            )
        )

    if failures:
        return 1
    if not replayer.blocks:
        print("⚠ NOTHING WAS CHECKED: no block was produced.", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
