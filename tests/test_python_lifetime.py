#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""test_python_lifetime.py — the one hazard ctypes adds that C does not have.

A history block owns its samples and frees them on release.  In C that is a
lifetime a compiler and a sanitizer both help with.  In Python it is a raw
address held by a ctypes array, and:

⚠ A SANITIZER DOES NOT SEE IT.  AddressSanitizer instruments the code it
compiled; a ctypes read happens inside CPython, which it did not.  A view kept
past its block's release returns PLAUSIBLE SAMPLE DATA under `build/san` with no
report at all.  That was measured, not assumed, and it is why `Samples._alive()`
exists rather than a comment telling people to be careful.

So this file pins the guard.  Without it the failure is a wrong swing rather
than a crash — the worst shape this library admits.

Usage: test_python_lifetime.py <libwrist_ffi.so> [<fixture.wrwire>]
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

failures = 0
checks = 0


def check(condition, what):
    global failures, checks
    checks += 1
    print(f"[{' ok ' if condition else 'FAIL'}] {what}")
    if not condition:
        failures += 1


def raises(fn, what):
    try:
        fn()
    except RuntimeError:
        check(True, what)
        return
    except Exception as exc:  # noqa: BLE001
        check(False, f"{what} — raised {type(exc).__name__}, wanted RuntimeError")
        return
    check(False, f"{what} — nothing was raised, so the read SUCCEEDED")


def first_block(fixture: Path):
    """Drive a real capture far enough to hold one history block.

    A compressed copy of tools/wr_replay_py.py's loop — this test needs a block
    that OUTLIVES its collection, which that tool deliberately never produces.
    """
    import wrist as wr
    from wrist import _types as T

    session = wr.Session(
        "lifetime",
        digest_ring=wr.WR_DIGEST_RING_RECOMMENDED,
        policy={"stream_start_timeout_us": 15_000_000},
    )
    pending = 0
    with wr.Replay(fixture) as replay:
        for chunk in replay:
            if chunk.direction == T.WireDirection.META:
                text = bytes(chunk.data[: chunk.length])
                if text.startswith(b"link_up"):
                    session.on_link_up(247, chunk.host_time_us)
                elif text.startswith(b"stream_start"):
                    session.start_stream()
            elif chunk.direction == T.WireDirection.HOST_TO_DEVICE:
                session.tick(chunk.host_time_us)
                if chunk.length == 5 and chunk.data[0] == 0xA1 and not pending:
                    snapshot = session.clock_or_none()
                    if snapshot and snapshot.flags & T.ClockFlag.HAS_FIT:
                        last = min(
                            (chunk.data[3] << 8) | chunk.data[4], snapshot.last_index
                        )
                        request = T.wr_history_request()
                        request.window.start_us = (
                            wr.clock_to_host_us(snapshot, last - 400) - 1
                        )
                        request.window.end_us = wr.clock_to_host_us(snapshot, last) + 1
                        request.deadline_us = chunk.host_time_us + 60_000_000
                        request.max_attempts = 1
                        try:
                            pending = session.history_reserve(request)
                        except wr.WristError:
                            pass
            else:
                session.on_bytes(bytes(chunk.data[: chunk.length]), chunk.host_time_us)

            while session.poll_writes(8):
                pass
            while session.poll_events(16):
                pass
            while len(session.poll_live(64)):
                pass

            if pending:
                block = session.history_collect(pending)
                if block is not None:
                    return session, block
    return session, None


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    os.environ["WRIST_LIBRARY"] = sys.argv[1]
    fixture = Path(sys.argv[2]) if len(sys.argv) > 2 else (
        Path(__file__).resolve().parent / "fixtures" / "swings.wrwire"
    )
    if not fixture.is_file():
        print(f"⚠ NOTHING WAS CHECKED: {fixture} is missing", file=sys.stderr)
        return 3

    session, block = first_block(fixture)
    if block is None:
        # ⚠ No evidence is not a pass.  This test is worthless without a real
        # block, so it says so and fails rather than reporting green over
        # nothing.
        print("⚠ NOTHING WAS CHECKED: the capture yielded no history block",
              file=sys.stderr)
        return 3

    samples = block.samples
    check(len(samples) > 0, f"the block carries samples ({len(samples)})")

    # ⚠ The numpy view is advertised as ZERO-COPY over the same memory, so it is
    # checked against the ctypes read rather than merely being called.  A view
    # built on a wrong stride would still produce an array of the right length.
    try:
        import numpy  # noqa: F401

        view = samples.numpy()
        agrees = (
            len(view) == len(samples)
            and int(view[0]["sample_index"]) == samples[0].sample_index
            and int(view[-1]["sample_index"]) == samples[len(samples) - 1].sample_index
        )
        check(agrees, "the numpy view agrees with the ctypes read, end to end")
        kept_array = view.copy()
    except ImportError:
        # ⚠ Reported, not skipped silently — "not installed" and "checked" are
        # different answers.
        print("[note] numpy is not installed; its view was NOT checked")
        kept_array = None

    # The copy is taken BEFORE the release, which is the documented way to keep
    # data past a block, and it must survive.
    kept = samples.copy()
    first_index = samples[0].sample_index

    block.release()

    raises(lambda: samples[0], "a sample view raises once its block is released")
    raises(lambda: len(samples), "len() on a released view raises")
    raises(lambda: list(samples), "iterating a released view raises")
    raises(lambda: samples.numpy(), "numpy() on a released view raises")
    raises(lambda: samples.step_density(), "step_density() on a released view raises")
    raises(lambda: block.samples, "the block itself raises once released")
    raises(lambda: block.density, "a released block's scalars raise")

    check(
        kept[0].sample_index == first_index and len(kept) > 0,
        "a copy taken before the release outlives it",
    )
    if kept_array is not None:
        check(
            int(kept_array[0]["sample_index"]) == first_index,
            "a numpy copy taken before the release outlives it",
        )

    # ⚠ Releasing twice through the wrapper must be harmless.  The C call is not
    # idempotent — a second wr_history_block_release() on the same pointer is a
    # use-after-free, which ASan does report — so the wrapper is what makes
    # `with` plus an explicit `.release()` safe to write.
    block.release()
    check(True, "releasing a block twice through the wrapper is harmless")

    session.destroy()
    session.destroy()
    check(True, "destroying a session twice is harmless")

    print(f"\n{checks - failures}/{checks} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
