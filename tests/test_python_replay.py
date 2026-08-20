#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""test_python_replay.py — the binding against the C tool, over real device bytes.

⚠ THIS IS THE ONLY CHECK THE BINDING GETS AGAINST DATA.  test_python_abi.py
compares declarations — sizes, offsets, enumerator values — and a layout can
agree with the compiler while the wrapper around it still hands back the wrong
thing: a block released too early, a sample array walked with the wrong stride, a
pointer read as an int.  So the same gather is driven twice over the same
capture, once from C and once from Python, and the two per-block tables must be
EQUAL.

⚠ EQUALITY IS THE ASSERTION, NOT A TABLE OF NUMBERS SOMEBODY TYPED IN.  Numbers
in a test rot into numbers somebody updates when they go red.  The C tool does
not use the binding at all, so a disagreement is always the binding's fault and
is never resolved by editing this file.

⚠ AND EQUALITY ALONE IS NOT ENOUGH, WHICH IS THE SECOND HALF OF THIS FILE.  Two
runs that both reached no swing would agree perfectly and check nothing.  So the
density signature is asserted absolutely: the swing pulls must read 1.000 (index
step 1, the full ≈799 Hz) and the still ones 0.125 (step 8, ≈100 Hz).  A number
that never varies is a check that has stopped running, and this project has been
caught by exactly that — `density` was pinned at 1.000 for every reply that ever
arrived, and was published as a positive finding across seven real pulls.

⚠ BOTH FIXTURES, EVERY TIME.  swings.hmwire is the 657-728 Hz regime and
session1.hmwire is the 100.5 Hz one past two index wraps.  A check that only ever
sees one of them has seen half the device.

Usage: test_python_replay.py <hm_gather_replay> <libhackmotion_ffi.so>
"""

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "fixtures" if (ROOT / "fixtures").is_dir() else ROOT / "tests" / "fixtures"
REPLAY_PY = ROOT / "tools" / "hm_replay_py.py"

failures = 0
checks = 0


def check(condition, what):
    global failures, checks
    checks += 1
    if condition:
        print(f"[ ok ] {what}")
    else:
        print(f"[FAIL] {what}")
        failures += 1


def run(argv, json_path, env):
    result = subprocess.run(
        argv + ["--json", str(json_path)], capture_output=True, text=True, env=env
    )
    if not json_path.exists():
        return result.returncode, None, result
    return result.returncode, json.loads(json_path.read_text()), result


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    gather_replay, library = sys.argv[1], sys.argv[2]

    env = dict(os.environ)
    # ⚠ Pin the binding to the object THIS build produced.  A search that found
    # build/dev while testing build/san would report on the wrong artefact.
    env["HACKMOTION_LIBRARY"] = library

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for name in ("swings", "session1", "smoke"):
            fixture = FIXTURES / f"{name}.hmwire"
            if not fixture.is_file():
                check(False, f"{name}.hmwire is present")
                continue

            c_rc, c_data, c_run = run(
                [gather_replay, str(fixture)], tmp / f"c_{name}.json", env
            )
            py_rc, py_data, py_run = run(
                [sys.executable, str(REPLAY_PY), str(fixture)],
                tmp / f"py_{name}.json",
                env,
            )

            if py_data is None:
                print(py_run.stdout[-2000:])
                print(py_run.stderr[-2000:], file=sys.stderr)
                check(False, f"{name}: the Python harness produced a table")
                continue
            if c_data is None:
                check(False, f"{name}: the C tool produced a table")
                continue

            # ⚠ The exit code is part of the answer, and the important half is
            # the empty case: smoke.hmwire has no retrieval in it, and BOTH tools
            # must say "nothing was checked" (3) rather than passing quietly.
            check(c_rc == py_rc, f"{name}: same exit code ({c_rc})")

            check(
                c_data == py_data,
                f"{name}: the block tables are identical"
                + ("" if c_data == py_data else f"\n  C : {c_data}\n  py: {py_data}"),
            )
            _absolute_checks(name, py_data)

    print(f"\n{checks - failures}/{checks} checks passed")
    if failures:
        print(
            "\n⚠ The binding and the C tool disagree over a real device's bytes.\n"
            "  The C tool does not use the binding, so the binding is what is wrong."
        )
        return 1
    return 0


def _absolute_checks(name, data):
    """⚠ The density signature, asserted by value rather than by agreement.

    Equality above would be satisfied by two runs that both reached nothing.
    These say which regime was actually exercised, in §7.3's own unit.
    """
    densities = [b["density"] for b in data["blocks"]]

    if name == "swings":
        check(len(densities) == 6, f"swings: six blocks ({len(densities)})")
        # Five pulls over swings, one control over a still wrist.
        check(
            densities.count(1.0) == 5,
            f"swings: five pulls at density 1.000 — index step 1, the full rate "
            f"({densities.count(1.0)})",
        )
        check(
            densities.count(0.125) == 1,
            f"swings: one pull at density 0.125 — index step 8, a still wrist "
            f"({densities.count(0.125)})",
        )
        # ⚠ The two regimes must SEPARATE.  If these ever converge, something has
        # been undone — that is the whole point of carrying two fixtures.
        check(
            max(densities) / min(densities) == 8.0,
            "swings: the two regimes separate by exactly the step-8 floor",
        )
        check(
            data["steps"]["0"] > 1000 and data["steps"]["4"] > 0,
            f"swings: both step buckets populated (step 1: {data['steps']['0']}, "
            f"step 8: {data['steps']['4']})",
        )
        check(data["steps"]["5"] == 0, "swings: no delivered step above the floor of 8")

    elif name == "session1":
        check(len(densities) == 1, f"session1: one block ({len(densities)})")
        check(
            densities == [0.125],
            f"session1: density 0.125 — the ≈100 Hz floor a still wrist returns "
            f"({densities})",
        )
        check(data["steps"]["5"] == 0, "session1: no delivered step above the floor of 8")

    elif name == "smoke":
        # ⚠ NO EVIDENCE AND AGREEMENT ARE DIFFERENT ANSWERS.  A capture with no
        # retrieval must produce no blocks and say so, in both tools.
        check(
            data["blocks_total"] == 0 and not densities,
            "smoke: no retrieval, so no block and no density — reported, not passed",
        )


if __name__ == "__main__":
    sys.exit(main())
