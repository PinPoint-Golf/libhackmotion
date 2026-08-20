#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""
test_capture_format.py — the capture harness writes what the library reads.

⚠ WHY THIS TEST EXISTS.  tools/wr_capture.py writes the `.wrwire` container in
Python and record/wr_record.c reads it in C.  That is two implementations of
one format, which is exactly the shape of bug this project has had escape
review three times: a design changed on one side and left stale on the other,
producing a plausible answer and no alarm.

So the two are pinned against each other here, and the same check runs at the
end of every real capture.  A drift fails the build rather than producing a
recording that only turns out to be unreadable later — by which time the sensor,
the session and the person wearing it are gone.

It also pins the safety property that matters: the script takes its command
allowlist from the built library and there is no second copy of it.

Usage: test_capture_format.py <path to the built wrwire>
"""

import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import wr_capture  # noqa: E402

failures = 0


def check(condition, what):
    global failures
    if condition:
        print(f"[ ok ] {what}")
    else:
        failures += 1
        print(f"[FAIL] {what}")


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: test_capture_format.py <wrwire>")
    wrwire = sys.argv[1]

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "capture.wrwire"

        rec = wr_capture.Recorder(path, "python-writer", record_identifiers=False)
        rec.meta("link_up mtu=247")
        # §9.1's bring-up, then a start command, then one 47-byte 0x90 frame.
        for cmd in wr_capture.BRINGUP:
            rec.write(wr_capture.DIR_HOST_TO_DEVICE, cmd)
        rec.write(wr_capture.DIR_HOST_TO_DEVICE, bytes([0xA0, 0x01, 0x7E]))
        frame = bytes([0x90]) + bytes(46)
        rec.write(wr_capture.DIR_DEVICE_TO_HOST, frame)
        # A redacted serial reply keeps its LENGTH and says so with a flag.
        rec.write(
            wr_capture.DIR_DEVICE_TO_HOST,
            bytes([0x86]) + bytes(9),
            wr_capture.FLAG_REDACTED,
        )
        expected_chunks = rec.chunks
        rec.close()

        verify = subprocess.run([wrwire, "verify", str(path)], capture_output=True, text=True)
        check(verify.returncode == 0, "the C reader verifies a Python-written recording")
        if verify.returncode != 0:
            print(verify.stdout + verify.stderr)

        info = subprocess.run([wrwire, "info", str(path)], capture_output=True, text=True)
        check(info.returncode == 0, "wrwire info reads the header")
        check("python-writer" in info.stdout, "the device id round-trips")
        check("config         0x7e" in info.stdout, "the configuration byte round-trips")
        check("monotonic_us" in info.stdout, "⚠ the recording says which host clock it used")
        check("redacted" in info.stdout, "identifiers are redacted by default")
        check(f"chunks         {expected_chunks}" in info.stdout,
              f"all {expected_chunks} chunks are readable")

        dump = subprocess.run([wrwire, "dump", str(path)], capture_output=True, text=True)
        check("[REDACTED]" in dump.stdout,
              "⚠ redaction is MARKED, so a reader knows something was removed not absent")
        check("90 frame" in dump.stdout, "the frame's message id decodes")

        # ⛔ The allowlist the script will enforce comes from the library, and
        # nothing else.  This is the check that keeps it that way.
        allowed = wr_capture.load_allowlist(wrwire)
        check(0xF0 not in allowed, "⛔ 0xf0 is not on the allowlist")
        check(allowed == {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0xA0, 0xA1, 0xA2, 0xFA},
              "the allowlist is exactly spec §4")
        for cmd in wr_capture.BRINGUP + [bytes([0xA0]), bytes([0x83]), bytes([0xA1]),
                                         bytes([0xA2]), bytes([0xFA])]:
            if cmd[0] not in allowed:
                check(False, f"the script would send 0x{cmd[0]:02x}, which is not allowed")
                break
        else:
            check(True, "every byte the script can send is on the allowlist")

    print(f"test_capture_format: {failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
