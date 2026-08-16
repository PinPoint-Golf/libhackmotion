#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
"""
hm_capture.py — record real HackMotion wG3 traffic into a `.hmwire` file.

    ./tools/hm_capture.py --out session.hmwire --duration 120
    ./build/dev/hmwire reconcile session.hmwire

⚠ WHY A SCRIPT AND NOT C.  The reference BlueZ transport is a later phase and
belongs in the library's own shape; this is a harness whose only job is to get
real bytes onto disk so `hmwire reconcile` can check the specification against
a sensor (design §11).  Nothing here is part of libhackmotion, nothing
links against it at runtime, and the artefact it produces is the deliverable.

=============================================================================
⛔ SAFETY — the one thing this file must get right
=============================================================================
Command `f0` reboots the sensor into FIRMWARE-UPDATE MODE, and it reaches that
mode through the ORDINARY data characteristic — the same pipe every other
command uses.  Avoiding the OTA service is explicitly not sufficient
(spec §2.3, §4.1).

So this script does not hold its own idea of what may be sent.  It asks the
built library:

    hmwire allowlist

and refuses to run at all if it cannot.  A second copy of a safety property is
a second thing that can drift, and this project has had documentation drift
past review three times.

**Never sweep or fuzz the command space.**  The known set came from reading the
vendor's binary, which cannot prove the firmware accepts nothing else.
Fuzzing the DECODER is a different activity and is welcome — point a fuzzer at
hm_codec_decode() instead.
=============================================================================

Requires `bleak`.  Linux/BlueZ, macOS and Windows all work; the MTU check below
is the one place they differ in what they will tell you.
"""

from __future__ import annotations

import argparse
import asyncio
import collections
import shutil
import struct
import subprocess
import sys
import time
import warnings
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner

    _BLEAK_ERROR = None
except ImportError as _exc:  # the module stays importable without a radio stack
    BleakClient = BleakScanner = None  # type: ignore[assignment]
    _BLEAK_ERROR = _exc

# --- spec §2.1, §2.3 --------------------------------------------------------
DEVICE_NAME = "HackMotion wG3"
DATA_CHAR = "413c3893-b7e8-4231-9673-7af7aed06ddc"

# spec §2.4: the calibration result is 65 bytes and stream notifications reach
# 93, both beyond the 20-byte payload of the default ATT MTU.
MIN_ATT_MTU = 96

# spec §9.2, measured: a silent connection drops at exactly 5.0 minutes, and AN
# ACTIVE STREAM DOES NOT PREVENT IT.  What resets the timer is a host→device
# write.  The vendor app polls `0x81` every 30 s and that poll is the keepalive;
# the battery reading is incidental.
KEEPALIVE_PERIOD_S = 30.0

# spec §9.1, verbatim.  Only step 2 is required — the protocol version from `80`
# gates features (§6.2, §7.1) — but reproducing the rest is a useful bring-up
# test (design §10.5).
BRINGUP = [b"\x80", b"\x81", b"\x84", b"\x81", b"\x86", b"\x86", b"\x86", b"\x85"]

# spec §6.2.  ⚠ Two of these bits change the WIRE FORMAT, not just the content,
# and 0x7e is the only configuration under which the relative-angle
# cancellation was ever measured.  This script does not offer a way to change it.
CONFIG_BITS = 0x7E

# --- the container, design §5.6 and include/hackmotion/record.h -------------
MAGIC = b"HMWIRE1\n"
ENTRY = struct.Struct("<IBBHIq")  # length, direction, flags, reserved, seq, time
CHUNK_MAX = 256

DIR_HOST_TO_DEVICE = 0
DIR_DEVICE_TO_HOST = 1
DIR_META = 2

FLAG_REDACTED = 1 << 0
FLAG_LOST = 1 << 1

# spec §7.3 puts the buffer at ~7.5 s.  Keep a little less, so a window chosen
# from it is still resident when the request goes out.
RECENT_WINDOW_US = 6_000_000

# ⚠ EVENT-TRIGGERED RETRIEVAL, because the buffer is ~7.5 s deep (§7.3) and a
# swing is unrecoverable once it has fallen out.  Deferring the pull to the end
# of a capture can only ever retrieve the last few seconds — so a session with
# swings in the middle of it returns the 100 Hz floor and proves nothing, which
# is exactly the trap §7.6 warns about.
#
# A struck golf swing peaks at 2,179-2,376 °/s and a deliberate wrist flick at
# 3,399 °/s (§6.4), against a few hundred for ordinary handling.  600 catches a
# swing without firing on someone picking the sensor up.
SWING_TRIGGER_DPS = 600.0
SWING_QUIET_DPS = 150.0

# Wait this long after the motion settles before asking, so the post-roll is in
# the buffer.  Still an order of magnitude inside the ~7.5 s depth.
SWING_POST_ROLL_US = 700_000

# §7.3's own table, for the bench summary.  Step 8 is a hard floor and step 1 a
# hard ceiling: across 17,739 measured steps none exceeded 8 and none was 0.
STEP_TABLE = [
    (1, "799 Hz", "780-850 °/s"),
    (2, "400 Hz", "~370 °/s"),
    (5, "160-270 Hz", "150-210 °/s"),
    (8, "99.9 Hz", "0.4-28 °/s"),
]


def record_gyro_dps(rec: bytes) -> float:
    """Angular rate of the faster unit in one 46-byte record, °/s.

    ⚠ The /8 divisor is config bit 6 (extended gyro range); this script only
    ever sends 0x7e, where it is set.  Under a configuration with it clear the
    scale halves and this would be wrong by 2× (§6.4).
    """
    best = 0.0
    for base in (2 + 14, 2 + 22 + 14):  # gyro triplet in each block
        x, y, z = struct.unpack_from(">hhh", rec, base)
        mag = (x * x + y * y + z * z) ** 0.5 / 8.0
        best = max(best, mag)
    return best


async def prompt(text: str) -> None:
    """⚠ input() ON A THREAD, NEVER ON THE EVENT LOOP.

    A bare input() blocks the asyncio loop, so bleak dispatches no
    notifications until the user presses Enter — and every frame that arrived
    meanwhile is then delivered in a burst and stamped with the time the loop
    resumed, not the time it arrived.

    Measured on the first calibrated capture: the `a0 01` start acknowledgement
    is timestamped 4.08 s after the start command, at the exact instant of the
    first Enter, and ~13 s of frames carry fabricated arrival times.  Host
    timestamps are the entire foundation of the clock fit (§10), so this is not
    cosmetic.

    The lower-envelope fit absorbed it — the rate moved 85 ppm when the affected
    window was excluded, because fabricated delay is one-sided like real BLE
    delay and the estimator is built for exactly that (hm_fit.h).  It was robust
    to the bug, which is not a reason to keep it.
    """
    await asyncio.to_thread(input, text)


async def negotiated_mtu(client) -> int:
    """The ATT MTU, or 0 meaning UNKNOWN.  ⚠ Never a guess, and never a number
    the platform did not actually give us.

    bleak's BlueZ backend returns the ATT default of 23 until something asks
    BlueZ for the real value — its own docstring says "The BlueZ backend will
    always return 23 (the minimum MTU size)".  Reading that 23 as a measurement
    is the same mistake as reading a zero mismatch count out of zero samples as
    agreement, run in the other direction: an ABSENT measurement dressed up as a
    bad one.  It refused a perfectly good link on this bench.

    session.h already has the right word for this case — "pass 0 if the platform
    will not tell you, which is treated as 'unknown, proceed' with a warning
    rather than as a failure" — so 0 is what an unanswered question returns.

    ⚠ And an unverified MTU is not an unverifiable one.  The frames settle it:
    a 47- or 93-byte notification cannot arrive over a 23-byte MTU, so the
    capture measures from the data what the platform would not say (§2.4).
    """
    # _acquire_mtu() is private, and it is what bleak's own mtu_size example
    # uses; there is no public route on this backend.
    backend = getattr(client, "_backend", client)
    acquire = getattr(backend, "_acquire_mtu", None)
    if acquire is not None:
        try:
            await acquire()
        except Exception as exc:  # noqa: BLE001 - any failure means "unknown"
            print(f"  MTU: BlueZ would not say ({type(exc).__name__}: {exc})")

    mtu = 0
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


def monotonic_us() -> int:
    """⚠ Monotonic, never a wall clock.

    An NTP step or a DST change mid-session corrupts a capture in a way that
    looks like a sensor fault (include/hackmotion/types.h).  The epoch is
    arbitrary and never interpreted; only differences matter.
    """
    return time.monotonic_ns() // 1000


class Recorder:
    """Writes the `.hmwire` container.  The C reader is the authority on the
    format; `hmwire verify` is run against the result before this script exits,
    so a drift between the two cannot produce a quietly bad artefact."""

    def __init__(self, path: Path, device_id: str, record_identifiers: bool):
        self.path = path
        self.fp = path.open("wb")
        self.seq = 0
        self.chunks = 0
        clean = "".join(c if 0x20 <= ord(c) < 0x7F else "_" for c in device_id)
        self.fp.write(MAGIC)
        self.fp.write(f"device_id={clean}\n".encode())
        self.fp.write(f"config=0x{CONFIG_BITS:02x}\n".encode())
        self.fp.write(b"layout_version=1\n")
        self.fp.write(b"clock=monotonic_us\n")
        self.fp.write(b"byte_order=little\n")
        self.fp.write(
            b"identifiers=recorded\n" if record_identifiers else b"identifiers=redacted\n"
        )
        self.fp.write(b"\n")

    def write(self, direction: int, data: bytes, flags: int = 0, when: int | None = None):
        if len(data) > CHUNK_MAX:
            # ⚠ The container refuses rather than clamping, and so does this.
            # A notification longer than the maximum message means the transport
            # coalesced, which §3 says cannot be resynchronised.
            raise ValueError(f"chunk of {len(data)} B exceeds {CHUNK_MAX}")
        t = monotonic_us() if when is None else when
        self.fp.write(ENTRY.pack(len(data), direction, flags, 0, self.seq, t))
        self.fp.write(data)
        self.seq += 1
        self.chunks += 1

    def meta(self, text: str):
        self.write(DIR_META, text.encode()[:CHUNK_MAX])

    def close(self):
        self.fp.flush()
        self.fp.close()


def load_allowlist(hmwire: str) -> set[int]:
    """⛔ The set of command bytes that may be written, from the built library.

    There is deliberately no fallback.  If the tool cannot be reached, the safe
    answer is to send nothing at all.
    """
    out = subprocess.run([hmwire, "allowlist"], capture_output=True, text=True, check=True)
    allowed = {int(line, 16) for line in out.stdout.split() if line}
    if not allowed:
        raise RuntimeError("empty allowlist")
    if 0xF0 in allowed:
        raise RuntimeError("⛔ 0xf0 is on the allowlist — refusing to send anything")
    return allowed


class Session:
    def __init__(self, args, allowed: set[int], recorder: Recorder):
        self.args = args
        self.allowed = allowed
        self.rec = recorder
        self.client: BleakClient | None = None
        self.last_index: int | None = None
        self.frames = 0
        # ⚠ The MTU measured from the data rather than asked of the platform.
        # An ATT notification is opcode + handle + value, so a payload of N
        # bytes proves the MTU was at least N + 3.
        self.max_notify_len = 0
        self.bracket_open = False
        self.stopping = False
        # Record header indices seen inside the current bracket, so the bench
        # gets its answer immediately rather than only from the offline report.
        self.bracket_indices: list[int] = []
        # ⚠ A rolling window of (host_us, unwrapped_index, |ω|) over roughly the
        # buffer's own depth, so a retrieval can be placed where the MOTION was.
        # The buffer is motion-adaptive (§7.3): it holds the full ≈799.2 Hz only
        # where the wrist moved fast and ~100 Hz where it did not, so a window
        # chosen by recency rather than by motion measures the floor.
        self.recent: collections.deque = collections.deque()
        self.unwrapped: int | None = None
        # Event detection, so a retrieval happens WHILE THE SWING IS STILL IN
        # THE BUFFER rather than whenever the capture happens to end.
        self.event_peak: tuple[int, float] | None = None
        self.event_quiet_us: int | None = None
        self.events_pulled = 0

    # --- writes ------------------------------------------------------------
    async def send(self, payload: bytes, why: str):
        """⛔ THE ONLY PLACE BYTES LEAVE THIS SCRIPT."""
        if not payload:
            raise ValueError("empty write")
        if payload[0] not in self.allowed:
            raise RuntimeError(
                f"⛔ refusing to write {payload.hex()} ({why}): "
                f"0x{payload[0]:02x} is not on the library's allowlist"
            )
        # ⚠ The write quiet period (design §5.6, session.h).  The device
        # replaying its buffer at ~260 notifications/s while an unrelated
        # command arrives is an interaction the specification never exercised,
        # and the failure shape would be a `0x81` reply interleaved into a
        # record stream with no length field, no sequence number and no
        # checksum to resynchronise on (§3).
        if self.bracket_open and payload[0] != 0xA1:
            return False
        self.rec.write(DIR_HOST_TO_DEVICE, payload)
        await self.client.write_gatt_char(DATA_CHAR, payload, response=True)
        return True

    # --- notifications -----------------------------------------------------
    def on_notify(self, _characteristic, data: bytearray):
        """⚠ ONE CALL, ONE NOTIFICATION.  Every stack preserves the ATT PDU
        boundary and the protocol has nothing to resynchronise on if it is
        lost, so the payload is recorded exactly as delivered — never
        concatenated, never re-framed."""
        when = monotonic_us()
        payload = bytes(data)
        flags = 0

        if payload and payload[0] in (0x85, 0x86) and not self.args.record_identifiers:
            # ⚠ The MAC and the serial identify a specific unit and a specific
            # owner (api-request §2.13).  Redaction keeps the LENGTH so the
            # framing stays honest, and is marked so a reader knows something
            # was removed rather than absent.
            payload = payload[:1] + b"\x00" * (len(payload) - 1)
            flags |= FLAG_REDACTED

        try:
            self.rec.write(DIR_DEVICE_TO_HOST, payload, flags, when)
        except ValueError as exc:
            print(f"  ⚠ {exc} — this notification was NOT recorded", file=sys.stderr)
            return

        if len(payload) > self.max_notify_len:
            self.max_notify_len = len(payload)

        if payload[:1] == b"\x90" and len(payload) >= 3:
            self.frames += 1
            self.last_index = (payload[1] << 8) | payload[2]
            body = payload[1:]
            for k in range(len(body) // 46):
                rec = body[k * 46:(k + 1) * 46]
                raw = (rec[0] << 8) | rec[1]
                if self.bracket_open:
                    self.bracket_indices.append(raw)
                    continue
                self.unwrapped = (
                    raw if self.unwrapped is None
                    else self.unwrapped + ((raw - self.unwrapped) & 0xFFFF)
                )
                omega = record_gyro_dps(rec)
                self.recent.append((when, self.unwrapped, omega))
                while self.recent and when - self.recent[0][0] > RECENT_WINDOW_US:
                    self.recent.popleft()

                if omega >= SWING_TRIGGER_DPS:
                    if self.event_peak is None or omega > self.event_peak[1]:
                        self.event_peak = (self.unwrapped, omega)
                    self.event_quiet_us = None
                elif self.event_peak is not None and omega < SWING_QUIET_DPS:
                    if self.event_quiet_us is None:
                        self.event_quiet_us = when
        elif payload[:2] == b"\xa1\x02":
            self.bracket_open = True
        elif payload[:2] == b"\xa1\x01":
            self.bracket_open = False
        elif payload[:1] == b"\xd0":
            print(f"  ⚠ device error {payload.hex()} (spec §7.2)", file=sys.stderr)
        elif payload[:1] == b"\xfb":
            print("  button pressed (§9.4: the count per press is not reliable)")

    # --- phases ------------------------------------------------------------
    async def bringup(self):
        print("  bring-up (§9.1)...", end="", flush=True)
        for cmd in BRINGUP:
            await self.send(cmd, "bring-up")
            await asyncio.sleep(0.15)
        print(" done")

    async def keepalive(self):
        """⚠ MANDATORY.  §9.2 measured a connection streaming continuously at
        25 Hz dropped at exactly 5.0 minutes, the same deadline as a fully
        silent one.  Receiving data does not keep the device alive."""
        while not self.stopping:
            await asyncio.sleep(KEEPALIVE_PERIOD_S)
            if self.stopping:
                return
            if not await self.send(b"\x81", "keepalive"):
                print("  keepalive deferred: a history bracket is open")

    async def calibrate(self):
        """§8.2.  ⚠ The device observes a CONTINUOUS RAISE between the two
        markers, so the stream must already be running — it cannot be done from
        two static samples.  And `0x94` is NOT a verdict: the device emits it
        for every `a2 01` and applies the transform every time."""
        print("\n  CALIBRATION (§8.2)")
        await prompt("    1. Forearm horizontal, wrist straight, then press Enter... ")
        await self.send(b"\xa2\x00", "calibration pose 0")
        print("    2. Now raise the forearm ~30° across the chest, smoothly.")
        await prompt("       Press Enter when the raise is complete... ")
        await self.send(b"\xa2\x01", "calibration pose 1")
        await asyncio.sleep(1.0)  # `0x94` follows within the result timeout
        await prompt("    3. Return to the reference pose and hold, then press Enter... ")
        await asyncio.sleep(1.0)
        print("    calibration markers sent (⚠ 0x94 is not an accept/reject)")

    async def pull(self, label: str, centre: int, width: int) -> None:
        """One `a1` retrieval centred on an unwrapped index."""
        half = width // 2
        lo, hi = centre - half, centre - half + width
        if lo < 0 or (lo >> 16) != (hi >> 16):
            # ⚠ A window spanning the u16 wrap needs two requests, which is the
            # session layer's job (design §8.3), not a harness's.
            print(f"    {label}: skipped, the window crosses the counter wrap")
            return
        first, last = lo & 0xFFFF, hi & 0xFFFF
        self.bracket_indices = []
        await self.send(bytes([0xA1]) + struct.pack(">HH", first, last), "history")
        for _ in range(120):  # §7.4: a pull takes about as long as it spans
            await asyncio.sleep(0.1)
            if not self.bracket_open and self.bracket_indices:
                break

        idx = self.bracket_indices
        if not idx:
            print(f"    {label}: nothing came back"
                  + (" and the bracket is STILL OPEN" if self.bracket_open else ""))
            return
        steps = {}
        for a, b in zip(idx, idx[1:]):
            steps[b - a] = steps.get(b - a, 0) + 1
        modal = max(steps, key=steps.get) if steps else 0
        rate = 799.2 / modal if modal else 0.0
        row = next((r for r in STEP_TABLE if modal <= r[0]), None)
        print(f"    {label}: {len(idx)} records, modal step {modal} "
              f"(~{rate:.0f} Hz), density {100*len(idx)/(width+1):.1f}%"
              + (f"  — §7.3 expects that step at {row[2]}" if row else ""))
        if max(steps, default=0) > 8:
            print("      ⚠ a step above 8 — §7.3 says none was ever seen")

    def event_due(self, now_us: int) -> bool:
        """A fast motion has happened and has settled — pull it NOW.

        ⚠ The buffer is ~7.5 s deep (§7.3).  A swing that is not retrieved
        within that window is gone, at any resolution, forever.
        """
        return (
            self.event_peak is not None
            and self.event_quiet_us is not None
            and now_us - self.event_quiet_us >= SWING_POST_ROLL_US
            and not self.bracket_open
        )

    async def pull_event(self, width: int) -> None:
        peak_index, peak_omega = self.event_peak
        self.event_peak = None
        self.event_quiet_us = None
        self.events_pulled += 1
        print(f"  swing #{self.events_pulled}: peak |ω| {peak_omega:.0f} °/s at index "
              f"{peak_index} — retrieving ±{width // 2} indices now")
        await self.pull(f"over swing #{self.events_pulled}", peak_index, width)

    async def pull_stillness(self, width: int) -> None:
        """The control, at the SAME width.  ⚠ Density is set by the motion in
        the window, not by its width (§7.3), so this is the comparison that
        separates a working full-rate path from a broken one — and it is the
        only one that does."""
        if len(self.recent) < 10:
            print("  stillness control skipped: too few live frames")
            return
        quiet = min(self.recent, key=lambda r: r[2])
        print(f"  stillness control: |ω| {quiet[2]:.1f} °/s at index {quiet[1]}")
        await self.pull("over the stillness", quiet[1], width)


async def find_device(args) -> str:
    """⚠ DISCOVERY IS A RACE (spec §2.1).

    The sensor advertises for only a few seconds after a physical button press
    and then stops.  A scan-then-pick-from-a-list flow assumes the device is
    discoverable whenever you go looking; here it is not.  So the scanner is
    armed BEFORE the user is prompted — a scan that starts after the prompt
    routinely finds nothing, and that reaches the user as "the device doesn't
    work".
    """
    if args.address:
        return args.address

    found: asyncio.Future[str] = asyncio.get_running_loop().create_future()

    def seen(device, adv):
        name = adv.local_name or device.name or ""
        if DEVICE_NAME in name and not found.done():
            found.set_result(device.address)

    kwargs = {"adapter": args.adapter} if args.adapter else {}
    async with BleakScanner(detection_callback=seen, **kwargs):
        print(f"  scanner armed, {args.scan:.0f} s window.")
        print("  ⚠ PRESS THE SENSOR'S BUTTON NOW.")
        print("    If the sensor was asleep the first press only WAKES it and it does")
        print("    not advertise — press, pause, then press again (§2.1).")
        try:
            return await asyncio.wait_for(found, timeout=args.scan)
        except asyncio.TimeoutError:
            raise SystemExit(
                f"hm_capture: no '{DEVICE_NAME}' in {args.scan:.0f} s.\n"
                "  The device advertises only for a few seconds after a button press,\n"
                "  and the vendor app will win the race if it is running (§2.2)."
            )


async def capture(args, allowed: set[int]):
    address = await find_device(args)
    print(f"  found {address}")

    rec = Recorder(Path(args.out), args.device_id or address, args.record_identifiers)
    session = Session(args, allowed, rec)
    keepalive_task = None

    try:
        async with BleakClient(address, **({"adapter": args.adapter} if args.adapter else {})) as client:
            session.client = client
            print("  connected (the device vibrates — §9.5)")
            mtu = await negotiated_mtu(client)
            rec.meta(f"link_up mtu={mtu if mtu else 'unknown'}")
            if mtu == 0:
                print("  ⚠ the platform will not report an ATT MTU — proceeding UNVERIFIED.")
                print("    The frames settle it: a 47- or 93-byte notification cannot")
                print("    arrive over a 23-byte MTU, and the summary below says what")
                print("    actually turned up (§2.4).")
            elif mtu < MIN_ATT_MTU:
                # ⚠ §2.4: the alternative is truncated frames that parse as
                # garbage, on whichever platform eventually gets it wrong.
                raise SystemExit(
                    f"hm_capture: negotiated MTU {mtu} < {MIN_ATT_MTU}; refusing to "
                    "capture, because 93-byte stream notifications would be truncated"
                )
            else:
                print(f"  ATT MTU {mtu} (§2.4 needs ≥ {MIN_ATT_MTU})")

            await client.start_notify(DATA_CHAR, session.on_notify)
            await session.bringup()

            keepalive_task = asyncio.create_task(session.keepalive())

            await session.send(bytes([0xA0, 0x01, CONFIG_BITS]), "start stream")
            rec.meta(f"stream_start config=0x{CONFIG_BITS:02x}")
            print(f"  stream started (a0 01 {CONFIG_BITS:02x})")

            if args.calibrate:
                await session.calibrate()

            print(f"\n  recording for {args.duration:.0f} s — Ctrl-C to stop early")
            print("  ⚠ what you do now decides what the reconciliation can check:")
            print("     · hold still for a while     → the rate fit and §6.6's +32 mode")
            print("     · one pure single-axis move  → §6.7's composition-order check")
            print("     · a fast flick of the wrist  → §6.4's headroom (83% of full scale)")
            print("     · a few swings               → §6.6's dense bursts, §6.3's palm check")
            width = int(args.history) if args.history else 0
            deadline = monotonic_us() + int(args.duration * 1e6)
            try:
                while monotonic_us() < deadline:
                    await asyncio.sleep(0.1)
                    # ⚠ Retrieve a swing WHILE IT IS STILL IN THE BUFFER.  A pull
                    # deferred to the end of the capture can only ever reach the
                    # last ~7.5 s (§7.3), so a swing in the middle of a session
                    # is unrecoverable and the reply returns the 100 Hz floor.
                    if width and session.event_due(monotonic_us()):
                        await session.pull_event(width)
            except asyncio.CancelledError:
                pass

            if width:
                if session.events_pulled == 0:
                    print("  ⚠ no motion above "
                          f"{SWING_TRIGGER_DPS:.0f} °/s was seen, so no swing was retrieved")
                    print("    §7.3: the buffer holds ~100 Hz unless the wrist moved fast, so")
                    print("    a capture without a real swing in it cannot test the full-rate path.")
                await session.pull_stillness(width)

            session.stopping = True
            await session.send(b"\x83", "stop stream")
            rec.meta("stream_stop")
            await asyncio.sleep(0.3)

            if args.power_off:
                # ⚠ §9.3: no acknowledgement arrives and the link stays up ~9 s.
                # The device then needs a PHYSICAL BUTTON PRESS to come back.
                await session.send(b"\xfa", "power off")
                print("  powered off — the link will drop in ~9 s and the device")
                print("  needs a physical button press to return (§9.3)")
                await asyncio.sleep(1.0)

            await client.stop_notify(DATA_CHAR)
            rec.meta("link_down local")
    except KeyboardInterrupt:
        print("\n  interrupted")
    finally:
        session.stopping = True
        if keepalive_task is not None:
            keepalive_task.cancel()
        rec.close()

    print(f"\n  wrote {rec.chunks} chunks to {args.out}")
    print(f"  {session.frames} stream frames, largest notification "
          f"{session.max_notify_len} B")
    if session.max_notify_len:
        # ⚠ The MTU check the platform refused to answer, answered by the data.
        implied = session.max_notify_len + 3
        if implied >= MIN_ATT_MTU:
            print(f"  ATT MTU was at least {implied} — §2.4's floor of "
                  f"{MIN_ATT_MTU} is met, measured not assumed")
        else:
            print(f"  ⚠ nothing larger than {session.max_notify_len} B arrived, so the MTU is "
                  f"only shown to be ≥ {implied}.")
            print("    That is expected if no 93-byte two-record frame occurred; it is a")
            print("    TRUNCATION if hmwire reconcile also reports odd frame lengths (§2.4).")
    if session.frames == 0:
        print("  ⚠ NO STREAM FRAMES AT ALL — the capture has nothing to reconcile.")
    return rec


def main():
    ap = argparse.ArgumentParser(
        description="Record HackMotion wG3 traffic to a .hmwire file.",
        epilog="Then: hmwire verify FILE && hmwire reconcile FILE",
    )
    ap.add_argument("--out", default="capture.hmwire", help="output .hmwire path")
    ap.add_argument("--duration", type=float, default=120.0, help="seconds to stream")
    ap.add_argument("--scan", type=float, default=90.0,
                    help="discovery window; §2.1 says be generous, it costs nothing")
    ap.add_argument("--address", help="skip discovery and connect to this address")
    ap.add_argument("--adapter", help="e.g. hci0 (Linux)")
    ap.add_argument("--device-id", help="opaque label for the recording; ⚠ never a MAC")
    ap.add_argument("--calibrate", action="store_true",
                    help="run §8.2's two-marker routine with prompts")
    ap.add_argument("--history", type=int, metavar="N",
                    help="retrieve an N-index window around EVERY fast motion, as it "
                         "happens (§7.6), plus one over stillness at the end as a control. "
                         "⚠ Triggered rather than deferred because the buffer is only ~7.5 s "
                         "deep: a swing not retrieved inside that window is gone at any "
                         "resolution. 400 is ±0.25 s")
    ap.add_argument("--power-off", action="store_true",
                    help="⚠ send `fa`; the device then needs a physical button press")
    ap.add_argument("--record-identifiers", action="store_true",
                    help="⚠ keep the MAC and serial replies in the recording")
    ap.add_argument("--hmwire", default=None,
                    help="path to the built hmwire tool (default: search build/ then PATH)")
    args = ap.parse_args()

    if _BLEAK_ERROR is not None:
        sys.exit(f"hm_capture: needs `bleak` (pip install bleak): {_BLEAK_ERROR}")

    hmwire = args.hmwire
    if hmwire is None:
        root = Path(__file__).resolve().parent.parent
        for candidate in (root / "build/dev/hmwire", root / "build/rel/hmwire"):
            if candidate.is_file():
                hmwire = str(candidate)
                break
        else:
            hmwire = shutil.which("hmwire")
    if hmwire is None:
        sys.exit(
            "hm_capture: cannot find the `hmwire` tool, so it cannot ask the library\n"
            "  which command bytes are allowed — and it will not guess.  Build it:\n"
            "    cmake -S . -B build/dev && cmake --build build/dev -j"
        )

    try:
        allowed = load_allowlist(hmwire)
    except Exception as exc:  # noqa: BLE001 - any failure here means send nothing
        sys.exit(f"hm_capture: could not read the allowlist from {hmwire}: {exc}")
    print(f"  allowlist from {hmwire}: " + " ".join(f"{b:02x}" for b in sorted(allowed)))

    try:
        asyncio.run(capture(args, allowed))
    except KeyboardInterrupt:
        pass

    # Close the loop on the one thing this script duplicates: the container
    # layout.  If it has drifted from record.h, this says so immediately rather
    # than leaving a file that only fails to parse later.
    print()
    verify = subprocess.run([hmwire, "verify", args.out])
    if verify.returncode != 0:
        sys.exit("hm_capture: ⚠ the recording did not verify — see above")
    print(f"\nNext: {hmwire} reconcile {args.out}")


if __name__ == "__main__":
    main()
