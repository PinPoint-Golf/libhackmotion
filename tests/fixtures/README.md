<!-- SPDX-License-Identifier: LGPL-2.1-or-later -->
<!-- Copyright (C) 2026 Mark Liversedge -->
# Replay fixtures — real bytes off a real wG3

Three recordings from one wG3 unit, 15 Aug, all under config `0x7e`, BlueZ, ATT
MTU 247, connection interval ~23 ms. They are the **only** real-hardware
recordings this project has, and design §10.5 wants exactly this: a capture that
outlives the theory it was taken to test.

| file | size | duration | what it carries | what it is the control for |
|---|---|---|---|---|
| `swings.hmwire` | 204 K | 46 s | five golf swings, **six mid-stream retrievals** | the whole of history retrieval — §7.3's step-1 runs and step-8 floor, §7.5's stall, §8.8's live-vs-history agreement |
| `session1.hmwire` | 368 K | 174 s | a calibration choreography, one retrieval **past two index wraps** | §8.2's `0x94` payload; §7.3's **100 Hz floor** over a still wrist; and §7.4's re-wrap above 131,072 |
| `smoke.hmwire` | 95 K | 32 s | live only, hand motion, **no retrieval at all** | §7.5's pull-free tick ratio (80.138), and the "nothing was checked" path |

Each is a control for a claim the others cannot settle, which is why all three
are here rather than just the one the tests read.

---

## ⚠ Before adding another one

**Check it is redacted, by looking at the bytes rather than at the header
flag.** The MAC (`0x85`) and serial (`0x86`) replies name a specific unit and a
specific owner (api-request §2.13). `tools/hm_capture.py` redacts them by
default and the header records that it did — but the header says what the
recorder *intended*, so confirm what it actually wrote:

```sh
./build/dev/hmwire info FILE.hmwire | grep identifiers    # says "redacted"
./build/dev/hmwire dump FILE.hmwire --hex | grep -A1 'REDACTED'
strings -n 6 FILE.hmwire | grep -E '([0-9A-F]{2}:){5}[0-9A-F]{2}|^WG3[0-9]'
```

The id byte survives so a reader can see *what* was removed; the payload must be
zeros, and the last command must print nothing. All three files here were
checked that way before they were tracked.

`.gitignore` still excludes `*.hmwire` everywhere else and un-excludes only this
directory, so an ad-hoc capture taken at the repo root cannot be committed by
accident. **Keep it that way**: the safe default is the one that catches the
tired mistake.

---

## What reads them

```sh
ctest --test-dir build/dev -R 'capture|gather_replay'
```

- `hmwire verify` — the container: framing, monotonic host time, no unmarked
  sequence gap. Run against all three, so a corrupted fixture is noticed on the
  commit that corrupts it rather than a phase later.
- `hm_gather_replay` — ⚠ the **implementation**, driven by the device's own
  bytes. §7.3's buffer is motion-adaptive, so a still wrist replays at ~100 Hz
  whether the gather is right or not, and no synthetic test can tell the two
  apart. This is the only validation retrieval gets. It runs over **two** of
  these captures, deliberately: `swings.hmwire` is the 657–728 Hz regime and
  `session1.hmwire` is the 100.5 Hz one, and a check that only ever sees one of
  them has seen half the device. ⚠ `session1.hmwire`'s first frame arrived
  4.08 s after `a0` — its harness blocked its own event loop — so the tool
  relaxes the stream-start bound for the replay and says why at the call site.
  The bound itself is tested in `tests/test_session.c`, where it belongs.
- `hmwire reconcile` — the **specification**, claim by claim. Deliberately *not*
  a `ctest`: three of its claims legitimately **DIFFER** on this hardware (that
  is the finding, and it is recorded), so a green/red gate
  over it would either lie or have to be updated every time the device teaches
  us something.

⚠ These files are **byte-exact recordings, not test data**. Do not regenerate,
trim or "clean" them. §5.6 is the reason they exist in this form: the `0x94`
payload was captured before anyone knew what it contained and decoded a phase
later, and §12 still has open items whose answers are already sitting in these
bytes.
