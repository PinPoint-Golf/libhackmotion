<!-- SPDX-License-Identifier: MIT -->
<!-- Copyright (C) 2026 Mark Liversedge -->
# wrist — Python bindings

ctypes bindings for [libwrist](../), the sans-I/O C library for the
HackMotion wrist sensor.

```sh
cmake -S .. -B ../build/dev -DCMAKE_BUILD_TYPE=Debug
cmake --build ../build/dev -j4
PYTHONPATH=. python3 -c "import wrist; print(wrist.VERSION)"
```

The binding loads `libwrist_ffi` — one shared object holding the core *and*
the `.wrwire` container. It looks in `WRIST_LIBRARY`, then the repo's
`build/` directories, then beside the package, then the platform loader.

---

## What this is and is not

**The library owns no radio, no thread, no timer and no clock, and the binding
adds none of them.** You bring the radio (`bleak` works well and is what this
repo's own tooling uses), you bring the clock, and you run the loop:

```python
import time, wrist as wr

def now_us():
    return time.monotonic_ns() // 1000      # ⚠ MONOTONIC, never a wall clock

with wr.Session(device_id="bench") as session:
    session.on_link_up(negotiated_mtu, now_us())
    session.start_stream()

    # from the notification callback, one call per notification:
    session.on_bytes(payload, now_us())

    # every pass round the loop:
    session.tick(now_us())
    for write in session.poll_writes():
        characteristic.write(write.data)
    for event in session.poll_events():
        print(event.text)
    samples = session.poll_live()
```

`session.next_due_us` is the absolute time of the next thing the session wants
to do — arm one timer on it and re-read it after every call.

---

## Or take the transport that already does that

```python
import wrist as wr
from wrist.bleak_transport import BleakTransport

device = await BleakTransport.discover(on_armed=lambda: print("press the button"))

with wr.Session("bench", wire_ring=wr.WR_WIRE_RING_RECOMMENDED) as session:
    async with BleakTransport(session, device,
                              on_event=handle_events,
                              on_live=handle_samples) as link:
        session.start_stream()
        await link.flush()          # the command is on the wire when this returns
        await asyncio.sleep(60)
```

⚠ **It is optional and it is not imported by `import wrist`.** `bleak` is
not a dependency of this binding. api-request §2.0 is why, and it is a blocker
rather than a preference: a consumer that already owns a cross-platform BLE stack
cannot embed a library that brings a second one contending for the same adapter.
`wrist.device` is the other half of that split — the advertised-name match
and the four GATT UUIDs, so you can run **your own** scanner without holding a
second copy of them.

`tools/wr_bench.py` is a whole session written against this, and is meant to be
copied. It has been driven against a real sensor.

**Three things about it that are not incidental.**

⚠ **The notification callback is synchronous and does nothing but stamp.** bleak
schedules an async callback as a task, and each task runs to its first `await`
and then yields — so two notifications that await a write can reach `on_bytes()`
in the wrong order. The protocol has nothing to resynchronise on, so that
corruption is silent. If you write your own transport, do not make that callback
a coroutine.

⚠ **The transport keeps no write queue of its own.** `poll_writes()` returns
nothing at all while a retrieval is in flight, on purpose. A transport that
"helpfully" kept its own queue moving during one would put an unrelated reply
into a stream that cannot be resynchronised.

⚠ **A disconnect is classified, not passed through as UNKNOWN.** The cause drives
the recovery advice a user acts on — reconnect, close the other application,
press the button, stop retrying — and `transport.link_down` carries the evidence
the classification was made from, because a bare cause is an assertion and a
cause beside a measurement is not.

---

## Five things that will bite you

**⚠ One thread, for the whole life of a session.** There are no locks, no atomics
and no threads in the library. Under asyncio that means the event loop thread.
`asyncio.to_thread` is for blocking on the user, never for calling in here.

**⚠ Do not block that thread.** Arrival stamps are the entire foundation of the
clock fit. A bare `input()` on the loop once fabricated 4.08 s of arrival times
in this repo's own capture harness, and that recording still needs a relaxed
stream-start bound to replay at all.

**⚠ A history block owns its samples.** They are library memory and they die when
the block is released. Use it as a context manager, and copy anything you want
to keep:

```python
with session.history_collect(request_id) as block:
    if block is not None:
        arr = block.samples.numpy().copy()      # ⚠ copy to outlive the block
```

Reading a view after its block is released raises rather than returning garbage —
but a *numpy* array you took a view of and did not copy is past where the guard
can see, so copy it.

**⚠ One call, one notification.** `on_bytes` takes a complete ATT notification
payload exactly as the transport delivered it. It is not a byte stream and the
library does not reassemble one: the protocol has no length field, no sequence
number and no checksum, so a coalescing transport corrupts silently.

**⚠ `poll_writes()` is the only source of bytes.** The library composed them
against a short allowlist. ⛔ Command `f0` reboots the sensor into
firmware-update mode through the *ordinary* data characteristic, so there is
deliberately no way to send an arbitrary command through this binding. Do not
add one, and never sweep or fuzz the command space.

---

## Reading the numbers a retrieval gives you

A history block reports **three numbers that answer three different questions**,
and no one of them can stand in for another:

| | |
|---|---|
| `coverage_fraction` | how much of what you **asked for** arrived |
| `density` | how closely spaced what arrived actually **was** |
| `achieved_hz` | the **average** rate across the span it reached |

`density` is a *spacing*: 1 / the median gap between consecutive delivered
indices. **1.0 is index step 1** — the full ≈799 Hz — and **0.125 is step 8**,
which is what a wrist held still returns. The device's buffer is
motion-adaptive, so a holed reply is the ordinary case and not a fault.

⚠ `live_overlap_samples == 0` means **no evidence**, not agreement — either the
pull covered a span the live stream never reached, or you did not ask for a
digest ring (`Session(digest_ring=wr.WR_DIGEST_RING_RECOMMENDED)`).

⚠ Gate alignment on `precision_us`, record `uncertainty_us`. Gating on the total
refuses every pull after the first second of a session.

---

## Layout and enum values are pinned, not trusted

`_types.py` declares every public struct and enum a second time. `wrwire abi`
reads the real ones out of the compiler and `tests/test_python_abi.py` compares
them — every size, every field offset, every enumerator, in both directions.

That check found two genuine transcription errors on the day it was written:
`wr_gap_kind`'s first two members were the wrong way round, and four of eleven
history statuses were on the wrong numbers. Both read as perfectly ordinary code.

`tests/test_python_replay.py` goes further and drives the same retrieval twice
over the same real capture — once from C, once through the binding — and requires
the two block tables to be identical.

---

## Offline: replaying a capture

```sh
../tools/wr_replay_py.py ../tests/fixtures/swings.wrwire
```

Reads a `.wrwire` recording, drives it through a real session, and prints what
the gather made of it. No radio involved.
