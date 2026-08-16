<!-- SPDX-License-Identifier: LGPL-2.1-or-later -->
<!-- Copyright (C) 2026 Mark Liversedge -->
# libhackmotion — library design

This document is the design of `libhackmotion`, a C library for the HackMotion
wG3 wrist sensor. It describes the library as built, and is the working resource
a developer implements against.

It depends on one document and nothing else:
[`specification.md`](specification.md) — what the protocol **is**. Section
references written bare, such as §6.5 and §10.2, are sections of *that*
document; references written `design §6.1.1` are sections of this one.

---

## Contents

1. [Scope, and the line it does not cross](#1-scope-and-the-line-it-does-not-cross)
3. [Architecture](#3-architecture)
4. [The type system, and why it is shaped this way](#4-the-type-system-and-why-it-is-shaped-this-way)
5. [The session](#5-the-session)
6. [Time](#6-time)
7. [Calibration](#7-calibration)
8. [History retrieval](#8-history-retrieval)
9. [Safety, privacy and logging](#9-safety-privacy-and-logging)
10. [Build, test and toolchain](#10-build-test-and-toolchain)
11. [Implementation status](#11-implementation-status)
12. [Appendix A — API index](#appendix-a--api-index)

⚠ **Section numbers are stable and are not reissued.** There is no §2 or §12;
the numbering of everything else is unchanged, because the headers, the tests and
the source comments cite these sections by number.

---

## 1. Scope, and the line it does not cross

**In scope: everything between the radio and a timestamped pair of quaternions.**
Framing, decode, the session state machine, calibration sequencing, the
device→host clock fit, and full-rate history retrieval with an honest account of
its coverage.

**Out of scope, permanently:**

- **Anatomical decomposition.** Flexion/extension, radial/ulnar deviation and
  rotation belong to the application. §0 makes the same cut from the device's
  side: the device streams per-unit orientation and nothing higher-level, and
  there is no field in a record that could carry a derived quantity. Deviation
  additionally depends on which hand the sensor is worn on, which the device
  never asks and this library has no way to know.
- **Swing detection or phase segmentation.** Address, top and impact are not
  flagged anywhere in the protocol.
- **Firmware update.** See §9.1 — it is not merely unimplemented, it is refused
  by construction.
- **Calibration persistence.** §8.3 settles this three ways over; see §7.4.
- **Any network anything.** The library reaches a local BLE peripheral and stops
  there.

**The one thing on the boundary that is in.** `hm_quat_relative()` and
`hm_relative_angle_deg()` — the relative rotation `q_palm ⊗ q_arm*` and its
angle. These are decode facts rather than analysis choices: the composition
order is a property of the wire format (§6.7), getting it wrong corrupts every
consumer identically, and the angle is what the calibration *presence* check is
built from. Nothing is decomposed, and no axis convention is chosen on a
caller's behalf.

### 1.1 Why C

The consuming application is Qt/C++, but the library is plain **C11** with a flat
C ABI. That is not a restriction to one host; it is what makes the library
reachable from every host. A C ABI with opaque handles and POD structs binds
directly from C++, Python (`ctypes`/`cffi`), Rust, C#, Java, Go and Swift with
no wrapper generation and no exception, template or name-mangling surface to
match. §4.6 covers this.

---

## 3. Architecture

### 3.1 Three layers, and the consumer links two

```
   ┌───────────────────────────────────────────────────────────────┐
   │ 3.  Reference transports — separate targets, nobody must link │
   │     BlueZ/D-Bus (Linux) · file replay · loopback for tests    │
   └───────────────────────────────────────────────────────────────┘
                                 ▲
   ┌───────────────────────────────────────────────────────────────┐
   │ 2.  The transport contract — NOT an interface the library     │
   │     defines and calls.  Four functions the host calls IN, and │
   │     four the host drains OUT.  §3.2                           │
   └───────────────────────────────────────────────────────────────┘
                                 ▲
   ┌───────────────────────────────────────────────────────────────┐
   │ 1.  hackmotion — the core.  No sockets, no threads, no timers,│
   │     no sleep, no clock, no file I/O, no logging it chose      │
   │     itself.  Links libm and nothing else.                     │
   │                                                               │
   │     session ── calibration ── history gather                  │
   │        │            │              │                          │
   │        └──── clock fit ── unwrap ──┘                          │
   │                    │                                          │
   │        codec ── commands(allowlist) ── coverage ── quat       │
   └───────────────────────────────────────────────────────────────┘
```

**Layer 1 is the whole library as far as a consumer is concerned.** PinPoint
links it and implements the contract of layer 2 over its existing
`BleImuTransport`. The reference transports of layer 3 exist so that a non-Qt
user gets something that works out of the box; they are separate CMake targets
and linking none of them is the expected case.

**There are two independent reasons for this split, and the second outlasts the
first.** The first is contention: a second scanner inside the same
process competes with the first for one adapter, which on BlueZ is a real
failure rather than a tidiness complaint. But even with no incumbent stack,
**the choice of BLE backend is a platform decision that belongs to the
application, not to a protocol library.** BlueZ, CoreBluetooth, WinRT, a
Zephyr port, a test double — mandating one of those would make the library
un-adoptable everywhere it was not chosen, and the protocol work has value to
anyone with one of these sensors. So the radio stays out even for consumers who
have no adapter to contend over.

**The purity rule is a test, not a promise.** `tests/purity.cmake` inspects the
built object's undefined symbols and fails if `clock_gettime`, `pthread_create`,
`socket`, `fopen`, `nanosleep`, `getenv`, `rand` or a D-Bus/HCI symbol appears.
Owning no radio is the property that makes the library adoptable at all, and it
is the kind of property that erodes one convenience at a time unless something
fails.
The current core's entire undefined-symbol set is `memcpy`, `memmove`, `memset`,
`memcmp`, `strcmp`, `strlen`, `snprintf`, `qsort`, `sqrt`, `sqrtf`, `acosf`,
`atan2f`, `llround` — plus the compiler's own stack-protector hook and GOT
reference, which are artefacts of the build rather than API the library reaches
for.

### 3.2 The transport contract

There is no abstract transport class. The host calls in; the library queues out.
That inverts the usual shape and it is the reason the stop barrier is free.

```c
/* IN — from the host's transport, on the session thread */
void hm_session_on_link_up(hm_session*, int32_t negotiated_mtu, hm_time_us now_us);
void hm_session_on_link_down(hm_session*, hm_link_down_cause, hm_time_us now_us);
void hm_session_on_bytes(hm_session*, const uint8_t*, size_t, hm_time_us host_recv_us);
void hm_session_on_advertising_seen(hm_session*, hm_time_us now_us);

/* CLOCK — the host owns the timer */
hm_time_us hm_session_next_due_us(const hm_session*);
void       hm_session_tick(hm_session*, hm_time_us now_us);

/* OUT — drained by the host, never pushed into it */
size_t hm_session_poll_writes(hm_session*, hm_write_request*, size_t);
size_t hm_session_poll_events (hm_session*, hm_event*,        size_t);
size_t hm_session_poll_live   (hm_session*, hm_sample*,       size_t);
size_t hm_session_poll_wire   (hm_session*, hm_wire_chunk*,   size_t);
```

A complete integration, in the shape PinPoint described:

```c
/* on QLowEnergyService::characteristicChanged */
hm_session_on_bytes(s, (const uint8_t*)v.constData(), (size_t)v.size(), now_us());
pump();

/* on the QTimer */
hm_session_tick(s, now_us());
pump();

static void pump(void) {
    hm_write_request w[8];
    size_t n = hm_session_poll_writes(s, w, 8);
    for (size_t i = 0; i < n; ++i)
        service->writeCharacteristic(chr, QByteArray((const char*)w[i].data, w[i].length),
            w[i].without_response ? WriteWithoutResponse : WriteWithResponse);

    hm_sample live[256];
    while ((n = hm_session_poll_live(s, live, 256)) > 0) ring.push(live, n);

    hm_event ev[64];
    while ((n = hm_session_poll_events(s, ev, 64)) > 0) for (size_t i=0;i<n;++i) handle(ev[i]);

    timer->start(msUntil(hm_session_next_due_us(s)));
}
```

`hm_session_next_due_us()` must be re-read after every call into the session.
It returns `HM_TIME_NEVER` when nothing is pending, and a host may then sleep
until transport traffic wakes it.

### 3.2.1 ⚠ One call, one notification — and why not a byte stream

**`hm_session_on_bytes()` takes one complete ATT notification payload.** It is
not a stream and the library does not reassemble one. It is the kind of choice
that is free now and a class of silent corruption later.

Three reasons, in increasing order of severity:

1. **Every stack preserves the boundary.** One `QByteArray` per
   `characteristicChanged`, one D-Bus signal or socket read on BlueZ, one
   `didUpdateValueForCharacteristic` on CoreBluetooth.
2. **The MTU floor guarantees it is sufficient.** `HM_MIN_ATT_MTU` is 96
   precisely so the 93-byte maximum message fits in one notification (§2.4).
   Having enforced that floor, the library is entitled to rely on it.
3. **⚠ The alternative is unimplementable in its failure case.** §3 gives the
   protocol *no length field, no sequence number and no checksum*. A byte stream
   with those three absences cannot be resynchronised after any loss, because
   there is nothing to synchronise **to**.

That last point is not theoretical. A `0x90` notification carries one or two
records with no count field, so a stream decoder has to decide by decoding the
second and checking its quaternion norms. When that check says no it reports
`consumed = 47`, leaving 46 bytes of a record in the
buffer — whose first byte is the high byte of a `u16be` sample counter, and
`0x90` is a perfectly reachable value for it. The next decode would then read 46
bytes of mid-record data as a frame. Everything downstream parses. Nothing
reports an error.

**So the record count comes from the length and never from the content:**

```
count = (length − 1) / hm_stream_config_record_size(cfg)     /* 46 or 42 */
```

- A payload that is not a whole number of records raises
  `HM_WARN_TRAILING_BYTES`. That is the signature of a coalescing transport, and
  the right response is to be loud on the first frame rather than absorb it.
  There is deliberately **no coalescing mode**: a mode nobody exercises is worse
  than a warning everybody sees.
- More records than §6.3 describes are decoded and reported
  (`HM_WARN_UNEXPECTED_RECORD_COUNT`) rather than silently truncated.
- **The norm check survives as what §6.4 offers it as** — evidence that the
  decode is *aligned* — reported on the sample (`HM_SAMPLE_QUAT_NORM_SUSPECT`)
  and as a warning, and never acted on.

Freed from framing duty, the tolerance could tighten to what the evidence
supports: **`HM_QUAT_NORM_TOLERANCE` went from 512 counts to 64**, against a
measured spread of ±0.41 over 6,064 records. 64 is still over 150× the observed
spread, so a correct frame can never trip it, while a decode misaligned by one
byte now does — which `codec_norm_tolerance_catches_a_one_byte_shift` asserts
both ways.

### 3.3 Poll, don't call back — and the stop barrier

**The library never invokes a consumer callback from inside its own I/O, and
teardown returns only once no further delivery can occur.** These are the same
property, and in this shape the second is a consequence of the first rather than
a promise about it:

- The library has no thread and no callback, so there is no producer to stop.
- `hm_session_close()` marks the session closed, seals the four queues, and
  returns. Nothing can be produced afterwards because nothing produces anything
  except a call the host itself makes.
- Destroying a ring, a file handle or a closure is safe from that instant.

A consumer's barrier is "stop draining", which it implements itself and cannot
get wrong. That is stated in `session.h`'s header comment as sentence three of
the threading contract.

### 3.4 Memory ownership

Three rules, and they cover everything:

1. **The session owns nothing the caller did not give it, except history
   blocks.** `hm_session_config.memory` carries caller-provided rings for live
   samples, events, wire chunks, the history gather area and the coverage
   intervals. ⚠ `hm_session_create()` makes **exactly one** allocation whatever
   the caller supplies — the session object itself has to live somewhere — and
   `hm_session_destroy()` makes exactly one free. Providing every ring does not
   reach zero; it reaches one, of a fixed and knowable size. Nothing allocates
   after create.
2. **A history block is an allocation, and it is the caller's.** It outlives the
   session, moves to another thread, and is freed with
   `hm_history_block_release()`. Supply `hm_allocator` to route those through a
   pool.
3. **Everything else is by value.** Samples, events, write requests, clock
   snapshots and wire chunks are POD with no pointers.

Rings are drop-oldest with a counter: `hm_session_dropped_live()`,
`_dropped_events()`, `_dropped_wire()`. A host that is not keeping up must be
able to see it, so loss is counted rather than silent, and `hm_event.sequence`
gaps show the same thing for events.

**Sizing.** `HM_LIVE_RING_RECOMMENDED` is 2048 samples. ⚠ Do not size it from a
rate: §6.6 measures dense bursts reaching index step 1 — the full ≈799.2 Hz — in
*every* session containing motion, so there is no live ceiling. Size it from how
often the host drains.

---

## 4. The type system, and why it is shaped this way

### 4.1 Time

Every `..._us` in the API is **microseconds on a clock the caller chooses and
the library never reads**. The library calls no clock function on any platform.
Host time enters only through `now_us` and `host_recv_us` arguments.

The clock must be **monotonic**. PinPoint uses `std::chrono::steady_clock`; a
wall clock would be stepped by NTP or DST and corrupt a capture in a way that
looks like a sensor fault. The epoch is arbitrary and never interpreted.

This also makes an entire session deterministic under a synthetic clock, which
is what the history-path tests of §10.4 need.

`HM_TIME_UNKNOWN` (`INT64_MIN`) marks a timestamp that is *structurally*
unavailable — the arrival instant of a history record, for instance, which
carries no information at all (§10.1). It is not a null; it is a statement.

### 4.2 One sample type

`hm_sample` serves live and history alike, rather than splitting out a separate
history type. One type is better for three reasons:

- A POD array with a documented, versioned layout is what a consumer needs. One
  layout is one version to manage.
- A consumer writes one conversion loop, not two.
- Live and history `0x90` frames are *byte-identical* (§10.1). Two types would
  imply a distinction the wire does not make, and the actual distinction —
  `source`, and whether `host_recv_us` means anything — is a field.

```c
typedef struct hm_sample {
    uint64_t   stream_id;         /* which a0 01 7e this index space belongs to */
    uint32_t   sample_index;      /* UNWRAPPED; monotonic within a stream */
    uint16_t   sample_index_raw;
    uint8_t    source;            /* live | history | replay */
    uint8_t    calibration;       /* unknown | uncalibrated | calibrated | lost */
    uint16_t   flags;
    uint8_t    config_bits;       /* the a0 01 <cfg> byte that produced this */
    uint8_t    reserved0;
    int32_t    skew_us;           /* palm − lower_arm */
    hm_time_us host_time_us;      /* MAPPED through the fit */
    hm_time_us host_recv_us;      /* arrival; UNKNOWN for history */
    uint32_t   precision_us;      /* link jitter + extrapolation — GATE ON THIS */
    uint32_t   uncertainty_us;    /* precision + the uncorrected systematic (§6.4) */
    hm_unit_sample lower_arm;     /* wire block 0 */
    hm_unit_sample palm;          /* wire block 1 */
} hm_sample;
```

192 bytes as built, 72 of them per unit — about 675 KB for a 4.5 s window, which
is the right trade against a second layout version.

### 4.3 The two units are named, not indexed

§6.3 settles the assignment by construction — the two units are one BLE
peripheral joined by a cable, the first block is the lower arm, the second is
the palm, and the wiring fixes it. A consumer that swaps them produces a
plausible-looking wrist angle that is simply **mirrored**, and every
plausibility check passes.

So the API offers `s.lower_arm` and `s.palm`, and `hm_sample_unit(s, unit)` for
generic code. There is no `unit[2]` to index wrongly.

**There is deliberately no aggregate.** §6.4: the units sit 3–8 cm apart, so
under rotation they are at different radii and their linear accelerations differ
by roughly ω²r — measured at 31–51 m/s² through a golf swing, consistently,
across five swings. They are supposed to disagree. There is no "device
acceleration", so the library does not offer one, does not average, and does not
treat a disagreement as a fault. The quaternions are unaffected, which is
exactly why the wrist metrics are built from orientation.

### 4.4 Raw counts are authoritative

`hm_unit_sample` carries both `int16` counts and scaled floats. The counts are
the record; the floats are a convenience computed under `config_bits`.

The reason is the *selector*, not the scale: the gyro divisor is 8 or
16 depending on config bit 6 and the block size is 22 or 20 depending on bit 5.
A sample exposing only `float gyro_dps` cannot say which configuration produced
it, and a recording made under one is silently wrong when read as the other.

`linear_accel_*` is named for what it is — §6.4's *gravity-removed* output,
reading ≈0 at rest. Anything called `accel` will be misread by someone.

### 4.4.1 The configuration carries its own justification

`hm_stream_config_nonstandard(bits, justification)` **copies** the string into
the config, and `hm_history_block` carries a `hm_stream_config` by value — so the
reason travels with the samples it describes and lands in the capture's
provenance without anything else having to route it.

⚠ **The string is stored, not discarded.** A mandatory string that is thrown away
reads as an audit trail and is not one. Keeping it grew the type from 4 to 68
bytes, which is the right trade — it is copied in a handful of places, none of
them hot, and it is the only way the mechanism's stated purpose is reachable.

A NULL or empty justification stores the literal `"(unjustified)"`. The function
returns by value and cannot report an error, and a recording that says nobody
gave a reason is more useful than one that is silent about it.

### 4.5 Errors

`hm_status`: non-negative is success (`HM_OK`, `HM_PENDING`, `HM_DONE`), negative
is failure. Test `st < HM_OK`, never `st != HM_OK`.

The library does not have an error *string* channel. Anything with detail is an
event, because an event is POD, queued, timestamped and loggable, and because a
string in a C API is a lifetime question nobody wants.

⚠ `0xd0 03` deserves its own note. §7.2 measured it across seven distinct
causes — a request before any stream ran, a reversed range, a null request, the
full index space, and indices from a previous session — and **every one returned
the same code**. So the wire cannot classify and the library must, from its own
state. `HM_EV_DEVICE_ERROR` reports the raw code; the *meaningful* status lands
on the history block (`HM_HIST_NO_STREAM`, `HM_HIST_EVICTED`, …) from what the
session knows.

### 4.6 ABI and bindings

- Opaque handle + POD structs + flat C functions. No callbacks in any hot path.
- `hm_abi_sizes_get()` reports every public struct's size as the library was
  built; `hm_abi_check()` compares a binding's expectation and fails at load
  rather than at random.
- `HM_SAMPLE_LAYOUT_VERSION` and `hm_history_block.sample_stride` let a
  persisted capture stay decodable when the struct grows.
- Reserved fields exist in `hm_sample` and `hm_history_block` so the first
  additions do not move anything.
- No global state. Multiple independent sessions may exist in one process.

An optional header-only C++ RAII wrapper (`hackmotion.hpp`) is planned; it adds
no ABI and nobody is required to use it.

#### 4.6.1 What the first binding taught us — ✅ phase 6

`python/hackmotion` is the first consumer of any of the above, and it found that
**`hm_abi_check()` is necessary and nowhere near sufficient.** It compares nine
struct *sizes*. It says nothing about where the fields inside them sit, and
nothing about any enumerator — so a binding whose `skew_us` is one slot out
passes the guard, decodes every sample, and returns plausible numbers with no
error anywhere. That is this project's recurring failure shape, in a new place.

Two things close it, and they are layered because neither is complete alone:

| | What it catches | What it cannot see |
|---|---|---|
| `hmwire abi` — sizes, field offsets and enumerator values read out of the compiler by `offsetof`, compared both ways against the binding (`tests/test_python_abi.py`) | any field moved, resized, added or dropped on either side; any enum value transcribed wrong | a field carved out of an existing `reserved` array and updated in neither the table nor the binding |
| `hm_abi_check()` at binding load | a library built from different headers than the binding was written against, **on a user's machine** | field offsets; enums; struct sizes outside its nine |

⚠ **The offset table is hand-written and its own completeness is bounded, in
writing.** `tools/hm_abi_table.c` requires its rows to *tile* each struct, which
catches an appended or mis-sized row — but a dropped row whose bytes are
indistinguishable from padding is invisible to any offset-based check, because
the information is not in the layout. Removing `hm_sample.skew_us` (an `int32`
before an 8-aligned field) leaves exactly the padding that would have been there
anyway; it was tried. The struct-size comparison is what actually catches that,
and the field-name comparison catches it again. All three are documented at the
top of that file with what each one misses.

⚠ **And a binding adds one hazard C does not have.** A `hm_history_block` owns
its samples; in ctypes that is a raw address, and a view kept past the release
returns plausible sample data. **A sanitizer does not see it** — ASan instruments
the code it compiled, and a ctypes read happens inside CPython, which it did not.
Measured: an escaped view read after release returns correct-looking indices
under `build/san` with no report. What ASan *does* catch there is a fault the
library commits at the binding's request — a double release reports
`heap-use-after-free` — so the sanitizer run is worth having for a different
reason than the obvious one. The stale-view half is covered by a guard in
`Samples._alive()` that raises, pinned by `tests/test_python_lifetime.py`.

The lesson generalises past Python: **every one of these checks is about a second
copy of a truth, and the useful question is never "is it checked" but "what can
this check not see".**

---

## 5. The session

Four small state machines rather than one large one, because the concerns are
close to orthogonal and a single combined machine would have states nobody can
reason about.

### 5.1 Link

| State | Meaning | On entry |
|---|---|---|
| `DOWN` | No link | Calibration → `UNCALIBRATED`; stream → `STOPPED`; fit sealed |
| `BRINGUP` | Link up, MTU accepted, §9.1 sequence running | Queue the bring-up commands; arm the bring-up watchdog |
| `READY` | Bring-up complete | Emit `HM_EV_READY`; start the keepalive |
| `CLOSED` | `hm_session_close()` called | Seal all queues |

| Transition | Trigger | Notes |
|---|---|---|
| `DOWN → BRINGUP` | `on_link_up(mtu ≥ 96)` | |
| `DOWN → DOWN` | `on_link_up(0 < mtu < 96)` | ⚠ Emit `HM_EV_MTU_REJECTED` and refuse to run — see §5.2 |
| `BRINGUP → READY` | `0x80` seen (the only required step) and the rest either answered or timed out | |
| `BRINGUP → DOWN` | bring-up watchdog (10 s) | Classify per §5.4 |
| any `→ DOWN` | `on_link_down` | ⚠ Always invalidates calibration |

`device_id` is a caller-supplied opaque string, never a MAC. On
macOS, CoreBluetooth never exposes a Bluetooth address; it gives a per-host,
per-application UUID that is not stable across machines, so any API shaped as
`connect(MacAddress)` is unimplementable there. The device *does* report its own
MAC over the wire (`0x85`), and that is a genuinely useful stable identifier —
but it arrives **after** connection, so it can identify a device in a recording
and cannot be used to find one.

### 5.2 MTU

The calibration result is 65 bytes and stream notifications reach 93, both
beyond the 20-byte payload of the default ATT MTU (§2.4). No platform lets an
application *request* an MTU through Qt — `QLowEnergyController::mtu()` is
read-only — so the library cannot ensure this. It checks:

| `negotiated_mtu` | Behaviour |
|---|---|
| `≥ 96` | Proceed |
| `1 … 95` | ⚠ `HM_EV_MTU_REJECTED` carrying the value; the session refuses to run |
| `0` | "The platform will not tell me": warn and proceed |

Failing loudly beats truncated frames that parse as garbage on whichever
platform eventually gets it wrong.

### 5.3 Bring-up

§9.1's sequence, with one deviation:

```
1  (the host enables notifications on CCCD 0x0019 — transport's job)
2  80   versions      ← the only REQUIRED step; protocol version gates features
3  81   status
4  84   sensor map
5  81   status again
6  86   serial        ← the vendor app sends this THREE times; we send it once
7  85   MAC
```

Steps 3–7 are informational and the library tolerates any of them going
unanswered. The triple serial read is the vendor app's behaviour, not a device
requirement; reproducing it exactly is a useful bring-up *test* (§10.5 keeps it
as a replay fixture) but not useful in production.

### 5.4 The keepalive, and link loss

⚠ **The 5-minute idle shutdown applies while connected, and an active stream
does not prevent it.** §9.2: a connection streaming continuously at 25 Hz was
dropped at exactly 5.0 minutes, the same deadline as a fully silent one. What
resets the timer is a **host→device write**, not motion and not data flowing the
other way.

So the library polls `0x81` every 30 s, unconditionally, for the whole
connection, from `READY` onward. `hm_session_policy.keepalive_period_us` clamps
to `(0, 60 s]` and 0 selects 30 s. **There is no way to switch it off.** The
battery reading is incidental.

Any host→device write resets the timer, so a busy session sends few extra polls.

#### ⚠ The write quiet period — and it is the keepalive's only exception

**No host→device write is emitted while a history bracket is open**, between the
`a1` going out and its closing `a1 01`. A keepalive that ran from `READY` onward
unconditionally would queue an `0x81` mid-retrieval.

That is an interaction the specification never exercised. §7.5's measurements
were an `a1` issued into a running stream with *nothing else being written*;
whether the device tolerates an unrelated command while it is bulk-replaying its
buffer at ~260 notifications/s is simply unknown. And the failure would be the
worst shape this protocol admits — an `0x81` reply interleaved into a record
stream that §3 gives no length field, no sequence number and no checksum to
resynchronise on.

It is also free, which is why it is worth deciding rather than discovering:

- **The `a1` write itself reset the idle timer** at the moment the bracket
  opened, so the keepalive has already been served.
- **A retrieval is bounded by the buffer depth** — ~7.5 s at the very most
  (§7.3) — against a 300 s idle shutdown. Two orders of magnitude of headroom.

The quiet period has **four** exits: the closing marker, link-down, a hard
bracket limit of 15 s (twice the §7.3 depth seed — §5.7), and consumer-initiated
teardown, which closes the bracket itself rather than waiting. The keepalive is
re-armed from whichever ends it, so a wedged bracket cannot hold the queue
indefinitely.

⚠ **NOT at the request's deadline, and that is deliberate.** A request
giving up does not tell the *device* to stop replaying, and closing the bracket
while records are still arriving would put ~4,000 bulk arrivals onto the live
path, where their indices — thousands of samples behind — take the live unwrapper
with them. The request materialises immediately with what it got; the bracket
stays open and its remaining records are counted and discarded. `session.h` says
so at the API.

**Link loss is classified, not retried.** §9.6: a dropped link is an ordinary
supervision timeout and reconnect usually succeeds; a slept or powered-off device
needs a physical button press and nothing else. Retrying against a slept device
cannot succeed at any interval — it burns host battery, occupies the adapter, and
shows the user a spinner for a problem only they can fix.

| Evidence | `advice` |
|---|---|
| We called `hm_session_power_off()` | `DO_NOT_RETRY` |
| We called `hm_session_close()` | `DO_NOT_RETRY` |
| Cause is `CONNECTION_TAKEN` | `NEEDS_OTHER_APP_CLOSED` |
| Idle ≥ 4.5 min, or clean remote close after a long quiet | `NEEDS_BUTTON_PRESS` |
| `on_advertising_seen()` within the last few seconds | `RECONNECT_WITH_BACKOFF` |
| Otherwise | `RECONNECT_WITH_BACKOFF`, with a suggested delay |

The library **advises**; the host reconnects, because the host owns the radio.
`hm_link_down_event.suggested_retry_delay_us` carries an exponential backoff the
host may use or ignore: reconnect competes for an adapter the application's own
pool arbitrates, so it has to stay the application's.

`calibration_invalidated` is always 1 in that event. §8.3 measured 0.70°
immediately before dropping a link and 18.80° at the same pose after
reconnecting, strap untouched and never removed.

### 5.5 The stream

**One stream, opened once, left open** (§7.6).

| State | Meaning |
|---|---|
| `STOPPED` | No stream. Nothing is being recorded anywhere — §6.5, there is no background sampling |
| `STARTING` | `a0 01 7e` queued; ⚠ waiting for the first `0x90`, **not** for the `a0 01` ack, which the vendor app ignores and so do we |
| `RUNNING` | Frames arriving; the clock fit is live |
| `STOPPING` | `83` queued |

`hm_session_start_stream()` allocates a new `stream_id`, resets the index and
tick unwrappers, and calls `hm_fit_begin_stream()` — which folds the finished
stream's rate into the connection-pooled estimate and throws the index space
away (§6.3).

⚠ **The library never restarts a stream on a consumer's behalf.** §7.6 lists
restarting first among the five silent ways capture goes wrong: it clears the
buffer, resets the index space, and starts the clock fit from nothing — and
whether it also costs the calibration is untested, where a *disconnect*
demonstrably does. If a stream is ever restarted, that is
`HM_EV_STREAM_RESTARTED`, not an implementation detail.

**Teardown.** `83` stops cleanly; disconnecting also stops the stream.
`hm_session_power_off()` writes `fa`, after which ⚠ **no acknowledgement arrives
and the link stays up for about 9 seconds** (§9.3). The session enters a linger
state, expects the gap, does not read it as failure and does not retry into it.

### 5.6 The wire log

⚠ **Record the wire bytes, not the decoded samples**. §12 still lists
undecoded fields — two bytes of the battery reply, the second sensor-map byte,
the 64-byte calibration payload, two configuration bits — and §6.6's burst
trigger and §10's unexplained drift are open. When any of those is settled, a
byte-level recording can be **re-decoded with the fix applied**; a sample-level
one cannot, because it has already discarded the bytes the fix would have
interpreted differently.

The core does no file I/O. It copies chunks into a caller-supplied
`hm_wire_chunk` ring and the host drains them with `hm_session_poll_wire()`.
Each chunk carries direction, host time, a sequence number and up to 256 bytes.
Supplying no ring disables the log and costs nothing.

⚠ MAC (`0x85`) and serial (`0x86`) replies are **redacted** in the log unless
`policy.record_identifiers` is set, and redaction is marked with
`HM_WIRE_REDACTED` so a reader knows something was removed rather than absent.

The optional `hackmotion_record` target writes chunks to a versioned container
and replays them back through `hm_session_on_bytes()` under a synthetic clock.
A text header so `file(1)` and a human can identify it, then fixed-size
length-prefixed records. **Implemented; this is the format, not a sketch.**

```
HMWIRE1\n
device_id=<opaque>\n
config=0x7e\n              (or `config=legacy` — the `82` start has no byte)
layout_version=1\n
clock=monotonic_us\n
byte_order=little\n
identifiers=redacted\n
\n
<u32 length><u8 direction><u8 flags><u16 reserved><u32 sequence><i64 host_time_us><bytes...>
```

Three things settled during implementation, each because leaving it implicit
would have cost something:

- ⚠ **`sequence` is carried, not renumbered.** `HM_WIRE_LOST` says chunks were dropped before this one but not *how
  many*, so a reader renumbering from its own ordinal would turn a lossy
  recording into a complete-looking one — the exact shape of "no evidence read
  as agreement" that §10.2's tests exist to prevent. Twenty bytes per record,
  not sixteen.
- **Integers are little-endian and the header says so.** §1 warns that byte
  order is not uniform across this protocol; a container that left its own order
  to be inferred would be one more thing to infer. The payload is untouched and
  still big-endian inside.
- **An unknown header key is ignored, a malformed known one is an error** — the
  same rule §5.1 gives for unknown message ids, for the same reason.

`hmwire reconcile` reads a capture back against this specification claim by
claim: framing, `|q|`, the fitted rate against 799.2 *and* against 800, the tick
ratio and the two units' agreement, the skew across both halves of the session,
§6.6's step distribution, §6.3's palm-identification check, §9.1's bring-up and
§9.2's keepalive gap. ⚠ Each verdict is one of *match*, *DIFFERS* or **no
evidence**, and the third is never folded into the first.

### 5.7 Timers

`hm_session_next_due_us()` is the minimum of whichever of these are armed:

| Deadline | Armed when | §  |
|---|---|---|
| keepalive | `READY` onward, always | 9.2 |
| bring-up watchdog | `BRINGUP` | 9.1 |
| stream-start watchdog, 3 s | `STARTING`; expiry → `STOPPED` + `HM_WARN_STREAM_START_TIMEOUT` | 6.1 |
| stream-stop watchdog, 3 s | `STOPPING`; expiry → `STOPPED` + `HM_WARN_STREAM_STOP_TIMEOUT`. ⚠ Same knob as the row above — one round trip, one measurement (§6.1's 50–80 ms). ⚠ Without it this would be the only device-facing wait in the table with no way out, and a device that dropped the `83` would wedge the session silently and permanently | 6.1 |
| calibration raise limit | `OBSERVING_RAISE` | 8.2, client policy |
| calibration device bound | `MARKING_POSE0`, `MARKING_POSE1`, `APPLYING` — a marker's reply or the result | 8.2 |
| calibration presence window, 2 s | a presence run is collecting live samples | 8.2 |
| history request start | a reservation whose window end is in the future | 7.6 |
| history request deadline | a request in flight or queued | 7.4 |
| history eviction warning | a queued request | 7.3 |
| clock event period, 1 s | fit has observations | 10 |
| keepalive alarm, 120 s | no host→device write for that long → `HM_WARN_KEEPALIVE_LATE` | 9.2 |
| live-gap alarm, 3 s | streaming and no live frame; ⚠ suppressed inside a bracket | 10.1 |
| pinned report, 1 s | pinned count rose since the last report → `HM_EV_PINNED_SAMPLES` | 6.4 |
| power-off linger | `fa` written | 9.3 |
| bracket limit, 15 s | a history bracket is open, or an `a1` is out with no `a1 02` yet. ⚠ **The real bound on the write quiet period**, and it was missing from this table — see §5.4 | 7.5 |

---

## 6. Time

This is the part of the library that decides whether a swing can be placed on a
camera timeline, so it is specified in more detail than the rest.

### 6.1 The framing that makes it necessary

**The live stream is not the data. It is the clock reference for the data**
(§7.6). A 250 ms downswing is 6 samples at the 25 Hz live rate and 25 at 100 Hz
— not enough to describe the event at all. The same 250 ms is ~200 samples in
the device's internal buffer, now **measured on hardware: 201 records over the
201 indices spanning ±125 ms of peak, in all ten swing captures** (§7.3). So the
live stream exists to produce `(index, hostArrival)` pairs for the fit, and every
sample a consumer analyses comes from a retrieval — whose arrival timestamps
carry nothing.

⚠ **That density is bought by the motion, not by the request.** The buffer is
motion-adaptive: the same retrieval returns ~100 Hz over a still pre-roll, and a
session with no real swing in it returns ~100 Hz throughout (§7.3). The premise
above holds exactly where the event is and only there — which is enough, because
that is where the data is wanted, but it means **a consumer cannot validate its
retrieval path on a stationary sensor.** At a desk a correct implementation and a
broken one produce the same even one-in-eight.

### 6.1.1 ⚠ The mapping is piecewise, and a retrieval is where it breaks

**The sample counter stops while a retrieval is in flight** (§7.5, measured
across six pulls). Wall time advances, the index does not, and nothing in the
data marks the step. So `host_us(index) = anchor + (index − anchor) × slope` is
not one line across a stream that contains a pull.

**One rate per connection, one offset per stretch between retrievals** (§10). A
history record is mapped with the offset of the stretch its index falls in.

Two rules follow from §10.2's wrap budget, and they are hard constraints on the
session rather than advice:

| | |
|---|---|
| **Re-anchor the fit after every pull** | A fit anchored once at stream start was out by 111,311 ticks after five pulls — 1.70 wraps of 65,536, or 3.40× the ±32,768 that decides a wrap. Every history record dated after the first pull is then wrong by a multiple of 1.023 s. |
| **Never two pulls inside one live-frame gap** | One stall is ~23,500 ticks, **72% of the ±32,768 budget**, where a pull-free gap uses 12. Two before the next live frame exceeds it and the unwrapper silently picks the wrong wrap. |

⚠ **The whole-session fit fails loudly, and that is worth keeping.** Fitting the
44.5 s six-pull capture as one segment returned **768 Hz against a true 801**,
and the lower-envelope estimator reported itself `HM_CLOCK_DEGENERATE` with
**1.7 s residuals** rather than misaligning quietly. A consumer that fits per
segment and compares segments has a consistency check; one that fits the whole
session and trusts the number does not. §10.2's `worst_margin` is the other half
of the same alarm.

### 6.2 Unwrapping: two clocks that solve each other's problem

| | Resolution | Wraps every | Weakness |
|---|---|---|---|
| Sample index | 1.251 ms | 82.0 s | Coarse |
| Tick counter | 15.6 µs | **1.023 s** | Wrap ambiguous across any gap > 1.02 s |

**Index.** Monotonic within a stream (§6.5), so the *unsigned* 16-bit difference
is the true step and wraps take care of themselves for any gap under 82 s. A
step above `HM_INDEX_REGRESSION_LIMIT` (49152) is counted as suspect and
reported; the sample is still delivered.

**Ticks.** §10.2's algorithm, encoded once:

```
predicted = round((index − anchor_index) × ratio)
residue   = (uint16)(raw − anchor_raw)
wraps     = round((predicted − residue) / 65536)
unwrapped = residue + wraps × 65536
margin    = 32768 − |unwrapped − predicted|      /* how far from ambiguous */
```

`ratio` seeds at the nominal 80.166 and is refitted from the endpoints once the
baseline reaches 4000 indices (≈5 s), then on every doubling. **No host clock is
involved**, which is why it works identically for history.

§10.2 verified this over a 238 s session — 2.9 index wraps, ~233 tick wraps,
worst-case error 2,745 ticks against the ±32,768 budget, 8.4%, zero failures.
`tests/test_unwrap.c` reproduces it at the same scale and additionally forces a
5-second hole — nearly five tick wraps — which is the case a host-clock-based
approach cannot survive. The session raises `HM_WARN_TICK_PREDICTION_MARGIN` if
the margin ever falls below a fifth of budget.

**Skew.** ⚠ The two counters are independent free-running MCU timers. §10.3
measures a stable 59-tick (0.92 ms) difference — identical median across the
first and second halves of a 238 s session. Interpreting the raw difference as
signed modulo 65536 recovers it exactly, including on the first record of a
stream before any anchor exists, and including when one counter has wrapped and
the other has not.

It matters: at 1,000 °/s of wrist rotation, 0.92 ms is ~0.9° in the relative
angle — the primary output. Being stable it is subtractable as a constant, so a
consumer can proceed without knowing whether it is true sampling skew or
arbitrary counter phase — but the library carries it explicitly in `skew_us`
rather than silently pairing the two blocks as simultaneous.

⚠ And the library does not try to resolve it. §10.3 records that the obvious
impulse experiment **cannot work in principle** — a tap is shorter than the
1.25 ms sample period, so it lands one sample wide in each unit with no waveform
to align. Nobody should spend a session on it.

### 6.3 The clock fit

**The estimator.** Minimise `Σ(hᵢ − line(iᵢ))` subject to `line(iᵢ) ≤ hᵢ` for
all i: the highest line that still lies under every observation.

⚠ **Not least squares.** BLE delay is one-sided — a notification can be late,
never early — so least squares biases the offset by the *mean* link delay, which
reaches the user as every swing landing consistently late (§10).
`tests/test_clock.c` computes both over the same synthetic session and asserts
the envelope beats least squares by more than 2×, with the least-squares bias
matching the mean delay.

**Why it is cheap.** Write the line as slope `s` through a point `(i₀, h₀)`:

```
cost(s, i₀, h₀) = (Σh − n·h₀) − s·(Σi − n·i₀)
```

which is *linear in s* given running sums. `min_j(hⱼ − s·iⱼ)` is concave, so the
objective is convex piecewise-linear in `s` and its minimum lies at a breakpoint
— and the breakpoints are exactly the edges of the **lower convex hull** of the
observations. So:

1. Maintain the lower hull incrementally. Points arrive with `i` ascending, so
   this is Andrew's monotone chain with O(1) amortised cost.
2. Scan its edges once, O(hull).

The result is **exact**, not iterative, and bounded in both time and memory.

**Bounded memory.** The hull is capped at 96 vertices. On overflow the interior
vertex whose removal changes the chain least is dropped — keeping the two
extremes, and therefore the baseline the rate rests on. A subset of points in
convex position is still in convex position, so the chain stays a valid hull;
the cost is that a candidate line may sit up to `hull_drop_error_us` above one
discarded observation, and that bias is accumulated and folded into the reported
residual maximum rather than quietly ignored.
`clock_bounded_hull_survives_an_all_vertices_session` forces every observation
onto the hull and checks the fit stays sane.

**Fit the rate too.** ⚠ §6.5 measures 799.19 and 799.32 Hz on two independent
host clocks and a second unit at 799.47–799.55 — about 400 ppm away. A round 800
costs ≈1,000 ppm: 1 ms per second of streaming, one-directional, 45 ms after
45 s, which is 11 frames at 240 fps.

And it does something worse. **The lower-envelope estimator silently
degenerates:** with the rate wrong, `hostArrival − index/rate` drifts
monotonically instead of hovering, so its minimum always lands on one *end* of
the session — in practice the first frame, the one most exposed to
connection-setup jitter. The estimator collapses from a robust fit over
thousands of frames onto a single worst-case point *while still reporting small
residuals*.

Three guards:

- The rate is fitted jointly, so this cannot arise from the library's own
  arithmetic.
- If the fitted rate falls outside 795–803 Hz, `HM_CLOCK_RATE_IMPLAUSIBLE` is
  raised and the seeded rate is used — an implausible rate is a symptom, not a
  value to publish.
- If the optimal support edge spans under 2% of the baseline, or if a *pinned*
  rate disagrees with the empirical slope by more than 200 ppm,
  `HM_CLOCK_DEGENERATE` is raised and the reported uncertainty is inflated.
  `hm_fit_force_slope()` is a test-only hook that pins the rate; the test pins
  800 Hz and asserts the flag fires, then releases it and asserts the joint fit
  over the same data does not.

**Rate pooled, offset per stream.** §10: the counter resets at every stream start
but the crystal does not, so `hm_fit_begin_stream()` folds the finished stream's
slope into a span-weighted pooled estimate and seeds the next stream with it.
Only a fit with a real baseline and no degeneracy earns a vote. A stream too
short to separate rate from offset (< 2 s) uses the seed and raises
`HM_CLOCK_SHORT_BASELINE` rather than publishing a wild slope derived from a few
milliseconds of jitter.

⚠ **Read "per stream" as "per stretch", because §6.1.1 reuses the same
operation at every history pull.** So the pooled rate is a span-weighted average
over the stretches *between* pulls as well as over streams, and a connection that
never restarts its stream still accumulates one. That is defensible — the crystal
is the same throughout — but it is not what "carried over from earlier streams"
says, and `HM_CLOCK_RATE_POOLED` is now raised only where the published slope
really *is* the seed rather than wherever a pooled value merely exists.

### 6.4 ⚠ Precision and accuracy are different numbers — and different fields

**Reporting both added together, in one figure, makes the feature unusable.**

**Why.** §10, validated against five struck golf balls with a microphone
reference on the same host clock: the mapping drifts at **2.2 ms per second of
session, R² = 1.000**, of which roughly two thirds is not accounted for by the
reference used to measure it — and three candidate causes were eliminated by
direct measurement. Under the one-stream cycle the library correctly insists on,
`first_index` sits at the start of the lesson and stays there, so the term grows
for the whole session:

| Session elapsed | Precision and accuracy as one figure | Meets a 4,167 µs budget? |
|---|---|---|
| 0.5 s | 4,098 µs | yes |
| **1.0 s** | **5,199 µs** | **no** |
| 60 s | 135,000 µs | no |
| 600 s | **1,323,000 µs** | no |

At ten minutes it reported 1.32 seconds of uncertainty — larger than the 4.5 s
window it was describing — so `alignment_budget_us` and `HM_HIST_REFUSED_ALIGNMENT`
were permanently on and a consumer gating on one camera frame got zero swings.

**And it is the wrong statistical treatment.** R² = 1.000 against session time
means this is a **systematic**, and systematics are *corrected*, not *budgeted*.
Folding a perfectly predictable linear term into an error bar says "we do not
know where this sample is to within 1.3 seconds" when the truth is "we know where
it is, up to a slope somebody measured once and nobody has applied".

**So the two are split at the API, not only in the comment:**

```c
typedef struct hm_clock_error {
    uint32_t precision_us;   /* link jitter + extrapolation — GATE ON THIS */
    uint32_t systematic_us;  /* §10's measured, uncorrected drift          */
    uint32_t total_us;       /* the honest total, for provenance           */
} hm_clock_error;

hm_clock_error hm_clock_error_at(const hm_clock_snapshot *fit, uint32_t index);
```

```
precision  = residual_p90_us                          link jitter
           + extrapolation                            beyond the observations
               = residual_max_us + gap_us × (residual_max_us / span_us)
           × 2 + residual_max_us   if DEGENERATE
           + residual_max_us       if SHORT_BASELINE
           ⌊ slope_us_per_index    if span_us == 0    ⚠ a FLOOR, not a term

systematic = accuracy_drift_us_per_s × seconds_since_first_index

total      = precision + systematic
```

⚠ **Every term but the last is a multiple of a residual, and that is what the
floor exists for**. A fit whose evidence rests on
a single instant has no residuals — one point cannot disagree with a line drawn
through it — so all of them evaluate to zero and the function reported *perfect
accuracy from one observation*. `hm_clock_meets_budget()` then returned true for
any budget whatever, which is B2's whole feature switched off without a trace.

⚠ **And §6.1.1 made that state recur once per PULL rather than once per stream.**
The re-anchor after every retrieval wipes the residual ring. Two things answer
it, and neither invents a number:

- **The residual figures carry across a re-anchor**, for the same reason the rate
  is pooled: jitter is a property of the *link*, and the link did not change while
  the device replayed 400 samples. `hm_fit.carry_p90_us` holds the connection's
  last real measurement and stands in while `span_us` is 0.
- **One sample period floors the first fit of a connection**, where there is
  nothing to carry. ≈1251 µs at 799.2 Hz — with one jittered arrival and a
  *seeded* rate, the anchor cannot be placed better than the interval between the
  samples that were not seen. It is under a third of a 240 fps camera frame, so
  the alignment a consumer actually asks for is still servable from frame one;
  a sub-millisecond budget is refused until the fit has earned it.

`HM_EV_CLOCK_DEGRADED` is deliberately **not** raised for this state. The flag
is set at every stream start and every pull and the event is edge-triggered, so
it would alarm on the ordinary shape of the data — the mistake §8.7 already
records about holing. An honest `precision_us` is the mechanism; an event is not.

`hm_clock_meets_budget()` compares against **precision**. Both numbers are on
every sample (`precision_us`, `uncertainty_us`) and both go into the capture's
provenance, so nothing is hidden — a consumer records that its alignment was
good to 3 ms of jitter *and* that an uncorrected systematic of 1.3 s was in
force. `UINT32_MAX` in all three means "no fit; do not align".

**The drift default stays at the full measured 2200 µs/s**, because the library
cannot attribute any of it. ⚠ But it is now documented for what it is: **the
least transferable number in the specification.** Everything else in §10 is a
property of the device; this is a property of a whole measurement chain —
sensor, link, host clock, sound card, room — measured once, on one rig, against
a reference whose own calibration §10 and §12 disagree about — 759 ppm slow in
one place, eliminated at −219 ppm in the other.

⚠ **The unit is µs per second.** 2.2 ms/s is 2200 µs/s. Getting this wrong by
three orders of magnitude is invisible; it happened once during development and
the test now asserts the number twice, in both forms.

**Correcting a rig moves two knobs, in one call.** `rate_ppm` changes the slope;
`residual_drift_us_per_s` changes the systematic that remains *after* it.
Exposing only the first would let a consumer feed back a measurement, see its
reported uncertainty unchanged, and reasonably conclude the call did nothing.
They come out of one measurement, so they are set from one struct:

```c
hm_clock_correction c = {0};
c.fields                  = HM_CORRECTION_RATE | HM_CORRECTION_DRIFT;
c.rate_ppm                = -350.0;
c.residual_drift_us_per_s =  400.0;
snprintf(c.provenance, sizeof c.provenance, "bay 3 camera+mic 2026-08-20");
hm_session_set_clock_correction(session, &c);
```

⚠ **`fields` is required and an empty one is an error.** A value sentinel —
negative meaning *leave alone* — would be worse: `= {0}` is the idiom for every
other struct in this API, so the one struct whose zero value wiped a term nobody had
measured was the one a consumer would reach for by accident. The result would be
a capture whose total equalled its precision, claiming an accuracy it had not
earned, with a provenance string describing a measurement that covered half of
it — silent, and optimistic.

The bitmask makes all three failures unreachable: `{0}` is rejected rather than
obeyed or ignored; setting only the rate leaves the systematic exactly where it
was; and "I measured it and it is zero" is expressible by flagging the field.

⚠ Note the deliberate asymmetry with `hm_session_policy`, where 0 means *use the
default*. Here a flagged 0 is a **measured** zero. The two reach the same
underlying value by different routes — one is a starting assumption, the other a
measurement — and they are named differently (`accuracy_drift_us_per_s` vs
`residual_drift_us_per_s`) for exactly that reason. Neither ever means "guess".

Everything set here is carried in every subsequent snapshot, and therefore into
every history block.

**Residual spread is a link-health signal** (§10). BLE at range delays
notifications and nothing in the protocol reports it — the fit degrades quietly
while every frame still parses. `HM_EV_CLOCK_DEGRADED` fires when p90 crosses
`policy.residual_alarm_us` (20 ms by default). A library reporting only a point
estimate of its clock offset has thrown away its only warning that the data is
drifting out of alignment.

### 6.5 Host time in, indices out — one conversion, written once

⚠ `hm_time_range` is **half-open** `[start, end)`; `hm_index_range` is
**inclusive** `[first, last]`. Both are documented, and that is exactly the
moment an off-by-one is born, so the conversion exists once:

```c
hm_status hm_clock_index_range_for_time(const hm_clock_snapshot *fit,
                                        hm_time_range window, hm_index_range *out);
```

`first` is the smallest index whose mapped host time is `>= start`; `last` is the
largest whose mapped time is `< end`. A window falling entirely between two
samples returns `HM_ERR_INVALID_ARG` rather than an inverted range. Nothing else
in the library derives one range type from the other.

### 6.6 The snapshot is a value

`hm_clock_snapshot` is POD, self-contained, and carries `anchor_index`,
`anchor_host_us`, `slope_us_per_index`, both rates, the external ppm and its
provenance, the observation count, span, index range, the residual distribution
and the drift constant. `hm_clock_to_host_us()` and friends are pure functions
on it.

That gives re-analysis determinism: a fit living on a live device object
cannot be persisted, and one queried after the fact is a different fit.

---

## 7. Calibration

### 7.1 What the device does

Three frames are in play — earth, sensor and anatomical. The relative rotation
cancels earth but still carries both mounting offsets; uncalibrated, a straight
wrist reads 11–15°, which is board placement and not anatomy. Calibration
supplies a per-unit mount quaternion, and **the device applies it itself** —
both quaternions step discontinuously the instant the result is emitted and the
relative angle collapses to ~0.4°.

So the library issues two markers and reads angles already in the anatomical
frame. There is no host-side transform to replicate, and none is written "for
validation".

### 7.2 The state machine

⚠ **The stream must already be running.** §8.2: calibration is not a standalone
transaction — the device observes a *continuous raise* between the two markers,
which cannot be done from two static samples. `hm_calibration_begin()` returns
`HM_ERR_NO_STREAM` rather than trying — and there is deliberately **no**
`AWAIT_STREAM` phase to wait in. Under the one-stream cycle the stream is open
from just after `HM_EV_READY` and stays open, so a wait would never be
entered, and an unreachable state is worse than no state. `confirm_horizontal()`
re-checks, because the stream could stop in between.

| Phase | Meaning | Exits |
|---|---|---|
| `IDLE` | | `begin()` → `AWAIT_HORIZONTAL`, or `HM_ERR_NO_STREAM` if not streaming |
| `AWAIT_HORIZONTAL` | UI asks for pose 0: forearm horizontal, wrist straight | `confirm_horizontal()` → `MARKING_POSE0` |
| `MARKING_POSE0` | `a2 00` written | `a2 01` reply → `OBSERVING_RAISE`; device bound → `ABORTED` |
| `OBSERVING_RAISE` | The device is watching a continuous raise | `confirm_raise()` → `MARKING_POSE1`; raise limit → `ABORTED` |
| `MARKING_POSE1` | `a2 01` written | `a2 01` reply → `APPLYING`; device bound → `ABORTED` |
| `APPLYING` | Awaiting `0x94` | `0x94` → `VERIFYING`; result timeout → `ABORTED` |
| `VERIFYING` | Transform applied; presence not yet measured | `confirm_reference_pose()` → measure → `COMPLETE`; `abort()` → `COMPLETE` |
| `COMPLETE` | | |
| `ABORTED` | A real state carrying a reason, not an error string | |

Every transition emits `HM_EV_CALIBRATION_PHASE`, so a UI renders progress from
events rather than polling.

⚠ **`abort()` at `VERIFYING` reaches `COMPLETE`, not `ABORTED`.** The
transform is *already applied* by then and no command reverses it (§8.2's device
re-references its own stream the instant it emits the result), so an abort there
is a decision to skip the presence check rather than to cancel a calibration.
The phase carries `HM_CAL_ABORT_CALLER` so the event says who ended it, the angle
stays NaN and the flag stays `HM_CAL_UNKNOWN`. Reporting `ABORTED` would tell a
consumer nothing happened to a stream whose frame had just changed underneath it.

⚠ **Both marker acknowledgements are bounded too**, by the same
`policy.calibration_result_timeout_us`.
§8.2 documents a reply to every marker and both captures show one, so a marker
that goes unanswered is a failure rather than a slow device — and unbounded it
strands a wizard on a user holding their arm out with no way back but an explicit
abort. **Which** wait expired is in `previous_phase` on the event, which is why
one bound needs no second abort reason.

⚠ **The device imposes no deadline.** §8.2: one attempt took 15.6 s between
markers and the device returned a result and applied it — it was the vendor's
*application* that rejected it. `policy.calibration_raise_limit_us` defaults to
6 s and is documented as **client policy**, never as a device constraint.

⚠ **`0x94` is not a verdict.** §8.2: it is emitted for *every* `a2 01` and the
device applies the transform every time, including attempts an application goes
on to reject. Nothing on the wire carries accept or reject. A client that treats
its arrival as success has no failure detection at all.

**Automatic raise detection is not shipped**: it means watching the stream for a
motion pattern whose criterion is not established. The
host-confirmed path keeps an undecided behaviour out of the contract, and a
consumer with a good UI can do something sensible immediately.

### 7.3 The per-sample flag

`hm_sample.calibration` is one of `UNKNOWN / UNCALIBRATED / CALIBRATED / LOST`
and travels with the data. Pre-calibration quaternions are perfectly
valid geometry and completely meaningless anatomically, and the transform is
applied on-device and is not recoverable later — so if the recording does not
carry this flag, the mistake is permanent and invisible.

**Where each value comes from**, settled in phase 3:

| Value | Written when |
|---|---|
| `UNCALIBRATED` | At create and at every link-up: no routine has been run, and we have looked |
| `UNKNOWN` | ⚠ From the arrival of `0x94` until a presence check passes — **and on a stream restart from `CALIBRATED`** |
| `CALIBRATED` | Only by a presence measurement below `HM_PRESENCE_CALIBRATED_MAX_DEG` |
| `LOST` | ⚠ **Never on a sample.** See below |

⚠ **`0x94` moves the flag whatever the state machine thinks.** The device
applies the transform for every `a2 01` and re-references its own stream at the
instant it emits the result (§8.1), so from there the streamed orientations are
in the anatomical frame — including for a result later than
`policy.calibration_result_timeout_us`, after the library has already given the
attempt up. The flag therefore follows the *device*: `HM_CAL_UNKNOWN`, with
`HM_WARN_CALIBRATION_UNSOLICITED` reporting the arrival. Continuing to label
those samples `UNCALIBRATED` would be a confident claim about a frame that had
just changed.

⚠ **A stream restart drops `CALIBRATED` to `UNKNOWN`.** §7.6 lists restarting
first among the five silent ways capture goes wrong, and whether it also costs
the device's transform is untested where a *disconnect* demonstrably does
(§8.3). The choice is between samples that keep
claiming `CALIBRATED` across an unmeasured boundary and samples that say we no
longer know; the second is recoverable by re-running the routine and the first is
permanent and invisible. It moves **only** from `CALIBRATED` — `UNCALIBRATED` is
a thing we know, and a restart is no reason to stop knowing it.

⚠ **`HM_CAL_LOST` never appears on a sample**, and phase 3 settled that rather
than inheriting it. §8.3 measures calibration being destroyed by a plain
disconnect, but **no sample can be captured between the loss and the notice of
one**, so a per-sample `LOST` would label nothing; §5.1's link-down goes straight
to `UNCALIBRATED`. The value's work is on `hm_calibration_span` (§8), where
`state_at_start = CALIBRATED` with `state_at_end = LOST` distinguishes "never
calibrated" from "calibrated, and then it went away inside this block" — which is
what `spans_transition` is for, and is phase 4's to write.

⚠ **Link-down drives it to `UNCALIBRATED`, unconditionally.** §8.3 measured
0.70° immediately before dropping a link and 18.80° at the same pose after
reconnecting, strap untouched and never removed. A reconnect that silently kept
the flag at `CALIBRATED` would be the single worst bug this API could ship.
There is no code path that can do it.

`hm_history_block.calibration` carries the same for the block's whole span,
including `spans_transition` when a calibration or a reconnect happened inside.

### 7.4 No persistence

There is no `hm_calibration_save()`, no `_load()` and no "reuse last session".
§8.3 settles it three ways: lost by remounting (by construction — it maps sensor
frame to arm frame), by power cycling (3.75° → 15.01° with the strap untouched)
and by a plain disconnect. Such a convenience would produce confidently wrong
data with no error, and an API that cannot express the impossible thing is worth
more than a comment.

The 64-byte `0x94` payload is carried verbatim into the wire log and deliberately
not decoded; the device acts on it itself.

### 7.5 The presence check — and it is not a score

⚠ **§8.2 measured this figure inverting:**

| Calibration | Residual at the reference pose |
|---|---|
| Correct routine — horizontal, then raised across the chest | 1.96° |
| Raise about the **wrong axis** — straight up | 6.10° |
| **No raise at all** — pose 1 marked without moving | **0.70°** |

The calibration carrying no axis information at all scored **best**, and not by
accident: the two-pose routine does two separate things — pose 0 *zeroes* the
wrist, and the raise between the poses *finds the axis* that separates flexion
from deviation. This figure tests only the zeroing, and with nothing moving
between the markers the zero is perfect by construction while the anatomical
frame is left entirely undetermined. **It measures exactly the half that cannot
fail.**

A library that selected between attempts on it would systematically prefer the
worst one available. So: **there is no API that returns a quality score and no
API that ranks two attempts.**

Its one sound use is catching *calibration never happened, or was lost* — a
failure that is otherwise silent, permanent, and reachable mid-session without
anyone doing anything wrong. The gap is an order of magnitude:

| State | Relative angle |
|---|---|
| Applied | 0.36° / 0.79° |
| Held pose shortly after | 3.73–3.80° |
| Uncalibrated, or after a power cycle | 15.01° (14.36–15.76) |
| After a plain disconnect | 18.80° |

**The modification.** The angle means nothing at an *unknown* pose, and only the
application knows when the user is at a known one. So the measurement is a
separate call, `hm_calibration_confirm_reference_pose()`, made when the UI has
returned the user to the neutral pose. The library then takes a short run of live
samples, reports the **medoid** as `HM_EV_CALIBRATION_PRESENCE` — not a median of
per-sample angles, for the reason two subsections below — and classifies:

| Angle | Result | |
|---|---|---|
| < 6° | `HM_CAL_CALIBRATED` | |
| 6–10° | Indeterminate: **state unchanged** | `HM_WARN_CALIBRATION_INDETERMINATE` |
| > 10° | `HM_CAL_UNCALIBRATED` | `HM_WARN_CALIBRATION_ABSENT` |

**The call returns before the measurement exists**, and it has to: measuring
synchronously would mean either blocking, which a sans-I/O core cannot do, or
reading a single sample, whose angle is worthless at the calibrated end (±4.45°
on a true 0.36° — see below). `HM_OK` means the run has *started*.

⚠ **The run is bounded twice, and the two bounds answer different failures**
(phase 3):

| Bound | Value | Why |
|---|---|---|
| Count | `HM_PRESENCE_MAX_SAMPLES`, 64 | What the estimator averages over. A burst fills it in 80 ms at the ≈799.2 Hz internal rate |
| Window | 2 s | A held pose is a *resting* wrist, and §6.6 puts the live rate there near 25 Hz — 64 records would take 2.6 s, so the count alone would make a UI wait on the slowest case. Two seconds yields ~50 records there |
| Floor | 8 records | Below it the run is reported as **not measured** rather than averaged |

⚠ The floor is not about the classification, which survives a single sample:
§8.2's populations are ≤3.80° and ≥14.36° against 6°/10° thresholds, and the
worst-case quantisation noise at 3.80° is ±0.42°. It is about `pose_spread_deg`.
A run of one reports a spread of exactly 0.0 — the strongest possible claim from
the weakest possible evidence, and precisely the "no evidence read as agreement"
failure §10.2's tests exist to prevent. A run that falls short emits
`HM_WARN_PRESENCE_NOT_MEASURED` carrying how many arrived, leaves the flag at
`HM_CAL_UNKNOWN` and the angle NaN, and still reaches `COMPLETE`: the transform
*was* applied; what did not happen is the check.

⚠ **Skipping the call leaves the flag at `HM_CAL_UNKNOWN`, never
`HM_CAL_CALIBRATED`.** The device applies the transform for every `a2 01`
including attempts an application would reject, so "we issued the markers" is not
evidence that calibration took, and the recording must say we did not check
rather than imply we did.

#### ⚠ Calibration and retrieval are mutually exclusive, in both directions

They meet in a normal lesson — a coach re-checking calibration shortly after a
shot, while the pull for that shot is still in flight:

- **Every `hm_calibration_*` call returns `HM_ERR_BUSY` while a bracket is
  open.** A retrieval suspends live delivery (§10.1) and the presence check is
  measured *from* live samples, so it would have nothing to measure; the pose
  markers are host→device writes, which the quiet period holds anyway. Refusing
  lets a UI retry a second later with the user standing still regardless. The
  alternative is a calibration that aborts for a reason that has nothing to do
  with calibration — the 3 s result timeout firing behind a 4.5 s pull.
- **A gather will not open a bracket while a calibration routine is active.** It
  stays queued. Holding an `a2 01` past the raise limit would abort the attempt
  for an unrelated reason, which is the same bug seen from the other side.

Neither can starve the other: a calibration is bounded by its raise limit plus
its result timeout, a retrieval by its deadline.

Skipping the presence call is allowed; the routine completes with the angle NaN.

#### The reference-pose anchor, kept rather than discarded

The device applies calibration itself and hands back quaternions in **its**
anatomical frame; §8.2 leaves the 64-byte result undecoded, correctly, since a
client does not need it. But a consumer with its own anatomical convention needs
the constant rotation between the two frames, and nothing in the API let it
solve for one.

The presence check already collects a run of samples at a pose the *application*
declared known, computes an angle, and keeps the quaternions rather than
throwing them away: `hm_calibration_presence_event` carries
`q_lower_arm[4]`, `q_palm[4]`, the record's `sample_index` and its `skew_us`,
and `hm_calibration_reference_anchor()` makes them queryable because the event
ring is drop-oldest and this is data that cannot be re-derived.

That is not a decomposition and not a frame choice. It is the raw measurement
the library already took, kept instead of discarded. Capturing it separately
from the live stream would anchor on an instant only *nearly* the one measured,
and "nearly" becomes a fixed rotation error in every subsequent reading.

**Two forms, answering two questions**:

| | What it is | What it is for |
|---|---|---|
| `q_lower_arm` / `q_palm` | The **medoid record** — one real measured pair, with its `sample_index` and a real `skew_us` | The angle and its provenance. `hm_relative_angle_deg()` of this pair reproduces `relative_angle_deg` exactly. |
| `q_lower_arm_mean` / `q_palm_mean` | The **averaged absolute pose** over the run | ⚠ A frame-reconciliation solve. |
| `pose_spread_deg[2]` | The largest angular deviation from that mean, per unit | Whether the pose was actually held. |

⚠ **The medoid is structurally blind to the error that matters here.** It is
selected on the *relative* rotation, which cancels any whole-arm movement
carrying both units together — precisely the motion that contaminates an
absolute pose. And the numbers are not close:

| Error source on an absolute rotation at the anchor | Scale |
|---|---|
| Q14 quantisation | ~0.007° per component |
| A person holding a declared pose still | **0.5–2°** |

So however centrally the medoid is chosen it still holds whatever the athlete
was doing at that instant, and averaging over the run is one to two orders
better. Both are kept because they answer different questions and neither can be
recovered once the pose has passed.

`pose_spread_deg` was not asked for and is the same argument one step further: a
consumer about to bake this anchor into every future reading needs to know
whether the athlete held the pose or drifted through it, and the samples are
gone afterwards. A mean without a spread is an estimate without evidence.

#### ⚠ The sub-degree quantisation floor — a finding, not a design choice

Implementing R14 turned up something the specification does not mention and that
changes how this figure should be read.

Quaternions arrive in Q14, so a half-LSB on each of four components moves the
dot product by ~1.2e-4, and `dθ = 2·δdot / sin(θ/2)` blows that up near zero:

| True relative angle | Single-sample noise |
|---|---|
| 0.36° | **±4.45°** |
| 0.72° | **±2.23°** |
| 2.88° | ±0.56° |
| 11.5° | ±0.14° |

So a single sample's angle is worthless at the calibrated end — it can read
exactly 0.0.

**This does not threaten the presence decision.** The populations §8.2 measured
are ≤3.80° applied and ≥14.36° absent, against thresholds of 6° and 10°, and the
worst-case noise at 3.80° is ±0.42°. The order-of-magnitude gap swallows it.

⚠ **But it is a second, independent reason never to rank two calibrations on
this number.** §8.2 shows a ranking would prefer the worst attempt available;
the arithmetic above shows that at the calibrated end it would also be ranking
quantisation noise. Two separate arguments, the same conclusion.

It also changed the estimator. A median of per-sample *angles* inherits the
noise, because `acos` rectifies it and biases the result high. The library
instead averages the relative *rotations* — where the noise is not rectified and
does average down — and returns the real measured record nearest that average.
The anchor stays a pair that came off the wire together, its skew is a real
measurement, and its angle is the one reported.

⚠ And one bug worth recording because the next person will hit it: comparing
those rotations must **normalise first**. §6.4's `|q| = 16384.7` against a Q14
divisor of 16384 means decoded quaternions are unit only to ~4e-5, and at 15° a
0.7° separation moves the dot product by 1.8e-5 — smaller than the 3.3e-5 the
magnitudes wander by. My first implementation compared them un-normalised and
therefore ranked by rounding error, picking an extreme of the run instead of its
centre. `test_presence.c` pins the corrected behaviour.
⚠ A skipped measurement leaves the flag at `HM_CAL_UNKNOWN`, never
optimistically `CALIBRATED`.

---

## 8. History retrieval

### 8.1 The cycle

```
connect → bring-up → start the 30 s 0x81 poll                    §9.1, §9.2
  → a0 01 7e — ONE stream, opened once and left open             §6.1
  → calibrate, with that stream still running                    §8.2
  → every live frame feeds the clock fit, for the whole session  §10
  → the application detects an event; asks in HOST time
  → a1 IN PLACE — do not stop                                    §7.5
  → map every returned record back to host time by its index     §10
  → repeat.  The stream never closed.  The fit never restarted.
```

§7.5 measured `a1` mid-stream returning 4,182 records at 58% coverage of an
over-wide request against 2,605 and 33% for a stopped control at the same width —
mid-stream was **not** worse.

⚠ **It does not remove the recording dead zone.**
Measured across six pulls on hardware: the sample counter stalls for the pull's
own duration — 90–99% of it across a 16× size range, 289 ms mean — so a
mid-stream pull still costs a hole, just a shorter one than a stop-and-restart.
What `a1` in place genuinely removes is three of the four costs: the index epoch
does not reset, the clock fit does not restart, and the calibration is
untouched. The fourth cost survives and has to be designed around:

- The block reports it, because nothing on the wire does — `self_recording_gap`
  in `hm_history_block`.
- The fit re-anchors on the far side of it, and two pulls never share one
  live-frame gap — §6.1.1, from §10.2's wrap budget.
- `hm_history_request_around()`'s window is sized for time and eviction margin,
  never for detail: density is set by the motion in the window (§8.7).

### 8.2 The two-phase API

```c
hm_history_request  r = hm_history_request_around(&policy, impact_us);  /* 3 s / 1.5 s */
uint64_t id;
hm_history_reserve(s, &r, &id);          /* at detection */
...
hm_history_block *b;
if (hm_history_collect(s, id, &b) == HM_OK) { use(b); hm_history_block_release(b); }
```

`hm_history_request_around()` takes the **policy**, not the session. Taking the
policy rather than the session keeps the function pure, needs no live
device, and is testable without one. `NULL` yields §7.6's 3 s / 1.5 s.

**`reserve()` validates at reserve time**. An empty or inverted window,
or a `deadline_us` at or before `window.end_us`, is `HM_ERR_INVALID_ARG` — the
window's last sample does not exist until `end_us`, so such a request is
unsatisfiable the moment it is made. Two comparisons, and they turn a silent
four-second timeout into a programming error at the call site.

`reserve` at detection lets retrieval start as soon as the window's last sample
can exist, so the ~4.5 s cost hides inside the post-impact wait a consumer was
taking anyway. `collect` never blocks and returns `HM_PENDING` until a
block exists. A block is produced for *every* terminal outcome — complete,
holed, short, timed out, cancelled — and always carries its coverage, because a
capture must record what it got even when what it got is nothing.

### 8.3 Host time in, index out — and the wrap

The index range is derived from the fit:

```
first = hm_clock_index_for_host_us(fit, window.start_us)
last  = hm_clock_index_for_host_us(fit, window.end_us)
```

⚠ Those are *unwrapped*; the wire takes `u16be` and §7.1 requires `first < last`.
So a window spanning the 82.0 s counter wrap must be issued as **two requests** —
`[first mod 65536, 65535]` and `[0, last mod 65536]` — whose results are merged
by unwrapped index. §7.4 says "unwrap internally, re-wrap when asking"; this is
what that means in practice, and it is a case a naive implementation gets wrong
exactly once every 82 seconds.

The far half waits for the near half's bracket to close, on the machinery a
refill already uses — so it obeys §6.1.1's "never two pulls inside one live-frame
gap" without needing a second rule, and the two runs merge by device index like
any other pair. Three details that are easy to get wrong:

- ⚠ **The far half is not a refill and is not charged to `max_attempts`.** It is
  the rest of the same ask; `a1` simply cannot address it in one command.
  Budgeting it would make any window straddling the wrap come back half-served
  whenever the caller passed `max_attempts = 1` — the exact silent half-window
  the split exists to prevent.
- ⚠ **A half of one index cannot be asked for at all**, because §7.1's
  `first < last` has no encoding for it. It is dropped and becomes an ordinary
  undelivered gap: at most one sample at the seam of a 4.5 s window. Widening
  the ask to make it addressable would fetch an index the caller never asked
  for, which the out-of-range filter would then throw away and warn about.
- ⚠ **A refill gap can itself straddle the wrap**, and is clamped to the near
  turn for the same reason.

A window spanning the wrap **twice** is still refused with `HM_HIST_ERROR`:
131,072 indices is 164 s against a buffer measured in seconds, so it cannot be
served whatever we do, and returning a third of it as though it were all of it
is the failure this whole section is about.

Refusals that happen before any radio traffic:

| Condition | Result |
|---|---|
| No fit, or fit worse than `request.alignment_budget_us` | `HM_HIST_REFUSED_ALIGNMENT` |
| ⚠ Window on the far side of a pull — see below | `HM_HIST_REFUSED_ALIGNMENT` |
| Window predates the current stream's start | `HM_HIST_NO_STREAM` |
| Window outside the resident-range estimate | `HM_HIST_EVICTED` |
| Legacy `0x7f` stream | `HM_HIST_NOT_ALIGNABLE` — `a1` addresses a header that does not exist (§6.3.1) |

⚠ **The second row is §6.1.1 reaching the addressing, and it was not in the
reviewed design.** The fit re-anchors at every pull, so a window older than the
current fit's first observation is separated from that line by at least one
stall — and the error is not confined to the timestamps: the conversion above
maps such a window to indices **too low by the width of the stall**, about 230
samples. The `a1` would then fetch a different span than the caller asked for,
densely and completely, and every check downstream would pass on it. So this one
fires whatever `alignment_budget_us` says: zero disables the *quality* gate, not
this. It is unreachable under the intended cycle, where reserving at detection
(C1) captures the mapping before any later pull disturbs it.

⚠ **The ask is also CLAMPED to what the device can have counted** —
`[first_index, head_index]` — and the remainder is reported as an
`HM_GAP_NOT_RECORDED` on the block rather than requested. A window mapped
through a fit can end a few samples past the head, and §7.2 measured "no data
yet" as one of seven distinct causes that all return the same `d0 03`, which
costs the whole reply rather than its tail. `swings.hmwire` contains exactly
that request: one pull asked for 200 indices that did not exist yet.

Refusing and recording the refusal is the point: a consumer must be able
to *refuse* a pull whose alignment is worse than a frame and write that into the
capture's provenance, rather than silently misaligning a wrist trace against
video.

### 8.4 The gather

| State | Meaning |
|---|---|
| `IDLE` | |
| `QUEUED` | Reserved; waiting for the window's last sample, or for an earlier request |
| `REQUESTED` | `a1` written; ⚠ waiting for `a1 02`, which is **the acceptance test** — not the absence of an error |
| `BRACKET_OPEN` | Records arriving; clock marked `HM_CLOCK_BLIND` |
| `MERGING` | `a1 01` seen; coverage evaluated |
| `REFILL` | Gaps found and budget remains: re-request the largest gaps |
| `DONE` | Block materialised |

Within `BRACKET_OPEN`, every `0x90` is a **history** record. §10.1: live and
history frames are byte-identical and the bracket is the only discriminator, so
the session must track that state to know how to timestamp what it holds. As a
defensive cross-check the session also verifies each record's index falls inside
the requested range — §10.1 measured that over 4,182 mid-stream records, every
one did — and warns rather than silently mixing if one does not.

An invalid range yields the leading `a1 01` and then `d0 03`, with **no start
marker**. `HM_MSG_ID_HISTORY_MARK` with any payload other than `01`/`02` is
rejected ("unknown magic cookie").

**Refill.** A holed result is exactly the thing worth asking for again,
and re-requesting is safe because `a1` works in place and the device never
stopped recording. The gather re-requests the largest gaps until the deadline or
`max_attempts` (default 3), merging by device index — the reliable key. A
consumer could only dedup by timestamp, which is the derived one, and a
duplicate or an inversion is a silently wrong interpolation rather than an error.

⚠ **A refill only chases a gap wider than §7.3's floor, and that qualifier is
the difference between a useful feature and a harmful one.** The buffer is
motion-adaptive: index step 8 is what a still wrist returns, measured as a hard
ceiling on the step across 25 retrievals and 17,739 steps. Those indices were
never *stored*, so re-requesting them returns nothing while costing another
~290 ms stall and another hole in the recording (§7.5). Without the qualifier
every ordinary pull burns all three attempts for no new data — replaying
`swings.hmwire` through the implemented gather, **all six pulls would have**,
and with it none does.

**Blind span.** From the `a1` write to the closing marker, live delivery
is suspended, so the fit gets no new observations for roughly the width of the
window — 4.5 s of blind extrapolation for a 4.5 s pull. The session marks the
fit `HM_CLOCK_BLIND`, emits `HM_EV_HISTORY_BLIND_SPAN` with the interval, and
records an `HM_GAP_FIT_BLIND` gap on the block.

⚠ **The uncertainty penalty is transient, by design.** Uncertainty grows across
the blind span only while the pull is the most recent thing that happened. The
extrapolation term keys on a
sample index falling outside `[first_index, last_index]`; during the pull the
blind span is beyond `last_index` and is penalised, but once live resumes
`last_index` advances past it and the gap becomes **interior**, contributing
nothing.

That is the correct behaviour, not a gap in it: a line supported by observations
on *both* sides interpolates across an interior gap perfectly well, and the
device clock has no discontinuity there — penalising it would be pessimism
without a cause. What durably records the blind span is `HM_GAP_FIT_BLIND` on the
block, which is the right mechanism and needs nothing from the clock.
`clock_penalises_a_blind_span_while_it_is_the_edge_and_not_once_it_is_interior`
now tests both halves, so the claim and the code cannot drift apart again.

### 8.4.1 When a request cannot be served, it says so immediately

Four cases, all reachable in a normal lesson, and one rule
covering them:

**Anything the consumer initiates yields `HM_HIST_CANCELLED`. Anything that
happens to us yields `HM_HIST_LINK_LOST`. Both materialise immediately.**

| Event with a reservation outstanding | Result |
|---|---|
| **Link drops.** The stream stops and the index space goes with it | `HM_HIST_LINK_LOST` with whatever arrived. **Immediately**, not at the deadline — a consumer's gather has a bounded wait, and a request that goes quiet after the link has visibly died costs a pipeline stall for no information. |
| **`hm_session_stop_stream()`** | `HM_HIST_CANCELLED`. §7.4: a restart clears the buffer and resets the index space, so the reservation is unfulfillable from that moment. |
| **`hm_session_power_off()`** | `HM_HIST_CANCELLED`. |
| **`hm_session_close()`** | `HM_HIST_CANCELLED`, consistent with `hm_history_cancel()`. |

⚠ All three consumer-initiated paths **cancel the gather first and queue their
own command second.** Without that ordering the write quiet period (§5.4) would
hold the consumer's own `83` behind a pull it no longer wants — a stop that takes
four seconds to leave the queue is a bug, and the middle case is the one a
consumer causes deliberately and will therefore hit first.

Blocks stay collectable after `close()`; `hm_session_destroy()` releases any the
caller never took.

`HM_HIST_LINK_LOST` is its own status rather than being folded into
`HM_HIST_NO_STREAM`. The whole design complains that `d0 03` means seven
different things (§4.5); reproducing that in our own status enum would be a poor
joke.

### 8.4.2 ⚠ Releasing a block is the one call that leaves the session thread

`hm_history_block_release()` runs the allocator, and a realistic integration
makes the consequence concrete: collect on the I/O thread, hand the block to
worker threads, and release it on a pool thread, possibly after the session is
gone. So the contract is stated in `history.h`:

1. It may be called from **any** thread, at any time, including after
   `hm_session_destroy()`.
2. It calls the `hm_allocator` supplied at session creation, so **that allocator
   must tolerate being called from a thread other than the session thread.**
   That is a real constraint on anyone routing it to a pool; the default
   malloc/free path already satisfies it.
3. The block owns a **copy** of the allocator, which is what lets it outlive its
   session.

Implementation note: the public `hm_history_block` is the first member of a
private allocation record that also holds the allocator copy and the arrays the
block points at, so `release()` recovers everything from the one pointer and the
public struct exposes no internals.

The natural reading of a POD struct is that it is inert memory. It is not.

### 8.5 Learning the buffer depth

§7.3's ~6000 samples / ~7.5 s was measured once, and it is not
established whether the buffer holds a fixed sample *count*, a fixed *duration*,
or something that varies with the live rate. **The three differ exactly during
§6.6's dense bursts, which is exactly when a consumer is pulling.**

So the library does not ship it as a constant. It maintains a bracket:

- `depth_lo` — the widest reach-back the device **actually served**. A lower
  bound.
- `depth_hi` — the narrowest reach-back it **demonstrably failed to serve**. An
  upper bound.

⚠ **The evidence is the old end of the delivered set, not the block's status,
and that is what makes the feature work at all.** A rule phrased as "the widest
span that came back COMPLETE" cannot work, because §7.3 makes
*every* reply holed — the buffer is motion-adaptive, so a still wrist returns an
even one-in-eight. Replaying `swings.hmwire` through the implemented gather
produced **six blocks and zero `HM_HIST_COMPLETE`**. A rule keyed on the status
would therefore never fire on a real device, and both queries would report
`HM_HISTORY_DEPTH_SEED_US` for ever — the seed figure, measured once, on somebody
else's session, dressed as a measurement of this connection. That is precisely
the failure the two queries were left refusing in order to avoid.

The discriminator is §7.3's own step-8 floor, the same one that decides what a
refill chases and when `HM_WARN_HISTORY_HOLED` fires:

| Observation | What it bounds |
|---|---|
| The oldest index that **arrived** | The buffer held at least that far back — `depth_lo` |
| The oldest index **asked for**, missing by more than the floor, on a retrieval that finished | The buffer did not reach that far — `depth_hi` |
| `d0 03` with nothing delivered | The whole span was gone — `depth_hi` |

Measured in **indices** and converted with the fit's slope, never from arrival
times: the counter advances at the internal rate whatever the wrist is doing
(§6.5), where a bulk reply's arrival times say only how fast the radio drained.
And the upper bound requires the retrieval to have *finished* — a pull we
abandoned at its deadline is missing its old end because we stopped listening,
and reading that as eviction would shrink the bracket every time a consumer
cancelled.

`hm_history_resident_range()` reports `[head − depth_lo, head]`, clamped to the
stream start so a stream that began 1.2 s before impact cannot claim to reach
back further (a different failure from eviction, needing its own
signal, or it reads as a device fault). The bracket is reset with the stream,
because §7.3 scopes the buffer to the current streaming session and §7.4
measured a restart clearing it outright.

⚠ **The status is what separates a measurement from the seed**, and shipping
that distinction *in the return value* is what let the query be implemented at
all. `HM_OK` means the width is what this connection's device actually served;
`HM_PENDING` means nothing has been *served* yet, so no residency has been
verified, and the range carries the best estimate available — the seed, narrowed
by any span the device has already **refused**. ⚠ So the width can move while
the status does not: a `d0 03` lowers the ceiling without anything being
delivered, and the range is then an upper bound rather than a claim.
`hm_history_coverage_available()` — a bool, with nowhere to put a caveat —
answers only from the `HM_OK` case, so before the first pull that delivers it is
false, which is "we cannot say".

⚠ **And `depth_lo` is bounded by the widest window anyone has asked for.** A
consumer that only ever pulls 0.5 s windows can only ever verify 0.5 s, whatever
the buffer holds. That under-claim is deliberate and asymmetric: a consumer that
skips a pull because the library said the span was resident has lost the swing,
where one that pulls a span already gone gets an `HM_HIST_EVICTED` block and
knows. The eviction *warning* below runs on a different, more generous estimate
for the mirror-image reason.

⚠ **The two halves can cross, and that is a finding rather than a fault.** A
fixed-*duration* buffer could not serve 2.2 s and then refuse 1.4 s; a
fixed-sample-*count* one can, because the depth in time shrinks by up to 8×
while the wrist is moving. When it happens the narrower claim wins and
`HM_WARN_HISTORY_DEPTH_CONFLICT` fires with the crossing. Whether the buffer
holds a fixed duration or a fixed sample count has been open since the first
capture; this is the channel it answers through.

The query is synchronous, performs no pull and does not block on the radio, so a
consumer can resolve a whole job — what window, what deadline, what to do if it
is short — on one thread. ⚠ It does return `HM_ERR_NO_FIT` for the few
tens of milliseconds after every pull, and that is §6.1.1 reaching this query
too: the fit re-anchors at each bracket close and cannot date the buffer's head
until the next live frame lands. Extrapolating through a stall of unknown width
is the thing §6.1.1 forbids.

### 8.6 Serialisation and eviction risk

⚠ A second `a1` cannot be issued until the first completes, and a pull
takes about as long as its window spans (§7.4). With a ~7.5 s buffer, a second
event a few seconds after the first is a real scenario — a golfer hitting balls
does not wait. The data *is* in the buffer, because the device never stopped
recording, but part of the second window may be gone by the time its turn comes.

So requests are queued and served in order, and for each queued request the
session estimates when its earliest sample will be evicted:

```
estimated_eviction_in_us = (window.start_us + depth_us) − now_us
```

If that is less than the estimated wait, `HM_EV_HISTORY_EVICTION_RISK` fires with
both figures. Silently returning a holed set for the second shot is the failure
mode this avoids designing around after the fact.

⚠ **The depth this uses is not the depth §8.5's query reports, and the asymmetry
is the point.** This one is a best estimate and starts from §7.3's seed,
raised by anything `depth_lo` has verified above it and capped by anything
`depth_hi` has verified below it. A warning that fires a little early costs a
consumer nothing; a *residency claim* that is a little generous costs them the
swing. Under-claim in the query, over-warn in the alarm — reading the same
number into both is how a residency estimate quietly becomes a guarantee.

⚠ **The wait is solved, not polled.** Serving is serial and in reservation
order, so the instant a queued request's own `a1` can go out is
`turn(t) = max(t + slack, fixed)`, where `slack` is the summed width of the
windows ahead of it (§7.4: a pull costs about its own span) and `fixed` is the
latest absolute instant the schedule is pinned to by a window that has not
closed yet. Both are constants of the current table, so the crossing instant is
computed rather than sampled — which is what lets this be §5.7's one deadline
row instead of a periodic wake, and what guarantees the row is never armed in
the past.

⚠ **It cancels nothing and refuses nothing**, and fires at most once per
request. The estimate rests on a depth measured only once, so acting on it would be
acting on an order of magnitude; and a warning restated on every pass is one
that stops being read, in a ring that is drop-oldest.

### 8.7 Coverage — intervals and density, never a count

⚠ §7.3: **every reply is holed, and the holes are not an error.** The buffer is
motion-adaptive — index step 8 (≈100 Hz) while the wrist is still, step 1 (the
full ≈799.2 Hz) only in fast motion, measured across 25 retrievals and 17,739
steps. So a reply spans the requested range at 16–50% coverage in a typical
session, with no error and no indication, and the density of any given part of
it is a property of **the motion that was happening there**.

⚠ **Holing is the normal shape of every reply, not a failure shape to detect,**
and two consequences follow that a "detect the failure" framing gets backwards:

- **A narrower request does not come back denser.** Density is set by motion,
  not width, so §8.4's sizing advice is about *time and eviction margin* only.
- **A consumer must not read a gap as a fault**, and must not resample onto a
  uniform 800 Hz grid without checking density — that would invent samples that
  were never taken.

Intervals-plus-density is the only way a consumer can tell "the swing is here at full rate and the pre-roll is at 100 Hz" — which is
the *correct* result — from a genuine delivery failure.

`hm_history_block` reports:

| Field | Question it answers |
|---|---|
| `requested`, `delivered[]` | Which spans arrived — as **intervals** |
| `coverage_fraction` | How much of what was **asked for** arrived |
| `density` | How closely spaced what arrived **was** — 1 / the median delivered index step; 1.0 is step 1, 0.125 is §7.3's at-rest floor |
| `largest_gap_us` | Whether impact itself survived |
| `gaps[]` | Where, and of which of the three kinds |
| `achieved_hz` | Measured over what actually arrived |

**A count cannot distinguish "dense over half the range" from "half-dense over
all of it"**, and at 33–58% coverage that distinction decides whether a metric
computed at impact exists. `coverage_distinguishes_dense_over_half_from_half_dense_over_all`
builds both cases with *identical sample counts* and asserts they are
distinguishable — largest gap 500 against 1.

⚠ **`density` IS A SPACING AND NOT A RATE, and that is why it is a field of its
own**. "Samples per unit of span" is `achieved_hz`
divided by the full internal rate — and §7.1's *even* holing makes
`coverage_fraction` equal to that on real hardware too, so the obvious definition
gives three fields carrying one number and none of them answering C4. The median
delivered step does answer it: §7.5's 58% reply is two dense runs with a
168-index hole, which *averages* to half rate and is *spaced* at step 1, and only
the second statement says whether a metric computed inside one of those runs
exists. The hole is `largest_gap_us`, which is where a hole belongs.

`hm_coverage` uses caller-provided storage. On overflow it coalesces the two
nearest intervals: the set stays a valid superset and the index total stays
exact, but the gap list becomes optimistic — so `overflowed` is surfaced rather
than swallowed.

⚠ **Which is why density is measured over the DELIVERED SAMPLES and not over
`hm_coverage`.** A reply at §7.3's floor is one interval per delivered index —
precisely the shape that exhausts the storage above — and the coalescing merges
two intervals *across their gap*, erasing the step being measured. The at-rest
regime is both the one that overflows and the one density matters most in.
`hm_sample_step_density()` (history.h) is public for a related reason: a consumer
gates on the effective rate in *its* window, typically ±125 ms around impact, and
the block's own figure is measured over the whole block, pre-roll included.

### 8.8 ⚠ The live-vs-history agreement, counted on every pull

A consumer stitching a shot's lane as `[live prefix] + [retrieved 800 Hz span] +
[live suffix]` — one ascending variable-rate trace every downstream stage reads
unchanged — is relying on history being a strict **superset** of live over the
same span: same index, **same values**.

§6.5 calls the live stream "a decimated view of the internal rate", which implies
it. The specification never asserts the *values* are identical for a given
index, and if they are not, the stitch has a discontinuity at the seam that will
look like a real wrist movement.

**The library is the only layer that ever holds both halves.** A mid-stream `a1`
(§7.5) covers a span the live stream already delivered, so the comparison is
available inside every gather it was performing anyway. Two counters on the
block make it a standing measurement:

```c
uint32_t live_overlap_samples;      /* history indices we had also seen live */
uint32_t live_overlap_mismatches;   /* ...where the raw counts differed      */
```

Implementation: a ring of `(index, digest)` for recent live samples, where the
digest is a 64-bit FNV-1a over **raw counts only** — both units' quaternion,
acceleration, gyroscope and tick fields. Raw only, because the scaled floats are
derived under a configuration that could differ between the two decodes, and the
question is whether the *device* sent the same values, not whether we scaled
them the same way. Lookup is a binary search over the ring's ascending order, so
a full-rate pull's thousands of comparisons cost nothing.

⚠ **`live_overlap_samples == 0` means NO EVIDENCE, not agreement** — either the
pull covered a span live never reached, or no digest ring was supplied. Never
read a zero mismatch count without the sample count beside it; that is the shape
of a check that has silently stopped running.

Always zero, and the stitching contract is proven in the field on hardware
nobody here has seen. Ever non-zero, and everyone finds out at once instead of
chasing a phantom movement at a seam.

⚠ **The first evidence is in: 234 indices checked across six real pulls, 0
mismatched.** `swings.hmwire` replayed through the implemented gather —
History *is* a strict superset of live over the same
span, so the stitch has no seam. The specification never asserts this, so it is
an assumption — but now one with samples behind it. One capture, one unit, which
is exactly why the counter ships rather than the conclusion.

### 8.9 No filtering, ever

The library performs no decimation, resampling, smoothing or interpolation on
history data — ~799.2 Hz samples are handed over exactly as the device sent them
Any filtering would be invisible in the artefact and unrecoverable
from it, which is the same argument as recording wire bytes.

---

## 9. Safety, privacy and logging

### 9.1 The command allowlist

⚠ §4.1: **`f0` reboots the device into firmware-update mode, and it reaches that
mode through the *ordinary data characteristic*** — the same pipe every other
command uses. §2.3's warning about avoiding the OTA service is explicitly *not
sufficient on its own*.

- **There is no `sendRaw()` and no `sendCommand(uint8_t, …)`.** The library uses
  an allowlist rather than a denylist, so the destructive command is refused by
  construction rather than by documentation.
- Every write in the library goes through **one gate**, `hm_command_emit()`,
  which refuses anything not on the list in `src/hm_command.c`.
- The list is eleven bytes: `80 81 82 83 84 85 86 a0 a1 a2 fa`. Every other
  value the device may accept — ⛔ `f0` above all — is deliberately absent.
- `hm_command_is_allowed()` and `hm_command_allowlist()` are public so a consumer
  or a test can check the claim. `tests/test_command.c` sweeps all 256 byte
  values and asserts the allowed set is exactly that list.

⚠ **Do not sweep or fuzz the device's command space**, and the contributor notes
say so. §4.1 and §12 are both explicit: the vendor-library enumeration was the
safe way to find these, it cannot prove the firmware accepts nothing else, and
undocumented values are unknown-and-possibly-destructive rather than unused.

**Fuzzing the decoder is a different activity and is encouraged.** It is already
partly automated: `codec_survives_every_prefix_of_every_fixture` runs every
truncation of every fixture under ASan.

The device accepts commands this library does not send. If any are ever added
they will be marked unverified in the API itself and gated on the protocol
version. They are not in this design.

### 9.2 Identifiers

The MAC (`0x85`) and serial (`0x86`) identify a specific unit and a specific
owner. Three mechanisms, so a consumer that turns on verbose logging
does not have to think about it:

- `hm_event_is_sensitive()` is true for exactly the events carrying them.
- `hm_event_format(..., include_identifiers=false)` redacts. A test formats
  *every* event type with identifiers populated and asserts none leaks — a
  logging path that only works for the events a developer happened to hit is one
  that fails during an incident.
- The wire log redacts `0x85`/`0x86` payloads unless
  `policy.record_identifiers` is set, and marks the chunk `HM_WIRE_REDACTED` so
  a reader knows something was removed rather than absent.

### 9.3 Unknown messages

§5.1: anything the device sends that is not in the table is **logged and
ignored**, not treated as an error. `hm_codec_decode()` returns
`HM_ERR_UNKNOWN_MESSAGE` with `consumed` set to the whole buffer — an unknown id
has no implied length, so nothing after it can be located — and the session
emits `HM_EV_UNKNOWN_MESSAGE` with the id and the first few bytes.

---

## 10. Build, test and toolchain

### 10.1 Toolchain

| | |
|---|---|
| Language | C11, no compiler extensions (`CMAKE_C_EXTENSIONS OFF`) |
| Build | CMake ≥ 3.16; presets `dev`, `san`, `cov`, `rel` |
| Dependencies | **None.** The core links `libm`. The test harness is vendored. |
| Warnings | `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Wcast-qual -Wstrict-prototypes -Wswitch-enum -Wdouble-promotion -Wvla`, `-Werror` by default |
| Sanitizers | `HM_ENABLE_ASAN`, `HM_ENABLE_UBSAN` |
| Coverage | `HM_ENABLE_COVERAGE` (gcov), CI gate 85% over `src/` |
| Install | Headers, static/shared lib, pkg-config, CMake package |
| CI | Linux gcc + clang, Linux ASan/UBSan, macOS, Windows MSVC, coverage |

`-Wswitch-enum` is deliberate: every enum switch must name every case, so adding
a state or a warning code breaks the build everywhere it needs attention.

The development machine for this draft has gcc 15.2, CMake 3.30, GNU Make and
gcov; clang, ninja and valgrind are not installed, so the build is written to
need none of them and CI covers the rest.

### 10.2 What the tests are for

The suite is not coverage theatre. Each test traces to a numbered claim, and its
name says which. 239 cases, ~51,000 assertions — the bulk of them from the
wrap-resolution test, which replays a full 238 s session, and from the `.hmwire`
round trip — all run under ASan+UBSan in CI as well as unsanitised.

⚠ **A test that agrees with the code is worse than no test**, and phase 5 found
three. So a new test that pins a *choice* is confirmed by disabling the fix and
watching it fail; eight of that phase's were. `tests/hm_test.h` aborts rather
than silently dropping a case past its cap, for the same reason.

| File | What it pins |
|---|---|
| `test_codec.c` | Byte order, field offsets, scale selectors, datagram framing, coalescing detection, the one-byte-shift norm check, legacy layout, pinned counting, truncation, unknown ids |
| `test_command.c` | ⛔ `f0` is unreachable; the allowlist is exactly §4; `a1` ordering and endianness |
| `test_unwrap.c` | §10.2's wrap resolution at full 238 s scale; a 5 s gap; inter-unit skew across independent wraps |
| `test_clock.c` | Rate fitted not assumed; envelope beats least squares; degeneracy detected; pooling; the precision/systematic split and the gate that depends on it; both correction knobs; half-open→inclusive range conversion |
| `test_coverage.c` | Holed vs short; dense-over-half vs half-dense-over-all with identical counts; overflow reported. ⚠ Reach only — spacing is not a property of an interval set |
| `test_density.c` | ⚠ C4's other half: the median delivered step, on ascending sample arrays. §7.3's step 1 and step 8 read 1.0 and 0.125; two dense runs with a hole read 1.0 where an averaged rate would read 0.58; and "not measurable" is a value rather than a zero |
| `test_quat.c` | The single-axis fixture, **and** that the angle check cannot catch a reversed order; the documented conversion between the two composition orders |
| `test_presence.c` | Classification across §8.2's three populations; the reference-pose anchor is a real record; the sub-degree quantisation floor |
| `test_overlap.c` | The digest covers raw counts and nothing derived; agreement and disagreement both detected; no storage reads as no evidence |
| `test_device.c` | UUIDs, discovery matching, MTU floor, policy defaults, identifier redaction, ABI self-description |
| `test_record.c` | The `.hmwire` round trip byte-exact including a preserved sequence gap; a refused over-long chunk; a truncated file; and the reconciliation recovering a rate, a tick ratio and a skew it was not given — plus the empty capture reporting **no evidence** on all nine claims |
| `test_capture_format.py` | ⚠ The Python writer against the C reader. Two implementations of one format, and ⛔ that the script's command set is exactly `hmwire allowlist` |
| `test_python_abi.py` | ⚠ The binding's ctypes declarations against the compiler's, both directions: every struct size, every field offset, every mirrored enumerator. §4.6.1 says what it cannot see |
| `test_python_replay.py` | ⚠ The same gather over the same real bytes, C against Python, block tables **equal** — and the density signature asserted by value, because equality alone is satisfied by two runs that reached nothing |
| `test_python_lifetime.py` | The released-block guard. ⚠ The one hazard ctypes adds, and the one no sanitizer sees |
| `purity.cmake` | The core still references no clock, thread, socket or file API. ⚠ Runs on `hackmotion`, never on `hackmotion_ffi` |

Four deserve singling out:

- **`clock_alignment_gate_survives_a_ten_minute_session`.** It asserts that a
  3 ms link meets a camera-frame budget at every point in a ten-minute lesson
  *and* that the reported total still says 1.32 s — precision and accuracy
  pinned together so neither can drift back into the other.

- **`codec_norm_tolerance_catches_a_one_byte_shift`.** §6.4 offers
  `|q| = 16384.7 ± 0.41` as the cheapest structural test that a frame has been
  located correctly, so the tolerance must be tight enough to catch a decode
  misaligned by one byte and loose enough that a correct frame can never trip it.
  It asserts both ways, at 64 counts against a measured spread of ±0.41.

  ⚠ **This entry used to name
  `codec_rejects_a_second_record_that_fails_the_norm_check`, which does not exist
  and describes the framing heuristic §3.2.1 REMOVED** — the record count comes
  from the length, and the norm check "survives as evidence that the decode is
  aligned … reported, never acted on". So the authoritative document was pointing
  a maintainer straight at the silent-corruption hazard the design exists to
  forbid.
- **`quat_angle_cannot_detect_a_reversed_composition_order`.** It asserts that
  the *obvious* validation passes under both orders, which is the reason the
  single-axis fixture has to exist. A test that only checked the correct order
  would not explain why.
- **`clock_flags_the_degenerate_estimator_when_the_rate_is_pinned_to_800`.** It
  reproduces §10's silent failure deliberately and asserts the library refuses to
  publish it quietly.
- **`reconcile_reports_no_evidence_rather_than_agreement_on_an_empty_capture`.**
  A capture with nothing in it must not report zero mismatches, zero suspect
  norms and zero disagreements as though the specification had been confirmed.
  All nine verdicts must come back `HM_CHECK_NO_EVIDENCE`. It is the shortest
  test in the suite and it is the one that decides whether a reconciliation
  report means anything.
- **`reconcile_keeps_bracketed_history_records_out_of_the_clock_fit`.** Live and
  history `0x90` frames are byte-identical and the `a1 02` … `a1 01` bracket is
  the only discriminator (§10.1). 500 bulk arrivals inside a bracket must reach
  the record census and **not** `hm_fit_observe` — exercised early against
  a real file rather than late against a state machine.

### 10.3 Golden vectors are hand-computed

`tests/fixtures/hm_fixtures.h` holds byte arrays written by hand with every
expected value computed by hand from the section named beside it. A fixture
generated by the code it tests proves only self-consistency; these catch byte
order, field offsets and scale selectors, which §1's table exists because of —
byte order is **not** uniform across this protocol.

`tests/hm_wire.h` is a builder for everything above that, at scale.

### 10.4 Testing without a radio

The session is deterministic under a synthetic clock because it never reads a
real one, so the whole of §5–§8 is testable with no hardware:

- **Live path.** Feed bytes with chosen arrival times; assert on drained writes,
  events and samples.
- **History path, including its failure shapes.** A scripted device replies to
  `a1` with a *holed* set, a *short* one, no start marker followed by `d0 03`, or
  nothing at all until the deadline. These matter because
  a deferred-source state machine's deadline and degrade paths must be exercised
  in CI on a machine with no radio — otherwise the only way to test the
  interesting branches is to unplug something at the right moment.
- **Timing.** Advance the synthetic clock to fire the keepalive at 30 s, the
  idle shutdown at 5 min, the 9 s power-off linger and the calibration bound —
  in microseconds of wall time.

These land with the session implementation (§11).

### 10.5 Bring-up as a replay fixture

§9.1's exact vendor sequence is reproducible and is kept as a replay fixture,
because reproducing it is a useful bring-up test even though the device does not
demand it.

---

## 11. Implementation status

**Implemented and tested** — the layers where the specification's measured facts
become code, and where a bug is a silently wrong swing rather than a crash:

| Module | Exec. lines | Line coverage |
|---|---|---|
| `hm_clock.c` — the fit, snapshot, error split, index ranges | 349 | 94.3% |
| `hm_overlap.c` — live-vs-history digest and ring | 67 | **100%** |
| `hm_presence.c` — presence classification, the medoid anchor and the averaged pose | 95 | 85.3% |
| `hm_codec.c` — framing, decode, scaling, pinned detection | 212 | 93.9% |
| `hm_coverage.c` — interval algebra (reach) | 170 | 88.2% |
| `hm_density.c` — the median delivered step (completeness) | 25 | — |
| `hm_unwrap.c` — index, ticks, skew | 78 | 98.7% |
| `hm_quat.c` — convention, presence angle, pinned counts | 84 | 92.9% |
| `hm_command.c` — encoders + allowlist gate | 55 | **100%** |
| `hm_config.c`, `hm_device.c`, `hm_defaults.c`, `hm_version.c` | 170 | 93.5% |
| `hm_session.c` — the link, stream, calibration and history machines, deadlines, live path, wire log | 1736 | 91.5% |
| `hm_status.c` — enum names and event formatting | 229 | 70.7% |
| **Total** | **3249** | **90.8%** |

`hm_status.c` is the outlier and is meant to be: it is almost entirely string
tables, and the uncovered lines are `default:` arms for enum values that cannot
exist. The algorithmic modules are held to a higher bar because a gap there is a
silently wrong swing rather than a crash.

**The capture harness:**

| Module | What it is |
|---|---|
| `record/hm_record.c` | The `.hmwire` container: writer, reader, byte-exact round trip (§5.6) |
| `record/hm_reconcile.c` | A capture replayed against this specification, claim by claim |
| `record/hm_report.c` | The report — ⚠ never an estimate without the count behind it |
| `tools/hm_wire_tool.c` | `hmwire info` / `dump` / `verify` / `reconcile` / `allowlist` / `abi` |
| `tools/hm_capture.py` | The radio front-end. ⛔ Takes its allowlist from `hmwire allowlist`; refuses to run without it |
| `tools/hm_gather_replay.c` | ⚠ Phase 4. A capture replayed through the *implemented* gather, where `hmwire reconcile` replays it against the *specification*. `--json PATH` writes the per-block table for the binding to be compared against, leaving stdout byte-identical |
| `tests/test_record.c` | 28 cases, 15,351 assertions |
| `tests/test_capture_format.py` | ⚠ The Python writer against the C reader — two implementations of one format, pinned |

`hackmotion_record` is a separate target and is deliberately **outside the
purity gate**: it opens files, which is exactly what the core must never do.

**The session core:**

| Module | What it is |
|---|---|
| `src/hm_session.c` | The link machine (§5.1-5.4), the stream machine (§5.5), the deadline table (§5.7), the live path and the clock fit (§6), the wire log (§5.6) |

**Calibration:**

| | What it is |
|---|---|
| The phase machine of §7.2 | Both markers, the shared `a2 01` acknowledgement, `0x94`, the raise limit and the device bound, every transition evented |
| The presence run of §7.5 | A bounded run of live samples → `hm_presence_select_reference()` → `HM_EV_CALIBRATION_PRESENCE`, the three bands, the kept anchor |
| The interlocks | `HM_ERR_BUSY` on every call while a bracket is open, with `abort()` exempt |
| `tests/test_session.c` | 89 cases, 26,203 assertions, driven entirely through a fake transport on a synthetic clock |

⚠ **Reaching `COMPLETE` is not being calibrated.** The library will say
`HM_CAL_CALIBRATED` only where a presence measurement passed; a skipped check, a
declined one and a starved one all leave `HM_CAL_UNKNOWN`, because `0x94` is not
a verdict and §7.3 records that a sample wrongly labelled `CALIBRATED` is
permanent and invisible.

**History retrieval:**

| | What it is |
|---|---|
| The gather of §8.4 | `reserve` / `collect` / `cancel`, the queued table, the `a1` and its `u16be` re-wrap, the bracket, the stateless history unwrap, the coverage accounting, the refill, the merge, the block |
| Every failure shape of §8.4.1 | holed, short, the leading `a1 01` + `d0 03`, timeout, cancel, link-lost, `REFUSED_ALIGNMENT`, `NOT_ALIGNABLE`, `EVICTED` — a block for every terminal outcome, always carrying its coverage |
| ⚠ §6.1.1, discharged | The fit re-anchors at every bracket close; a request is dated by the snapshot taken when its **window** closed; no second `a1` goes out before a live frame; a window on the far side of a pull is refused rather than addressed wrong |
| ⚠ §7.3, discharged | A refill chases only gaps wider than the measured step-8 floor, and `HM_WARN_HISTORY_HOLED` only fires above it — otherwise every ordinary pull burns its whole attempt budget re-requesting samples that were never stored |
| §8.5's depth bracket | Learned from the **old end of the delivered set**, never from the block's status — which on real hardware is `HOLED` every time. `hm_history_resident_range()` says `HM_OK` once something has been served and `HM_PENDING` while the width is still only an estimate; `hm_history_coverage_available()` answers only from the served case |
| §8.6's eviction estimate | §5.7's third history row. The serial schedule is **solved**, so the warning is one deadline rather than a poll, and it fires once per request, cancels nothing, and runs on a deliberately more generous depth than the query reports |
| §8.3's wrap split | A window across the 82.0 s counter wrap goes out as two `a1`s and comes back as one block, ascending across the seam. The far half is **not** charged to `max_attempts`, and it waits for a live frame like every other second pull |
| The R8 interlock, both halves | A gather will not open a bracket while a calibration routine is running, and every `hm_calibration_*` call refuses inside one |
| `tools/hm_gather_replay.c` | ⚠ The gather driven by a real device's own bytes — the only validation retrieval gets |

⚠ **Validated against hardware, because it cannot be validated at a desk.**
§7.3's buffer is motion-adaptive, so a synthetic full-rate reply proves only
that the gather handles one. `swings.hmwire` — five golf swings, six mid-stream
pulls — was replayed through a real session with the gather live: six blocks,
657–728 Hz over the swings and 99.9 Hz over the still one, **density 1.000 over
the swings against 0.125 over the still wrist**, no step above the floor in 1,736
records, and 234 live-vs-history indices checked with 0 mismatches.

```sh
ctest --test-dir build/dev -R 'capture|gather_replay'
```

✅ **And it now runs on every `ctest`.** The three captures are tracked in
`tests/fixtures/` — 672 K of write-once bytes, which is what §10.5 meant by
replay fixtures — so the one test that exercises retrieval against real
hardware is no longer something a session has to remember to do by hand.
`.gitignore` still excludes `*.hmwire` everywhere else, so an ad-hoc recording
cannot be committed by accident, and each fixture was checked redacted at the
byte level before it was added.

⚠ **The exit codes still separate "passed" from "nothing to pass", and that is
the load-bearing part.** **0** every check passed, **1** a check failed, **2**
usage or I/O error, **3** nothing was checked. A capture with no retrieval in it
— `smoke.hmwire` — reports `NO EVIDENCE` on every line rather than six green
checks over zero samples, and `gather_replay_reports_no_evidence_when_nothing_
was_pulled` pins that, because it is the half that rots first.

✅ **`session1.hmwire` now replays too, and it is the other regime.**
`swings.hmwire`'s pulls are over swings and come back at 657–728 Hz; this one is
a calibration choreography, so its single pull is over a wrist held still and
comes back at §7.3's 100 Hz floor — 495 of its 501 steps are exactly 8. It also
sits at 173 s, **past two index wraps**, which is the only real-hardware
evidence this project has that "unwrap internally, re-wrap when asking" (§7.4)
holds above 131,072. What kept it out before was its own harness: its first
frame arrived 4.08 s after `a0`, against §6.1's measured 50–80 ms and the
library's 3 s bound, so the session correctly gave up on that stream. The
replay tool relaxes *that one bound* and says so at the call site — the delay is
a property of the recorder, not of the device, and the bound is tested where it
belongs, in `tests/test_session.c`.

⚠ `hmwire reconcile` is deliberately **not** wired into `ctest` alongside it:
three of its claims legitimately DIFFER on this hardware, and that is the
finding rather than a regression. A gate over it would either lie or need
editing every time the device teaches us something.

**The Python binding:**

| Module | What it is |
|---|---|
| `hackmotion_ffi` (CMake) | ⚠ ONE shared object holding the core **and** the record module. `hackmotion_record` reaches internal headers, so a shared core hides what it needs and the ordinary target is disabled for `BUILD_SHARED_LIBS`; compiling both source lists into one object sidesteps that, since hidden visibility only hides symbols from *outside*. ⚠ The purity gate stays on `hackmotion` and must never be pointed here — this object opens files by construction |
| `tools/hm_abi_table.c` | Every public struct's size and field offsets, and every mirrored enum's values, straight from the compiler. Self-checks that its rows tile each struct. `hmwire abi` |
| `python/hackmotion/_types.py` | The structs and enums in ctypes, hand-written so the load-bearing warnings travel with them, and pinned field-for-field against the table above |
| `python/hackmotion/_library.py` | Locating and loading the object, every prototype declared, `hm_abi_check()` at import. ⚠ An undeclared prototype is not callable: ctypes would default it to `int` and truncate a pointer |
| `python/hackmotion/session.py` | `Session`, `Event`, `Samples`, `HistoryBlock`. ⚠ Capacities only — the library owns every buffer, because a Python object outliving a C session is a use-after-free waiting to be written |
| `python/hackmotion/record.py` | `Recorder` / `Replay` over `record/hm_record.c` — ⚠ the C implementation, not a second one |
| `tools/hm_replay_py.py` | `hm_gather_replay.c` through the binding, line for line |
| `tests/test_python_abi.py` | 331 checks: every struct size, every field offset, every enumerator, both directions |
| `tests/test_python_replay.py` | ⚠ The same gather driven twice over the same real bytes, C and Python, block table equal — **plus the density signature by value**, because equality alone would be satisfied by two runs that reached nothing |
| `tests/test_python_lifetime.py` | The block-release guard, which is the one hazard ctypes adds |

⚠ **The parity test is the one that matters, and it is not a table of numbers.**
It asserts that two implementations agree over `swings.hmwire`, `session1.hmwire`
and `smoke.hmwire` — six blocks, one block and none — including the exit codes,
so the empty case still reports NOTHING WAS CHECKED from both sides. The C tool
does not use the binding, so a disagreement is always the binding's fault and is
never resolved by editing the test. §4.6.1 has what these checks cannot see.

⚠ **Two real transcription errors were caught by the enum table on the day it was
written**: `hm_gap_kind`'s first two members were the wrong way round, and four
of eleven history statuses were on the wrong numbers. Both read as perfectly
ordinary Python. That is why the enum dump exists at all — the library's own
`hm_*_name()` functions could only have pinned about half of them.

**The radio:**

| Module | What it is |
|---|---|
| `python/hackmotion/device.py` | What a scanner needs, published as data. ⚠ The four UUIDs are read out of the loaded object with `in_dll` and the name match is `hm_looks_like_hackmotion()` — **so a transport built on this holds no second copy of the device's identity**, and there is nothing here that can drift from `device.h` |
| `python/hackmotion/bleak_transport.py` | The reference transport. ⚠ OPTIONAL and outside the core: `import hackmotion` does not import it and `bleak` is not a dependency of the binding. Five arrows and nothing else — connect, notify, pump, write, classify |
| `tools/hm_bench.py` | The exemplar. Connect, bring up, stream, optionally calibrate, retrieve each swing **while it is still in the buffer**, record through `hm_recorder_*`, and report the three coverage numbers each with the raw number behind it |
| `tests/test_python_transport.py` | ⚠ The transport with the radio replaced and nothing else, over **both** fixtures. 52 checks. Needs no `bleak` and no adapter |

⚠ **Four properties the loopback pins, and two were confirmed by breaking them.**
One call one notification — never coalesced, never split, never reordered;
⛔ every byte written is on the allowlist and a non-allowlisted one is *refused*;
**no write queue of its own**, so nothing goes out while a history bracket is
open; and a whole retrieval survives the drain path at the density signature the
C tool produces. Coalescing two notifications reports "1154 calls for 2308
notifications"; giving the transport its own queue reports "6 writes inside a
bracket".

⚠ **The notification callback is SYNCHRONOUS, and that is load bearing.** bleak
schedules an async callback as a task; each task runs to its first `await` and
yields, so two notifications that await a write can reach `on_bytes()` out of
order. §3 gives the protocol nothing to resynchronise on, so that corruption is
silent. A sync callback is called in arrival order by every backend and stamps
before it returns.

⚠ **The link-down classification is never UNKNOWN, and it carries its evidence.**
Three arms are facts the transport holds — we asked, a write failed, the adapter
went. The fourth is a heuristic and is labelled as one: a link that outlived
`policy.live_gap_alarm_us` of device silence was alive through that silence, so
the far end chose to go (REMOTE_CLOSED); below that the two are indistinguishable
and SUPERVISION_TIMEOUT claims less. It matters at exactly one place — §5.4 turns
REMOTE_CLOSED after 10 s of quiet into NEEDS_BUTTON_PRESS.

✅ **It has met the device.** Two recordings off one sensor, one through the
library and one through the frozen harness, agree on
**ten `(message id, length)` shapes with an empty set difference both ways**, on the same eight command bytes, and on the same three
claims that DIFFER from the specification. First-frame latency 80.9 ms, inside
§6.1's band. A run of **339 consecutive adjacent samples — 424 ms of full-rate
replay**, a new best.

**Still designed, headers complete, not implemented:**

| | Why it is deferred |
|---|---|
| Reference BlueZ transport | ⚠ **Deferred, not cancelled, and its justification has shrunk.** `bleak` covers Linux, macOS and Windows and is already proven against this sensor, so the Python transport proves the sans-I/O shape composes on three platforms where a BlueZ one proves it on one. What remains is "a C consumer wants a copy-paste example", and it can be written when somebody asks. ⚠ It is also the only way to answer "what does an event loop cost the clock fit" — see §6.4 of the findings, where that comparison is recorded as NO EVIDENCE. |
| Wheels / PyPI | The shared object is found by environment variable or build directory. Bundling one per platform is its own work and belongs *after* the binding has met a sensor. |
| `hackmotion.hpp` — optional C++ RAII wrapper | Adds no ABI; follows the core. |

---

## Appendix A — API index

| Header | Contents |
|---|---|
| `hackmotion/hackmotion.h` | Umbrella |
| `hackmotion/types.h` | `hm_status`, `hm_time_us`, `hm_uuid`, ranges, `hm_allocator` |
| `hackmotion/version.h` | Version, `hm_abi_sizes`, `hm_abi_check()` |
| `hackmotion/device.h` | Advertised name, GATT UUIDs, MTU floor, device limits, `hm_device_info` |
| `hackmotion/config.h` | `hm_stream_config` and its wire-format consequences |
| `hackmotion/sample.h` | `hm_sample`, `hm_unit_sample`, scales, flags, pinned counts |
| `hackmotion/quat.h` | The composition order, relative rotation, presence angle |
| `hackmotion/clock.h` | `hm_clock_snapshot`, mapping, uncertainty, rate constants |
| `hackmotion/coverage.h` | Interval algebra |
| `hackmotion/event.h` | `hm_event` and its payloads |
| `hackmotion/history.h` | Request, block, statuses, gaps |
| `hackmotion/session.h` | The session, the threading contract, calibration, the allowlist |

Internal, reachable from tests: `src/hm_codec.h`, `src/hm_command.h`,
`src/hm_unwrap.h`, `src/hm_fit.h`.

