<!-- SPDX-License-Identifier: LGPL-2.1-or-later -->
<!-- Copyright (C) 2026 Mark Liversedge -->
# HackMotion Sensor — BLE Protocol Specification

Implementation-facing description of the protocol spoken by a **HackMotion wG3** (hardware
4.1, firmware 4.8, protocol 4.0). Written to be implemented from directly.

**This document states what the protocol *is*.** It carries no history, no dates and no
account of how anything was found. It is complete in itself — everything needed to write a
client is here, and nothing in it defers to a document you do not have.

**How it was established.** By observing traffic between a HackMotion sensor and the vendor's
own Android application, decoding it offline against known motion, and then reproducing each
behaviour independently from a Linux laptop with the application closed. Where the vendor's
own binaries named something that had been measured, their name is used. Measurements are
quoted with the conditions that produced them wherever the conditions matter.

**Everything in this document has been observed on the device under test**, and in most cases
reproduced independently from a laptop with the vendor app closed. Behaviour that was inferred
rather than exercised — and every command this client does not send — is deliberately not
documented here.

⛔ **One hazard is stated even though it has never been exercised, and deliberately so:** `f0`
reboots the device into firmware-update mode through the ordinary data characteristic (§4.1). It
has never been sent and must not be. `fa`, power off, **has** been sent and behaves as named
(§9.3).

**One device.** Every measurement here comes from a single unit. Anything that could plausibly
vary between units — the crystal rate above all (§6.5, §10) — is flagged where it arises, and a
client should measure such values per device rather than inherit the constants quoted here.

---

## 0. What the device provides, and what it does not

Read this before hunting for a field that is not there.

**The device streams per-unit orientation, and nothing higher-level.** Each record carries two
independent 22-byte blocks — one per physical unit — holding a quaternion, linear acceleration,
angular rate and a tick counter (§6.3). After calibration those quaternions are in the wrist's
anatomical frame (§8), but they are still *per unit*.

**Everything anatomical is the client's to compute.** The wrist metrics — flexion/extension,
radial/ulnar deviation, rotation — are properties of the *relative* rotation between the two
units, `q_palm ⊗ q_arm*` (§6.7). The device never sends them, and the vendor computes them in
its application, not on the sensor. Deviation additionally depends on which hand the sensor is
worn on, which the device does not know and never asks.

**Swing detection and phase segmentation are also the client's.** Address, top and impact are
not flagged anywhere in the protocol. The vendor detects them in application code.

**There is no field that could carry a derived quantity.** Every byte of a record is accounted
for: two header counter bytes and two per-unit blocks whose contents are fully identified
(§6.3, §6.4). If a client is looking for a wrist angle on the wire, it is looking for something
that does not exist.

**There is no background recording.** The device samples only while a stream is open (§6.5), so
history (§7) can only return what was captured during an open stream. A client that wants
full-rate data must hold a stream open for the whole session.

**Nor does it record while it replays.** The sample counter stops for the duration of every
history retrieval, including one issued without stopping the stream, so each pull leaves a hole
the width of the pull with nothing on the wire to mark it (§7.5). The device does one or the
other, never both.

**And the buffer is not a flat 800 Hz recorder.** Even with a stream open it stores at ~100 Hz
while the wrist is still, rising to the full ≈799.2 Hz only in fast motion (§7.3). Full-rate
data exists for a swing; it does not exist for a sensor lying on a desk, and nothing on the wire
distinguishes the two cases.

So a client's job is: manage the link and the session, drive calibration, decode records into
two timestamped quaternions, and maintain the device→host clock mapping (§10). Interpretation
starts above that line.

---

## 1. Conventions

`u8`, `u16`, `i16` are unsigned/signed integers. `be` suffix means big-endian.
Byte sequences are written in hex without separators: `a0 01 7e`.

**⚠ Byte order is not uniform across the protocol.** There is no single rule; use this table.

| Where | Order |
|---|---|
| Sensor stream — quaternion, accelerometer, gyroscope, tick counter, record header | **big-endian** |
| `a1` history command arguments | **big-endian** |
| `0x80` version reply | sequence of independent `u8` pairs, no multi-byte integers |
| `0x81` status bytes 1–2 | `u16be` |
| `0x84` sensor map | independent `u8` location codes — and its **length** is the sensor count (§5.4) |
| `0x85`, `0x86` replies | ASCII text, not integers |

---

## 2. Transport

### 2.1 Discovery

The sensor advertises the local name **`HackMotion wG3`** for only a few seconds following a
physical button press, then stops. A scanner must already be running when the button is
pressed.

**Discovery is a race.** A conventional scan-then-pick-from-a-list flow assumes the device is
discoverable whenever you go looking; here it is not. Arm the scanner *first*, then prompt the
user to press the button. A scan that starts after the prompt will routinely find nothing, and
that failure reaches the user as "the device doesn't work".

- **Allow a generous scan window** — 90 s is comfortable, and costs nothing if the device
  appears in the first second.
- **⚠ If the sensor has been asleep, the first press only wakes it.** It does not advertise on
  that press. Tell the user to press, pause, then press again. Without this a client looks
  broken on exactly the first-run path where confidence matters most.
- The device **vibrates** when a connection is established (§9.5), which is the user's
  confirmation that the race was won.

The device powers itself down after **5 minutes idle**, and this timer runs whether or not a
central is connected (§9.2).

### 2.2 Connection

**No pairing, no bonding, no link-layer security.** A client needs no LTK, no IRK and no bond
slot. The device accepts **one** connection; the vendor app will win the race if it is running.

### 2.3 GATT

| Service | UUID |
|---|---|
| Generic Access | `0x1800` |
| Generic Attribute | `0x1801` |
| Device Information | `0x180A` |
| **ISSC Transparent UART** | `49535343-fe7d-4ae5-8fa9-9fafd205e455` |
| Microchip OTA / DFU | `1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0` |

The vendor services are stock Microchip/ISSC module services. **The entire HackMotion protocol
is a byte stream inside the Transparent UART pipe** — it is not expressed as a characteristic
per quantity. There is no Battery Service; battery comes from command `0x81`.

**The data characteristic** carries both directions:

| | |
|---|---|
| UUID | `413c3893-b7e8-4231-9673-7af7aed06ddc` |
| Declaration handle | `0x0017` |
| **Value handle** | **`0x0018`** |
| CCCD handle | `0x0019` |
| Properties | `write-without-response`, `write`, `notify` |

The service also exposes the stock ISSC pipe characteristic
`49535343-1e4d-4bd9-ba61-23c647249616` (value handle `0x0015`). **It is inert** — commands
written there receive no reply.

**⚠ Do not write to the OTA service.** `f7bf3564-…` (`0x001c`) and `984227f3-…` (`0x001e`) are
the firmware update path and can brick the device. **Avoiding this service is not sufficient on
its own** — command `f0` on the ordinary data characteristic reboots the device into
firmware-update mode. See §4.

### 2.4 MTU

The calibration result is 65 bytes (§8.2) and stream notifications reach 93, both beyond the
20-byte payload of the default ATT MTU. Negotiate an MTU of at least 96. BlueZ does this
automatically.

---

## 3. Framing

```
host   -> device:   <command u8> <arguments...>
device -> host:     <message-id u8> <payload...>
```

There is **no length field, no sequence number and no checksum**. Payload length is implied by
the message id and by the notification boundary. Most commands are a single byte; the `0xa0`,
`0xa1` and `0xa2` families take arguments. A reply's first byte generally echoes the command
that provoked it.

**⚠ For three messages the length itself carries meaning**, so it must be read rather than
assumed: `0x84`'s length *is* the sensor count (§5.4), `0x90`'s decides how many records are
present (§6.3), and `0x94`'s selects between two different payload formats (§8.2).

With no checksum, **a structural check is the only validation available** — the quaternion norm
(§6.4) and the requirement that a stream payload divide exactly into whole records.

⚠ **A short frame will not announce itself.** A truncated block is dropped *silently* — the
vendor application's own logs do not report one, so they cannot be used as a reference for
whether a stream is arriving intact. **A client gets no truncation detection it does not write
itself**, which is what the two structural checks above are for.

---

## 4. Commands (host → device)

| Command | Reply | Meaning |
|---|---|---|
| `80` | `80` + 7 | Versions and product id — §5.2 |
| `81` | `81` + 3 | Battery and status — §5.3 |
| `82` | `82 01` | **Legacy start streaming.** No config argument; produces `0x7f` frames — §6.3.1. The app only sends it to protocol < 3 hardware, but this device accepts it |
| `83` | `83 01` | **Stop streaming** — §6.1 |
| `84` | `84` + *n* | Sensor map, **one byte per sensor** — §5.4. Two on this device |
| `85` | `85` + 12 | BLE MAC address, ASCII hex, no separators |
| `86` | `86` + 9 | Serial number, ASCII |
| `a0 01 <config>` | `a0 01` | **Start streaming** — §6.1 |
| `a1 <first u16be> <last u16be>` | markers + frames | **Retrieve history** — §7 |
| `a2 00` | `a2 01` | Calibration pose marker 0 — §8 |
| `a2 01` | `a2 01`, then `94` + 64 | Calibration pose marker 1 — §8 |
| `fa` | — | **Power the sensor off** — no reply, link drops ~9 s later — §9.3 |

`0x90` is a *notification* id with no command counterpart.

`0x83` may be sent whether or not a stream is running, and stopping is idempotent in practice.
Start, stop and restart all work repeatedly within one connection.

### 4.1 Commands this client never sends

The table above is what a client needs and what has been **observed on the wire**. It is not the
whole command set: the device accepts further commands that this client neither sends nor
implements, and which are deliberately not documented here.

#### ⛔ `f0`, and why not to go looking

§2.3 warns against writing to the OTA service. **That warning is not sufficient on its own**,
because `f0` reaches firmware-update mode through the *ordinary data characteristic* — the pipe
every other command in this document uses.

Never send it, and **do not sweep or fuzz the command space**. Send only the commands in the
table above: probing for the rest puts a device into a state from which a bad write bricks it.

---

## 5. Notifications (device → host)

### 5.1 The set a client handles

The device's message ids fall in `0x7f`–`0xfb`; the vast majority of that range is unused. The
table below is every notification a client needs to handle, and names are the vendor's own where
their binaries supplied one. **Anything not listed here should be logged and ignored rather than
treated as an error** — the range is sparse but this table is scoped to what a client uses, not
to every byte the device could conceivably emit.

| Id | Meaning |
|---|---|
| `0x7f` | **Stream frame, legacy layout** — §6.3.1 |
| `0x80` | Versions — §5.2 |
| `0x81` | Battery / status — §5.3 |
| `0x82` | "Stream started" — reply to the legacy `82` |
| `0x83` | "Stream stopped" |
| `0x84` | Sensor map — §5.4 |
| `0x85` | MAC address |
| `0x86` | Serial number |
| `0x90` | **Sensor stream frame** — §6.3 |
| `0x94` | Calibration result — §8.2. ⚠ **Two forms**, told apart by the notification's length |
| `0xa1` | History start/end marker — §7.2 |
| `0xa2` | Calibration pose set result |
| `0xd0` | **Device error**, one payload byte — only `03` has ever been seen (§7.2) |
| ~~`0xfa`~~ | Named by the vendor as "device turned off", but **the device does not send it** — see §9.3 |
| `0xfb` | **Sensor button pressed**, payload `01` — §9.4 |

**`0xa0` is not in the table.** The device acknowledges a start command with `a0 01`, and the
app ignores it. A client must key stream startup off the arrival of the first `0x90` frame,
not off an acknowledgement.

### 5.2 `0x80` — versions

Seven bytes, three `u8` pairs and a scalar.

| Byte | Field | Observed |
|---|---|---|
| 0–1 | Hardware version, major.minor | `04 01` → 4.1 |
| 2–3 | Protocol version, major.minor | `04 00` → 4.0 |
| 4–5 | Firmware version, major.minor | `04 08` → 4.8 |
| 6 | Product id | `14` → 20 |

The vendor calls firmware "embedded version". **Protocol version drives feature gating** —
see §6.2 and §7.1.

### 5.3 `0x81` — battery and status

| Byte | Field |
|---|---|
| 0 | Battery percentage, `0x64` = 100% |
| 1–2 | `u16be`, observed 2226–2235, drifting slowly. **Not millivolts** — undecoded |

### 5.4 `0x84` — sensor map

**The reply is one byte per sensor, and every byte is a location code.**

| Byte | Field |
|---|---|
| 0 | Location code of the first sensor, observed `02` |
| 1 | Location code of the second sensor, observed `01` |

**⚠ The sensor count is the reply's LENGTH, not a byte in it.** `84 02 01` means two sensors
because it carries two payload bytes. The device under test replies with a leading `02` and has
two sensors, so the two readings coincide here — **and that coincidence is the trap.** A client
that reads byte 0 as a count is correct on this device and wrong on any device whose first
sensor sits somewhere else. Take the count from the length.

The count matters because it sizes a record: §6.3's record is a header followed by **one block
per sensor**, so getting it wrong misplaces every field after the first block.

**The location codes name a point on the limb**, as a distance from the joint:

| Code | Offset | What sits there |
|---|---|---|
| `0` | 0.00 m | at the joint |
| `1` | **0.10 m** | wrist-to-knuckles on an adult — **the palm unit** |
| `2` | **0.26 m** | elbow-to-wrist on an adult — **the lower-arm unit** |

So `02 01` reads as *lower arm first, palm second*, which is exactly the block order of a record
(§6.3) — and it is one of the three independent routes that settle that order. No reading puts a
0.26 m segment on a hand.

---

## 6. The sensor stream

### 6.1 Starting and stopping

```
-> a0 01 7e        <- a0 01        first 0x90 frame within ~50–80 ms
-> 83              <- 83 01        frames cease immediately
```

The stream **does not survive a disconnect**: a fresh connection always begins idle, whether or
not the previous session was stopped cleanly.

### 6.2 The configuration byte

The third byte of the start command is a feature bitmask. The vendor app sends **`0x7e`** on
this hardware.

| Bit | Mask | Meaning | Set in `0x7e` |
|---|---|---|---|
| 0 | `01` | Alternate hardware path, protocol ≥ 7 | no |
| 1–2 | `06` | **No magnetometer** — fusion runs 6-DOF | yes |
| 3 | `08` | Undecoded. Gated on a version check in the app; never sent clear | yes |
| 4 | `10` | Undecoded; never sent clear | yes |
| 5 | `20` | **Timestamps** — adds the per-block tick counter | yes |
| 6 | `40` | **Extended gyro range** — halves the gyro scale | yes |

**⚠ Two of these change the wire format, not just content:**

- **Bit 5 changes the block size.** Set, blocks are 22 bytes and records 46. Clear, blocks are
  20 bytes and records 42, with no tick counter — notification sizes 43 and 85 instead of 47
  and 93. Measured with `a0 01 5e`.
- **Bit 6 changes the gyro scale**, 8 LSB/°/s instead of 16 — §6.4.

A client that sends a different configuration must decode accordingly. Bits 3 and 4 are
undecoded and may do the same.

**Consequence of bit 1–2:** the magnetometer is disabled, so the fusion is 6-DOF. Absolute
heading has no reference and drifts — measured at rest, after the fusion settles, at
**0.11–0.16°/min per unit**, mostly about vertical as a heading error should be.

**The legacy `82` path does not change this.** Its firmware default was the obvious candidate
for a 9-DOF mode, but a stationary hold under `82` drifts at the same rate (0.107°/min settled,
against 0.160 under `7e`). No reachable configuration makes this device fuse with its magnetometer.

**The relative angle is far more stable than either unit.** Under `7e` it stayed within a
**0.58° range over 5 minutes** while the units themselves drifted 1.95° and 1.13°. Under `82`
it ranged **2.75° over 7 minutes**. So the drift is substantially common-mode under the
configuration the app uses — the opposite of what was predicted — but the cancellation is
configuration-dependent and should not be assumed outside `7e`.

### 6.3 Frame and record layout

```
notification := 0x90 , record+
record       := header[2] , block × sensor_count               // 46 bytes at 2 sensors
block        := quaternion[8] , accel[6] , gyro[6] , ticks[2]  // 22 bytes
quaternion   := i16be w , i16be x , i16be y , i16be z
accel        := i16be x , i16be y , i16be z
gyro         := i16be x , i16be y , i16be z
ticks        := u16be
header       := u16be sample counter
```

**Every multi-byte field in a block is big-endian, without exception** — quaternion,
accelerometer, gyroscope, tick counter and the record header alike. ⚠ That uniformity is a
property of the *stream* and not of the protocol; §1's table is the whole rule.

`sensor_count` comes from the `0x84` reply and is **2** on this hardware (§5.4). Records repeat
until the payload is consumed, and the observed notification sizes follow from that:

```
47 bytes = 1 + (2 + 22 + 22)        one record
93 bytes = 1 + (2 + 22 + 22) × 2    two records
```

**⚠ One or two records is what fits a 96-byte MTU, not a limit of the encoding.** Nothing in the
format bounds records per notification. **Loop until the payload is consumed**; do not
special-case the two observed sizes, and do not size a buffer on the assumption that they are
the only ones. A payload that does not divide exactly into whole records is malformed (§6.4).

#### ⚠ The header is read once per record and applies to every block in it

The sample index belongs to the *record*, not to a block: one header, then every block. **This
is the structural basis for treating the two blocks as simultaneous samples** rather than merely
adjacent ones — they are the same sample of the same instant, taken by two units.

⚠ It does not make their **tick counters** equal, and it does not settle §10.3's 0.92 ms
inter-unit offset. The index says the blocks belong to one sample; the ticks are two independent
MCU timers and disagree by an amount whose physical meaning is unresolved.

#### Which block is which

**The first block is the lower-arm unit; the second is the palm unit.** The two are one BLE
peripheral joined by a cable, and the assignment is fixed by that wiring. A client that swaps
them produces a plausible but mirrored wrist angle. **Three independent routes agree**:

| Route | What it shows |
|---|---|
| **The sensor map** (§5.4) | `02 01` assigns block 0 a 0.26 m lever arm and block 1 a 0.10 m one — elbow-to-wrist and wrist-to-knuckles. No reading puts a 0.26 m segment on a hand |
| **The calibration result** (§8.2) | Its four pose quaternions are named per unit. Matched against both blocks with no prior assumption, the palm poses fit block 1 by margins of 7.0° and 8.2°, the arm poses fit block 0 by 3.8° and 2.5°. Both captured attempts agree on all four |
| **Acceleration radius** | The palm sits at the larger radius under rotation and reads the higher magnitude. Across **30 swing captures**, block 1 is higher in **4,996 of 5,487 moving samples (91.1%)**, where *moving* means max(&#124;a₀&#124;, &#124;a₁&#124;) > 20 m/s² |

**There is a field check**, and it is the third route used deliberately: rotate the forearm and
compare acceleration magnitudes — the block with the larger acceleration is the palm.

⚠ **It is a strong majority, not a per-sample rule.** 91.1% is not 100%, and the margin grows
with the vigour of the motion, so compare *peaks over a genuine swing* rather than trusting one
sample or a gentle wave. At rest, where both units read ≈0 (§6.4), it says nothing at all.

#### 6.3.1 The `0x7f` variant — produced by the legacy `82` start

Starting with the bare `82` command (§4) yields message id `0x7f` instead of `0x90`, in a
different and **more restricted** layout:

```
notification := 0x7f , record+
record       := block × sensor_count                      // 42 bytes at 2 sensors, NO header
block        := quaternion[8] , accel[6] , gyro[6] , u8   // 21 bytes, always
```

**There is no record header and no tick counter.** Blocks start at offset 0.

⚠ **The 21-byte block is fixed for this message id**, and in particular it does *not* follow
configuration bit 5 the way a `0x90` block does (§6.2). Where a `0x90` block carries a two-byte
tick counter, a `0x7f` block carries **one unsigned byte**, which read `00` in every frame of a
981-frame session. So the timestamp feature flag sizes a `0x90` block and has no effect here.

⚠ **This format carries no device clock whatsoever.** Consequences:

- Timing can only come from host arrival, which BLE bunching makes unreliable.
- **History retrieval is impossible against it** — `a1` addresses the record header (§6.5),
  and there isn't one.
- Sample rate and gyro scale cannot be separated from the data alone, since each is only
  recoverable given the other.

`|q| = 16383.2 ± 0.31` over 1,962 quaternions confirms the field positions.

**Prefer `a0 01 7e`.** The legacy path exists, and the device accepts it despite the vendor app
gating it to protocol < 3, but it gives up the sample counter, the tick counters and history.

### 6.4 Field semantics and scales

| Field | Type | Scale | Unit | Full scale |
|---|---|---|---|---|
| Quaternion | 4 × `i16be` | `/ 16384` (Q14) | unit quaternion, order **w, x, y, z** | cannot clip — ±2.0 representable, components never exceed ±1 |
| Linear acceleration | 3 × `i16be` | `× 0.0098` | m/s² (1 LSB = 1 mg) | **±32.8 g** (±321 m/s²) |
| Gyroscope | 3 × `i16be` | `/ 8` | °/s — **with config bit 6 set** | **±4096 °/s** |
| Gyroscope | 3 × `i16be` | `/ 16` | °/s — bit 6 clear | **±2048 °/s** |
| Tick counter | `u16be` | — | ticks, per block — §6.5 | wraps at 65536 |

Acceleration is the fusion's **gravity-removed** linear acceleration, not a raw accelerometer
reading — it reads ≈0 at rest. Name it accordingly in any API.

**⚠ The full-scale limits are reachable, and clipping is silent.** These are `int16` fields:
they saturate rather than wrap, so clipped data still looks like a plausible waveform — a
flattened peak, not an obvious fault — and nothing in the protocol reports it.

How much headroom there is depends on the motion, and not the way you would guess:

| Motion | Peak gyro | Of ±4096 °/s full scale |
|---|---|---|
| Golf swing, driver, struck ball | 2,179–2,376 °/s | 53–58% |
| **Sharp hand shake, sensor held loose** | **3,399 °/s** | **83%** |

**A deliberate flick of the wrist beats a golf swing**, because the sensor sits on the wrist
rather than the clubhead. So a client cannot size expectations from the sport: the worst case is
whatever the *sensor* is subjected to, including handling, knocks and someone waving it about
before a session starts.

Nothing clipped in any measured session, but the margin is under 2×. A client should **count
pinned samples per channel** (`|count| ≥ 32767`) and report them alongside the data rather than
assume the range is generous. ⚠ Under a configuration with bit 6 clear the gyro range halves to
±2048 °/s, at which point both motions above clip.

Quaternions are normalised on the wire: `|q| = 16384.7 ± 0.41` measured over 6,064 records.
That norm is the cheapest structural check that a frame has been located correctly. Compute it
on the **raw `i16` values**, before any scaling.

**⚠ Prefer normalising to dividing by 16384.** Q14 describes the encoding correctly, and on the
stream the two agree to about 4 × 10⁻⁵ — but normalising is insensitive to firmware-dependent
precision variation, and §8.2 documents a payload where the difference is decisive. Whichever a
client picks, the norm must be **reported and not enforced**: a decoder that range-checks after
scaling and rejects what falls outside will reject real device output.

Axis order within each triplet is the sensor's own frame. **The gyroscope shares the
quaternion's frame exactly** — identity axis map, cross-coupling below 0.25%.

**The tick rate is ≈64,068 ticks/s, and is not a constant to rely on.** It is not measured
directly; it follows from the two figures in §6.5 — the sample rate and the ticks-per-sample
ratio:

```
799.19 Hz × 80.166 ticks/sample ≈ 64,068 ticks/s      → 15.61 µs per tick, wrap every 1.023 s
```

A second session gives 799.32 × 80.136 ≈ **64,054**, and across four sessions the measured
range is **64,025–64,088**, about 0.1%. That spread is the same crystal variation §6.5 and §10
warn about, seen through a different lens. Use ≈64,068 for sizing and wrap arithmetic, where
0.1% is irrelevant; **fit it from the data for anything where timing accuracy matters**.

#### ⚠ The two accelerometers are not redundant, and must not be averaged

The two units sit **3–8 cm apart** on the hand and forearm, varying with the wearer. Under
rotation they are therefore at different radii from the axis, and their linear accelerations
differ by roughly `ω²r` — a real physical difference, not sensor disagreement.

It is large. During a golf swing at ~2,100 °/s the palm unit read **31–51 m/s² more** than the
lower-arm unit, consistently across five swings — several g of legitimate difference.

Consequences for a client:

- **Do not average the two, and do not treat a disagreement as an error or a fault.** They are
  measuring different points on a rotating body and are supposed to differ.
- **Do not derive a shared linear acceleration for "the device".** There isn't one. Any quantity
  computed from linear acceleration must state which unit it came from.
- **The separation is a property of the wearer**, not of the hardware, so it cannot be a baked
  constant. A client needing it must obtain it per user or avoid depending on it.

**The quaternions are unaffected** — rotation is the same about any point on a rigid segment, so
the relative rotation `q_palm ⊗ q_arm*` (§6.7) carries no `r` term at all. This is specifically a
linear-acceleration effect, and it is one reason the wrist metrics are built from orientation
rather than from acceleration.

### 6.5 The sample counter (record header)

A `u16be` counting the device's internal samples, which run at **≈799.2 Hz** — read the rate
warning below before using it for timing.

- **Starts at 0 on every start command.** It is per-stream, not per-power-cycle.
- **Advances only while streaming.** There is no background sampling — not on a timer, not on
  motion.
- **⚠ And it stops while a history retrieval is in flight** — including one issued without
  stopping the stream, for about as long as the retrieval takes (§7.5). The tick counters do not
  stop with it, so index and device time part company at every pull.
- Wraps every **82.0 s** (65536 / 799.2).
- Increments by **32** per record at the nominal 25 Hz and **8** at 100 Hz, because the live
  stream is a decimated view of the internal rate. Those increments are exact integers, so the
  live rates they imply are 24.98 Hz and 99.9 Hz rather than round numbers. **But 32 and 8 are
  not the only steps** — see §6.6.
- Is **approximately** the per-block tick counter ÷ 80 — close enough to identify the field,
  not close enough to time with. The ratio measured over long live spans is **80.166**
  (6,063 steps across 238 s) and 80.136 (1,123 steps across 15 s). Both blocks of a record
  agree on it to within 2 ppm, confirming the two MCU timers run at one rate.

#### ⚠ The internal rate is ≈799.2 Hz, not 800

Measured directly — sample index against host clock, lower-envelope fit — on two independent
host clocks, which rules out host-side error:

| Session | Host clock | Span | Rate |
|---|---|---|---|
| Phone, driven by the vendor app | Android btsnoop | 238.0 s, 6,064 records | **799.19 Hz** |
| Laptop, no app | Linux `CLOCK_REALTIME` | 14.9 s, 1,124 records | **799.32 Hz** |

A second unit-session measured **799.47–799.55 Hz** across three independent 20–45 s fits —
about 400 ppm above the figures in the table. Between sessions and between devices this constant
moves; that is why §10 says to fit it rather than adopt it.

**Assume 799.2 Hz, never a round 800.** The round number costs **≈+1,000 ppm, or 1 ms per
second of streaming**, and the error is one-directional: mapped times run *early*, by 12 ms
after 12 s of streaming and 45 ms after 45 s. Against a 240 fps camera frame of 4.2 ms (§10)
that is 3 to 11 frames.

The two sessions differ by ~160 ppm, so even the right constant leaves ≈0.2 ms/s. **Where
alignment matters, fit the rate per connection** instead of trusting any constant (§10).

**It is also the address space the history command selects on** (§7). Two consequences a
client cannot ignore: indices are **not unique across streaming sessions**, so anything
persisted needs a session identifier; and any device→host clock mapping must be rebuilt at
each start.

### 6.6 Sample rate

The **live** rate is adaptive. Two regimes dominate — **25 Hz at rest, 100 Hz in motion**,
exactly 4×, switching per record rather than wholesale — and the tick clock is constant across
both, so this is genuine resampling rather than a clock change. What triggers the switch is
undecoded.

**⚠ 25 and 100 Hz are not the only live rates.** The device also emits short **bursts denser
than 100 Hz**, with index steps anywhere from 1 upward. Measured across every session carrying
a header index:

| Session | Config | Steps | 32 | 8 | **1–7** | 9–31 | Dense |
|---|---|---|---|---|---|---|---|
| 238 s, arm mostly at rest | `7e` | 6,063 | 5,904 | 102 | **35** | 22 | 0.6% |
| Single-axis motion | `7e` | 5,378 | 4,558 | 771 | **25** | 24 | 0.5% |
| Single-axis motion | `7e` | 5,395 | 4,059 | 1,279 | **8** | 49 | 0.1% |
| 15 s, hand motion | `7e` | 1,123 | 135 | 905 | **75** | 8 | 6.7% |
| Hand motion | `5e` | 920 | 190 | 650 | **54** | 26 | 5.9% |
| 4 s, hand motion | `7e` | 254 | 48 | 182 | **16** | 8 | 6.3% |
| **Stationary throughout** | `7e` | 99 | 99 | 0 | **0** | 0 | 0% |

Dense steps appear in **every session containing motion**, and only the stationary one is pure
+32. The step distribution is a continuum with two strong modes, not a two-state switch: the
238 s session alone shows every value from 1 to 8 plus 10, 12–19, 22, 24, 25, 27, 28 and 32.
Bursts reach the full internal rate — one run in it is eight consecutive steps of 1, nine
records at adjacent internal samples.

They are genuine extra samples, not a re-phasing onto a new grid: across 55 runs, only 2 sum to
a multiple of 8. They are structurally sound (`|q| = 16384.7` across all 75 dense steps in the
15 s session), occur within single notifications as well as across boundaries, and are
independent of the configuration byte (`5e` behaves like `7e`).

**So 100 Hz is not the live ceiling**, and a client that assumes a maximum live rate — for
buffer sizing, for a fixed-size ring, or for a decimation assumption — will be wrong under
exactly the conditions that matter most.

**What triggers a burst is unknown.** Motion is necessary but is not the trigger: measured
against the +8 regime rather than the session as a whole, median angular rate during dense
steps is *lower* in the three slow sessions (32.6 / 46.9 / 63.0 °/s against 105.3 / 108.1 /
86.1) and higher in only one. Do not model the live rate as a function of angular rate.

**The internal rate is ≈799.2 Hz regardless** (§6.5) — but ⚠ history does **not** return it in
full everywhere. The buffer is motion-adaptive too, floored at 100 Hz and reaching the full rate
only in fast motion (§7.3). Live and buffered density rise and fall together; history is denser
than live at every point, never uniformly 800 Hz.

**One useful consequence of the bursts:** because the rate rises exactly when the motion is
fastest, the live stream captures peak *magnitude* well even though it cannot describe the
*waveform*. Across five golf swings the largest angular rate seen live matched the full-rate
history to within 3%. So live data is sound for **detection and thresholding**; it is the shape
of the event, not its size, that requires a retrieval.

A client must **never assume a fixed period**. Derive timing from the sample counter or the
tick counter, never from notification arrival — BLE bunches notifications and inflates any rate
measured from delivery.

### 6.7 ⚠ Quaternion convention

**The streamed quaternion maps world → body.** This is the conjugate of the convention most IMU
code assumes, and it determines every composition order downstream.

```
correct:    r = q(t+1) ⊗ q(t)*        ω_body = −2·atan2(|r.v|, r.w)/dt · unit(r.v)
wrong:      r = q(t)*  ⊗ q(t+1)
```

The relative rotation of palm with respect to lower arm is therefore

```
q_rel = q_palm ⊗ q_arm*          NOT q_arm* ⊗ q_palm
```

Getting this backwards leaves the wrist **angle** correct — `2·acos|q_a·q_b|` is
convention-blind — while mirroring every decomposed component.

**⚠ So the obvious check cannot catch it.** A client that validates its decode by confirming the
wrist angle looks sensible will pass with the composition order reversed, and every downstream
component will have the wrong sign. Verify against a **known single-axis motion** instead: a
pure rotation about one anatomical axis should move one decomposed component and leave the
others near zero, and a reversed order shows as inverted signs rather than wrong magnitudes.

---

## 7. History / replay

The device buffers its full-rate internal samples and replays them on request. This is the only
way to obtain data above the live rate.

### 7.1 Command

```
a1 <first u16be> <last u16be>        protocol version ≥ 3
```

Arguments are sample-counter indices (§6.5). **`first` must be below `last`.**

**⚠ A reply is holed by default, and the holes are not an error.** The returned set spans the
requested range but carries only 16–50% of its indices in a typical session (§7.3), because the
buffer itself is motion-adaptive: it holds every internal sample only where the wrist was moving
fast. There is no error and no indication. A client must compare the delivered index range *and*
density against what it asked for (§7.6), and must not read a gap as a fault. A range that is
invalid rather than merely unavailable returns `d0 03` (§7.2).

### 7.2 Reply sequence

```
a1 01        end marker    (closes any previous retrieval)
a1 02        start marker  (request accepted)
90 ...       records, consecutive — sample counter increments by 1
a1 01        end marker
```

The `0xa1` payload byte is a marker discriminator: `02` = start, `01` = end. **Any other value
is rejected** by the app with "unknown magic cookie".

An invalid range yields the leading end marker followed by **`d0 03`**, and no start marker:

```
-> a1 0000 0000      <- a1 01        <- d0 03
```

So **the presence of `a1 02` is the acceptance test**, not the absence of an error.

**⚠ `d0 03` is the only error the history command produces**, and it does not say what went
wrong. Measured across seven cases — a request before any stream had ever run, a reversed range
with `first` above `last`, a null request, the full index space, and indices belonging to a
previous streaming session — **every one returned `d0 03`**. A client therefore cannot tell
*no data yet* from *malformed request* from *evicted range*, and must track for itself which of
those it is in.

**Frames are `0x90`, not `0x7f`.**

### 7.3 Buffer properties

| Property | Value |
|---|---|
| Sample rate | **Motion-adaptive, ≈100–799.2 Hz** — index step 8 at rest, step 1 in fast motion (see below) |
| Depth | ~6000 samples ≈ **7.5 s** — see below |
| Scope | The **current streaming session only** |
| Delivery rate | ~260 notifications/s, ~10× live |

#### ⚠ The buffer is not a flat 800 Hz recorder

**It stores densely only where the wrist moved.** The step between consecutive returned records
tracks angular rate, monotonically, and the relationship is the same in every retrieval we hold
(25 retrievals, 17,739 steps, five sessions):

| Delivered index step | Effective rate | Median \|ω\| at that step |
|---|---|---|
| 1 | 799 Hz | 780–850 °/s |
| 2 | 400 Hz | ~370 °/s |
| 3–5 | 160–270 Hz | 150–210 °/s |
| 8 | **99.9 Hz** | 0.4–28 °/s |

**Step 8 is a hard floor and step 1 a hard ceiling**: across all 17,739 measured steps, no step
exceeded 8 and none was 0. The live stream's +32 (25 Hz) and its >100 Hz bursts have no
counterpart here — the buffer never drops to the 25 Hz live rate and never exceeds the internal
rate.

Two consequences that decide whether a client's data is usable:

- **A stationary or gently-moved sensor replays at 100 Hz, evenly, across the whole window** —
  one index in eight, no holes at the edges, nothing ragged. This is the *correct* behaviour and
  it is indistinguishable from a broken 800 Hz path unless you know to look at the motion. Bench
  testing at a desk therefore measures the floor, never the ceiling.
- **A real golf swing replays in full.** Across ten swings peaking at 1,900–2,100 °/s, the ±125 ms
  around peak came back as **201 records over 201 indices — every internal sample, step 1, in all
  ten**. The full-rate promise holds exactly where the event is, and only there. Confirmed again
  2026-08-15 from a second implementation's capture, on five mid-stream pulls of a 400-index
  window each: longest unbroken step-1 runs of **278, 292, 308, 294 and 330 records** —
  **413 ms of uninterrupted 799 Hz** in the best of them, at 958 ± 327 °/s.

Coverage of a whole requested window is therefore a property of the *motion in it*, not of its
width: measured coverage runs 16% (slow session, 4.5 s request) to 50% (brisk session, 1.7 s
request), and within one window rises to ~76% over the swing and falls to ~13% over the still
pre-roll. Sizing a request smaller does not make it denser.

**The depth figure is weaker than the others.** It was measured once, after 20 s of streaming
at the 25 Hz live rate. Whether the buffer holds a fixed *sample count*, a fixed *duration*, or
something that varies with motion has not been established — and given the adaptive rate above,
a fixed sample count and a fixed duration are very different guarantees.

A client should **not hard-code 6000 samples**: size requests from what the device actually
returns (`requested` versus `delivered`), and treat 7.5 s as an order of magnitude rather than
a contract.

### 7.4 ⚠ Constraints on use

- **The device only buffers while streaming.** A client that wants history must hold a stream
  open for the whole session; there is nothing to retrieve otherwise.
- **Retrieval takes about as long as the requested window spans.** Pulling the full 7.5 s
  buffer takes just under 8 s. Request a narrow window around the event of interest; a client
  that dumps everything after each event has no margin before eviction.
- **⚠ That time is also a recording gap, whether or not the stream was stopped.** The sample
  counter stops for the duration of the pull (§7.5), so retrieval time is lost data, not just
  latency — which is the second, stronger reason to keep the window narrow.
- **Stopping the stream does not clear the buffer** — retrieval after `0x83` returns the
  session just stopped.
- **⚠ Starting a new stream DOES clear it.** Measured: after a stop and restart, a request for
  indices belonging to the previous session returns `d0 03` with no start marker and no records.
  The buffer and the index space are per-session together, so **nothing survives a restart** —
  neither the data nor the addressing.
- A range spanning the 82.0 s counter wrap needs care: unwrap internally, re-wrap when asking.

### 7.5 Retrieval does not require stopping the stream

**`a1` may be issued while streaming.** The device accepts it, brackets the reply normally, and
returns records without the stream having to be stopped first.

Measured against a stopped control at the same request width:

| | Records | Coverage of the requested span |
|---|---|---|
| Issued mid-stream | 4,182 | **58%** |
| Issued after `0x83` | 2,605 | 33% |

Mid-stream was **not** worse. ⚠ Both are partial because the buffer is motion-adaptive (§7.3),
**not** because the request was wider than the buffer holds — narrow requests come back holed
too, at the same 16–50%. Neither figure says anything about request width.

**Why this matters, and the one thing it does not fix.** A client that must stop to retrieve pays
four costs: a recording gap, a fresh index space, a clock fit rebuilt from nothing, and a
calibration to re-run. **Issuing `a1` in place removes three of them. It does not remove the
recording gap.**

#### ⚠ The sample counter stops during a retrieval, and issuing `a1` in place does not prevent it

**The device stops taking samples while it is replaying them**, and nothing on the wire says so.
Measured on a 44.5 s stream carrying six mid-stream pulls, 1,728 live records: across each pull
the tick counters ran on while the sample counter did not.

| Pull | Bracket `a1 02`→`a1 01` | Records returned | Samples never taken | Recording lost |
|---|---|---|---|---|
| 1 | 344 ms | 329 | 267 | **334 ms** |
| 2 | 354 ms | 333 | 272 | **340 ms** |
| 3 | 370 ms | 347 | 292 | **365 ms** |
| 4 | 331 ms | 337 | 253 | **317 ms** |
| 5 | 406 ms | 364 | 293 | **367 ms** |
| 6 | 25 ms | 27 | 12 | **15 ms** |

**The gap is the retrieval's own duration** — 90–99% of it in the five full-size pulls, and it
tracks pull size across a 16× range. It is not a fixed per-pull cost a client can budget once.

The two counters disagree only there, which is what makes this device behaviour rather than a
link or host artefact — both numbers come out of the same frames:

| Over the whole session | Ticks per sample index | Implied internal rate |
|---|---|---|
| All 1,727 live-frame pairs | 83.40 | 768 Hz |
| Excluding the six pairs that straddle a pull | **80.14** (§6.5: 80.166) | **801 Hz** |

Six pairs out of 1,727 carry the entire distortion. Two controls agree: a capture with no
retrieval in it reads 80.138, and one whose single pull falls after the last live frame reads
80.15.

**What it costs beyond the lost samples:**

- **The index→host-time mapping gains a hidden offset at every pull** (§10). Sample number and
  wall clock part company by the width of the stall, and nothing in the data marks the seam.
- **It eats the tick-counter wrap budget** (§10.2). One stall is ~23,000 ticks of unpredicted
  advance against the ±32,768 that picks the wrong wrap — 72% of it, where a pull-free gap uses
  12 ticks.

**What survives, and is still worth having.** The buffer is not cleared, the index space does not
restart, the clock fit does not restart, the calibration is untouched, and coverage is no worse
than a stopped pull (table above). A mid-stream pull remains the right way to retrieve. It is the
recording gap that has to be designed around rather than assumed away.

The missing time is unaddressable either way, since `a1` selects on the counter (§7.1).

### 7.6 Capturing an event, end to end

This is the operating mode the whole protocol resolves to, and it is not obvious from the parts.
**The live stream is not the data. It is the clock reference for the data.**

A 250 ms downswing is 6 samples at the 25 Hz live rate and 25 at 100 Hz — not enough to describe
the event at all. The same 250 ms is ~200 samples in the device's internal buffer — **measured,
in all ten swing captures: 201 records over the 201 indices spanning ±125 ms of peak** (§7.3). So
the live stream exists to produce `(index, hostArrival)` pairs for the fit, and every sample a
client actually analyses comes from a retrieval.

⚠ That density is bought by the motion, not by the request: the same retrieval returns ~100 Hz
over the still pre-roll, and a session with no real swing in it returns ~100 Hz throughout
(§7.3). **A client cannot validate its retrieval path on a stationary sensor** — at a desk, a
correct implementation and a broken one produce the same 100 Hz.

**The cycle, once per session:**

```
1  connect, bring-up, start the 30 s 0x81 poll               §9.1, §9.2
2  a0 01 7e — ONE stream, opened once and left open          §6.1
3  calibrate, with that stream still running                 §8.2
4  from here on, every live frame feeds the clock fit        §10
   ...
5  the application detects an event however it detects it — the device does not help  §0
6  choose a window around it, in host time
7  convert host time -> index through the fit, and issue a1 IN PLACE — do not stop    §7.5
8  map every returned record back to host time by its index                           §10
9  go to 5. The stream never closed, the fit never restarted.
```

**Sizing the window.** Convert with the *fitted* rate, not a constant (§6.5):

```
firstIndex = round((eventHostTime - preRoll)  * rateHz) + originIndex
lastIndex  = round((eventHostTime + postRoll) * rateHz) + originIndex
```

A 3 s pre-roll and 1.5 s post-roll is a 4.5 s window against a ~7.5 s buffer, which leaves
usable margin. **Do not request the whole buffer**: retrieval takes about as long as the window
spans (§7.4), so a client that dumps everything after each event has no margin before the next
one evicts. Note that a *narrower* request does not come back denser — density is set by the
motion in the window (§7.3), not by its width — so narrow the window for time, not for detail.

**⚠ The six ways this goes wrong**, all of them silent:

- **Treating the timeline as continuous across a pull.** The device stops sampling while it
  replays (§7.5), so every retrieval leaves a hole the width of the retrieval — ~350 ms for the
  narrow pulls measured, and as long as the request for a wide one. Two things follow: a second
  event landing inside that window is lost exactly as it would have been on a stopped stream, and
  the clock fit must be re-anchored after the pull rather than carried through it (§10).

- **Restarting the stream.** It clears the buffer and resets the index space (§7.4, §6.5), so
  every event captured before the restart becomes unaddressable and the clock fit starts from
  nothing. Keep one stream.
- **Requesting too late.** The buffer holds ~7.5 s. Retrieval is not instant, and the clock is
  running from the moment the event happened.
- **Reading the holes as a fault.** Every reply is holed (§7.3). A client that treats missing
  indices as corruption, or that resamples onto a uniform 800 Hz grid without checking density,
  will either reject good data or invent samples that were never taken.
- **Trusting `d0 03`.** It is the only error code and means nothing specific (§7.2).
- **Assuming the pull is complete.** Compare the delivered index range against the requested one
  and report the difference; a short pull that quietly drops the start of the event looks
  identical to an event that started late.

**If a client must stop to retrieve** — because a platform, a library, or a cautious first
implementation prefers it — the sequence still works: `0x83`, retrieve, `a0 01 7e`. It costs a
fresh index space, a clock fit rebuilt from nothing, and a calibration that must be re-run.
§7.5 exists so that no client has to pay *those three*. The recording gap is not on that list:
it is paid either way, and its size is the retrieval's own duration.

---

## 8. Calibration

### 8.1 Model

Three frames are in play: **earth** (gravity), **sensor** (however the board sits on the arm)
and **anatomical** (the wrist's own axes).

1. Each unit's fusion outputs absolute attitude, world → body.
2. The relative rotation `q_palm ⊗ q_arm*` cancels earth — which is why a swing can face any
   direction — but still carries both mounting offsets. Uncalibrated, a straight wrist reads
   11–15°, which is board placement, not anatomy.
3. Calibration supplies a per-unit **mount quaternion** mapping sensor → anatomical, plus a
   per-unit **rotation about z** carrying the heading the raise revealed. Those four values are
   the whole calibration state, and the device hands all four back in §8.2's result.
4. **The device applies it itself.** Both quaternions step discontinuously the instant the
   result is emitted (56° and 79° within one 30 ms record) and the relative angle collapses to
   ~0.4°.

**A client issues the two markers and reads angles already in the anatomical frame.** There is
no host-side transform to replicate.

Two poses are needed rather than one because zeroing at a single pose establishes that the
wrist is neutral but not which direction is which. Rotating between two known poses reveals the
axis the motion occurred about, and that axis plus gravity pins the full anatomical frame.

### 8.2 Transaction

```
-> a2 00     <- a2 01        pose 0: forearm horizontal, wrist straight
-> a2 01     <- a2 01        pose 1: forearm raised ~30° across the chest
             <- 94 + 64      calibration result; eight quaternions, all identified
```

The device re-references its own stream at the moment `0x94` is emitted. A client does not need
the payload, since the transform is applied on-device — but it is not opaque, and what is
in it bears directly on the quality problem below.

**⚠ The stream must already be running.** Calibration is not a standalone transaction: the
device observes a *continuous raise* between the two markers, which cannot be done from two
static samples. Start streaming first (§6.1), then send the markers. Measured order is
`a0 01 7e`, then `a2 00` 40 s later, with the stream running throughout and never stopped.

#### ⚠ `0x94` has two forms, and the length is what tells them apart

| Form | Length | Contents |
|---|---|---|
| **Long** | 65 bytes — the id and 64 payload bytes | Eight quaternions. **No status byte anywhere in it.** |
| **Short** | 63 bytes or fewer | A single **status code**, in the byte after the id |

The device splits on the notification's *total* length, so **64 bytes is neither**: it is above
the short form's ceiling and one byte shy of the long form's eighth quaternion. Treat it as
truncated rather than salvaging seven quaternions and guessing the eighth.

Every payload ever captured is the long form. **The short form has never been observed and no
value of its status code has a known meaning** — but a parser must not decode a short `0x94` as
a truncated long one, because that discards the only form that carries a verdict-shaped field at
all. See the verdict warning below for why the distinction matters.

#### The long form is eight quaternions, and all eight are identified

**Layout: 64 bytes = eight quaternions, each four `i16be` in `w x y z` order** — the same
component order and endianness a record carries its orientation in (§6.3, §6.4). No gaps, no
padding, no reordering: slot order is byte order.

| | Offset | What it is | Angle from the streamed orientation at that marker |
|---|---|---|---|
| **q1** | 0–7 | **palm unit — rotation about z** | |
| **q2** | 8–15 | **palm unit — mount quaternion** | |
| **q3** | 16–23 | **lower-arm unit — rotation about z** | |
| **q4** | 24–31 | **lower-arm unit — mount quaternion** | |
| **q5** | 32–39 | **palm unit at the `a2 00` marker** | **0.06°** / 0.3° |
| **q6** | 40–47 | **palm unit at the `a2 01` marker** | **0.11°** / 2.7° |
| **q7** | 48–55 | **lower-arm unit at the `a2 00` marker** | **0.52°** / 0.7° |
| **q8** | 56–63 | **lower-arm unit at the `a2 01` marker** | **1.80°** / unresolved |

So the payload is **the complete calibration state the device computed, followed by the two
poses it computed that state from**: one mount quaternion and one rotation-about-z per unit,
which is exactly the per-unit state model the device maintains, and then the four pose readings.
Each of the eight is a named field, and the names map one-to-one onto this document's palm /
lower-arm vocabulary; they are given descriptively above.

The two pose columns carry one figure each from two captures: a library bench capture with a
full ~30° raise, and the app-driven `07-calibration`, whose raise was barely 4° — too small for
its q8 to be separated from the pose-0 value at that payload's precision.

**⚠ `q1` and `q3` have `x = y = 0` exactly**, not approximately: they are pure rotations about z
by construction, reducing to `(w, x, y, z) = (cos θ/2, 0, 0, sin θ/2)`. Those four exact zeros
are a property of the *field*, not of the pose, so they are a free structural check that a
payload has been located correctly — and see the counting warning below for the one place they
mislead.

⚠ **The unit order is the reverse of a record's.** A `0x90` record puts the lower arm first
(§6.3); this payload puts the **palm** first. Measured on both captures, with the two units
10.6° and 14.8° apart at the marker, so it is not a coincidence of one pose. A client that
carries the record's convention across gets the units swapped — which §6.3 warns is silent, and
produces a wrist angle that looks entirely plausible and is mirrored.

⚠ **Match these against the sample index, not the host arrival time.** Live delivery bunches, and
at stream start it can run seconds behind (§10): in the bench capture the first frames arrived
4.08 s late, so the frame *arriving* at the `a2 01` marker carried an index from 6.5 s earlier
and the payload appears to hold the *raised* pose first. Mapped through the index, the order is
`a2 00` then `a2 01` — pose 0 first, in both captures.

The field names settle it independently, which is the point of naming them: **pose 0 comes first
within each pair**, and that conclusion now rests on the layout rather than on any timing
reconstruction. The warning above stands for anyone verifying the payload against a live stream,
where the trap is easy to fall into and reverses the answer.

**The separation within each pair is the raise that was actually performed:**

| Raise, measured from the stream between the markers | q5–q6 (palm) | q7–q8 (arm) |
|---|---|---|
| ~30°, the routine as specified | **28.9°** | **30.9°** |
| 7.6° palm / 8.1° arm | 11.7° | 10.2° |
| 4.2° palm / 4.7° arm | 8.5° | 7.6° |

So the payload carries the one thing the presence check below is structurally blind to — whether
the raise happened at all. A calibration with no raise leaves the anatomical frame undetermined
while scoring *perfectly* on the residual, and a separation far below the routine's ~30° target
is the signal that says so.

⚠ **But it bounds the raise's SIZE, not its AXIS.** A large raise about the wrong axis scores
exactly as well as a correct one, so this catches the missing-raise failure and not the
wrong-raise failure — which the residual, measured below, also fails to catch. And the decisive
test has not been run: a deliberate no-raise attempt, which should show both pairs near zero,
has never been captured with its payload. **Treat the separation as a promising signal rather
than a validated one**, and do not build an accept/reject gate on it alone.

**Precision is not the same in every payload, and a parser must not assume 16 bits.** The three
payloads captured on firmware 4.5 carry only the **high byte** of each component — every one is an
exact multiple of 256, so |q| lands within 200 of 16384 rather than within 1, and each pose is
good to about a degree. The one payload captured on 4.8 uses the full range.

Counted over the non-zero components, which is the test that separates the two:

| Payload | Non-zero components that are multiples of 256 | |q| |
|---|---|---|
| Firmware 4.8 | **0 of 28** | 16384 ± 1 |
| Firmware 4.5, attempt 1 | **27 of 27** | 16384 ± 200 |
| Firmware 4.5, attempt 2 | **26 of 26** | 16384 ± 200 |

⚠ Count only the **non-zero** components. Every payload has four exact zeros — `x` and `y` of q1
and q3, which are pure rotations about z by construction — and zero is a multiple of 256, so a
naive count reports 4 of 32 (12.5%) on a full-precision payload and reads as a partial result. It
is not: chance would put a non-zero component on a multiple of 256 once in 256, so 0 of 28 is the
expected count for a 16-bit encoder and 27 of 27 is impossible for one.

Three payloads on 4.5 and one on 4.8 is not enough to attribute the change to the firmware update
itself, and all three of the 4.5 payloads are from one device on one day. But the difference is in
the data rather than in any decoder, so **a parser should take the precision from the payload in
front of it** — and must not reject a coarse one as malformed, since |q| off by 200 counts is the
correct reading of a real device's output.

#### ⚠ Normalise. Do not scale and validate

**There is no fixed-point scale in this payload.** The four `i16be` components are sign-extended
and normalised; the 16384 is implicit in the normalisation and is never divided out. That is not
a stylistic preference, it is what makes a decoder work across firmware revisions:

```
w, x, y, z = i16be × 4                      # sign-extended
n          = sqrt(w² + x² + y² + z²)        # reject only n == 0
q          = (w/n, x/n, y/n, z/n)           # unit, (w, x, y, z)
```

**A parser that divides by 16384 and then range-checks the norm rejects real device output**, on
every coarse payload above. A parser that normalises never notices the difference. The norms are
worth *reporting* as a structural check — a correctly located payload is within a few hundred
counts of 16384, never within 1 — but they must not be a validity gate.

#### What the four state quaternions are

The two poses and the four applied state quaternions are related. Taking one unit's pose pair as
the reference:

```
q_rel      = Pose1⁻¹ ⊗ Pose2                    of the reference unit
axis, θ    = axis-angle decomposition of q_rel
heading    = atan2(axis.y, axis.x) ± π           (+π if negative, −π otherwise)
RotationZ  = quaternion(axis = (0,0,1), angle = heading)
Mount[n]   = (Pose1[n] ⊗ RotationZ)⁻¹
```

with `⊗` the Hamilton product in the standard convention. Checked against two captured payloads,
using each payload's own `RotationZ` and comparing against its own `Mount`, this lands **3.61° /
1.73°** and **1.77° / 1.96°** out (palm / arm) — agreement, given ~1° per component compounding
through a product, and better than the next candidate form by a factor of two or more.

The heading agrees too. Taking the **lower-arm** pose pair as the reference, the payload's palm
`RotationZ` sits 2.62° / 1.23° from the raw heading and its lower-arm `RotationZ` 2.05° / 1.37°
from the flipped one.

⚠ **This is the form, not a reproduction of the device.** That last figure is the tell: the two
units' `RotationZ` values in a real payload are **180.6°** and **180.2°** apart, so the firmware
applies the ±π flip to one unit and not the other, and one heading computed once for both units
does not reproduce that. **A client reads the payload; it has no reason to derive it**, and must
not present a locally computed result as equivalent to the device's.

#### ⚠ `0x94` is not a verdict, and the device does not judge the attempt

`0x94` is the device answering the marker. **It is emitted for every `a2 01`, and the device
applies the transform every time** — including for attempts a user-facing application goes on
to reject. Two consequences:

- **Nothing on the wire carries accept/reject.** A client that treats the arrival of `0x94` as
  success has no failure detection at all. Any verdict is the client's own to compute. The long
  form cannot carry one even in principle: all 64 bytes are accounted for by the eight
  quaternions above, with no field left over — and the status byte belongs to the short form,
  which no device has ever been seen to send in answer to `a2 01`.
- **The device imposes no observed deadline** between the two markers. One attempt took
  **15.6 s** between `a2 00` and `a2 01`; the device returned a result and applied it, and it
  was the *application* that rejected it as too slow. Any time limit is therefore client policy,
  not a device constraint — but the raise cannot be arbitrarily slow and still describe a single
  motion, so a bound is worth having.

**What a client can check, and what it cannot.** The relative angle between the two units at
the reference pose distinguishes *calibrated* from *uncalibrated*, and the gap is enormous:

| State | Relative angle at the reference pose |
|---|---|
| Calibration applied | **0.36° / 0.79°** (two attempts) |
| Held pose shortly after | 3.73–3.80° |
| Uncalibrated, or after a power cycle with the strap untouched | **15.01°** (14.36–15.76) |

⚠ **But it tests the zeroing, not the axis, and under a deliberately bad calibration it
inverts.** The two-pose routine does two separate things (§8.1): pose 0 zeroes the wrist, and
the *raise between* the poses reveals which axis the motion occurred about, which is what
separates flexion from deviation. The relative angle at the reference pose tests only the first
of those.

**Measured directly**, three calibrations in one session on one mounting, each read at the same
reference pose:

| Calibration | Residual |
|---|---|
| Correct routine — horizontal, then raised across the chest | 1.96° |
| Raise about the **wrong axis** — straight up, no across-the-chest component | 6.10° |
| **No raise at all** — pose 1 marked without moving | **0.70°** |

**The calibration carrying no axis information at all scored best.** That is not a fluke of one
run: with nothing moving between the two markers the zero is perfect by construction, while the
anatomical frame is left entirely undetermined. The residual measures exactly the half that
cannot fail.

So a client must **not** present this figure as a quality score, and must not select between
calibration attempts on it — doing either would systematically prefer the worst attempt
available. Its one sound use is detecting **"calibration never happened, or was lost"**, which
is otherwise silent and permanent: that failure sits at 12–19°, an order of magnitude away from
any of the three above.

### 8.3 Lifetime

**Calibration is session-scoped and must be re-run every session.**

- Lost by **power cycling** — measured 3.75° → 15.01° at the same pose with the strap untouched.
- Lost by **remounting** — it maps sensor frame to arm frame, so moving the board invalidates it.
- **⚠ Lost by a plain disconnect.** Measured 0.70° immediately before dropping the link, 18.80°
  at the same pose after reconnecting, strap untouched and never removed. A BLE link drop is
  neither a power cycle nor a remount, and it costs the calibration just the same — so an
  ordinary reconnect must re-run the routine, not resume.

No client can safely inherit a calibration. **Do not build persistence for it** — a
`saveCalibration` / `loadCalibration` / "reuse last session" convenience would produce
confidently wrong data with no error, which is worse than requiring the routine every time.
An API that cannot express the impossible thing is worth more here than a comment.

---

## 9. Session lifecycle

### 9.1 Bring-up

The vendor app's sequence, reproducible verbatim:

1. Enable notifications — write `0x0100` to CCCD `0x0019`
2. `80` — versions
3. `81` — status
4. `84` — sensor map
5. `81` — status again
6. `86` × 3 — serial
7. `85` — MAC

**Only step 2 is required.** The rest is the vendor app's behaviour, recorded because
reproducing it is a useful bring-up test, not because the device demands it. Protocol version
from `80` gates features (§6.2, §7.1); everything else is informational.

### 9.2 ⚠ Keepalive is mandatory

**The 5-minute idle shutdown applies while connected.** A silent connection drops at 5.0
minutes. The vendor app polls `0x81` every 30 s, and that poll is a keepalive — the battery
reading is incidental.

**⚠ An active stream does NOT prevent it.** A connection streaming continuously at 25 Hz was
dropped at exactly 5.0 minutes, the same deadline as a fully silent one. So a client cannot
rely on receiving data to stay alive.

**What resets it is a host → device write, not motion.** Both connections that died were also
completely stationary, so "idle" could have meant either. A **stationary** sensor held for
**7 minutes** with a 30 s `0x81` poll survived, which separates them:

| Session | Host writes | Moving | Outcome |
|---|---|---|---|
| Connected, no stream | none | no | dropped at 5.0 min |
| Streaming continuously | none | no | dropped at 5.0 min |
| Connected, 30 s `0x81` poll | every 30 s | **no** | **survived 7 min** |

**Poll `0x81` every 30 s.** It is what the vendor app does, and it is now measured rather than
assumed.

### 9.3 Teardown

`83` stops the stream cleanly. Disconnecting also stops it.

**`fa` powers the device off**, after which it needs a physical button press to return; the
device does come back cleanly on that press. Two things a client should expect:

- **No acknowledgement is sent.** The vendor's own tables name a "device turned off"
  notification, but none arrives — measured, zero notifications of any kind after the write.
- **The link stays up for several seconds.** Measured at **9 s** between the write and the
  device dropping the connection. A client must not read that gap as the command having failed,
  and must not retry into it.

### 9.4 The button

`0xfb` with payload `01` signals a button press. **The number of notifications per press is not
reliable** — one confirmed press produced none, another produced two 187 ms apart, with
identical payloads. Treat it as an edge hint, debounce anything within ~250 ms, and do not
build a press counter on it.

**⚠ Holding the button ~2 s powers the device off**, despite the manual stating 3 s. Any UI
that asks a user to hold the button must stay well under 2 s.

Press once to switch on; hold to switch off. A powered-off device needs a physical press to
return — no amount of scanning or reconnecting will raise it (§9.6).

### 9.5 Physical feedback — what the user sees and feels

A client drives a person through discovery, calibration and capture, and these are the only
signals that person gets from the hardware. LED semantics are the vendor's; vibration and beep
are observed.

| State | Indication |
|---|---|
| Advertising, ready to connect | **Pulsating** glow |
| Connected | **Steady** glow |
| Connection established | **Vibrates** |
| Powering off | **Beeps** |
| Charging | Red; off when fully charged |
| Battery very low | Red **blinking** |

The vibration on connect is the most useful of these to a client: it is immediate, unambiguous,
and confirms the connection to the user before any data flows.

### 9.6 ⚠ "Link dropped" and "device slept" are different states

They need different handling and only the client can tell them apart:

| | Cause | Remedy |
|---|---|---|
| **Link dropped** | Ordinary BLE supervision timeout, range, interference | Reconnect, with backoff. Usually succeeds |
| **Device slept or powered off** | 5-minute idle timer (§9.2), a ~2 s button hold, or `fa` | **A physical button press. Nothing else.** |

Retrying against a slept device cannot succeed at any interval. It burns host battery, occupies
the adapter, and shows the user a spinner for a problem only they can fix. Distinguish on
whether the device is advertising, how long the connection was idle, and whether the disconnect
was clean or a supervision timeout — then either back off, or tell the user to press the button.

---

## 10. Timing and host-clock mapping

Required by any consumer aligning this data against another source — cameras above all.

**History retrieval makes the mapping impossible to reconstruct after the fact.** Bulk records
arrive thousands at a time, seconds late, at whatever rate the link manages; their arrival
timestamps carry no information. The only anchor is the sample index they carry.

**The mapping must therefore be maintained during recording**, and it can be, because live
frames and history records share one index space:

1. While streaming, each live frame yields a `(sampleIndex, hostArrivalTime)` pair.
2. Fit those to get offset and rate against the host clock.
3. Apply the fit to history records by index.

**⚠ Fit the lower envelope, not least squares.** BLE delay is one-sided — a notification can be
late, never early — so least squares biases the offset by the mean link delay, which shows up
as every event landing consistently late. The minimum observed `hostArrival − index/rate` is the
best offset estimate; the spread of residuals above it is the honest uncertainty.

**⚠ Fit the rate too — do not hard-code it, and especially not as 800.** The internal rate is
≈799.2 Hz (§6.5). Two failures follow from assuming a round 800, and the second is the
subtle one:

1. Mapped times run early by ≈1 ms per second of streaming, one-directional.
2. **The lower-envelope estimator silently degenerates.** With the rate wrong, `hostArrival −
   index/rate` drifts monotonically instead of hovering, so its minimum always lands on one
   *end* of the session — in practice the first frame, the one most exposed to
   connection-setup jitter. The estimator collapses from a robust fit over thousands of frames
   to a single worst-case point, while still reporting small-looking residuals.

Fit rate and offset together over the connection's live frames. Because the counter resets at
each stream start (§6.5) but the crystal does not change, the correct decomposition is a
**per-session offset with a rate pooled across the whole connection**.

Residuals cannot validate this on their own — they measure link jitter around the fit, not
whether the fit is right. **Only an external physical reference can**, so validate against one
before trusting the numbers.

**⚠ Poor link quality shows up here, not as an error.** BLE at range drops and delays
notifications, and nothing in the protocol reports that. What a client sees instead is the
residual spread widening — the fit degrading quietly while every frame still parses and every
checksum-equivalent still passes. Two consequences: keep the host close to the sensor when
timing matters, and **treat residual spread as a link-health signal**, surfacing it rather than
averaging it away. A client that reports only a point estimate of its clock offset has thrown
away its only warning that the data is drifting out of alignment.

### ⚠ Validated against an external reference, and it does not yet hold

The fit has now been checked against a physical reference: five struck golf balls, with the
impact recorded on a microphone whose start instant was stamped on the same host clock. Five
audio impulses, five mapped acceleration transients.

| | |
|---|---|
| Fixed offset | ~142 ms — microphone and buffer latency, expected and harmless |
| **Drift** | **2.2 ms per second of session**, consistent across all five |
| Attributable to the audio reference | 0.76 ms/s — that sound card measured 759 ppm slow |
| **Unexplained** | **≈1.4 ms/s** |

The pass condition was a flat delta. It is not flat. **Over a session of any length the mapping
loses alignment at a rate well above one camera frame per few seconds**, and roughly two thirds
of that drift is not accounted for by the reference used to measure it.

The residue has **not** been shown to be the device: the audio timebase is only good to the
same order as the effect. What is established is that per-session fits, each conditioned on its
own live frames, do not agree with an external clock to better than a few milliseconds per
second. A client aligning against video should measure this for itself rather than assume
sample-level accuracy, and should prefer one long fit over many short ones (§7.5).

Precision available in principle: one sample is 1.25 ms against 4.2 ms for a 240 fps camera
frame, so sample-level alignment is reachable on paper — but the paragraph above is what was
actually measured, and it is the number to design against. The device clock is
stable enough to support it — but only just: the per-device spread is ~0.1% (§6.4), which is
why the rate is fitted rather than assumed.

Rebuild the fit at every stream start, since the counter resets (§6.5).

### ⚠ Re-anchor the fit after every retrieval — the mapping is piecewise, not continuous

The sample counter stops while a retrieval is in flight (§7.5), so one fit cannot span a pull:
index and wall clock gain a step of the pull's own width, ~350 ms in the pulls measured, with
nothing in the data to mark it. **Fit the rate across the whole connection as before, but carry a
separate offset for each stretch between retrievals**, and map a history record with the offset
of the stretch its index falls in.

The failure is loud rather than quiet, which is the one piece of luck here. Fitting a 44.5 s
session with six pulls in it as a single segment returned **768 Hz against a true 801 Hz**, and
the lower-envelope estimator reported itself degenerate with **1.7 s residuals** — the stalls
push the minimum of `hostArrival − index/rate` so far that the fit cannot hide it. A client that
fits per segment and compares the segments has a working consistency check; one that fits the
whole session and trusts the number does not.

### 10.1 Which notifications carry time, and which do not

Timestamping strategy is **not uniform**, and the wire gives no type tag to distinguish the two
kinds of `0x90`.

| Notification | Arrival | Device time in the frame | Strategy |
|---|---|---|---|
| `0x90` **live** | real time | sample index + 2 tick counters | Stamp on receipt; these pairs *are* the fit |
| `0x90` **history** | post-hoc, bulk | sample index + 2 tick counters | **Derive from the index** via the fit; arrival is meaningless |
| `0x7f` (legacy `82`) | real time | **none** — no header, no ticks | Arrival only. Cannot be aligned; do not use where timing matters |
| `0xfb` button | real time | **none** | Arrival only, and the count per press is unreliable (§9.4) |
| `0x94` calibration result | real time | none | The stream discontinuity at that instant is the timestamp (§8.2) |
| `0x80`–`0x86`, `0xd0` | real time, solicited | none | Arrival; none are time-critical |

**⚠ Live and history `0x90` frames are byte-identical in form.** The only discriminator is the
`a1 02` … `a1 01` bracketing (§7.2), which is protocol state, not frame content — so a client
must track that state to know how to timestamp what it is holding.

**Retrieval can be issued without stopping the stream** (§7.5), and doing so does *not* mix the
two kinds: measured over a mid-stream retrieval of 4,182 records, **every record delivered
inside the bracket fell within the requested index range**, and none carried an index beyond it.
Live delivery is suspended for the duration rather than interleaved — and so is sampling (§7.5),
so the first live frame after the bracket carries an index only a few counts beyond the last one
before it, however long the pull took. The bracket therefore remains a sound discriminator either
way, and it is also the marker for the seam in the clock fit.

### 10.2 The two counters are a coarse/fine pair — use both

The frame carries two clocks, and they solve each other's problem:

| | Resolution | Wraps every | Weakness |
|---|---|---|---|
| Sample index | 1.251 ms | 82.0 s | Coarse |
| Tick counter | 15.6 µs | **1.023 s** | Wrap is ambiguous across any gap > 1.02 s |

**The index resolves the tick counter's wrap, with no host clock involved.** Since ticks ≈
index × 80.166 (§6.5), an unwrapped index predicts the tick value, and the wrap count falls out.
Verified over a 238 s session: a ratio fitted on the **first 5 s** predicts the tick counter
across the whole of it — 2.9 index wraps, ~233 tick wraps — with a worst-case error of
**2,745 ticks against the ±32,768 needed to pick the wrong wrap**, 8.4% of budget, zero
failures on either unit.

So **device time is fully self-describing from the frame alone**, and it works identically for
history, where arrival times carry nothing. A client does **not** need host arrival to
disambiguate a tick wrap after a BLE gap.

**⚠ That budget assumes the sample counter never stops. It stops at every retrieval** (§7.5),
and the index then under-predicts the ticks by the whole width of the stall:

| Live-frame gap | Worst index→tick prediction error | Margin left of ±32,768 |
|---|---|---|
| Benchmark session, no retrieval in it | 2,745 (8.4%) | 30,023 |
| Measured, gaps with no pull in them | **12** (0.04%) | 32,756 |
| Measured, a gap containing one pull | **23,481** (72%) | **9,287** |

Two rules follow. **Re-anchor the ratio after every pull**: a client that anchors once at stream
start was out by 111,351 ticks — 3.4 whole wraps — after five pulls in a 44.5 s session, and
every history record it dated after the first pull was wrong by a multiple of 1.023 s. And
**never let two retrievals fall inside one live-frame gap**: one pull uses 72% of the budget, so
two back to back exceed it and the wrap resolves to the wrong second entirely, silently.

### 10.3 Inter-unit skew is real and stable, and not measurable by impulse

The two blocks' tick counters are independent MCU timers. Over a 238 s session their difference
is a **stable offset of 59 ticks (0.92 ms)** — identical median across the first and second
halves — with a central-98% spread of 80 ticks (1.25 ms), which is almost
exactly one sample period and is therefore the expected ±½-sample pairing jitter.

**⚠ Whether that 0.92 ms is true sampling skew or just arbitrary phase between two free-running
counters cannot be told from the counters alone.** It matters: at 1,000 °/s of wrist rotation,
0.92 ms is ~0.9° of error in the relative angle, which is the primary output.

**⚠ It cannot be resolved with a shared impulse, and attempting it wastes a session.** The
obvious experiment — rest both units on a rigid surface, strike the surface, compare when the
acceleration transient reaches each — was run and **cannot work in principle**. A tap is shorter
than the 1.25 ms sample period, so it lands as a *single* sample in each unit:

```
u0 accel through the transient:   8.9 → 55.9 → 164.3 → 31.2 → 17.8
```

One sample of rise and one of fall leaves no waveform to align, so cross-correlating the two
units has nothing to work with; across four taps the fitted lag scattered over tens of
milliseconds while the quantity sought is under one. Sampling is the floor here, not noise, and
more taps do not help.

Treat the offset as stable and calibratable but of unknown physical meaning, and
budget ~0.9° of relative-angle error at swing rates.

⚠ **And it is a session-median figure, not a per-record one.** Single-record tick differences
vary widely — the ±½-sample pairing jitter above dominates them — so a client must **not** read
the difference between one record's two tick counters as a skew measurement. Two consecutive
records of one capture read 89 and 99 ticks apart while the session median is 59.

`5e` (config bit 5 clear) **removes the tick counter entirely** (§6.3), losing both the fine
clock and any skew measurement. Prefer `7e` for anything timing-sensitive.

---

## 11. Client checklist

1. **Arm the scan first, then** prompt the user to press the button (§2.1). If the sensor has
   been asleep, tell them to press, pause, press again.
2. Connect. No pairing. The device vibrates when the link is up (§9.5).
3. Enable notifications on CCCD `0x0019`.
4. Bring-up per §9.1 — at minimum `80` for versions, since protocol version gates features.
   Send only the commands in §4; do not probe undocumented values (§4).
5. Start a 30 s `0x81` poll and keep it running for the whole session (§9.2).
6. `a0 01 7e` to stream. Decode per §6.3–6.4. Expect 25/100 Hz, never a fixed period.
7. Run calibration (§8.2) — mandatory every session, and **with the stream already open**:
   the device watches a continuous raise between the two markers. Do not stop the stream
   afterwards; the same one carries on.
8. Maintain the clock fit (§10) from the moment streaming starts.
9. For full-rate data, request a narrow history window promptly (§7.4) — **you do not have to
   stop the stream to do it** (§7.5), but the device still records nothing for as long as the
   pull takes, so keep it narrow and re-anchor the clock fit on the far side of it (§10, §10.2).
   The live stream's job is to be the clock reference;
   history is the data (§0, §10). **The full cycle is worked through in §7.6** — read that
   before designing the capture path, because the sequence is not obvious from the parts.
10. `83` to stop.

Throughout: treat clock-fit residual spread as a link-health signal (§10), and on a lost
connection decide whether the device dropped or slept before retrying (§9.6).

---

## 12. Limits and unknowns

**Hard limits.** A history buffer of ~7.5 s (§7.3 — measured once, and not known to be a fixed
duration), covering the current streaming session only, and holding ~100 Hz at rest rather than
the full internal rate (§7.3); a sample counter that wraps every 82.0 s; a five-minute idle
shutdown; one connection at a time; no recording of any kind unless a stream is open; and **none
while a retrieval is in flight either**, mid-stream or not, for as long as the retrieval takes
(§7.5). The device cannot record and replay at the same time, which is the one limit that shapes
the capture cycle in §7.6.

**Undecoded fields.** Two small ones, neither of which a client needs: two bytes of the
battery/status reply that drift slowly and are not millivolts (§5.3), and two bits of the stream
configuration byte, which the vendor app always sets (§6.2). Two entries have left this list:
**the calibration result**, all eight of whose quaternions are identified (§8.2), and **the
second sensor-map byte**, which is the second sensor's location code and was never a spare byte
(§5.4). What remains from the calibration result is the **status byte on its short form**: no
value of it has ever been captured, and the form itself has never been seen on the wire.

**The `0x90` stream frame is now fully accounted for.** Every byte of every block is identified,
and the record's own length rule explains the rest (§6.3, §6.4).

**Unobserved behaviour.** Whatever makes the live sample rate change (§6.6). The legacy frame
layout has likewise never appeared outside a deliberate legacy start.

**⚠ The command set is larger than the commands a client uses** (§4.1), and this document does
not enumerate the remainder. Treat undocumented values as unknown-and-possibly-destructive rather
than unused, and do not sweep the space to find out: one of them reboots the device into
firmware-update mode through the ordinary data characteristic, not through the update service a
client would know to avoid.

**Scope.** This document describes a local BLE peripheral and stops at the radio. It is not a
route to any online service, account system or API, and is not to be made into one.

**No magnetometer-aided mode is reachable.** The hardware has a magnetometer, but the start
command the vendor app uses switches it off, leaving the orientation fusion running on
accelerometer and gyroscope alone (§6.2). The legacy start command was the last candidate for a
magnetometer-aided mode, since it carries no configuration byte at all and should therefore run
whatever the firmware defaults to — but a stationary hold under it drifts at the same rate. So
**absolute heading has no reference under any configuration the device accepts, and it will
drift**: roughly 0.1–0.2° per minute per unit once the fusion has settled.

### ⚠ The two open questions that bear on a design

Everything else above is a gap in knowledge. These two are gaps that change what a client can
promise, and both are stated with their size so they can be designed around rather than
discovered.

**1. Host-clock alignment drifts, and the cause is not known** (§10). Measured against five
struck golf balls with a microphone reference: a consistent **2.2 ms per second of session**,
fitting session time with R² = 1.000. Three candidate causes were eliminated by direct
measurement — the sound card (−219 ppm), the host clock (NTP disciplining at 6 ppm), and bias
from pooling the fitted rate (which would track each fit's span, R² 0.79, not session time).

*Why it matters:* a consumer aligning against 240 fps video has a 4.2 ms budget, and this
consumes it in under two seconds of session. **A client must not promise sample-level alignment
on the strength of this document.** Measure it against an external reference in the target
environment, and treat the fit's own residuals as insufficient evidence — they measure link
jitter around the fit, not whether the fit is right.

**2. Inter-unit skew of 0.92 ms, of unknown physical meaning** (§10.3). Stable and repeatable,
worth ~0.9° in the relative angle at 1,000 °/s — which is the primary output. Whether it is real
sampling skew or arbitrary counter phase is unresolved, and **cannot** be resolved by impulse:
a tap is shorter than the sample period, so it lands one sample wide with no waveform to align.

*Why it matters:* being stable, it is subtractable as a constant, so a client can proceed
without knowing which it is — but it should carry the offset explicitly rather than silently
pair the two blocks as simultaneous.

**Open, and worth knowing.** Each unit's heading drifts on its own, but the wrist measurement is
the *relative* rotation between the two units, and most of that drift cancels between them: under
the configuration the vendor app uses, the relative angle stayed within a 0.58° range over five
minutes while the two units individually drifted 1.95° and 1.13°.

**Why the cancellation is so complete is not understood, and it is not universal.** Under the
legacy start command the same measurement ranged 2.75° over seven minutes — better than either
unit alone, but four times worse than under the normal configuration. Treat the cancellation as
an observed property of the configuration in §6.2 rather than something guaranteed by the
geometry. A client that changes configuration, or that runs sessions much longer than a few
minutes, should measure it again rather than assume it holds.
