<!-- SPDX-License-Identifier: MIT -->
<!-- Copyright (C) 2026 Mark Liversedge -->
# Contributing to libwrist

Read [`docs/design.md`](docs/design.md) first. It states not only what the
library does but why each decision was made, and most review comments are
answered there.

## ⛔ Never sweep or fuzz the device's command space

Command `f0` reboots the sensor into **firmware-update mode**, and it reaches
that mode through the *ordinary data characteristic* — the same pipe every other
command uses. Avoiding the OTA service is explicitly not sufficient on its own
(`docs/specification.md` §2.3, §4.1).

- The known command set was recovered by reading the vendor's binary. That was
  the safe way to find it. It **cannot** prove the firmware accepts nothing else.
- Treat undocumented command values as **unknown and possibly destructive**, not
  as unused (§12).
- Do not add a `sendRaw()`. Every write goes through `wr_command_emit()` and the
  allowlist in `src/wr_command.c`; `tests/test_command.c` sweeps all 256 byte
  values and will fail if that stops being true.
- **The capture harness holds no second copy of the allowlist.**
  `tools/wr_capture.py` asks the built library (`wrwire allowlist`) and refuses
  to run if it cannot. Do not give it a fallback: a second copy of a safety
  property is a second thing that can drift, and if the tool is unreachable the
  safe answer is to send nothing.
- **A binding must not become a second way to reach the wire.** `python/` has no
  `send()`, and the reason is structural rather than a rule anyone has to keep:
  the only bytes that exist came out of `wr_session_poll_writes()`, which the
  library composed against the allowlist. ⚠ A transport writes exactly those and
  synthesises nothing of its own — not a keepalive, not a bring-up step, not a
  "harmless" probe. Anything a transport needs to send, the session already
  emits; if it does not, that is a change to the session, in C, with a test.

**Fuzzing the decoder is a different activity and is welcome.** Point a fuzzer at
`wr_codec_decode()` under ASan and UBSan. That touches no hardware.
`wr_replay_next()` is the other good target: it is where a file's own length
field meets a fixed-size buffer, and it refuses rather than clamping.

## Things the library must never grow

Each of these would produce confidently wrong data with no error anywhere, which
is worse than not having the feature. They are refused by construction, and a PR
adding one will be declined regardless of how convenient it is.

| | Why |
|---|---|
| `saveCalibration` / `loadCalibration` / "reuse last session" | Calibration is lost by remounting, by power cycling **and by a plain disconnect** (§8.3). No client can safely inherit one. |
| A calibration quality score, or ranking two attempts | The one figure that looks like a score **inverts**: a no-raise calibration measured *best* (§8.2). A ranking built on it would systematically prefer the worst attempt. |
| An averaged or aggregate acceleration across the two units | They sit 3–8 cm apart and are *supposed* to differ by ω²r — 31–51 m/s² through a swing (§6.4). |
| Anatomical decomposition, or swing/phase detection | The application's job, and the device flags none of it (§0). |
| Decimation, resampling, smoothing or interpolation of history | Invisible in the artefact and unrecoverable from it (AR B13). |
| A hard-coded 800 Hz | ≈1 ms per second of error *and* it silently degenerates the clock estimator (§6.5, §10). |
| A clock read, a thread, a timer or a socket in the core | `tests/purity.cmake` will fail the build. |
| An attempt to resolve the inter-unit skew with a shared impulse | Tried; **cannot work in principle** — a tap is shorter than the sample period (§10.3). |

## Style

- C11, no compiler extensions. The build runs with `-Werror` and a wide warning
  set including `-Wconversion` and `-Wswitch-enum`; both are load-bearing.
- `clang-format` with the repository's `.clang-format`.
- Public API is `wr_` prefixed, snake_case, POD structs, opaque handles.
- A comment that repeats the code is noise. A comment that carries a *measured
  number and the section it came from* is the reason this library is safe to
  use — the specification's warnings belong next to the code that honours them.

## Tests

Every change to decode, timing or coverage needs a test that names the claim it
pins. Golden vectors in `tests/fixtures/` are **hand-computed** from the
specification; do not generate them from the code they test.

```sh
cmake -S . -B build/dev && cmake --build build/dev -j4 && ctest --test-dir build/dev
cmake -S . -B build/san -DHM_ENABLE_ASAN=ON -DHM_ENABLE_UBSAN=ON
cmake --build build/san -j4 && ctest --test-dir build/san
```

### Touching a public struct or enum

⚠ **Three places, not one.** `python/wrist/_types.py` declares every public
struct and enum a second time in ctypes, and `tools/wr_abi_table.c` is what
proves the two agree. Change a header and you change all three, or
`ctest -R python_abi` fails — which is the point.

⚠ **`wr_abi_check()` is not enough on its own and was never meant to be.** It
compares nine struct *sizes*; it says nothing about field offsets or enumerator
values. The four layered checks and, more usefully, **what each of them cannot
see** are documented at the top of `tools/wr_abi_table.c` and in design §4.6.1.
Read that before deciding a check is redundant.

### Python

The binding is ctypes over `libwrist_ffi`, and its tests are ordinary ctest
entries — nothing extra to install. `numpy` and `bleak` are optional and only
`Samples.numpy()` and the transport touch them.

⚠ **The sanitizer build runs the Python tests too, and what it buys is not the
obvious thing.** ASan instruments the code it compiled, so a *ctypes* read of
freed memory — a sample view kept past its block's release — returns plausible
data with no report. That hazard is covered by the guard in `Samples._alive()`
and by `tests/test_python_lifetime.py`. What ASan does catch is a fault the
library commits at the binding's request, such as a double release.

## Licensing of contributions

This project is MIT licensed. By submitting a patch you certify that you wrote
it, or otherwise have the right to submit it, and that you agree to it being
distributed under the MIT licence — the [Developer Certificate of
Origin](https://developercertificate.org/) states this precisely. Sign off your
commits with `git commit -s` to record that.

No contribution may include code copied from the vendor's application,
firmware, or any other party's source. See the Disclaimer in the README for why
that line matters here.
