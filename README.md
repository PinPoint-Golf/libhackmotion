<!-- SPDX-License-Identifier: LGPL-2.1-or-later -->
<!-- Copyright (C) 2026 Mark Liversedge -->
# libhackmotion

A **C11 library for integrating with HackMotion wG3 wrist sensors** over
Bluetooth Low Energy, with **Python bindings** for driving one from Python.

The library implements the sensor's protocol: framing and decode, session
sequencing, calibration, the device→host clock fit, and full-rate retrieval of
the sensor's on-board history buffer.

It is **sans-I/O** — it owns no radio, thread, timer or clock. Your code supplies
bytes and timestamps; the library returns commands, samples and events. That lets
it sit alongside a BLE stack you already use, and lets it be tested without a
radio or a sensor.

## Building

A C11 compiler and CMake ≥ 3.16. No dependencies.

```sh
cmake -S . -B build/dev -DCMAKE_BUILD_TYPE=Debug
cmake --build build/dev -j4
ctest --test-dir build/dev --output-on-failure
```

Presets `dev`, `san`, `cov` and `rel` wrap the usual configurations.

### Embedding

The library is meant to be embedded, so **adding it to your project changes nothing
about your project** — not its build type, not its test registry, not its install
manifest. Drop it in and link:

```cmake
add_subdirectory(libhackmotion)        # or FetchContent_MakeAvailable(hackmotion)
target_link_libraries(your_app PRIVATE hackmotion)
```

`#include <hackmotion/hackmotion.h>` then resolves through the link — the include
directory is `PUBLIC` on the target.

You get the library and nothing else. The tests, the command line tools, the FFI
shared object, `-Werror` and the install rules all default to ON when this is the
top-level project and OFF when it is not, so there is no list of options to switch
off first. Each is still an ordinary option if you want it: `HM_BUILD_TESTS`,
`HM_BUILD_TOOLS`, `HM_BUILD_FFI`, `HM_WERROR`, `HM_INSTALL`.

`HM_BUILD_RECORD` is the exception and defaults ON either way — `hackmotion_record`
is a library target rather than developer tooling, and linking the `.hmwire`
container is an ordinary thing for a consumer to want. It needs the core's internal
symbols, so it is available in a static build only.

Static or shared is `BUILD_SHARED_LIBS` as usual.

## Python

The build also produces `libhackmotion_ffi`, which the binding loads. There is
nothing further to install:

```sh
PYTHONPATH=python python3 -c "import hackmotion; print(hackmotion.VERSION)"
```

An optional [`bleak`](https://github.com/hbldh/bleak) transport drives a real
sensor on Linux, macOS and Windows. `tools/hm_bench.py` is a complete session
written against it — connect, stream, retrieve, record — and is meant to be
copied:

```sh
pip install bleak
./tools/hm_bench.py --out bench.hmwire --duration 90
```

`bleak` is required only for that transport; `import hackmotion` neither imports
nor needs it. See [`python/README.md`](python/README.md).

## Safety

⛔ **Do not sweep or fuzz the device's command space.** Command `f0` reboots the
sensor into firmware-update mode, and it reaches that mode through the *ordinary
data characteristic* — avoiding the OTA service is not sufficient on its own.

The bytes this library can emit are a short allowlist in `src/hm_command.c`,
enforced by a single gate. There is no `sendRaw()` and there will not be one.

Fuzzing the **decoder** is a different activity and is encouraged.

## Documentation

- [`docs/design.md`](docs/design.md) — how the library works, and why.
- [`docs/specification.md`](docs/specification.md) — the wire protocol itself.

## Disclaimer

This project is an independent, unofficial work. It is not affiliated
with, authorised, endorsed, sponsored or supported by HackMotion or any
of its subsidiaries or affiliates.

### Scope: the device only

This library communicates with the HackMotion **sensor hardware** over
Bluetooth Low Energy, and nothing else. To be explicit about what it
does not do:

- It does **not** interoperate with, extend, modify, replace, emulate or
  interfere with the HackMotion mobile application in any way.
- It does **not** connect to, authenticate against, query, scrape or
  otherwise interact with HackMotion's cloud services, servers, APIs or
  online accounts.
- It does **not** access, transmit, retrieve or store any HackMotion
  account, subscription, licence or user data.
- It does **not** contain, reproduce or redistribute any HackMotion
  firmware, application code, or other proprietary or confidential
  material. This library is an independent implementation of the
  protocol as observed.
- It does **not** unlock, bypass or circumvent any paid feature,
  subscription tier or access control.

Its sole purpose is to allow independent software to read sensor data
from a device the user already owns, so that the data can be used in an
application of their own choosing. No part of HackMotion's software
ecosystem is a target, a dependency, or a point of contact.

### How it was developed

The protocol was determined independently, on hardware owned by the
author, for the sole purpose of achieving interoperability between the
sensor and independent software.

Almost all of it was established by observing the Bluetooth Low Energy
traffic between the sensor and its host, decoding those recordings
offline against known motion, and then reproducing each behaviour from a
Linux machine with the vendor's application closed. Every behavioural
claim in [`docs/specification.md`](docs/specification.md) rests on a
measurement that can be repeated that way.

The set of commands the sensor accepts was additionally established by
examining the vendor's own application. That was a safety decision: the
only safe way to learn that a particular command reboots the sensor into
firmware-update mode is to read it rather than to send it, which is why
this library ships a fixed allowlist and refuses every other value by
construction. Examining a program to obtain the information necessary
for interoperability is expressly permitted by Article 6 of EU Directive
2009/24/EC, with comparable allowances elsewhere. No HackMotion code was
copied, translated or included in this project.

### Trademarks

"HackMotion" and any associated logos or product names are trademarks of
their respective owners. They are used here solely to identify the
hardware with which this software is designed to interoperate, as
permitted by nominative fair use. Their use does not imply any
association, sponsorship or endorsement.

### Warranty

This software is provided "as is", without warranty of any kind. It may
stop working at any time — for example following a firmware update — and
use is entirely at your own risk. The author accepts no liability for
any damage to hardware, loss of data, or effect on any warranty or terms
of service applicable to your device.

Please do not contact HackMotion for support with this library. Issues
should be raised in this repository.

## Licence

**GNU Lesser General Public License, version 2.1 or later** — full text in
[`LICENSE`](LICENSE), and every source file carries an
`SPDX-License-Identifier: LGPL-2.1-or-later` line.

The LGPL suits a library meant to be embedded: an application may link it,
statically or dynamically and whatever its own licence, provided the recipient
can relink against a modified version. Changes *to this library* are what must be
shared back.
