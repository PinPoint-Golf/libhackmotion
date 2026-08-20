# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""What a scanner needs, published as data — because the library does not scan.

api-request §2.0.5: every consumer that matters already owns a scanner, an
adapter pool and a device registry, and a second scanner inside the same process
contends with the first for one adapter.  So `device.h` publishes the matching
data and the HOST scans.  This module is that header, reachable from Python.

⚠ NOTHING HERE IS A COPY.  The four UUIDs are read out of the loaded shared
object with ``ctypes.Structure.in_dll`` — the same bytes the C consumer links
against — and the advertised-name match is the library's own
``hm_looks_like_hackmotion()``.  A transport built on this holds no second copy
of the device's identity, so there is nothing here that can drift from
`device.h`, which is the failure this project has had escape review three times.
"""

from __future__ import annotations

import ctypes
from typing import Iterable

from . import _types as T
from ._library import check, lib

__all__ = [
    "UUID_DATA_CHARACTERISTIC",
    "UUID_ISSC_PIPE_INERT",
    "UUID_OTA_SERVICE_FORBIDDEN",
    "UUID_TRANSPARENT_UART_SERVICE",
    "looks_like_hackmotion",
    "parse_uuid",
    "uuid_str",
]

_UUID_STRING_SIZE = 37  # HM_UUID_STRING_SIZE


def uuid_str(uuid: T.hm_uuid) -> str:
    """The canonical lower-case 8-4-4-4-12 form, rendered by the library."""
    buf = ctypes.create_string_buffer(_UUID_STRING_SIZE)
    lib.hm_uuid_format(ctypes.byref(uuid), buf, len(buf))
    return buf.value.decode()


def parse_uuid(text: str) -> T.hm_uuid:
    """⚠ The library's parser, not a second one.  Accepts the canonical form
    with or without braces, in any case, and refuses anything else — which is
    what makes a platform's UUID string comparable to the constants below
    without a normalisation step somebody has to remember."""
    out = T.hm_uuid()
    check(lib.hm_uuid_parse(text.encode(), ctypes.byref(out)), f"hm_uuid_parse({text!r})")
    return out


def _from_dll(symbol: str) -> T.hm_uuid:
    return T.hm_uuid.in_dll(lib, symbol)


# --- GATT, spec §2.3 --------------------------------------------------------
#
# The whole protocol is a byte stream inside ONE bidirectional characteristic.
# ⚠ It is not a characteristic per quantity, and the data characteristic does
# NOT sit under the service's UUID base — a transport that derives one from the
# other by fragment substitution cannot express this device at all (AR §2.0.5).
# Resolve them independently.
UUID_TRANSPARENT_UART_SERVICE = _from_dll("HM_UUID_TRANSPARENT_UART_SERVICE")
UUID_DATA_CHARACTERISTIC = _from_dll("HM_UUID_DATA_CHARACTERISTIC")

# ⛔ NEVER WRITE TO THIS.  The Microchip OTA/DFU service is the firmware update
# path and a bad write bricks the device (spec §2.3).  It is published so a
# transport can refuse it BY CONSTRUCTION rather than by documentation
# (AR §2.16) — see `BleakTransport._resolve_characteristic`.
UUID_OTA_SERVICE_FORBIDDEN = _from_dll("HM_UUID_OTA_SERVICE_FORBIDDEN")

# Inert: the stock ISSC pipe characteristic accepts writes and never replies.
# Published so a transport that enumerates it first does not pick it.
UUID_ISSC_PIPE_INERT = _from_dll("HM_UUID_ISSC_PIPE_INERT")


def looks_like_hackmotion(
    local_name: str | None,
    advertised_services: Iterable[str | T.hm_uuid] | None = None,
) -> bool:
    """True if an advertisement looks like a HackMotion wrist sensor.

    ⚠ ANY GENERATION, not only the wG3 the specification was measured on:
    "wG3" is wrist, generation 3, and the match is on the family prefix so a
    later sensor is still discoverable.  Being found is not being supported —
    the link-up checks settle that.

    ⚠ THE MATCH IS THE LIBRARY'S, so the advertised name lives in exactly one
    place in this project.  `local_name` may be None.  `advertised_services` may
    be empty — many stacks report no service UUIDs in an advertisement at all,
    so a name match alone is accepted.

    This is a filter for a scanner the HOST runs.  It performs no I/O.

    ⚠ A service UUID this library cannot parse is SKIPPED rather than raised on.
    Advertised UUIDs only ever add evidence — the name alone is already a match —
    and a scanner that throws on one odd advertisement stops seeing the sensor
    because of a device that has nothing to do with it.
    """
    uuids: list[T.hm_uuid] = []
    for entry in advertised_services or ():
        if isinstance(entry, T.hm_uuid):
            uuids.append(entry)
            continue
        try:
            uuids.append(parse_uuid(entry))
        except Exception:  # noqa: BLE001 — an unparseable UUID is not evidence
            continue
    array = (T.hm_uuid * len(uuids))(*uuids) if uuids else None
    return bool(
        lib.hm_looks_like_hackmotion(
            local_name.encode() if local_name else None, array, len(uuids)
        )
    )
