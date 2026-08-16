# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2026 Mark Liversedge
"""The `.hmwire` container, read and written by the library's own code.

⚠ THIS IS THE C IMPLEMENTATION, NOT A SECOND ONE.  `tools/hm_capture.py` writes
the container in Python and is pinned against the C reader by
`tests/test_capture_format.py`, because it predates the binding and is
deliberately kept independent.  Anything written through *this* module goes
through `record/hm_record.c`, so there is no second format to keep in step.

⚠ RECORD THE WIRE BYTES, NOT THE DECODED SAMPLES.  Undecoded fields remain — two
bytes of the battery reply, the second sensor-map byte, the calibration payload,
two configuration bits — and when one is settled a byte-level recording can be
RE-DECODED with the fix applied.  A sample-level one cannot: it already threw
away the bytes the fix would have read differently.
"""

from __future__ import annotations

import ctypes
from pathlib import Path
from typing import Iterator

from . import _types as T
from ._library import check, lib

__all__ = ["Recorder", "Replay", "recording_info_default"]


def recording_info_default() -> T.hm_recording_info:
    return lib.hm_recording_info_default()


class Recorder:
    """Appends wire chunks to a `.hmwire` file.

    Feed it whatever `Session.poll_wire()` returns.  ⚠ The session's wire ring
    is OFF unless you asked for one (`Session(wire_ring=...)`), and with it off
    `poll_wire()` is always empty — which is an absent recording, not an empty
    session."""

    __slots__ = ("_handle", "_path")

    def __init__(
        self,
        path: str | Path,
        *,
        device_id: str = "",
        config_bits: int = T.HM_CONFIG_OBSERVED_DEFAULT,
        record_identifiers: bool = False,
    ):
        info = lib.hm_recording_info_default()
        info.device_id = device_id.encode()[: T.HM_DEVICE_ID_MAX - 1]
        info.config_bits = config_bits
        # ⚠ Redaction is the default here as it is everywhere else: the MAC and
        # the serial name a specific unit and a specific owner.
        info.identifiers_recorded = record_identifiers

        handle = ctypes.c_void_p()
        check(
            lib.hm_recorder_open(
                str(path).encode(), ctypes.byref(info), ctypes.byref(handle)
            ),
            f"hm_recorder_open({path})",
        )
        self._handle = handle
        self._path = Path(path)

    def __enter__(self) -> "Recorder":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def path(self) -> Path:
        return self._path

    def write(self, chunks) -> None:
        chunks = list(chunks)
        if not chunks:
            return
        array = (T.hm_wire_chunk * len(chunks))(*chunks)
        check(lib.hm_recorder_write(self._handle, array, len(chunks)), "hm_recorder_write")

    @property
    def chunks(self) -> int:
        return lib.hm_recorder_chunks(self._handle)

    @property
    def bytes_written(self) -> int:
        return lib.hm_recorder_bytes(self._handle)

    def close(self) -> None:
        """⚠ Returns the first write error the recorder saw, so a full disk
        cannot end a capture quietly — raised here rather than swallowed."""
        if self._handle:
            handle, self._handle = self._handle, ctypes.c_void_p()
            check(lib.hm_recorder_close(handle), "hm_recorder_close")

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # pragma: no cover - interpreter teardown
            pass


class Replay:
    """Reads a `.hmwire` file back, byte-exact.

        with Replay("swings.hmwire") as replay:
            for chunk in replay:
                ...
    """

    __slots__ = ("_handle", "_path")

    def __init__(self, path: str | Path):
        handle = ctypes.c_void_p()
        check(lib.hm_replay_open(str(path).encode(), ctypes.byref(handle)),
              f"hm_replay_open({path})")
        self._handle = handle
        self._path = Path(path)

    def __enter__(self) -> "Replay":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def info(self) -> T.hm_recording_info:
        ptr = lib.hm_replay_info(self._handle)
        if not ptr:
            raise RuntimeError("replay has no header")
        return T.hm_recording_info.from_buffer_copy(ptr.contents)

    def __iter__(self) -> Iterator[T.hm_wire_chunk]:
        """⚠ Yields a COPY per chunk.  A shared buffer would be faster and would
        hand every caller a value that changes under them on the next step."""
        chunk = T.hm_wire_chunk()
        while True:
            status = lib.hm_replay_next(self._handle, ctypes.byref(chunk))
            if status == T.Status.DONE:
                return
            check(status, "hm_replay_next")
            yield T.hm_wire_chunk.from_buffer_copy(chunk)

    @property
    def chunks_read(self) -> int:
        return lib.hm_replay_chunks_read(self._handle)

    def close(self) -> None:
        if self._handle:
            handle, self._handle = self._handle, ctypes.c_void_p()
            lib.hm_replay_close(handle)

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # pragma: no cover - interpreter teardown
            pass
