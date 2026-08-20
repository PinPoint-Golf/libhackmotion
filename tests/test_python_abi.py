#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Mark Liversedge
"""test_python_abi.py — the ctypes layout against the compiler's.

⚠ WHY THIS TEST EXISTS.  python/wrist/_types.py declares every public
struct and enum a SECOND time.  That is the same shape of bug this project has
had escape review three times: one side changes, the other is left stale, and
the result is a plausible answer with no alarm.

wr_abi_check() does not close it.  It compares NINE STRUCT SIZES and says
nothing about where the fields inside them sit, or about any enum at all.  A
`skew_us` one slot out decodes every sample into plausible nonsense.

So the binding is pinned against `wrwire abi`, which reads sizes, offsets and
enumerator values straight out of the compiler.  ⚠ It writes this test's
authority down honestly: see tools/wr_abi_table.c for the four checks and the
one shape all four miss.

This test paid for itself on the day it was written: the hand-written Python had
wr_gap_kind's first two members THE WRONG WAY ROUND and four of eleven history
statuses on the wrong numbers.  Both read as perfectly ordinary code.

Usage: test_python_abi.py <path to the built wrwire> [<path to the .so>]
"""

import ctypes
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

failures = 0
checks = 0


def check(condition, what):
    global failures, checks
    checks += 1
    if condition:
        print(f"[ ok ] {what}")
    else:
        print(f"[FAIL] {what}")
        failures += 1


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2

    wrwire = sys.argv[1]
    if len(sys.argv) > 2:
        # ⚠ The test must pin the layout of the object the binding will actually
        # load, not whichever build happens to be found first on this machine.
        import os

        os.environ["WRIST_LIBRARY"] = sys.argv[2]

    out = subprocess.run([wrwire, "abi"], capture_output=True, text=True)
    if out.returncode != 0:
        print(out.stderr, file=sys.stderr)
        print("⚠ `wrwire abi` FAILED — the C table is out of step with the headers.")
        print("  NOTHING WAS CHECKED.")
        return 1
    table = json.loads(out.stdout)

    from wrist import _types as T

    # --- the loader's own guard ------------------------------------------
    # ⚠ Importing at all runs wr_abi_check().  If that raised we would never
    # reach here, so this line is not the check — it is the record that the
    # check ran.
    import wrist

    check(
        wrist.ABI_VERSION == table["abi_version"],
        f"ABI version agrees ({wrist.ABI_VERSION})",
    )

    # ------------------------------------------------------------------
    # Structs
    # ------------------------------------------------------------------
    c_structs = table["structs"]
    py_structs = T.PINNED_STRUCTS

    # ⚠ Both directions.  A struct in the C table with no ctypes counterpart is
    # a struct nothing compares; a ctypes struct with no C row is a layout
    # nobody checked against the compiler.  Neither is allowed to be quiet.
    missing_in_py = sorted(set(c_structs) - set(py_structs))
    missing_in_c = sorted(set(py_structs) - set(c_structs))
    check(not missing_in_py, f"every C struct is mirrored in ctypes{_tail(missing_in_py)}")
    check(not missing_in_c, f"every ctypes struct has a C row{_tail(missing_in_c)}")

    # ⚠ And that PINNED_STRUCTS really covers the module, so a Structure added
    # to _types.py and left out of the dict is not silently unchecked.
    declared = {
        name
        for name, obj in vars(T).items()
        if isinstance(obj, type)
        and issubclass(obj, (ctypes.Structure, ctypes.Union))
        and obj.__module__ == T.__name__
        and not name.startswith("wr_event_payload")  # a member of wr_event, pinned via `u`
    }
    check(
        declared <= set(py_structs),
        f"PINNED_STRUCTS covers every struct in _types.py"
        f"{_tail(sorted(declared - set(py_structs)))}",
    )

    for name in sorted(set(c_structs) & set(py_structs)):
        _check_struct(name, c_structs[name], py_structs[name])

    # ------------------------------------------------------------------
    # Enums
    # ------------------------------------------------------------------
    c_enums = table["enums"]
    mirrored = {c_name for c_name, _ in T.PINNED_ENUMS.values()}

    # ⚠ Reported, not asserted: an enum the C table dumps and Python does not
    # mirror is a deliberate choice (wr_warning_code is rendered through the
    # library's own name function rather than copied).  It is printed so the
    # choice stays visible rather than becoming an oversight nobody sees.
    unmirrored = sorted(set(c_enums) - mirrored)
    if unmirrored:
        print(f"[note] not mirrored in Python, by choice: {', '.join(unmirrored)}")

    for py_enum, (c_name, prefix) in T.PINNED_ENUMS.items():
        _check_enum(py_enum, c_name, prefix, c_enums)

    print(f"\n{checks - failures}/{checks} checks passed")
    if failures:
        print(
            "\n⚠ The binding and the headers disagree.  Fix python/wrist/_types.py\n"
            "  — the compiler is right and the transcription is not."
        )
        return 1
    return 0


def _tail(items):
    return f": {', '.join(items)}" if items else ""


def _check_struct(name, c_struct, py_struct):
    c_size = c_struct["size"]
    py_size = ctypes.sizeof(py_struct)
    # ⚠ THE SIZE IS THE CHECK THAT ACTUALLY CATCHES A FORGOTTEN FIELD, because a
    # field added to a header changes sizeof even when the C table's own tiling
    # check cannot see it (tools/wr_abi_table.c).
    check(c_size == py_size, f"{name}: sizeof {py_size} == {c_size}")

    c_fields = {f["name"]: f for f in c_struct["fields"]}
    py_fields = {f[0]: f for f in py_struct._fields_}

    extra_py = sorted(set(py_fields) - set(c_fields))
    extra_c = sorted(set(c_fields) - set(py_fields))
    check(not extra_py, f"{name}: no ctypes field is absent from C{_tail(extra_py)}")
    check(not extra_c, f"{name}: no C field is absent from ctypes{_tail(extra_c)}")

    for field in sorted(set(c_fields) & set(py_fields)):
        descriptor = getattr(py_struct, field)
        c_field = c_fields[field]
        ok = descriptor.offset == c_field["offset"] and descriptor.size == c_field["size"]
        check(
            ok,
            f"{name}.{field}: offset {descriptor.offset}/{c_field['offset']}, "
            f"size {descriptor.size}/{c_field['size']}",
        )


def _check_enum(py_enum, c_name, prefix, c_enums):
    if c_name not in c_enums:
        check(False, f"{py_enum.__name__}: `{c_name}` is not in the C enum table")
        return

    c_members = {}
    for enumerator, value in c_enums[c_name].items():
        if not enumerator.startswith(prefix):
            check(False, f"{c_name}: `{enumerator}` does not start with `{prefix}`")
            return
        c_members[enumerator[len(prefix):]] = value

    # ⚠ `__members__`, never iteration.  Iterating an IntFlag yields only the
    # canonical single-bit members, so WR_CFG_NO_MAGNETOMETER — which covers two
    # bits, deliberately — would vanish from the comparison and read as agreement.
    py_members = {name: int(member.value) for name, member in py_enum.__members__.items()}

    extra_py = sorted(set(py_members) - set(c_members))
    extra_c = sorted(set(c_members) - set(py_members))
    check(not extra_py, f"{py_enum.__name__}: no member absent from C{_tail(extra_py)}")
    check(not extra_c, f"{py_enum.__name__}: no C member absent{_tail(extra_c)}")

    wrong = [
        f"{k}={py_members[k]} vs {c_members[k]}"
        for k in sorted(set(py_members) & set(c_members))
        if py_members[k] != c_members[k]
    ]
    check(not wrong, f"{py_enum.__name__}: every value matches{_tail(wrong)}")


if __name__ == "__main__":
    sys.exit(main())
