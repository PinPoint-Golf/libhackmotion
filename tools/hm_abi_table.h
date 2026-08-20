/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_abi_table.h — the public structs' layout, as the compiler built them.
 *
 * ⚠ WHY THIS EXISTS.  hm_abi_check() compares NINE STRUCT SIZES
 * (include/hackmotion/version.h) and says nothing whatever about where the
 * fields inside them sit.  A language binding that declares `skew_us` one slot
 * out of place passes that check, decodes every sample, and returns plausible
 * numbers with no error anywhere — which is the exact failure shape that has
 * escaped review in this project three times (R21 and the two in the
 * pre-implementation sweep).
 *
 * So the binding's layout is not trusted and is not hand-verified: it is pinned
 * against this table, which comes from `offsetof` in the same translation unit
 * the headers describe.  tests/test_python_abi.py is the comparison.
 *
 * This is a DEVELOPMENT tool.  Nothing at runtime reads it — a shipped binding
 * still guards itself with hm_abi_check() at load.
 */
#ifndef HM_ABI_TABLE_H
#define HM_ABI_TABLE_H

#include <stdio.h>

/* Emits the table as JSON on `out`.  Returns the number of self-check problems
 * found — see hm_abi_selfcheck() — which is 0 for a healthy build. */
int hm_abi_print_json(FILE *out, FILE *err);

#endif /* HM_ABI_TABLE_H */
