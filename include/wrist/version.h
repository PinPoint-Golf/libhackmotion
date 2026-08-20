/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#ifndef WRIST_VERSION_H
#define WRIST_VERSION_H

#include "wrist/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WR_VERSION_MAJOR 0
#define WR_VERSION_MINOR 2
#define WR_VERSION_PATCH 0
#define WR_VERSION_STRING "0.2.0"

/*
 * The ABI version changes whenever a public struct's layout changes.  A binding
 * generated against one ABI must refuse to load a library reporting another —
 * wr_abi_check() does that comparison for it, including the struct sizes, which
 * is what actually goes wrong when a header and a shared object drift apart.
 */
#define WR_ABI_VERSION 1

WR_API const char *wr_version_string(void);
WR_API uint32_t wr_abi_version(void);

/* Size of each public POD struct, as the LIBRARY was built.  A binding compares
 * these against its own sizeof() and fails at load rather than at random. */
typedef struct wr_abi_sizes {
    uint32_t abi_version;
    uint32_t sample;
    uint32_t unit_sample;
    uint32_t event;
    uint32_t clock_snapshot;
    uint32_t history_block;
    uint32_t write_request;
    uint32_t wire_chunk;
    uint32_t session_config;
    uint32_t sample_layout_version;
} wr_abi_sizes;

WR_API void wr_abi_sizes_get(wr_abi_sizes *out);

/* WR_OK if `expected` (filled in by the caller from its own headers) matches
 * this build; WR_ERR_NOT_SUPPORTED otherwise. */
WR_API wr_status wr_abi_check(const wr_abi_sizes *expected);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WRIST_VERSION_H */
