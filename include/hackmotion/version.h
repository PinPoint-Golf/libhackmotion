/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#ifndef HACKMOTION_VERSION_H
#define HACKMOTION_VERSION_H

#include "hackmotion/types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HM_VERSION_MAJOR 0
#define HM_VERSION_MINOR 1
#define HM_VERSION_PATCH 0
#define HM_VERSION_STRING "0.1.0"

/*
 * The ABI version changes whenever a public struct's layout changes.  A binding
 * generated against one ABI must refuse to load a library reporting another —
 * hm_abi_check() does that comparison for it, including the struct sizes, which
 * is what actually goes wrong when a header and a shared object drift apart.
 */
#define HM_ABI_VERSION 1

HM_API const char *hm_version_string(void);
HM_API uint32_t hm_abi_version(void);

/* Size of each public POD struct, as the LIBRARY was built.  A binding compares
 * these against its own sizeof() and fails at load rather than at random. */
typedef struct hm_abi_sizes {
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
} hm_abi_sizes;

HM_API void hm_abi_sizes_get(hm_abi_sizes *out);

/* HM_OK if `expected` (filled in by the caller from its own headers) matches
 * this build; HM_ERR_NOT_SUPPORTED otherwise. */
HM_API hm_status hm_abi_check(const hm_abi_sizes *expected);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_VERSION_H */
