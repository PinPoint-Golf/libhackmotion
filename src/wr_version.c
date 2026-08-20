/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "wrist/version.h"
#include "wrist/wrist.h"

#include <string.h>

const char *wr_version_string(void)
{
    return WR_VERSION_STRING;
}

uint32_t wr_abi_version(void)
{
    return WR_ABI_VERSION;
}

void wr_abi_sizes_get(wr_abi_sizes *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->abi_version = WR_ABI_VERSION;
    out->sample = (uint32_t)sizeof(wr_sample);
    out->unit_sample = (uint32_t)sizeof(wr_unit_sample);
    out->event = (uint32_t)sizeof(wr_event);
    out->clock_snapshot = (uint32_t)sizeof(wr_clock_snapshot);
    out->history_block = (uint32_t)sizeof(wr_history_block);
    out->write_request = (uint32_t)sizeof(wr_write_request);
    out->wire_chunk = (uint32_t)sizeof(wr_wire_chunk);
    out->session_config = (uint32_t)sizeof(wr_session_config);
    out->sample_layout_version = WR_SAMPLE_LAYOUT_VERSION;
}

wr_status wr_abi_check(const wr_abi_sizes *expected)
{
    wr_abi_sizes actual;

    if (expected == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    wr_abi_sizes_get(&actual);
    if (memcmp(&actual, expected, sizeof(actual)) != 0) {
        return WR_ERR_NOT_SUPPORTED;
    }
    return WR_OK;
}
