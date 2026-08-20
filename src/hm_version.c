/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "hackmotion/version.h"
#include "hackmotion/hackmotion.h"

#include <string.h>

const char *hm_version_string(void)
{
    return HM_VERSION_STRING;
}

uint32_t hm_abi_version(void)
{
    return HM_ABI_VERSION;
}

void hm_abi_sizes_get(hm_abi_sizes *out)
{
    if (out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));
    out->abi_version = HM_ABI_VERSION;
    out->sample = (uint32_t)sizeof(hm_sample);
    out->unit_sample = (uint32_t)sizeof(hm_unit_sample);
    out->event = (uint32_t)sizeof(hm_event);
    out->clock_snapshot = (uint32_t)sizeof(hm_clock_snapshot);
    out->history_block = (uint32_t)sizeof(hm_history_block);
    out->write_request = (uint32_t)sizeof(hm_write_request);
    out->wire_chunk = (uint32_t)sizeof(hm_wire_chunk);
    out->session_config = (uint32_t)sizeof(hm_session_config);
    out->sample_layout_version = HM_SAMPLE_LAYOUT_VERSION;
}

hm_status hm_abi_check(const hm_abi_sizes *expected)
{
    hm_abi_sizes actual;

    if (expected == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    hm_abi_sizes_get(&actual);
    if (memcmp(&actual, expected, sizeof(actual)) != 0) {
        return HM_ERR_NOT_SUPPORTED;
    }
    return HM_OK;
}
