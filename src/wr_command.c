/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "wr_command.h"

#include <string.h>

/*
 * The allowlist, in one place, sorted.  Everything §4 documents as needed by a
 * client and nothing else.
 *
 * ⛔ The device accepts further commands that are deliberately absent from this
 *    table and are not enumerated anywhere in this project.  0xf0 is the one
 *    worth naming: it reboots into firmware-update mode through THIS
 *    characteristic, not through the OTA service a client would know to avoid.
 *    None of them has a path to the wire — wr_command_emit() is the only gate,
 *    and tests/test_command.c sweeps all 245 non-allowlisted values through it.
 */
static const uint8_t k_allowlist[] = {
    WR_CMD_VERSIONS,     /* 0x80 */
    WR_CMD_STATUS,       /* 0x81 */
    WR_CMD_LEGACY_START, /* 0x82 */
    WR_CMD_STOP_STREAM,  /* 0x83 */
    WR_CMD_SENSOR_MAP,   /* 0x84 */
    WR_CMD_MAC,          /* 0x85 */
    WR_CMD_SERIAL,       /* 0x86 */
    WR_CMD_START_STREAM, /* 0xa0 */
    WR_CMD_HISTORY,      /* 0xa1 */
    WR_CMD_CALIBRATE,    /* 0xa2 */
    WR_CMD_POWER_OFF     /* 0xfa */
};

bool wr_command_is_allowed(uint8_t command_byte)
{
    /* Belt and braces: the loop below would already refuse it, but a reader
     * scanning for "f0" should find it named and rejected. */
    if (command_byte == 0xf0u) {
        return false;
    }
    for (size_t i = 0; i < sizeof(k_allowlist); ++i) {
        if (k_allowlist[i] == command_byte) {
            return true;
        }
    }
    return false;
}

size_t wr_command_allowlist(uint8_t *out, size_t max)
{
    if (out != NULL) {
        size_t n = (max < sizeof(k_allowlist)) ? max : sizeof(k_allowlist);
        memcpy(out, k_allowlist, n);
    }
    return sizeof(k_allowlist);
}

wr_status wr_command_emit(wr_write_request *out, const uint8_t *bytes, size_t length,
                          bool without_response)
{
    if (out == NULL || bytes == NULL || length == 0u || length > WR_MAX_COMMAND_LEN) {
        return WR_ERR_INVALID_ARG;
    }
    if (!wr_command_is_allowed(bytes[0])) {
        return WR_ERR_NOT_ALLOWED;
    }
    memset(out, 0, sizeof(*out));
    memcpy(out->data, bytes, length);
    out->length = (uint8_t)length;
    out->without_response = without_response ? 1u : 0u;
    return WR_OK;
}

static wr_status emit1(wr_write_request *out, uint8_t id)
{
    uint8_t b[1];
    b[0] = id;
    return wr_command_emit(out, b, sizeof(b), false);
}

wr_status wr_cmd_versions(wr_write_request *out)   { return emit1(out, WR_CMD_VERSIONS); }
wr_status wr_cmd_status(wr_write_request *out)     { return emit1(out, WR_CMD_STATUS); }
wr_status wr_cmd_sensor_map(wr_write_request *out) { return emit1(out, WR_CMD_SENSOR_MAP); }
wr_status wr_cmd_mac(wr_write_request *out)        { return emit1(out, WR_CMD_MAC); }
wr_status wr_cmd_serial(wr_write_request *out)     { return emit1(out, WR_CMD_SERIAL); }
wr_status wr_cmd_stop_stream(wr_write_request *out){ return emit1(out, WR_CMD_STOP_STREAM); }

wr_status wr_cmd_power_off(wr_write_request *out)
{
    /* ⚠ No reply arrives and the link stays up ~9 s (§9.3).  The session must
     * not read that gap as failure and must not retry into it. */
    return emit1(out, WR_CMD_POWER_OFF);
}

wr_status wr_cmd_start_stream(wr_write_request *out, wr_stream_config cfg)
{
    if (wr_stream_config_is_legacy(cfg)) {
        /* The legacy start takes no configuration byte at all. */
        return emit1(out, WR_CMD_LEGACY_START);
    }
    {
        uint8_t b[3];
        b[0] = WR_CMD_START_STREAM;
        b[1] = 0x01u;
        b[2] = cfg.bits;
        return wr_command_emit(out, b, sizeof(b), false);
    }
}

wr_status wr_cmd_history(wr_write_request *out, uint16_t first, uint16_t last)
{
    uint8_t b[5];
    /* §7.1: `first` must be below `last`.  A reversed range is one of the seven
     * distinct causes that all return the same `d0 03`, so refusing it here
     * removes one thing the session would otherwise have to guess at. */
    if (first >= last) {
        return WR_ERR_INVALID_ARG;
    }
    b[0] = WR_CMD_HISTORY;
    b[1] = (uint8_t)(first >> 8);
    b[2] = (uint8_t)(first & 0xffu);
    b[3] = (uint8_t)(last >> 8);
    b[4] = (uint8_t)(last & 0xffu);
    return wr_command_emit(out, b, sizeof(b), false);
}

wr_status wr_cmd_calibration_marker(wr_write_request *out, uint8_t pose)
{
    uint8_t b[2];
    if (pose > 1u) {
        return WR_ERR_INVALID_ARG;
    }
    b[0] = WR_CMD_CALIBRATE;
    b[1] = pose;
    return wr_command_emit(out, b, sizeof(b), false);
}
