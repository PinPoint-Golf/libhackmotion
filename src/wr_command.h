/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_command.h — internal.  Every byte the library can ever send.
 *
 * ⚠ THIS FILE IS THE ALLOWLIST.  api-request §2.16 asked that the set of bytes
 * that can leave be a short reviewable list rather than "whatever a caller
 * passes", and that the destructive command be refused BY CONSTRUCTION rather
 * than by documentation.  There is exactly one gate — wr_command_emit() — and
 * every encoder below goes through it.
 *
 * ⛔ 0xf0 reboots the device into firmware-update mode
 * through the ORDINARY data characteristic, so avoiding the OTA service is not
 * sufficient on its own (spec §2.3, §4.1).  It is not in the table and there is
 * no path that could put it there.
 *
 * ⚠ DO NOT SWEEP OR FUZZ THE COMMAND SPACE.  The vendor-library enumeration was
 * the safe way to find these; it cannot prove the firmware accepts nothing
 * else, and undocumented values are unknown-and-possibly-destructive rather
 * than unused (spec §12).  Fuzz the decoder instead — see wr_codec.h.
 */
#ifndef WR_COMMAND_H
#define WR_COMMAND_H

#include "wrist/session.h"
#include "wrist/config.h"

/* Spec §4. */
typedef enum wr_cmd_id {
    WR_CMD_VERSIONS      = 0x80,
    WR_CMD_STATUS        = 0x81, /* also THE KEEPALIVE — the reading is incidental */
    WR_CMD_LEGACY_START  = 0x82, /* reachable only through wr_stream_config_legacy() */
    WR_CMD_STOP_STREAM   = 0x83,
    WR_CMD_SENSOR_MAP    = 0x84,
    WR_CMD_MAC           = 0x85,
    WR_CMD_SERIAL        = 0x86,
    WR_CMD_START_STREAM  = 0xa0,
    WR_CMD_HISTORY       = 0xa1,
    WR_CMD_CALIBRATE     = 0xa2,
    WR_CMD_POWER_OFF     = 0xfa
} wr_cmd_id;

/*
 * The single gate.  Refuses anything not on the allowlist and anything longer
 * than WR_MAX_COMMAND_LEN.  Every encoder in this file calls it; nothing else
 * in the library constructs a wr_write_request.
 */
wr_status wr_command_emit(wr_write_request *out, const uint8_t *bytes, size_t length,
                          bool without_response);

/* Encoders.  Each returns WR_OK and fills `out`, or WR_ERR_INVALID_ARG. */
wr_status wr_cmd_versions(wr_write_request *out);
wr_status wr_cmd_status(wr_write_request *out);
wr_status wr_cmd_sensor_map(wr_write_request *out);
wr_status wr_cmd_mac(wr_write_request *out);
wr_status wr_cmd_serial(wr_write_request *out);
wr_status wr_cmd_start_stream(wr_write_request *out, wr_stream_config cfg);
wr_status wr_cmd_stop_stream(wr_write_request *out);
wr_status wr_cmd_history(wr_write_request *out, uint16_t first, uint16_t last);
wr_status wr_cmd_calibration_marker(wr_write_request *out, uint8_t pose /* 0 or 1 */);
wr_status wr_cmd_power_off(wr_write_request *out);

#endif /* WR_COMMAND_H */
