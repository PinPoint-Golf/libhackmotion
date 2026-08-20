/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_command.h — internal.  Every byte the library can ever send.
 *
 * ⚠ THIS FILE IS THE ALLOWLIST.  api-request §2.16 asked that the set of bytes
 * that can leave be a short reviewable list rather than "whatever a caller
 * passes", and that the destructive command be refused BY CONSTRUCTION rather
 * than by documentation.  There is exactly one gate — hm_command_emit() — and
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
 * than unused (spec §12).  Fuzz the decoder instead — see hm_codec.h.
 */
#ifndef HM_COMMAND_H
#define HM_COMMAND_H

#include "hackmotion/session.h"
#include "hackmotion/config.h"

/* Spec §4. */
typedef enum hm_cmd_id {
    HM_CMD_VERSIONS      = 0x80,
    HM_CMD_STATUS        = 0x81, /* also THE KEEPALIVE — the reading is incidental */
    HM_CMD_LEGACY_START  = 0x82, /* reachable only through hm_stream_config_legacy() */
    HM_CMD_STOP_STREAM   = 0x83,
    HM_CMD_SENSOR_MAP    = 0x84,
    HM_CMD_MAC           = 0x85,
    HM_CMD_SERIAL        = 0x86,
    HM_CMD_START_STREAM  = 0xa0,
    HM_CMD_HISTORY       = 0xa1,
    HM_CMD_CALIBRATE     = 0xa2,
    HM_CMD_POWER_OFF     = 0xfa
} hm_cmd_id;

/*
 * The single gate.  Refuses anything not on the allowlist and anything longer
 * than HM_MAX_COMMAND_LEN.  Every encoder in this file calls it; nothing else
 * in the library constructs a hm_write_request.
 */
hm_status hm_command_emit(hm_write_request *out, const uint8_t *bytes, size_t length,
                          bool without_response);

/* Encoders.  Each returns HM_OK and fills `out`, or HM_ERR_INVALID_ARG. */
hm_status hm_cmd_versions(hm_write_request *out);
hm_status hm_cmd_status(hm_write_request *out);
hm_status hm_cmd_sensor_map(hm_write_request *out);
hm_status hm_cmd_mac(hm_write_request *out);
hm_status hm_cmd_serial(hm_write_request *out);
hm_status hm_cmd_start_stream(hm_write_request *out, hm_stream_config cfg);
hm_status hm_cmd_stop_stream(hm_write_request *out);
hm_status hm_cmd_history(hm_write_request *out, uint16_t first, uint16_t last);
hm_status hm_cmd_calibration_marker(hm_write_request *out, uint8_t pose /* 0 or 1 */);
hm_status hm_cmd_power_off(hm_write_request *out);

#endif /* HM_COMMAND_H */
