/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "wrist/config.h"

#include <stdio.h>
#include <string.h>

wr_stream_config wr_stream_config_default(void)
{
    wr_stream_config cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.bits = WR_CONFIG_OBSERVED_DEFAULT;
    return cfg;
}

wr_stream_config wr_stream_config_nonstandard(uint8_t bits, const char *justification)
{
    wr_stream_config cfg;
    const char *text;
    size_t n;

    memset(&cfg, 0, sizeof(cfg));
    cfg.bits = bits;

    /*
     * A configuration other than 0x7e is a decision somebody made, and the
     * recording should say why.  The string is COPIED into the config, which
     * wr_history_block carries by value — so it reaches the capture's
     * provenance without anything else having to route it.
     */
    text = (justification != NULL && justification[0] != '\0') ? justification : "(unjustified)";
    n = strlen(text);
    if (n > sizeof(cfg.justification) - 1u) {
        n = sizeof(cfg.justification) - 1u;
    }
    memcpy(cfg.justification, text, n);
    return cfg;
}

wr_stream_config wr_stream_config_legacy(void)
{
    wr_stream_config cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.legacy = 1;
    return cfg;
}

bool wr_stream_config_is_legacy(wr_stream_config cfg)
{
    return cfg.legacy != 0;
}

bool wr_stream_config_has_ticks(wr_stream_config cfg)
{
    if (cfg.legacy) {
        return false; /* §6.3.1 — no record header and no tick counter at all */
    }
    return (cfg.bits & (uint8_t)WR_CFG_TIMESTAMPS) != 0;
}

size_t wr_stream_config_block_size(wr_stream_config cfg)
{
    if (cfg.legacy) {
        return 21u; /* quaternion 8 + accel 6 + gyro 6 + one trailing byte */
    }
    return wr_stream_config_has_ticks(cfg) ? 22u : 20u;
}

size_t wr_stream_config_record_size(wr_stream_config cfg)
{
    if (cfg.legacy) {
        return 42u; /* two 21-byte blocks, NO header */
    }
    return 2u * wr_stream_config_block_size(cfg) + 2u; /* + u16be sample counter */
}

int wr_stream_config_gyro_divisor(wr_stream_config cfg)
{
    if (cfg.legacy) {
        /* ⚠ NOT ESTABLISHED, and it cannot be from the data alone (§6.3.1).
         * ~100 Hz at the nominal 16 LSB/°/s is the consistent reading; ~200 Hz
         * at 8 fits equally well and corresponds to no rate the device is
         * otherwise known to use.  Rate and scale are only recoverable given
         * each other, and the legacy layout carries no clock to measure
         * against. */
        return 16;
    }
    return (cfg.bits & (uint8_t)WR_CFG_EXTENDED_GYRO) ? 8 : 16;
}

double wr_stream_config_gyro_full_scale_dps(wr_stream_config cfg)
{
    return 32768.0 / (double)wr_stream_config_gyro_divisor(cfg);
}

bool wr_stream_config_is_time_alignable(wr_stream_config cfg)
{
    /*
     * The legacy layout carries no record header, so `a1` has nothing to
     * address and timing can only come from arrival.  Without the tick counter
     * there is no fine clock and no inter-unit skew, but the record header
     * still supports the fit and history retrieval — so 0x5e is degraded, not
     * unalignable.
     */
    return !cfg.legacy;
}

bool wr_stream_config_is_observed_default(wr_stream_config cfg)
{
    return !cfg.legacy && cfg.bits == WR_CONFIG_OBSERVED_DEFAULT;
}

char *wr_stream_config_describe(wr_stream_config cfg, char *out, size_t size)
{
    if (out == NULL || size == 0) {
        return out;
    }
    if (cfg.legacy) {
        (void)snprintf(out, size, "legacy 0x82 start (no clock, no history)");
        return out;
    }
    (void)snprintf(out, size, "0x%02x (%s%s%s%s%s)", cfg.bits,
                   wr_stream_config_is_observed_default(cfg) ? "default: " : "NONSTANDARD: ",
                   (cfg.bits & (uint8_t)WR_CFG_NO_MAGNETOMETER) ? "6-DOF" : "9-DOF?",
                   wr_stream_config_has_ticks(cfg) ? ", ticks" : ", NO TICKS",
                   (cfg.bits & (uint8_t)WR_CFG_EXTENDED_GYRO) ? ", extended gyro"
                                                             : ", HALVED GYRO RANGE",
                   (cfg.bits & (uint8_t)WR_CFG_ALT_HARDWARE_PATH) ? ", alt hw path" : "");
    return out;
}
