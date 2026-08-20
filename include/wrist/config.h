/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wrist/config.h — the stream configuration byte, as named flags.
 *
 * api-request §2.12: a raw uint8_t parameter invites exactly the experiment the
 * specification warns against, and the consequence — a differently shaped
 * payload silently misparsed — is not obvious at the call site.  Two of these
 * bits change the WIRE FORMAT, not just the content.
 */
#ifndef WRIST_CONFIG_H
#define WRIST_CONFIG_H

#include "wrist/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Spec §6.2.  The vendor app sends 0x7e on this hardware.
 * Named `mask` rather than `bit` because WR_CFG_NO_MAGNETOMETER covers two. */
typedef enum wr_config_mask {
    WR_CFG_ALT_HARDWARE_PATH = 0x01u, /* protocol >= 7; not set in 0x7e          */
    WR_CFG_NO_MAGNETOMETER   = 0x06u, /* fusion runs 6-DOF; set in 0x7e          */
    WR_CFG_UNDECODED_BIT3    = 0x08u, /* never sent clear; meaning unknown       */
    WR_CFG_UNDECODED_BIT4    = 0x10u, /* never sent clear; meaning unknown       */
    WR_CFG_TIMESTAMPS        = 0x20u, /* ⚠ CHANGES BLOCK SIZE: 22 B vs 20 B      */
    WR_CFG_EXTENDED_GYRO     = 0x40u  /* ⚠ CHANGES GYRO SCALE: /8 vs /16         */
} wr_config_mask;

/*
 * The only configuration this library sends by default, and the only one under
 * which the specification's headline property was measured:
 *
 *   Under 0x7e the RELATIVE angle stayed within 0.58° over 5 minutes while the
 *   two units individually drifted 1.95° and 1.13° (§6.2).  Under the legacy
 *   start the same measurement ranged 2.75° over 7 minutes.  The cancellation
 *   is an observed property of THIS configuration and is not guaranteed by the
 *   geometry — §12 is explicit about that.
 *
 * Since the relative rotation is the whole point of the device, anything other
 * than 0x7e is reachable only deliberately and marks its data as such.
 */
#define WR_CONFIG_OBSERVED_DEFAULT 0x7eu

#define WR_CONFIG_JUSTIFICATION_MAX 64

typedef struct wr_stream_config {
    uint8_t bits;
    /* The legacy `82` start takes no configuration byte at all, so it cannot be
     * represented by any value of `bits`.  It gets its own flag rather than a
     * sentinel byte that a real configuration could collide with. */
    uint8_t legacy;
    uint8_t reserved[2];
    /*
     * ⚠ Why a non-default configuration was chosen.  Empty for the default.
     *
     * It lives IN the type, not beside it, because that is the only way it
     * reaches the recording: wr_history_block carries a wr_stream_config by
     * value, so the string travels with the samples it describes and lands in
     * the capture's provenance for free.  A mandatory justification that got
     * discarded would be worse than none — it would read as an audit trail
     * without being one.
     */
    char justification[WR_CONFIG_JUSTIFICATION_MAX];
} wr_stream_config;

/* The one blessed constructor.  Returns { 0x7e }. */
WR_API wr_stream_config wr_stream_config_default(void);

/*
 * Any other configuration.  Deliberately awkward: it is not a constructor you
 * reach for by accident, it demands a written justification that is COPIED into
 * the returned config and therefore into the recording's provenance, and it
 * marks every resulting sample WR_SAMPLE_NONSTANDARD_CONFIG.
 *
 * A NULL or empty justification stores the literal "(unjustified)" rather than
 * failing — the function returns by value and cannot report an error, and a
 * recording that says nobody gave a reason is more useful than one that is
 * silent about it.
 *
 * Callers that only ever want the default never see this function.
 */
WR_API wr_stream_config wr_stream_config_nonstandard(uint8_t bits, const char *justification);

/* --- Wire-format consequences.  These are the reason this type exists. ---- */

/* Bit 5 set  → block 22 B, record 46 B, notification 47/93 B, ticks present.
 * Bit 5 clear→ block 20 B, record 42 B, notification 43/85 B, NO tick counter. */
WR_API bool   wr_stream_config_has_ticks(wr_stream_config cfg);
WR_API size_t wr_stream_config_block_size(wr_stream_config cfg);   /* 22 or 20 */
WR_API size_t wr_stream_config_record_size(wr_stream_config cfg);  /* 46 or 42 */

/* Bit 6 set → 8 LSB/°/s (±4096 °/s).  Clear → 16 LSB/°/s (±2048 °/s).
 * ⚠ With bit 6 clear BOTH a golf swing and a hand shake clip (§6.4). */
WR_API int wr_stream_config_gyro_divisor(wr_stream_config cfg);    /* 8 or 16  */
WR_API double wr_stream_config_gyro_full_scale_dps(wr_stream_config cfg);

/*
 * False when the resulting stream carries no device clock a host time can be
 * derived from.  Samples from such a stream are flagged WR_SAMPLE_NOT_TIME_ALIGNABLE
 * so a consumer cannot feed them into a path that assumes otherwise.
 *
 * Config 0x5e (bit 5 clear) removes the tick counters, losing the fine clock
 * and any inter-unit skew measurement (§10.3).  The legacy 0x82 start is worse
 * still — see wr_stream_config_is_legacy().
 */
WR_API bool wr_stream_config_is_time_alignable(wr_stream_config cfg);

WR_API bool wr_stream_config_is_observed_default(wr_stream_config cfg);

/*
 * The legacy path: the bare 0x82 start command, which takes no configuration
 * byte and yields 0x7f frames — no record header, no tick counters, 21-byte
 * blocks (§6.3.1).
 *
 * ⚠ It carries NO DEVICE CLOCK WHATSOEVER.  Timing can only come from arrival,
 * and HISTORY RETRIEVAL IS IMPOSSIBLE against it because `a1` addresses a
 * record header that does not exist.  The library decodes it — a recording may
 * contain one — but will not start one without wr_stream_config_nonstandard().
 */
WR_API wr_stream_config wr_stream_config_legacy(void);
WR_API bool wr_stream_config_is_legacy(wr_stream_config cfg);

/* Human-readable, e.g. "0x7e (default: 6-DOF, ticks, extended gyro)".
 * Writes at most `size` bytes including the terminator; returns `out`. */
#define WR_CONFIG_DESCRIBE_SIZE 96
WR_API char *wr_stream_config_describe(wr_stream_config cfg, char *out, size_t size);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WRIST_CONFIG_H */
