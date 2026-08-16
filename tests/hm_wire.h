/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_wire.h — a builder for synthetic wire frames, so tests can express what
 * the device would send rather than a wall of hex.
 *
 * The hand-written golden vectors in fixtures/hm_fixtures.h check byte order
 * and field offsets against numbers computed by hand from the specification.
 * This builder checks everything above that, at scale.
 */
#ifndef HM_WIRE_H
#define HM_WIRE_H

#include <stdint.h>
#include <string.h>

typedef struct hm_wire_block {
    int16_t  q[4];      /* w, x, y, z — world → body */
    int16_t  accel[3];
    int16_t  gyro[3];
    uint16_t ticks;
} hm_wire_block;

static hm_wire_block hm_wire_identity_block(uint16_t ticks)
{
    hm_wire_block b;
    memset(&b, 0, sizeof(b));
    b.q[0] = 16384; /* w = 1.0 in Q14 */
    b.ticks = ticks;
    return b;
}

static void hm_wire_put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xffu);
}

/* Writes one 22-byte block (config with timestamps). */
static size_t hm_wire_write_block(uint8_t *p, const hm_wire_block *b, int with_ticks)
{
    size_t o = 0;
    for (int i = 0; i < 4; ++i) {
        hm_wire_put_be16(p + o, (uint16_t)b->q[i]);
        o += 2;
    }
    for (int i = 0; i < 3; ++i) {
        hm_wire_put_be16(p + o, (uint16_t)b->accel[i]);
        o += 2;
    }
    for (int i = 0; i < 3; ++i) {
        hm_wire_put_be16(p + o, (uint16_t)b->gyro[i]);
        o += 2;
    }
    if (with_ticks) {
        hm_wire_put_be16(p + o, b->ticks);
        o += 2;
    }
    return o;
}

/* One 46-byte record: u16be header, then lower-arm block, then palm block. */
static size_t hm_wire_write_record(uint8_t *p, uint16_t index, const hm_wire_block *lower_arm,
                                   const hm_wire_block *palm, int with_ticks)
{
    size_t o = 0;
    hm_wire_put_be16(p, index);
    o += 2;
    o += hm_wire_write_block(p + o, lower_arm, with_ticks);
    o += hm_wire_write_block(p + o, palm, with_ticks);
    return o;
}

/* A 0x90 notification carrying one record (47 bytes). */
static size_t hm_wire_notification1(uint8_t *out, uint16_t index,
                                    const hm_wire_block *lower_arm, const hm_wire_block *palm)
{
    out[0] = 0x90;
    return 1u + hm_wire_write_record(out + 1, index, lower_arm, palm, 1);
}

/* A 0x90 notification carrying two records (93 bytes). */
static size_t hm_wire_notification2(uint8_t *out, uint16_t index_a,
                                    const hm_wire_block *arm_a, const hm_wire_block *palm_a,
                                    uint16_t index_b, const hm_wire_block *arm_b,
                                    const hm_wire_block *palm_b)
{
    size_t o = 1;
    out[0] = 0x90;
    o += hm_wire_write_record(out + o, index_a, arm_a, palm_a, 1);
    o += hm_wire_write_record(out + o, index_b, arm_b, palm_b, 1);
    return o;
}

#endif /* HM_WIRE_H */
