/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_session.c — the session core against a fake transport.
 *
 * ⚠ There is no radio here and there does not need to be (design §10.4).  The
 * session is sans-I/O, so a test IS a host: it calls in with bytes and a
 * synthetic clock, and drains what comes back.  Every deadline the library owns
 * — the 30 s keepalive, the bring-up watchdog, the stream-start bound — runs on
 * that synthetic clock at whatever speed the test likes.
 *
 * The cases below are the ones docs/implementation-notes.md §2 names for
 * increments 1-3, plus the failure shapes that are silent if nobody looks.
 */
#include "hm_test.h"

#include "hackmotion/hackmotion.h"
#include "hackmotion/session.h"

#include "hm_codec.h"
#include "hm_command.h"

/* ------------------------------------------------------------------------ */
/* A fake transport                                                          */
/* ------------------------------------------------------------------------ */
#define FAKE_MAX_WRITES 4096
#define FAKE_MAX_EVENTS 4096
#define FAKE_MAX_LIVE   8192
#define FAKE_MAX_WIRE   1024

typedef struct fake {
    hm_session *s;
    hm_time_us  now;
    bool        drain_writes;

    uint8_t written[FAKE_MAX_WRITES][HM_MAX_COMMAND_LEN];
    uint8_t written_len[FAKE_MAX_WRITES];
    size_t  nwritten;

    hm_event events[FAKE_MAX_EVENTS];
    size_t   nevents;

    hm_sample live[FAKE_MAX_LIVE];
    size_t    nlive;

    hm_wire_chunk wire[FAKE_MAX_WIRE];
    size_t        nwire;
} fake;

/* An allocator that counts, so "exactly one allocation" is a measurement
 * rather than a claim. */
static size_t g_allocs;
static size_t g_frees;
static size_t g_alloc_bytes;

static void *counting_alloc(void *ctx, size_t size)
{
    (void)ctx;
    g_allocs++;
    g_alloc_bytes = size;
    return malloc(size);
}

static void counting_free(void *ctx, void *ptr)
{
    (void)ctx;
    if (ptr != NULL) {
        g_frees++;
    }
    free(ptr);
}

static void drain(fake *f)
{
    for (;;) {
        hm_write_request w[8];
        size_t           n;
        if (!f->drain_writes) {
            break;
        }
        n = hm_session_poll_writes(f->s, w, 8u);
        if (n == 0u) {
            break;
        }
        for (size_t i = 0; i < n; ++i) {
            if (f->nwritten < FAKE_MAX_WRITES) {
                memcpy(f->written[f->nwritten], w[i].data, sizeof(w[i].data));
                f->written_len[f->nwritten] = w[i].length;
                f->nwritten++;
            }
        }
    }
    for (;;) {
        hm_event ev[8];
        size_t   n = hm_session_poll_events(f->s, ev, 8u);
        if (n == 0u) {
            break;
        }
        for (size_t i = 0; i < n; ++i) {
            if (f->nevents < FAKE_MAX_EVENTS) {
                f->events[f->nevents++] = ev[i];
            }
        }
    }
    for (;;) {
        hm_sample sm[16];
        size_t    n = hm_session_poll_live(f->s, sm, 16u);
        if (n == 0u) {
            break;
        }
        for (size_t i = 0; i < n; ++i) {
            if (f->nlive < FAKE_MAX_LIVE) {
                f->live[f->nlive++] = sm[i];
            }
        }
    }
    for (;;) {
        hm_wire_chunk wc[4];
        size_t        n = hm_session_poll_wire(f->s, wc, 4u);
        if (n == 0u) {
            break;
        }
        for (size_t i = 0; i < n; ++i) {
            if (f->nwire < FAKE_MAX_WIRE) {
                f->wire[f->nwire++] = wc[i];
            }
        }
    }
}

/* The accumulators are megabytes, so the fake lives on the heap rather than
 * eating the stack a sanitiser build has less of. */
static fake *fake_open(const hm_session_config *cfg)
{
    hm_session_config c = (cfg != NULL) ? *cfg : hm_session_config_default();
    fake             *f = (fake *)calloc(1u, sizeof(fake));
    HM_ASSERT(f != NULL);
    if (f == NULL) {
        return NULL;
    }
    f->drain_writes = true;
    /* An arbitrary non-zero epoch: the library must never interpret it. */
    f->now = (hm_time_us)1000 * 1000 * 1000;
    HM_ASSERT_EQ(hm_session_create(&c, &f->s), HM_OK);
    return f;
}

static void fake_close(fake *f)
{
    hm_session_destroy(f->s);
    free(f);
}

static void feed(fake *f, const uint8_t *data, size_t n)
{
    hm_session_on_bytes(f->s, data, n, f->now);
    drain(f);
}

static void tick_at(fake *f, hm_time_us when)
{
    f->now = when;
    hm_session_tick(f->s, when);
    drain(f);
}

/* Advance to `target`, honouring every deadline on the way. */
static void run_to(fake *f, hm_time_us target)
{
    for (;;) {
        hm_time_us due = hm_session_next_due_us(f->s);
        if (due == HM_TIME_NEVER || due > target) {
            break;
        }
        tick_at(f, (due < f->now) ? f->now : due);
    }
    tick_at(f, target);
}

/* ------------------------------------------------------------------------ */
/* Device replies                                                            */
/* ------------------------------------------------------------------------ */
/* §5.2's observed bytes: hardware 4.1, protocol 4.0, firmware 4.8, product 20. */
static const uint8_t k_versions[] = { 0x80, 0x04, 0x01, 0x04, 0x00, 0x04, 0x08, 0x14 };
/* §5.3: 100 %, and a second field observed at 2226-2235 that is NOT millivolts. */
static const uint8_t k_status[] = { 0x81, 0x64, 0x08, 0xb2 };
/* §5.4: sensor count 2 — why every record carries two blocks. */
static const uint8_t k_sensor_map[] = { 0x84, 0x02, 0x01 };
static const uint8_t k_serial[] = { 0x86, 'W', 'G', '3', '0', '0', '1', '2', '3', '4' };
static const uint8_t k_mac[] = { 0x85, 'A', '1', 'B', '2', 'C', '3', 'D', '4', 'E', '5', 'F', '6' };

static void reply_bringup(fake *f)
{
    feed(f, k_versions, sizeof(k_versions));
    feed(f, k_status, sizeof(k_status));
    feed(f, k_sensor_map, sizeof(k_sensor_map));
    feed(f, k_status, sizeof(k_status));
    feed(f, k_serial, sizeof(k_serial));
    feed(f, k_mac, sizeof(k_mac));
}

static void bring_up(fake *f)
{
    hm_session_on_link_up(f->s, 247, f->now);
    drain(f);
    reply_bringup(f);
}

/* ------------------------------------------------------------------------ */
/* Frame construction (§6.3)                                                 */
/* ------------------------------------------------------------------------ */
static void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v & 0xffu);
}

/*
 * quaternion[8] accel[6] gyro[6] ticks[2] = 22 bytes.  Any unit quaternion in
 * Q14 gives |q| = 16384, which sits 0.7 counts from §6.4's measured 16384.7
 * nominal and therefore inside the 64-count structural tolerance.
 */
static void put_block_q(uint8_t *p, uint16_t ticks, const int16_t q[4], int16_t accel_x)
{
    put_be16(p + 0, (uint16_t)q[0]);
    put_be16(p + 2, (uint16_t)q[1]);
    put_be16(p + 4, (uint16_t)q[2]);
    put_be16(p + 6, (uint16_t)q[3]);
    put_be16(p + 8, (uint16_t)accel_x);
    put_be16(p + 10, 0u);
    put_be16(p + 12, 0u);
    put_be16(p + 14, 0u);
    put_be16(p + 16, 0u);
    put_be16(p + 18, 0u);
    put_be16(p + 20, ticks);
}

static void put_block(uint8_t *p, uint16_t ticks, int16_t accel_x)
{
    static const int16_t identity[4] = { 16384, 0, 0, 0 };
    put_block_q(p, ticks, identity, accel_x);
}

/* §10.3's stable 59-tick (0.92 ms) palm − lower_arm difference, reproduced. */
#define TEST_SKEW_TICKS 59u

static size_t build_frame(uint8_t *out, const uint16_t *index, const uint16_t *ticks, size_t n,
                          int16_t accel_x)
{
    out[0] = 0x90;
    for (size_t i = 0; i < n; ++i) {
        uint8_t *r = out + 1u + i * 46u;
        put_be16(r, index[i]);
        put_block(r + 2, ticks[i], accel_x);
        put_block(r + 2 + 22, (uint16_t)(ticks[i] + TEST_SKEW_TICKS), accel_x);
    }
    return 1u + n * 46u;
}

static void feed_frame(fake *f, uint16_t index, uint16_t ticks, int16_t accel_x)
{
    uint8_t  buf[1 + 4 * 46];
    uint16_t idx[1];
    uint16_t tk[1];
    size_t   n;
    idx[0] = index;
    tk[0] = ticks;
    n = build_frame(buf, idx, tk, 1u, accel_x);
    feed(f, buf, n);
}

static uint16_t ticks_for(uint32_t index)
{
    return (uint16_t)((uint32_t)(index * HM_NOMINAL_TICKS_PER_SAMPLE + 0.5) & 0xffffu);
}

/*
 * One frame whose two units sit a KNOWN angle apart: the lower arm at the
 * identity and the palm rotated `deg` about z.  §6.7's relative angle is
 * 2·acos|q_palm · q_arm|, so this reads back as `deg` — which is what lets a
 * test place a synthetic pose in one of §8.2's two measured populations.
 */
static void feed_frame_split(fake *f, uint16_t index, uint16_t ticks, double deg)
{
    uint8_t       buf[1 + 46];
    const int16_t arm[4] = { 16384, 0, 0, 0 };
    int16_t       palm[4];
    double        half = deg * 0.5 * (3.14159265358979323846 / 180.0);

    palm[0] = (int16_t)lrint(16384.0 * cos(half));
    palm[1] = 0;
    palm[2] = 0;
    palm[3] = (int16_t)lrint(16384.0 * sin(half));

    buf[0] = 0x90;
    put_be16(buf + 1, index);
    put_block_q(buf + 3, ticks, arm, 0);
    put_block_q(buf + 3 + 22, (uint16_t)(ticks + TEST_SKEW_TICKS), palm, 0);
    feed(f, buf, sizeof(buf));
}

/* ------------------------------------------------------------------------ */
/* Calibration fixtures (§8.2)                                               */
/* ------------------------------------------------------------------------ */
/* ⚠ The device answers BOTH markers with the same two bytes, so which marker an
 * ack belongs to is the session's own state and nothing on the wire. */
static const uint8_t k_cal_ack[] = { 0xa2, 0x01 };

static void feed_cal_result(fake *f)
{
    uint8_t msg[1 + 64];

    /* §8.2's LONG FORM: eight quaternions — the four applied state values, then
     * the two poses per unit — ⚠ palm first in both halves, the reverse of a
     * record.  The library carries the payload verbatim into the wire log and
     * decodes none of it, so eight identities are as informative as the real
     * thing. */
    memset(msg, 0, sizeof(msg));
    msg[0] = 0x94;
    for (int i = 0; i < 8; ++i) {
        put_be16(msg + 1 + i * 8, 16384u);
    }
    feed(f, msg, sizeof(msg));
}

/* Bring-up, one stream, one frame, and the routine walked as far as the device
 * can take it: transform applied, presence not yet measured. */
static fake *cal_open_to_verifying(void)
{
    fake *f = fake_open(NULL);

    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);

    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    feed_cal_result(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_VERIFYING);
    return f;
}

/*
 * The whole routine, then a presence run over `frames` records at a pose `deg`
 * degrees open.  The 0.01°/record drift is deliberate: a run with no variation
 * at all reports a `pose_spread_deg` of exactly 0, which is the one number a
 * synthetic fixture can produce and a person never can.
 */
static fake *cal_measure_at(double deg, size_t frames)
{
    fake *f = cal_open_to_verifying();

    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_OK);
    for (size_t i = 0; i < frames; ++i) {
        uint32_t index = 200u + (uint32_t)i;
        f->now += 4000; /* 250 Hz — well inside the run's own window */
        feed_frame_split(f, (uint16_t)index, ticks_for(index), deg + 0.01 * (double)i);
    }
    return f;
}

static const hm_event *last_phase_event(const fake *f)
{
    const hm_event *found = NULL;
    for (size_t i = 0; i < f->nevents; ++i) {
        if (f->events[i].type == (uint16_t)HM_EV_CALIBRATION_PHASE) {
            found = &f->events[i];
        }
    }
    return found;
}

/* ------------------------------------------------------------------------ */
/* Small queries over what the fake collected                                */
/* ------------------------------------------------------------------------ */
static size_t count_events(const fake *f, hm_event_type type)
{
    size_t n = 0;
    for (size_t i = 0; i < f->nevents; ++i) {
        if (f->events[i].type == (uint16_t)type) {
            n++;
        }
    }
    return n;
}

static const hm_event *find_event(const fake *f, hm_event_type type)
{
    for (size_t i = 0; i < f->nevents; ++i) {
        if (f->events[i].type == (uint16_t)type) {
            return &f->events[i];
        }
    }
    return NULL;
}

static size_t count_warnings(const fake *f, hm_warning_code code)
{
    size_t n = 0;
    for (size_t i = 0; i < f->nevents; ++i) {
        if (f->events[i].type == (uint16_t)HM_EV_WARNING &&
            f->events[i].u.warning.code == (uint16_t)code) {
            n++;
        }
    }
    return n;
}

static size_t count_writes(const fake *f, uint8_t command, size_t from)
{
    size_t n = 0;
    for (size_t i = from; i < f->nwritten; ++i) {
        if (f->written_len[i] > 0u && f->written[i][0] == command) {
            n++;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------ */
/* Increment 1 — lifecycle, memory, rings, link-up, MTU                      */
/* ------------------------------------------------------------------------ */
HM_TEST(session_makes_exactly_one_allocation_and_one_free)
{
    hm_session_config cfg = hm_session_config_default();
    hm_session       *s = NULL;

    g_allocs = 0;
    g_frees = 0;
    cfg.allocator.alloc = counting_alloc;
    cfg.allocator.free = counting_free;

    HM_ASSERT_EQ(hm_session_create(&cfg, &s), HM_OK);
    HM_ASSERT(s != NULL);
    HM_ASSERT_EQ(g_allocs, 1u);
    /* The session object lives inside that one block, so the footprint is one
     * knowable number rather than a scatter of small allocations. */
    HM_ASSERT(g_alloc_bytes > sizeof(hm_sample) * HM_LIVE_RING_RECOMMENDED);

    hm_session_destroy(s);
    HM_ASSERT_EQ(g_frees, 1u);

    /* And nothing allocates after create: the library owns no growth path. */
    HM_ASSERT_EQ(g_allocs, 1u);
}

HM_TEST(supplying_every_ring_still_costs_exactly_one_allocation)
{
    static hm_sample      live[64];
    static hm_event       events[16];
    static hm_wire_chunk  wire[8];
    static hm_index_range coverage[8];
    static hm_live_digest digests[8];
    static hm_sample      gather[8];
    hm_session_config     cfg = hm_session_config_default();
    hm_session           *s = NULL;
    size_t                default_bytes;
    size_t                supplied_bytes;

    g_allocs = 0;
    g_frees = 0;
    cfg.allocator.alloc = counting_alloc;
    cfg.allocator.free = counting_free;
    HM_ASSERT_EQ(hm_session_create(&cfg, &s), HM_OK);
    default_bytes = g_alloc_bytes;
    hm_session_destroy(s);
    s = NULL;

    g_allocs = 0;
    g_frees = 0;
    cfg.allocator.alloc = counting_alloc;
    cfg.allocator.free = counting_free;
    cfg.memory.live_ring = live;
    cfg.memory.live_ring_capacity = 64u;
    cfg.memory.event_ring = events;
    cfg.memory.event_ring_capacity = 16u;
    cfg.memory.wire_ring = wire;
    cfg.memory.wire_ring_capacity = 8u;
    cfg.memory.coverage_storage = coverage;
    cfg.memory.coverage_capacity = 8u;
    cfg.memory.digest_ring = digests;
    cfg.memory.digest_ring_capacity = 8u;
    cfg.memory.history_gather = gather;
    cfg.memory.history_gather_capacity = 8u;

    HM_ASSERT_EQ(hm_session_create(&cfg, &s), HM_OK);
    HM_ASSERT_EQ(g_allocs, 1u);
    supplied_bytes = g_alloc_bytes;
    hm_session_destroy(s);
    HM_ASSERT_EQ(g_frees, 1u);

    /*
     * ⚠ ONE allocation, not zero — the session object itself, which the caller
     * cannot supply.  What supplying the rings buys is size: the whole ring
     * plan drops out of the allocation, leaving only fixed state (the clock
     * fit's bounded hull and residual window dominate it).
     */
    HM_ASSERT(supplied_bytes + sizeof(hm_sample) * HM_LIVE_RING_RECOMMENDED < default_bytes);
    HM_ASSERT(supplied_bytes < 64u * 1024u);
}

HM_TEST(a_zeroed_config_gets_the_observed_default_not_config_zero)
{
    hm_session_config cfg;

    memset(&cfg, 0, sizeof(cfg));
    fake *f = fake_open(&cfg);

    /*
     * ⚠ `= {0}` leaves stream_config.bits at 0x00, which is a REAL and
     * destructive configuration: no tick counters, a different block size and
     * half the gyro scale.  It is not a configuration anyone picks on purpose,
     * and hm_stream_config_nonstandard() always writes a justification, so an
     * entirely empty one means "unset".
     */
    HM_ASSERT_EQ(hm_session_stream_config(f->s).bits, HM_CONFIG_OBSERVED_DEFAULT);
    HM_ASSERT(hm_stream_config_has_ticks(hm_session_stream_config(f->s)));
    fake_close(f);
}

HM_TEST(bringup_runs_the_nine_one_sequence_and_reaches_ready)
{
    fake *f = fake_open(NULL);

    hm_session_on_link_up(f->s, 247, f->now);
    drain(f);

    /* §9.1: 80, 81, 84, 81, 86, 85.  The vendor app sends `86` three times;
     * that is its behaviour, not a device requirement, so we send it once. */
    HM_ASSERT_EQ(f->nwritten, 6u);
    HM_ASSERT_EQ(f->written[0][0], 0x80);
    HM_ASSERT_EQ(f->written[1][0], 0x81);
    HM_ASSERT_EQ(f->written[2][0], 0x84);
    HM_ASSERT_EQ(f->written[3][0], 0x81);
    HM_ASSERT_EQ(f->written[4][0], 0x86);
    HM_ASSERT_EQ(f->written[5][0], 0x85);
    HM_ASSERT_EQ(count_writes(f, 0x86, 0u), 1u);

    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_UP), 1u);
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 0u);

    reply_bringup(f);
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 1u);

    {
        hm_device_info info;
        HM_ASSERT_EQ(hm_session_device_info(f->s, &info), HM_OK);
        HM_ASSERT_EQ(info.protocol_major, 4);
        HM_ASSERT_EQ(info.firmware_minor, 8);
        HM_ASSERT_EQ(info.product_id, 0x14);
        /* ⚠ Two sensors because the reply carries two BYTES, not because its
         * first byte reads 2 (§5.4).  The bytes are location codes: 0.26 m then
         * 0.10 m, the lower arm first, matching the block order of a record. */
        HM_ASSERT_EQ(info.sensor_count, 2);
        HM_ASSERT_EQ(info.sensor_location[0], 0x02);
        HM_ASSERT_EQ(info.sensor_location[1], 0x01);
        HM_ASSERT_EQ(info.battery_percent, 100);
        HM_ASSERT_EQ(info.status_undecoded, 2226); /* ⚠ NOT millivolts */
        HM_ASSERT_STR(info.mac, "A1:B2:C3:D4:E5:F6");
        HM_ASSERT_STR(info.serial, "WG3001234");
        HM_ASSERT_EQ(info.valid, (uint32_t)(HM_INFO_VERSIONS | HM_INFO_SENSOR_MAP |
                                            HM_INFO_BATTERY | HM_INFO_MAC | HM_INFO_SERIAL));
    }
    /* Two `81` polls in the sequence produce two battery events. */
    HM_ASSERT_EQ(count_events(f, HM_EV_BATTERY), 2u);
    fake_close(f);
}

/*
 * ⚠ §5.4/§6.3.  A record is a header followed by one block PER SENSOR, so the
 * sensor count sizes the record.  Every layout this library decodes has exactly
 * two blocks, so a map reporting anything else means the stream is about to be
 * read at the wrong offsets from the second block on.
 *
 * Reported when the MAP arrives, not when the frames start going wrong. The
 * quaternion norm would catch the misalignment (§6.4), but it would fire once
 * per record and never name the cause, and the cause is knowable here.
 */
HM_TEST(a_sensor_count_the_record_layout_cannot_carry_is_reported_at_the_map)
{
    fake         *f = fake_open(NULL);
    /* Three payload bytes: three sensors, whatever the individual codes say. */
    const uint8_t three_sensors[4] = { 0x84, 0x02, 0x01, 0x00 };

    hm_session_on_link_up(f->s, 247, f->now);
    drain(f);
    feed(f, k_versions, sizeof(k_versions));
    feed(f, k_status, sizeof(k_status));
    feed(f, three_sensors, sizeof(three_sensors));

    HM_ASSERT_EQ(count_warnings(f, HM_WARN_SENSOR_COUNT_UNSUPPORTED), 1u);
    {
        size_t i;
        for (i = 0; i < f->nevents; ++i) {
            if (f->events[i].type == (uint16_t)HM_EV_WARNING &&
                f->events[i].u.warning.code == (uint16_t)HM_WARN_SENSOR_COUNT_UNSUPPORTED) {
                HM_ASSERT_EQ(f->events[i].u.warning.detail_i32, 3);
            }
        }
    }

    /* ⚠ Reported, not refused.  The map is still recorded — a count this
     * library cannot decode is exactly the thing a capture should preserve, and
     * the location codes are what would identify the extra unit. */
    {
        hm_device_info info;
        HM_ASSERT_EQ(hm_session_device_info(f->s, &info), HM_OK);
        HM_ASSERT_EQ(info.sensor_count, 3);
        HM_ASSERT_EQ(info.sensor_location[2], 0x00);
        HM_ASSERT(info.valid & (uint32_t)HM_INFO_SENSOR_MAP);
    }
    fake_close(f);
}

/*
 * ⚠ "wG3" is wrist, GENERATION 3, so later generations are expected and the
 * library must not pretend one it has never seen is the one every constant was
 * measured on.  A different product id is neither an error nor a refusal — the
 * protocol may be identical — but the ≈799.2 Hz sample rate, the tick rate,
 * the history depth, the field scales and the meaning of every configuration
 * bit came from ONE product and are unverified anywhere else.
 *
 * Reported once, at the version reply, so a capture says which hardware made
 * it rather than being re-read later under constants that never applied.
 */
HM_TEST(a_product_the_specification_was_not_measured_on_is_reported_not_refused)
{
    fake *f = fake_open(NULL);
    /* Same protocol 4.0, same shape of reply — only the product id differs. */
    uint8_t next_generation[sizeof(k_versions)];

    memcpy(next_generation, k_versions, sizeof(next_generation));
    next_generation[7] = 0x15; /* not HM_PRODUCT_ID_MEASURED */

    hm_session_on_link_up(f->s, 247, f->now);
    drain(f);
    feed(f, next_generation, sizeof(next_generation));

    HM_ASSERT_EQ(count_warnings(f, HM_WARN_UNVERIFIED_PRODUCT), 1u);
    {
        size_t i;
        for (i = 0; i < f->nevents; ++i) {
            if (f->events[i].type == (uint16_t)HM_EV_WARNING &&
                f->events[i].u.warning.code == (uint16_t)HM_WARN_UNVERIFIED_PRODUCT) {
                HM_ASSERT_EQ(f->events[i].u.warning.detail_i32, 0x15);
            }
        }
    }

    /* ⚠ REPORTED, NOT REFUSED.  Bring-up carries on and the session is not
     * failed: the library has no evidence the protocol differs, and refusing
     * hardware on the strength of a version byte would be a guess in the one
     * direction that cannot be recovered from. */
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_DOWN), 0u);
    {
        hm_device_info info;
        HM_ASSERT_EQ(hm_session_device_info(f->s, &info), HM_OK);
        HM_ASSERT_EQ(info.product_id, 0x15);
        HM_ASSERT_EQ(info.protocol_major, 4); /* gating still reads THIS */
    }
    fake_close(f);
}

/* The measured device says 0x14, and says it without a warning. */
HM_TEST(the_product_the_specification_was_measured_on_is_not_warned_about)
{
    fake *f = fake_open(NULL);

    bring_up(f);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_UNVERIFIED_PRODUCT), 0u);
    HM_ASSERT_EQ(k_versions[7], HM_PRODUCT_ID_MEASURED);
    fake_close(f);
}

/* The device under test says two, and says it without a warning. */
HM_TEST(the_two_sensor_map_this_device_sends_is_not_warned_about)
{
    fake *f = fake_open(NULL);

    bring_up(f);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_SENSOR_COUNT_UNSUPPORTED), 0u);
    fake_close(f);
}

HM_TEST(an_mtu_below_the_floor_is_rejected_and_nothing_is_ever_written)
{
    fake *f = fake_open(NULL);

    /*
     * ⚠ The calibration result is 65 bytes and stream notifications reach 93
     * (§2.4).  No platform lets an application REQUEST an MTU through Qt, so
     * the library can only check — and failing loudly beats truncated frames
     * that parse as garbage on whichever platform eventually gets it wrong.
     */
    hm_session_on_link_up(f->s, 64, f->now);
    drain(f);

    {
        const hm_event *ev = find_event(f, HM_EV_MTU_REJECTED);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.mtu, 64);
        }
    }
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_UP), 0u);
    HM_ASSERT_EQ(f->nwritten, 0u);

    /* And it stays refused: no deadline fires anything into the queue, and the
     * consumer cannot talk it round either. */
    run_to(f, f->now + (hm_time_us)600 * 1000 * 1000);
    HM_ASSERT_EQ(f->nwritten, 0u);
    HM_ASSERT(hm_session_start_stream(f->s) < HM_OK);
    drain(f);
    HM_ASSERT_EQ(f->nwritten, 0u);
    fake_close(f);
}

HM_TEST(an_mtu_of_zero_warns_and_proceeds)
{
    fake *f = fake_open(NULL);

    /* "The platform will not tell me."  Proceeding is right; proceeding
     * silently would make "we did not check" indistinguishable from "we
     * checked and it was fine". */
    hm_session_on_link_up(f->s, 0, f->now);
    drain(f);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_MTU_UNKNOWN), 1u);
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_UP), 1u);
    HM_ASSERT_EQ(f->nwritten, 6u);
    fake_close(f);
}

HM_TEST(the_bringup_watchdog_without_versions_takes_the_link_down)
{
    fake *f = fake_open(NULL);
    hm_session_on_link_up(f->s, 247, f->now);
    drain(f);

    /* Everything informational answers; the ONE required step does not. */
    feed(f, k_status, sizeof(k_status));
    feed(f, k_sensor_map, sizeof(k_sensor_map));
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 0u);

    run_to(f, f->now + (hm_time_us)11 * 1000 * 1000);
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 0u);
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_DOWN), 1u);
    fake_close(f);
}

HM_TEST(the_bringup_watchdog_with_versions_alone_still_reaches_ready)
{
    fake *f = fake_open(NULL);
    hm_session_on_link_up(f->s, 247, f->now);
    drain(f);

    /* §9.1: only step 2 is required, and the library tolerates any of the rest
     * going unanswered.  What it must NOT do is hide which ones did. */
    feed(f, k_versions, sizeof(k_versions));
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 0u);

    run_to(f, f->now + (hm_time_us)11 * 1000 * 1000);
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 1u);
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_DOWN), 0u);
    {
        const hm_event *ev = find_event(f, HM_EV_DEVICE_INFO);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.device_info.valid, (uint32_t)HM_INFO_VERSIONS);
        }
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Increment 2 — the deadline table                                          */
/* ------------------------------------------------------------------------ */
HM_TEST(ten_minutes_of_deadlines_terminate_with_strictly_increasing_wakes)
{
    hm_time_us start;
    hm_time_us previous;
    size_t     writes_at_ready;
    int        wakes = 0;

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(count_events(f, HM_EV_READY), 1u);
    start = f->now;
    writes_at_ready = f->nwritten;
    previous = f->now;

    /*
     * ⚠ Step `now` to exactly next_due_us and never past it.  A deadline
     * re-armed at the instant that fired it would show up here as a wake time
     * that did not advance — and in a real host as a loop spinning at 100 %
     * CPU, in THEIR code, looking like their bug.
     */
    for (;;) {
        hm_time_us due = hm_session_next_due_us(f->s);
        HM_ASSERT(due != HM_TIME_NEVER);
        if (due == HM_TIME_NEVER || due > start + (hm_time_us)600 * 1000 * 1000) {
            break;
        }
        HM_ASSERT(due > previous);
        previous = due;
        tick_at(f, due);
        wakes++;
        HM_ASSERT(wakes < 10000); /* the loop must terminate, not merely finish */
    }

    /* §9.2: 30 s polls across ten minutes.  Not "about twenty". */
    HM_ASSERT_EQ(count_writes(f, 0x81, writes_at_ready), 20u);
    /* The alarm must not fire while the polls are going out on time. */
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_KEEPALIVE_LATE), 0u);
    fake_close(f);
}

HM_TEST(a_host_that_oversleeps_gets_one_keepalive_not_twenty)
{
    size_t writes_at_ready;

    fake *f = fake_open(NULL);
    bring_up(f);
    writes_at_ready = f->nwritten;

    /*
     * ⚠ A host is free to miss its timer — the loop is theirs and something
     * else may have held it.  Ten minutes of missed 30 s deadlines must
     * collapse into ONE poll, not a burst of twenty identical ones that the
     * device would see as a flood and that would consume the write queue.
     * The dispatcher disarms before it fires and the handler re-arms forward
     * from `now`, which is what makes catching up cheap.
     */
    tick_at(f, f->now + (hm_time_us)600 * 1000 * 1000);
    HM_ASSERT_EQ(count_writes(f, 0x81, writes_at_ready), 1u);

    /* And the missed decade of polls is REPORTED: 600 s of no host→device write
     * against the device's own 300 s idle shutdown (§9.2). */
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_KEEPALIVE_LATE), 1u);

    /* Whatever it returns, one tick clears it — so the next one is in the
     * future rather than immediately due again. */
    HM_ASSERT(hm_session_next_due_us(f->s) > f->now);
    fake_close(f);
}

HM_TEST(a_host_that_never_drains_writes_is_told_the_keepalive_is_late)
{
    fake *f = fake_open(NULL);
    bring_up(f);

    /*
     * The device dies at 300 s of no host→device write and an active stream
     * does not prevent it (§9.2).  A host that queues but never writes has
     * exactly that failure, and it is invisible from inside the library unless
     * the queue's drain is what marks time.
     */
    f->drain_writes = false;
    run_to(f, f->now + (hm_time_us)150 * 1000 * 1000);
    HM_ASSERT(count_warnings(f, HM_WARN_KEEPALIVE_LATE) >= 1u);
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Increment 3 — the stream and the clock                                    */
/* ------------------------------------------------------------------------ */
HM_TEST(the_stream_starts_on_the_first_frame_not_on_the_ack)
{
    const uint8_t ack[] = { 0xa0, 0x01 };

    fake *f = fake_open(NULL);
    bring_up(f);

    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(f->written[f->nwritten - 1u][0], 0xa0);
    HM_ASSERT_EQ(f->written[f->nwritten - 1u][2], HM_CONFIG_OBSERVED_DEFAULT);
    HM_ASSERT(!hm_session_is_streaming(f->s));

    /* ⚠ The vendor app ignores `a0 01` and so do we (§5.1): a start that is
     * acknowledged and produces nothing is a start that failed. */
    feed(f, ack, sizeof(ack));
    HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_STARTED), 0u);
    HM_ASSERT(!hm_session_is_streaming(f->s));

    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_STARTED), 1u);
    HM_ASSERT(hm_session_is_streaming(f->s));
    HM_ASSERT_EQ(f->nlive, 1u);
    HM_ASSERT(hm_session_stream_id(f->s) != 0u);
    HM_ASSERT_EQ(f->live[0].stream_id, hm_session_stream_id(f->s));
    fake_close(f);
}

HM_TEST(a_stream_that_never_starts_times_out_and_is_not_retried)
{
    fake *f = fake_open(NULL);
    bring_up(f);

    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    {
        size_t starts_before = count_writes(f, 0xa0, 0u);
        HM_ASSERT_EQ(starts_before, 1u);

        /* §6.1 measures the first frame at 50-80 ms, so 3 s is two orders of
         * margin: anything approaching it means the start was not accepted. */
        run_to(f, f->now + (hm_time_us)4 * 1000 * 1000);
        HM_ASSERT_EQ(count_warnings(f, HM_WARN_STREAM_START_TIMEOUT), 1u);
        HM_ASSERT(!hm_session_is_streaming(f->s));

        /* ⚠ No silent retry: a start that failed once will fail again, and the
         * consumer needs to see it rather than a queue of `a0`s. */
        run_to(f, f->now + (hm_time_us)20 * 1000 * 1000);
        HM_ASSERT_EQ(count_writes(f, 0xa0, 0u), 1u);
    }
    fake_close(f);
}

HM_TEST(two_thousand_jittered_frames_fit_a_plausible_rate_that_is_not_800)
{
    hm_time_us t0;
    /* The truth the fake device is built from.  ⚠ 799.2, never a round 800:
     * §6.5 measured 799.19 and 799.32 Hz on two independent host clocks. */
    const double true_rate_hz = 799.2;
    const int    frames = 2000;
    const int    step = 32; /* ~25 Hz live, a decimated view of the internal rate */
    size_t       differ = 0;
    size_t       live_before;

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    t0 = f->now;
    live_before = f->nlive;

    for (int i = 0; i < frames; ++i) {
        uint32_t index = (uint32_t)(i * step);
        /* ⚠ BLE delay is ONE-SIDED — a notification can be late, never early —
         * which is why the fit is a lower envelope and not least squares. */
        hm_time_us jitter = 200 + (hm_time_us)((i * 7919) % 5000);
        f->now = t0 + (hm_time_us)((double)index / true_rate_hz * 1e6) + jitter;
        hm_session_tick(f->s, f->now);
        feed_frame(f, (uint16_t)(index & 0xffffu), ticks_for(index), 0);
    }

    HM_ASSERT_EQ(f->nlive - live_before, (size_t)frames);

    /* The index space wraps every 82.0 s at this rate, and 2000 × 32 crosses
     * 65536 — the unwrapped counter must not care. */
    HM_ASSERT_EQ(f->live[f->nlive - 1u].sample_index, (uint32_t)((frames - 1) * step));

    {
        hm_clock_snapshot snap;
        HM_ASSERT_EQ(hm_session_clock(f->s, &snap), HM_OK);
        HM_ASSERT((snap.flags & (uint32_t)HM_CLOCK_HAS_FIT) != 0u);
        HM_ASSERT((snap.flags & (uint32_t)HM_CLOCK_SHORT_BASELINE) == 0u);
        HM_ASSERT(snap.fitted_rate_hz > HM_RATE_PLAUSIBLE_MIN_HZ);
        HM_ASSERT(snap.fitted_rate_hz < HM_RATE_PLAUSIBLE_MAX_HZ);
        HM_ASSERT_NEAR(snap.fitted_rate_hz, true_rate_hz, 0.5);
        /* ⚠ And it is NOT 800.  Assuming 800 costs ≈1,000 ppm — 45 ms after
         * 45 s, which is 11 frames at 240 fps — and silently degenerates the
         * estimator onto one end-of-session point. */
        HM_ASSERT(fabs(snap.fitted_rate_hz - 800.0) > 0.3);
    }

    for (size_t i = live_before; i < f->nlive; ++i) {
        const hm_sample *sm = &f->live[i];

        /* ⚠ host_time_us is the FIT applied to the index, not the arrival
         * instant.  Arrival is what the fit is built from and is one-sidedly
         * late; mapping straight from it puts every swing late by the mean
         * link delay. */
        HM_ASSERT(sm->host_time_us <= sm->host_recv_us);
        if (sm->host_time_us != sm->host_recv_us) {
            differ++;
        }

        /* Device time comes from the frame alone (§10.2) and never waits for
         * the fit. */
        HM_ASSERT(sm->lower_arm.device_time_us != HM_TIME_UNKNOWN);
        HM_ASSERT(sm->palm.device_time_us != HM_TIME_UNKNOWN);
        if (i > live_before) {
            HM_ASSERT(sm->lower_arm.device_time_us > f->live[i - 1u].lower_arm.device_time_us);
            HM_ASSERT(sm->palm.device_time_us > f->live[i - 1u].palm.device_time_us);
        }

        /* §10.3's 59 ticks is 0.92 ms, and it is carried rather than silently
         * treated as zero: at 1,000 °/s it is worth ~0.9° of relative angle. */
        HM_ASSERT(sm->skew_us > 900 && sm->skew_us < 945);

        HM_ASSERT_EQ(sm->source, (uint8_t)HM_SOURCE_LIVE);
        HM_ASSERT_EQ(sm->calibration, (uint8_t)HM_CAL_UNCALIBRATED);
        HM_ASSERT_EQ(sm->config_bits, HM_CONFIG_OBSERVED_DEFAULT);
        HM_ASSERT((sm->flags & (uint16_t)HM_SAMPLE_QUAT_NORM_SUSPECT) == 0u);
    }
    /* ⚠ Assert the count, not merely the absence of a mismatch: "every sample
     * agreed" over zero samples reads identically to a check that stopped
     * running.  The handful of exceptions are the fit's own support points,
     * where the envelope line touches an observation by construction. */
    HM_ASSERT(differ >= (size_t)frames - 32);

    /* The clock event carries the snapshot, once a second, while the fit has
     * observations to report. */
    HM_ASSERT(count_events(f, HM_EV_CLOCK_UPDATED) > 1u);
    fake_close(f);
}

HM_TEST(a_live_ring_that_overflows_says_so)
{
    static hm_sample  small[8];
    hm_session_config cfg = hm_session_config_default();

    cfg.memory.live_ring = small;
    cfg.memory.live_ring_capacity = 8u;
    fake *f = fake_open(&cfg);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    f->drain_writes = false; /* stop draining anything at all */

    for (uint32_t i = 0; i < 20u; ++i) {
        uint8_t  buf[1 + 46];
        uint16_t idx[1];
        uint16_t tk[1];
        size_t   n;
        idx[0] = (uint16_t)(i * 8u);
        tk[0] = ticks_for(i * 8u);
        n = build_frame(buf, idx, tk, 1u, 0);
        hm_session_on_bytes(f->s, buf, n, f->now + (hm_time_us)i * 1000);
    }

    /* Drop-oldest, and COUNTED: a host that is not keeping up must be able to
     * see that data was lost rather than infer it from a gap. */
    HM_ASSERT_EQ(hm_session_dropped_live(f->s), 12u);
    {
        hm_sample out[32];
        HM_ASSERT_EQ(hm_session_poll_live(f->s, out, 32u), 8u);
        HM_ASSERT_EQ(out[0].sample_index, 96u); /* the newest eight survive */
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* The bracket discriminator — live and history frames are byte-identical    */
/* ------------------------------------------------------------------------ */
HM_TEST(an_orphan_bracket_reaches_neither_the_live_ring_nor_the_fit)
{
    const uint8_t     mark_start[] = { 0xa1, 0x02 };
    const uint8_t     mark_end[] = { 0xa1, 0x01 };
    hm_session_config cfg = hm_session_config_default();
    int32_t           observations_before;
    size_t            live_before;
    size_t            writes_before;

    /* A 5 s poll so the keepalive is genuinely DUE inside the bracket: with the
     * measured-good 30 s it would not be, and the suppression below would pass
     * without ever having been exercised. */
    cfg.policy.keepalive_period_us = (hm_time_us)5 * 1000 * 1000;

    fake *f = fake_open(&cfg);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);

    for (uint32_t i = 0; i < 10u; ++i) {
        f->now += 40000;
        feed_frame(f, (uint16_t)(40000u + i * 32u), ticks_for(40000u + i * 32u), 0);
    }
    live_before = f->nlive;
    {
        hm_clock_snapshot snap;
        (void)hm_session_clock(f->s, &snap);
        observations_before = snap.observations;
    }

    /*
     * ⚠ §10.1: live and history `0x90` frames are BYTE-IDENTICAL, and the
     * `a1 02` … `a1 01` bracket is the only thing that separates them.  Phase 2
     * never issues an `a1`, so this bracket is an orphan — and its records
     * still must not reach the fit (their indices run BEHIND live and would
     * read as a near-full-modulus forward step) or the consumer's live ring
     * (where ~4,000 bulk arrivals would look exactly like real motion).
     */
    writes_before = f->nwritten;
    feed(f, mark_start, sizeof(mark_start));
    for (uint32_t i = 0; i < 200u; ++i) {
        feed_frame(f, (uint16_t)(38000u + i), ticks_for(38000u + i), 0);
    }
    HM_ASSERT_EQ(f->nlive, live_before);

    /*
     * ⚠ And the quiet period holds: TWO keepalives fall due across this span
     * and neither goes out.  §7.5 measured `a1` issued into a running stream
     * with nothing else being written; an `0x81` reply interleaved into a
     * record stream that has no length field, no sequence number and no
     * checksum is the worst failure this protocol admits.
     */
    run_to(f, f->now + (hm_time_us)12 * 1000 * 1000);
    HM_ASSERT_EQ(count_writes(f, 0x81, writes_before), 0u);

    feed(f, mark_end, sizeof(mark_end));

    /* The `a1` write itself served the device's idle timer when the bracket
     * opened, and the keepalive is re-armed from the close. */
    run_to(f, f->now + (hm_time_us)6 * 1000 * 1000);
    HM_ASSERT(count_writes(f, 0x81, writes_before) >= 1u);

    /*
     * ⚠ And the suspension is REPORTED.  Nothing on the wire marks the span
     * over which live delivery stopped (api-request B11), so a consumer
     * stitching a session's lane has no other way to tell a window that was
     * never delivered from one that was never requested.
     */
    {
        const hm_event *ev = find_event(f, HM_EV_HISTORY_BLIND_SPAN);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.history_blind_span.request_id, 0u); /* we never asked */
            HM_ASSERT(ev->u.history_blind_span.span.end_us >
                      ev->u.history_blind_span.span.start_us);
        }
    }

    /* Live resumes on the index it left off at — the INDEX space is untouched
     * by a retrieval, and only the host mapping restarts. */
    f->now += 40000;
    feed_frame(f, (uint16_t)(40000u + 10u * 32u), ticks_for(40000u + 10u * 32u), 0);
    HM_ASSERT_EQ(f->nlive, live_before + 1u);
    HM_ASSERT_EQ(f->live[f->nlive - 1u].sample_index, 40000u + 10u * 32u);

    /*
     * ⚠ AND THE FIT RE-ANCHORED ACROSS THE BRACKET (design §6.1.1), which is
     * what the ten observations before it no longer being there means.
     *
     * The device stops COUNTING samples while it replays them (§7.5, measured
     * across six pulls: the stall is 90-99 % of the pull's own duration), so
     * wall time advanced and the index did not, and one line cannot span that.
     * §10.2 measured the cost of carrying on regardless: a fit anchored once at
     * stream start was out by 111,311 ticks after five pulls — 1.70 wraps of
     * 65,536 — and fitting a 44.5 s six-pull capture as one segment returned
     * 768 Hz against a true 801.
     *
     * ⚠ This holds for an ORPHAN bracket too, and deliberately: the stall is
     * the device replaying, not us asking, so it costs the mapping whether or
     * not anybody was waiting for the records.
     */
    {
        hm_clock_snapshot snap;
        HM_ASSERT(observations_before > 1);
        (void)hm_session_clock(f->s, &snap);
        HM_ASSERT_EQ(snap.observations, 1);
        /* Still usable from the very first frame of the new stretch, with the
         * rate seeded rather than refitted — one offset per stretch, one rate
         * per connection. */
        HM_ASSERT((snap.flags & (uint32_t)HM_CLOCK_HAS_FIT) != 0u);
        HM_ASSERT((snap.flags & (uint32_t)HM_CLOCK_BLIND) == 0u);
        HM_ASSERT(f->live[f->nlive - 1u].host_time_us != HM_TIME_UNKNOWN);
    }
    fake_close(f);
}

HM_TEST(the_bracket_limit_closes_a_bracket_the_device_never_closed)
{
    const uint8_t     mark_start[] = { 0xa1, 0x02 };
    hm_session_config cfg = hm_session_config_default();
    size_t            writes_before;

    cfg.policy.keepalive_period_us = (hm_time_us)5 * 1000 * 1000;

    fake *f = fake_open(&cfg);
    bring_up(f);
    writes_before = f->nwritten;

    feed(f, mark_start, sizeof(mark_start));
    /* Suppressed while the bracket is open — deliberately, and affordably: a
     * retrieval is bounded by the ~7.5 s buffer depth against a 300 s idle
     * shutdown, two orders of magnitude of headroom. */
    run_to(f, f->now + (hm_time_us)12 * 1000 * 1000);
    HM_ASSERT_EQ(count_writes(f, 0x81, writes_before), 0u);

    /*
     * ⚠ But a bracket that never closes would hold the queue until the device
     * died at 5.0 minutes, which reads as a radio fault.  The hard limit is
     * twice the buffer depth (15 s) and is independent of any request's
     * deadline — a marker whose end never arrives is not a slow pull.
     */
    run_to(f, f->now + (hm_time_us)10 * 1000 * 1000);
    HM_ASSERT(count_writes(f, 0x81, writes_before) >= 1u);
    /* The suspension is reported rather than merely ended. */
    HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_BLIND_SPAN), 1u);
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Link loss                                                                 */
/* ------------------------------------------------------------------------ */
HM_TEST(link_down_always_invalidates_calibration)
{
    fake *f = fake_open(NULL);
    bring_up(f);

    hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now + 1000);
    drain(f);
    {
        const hm_event *ev = find_event(f, HM_EV_LINK_DOWN);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            /*
             * ⚠ ALWAYS 1.  §8.3 measured 0.70° immediately before dropping a
             * link and 18.80° at the same pose after reconnecting, strap
             * untouched and never removed.
             */
            HM_ASSERT_EQ(ev->u.link_down.calibration_invalidated, 1);
            HM_ASSERT_EQ(ev->u.link_down.advice, (uint8_t)HM_RECOVER_RECONNECT_WITH_BACKOFF);
            HM_ASSERT(ev->u.link_down.suggested_retry_delay_us > 0);
        }
    }
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNCALIBRATED);
    /* Nothing is due once the link is gone: the host may sleep until it has
     * something to say. */
    HM_ASSERT(hm_session_next_due_us(f->s) == HM_TIME_NEVER);
    fake_close(f);
}

HM_TEST(link_down_is_classified_rather_than_retried)
{
    /* ⚠ Retrying against a slept device cannot succeed at any interval (§9.6).
     * The discriminators are available to the library and to nobody above it. */
    {
        fake *f = fake_open(NULL);
        bring_up(f);
        hm_session_on_link_down(f->s, HM_LINK_DOWN_CONNECTION_TAKEN, f->now + 1000);
        drain(f);
        {
            const hm_event *ev = find_event(f, HM_EV_LINK_DOWN);
            HM_ASSERT(ev != NULL);
            if (ev != NULL) {
                /* The device accepts ONE connection and the vendor app wins the
                 * race if it is running (§2.2). */
                HM_ASSERT_EQ(ev->u.link_down.advice, (uint8_t)HM_RECOVER_NEEDS_OTHER_APP_CLOSED);
                HM_ASSERT_EQ(ev->u.link_down.suggested_retry_delay_us, 0);
            }
        }
        fake_close(f);
    }
    {
        fake *f = fake_open(NULL);
        bring_up(f);
        /* No host write for 4.5 of the device's 5.0 idle minutes: it has almost
         * certainly slept, and only a physical button press brings it back. */
        f->drain_writes = false;
        f->now += (hm_time_us)280 * 1000 * 1000;
        hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now);
        f->drain_writes = true;
        drain(f);
        {
            const hm_event *ev = find_event(f, HM_EV_LINK_DOWN);
            HM_ASSERT(ev != NULL);
            if (ev != NULL) {
                HM_ASSERT_EQ(ev->u.link_down.advice, (uint8_t)HM_RECOVER_NEEDS_BUTTON_PRESS);
                HM_ASSERT_EQ(ev->u.link_down.suggested_retry_delay_us, 0);
            }
        }
        fake_close(f);
    }
    {
        fake *f = fake_open(NULL);
        bring_up(f);
        /* A device that is still advertising did not go to sleep — and it will
         * stop advertising within seconds, so this retry is the urgent one. */
        hm_session_on_advertising_seen(f->s, f->now);
        hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now + 1000);
        drain(f);
        {
            const hm_event *ev = find_event(f, HM_EV_LINK_DOWN);
            HM_ASSERT(ev != NULL);
            if (ev != NULL) {
                HM_ASSERT_EQ(ev->u.link_down.advice,
                             (uint8_t)HM_RECOVER_RECONNECT_WITH_BACKOFF);
                HM_ASSERT(ev->u.link_down.suggested_retry_delay_us > 0);
                HM_ASSERT(ev->u.link_down.suggested_retry_delay_us <= 250000);
            }
        }
        fake_close(f);
    }
}

HM_TEST(power_off_lingers_and_advises_against_retrying)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 10u, ticks_for(10u), 0);

    HM_ASSERT_EQ(hm_session_power_off(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(f->written[f->nwritten - 1u][0], 0xfa);

    /*
     * ⚠ §9.3: no acknowledgement is ever sent and the link stays up for ~9 s.
     * The session expects the gap, does not read it as failure, and does not
     * retry into it.
     */
    run_to(f, f->now + (hm_time_us)8 * 1000 * 1000);
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_DOWN), 0u);
    HM_ASSERT_EQ(count_writes(f, 0xfa, 0u), 1u);

    /* HM_POWER_OFF_LINGER_US is 12 s: deliberate margin over one 9 s
     * measurement, not a transcription of it. */
    run_to(f, f->now + (hm_time_us)6 * 1000 * 1000);
    {
        const hm_event *ev = find_event(f, HM_EV_LINK_DOWN);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.link_down.advice, (uint8_t)HM_RECOVER_DO_NOT_RETRY);
            HM_ASSERT_EQ(ev->u.link_down.suggested_retry_delay_us, 0);
        }
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* The wire log                                                              */
/* ------------------------------------------------------------------------ */
/*
 * ⚠⚠ EVERY EVENT A REAL BRING-UP EMITS THAT CARRIES AN IDENTIFIER IS FLAGGED AS
 * CARRYING ONE.
 *
 * implementation-review I4.  The unit test in test_device.c asks the predicate
 * about a type; this asks the SESSION what it actually puts on the wire to a
 * consumer, which is where the miss was: `enter_ready()` ships the whole
 * `hm_device_info` under HM_EV_DEVICE_INFO, and bring-up does not reach READY
 * until the MAC and serial have landed in it, so that event carries both every
 * single time.  `hm_event_is_sensitive()` said no.
 *
 * ⚠ Design §9.2 names three mechanisms and this is the one a binding's
 * serialisation and a consumer's telemetry filter on.  The other two — the
 * formatter's redaction and the wire log's — were tested; this was not, and
 * the formatter happens not to print those fields for this type, which is
 * exactly why it looked fine.
 */
HM_TEST(every_bring_up_event_carrying_an_identifier_says_so)
{
    fake  *f = fake_open(NULL);
    size_t carriers = 0;

    bring_up(f);

    for (size_t i = 0; i < f->nevents; ++i) {
        const hm_event *ev = &f->events[i];
        bool            has_id = false;

        if (ev->type != (uint16_t)HM_EV_DEVICE_INFO && ev->type != (uint16_t)HM_EV_IDENTITY) {
            continue;
        }
        has_id = ev->u.device_info.mac[0] != '\0' || ev->u.device_info.serial[0] != '\0';
        if (!has_id) {
            continue;
        }
        carriers++;
        HM_ASSERT_MSG(hm_event_is_sensitive(ev),
                      "an event whose payload holds a MAC or a serial must be "
                      "filterable by the mechanism design §9.2 points at");
    }

    /* ⚠ And the count, because "none of zero events leaked" reads identically to
     * "nothing was checked" — which is the failure this library is built around. */
    HM_ASSERT_MSG(carriers >= 3u,
                  "bring-up must emit the identity events AND the device-info "
                  "event that quietly repeats them");
    fake_close(f);
}

HM_TEST(identifiers_are_redacted_in_the_wire_log_unless_asked_for)
{
    static hm_wire_chunk chunks[64];
    hm_session_config    cfg = hm_session_config_default();
    size_t               redacted = 0;
    size_t               mac_chunks = 0;

    cfg.memory.wire_ring = chunks;
    cfg.memory.wire_ring_capacity = 64u;
    fake *f = fake_open(&cfg);
    bring_up(f);

    for (size_t i = 0; i < f->nwire; ++i) {
        if (f->wire[i].direction != (uint8_t)HM_WIRE_DEVICE_TO_HOST) {
            continue;
        }
        if (f->wire[i].length > 0u && f->wire[i].data[0] == 0x85) {
            mac_chunks++;
            /* ⚠ The id survives so a reader can see WHAT was removed; the
             * payload does not.  Marked, so absent and redacted stay
             * distinguishable (api-request §2.13). */
            HM_ASSERT((f->wire[i].flags & (uint8_t)HM_WIRE_REDACTED) != 0u);
            HM_ASSERT_EQ(f->wire[i].length, sizeof(k_mac));
            HM_ASSERT_EQ(f->wire[i].data[1], 0);
        }
        if ((f->wire[i].flags & (uint8_t)HM_WIRE_REDACTED) != 0u) {
            redacted++;
        }
    }
    HM_ASSERT_EQ(mac_chunks, 1u);
    HM_ASSERT_EQ(redacted, 2u); /* MAC and serial */
    fake_close(f);

    {
        static hm_wire_chunk chunks2[64];
        hm_session_config    cfg2 = hm_session_config_default();
        bool                 saw_mac_bytes = false;

        cfg2.memory.wire_ring = chunks2;
        cfg2.memory.wire_ring_capacity = 64u;
        cfg2.policy.record_identifiers = true;
        fake *g = fake_open(&cfg2);
        bring_up(g);
        for (size_t i = 0; i < g->nwire; ++i) {
            /* ⚠ The host's own `85` REQUEST is one byte and also starts 0x85;
             * it is the device's REPLY that carries the identifier. */
            if (g->wire[i].direction != (uint8_t)HM_WIRE_DEVICE_TO_HOST) {
                continue;
            }
            if (g->wire[i].length > 0u && g->wire[i].data[0] == 0x85) {
                HM_ASSERT((g->wire[i].flags & (uint8_t)HM_WIRE_REDACTED) == 0u);
                HM_ASSERT_EQ(g->wire[i].data[1], 'A');
                saw_mac_bytes = true;
            }
        }
        HM_ASSERT(saw_mac_bytes);
        fake_close(g);
    }
}

HM_TEST(the_wire_log_marks_loss_and_carries_its_own_sequence)
{
    static hm_wire_chunk chunks[8];
    hm_session_config    cfg = hm_session_config_default();

    cfg.memory.wire_ring = chunks;
    cfg.memory.wire_ring_capacity = 8u;
    fake *f = fake_open(&cfg);
    hm_session_on_link_up(f->s, 247, f->now);
    /* Never drain the wire ring, so it overflows. */
    for (int i = 0; i < 30; ++i) {
        hm_session_on_bytes(f->s, k_status, sizeof(k_status), f->now + i * 1000);
    }
    HM_ASSERT(hm_session_dropped_wire(f->s) > 0u);
    {
        hm_wire_chunk out[8];
        size_t        n = hm_session_poll_wire(f->s, out, 8u);
        HM_ASSERT_EQ(n, 8u);
        /* ⚠ The chunk that FOLLOWS the loss is marked, and `sequence` is
         * carried rather than renumbered — a reader numbering from its own
         * ordinal would turn a lossy recording into a complete-looking one. */
        HM_ASSERT((out[0].flags & (uint8_t)HM_WIRE_LOST) != 0u);
        HM_ASSERT(out[0].sequence > 0u);
        HM_ASSERT_EQ(out[7].sequence, out[0].sequence + 7u);
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Oddments that must not be silent                                          */
/* ------------------------------------------------------------------------ */
HM_TEST(an_unknown_message_is_logged_and_ignored)
{
    const uint8_t junk[] = { 0x77, 0x01, 0x02, 0x03 };

    fake *f = fake_open(NULL);
    bring_up(f);
    feed(f, junk, sizeof(junk));

    /* §5.1: anything not in the table is logged and ignored rather than treated
     * as an error — the range is sparse and the table is scoped to what a
     * client uses, not to every byte the device could emit. */
    {
        const hm_event *ev = find_event(f, HM_EV_UNKNOWN_MESSAGE);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.unknown_message.message_id, 0x77);
            HM_ASSERT_EQ(ev->u.unknown_message.first_bytes[0], 0x77);
        }
    }
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_DOWN), 0u);
    fake_close(f);
}

HM_TEST(a_coalescing_transport_is_loud_rather_than_absorbed)
{
    uint8_t  buf[1 + 46 + 10];
    uint16_t idx[1];
    uint16_t tk[1];
    size_t   n;

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);

    idx[0] = 500u;
    tk[0] = ticks_for(500u);
    n = build_frame(buf, idx, tk, 1u, 0);
    memset(buf + n, 0, 10u);

    /*
     * ⚠ Under one-call-one-notification this cannot happen, so it is the
     * signature of a transport that coalesced — and §3 gives no length field,
     * no sequence number and no checksum to resynchronise on, so absorbing it
     * would corrupt silently.
     */
    feed(f, buf, n + 10u);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_TRAILING_BYTES), 1u);
    fake_close(f);
}

HM_TEST(the_button_is_reported_as_an_edge_hint)
{
    const uint8_t press[] = { 0xfb, 0x01 };

    fake *f = fake_open(NULL);
    bring_up(f);
    feed(f, press, sizeof(press));
    feed(f, press, sizeof(press));

    /* §9.4: one confirmed press produced NO notification and another produced
     * two 187 ms apart.  The library reports edges and builds no counter. */
    HM_ASSERT_EQ(count_events(f, HM_EV_BUTTON), 2u);
    fake_close(f);
}

HM_TEST(a_device_error_carries_no_meaning_beyond_its_arrival)
{
    const uint8_t err[] = { 0xd0, 0x03 };

    fake *f = fake_open(NULL);
    bring_up(f);
    feed(f, err, sizeof(err));
    {
        const hm_event *ev = find_event(f, HM_EV_DEVICE_ERROR);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            /* ⚠ Seven distinct causes all returned d0 03 (§7.2), so the code
             * classifies nothing.  Unsolicited here: no request is in flight. */
            HM_ASSERT_EQ(ev->u.device_error.code, 0x03);
            HM_ASSERT_EQ(ev->u.device_error.request_id, 0u);
        }
    }
    fake_close(f);
}

HM_TEST(a_regressing_host_clock_is_clamped_and_reported)
{
    fake *f = fake_open(NULL);
    bring_up(f);

    /* ⚠ types.h requires a MONOTONIC clock; a wall clock stepped by NTP or DST
     * corrupts a capture in a way that looks like a sensor fault.  The session
     * clamps and continues — and says so, because it is entirely on the
     * consumer's side of the boundary and they cannot fix what they cannot
     * see. */
    hm_session_on_bytes(f->s, k_status, sizeof(k_status), f->now - (hm_time_us)5 * 1000 * 1000);
    drain(f);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HOST_CLOCK_REGRESSION), 1u);
    fake_close(f);
}

HM_TEST(pinned_samples_are_reported_with_the_span_they_cover)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);

    for (uint32_t i = 0; i < 5u; ++i) {
        f->now += 1000;
        /* int16 SATURATES rather than wrapping, so clipped data still looks
         * like a plausible waveform (§6.4) and nothing on the wire reports it. */
        feed_frame(f, (uint16_t)(200u + i), ticks_for(200u + i), 32767);
    }
    HM_ASSERT((f->live[f->nlive - 1u].flags & (uint16_t)HM_SAMPLE_PINNED) != 0u);

    run_to(f, f->now + (hm_time_us)2 * 1000 * 1000);
    {
        const hm_event *ev = find_event(f, HM_EV_PINNED_SAMPLES);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.pinned.counts.total, 10u); /* accel_x, both units */
            /* ⚠ A count without the span it covers is an estimate without
             * evidence: 10 pins in 5 samples and 10 in 10,000 are different
             * facts. */
            HM_ASSERT(ev->u.pinned.over_samples >= 5u);
        }
    }
    fake_close(f);
}

HM_TEST(set_clock_correction_rejects_an_empty_field_mask)
{
    hm_clock_correction c;

    fake *f = fake_open(NULL);
    memset(&c, 0, sizeof(c));

    /* ⚠ `= {0}` is the idiom for every other struct in this API, so the one
     * struct whose zero value would wipe a term nobody measured is the one a
     * consumer reaches for by accident.  It is rejected, not obeyed. */
    HM_ASSERT_EQ(hm_session_set_clock_correction(f->s, &c), HM_ERR_INVALID_ARG);

    c.fields = (uint32_t)(HM_CORRECTION_RATE | HM_CORRECTION_DRIFT);
    c.rate_ppm = -350.0;
    c.residual_drift_us_per_s = 400.0;
    memcpy(c.provenance, "bay 3 camera+mic", 17u);
    HM_ASSERT_EQ(hm_session_set_clock_correction(f->s, &c), HM_OK);

    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 1u, ticks_for(1u), 0);
    {
        hm_clock_snapshot snap;
        (void)hm_session_clock(f->s, &snap);
        /* Everything set is carried in every subsequent snapshot, so a
         * re-analysis a year later reproduces the same alignment. */
        HM_ASSERT((snap.flags & (uint32_t)HM_CLOCK_EXTERNAL_CORRECTION) != 0u);
        HM_ASSERT_NEAR(snap.external_ppm, -350.0, 1e-9);
        HM_ASSERT_NEAR(snap.accuracy_drift_us_per_s, 400.0, 1e-9);
        HM_ASSERT_STR(snap.provenance, "bay 3 camera+mic");
    }
    fake_close(f);
}

HM_TEST(close_seals_every_drain)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    f->drain_writes = false;
    feed_frame(f, 5u, ticks_for(5u), 0);

    hm_session_close(f->s);

    /*
     * ⚠ The stop barrier (api-request §2.11).  After close() returns, nothing
     * in the library will read or write a buffer the consumer supplied — which
     * is precisely why destroying one is safe from this instant.
     */
    {
        hm_write_request w[4];
        hm_event         ev[4];
        hm_sample        sm[4];
        hm_wire_chunk    wc[4];
        HM_ASSERT_EQ(hm_session_poll_writes(f->s, w, 4u), 0u);
        HM_ASSERT_EQ(hm_session_poll_events(f->s, ev, 4u), 0u);
        HM_ASSERT_EQ(hm_session_poll_live(f->s, sm, 4u), 0u);
        HM_ASSERT_EQ(hm_session_poll_wire(f->s, wc, 4u), 0u);
    }
    HM_ASSERT(hm_session_next_due_us(f->s) == HM_TIME_NEVER);
    /* And nothing can be started again. */
    HM_ASSERT(hm_session_start_stream(f->s) < HM_OK);
    hm_session_tick(f->s, f->now + 1000000);
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* The availability queries, before anything has measured them (§8.5)         */
/* ------------------------------------------------------------------------ */
HM_TEST(the_availability_queries_separate_a_measurement_from_the_seed)
{
    hm_time_range range;

    fake *f = fake_open(NULL);

    /* ⚠ NO STREAM IS NOT A NARROW RANGE, IT IS NO RANGE.  §7.4: the device only
     * buffers while streaming, so before one there is nothing resident to
     * report and saying "0 µs wide" would be a measurement of a buffer that
     * does not exist. */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_ERR_NO_STREAM);
    HM_ASSERT(!hm_history_coverage_available(f->s, 0, 1000));

    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    for (uint32_t i = 0; i < 40u; ++i) {
        f->now += 40000; /* ≈25 Hz live, so the stream has a real age */
        hm_session_tick(f->s, f->now);
        feed_frame(f, (uint16_t)(7u + i * 32u), ticks_for(7u + i * 32u), 0);
    }

    /* ⚠ Before any routine has run, every sample says UNCALIBRATED — never
     * CALIBRATED, and never a hopeful UNKNOWN.  We have looked: none has been
     * performed (§8.3). */
    HM_ASSERT_EQ(f->live[f->nlive - 1u].calibration, (uint8_t)HM_CAL_UNCALIBRATED);
    HM_ASSERT(isnan(hm_calibration_presence_angle_deg(f->s)));

    /*
     * ⚠⚠ THE STATUS IS THE WHOLE POINT OF THIS TEST.  §7.3's ~7.5 s was measured ONCE
     * — measured ONCE, after 20 s of streaming on somebody else's session — and
     * whether the buffer holds a fixed sample count or a fixed duration is not
     * established, which the motion-adaptive rate turns into an 8× difference.
     * So the range is filled from the seed, because an order of magnitude is
     * useful, and the status says HM_PENDING, because nothing on THIS connection
     * has measured it.  A stub that returned HM_OK here would be the seed
     * wearing a measurement's clothes, which is increment 7's whole failure
     * mode.
     */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_PENDING);
    HM_ASSERT(range.end_us > range.start_us);
    /* Clamped to the stream's own start (AR B10): this stream is milliseconds
     * old, so it cannot claim to reach back 7.5 s. */
    HM_ASSERT(range.end_us - range.start_us < HM_HISTORY_DEPTH_SEED_US);

    /* ⚠ And the bool answers only from the MEASURED case, because it has
     * nowhere to put the caveat.  False is "we cannot say" — the only answer
     * that cannot mislead a consumer into skipping a check. */
    HM_ASSERT(!hm_history_coverage_available(f->s, range.start_us, range.end_us));

    /* An id nobody issued is a programming error at the call site, not a wait
     * that never ends. */
    {
        hm_history_block *block = (hm_history_block *)(uintptr_t)1;
        HM_ASSERT_EQ(hm_history_collect(f->s, 12345u, &block), HM_ERR_INVALID_ARG);
        HM_ASSERT(block == NULL);
        HM_ASSERT_EQ(hm_history_cancel(f->s, 12345u), HM_ERR_INVALID_ARG);
        HM_ASSERT_EQ(hm_history_pending(f->s), 0u);
    }
    fake_close(f);
}

HM_TEST(the_live_gap_alarm_is_suppressed_inside_a_bracket_and_returns_after_it)
{
    const uint8_t mark_start[] = { 0xa1, 0x02 };
    const uint8_t mark_end[] = { 0xa1, 0x01 };
    size_t        gaps_after_stream;

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 300u, ticks_for(300u), 0);

    /* Streaming and then nothing: BLE at range delays notifications and nothing
     * in the protocol reports it, so the silence has to be reported here. */
    run_to(f, f->now + (hm_time_us)4 * 1000 * 1000);
    HM_ASSERT(count_warnings(f, HM_WARN_LIVE_GAP) >= 1u);
    gaps_after_stream = count_warnings(f, HM_WARN_LIVE_GAP);

    /* ⚠ Inside a retrieval, live delivery is suspended BY DESIGN for up to the
     * width of the pull (§10.1).  Warning there would cry wolf on every
     * gather, so the alarm is suppressed rather than merely late. */
    feed(f, mark_start, sizeof(mark_start));
    run_to(f, f->now + (hm_time_us)10 * 1000 * 1000);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_LIVE_GAP), gaps_after_stream);

    /*
     * ⚠ And it comes BACK when the bracket closes, without waiting for a frame
     * that may never arrive — otherwise a pull that ends with the device wedged
     * would leave the one warning that says "nothing is arriving" disarmed for
     * good, and that silence is indistinguishable from a healthy stream.
     */
    feed(f, mark_end, sizeof(mark_end));
    run_to(f, f->now + (hm_time_us)4 * 1000 * 1000);
    HM_ASSERT(count_warnings(f, HM_WARN_LIVE_GAP) > gaps_after_stream);
    fake_close(f);
}

HM_TEST(a_truncated_notification_is_short_rather_than_silent)
{
    const uint8_t stub[] = { 0x90, 0x00 };
    const uint8_t started[] = { 0x82, 0x01 };
    const uint8_t cal_ack[] = { 0xa2, 0x01 };
    uint8_t       cal_result[1 + 64];

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);

    /* Not a whole record.  §3 gives no length field, no sequence number and no
     * checksum, so there is nothing to resynchronise on — the notification is
     * dropped, loudly. */
    feed(f, stub, sizeof(stub));
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_SHORT_FRAME), 1u);
    HM_ASSERT_EQ(f->nlive, 0u);

    /* Messages in §5.1's table that a client has nothing to do with are
     * accepted and ignored — never errors, never link-down. */
    memset(cal_result, 0, sizeof(cal_result));
    cal_result[0] = 0x94;
    feed(f, started, sizeof(started));
    /* An ack with no marker outstanding is a duplicate or the tail of a routine
     * that already ended: it moves no phase and is not evented. */
    feed(f, cal_ack, sizeof(cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_IDLE);
    HM_ASSERT_EQ(count_events(f, HM_EV_CALIBRATION_PHASE), 0u);

    /*
     * ⚠ A `0x94` NOBODY ASKED FOR STILL MOVES THE FLAG.  It is not a verdict —
     * the device emits it for every `a2 01` and applies the transform every
     * time (§8.2) — but it does mean the device re-referenced its own stream at
     * that instant, so continuing to label samples UNCALIBRATED would be a
     * confident claim about a frame that has just changed.  The state drops to
     * UNKNOWN and the arrival is reported.
     */
    feed(f, cal_result, sizeof(cal_result));
    HM_ASSERT_EQ(count_events(f, HM_EV_LINK_DOWN), 0u);
    HM_ASSERT_EQ(count_events(f, HM_EV_UNKNOWN_MESSAGE), 0u);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_CALIBRATION_UNSOLICITED), 1u);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_IDLE);
    fake_close(f);
}

HM_TEST(a_legacy_stream_is_decoded_and_marked_not_time_alignable)
{
    hm_session_config cfg = hm_session_config_default();
    uint8_t           frame[1 + 42];
    const uint8_t     started[] = { 0x82, 0x01 };

    /*
     * ⚠ The legacy `82` start yields 0x7f frames with NO record header and NO
     * tick counters (§6.3.1), so it carries no device clock at all and history
     * retrieval against it is impossible — `a1` addresses a header that does
     * not exist.  The library decodes it, because a recording may contain one,
     * and marks every sample so a consumer cannot feed it into a path that
     * assumes otherwise.
     */
    cfg.stream_config = hm_stream_config_legacy();
    {
        fake *f = fake_open(&cfg);
        bring_up(f);
        HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
        drain(f);
        HM_ASSERT_EQ(f->written[f->nwritten - 1u][0], 0x82);
        HM_ASSERT_EQ(f->written_len[f->nwritten - 1u], 1u); /* no config byte */
        HM_ASSERT_EQ(count_warnings(f, HM_WARN_LEGACY_STREAM), 1u);

        memset(frame, 0, sizeof(frame));
        frame[0] = 0x7f;
        put_be16(frame + 1, 16384u);  /* lower arm w */
        put_be16(frame + 1 + 21, 16384u); /* palm w */
        feed(f, started, sizeof(started));
        feed(f, frame, sizeof(frame));

        HM_ASSERT_EQ(f->nlive, 1u);
        if (f->nlive == 1u) {
            const hm_sample *sm = &f->live[0];
            HM_ASSERT((sm->flags & (uint16_t)HM_SAMPLE_INDEX_MISSING) != 0u);
            HM_ASSERT((sm->flags & (uint16_t)HM_SAMPLE_NOT_TIME_ALIGNABLE) != 0u);
            HM_ASSERT((sm->flags & (uint16_t)HM_SAMPLE_TICKS_MISSING) != 0u);
            /* No index means nothing the fit could anchor on, so there is no
             * host time — and it says so rather than inventing one. */
            HM_ASSERT((sm->flags & (uint16_t)HM_SAMPLE_NO_FIT) != 0u);
            HM_ASSERT(sm->host_time_us == HM_TIME_UNKNOWN);
            HM_ASSERT_EQ(sm->uncertainty_us, UINT32_MAX);
            HM_ASSERT(sm->lower_arm.device_time_us == HM_TIME_UNKNOWN);
        }
        fake_close(f);
    }
}

/* ------------------------------------------------------------------------ */
/* Increment 4 — calibration (design §7, spec §8)                            */
/* ------------------------------------------------------------------------ */
HM_TEST(the_routine_walks_both_markers_and_a_skipped_check_never_says_calibrated)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);

    /* Nothing is written until the user is in pose 0: begin() is a UI state, not
     * a device transaction. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_AWAIT_HORIZONTAL);
    HM_ASSERT_EQ(count_writes(f, 0xa2, 0u), 0u);

    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_MARKING_POSE0);
    HM_ASSERT_EQ(count_writes(f, 0xa2, 0u), 1u);
    HM_ASSERT_EQ(f->written_len[f->nwritten - 1u], 2u);
    HM_ASSERT_EQ(f->written[f->nwritten - 1u][1], 0x00); /* `a2 00` */

    /* ⚠ The device answers both markers with the same `a2 01`; which marker it
     * acknowledges is our own state and nothing on the wire (§8.2). */
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_MARKING_POSE1);
    HM_ASSERT_EQ(count_writes(f, 0xa2, 0u), 2u);
    HM_ASSERT_EQ(f->written[f->nwritten - 1u][1], 0x01); /* `a2 01` */

    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_APPLYING);

    feed_cal_result(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_VERIFYING);
    /* Six transitions, every one of them evented, so a UI renders progress from
     * the queue rather than polling. */
    HM_ASSERT_EQ(count_events(f, HM_EV_CALIBRATION_PHASE), 6u);

    /*
     * ⚠ THE ONE THIS INCREMENT EXISTS FOR.  The presence check is skipped, so
     * every later sample reads HM_CAL_UNKNOWN — never CALIBRATED.  The device
     * applies the transform for every `a2 01` including attempts an application
     * would reject, so "we issued the markers" is not evidence that calibration
     * took, and the recording has to say we did not check rather than imply we
     * did (§7.5, design review 12.6).
     */
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    {
        size_t before = f->nlive;
        f->now += 40000;
        feed_frame(f, 400u, ticks_for(400u), 0);
        HM_ASSERT(f->nlive > before);
        HM_ASSERT_EQ(f->live[f->nlive - 1u].calibration, (uint8_t)HM_CAL_UNKNOWN);
    }
    /* And no measurement means no angle and no anchor — not a zero and not an
     * empty struct that reads like one. */
    HM_ASSERT(isnan(hm_calibration_presence_angle_deg(f->s)));
    {
        hm_calibration_presence_event anchor;
        HM_ASSERT_EQ(hm_calibration_reference_anchor(f->s, &anchor), HM_ERR_INVALID_STATE);
    }
    fake_close(f);
}

HM_TEST(a_presence_check_at_the_applied_population_confirms_and_keeps_its_anchor)
{
    /* §8.2's applied population: 0.36-0.79° at the reference pose, against a
     * 6° threshold with an order of magnitude of margin. */
    fake *f = cal_measure_at(0.5, HM_PRESENCE_MAX_SAMPLES);

    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_COMPLETE);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);
    HM_ASSERT_EQ(count_events(f, HM_EV_CALIBRATION_PRESENCE), 1u);

    {
        const hm_event *ev = find_event(f, HM_EV_CALIBRATION_PRESENCE);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            const hm_calibration_presence_event *p = &ev->u.calibration_presence;
            HM_ASSERT_EQ(p->samples_used, HM_PRESENCE_MAX_SAMPLES);
            HM_ASSERT_EQ(p->state, (uint8_t)HM_CAL_CALIBRATED);
            HM_ASSERT(p->relative_angle_deg < HM_PRESENCE_CALIBRATED_MAX_DEG);
            /* ⚠ The medoid is a REAL RECORD: its index is one of the run's and
             * its skew is a measurement, not an average (§10.3's 59 ticks). */
            HM_ASSERT(p->sample_index >= 200u && p->sample_index < 200u + HM_PRESENCE_MAX_SAMPLES);
            HM_ASSERT(p->skew_us > 0);
            /* ⚠ A mean without a spread is an estimate without evidence.  The
             * lower arm never moved in this fixture and the palm did, so the
             * two entries must not read the same. */
            HM_ASSERT(p->pose_spread_deg[HM_UNIT_PALM] > 0.0f);
            HM_ASSERT_NEAR(p->pose_spread_deg[HM_UNIT_LOWER_ARM], 0.0f, 1e-3f);
        }
    }

    /* Queryable as well as evented: the event ring is drop-oldest and the pose
     * has passed, so this measurement cannot be re-derived (R14). */
    {
        hm_calibration_presence_event anchor;
        HM_ASSERT_EQ(hm_calibration_reference_anchor(f->s, &anchor), HM_OK);
        HM_ASSERT_NEAR(anchor.relative_angle_deg, hm_calibration_presence_angle_deg(f->s), 1e-6f);
    }

    /* Only now does a sample claim CALIBRATED. */
    f->now += 40000;
    feed_frame(f, 500u, ticks_for(500u), 0);
    HM_ASSERT_EQ(f->live[f->nlive - 1u].calibration, (uint8_t)HM_CAL_CALIBRATED);
    fake_close(f);
}

HM_TEST(a_pose_in_the_uncalibrated_population_is_reported_rather_than_assumed)
{
    /* §8.2: 15.01° after a power cycle with the strap untouched, 18.80° after a
     * plain disconnect — an order of magnitude from anything applied. */
    fake *f = cal_measure_at(15.0, HM_PRESENCE_MAX_SAMPLES);

    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_COMPLETE);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNCALIBRATED);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_CALIBRATION_ABSENT), 1u);
    /* ⚠ The anchor is kept: a measurement that says "absent" is still a
     * measurement, and it is the one that says why. */
    {
        hm_calibration_presence_event anchor;
        HM_ASSERT_EQ(hm_calibration_reference_anchor(f->s, &anchor), HM_OK);
        HM_ASSERT(anchor.relative_angle_deg > HM_PRESENCE_ABSENT_MIN_DEG);
    }
    fake_close(f);
}

HM_TEST(the_indeterminate_band_leaves_the_state_exactly_as_it_was)
{
    /*
     * ⚠ Between the two populations §8.2 measured, so it is evidence of
     * NEITHER.  Moving the state either way on this band is how a presence check
     * becomes a quality score — and §8.2 measured that score inverting: the
     * calibration with no raise at all, carrying no axis information
     * whatsoever, scored 0.70° against 1.96° for the correct routine.
     */
    fake *f = cal_measure_at(8.0, HM_PRESENCE_MAX_SAMPLES);

    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_COMPLETE);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_CALIBRATION_INDETERMINATE), 1u);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_CALIBRATION_ABSENT), 0u);
    fake_close(f);
}

HM_TEST(the_presence_window_measures_what_arrived_rather_than_waiting_for_a_full_run)
{
    /* A held pose is a resting wrist and §6.6 puts the live rate there around
     * 25 Hz, so the buffer would take 2.6 s to fill.  The window closes first
     * and the run is measured on what it has. */
    fake *f = cal_measure_at(0.5, 20u);

    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_VERIFYING); /* still open */
    run_to(f, f->now + (hm_time_us)3 * 1000 * 1000);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_COMPLETE);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);
    {
        const hm_event *ev = find_event(f, HM_EV_CALIBRATION_PRESENCE);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.calibration_presence.samples_used, 20u);
        }
    }
    fake_close(f);
}

HM_TEST(a_presence_run_that_starves_says_it_could_not_measure)
{
    /*
     * ⚠ "No evidence" is not "agreement".  The check was ASKED FOR and could
     * not be taken, so the state stays UNKNOWN, the angle stays NaN, and the
     * consumer is told — a run of three reports a pose spread of 0.0, which is
     * the strongest possible claim from the weakest possible evidence.
     */
    fake *f = cal_measure_at(0.5, 3u);

    run_to(f, f->now + (hm_time_us)3 * 1000 * 1000);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_COMPLETE);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    HM_ASSERT_EQ(count_events(f, HM_EV_CALIBRATION_PRESENCE), 0u);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_PRESENCE_NOT_MEASURED), 1u);
    {
        const hm_event *ev = NULL;
        for (size_t i = 0; i < f->nevents; ++i) {
            if (f->events[i].type == (uint16_t)HM_EV_WARNING &&
                f->events[i].u.warning.code == (uint16_t)HM_WARN_PRESENCE_NOT_MEASURED) {
                ev = &f->events[i];
            }
        }
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.warning.detail_i32, 3); /* how many arrived */
        }
    }
    HM_ASSERT(isnan(hm_calibration_presence_angle_deg(f->s)));
    fake_close(f);
}

HM_TEST(calibration_refuses_without_a_stream_and_while_a_bracket_is_open)
{
    const uint8_t mark_start[] = { 0xa1, 0x02 };
    size_t        writes_before;

    fake *f = fake_open(NULL);
    bring_up(f);

    /* ⚠ §8.2: the device observes a CONTINUOUS RAISE between the markers, which
     * cannot be done from two static samples.  It refuses rather than waiting —
     * there is deliberately no AWAIT_STREAM phase to wait in. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_ERR_NO_STREAM);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_ERR_NO_STREAM); /* STARTING is not RUNNING */
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    drain(f);

    /*
     * ⚠ A retrieval suspends live delivery (§10.1) and holds the write queue, so
     * a marker could not go out and the presence check would have nothing to
     * measure.  Refusing lets a UI retry a second later with the user standing
     * still regardless; the alternative is a calibration that aborts for a
     * reason that has nothing to do with calibration (R8).
     */
    writes_before = f->nwritten;
    feed(f, mark_start, sizeof(mark_start));
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_ERR_BUSY);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_ERR_BUSY);
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_ERR_BUSY);
    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_ERR_BUSY);
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa2, writes_before), 0u);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_AWAIT_HORIZONTAL);

    /* ⚠ ONE EXEMPTION: abort writes nothing, so the quiet period does not apply
     * and a UI can always cancel a routine it has given up on. */
    HM_ASSERT_EQ(hm_calibration_abort(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.calibration_phase.abort_reason, (uint8_t)HM_CAL_ABORT_CALLER);
        }
    }
    /* Nothing to abort twice. */
    HM_ASSERT_EQ(hm_calibration_abort(f->s), HM_ERR_INVALID_STATE);
    fake_close(f);
}

HM_TEST(the_calibration_result_reaches_the_wire_log_verbatim_and_the_decoder_never)
{
    static hm_wire_chunk chunks[128];
    hm_session_config    cfg = hm_session_config_default();
    size_t               results = 0;

    cfg.memory.wire_ring = chunks;
    cfg.memory.wire_ring_capacity = 128u;

    {
        fake *f = fake_open(&cfg);
        bring_up(f);
        HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
        drain(f);
        feed_frame(f, 100u, ticks_for(100u), 0);
        HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
        HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
        drain(f);
        feed(f, k_cal_ack, sizeof(k_cal_ack));
        HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
        drain(f);
        feed(f, k_cal_ack, sizeof(k_cal_ack));
        feed_cal_result(f);

        /*
         * ⚠ RECORD THE BYTES, NOT THE DECODE (§5.6).  The library decodes none
         * of this 64-byte payload — the device applies the transform itself —
         * and §8.2 records TWO encoders for it: firmware 4.5 populates only the
         * high byte of each component, 4.8 uses all 16 bits.  A byte-level
         * recording can be re-decoded by anything that later wants the values;
         * a sample-level one has already thrown the bytes away.
         */
        for (size_t i = 0; i < f->nwire; ++i) {
            if (f->wire[i].direction != (uint8_t)HM_WIRE_DEVICE_TO_HOST ||
                f->wire[i].length == 0u || f->wire[i].data[0] != 0x94) {
                continue;
            }
            results++;
            HM_ASSERT_EQ(f->wire[i].length, 65u); /* id + 64 */
            HM_ASSERT_EQ(f->wire[i].flags & (uint8_t)HM_WIRE_REDACTED, 0);
            /* The first quaternion, w = 16384, intact — §8.2's q1, the palm
             * unit's rotation about z. */
            HM_ASSERT_EQ(f->wire[i].data[1], 0x40);
            HM_ASSERT_EQ(f->wire[i].data[2], 0x00);
        }
        HM_ASSERT_EQ(results, 1u);

        /* And both markers went out as bytes too, so a recording replays the
         * whole transaction and not just its effect. */
        {
            size_t markers = 0;
            for (size_t i = 0; i < f->nwire; ++i) {
                if (f->wire[i].direction == (uint8_t)HM_WIRE_HOST_TO_DEVICE &&
                    f->wire[i].length == 2u && f->wire[i].data[0] == 0xa2) {
                    markers++;
                }
            }
            HM_ASSERT_EQ(markers, 2u);
        }
        fake_close(f);
    }
}

HM_TEST(every_calibration_call_out_of_turn_names_what_is_wrong)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);

    /*
     * ⚠ A pose call out of turn is INVALID_STATE, never a silent no-op and
     * never a marker written anyway: the device reads a CONTINUOUS RAISE
     * between two markers (§8.2), so an `a2 01` with no `a2 00` behind it would
     * calibrate against whatever the arm happened to be doing.
     */
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_ERR_INVALID_STATE);
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_ERR_INVALID_STATE);
    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_ERR_INVALID_STATE);
    HM_ASSERT_EQ(hm_calibration_abort(f->s), HM_ERR_INVALID_STATE); /* nothing running */
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa2, 0u), 0u);

    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    /* A routine already running is not silently restarted underneath a UI that
     * has lost track of it; abort first. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_ERR_INVALID_STATE);
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_ERR_INVALID_STATE);

    /* One run at a time — a second call has the same pose to measure. */
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    feed_cal_result(f);
    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_ERR_BUSY);

    /* And with the link gone it is the LINK that is reported, not the phase the
     * link took with it. */
    hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now + 1000);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_ERR_LINK_DOWN);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_ERR_LINK_DOWN);
    fake_close(f);
}

HM_TEST(the_raise_limit_is_client_policy_and_says_so_when_it_fires)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    /*
     * ⚠ CLIENT POLICY, NOT A DEVICE CONSTRAINT.  §8.2 measured the device
     * returning and applying a result 15.6 s after the first marker — it was the
     * vendor's APPLICATION that rejected it.  The default 6 s is ours.
     */
    run_to(f, f->now + (hm_time_us)7 * 1000 * 1000);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.calibration_phase.abort_reason,
                         (uint8_t)HM_CAL_ABORT_RAISE_TOO_SLOW);
            HM_ASSERT_EQ(ev->u.calibration_phase.previous_phase,
                         (uint8_t)HM_CALP_OBSERVING_RAISE);
            HM_ASSERT(ev->u.calibration_phase.elapsed_us > 0);
        }
    }
    /* Nothing was applied, so the flag is untouched. */
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNCALIBRATED);
    fake_close(f);
}

HM_TEST(a_marker_the_device_never_answers_ends_the_routine_rather_than_hanging)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);

    /* No ack.  ⚠ The bound is not in §8.2 and a stuck UI is: without it a
     * wizard waits on a user holding their arm out with no way back. */
    run_to(f, f->now + (hm_time_us)4 * 1000 * 1000);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            /* ⚠ WHICH wait expired is in `previous_phase`, which is why one
             * bound does not need two abort reasons to stay legible. */
            HM_ASSERT_EQ(ev->u.calibration_phase.abort_reason, (uint8_t)HM_CAL_ABORT_NO_RESULT);
            HM_ASSERT_EQ(ev->u.calibration_phase.previous_phase, (uint8_t)HM_CALP_MARKING_POSE0);
        }
    }
    fake_close(f);
}

HM_TEST(a_result_that_never_comes_aborts_and_a_late_one_still_moves_the_flag)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_APPLYING);

    run_to(f, f->now + (hm_time_us)4 * 1000 * 1000);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.calibration_phase.previous_phase, (uint8_t)HM_CALP_APPLYING);
        }
    }
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNCALIBRATED);

    /*
     * ⚠ AND THEN IT ARRIVES ANYWAY.  Our bound is client policy; the device's
     * is not bounded at all, and it applies the transform for every `a2 01`.
     * The routine stays ABORTED — it is over — but the per-sample flag follows
     * the DEVICE: the streamed orientations are in the anatomical frame from
     * here, and continuing to label them UNCALIBRATED would be a confident lie
     * about a frame that has just changed underneath the consumer.
     */
    feed_cal_result(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_CALIBRATION_UNSOLICITED), 1u);
    f->now += 40000;
    feed_frame(f, 600u, ticks_for(600u), 0);
    HM_ASSERT_EQ(f->live[f->nlive - 1u].calibration, (uint8_t)HM_CAL_UNKNOWN);
    fake_close(f);
}

/*
 * ⚠ §8.2's SHORT FORM.  `0x94` has two forms and the device tells them apart by
 * the notification's total length: 65 bytes of quaternions, or a status byte.
 * A short one is not a truncated long one, and dropping it as one would leave
 * the routine to starve on its own result timeout with the device's answer
 * already on the wire.
 *
 * What the status byte MEANS is unknown and no value of it has been captured,
 * so the library reports it and does not act on it: the routine proceeds and
 * the presence measurement decides, because that tests what the device is
 * emitting rather than what a byte claims about it.
 */
HM_TEST(the_short_form_of_the_result_is_answered_and_its_status_byte_reported)
{
    fake         *f = fake_open(NULL);
    const uint8_t short_form[2] = { 0x94, 0x2A };

    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_APPLYING);

    feed(f, short_form, sizeof(short_form));

    /* Not a short frame, not an unknown message, and not silently swallowed. */
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_SHORT_FRAME), 0u);
    HM_ASSERT_EQ(count_events(f, HM_EV_UNKNOWN_MESSAGE), 0u);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_CALIBRATION_STATUS_FORM), 1u);
    {
        size_t i;
        int    seen = 0;
        for (i = 0; i < f->nevents; ++i) {
            if (f->events[i].type == (uint16_t)HM_EV_WARNING &&
                f->events[i].u.warning.code == (uint16_t)HM_WARN_CALIBRATION_STATUS_FORM) {
                /* The byte itself travels, so a consumer that learns what the
                 * values mean can read them out of an existing recording. */
                HM_ASSERT_EQ(f->events[i].u.warning.detail_i32, 0x2A);
                seen++;
            }
        }
        HM_ASSERT_EQ(seen, 1);
    }

    /* The routine advances exactly as it does for the long form, and the flag
     * drops to UNKNOWN rather than claiming either outcome. */
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_VERIFYING);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    fake_close(f);
}

HM_TEST(aborting_after_the_transform_completes_rather_than_pretending_nothing_happened)
{
    fake *f = cal_open_to_verifying();

    /*
     * ⚠ At VERIFYING the transform is ALREADY APPLIED and no command reverses
     * it: §8.2's device re-references its own stream the instant it emits
     * `0x94`.  So an abort there declines the presence check rather than
     * cancelling a calibration, and reporting ABORTED would tell a consumer
     * nothing happened to a stream whose frame had just changed.
     */
    HM_ASSERT_EQ(hm_calibration_abort(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_COMPLETE);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            /* COMPLETE, and the reason says who ended it. */
            HM_ASSERT_EQ(ev->u.calibration_phase.phase, (uint8_t)HM_CALP_COMPLETE);
            HM_ASSERT_EQ(ev->u.calibration_phase.abort_reason, (uint8_t)HM_CAL_ABORT_CALLER);
        }
    }
    /* ⚠ Complete is not calibrated: the check was declined, so the angle stays
     * NaN and every later sample says UNKNOWN. */
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    HM_ASSERT(isnan(hm_calibration_presence_angle_deg(f->s)));
    fake_close(f);
}

HM_TEST(link_down_aborts_the_routine_and_takes_the_anchor_with_it)
{
    fake *f = cal_measure_at(0.5, HM_PRESENCE_MAX_SAMPLES);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);

    /*
     * ⚠ §8.3 measured 0.70° immediately before dropping a link and 18.80° at
     * the same pose after reconnecting, strap untouched and never removed.  The
     * anchor goes with it: it describes a reference pose under a mount transform
     * the device no longer holds.
     */
    hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now + 1000);
    drain(f);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNCALIBRATED);
    HM_ASSERT(isnan(hm_calibration_presence_angle_deg(f->s)));
    {
        hm_calibration_presence_event anchor;
        HM_ASSERT_EQ(hm_calibration_reference_anchor(f->s, &anchor), HM_ERR_INVALID_STATE);
    }
    fake_close(f);
}

HM_TEST(a_routine_in_flight_names_the_link_rather_than_the_stream_it_also_lost)
{
    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));

    /* A link drop stops the stream too, so both reasons are true — and the
     * larger one is the one worth reporting: a stream stop ends the routine,
     * where a link drop destroys the calibration outright (§8.3). */
    hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now + 1000);
    drain(f);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.calibration_phase.abort_reason, (uint8_t)HM_CAL_ABORT_LINK_LOST);
        }
    }
    fake_close(f);
}

/*
 * ⚠⚠ A STOP THE DEVICE NEVER ANSWERS MUST NOT WEDGE THE SESSION.
 *
 * implementation-review I9.  stop_stream() set HM_STREAM_STOPPING and queued
 * `83`, and the only exit was the device's own `0x83`.  Nothing armed a
 * deadline and no §5.7 row covered it — the ONE device-facing wait in that
 * table without a bound, where the bring-up, the stream START, the calibration
 * round trip, the bracket and the power-off linger all have one.
 *
 * ⚠ The failure is silent and total: start_stream() returns
 * HM_ERR_INVALID_STATE, every hm_calibration_* call returns HM_ERR_NO_STREAM,
 * history never issues, hm_history_resident_range() refuses — and no event or
 * warning explains any of it.  A host that simply stopped draining
 * poll_writes(), so the `83` never left, reaches the same place.
 */
HM_TEST(a_stop_the_device_never_answers_is_bounded_like_every_other_wait)
{
    hm_session_config cfg = hm_session_config_default();
    fake             *f;

    f = fake_open(&cfg);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);

    HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
    drain(f);
    /* ⚠ And the device says nothing at all from here on. */

    /* The wait is visible as a deadline rather than as a hope. */
    HM_ASSERT(hm_session_next_due_us(f->s) <= f->now + cfg.policy.stream_start_timeout_us);

    run_to(f, f->now + (hm_time_us)cfg.policy.stream_start_timeout_us + 1000);

    HM_ASSERT_EQ(count_warnings(f, HM_WARN_STREAM_STOP_TIMEOUT), 1u);
    HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_STOPPED), 1u);
    /* ⚠ And the session is usable again, which is the whole point. */
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    fake_close(f);
}

/* And the ordinary case is unaffected: the reply lands, the watchdog does not. */
HM_TEST(a_stop_the_device_answers_leaves_no_watchdog_behind)
{
    const uint8_t stopped[] = { 0x83, 0x01 };
    fake         *f = fake_open(NULL);

    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
    drain(f);
    feed(f, stopped, sizeof(stopped));

    run_to(f, f->now + (hm_time_us)10 * 1000 * 1000);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_STREAM_STOP_TIMEOUT), 0u);
    HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_STOPPED), 1u);
    fake_close(f);
}

HM_TEST(a_stopped_stream_ends_the_routine_and_a_restart_does_not_carry_one_across)
{
    const uint8_t stopped[] = { 0x83, 0x01 };
    fake         *f = cal_measure_at(0.5, HM_PRESENCE_MAX_SAMPLES);

    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);
    HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
    drain(f);
    feed(f, stopped, sizeof(stopped));
    /* Nothing is being sampled anywhere while stopped, so nothing is labelled
     * and the state is left alone. */
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);

    /*
     * ⚠ But a restart produces samples again, and whether it costs the
     * device's transform is untested where a disconnect demonstrably does
     * (§7.6, §8.3).  The choice is between samples that keep claiming CALIBRATED
     * across an unmeasured boundary and samples that say we no longer know; the
     * second is recoverable and the first is permanent and invisible.
     */
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_RESTARTED), 1u);
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNKNOWN);
    {
        hm_calibration_presence_event anchor;
        HM_ASSERT_EQ(hm_calibration_reference_anchor(f->s, &anchor), HM_ERR_INVALID_STATE);
    }
    fake_close(f);
}

HM_TEST(a_stream_that_stops_mid_routine_aborts_it_with_the_reason)
{
    const uint8_t stopped[] = { 0x83, 0x01 };

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 100u, ticks_for(100u), 0);
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
    drain(f);
    feed(f, stopped, sizeof(stopped));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_ABORTED);
    {
        const hm_event *ev = last_phase_event(f);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.calibration_phase.abort_reason, (uint8_t)HM_CAL_ABORT_STREAM_LOST);
        }
    }
    /* And the raise limit went with it: an aborted routine leaves no deadline
     * armed behind it. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_ERR_NO_STREAM);
    fake_close(f);
}

HM_TEST(a_restarted_stream_is_reported_never_silent)
{
    const uint8_t stopped[] = { 0x83, 0x01 };

    fake *f = fake_open(NULL);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    feed_frame(f, 1u, ticks_for(1u), 0);
    {
        uint64_t first_id = hm_session_stream_id(f->s);

        /* ⚠ One stream, opened once, left open (AR B8): a second start while
         * one is running is refused rather than obeyed. */
        HM_ASSERT(hm_session_start_stream(f->s) < HM_OK);

        HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
        drain(f);
        feed(f, stopped, sizeof(stopped));
        HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_STOPPED), 1u);

        HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
        drain(f);
        /* §7.6 lists restarting FIRST among the five silent ways capture goes
         * wrong: it clears the buffer, resets the index space and starts the
         * clock fit from nothing. */
        HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_RESTARTED), 1u);
        HM_ASSERT(hm_session_stream_id(f->s) != first_id);
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* History retrieval — increments 5 and 6                                     */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ WHAT A DESK CANNOT TEST, AND WHY THESE TESTS STILL EARN THEIR KEEP.
 *
 * §7.3's buffer is MOTION-ADAPTIVE: index step 8 (≈100 Hz) at rest and step 1
 * (≈799 Hz) only in fast motion, measured across 25 retrievals and 17,739 steps.
 * So a synthetic step-1 reply here proves the gather HANDLES a full-rate reply;
 * it cannot prove a real device produces one, and a real device at rest returns
 * an even one-in-eight that is indistinguishable from a broken full-rate path.
 * The evidence for the premise is hardware's: `swings.hmwire` carries five
 * mid-stream pulls whose longest unbroken step-1 runs were 278, 292, 308, 294
 * and 330 records — 413 ms of uninterrupted 799 Hz in the best of them.
 *
 * What these tests DO pin is everything the library decides: which bytes go
 * out, what is discarded, how coverage is reported, when a request is refused,
 * and — the part hardware taught us and the reviewed design did not know — that
 * the index→host mapping is piecewise across every pull.
 */
#define HIST_LIVE_STEP 32u /* ≈25 Hz live: a decimated view of the internal rate */
#define HIST_TRUE_RATE 799.2

static const uint8_t k_mark_start[] = { 0xa1, 0x02 };
static const uint8_t k_mark_end[] = { 0xa1, 0x01 };

/* A session streaming live long enough for the fit to have separated the rate
 * from the offset, with the live-vs-history digest ring turned ON — it is off
 * by default, and with it off a block reports live_overlap_samples = 0, which
 * means NO EVIDENCE rather than agreement. */
static fake *hist_open(uint32_t frames, uint32_t first_index)
{
    hm_session_config cfg = hm_session_config_default();
    hm_time_us        t0;
    fake             *f;

    cfg.memory.digest_ring_capacity = HM_DIGEST_RING_RECOMMENDED;
    f = fake_open(&cfg);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);

    t0 = f->now;
    for (uint32_t i = 0; i < frames; ++i) {
        uint32_t index = first_index + i * HIST_LIVE_STEP;
        /* ⚠ BLE delay is ONE-SIDED — a notification can be late, never early —
         * which is why the fit is a lower envelope.  The spread matters here as
         * well as the offset: it is what `precision_us` measures, and an
         * alignment budget is judged against it. */
        hm_time_us jitter = 200 + (hm_time_us)((i * 7919u) % 5000u);
        f->now = t0 + (hm_time_us)((double)(i * HIST_LIVE_STEP) / HIST_TRUE_RATE * 1e6) + jitter;
        hm_session_tick(f->s, f->now);
        feed_frame(f, (uint16_t)(index & 0xffffu), ticks_for(index), 0);
    }
    return f;
}

/* One live frame, continuing the stream from `index`. */
static void hist_live_frame(fake *f, uint32_t index)
{
    f->now += 40000;
    hm_session_tick(f->s, f->now);
    feed_frame(f, (uint16_t)(index & 0xffffu), ticks_for(index), 0);
}

/* Records inside an already-open bracket, ~260 notifications/s (§7.3). */
static void feed_history_records(fake *f, uint32_t first, uint32_t last, uint32_t step)
{
    for (uint32_t i = first; i <= last; i += step) {
        f->now += 3800;
        feed_frame(f, (uint16_t)(i & 0xffffu), ticks_for(i), 0);
    }
}

/* A whole retrieval: the start marker, the records, the end marker. */
static void feed_history(fake *f, uint32_t first, uint32_t last, uint32_t step)
{
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, first, last, step);
    feed(f, k_mark_end, sizeof(k_mark_end));
}

/*
 * A request for exactly the indices [first, last], expressed in HOST time the
 * way a consumer would.  ⚠ The one-microsecond nudge at each end is the
 * half-open/inclusive boundary of hm_clock_index_range_for_time(), pushed away
 * from the rounding edge rather than sat on it.
 */
static hm_history_request hist_request(fake *f, uint32_t first, uint32_t last)
{
    hm_clock_snapshot  snap;
    hm_history_request r;

    memset(&r, 0, sizeof(r));
    HM_ASSERT_EQ(hm_session_clock(f->s, &snap), HM_OK);
    r.window.start_us = hm_clock_to_host_us(&snap, first) - 1;
    r.window.end_us = hm_clock_to_host_us(&snap, last) + 1;
    r.deadline_us = f->now + (hm_time_us)60 * 1000 * 1000;
    r.max_attempts = 1u;
    return r;
}

static const hm_event *find_last_event(const fake *f, hm_event_type type)
{
    const hm_event *found = NULL;
    for (size_t i = 0; i < f->nevents; ++i) {
        if (f->events[i].type == (uint16_t)type) {
            found = &f->events[i];
        }
    }
    return found;
}

/* The index of the last `a1` written, or SIZE_MAX. */
static size_t last_a1(const fake *f)
{
    size_t found = SIZE_MAX;
    for (size_t i = 0; i < f->nwritten; ++i) {
        if (f->written_len[i] == 5u && f->written[i][0] == 0xa1) {
            found = i;
        }
    }
    return found;
}

static uint16_t a1_first(const fake *f, size_t at)
{
    return (uint16_t)(((uint16_t)f->written[at][1] << 8) | f->written[at][2]);
}

static uint16_t a1_last(const fake *f, size_t at)
{
    return (uint16_t)(((uint16_t)f->written[at][3] << 8) | f->written[at][4]);
}

/* ------------------------------------------------------------------------ */
/* Increment 5 — the gather, end to end                                       */
/* ------------------------------------------------------------------------ */
HM_TEST(a_mid_stream_pull_is_complete_dense_and_dated_by_its_own_fit)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             live_before;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, first, last);
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    HM_ASSERT(id != 0u);
    HM_ASSERT_EQ(hm_history_pending(f->s), 1u);
    drain(f);

    /* ⚠ ONE `a1`, in place, with the range re-wrapped to u16be (§7.1).  The
     * stream is never stopped: doing so would cost a fresh index space, a clock
     * fit rebuilt from nothing and a calibration to re-run (§7.5). */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    HM_ASSERT_EQ(count_writes(f, 0x83, writes_before), 0u);
    {
        size_t at = last_a1(f);
        HM_ASSERT(at != SIZE_MAX);
        HM_ASSERT_EQ(a1_first(f, at), (uint16_t)first);
        HM_ASSERT_EQ(a1_last(f, at), (uint16_t)last);
    }

    /* collect() never blocks; it says PENDING until a block exists. */
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_PENDING);
    HM_ASSERT(b == NULL);

    live_before = f->nlive;
    feed_history(f, first, last, 1u);

    /*
     * ⚠ poll_live() YIELDS NOTHING ACROSS THE BRACKET.  §10.1: live and history
     * `0x90` frames are byte-identical and the bracket is the only
     * discriminator, so 400 bulk arrivals reaching the live ring would look
     * exactly like a burst of real motion.
     */
    HM_ASSERT_EQ(f->nlive, live_before);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b == NULL) {
        fake_close(f);
        return;
    }
    HM_ASSERT_EQ(hm_history_pending(f->s), 0u);

    HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_COMPLETE);
    HM_ASSERT_EQ(b->attempts, 1u);
    HM_ASSERT_EQ(b->sample_count, (size_t)(last - first + 1u));
    HM_ASSERT_EQ(b->delivered_count, 1u);
    HM_ASSERT_EQ(b->delivered_indices[0].first, first);
    HM_ASSERT_EQ(b->delivered_indices[0].last, last);
    HM_ASSERT_EQ(b->gap_count, 0u);
    HM_ASSERT_EQ(b->largest_gap_us, 0u);
    HM_ASSERT_NEAR(b->coverage_fraction, 1.0, 1e-9);
    /* ⚠ Density AND coverage, never one alone: a count cannot tell "dense over
     * half the range" from "half-dense over all of it" (AR C4). */
    HM_ASSERT_NEAR(b->density, 1.0, 1e-9);
    HM_ASSERT_NEAR(b->achieved_hz, HIST_TRUE_RATE, 1.0);
    HM_ASSERT_EQ(b->coverage_overflowed, 0u);
    HM_ASSERT_EQ(b->layout_version, HM_SAMPLE_LAYOUT_VERSION);
    HM_ASSERT_EQ(b->sample_stride, (uint32_t)sizeof(hm_sample));
    HM_ASSERT_EQ(b->config.bits, HM_CONFIG_OBSERVED_DEFAULT);

    /*
     * ⚠ THE BLOCK IS INTERNALLY REPRODUCIBLE: the fit it carries is the one
     * that dated every sample in it, so re-deriving a timestamp a year later
     * yields the same answer it did on the day (AR C9).
     */
    for (size_t i = 0; i < b->sample_count; ++i) {
        const hm_sample *sm = &b->samples[i];
        HM_ASSERT_EQ(sm->sample_index, first + (uint32_t)i);
        HM_ASSERT_EQ(sm->source, (uint8_t)HM_SOURCE_HISTORY);
        /* ⚠ Bulk arrival timestamps carry no information at all (§10.1). */
        HM_ASSERT_EQ(sm->host_recv_us, HM_TIME_UNKNOWN);
        HM_ASSERT((sm->flags & (uint16_t)HM_SAMPLE_NO_FIT) == 0u);
        HM_ASSERT_EQ(sm->host_time_us, hm_clock_to_host_us(&b->fit, sm->sample_index));
        /* Device time comes from the frame alone and never waits for a fit. */
        HM_ASSERT(sm->lower_arm.device_time_us != HM_TIME_UNKNOWN);
        if (i > 0u) {
            HM_ASSERT(sm->sample_index > b->samples[i - 1u].sample_index);
            HM_ASSERT(sm->host_time_us > b->samples[i - 1u].host_time_us);
        }
    }

    /*
     * ⚠ THE LIVE-VS-HISTORY AGREEMENT, and the COUNT is asserted beside the
     * mismatch: a zero mismatch count over zero samples reads identically to a
     * check that silently stopped running (§8.8).
     */
    HM_ASSERT(b->live_overlap_samples > 0u);
    HM_ASSERT_EQ(b->live_overlap_mismatches, 0u);

    /* ⚠ The pull cost the session a hole of its own width, and nothing on the
     * wire marks it (§7.5, B11). */
    HM_ASSERT(b->self_recording_gap.end_us > b->self_recording_gap.start_us);

    {
        const hm_event *ev = find_last_event(f, HM_EV_HISTORY_BLIND_SPAN);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.history_blind_span.request_id, id);
        }
        HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_STARTED), 1u);
        HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_READY), 1u);
        /* Not an alarm: the reply's shape was exactly what was asked for. */
        HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_HOLED), 0u);
        HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_SHORT), 0u);
        HM_ASSERT_EQ(count_warnings(f, HM_WARN_LIVE_GAP), 0u);
    }

    hm_history_block_release(b);
    fake_close(f);
}

HM_TEST(a_block_outlives_the_session_that_produced_it)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             count;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7099u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, 7000u, 7099u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b == NULL) {
        fake_close(f);
        return;
    }
    count = b->sample_count;

    /* A second reservation that is never collected — destroy() must release it
     * rather than leak it. */
    {
        uint64_t orphan = 0u;
        hist_live_frame(f, 17800u);
        req = hist_request(f, 7200u, 7299u);
        HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &orphan), HM_OK);
        HM_ASSERT(orphan != id);
    }

    fake_close(f);

    /*
     * ⚠ THE BLOCK IS STILL VALID, and this is the contract §8.4.2 states: PPS
     * collects on the I/O thread, hands the block to a worker pool, and
     * releases it there, possibly after the session is gone.  It owns its
     * memory and a COPY of the allocator, which is what makes that legal.
     */
    HM_ASSERT_EQ(b->sample_count, count);
    HM_ASSERT_EQ(b->samples[0].sample_index, 7000u);
    hm_history_block_release(b);

    /*
     * ⚠ A pointer that never came from here is rejected rather than freed.
     *
     * ⚠ AND A DOUBLE RELEASE IS NOT TESTED, BECAUSE IT CANNOT BE MADE SAFE.
     * The first draft of this test called release() twice and asserted it was a
     * no-op; ASan showed the magic read is itself a use-after-free.  A guard
     * that happens to work while the allocator has not reused the page is the
     * shape of a bug that passes every test and fails in the field, so the
     * header now says "release exactly once, as with free()" instead.
     */
    {
        /* Sized past the public struct so the magic read stays in bounds
         * whatever private header sits behind it. */
        static union {
            hm_history_block pub;
            uint8_t          raw[sizeof(hm_history_block) + 256];
        } foreign;
        memset(&foreign, 0, sizeof(foreign));
        hm_history_block_release(&foreign.pub);
    }
}

HM_TEST(reserve_refuses_at_the_call_site_what_cannot_possibly_succeed)
{
    hm_session_config  cfg = hm_session_config_default();
    hm_history_request req;
    uint64_t           id = 99u;
    fake              *f;

    /* ⚠ Before any live frame there is no mapping from a host-time window to an
     * index range, so there is genuinely nothing to ask the device for.  A
     * STRUCTURAL refusal, unrelated to alignment quality. */
    f = fake_open(&cfg);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    req = hm_history_request_around(NULL, f->now);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_ERR_NO_FIT);
    HM_ASSERT_EQ(id, 0u);
    fake_close(f);

    f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7099u);
    {
        hm_history_request bad = req;
        bad.window.end_us = bad.window.start_us;
        HM_ASSERT_EQ(hm_history_reserve(f->s, &bad, &id), HM_ERR_INVALID_ARG);
        bad.window.end_us = bad.window.start_us - 1000;
        HM_ASSERT_EQ(hm_history_reserve(f->s, &bad, &id), HM_ERR_INVALID_ARG);
    }
    {
        /* ⚠ The window's last sample does not exist until end_us, so a deadline
         * at or before it is unsatisfiable the moment it is made (R11). */
        hm_history_request bad = req;
        bad.deadline_us = bad.window.end_us;
        HM_ASSERT_EQ(hm_history_reserve(f->s, &bad, &id), HM_ERR_INVALID_ARG);
    }
    {
        /* Narrower than one sample period: it maps to no index at all. */
        hm_history_request bad = req;
        bad.window.end_us = bad.window.start_us + 1;
        HM_ASSERT_EQ(hm_history_reserve(f->s, &bad, &id), HM_ERR_INVALID_ARG);
    }
    {
        /* ⚠ Wider than the gather area — REFUSED, never truncated (R11): a
         * silently clipped window returns a block that looks complete. */
        hm_history_request bad = req;
        bad.window.end_us = bad.window.start_us + (hm_time_us)60 * 1000 * 1000;
        bad.deadline_us = bad.window.end_us + 1000;
        HM_ASSERT_EQ(hm_history_reserve(f->s, &bad, &id), HM_ERR_BUFFER_TOO_SMALL);
    }

    /* ⚠ The queue is short on purpose: requests are serialised and a pull takes
     * about as long as its window spans, so a deeper queue would only be
     * reserving windows certain to be evicted by their turn (§8.6). */
    {
        uint64_t ids[8];
        size_t   accepted = 0u;
        for (size_t i = 0; i < 8u; ++i) {
            if (hm_history_reserve(f->s, &req, &ids[i]) == HM_OK) {
                accepted++;
            } else {
                HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_ERR_BUSY);
                break;
            }
        }
        HM_ASSERT(accepted > 0u);
        HM_ASSERT(accepted < 8u);
        HM_ASSERT_EQ(hm_history_pending(f->s), accepted);
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* ⚠ §6.1.1 — the mapping is piecewise, and a retrieval is where it breaks    */
/* ------------------------------------------------------------------------ */
HM_TEST(the_fit_re_anchors_at_every_pull_and_a_queued_request_keeps_its_own_mapping)
{
    hm_history_request req;
    hm_history_block  *ba = NULL;
    hm_history_block  *bb = NULL;
    uint64_t           id_a = 0u;
    uint64_t           id_b = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    writes_before = f->nwritten;
    req = hist_request(f, 7000u, 7199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id_a), HM_OK);
    req = hist_request(f, 8000u, 8199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id_b), HM_OK);
    drain(f);

    /* ⚠ SERIALISED: a second `a1` cannot be issued until the first completes
     * (AR B16), even though both windows closed long ago. */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);

    feed_history(f, 7000u, 7199u, 1u);

    /*
     * ⚠ AND STILL ONE.  §10.2's wrap budget is ±32,768 ticks; one stall is
     * ~23,500 of them — 72 % — where a pull-free live gap uses 12.  Two pulls
     * before the next live frame exceed it and the tick unwrapper silently
     * picks the wrong wrap, so the second request waits for a frame.
     */
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);

    hist_live_frame(f, 17800u);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 2u);
    feed_history(f, 8000u, 8199u, 1u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id_a, &ba), HM_OK);
    HM_ASSERT_EQ(hm_history_collect(f->s, id_b, &bb), HM_OK);
    HM_ASSERT(ba != NULL && bb != NULL);
    if (ba == NULL || bb == NULL) {
        fake_close(f);
        return;
    }
    HM_ASSERT_EQ(ba->status, (uint8_t)HM_HIST_COMPLETE);
    HM_ASSERT_EQ(bb->status, (uint8_t)HM_HIST_COMPLETE);

    /*
     * ⚠⚠ THE POINT OF THIS TEST.  The second request's window closed BEFORE the
     * first request's pull, so it is dated by the mapping that was in force
     * then — the same snapshot the first block carries — and not by the one
     * re-anchored on the far side of a 23,500-tick stall it knows nothing
     * about.  §10.2 measured what the other choice costs: a fit anchored once
     * was out by 111,311 ticks after five pulls, 1.70 wraps of 65,536.
     */
    HM_ASSERT_EQ(ba->fit.anchor_host_us, bb->fit.anchor_host_us);
    HM_ASSERT_EQ(ba->fit.anchor_index, bb->fit.anchor_index);
    HM_ASSERT_EQ(ba->fit.first_index, bb->fit.first_index);
    HM_ASSERT(bb->fit.observations > 100);
    for (size_t i = 0; i < bb->sample_count; ++i) {
        HM_ASSERT_EQ(bb->samples[i].host_time_us,
                     hm_clock_to_host_us(&bb->fit, bb->samples[i].sample_index));
        HM_ASSERT((bb->samples[i].flags & (uint16_t)HM_SAMPLE_NO_FIT) == 0u);
    }

    /* ⚠ And the SESSION's fit did re-anchor: two pulls, two new stretches, so
     * what it carries now is the handful of frames since the last one — and its
     * first observation is on the far side of both. */
    hist_live_frame(f, 17832u);
    {
        hm_clock_snapshot snap;
        HM_ASSERT_EQ(hm_session_clock(f->s, &snap), HM_OK);
        HM_ASSERT_EQ(snap.observations, 1);
        HM_ASSERT_EQ(snap.first_index, 17832u);
        HM_ASSERT(snap.first_index > ba->fit.first_index);
        /* One rate per connection: the offset restarted, the rate did not. */
        HM_ASSERT((snap.flags & (uint32_t)HM_CLOCK_HAS_FIT) != 0u);
    }

    hm_history_block_release(ba);
    hm_history_block_release(bb);
    fake_close(f);
}

HM_TEST(a_window_on_the_far_side_of_a_pull_is_refused_rather_than_addressed_wrong)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, 7000u, 7199u, 1u);
    {
        hm_history_block *done = NULL;
        HM_ASSERT_EQ(hm_history_collect(f->s, id, &done), HM_OK);
        hm_history_block_release(done);
    }
    hist_live_frame(f, 17800u);
    hist_live_frame(f, 17832u);

    /*
     * Now ask — for the first time — for a window that predates that pull.  The
     * fit has re-anchored, so the window maps to indices TOO LOW by the width
     * of the stall: the `a1` would fetch a different span than the caller
     * asked for and every check downstream would pass on it.
     *
     * ⚠ Refused whatever `alignment_budget_us` says, because what is missing is
     * the mapping itself rather than confidence in it.  A block of the wrong
     * samples is worse than no block.
     */
    writes_before = f->nwritten;
    req = hist_request(f, 7000u, 7199u);
    req.alignment_budget_us = 0u; /* the quality gate is explicitly OFF */
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 0u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_REFUSED_ALIGNMENT);
        HM_ASSERT_EQ(b->sample_count, 0u);
        HM_ASSERT_EQ(b->attempts, 0u);
        /* ⚠ The refusal is still an artefact: it carries the window, the fit it
         * was judged against and the tag, so a capture records that we declined
         * rather than recording nothing (AR B2). */
        HM_ASSERT_EQ(b->requested.start_us, req.window.start_us);
        hm_history_block_release(b);
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Increment 6 — the failure shapes                                           */
/* ------------------------------------------------------------------------ */
HM_TEST(an_over_wide_request_comes_back_holed_and_not_short)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, first, last);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /*
     * §7.5's measured shape: 58 % of the requested span, delivered as two dense
     * runs with a hole between them.  ⚠ That is HOLED, not SHORT, and the
     * distinction decides whether a metric computed at impact exists at all —
     * a contiguous short block is at least obviously short, where a holed one
     * spans the whole range and looks complete to a count.
     */
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, first, first + 115u, 1u);
    feed_history_records(f, last - 115u, last, 1u);
    feed(f, k_mark_end, sizeof(k_mark_end));

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b == NULL) {
        fake_close(f);
        return;
    }
    HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_HOLED);
    HM_ASSERT_EQ(b->sample_count, 232u);
    HM_ASSERT_EQ(b->delivered_count, 2u);
    HM_ASSERT_NEAR(b->coverage_fraction, 0.58, 0.005);
    /* ⚠ Dense over 58 %, not half-dense over all of it — and only the two
     * numbers together can say which (AR C4).  The spacing inside the two runs
     * is step 1, so density reads the full internal rate while coverage reads
     * 0.58; a uniformly half-dense reply of the same count reads 0.5 and 0.5.
     * The hole between them is `largest_gap_us`, asserted below. */
    HM_ASSERT_NEAR(b->density, 1.0, 1e-9);
    HM_ASSERT(b->largest_gap_us > 0u);
    HM_ASSERT_EQ(b->gap_count, 1u);
    if (b->gap_count == 1u) {
        HM_ASSERT_EQ(b->gaps[0].kind, (uint8_t)HM_GAP_NOT_DELIVERED);
        HM_ASSERT_EQ(b->gaps[0].indices.first, first + 116u);
        HM_ASSERT_EQ(b->gaps[0].indices.last, last - 116u);
    }
    /* 168 indices is far past §7.3's step-8 floor, so this holing cannot be the
     * motion-adaptive buffer and IS worth a warning. */
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_HOLED), 1u);
    hm_history_block_release(b);
    fake_close(f);
}

HM_TEST(an_at_rest_reply_is_holed_at_the_hundred_hertz_floor_and_that_is_not_an_error)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, first, last);
    /* ⚠ Refill is ON, and the point of this test is that it does not fire. */
    req.refill_gaps = true;
    req.max_attempts = 3u;
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /*
     * ⚠ §7.3: index step 8 is a HARD FLOOR — across 17,739 measured steps none
     * exceeded 8 and none was 0 — and it is what a still wrist returns.  This
     * reply is the buffer working correctly, and it is INDISTINGUISHABLE at a
     * desk from a broken full-rate path.  The library must not call it a fault.
     */
    feed_history(f, first, last, 8u);
    drain(f);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b == NULL) {
        fake_close(f);
        return;
    }
    HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_HOLED);
    HM_ASSERT_EQ(b->sample_count, 50u);
    /* ⚠ §7.3's floor, stated as a density: the spacing IS step 8, so the field
     * reads 1/8.  It read 1.000 here until implementation-review I1 — the
     * divisor was the delivered count, which the session increments on the same
     * line as the numerator. */
    HM_ASSERT_NEAR(b->density, 0.125, 1e-9);
    /* §7.3's own number for step 8, arrived at from the delivered indices. */
    HM_ASSERT_NEAR(b->achieved_hz, 99.9, 1.5);

    /* ⚠ NO WARNING.  Every reply is holed and the holes are not an error; an
     * alarm on the normal shape of the data is how a real one stops being read. */
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_HOLED), 0u);

    /*
     * ⚠ AND NO REFILL.  Those indices were never stored, so asking again
     * returns nothing while costing another stall and another hole in the
     * recording (§7.5).  Without the floor, every at-rest pull would burn all
     * three attempts for no new data.
     */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    HM_ASSERT_EQ(b->attempts, 1u);
    hm_history_block_release(b);
    fake_close(f);
}

/*
 * ⚠⚠ THE TWO 12.5 % REPLIES, AND WHY api-request C4 IS MARKED CRITICAL.
 *
 * capture-findings §4.1 put the question in as many words: "a 12.5 % coverage
 * that is CORRECT and a 12.5 % coverage that is a DELIVERY FAILURE are the same
 * number".  One is a still wrist at §7.3's floor, where the device stored one
 * index in eight and delivered every one of them.  The other reached an eighth
 * of the way and stopped.  Same window, same count, same `coverage_fraction` —
 * and a consumer's decision about whether a metric computed at impact exists
 * differs completely between them.
 *
 * ⚠ BOTH SHAPES ARE BUILT THROUGH THE SESSION, not handed to a helper.  The test
 * this replaces fed `hm_coverage_density()` a sample count chosen independently
 * of its coverage set — a shape gather_record() cannot produce, which is why it
 * passed over a field pinned to 1.0 (implementation-review I1).
 */
HM_TEST(density_separates_a_correct_twelve_percent_from_a_failed_one)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    /* Reply A: the buffer working correctly.  Every stored index arrived, and
     * the stored ones are one in eight because the wrist was still. */
    req = hist_request(f, first, last);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, first, last, 8u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->sample_count, 50u);
        HM_ASSERT_NEAR(b->coverage_fraction, 0.125, 1e-9);
        HM_ASSERT_NEAR(b->density, 0.125, 1e-9);
        hm_history_block_release(b);
        b = NULL;
    }
    fake_close(f);

    /* Reply B: the same count over the same window, reaching an eighth of the
     * way at the full internal rate and then stopping. */
    f = hist_open(400u, 5000u);
    req = hist_request(f, first, last);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, first, first + 49u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->sample_count, 50u);
        HM_ASSERT_NEAR(b->coverage_fraction, 0.125, 1e-9);
        /* ⚠ The number that separates them, and the only one that does. */
        HM_ASSERT_MSG(b->density > 0.99,
                      "a reply dense at step 1 must not read like the at-rest floor");
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_refill_chases_only_the_gap_that_motion_cannot_explain)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, first, last);
    req.refill_gaps = true;
    req.max_attempts = 3u;
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, first, first + 115u, 1u);
    feed_history_records(f, last - 115u, last, 1u);
    feed(f, k_mark_end, sizeof(k_mark_end));
    drain(f);

    /* ⚠ Still one: §6.1.1's rule applies to a refill exactly as it does to a
     * first attempt. */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    hist_live_frame(f, 17800u);

    /* ⚠ Safe to ask again in place: the device never stopped recording and `a1`
     * works mid-stream (AR C5).  The second ask is exactly the hole. */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 2u);
    {
        size_t at = last_a1(f);
        HM_ASSERT_EQ(a1_first(f, at), (uint16_t)(first + 116u));
        HM_ASSERT_EQ(a1_last(f, at), (uint16_t)(last - 116u));
    }

    feed_history(f, first + 116u, last - 116u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_COMPLETE);
        HM_ASSERT_EQ(b->attempts, 2u);
        HM_ASSERT_EQ(b->sample_count, (size_t)(last - first + 1u));
        /* ⚠ Merged by DEVICE INDEX — ascending, strictly monotonic and
         * deduplicated across the two attempts (AR C3). */
        for (size_t i = 1; i < b->sample_count; ++i) {
            HM_ASSERT(b->samples[i].sample_index > b->samples[i - 1u].sample_index);
        }
        /* ⚠ Two stalls, one envelope: over-claiming a recording gap is the safe
         * direction, and one half-open range cannot say otherwise. */
        HM_ASSERT(b->self_recording_gap.end_us > b->self_recording_gap.start_us);
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_leading_end_marker_and_d0_03_is_a_refusal_and_never_a_bracket)
{
    const uint8_t      device_error[] = { 0xd0, 0x03 };
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             live_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /*
     * ⚠ §7.2: an invalid range yields the LEADING end marker and then `d0 03`,
     * with NO start marker — so the presence of `a1 02` is the acceptance test,
     * not the absence of an error.  Treating this leading `a1 01` as our own
     * closing marker, or opening on the `a1` write, are two of the four ways to
     * get the bracket wrong, and both suppress live delivery for a pull that
     * never happened.
     */
    feed(f, k_mark_end, sizeof(k_mark_end));
    live_before = f->nlive;
    hist_live_frame(f, 17800u);
    HM_ASSERT_EQ(f->nlive, live_before + 1u); /* live never stopped */

    feed(f, device_error, sizeof(device_error));
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        /*
         * ⚠ `d0 03` means nothing specific — seven distinct causes all returned
         * it (§7.2) — so this is classified by ELIMINATION from our own state:
         * a stream is running, the range was built ascending and non-empty and
         * inside what the device has counted, and a restart would have
         * cancelled the reservation.  What is left is a range the buffer no
         * longer holds.
         */
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_EVICTED);
        HM_ASSERT_EQ(b->sample_count, 0u);
        HM_ASSERT_EQ(b->attempts, 1u);
        hm_history_block_release(b);
    }
    {
        const hm_event *ev = find_event(f, HM_EV_DEVICE_ERROR);
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.device_error.request_id, id);
        }
    }
    /* No bracket was ever opened, so nothing reported a blind span. */
    HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_BLIND_SPAN), 0u);
    HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_STARTED), 0u);
    fake_close(f);
}

HM_TEST(a_request_that_times_out_keeps_what_arrived_and_orphans_the_replay)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             live_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7399u);
    /* ⚠ A pull that half-delivers forever is worse than one that fails (B7). */
    req.deadline_us = f->now + (hm_time_us)400 * 1000;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 7000u, 7049u, 1u);
    run_to(f, req.deadline_us + 1000);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_TIMED_OUT);
        /* Partial content is still valid, and still carries its coverage. */
        HM_ASSERT_EQ(b->sample_count, 50u);
        HM_ASSERT(b->coverage_fraction > 0.0);
        HM_ASSERT(b->coverage_fraction < 1.0);
        hm_history_block_release(b);
    }

    /*
     * ⚠ THE DEVICE DOES NOT KNOW WE GAVE UP.  It is still replaying, and the
     * bracket is deliberately left OPEN so those records stay orphans — counted
     * and discarded.  Closing it here would let ~4,000 bulk arrivals into the
     * live ring, where they look exactly like a burst of real motion.
     */
    live_before = f->nlive;
    feed_history_records(f, 7050u, 7199u, 1u);
    HM_ASSERT_EQ(f->nlive, live_before);
    HM_ASSERT_EQ(hm_history_pending(f->s), 0u);

    feed(f, k_mark_end, sizeof(k_mark_end));
    hist_live_frame(f, 17800u);
    HM_ASSERT_EQ(f->nlive, live_before + 1u); /* and live comes back */
    fake_close(f);
}

/*
 * ⚠⚠ CLOSE FINISHES WORK, IT DOES NOT DISCARD IT — and the function used to
 * assert that in a comment and undo it eight lines later.
 *
 * implementation-review I5.  hm_session_close() abandons every outstanding
 * reservation "before the queues are sealed, so the events reporting them still
 * reach the consumer", then cleared the event ring and made poll_events()
 * return 0 for ever after.  So the documented shape — close, then drain once
 * more to record what you got — returned nothing at all.
 *
 * ⚠ The stop barrier is untouched by this, because it is about PRODUCTION.
 * Nothing is produced after close; the ring merely still holds what close
 * itself put there, exactly as hm_history_collect() already worked.
 */
HM_TEST(closing_still_hands_back_the_events_it_generated_while_closing)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    hm_event           ev[16];
    uint64_t           id = 0u;
    size_t             n;
    size_t             ready = 0;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7399u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 7000u, 7099u, 1u);
    drain(f); /* everything produced BEFORE the close is already taken */

    hm_session_close(f->s);

    n = hm_session_poll_events(f->s, ev, 16u);
    for (size_t i = 0; i < n; ++i) {
        if (ev[i].type == (uint16_t)HM_EV_HISTORY_READY) {
            ready++;
        }
    }
    HM_ASSERT_MSG(ready == 1u,
                  "the request close itself finished must report that it did");

    /* And the block is there too, carrying what had arrived. */
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_CANCELLED);
        HM_ASSERT_EQ(b->sample_count, 100u);
        hm_history_block_release(b);
    }

    /* ⚠ The barrier itself: no WRITE survives a close, whatever else does. */
    {
        hm_write_request w[4];
        HM_ASSERT_EQ(hm_session_poll_writes(f->s, w, 4u), 0u);
    }
    fake_close(f);
}

HM_TEST(cancelling_materialises_a_block_with_whatever_arrived)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7399u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 7000u, 7099u, 1u);

    HM_ASSERT_EQ(hm_history_cancel(f->s, id), HM_OK);
    HM_ASSERT_EQ(hm_history_pending(f->s), 0u);
    /* Cancelling twice is a programming error, not a second block. */
    HM_ASSERT(hm_history_cancel(f->s, id) < HM_OK);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        /* ⚠ A capture should record what it got even when the answer is
         * partial: the block carries its coverage either way. */
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_CANCELLED);
        HM_ASSERT_EQ(b->sample_count, 100u);
        HM_ASSERT_EQ(b->delivered_count, 1u);
        hm_history_block_release(b);
    }
    feed(f, k_mark_end, sizeof(k_mark_end));
    fake_close(f);
}

HM_TEST(a_link_drop_answers_every_request_at_once_and_says_the_calibration_went_with_it)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    uint64_t           queued = 0u;
    uint32_t           index = 5000u + 400u * HIST_LIVE_STEP;
    fake              *f = hist_open(400u, 5000u);

    /* Walk the routine as far as a passing presence check, so the samples this
     * block will carry were genuinely captured under a transform. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    feed_cal_result(f);
    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_OK);
    for (unsigned i = 0; i < HM_PRESENCE_MAX_SAMPLES; ++i) {
        index += HIST_LIVE_STEP;
        f->now += 4000;
        feed_frame_split(f, (uint16_t)(index & 0xffffu), ticks_for(index), 0.5);
    }
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);

    req = hist_request(f, 7000u, 7399u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &queued), HM_OK);
    drain(f);
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 7000u, 7099u, 1u);

    /* ⚠ IMMEDIATELY, not at the deadline: the index space went with the link,
     * so the request can never be fulfilled and a consumer's gather has a
     * bounded wait (§8.4.1). */
    hm_session_on_link_down(f->s, HM_LINK_DOWN_SUPERVISION_TIMEOUT, f->now + 1000);
    drain(f);
    HM_ASSERT_EQ(hm_history_pending(f->s), 0u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_LINK_LOST);
        HM_ASSERT_EQ(b->sample_count, 100u);
        /*
         * ⚠⚠ THE ONE PLACE HM_CAL_LOST IS EVER WRITTEN.  No sample carries it —
         * none can be captured between the loss and the notice of one — but
         * here it does work UNCALIBRATED cannot: these samples WERE taken under
         * a transform, and §8.3 measured 0.70° before dropping a link and
         * 18.80° at the same pose after reconnecting, strap untouched.  "There
         * never was one" would be a different and false claim.
         */
        HM_ASSERT_EQ(b->calibration.state_at_start, (uint8_t)HM_CAL_CALIBRATED);
        HM_ASSERT_EQ(b->calibration.state_at_end, (uint8_t)HM_CAL_LOST);
        HM_ASSERT_EQ(b->calibration.spans_transition, 1u);
        /* And the angle it was measured at survives, so the claim is about
         * something rather than about nothing. */
        HM_ASSERT(!isnan(b->calibration.presence_angle_deg));
        /* ⚠ No sample carries LOST, and that is a decision rather than an
         * omission (sample.h). */
        for (size_t i = 0; i < b->sample_count; ++i) {
            HM_ASSERT(b->samples[i].calibration != (uint8_t)HM_CAL_LOST);
        }
        hm_history_block_release(b);
    }
    /* The one that never got its turn is answered too, with nothing in it. */
    b = NULL;
    HM_ASSERT_EQ(hm_history_collect(f->s, queued, &b), HM_OK);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_LINK_LOST);
        HM_ASSERT_EQ(b->sample_count, 0u);
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(an_alignment_budget_refuses_before_any_radio_traffic)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7199u);
    /* One microsecond of budget: no fit meets it.  ⚠ It gates on PRECISION,
     * not on the total — gating on the total would refuse every pull after the
     * first second of any session, which disables the feature rather than
     * informing it (clock.h). */
    req.alignment_budget_us = 1u;
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 0u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_REFUSED_ALIGNMENT);
        /* ⚠ Refusing AND RECORDING the refusal is the point (AR B2): the block
         * carries the fit it was judged against, so a consumer can write "we
         * declined this pull, and this is what the clock was worth" into the
         * capture's provenance. */
        HM_ASSERT((b->fit.flags & (uint32_t)HM_CLOCK_HAS_FIT) != 0u);
        /*
         * ⚠ AND EVERY GAP LIES INSIDE THE REQUEST (implementation-review I7).
         *
         * This path returns before `r->recorded` is written, and the sentinel
         * that was supposed to catch that tested a WIDTH — but hm_index_range is
         * inclusive, so index_width({0,0}) is 1, not 0, and a zeroed range read
         * as "the single index 0".  The block then handed back a gap over [0,0]
         * and another over [1, window.last]: anchored below the request, in a
         * struct whose own header gives that invariant as the reason
         * `self_recording_gap` is a separate field.
         */
        HM_ASSERT(b->gap_count > 0u);
        for (size_t i = 0; i < b->gap_count; ++i) {
            HM_ASSERT_MSG(b->gaps[i].indices.first >= b->requested_indices.first,
                          "a gap below the request is a statement about a span "
                          "nobody asked about");
            HM_ASSERT(b->gaps[i].indices.last <= b->requested_indices.last);
        }
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_legacy_stream_records_the_refusal_rather_than_returning_a_status)
{
    hm_session_config  cfg = hm_session_config_default();
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    uint8_t            frame[1 + 42];
    const uint8_t      started[] = { 0x82, 0x01 };
    fake              *f;

    cfg.stream_config = hm_stream_config_legacy();
    f = fake_open(&cfg);
    bring_up(f);
    HM_ASSERT_EQ(hm_session_start_stream(f->s), HM_OK);
    drain(f);
    memset(frame, 0, sizeof(frame));
    frame[0] = 0x7f;
    put_be16(frame + 1, 16384u);
    put_be16(frame + 1 + 21, 16384u);
    feed(f, started, sizeof(started));
    feed(f, frame, sizeof(frame));

    /*
     * ⚠ §6.3.1: a 0x7f record has no header, so there is no counter for `a1` to
     * address and no fit for a window to map through.  The reservation
     * therefore succeeds and its block is immediately collectable carrying
     * HM_HIST_NOT_ALIGNABLE — refusing with a bare status code would leave the
     * capture with no record of why nothing was retrieved.
     */
    req = hm_history_request_around(NULL, f->now - (hm_time_us)2 * 1000 * 1000);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    HM_ASSERT(id != 0u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_NOT_ALIGNABLE);
        HM_ASSERT_EQ(b->sample_count, 0u);
        HM_ASSERT_EQ(b->attempts, 0u);
        /* ⚠ And no gaps: the window never reached an index range at all, so
         * "one gap at index 0" would describe a span nobody identified. */
        HM_ASSERT_EQ(b->gap_count, 0u);
        HM_ASSERT_EQ(b->delivered_count, 0u);
        hm_history_block_release(b);
    }
    HM_ASSERT_EQ(count_writes(f, 0xa1, 0u), 0u);
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Increment 7 — the wrap split, the depth bracket, the eviction estimate     */
/* ------------------------------------------------------------------------ */
HM_TEST(a_window_across_the_index_wrap_is_issued_as_two_asks_and_merged)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    size_t             at;
    /* Start high enough that 400 live frames at step 32 cross 65536. */
    fake *f = hist_open(400u, 60000u);

    HM_ASSERT(f->live[f->nlive - 1u].sample_index > 65536u);

    /*
     * ⚠ §8.3's SPLIT.  `a1` takes two u16be and §7.1 requires `first < last`,
     * so one command cannot address a window spanning the 82.0 s counter wrap.
     * It goes out as TWO — [first, ffff] and [0, last] — merged by UNWRAPPED
     * index.  Half a window returned as though it were the whole one is exactly
     * the silent failure this library exists to avoid, and a naive
     * implementation gets it wrong exactly once every 82 seconds.
     */
    req = hist_request(f, 65400u, 65700u);
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /* The near half, re-wrapped to u16 on the way out (§7.4). */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    at = last_a1(f);
    HM_ASSERT_EQ(a1_first(f, at), 65400u);
    HM_ASSERT_EQ(a1_last(f, at), 65535u);
    feed_history(f, 65400u, 65535u, 1u);
    drain(f);

    /*
     * ⚠ AND THE FAR HALF WAITS FOR A LIVE FRAME, because §6.1.1 does not make
     * an exception for it: one stall is 72 % of §10.2's ±32,768 wrap budget, so
     * two pulls inside one live-frame gap exceed it and the tick unwrapper picks
     * the wrong wrap in silence.  A split window is two pulls like any other.
     */
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    hist_live_frame(f, 72800u);
    drain(f);

    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 2u);
    at = last_a1(f);
    HM_ASSERT_EQ(a1_first(f, at), 0u);
    HM_ASSERT_EQ(a1_last(f, at), 164u);
    feed_history(f, 65536u, 65700u, 1u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        /* ⚠ ONE BLOCK, ONE REQUEST.  Two asks on the wire and the caller sees
         * neither of them. */
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_COMPLETE);
        HM_ASSERT_EQ(b->attempts, 2u);
        HM_ASSERT_EQ(b->sample_count, 301u);
        HM_ASSERT_EQ(b->samples[0].sample_index, 65400u);
        HM_ASSERT_EQ(b->samples[b->sample_count - 1u].sample_index, 65700u);
        /* ⚠ ASCENDING AND STRICTLY MONOTONIC ACROSS THE SEAM, with no
         * duplicate — the merge keys on the UNWRAPPED index, so the two halves
         * do not collide at 65535/0 the way the raw u16 would. */
        for (size_t i = 1; i < b->sample_count; ++i) {
            HM_ASSERT(b->samples[i].sample_index > b->samples[i - 1u].sample_index);
        }
        HM_ASSERT_NEAR(b->density, 1.0, 1e-9);
        HM_ASSERT_NEAR(b->coverage_fraction, 1.0, 1e-9);
        hm_history_block_release(b);
    }

    fake_close(f);

    /*
     * The control, and it needs its own session rather than another request on
     * that one: §6.1.1 refuses a window that predates the pull which re-anchored
     * the fit, so anything reserved after the split above is refused for a
     * reason that has nothing to do with the wrap.
     *
     * A window entirely on one side of the wrap is still ONE ask.
     */
    f = hist_open(400u, 60000u);
    b = NULL;
    writes_before = f->nwritten;
    req = hist_request(f, 65600u, 65799u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    feed_history(f, 65600u, 65799u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_COMPLETE);
        HM_ASSERT_EQ(b->attempts, 1u);
        HM_ASSERT_EQ(b->samples[0].sample_index, 65600u);
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_record_outside_the_requested_range_is_dropped_and_reported_once)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /*
     * ⚠ §10.1 measured 4,182 mid-stream records with every one inside the
     * requested range, so this is a finding rather than a routine case — and a
     * stray record placed on the wrong timeline is a swing aligned seconds off
     * the video.  Dropped and counted, reported ONCE per bracket: one event per
     * record would evict a 256-entry ring in 320 ms.
     */
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 6900u, 6949u, 1u); /* before the range */
    feed_history_records(f, 7000u, 7199u, 1u);
    feed(f, k_mark_end, sizeof(k_mark_end));

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_COMPLETE);
        HM_ASSERT_EQ(b->sample_count, 200u);
        HM_ASSERT_EQ(b->samples[0].sample_index, 7000u);
        hm_history_block_release(b);
    }
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_OUT_OF_RANGE), 1u);
    {
        const hm_event *ev = find_event(f, HM_EV_WARNING);
        for (size_t i = 0; i < f->nevents; ++i) {
            if (f->events[i].type == (uint16_t)HM_EV_WARNING &&
                f->events[i].u.warning.code == (uint16_t)HM_WARN_HISTORY_OUT_OF_RANGE) {
                ev = &f->events[i];
            }
        }
        HM_ASSERT(ev != NULL);
        if (ev != NULL) {
            HM_ASSERT_EQ(ev->u.warning.detail_i32, 50);
        }
    }
    fake_close(f);
}

HM_TEST(a_reply_that_stops_early_is_short_where_one_with_a_hole_in_it_is_holed)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, first, last);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /*
     * ⚠ Contiguous and narrower than the request — one delivered interval, no
     * interior hole.  §8.7: a short block is at least OBVIOUSLY short, where a
     * holed one spans the whole range and looks complete to a count.  Which of
     * the two it is decides whether re-requesting can help, and it is the shape
     * that carries evidence about how far back the buffer reaches.
     */
    feed_history(f, first, first + 199u, 1u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_SHORT);
        HM_ASSERT_EQ(b->sample_count, 200u);
        HM_ASSERT_EQ(b->delivered_count, 1u);
        HM_ASSERT_NEAR(b->coverage_fraction, 0.5, 1e-9);
        HM_ASSERT_NEAR(b->density, 1.0, 1e-9);
        hm_history_block_release(b);
    }
    /* The 200 indices that never arrived are what says something about depth. */
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_SHORT), 1u);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_HOLED), 0u);
    fake_close(f);
}

HM_TEST(history_that_disagrees_with_live_is_counted_rather_than_quietly_preferred)
{
    const uint32_t     first = 7000u;
    const uint32_t     last = 7399u;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, first, last);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /*
     * ⚠ THE CHECK THAT MUST BE ABLE TO FAIL.  Every other history test asserts
     * `live_overlap_mismatches == 0`, and a check that has silently stopped
     * running asserts exactly the same thing.  So: the same indices the live
     * stream already delivered, with DIFFERENT raw counts.
     *
     * §8.8: a consumer stitching [live prefix] + [retrieved span] + [live
     * suffix] into one lane depends on history being a strict superset of live —
     * same index, same values — and if it is not, the seam looks like a real
     * wrist movement.  The library is the only layer that ever holds both
     * halves, so it counts rather than picking a winner.
     */
    feed(f, k_mark_start, sizeof(k_mark_start));
    for (uint32_t i = first; i <= last; ++i) {
        f->now += 3800;
        feed_frame(f, (uint16_t)(i & 0xffffu), ticks_for(i), 500);
    }
    feed(f, k_mark_end, sizeof(k_mark_end));

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT(b->live_overlap_samples > 0u);
        HM_ASSERT_EQ(b->live_overlap_mismatches, b->live_overlap_samples);
        /* ⚠ And the disagreement changes nothing about the data: no sample is
         * dropped, corrected or preferred.  The counters are the report. */
        HM_ASSERT_EQ(b->sample_count, (size_t)(last - first + 1u));
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_window_reaching_back_before_the_stream_says_never_recorded)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             not_recorded = 0u;
    size_t             fit_blind = 0u;
    fake              *f = hist_open(400u, 5000u);

    /* The stream's first sample is index 5000; ask from 4800. */
    req = hist_request(f, 4800u, 5100u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /* ⚠ The ask was CLAMPED: the device is asked for what it can have taken,
     * and the rest is reported rather than requested. */
    {
        size_t at = last_a1(f);
        HM_ASSERT(at != SIZE_MAX);
        HM_ASSERT_EQ(a1_first(f, at), 5000u);
        HM_ASSERT_EQ(a1_last(f, at), 5100u);
    }
    feed_history(f, 5000u, 5100u, 1u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b == NULL) {
        fake_close(f);
        return;
    }
    HM_ASSERT_EQ(b->sample_count, 101u);
    for (size_t i = 0; i < b->gap_count; ++i) {
        if (b->gaps[i].kind == (uint8_t)HM_GAP_NOT_RECORDED) {
            not_recorded++;
            HM_ASSERT_EQ(b->gaps[i].indices.first, 4800u);
            HM_ASSERT_EQ(b->gaps[i].indices.last, 4999u);
        }
        if (b->gaps[i].kind == (uint8_t)HM_GAP_FIT_BLIND) {
            fit_blind++;
            /* ⚠ Undatable, and it says so with HM_TIME_UNKNOWN rather than an
             * extrapolated number that looks like a measurement. */
            HM_ASSERT_EQ(b->gaps[i].span.start_us, HM_TIME_UNKNOWN);
        }
    }
    /*
     * ⚠ The device recorded NOTHING there — a different fact from "recorded and
     * did not deliver", and the two must not be conflated (C7).  The same span
     * is also outside what the fit ever observed, so both kinds cover it and
     * the stronger one wins for a consumer reading them.
     */
    HM_ASSERT_EQ(not_recorded, 1u);
    HM_ASSERT_EQ(fit_blind, 1u);
    hm_history_block_release(b);
    fake_close(f);
}

/*
 * ⚠⚠ A RECORD DELIVERED BELOW THE CLAMPED ASK MUST NOT TEACH THE DEPTH BRACKET
 * ANYTHING, LET ALONE THE OPPOSITE OF WHAT IT SHOWS.
 *
 * implementation-review I8, and it is the same shape as the test above: the ask
 * is clamped to `[first_index, head]` but gather_record() filters arrivals
 * against the FULL mapped window, which is wider.  A record in between puts the
 * coverage set's oldest index below `recorded.first`, and
 * `oldest - r->recorded.first` is unsigned — so it wrapped to ≈4.29e9, cleared
 * §7.3's step-8 floor trivially, and wrote a `depth_hi` meaning "the buffer
 * demonstrably failed to reach that far" from evidence saying it reached
 * FURTHER than we asked.
 *
 * ⚠ `depth_hi` only ever narrows, so the wrong claim would be permanent for the
 * stream: hm_history_resident_range() silently shrinks, and
 * HM_WARN_HISTORY_DEPTH_CONFLICT can fire over it.  §10.1 measured 4,182 records
 * with every one inside the requested range, so this needs the device to do
 * something never observed — and the consequence outlives the pull that caused
 * it.
 */
HM_TEST(a_record_below_the_clamped_ask_does_not_invert_the_depth_bracket)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    hm_time_range      range;
    fake              *f = hist_open(400u, 5000u);

    /* The stream's first sample is index 5000; ask from 4800, so the ask is
     * clamped to 5000 while the gather still accepts from 4800. */
    req = hist_request(f, 4800u, 5100u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /* ⚠ And the device answers with one record BELOW what it was asked for. */
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 4900u, 4900u, 1u);
    feed_history_records(f, 5000u, 5100u, 1u);
    feed(f, k_mark_end, sizeof(k_mark_end));

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->sample_count, 102u);
        hm_history_block_release(b);
    }

    /* A live frame, so §6.1.1's re-anchored fit can date the buffer's head and
     * the query is answerable at all. */
    hist_live_frame(f, 5000u + 401u * HIST_LIVE_STEP);

    /*
     * ⚠ The bracket may report a lower bound — the device really did serve back
     * to 4900 — but it must never report an UPPER bound narrower than that from
     * the same reply.  Under the underflow it recorded one, permanently.
     */
    if (hm_history_resident_range(f->s, &range) == HM_OK) {
        hm_time_us reached =
            (hm_time_us)((double)(5000u + 401u * HIST_LIVE_STEP - 4900u) * 1251.0);
        HM_ASSERT_MSG(range.end_us - range.start_us > reached / 2u,
                      "a reply that reached FURTHER back than the ask must not "
                      "shrink the residency estimate");
    }
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_DEPTH_CONFLICT), 0u);
    fake_close(f);
}

HM_TEST(a_stop_during_a_pull_leaves_on_that_call_and_delivers_nothing_until_it_lands)
{
    const uint8_t      stopped[] = { 0x83, 0x01 };
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    size_t             live_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7399u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 7000u, 7099u, 1u);

    /*
     * ⚠ THE ORDERING OF §8.4.1: the retrieval is cancelled FIRST and the `83`
     * queued second, so the write quiet period cannot hold the consumer's own
     * teardown behind a pull it no longer wants.  A stop that takes four
     * seconds to leave the queue is a bug, and it is the case a consumer causes
     * deliberately and will therefore hit first.
     */
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0x83, writes_before), 1u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_CANCELLED);
        HM_ASSERT_EQ(b->sample_count, 100u);
        hm_history_block_release(b);
    }

    /*
     * ⚠ AND THE DEVICE DOES NOT KNOW.  It keeps replaying, and those records
     * are byte-identical to live ones with indices thousands of samples behind
     * — so nothing at all is delivered until the stream actually stops.
     * Letting them through would drag the live index unwrapper backwards and
     * every later request's range with it.
     */
    live_before = f->nlive;
    feed_history_records(f, 7100u, 7199u, 1u);
    HM_ASSERT_EQ(f->nlive, live_before);
    feed(f, k_mark_end, sizeof(k_mark_end));
    feed_history_records(f, 7200u, 7220u, 1u);
    HM_ASSERT_EQ(f->nlive, live_before);

    feed(f, stopped, sizeof(stopped));
    HM_ASSERT_EQ(count_events(f, HM_EV_STREAM_STOPPED), 1u);
    fake_close(f);
}

/*
 * ⚠⚠ AND A RESERVATION MADE AFTER THE STREAM ENDED IS ANSWERED AT ONCE TOO,
 * WITH THE RIGHT REASON.
 *
 * implementation-review I6.  The fit is reset at start_stream() and never at a
 * stop, so HM_CLOCK_HAS_FIT survives one and reserve() sailed past every check.
 * The request then sat QUEUED while history_service() returned silently at its
 * `stream != RUNNING` line on every pass — so history_issue(), the ONLY producer
 * of HM_HIST_NO_STREAM, was never reached, and the request waited out its whole
 * deadline to materialise HM_HIST_TIMED_OUT.
 *
 * ⚠ Two separate harms, and the second is the worse one.  §8.4.1 exists to stop
 * a consumer's gather stalling on a request that has gone quiet — and the status
 * written into the capture was simply wrong about why it failed.
 */
HM_TEST(reserving_after_the_stream_ended_says_so_now_rather_than_at_the_deadline)
{
    const uint8_t      stopped[] = { 0x83, 0x01 };
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7399u);
    /* Far enough out that waiting for it would be a visible stall. */
    req.deadline_us = f->now + (hm_time_us)60 * 1000 * 1000;

    HM_ASSERT_EQ(hm_session_stop_stream(f->s), HM_OK);
    drain(f);
    feed(f, stopped, sizeof(stopped));

    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /* ⚠ Immediately, and before any radio traffic (AR C1). */
    HM_ASSERT_EQ(hm_history_pending(f->s), 0u);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 0u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_MSG(b->status == (uint8_t)HM_HIST_NO_STREAM,
                      "there was no stream, and that is what the capture must say");
        HM_ASSERT_EQ(b->sample_count, 0u);
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_stream_the_device_stopped_answers_its_reservations_at_once)
{
    const uint8_t      stopped[] = { 0x83, 0x01 };
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 7000u, 7399u);
    /* A deadline far enough out that timing out would be a visible stall. */
    req.deadline_us = f->now + (hm_time_us)60 * 1000 * 1000;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed(f, k_mark_start, sizeof(k_mark_start));
    feed_history_records(f, 7000u, 7099u, 1u);

    /*
     * ⚠ The third way a stream can end — the device stopped it, not us and not
     * a link drop.  §8.4.1's argument applies unchanged: a request that goes
     * quiet for its whole deadline costs a consumer's gather a pipeline stall
     * for no information, so it is answered now, with what arrived.
     */
    feed(f, stopped, sizeof(stopped));
    HM_ASSERT_EQ(hm_history_pending(f->s), 0u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_NO_STREAM);
        HM_ASSERT_EQ(b->sample_count, 100u);
        hm_history_block_release(b);
    }
    fake_close(f);
}

HM_TEST(a_gather_waits_for_a_calibration_rather_than_interrupting_it)
{
    hm_history_request req;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 5000u);

    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    /*
     * ⚠ THE OTHER HALF OF THE R8 INTERLOCK.  Every hm_calibration_* call
     * already refuses with HM_ERR_BUSY inside a bracket; this is the same rule
     * from the other side.  §8.2's device observes a CONTINUOUS RAISE between
     * the markers, and a bracket opened here would suspend live delivery for
     * seconds — aborting the attempt at the raise limit for a reason that has
     * nothing to do with calibration, with a user standing there holding their
     * arm out.
     */
    writes_before = f->nwritten;
    req = hist_request(f, 7000u, 7199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 0u);

    /* Live frames keep flowing through the routine, and none of them lets the
     * pull in early. */
    hist_live_frame(f, 17800u);
    hist_live_frame(f, 17832u);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 0u);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    /* The routine ends; the next live frame picks the request up. */
    HM_ASSERT_EQ(hm_calibration_abort(f->s), HM_OK);
    hist_live_frame(f, 17864u);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    fake_close(f);
}

/*
 * ⚠⚠ AND THE SAMPLES THAT COME BACK FROM THAT WAIT ARE STAMPED WITH THE STATE
 * THEY WERE CAPTURED UNDER, NOT THE ONE THE WAIT PRODUCED.
 *
 * implementation-review I3, and the interlock above is what makes it ordinary
 * rather than exotic.  The sequence is entirely normal:
 *
 *   1. a window is recorded while the device is UNCALIBRATED;
 *   2. a consumer reserves it;
 *   3. a calibration runs and its presence check passes — the pull is HELD for
 *      the whole routine, by the rule the test above pins;
 *   4. the pull finally goes out, and every record in it predates the transform.
 *
 * stamp_with() read `s->cal_state` at that moment, so all of them came back
 * labelled CALIBRATED: orientations in the mounting frame, marked as being in
 * the anatomical one, permanently and invisibly (sample.h).  The block's own
 * `hm_calibration_span` had it right all along, so the block contradicted
 * itself — the summary said the span predates the transform and the samples
 * said they were calibrated.
 */
HM_TEST(a_pull_held_behind_a_calibration_is_not_relabelled_by_it)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    uint32_t           index = 5000u + 400u * HIST_LIVE_STEP;
    fake              *f = hist_open(400u, 5000u);

    /* 1-2.  The window is behind us and uncalibrated; reserve it. */
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_UNCALIBRATED);
    req = hist_request(f, 7000u, 7199u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);

    /* 3.  A whole routine, through to a passing presence measurement — the only
     * route to HM_CAL_CALIBRATED (§8.2).  The request waits it out. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_confirm_raise(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    feed_cal_result(f);
    HM_ASSERT_EQ(hm_calibration_confirm_reference_pose(f->s), HM_OK);
    for (unsigned i = 0; i < HM_PRESENCE_MAX_SAMPLES; ++i) {
        index += HIST_LIVE_STEP;
        f->now += 4000;
        feed_frame_split(f, (uint16_t)(index & 0xffffu), ticks_for(index), 0.5);
    }
    HM_ASSERT_EQ(hm_session_calibration_state(f->s), HM_CAL_CALIBRATED);

    /* 4.  Now it goes out, and comes back. */
    drain(f);
    feed_history(f, 7000u, 7199u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b == NULL) {
        fake_close(f);
        return;
    }
    HM_ASSERT_EQ(b->sample_count, 200u);

    /*
     * ⚠ NOT CALIBRATED, which is the whole of the harm.  These orientations are
     * in the mounting frame — §8.1 puts that at 11-15° from a straight wrist —
     * and a recording that says otherwise is wrong permanently and invisibly.
     *
     * ⚠ It reads UNKNOWN rather than UNCALIBRATED, and that is the documented
     * one-transition limit rather than an accident: a successful routine moves
     * the state TWICE (`0x94` → UNKNOWN, then the presence measurement →
     * CALIBRATED), and only one step back is retained.  UNKNOWN is "we cannot
     * say", which is the safe direction; sample.h's rule is that it is never a
     * hopeful CALIBRATED.
     */
    HM_ASSERT(b->calibration.state_at_start != (uint8_t)HM_CAL_CALIBRATED);

    /* ⚠ AND THE BLOCK NO LONGER CONTRADICTS ITSELF.  Its summary and its samples
     * are resolved from the same transition, at the same instants, so they
     * cannot disagree — which is what implementation-review I3 caught them
     * doing. */
    for (size_t i = 0; i < b->sample_count; ++i) {
        HM_ASSERT_MSG(b->samples[i].calibration != (uint8_t)HM_CAL_CALIBRATED,
                      "a history sample carries the state at CAPTURE, never the "
                      "state at materialisation");
        HM_ASSERT_EQ(b->samples[i].calibration, b->calibration.state_at_start);
        /* No sample ever carries LOST, whatever else changes (sample.h). */
        HM_ASSERT(b->samples[i].calibration != (uint8_t)HM_CAL_LOST);
    }
    hm_history_block_release(b);

    /* ⚠ And the live path is unaffected: a frame arriving NOW was captured now,
     * and the transform really is in force for it. */
    index += HIST_LIVE_STEP;
    f->now += 4000;
    feed_frame(f, (uint16_t)(index & 0xffffu), ticks_for(index), 0);
    HM_ASSERT_EQ(f->live[f->nlive - 1u].calibration, (uint8_t)HM_CAL_CALIBRATED);
    fake_close(f);
}

/*
 * ⚠ AND THE INTERLOCK REACHES A REQUEST WHOSE BRACKET IS ALREADY CLOSED, which
 * is the half that is easy to miss.  Between two attempts — a refill, or the far
 * side of §8.3's wrap split — the request is still "in flight" but `bracket_open`
 * is false, so `cal_guard()` lets `hm_calibration_begin()` through.  If the next
 * attempt then went out, it would open a bracket in the MIDDLE of the routine:
 * live delivery suspended (§10.1) while §8.2's device is watching for a
 * continuous raise, and the attempt aborted at the raise limit for a reason that
 * has nothing to do with calibration, with a user standing there holding their
 * arm out.
 */
HM_TEST(a_second_attempt_waits_for_a_calibration_the_same_way_a_first_one_does)
{
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    size_t             writes_before;
    fake              *f = hist_open(400u, 60000u);

    /* A window across the wrap: two asks, and the second is the one at risk. */
    req = hist_request(f, 65400u, 65700u);
    writes_before = f->nwritten;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    feed_history(f, 65400u, 65535u, 1u);
    drain(f);

    /* The bracket has closed, so a routine is allowed to start. */
    HM_ASSERT_EQ(hm_calibration_begin(f->s), HM_OK);
    HM_ASSERT_EQ(hm_calibration_confirm_horizontal(f->s), HM_OK);
    drain(f);
    feed(f, k_cal_ack, sizeof(k_cal_ack));
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    /* Live frames keep flowing, and none of them lets the far half out. */
    hist_live_frame(f, 72800u);
    hist_live_frame(f, 72832u);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 1u);
    HM_ASSERT_EQ(hm_calibration_current_phase(f->s), HM_CALP_OBSERVING_RAISE);

    /* The routine ends; the next live frame picks the far half up. */
    HM_ASSERT_EQ(hm_calibration_abort(f->s), HM_OK);
    hist_live_frame(f, 72864u);
    HM_ASSERT_EQ(count_writes(f, 0xa1, writes_before), 2u);
    feed_history(f, 65536u, 65700u, 1u);

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_COMPLETE);
        HM_ASSERT_EQ(b->sample_count, 301u);
        hm_history_block_release(b);
    }
    fake_close(f);
}

/* ------------------------------------------------------------------------ */
/* Increment 7 — §8.5's depth bracket, learned from the device               */
/* ------------------------------------------------------------------------ */
/* One whole pull of [first,last] delivered at `step`, and the block released:
 * these tests are about what the SESSION learned, not about the block. */
static void hist_pull(fake *f, uint32_t first, uint32_t last, uint32_t step)
{
    hm_history_request req = hist_request(f, first, last);
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;

    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, first, last, step);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        hm_history_block_release(b);
    }
}

/*
 * ⚠⚠ THE TEST THAT PINS THE CHOICE RATHER THAN THE OUTCOME, AND IT IS THE ONE
 * INCREMENT 7 EXISTS FOR.
 *
 * §8.5 as drafted said the lower bound comes from a span that came back
 * COMPLETE.  §7.3 was then rewritten from hardware, and it makes every reply
 * HOLED — the buffer is motion-adaptive, so a still wrist returns an even
 * one-in-eight.  Replaying `swings.hmwire` through this gather produced six
 * blocks and ZERO HM_HIST_COMPLETE.  A rule keyed on the status would therefore
 * learn NOTHING on a real device and leave both queries reporting
 * HM_HISTORY_DEPTH_SEED_US — a figure measured once, on somebody else's session
 * — dressed as a measurement of this connection.
 *
 * So the evidence is the OLD END OF THE DELIVERED SET, judged by §7.3's step-8
 * floor.  This reply is holed from end to end at that floor, is the shape a
 * still wrist actually produces, and it MUST move the bracket.
 */
HM_TEST(a_reply_holed_at_the_hundred_hertz_floor_still_measures_the_depth)
{
    hm_time_range range;
    hm_time_range before;
    fake         *f = hist_open(400u, 5000u);

    /* Nothing measured yet: the seed, and a status that says so. */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &before), HM_PENDING);

    /* ⚠ Step 8 end to end — HM_HIST_HOLED, never HM_HIST_COMPLETE, and the
     * §7.3 shape of a pull over a still wrist. */
    hist_pull(f, 17000u, 17400u, 8u);

    /*
     * ⚠ AND FOR A MOMENT AFTER THE PULL THERE IS NO ANSWER AT ALL, WHICH IS
     * §6.1.1 REACHING THIS QUERY TOO.  The fit re-anchored when the bracket
     * closed and has not seen a live frame since, so there is no mapping from
     * the buffer's head to a host time — and a residency range is a claim in
     * host time.  Refusing for those few tens of milliseconds is right;
     * extrapolating through a stall of unknown width is what §6.1.1 forbids.
     */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_ERR_NO_FIT);
    hist_live_frame(f, 17800u);

    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_OK);
    /*
     * The device served back to index 17000 from a head of 17768, so the
     * verified reach-back is 768 indices ≈ 0.96 s at ≈799.2 Hz.  ⚠ Measured in
     * INDICES and converted with the fit's slope: the counter advances at the
     * internal rate whatever the wrist is doing (§6.5), where a bulk reply's
     * arrival times say only how fast the radio drained.
     */
    HM_ASSERT_NEAR((double)(range.end_us - range.start_us), 768.0 * 1e6 / 799.2, 40000.0);

    /* ⚠ And it under-claims on purpose: what was verified, not the seed. */
    HM_ASSERT(range.end_us - range.start_us < HM_HISTORY_DEPTH_SEED_US);

    /* The bool answers from the measurement now, both ways. */
    HM_ASSERT(hm_history_coverage_available(f->s, range.end_us - 500000, range.end_us));
    HM_ASSERT(!hm_history_coverage_available(f->s, range.end_us - 5000000, range.end_us));
    fake_close(f);
}

HM_TEST(a_reply_that_never_reached_its_old_end_caps_the_bracket_from_above)
{
    hm_time_range range;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    /*
     * Asked back to 17000, served only from 17100 — a leading gap of 100
     * indices, twelve times §7.3's floor of 8 and therefore not explicable by
     * motion.  The buffer did not reach that far.
     */
    req = hist_request(f, 17000u, 17400u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, 17100u, 17400u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_SHORT);
        hm_history_block_release(b);
    }
    hist_live_frame(f, 17800u); /* §6.1.1: the re-anchored fit needs one */

    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_OK);
    /* The lower bound is what ARRIVED — 17768 back to 17100, 668 indices — and
     * the upper bound sits above it at 768.  The claim is the lower one. */
    HM_ASSERT_NEAR((double)(range.end_us - range.start_us), 668.0 * 1e6 / 799.2, 40000.0);
    HM_ASSERT(!hm_history_coverage_available(f->s, range.start_us - 1, range.end_us));
    fake_close(f);
}

/*
 * ⚠ NO EVIDENCE IS NOT A BOUND.  A pull that delivered nothing because WE
 * stopped listening says nothing about the buffer, and reading it as eviction
 * would shrink the bracket every time a consumer cancelled or a deadline
 * expired — a measurement that decays with use is worse than none.
 */
HM_TEST(a_pull_that_delivered_nothing_teaches_the_bracket_nothing)
{
    hm_time_range      range;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    req = hist_request(f, 17000u, 17400u);
    req.deadline_us = f->now + 1000000;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed(f, k_mark_start, sizeof(k_mark_start)); /* accepted, and then silence */

    f->now += 2000000;
    hm_session_tick(f->s, f->now);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_TIMED_OUT);
        HM_ASSERT_EQ(b->sample_count, 0u);
        hm_history_block_release(b);
    }

    /* Still the seed, and still saying so. */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_PENDING);
    HM_ASSERT(!hm_history_coverage_available(f->s, range.start_us, range.end_us));
    fake_close(f);
}

/*
 * ⚠ THE TWO HALVES OF THE BRACKET CAN CROSS, AND THAT IS A FINDING ABOUT THE
 * DEVICE RATHER THAN A FAULT IN THE SESSION.
 *
 * If the buffer held a fixed DURATION they never could.  They can, because §7.3
 * leaves open whether it holds a fixed duration or a fixed sample COUNT — and
 * under a fixed count the depth in TIME shrinks by up to 8× when the wrist is
 * moving, which is exactly when a consumer is pulling.  So the library says so
 * on the warning channel and falls back to the narrower claim, rather than
 * quietly keeping the wider one and being wrong in the direction that loses
 * data.  ⚠ Fixed duration or fixed sample count (§7.3) is the open question
 * this is the answer channel for.
 */
HM_TEST(a_depth_bracket_that_contradicts_itself_says_so_and_takes_the_narrow_claim)
{
    hm_time_range      range;
    hm_history_request req;
    hm_history_block  *b = NULL;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    /* A wide pull, served to its old end: the buffer reached back 1,768
     * indices from a head of 17,768 — about 2.2 s. */
    hist_pull(f, 16000u, 17400u, 1u);

    /* Live resumes and the head moves on.  §6.1.1: the fit re-anchored at the
     * bracket close, so the next window has to live in the NEW stretch. */
    for (uint32_t i = 0; i < 40u; ++i) {
        hist_live_frame(f, 17800u + i * 32u);
    }

    /* A narrower pull that did NOT reach its old end — missing 200 indices,
     * twenty-five times §7.3's floor.  The buffer failed at ~1.4 s, which is
     * LESS than the 2.2 s it served a moment ago. */
    req = hist_request(f, 17900u, 18400u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, 18100u, 18400u, 1u);
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        hm_history_block_release(b);
    }
    drain(f);

    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_DEPTH_CONFLICT), 1u);

    hist_live_frame(f, 19100u);
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &range), HM_OK);
    /* ⚠ The NARROW claim wins.  Keeping the wider one would be claiming
     * residency the device has already refused once. */
    HM_ASSERT(range.end_us - range.start_us < 2000000);

    /* ⚠ And once per stream.  A warning that repeats every pull is one that
     * stops being read, and the event ring is drop-oldest. */
    for (uint32_t i = 0; i < 40u; ++i) {
        hist_live_frame(f, 19200u + i * 32u);
    }
    req = hist_request(f, 19300u, 19800u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed_history(f, 19600u, 19800u, 1u);
    b = NULL;
    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    if (b != NULL) {
        hm_history_block_release(b);
    }
    drain(f);
    HM_ASSERT_EQ(count_warnings(f, HM_WARN_HISTORY_DEPTH_CONFLICT), 1u);
    fake_close(f);
}

/*
 * ⚠ `d0 03` WITH NOTHING DELIVERED IS AN UPPER BOUND, AND THE STATUS MUST STILL
 * SAY "NOT VERIFIED".
 *
 * This is the one shape where the bracket has a ceiling and no floor, and the
 * two halves of the answer pull in opposite directions: the WIDTH narrows,
 * because the device has demonstrably refused that span, while the STATUS stays
 * HM_PENDING, because nothing has been served and no residency has been
 * verified.  Reporting HM_OK here would be the seed's failure mode inverted —
 * an upper bound presented as though it were a measurement of what is there.
 *
 * §7.2's code means seven different things, and this session's own state rules
 * out six (§11 of implementation-notes).  If that elimination is ever wrong the
 * error is in the safe direction: the library claims LESS residency than it has.
 */
HM_TEST(a_refused_range_lowers_the_ceiling_without_claiming_a_measurement)
{
    const uint8_t      device_error[] = { 0xd0, 0x03 };
    hm_history_request req;
    hm_history_block  *b = NULL;
    hm_time_range      before;
    hm_time_range      after;
    uint64_t           id = 0u;
    fake              *f = hist_open(400u, 5000u);

    /* Nothing measured: the seed, and a status that says so. */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &before), HM_PENDING);
    HM_ASSERT(before.end_us - before.start_us > 5000000);

    req = hist_request(f, 17000u, 17400u);
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &id), HM_OK);
    drain(f);
    feed(f, k_mark_end, sizeof(k_mark_end)); /* §7.2's LEADING end marker */
    feed(f, device_error, sizeof(device_error));

    HM_ASSERT_EQ(hm_history_collect(f->s, id, &b), HM_OK);
    HM_ASSERT(b != NULL);
    if (b != NULL) {
        HM_ASSERT_EQ(b->status, (uint8_t)HM_HIST_EVICTED);
        HM_ASSERT_EQ(b->sample_count, 0u);
        hm_history_block_release(b);
    }

    /*
     * ⚠ No bracket ever opened, so the fit did NOT re-anchor — §6.1.1's usual
     * "wait for a live frame" does not apply to a pull the device refused.
     */
    HM_ASSERT_EQ(hm_history_resident_range(f->s, &after), HM_PENDING);

    /* The ceiling came down: the device refused 768 indices back from a head of
     * 17,768, about 0.96 s, so the estimate can no longer be the 7.5 s seed. */
    HM_ASSERT(after.end_us - after.start_us < before.end_us - before.start_us);
    HM_ASSERT_NEAR((double)(after.end_us - after.start_us), 768.0 * 1e6 / 799.2, 40000.0);

    /* ⚠ And still false, because still nothing has been SERVED.  An upper bound
     * is not a reason to skip a check; it is a reason to expect less. */
    HM_ASSERT(!hm_history_coverage_available(f->s, after.start_us, after.end_us));
    fake_close(f);
}

/*
 * §8.6 — SERIALISATION AND EVICTION RISK.  ⚠ A second shot a few seconds after
 * the first is an ordinary thing, and a pull takes about as long as its window
 * spans (§7.4), so part of the second window can be gone by the time its turn
 * comes.  The data was there; nobody asked in time.  Returning a holed set for
 * it with nothing anywhere saying why is the failure this warns about.
 */
HM_TEST(a_queued_request_that_may_not_survive_its_wait_says_so_before_it_runs)
{
    hm_history_request req;
    uint64_t           first_id = 0u;
    uint64_t           second_id = 0u;
    const hm_event    *ev;
    fake              *f = hist_open(400u, 5000u);
    hm_time_us         t0 = f->now;

    /* Ahead in the queue: three seconds wide, and its window has not closed —
     * so it cannot even start yet, let alone finish. */
    memset(&req, 0, sizeof(req));
    req.window.start_us = t0 - 1000000;
    req.window.end_us = t0 + 2000000;
    req.deadline_us = t0 + 60000000;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &first_id), HM_OK);

    /* Behind it: a window whose oldest sample is already five seconds old.  Its
     * turn is estimated at t0 + 5 s — the leader's window close at t0 + 2 s plus
     * the 3 s that leader's own pull will cost (§7.4) — by which time §7.3's
     * depth puts its first sample at t0 + 2.5 s.  Two and a half seconds short. */
    memset(&req, 0, sizeof(req));
    req.window.start_us = t0 - 5000000;
    req.window.end_us = t0 + 1000000;
    req.deadline_us = t0 + 60000000;
    HM_ASSERT_EQ(hm_history_reserve(f->s, &req, &second_id), HM_OK);
    drain(f);

    HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_EVICTION_RISK), 1u);
    ev = find_event(f, HM_EV_HISTORY_EVICTION_RISK);
    HM_ASSERT(ev != NULL);
    if (ev != NULL) {
        HM_ASSERT_EQ(ev->u.history_eviction_risk.request_id, second_id);
        /* ⚠ BOTH FIGURES TRAVEL TOGETHER.  A margin with no wait beside it
         * cannot be acted on, which is §8.6's whole shape. */
        HM_ASSERT_NEAR((double)ev->u.history_eviction_risk.estimated_eviction_in_us, 2500000.0,
                       1000.0);
        HM_ASSERT(ev->u.history_eviction_risk.queued_for_us >= 0);
    }

    /* ⚠ Once per request.  The risk does not become newer by being restated,
     * and the event ring is drop-oldest — a repeat would evict the evidence of
     * everything else that happened while the queue drained. */
    hist_live_frame(f, 17800u);
    hist_live_frame(f, 17832u);
    HM_ASSERT_EQ(count_events(f, HM_EV_HISTORY_EVICTION_RISK), 1u);

    /* ⚠ AND IT CANCELS NOTHING.  The estimate rests on a depth measured only once, so
     * acting on it would be acting on an order of magnitude — the request stays
     * exactly where it was, in the queue, in reservation order. */
    HM_ASSERT_EQ(hm_history_pending(f->s), 2u);

    /*
     * ⚠ AND THE ROW IT ADDED OBEYS THE TABLE'S ONE RULE (implementation-notes
     * §3): whatever next_due_us() returns, one tick at exactly that time clears
     * it.  The eviction estimate is solved rather than polled, so it must never
     * re-arm at the instant it fired — that spins the HOST's loop at 100 % CPU,
     * in their code, looking like their bug.  Stepping to exactly the deadline
     * and never past it is what catches it.
     */
    {
        hm_time_us previous = f->now;
        int        wakes = 0;
        for (;;) {
            hm_time_us due = hm_session_next_due_us(f->s);
            if (due == HM_TIME_NEVER || due > t0 + (hm_time_us)20 * 1000 * 1000) {
                break;
            }
            HM_ASSERT(due > previous);
            previous = due;
            tick_at(f, due);
            wakes++;
            HM_ASSERT(wakes < 500); /* it must terminate, not merely finish */
        }
    }
    fake_close(f);
}

HM_TEST_MAIN()
