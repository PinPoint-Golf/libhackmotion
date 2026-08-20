/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_session.c — the sans-I/O session core.
 *
 * The host owns the radio, the thread, the timer and the clock.  This file owns
 * the protocol state: four small state machines (link, stream, calibration,
 * history), a table of deadlines the host arms one timer against, and four
 * output rings the host drains.  Nothing here reads a clock or performs I/O —
 * tests/purity.cmake fails the build if it ever starts to.
 *
 * docs/design.md §5 and §6 say WHAT this must do and why.
 * docs/implementation-notes.md says HOW the file is put together; where the two
 * disagree, design.md wins.
 *
 * ── What is in this file today ──────────────────────────────────────────────
 * Phase 2, the session core: lifecycle and the memory plan, the rings, the
 * deadline table, the link machine with §9.1 bring-up and the §9.2 keepalive,
 * the stream machine, the decode dispatch, the live path with the clock fit,
 * and the wire log.
 *
 * Phase 3, calibration: the phase machine of design §7, both markers, `0x94`,
 * the raise limit and the device bound, the presence run over live samples and
 * the reference-pose anchor it keeps, and the WR_ERR_BUSY interlocks against a
 * retrieval.
 *
 * Phase 4, the history gather (increments 5 and 6): reserve/collect/cancel, the
 * `a1`, the bracket, the coverage accounting, the refill, the block, and every
 * shape a request can fail in.  ⚠ Two things the reviewed design did not know,
 * both learned from hardware, are load-bearing here and are marked §6.1.1 (the
 * mapping is PIECEWISE — the sample counter stalls for the duration of every
 * pull, so the fit re-anchors after each one) and §7.3 (the buffer is
 * MOTION-ADAPTIVE — every reply is holed and the holes are not an error).
 *
 * Increment 7 adds what is measured rather than assumed: the buffer-depth
 * bracket learned from what the device actually returned, the two availability
 * queries built on it, §8.6's eviction estimate, and the two-request split for
 * a window spanning the 82.0 s index wrap.  ⚠ The depth bracket's evidence is
 * the OLD END OF THE DELIVERED SET, never the status enum — §7.3 makes every
 * real reply HOLED, so a rule keyed on WR_HIST_COMPLETE would learn nothing on
 * hardware and report the seed forever.  history_learn_depth() carries the
 * measurement that says so.
 *
 * ── The five riskiest things, and where they are answered ───────────────────
 *  1. The bracket bool — one discriminator line in on_frame(), two writers,
 *     one wr_fit_observe() call site.                              §5 of notes
 *  2. The per-sample stamp — one stamp_with(), so `calibration`, `stream_id`
 *     and `config_bits` cannot disagree with each other.  ⚠ And the CALLER
 *     states the instant of capture, because one write site says nothing about
 *     when the read happens: a history sample stamped with `cal_state` at
 *     materialisation is a pre-transform orientation labelled CALIBRATED.
 *  3. Deadline discipline — arm()/disarm() are the only writers, arm() clamps
 *     into the future during a tick, and the dispatcher disarms before it
 *     fires so a forgotten re-arm is a DEAD timer (which a counting test
 *     catches) rather than a LIVE one (which nothing catches).
 *  4. The write quiet period — nothing is queued during a bracket, and
 *     poll_writes() returns nothing anyway.
 *  5. Anything that could be lost silently — every drop is counted and every
 *     count is reachable.
 */
#include "wrist/session.h"
#include "wrist/history.h"

#include "wr_codec.h"
#include "wr_command.h"
#include "wr_fit.h"
#include "wr_overlap.h"
#include "wr_presence.h"
#include "wr_unwrap.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------ */
/* Constants that are decisions rather than transcriptions                   */
/* ------------------------------------------------------------------------ */

/* §9.1 step 2 is the only required one; the rest are informational and any of
 * them may go unanswered without failing bring-up. */
#define BRINGUP_REQUIRED_INFO ((uint32_t)WR_INFO_VERSIONS)
#define BRINGUP_ALL_INFO                                                                     \
    ((uint32_t)(WR_INFO_VERSIONS | WR_INFO_SENSOR_MAP | WR_INFO_BATTERY | WR_INFO_MAC |      \
                WR_INFO_SERIAL))

/*
 * ⚠ A hard limit on how long a history bracket may hold the write queue,
 * independent of any request's own deadline (implementation-notes §6.4).
 *
 * 2× the §7.3 buffer-depth seed.  A retrieval cannot outlast the buffer it is
 * replaying, so a bracket still open at twice that is not a slow pull — it is a
 * marker we will never see the end of, and it is holding the keepalive.  The
 * device dies at 5.0 minutes of no host→device write (§9.2) and that failure
 * reads as a radio fault, so this limit exists to make sure it cannot happen
 * for this reason.
 */
#define BRACKET_LIMIT_US (2 * WR_HISTORY_DEPTH_SEED_US)

/*
 * ⚠ Reservations outstanding at once.  §8.6: a second shot arrives before the
 * first pull has finished — "a golfer hitting balls does not wait" — so a queue
 * has to exist.  It is deliberately SHORT, because requests are serialised and
 * a pull takes about as long as its window spans (§7.4): four 4.5 s windows is
 * already 18 s of queue against a ~7.5 s buffer, so anything deeper would only
 * be reserving windows that are certain to have been evicted by their turn.
 */
#define HISTORY_MAX_PENDING 4

/*
 * Attempts per request, counting the first.  `max_attempts` is the caller's
 * (default 3, §8.4) and is clamped to this: each attempt is another stall and
 * therefore another hole in the recording (§7.5), and the runs it produces are
 * merged k-way at materialisation.
 */
#define HISTORY_MAX_ATTEMPTS 8u

/*
 * ⚠ §7.3's HARD FLOOR, and it is what a refill is for.
 *
 * Across 25 retrievals and 17,739 measured steps, no delivered index step
 * exceeded 8 and none was 0: step 8 (≈100 Hz) is what a still wrist returns and
 * step 1 (≈799 Hz) is what a swing returns.  So a gap of up to 7 missing
 * indices IS the buffer working correctly — those samples were never stored and
 * re-requesting them returns nothing while costing another stall.  A gap wider
 * than that is not explicable by motion and is worth asking for again.
 */
#define HISTORY_ADAPTIVE_STEP_MAX 8u

/* One progress event per this many records.  A full-rate 4.5 s pull is ~3,600
 * records against a 256-entry event ring, so per-record would evict the ring
 * and per-pull would say nothing while the pull was happening. */
#define HISTORY_PROGRESS_EVERY 512u

/* Catches a double release or a foreign pointer handed to
 * wr_history_block_release() — the one call that leaves the session thread. */
#define HISTORY_BLOCK_MAGIC 0x484d4231u /* "HMB1" */

/*
 * §9.6 classification thresholds.  Both are heuristics over evidence the
 * library has and nobody above it does; both are named so a reader can see what
 * is measured (the 300 s shutdown) and what is judgement (the margin).
 */
#define SLEEP_SUSPECT_IDLE_US ((wr_time_us)270 * 1000 * 1000) /* 4.5 min of 5.0 */
#define SLEEP_SUSPECT_QUIET_US ((wr_time_us)10 * 1000 * 1000)
#define ADVERTISING_FRESH_US ((wr_time_us)5 * 1000 * 1000)

/* Exponential backoff for WR_RECOVER_RECONNECT_WITH_BACKOFF.  The host may use
 * it or ignore it — it owns the adapter (design §5.4). */
#define RETRY_BACKOFF_FIRST_US ((wr_time_us)500 * 1000)
#define RETRY_BACKOFF_MAX_US ((wr_time_us)30 * 1000 * 1000)
/* Discovery is a race and a device that is advertising right now will stop
 * within seconds (§2.1), so this one is short on purpose. */
#define RETRY_ADVERTISING_US ((wr_time_us)250 * 1000)

/* §10.2's wrap decision has a ±32,768 tick budget and the worst case measured
 * over a 238 s session used 8.4 % of it.  A fifth of budget is therefore an
 * order of magnitude worse than anything observed, and still not ambiguous. */
#define TICK_MARGIN_ALARM (WR_TICK_MODULUS / 10u) /* a fifth of ±32768 */

/* ⚠ Per-record warnings are throttled to one event per code per second.  At the
 * full 799.2 Hz internal rate a per-record warning evicts a 256-entry event
 * ring in 320 ms, so the burst would destroy the evidence of itself. */
#define WARN_THROTTLE_US ((wr_time_us)1000 * 1000)

/*
 * ⚠ THE PRESENCE RUN IS BOUNDED BOTH WAYS, and the two bounds answer different
 * failures.
 *
 * The count: WR_PRESENCE_MAX_SAMPLES (64) is what wr_presence_select_reference()
 * will average over, and a burst fills it in 80 ms at the ≈799.2 Hz internal
 * rate (§6.6).
 *
 * The window: a held pose is a resting wrist, and §6.6 measures the live rate at
 * rest around 25 Hz — so 64 records would take 2.6 s to arrive and the count
 * alone would make a UI wait on the slowest case.  Two seconds yields ~50
 * records there and the whole buffer in anything faster.
 *
 * The floor: below this the run is reported as NOT MEASURED rather than
 * averaged.  ⚠ It is not about the classification, which survives a single
 * sample — §8.2's populations are ≤3.80° and ≥14.36° against 6°/10° thresholds,
 * and the worst-case Q14 noise at 3.80° is ±0.42°.  It is about
 * `pose_spread_deg`: a run of one reports a spread of exactly 0.0, which reads
 * as a perfectly held pose and is the strongest possible claim from the weakest
 * possible evidence.
 */
#define PRESENCE_RUN_WINDOW_US ((wr_time_us)2 * 1000 * 1000)
#define PRESENCE_RUN_MIN 8u

/* A buggy handler must still yield a finite sleep (implementation-notes §3). */
#define TICK_MAX_FIRINGS 64

/* Bounded, and bounded well above what the session can have outstanding: six
 * bring-up commands, one keepalive, one stream control, one calibration marker,
 * one `a1`.  Overflow is preceded by minutes of WR_WARN_KEEPALIVE_LATE. */
#define WRITE_QUEUE_CAPACITY 16u

/* ------------------------------------------------------------------------ */
/* One ring, used four times                                                 */
/* ------------------------------------------------------------------------ */
/*
 * Drop-oldest, because the newest sample is the one a consumer still has a use
 * for, and modular index arithmetic is worth writing exactly once.  `dropped`
 * is never reset: a consumer that sees it move knows data was lost, which is
 * the whole point of counting.
 */
typedef struct wr_ring {
    uint8_t *base;
    size_t   elem;
    size_t   capacity;
    size_t   head; /* oldest */
    size_t   count;
    uint64_t dropped;
} wr_ring;

static void ring_init(wr_ring *r, void *base, size_t capacity, size_t elem)
{
    r->base = (uint8_t *)base;
    r->elem = elem;
    r->capacity = (base != NULL) ? capacity : 0u;
    r->head = 0u;
    r->count = 0u;
    r->dropped = 0u;
}

static void *ring_slot(const wr_ring *r, size_t logical)
{
    return r->base + ((r->head + logical) % r->capacity) * r->elem;
}

/* Returns the slot the value was copied into, or NULL when the ring has no
 * storage at all (a disabled facility, not a failure). */
static void *ring_push(wr_ring *r, const void *value)
{
    void *slot;

    if (r->base == NULL || r->capacity == 0u) {
        return NULL;
    }
    if (r->count == r->capacity) {
        r->head = (r->head + 1u) % r->capacity;
        r->count--;
        r->dropped++;
    }
    slot = ring_slot(r, r->count);
    memcpy(slot, value, r->elem);
    r->count++;
    return slot;
}

static size_t ring_pop(wr_ring *r, void *out, size_t max)
{
    size_t n = (max < r->count) ? max : r->count;
    for (size_t i = 0; i < n; ++i) {
        memcpy((uint8_t *)out + i * r->elem, ring_slot(r, i), r->elem);
    }
    r->head = (r->head + n) % ((r->capacity != 0u) ? r->capacity : 1u);
    r->count -= n;
    return n;
}

static void ring_clear(wr_ring *r)
{
    r->head = 0u;
    r->count = 0u;
}

/* ------------------------------------------------------------------------ */
/* Deadlines                                                                 */
/* ------------------------------------------------------------------------ */
/*
 * One row per row of design.md §5.7 that phase 2 arms.  The calibration and
 * history rows land with their state machines: a row whose `fire` is NULL is
 * caught at create rather than becoming a silent no-op, so an unimplemented row
 * cannot be added by accident.
 */
typedef enum wr_due_id {
    WR_DUE_KEEPALIVE = 0,
    WR_DUE_BRINGUP,
    WR_DUE_STREAM_START,
    WR_DUE_STREAM_STOP, /* ⚠ the device's `0x83`; see due_stream_stop()        */
    WR_DUE_CAL_DEVICE,   /* a marker's `a2 01` reply, or the `0x94` result   */
    WR_DUE_CAL_RAISE,    /* ⚠ CLIENT policy — the device imposes no deadline */
    WR_DUE_CAL_PRESENCE, /* the presence run's window over live samples      */
    WR_DUE_CLOCK_EVENT,
    WR_DUE_KEEPALIVE_ALARM,
    WR_DUE_LIVE_GAP,
    WR_DUE_PINNED_REPORT,
    WR_DUE_POWER_OFF_LINGER,
    WR_DUE_BRACKET_LIMIT,
    /* ⚠ AGGREGATES over the request table, recomputed by history_arm() every
     * time the table changes, so the O(1) next_due_us() scan stays O(1) in
     * queue depth.  All three of §5.7's history rows are live; adding an enum
     * row without the `fire` beside it fails wr_session_create(), which is the
     * point. */
    WR_DUE_HISTORY_START,    /* the earliest reservation whose window can close */
    WR_DUE_HISTORY_DEADLINE, /* the earliest outstanding request's own deadline */
    WR_DUE_HISTORY_EVICTION, /* §8.6: the earliest queued window at risk of it  */
    WR_DUE_COUNT
} wr_due_id;

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */
typedef enum wr_link_state {
    WR_LINK_DOWN = 0,
    WR_LINK_BRINGUP,
    WR_LINK_READY,
    WR_LINK_CLOSED
} wr_link_state;

typedef enum wr_stream_state {
    WR_STREAM_STOPPED = 0,
    WR_STREAM_STARTING, /* ⚠ waiting for the first 0x90, NOT for the a0 01 ack */
    WR_STREAM_RUNNING,
    WR_STREAM_STOPPING
} wr_stream_state;

/* ------------------------------------------------------------------------ */
/* History — the gather (design §8.4)                                        */
/* ------------------------------------------------------------------------ */
/*
 * The public wr_history_block is the FIRST member, so wr_history_block_release()
 * recovers everything from the one pointer the caller holds — including a COPY
 * of the allocator, which is what lets a block outlive its session and be freed
 * on somebody else's thread (design §8.4.2).  The four arrays live in the same
 * allocation, after this header.
 */
typedef struct wr_history_record {
    wr_history_block pub; /* ⚠ MUST BE FIRST */
    uint32_t         magic;
    uint32_t         reserved;
    wr_allocator     alloc; /* ⚠ a COPY — see above */
    size_t           bytes;
} wr_history_record;

typedef enum wr_gather_state {
    WR_GATHER_FREE = 0,
    WR_GATHER_QUEUED,    /* reserved; waiting for the window to close, or for a turn */
    WR_GATHER_REQUESTED, /* `a1` queued; ⚠ waiting for `a1 02`, the ACCEPTANCE TEST */
    WR_GATHER_BRACKET,   /* records arriving                                        */
    WR_GATHER_REFILL,    /* an attempt ended holed; waiting for a live frame        */
    WR_GATHER_READY      /* block materialised, waiting to be collected             */
} wr_gather_state;

/* One attempt's records, as they landed in the shared gather area.  Each run is
 * ascending on its own; they are merged k-way at materialisation, which is
 * where duplicates across attempts die (api-request C3). */
typedef struct wr_gather_run {
    size_t start;
    size_t count;
} wr_gather_run;

typedef struct wr_request {
    wr_gather_state    state;
    uint64_t           id;
    wr_history_request req; /* the caller's, copied and clamped */
    wr_time_us         reserved_us;

    /*
     * ⚠ THE FIT IS SNAPSHOTTED WHEN THE WINDOW CLOSES, NOT AT MATERIALISATION,
     * AND §6.1.1 IS THE REASON.  The sample counter stalls for the duration of
     * every pull, so the fit re-anchors at each bracket close and the mapping
     * that dates THESE indices is the one in force before the pull that
     * retrieves them.  Snapshotting at materialisation would date every block
     * with the offset of the stretch that starts AFTER it — wrong by the width
     * of the stall, silently, on every pull.
     *
     * It is also the snapshot carried on the block and used for every sample in
     * it, so the block stays internally reproducible (implementation-notes §4).
     */
    wr_clock_snapshot fit;
    bool              have_fit;

    wr_index_range indices;  /* what the window maps to — unwrapped, inclusive */
    wr_index_range recorded; /* ...clamped to what the device can have taken   */
    wr_index_range ask;      /* ...and the sub-range of the attempt in flight  */
    bool           have_indices;
    /* ⚠ An EXPLICIT flag, because a width cannot be the sentinel: wr_index_range
     * is inclusive, so index_width({0,0}) is 1 and a zeroed `recorded` reads as
     * "the single index 0" (implementation-review I7).  Two refusals reach
     * history_finish() with `indices` set and this false. */
    bool           have_recorded;

    /*
     * ⚠ §8.3's WRAP SPLIT.  `a1` takes u16be and §7.1 requires first < last, so
     * one command cannot cross the 82.0 s counter wrap.  A window that does is
     * issued as TWO asks — [first, …ffff] and [next base, last] — and this is
     * the far side of it, waiting for the near side's bracket to close.  The
     * two are merged by UNWRAPPED index by the machinery a refill already uses.
     */
    wr_index_range pending_ask;
    bool           have_pending_ask;

    /*
     * ⚠ Where the buffer's head stood when the FIRST `a1` went out, which is
     * what §8.5's depth is measured back from.  Taken at the ask rather than at
     * materialisation because the counter stalls for the pull's own duration
     * (§7.5) — head_index by then belongs to the stretch after the pull.
     */
    uint32_t head_at_ask;
    bool     have_head_at_ask;

    /* §8.6 fires at most once per request: the risk does not become newer by
     * being restated, and the event ring is drop-oldest. */
    bool eviction_warned;

    uint8_t       attempts;
    wr_gather_run run[HISTORY_MAX_ATTEMPTS];
    size_t        run_count;
    bool          run_open;
    bool          run_have_index;
    uint32_t      run_last_index;
    size_t        sample_count;

    wr_coverage      cov;
    wr_pinned_counts pinned;
    uint32_t         out_of_range;
    uint32_t         out_of_range_reported;
    uint32_t         duplicates;
    uint32_t         overflowed;
    uint32_t         overlap_samples;
    uint32_t         overlap_mismatches;

    /* ⚠ VALUE-COPIED from the live unwrappers at `a1 02`, so history device time
     * lands on the live timeline without polluting the live ratio fit (§10.1). */
    wr_tick_unwrapper tick[WR_UNIT_COUNT];

    /* The recording hole this pull itself cost (§7.5, api-request B11). */
    wr_time_range stall;
    bool          have_stall;

    wr_history_record *block;
    bool               alloc_failed;
} wr_request;

struct wr_session {
    /* --- Config ---------------------------------------------------------- */
    wr_session_policy policy; /* resolved and clamped at create */
    wr_stream_config  requested_cfg;
    wr_stream_config  active_cfg;
    char              device_id[WR_DEVICE_ID_MAX];
    wr_allocator      alloc;
    void             *owned; /* the ONE allocation; this struct lives in it */
    bool              closed;
    bool              have_now;
    wr_time_us        now_us;

    /* --- Link ------------------------------------------------------------ */
    wr_link_state  link;
    bool           mtu_rejected;
    wr_time_us     link_up_us;
    wr_time_us     last_device_byte_us;
    wr_time_us     last_host_write_us;
    wr_time_us     last_advertising_us;
    bool           have_advertising;
    bool           local_teardown;
    bool           powering_off;
    uint32_t       consecutive_failures;
    wr_device_info info;

    /* --- Stream ---------------------------------------------------------- */
    wr_stream_state stream;
    uint64_t        stream_id;
    uint64_t        next_stream_id;
    uint32_t        stream_starts;
    wr_time_us      stream_started_us; /* phase 4 reads it for the resident range */
    wr_time_us      last_live_us;
    bool            have_live;
    /* The stream's index span so far.  ⚠ Phase 4 needs BOTH ends: `first_index`
     * bounds what the device could still hold and `head_index` is where a
     * request's window is measured back from. */
    uint32_t        first_index;
    uint32_t        head_index;
    bool            have_index;

    /* --- Per-stream decode ------------------------------------------------ */
    wr_index_unwrapper idx; /* ⚠ LIVE ONLY.  History never touches it (§10.1). */
    wr_tick_unwrapper  tick[WR_UNIT_COUNT];
    wr_fit             fit;
    wr_clock_snapshot  snap; /* refreshed once per notification, not per record */
    bool               snap_valid;
    wr_overlap         overlap;
    wr_pinned_counts   pinned;      /* since the last WR_EV_PINNED_SAMPLES */
    uint64_t           pinned_span; /* samples those counts cover */
    bool               clock_degraded;
    bool               tick_margin_warned; /* once per stream */

    /* --- Calibration ------------------------------------------------------ */
    /* ⚠ Phase 4 reads the previous state and its timestamp to fill
     * wr_calibration_span.spans_transition on a block. */
    wr_calibration_state cal_state;
    wr_calibration_state cal_prev_state;
    wr_time_us           cal_changed_us;

    wr_calibration_phase        cal_phase;
    wr_calibration_phase        cal_phase_prev;
    wr_calibration_abort_reason cal_abort_reason;
    wr_time_us                  cal_started_us;

    /*
     * The presence run.  ⚠ Live samples only — §7.5 measures the reference-pose
     * angle FROM the stream, and a retrieval suspends it, which is why every
     * wr_calibration_* call refuses while a bracket is open.
     */
    bool                          cal_run_active;
    size_t                        cal_run_count;
    wr_sample                     cal_run[WR_PRESENCE_MAX_SAMPLES];
    /* ⚠ Kept as well as evented: the event ring is drop-oldest and the pose has
     * passed, so this measurement cannot be re-derived (design §7.5, R14). */
    wr_calibration_presence_event cal_anchor;
    bool                          cal_have_anchor;

    /* --- History (phase 4) ------------------------------------------------ */
    /* ⚠ Written in exactly two places — on `a1 02` and in close_bracket().
     * Read in exactly one place that matters, the discriminator in on_frame(). */
    bool       bracket_open;
    wr_time_us bracket_open_us;
    /* Records that arrived inside a bracket nobody owns and were discarded.
     * The reportable fact — that live delivery was suspended — is the
     * blind-span event, emitted whatever opened the bracket. */
    uint64_t orphan_records;

    /* Whether the open bracket belongs to reqs[active], and to which id.  An
     * `a1 02` the session never asked for, or one answering a request that has
     * since been abandoned, opens an ORPHAN: its records reach neither the live
     * ring nor wr_fit_observe(). */
    bool     bracket_owned;
    uint64_t bracket_request_id;

    int      active; /* index into reqs[] of the one request in flight, or -1 */
    uint64_t next_request_id;
    /*
     * ⚠ A request we abandoned may still be answered — the device does not know
     * we gave up.  Until that reply arrives (or the bracket limit gives up on
     * it) no second `a1` may go out, or the two replies become one bracket.
     */
    bool replay_pending;
    /*
     * ⚠ §6.1.1: NEVER TWO PULLS INSIDE ONE LIVE-FRAME GAP.  One stall is
     * ~23,500 ticks — 72 % of §10.2's ±32,768 wrap budget — where a pull-free
     * gap uses 12, so two before the next live frame exceeds it and the tick
     * unwrapper silently picks the wrong wrap.  Set at every bracket close and
     * cleared by the next live frame.
     */
    bool pull_needs_live;
    /*
     * A teardown closed a bracket while the device was still replaying.  Those
     * records are byte-identical to live ones (§10.1) and would take the live
     * index unwrapper thousands of samples backwards, so nothing is delivered
     * until the stream actually stops.
     */
    bool abandoned_replay;
    /* Set while the request table is being torn down, so materialising one
     * request cannot let the next one issue an `a1` into a dying session. */
    bool history_frozen;

    /*
     * ⚠ §8.5's DEPTH BRACKET — learned, never a constant, and reset with the
     * stream because §7.3 scopes the buffer to "the current streaming session
     * only".  `depth_lo_us` is the widest reach-back the device has actually
     * served; `depth_hi_us` is the narrowest it demonstrably failed to serve.
     * Neither is a model of the buffer: §7.3's ~7.5 s was measured once and whether
     * the depth is a fixed sample count or a fixed duration is not established,
     * so what is recorded here is a pair of OBSERVATIONS and nothing more.
     *
     * ⚠ The count behind the estimate is not a field here because it is already
     * reachable: wr_history_resident_range() answers WR_OK once anything has
     * been measured and WR_PENDING while it has not.  A counter nothing can
     * read is the same shape as a check that has silently stopped running.
     */
    wr_time_us depth_lo_us;
    wr_time_us depth_hi_us;
    bool       have_depth_lo;
    bool       have_depth_hi;
    /* Both bounds measured and crossing — §7.3's open question answering itself, once
     * per stream, through WR_WARN_HISTORY_DEPTH_CONFLICT. */
    bool depth_conflict_warned;

    wr_request      reqs[HISTORY_MAX_PENDING];
    wr_sample      *gather; /* shared: only the request in flight writes here */
    size_t          gather_cap;
    wr_index_range *cov_storage;
    size_t          cov_cap;

    /* --- Deadlines -------------------------------------------------------- */
    wr_time_us due[WR_DUE_COUNT];
    /* ⚠ The seatbelt's own record.  The OBSERVABLE guarantee is that
     * wr_session_next_due_us() strictly increases when a host steps to exactly
     * it — which tests/test_session.c checks over ten simulated minutes — and
     * this counter is what a debugger looks at when it does not. */
    uint32_t   deadline_clamped;
    bool       in_tick;

    /* --- Warning throttle -------------------------------------------------- */
    uint32_t   warn_pending[WR_WARN_CODE_COUNT];
    wr_time_us warn_last_us[WR_WARN_CODE_COUNT];
    bool       warn_seen[WR_WARN_CODE_COUNT];

    /* --- Output ------------------------------------------------------------ */
    wr_ring  live;
    wr_ring  events;
    wr_ring  wire;
    wr_ring  writes;
    uint32_t event_sequence;
    uint32_t wire_sequence;
    bool     wire_lost_pending;
};

/* ------------------------------------------------------------------------ */
/* Forward declarations                                                      */
/* ------------------------------------------------------------------------ */
/*
 * The history machine reaches three sections of this file that are not next to
 * each other — the deadline handlers above the table, close_bracket() beside
 * the link machine, and the live path — and it needs the live path's stamping
 * helpers itself.  Five declarations are cheaper than moving three unrelated
 * sections around them.
 */
static void history_arm(wr_session *s);
static void history_service(wr_session *s);
static void history_abandon(wr_session *s, wr_request *r, wr_history_status status);
static void history_bracket_opened(wr_session *s);
static void history_bracket_closed(wr_session *s);
static void history_abandon_all(wr_session *s, wr_history_status status);
static bool history_outstanding(const wr_session *s);

/* ------------------------------------------------------------------------ */
/* Allocation                                                                */
/* ------------------------------------------------------------------------ */
static void *default_alloc(void *ctx, size_t size)
{
    (void)ctx;
    return malloc(size);
}

static void default_free(void *ctx, void *ptr)
{
    (void)ctx;
    free(ptr);
}

/*
 * A two-pass bump layout: measure with `base == NULL`, allocate, then place.
 * Everything the session owns lives in ONE allocation, this struct included, so
 * wr_session_destroy() is one free and a caller can account for the library's
 * footprint with a single number.
 */
typedef struct wr_layout {
    uint8_t *base;
    size_t   used;
} wr_layout;

static void *layout_take(wr_layout *l, size_t bytes, size_t align)
{
    size_t offset = (l->used + (align - 1u)) & ~(align - 1u);
    void  *p = NULL;
    if (bytes == 0u) {
        return NULL;
    }
    if (l->base != NULL) {
        /* void* on the way out: the region is 16-aligned and the base comes
         * from the allocator, so this is well defined, and returning void*
         * keeps -Wcast-align out of every call site. */
        p = l->base + offset;
    }
    l->used = offset + bytes;
    return p;
}

/* ------------------------------------------------------------------------ */
/* Events, warnings and the wire log                                         */
/* ------------------------------------------------------------------------ */
static void event_push(wr_session *s, wr_event *ev)
{
    if (s->closed) {
        return; /* the drains are sealed; the caller's ring may already be gone */
    }
    /* ⚠ `sequence` is monotonic per session and is never renumbered, so a gap
     * means the queue overflowed — which wr_session_dropped_events() also
     * counts.  A consumer reading either one can tell "nothing happened" from
     * "we could not tell you". */
    ev->sequence = s->event_sequence++;
    ev->host_time_us = s->now_us;
    ev->stream_id = s->stream_id;
    (void)ring_push(&s->events, ev);
}

static void event_emit(wr_session *s, wr_event_type type)
{
    wr_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)type;
    event_push(s, &ev);
}

/* An unthrottled warning, for the rare deadline-driven ones. */
static void warn_now(wr_session *s, wr_warning_code code, int32_t detail_i32, double detail_f64)
{
    wr_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_WARNING;
    ev.u.warning.code = (uint16_t)code;
    ev.u.warning.detail_i32 = detail_i32;
    ev.u.warning.detail_f64 = detail_f64;
    event_push(s, &ev);
    s->warn_last_us[code] = s->now_us;
    s->warn_seen[code] = true;
}

/*
 * A per-record warning.  ⚠ Reported at most once per second per code, carrying
 * HOW MANY occurrences it stands for in `detail_i32` — a burst is then visible
 * AS a burst, where one event per record would evict the ring in 320 ms and one
 * event per burst would understate it by three orders of magnitude.
 */
static void warn_rate(wr_session *s, wr_warning_code code)
{
    s->warn_pending[code]++;
    if (s->warn_seen[code] && (s->now_us - s->warn_last_us[code]) < WARN_THROTTLE_US) {
        return;
    }
    {
        int32_t n = (s->warn_pending[code] > (uint32_t)INT32_MAX) ? INT32_MAX
                                                                 : (int32_t)s->warn_pending[code];
        s->warn_pending[code] = 0u;
        warn_now(s, code, n, 0.0);
    }
}

/* Any code whose burst ended before its throttle expired still has to be told.
 * Ticks happen at least every keepalive period, so nothing waits long. */
static void warn_flush(wr_session *s)
{
    for (int i = 0; i < (int)WR_WARN_CODE_COUNT; ++i) {
        if (s->warn_pending[i] == 0u) {
            continue;
        }
        if (s->warn_seen[i] && (s->now_us - s->warn_last_us[i]) < WARN_THROTTLE_US) {
            continue;
        }
        {
            int32_t n = (s->warn_pending[i] > (uint32_t)INT32_MAX)
                            ? INT32_MAX
                            : (int32_t)s->warn_pending[i];
            s->warn_pending[i] = 0u;
            warn_now(s, (wr_warning_code)i, n, 0.0);
        }
    }
}

/*
 * ⚠ Record the WIRE BYTES, not the decoded samples (design §5.6).  §12 still
 * lists undecoded fields, so a byte-level recording can be re-decoded when one
 * of them is settled and a sample-level one cannot.
 */
static void wire_log(wr_session *s, wr_wire_direction dir, const uint8_t *data, size_t length,
                     bool redact)
{
    wr_wire_chunk chunk;
    void         *slot;

    if (s->closed || s->wire.capacity == 0u) {
        return; /* no ring supplied: the log is off and costs nothing */
    }

    memset(&chunk, 0, sizeof(chunk));
    chunk.host_time_us = s->now_us;
    chunk.direction = (uint8_t)dir;
    /* ⚠ `sequence` is CARRIED, not renumbered by the reader: WR_WIRE_LOST says
     * chunks were dropped but not how many, so a reader numbering from its own
     * ordinal would turn a lossy recording into a complete-looking one. */
    chunk.sequence = s->wire_sequence++;

    if (length > WR_WIRE_CHUNK_MAX) {
        length = WR_WIRE_CHUNK_MAX; /* unreachable: the longest message is 93 B */
    }
    chunk.length = (uint16_t)length;
    if (redact) {
        /* ⚠ MAC (0x85) and serial (0x86) name a specific unit and a specific
         * owner.  The id survives so a reader can see WHAT was removed; the
         * payload does not.  Marked, so absent and redacted stay distinct. */
        chunk.flags |= (uint8_t)WR_WIRE_REDACTED;
        if (length > 0u) {
            chunk.data[0] = data[0];
        }
    } else if (length > 0u) {
        memcpy(chunk.data, data, length);
    }

    if (s->wire_lost_pending) {
        chunk.flags |= (uint8_t)WR_WIRE_LOST;
        s->wire_lost_pending = false;
    }

    {
        uint64_t before = s->wire.dropped;
        slot = ring_push(&s->wire, &chunk);
        if (slot != NULL && s->wire.dropped != before) {
            /* Drop-oldest, so the gap now sits in front of the new head: mark
             * the chunk that FOLLOWS the loss, which is the one a reader
             * encounters next. */
            wr_wire_chunk *head = (wr_wire_chunk *)ring_slot(&s->wire, 0u);
            head->flags |= (uint8_t)WR_WIRE_LOST;
        }
    }
}

/* Link, MTU and stream boundaries, as short ASCII.  tools/wr_capture.py writes
 * the same vocabulary, so one reader handles both. */
static void wire_meta(wr_session *s, const char *text)
{
    wire_log(s, WR_WIRE_META, (const uint8_t *)text, strlen(text), false);
}

/* ------------------------------------------------------------------------ */
/* Time                                                                      */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ The clock contract (types.h) requires a MONOTONIC host clock.  A wall clock
 * stepped by NTP or DST corrupts a capture in a way that looks like a sensor
 * fault, so the session clamps, counts and SAYS SO — it is a consumer-side bug
 * and they cannot fix what they cannot see.
 */
static void note_now(wr_session *s, wr_time_us now_us)
{
    if (!s->have_now) {
        s->have_now = true;
        s->now_us = now_us;
        return;
    }
    if (now_us < s->now_us) {
        warn_now(s, WR_WARN_HOST_CLOCK_REGRESSION, 0, (double)(s->now_us - now_us));
        return; /* clamped: time does not go backwards inside this file */
    }
    s->now_us = now_us;
}

/* ------------------------------------------------------------------------ */
/* Deadline table — arm() and disarm() are the ONLY writers                  */
/* ------------------------------------------------------------------------ */
static void arm(wr_session *s, wr_due_id id, wr_time_us when)
{
    /*
     * ⚠ A deadline re-armed at the instant that fired it spins the host's loop
     * at 100 % CPU — and the loop is THEIRS, so it looks like their bug.  The
     * clamp is the seatbelt; `deadline_clamped` staying 0 is the guarantee, and
     * the ten-minute stepping test is what checks it.
     */
    if (s->in_tick && when <= s->now_us) {
        when = s->now_us + 1;
        s->deadline_clamped++;
    }
    s->due[id] = when;
}

static void disarm(wr_session *s, wr_due_id id)
{
    s->due[id] = WR_TIME_NEVER;
}

static void disarm_all(wr_session *s)
{
    for (int i = 0; i < (int)WR_DUE_COUNT; ++i) {
        s->due[i] = WR_TIME_NEVER;
    }
}

/* ------------------------------------------------------------------------ */
/* Writes                                                                    */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THE WRITE QUIET PERIOD.  Nothing is QUEUED while a history bracket is open,
 * and poll_writes() returns nothing anyway — two independent lines, because the
 * failure they prevent is an `0x81` reply interleaved into a record stream that
 * §3 gives no length field, no sequence number and no checksum to resynchronise
 * on.  The `a1` write itself reset the device's idle timer when the bracket
 * opened, and a retrieval is bounded by the ~7.5 s buffer depth against a 300 s
 * shutdown, so suppression costs nothing (design §5.4).
 */
static bool writes_allowed(const wr_session *s)
{
    if (s->closed || s->link == WR_LINK_CLOSED) {
        return false;
    }
    /* ⚠ An MTU below 96 means the session refuses to run: not one byte leaves
     * (design §5.2).  The calibration result is 65 bytes and stream
     * notifications reach 93. */
    if (s->mtu_rejected) {
        return false;
    }
    if (s->link == WR_LINK_DOWN) {
        return false;
    }
    return !s->bracket_open;
}

static void queue_write(wr_session *s, const wr_write_request *req)
{
    if (!writes_allowed(s)) {
        return;
    }
    (void)ring_push(&s->writes, req);
}

static void queue_command(wr_session *s, wr_status (*encode)(wr_write_request *))
{
    wr_write_request req;
    if (encode(&req) == WR_OK) {
        queue_write(s, &req);
    }
}

/*
 * The same, for the one encoder that takes a parameter — `a2 <pose>`.  It still
 * goes through wr_command_emit()'s allowlist; there is no second path out.
 *
 * ⚠ This one reports, where queue_command() does not.  A calibration phase that
 * advanced without its marker reaching the queue would wait out the device bound
 * and abort for a reason that names the device, so the caller checks.
 */
static wr_status queue_command_arg(wr_session *s,
                                   wr_status (*encode)(wr_write_request *, uint8_t), uint8_t arg)
{
    wr_write_request req;
    wr_status        st = encode(&req, arg);
    if (st < WR_OK) {
        return st;
    }
    queue_write(s, &req);
    return WR_OK;
}

/* ------------------------------------------------------------------------ */
/* Link                                                                      */
/* ------------------------------------------------------------------------ */
static void stream_reset_decode(wr_session *s)
{
    wr_index_unwrapper_reset(&s->idx);
    wr_tick_unwrapper_reset(&s->tick[WR_UNIT_LOWER_ARM]);
    wr_tick_unwrapper_reset(&s->tick[WR_UNIT_PALM]);
    wr_overlap_reset(&s->overlap);
    s->have_index = false;
    s->first_index = 0u;
    s->head_index = 0u;
    s->have_live = false;
    s->snap_valid = false;
    s->tick_margin_warned = false;
    /* ⚠ Pinned counts belong to a stream, and so does the span they are
     * measured over.  Carrying either across a restart would report a rate of
     * saturation against samples from an index space that no longer exists. */
    wr_pinned_counts_reset(&s->pinned);
    s->pinned_span = 0u;
    disarm(s, WR_DUE_PINNED_REPORT);
    /* ⚠ §7.4: starting a stream CLEARS the device's buffer and the index space
     * with it, so nothing survives here either.  The requests themselves were
     * already cancelled by the stop that preceded this. */
    s->abandoned_replay = false;
    s->pull_needs_live = false;
    s->replay_pending = false;
    /*
     * ⚠ AND THE DEPTH BRACKET GOES WITH IT (§8.5).  §7.3 scopes the buffer to
     * the current streaming session and §7.4 measured a restart clearing it
     * outright, so a bound learned before one says nothing about after it.
     * Carrying it across would be the same class of error as carrying the index
     * space across: a number that still looks like a measurement of this
     * connection and is a measurement of a buffer that no longer exists.
     */
    s->depth_lo_us = 0;
    s->depth_hi_us = 0;
    s->have_depth_lo = false;
    s->have_depth_hi = false;
    s->depth_conflict_warned = false;
}

static void set_calibration_state(wr_session *s, wr_calibration_state state)
{
    if (state == s->cal_state) {
        return;
    }
    s->cal_prev_state = s->cal_state;
    s->cal_state = state;
    s->cal_changed_us = s->now_us;
}

/*
 * ⚠ THE STATE AT AN INSTANT, NOT THE STATE NOW, AND FOR A HISTORY SAMPLE THOSE
 * ARE ROUTINELY DIFFERENT (implementation-review I3).
 *
 * stamp_with() used to read `s->cal_state` directly.  That is right for a live
 * sample, whose capture instant IS now, and wrong for a retrieved one — and the
 * R8 interlock makes the bad ordering ordinary rather than exotic:
 * history_service() refuses to open a bracket while a calibration routine runs,
 * so a window recorded BEFORE a routine is held until after it and every sample
 * in it would be stamped with the state the routine produced.  Pre-transform
 * samples labelled CALIBRATED, permanently and invisibly (sample.h).
 *
 * ⚠ Resolving by TIME rather than by capturing at reserve also settles the case
 * one request can straddle: history_service()'s in-flight branch holds a REFILL
 * across a routine too, so attempt 1 and attempt 2 of one request can fall on
 * opposite sides of a transition and must still agree.
 *
 * ⚠ Only one transition back is retained (`cal_prev_state`, `cal_changed_us`),
 * so two calibrations between capture and materialisation cannot be
 * reconstructed — and a round trip leaves prev == current and reads as no
 * transition at all.  The block's `wr_calibration_span` has the same limit and
 * says so; nothing on the wire offers more.
 *
 * ⚠ Never returns WR_CAL_LOST.  It cannot: set_calibration_state() is only ever
 * called with UNKNOWN, UNCALIBRATED or CALIBRATED.  LOST belongs to
 * wr_calibration_span alone (sample.h, history.h) and no sample carries it.
 */
static wr_calibration_state cal_state_at(const wr_session *s, wr_time_us at_us)
{
    if (s->cal_prev_state == s->cal_state || s->cal_changed_us == 0 ||
        at_us == WR_TIME_UNKNOWN || at_us >= s->cal_changed_us) {
        return s->cal_state;
    }
    return s->cal_prev_state;
}

/* ------------------------------------------------------------------------ */
/* Calibration — the machine of design §7                                    */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THREE FACTS SHAPE EVERY LINE BELOW, AND ALL THREE ARE MEASURED.
 *
 * 1. `0x94` IS NOT A VERDICT (§8.2).  The device emits it for every `a2 01` and
 *    applies the transform every time, including attempts an application goes on
 *    to reject.  Nothing on the wire carries accept or reject, so reaching
 *    WR_CALP_COMPLETE is not evidence that calibration took.
 * 2. THE PRESENCE ANGLE INVERTS (§8.2).  A calibration with no raise at all —
 *    carrying no axis information whatsoever — scored 0.70° against 1.96° for
 *    the correct routine.  So it is used to separate "applied" from "absent",
 *    where the gap is an order of magnitude, and for nothing else.  There is no
 *    score here and no ranking of two attempts.
 * 3. SKIPPING THE CHECK LEAVES THE FLAG AT WR_CAL_UNKNOWN (§7.5, design review
 *    12.6).  Never at CALIBRATED.  The recording must say we did not check.
 *
 * The device imposes no deadline between the markers — one attempt took 15.6 s
 * and was applied — so every bound in here is CLIENT POLICY and is named as
 * such.
 */
static bool cal_in_progress(const wr_session *s)
{
    switch (s->cal_phase) {
        case WR_CALP_AWAIT_HORIZONTAL:
        case WR_CALP_MARKING_POSE0:
        case WR_CALP_OBSERVING_RAISE:
        case WR_CALP_MARKING_POSE1:
        case WR_CALP_APPLYING:
        case WR_CALP_VERIFYING:
            return true;
        case WR_CALP_IDLE:
        case WR_CALP_COMPLETE:
        case WR_CALP_ABORTED:
        case WR_CALIBRATION_PHASE_COUNT:
            break;
    }
    return false;
}

/*
 * Every transition re-arms the deadline table from scratch: the phase the
 * machine is IN decides which bound applies, so a transition cannot leave a
 * stale one armed behind it (implementation-notes §3, rule 3).
 */
static void cal_arm_for_phase(wr_session *s)
{
    disarm(s, WR_DUE_CAL_DEVICE);
    disarm(s, WR_DUE_CAL_RAISE);
    disarm(s, WR_DUE_CAL_PRESENCE);

    switch (s->cal_phase) {
        case WR_CALP_MARKING_POSE0:
        case WR_CALP_MARKING_POSE1:
        case WR_CALP_APPLYING:
            /* Waiting on the device: a marker's `a2 01` reply, or the result.
             * ⚠ Measured from the QUEUEING of the marker rather than from its
             * drain, so a host sitting on poll_writes() spends the bound it was
             * given — which is the failure it has, and WR_WARN_KEEPALIVE_LATE
             * names it directly an order of magnitude before this matters. */
            arm(s, WR_DUE_CAL_DEVICE, s->now_us + s->policy.calibration_result_timeout_us);
            break;
        case WR_CALP_OBSERVING_RAISE:
            /* ⚠ CLIENT policy.  §8.2 measured the device returning and applying
             * a result 15.6 s after the first marker; it was the vendor's
             * APPLICATION that rejected it.  A raise still cannot be arbitrarily
             * slow and remain one motion, which is why a bound exists at all. */
            arm(s, WR_DUE_CAL_RAISE, s->now_us + s->policy.calibration_raise_limit_us);
            break;
        /* WR_CALP_VERIFYING has no bound of its own: the presence run is
         * optional, and its window is armed by the call that starts it. */
        case WR_CALP_IDLE:
        case WR_CALP_AWAIT_HORIZONTAL:
        case WR_CALP_VERIFYING:
        case WR_CALP_COMPLETE:
        case WR_CALP_ABORTED:
        case WR_CALIBRATION_PHASE_COUNT:
            break;
    }
}

static void cal_set_phase(wr_session *s, wr_calibration_phase phase,
                          wr_calibration_abort_reason reason)
{
    wr_event ev;

    s->cal_phase_prev = s->cal_phase;
    s->cal_phase = phase;
    s->cal_abort_reason = reason;
    cal_arm_for_phase(s);

    /* Every transition is evented, so a UI renders progress from the queue
     * rather than polling for it (design §7.2). */
    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_CALIBRATION_PHASE;
    ev.u.calibration_phase.phase = (uint8_t)phase;
    ev.u.calibration_phase.previous_phase = (uint8_t)s->cal_phase_prev;
    ev.u.calibration_phase.abort_reason = (uint8_t)reason;
    ev.u.calibration_phase.elapsed_us = (s->cal_started_us != 0) ? (s->now_us - s->cal_started_us)
                                                                : 0;
    event_push(s, &ev);
}

/* The run is abandoned without a measurement; the phase is the caller's. */
static void cal_run_stop(wr_session *s)
{
    s->cal_run_active = false;
    s->cal_run_count = 0u;
    disarm(s, WR_DUE_CAL_PRESENCE);
}

static void cal_abort(wr_session *s, wr_calibration_abort_reason reason)
{
    if (!cal_in_progress(s)) {
        return;
    }
    cal_run_stop(s);
    /*
     * ⚠ The calibration STATE is not touched here.  Before `0x94` it is whatever
     * it was and no transform has been applied; after `0x94` it is already
     * WR_CAL_UNKNOWN and the device has re-referenced its stream — an abort
     * cannot undo that, and claiming UNCALIBRATED would label anatomically
     * referenced samples as raw ones.
     */
    cal_set_phase(s, WR_CALP_ABORTED, reason);
}

/*
 * The run is over.  Either it has enough to average, or it says so — ⚠ "no
 * evidence" is not "agreement", and a presence check that could not be taken
 * must not read as one that passed.
 */
static void cal_finish_presence(wr_session *s)
{
    wr_event ev;
    size_t   n = s->cal_run_count;

    s->cal_run_active = false;
    disarm(s, WR_DUE_CAL_PRESENCE);

    if (n < PRESENCE_RUN_MIN ||
        wr_presence_select_reference(s->cal_run, n, &s->cal_anchor) != WR_OK) {
        s->cal_run_count = 0u;
        warn_now(s, WR_WARN_PRESENCE_NOT_MEASURED, (int32_t)n, 0.0);
        /* The transform IS applied, so the routine completed; what did not
         * happen is the check, and the state stays UNKNOWN to say so. */
        cal_set_phase(s, WR_CALP_COMPLETE, WR_CAL_ABORT_NONE);
        return;
    }
    s->cal_run_count = 0u;
    s->cal_have_anchor = true;

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_CALIBRATION_PRESENCE;
    ev.u.calibration_presence = s->cal_anchor;
    event_push(s, &ev);

    /* §7.5's three bands.  wr_presence_classify() drew them from §8.2's two
     * populations and returns WR_CAL_UNKNOWN for the gap between them. */
    switch ((wr_calibration_state)s->cal_anchor.state) {
        case WR_CAL_CALIBRATED:
            set_calibration_state(s, WR_CAL_CALIBRATED);
            break;
        case WR_CAL_UNCALIBRATED:
            warn_now(s, WR_WARN_CALIBRATION_ABSENT, 0,
                     (double)s->cal_anchor.relative_angle_deg);
            set_calibration_state(s, WR_CAL_UNCALIBRATED);
            break;
        case WR_CAL_UNKNOWN:
        case WR_CAL_LOST:
        default:
            /* ⚠ Between the two populations, so it is evidence of NEITHER: the
             * state is left exactly as it was.  Moving it either way on this
             * band is how a presence check becomes a quality score. */
            warn_now(s, WR_WARN_CALIBRATION_INDETERMINATE, 0,
                     (double)s->cal_anchor.relative_angle_deg);
            break;
    }
    cal_set_phase(s, WR_CALP_COMPLETE, WR_CAL_ABORT_NONE);
}

/* One live sample, offered to a run that may not be collecting.  Called from
 * live_deliver() and from nowhere else — history records reach neither. */
static void cal_run_note_sample(wr_session *s, const wr_sample *sample)
{
    if (!s->cal_run_active) {
        return;
    }
    if (s->cal_run_count < WR_PRESENCE_MAX_SAMPLES) {
        s->cal_run[s->cal_run_count++] = *sample;
    }
    if (s->cal_run_count >= WR_PRESENCE_MAX_SAMPLES) {
        cal_finish_presence(s);
    }
}

static void stop_stream_locally(wr_session *s, bool emit_event)
{
    if (s->stream == WR_STREAM_STOPPED) {
        return;
    }
    /* ⚠ §8.2: the device observes a CONTINUOUS RAISE between the markers, so a
     * routine without a stream behind it has nothing to observe — and a presence
     * run has nothing to measure.  The reason is carried; it is not an error. */
    cal_abort(s, WR_CAL_ABORT_STREAM_LOST);
    s->stream = WR_STREAM_STOPPED;
    /* Whatever the device was still replaying, it has stopped now. */
    s->abandoned_replay = false;
    /*
     * ⚠ And any reservation left is answered NOW rather than at its deadline.
     * The consumer-initiated teardowns have already emptied the table by the
     * time they get here; what reaches this line is a stream the DEVICE stopped,
     * and a request that then goes quiet for its whole deadline costs a
     * consumer's gather a pipeline stall for no information (§8.4.1's argument,
     * applied to the third way a stream can end).
     */
    history_abandon_all(s, WR_HIST_NO_STREAM);
    disarm(s, WR_DUE_STREAM_START);
    disarm(s, WR_DUE_STREAM_STOP);
    disarm(s, WR_DUE_LIVE_GAP);
    disarm(s, WR_DUE_CLOCK_EVENT);
    if (emit_event) {
        wr_event ev;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)WR_EV_STREAM_STOPPED;
        ev.u.stream.stream_id = s->stream_id;
        ev.u.stream.config_bits = s->active_cfg.bits;
        event_push(s, &ev);
        wire_meta(s, "stream_stop");
    }
}

static void close_bracket(wr_session *s)
{
    bool owned;

    if (!s->bracket_open) {
        return;
    }
    s->bracket_open = false;
    s->replay_pending = false;
    owned = s->bracket_owned;
    s->bracket_owned = false;
    disarm(s, WR_DUE_BRACKET_LIMIT);

    /*
     * ⚠ Live delivery was suspended for the whole bracket, and nothing on the
     * wire marks it (api-request B11).  A consumer stitching a session's lane
     * has no other way to learn that a span was never delivered rather than
     * merely never requested — so the span is reported whatever opened the
     * bracket.  `request_id` is 0 for one the session never asked for.
     */
    {
        wr_event ev;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)WR_EV_HISTORY_BLIND_SPAN;
        ev.u.history_blind_span.request_id = s->bracket_request_id;
        ev.u.history_blind_span.span.start_us = s->bracket_open_us;
        ev.u.history_blind_span.span.end_us = s->now_us;
        event_push(s, &ev);
    }
    s->bracket_request_id = 0u;

    /*
     * ⚠⚠ RE-ANCHOR THE FIT, AND THIS IS THE HALF OF §6.1.1 THE REVIEWED DESIGN
     * DID NOT KNOW.  The device stops counting samples while it replays them
     * (§7.5, measured across six pulls: the stall is 90-99 % of the pull's own
     * duration).  Wall time advances, the index does not, and nothing in the
     * data marks the step — so one line cannot span a pull.  §10.2 measured the
     * cost of pretending otherwise: a fit anchored once at stream start was out
     * by 111,311 ticks after five pulls, 1.70 wraps of 65,536, and fitting the
     * whole 44.5 s capture as one segment returned 768 Hz against a true 801.
     *
     * One rate per connection, one offset per stretch between pulls.
     * wr_fit_begin_stream() is exactly that operation: it folds this stretch's
     * slope into the connection-pooled rate and starts the offset again.  The
     * INDEX space is untouched — only the host mapping restarts — so the live
     * unwrapper, the tick unwrappers and head_index all carry straight on.
     *
     * ⚠ Order matters: any block gathered here was already dated from the
     * snapshot taken when its window closed, which is the stretch its indices
     * belong to.  Re-anchoring is for what comes next.
     */
    wr_fit_set_blind(&s->fit, false);
    wr_fit_begin_stream(&s->fit, s->stream_id);
    s->snap_valid = false;
    s->pull_needs_live = true;

    /* The quiet period is over: re-arm the keepalive from now, since the write
     * that would have served it was suppressed. */
    if (s->link == WR_LINK_READY) {
        arm(s, WR_DUE_KEEPALIVE, s->now_us + s->policy.keepalive_period_us);
    }
    /*
     * ⚠ And re-arm the live-gap alarm here, not only on the next live frame.
     * The alarm is SUPPRESSED inside a bracket rather than merely delayed, so
     * without this a pull that ends with the device wedged would leave the one
     * warning that says "no frames are arriving" disarmed for good — a silence
     * indistinguishable from a healthy stream.
     */
    if (s->stream == WR_STREAM_RUNNING) {
        arm(s, WR_DUE_LIVE_GAP, s->now_us + s->policy.live_gap_alarm_us);
    }

    if (owned) {
        history_bracket_closed(s);
    } else {
        history_service(s);
    }
}

static wr_recovery_advice classify_link_down(const wr_session *s, wr_link_down_cause cause,
                                             wr_time_us *out_delay)
{
    wr_time_us idle_write = s->now_us - s->last_host_write_us;
    wr_time_us quiet = s->now_us - s->last_device_byte_us;

    *out_delay = 0;

    /* We asked for it: power-off needs a physical button press to return (§9.3)
     * and close() means the consumer is finished. */
    if (s->powering_off || s->local_teardown) {
        return WR_RECOVER_DO_NOT_RETRY;
    }
    /* ⚠ The device accepts ONE connection and the vendor app wins the race if
     * it is running (§2.2).  "Another application is using this sensor" is
     * something a user can act on; a retry loop against it is pure waste. */
    if (cause == WR_LINK_DOWN_CONNECTION_TAKEN) {
        return WR_RECOVER_NEEDS_OTHER_APP_CLOSED;
    }
    /*
     * ⚠ A slept device needs a PHYSICAL BUTTON PRESS AND NOTHING ELSE (§9.6).
     * The evidence is ours alone: how long since a host→device write, which is
     * what the device's 300 s idle timer actually measures (§9.2), and whether
     * the far end closed cleanly after a long quiet.
     */
    if (s->last_host_write_us != 0 && idle_write >= SLEEP_SUSPECT_IDLE_US) {
        return WR_RECOVER_NEEDS_BUTTON_PRESS;
    }
    if (cause == WR_LINK_DOWN_REMOTE_CLOSED && quiet >= SLEEP_SUSPECT_QUIET_US) {
        return WR_RECOVER_NEEDS_BUTTON_PRESS;
    }

    /* A device that is still advertising did not go to sleep — and it will stop
     * advertising within seconds, so this is the one case worth hurrying. */
    if (s->have_advertising && (s->now_us - s->last_advertising_us) <= ADVERTISING_FRESH_US) {
        *out_delay = RETRY_ADVERTISING_US;
        return WR_RECOVER_RECONNECT_WITH_BACKOFF;
    }

    {
        wr_time_us delay = RETRY_BACKOFF_FIRST_US;
        for (uint32_t i = 0; i < s->consecutive_failures && delay < RETRY_BACKOFF_MAX_US; ++i) {
            delay *= 2;
        }
        *out_delay = (delay > RETRY_BACKOFF_MAX_US) ? RETRY_BACKOFF_MAX_US : delay;
    }
    return WR_RECOVER_RECONNECT_WITH_BACKOFF;
}

static void go_link_down(wr_session *s, wr_link_down_cause cause)
{
    wr_event   ev;
    wr_time_us delay = 0;

    if (s->link == WR_LINK_DOWN || s->link == WR_LINK_CLOSED) {
        return;
    }

    /* ⚠ BEFORE the stream stops, so the routine reports the reason it actually
     * ended for.  A link drop costs the calibration outright (§8.3), where a
     * stream stop only ends the routine — reporting STREAM_LOST here would name
     * the smaller of the two failures. */
    cal_abort(s, WR_CAL_ABORT_LINK_LOST);
    /*
     * ⚠ THE CALIBRATION STATE MOVES BEFORE THE RESERVATIONS ARE ANSWERED, and
     * the ordering is the whole of WR_CAL_LOST.  A block gathered under a
     * transform that this disconnect has just destroyed has to say so — and
     * `state_at_end = LOST` is only reachable once the state has actually
     * moved.  Answering first would hand back a block claiming a calibration
     * that no longer exists and could never be re-checked.
     */
    set_calibration_state(s, WR_CAL_UNCALIBRATED);
    /*
     * ⚠ IMMEDIATELY, NOT AT THE DEADLINE (§8.4.1).  The stream stopped and the
     * index space went with it, so the request can never be fulfilled — and a
     * request that goes quiet after the link has visibly died costs a
     * consumer's gather a pipeline stall for no information.  Anything that
     * HAPPENS TO US is LINK_LOST; anything the consumer initiates is CANCELLED.
     *
     * ⚠ And BEFORE stop_stream_locally(), which would otherwise answer them
     * first with the smaller of the two truths: a link drop is not merely a
     * stream that ended.
     */
    history_abandon_all(s, WR_HIST_LINK_LOST);
    stop_stream_locally(s, s->stream != WR_STREAM_STOPPED);
    close_bracket(s);

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_LINK_DOWN;
    ev.u.link_down.cause = (uint8_t)cause;
    ev.u.link_down.advice = (uint8_t)classify_link_down(s, cause, &delay);
    /*
     * ⚠ ALWAYS 1.  §8.3 measured 0.70° immediately before dropping a link and
     * 18.80° at the same pose after reconnecting, strap untouched and never
     * removed.  A reconnect that silently kept the flag at Calibrated is the
     * single worst bug this API could ship, so it is not expressible.
     */
    ev.u.link_down.calibration_invalidated = 1u;
    ev.u.link_down.connected_for_us = (s->link_up_us != 0) ? (s->now_us - s->link_up_us) : 0;
    ev.u.link_down.since_last_device_byte_us =
        (s->last_device_byte_us != 0) ? (s->now_us - s->last_device_byte_us) : 0;
    ev.u.link_down.suggested_retry_delay_us =
        (ev.u.link_down.advice == (uint8_t)WR_RECOVER_RECONNECT_WITH_BACKOFF) ? delay : 0;
    event_push(s, &ev);

    if (ev.u.link_down.advice == (uint8_t)WR_RECOVER_RECONNECT_WITH_BACKOFF) {
        s->consecutive_failures++;
    }

    /* ⚠ The anchor belongs to the calibration that just died with the link, and
     * wr_calibration_reference_anchor() promises "the CURRENT calibration".
     * Keeping it would hand a consumer a frame-reconciliation anchor solved
     * against a mount transform the device no longer holds.
     *
     * ⚠ Dropped AFTER the reservations were answered, deliberately: a block
     * whose samples were captured under that calibration still reports the
     * angle it was measured at, which is what makes `state_at_end = LOST` a
     * statement about something rather than about nothing. */
    s->cal_have_anchor = false;
    s->link = WR_LINK_DOWN;
    s->powering_off = false;
    disarm_all(s);
    /* Queued writes cannot reach a link that is down; the event above is the
     * loud signal, not a silently drained queue. */
    ring_clear(&s->writes);
    wire_meta(s, "link_down");
}

static void enter_ready(wr_session *s)
{
    wr_event ev;

    if (s->link != WR_LINK_BRINGUP) {
        return;
    }
    s->link = WR_LINK_READY;
    s->consecutive_failures = 0u;
    disarm(s, WR_DUE_BRINGUP);

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_DEVICE_INFO;
    ev.u.device_info = s->info;
    /* ⚠ `valid` says which replies arrived.  An informational step going
     * unanswered is tolerated (§9.1), and the bitmask is how that stays visible
     * rather than reading as a zero. */
    event_push(s, &ev);

    event_emit(s, WR_EV_READY);

    /* §9.2 — the keepalive runs from READY onward, always, for the whole
     * connection.  There is no way to switch it off. */
    arm(s, WR_DUE_KEEPALIVE, s->now_us + s->policy.keepalive_period_us);
    arm(s, WR_DUE_KEEPALIVE_ALARM, s->now_us + s->policy.keepalive_alarm_us);
}

static void maybe_enter_ready(wr_session *s)
{
    if (s->link != WR_LINK_BRINGUP) {
        return;
    }
    if ((s->info.valid & BRINGUP_REQUIRED_INFO) != BRINGUP_REQUIRED_INFO) {
        return;
    }
    if ((s->info.valid & BRINGUP_ALL_INFO) != BRINGUP_ALL_INFO) {
        return; /* still waiting; the watchdog is the other way out */
    }
    enter_ready(s);
}

/* ------------------------------------------------------------------------ */
/* Deadline handlers                                                         */
/* ------------------------------------------------------------------------ */
static void due_keepalive(wr_session *s)
{
    /*
     * ⚠ §9.2, measured: a connection streaming continuously at 25 Hz was
     * dropped at exactly 5.0 minutes, the same deadline as a fully silent one.
     * What resets the device's idle timer is a host→device WRITE, so the poll
     * is unconditional and the battery reading is incidental.
     */
    if (s->link != WR_LINK_READY) {
        return;
    }
    if (!s->bracket_open) {
        queue_command(s, wr_cmd_status);
    }
    /* Inside a bracket the write is suppressed, not lost: the `a1` that opened
     * it already served the idle timer, and close_bracket() re-arms from there. */
    arm(s, WR_DUE_KEEPALIVE, s->now_us + s->policy.keepalive_period_us);
}

static void due_bringup(wr_session *s)
{
    if (s->link != WR_LINK_BRINGUP) {
        return;
    }
    /*
     * One watchdog, two outcomes, and which one it is depends on whether the
     * ONE required step answered (§9.1):
     *
     *   0x80 seen  → READY.  The informational replies are allowed to go
     *                missing; `wr_device_info.valid` records which did.
     *   otherwise  → the link is useless.  The session goes DOWN and classifies,
     *                because the host owns the radio and only it can reconnect.
     */
    if ((s->info.valid & BRINGUP_REQUIRED_INFO) == BRINGUP_REQUIRED_INFO) {
        enter_ready(s);
        return;
    }
    go_link_down(s, WR_LINK_DOWN_UNKNOWN);
}

static void due_stream_start(wr_session *s)
{
    if (s->stream != WR_STREAM_STARTING) {
        return;
    }
    /*
     * §6.1 measures the first 0x90 arriving within 50-80 ms, so the 3 s default
     * is two orders of margin and anything approaching it means the start was
     * not accepted.  ⚠ The session does NOT retry: a start that failed once
     * will fail again and the consumer needs to see it.
     */
    stop_stream_locally(s, false);
    warn_now(s, WR_WARN_STREAM_START_TIMEOUT, 0, 0.0);
}

/*
 * ⚠ THE ONE DEVICE-FACING WAIT §5.7 LEFT UNBOUNDED (implementation-review I9).
 *
 * stop_stream() sets WR_STREAM_STOPPING and queues `83`; the only way out was
 * the device's own `0x83` reply.  A device that drops it — or a host that stops
 * draining poll_writes(), so the `83` never even leaves — wedged the session
 * there for ever: start_stream() returned WR_ERR_INVALID_STATE, cal_guard()
 * returned WR_ERR_NO_STREAM, history_service() never issued, and
 * wr_history_resident_range() refused, with no event or warning explaining any
 * of it.  Every other wait in §5.7 is bounded; this one now is too.
 *
 * ⚠ It shares `stream_start_timeout_us` rather than growing the policy struct.
 * Both bound the same round trip — a stream command and the device's answer to
 * it — and §6.1 measured that at 50-80 ms, so the 3 s default is two orders of
 * margin either way.  A second knob would be a second thing to get wrong with
 * no second measurement behind it.
 */
static void due_stream_stop(wr_session *s)
{
    if (s->stream != WR_STREAM_STOPPING) {
        return;
    }
    stop_stream_locally(s, true);
    warn_now(s, WR_WARN_STREAM_STOP_TIMEOUT, 0, 0.0);
}

static void due_cal_device(wr_session *s)
{
    /*
     * The device has not answered — a marker's `a2 01` reply, or the `0x94`
     * result.  ⚠ Which of the two is in `previous_phase` on the event, so one
     * bound serves both halves of the same round trip without the abort reason
     * having to carry the distinction.
     *
     * §8.2 documents a reply to every marker and both captures show one; a
     * marker that goes unanswered is therefore a failure rather than a slow
     * device, and leaving it unbounded would strand a UI on a user holding
     * their arm out with no way back except an explicit abort.
     */
    cal_abort(s, WR_CAL_ABORT_NO_RESULT);
}

static void due_cal_raise(wr_session *s)
{
    if (s->cal_phase != WR_CALP_OBSERVING_RAISE) {
        return;
    }
    /* ⚠ CLIENT POLICY, NOT A DEVICE CONSTRAINT.  §8.2: one attempt took 15.6 s
     * between the markers and the device returned a result and applied it. */
    cal_abort(s, WR_CAL_ABORT_RAISE_TOO_SLOW);
}

static void due_cal_presence(wr_session *s)
{
    if (!s->cal_run_active) {
        return;
    }
    /* The window closed.  Measure what arrived, or say it could not be
     * measured — cal_finish_presence() decides on the count. */
    cal_finish_presence(s);
}

static void due_clock_event(wr_session *s)
{
    wr_event ev;
    bool     degraded;

    if (s->fit.n <= 0) {
        return; /* no observations: nothing to report, and nothing to re-arm for */
    }

    wr_fit_snapshot(&s->fit, s->now_us, &s->snap);
    s->snap_valid = true;

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_CLOCK_UPDATED;
    ev.u.clock = s->snap;
    event_push(s, &ev);

    /*
     * ⚠ Residual spread is a LINK-HEALTH signal (§10).  BLE at range delays
     * notifications and nothing in the protocol reports it, so the fit degrades
     * quietly while every frame still parses.  Edge-triggered: the alarm is
     * about crossing, not about staying.
     */
    degraded = (s->snap.residual_p90_us > s->policy.residual_alarm_us) ||
               ((s->snap.flags & (uint32_t)(WR_CLOCK_DEGENERATE | WR_CLOCK_RATE_IMPLAUSIBLE)) !=
                0u);
    if (degraded && !s->clock_degraded) {
        wr_event dev;
        memset(&dev, 0, sizeof(dev));
        dev.type = (uint16_t)WR_EV_CLOCK_DEGRADED;
        dev.u.clock = s->snap;
        event_push(s, &dev);
    }
    s->clock_degraded = degraded;

    arm(s, WR_DUE_CLOCK_EVENT, s->now_us + s->policy.clock_event_period_us);
}

static void due_keepalive_alarm(wr_session *s)
{
    wr_time_us since = s->now_us - s->last_host_write_us;

    if (s->link != WR_LINK_READY) {
        return;
    }
    /*
     * ⚠ The device's own idle shutdown is 300 s and the poll is 30 s, so 120 s
     * without a write means several polls have gone missing and the deadline is
     * genuinely in view while there is still time to act.  The usual cause is a
     * host that queues but never drains poll_writes().
     */
    if (since >= s->policy.keepalive_alarm_us) {
        warn_now(s, WR_WARN_KEEPALIVE_LATE, 0, (double)since);
        arm(s, WR_DUE_KEEPALIVE_ALARM, s->now_us + s->policy.keepalive_alarm_us);
        return;
    }
    arm(s, WR_DUE_KEEPALIVE_ALARM, s->last_host_write_us + s->policy.keepalive_alarm_us);
}

static void due_live_gap(wr_session *s)
{
    wr_time_us since;

    /* ⚠ Suppressed inside a bracket, where live delivery is legitimately
     * suspended for the width of the pull (§10.1).  Warning there would cry
     * wolf on every gather. */
    if (s->stream != WR_STREAM_RUNNING || s->bracket_open || !s->have_live) {
        return;
    }
    since = s->now_us - s->last_live_us;
    if (since >= s->policy.live_gap_alarm_us) {
        warn_now(s, WR_WARN_LIVE_GAP, 0, (double)since);
        arm(s, WR_DUE_LIVE_GAP, s->now_us + s->policy.live_gap_alarm_us);
        return;
    }
    arm(s, WR_DUE_LIVE_GAP, s->last_live_us + s->policy.live_gap_alarm_us);
}

static void due_pinned_report(wr_session *s)
{
    wr_event ev;

    if (s->pinned.total == 0u) {
        return; /* left disarmed until the next pinned sample re-arms it */
    }
    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_PINNED_SAMPLES;
    ev.u.pinned.counts = s->pinned;
    /* ⚠ A count without the span it covers is an estimate without evidence. */
    ev.u.pinned.over_samples = s->pinned_span;
    event_push(s, &ev);

    wr_pinned_counts_reset(&s->pinned);
    s->pinned_span = 0u;
}

static void due_power_off_linger(wr_session *s)
{
    /*
     * §9.3: `fa` is never acknowledged and the link stays up for ~9 s.  The
     * session expects that gap and does not read it as failure.  If the host
     * has still not reported the disconnect by WR_POWER_OFF_LINGER_US, the
     * session declares the link gone itself — with DO_NOT_RETRY, because the
     * device now needs a physical button press.
     */
    go_link_down(s, WR_LINK_DOWN_LOCAL_REQUEST);
}

static void due_bracket_limit(wr_session *s)
{
    /*
     * A bracket still open at twice the buffer depth is a marker whose end we
     * will never see.  It is holding the keepalive, and the device dies at
     * 5.0 minutes of silence looking exactly like a radio fault, so the bracket
     * is force-closed rather than trusted.
     */
    if (!s->bracket_open) {
        /* ⚠ The other half of the same bound: a request we abandoned between
         * the `a1` going out and its `a1 02` arriving.  The reply may still be
         * coming and no second `a1` may go out until we stop expecting it —
         * otherwise the two replies arrive inside one bracket and are merged
         * into one request's block.  Giving up on it re-opens the queue. */
        if (s->replay_pending) {
            s->replay_pending = false;
            history_service(s);
        }
        return;
    }
    close_bracket(s);
}

static void due_history_start(wr_session *s)
{
    /* A reservation's window has closed: its last sample can now exist, so the
     * pull can start (api-request C1).  history_service() decides whether this
     * one is next; history_arm() recomputes the aggregate either way. */
    history_service(s);
    history_arm(s);
}

static void due_history_deadline(wr_session *s)
{
    /*
     * ⚠ EVERY overdue request, not just the earliest.  The aggregate row is a
     * minimum over the table, so leaving a second overdue request behind would
     * re-arm the row in the PAST — which arm() would clamp and count, and which
     * spins the host's loop until it clears.
     *
     * §8.2: a block is produced for every terminal outcome, including this one.
     * A pull that half-delivers forever is worse than one that fails (B7).
     */
    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        wr_request *r = &s->reqs[i];
        if (r->state == WR_GATHER_FREE || r->state == WR_GATHER_READY) {
            continue;
        }
        if (r->req.deadline_us <= s->now_us) {
            history_abandon(s, r, WR_HIST_TIMED_OUT);
        }
    }
    history_arm(s);
    history_service(s);
}

static void due_history_eviction(wr_session *s)
{
    /*
     * §8.6: a queued request's earliest sample is estimated to leave the buffer
     * before its turn comes.  ⚠ The row is a pure WARNING and cancels nothing —
     * the data may well still be there, because the estimate rests on a depth
     * bracket that is an order of magnitude rather than a contract (§7.3).  The
     * failure it exists to prevent is the silent one: a holed set for the second
     * shot with nothing anywhere saying why.
     *
     * history_arm() runs the estimate and owns this row, so a host that
     * oversleeps it gets one event per request and never a queue of them —
     * the risk does not become newer by being restated.
     */
    history_arm(s);
}

typedef struct wr_due_row {
    const char *name;
    void (*fire)(wr_session *);
} wr_due_row;

static const wr_due_row k_due[WR_DUE_COUNT] = {
    { "keepalive", due_keepalive },
    { "bringup", due_bringup },
    { "stream_start", due_stream_start },
    { "stream_stop", due_stream_stop },
    { "cal_device", due_cal_device },
    { "cal_raise", due_cal_raise },
    { "cal_presence", due_cal_presence },
    { "clock_event", due_clock_event },
    { "keepalive_alarm", due_keepalive_alarm },
    { "live_gap", due_live_gap },
    { "pinned_report", due_pinned_report },
    { "power_off_linger", due_power_off_linger },
    { "bracket_limit", due_bracket_limit },
    { "history_start", due_history_start },
    { "history_deadline", due_history_deadline },
    { "history_eviction", due_history_eviction },
};

_Static_assert(sizeof(k_due) / sizeof(k_due[0]) == (size_t)WR_DUE_COUNT,
               "every wr_due_id needs a row: a missing one must not be a silent no-op");

/* ------------------------------------------------------------------------ */
/* The live path                                                             */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ ONE STAMP.  `calibration`, `stream_id` and `config_bits` are all "state at
 * the instant of capture", and a sample labelled CALIBRATED that predates the
 * transform is permanent and invisible.  They are written here and nowhere
 * else (implementation-notes §6.2).
 */
/*
 * `fit` is NULL when this sample cannot be dated at all — no fit yet, or, for a
 * history record, a fit that belongs to a different stretch of the piecewise
 * mapping (§6.1.1).  Undated is a real answer here: the record still carries its
 * index and both units' device time, which are derived from the frame alone.
 */
static void stamp_with(wr_session *s, wr_sample *sample, const wr_clock_snapshot *fit,
                       wr_time_us host_recv_us, wr_time_us captured_at_us,
                       wr_sample_source source)
{
    sample->stream_id = s->stream_id;
    sample->source = (uint8_t)source;
    sample->config_bits = s->active_cfg.bits;
    sample->host_recv_us = host_recv_us;

    if (fit != NULL && (fit->flags & (uint32_t)WR_CLOCK_HAS_FIT) != 0u &&
        (sample->flags & (uint16_t)WR_SAMPLE_INDEX_MISSING) == 0u) {
        wr_clock_error err = wr_clock_error_at(fit, sample->sample_index);
        sample->host_time_us = wr_clock_to_host_us(fit, sample->sample_index);
        sample->precision_us = err.precision_us;
        sample->uncertainty_us = err.total_us;
        if (sample->sample_index > fit->last_index || sample->sample_index < fit->first_index) {
            sample->flags |= (uint16_t)WR_SAMPLE_HOST_TIME_EXTRAPOLATED;
        }
    } else {
        sample->host_time_us = WR_TIME_UNKNOWN;
        sample->precision_us = UINT32_MAX;
        sample->uncertainty_us = UINT32_MAX;
        sample->flags |= (uint16_t)WR_SAMPLE_NO_FIT;
    }

    /*
     * ⚠ `captured_at_us` IS THE INSTANT THE DEVICE TOOK THIS SAMPLE, and the
     * caller states it because only the caller knows (implementation-review I3).
     * A live sample gives its ARRIVAL — real, measured, and the honest answer
     * for something that reached us microseconds after it was taken.  A history
     * sample gives its mapped host time, which is the only answer there is for a
     * record the device took seconds ago.
     */
    sample->calibration = (uint8_t)cal_state_at(s, captured_at_us);
}

static void stamp(wr_session *s, wr_sample *sample, wr_time_us host_recv_us,
                  wr_sample_source source)
{
    stamp_with(s, sample, s->snap_valid ? &s->snap : NULL, host_recv_us, host_recv_us, source);
}

/*
 * Everything derivable from the frame ALONE — the unwrapped index, each unit's
 * device time, and the inter-unit skew.  ⚠ None of it depends on the clock fit
 * (clock.h): analysis anchored on device time never waits for a host mapping.
 */
static void derive_from_frame(wr_session *s, wr_sample *sample)
{
    bool     suspect = false;
    uint32_t unwrapped;

    if ((sample->flags & (uint16_t)WR_SAMPLE_INDEX_MISSING) != 0u) {
        /* Legacy 0x7f: no record header, so there is nothing to unwrap and
         * nothing the fit could anchor on (§6.3.1). */
        sample->sample_index = 0u;
        return;
    }

    unwrapped = wr_index_unwrap(&s->idx, sample->sample_index_raw, &suspect);
    sample->sample_index = unwrapped;
    if (suspect) {
        /* A step above WR_INDEX_REGRESSION_LIMIT is almost certainly a
         * reordered or corrupt frame rather than a 60 s gap.  Counted and
         * reported; the sample is still delivered so a consumer can see it. */
        warn_rate(s, WR_WARN_INDEX_REGRESSION);
    }
    if (!s->have_index) {
        s->have_index = true;
        s->first_index = unwrapped;
    }
    s->head_index = unwrapped;

    {
        wr_unit_sample *unit[WR_UNIT_COUNT];
        unit[WR_UNIT_LOWER_ARM] = &sample->lower_arm;
        unit[WR_UNIT_PALM] = &sample->palm;

        for (int u = 0; u < (int)WR_UNIT_COUNT; ++u) {
            uint32_t margin = 0u;
            int64_t  ticks;
            if (!unit[u]->has_ticks) {
                unit[u]->device_time_us = WR_TIME_UNKNOWN;
                continue;
            }
            ticks = wr_tick_unwrap(&s->tick[u], unwrapped, unit[u]->ticks_raw, &margin);
            /*
             * ⚠ The tick rate comes from the FITTED ratio and the NOMINAL sample
             * rate, never from the clock fit — that is what makes device_time_us
             * identical for live and history and available from the first frame
             * (§10.2).  The residual uncertainty is the ~400 ppm the rate itself
             * moves by, which is three orders below the wrap budget it feeds.
             */
            unit[u]->device_time_us =
                wr_ticks_to_us(ticks, wr_tick_rate_hz(&s->tick[u], WR_NOMINAL_SAMPLE_RATE_HZ));
            if (margin < TICK_MARGIN_ALARM && !s->tick_margin_warned) {
                s->tick_margin_warned = true; /* once per stream: it is a property
                                               * of the gap, not of the record */
                warn_now(s, WR_WARN_TICK_PREDICTION_MARGIN, (int32_t)margin, 0.0);
            }
        }

        /*
         * ⚠ palm − lower_arm, for THIS record.  §10.3 measures a stable 59
         * ticks (0.92 ms) whose physical meaning is unresolved and CANNOT be
         * settled by a shared impulse — a tap is shorter than the 1.25 ms
         * sample period.  It is carried explicitly rather than silently pairing
         * the two blocks as simultaneous: at 1,000 °/s it is worth ~0.9° in the
         * relative angle, which is the primary output.
         *
         * ⚠ ONE RECORD'S VALUE IS NOT THE SKEW — pairing jitter dominates it,
         * and 59 is a session median, not a per-record constant.  The raw
         * measurement is what travels; aggregating it is the consumer's, and
         * wr_sample.skew_us says so.
         */
        if (sample->lower_arm.has_ticks && sample->palm.has_ticks) {
            int32_t skew_ticks = wr_tick_skew(sample->palm.ticks_raw, sample->lower_arm.ticks_raw);
            double  rate = wr_tick_rate_hz(&s->tick[WR_UNIT_LOWER_ARM], WR_NOMINAL_SAMPLE_RATE_HZ);
            sample->skew_us = (int32_t)wr_ticks_to_us(skew_ticks, rate);
        }
    }
}

static void live_deliver(wr_session *s, wr_sample *sample, wr_time_us host_recv_us)
{
    stamp(s, sample, host_recv_us, WR_SOURCE_LIVE);

    if ((sample->flags & (uint16_t)WR_SAMPLE_INDEX_MISSING) == 0u) {
        wr_overlap_note_live(&s->overlap, sample->sample_index, wr_sample_raw_digest(sample));
    }

    /* ⚠ The presence run is fed HERE, from the live path, and from nowhere
     * else: §7.5 measures the reference-pose angle from live samples, and the
     * bracket discriminator in on_frame() is what keeps a retrieval's records
     * out of it.  The sample is already stamped, so the run carries the same
     * calibration state and stream id the consumer sees. */
    cal_run_note_sample(s, sample);

    if ((sample->flags & (uint16_t)WR_SAMPLE_PINNED) != 0u) {
        wr_pinned_counts_add(&s->pinned, sample);
        if (s->due[WR_DUE_PINNED_REPORT] == WR_TIME_NEVER) {
            arm(s, WR_DUE_PINNED_REPORT, s->now_us + s->policy.pinned_report_period_us);
        }
    }
    s->pinned_span++;

    (void)ring_push(&s->live, sample);
}

/* ------------------------------------------------------------------------ */
/* History — the gather                                                      */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THREE MEASURED FACTS SHAPE EVERYTHING BELOW.
 *
 * 1. LIVE AND HISTORY FRAMES ARE BYTE-IDENTICAL (§10.1).  The bracket is the
 *    only discriminator and it is protocol state, so one line in on_frame()
 *    decides it and nothing else in this file repeats the test.
 * 2. THE BUFFER IS MOTION-ADAPTIVE (§7.3).  Index step 8 (≈100 Hz) at rest and
 *    step 1 (≈799 Hz) in fast motion, measured across 25 retrievals and 17,739
 *    steps with no step above 8 and none at 0.  EVERY reply is holed and the
 *    holes are not an error — so coverage is reported as intervals and density,
 *    a refill only chases gaps the motion cannot explain, and a pull over a
 *    still wrist is indistinguishable from a broken full-rate path at a desk.
 * 3. THE SAMPLE COUNTER STALLS FOR THE PULL'S OWN DURATION (§7.5).  So the
 *    index→host mapping is PIECEWISE (§6.1.1): the fit re-anchors at every
 *    bracket close, a request is dated by the snapshot taken when its window
 *    closed, and no second `a1` goes out before a live frame has landed.
 */
static int slot_of(const wr_session *s, const wr_request *r)
{
    return (int)(r - s->reqs);
}

static uint64_t index_width(wr_index_range r)
{
    return (r.first > r.last) ? 0u : ((uint64_t)r.last - (uint64_t)r.first + 1u);
}

static bool history_outstanding(const wr_session *s)
{
    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        if (s->reqs[i].state != WR_GATHER_FREE && s->reqs[i].state != WR_GATHER_READY) {
            return true;
        }
    }
    return false;
}

/*
 * ⚠ HISTORY NEVER TOUCHES THE LIVE INDEX UNWRAPPER.  History indices run BEHIND
 * live and would read there as a near-full-modulus forward step, taking
 * head_index — and therefore every later request's range — with them.  The
 * unwrap is stateless from the requested segment's base instead.
 */
static uint32_t history_unwrap(uint32_t base, uint16_t raw)
{
    return base + (uint32_t)(uint16_t)(raw - (uint16_t)base);
}

/*
 * ⚠ §8.3's WRAP SPLIT, AND IT IS A CASE A NAIVE IMPLEMENTATION GETS WRONG
 * EXACTLY ONCE EVERY 82 SECONDS.
 *
 * `a1` takes two u16be and §7.1 requires `first < last`, so ONE command can
 * only address indices inside one turn of the 65,536 counter.  §7.4 states the
 * rule as "unwrap internally, re-wrap when asking"; this is what that means in
 * practice — the caller's unwrapped range is cut at the modulus and the two
 * halves are asked for separately, then merged by unwrapped index.
 *
 * Returns the near half in `*head` and, when the range crossed, the far half in
 * `*rest`.
 *
 * ⚠ A half of ONE INDEX CANNOT BE ASKED FOR AT ALL — §7.1's `first < last` has
 * no encoding for it — so it is dropped and becomes an ordinary undelivered gap
 * on the block.  That is at most one sample at the seam of a 4.5 s window, and
 * reporting it as a gap is the truthful shape: silently widening the ask to
 * make it addressable would fetch an index the caller never asked for, and the
 * out-of-range filter in gather_record() would then throw it away and warn.
 */
static bool split_at_wrap(wr_index_range want, wr_index_range *head, wr_index_range *rest)
{
    uint32_t boundary;

    if ((want.first >> 16) == (want.last >> 16)) {
        *head = want;
        return false;
    }
    boundary = (want.first | 0xffffu); /* the last index of first's own turn */
    head->first = want.first;
    head->last = boundary;
    rest->first = boundary + 1u;
    rest->last = want.last;
    return true;
}

/*
 * Everything derivable from a history record alone, using the request's OWN
 * copy of the tick unwrappers (taken at `a1 02`).  ⚠ None of it involves a host
 * clock, which is why device time is identical for live and history (§10.2) —
 * and why a block whose host mapping was refused is still usable for analysis
 * anchored on device time.
 */
static void derive_history(wr_session *s, wr_request *r, wr_sample *sample)
{
    wr_unit_sample *unit[WR_UNIT_COUNT];

    unit[WR_UNIT_LOWER_ARM] = &sample->lower_arm;
    unit[WR_UNIT_PALM] = &sample->palm;

    for (int u = 0; u < (int)WR_UNIT_COUNT; ++u) {
        uint32_t margin = 0u;
        int64_t  ticks;
        if (!unit[u]->has_ticks) {
            unit[u]->device_time_us = WR_TIME_UNKNOWN;
            continue;
        }
        ticks = wr_tick_unwrap(&r->tick[u], sample->sample_index, unit[u]->ticks_raw, &margin);
        unit[u]->device_time_us =
            wr_ticks_to_us(ticks, wr_tick_rate_hz(&r->tick[u], WR_NOMINAL_SAMPLE_RATE_HZ));
        if (margin < TICK_MARGIN_ALARM) {
            /* Throttled rather than once-per-stream: a bulk replay can produce
             * thousands of these and the count is the interesting part. */
            warn_rate(s, WR_WARN_TICK_PREDICTION_MARGIN);
        }
    }

    if (sample->lower_arm.has_ticks && sample->palm.has_ticks) {
        int32_t skew_ticks = wr_tick_skew(sample->palm.ticks_raw, sample->lower_arm.ticks_raw);
        double  rate = wr_tick_rate_hz(&r->tick[WR_UNIT_LOWER_ARM], WR_NOMINAL_SAMPLE_RATE_HZ);
        sample->skew_us = (int32_t)wr_ticks_to_us(skew_ticks, rate);
    }
}

/*
 * ⚠ §6.1.1 IN ONE PREDICATE.  This fit was taken when the request's window
 * closed, so it speaks for the stretch that window lives in.  A record OLDER
 * than the fit's first observation belongs to an earlier stretch — and the only
 * thing that starts a new stretch inside a stream is a pull, whose stall this
 * fit knows nothing about.  Dating it here would be wrong by the width of that
 * stall (289 ms mean, §7.5) with nothing to say so.
 *
 * So it is delivered UNDATED and clearly marked, never dated wrong: the shape
 * history.h prescribes is "we could not date these, and here they are anyway".
 * The index and both units' device_time_us are unaffected.
 */
static void stamp_history(wr_session *s, wr_request *r, wr_sample *sample)
{
    bool       datable = (r->fit.flags & (uint32_t)WR_CLOCK_HAS_FIT) != 0u &&
                   sample->sample_index >= r->fit.first_index;
    /*
     * ⚠ WHEN THE DEVICE TOOK IT, for the calibration stamp (I3).  Through this
     * request's own fit, which §6.1.1 froze when the window closed — so it is
     * the mapping that belongs to the stretch these indices live in.
     *
     * An UNDATED record still needs an answer, and the window's START is the one
     * that under-claims: the earliest instant any sample in this request could
     * have been captured, so a transition inside the window resolves to the
     * state that preceded it.  A sample labelled CALIBRATED that predates the
     * transform is permanent and invisible (sample.h); one labelled
     * UNCALIBRATED that was in fact transformed is contradicted in the open by
     * the block's own `wr_calibration_span`.
     */
    wr_time_us captured_at = datable ? wr_clock_to_host_us(&r->fit, sample->sample_index)
                                     : r->req.window.start_us;
    stamp_with(s, sample, datable ? &r->fit : NULL, WR_TIME_UNKNOWN, captured_at,
               WR_SOURCE_HISTORY);
}

static void history_progress_event(wr_session *s, const wr_request *r, wr_event_type type)
{
    wr_event ev;

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)type;
    ev.u.history_progress.request_id = r->id;
    ev.u.history_progress.records_delivered = (uint32_t)r->sample_count;
    /* ⚠ The count travels with the fraction, always: coverage alone cannot
     * tell "dense over half the range" from "half-dense over all of it". */
    ev.u.history_progress.fraction =
        r->have_indices ? (float)wr_coverage_fraction(&r->cov, r->indices) : 0.0f;
    ev.u.history_progress.elapsed_us = s->now_us - r->reserved_us;
    event_push(s, &ev);
}

/*
 * ONE RECORD, INSIDE A BRACKET.  Reached from the single discriminator line in
 * on_frame() and from nowhere else.
 */
static void gather_record(wr_session *s, wr_sample *sample)
{
    wr_request *r = NULL;
    uint32_t    idx;

    if (s->bracket_owned && s->active >= 0 &&
        s->reqs[s->active].state == WR_GATHER_BRACKET) {
        r = &s->reqs[s->active];
    }
    if (r == NULL || (sample->flags & (uint16_t)WR_SAMPLE_INDEX_MISSING) != 0u) {
        /*
         * ⚠ NOBODY IS WAITING FOR THIS RECORD.  Either the bracket is an orphan
         * — a marker the session never asked for, or one answering a request
         * abandoned at its deadline, leaving the device replaying ~4,000 records
         * into nothing — or the record carries no index to place it by.  Counted
         * and discarded either way.  They must not reach wr_fit_observe() and
         * they must not reach the consumer's live ring, where a bulk arrival
         * masquerades as a burst of real motion.
         */
        s->orphan_records++;
        return;
    }

    idx = history_unwrap(r->indices.first, sample->sample_index_raw);
    if (idx < r->indices.first || idx > r->indices.last) {
        /* ⚠ §10.1 measured 4,182 mid-stream records with every one inside the
         * requested range, so this is a finding rather than a routine case.
         * Reported once per bracket with its count — one event per record would
         * evict a 256-entry ring in 320 ms. */
        r->out_of_range++;
        return;
    }
    /*
     * Dedup within the attempt.  Records arrive ascending inside a bracket, so
     * this is one comparison — and it deliberately does NOT consult the
     * coverage set, which becomes an optimistic SUPERSET once its storage
     * overflows and would then start dropping records that were never sent.
     * Duplicates ACROSS attempts die in the merge, by device index (AR C3).
     */
    if (r->run_have_index && idx <= r->run_last_index) {
        r->duplicates++;
        return;
    }
    if (r->sample_count >= s->gather_cap) {
        /* Unreachable by construction — reserve() refuses a window wider than
         * this area and the range filter above bounds the rest — so it is a
         * counted drop rather than a warning code nothing can produce. */
        r->overflowed++;
        return;
    }

    sample->sample_index = idx;
    derive_history(s, r, sample);
    stamp_history(s, r, sample);

    if ((sample->flags & (uint16_t)WR_SAMPLE_PINNED) != 0u) {
        wr_pinned_counts_add(&r->pinned, sample);
    }
    /*
     * ⚠ THE LIVE-VS-HISTORY AGREEMENT, measured on every pull.  A mid-stream
     * `a1` covers a span live already delivered, so this library is the only
     * layer that ever holds both halves — and a stitched lane depends on the
     * two agreeing index for index, value for value (§8.8).  ABSENT is NO
     * EVIDENCE, which is why the block carries the sample count beside the
     * mismatch count.
     */
    switch (wr_overlap_check(&s->overlap, idx, wr_sample_raw_digest(sample))) {
        case WR_OVERLAP_AGREES:
            r->overlap_samples++;
            break;
        case WR_OVERLAP_DIFFERS:
            r->overlap_samples++;
            r->overlap_mismatches++;
            break;
        case WR_OVERLAP_ABSENT:
        default:
            break;
    }

    (void)wr_coverage_add(&r->cov, idx);
    s->gather[r->sample_count] = *sample;
    r->sample_count++;
    r->run[r->run_count].count++;
    r->run_last_index = idx;
    r->run_have_index = true;

    if ((r->sample_count % HISTORY_PROGRESS_EVERY) == 0u) {
        history_progress_event(s, r, WR_EV_HISTORY_PROGRESS);
    }
}

static void gather_close_run(wr_request *r)
{
    if (!r->run_open) {
        return;
    }
    r->run_open = false;
    if (r->run[r->run_count].count > 0u) {
        r->run_count++;
    }
}

/* Merge the per-attempt runs into one ascending, strictly monotonic, duplicate-
 * free array.  ⚠ The key is the DEVICE INDEX, never the timestamp: a consumer
 * could only dedup by the derived one, and a duplicate or an inversion there is
 * a silently wrong interpolation rather than an error (AR C3). */
static size_t merge_runs(const wr_sample *src, const wr_gather_run *run, size_t run_count,
                         wr_sample *dst, size_t dst_cap)
{
    size_t cursor[HISTORY_MAX_ATTEMPTS];
    size_t out = 0;

    for (size_t i = 0; i < run_count; ++i) {
        cursor[i] = run[i].start;
    }
    for (;;) {
        size_t   best = run_count;
        uint32_t best_index = 0u;

        for (size_t i = 0; i < run_count; ++i) {
            uint32_t idx;
            if (cursor[i] >= run[i].start + run[i].count) {
                continue;
            }
            idx = src[cursor[i]].sample_index;
            if (best == run_count || idx < best_index) {
                best = i;
                best_index = idx;
            }
        }
        if (best == run_count) {
            break;
        }
        if (out > 0u && dst[out - 1u].sample_index == best_index) {
            cursor[best]++; /* the same index from two attempts */
            continue;
        }
        if (out >= dst_cap) {
            break;
        }
        dst[out++] = src[cursor[best]++];
    }
    return out;
}

/* Host time of one index under the block's own fit, or WR_TIME_UNKNOWN where
 * that fit cannot speak for it (§6.1.1). */
static wr_time_us block_time_at(const wr_request *r, uint32_t index)
{
    if ((r->fit.flags & (uint32_t)WR_CLOCK_HAS_FIT) == 0u || index < r->fit.first_index) {
        return WR_TIME_UNKNOWN;
    }
    return wr_clock_to_host_us(&r->fit, index);
}

/* ⚠ wr_index_range is INCLUSIVE and wr_time_range is HALF-OPEN, so the end
 * carries one sample period.  This is where that off-by-one would be born. */
static wr_time_range block_span(const wr_request *r, uint32_t first, uint32_t last)
{
    wr_time_range t;

    t.start_us = block_time_at(r, first);
    t.end_us = block_time_at(r, last);
    if (t.start_us == WR_TIME_UNKNOWN || t.end_us == WR_TIME_UNKNOWN) {
        t.start_us = WR_TIME_UNKNOWN;
        t.end_us = WR_TIME_UNKNOWN;
        return t;
    }
    t.end_us += (wr_time_us)llround(r->fit.slope_us_per_index);
    return t;
}

/*
 * ⚠ COMPLETE / SHORT / HOLED IS ABOUT THE SHAPE OF THE DELIVERED SET, not about
 * whether anything went wrong.  §7.3: at 16-50 % coverage in a typical session,
 * HOLED is the NORMAL answer and the interval list is what a consumer reads.
 * One contiguous interval that stops early is SHORT — which is the shape that
 * says something about buffer depth — and more than one is HOLED.
 */
static wr_history_status gather_status(const wr_request *r)
{
    if (!r->have_indices) {
        return WR_HIST_SHORT;
    }
    if (wr_coverage_indices_in(&r->cov, r->indices) >= index_width(r->indices)) {
        return WR_HIST_COMPLETE;
    }
    return (r->cov.count <= 1u) ? WR_HIST_SHORT : WR_HIST_HOLED;
}

/*
 * ⚠ THE ONE PLACE WR_CAL_LOST IS EVER WRITTEN (sample.h, history.h, design
 * §7.3).  No sample carries it: none can be captured between the loss and the
 * notice of one.  Here it does work UNCALIBRATED cannot — these samples WERE
 * taken under a transform, and by the time the block reached the caller that
 * transform was gone, so it can no longer be verified or reproduced.  That is a
 * different claim from "there never was one".
 */
static void fill_calibration_span(const wr_session *s, wr_time_range span,
                                  wr_calibration_span *out)
{
    /* Only one transition back is retained, and prev != current is exactly the
     * condition that says one has happened at all. */
    bool have_transition = (s->cal_prev_state != s->cal_state);

    memset(out, 0, sizeof(*out));
    out->presence_angle_deg = s->cal_have_anchor ? s->cal_anchor.relative_angle_deg : NAN;
    out->state_at_start = (uint8_t)s->cal_state;
    out->state_at_end = (uint8_t)s->cal_state;

    if (!have_transition || span.start_us == WR_TIME_UNKNOWN) {
        return;
    }
    if (s->cal_changed_us >= span.start_us && s->cal_changed_us < span.end_us) {
        /* A calibration or a reconnect happened INSIDE the block's own span. */
        out->state_at_start = (uint8_t)s->cal_prev_state;
        out->state_at_end = (uint8_t)s->cal_state;
        out->spans_transition = 1u;
        return;
    }
    if (s->cal_changed_us >= span.end_us) {
        /* The change is later than every sample here, so what these samples
         * were captured under is the PREVIOUS state. */
        out->state_at_start = (uint8_t)s->cal_prev_state;
        out->state_at_end = (uint8_t)s->cal_prev_state;
        if (s->cal_prev_state == WR_CAL_CALIBRATED && s->cal_state != WR_CAL_CALIBRATED) {
            out->state_at_end = (uint8_t)WR_CAL_LOST;
            out->spans_transition = 1u;
        }
    }
}

/*
 * §8.5's DEPTH BRACKET, LEARNED FROM ONE RETRIEVAL.
 *
 * ⚠⚠ THE EVIDENCE IS THE OLD END OF THE DELIVERED SET, NOT THE STATUS ENUM, AND
 * THAT IS THE WHOLE OF THIS FUNCTION.  §8.5 as written says "the widest span
 * that came back COMPLETE" and "the narrowest that came back holed at the old
 * end" — but §7.3 was rewritten from hardware after that was drafted, and it
 * makes EVERY reply holed: the buffer is motion-adaptive, so a still wrist
 * returns an even one-in-eight.  Replaying `swings.wrwire` through this gather
 * produced six blocks and ZERO WR_HIST_COMPLETE.  A rule keyed on the status
 * would therefore learn nothing on real hardware and leave both queries
 * reporting WR_HISTORY_DEPTH_SEED_US — a figure measured once, on somebody
 * else's session — dressed as a measurement of this connection.  That is
 * precisely the failure the two queries were left refusing to avoid.
 *
 * So the discriminator is §7.3's own step-8 floor, the same one that decides
 * what a refill chases and when WR_WARN_HISTORY_HOLED fires:
 *
 *   - the oldest index that ARRIVED is a verified reach-back → a LOWER bound;
 *   - the oldest index we ASKED FOR missing by more than the floor is the
 *     buffer failing to reach that far → an UPPER bound.
 *
 * ⚠ Measured in INDICES and converted with the fit's slope, never from the
 * arrival times: the counter advances at the internal rate whatever the wrist
 * is doing (§6.5), where a bulk reply's arrival times say only how fast the
 * radio drained.
 *
 * ⚠ AND NEITHER BOUND IS A MODEL OF THE BUFFER.  §7.3 leaves open whether the
 * depth is a fixed sample count or a fixed duration, and the adaptive rate
 * makes those differ by 8×.  A bracket of observations does not have to choose;
 * a constant would have to, and would be wrong half the time.
 */
static void history_learn_depth(wr_session *s, const wr_request *r, wr_history_status status)
{
    double     slope = r->fit.slope_us_per_index;
    uint32_t   oldest;
    wr_time_us bound;

    if (!r->have_head_at_ask || !r->have_indices || !r->have_recorded || !(slope > 0.0)) {
        return;
    }
    if (index_width(r->recorded) == 0u || r->head_at_ask < r->recorded.first) {
        return;
    }

    /*
     * ⚠ `d0 03` WITH NOTHING DELIVERED IS AN UPPER BOUND AND NOTHING ELSE.  The
     * code means seven different things (§7.2) and this session's own state
     * rules out six of them, which is what WR_HIST_EVICTED already asserts — so
     * being consistent here costs nothing, and if that elimination is ever
     * wrong the error is in the safe direction: it makes the library claim LESS
     * residency than it has.
     */
    if (status == WR_HIST_EVICTED && !r->cov.has_bounds) {
        bound = (wr_time_us)llround(
            (double)(r->head_at_ask - r->recorded.first) * slope);
        if (!s->have_depth_hi || bound < s->depth_hi_us) {
            s->depth_hi_us = bound;
            s->have_depth_hi = true;
        }
        return;
    }
    if (!r->cov.has_bounds) {
        return; /* nothing arrived and nothing said why: no evidence either way */
    }

    oldest = r->cov.bounds.first;
    if (oldest > r->head_at_ask) {
        return; /* defensive: an index ahead of the head cannot measure a depth */
    }
    /* The device served back to `oldest`, so the buffer held at least that far.
     * ⚠ True whatever ended the retrieval — a cancelled or timed-out pull still
     * DELIVERED what it delivered, and that is what a lower bound is. */
    bound = (wr_time_us)llround((double)(r->head_at_ask - oldest) * slope);
    if (!s->have_depth_lo || bound > s->depth_lo_us) {
        s->depth_lo_us = bound;
        s->have_depth_lo = true;
    }

    /*
     * ⚠ THE UPPER BOUND NEEDS THE RETRIEVAL TO HAVE FINISHED, and that is not
     * fussiness.  A pull we abandoned at its deadline, cancelled, or lost with
     * the link is missing its old end because WE STOPPED LISTENING — reading
     * that as eviction would teach the bracket a bound the device never
     * demonstrated, and the bracket would then shrink every time a consumer
     * cancelled a pull.
     */
    if (status != WR_HIST_COMPLETE && status != WR_HIST_HOLED && status != WR_HIST_SHORT) {
        return;
    }
    /*
     * ⚠ THE MIRROR OF THE GUARD TWENTY LINES UP, AND IT WAS MISSING
     * (implementation-review I8).
     *
     * gather_record() filters arrivals against `r->indices` — the full mapped
     * window — which is WIDER than `r->recorded` whenever the window reaches
     * back before the stream's first observed index.  One delivered record in
     * [indices.first, recorded.first) puts `oldest` below `recorded.first`, and
     * the unsigned subtraction below wraps to ≈4.29e9: it clears the step-8
     * floor trivially and falls straight through to record "the buffer
     * demonstrably failed to reach that far" from evidence saying the opposite.
     *
     * `depth_hi` only ever NARROWS, so that claim would be permanent for the
     * stream — silently shrinking wr_history_resident_range() and able to fire
     * WR_WARN_HISTORY_DEPTH_CONFLICT.  §10.1 measured 4,182 records with every
     * one inside the requested range, so this needs the device to do something
     * never observed; the consequence is permanent and the guard is one line.
     */
    if (oldest < r->recorded.first) {
        return;
    }
    if (oldest - r->recorded.first < HISTORY_ADAPTIVE_STEP_MAX) {
        return; /* the old end was reached, within what the motion can explain */
    }
    bound = (wr_time_us)llround((double)(r->head_at_ask - r->recorded.first) * slope);
    if (!s->have_depth_hi || bound < s->depth_hi_us) {
        s->depth_hi_us = bound;
        s->have_depth_hi = true;
    }
    /*
     * ⚠ AND A CONTRADICTION IS A FINDING, NOT AN ERROR TO SMOOTH OVER.  The two
     * bounds can cross, because a depth in TIME is not a constant if the buffer
     * holds a fixed sample COUNT — §7.3's open question.  Counting it is
     * how the answer gets measured instead of assumed; the queries below take
     * the pessimistic bound when it happens, which is the only claim both
     * observations support.
     */
    if (s->have_depth_lo && s->depth_hi_us <= s->depth_lo_us && !s->depth_conflict_warned) {
        s->depth_conflict_warned = true;
        warn_now(s, WR_WARN_HISTORY_DEPTH_CONFLICT,
                 (int32_t)((s->depth_lo_us - s->depth_hi_us) / 1000), 0.0);
    }
}

/*
 * Materialise.  A block is produced for EVERY terminal outcome — complete,
 * holed, short, timed out, cancelled, refused — and always carries its
 * coverage, because a capture must record what it got even when what it got is
 * nothing (design §8.2).
 */
static void history_finish(wr_session *s, wr_request *r, wr_history_status status)
{
    wr_history_record *rec;
    wr_layout          layout;
    wr_sample         *samples;
    wr_time_range     *delivered;
    wr_index_range    *delivered_idx;
    wr_gap            *gaps;
    size_t             n_delivered = r->cov.count;
    /* At most: one FIT_BLIND, one NOT_RECORDED at each end, and one
     * NOT_DELIVERED between each pair of delivered intervals plus one at each
     * end of what was asked for. */
    size_t             gap_cap = n_delivered + 5u;
    size_t             gap_count = 0u;
    wr_index_range     window = r->have_indices ? r->indices : r->recorded;
    wr_index_range     served = r->have_indices ? r->recorded : r->indices;
    void              *mem;

    if (s->active == slot_of(s, r)) {
        s->active = -1;
        s->bracket_owned = false;
    }
    gather_close_run(r);
    /* ⚠ Before the coverage set is handed back to the next request, and before
     * anything below can fail on an allocation — the measurement belongs to the
     * connection, not to the block. */
    history_learn_depth(s, r, status);
    if (!r->have_indices) {
        window.first = 0u;
        window.last = 0u;
        served = window;
    } else if (!r->have_recorded) {
        /*
         * ⚠ NOTHING WAS EVER ASKED FOR: all of the window is undelivered.
         *
         * The flag is the sentinel because a WIDTH cannot be
         * (implementation-review I7).  `index_width({0,0})` is 1, not 0 — the
         * range type is inclusive — so testing the width let a zeroed
         * `r->recorded` through as though it were the single index 0.  Both
         * refusals that reach here with `have_indices` set and `recorded`
         * untouched — the alignment budget and the post-clamp NO_STREAM — then
         * handed back a WR_GAP_NOT_DELIVERED over [0,0] and a
         * WR_GAP_NOT_RECORDED over [1, window.last], anchored below the request
         * and breaking the invariant history.h states about every gap lying
         * inside it.
         */
        served = window;
    }

    /* Pass 1: measure.  One allocation, so release() is one free. */
    layout.base = NULL;
    layout.used = 0u;
    (void)layout_take(&layout, sizeof(wr_history_record), 16u);
    (void)layout_take(&layout, r->sample_count * sizeof(wr_sample), 16u);
    (void)layout_take(&layout, n_delivered * sizeof(wr_time_range), 16u);
    (void)layout_take(&layout, n_delivered * sizeof(wr_index_range), 16u);
    (void)layout_take(&layout, gap_cap * sizeof(wr_gap), 16u);

    mem = s->alloc.alloc(s->alloc.ctx, layout.used);
    if (mem == NULL) {
        /* ⚠ Still terminal, and still reported.  collect() returns
         * WR_ERR_NO_MEMORY rather than WR_PENDING forever, because a request
         * that goes quiet is the one failure a consumer cannot act on. */
        r->alloc_failed = true;
        r->block = NULL;
        r->state = WR_GATHER_READY;
        wr_coverage_init(&r->cov, NULL, 0u);
        history_progress_event(s, r, WR_EV_HISTORY_READY);
        history_arm(s);
        history_service(s);
        return;
    }
    memset(mem, 0, layout.used);

    layout.base = (uint8_t *)mem;
    layout.used = 0u;
    rec = layout_take(&layout, sizeof(wr_history_record), 16u);
    samples = layout_take(&layout, r->sample_count * sizeof(wr_sample), 16u);
    delivered = layout_take(&layout, n_delivered * sizeof(wr_time_range), 16u);
    delivered_idx = layout_take(&layout, n_delivered * sizeof(wr_index_range), 16u);
    gaps = layout_take(&layout, gap_cap * sizeof(wr_gap), 16u);

    rec->magic = HISTORY_BLOCK_MAGIC;
    rec->alloc = s->alloc; /* ⚠ a COPY: this is what lets the block outlive us */
    rec->bytes = layout.used;

    rec->pub.layout_version = WR_SAMPLE_LAYOUT_VERSION;
    rec->pub.sample_stride = (uint32_t)sizeof(wr_sample);
    rec->pub.samples = samples;
    rec->pub.sample_count = merge_runs(s->gather, r->run, r->run_count, samples, r->sample_count);
    /* Keep the slot's count in step with the block's, so the READY event below
     * reports what the caller will actually receive rather than what arrived
     * before the cross-attempt duplicates were dropped. */
    r->sample_count = rec->pub.sample_count;
    rec->pub.status = (uint8_t)status;
    rec->pub.attempts = r->attempts;
    rec->pub.coverage_overflowed = r->cov.overflowed ? 1u : 0u;
    rec->pub.stream_id = s->stream_id;
    rec->pub.user_tag = r->req.user_tag;
    rec->pub.requested = r->req.window;
    rec->pub.requested_indices = window;

    for (size_t i = 0; i < n_delivered; ++i) {
        delivered_idx[i] = r->cov.ranges[i];
        delivered[i] = block_span(r, r->cov.ranges[i].first, r->cov.ranges[i].last);
    }
    rec->pub.delivered = delivered;
    rec->pub.delivered_indices = delivered_idx;
    rec->pub.delivered_count = n_delivered;

    /*
     * Gaps, in three kinds that may overlap, emitted in non-decreasing order of
     * first index: what this fit could not date, what the device never
     * recorded, what it recorded and did not deliver.
     */
    if (r->have_indices && (r->fit.flags & (uint32_t)WR_CLOCK_HAS_FIT) != 0u &&
        r->fit.first_index > window.first) {
        /* ⚠ §6.1.1: a pull separates these indices from the fit that would
         * date them.  The samples are here; their host time is not. */
        gaps[gap_count].indices.first = window.first;
        gaps[gap_count].indices.last =
            (r->fit.first_index - 1u < window.last) ? (r->fit.first_index - 1u) : window.last;
        gaps[gap_count].span.start_us = WR_TIME_UNKNOWN;
        gaps[gap_count].span.end_us = WR_TIME_UNKNOWN;
        gaps[gap_count].kind = (uint8_t)WR_GAP_FIT_BLIND;
        gap_count++;
    }
    if (served.first > window.first) {
        /* Before the stream's first sample: the device recorded nothing. */
        gaps[gap_count].indices.first = window.first;
        gaps[gap_count].indices.last = served.first - 1u;
        gaps[gap_count].span = block_span(r, window.first, served.first - 1u);
        gaps[gap_count].kind = (uint8_t)WR_GAP_NOT_RECORDED;
        gap_count++;
    }
    if (r->have_indices && index_width(served) > 0u) {
        /* The complement of the delivered set inside what we actually asked
         * for.  ⚠ §7.3: most of these are the motion-adaptive floor and are the
         * buffer working correctly, which is why they are reported as data
         * rather than raised as an alarm.
         *
         * ⚠ And nothing at all when the window never reached an index range —
         * a legacy stream, or a conversion that could not be made.  "One gap at
         * index 0" would be a statement about a span that was never identified. */
        uint64_t cursor = served.first;
        for (size_t i = 0; i < r->cov.count && gap_count + 1u < gap_cap; ++i) {
            uint32_t lo = r->cov.ranges[i].first;
            uint32_t hi = r->cov.ranges[i].last;
            if (hi < served.first) {
                continue;
            }
            if (lo > served.last) {
                break;
            }
            if ((uint64_t)lo > cursor) {
                gaps[gap_count].indices.first = (uint32_t)cursor;
                gaps[gap_count].indices.last = lo - 1u;
                gaps[gap_count].span = block_span(r, (uint32_t)cursor, lo - 1u);
                gaps[gap_count].kind = (uint8_t)WR_GAP_NOT_DELIVERED;
                gap_count++;
            }
            if ((uint64_t)hi + 1u > cursor) {
                cursor = (uint64_t)hi + 1u;
            }
        }
        if (cursor <= (uint64_t)served.last && gap_count + 1u < gap_cap) {
            gaps[gap_count].indices.first = (uint32_t)cursor;
            gaps[gap_count].indices.last = served.last;
            gaps[gap_count].span = block_span(r, (uint32_t)cursor, served.last);
            gaps[gap_count].kind = (uint8_t)WR_GAP_NOT_DELIVERED;
            gap_count++;
        }
    }
    if (served.last < window.last && index_width(window) > 0u && gap_count < gap_cap) {
        /* Past the head the device had reached — never taken, not merely never
         * delivered, and the stronger of the two kinds wins. */
        gaps[gap_count].indices.first = served.last + 1u;
        gaps[gap_count].indices.last = window.last;
        gaps[gap_count].span = block_span(r, served.last + 1u, window.last);
        gaps[gap_count].kind = (uint8_t)WR_GAP_NOT_RECORDED;
        gap_count++;
    }
    rec->pub.gaps = gaps;
    rec->pub.gap_count = gap_count;

    rec->pub.coverage_fraction = r->have_indices ? wr_coverage_fraction(&r->cov, window) : 0.0;
    /* ⚠ From the MERGED SAMPLES, not from the coverage set: at §7.3's floor the
     * set is one interval per delivered index and overflow coalesces two of them
     * across their gap, which is the step this measures (history.h). */
    rec->pub.density = wr_sample_step_density(samples, rec->pub.sample_count);
    {
        uint32_t largest = r->have_indices ? wr_coverage_largest_gap(&r->cov, window) : 0u;
        double   us = (double)largest * r->fit.slope_us_per_index;
        rec->pub.largest_gap_us = (us >= 4294967295.0) ? UINT32_MAX : (uint32_t)us;

        /* ⚠ Measured over what actually arrived, so a still pre-roll reads
         * ≈100 Hz and a swing reads ≈799 Hz — which is the correct answer and
         * the one §7.3's table is written in. */
        rec->pub.achieved_hz = 0.0;
        if (rec->pub.sample_count > 1u && r->cov.has_bounds &&
            r->cov.bounds.last > r->cov.bounds.first && r->fit.slope_us_per_index > 0.0) {
            double span_us = (double)((uint64_t)r->cov.bounds.last - (uint64_t)r->cov.bounds.first) *
                             r->fit.slope_us_per_index;
            rec->pub.achieved_hz = (double)(rec->pub.sample_count - 1u) * 1e6 / span_us;
        }
    }

    rec->pub.live_overlap_samples = r->overlap_samples;
    rec->pub.live_overlap_mismatches = r->overlap_mismatches;
    if (r->have_stall) {
        rec->pub.self_recording_gap = r->stall;
    }
    rec->pub.fit = r->fit;
    fill_calibration_span(s, r->req.window, &rec->pub.calibration);
    rec->pub.config = s->active_cfg;
    rec->pub.pinned = r->pinned;
    rec->pub.requested_at_us = r->reserved_us;
    rec->pub.completed_at_us = s->now_us;

    /*
     * ⚠ The warnings are NOT "this reply had gaps" — every reply has gaps
     * (§7.3).  HOLED fires only where the holing exceeds what the adaptive
     * buffer can produce; SHORT fires where the delivered set ran out before
     * the request did, which is evidence about depth.
     */
    if (status == WR_HIST_HOLED) {
        /* ⚠ Measured over what was ASKED FOR, not over the whole window.  A
         * window reaching past the head is short of samples that were never
         * TAKEN, which is a different fact and already an WR_GAP_NOT_RECORDED —
         * folding it in here would raise a delivery alarm about a span the
         * device was never asked to deliver.  `largest_gap_us` on the block
         * covers the whole window, because for "did impact survive" the two
         * kinds are equally fatal. */
        uint32_t undelivered = wr_coverage_largest_gap(&r->cov, served);
        if (undelivered >= HISTORY_ADAPTIVE_STEP_MAX) {
            warn_now(s, WR_WARN_HISTORY_HOLED, (int32_t)undelivered,
                     rec->pub.coverage_fraction);
        }
    }
    if (status == WR_HIST_SHORT && r->have_indices) {
        uint64_t missing = index_width(window) - wr_coverage_indices_in(&r->cov, window);
        if (missing > 0u) {
            warn_now(s, WR_WARN_HISTORY_SHORT,
                     (missing > (uint64_t)INT32_MAX) ? INT32_MAX : (int32_t)missing,
                     rec->pub.coverage_fraction);
        }
    }

    history_progress_event(s, r, WR_EV_HISTORY_READY);

    r->block = rec;
    r->state = WR_GATHER_READY;
    wr_coverage_init(&r->cov, NULL, 0u); /* the shared storage belongs to the next request */

    history_arm(s);
    history_service(s);
}

/*
 * Give up on a request that is still in flight.  ⚠ The BRACKET is not closed
 * here: the device does not know we have stopped listening and is still
 * replaying, so the marker has to close it.  Until then its records are orphans
 * — counted and discarded — which is the whole reason the orphan path exists.
 */
static void history_abandon(wr_session *s, wr_request *r, wr_history_status status)
{
    if (s->active == slot_of(s, r)) {
        if (r->state == WR_GATHER_REQUESTED) {
            /*
             * ⚠ The `a1` is out and its `a1 02` has not arrived.  No second one
             * may go out until we stop expecting this one, or the two replies
             * arrive inside one bracket and are merged into one request.  The
             * bracket limit is the bound on how long we wait to stop expecting.
             */
            s->replay_pending = true;
            arm(s, WR_DUE_BRACKET_LIMIT, s->now_us + BRACKET_LIMIT_US);
        }
        s->bracket_owned = false;
        s->active = -1;
    }
    history_finish(s, r, status);
}

static void history_abandon_all(wr_session *s, wr_history_status status)
{
    /* ⚠ Frozen for the duration: materialising one request must not let the
     * next one in the table issue an `a1` into a session that is going away. */
    s->history_frozen = true;
    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        wr_request *r = &s->reqs[i];
        if (r->state == WR_GATHER_FREE || r->state == WR_GATHER_READY) {
            continue;
        }
        history_abandon(s, r, status);
    }
    s->history_frozen = false;
}

static void history_attempt(wr_session *s, wr_request *r)
{
    wr_write_request w;

    if (wr_cmd_history(&w, (uint16_t)(r->ask.first & 0xffffu),
                       (uint16_t)(r->ask.last & 0xffffu)) < WR_OK) {
        /* §7.1 requires first < last, and wr_cmd_history() refuses anything
         * else — so a mis-split fails loudly here rather than returning a
         * dense, complete and entirely wrong block. */
        history_finish(s, r, WR_HIST_ERROR);
        return;
    }
    queue_write(s, &w);
    /*
     * ⚠ §8.5's depth is measured back from the head AT THE ASK, and only from
     * the FIRST one.  The counter stalls for the pull's own duration (§7.5), so
     * by the time the block materialises `head_index` belongs to the stretch
     * after the pull — and a second attempt's head has moved on by however long
     * the first one took.  What the retrieval measures is how far back the
     * buffer reached at the moment it was asked.
     */
    if (!r->have_head_at_ask && s->have_index) {
        r->head_at_ask = s->head_index;
        r->have_head_at_ask = true;
    }
    r->attempts++;
    r->state = WR_GATHER_REQUESTED;
    s->active = slot_of(s, r);
    history_arm(s);
}

static void history_issue(wr_session *s, wr_request *r)
{
    wr_index_range range;
    wr_index_range recorded;

    /* ⚠ Refusals that happen BEFORE ANY RADIO TRAFFIC (§8.3).  Refusing and
     * RECORDING the refusal is the point: a consumer must be able to write
     * "we declined this pull, and why" into a capture's provenance rather than
     * silently misaligning a wrist trace against video (AR B2). */
    if (s->stream != WR_STREAM_RUNNING || !s->have_index) {
        history_finish(s, r, WR_HIST_NO_STREAM);
        return;
    }
    if (wr_stream_config_is_legacy(s->active_cfg)) {
        /* §6.3.1: a 0x7f record has no header, so `a1` addresses a counter that
         * does not exist. */
        history_finish(s, r, WR_HIST_NOT_ALIGNABLE);
        return;
    }
    if (wr_clock_index_range_for_time(&r->fit, r->req.window, &range) < WR_OK) {
        history_finish(s, r, WR_HIST_ERROR);
        return;
    }
    r->indices = range;
    r->have_indices = true;

    if (r->req.alignment_budget_us != 0u) {
        /* ⚠ PRECISION, never the total (clock.h): the caller is asking about
         * the part of the error that varies with the link and the fit.  Gating
         * on the total would refuse every pull after one second of session. */
        if (!wr_clock_meets_budget(&r->fit, range.first, r->req.alignment_budget_us) ||
            !wr_clock_meets_budget(&r->fit, range.last, r->req.alignment_budget_us)) {
            history_finish(s, r, WR_HIST_REFUSED_ALIGNMENT);
            return;
        }
    }

    /*
     * ⚠ CLAMP THE ASK TO WHAT THE DEVICE CAN HAVE TAKEN, and report the rest
     * rather than asking for it.  The window is mapped through a fit, so its
     * last index can sit a few samples past the head the device has actually
     * reached — and §7.2 measured "no data yet" as one of seven distinct causes
     * that all return the same `d0 03`, which would cost the whole reply rather
     * than its tail.  What falls outside becomes WR_GAP_NOT_RECORDED on the
     * block, which is exactly what it is: those samples were never taken.
     */
    recorded = range;
    if (recorded.first < s->first_index) {
        recorded.first = s->first_index;
    }
    if (recorded.last > s->head_index) {
        recorded.last = s->head_index;
    }
    if (recorded.first > recorded.last) {
        history_finish(s, r, WR_HIST_NO_STREAM);
        return;
    }
    r->recorded = recorded;
    r->have_recorded = true;
    if (recorded.first == recorded.last) {
        /* §7.1 requires first < last: a single index cannot be addressed. */
        history_finish(s, r, WR_HIST_SHORT);
        return;
    }

    /*
     * ⚠⚠ THE WINDOW LIES ON THE FAR SIDE OF A PULL, AND THIS IS A STRUCTURAL
     * REFUSAL RATHER THAN A QUALITY ONE — it fires whatever
     * `alignment_budget_us` says, because what is missing is the mapping
     * itself, not confidence in it.
     *
     * §6.1.1: the fit re-anchors at every bracket close, so its first
     * observation marks the start of the stretch it can speak for.  A window
     * older than that is separated from this line by at least one stall, and
     * the error is not confined to the timestamps: `wr_clock_index_range_for_time`
     * would map the host window to indices that are TOO LOW by the width of the
     * stall (289 ms mean, §7.5 — about 230 samples), so the `a1` would ask for
     * a different span than the caller wanted and every check downstream would
     * pass on it.  A block of the wrong samples is worse than no block.
     *
     * ⚠ It is nearly unreachable by the intended cycle: reserving at detection
     * (AR C1) captures the fit when the window closes, which is before any
     * later pull re-anchors it — so §8.6's "second shot a few seconds after the
     * first" is served correctly.  This catches a window reserved AFTER a pull
     * that it predates.
     */
    if (recorded.first < r->fit.first_index) {
        history_finish(s, r, WR_HIST_REFUSED_ALIGNMENT);
        return;
    }

    /*
     * ⚠ §8.3's WRAP SPLIT.  A window spanning the 82.0 s counter wrap is issued
     * as TWO `a1`s and merged by unwrapped index; split_at_wrap() carries the
     * reasoning.  The far half waits for the near half's bracket to close, on
     * the machinery a refill already uses — which also means it obeys §6.1.1's
     * "never two pulls inside one live-frame gap" without a second rule.
     *
     * ⚠ MORE than one crossing is still refused.  Two turns of the counter is
     * 131,072 indices — 164 s against a buffer §7.3 measures in seconds — so
     * such a window cannot be served whatever we do, and WR_HIST_ERROR says we
     * will not guess rather than returning a third of it as though it were all.
     * wr_history_reserve()'s gather-area check refuses it first in every default
     * configuration; this is the backstop for a caller that supplied their own.
     */
    if ((recorded.last >> 16) - (recorded.first >> 16) > 1u) {
        history_finish(s, r, WR_HIST_ERROR);
        return;
    }

    wr_coverage_init(&r->cov, s->cov_storage, s->cov_cap);
    {
        wr_index_range head;
        wr_index_range rest;

        r->have_pending_ask = split_at_wrap(recorded, &head, &rest);
        if (r->have_pending_ask) {
            r->pending_ask = rest;
            /* ⚠ Either half may be a single index, which §7.1 cannot encode.
             * Drop that half rather than distorting the ask; the index becomes
             * an ordinary gap on the block. */
            if (index_width(rest) < 2u) {
                r->have_pending_ask = false;
            }
            if (index_width(head) < 2u) {
                if (!r->have_pending_ask) {
                    history_finish(s, r, WR_HIST_SHORT);
                    return;
                }
                head = r->pending_ask;
                r->have_pending_ask = false;
            }
        }
        r->ask = head;
    }
    history_attempt(s, r);
}

/*
 * ⚠ THE FIT IS CAPTURED FOR EVERY ELIGIBLE REQUEST, not only the one about to
 * be served, and §6.1.1 is why.  A request whose window closed BEFORE an
 * earlier request's pull must keep the mapping that was in force then; the
 * pull re-anchors the fit, and a snapshot taken after it is out by the stall.
 */
static void history_capture_fits(wr_session *s)
{
    /* ⚠ Zero-initialised for MSVC, which cannot see that `have` guarantees the
     * snapshot is taken before `snap.flags` is read (C4701, fatal under /WX).
     * The zero value is the SAFE one if the flow ever changes: flags without
     * WR_CLOCK_HAS_FIT makes the branch below return rather than proceed. */
    wr_clock_snapshot snap = {0};
    bool              have = false;

    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        wr_request *r = &s->reqs[i];
        if (r->state != WR_GATHER_QUEUED || r->have_fit) {
            continue;
        }
        if (r->req.window.end_us > s->now_us) {
            continue; /* the window's last sample does not exist yet */
        }
        if (!have) {
            wr_fit_snapshot(&s->fit, s->now_us, &snap);
            have = true;
        }
        if ((snap.flags & (uint32_t)WR_CLOCK_HAS_FIT) == 0u) {
            return; /* nothing to capture yet — try again next pass */
        }
        r->fit = snap;
        r->have_fit = true;
    }
}

/*
 * ⚠ THE DEPTH THE WARNING USES IS NOT THE DEPTH THE QUERIES REPORT, AND THE
 * ASYMMETRY IS DELIBERATE.
 *
 * This one is a best ESTIMATE and is allowed to start from §7.3's seed: it
 * feeds a warning, and a warning that fires a little early costs a consumer
 * nothing.  wr_history_resident_range() below is a CLAIM OF RESIDENCY, so it
 * refuses to speak from the seed at all — a consumer that skips a check because
 * the library said the data was there has lost the data.
 *
 * Under-claim in the query; over-warn in the alarm.  Reading the same number
 * into both is how a residency estimate quietly becomes a guarantee.
 */
static wr_time_us history_depth_estimate_us(const wr_session *s)
{
    wr_time_us depth = WR_HISTORY_DEPTH_SEED_US;

    /* Verified more than the seed — the device has actually served it. */
    if (s->have_depth_lo && s->depth_lo_us > depth) {
        depth = s->depth_lo_us;
    }
    /* Verified less than that — the device demonstrably failed to serve it. */
    if (s->have_depth_hi && s->depth_hi_us < depth) {
        depth = s->depth_hi_us;
    }
    return depth;
}

/* The outstanding requests ahead of `before_id`, in the order they will be
 * served — reservation order (§8.6).  At most HISTORY_MAX_PENDING of them. */
static size_t history_queue_ahead(const wr_session *s, uint64_t before_id,
                                  const wr_request **out)
{
    size_t n = 0u;

    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        const wr_request *q = &s->reqs[i];
        size_t            k;
        if (q->state == WR_GATHER_FREE || q->state == WR_GATHER_READY) {
            continue;
        }
        if (q->id >= before_id) {
            continue;
        }
        k = n++;
        while (k > 0u && out[k - 1u]->id > q->id) {
            out[k] = out[k - 1u];
            k--;
        }
        out[k] = q;
    }
    return n;
}

/*
 * §8.6 — SERIALISATION AND EVICTION RISK.
 *
 * ⚠ A second `a1` cannot go out until the first completes (AR B16) and a pull
 * takes about as long as its window spans (§7.4).  Against a buffer measured in
 * seconds, a second shot a few seconds after the first is an ordinary thing —
 * "a golfer hitting balls does not wait" — and part of its window can be gone
 * by the time its turn comes.  The data was there; nobody asked in time.
 * Silently returning a holed set for the second shot is the failure this warns
 * about instead of designing around afterwards.
 *
 * The schedule is exact rather than sampled.  Serving is serial and in
 * reservation order, so the instant this request's own `a1` can go out is
 *
 *     turn(t) = max(t + slack, fixed)
 *
 * where `slack` is the summed width of the windows ahead of it (§7.4's "a pull
 * costs about its own span") and `fixed` is the latest absolute instant the
 * schedule is pinned to by a window that has not closed yet.  Both are constants
 * of the current table, so the crossing instant is solved rather than polled —
 * which is what lets this be one deadline row instead of a periodic wake.
 *
 * ⚠ It cancels nothing and refuses nothing.  The estimate rests on a depth that
 * §7.3 measured only once, so acting on it would be acting on an order of magnitude.
 */
static void history_eviction_check(wr_session *s, wr_time_us *out_next)
{
    wr_time_us depth = history_depth_estimate_us(s);

    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        const wr_request *ahead[HISTORY_MAX_PENDING];
        wr_request       *r = &s->reqs[i];
        wr_time_us        slack = 0;
        wr_time_us        fixed = 0;
        wr_time_us        turn_at;
        wr_time_us        eviction_at;
        size_t            n;

        /* Only a QUEUED request can be overtaken by eviction; one in flight is
         * being served, and one that is READY has been. */
        if (r->state != WR_GATHER_QUEUED) {
            continue;
        }

        n = history_queue_ahead(s, r->id, ahead);
        for (size_t k = 0; k < n; ++k) {
            wr_time_us width = ahead[k]->req.window.end_us - ahead[k]->req.window.start_us;
            if (ahead[k]->req.window.end_us > fixed) {
                fixed = ahead[k]->req.window.end_us; /* it cannot start before then */
            }
            slack += width;
            fixed += width;
        }
        /* ...and this request cannot start before its own window has closed. */
        if (r->req.window.end_us > fixed) {
            fixed = r->req.window.end_us;
        }

        turn_at = (s->now_us + slack > fixed) ? (s->now_us + slack) : fixed;
        eviction_at = r->req.window.start_us + depth;

        if (eviction_at < turn_at) {
            wr_event ev;
            if (r->eviction_warned) {
                continue;
            }
            r->eviction_warned = true;
            memset(&ev, 0, sizeof(ev));
            ev.type = (uint16_t)WR_EV_HISTORY_EVICTION_RISK;
            ev.u.history_eviction_risk.request_id = r->id;
            ev.u.history_eviction_risk.queued_for_us = s->now_us - r->reserved_us;
            /* ⚠ Signed, and negative means the window's oldest sample is
             * estimated to be gone ALREADY (event.h).  Both figures travel
             * together: the wait is what makes the margin mean anything. */
            ev.u.history_eviction_risk.estimated_eviction_in_us = eviction_at - s->now_us;
            event_push(s, &ev);
            continue;
        }
        /*
         * Not at risk yet.  `fixed` cannot move, so the crossing comes from the
         * `t + slack` term alone and is solved exactly.  ⚠ Strictly in the
         * future by construction — eviction_at >= now + slack here — so this row
         * never asks arm() for a time in the past.
         */
        {
            wr_time_us crossing = eviction_at - slack + 1;
            if (crossing < *out_next) {
                *out_next = crossing;
            }
        }
    }
}

static void history_arm(wr_session *s)
{
    wr_time_us start = WR_TIME_NEVER;
    wr_time_us deadline = WR_TIME_NEVER;
    wr_time_us eviction = WR_TIME_NEVER;

    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        const wr_request *r = &s->reqs[i];
        if (r->state == WR_GATHER_FREE || r->state == WR_GATHER_READY) {
            continue;
        }
        if (r->req.deadline_us < deadline) {
            deadline = r->req.deadline_us;
        }
        if (r->state == WR_GATHER_QUEUED && r->req.window.end_us > s->now_us &&
            r->req.window.end_us < start) {
            start = r->req.window.end_us;
        }
    }
    /* ⚠ Only ever armed in the future.  A queued request that is ALREADY
     * eligible but blocked — behind another pull, or behind §6.1.1's wait for a
     * live frame — is served by the event that unblocks it, never by a deadline
     * in the past that would spin the host's loop. */
    if (start == WR_TIME_NEVER) {
        disarm(s, WR_DUE_HISTORY_START);
    } else {
        arm(s, WR_DUE_HISTORY_START, start);
    }
    if (deadline == WR_TIME_NEVER) {
        disarm(s, WR_DUE_HISTORY_DEADLINE);
    } else {
        arm(s, WR_DUE_HISTORY_DEADLINE, deadline);
    }
    /* §5.7's third history row, and the same aggregate discipline as the other
     * two: the estimate runs over the whole table here, so next_due_us() stays
     * O(1) in queue depth. */
    history_eviction_check(s, &eviction);
    if (eviction == WR_TIME_NEVER) {
        disarm(s, WR_DUE_HISTORY_EVICTION);
    } else {
        arm(s, WR_DUE_HISTORY_EVICTION, eviction);
    }
}

static void history_service(wr_session *s)
{
    wr_request *pick = NULL;

    if (s->closed || s->history_frozen) {
        return;
    }
    history_capture_fits(s);

    if (s->active >= 0) {
        /*
         * ⚠ SERIALISED: a second `a1` cannot be issued until the first
         * completes (AR B16).  The one thing the request in flight may still
         * want is its next attempt — a refill, or the far half of §8.3's wrap
         * split.
         *
         * ⚠ AND THE R8 INTERLOCK APPLIES HERE TOO, which is not obvious: this
         * request is "in flight" but its bracket is CLOSED, so cal_guard() would
         * let wr_calibration_begin() through and this line would then open a new
         * bracket in the middle of the routine — suspending live delivery
         * (§10.1) while §8.2's device is watching for a CONTINUOUS RAISE, and
         * aborting the attempt at the raise limit for a reason that has nothing
         * to do with calibration.  The next attempt waits for the routine to end
         * exactly as a fresh request does, below.
         */
        wr_request *a = &s->reqs[s->active];
        if (a->state == WR_GATHER_REFILL && !s->bracket_open && !s->pull_needs_live &&
            s->stream == WR_STREAM_RUNNING && !cal_in_progress(s)) {
            history_attempt(s, a);
        }
        return;
    }
    if (s->bracket_open || s->replay_pending) {
        return;
    }
    if (s->pull_needs_live) {
        return; /* ⚠ §6.1.1: never two pulls inside one live-frame gap */
    }
    if (s->stream != WR_STREAM_RUNNING) {
        return;
    }
    /*
     * ⚠ THE OTHER HALF OF THE R8 INTERLOCK, and it is the half that is easy to
     * forget.  Every wr_calibration_* call already refuses with WR_ERR_BUSY
     * inside a bracket; this is the same rule from the other side.  A bracket
     * opened during a routine would suspend live delivery (§10.1) for seconds,
     * and §8.2's device observes a CONTINUOUS RAISE between the markers — so
     * holding an `a2 01` past the raise limit would abort an attempt for a
     * reason that has nothing to do with calibration, with a user standing
     * there holding their arm out.  The request waits; the next live frame
     * after the routine ends picks it up.
     */
    if (cal_in_progress(s)) {
        return;
    }

    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        wr_request *r = &s->reqs[i];
        if (r->state != WR_GATHER_QUEUED || !r->have_fit) {
            continue;
        }
        if (r->req.window.end_us > s->now_us) {
            continue;
        }
        if (pick == NULL || r->id < pick->id) {
            pick = r; /* served in reservation order (§8.6) */
        }
    }
    if (pick != NULL) {
        history_issue(s, pick);
    }
}

static void history_bracket_opened(wr_session *s)
{
    wr_request *r = NULL;

    /* ⚠ Live delivery is suspended from here to the closing marker, so the fit
     * gets no new observations for roughly the width of the window (AR B15).
     * The flag is what makes that visible in every snapshot taken meanwhile. */
    wr_fit_set_blind(&s->fit, true);

    if (s->active >= 0 && s->reqs[s->active].state == WR_GATHER_REQUESTED) {
        r = &s->reqs[s->active];
    }
    s->bracket_owned = (r != NULL);
    s->bracket_request_id = (r != NULL) ? r->id : 0u;
    if (r == NULL) {
        return;
    }

    r->state = WR_GATHER_BRACKET;
    /* ⚠ VALUE-COPIED, so history device time lands on the live timeline without
     * polluting the live ratio fit (implementation-notes §5). */
    r->tick[WR_UNIT_LOWER_ARM] = s->tick[WR_UNIT_LOWER_ARM];
    r->tick[WR_UNIT_PALM] = s->tick[WR_UNIT_PALM];
    r->run[r->run_count].start = r->sample_count;
    r->run[r->run_count].count = 0u;
    r->run_open = true;
    r->run_have_index = false;
    if (!r->have_stall) {
        r->stall.start_us = s->bracket_open_us;
        r->have_stall = true;
    }
    r->stall.end_us = s->bracket_open_us;

    history_progress_event(s, r, WR_EV_HISTORY_STARTED);
}

/* An attempt has ended.  Either the holes are worth another `a1`, or the block
 * is what it is. */
static bool pick_refill_gap(const wr_request *r, wr_index_range *out)
{
    uint64_t cursor = r->recorded.first;
    uint64_t best = 0u;
    bool     found = false;

    for (size_t i = 0; i < r->cov.count; ++i) {
        uint32_t lo = r->cov.ranges[i].first;
        uint32_t hi = r->cov.ranges[i].last;
        if (hi < r->recorded.first) {
            continue;
        }
        if (lo > r->recorded.last) {
            break;
        }
        if ((uint64_t)lo > cursor) {
            uint64_t w = (uint64_t)lo - cursor;
            if (w > best) {
                best = w;
                out->first = (uint32_t)cursor;
                out->last = lo - 1u;
                found = true;
            }
        }
        if ((uint64_t)hi + 1u > cursor) {
            cursor = (uint64_t)hi + 1u;
        }
    }
    if (cursor <= (uint64_t)r->recorded.last) {
        uint64_t w = (uint64_t)r->recorded.last - cursor + 1u;
        if (w > best) {
            best = w;
            out->first = (uint32_t)cursor;
            out->last = r->recorded.last;
            found = true;
        }
    }
    /*
     * ⚠ §7.3's FLOOR DECIDES WHAT A REFILL IS FOR.  A gap of up to 7 indices is
     * step 8 — the buffer working correctly over a still wrist — and those
     * samples were never stored, so re-requesting them returns nothing while
     * costing another stall and another hole in the recording (§7.5).  Without
     * this every ordinary at-rest pull would burn all three attempts for no new
     * data.  A gap wider than the floor is not explicable by motion.
     */
    if (!found || best < (uint64_t)HISTORY_ADAPTIVE_STEP_MAX) {
        return false;
    }
    /*
     * ⚠ AND THE REFILL OBEYS THE WRAP TOO.  A gap in a window that straddled
     * the 82.0 s counter wrap can itself straddle it, and `a1` cannot address
     * that (§7.1) — wr_cmd_history() would refuse the inverted re-wrapped pair
     * and take the whole request down with WR_HIST_ERROR.  Chase the near turn
     * only; the far one stays an honest gap on the block.
     */
    if ((out->first >> 16) != (out->last >> 16)) {
        out->last = (out->first | 0xffffu);
        if (index_width(*out) < 2u) {
            return false;
        }
    }
    return true;
}

static void history_bracket_closed(wr_session *s)
{
    wr_request    *r;
    wr_index_range gap;
    uint16_t       budget;

    if (s->active < 0) {
        return;
    }
    r = &s->reqs[s->active];
    if (r->state != WR_GATHER_BRACKET) {
        return;
    }
    gather_close_run(r);
    r->stall.end_us = s->now_us;

    if (r->out_of_range > r->out_of_range_reported) {
        warn_now(s, WR_WARN_HISTORY_OUT_OF_RANGE,
                 (int32_t)(r->out_of_range - r->out_of_range_reported), 0.0);
        r->out_of_range_reported = r->out_of_range;
    }
    history_progress_event(s, r, WR_EV_HISTORY_PROGRESS);

    /*
     * ⚠ THE FAR SIDE OF A WRAP SPLIT IS NOT A REFILL, AND MUST NOT BE BUDGETED
     * AS ONE (§8.3).  It is the rest of the same ask: the caller asked for one
     * window and `a1` simply cannot address it in one command.  Charging it to
     * `max_attempts` — whose default is 3 and whose whole purpose is to bound
     * how many times we re-request HOLES — would make a window that happens to
     * straddle the 82.0 s wrap come back half-served whenever the caller passed
     * `max_attempts = 1`, which is exactly the silent half-window this split
     * exists to prevent.  HISTORY_MAX_ATTEMPTS and the request's own deadline
     * still bound it.
     */
    if (r->have_pending_ask && r->attempts < HISTORY_MAX_ATTEMPTS &&
        s->now_us < r->req.deadline_us) {
        r->ask = r->pending_ask;
        r->have_pending_ask = false;
        r->state = WR_GATHER_REFILL;
        history_arm(s);
        return;
    }
    r->have_pending_ask = false;

    budget = (r->req.max_attempts > 0u) ? r->req.max_attempts : 3u;
    if (budget > HISTORY_MAX_ATTEMPTS) {
        budget = HISTORY_MAX_ATTEMPTS;
    }

    if (r->req.refill_gaps && r->attempts < budget && s->now_us < r->req.deadline_us &&
        wr_coverage_indices_in(&r->cov, r->indices) < index_width(r->indices) &&
        pick_refill_gap(r, &gap)) {
        /* ⚠ Safe to ask again: `a1` works in place and the device never stopped
         * recording (AR C5).  The wait for a live frame is §6.1.1's. */
        r->ask = gap;
        r->state = WR_GATHER_REFILL;
        history_arm(s);
        return;
    }
    history_finish(s, r, gather_status(r));
}

/*
 * §10.1: live and history `0x90` frames are BYTE-IDENTICAL.  The
 * `a1 02` … `a1 01` bracket is the only discriminator, and it is protocol
 * state, so exactly one line in this file decides it.
 */
static void on_frame(wr_session *s, wr_decoded *dec, wr_time_us host_recv_us)
{
    size_t n = dec->u.frame.count;

    if (s->bracket_open) {
        /*
         * ⚠ THE ONE DISCRIMINATOR.  Everything inside the bracket is a history
         * record; gather_record() decides whether anybody is waiting for it.
         */
        for (size_t i = 0; i < n; ++i) {
            gather_record(s, &dec->u.frame.sample[i]);
        }
        return;
    }

    if (s->abandoned_replay) {
        /*
         * ⚠ A teardown closed a bracket while the device was still replaying.
         * These records are byte-identical to live ones and their indices run
         * thousands of samples behind, so delivering them would take the live
         * unwrapper — and every later request's range — with them.  Counted and
         * discarded until the stream actually stops.
         */
        s->orphan_records += (uint64_t)n;
        return;
    }

    if (s->stream == WR_STREAM_STOPPED) {
        /*
         * A frame with no stream behind it.  §6.1: a fresh connection always
         * begins idle, so this cannot happen against this device — and it is
         * not evented because there is no stream id, no index space and no fit
         * to attach it to, so anything delivered would be a sample the
         * consumer could neither date nor place.  The wire log holds the bytes
         * verbatim if it ever does happen.
         */
        return;
    }

    if (s->stream == WR_STREAM_STARTING) {
        wr_event ev;
        /* ⚠ Keyed off the FIRST FRAME, not off the `a0 01` ack, which the
         * vendor app ignores and so do we (§5.1). */
        s->stream = WR_STREAM_RUNNING;
        s->stream_started_us = host_recv_us;
        disarm(s, WR_DUE_STREAM_START);
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)WR_EV_STREAM_STARTED;
        ev.u.stream.stream_id = s->stream_id;
        ev.u.stream.config_bits = s->active_cfg.bits;
        event_push(s, &ev);
    }

    /* Pass 1: everything derivable from the frame, then the ONE fit
     * observation site.  Both records are observed before the snapshot is
     * refreshed, so a sample is never dated by a fit that excludes it. */
    for (size_t i = 0; i < n; ++i) {
        wr_sample *sample = &dec->u.frame.sample[i];
        derive_from_frame(s, sample);
        if ((sample->flags & (uint16_t)WR_SAMPLE_INDEX_MISSING) == 0u) {
            /* ⚠ THE ONE wr_fit_observe() CALL SITE.  Live frames only: history
             * arrival timestamps carry no information at all (§10, §10.1). */
            wr_fit_observe(&s->fit, sample->sample_index, host_recv_us);
        }
    }

    /* Refreshed once per NOTIFICATION rather than once per record: the refit is
     * O(hull) and the mapping cannot move between two records of one frame. */
    wr_fit_snapshot(&s->fit, s->now_us, &s->snap);
    s->snap_valid = true;

    for (size_t i = 0; i < n; ++i) {
        live_deliver(s, &dec->u.frame.sample[i], host_recv_us);
    }

    s->last_live_us = host_recv_us;
    s->have_live = true;
    arm(s, WR_DUE_LIVE_GAP, host_recv_us + s->policy.live_gap_alarm_us);
    if (s->due[WR_DUE_CLOCK_EVENT] == WR_TIME_NEVER) {
        arm(s, WR_DUE_CLOCK_EVENT, s->now_us + s->policy.clock_event_period_us);
    }

    /*
     * ⚠ §6.1.1's second rule, discharged here: one stall is ~23,500 ticks — 72 %
     * of §10.2's ±32,768 wrap budget, where a pull-free gap uses 12 — so two
     * pulls before the next live frame exceed it and the tick unwrapper picks
     * the wrong wrap in silence.  A live frame is what re-opens the queue, and
     * it is also the observation the re-anchored fit needs before it can date
     * anything at all.
     */
    if (s->pull_needs_live) {
        s->pull_needs_live = false;
    }
    if (history_outstanding(s)) {
        history_service(s);
    }
}

/* ------------------------------------------------------------------------ */
/* Message dispatch                                                          */
/* ------------------------------------------------------------------------ */
static void emit_codec_warnings(wr_session *s, uint32_t warnings)
{
    for (int i = 0; i < (int)WR_WARN_CODE_COUNT; ++i) {
        if ((warnings & (1u << (unsigned)i)) != 0u) {
            warn_rate(s, (wr_warning_code)i);
        }
    }
}

static void on_message(wr_session *s, wr_decoded *dec, wr_time_us host_recv_us)
{
    emit_codec_warnings(s, dec->warnings);

    switch (dec->kind) {
        case WR_MSGK_FRAME:
        case WR_MSGK_LEGACY_FRAME:
            on_frame(s, dec, host_recv_us);
            break;

        case WR_MSGK_VERSIONS:
            s->info.hardware_major = dec->u.versions.hardware_major;
            s->info.hardware_minor = dec->u.versions.hardware_minor;
            s->info.protocol_major = dec->u.versions.protocol_major;
            s->info.protocol_minor = dec->u.versions.protocol_minor;
            s->info.firmware_major = dec->u.versions.firmware_major;
            s->info.firmware_minor = dec->u.versions.firmware_minor;
            s->info.product_id = dec->u.versions.product_id;
            s->info.valid |= (uint32_t)WR_INFO_VERSIONS;
            /*
             * ⚠ NOT THE HARDWARE THE SPECIFICATION WAS MEASURED ON.  "wG3"
             * names a generation, so later ones are expected and this is
             * neither an error nor a refusal — the protocol may be identical,
             * and feature gating reads the PROTOCOL version above, not this.
             *
             * What it does say is that every constant this library carries was
             * established on one product and none of it has been checked here.
             * Reported once, on the version reply, so it reaches the recording
             * rather than being assumed away.
             */
            if (dec->u.versions.product_id != WR_PRODUCT_ID_MEASURED) {
                warn_now(s, WR_WARN_UNVERIFIED_PRODUCT, (int32_t)dec->u.versions.product_id,
                         0.0);
            }
            maybe_enter_ready(s);
            break;

        case WR_MSGK_STATUS: {
            wr_event ev;
            s->info.battery_percent = dec->u.status.percent;
            s->info.status_undecoded = dec->u.status.undecoded;
            s->info.valid |= (uint32_t)WR_INFO_BATTERY;
            memset(&ev, 0, sizeof(ev));
            ev.type = (uint16_t)WR_EV_BATTERY;
            ev.u.battery.percent = dec->u.status.percent;
            ev.u.battery.status_undecoded = dec->u.status.undecoded;
            event_push(s, &ev);
            maybe_enter_ready(s);
            break;
        }

        case WR_MSGK_SENSOR_MAP:
            s->info.sensor_count = dec->u.sensor_map.count;
            memcpy(s->info.sensor_location, dec->u.sensor_map.location,
                   sizeof(s->info.sensor_location));
            s->info.valid |= (uint32_t)WR_INFO_SENSOR_MAP;
            /*
             * ⚠ THE RECORD LAYOUT THIS LIBRARY DECODES HAS EXACTLY TWO BLOCKS.
             * §6.3's record is a header followed by one block PER SENSOR, so a
             * device reporting anything but two has a record size this library
             * computes wrongly — and every field after the first block would be
             * read at the wrong offset.
             *
             * It is reported here rather than left to the quaternion norm check
             * downstream.  The norm WOULD catch it (§6.4), but it would report
             * a misaligned frame every record without ever saying why, and the
             * cause is knowable the moment the map arrives.
             */
            if (dec->u.sensor_map.count != 2u) {
                warn_now(s, WR_WARN_SENSOR_COUNT_UNSUPPORTED,
                         (int32_t)dec->u.sensor_map.count, 0.0);
            }
            maybe_enter_ready(s);
            break;

        case WR_MSGK_MAC: {
            wr_event ev;
            memcpy(s->info.mac, dec->u.text.text, sizeof(s->info.mac) - 1u);
            s->info.mac[sizeof(s->info.mac) - 1u] = '\0';
            s->info.valid |= (uint32_t)WR_INFO_MAC;
            memset(&ev, 0, sizeof(ev));
            ev.type = (uint16_t)WR_EV_IDENTITY;
            ev.u.device_info = s->info;
            event_push(s, &ev);
            maybe_enter_ready(s);
            break;
        }

        case WR_MSGK_SERIAL: {
            wr_event ev;
            memcpy(s->info.serial, dec->u.text.text, sizeof(s->info.serial) - 1u);
            s->info.serial[sizeof(s->info.serial) - 1u] = '\0';
            s->info.valid |= (uint32_t)WR_INFO_SERIAL;
            memset(&ev, 0, sizeof(ev));
            ev.type = (uint16_t)WR_EV_IDENTITY;
            ev.u.device_info = s->info;
            event_push(s, &ev);
            maybe_enter_ready(s);
            break;
        }

        case WR_MSGK_STREAM_STOPPED:
            stop_stream_locally(s, true);
            break;

        case WR_MSGK_STREAM_STARTED:
            /* The legacy `82` path's acknowledgement.  Like `a0 01` it is not
             * the start signal — the first frame is (§5.1). */
            break;

        case WR_MSGK_START_ACK:
            /* ⚠ Deliberately ignored, exactly as the vendor app does. */
            break;

        case WR_MSGK_HISTORY_MARK:
            if (!dec->u.history.valid) {
                /* Any payload other than 01/02 is what the vendor app calls an
                 * unknown magic cookie.  Not a bracket edge. */
                break;
            }
            if (dec->u.history.marker == WR_HISTORY_MARK_START) {
                /* ⚠ §7.2: the START MARKER is the acceptance test, not the `a1`
                 * write.  Opening on the write would let a refused request
                 * suppress live delivery for its whole deadline. */
                if (s->bracket_open) {
                    break; /* already open; a second start marker is not an edge */
                }
                s->bracket_open = true;
                s->bracket_open_us = s->now_us;
                /* implementation-notes §3: bracket_open ⇒ the bracket limit is
                 * armed.  Without it a host can sleep forever with writes
                 * suppressed and pending. */
                arm(s, WR_DUE_BRACKET_LIMIT, s->now_us + BRACKET_LIMIT_US);
                history_bracket_opened(s);
            } else {
                /* ⚠ §7.2's LEADING `a1 01` closes any PREVIOUS retrieval.  With
                 * no bracket open it is exactly that and closes nothing. */
                close_bracket(s);
            }
            break;

        case WR_MSGK_CALIBRATION_RESULT:
            /*
             * ⚠ NOT A VERDICT, AND NOT CONDITIONAL ON OUR STATE MACHINE.  §8.2:
             * the device emits `0x94` for every `a2 01`, applies the transform
             * every time, and re-references its own stream at that instant —
             * both quaternions step discontinuously and the relative angle
             * collapses to ~0.4°.
             *
             * So from here the streamed orientations are in the device's
             * anatomical frame whatever phase this library believes it is in,
             * and the per-sample flag follows the DEVICE rather than the machine:
             * WR_CAL_UNKNOWN, because the transform is applied and whether it
             * took is exactly what nothing on the wire reports.
             *
             * ⚠ AND THE LONG FORM CANNOT CARRY A VERDICT EVEN IN PRINCIPLE.
             * §8.2: all 64 bytes are accounted for — eight quaternions, four
             * of them the applied state and four the two poses — with no field
             * left for one.  The status byte belongs to the SHORT form, handled
             * below.  This is why the presence measurement exists.
             *
             * The 64-byte payload is carried verbatim into the wire log and
             * decoded by nobody: the device acts on it itself, so no client
             * needs the values, and §8.2's two encoders (4.5 writes only the
             * high byte of each component, 4.8 all 16 bits) mean a decoder here
             * would have to take its precision from the payload in front of it.
             */
            set_calibration_state(s, WR_CAL_UNKNOWN);
            if (dec->u.calibration.is_status) {
                /*
                 * ⚠ THE SHORT FORM.  §8.2: a status byte instead of the eight
                 * quaternions, never yet seen on the wire, and no value of it
                 * is known.  So it is reported and NOT interpreted — the
                 * routine proceeds exactly as for the long form and the
                 * presence check decides, because that measures what the device
                 * is actually emitting rather than what a byte claims.
                 */
                warn_now(s, WR_WARN_CALIBRATION_STATUS_FORM,
                         (int32_t)dec->u.calibration.status, 0.0);
            }
            if (s->cal_phase != WR_CALP_APPLYING) {
                /* A result later than our own bound, or one nothing asked for.
                 * Reported, never dropped — the frame changed underneath the
                 * consumer either way. */
                warn_now(s, WR_WARN_CALIBRATION_UNSOLICITED, (int32_t)s->cal_phase, 0.0);
                break;
            }
            cal_set_phase(s, WR_CALP_VERIFYING, WR_CAL_ABORT_NONE);
            break;

        case WR_MSGK_CAL_ACK:
            /* ⚠ §8.2: the device answers BOTH markers with the same `a2 01`, so
             * which marker an ack belongs to is OUR state and nothing on the
             * wire.  An ack in any other phase is not evented: it is either a
             * duplicate or the tail of a routine that has already ended. */
            if (s->cal_phase == WR_CALP_MARKING_POSE0) {
                cal_set_phase(s, WR_CALP_OBSERVING_RAISE, WR_CAL_ABORT_NONE);
            } else if (s->cal_phase == WR_CALP_MARKING_POSE1) {
                cal_set_phase(s, WR_CALP_APPLYING, WR_CAL_ABORT_NONE);
            }
            break;

        case WR_MSGK_DEVICE_ERROR: {
            wr_event    ev;
            wr_request *r = NULL;

            /* ⚠ §7.2: an invalid range yields the LEADING `a1 01` and then
             * `d0 03`, with NO start marker — so this, arriving while we are
             * waiting for `a1 02`, is the refusal of the request in flight. */
            if (s->active >= 0 && s->reqs[s->active].state == WR_GATHER_REQUESTED) {
                r = &s->reqs[s->active];
            }
            memset(&ev, 0, sizeof(ev));
            ev.type = (uint16_t)WR_EV_DEVICE_ERROR;
            ev.u.device_error.code = dec->u.device_error.code;
            /* `d0 03` means nothing specific — seven distinct causes all
             * returned it — so the id says which request it belongs to and the
             * classification comes from our own state, never from the code. */
            ev.u.device_error.request_id = (r != NULL) ? r->id : 0u;
            event_push(s, &ev);

            if (r != NULL) {
                /*
                 * ⚠ CLASSIFIED BY ELIMINATION, WHICH IS WHAT §7.2 ASKS A CLIENT
                 * TO DO.  Of the seven measured causes, this session's own state
                 * rules out six: a stream is running and has been (not "before
                 * any stream"), the range was built ascending and non-empty (not
                 * reversed, not null), it is a window rather than the full index
                 * space, and a restart would have cancelled the reservation (not
                 * a previous session's indices).  What is left is a range the
                 * buffer no longer holds.
                 */
                s->active = -1;
                history_finish(s, r, WR_HIST_EVICTED);
            }
            break;
        }

        case WR_MSGK_BUTTON: {
            wr_event ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = (uint16_t)WR_EV_BUTTON;
            /* ⚠ An edge HINT.  §9.4: one confirmed press produced no
             * notification and another produced two 187 ms apart, so the count
             * per press is not reliable and no counter is built on it. */
            event_push(s, &ev);
            break;
        }

        case WR_MSGK_IGNORED:
            /* In §5.1's table and there is nothing for a client to do with it. */
            break;

        case WR_MSGK_UNKNOWN:
        default:
            break;
    }
}

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------ */
static wr_time_us pick(wr_time_us configured, wr_time_us fallback)
{
    return (configured > 0) ? configured : fallback;
}

static void resolve_policy(wr_session_policy *p)
{
    const wr_session_policy d = wr_session_policy_default();

    /*
     * ⚠ 0 means "use the default" for every field here — the opposite of
     * wr_clock_correction, where a flagged 0 is a MEASURED zero.  One is a
     * starting assumption, the other a measurement; neither ever means "guess".
     */
    p->keepalive_period_us = pick(p->keepalive_period_us, d.keepalive_period_us);
    /* ⚠ Not disableable and not adjustable upward past a minute: §9.2's
     * shutdown is 300 s and the measured-good poll is 30 s. */
    if (p->keepalive_period_us > (wr_time_us)60 * 1000 * 1000) {
        p->keepalive_period_us = (wr_time_us)60 * 1000 * 1000;
    }
    p->calibration_raise_limit_us =
        pick(p->calibration_raise_limit_us, d.calibration_raise_limit_us);
    p->calibration_result_timeout_us =
        pick(p->calibration_result_timeout_us, d.calibration_result_timeout_us);
    p->bringup_timeout_us = pick(p->bringup_timeout_us, d.bringup_timeout_us);
    p->stream_start_timeout_us = pick(p->stream_start_timeout_us, d.stream_start_timeout_us);
    p->keepalive_alarm_us = pick(p->keepalive_alarm_us, d.keepalive_alarm_us);
    p->live_gap_alarm_us = pick(p->live_gap_alarm_us, d.live_gap_alarm_us);
    p->pinned_report_period_us = pick(p->pinned_report_period_us, d.pinned_report_period_us);
    p->history_pre_roll_us = pick(p->history_pre_roll_us, d.history_pre_roll_us);
    p->history_post_roll_us = pick(p->history_post_roll_us, d.history_post_roll_us);
    p->clock_event_period_us = pick(p->clock_event_period_us, d.clock_event_period_us);
    if (p->residual_alarm_us == 0u) {
        p->residual_alarm_us = d.residual_alarm_us;
    }
    if (!(p->accuracy_drift_us_per_s > 0.0)) {
        p->accuracy_drift_us_per_s = d.accuracy_drift_us_per_s;
    }
}

static size_t cap_or(size_t requested, size_t recommended)
{
    return (requested > 0u) ? requested : recommended;
}

wr_status wr_session_create(const wr_session_config *config, wr_session **out_session)
{
    wr_session_config cfg;
    wr_allocator      alloc;
    wr_layout         layout;
    wr_session       *s;
    size_t            live_cap;
    size_t            event_cap;
    size_t            wire_cap;
    size_t            gather_cap;
    size_t            coverage_cap;
    size_t            digest_cap;
    void             *block;

    if (out_session == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    *out_session = NULL;

    cfg = (config != NULL) ? *config : wr_session_config_default();

    for (int i = 0; i < (int)WR_DUE_COUNT; ++i) {
        /* A row that was added to the enum and not to the table would otherwise
         * be a deadline that arms and never fires. */
        if (k_due[i].fire == NULL || k_due[i].name == NULL) {
            return WR_ERR_INVALID_STATE;
        }
    }

    alloc = cfg.allocator;
    if (alloc.alloc == NULL || alloc.free == NULL) {
        alloc.alloc = default_alloc;
        alloc.free = default_free;
        alloc.ctx = NULL;
    }

    /*
     * The CAPACITY says what you want, the POINTER says who owns it, and the
     * two no longer interact (session.h).  `= {0}` therefore yields recommended
     * sizes with the wire log and the overlap check off.
     */
    live_cap = cap_or(cfg.memory.live_ring_capacity, WR_LIVE_RING_RECOMMENDED);
    event_cap = cap_or(cfg.memory.event_ring_capacity, WR_EVENT_RING_RECOMMENDED);
    wire_cap = cfg.memory.wire_ring_capacity;         /* ⚠ 0 → OFF */
    digest_cap = cfg.memory.digest_ring_capacity;     /* ⚠ 0 → OFF */
    gather_cap = cap_or(cfg.memory.history_gather_capacity, WR_HISTORY_GATHER_RECOMMENDED);
    coverage_cap = cap_or(cfg.memory.coverage_capacity, WR_COVERAGE_RECOMMENDED);

    /* Pass 1: measure. */
    layout.base = NULL;
    layout.used = 0u;
    (void)layout_take(&layout, sizeof(wr_session), 16u);
    (void)layout_take(&layout, WRITE_QUEUE_CAPACITY * sizeof(wr_write_request), 16u);
    if (cfg.memory.live_ring == NULL) {
        (void)layout_take(&layout, live_cap * sizeof(wr_sample), 16u);
    }
    if (cfg.memory.event_ring == NULL) {
        (void)layout_take(&layout, event_cap * sizeof(wr_event), 16u);
    }
    if (cfg.memory.wire_ring == NULL) {
        (void)layout_take(&layout, wire_cap * sizeof(wr_wire_chunk), 16u);
    }
    if (cfg.memory.history_gather == NULL) {
        (void)layout_take(&layout, gather_cap * sizeof(wr_sample), 16u);
    }
    if (cfg.memory.coverage_storage == NULL) {
        (void)layout_take(&layout, coverage_cap * sizeof(wr_index_range), 16u);
    }
    if (cfg.memory.digest_ring == NULL) {
        (void)layout_take(&layout, digest_cap * sizeof(wr_live_digest), 16u);
    }

    block = alloc.alloc(alloc.ctx, layout.used);
    if (block == NULL) {
        return WR_ERR_NO_MEMORY;
    }
    memset(block, 0, layout.used);

    /* Pass 2: place.  Same sequence, same sizes, so the offsets agree. */
    layout.base = (uint8_t *)block;
    layout.used = 0u;
    s = layout_take(&layout, sizeof(wr_session), 16u);
    s->owned = block;
    s->alloc = alloc;

    {
        void *writes = layout_take(&layout, WRITE_QUEUE_CAPACITY * sizeof(wr_write_request), 16u);
        void *live = cfg.memory.live_ring;
        void *events = cfg.memory.event_ring;
        void *wire = cfg.memory.wire_ring;
        void *gather = cfg.memory.history_gather;
        void *coverage = cfg.memory.coverage_storage;
        void *digest = cfg.memory.digest_ring;

        if (live == NULL) {
            live = layout_take(&layout, live_cap * sizeof(wr_sample), 16u);
        }
        if (events == NULL) {
            events = layout_take(&layout, event_cap * sizeof(wr_event), 16u);
        }
        if (wire == NULL) {
            wire = layout_take(&layout, wire_cap * sizeof(wr_wire_chunk), 16u);
        }
        if (gather == NULL) {
            gather = layout_take(&layout, gather_cap * sizeof(wr_sample), 16u);
        }
        if (coverage == NULL) {
            coverage = layout_take(&layout, coverage_cap * sizeof(wr_index_range), 16u);
        }
        if (digest == NULL) {
            digest = layout_take(&layout, digest_cap * sizeof(wr_live_digest), 16u);
        }

        ring_init(&s->writes, writes, WRITE_QUEUE_CAPACITY, sizeof(wr_write_request));
        ring_init(&s->live, live, live_cap, sizeof(wr_sample));
        ring_init(&s->events, events, event_cap, sizeof(wr_event));
        ring_init(&s->wire, wire, wire_cap, sizeof(wr_wire_chunk));
        wr_overlap_init(&s->overlap, (wr_live_digest *)digest, digest_cap);
        /* ⚠ Both are SHARED by whichever request is in flight, and only one ever
         * is (AR B16).  Sizing the gather from the widest window a consumer will
         * ever request — a 4.5 s pull is ~3,600 samples — rather than from the
         * buffer depth is the note that matters (implementation-notes §1). */
        s->gather = (wr_sample *)gather;
        s->gather_cap = (gather != NULL) ? gather_cap : 0u;
        s->cov_storage = (wr_index_range *)coverage;
        s->cov_cap = (coverage != NULL) ? coverage_cap : 0u;
    }

    s->policy = cfg.policy;
    resolve_policy(&s->policy);

    /*
     * ⚠ `wr_session_config c = {0}` leaves stream_config.bits at 0x00, which is
     * a real and destructive configuration — no ticks, /16 gyro, a different
     * wire format.  Nobody chooses it deliberately without a justification, and
     * wr_stream_config_nonstandard() always writes one, so an entirely empty
     * config means "unset" and gets the observed default (§6.2).
     */
    if (cfg.stream_config.bits == 0u && cfg.stream_config.legacy == 0u &&
        cfg.stream_config.justification[0] == '\0') {
        cfg.stream_config = wr_stream_config_default();
    }
    s->requested_cfg = cfg.stream_config;
    s->active_cfg = cfg.stream_config;

    memcpy(s->device_id, cfg.device_id, sizeof(s->device_id) - 1u);
    s->device_id[sizeof(s->device_id) - 1u] = '\0';

    wr_fit_init(&s->fit, s->policy.accuracy_drift_us_per_s);
    stream_reset_decode(s);
    disarm_all(s);

    s->link = WR_LINK_DOWN;
    s->stream = WR_STREAM_STOPPED;
    s->next_stream_id = 1u;
    /* ⚠ Not zero: zero is a valid slot.  And ids start at 1 so that 0 can mean
     * "no request" on every event that carries one. */
    s->active = -1;
    s->next_request_id = 1u;
    /* Before any link there is no calibration, and saying UNKNOWN would claim
     * we had not looked.  We have: none has been performed (§8.3). */
    s->cal_state = WR_CAL_UNCALIBRATED;
    s->cal_prev_state = WR_CAL_UNCALIBRATED;
    s->cal_phase = WR_CALP_IDLE;
    s->cal_phase_prev = WR_CALP_IDLE;
    s->cal_abort_reason = WR_CAL_ABORT_NONE;

    *out_session = s;
    return WR_OK;
}

void wr_session_close(wr_session *session)
{
    if (session == NULL || session->closed) {
        return;
    }
    /*
     * ⚠ THE STOP BARRIER (api-request §2.11).  In this design it is structural:
     * the library never pushes, so "stopped" is "the queues are sealed".  After
     * this returns nothing in this file will read or write a buffer the
     * consumer supplied, so destroying one is safe from this instant.
     */
    session->local_teardown = true;
    wire_meta(session, "session_close");

    /* ⚠ Before the queues are sealed, so the events reporting them still reach
     * the consumer.  Each materialises with WR_HIST_CANCELLED and whatever had
     * arrived, and stays collectable until destroy() — a capture should record
     * what it got even when the answer is nothing. */
    history_abandon_all(session, WR_HIST_CANCELLED);

    session->closed = true;
    session->link = WR_LINK_CLOSED;
    stop_stream_locally(session, false);
    disarm_all(session);
    ring_clear(&session->writes); /* the barrier drops queued writes, by contract */
    ring_clear(&session->live);
    ring_clear(&session->wire);
    /*
     * ⚠ THE EVENT RING IS NOT CLEARED, and the comment eight lines up is why —
     * it used to say those events reach the consumer and then this function
     * threw them away in its own body (implementation-review I5).
     *
     * The barrier is about PRODUCTION, not about draining: nothing is produced
     * after this returns, so a consumer may still destroy every buffer it
     * supplied the moment it likes.  What it may now also do is poll once more
     * and find the terminal status of everything this close just finished —
     * exactly as wr_history_collect() already works after close, and for the
     * same reason.  A capture should record what it got.
     */
}

void wr_session_destroy(wr_session *session)
{
    wr_allocator alloc;
    void        *block;

    if (session == NULL) {
        return;
    }
    wr_session_close(session);

    /* Any block the caller never took.  ⚠ Blocks collected BEFORE this stay
     * valid: they own their memory and a copy of the allocator (§8.4.2). */
    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        if (session->reqs[i].block != NULL) {
            wr_history_block_release(&session->reqs[i].block->pub);
            session->reqs[i].block = NULL;
        }
    }

    /* ⚠ Copy the allocator out before the free: the free eats the struct the
     * allocator lives in. */
    alloc = session->alloc;
    block = session->owned;
    alloc.free(alloc.ctx, block);
}

/* ------------------------------------------------------------------------ */
/* Transport inputs                                                          */
/* ------------------------------------------------------------------------ */
void wr_session_on_link_up(wr_session *session, int32_t negotiated_mtu, wr_time_us now_us)
{
    static const struct {
        wr_status (*encode)(wr_write_request *);
    } k_bringup[] = {
        { wr_cmd_versions },   /* 80 — ⚠ the only REQUIRED step (§9.1)          */
        { wr_cmd_status },     /* 81                                            */
        { wr_cmd_sensor_map }, /* 84                                            */
        { wr_cmd_status },     /* 81 again, as the vendor app does              */
        { wr_cmd_serial },     /* 86 — the vendor app sends this THREE times    */
        { wr_cmd_mac },        /* 85                                            */
    };

    if (session == NULL || session->closed) {
        return;
    }
    note_now(session, now_us);

    if (session->link != WR_LINK_DOWN) {
        return;
    }

    session->mtu_rejected = false;

    if (negotiated_mtu > 0 && negotiated_mtu < WR_MIN_ATT_MTU) {
        /*
         * ⚠ Fail loudly and refuse to run.  The calibration result is 65 bytes
         * and stream notifications reach 93 (§2.4); the alternative is
         * truncated frames that parse as garbage on whichever platform
         * eventually gets it wrong.  No platform lets an application REQUEST an
         * MTU through Qt, so the library can only check.
         */
        wr_event ev;
        session->mtu_rejected = true;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)WR_EV_MTU_REJECTED;
        ev.u.mtu = negotiated_mtu;
        event_push(session, &ev);
        wire_meta(session, "mtu_rejected");
        return;
    }
    if (negotiated_mtu == 0) {
        /* "The platform will not tell me": proceed, and say that we could not
         * check rather than implying we did. */
        warn_now(session, WR_WARN_MTU_UNKNOWN, 0, 0.0);
    }

    session->link = WR_LINK_BRINGUP;
    session->link_up_us = session->now_us;
    session->last_device_byte_us = session->now_us;
    session->last_host_write_us = session->now_us;
    session->info.valid = 0u;
    /* ⚠ A reconnect never resumes a calibration: §8.3 measured 18.80° at the
     * same pose after a plain disconnect, strap untouched. */
    set_calibration_state(session, WR_CAL_UNCALIBRATED);

    wire_meta(session, "link_up");
    event_emit(session, WR_EV_LINK_UP);

    for (size_t i = 0; i < sizeof(k_bringup) / sizeof(k_bringup[0]); ++i) {
        queue_command(session, k_bringup[i].encode);
    }
    arm(session, WR_DUE_BRINGUP, session->now_us + session->policy.bringup_timeout_us);
}

void wr_session_on_link_down(wr_session *session, wr_link_down_cause cause, wr_time_us now_us)
{
    if (session == NULL || session->closed) {
        return;
    }
    note_now(session, now_us);
    go_link_down(session, cause);
}

void wr_session_on_bytes(wr_session *session, const uint8_t *data, size_t length,
                         wr_time_us host_recv_us)
{
    wr_decoded dec;
    wr_status  st;
    bool       redact;

    if (session == NULL || session->closed || data == NULL || length == 0u) {
        return;
    }
    note_now(session, host_recv_us);
    session->last_device_byte_us = session->now_us;

    /*
     * ⚠ MAC (0x85) and serial (0x86) replies are redacted unless the caller
     * asked for them (api-request §2.13).  The decision is made on the raw id
     * so it does not depend on the decode succeeding.
     */
    redact = !session->policy.record_identifiers &&
             (data[0] == (uint8_t)WR_MSG_ID_MAC || data[0] == (uint8_t)WR_MSG_ID_SERIAL);
    wire_log(session, WR_WIRE_DEVICE_TO_HOST, data, length, redact);

    st = wr_codec_decode(data, length, session->active_cfg, &dec);
    if (st == WR_ERR_UNKNOWN_MESSAGE) {
        /* §5.1: logged and ignored, never an error condition for the session. */
        wr_event ev;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)WR_EV_UNKNOWN_MESSAGE;
        ev.u.unknown_message.message_id = dec.message_id;
        ev.u.unknown_message.length = (uint8_t)((length > 255u) ? 255u : length);
        for (size_t i = 0; i < sizeof(ev.u.unknown_message.first_bytes) && i < length; ++i) {
            ev.u.unknown_message.first_bytes[i] = data[i];
        }
        event_push(session, &ev);
        return;
    }
    if (st == WR_ERR_TRUNCATED) {
        warn_rate(session, WR_WARN_SHORT_FRAME);
        return;
    }
    if (st < WR_OK) {
        return;
    }

    on_message(session, &dec, session->now_us);
}

void wr_session_on_advertising_seen(wr_session *session, wr_time_us now_us)
{
    if (session == NULL || session->closed) {
        return;
    }
    note_now(session, now_us);
    session->have_advertising = true;
    session->last_advertising_us = session->now_us;
}

/* ------------------------------------------------------------------------ */
/* The core's clock                                                          */
/* ------------------------------------------------------------------------ */
wr_time_us wr_session_next_due_us(const wr_session *session)
{
    wr_time_us best = WR_TIME_NEVER;

    if (session == NULL || session->closed) {
        return WR_TIME_NEVER;
    }
    /*
     * A const scan of a handful of slots, with no cache — a cache that can go
     * stale is exactly the bug class this file must not add
     * (implementation-notes §3).
     */
    for (int i = 0; i < (int)WR_DUE_COUNT; ++i) {
        if (session->due[i] < best) {
            best = session->due[i];
        }
    }
    return best;
}

void wr_session_tick(wr_session *session, wr_time_us now_us)
{
    int firings = 0;

    if (session == NULL || session->closed) {
        return;
    }
    note_now(session, now_us);

    session->in_tick = true;
    /*
     * ⚠ The dispatcher DISARMS BEFORE IT FIRES.  A handler that forgets to
     * re-arm then leaves a DEAD timer, which a counting test catches, rather
     * than a LIVE one, which nothing catches.
     */
    while (firings < TICK_MAX_FIRINGS) {
        int        which = -1;
        wr_time_us best = WR_TIME_NEVER;
        for (int i = 0; i < (int)WR_DUE_COUNT; ++i) {
            if (session->due[i] <= session->now_us && session->due[i] < best) {
                best = session->due[i];
                which = i;
            }
        }
        if (which < 0) {
            break;
        }
        disarm(session, (wr_due_id)which);
        k_due[which].fire(session);
        firings++;
    }
    session->in_tick = false;

    warn_flush(session);
}

/* ------------------------------------------------------------------------ */
/* Outputs                                                                   */
/* ------------------------------------------------------------------------ */
size_t wr_session_poll_writes(wr_session *session, wr_write_request *out, size_t max)
{
    size_t n;

    if (session == NULL || out == NULL || max == 0u || session->closed) {
        return 0u;
    }
    /* ⚠ Nothing at all while a bracket is open — the second of the two
     * independent lines that hold the quiet period. */
    if (session->bracket_open) {
        return 0u;
    }

    n = ring_pop(&session->writes, out, max);
    if (n > 0u) {
        /*
         * ⚠ The idle timer resets when the write reaches the DEVICE, and handing
         * it to the host is the closest instant this library can observe.  That
         * is deliberate: a host that queues and never drains then trips
         * WR_WARN_KEEPALIVE_LATE, which is exactly the failure it has.
         */
        session->last_host_write_us = session->now_us;
        if (session->link == WR_LINK_READY) {
            arm(session, WR_DUE_KEEPALIVE, session->now_us + session->policy.keepalive_period_us);
            arm(session, WR_DUE_KEEPALIVE_ALARM,
                session->now_us + session->policy.keepalive_alarm_us);
        }
        for (size_t i = 0; i < n; ++i) {
            wire_log(session, WR_WIRE_HOST_TO_DEVICE, out[i].data, out[i].length, false);
        }
    }
    return n;
}

size_t wr_session_poll_events(wr_session *session, wr_event *out, size_t max)
{
    /* ⚠ NO `closed` GUARD, unlike the polls below.  Closing MATERIALISES every
     * outstanding reservation and pushes its WR_EV_HISTORY_READY, so refusing to
     * hand those back made the documented "close, then drain once more to record
     * what you got" shape return nothing (implementation-review I5).  Blocks are
     * already collectable after close for the same reason. */
    if (session == NULL || out == NULL || max == 0u) {
        return 0u;
    }
    return ring_pop(&session->events, out, max);
}

size_t wr_session_poll_live(wr_session *session, wr_sample *out, size_t max)
{
    if (session == NULL || out == NULL || max == 0u || session->closed) {
        return 0u;
    }
    return ring_pop(&session->live, out, max);
}

size_t wr_session_poll_wire(wr_session *session, wr_wire_chunk *out, size_t max)
{
    if (session == NULL || out == NULL || max == 0u || session->closed) {
        return 0u;
    }
    return ring_pop(&session->wire, out, max);
}

uint64_t wr_session_dropped_live(const wr_session *session)
{
    return (session != NULL) ? session->live.dropped : 0u;
}

uint64_t wr_session_dropped_events(const wr_session *session)
{
    return (session != NULL) ? session->events.dropped : 0u;
}

uint64_t wr_session_dropped_wire(const wr_session *session)
{
    return (session != NULL) ? session->wire.dropped : 0u;
}

/* ------------------------------------------------------------------------ */
/* Session control                                                           */
/* ------------------------------------------------------------------------ */
wr_status wr_session_start_stream(wr_session *session)
{
    wr_write_request req;
    wr_status        st;

    if (session == NULL || session->closed) {
        return WR_ERR_INVALID_STATE;
    }
    if (session->link != WR_LINK_READY) {
        return WR_ERR_LINK_DOWN;
    }
    if (session->stream != WR_STREAM_STOPPED) {
        /* ⚠ ONE STREAM, OPENED ONCE, LEFT OPEN (AR B8).  Restarting clears the
         * buffer, resets the index space and starts the fit from nothing. */
        return WR_ERR_INVALID_STATE;
    }

    st = wr_cmd_start_stream(&req, session->requested_cfg);
    if (st < WR_OK) {
        return st;
    }

    session->active_cfg = session->requested_cfg;
    session->stream_id = session->next_stream_id++;
    session->stream_starts++;
    /* Folds the finished stream's rate into the connection-pooled estimate and
     * throws the index space away — the counter resets at every stream start
     * but the crystal does not (§10). */
    wr_fit_begin_stream(&session->fit, session->stream_id);
    stream_reset_decode(session);
    session->clock_degraded = false;

    session->stream = WR_STREAM_STARTING;
    queue_write(session, &req);
    arm(session, WR_DUE_STREAM_START, session->now_us + session->policy.stream_start_timeout_us);
    wire_meta(session, "stream_start");

    if (session->stream_starts > 1u) {
        /* ⚠ Never silent.  §7.6 lists restarting first among the five ways
         * capture goes wrong, and whether it also costs the calibration is
         * untested where a disconnect demonstrably does. */
        wr_event ev;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)WR_EV_STREAM_RESTARTED;
        ev.u.stream.stream_id = session->stream_id;
        ev.u.stream.config_bits = session->active_cfg.bits;
        event_push(session, &ev);
        /*
         * ⚠ And a calibration does not ride across the restart.  It is untested
         * in the direction that matters: a disconnect demonstrably destroys the
         * transform (§8.3) and nothing has measured whether a restart does, so
         * the choice is between samples that keep claiming CALIBRATED across an
         * unmeasured boundary and samples that say we no longer know.  The
         * second is recoverable — re-run the routine — and the first is
         * permanent and invisible (sample.h).
         *
         * ⚠ Only from CALIBRATED.  UNCALIBRATED is a thing we KNOW — no routine
         * has been run — and a restart is no reason to stop knowing it.
         */
        if (session->cal_state == WR_CAL_CALIBRATED) {
            set_calibration_state(session, WR_CAL_UNKNOWN);
        }
        session->cal_have_anchor = false;
    }
    if (wr_stream_config_is_legacy(session->active_cfg)) {
        warn_now(session, WR_WARN_LEGACY_STREAM, 0, 0.0);
    } else if (!wr_stream_config_is_observed_default(session->active_cfg)) {
        warn_now(session, WR_WARN_NONSTANDARD_CONFIG, (int32_t)session->active_cfg.bits, 0.0);
    }
    return WR_OK;
}

wr_status wr_session_stop_stream(wr_session *session)
{
    wr_write_request req;

    if (session == NULL || session->closed) {
        return WR_ERR_INVALID_STATE;
    }
    if (session->link != WR_LINK_READY) {
        return WR_ERR_LINK_DOWN;
    }
    if (session->stream == WR_STREAM_STOPPED || session->stream == WR_STREAM_STOPPING) {
        return WR_ERR_INVALID_STATE;
    }
    if (wr_cmd_stop_stream(&req) < WR_OK) {
        return WR_ERR_INVALID_STATE;
    }

    /*
     * ⚠ OUTSTANDING RESERVATIONS ARE CANCELLED HERE, BEFORE THE `83` IS QUEUED
     * (§8.4.1).  Without that ordering the write quiet period would hold the
     * consumer's own stop behind a pull it no longer wants, and a stop that
     * takes four seconds to leave the queue is a bug.  §7.4: a restart clears
     * the buffer and resets the index space, so the reservation is
     * unfulfillable from this moment whatever we do with it.
     */
    session->stream = WR_STREAM_STOPPING;
    history_abandon_all(session, WR_HIST_CANCELLED);
    /* ⚠ The device does not know we stopped listening and is still replaying.
     * Its records are byte-identical to live ones (§10.1), so nothing is
     * delivered until the stream actually stops. */
    session->abandoned_replay = session->bracket_open;
    close_bracket(session);
    queue_write(session, &req);
    /* ⚠ Bounded, like every other device-facing wait in §5.7 — see
     * due_stream_stop().  Armed from the queue rather than from the write
     * leaving, so a host that never drains poll_writes() is caught too. */
    arm(session, WR_DUE_STREAM_STOP,
        session->now_us + session->policy.stream_start_timeout_us);
    return WR_OK;
}

bool wr_session_is_streaming(const wr_session *session)
{
    return (session != NULL) && session->stream == WR_STREAM_RUNNING;
}

uint64_t wr_session_stream_id(const wr_session *session)
{
    return (session != NULL) ? session->stream_id : 0u;
}

wr_status wr_session_power_off(wr_session *session)
{
    wr_write_request req;

    if (session == NULL || session->closed) {
        return WR_ERR_INVALID_STATE;
    }
    if (session->link != WR_LINK_READY && session->link != WR_LINK_BRINGUP) {
        return WR_ERR_LINK_DOWN;
    }
    if (wr_cmd_power_off(&req) < WR_OK) {
        return WR_ERR_INVALID_STATE;
    }

    /* ⚠ Same ordering as stop_stream(), and for the same reason: the `fa` must
     * not sit behind a pull the consumer has already given up on (§8.4.1). */
    history_abandon_all(session, WR_HIST_CANCELLED);
    close_bracket(session);
    stop_stream_locally(session, session->stream != WR_STREAM_STOPPED);

    session->powering_off = true;
    session->local_teardown = true;
    queue_write(session, &req);

    /*
     * ⚠ No acknowledgement arrives and the link stays up for ~9 s (§9.3).  The
     * session enters the linger, expects the gap, and does not read it as
     * failure or retry into it.  WR_POWER_OFF_LINGER_US is 12 s: deliberate
     * margin over a single 9 s measurement, not a transcription of it.
     */
    disarm(session, WR_DUE_KEEPALIVE);
    disarm(session, WR_DUE_KEEPALIVE_ALARM);
    arm(session, WR_DUE_POWER_OFF_LINGER, session->now_us + WR_POWER_OFF_LINGER_US);
    return WR_OK;
}

/* ------------------------------------------------------------------------ */
/* Queries                                                                   */
/* ------------------------------------------------------------------------ */
wr_status wr_session_clock(const wr_session *session, wr_clock_snapshot *out)
{
    wr_session *mutable_session = (wr_session *)(uintptr_t)session;

    if (session == NULL || out == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    /* The snapshot is a VALUE and refitting is what produces it; the fit's
     * laziness is an implementation detail the const query hides. */
    wr_fit_snapshot(&mutable_session->fit, session->now_us, out);
    /* ⚠ `out` is filled either way, so a caller that ignores the status still
     * gets a snapshot whose flags say it carries no fit. */
    return ((out->flags & (uint32_t)WR_CLOCK_HAS_FIT) != 0u) ? WR_OK : WR_ERR_NO_FIT;
}

wr_status wr_session_device_info(const wr_session *session, wr_device_info *out)
{
    if (session == NULL || out == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    /* `valid` says which replies have arrived; an empty one is "no evidence",
     * which is a different answer from zeroes that look like data. */
    *out = session->info;
    return WR_OK;
}

wr_stream_config wr_session_stream_config(const wr_session *session)
{
    if (session == NULL) {
        return wr_stream_config_default();
    }
    return session->active_cfg;
}

wr_status wr_session_set_clock_correction(wr_session *session,
                                          const wr_clock_correction *correction)
{
    if (session == NULL || correction == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    /* ⚠ A zero `fields` is rejected rather than obeyed or ignored — wr_fit
     * enforces it, and clock.h has the three failure shapes that makes
     * unreachable. */
    return wr_fit_set_correction(&session->fit, correction);
}

/* ------------------------------------------------------------------------ */
/* Calibration — the public entry points                                     */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ ONE GUARD ORDER, SHARED BY ALL THREE POSE CALLS, and the order is the
 * answer they give:
 *
 *   bracket open  → WR_ERR_BUSY       a retrieval suspends live delivery
 *                                     (§10.1) and holds the write queue, so a
 *                                     marker could not go out and the presence
 *                                     run would have nothing to measure.  A UI
 *                                     retries a second later with the user
 *                                     standing still regardless (R8).
 *   link not up   → WR_ERR_LINK_DOWN
 *   stream not up → WR_ERR_NO_STREAM  ⚠ §8.2: the device observes a CONTINUOUS
 *                                     RAISE between the markers, which cannot be
 *                                     done from two static samples.  Checked
 *                                     BEFORE the phase so a stopped stream names
 *                                     itself rather than surfacing as the
 *                                     ABORTED phase it also caused.
 *   wrong phase   → WR_ERR_INVALID_STATE
 *
 * ⚠ wr_calibration_abort() is exempt from all of it: it writes nothing, and a
 * UI must always be able to cancel a routine it has given up on.
 */
static wr_status cal_guard(const wr_session *session)
{
    if (session == NULL || session->closed) {
        return WR_ERR_INVALID_STATE;
    }
    if (session->bracket_open) {
        return WR_ERR_BUSY;
    }
    if (session->link != WR_LINK_READY) {
        return WR_ERR_LINK_DOWN;
    }
    if (session->stream != WR_STREAM_RUNNING) {
        return WR_ERR_NO_STREAM;
    }
    return WR_OK;
}

wr_status wr_calibration_begin(wr_session *session)
{
    wr_status st = cal_guard(session);
    if (st < WR_OK) {
        return st;
    }
    /* ⚠ There is deliberately no AWAIT_STREAM phase to wait in (design §7.2):
     * under the one-stream cycle the stream is open from just after
     * WR_EV_READY and stays open, so a wait would never be entered, and an
     * unreachable state is worse than no state. */
    if (cal_in_progress(session)) {
        /* A routine already running is not silently restarted; abort it first.
         * Re-running from COMPLETE or ABORTED is ordinary and allowed. */
        return WR_ERR_INVALID_STATE;
    }

    /* ⚠ The previous measurement does not survive into a new attempt: the anchor
     * is "the reference pose under THIS calibration", and the transform is about
     * to be replaced. */
    session->cal_have_anchor = false;
    session->cal_run_active = false;
    session->cal_run_count = 0u;
    session->cal_started_us = session->now_us;
    cal_set_phase(session, WR_CALP_AWAIT_HORIZONTAL, WR_CAL_ABORT_NONE);
    return WR_OK;
}

wr_status wr_calibration_confirm_horizontal(wr_session *session)
{
    wr_status st = cal_guard(session);
    if (st < WR_OK) {
        return st;
    }
    if (session->cal_phase != WR_CALP_AWAIT_HORIZONTAL) {
        return WR_ERR_INVALID_STATE;
    }
    st = queue_command_arg(session, wr_cmd_calibration_marker, 0u);
    if (st < WR_OK) {
        return st; /* the phase does not move without its marker */
    }
    cal_set_phase(session, WR_CALP_MARKING_POSE0, WR_CAL_ABORT_NONE);
    return WR_OK;
}

wr_status wr_calibration_confirm_raise(wr_session *session)
{
    wr_status st = cal_guard(session);
    if (st < WR_OK) {
        return st;
    }
    if (session->cal_phase != WR_CALP_OBSERVING_RAISE) {
        return WR_ERR_INVALID_STATE;
    }
    st = queue_command_arg(session, wr_cmd_calibration_marker, 1u);
    if (st < WR_OK) {
        return st;
    }
    cal_set_phase(session, WR_CALP_MARKING_POSE1, WR_CAL_ABORT_NONE);
    return WR_OK;
}

wr_status wr_calibration_confirm_reference_pose(wr_session *session)
{
    wr_status st = cal_guard(session);
    if (st < WR_OK) {
        return st;
    }
    if (session->cal_phase != WR_CALP_VERIFYING) {
        return WR_ERR_INVALID_STATE;
    }
    if (session->cal_run_active) {
        return WR_ERR_BUSY; /* one run at a time; the pose has not changed */
    }

    /*
     * ⚠ THE MEASUREMENT DOES NOT EXIST YET.  This starts a run over the next
     * live samples and returns; WR_EV_CALIBRATION_PRESENCE carries the answer,
     * or WR_WARN_PRESENCE_NOT_MEASURED says none could be taken.  Measuring
     * synchronously would mean either blocking — which this library cannot do —
     * or reading one sample, whose angle is worthless at the calibrated end:
     * §6.4's Q14 quantisation puts ±4.45° on a true 0.36°.
     */
    session->cal_run_active = true;
    session->cal_run_count = 0u;
    arm(session, WR_DUE_CAL_PRESENCE, session->now_us + PRESENCE_RUN_WINDOW_US);
    return WR_OK;
}

wr_status wr_calibration_abort(wr_session *session)
{
    /* ⚠ No cal_guard(): abort writes nothing, so neither the quiet period nor a
     * stopped stream can refuse it. */
    if (session == NULL || session->closed) {
        return WR_ERR_INVALID_STATE;
    }
    if (!cal_in_progress(session)) {
        return WR_ERR_INVALID_STATE; /* nothing to abort */
    }
    /*
     * ⚠ AT VERIFYING THE TRANSFORM IS ALREADY APPLIED and no command reverses
     * it: §8.2's device re-references its own stream the instant it emits
     * `0x94`.  So an abort there declines the presence check rather than
     * cancelling a calibration, and the routine ends at COMPLETE carrying
     * WR_CAL_ABORT_CALLER — with the angle NaN and the state still UNKNOWN,
     * which is precisely what happened.  Reporting ABORTED would tell a
     * consumer nothing happened to a stream whose frame had just changed.
     */
    if (session->cal_phase == WR_CALP_VERIFYING) {
        cal_run_stop(session);
        cal_set_phase(session, WR_CALP_COMPLETE, WR_CAL_ABORT_CALLER);
        return WR_OK;
    }
    cal_abort(session, WR_CAL_ABORT_CALLER);
    return WR_OK;
}

wr_calibration_phase wr_calibration_current_phase(const wr_session *session)
{
    return (session != NULL) ? session->cal_phase : WR_CALP_IDLE;
}

wr_calibration_state wr_session_calibration_state(const wr_session *session)
{
    return (session != NULL) ? session->cal_state : WR_CAL_UNKNOWN;
}

float wr_calibration_presence_angle_deg(const wr_session *session)
{
    if (session == NULL || !session->cal_have_anchor) {
        return NAN; /* no measurement has been taken — not a zero */
    }
    return session->cal_anchor.relative_angle_deg;
}

wr_status wr_calibration_reference_anchor(const wr_session *session,
                                          wr_calibration_presence_event *out)
{
    if (session == NULL || out == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    if (!session->cal_have_anchor) {
        return WR_ERR_INVALID_STATE;
    }
    *out = session->cal_anchor;
    return WR_OK;
}

/* ------------------------------------------------------------------------ */
/* History — phase 4                                                         */
/* ------------------------------------------------------------------------ */
static wr_request *history_find(wr_session *s, uint64_t id)
{
    if (id == 0u) {
        return NULL;
    }
    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        if (s->reqs[i].state != WR_GATHER_FREE && s->reqs[i].id == id) {
            return &s->reqs[i];
        }
    }
    return NULL;
}

/*
 * ⚠ VALIDATED AT RESERVE TIME, NOT DISCOVERED AT DEADLINE TIME (R11).  Each of
 * these is one comparison, and each turns a silent four-second timeout into a
 * programming error at the call site.
 */
wr_status wr_history_reserve(struct wr_session *session, const wr_history_request *request,
                             uint64_t *out_request_id)
{
    wr_clock_snapshot snap;
    wr_request       *slot = NULL;
    double            span_indices;

    if (session == NULL || request == NULL || out_request_id == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    *out_request_id = 0u;
    if (session->closed) {
        return WR_ERR_INVALID_STATE;
    }
    if (request->window.start_us >= request->window.end_us) {
        return WR_ERR_INVALID_ARG; /* empty or inverted */
    }
    if (request->deadline_us <= request->window.end_us) {
        /* The window's last sample does not exist until end_us, so a deadline
         * at or before it is unsatisfiable the moment it is made. */
        return WR_ERR_INVALID_ARG;
    }

    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        if (session->reqs[i].state == WR_GATHER_FREE) {
            slot = &session->reqs[i];
            break;
        }
    }
    if (slot == NULL) {
        return WR_ERR_BUSY;
    }

    /*
     * ⚠ A LEGACY STREAM IS ANSWERED WITH A BLOCK, NOT WITH A STATUS CODE, and
     * that is deliberate.  §6.3.1: a 0x7f record carries no header, so there is
     * no counter for `a1` to address and no fit for a window to map through —
     * which means the fit check below would refuse it with WR_ERR_NO_FIT and
     * the capture would record nothing about why.  Refusing and RECORDING the
     * refusal is the point (AR B2), so the reservation succeeds and its block
     * is immediately collectable carrying WR_HIST_NOT_ALIGNABLE.
     */
    if (wr_stream_config_is_legacy(session->active_cfg)) {
        memset(slot, 0, sizeof(*slot));
        slot->state = WR_GATHER_QUEUED;
        slot->id = session->next_request_id++;
        slot->req = *request;
        slot->reserved_us = session->now_us;
        wr_coverage_init(&slot->cov, NULL, 0u);
        *out_request_id = slot->id;
        history_finish(session, slot, WR_HIST_NOT_ALIGNABLE);
        return WR_OK;
    }

    /*
     * ⚠ AND SO IS "THERE IS NO STREAM", FOR THE SAME REASON AND BY THE SAME
     * ROUTE (implementation-review I6).
     *
     * The fit is only reset at wr_session_start_stream(), never at a stop, so
     * WR_CLOCK_HAS_FIT survives one — and a reservation made after the stream
     * ended sailed past every check here, then sat QUEUED while
     * history_service() returned silently at its `stream != RUNNING` line on
     * every pass.  history_issue(), the only producer of WR_HIST_NO_STREAM, was
     * never reached, so the request waited out its whole deadline and
     * materialised WR_HIST_TIMED_OUT — the exact "silent four-second timeout"
     * the comment at the top of this function claims to have removed, and the
     * status written into the capture was wrong about why.
     *
     * ⚠ STARTING is allowed through: the window's last sample need not exist yet
     * (C1), so a reservation made while the stream comes up is legitimate and is
     * refused a few lines below with WR_ERR_NO_FIT if no frame ever arrives.
     */
    if (session->stream != WR_STREAM_RUNNING && session->stream != WR_STREAM_STARTING) {
        memset(slot, 0, sizeof(*slot));
        slot->state = WR_GATHER_QUEUED;
        slot->id = session->next_request_id++;
        slot->req = *request;
        slot->reserved_us = session->now_us;
        wr_coverage_init(&slot->cov, NULL, 0u);
        *out_request_id = slot->id;
        history_finish(session, slot, WR_HIST_NO_STREAM);
        return WR_OK;
    }

    /*
     * ⚠ A STRUCTURAL refusal, not a quality judgement: with no observation at
     * all there is no mapping from a host-time window to an index range, so
     * there is genuinely nothing to ask the device for.  Unrelated to
     * `alignment_budget_us`, which gates on how GOOD an existing fit is.
     */
    wr_fit_snapshot(&session->fit, session->now_us, &snap);
    if ((snap.flags & (uint32_t)WR_CLOCK_HAS_FIT) == 0u || !(snap.slope_us_per_index > 0.0)) {
        return WR_ERR_NO_FIT;
    }

    span_indices = (double)(request->window.end_us - request->window.start_us) /
                   snap.slope_us_per_index;
    if (span_indices < 1.0) {
        return WR_ERR_INVALID_ARG; /* narrower than one sample period */
    }
    /* ⚠ Refused, never truncated (R11): a window silently clipped to fit the
     * gather area returns a block that looks complete and is not. */
    if (session->gather == NULL || span_indices > (double)session->gather_cap) {
        return WR_ERR_BUFFER_TOO_SMALL;
    }

    memset(slot, 0, sizeof(*slot));
    slot->state = WR_GATHER_QUEUED;
    slot->id = session->next_request_id++;
    slot->req = *request;
    slot->reserved_us = session->now_us;
    wr_coverage_init(&slot->cov, NULL, 0u);

    history_arm(session);
    /* Reserving issues no radio traffic before the window's last sample can
     * exist — but if it already can, there is no reason to wait for a tick. */
    history_service(session);

    *out_request_id = slot->id;
    return WR_OK;
}

wr_status wr_history_collect(struct wr_session *session, uint64_t request_id,
                             wr_history_block **out_block)
{
    wr_request *r;

    if (session == NULL || out_block == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    *out_block = NULL;
    /* ⚠ No `closed` guard: blocks stay collectable after wr_session_close() and
     * until wr_session_destroy(), which releases whatever was never taken. */
    r = history_find(session, request_id);
    if (r == NULL) {
        return WR_ERR_INVALID_ARG; /* unknown, or already collected */
    }
    if (r->state != WR_GATHER_READY) {
        return WR_PENDING; /* never blocks */
    }
    if (r->alloc_failed || r->block == NULL) {
        memset(r, 0, sizeof(*r));
        history_arm(session);
        return WR_ERR_NO_MEMORY;
    }

    *out_block = &r->block->pub;
    memset(r, 0, sizeof(*r)); /* ownership has transferred to the caller */
    history_arm(session);
    return WR_OK;
}

wr_status wr_history_cancel(struct wr_session *session, uint64_t request_id)
{
    wr_request *r;

    if (session == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    r = history_find(session, request_id);
    if (r == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    if (r->state == WR_GATHER_READY) {
        return WR_ERR_INVALID_STATE; /* it has already finished; collect it */
    }
    /* ⚠ The block still materialises, with whatever arrived and
     * WR_HIST_CANCELLED, so a capture records what it got. */
    history_abandon(session, r, WR_HIST_CANCELLED);
    return WR_OK;
}

void wr_history_block_release(wr_history_block *block)
{
    wr_history_record *rec;
    wr_allocator       alloc;

    if (block == NULL) {
        return;
    }
    /* ⚠ The public struct is the FIRST member of the private record, so the one
     * pointer the caller holds recovers everything. */
    rec = (wr_history_record *)(void *)block;
    /*
     * ⚠ THIS CATCHES A POINTER THAT NEVER CAME FROM HERE.  It does NOT make a
     * double release safe, and saying it did would be worse than saying
     * nothing: by the second call the memory is freed, so reading the magic out
     * of it is already undefined — it merely happens to work on an allocator
     * that has not reused the page yet, which is the shape of a bug that passes
     * every test and fails in the field.  Release exactly once, as with free().
     */
    if (rec->magic != HISTORY_BLOCK_MAGIC) {
        return;
    }
    rec->magic = 0u;
    /* ⚠ A COPY of the allocator, taken before the free — the free eats the
     * struct the allocator lives in, and this may be running on a thread the
     * session never saw, after the session itself is gone (§8.4.2). */
    alloc = rec->alloc;
    alloc.free(alloc.ctx, rec);
}

size_t wr_history_pending(const struct wr_session *session)
{
    size_t n = 0u;

    if (session == NULL) {
        return 0u;
    }
    /* Outstanding means "not yet answered": a request whose block is waiting to
     * be collected has been answered. */
    for (int i = 0; i < HISTORY_MAX_PENDING; ++i) {
        if (session->reqs[i].state != WR_GATHER_FREE &&
            session->reqs[i].state != WR_GATHER_READY) {
            n++;
        }
    }
    return n;
}

/*
 * ⚠ THE CLAIM SIDE OF THE DEPTH BRACKET (§8.5, AR B9, AR C6).
 *
 * This one under-claims on purpose, and the reason is asymmetric cost: a
 * consumer that skips a pull because the library said the span was resident has
 * lost the swing, where one that pulls a span already gone gets an
 * WR_HIST_EVICTED block and knows.  So the width reported is `depth_lo` — the
 * widest reach-back the DEVICE ACTUALLY SERVED this connection — and never the
 * estimate the eviction warning runs on.
 *
 * ⚠ WR_PENDING IS NOT A FAILURE AND IT IS NOT AN EMPTY ANSWER.  It means "the
 * range is filled from §7.3's seed and nothing on this connection has measured
 * it".  The seed was measured ONCE, after 20 s of streaming on another session,
 * and §7.3 leaves open whether the depth is a fixed sample count or a fixed
 * duration — which the motion-adaptive rate turns into an 8× difference.  A
 * consumer that wants an order of magnitude may use it; one that is about to
 * skip a check must not, and the status is what separates the two.  One pull is
 * enough to move it to WR_OK.
 */
wr_status wr_history_resident_range(const struct wr_session *session, wr_time_range *out_range)
{
    wr_clock_snapshot snap;
    wr_time_us        depth;
    wr_time_us        head_us;
    /* The snapshot is a VALUE and refitting is what produces it; the fit's
     * laziness is an implementation detail the const query hides, exactly as in
     * wr_session_clock(). */
    wr_session       *mutable_session = (wr_session *)(uintptr_t)session;

    if (session == NULL || out_range == NULL) {
        return WR_ERR_INVALID_ARG;
    }
    out_range->start_us = 0;
    out_range->end_us = 0;

    /* §7.4: the device only buffers while streaming, and a restart clears both
     * the buffer and the index space. */
    if (session->stream != WR_STREAM_RUNNING || !session->have_index) {
        return WR_ERR_NO_STREAM;
    }
    /* ⚠ Structural, not a quality judgement: without a fit there is no mapping
     * from an index to a host time, so there is no host-time range to report. */
    wr_fit_snapshot(&mutable_session->fit, session->now_us, &snap);
    if ((snap.flags & (uint32_t)WR_CLOCK_HAS_FIT) == 0u || !(snap.slope_us_per_index > 0.0)) {
        return WR_ERR_NO_FIT;
    }
    head_us = wr_clock_to_host_us(&snap, session->head_index);
    if (head_us == WR_TIME_UNKNOWN) {
        return WR_ERR_NO_FIT;
    }

    depth = session->have_depth_lo ? session->depth_lo_us : WR_HISTORY_DEPTH_SEED_US;
    /* ⚠ Contradictory bounds mean the depth is not a fixed DURATION — §7.3's
     * open question — and the narrower of the two is then the only claim both
     * observations support. */
    if (session->have_depth_hi && session->depth_hi_us < depth) {
        depth = session->depth_hi_us;
    }

    out_range->end_us = head_us;
    out_range->start_us = head_us - depth;
    /*
     * ⚠ CLAMPED TO THE STREAM'S OWN START (AR B10).  A stream that began 1.2 s
     * before impact cannot claim to reach back further, and "we never recorded
     * that" is a different failure from "it has been evicted" — reported as the
     * same thing it reads as a device fault.
     */
    if (session->stream_started_us != 0 && out_range->start_us < session->stream_started_us) {
        out_range->start_us = session->stream_started_us;
    }
    if (out_range->start_us > out_range->end_us) {
        out_range->start_us = out_range->end_us;
    }
    return session->have_depth_lo ? WR_OK : WR_PENDING;
}

/*
 * ⚠ FALSE IS "WE CANNOT SAY", AND IT STAYS THAT WAY UNTIL SOMETHING HAS BEEN
 * MEASURED.  This is a bool with nowhere to put a caveat, so it answers only
 * from wr_history_resident_range()'s WR_OK — the measured case.  Before the
 * first pull of a connection there is no measurement, so the answer is false
 * and a consumer that gates on it takes the check rather than skipping it.
 */
bool wr_history_coverage_available(const struct wr_session *session, wr_time_us start_us,
                                   wr_time_us end_us)
{
    wr_time_range resident;

    if (session == NULL || start_us >= end_us) {
        return false;
    }
    if (wr_history_resident_range(session, &resident) != WR_OK) {
        return false;
    }
    /* `resident` is half-open at the head like every other wr_time_range here,
     * so the window's end may reach it but not pass it. */
    return start_us >= resident.start_us && end_us <= resident.end_us;
}
