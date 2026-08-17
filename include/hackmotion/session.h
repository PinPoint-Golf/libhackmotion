/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/session.h — the sans-I/O core.
 *
 * ============================================================================
 * THE THREADING CONTRACT.  Three sentences, as api-request §2.0.7 asked.
 * ============================================================================
 * 1. Every hm_session_* / hm_history_* / hm_calibration_* function must be
 *    called from ONE thread — the same thread for the whole life of the
 *    session.  The library contains no locks, no atomics and no threads, so
 *    this is not advice.
 * 2. Nothing here is any-thread; the only objects that leave the session thread
 *    are hm_history_block values, which are immutable and owned by the caller
 *    from the moment hm_history_collect() returns them, and hm_sample / hm_event
 *    copies, which are POD.
 * 3. After hm_session_close() returns, no further sample or event can be
 *    produced by anything the library owns — because the library never pushes;
 *    the host drains.  Destroying a buffer, a file handle or a closure the
 *    consumer passed in is safe from that instant onwards.
 * ============================================================================
 *
 * ⚠ THE LIBRARY DOES NOT OWN A RADIO.  api-request §2.0 — the request that most
 * shapes adoption.  There is no scanner here, no controller, no thread, no
 * timer, no sleep, and no clock read.  A consumer that already owns a
 * cross-platform BLE stack composes with this library instead of competing with
 * it for the same adapter.
 *
 * The host's loop is:
 *
 *     on transport bytes ......... hm_session_on_bytes(s, p, n, now)
 *     on link up/down ............ hm_session_on_link_up / _on_link_down
 *     arm one timer to ........... hm_session_next_due_us(s)
 *     when it fires .............. hm_session_tick(s, now)
 *     then always ................ drain poll_writes / poll_events / poll_live
 *                                  and write the requests to the characteristic
 *
 * That is the whole integration.  Everything with a deadline attached — the
 * 30 s keepalive, the calibration bound, the history deadline — lives in this
 * state machine where it can be tested, and executes on the host's thread where
 * it composes with everything else the host runs.
 */
#ifndef HACKMOTION_SESSION_H
#define HACKMOTION_SESSION_H

#include "hackmotion/types.h"
#include "hackmotion/device.h"
#include "hackmotion/config.h"
#include "hackmotion/sample.h"
#include "hackmotion/clock.h"
#include "hackmotion/event.h"
#include "hackmotion/history.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hm_session hm_session;

/* ------------------------------------------------------------------------ */
/* Writes the host must perform                                              */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ EVERY BYTE THIS LIBRARY CAN EMIT IS ON A SHORT REVIEWABLE ALLOWLIST.
 * api-request §2.16.  There is no sendRaw(), no sendCommand(uint8_t, ...), and
 * no escape hatch — because ⛔ `f0` reboots the device into firmware-update mode
 * through the ORDINARY data characteristic, the same pipe every other command
 * uses, so avoiding the OTA service is explicitly not sufficient on its own.
 *
 * hm_command_is_allowed() is the gate every write passes through, and the test
 * suite asserts that the set of first bytes the library can produce is exactly
 * the §4 allowlist and that 0xf0 is not in it.
 *
 * Commands are short — the longest is `a1 <u16be> <u16be>`, five bytes — so a
 * write request is by value.  No lifetime, no aliasing, trivially bindable.
 */
#define HM_MAX_COMMAND_LEN 8

typedef struct hm_write_request {
    uint8_t data[HM_MAX_COMMAND_LEN];
    uint8_t length;
    uint8_t without_response;  /* the characteristic supports both */
    uint8_t reserved[2];
} hm_write_request;

/* True only for the command bytes in spec §4 that this library sends.
 * ⛔ Always false for 0xf0. */
HM_API bool hm_command_is_allowed(uint8_t command_byte);

/* The allowlist itself, for a reviewer or a test.  Returns the count and, if
 * `out` is non-NULL, writes up to `max` command bytes. */
HM_API size_t hm_command_allowlist(uint8_t *out, size_t max);

/* ------------------------------------------------------------------------ */
/* Wire log — byte-level record and replay (api-request §2.9)                */
/* ------------------------------------------------------------------------ */
/*
 * RECORD THE WIRE BYTES, NOT THE DECODED SAMPLES.  §12 still lists undecoded
 * fields — two bytes of the battery reply, the second sensor-map byte, two
 * configuration bits, the status byte on §8.2's short-form calibration result —
 * the 64-byte calibration payload is understood but decoded by nothing here, and
 * §6.6's burst trigger and §10's unexplained drift are open.  When any of those is settled,
 * a byte-level recording can be RE-DECODED with the fix applied; a sample-level
 * one cannot, because it has already discarded the bytes the fix would have
 * interpreted differently.
 *
 * The core does no file I/O.  It buffers chunks into caller-supplied memory and
 * the host drains them with hm_session_poll_wire().  Writing them anywhere is
 * the host's business (or the optional hackmotion_record module's).
 */
#define HM_WIRE_CHUNK_MAX 256

typedef enum hm_wire_direction {
    HM_WIRE_HOST_TO_DEVICE = 0,
    HM_WIRE_DEVICE_TO_HOST = 1,
    HM_WIRE_META           = 2   /* link up/down, MTU, stream boundaries */
} hm_wire_direction;

typedef enum hm_wire_flag {
    /* ⚠ Identifiers were removed before this chunk was queued.  MAC (0x85) and
     * serial (0x86) replies are redacted by default (api-request §2.13). */
    HM_WIRE_REDACTED = 1u << 0,
    HM_WIRE_LOST     = 1u << 1   /* chunks were dropped before this one */
} hm_wire_flag;

typedef struct hm_wire_chunk {
    hm_time_us host_time_us;
    uint16_t   length;
    uint8_t    direction;   /* hm_wire_direction */
    uint8_t    flags;       /* hm_wire_flag */
    uint32_t   sequence;
    uint8_t    data[HM_WIRE_CHUNK_MAX];
} hm_wire_chunk;

/* ------------------------------------------------------------------------ */
/* Policy                                                                    */
/* ------------------------------------------------------------------------ */
typedef struct hm_session_policy {
    /*
     * ⚠ NOT ADJUSTABLE DOWNWARD TO ZERO, AND NOT DISABLEABLE.  Spec §9.2: a
     * silent connection drops at exactly 5.0 minutes and AN ACTIVE STREAM DOES
     * NOT PREVENT IT — a connection streaming continuously at 25 Hz was dropped
     * at the same deadline as a fully silent one.  What resets the timer is a
     * host→device write.  So a client that streams and never writes is dropped
     * mid-session; for a golf lesson that is mid-lesson, with an athlete
     * standing on a mat.
     *
     * 0 selects the measured-good 30 s.  Values above 60 s are clamped.
     *
     * ⚠ ONE EXCEPTION, AND IT IS THE ONLY ONE: no host→device write is emitted
     * while a history bracket is open.  See "the write quiet period" below.
     */
    hm_time_us keepalive_period_us;

    /*
     * ⚠ CLIENT POLICY, NOT A DEVICE CONSTRAINT.  §8.2: the device imposes no
     * observed deadline between the two calibration markers — one attempt took
     * 15.6 s, and the device returned a result and applied it.  It was the
     * vendor's APPLICATION that rejected it.  A bound is still worth having,
     * because a raise cannot be arbitrarily slow and still describe a single
     * motion — but it must not be presented as the device's.
     * 0 selects 6 s.
     */
    hm_time_us calibration_raise_limit_us;

    /*
     * How long to wait for the DEVICE to answer during a calibration: a
     * marker's `a2 01` reply, or the `0x94` result after `a2 01`.  0 → 3 s.
     *
     * ⚠ It bounds the marker acknowledgements as well as the result, which §8.2
     * does not require and a stuck UI does: the device answered every marker in
     * both captures, so an ack that never arrives is a failure, and without a
     * bound it leaves a wizard waiting on a user holding their arm out with no
     * way back except hm_calibration_abort().  Which of the two waits expired is
     * in `previous_phase` on the HM_EV_CALIBRATION_PHASE event.
     */
    hm_time_us calibration_result_timeout_us;

    /* Bring-up watchdog: if the device has said nothing by then, give up and
     * classify.  0 → 10 s. */
    hm_time_us bringup_timeout_us;

    /*
     * How long to wait after `a0 01 7e` for the FIRST 0x90 frame before giving
     * up.  §6.1 measures it arriving within 50-80 ms, so the 3 s default is two
     * orders of margin; anything approaching it means the start was not
     * accepted.  On expiry the stream returns to STOPPED and
     * HM_WARN_STREAM_START_TIMEOUT is emitted — the session does NOT silently
     * retry, because a start that failed once will fail again and the consumer
     * needs to see it.  0 → 3 s.
     */
    hm_time_us stream_start_timeout_us;

    /*
     * ⚠ How long without a host→device write before HM_WARN_KEEPALIVE_LATE.
     * The device's own idle shutdown is 300 s (§9.2) and the poll is 30 s, so
     * the default 120 s means several polls have been missed and the deadline
     * is genuinely in view while there is still time to act.  0 → 120 s.
     */
    hm_time_us keepalive_alarm_us;

    /*
     * How long without a live frame, WHILE STREAMING AND OUTSIDE A RETRIEVAL,
     * before HM_WARN_LIVE_GAP.  ⚠ Suppressed during a history bracket, where
     * live delivery is legitimately suspended for up to the width of the pull
     * (§10.1) — warning there would cry wolf on every gather.  0 → 3 s, which
     * matches the fit's own staleness threshold.
     */
    hm_time_us live_gap_alarm_us;

    /*
     * How often to emit HM_EV_PINNED_SAMPLES, and only when the count has
     * risen since the last one.  ⚠ int16 saturation is silent (§6.4) and a
     * per-sample event would drown the queue during the very burst that caused
     * it, so the counts are accumulated and reported periodically with the
     * sample span they cover.  0 → 1 s.
     */
    hm_time_us pinned_report_period_us;

    /* Defaults for hm_history_request_around().  0 → §7.6's 3 s / 1.5 s. */
    hm_time_us history_pre_roll_us;
    hm_time_us history_post_roll_us;

    /*
     * The systematic carried in every clock snapshot, microseconds of error per
     * second of session.  ⚠ 0 here means "use the default"
     * (HM_ACCURACY_DRIFT_US_PER_S_DEFAULT, 2200 µs/s — §10's measured figure),
     * consistent with every other field in this struct.
     *
     * ⚠ That is the OPPOSITE of hm_clock_correction.residual_drift_us_per_s,
     * where a flagged 0 is a measured zero.  The two reach the same underlying
     * value by different routes and are named differently for that reason: this
     * one is a starting assumption, that one is a measurement.  Neither ever
     * means "guess".
     */
    double accuracy_drift_us_per_s;

    /* Report HM_EV_CLOCK_DEGRADED when residual p90 exceeds this.  0 → 20 ms. */
    uint32_t residual_alarm_us;

    /* Emit HM_EV_CLOCK_UPDATED at most this often.  0 → 1 s. */
    hm_time_us clock_event_period_us;

    /* ⚠ Copy the MAC and serial replies into the wire log rather than redacting
     * them.  Off by default; a consumer that turns on verbose logging should
     * not have to think about it (api-request §2.13). */
    bool record_identifiers;
} hm_session_policy;

HM_API hm_session_policy hm_session_policy_default(void);

/* ------------------------------------------------------------------------ */
/* Memory                                                                    */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ TWO SEPARATE QUESTIONS, AND THEY ARE ANSWERED BY TWO SEPARATE FIELDS.
 *
 *   The CAPACITY says what you want:
 *     0   — this facility is OFF (wire log, digest ring), or takes its
 *           recommended default (live, events, gather, coverage).  Each field
 *           below says which it is; a session cannot run with no live ring, and
 *           should not be forced to carry a wire log it never asked for.
 *     &gt; 0 — exactly this many entries.
 *
 *   The POINTER says who owns it:
 *     non-NULL — your memory, used as-is.
 *     NULL     — the library allocates it, once, at create time, from
 *                `allocator` if you gave one and from the C library otherwise.
 *
 * ⚠ hm_session_create() makes EXACTLY ONE allocation and hm_session_destroy()
 * makes exactly one free, whatever you supply: the session object itself lives
 * in that allocation, together with whichever rings you did not provide.  So
 * supplying every pointer does not reach zero allocations — it reaches one, of
 * a fixed and knowable size.  Nothing in this library allocates after create.
 *
 * So `= {0}` gives a working session with recommended sizes, the wire log off
 * and the overlap check off — and turning either on is setting one capacity,
 * not discovering an interaction between a pointer and a size.
 *
 * History BLOCKS are the one exception: they are always allocated, because they
 * outlive the session and belong to the caller (§8.4.2).
 */
/*
 * One live sample's identity, kept so a history record covering the same index
 * can be compared against it.  See `digest_ring` below.
 */
typedef struct hm_live_digest {
    uint32_t sample_index;
    uint32_t reserved;
    uint64_t digest;      /* over the RAW counts of both units */
} hm_live_digest;

typedef struct hm_session_memory {
    hm_sample     *live_ring;         size_t live_ring_capacity;    /* 0 → recommended */
    hm_event      *event_ring;        size_t event_ring_capacity;   /* 0 → recommended */
    hm_wire_chunk *wire_ring;         size_t wire_ring_capacity;    /* ⚠ 0 → OFF */
    hm_sample     *history_gather;    size_t history_gather_capacity; /* 0 → recommended */
    hm_index_range *coverage_storage; size_t coverage_capacity;     /* 0 → recommended */

    /*
     * ⚠ The live-vs-history agreement check.  0 = not performed, and the block
     * then reports `live_overlap_samples = 0`, which means NO EVIDENCE — not
     * agreement.
     *
     * A consumer stitching [live prefix] + [retrieved span] + [live suffix]
     * into one lane depends on history being a strict superset of live over the
     * same span: same index, SAME VALUES.  §6.5 calls live "a decimated view of
     * the internal rate", which implies it, but the specification never asserts
     * the values are identical for a given index — and if they are not, the
     * seam looks like a real wrist movement.
     *
     * A mid-stream `a1` covers a span live already delivered, so the library is
     * the only layer that ever holds both halves, and the comparison is free
     * inside a gather it was performing anyway.
     */
    hm_live_digest *digest_ring;      size_t digest_ring_capacity;  /* ⚠ 0 → OFF */
} hm_session_memory;

/*
 * Recommended capacities.
 *
 * ⚠ There is NO LIVE RATE CEILING.  §6.6: 25 Hz at rest and 100 Hz in motion
 * are two strong modes of a continuum, not a two-state switch, and dense bursts
 * reach index step 1 — the full ≈799.2 Hz internal rate — in EVERY session
 * containing motion.  A fixed-size ring sized from an assumed maximum will be
 * wrong under exactly the conditions that matter most, so size the live ring
 * from how often the host drains, not from a rate.
 */
#define HM_LIVE_RING_RECOMMENDED       2048u
#define HM_EVENT_RING_RECOMMENDED       256u
#define HM_WIRE_RING_RECOMMENDED       1024u
#define HM_HISTORY_GATHER_RECOMMENDED 12288u   /* > the ~6000-sample estimate */
#define HM_COVERAGE_RECOMMENDED         512u
/*
 * ⚠ NOT sized from an assumed live rate — there is no live rate ceiling (§6.6),
 * and a ring sized from one would be wrong under exactly the conditions that
 * matter.  1024 × 16 B = 16 KB is simply a generous default.
 *
 * Undersizing this loses EVIDENCE, not data: a pull reaching further back than
 * the ring covers is compared over fewer indices, `hm_overlap.evicted` counts
 * what fell off, and `live_overlap_samples` on the block reports how many were
 * actually checked.  A burst-heavy session — which is to say a session with
 * swings in it — will consume the ring faster, so a smaller
 * `live_overlap_samples` there is expected rather than alarming.
 */
#define HM_DIGEST_RING_RECOMMENDED     1024u

/* ------------------------------------------------------------------------ */
/* Configuration                                                             */
/* ------------------------------------------------------------------------ */
typedef struct hm_session_config {
    hm_stream_config  stream_config;   /* hm_stream_config_default() unless you mean it */
    hm_session_policy policy;
    hm_session_memory memory;          /* zeroed → allocate from `allocator` */
    hm_allocator      allocator;       /* zeroed → malloc/free */

    /* Opaque, caller-chosen device identity, copied in.  ⚠ Never a MAC: on
     * macOS CoreBluetooth never exposes one (api-request §2.0.4).  Used only to
     * label events, recordings and log lines. */
    char device_id[HM_DEVICE_ID_MAX];
} hm_session_config;

HM_API hm_session_config hm_session_config_default(void);

/* ------------------------------------------------------------------------ */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------ */
HM_API hm_status hm_session_create(const hm_session_config *config, hm_session **out_session);

/*
 * ⚠ THE STOP BARRIER (api-request §2.11).  Returns only once no further sample
 * delivery can occur.  In this design that is structural rather than a promise:
 * the library never pushes, so "stopped" means "the queues are sealed".  After
 * this returns it is safe to destroy any buffer, ring, file handle or closure
 * the consumer owns.  Queued writes are dropped; the host should still write
 * the `83` produced by hm_session_stop_stream() first if it wants a clean stop.
 *
 * ⚠ CLOSE, THEN DRAIN ONCE MORE — closing FINISHES work, it does not discard it.
 * Every outstanding reservation materialises here with HM_HIST_CANCELLED and
 * whatever had arrived, so after this returns:
 *
 *   - hm_session_poll_events() still yields the HM_EV_HISTORY_READY (and any
 *     warning) for each of them;
 *   - hm_history_collect() still hands the blocks over, until destroy().
 *
 * The barrier is about PRODUCTION, not about draining: nothing further is
 * produced, so destroying a buffer remains safe from the instant this returns —
 * a host that does not care simply skips the last poll.  A capture that wants
 * to record what it got should take it.  Live samples and wire chunks ARE
 * dropped: they are bulk data the caller chose to abandon by closing.
 */
HM_API void hm_session_close(hm_session *session);

/* Implies close().  Any hm_history_block already collected stays valid. */
HM_API void hm_session_destroy(hm_session *session);

/* ------------------------------------------------------------------------ */
/* Transport inputs — called by the host's transport, from the session thread */
/* ------------------------------------------------------------------------ */

/*
 * ⚠ Fails loudly if `negotiated_mtu` < HM_MIN_ATT_MTU: it emits
 * HM_EV_MTU_REJECTED carrying the negotiated value and refuses to run, because
 * the alternative is truncated frames that parse as garbage on whichever
 * platform eventually gets it wrong (api-request §2.13).  Pass the value the
 * platform reports; pass 0 if the platform will not tell you, which is treated
 * as "unknown, proceed" with a warning rather than as a failure.
 */
HM_API void hm_session_on_link_up(hm_session *session, int32_t negotiated_mtu, hm_time_us now_us);

/*
 * ⚠ ALWAYS INVALIDATES CALIBRATION.  §8.3 measured 0.70° immediately before
 * dropping a link and 18.80° at the same pose after reconnecting, strap
 * untouched and never removed.  A reconnect that silently keeps the flag at
 * Calibrated is the single worst bug this API could ship, so it is not
 * expressible: link-down drives every subsequent sample to HM_CAL_UNCALIBRATED
 * and the routine must be re-run.
 */
HM_API void hm_session_on_link_down(hm_session *session, hm_link_down_cause cause,
                                    hm_time_us now_us);

/*
 * ⚠ ONE CALL, ONE NOTIFICATION.  `data`/`length` is a complete ATT notification
 * payload exactly as the transport delivered it.  This is NOT a byte stream and
 * the library does NOT reassemble one.
 *
 * Every stack preserves the boundary — one QByteArray per
 * characteristicChanged, one D-Bus signal or socket read on BlueZ, one
 * didUpdateValueForCharacteristic on CoreBluetooth — and HM_MIN_ATT_MTU exists
 * precisely so the 93-byte maximum message fits in one of them (§2.4).
 *
 * ⚠ The alternative is not merely unnecessary, it is unimplementable: §3 says
 * the protocol has no length field, no sequence number and no checksum, so a
 * byte stream cannot be resynchronised after any loss — there is nothing to
 * synchronise TO.  A transport that coalesces would therefore corrupt silently;
 * instead it raises HM_WARN_TRAILING_BYTES on the first frame that is not a
 * whole number of records, loudly and immediately.
 */
HM_API void hm_session_on_bytes(hm_session *session, const uint8_t *data, size_t length,
                                hm_time_us host_recv_us);

/* Optional, and it improves the §9.6 classification: tell the session when the
 * host's scanner has seen this device advertising.  A device that is still
 * advertising did not go to sleep. */
HM_API void hm_session_on_advertising_seen(hm_session *session, hm_time_us now_us);

/* ------------------------------------------------------------------------ */
/* The core's clock — it never sleeps and never owns a timer                  */
/* ------------------------------------------------------------------------ */

/* Absolute host time of the next thing the session wants to do, or
 * HM_TIME_NEVER.  Re-read after every call into the session. */
HM_API hm_time_us hm_session_next_due_us(const hm_session *session);

/* Run everything now due.  Cheap and idempotent when nothing is. */
HM_API void hm_session_tick(hm_session *session, hm_time_us now_us);

/* ------------------------------------------------------------------------ */
/* Outputs — drained by the host, never pushed into it                        */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THE WRITE QUIET PERIOD.  poll_writes() returns nothing at all while a
 * history bracket is open — between the `a1` going out and its closing `a1 01`.
 *
 * The device replaying its buffer at ~260 notifications/s while an unrelated
 * command arrives is an interaction the specification never exercised: §7.5
 * measured `a1` issued into a running stream with nothing else being written.
 * And the failure shape would be the worst kind this protocol admits — a `0x81`
 * reply interleaved into a record stream that has no length field, no sequence
 * number and no checksum to resynchronise on (§3).
 *
 * Suppressing writes for the duration costs nothing, which is why it is worth
 * deciding rather than discovering:
 *
 *   - The `a1` write ITSELF reset the idle timer at the moment the bracket
 *     opened, so the keepalive has already been served.
 *   - A retrieval is bounded by the buffer depth — ~7.5 s at the very most
 *     (§7.3) — against a 300 s idle shutdown.  There is two orders of magnitude
 *     of headroom.
 *
 * The quiet period ends at the closing marker, on link-down, at a hard bracket
 * limit of twice the buffer depth, or on a consumer-initiated teardown, which
 * closes the bracket itself rather than waiting — so a wedged bracket cannot hold
 * the queue indefinitely.  The keepalive is re-armed from whichever ends it.
 *
 * ⚠ NOT at the request's deadline, and that is deliberate.  A request giving up
 * does not tell the DEVICE to stop replaying, and closing the bracket while
 * records are still arriving would put them on the live path — where their
 * indices, running thousands of samples behind, take the live unwrapper with
 * them.  The request materialises immediately with what it got; the bracket
 * stays open and its remaining records are counted and discarded.
 */
HM_API size_t hm_session_poll_writes(hm_session *session, hm_write_request *out, size_t max);
HM_API size_t hm_session_poll_events(hm_session *session, hm_event *out, size_t max);
HM_API size_t hm_session_poll_live(hm_session *session, hm_sample *out, size_t max);
HM_API size_t hm_session_poll_wire(hm_session *session, hm_wire_chunk *out, size_t max);

/* Dropped-because-nobody-drained counters.  Non-zero means the host is not
 * keeping up and data has been lost — which must be visible, not silent. */
HM_API uint64_t hm_session_dropped_live(const hm_session *session);
HM_API uint64_t hm_session_dropped_events(const hm_session *session);
HM_API uint64_t hm_session_dropped_wire(const hm_session *session);

/* ------------------------------------------------------------------------ */
/* Session control                                                           */
/* ------------------------------------------------------------------------ */
/*
 * ONE STREAM, OPENED ONCE, LEFT OPEN (api-request B8).  Start it after
 * HM_EV_READY and leave it running for the whole session: calibration needs it
 * open (§8.2), the clock fit needs it continuous (§10), and `a1` works in place
 * (§7.5) so nothing ever requires stopping it.
 *
 * ⚠ A RESTART DOES NOT CARRY A CALIBRATION ACROSS.  Stopping the stream aborts
 * any routine in progress, and a second start drops HM_CAL_CALIBRATED to
 * HM_CAL_UNKNOWN — because whether a restart costs the device's transform is
 * untested where a disconnect demonstrably does (§7.6, §8.3), and the untested
 * direction is the one where samples keep claiming a calibration nobody
 * re-checked.  Re-run the routine, or do not restart.
 */
HM_API hm_status hm_session_start_stream(hm_session *session);
HM_API hm_status hm_session_stop_stream(hm_session *session);
HM_API bool      hm_session_is_streaming(const hm_session *session);
HM_API uint64_t  hm_session_stream_id(const hm_session *session);

/*
 * ⚠ Powers the device off.  It then needs a PHYSICAL BUTTON PRESS to return.
 * The device sends no acknowledgement and the link stays up for ~9 s afterwards
 * (§9.3); the session expects that gap and will not read it as failure.
 */
HM_API hm_status hm_session_power_off(hm_session *session);

/*
 * ⚠ TEARDOWN AND OUTSTANDING RESERVATIONS.  hm_session_stop_stream(),
 * hm_session_power_off() and hm_session_close() each CANCEL any queued or
 * in-flight retrieval FIRST, materialising a block with HM_HIST_CANCELLED and
 * whatever had arrived, and only then queue their own command.
 *
 * Two reasons.  A stream restart clears the buffer and resets the index space
 * (§7.4), so a surviving reservation could never be fulfilled — and without
 * this rule the write quiet period would delay the consumer's own `83` behind a
 * pull it no longer wants.
 *
 * The rule across all four teardown paths is one line: anything the CONSUMER
 * initiates yields HM_HIST_CANCELLED; anything that HAPPENS TO US — a link drop
 * — yields HM_HIST_LINK_LOST, immediately rather than at the deadline.
 *
 * ⚠ ONE VISIBLE CONSEQUENCE.  hm_session_stop_stream() called DURING a
 * retrieval has to close the bracket to let its own `83` out — and the device,
 * which does not know, carries on replaying.  Those records are byte-identical
 * to live ones (§10.1) and their indices run thousands of samples behind, so
 * NOTHING is delivered to the live ring between that call and the device's
 * `0x83` reply.  The alternative is a burst of bulk arrivals masquerading as
 * real motion and a live index space dragged backwards with them.
 */

/* ------------------------------------------------------------------------ */
/* Calibration — an observable state machine (api-request §2.5)              */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ MANDATORY EVERY SESSION.  §8.3: calibration is lost by power cycling, by
 * remounting and by a PLAIN DISCONNECT.  No client can safely inherit one.
 *
 * ⚠ THERE IS DELIBERATELY NO hm_calibration_save() / _load() / "reuse last
 * session" (api-request §2.6).  Such a convenience would produce confidently
 * wrong data with no error anywhere, and an API that cannot express the
 * impossible thing is worth more here than a comment.
 *
 * ⚠ `0x94` IS NOT A VERDICT.  §8.2: the device emits it for EVERY `a2 01` and
 * applies the transform every time, including attempts an application goes on
 * to reject.  Any verdict is the client's own; nothing on the wire carries
 * accept or reject.
 *
 * The host-confirmed path is the only one shipped (api-request §5.2):
 * automatic raise detection means watching the stream for a motion pattern
 * whose criterion is not established, so it stays out of the contract.
 *
 * ⚠ CALIBRATION AND RETRIEVAL ARE MUTUALLY EXCLUSIVE, in both directions:
 *
 *   - Every hm_calibration_* call returns HM_ERR_BUSY while a history bracket
 *     is open.  A retrieval suspends live delivery (§10.1), and the presence
 *     check is measured FROM live samples — so it would have nothing to measure
 *     — while the pose markers are host→device writes, which the quiet period
 *     above holds anyway.  Refusing lets a UI retry a second later, with the
 *     user standing still regardless; the alternative is a calibration that
 *     aborts for a reason that has nothing to do with calibration.
 *   - A gather will not OPEN a bracket while a calibration routine is active.
 *     It stays queued.  Holding an `a2 01` past the raise limit would abort the
 *     attempt for an unrelated reason, which is the same bug seen from the
 *     other side.
 *
 * Neither can starve the other: a calibration is bounded by its raise limit
 * plus its result timeout, a retrieval by its deadline.
 *
 * ⚠ ONE EXEMPTION: hm_calibration_abort() ALWAYS works, bracket or no bracket.
 * It writes nothing — it is a local state reset — so the quiet period does not
 * apply, and a UI must always be able to cancel a routine it has given up on.
 * Refusing an abort would leave a stuck calibration with no way out until the
 * pull completed.
 */

/*
 * ⚠ Returns HM_ERR_NO_STREAM if no stream is running.  §8.2: the device
 * observes a CONTINUOUS RAISE between the two markers, which cannot be done
 * from two static samples, so calibration is not a standalone transaction.
 * It refuses rather than waiting — see hm_calibration_phase for why there is no
 * AWAIT_STREAM state.
 */
HM_API hm_status hm_calibration_begin(hm_session *session);

/* The user is in pose 0 — forearm horizontal, wrist straight.  Sends `a2 00`.
 * Refuses with HM_ERR_NO_STREAM if the stream is not already running: the
 * device cannot observe a raise from two static samples (§8.2). */
HM_API hm_status hm_calibration_confirm_horizontal(hm_session *session);

/* The user has completed the raise — forearm raised ~30° across the chest.
 * Sends `a2 01`. */
HM_API hm_status hm_calibration_confirm_raise(hm_session *session);

/*
 * The user has returned to the REFERENCE POSE (forearm horizontal, wrist
 * straight) and the library may now measure the presence check.  Optional: skip
 * it and the routine completes with the presence angle left as NaN.
 *
 * It is a separate call because only the application knows when the user is in
 * a known pose, and the angle means nothing at an unknown one.  The library
 * averages a short run of live samples and emits HM_EV_CALIBRATION_PRESENCE.
 *
 * ⚠ SKIPPING IT LEAVES hm_sample.calibration AT HM_CAL_UNKNOWN, not
 * HM_CAL_CALIBRATED.  The device applies the transform for every `a2 01`
 * including attempts an application would reject, so "we issued the markers" is
 * not evidence that calibration took — and the recording must say we did not
 * check rather than imply we did.
 *
 * ⚠ Returns HM_ERR_BUSY while a history bracket is open: the measurement is
 * taken from live samples and a retrieval suspends them.  Retry once the pull
 * completes — the user is standing still either way.
 *
 * ⚠ IT RETURNS BEFORE THE MEASUREMENT EXISTS.  HM_OK means the run has started;
 * the library then collects the next live samples at that pose and reports
 * HM_EV_CALIBRATION_PRESENCE when it has enough, or HM_WARN_PRESENCE_NOT_MEASURED
 * when too few arrived to average.  The phase reaches HM_CALP_COMPLETE either
 * way, so a UI waits on the event and not on this return value.
 */
HM_API hm_status hm_calibration_confirm_reference_pose(hm_session *session);

/*
 * ⚠ TWO OUTCOMES, BECAUSE THE DEVICE CANNOT BE UNDONE.
 *
 * Before `0x94`, nothing has been applied and this ends the routine at
 * HM_CALP_ABORTED with HM_CAL_ABORT_CALLER.
 *
 * At HM_CALP_VERIFYING the transform is ALREADY APPLIED — the device
 * re-referenced its own stream when it emitted the result, and no command
 * reverses that (§8.2).  So aborting there is a decision to skip the presence
 * check, not to undo a calibration, and the phase goes to HM_CALP_COMPLETE
 * carrying HM_CAL_ABORT_CALLER: the routine finished, the check was declined,
 * the angle stays NaN and hm_sample.calibration stays HM_CAL_UNKNOWN.  Reporting
 * ABORTED there would tell a consumer nothing happened to a stream whose frame
 * had just changed underneath it.
 *
 * Returns HM_ERR_INVALID_STATE when no routine is running.
 */
HM_API hm_status hm_calibration_abort(hm_session *session);

HM_API hm_calibration_phase hm_calibration_current_phase(const hm_session *session);
HM_API hm_calibration_state hm_session_calibration_state(const hm_session *session);

/*
 * The reference-pose relative angle, degrees.  ⚠ A PRESENCE CHECK, NOT A
 * QUALITY SCORE — see hm_calibration_presence_event for the measurement that
 * shows it inverts.  NaN before any measurement.
 */
HM_API float hm_calibration_presence_angle_deg(const hm_session *session);

/*
 * The reference-pose anchor from the most recent presence measurement — the
 * same value HM_EV_CALIBRATION_PRESENCE carried.
 *
 * It is queryable as well as evented because the event ring is drop-oldest and
 * this is data that cannot be re-derived: the pose has passed, and capturing it
 * again from the live stream would anchor on an instant that is only NEARLY the
 * one measured.  Returns HM_ERR_INVALID_STATE if no measurement has been taken
 * on the current calibration.
 */
HM_API hm_status hm_calibration_reference_anchor(const hm_session *session,
                                                 hm_calibration_presence_event *out);

/* ------------------------------------------------------------------------ */
/* Queries — synchronous, no radio, no blocking                              */
/* ------------------------------------------------------------------------ */
HM_API hm_status hm_session_clock(const hm_session *session, hm_clock_snapshot *out);
HM_API hm_status hm_session_device_info(const hm_session *session, hm_device_info *out);
HM_API hm_stream_config hm_session_stream_config(const hm_session *session);

/*
 * Feed back a clock correction the consumer measured itself (api-request §2.1).
 * A consumer with a camera and a microphone on the same host clock can measure
 * §10's drift per rig; feeding the answer into the fit is better than
 * post-correcting timestamps and leaving the recorded uncertainty a fiction.
 *
 * ⚠ BOTH KNOBS MOVE TOGETHER — see hm_clock_correction in clock.h for why, and
 * for a worked example.  Everything set here is copied and carried in every
 * subsequent clock snapshot, and therefore into every history block, so a
 * re-analysis a year later reproduces the same alignment.
 *
 * ⚠ `correction->fields` is REQUIRED and a zero bitmask returns
 * HM_ERR_INVALID_ARG.  Set HM_CORRECTION_RATE, HM_CORRECTION_DRIFT, or both;
 * a field you do not flag is left exactly as it was, and a flagged zero is a
 * MEASURED zero.  There is deliberately no value sentinel — `= {0}` is the
 * idiom for every other struct in this API, so a struct whose zero value
 * silently wiped a term nobody measured would be the one reached for by
 * accident.  clock.h has the reasoning and a worked example.
 */
HM_API hm_status hm_session_set_clock_correction(hm_session *session,
                                                 const hm_clock_correction *correction);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_SESSION_H */
