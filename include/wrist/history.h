/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wrist/history.h — the primary data path.
 *
 * WE ASK IN HOST TIME AND WE GET HOST TIME BACK.  api-request §2.14 B1: a
 * consumer should never see a device index, never unwrap a uint16, and never
 * need to know that an index space exists.
 *
 * The cycle the library owns (spec §7.6, api-request B8):
 *
 *     connect → bring-up → start the 30 s 0x81 poll            §9.1, §9.2
 *       → a0 01 7e — ONE stream, opened once and left open     §6.1
 *       → calibrate, with that stream still running            §8.2
 *       → every live frame feeds the clock fit, all session    §10
 *       → the application detects an event; asks in host time
 *       → a1 IN PLACE — do not stop                            §7.5
 *       → map every returned record back to host time by index §10
 *       → repeat.  The stream never closed.  The fit never restarted.
 *
 * ⚠ Restarting the stream is first among §7.6's five silent ways this goes
 * wrong: it clears the buffer, resets the index space, and starts the clock fit
 * from nothing.  This library never restarts a stream on a consumer's behalf,
 * and if one is ever restarted that is a reported event (WR_EV_STREAM_RESTARTED),
 * not an implementation detail.
 */
#ifndef WRIST_HISTORY_H
#define WRIST_HISTORY_H

#include "wrist/types.h"
#include "wrist/sample.h"
#include "wrist/clock.h"
#include "wrist/coverage.h"
#include "wrist/config.h"

#ifdef __cplusplus
extern "C" {
#endif

struct wr_session;
struct wr_session_policy; /* session.h — pointer only, so the headers stay acyclic */

/* ------------------------------------------------------------------------ */
/* Request                                                                   */
/* ------------------------------------------------------------------------ */
typedef struct wr_history_request {
    /* The window in HOST time.  §7.6 recommends 3 s pre-roll and 1.5 s
     * post-roll around the event — 4.5 s against a ~7.5 s buffer, leaving
     * usable margin.  wr_history_request_around() applies that default. */
    wr_time_range window;

    /* Absolute deadline on the caller's clock.  On expiry the block returns
     * what arrived plus its delivered intervals: a pull that half-delivers
     * forever is worse than one that fails (api-request B7). */
    wr_time_us deadline_us;

    /*
     * ⚠ Re-request the holes (api-request C5).  Given that the device HOLES an
     * over-wide request rather than clamping it, a partial result is exactly
     * the thing worth asking for again — and re-requesting is safe, because
     * `a1` may be issued in place and the device never stopped recording.
     *
     * The library serialises attempts, merges by device index (the reliable
     * key), and stops at the deadline or at max_attempts.
     */
    bool     refill_gaps;
    uint16_t max_attempts;      /* 0 → library default (3) */

    /*
     * Refuse rather than misalign (api-request B2).  If non-zero and the fit's
     * PRECISION at the window exceeds this (wr_clock_error.precision_us — not
     * the total, see clock.h), the request completes with
     * WR_HIST_REFUSED_ALIGNMENT and no radio traffic at all, so the refusal
     * lands in the capture's provenance instead of a silently misaligned trace.
     * A 240 fps camera frame is 4,167 µs.
     *
     * ⚠ ZERO DISABLES THE GATE AND KEEPS THE PULL.  That is the intended way to
     * ask for data with no alignment claim at all — event-anchored analysis, or
     * a post-hoc look at a session whose fit degraded.  The block still carries
     * everything except the host mapping: every sample's `sample_index` and
     * per-unit `device_time_us`, the coverage intervals, the density, the
     * pinned counts, and the fit with its flags saying exactly what it is worth.
     *
     * The shape is "we could not date these, and here they are anyway, clearly
     * marked" — never "we could not date these, so you get nothing".
     *
     * ⚠ Zero disables the QUALITY gate and nothing else.  A window that cannot
     * be addressed at all — one on the far side of a retrieval, see
     * WR_HIST_REFUSED_ALIGNMENT — is still refused, because there the `a1`
     * would ask for a different span than you asked for and hand you the wrong
     * samples rather than undated ones.
     */
    uint32_t alignment_budget_us;

    /* Opaque caller tag, echoed on the block.  The library never reads it. */
    uint64_t user_tag;
} wr_history_request;

/*
 * Fills a request with the recommended sizing around an event instant.
 * §7.6: do NOT request the whole buffer.
 *
 * Takes the policy so that `history_pre_roll_us` / `_post_roll_us` actually
 * reach it — a policy field nothing can read is a policy field that does not
 * exist.  It takes the policy rather than the session deliberately: the
 * function stays pure, needs no live device, and is testable without one.
 *
 * Pass NULL for §7.6's recommended 3 s / 1.5 s.
 */
WR_API wr_history_request wr_history_request_around(const struct wr_session_policy *policy,
                                                    wr_time_us event_host_us);

/* ------------------------------------------------------------------------ */
/* Result status                                                             */
/* ------------------------------------------------------------------------ */
typedef enum wr_history_status {
    WR_HIST_COMPLETE = 0,  /* every requested index arrived, step 1           */
    WR_HIST_SHORT,         /* contiguous but narrower than requested          */
    WR_HIST_HOLED,         /* ⚠ the device's main failure mode — gaps INSIDE  */
    WR_HIST_TIMED_OUT,     /* deadline hit; partial content is still valid    */
    WR_HIST_CANCELLED,
    /*
     * Not attempted, and no radio traffic at all.  TWO producers:
     *
     *  - the fit's PRECISION at the window exceeds `alignment_budget_us`;
     *  - ⚠ the window lies on the far side of a RETRIEVAL.  The device stops
     *    counting samples while it replays them (§7.5), so the index→host
     *    mapping is piecewise and a fit re-anchors at every pull.  A window
     *    older than the current fit's first observation cannot even be
     *    ADDRESSED through it — the derived index range would be too low by the
     *    width of the stall — so this one fires whatever the budget says.  It
     *    is unreachable under the intended cycle, where reserving at detection
     *    captures the mapping before any later pull disturbs it.
     */
    WR_HIST_REFUSED_ALIGNMENT,
    WR_HIST_EVICTED,       /* the range had already left the buffer           */
    /* ⚠ There is no stream this can be served from — nothing was ever recorded
     * for that window, the stream ended before the request got its turn, or
     * there was no stream when it was reserved.  All three answer IMMEDIATELY
     * rather than running out a deadline: a request that goes quiet costs a
     * consumer's gather a pipeline stall for no information (§8.4.1).
     *
     * ⚠ The last of the three is a BLOCK and not an error code, deliberately.
     * The fit outlives a stream stop, so reserving after one used to succeed and
     * then time out — writing a status into the capture that was wrong about why
     * (implementation-review I6).  Refusing and RECORDING the refusal is the
     * point (AR B2), so the reservation succeeds and its block is immediately
     * collectable carrying this. */
    WR_HIST_NO_STREAM,
    /* ⚠ The link dropped with this request outstanding.  The stream stopped and
     * the index space went with it, so the request can never be fulfilled — it
     * materialises IMMEDIATELY rather than waiting out its deadline, because a
     * consumer's gather has a bounded wait and a request that goes quiet after
     * the link has visibly died costs a pipeline stall for no information. */
    WR_HIST_LINK_LOST,
    WR_HIST_NOT_ALIGNABLE, /* legacy 0x7f stream: `a1` addresses a header that
                            * does not exist (§6.3.1) */
    /*
     * ⚠ The library could not serve the request and will not guess.
     *
     * A window spanning the 82.0 s counter wrap is NOT this: §8.3 requires it to
     * be issued as TWO `a1`s merged by unwrapped index, and that is what
     * happens — the caller sees one block and never learns there were two.
     * What reaches here is a window spanning the wrap TWICE (131,072 indices,
     * 164 s, against a buffer measured in seconds — unservable whatever we do),
     * a host window the fit cannot convert to an index range at all, and a
     * re-wrapped pair the §7.1 encoder refuses.  Each would otherwise return
     * part of a window as though it were all of it.
     */
    WR_HIST_ERROR,
    WR_HISTORY_STATUS_COUNT
} wr_history_status;

WR_API const char *wr_history_status_name(wr_history_status status);

/* ------------------------------------------------------------------------ */
/* Gaps — and there are two kinds that must not be conflated (C7)            */
/* ------------------------------------------------------------------------ */
typedef enum wr_gap_kind {
    /*
     * The device recorded NOTHING across this span.
     *
     * ⚠ TWO CAUSES, AND THE SECOND WAS A SURPRISE.  The stream not being open
     * is the obvious one, and under the one-stream cycle that is only before
     * the stream starts and after it stops.  The other is A RETRIEVAL: §7.5's
     * claim that issuing `a1` in place "removes that window entirely" is wrong
     * — measured across six pulls, the sample counter stalls for the pull's own
     * duration, 90-99% of it across a 16× size range.  So a mid-session pull
     * leaves one of these, and nothing on the wire marks it.
     */
    WR_GAP_NOT_RECORDED = 0,

    /* The device recorded, and did not deliver.  This is the holed pull. */
    WR_GAP_NOT_DELIVERED,

    /*
     * The CLOCK FIT was blind across this span — the link was degraded, or a
     * retrieval was in flight and live delivery is suspended for its duration
     * (api-request B15).  Samples here are aligned by extrapolation and their
     * uncertainty says so.
     *
     * ⚠ Across a RETRIEVAL this overlaps WR_GAP_NOT_RECORDED and the stronger
     * one wins: the device was not merely undelivered-from, it was not
     * sampling.  Do not read a FIT_BLIND span as "the data exists, we just
     * cannot date it well".
     */
    WR_GAP_FIT_BLIND
} wr_gap_kind;

typedef struct wr_gap {
    wr_time_range  span;
    wr_index_range indices;
    uint8_t        kind;     /* wr_gap_kind */
    uint8_t        reserved[7];
} wr_gap;

WR_API const char *wr_gap_kind_name(wr_gap_kind kind);

/* ------------------------------------------------------------------------ */
/* Calibration span (api-request C10)                                        */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THIS IS THE ONE PLACE WR_CAL_LOST IS EVER WRITTEN.  No wr_sample carries it
 * (sample.h): no sample can be captured between a disconnect and the notice of
 * one, so a per-sample `LOST` would label nothing.  Here it does work
 * UNCALIBRATED cannot — `state_at_start = CALIBRATED` with `state_at_end = LOST`
 * says the calibration went away INSIDE this block, which is a different claim
 * from "there never was one", and is exactly what `spans_transition` is for.
 */
typedef struct wr_calibration_span {
    uint8_t state_at_start;  /* wr_calibration_state */
    uint8_t state_at_end;
    uint8_t spans_transition;/* ⚠ a calibration or a reconnect happened inside */
    uint8_t reserved;
    float   presence_angle_deg; /* last presence measurement; NaN if never taken */
} wr_calibration_span;

/* ------------------------------------------------------------------------ */
/* The block — ONE owning value (api-request C2)                             */
/* ------------------------------------------------------------------------ */
/*
 * Everything a consumer needs is inside, by value or in memory the block owns.
 * There are no pointers back into the session, no shared state, and nothing
 * that becomes invalid when the session moves on or is destroyed.  It is safe
 * to move to another thread and read concurrently for as long as you like; it
 * is immutable from the moment wr_history_collect() hands it over.
 *
 * Release it with wr_history_block_release().  That is the only cleanup.
 *
 * ⚠ Samples are ASCENDING, STRICTLY MONOTONIC IN DEVICE INDEX, AND DEDUPLICATED
 * ACROSS RETRIES (api-request C3).  The library dedups by device index — the
 * reliable key — because a consumer could only dedup by timestamp, which is the
 * derived one, and a duplicate or an inversion is a silently wrong
 * interpolation rather than an error.
 */
typedef struct wr_history_block {
    uint32_t layout_version;   /* WR_SAMPLE_LAYOUT_VERSION when written */
    uint32_t sample_stride;    /* sizeof(wr_sample) in the producing build */

    const wr_sample *samples;  /* owned by the block; POD; stride as above */
    size_t           sample_count;

    uint8_t          status;   /* wr_history_status */
    uint8_t          attempts; /* how many `a1` requests were issued */
    /*
     * ⚠ THE INTERVAL LIST BELOW IS A SUPERSET, NOT THE TRUTH, WHEN THIS IS SET.
     *
     * `wr_coverage` uses fixed caller-provided storage (wr_session_memory.
     * coverage_storage).  A reply at §7.3's 100 Hz floor is one interval per
     * DELIVERED INDEX — one in eight — so a long window over a still wrist can
     * exhaust it.  On overflow the two nearest intervals are coalesced across
     * their gap: `sample_count` and the covered index total stay exact, and
     * `coverage_fraction` and `largest_gap_us` become OPTIMISTIC.
     *
     * It is surfaced rather than swallowed because an optimistic gap list that
     * does not say it is optimistic reads as a clean pull.
     */
    uint8_t          coverage_overflowed;
    uint8_t          reserved0[5];

    uint64_t         stream_id;
    uint64_t         user_tag;

    /*
     * ⚠ INTERVALS, NEVER COUNTS (api-request C4).
     *
     * ⚠ AND THE TWO RANGE TYPES DIFFER: wr_time_range is HALF-OPEN
     * [start_us, end_us); wr_index_range is INCLUSIVE [first, last].  That is
     * where an off-by-one is born, so the conversion exists exactly once, in
     * wr_clock_index_range_for_time().  Do not derive one from the other by
     * hand.
     */
    wr_time_range    requested;         /* half-open */
    wr_index_range   requested_indices; /* inclusive */
    const wr_time_range  *delivered;        /* owned; ascending, disjoint */
    size_t                delivered_count;
    const wr_index_range *delivered_indices;/* owned; same order */

    /*
     * Owned; ascending by first index, and ⚠ the three kinds MAY OVERLAP — see
     * wr_gap_kind.  A span the device never recorded is also a span it never
     * delivered, and a span the fit could not date may sit under either.  Read
     * them as three independent statements about the same index axis, not as a
     * partition of it.
     */
    const wr_gap    *gaps;
    size_t           gap_count;

    /*
     * ⚠ THREE NUMBERS THAT ANSWER THREE DIFFERENT QUESTIONS, and C4 is Critical
     * because no one of them can stand in for another.
     *
     *   coverage_fraction  how much of what you ASKED FOR arrived
     *   density            how closely spaced what arrived actually was
     *   achieved_hz        the AVERAGE rate across the span it reached
     *
     * ⚠ `density` is a SPACING, not a rate: 1 / the median gap between
     * consecutive delivered indices (wr_sample_step_density below).  §7.3's two
     * regimes are its two ends — 1.0 is index step 1, the full ≈799.2 Hz, and
     * 0.125 is step 8, the ≈100 Hz a still wrist returns.  0.0 means NOT
     * MEASURABLE, never "empty": see wr_sample_step_density.
     *
     * ⚠ Do not read `density` as `achieved_hz` normalised — they disagree
     * exactly where it matters.  §7.5's measured 58 % reply is two dense runs
     * with one 168-index hole: `achieved_hz` averages it to half rate, `density`
     * reads 1.0 because the spacing really was step 1, and `largest_gap_us`
     * carries the hole.  A consumer deciding whether a metric computed AT IMPACT
     * exists needs all three and the gap list.
     */
    double coverage_fraction;  /* of `requested`, [0,1] */
    double density;            /* 1 / median delivered index step, [0,1] */
    double achieved_hz;        /* measured over what actually arrived */
    uint32_t largest_gap_us;   /* the number that decides whether impact survives */
    uint32_t reserved1;

    /*
     * ⚠ THE LIVE-VS-HISTORY AGREEMENT, measured on this pull.
     *
     * `live_overlap_samples` is how many delivered indices the live stream had
     * ALSO delivered; `live_overlap_mismatches` is how many of those carried
     * different raw counts.  A consumer stitching a live prefix, a retrieved
     * span and a live suffix into one lane is assuming history is a strict
     * superset of live — same index, same values — and these two numbers are
     * that assumption measured rather than argued, on every capture.
     *
     * ⚠ `live_overlap_samples == 0` means NO EVIDENCE, not agreement: either
     * the pull covered a span live never reached, or no digest ring was
     * supplied (see wr_session_memory).  Never read a zero mismatch count
     * without checking the sample count beside it.
     */
    uint32_t live_overlap_samples;
    uint32_t live_overlap_mismatches;

    /*
     * ⚠ THE RECORDING GAP THIS PULL ITSELF CAUSED (api-request B11).
     *
     * Half-open host time, empty when nothing was measured.  The device stops
     * counting samples while a retrieval is in flight (§7.5, §10), so the act
     * of collecting this block cost the session a hole of the pull's own
     * width — 289 ms mean, 15-366 ms, across six measured pulls.
     *
     * ⚠ It is a separate field rather than an entry in `gaps[]` because it
     * falls OUTSIDE `requested`: the pull happens after the window it asks
     * for, so the hole lands in whatever comes next.  Putting it in `gaps[]`
     * would break the invariant that every gap lies within the request.
     *
     * B11 originally asked the vendor whether this happened.  It does, so the
     * ask became this: nothing on the wire marks the hole, and the block is the
     * only artefact that survives — so if the block does not carry it, a
     * consumer stitching a session has no way to know a span was never
     * recorded rather than merely never requested.
     *
     * ⚠ WITH MORE THAN ONE `attempts` THIS IS THE ENVELOPE over all of them.
     * Live delivery resumed for a few milliseconds between attempts, so a
     * little of this range was in fact recorded.  One half-open range cannot
     * say otherwise, and over-claiming is the safe direction: a consumer that
     * trusts it discards data that exists, where under-claiming would invent
     * continuity that never happened.
     */
    wr_time_range self_recording_gap;

    /* Provenance — everything a consumer writes into swing.json (C13). */
    wr_clock_snapshot   fit;            /* immutable, carried WITH the data (C9) */
    wr_calibration_span calibration;    /* covering the block's span (C10)       */
    wr_stream_config    config;         /* the byte that produced these samples  */
    uint8_t             reserved2[7];
    wr_pinned_counts    pinned;         /* ⚠ silent int16 saturation             */

    wr_time_us   requested_at_us;
    wr_time_us   completed_at_us;
} wr_history_block;

/* ------------------------------------------------------------------------ */
/* Achieved density (api-request C4)                                         */
/* ------------------------------------------------------------------------ */

/* The widest typical spacing wr_sample_step_density() will characterise.  Well
 * past §7.3's floor of 8, which no step exceeded across 17,739 measured ones. */
#define WR_DENSITY_STEP_MAX 256u

/*
 * 1 / the median gap between consecutive `sample_index` values, in [0,1].
 *
 * ⚠ PUBLIC BECAUSE THE BLOCK-LEVEL FIGURE IS THE WRONG SCOPE FOR THE DECISION
 * C4 DESCRIBES.  A consumer gates an analysis stage on the effective rate of
 * what is in ITS window — typically ±125 ms around impact — and `block->density`
 * is measured over the whole block, pre-roll included.  Point this at the
 * sub-range you actually care about.
 *
 * ⚠ Returns 0.0 for NOT MEASURABLE, which is three cases and never "empty":
 * fewer than two samples; samples that are not strictly ascending in device
 * index or that carry WR_SAMPLE_INDEX_MISSING; or a median gap wider than
 * WR_DENSITY_STEP_MAX, which is fewer than one sample in 256 indices.  Read it
 * beside `sample_count` and `delivered[]`, which say which.
 *
 * Samples must be ascending, strictly monotonic in `sample_index` and
 * deduplicated — which is exactly what wr_history_block guarantees (C3).
 */
WR_API double wr_sample_step_density(const wr_sample *samples, size_t count);

/* ------------------------------------------------------------------------ */
/* API                                                                       */
/* ------------------------------------------------------------------------ */

/*
 * C1 — TWO PHASE.  `reserve` at detection, `collect` when you are ready.
 *
 * A consumer's pipeline typically waits 0.5-1.25 s after impact before freezing
 * anything, so the follow-through lands in its rings.  Telling the library the
 * range you are GOING to want lets retrieval start as soon as its last sample
 * exists — and with `a1` working in place, without interrupting anything.  The
 * ~4.5 s cost then hides inside a wait the consumer was taking anyway instead
 * of being added after the post-roll.
 *
 * Returns WR_OK and an id.  Reserving does not block and issues no radio
 * traffic before the window's last sample can exist.
 *
 * ⚠ Validated at RESERVE time, not discovered at deadline time.  Returns
 * WR_ERR_INVALID_ARG for a request that cannot possibly succeed:
 *
 *   - an empty or inverted window (`start_us >= end_us`);
 *   - `deadline_us <= window.end_us` — the window's last sample does not exist
 *     until `end_us`, so a deadline at or before it is unsatisfiable the moment
 *     it is made.  Note §7.4: a pull takes about as long as its window spans,
 *     so a deadline only just past `end_us` is legal but optimistic.
 *
 * One comparison each, and it turns a silent four-second timeout into a
 * programming error at the call site.
 *
 * ⚠ Also returns WR_ERR_NO_FIT if no live frame has EVER arrived on the current
 * stream.  That is a structural refusal, not a quality judgement: without a
 * single observation there is no mapping from the host-time window to an index
 * range, so there is genuinely nothing to ask the device for.  It is unrelated
 * to `alignment_budget_us`, which gates on how GOOD an existing fit is and
 * yields WR_HIST_REFUSED_ALIGNMENT on the block instead.
 *
 * In practice this is nearly unreachable: the fit exists from the FIRST live
 * frame (see clock.h), so it fires only if a reservation is made before the
 * stream has delivered anything at all.
 *
 * Two more refusals, both at the call site rather than four seconds later:
 *
 *   - WR_ERR_BUFFER_TOO_SMALL — the window is wider than the gather area
 *     (wr_session_memory.history_gather).  ⚠ Refused, never truncated: a
 *     silently clipped window returns a block that looks complete and is not.
 *   - WR_ERR_BUSY — every reservation slot is taken.  Requests are serialised
 *     (a second `a1` cannot be issued until the first completes), so the queue
 *     is deliberately short.
 */
WR_API wr_status wr_history_reserve(struct wr_session *session,
                                    const wr_history_request *request,
                                    uint64_t *out_request_id);

/*
 * WR_PENDING while in flight; WR_OK with `*out_block` set once complete,
 * timed out or cancelled — a block is produced in all three cases and always
 * carries its coverage.  Never blocks.  Ownership transfers to the caller.
 */
WR_API wr_status wr_history_collect(struct wr_session *session,
                                    uint64_t request_id,
                                    wr_history_block **out_block);

/* Cancel a reservation.  The block still materialises, with whatever arrived
 * and WR_HIST_CANCELLED, so a capture records what it got.
 *
 * ⚠ wr_session_close() does the same to every outstanding reservation:
 * each one materialises with WR_HIST_CANCELLED and whatever had arrived, and
 * stays collectable until wr_session_destroy(), which releases any block the
 * caller never took.  A capture should record what it got even when the answer
 * is nothing. */
WR_API wr_status wr_history_cancel(struct wr_session *session, uint64_t request_id);

/*
 * ⚠ THREAD AFFINITY — the one place this library steps outside its single-thread
 * contract, and it does so deliberately (api-request C2).
 *
 *  1. wr_history_block_release() may be called from ANY thread, at any time,
 *     including after wr_session_destroy().
 *  2. It calls the wr_allocator supplied at session creation, so THAT ALLOCATOR
 *     MUST TOLERATE BEING CALLED FROM A THREAD OTHER THAN THE SESSION THREAD.
 *     That is a real constraint on anyone routing it to a pool; the default
 *     malloc/free path already satisfies it.
 *  3. The block owns a COPY of the allocator, which is what lets it outlive the
 *     session it came from.
 *
 * The natural reading of a POD struct is that it is inert memory.  It is not:
 * it is one allocation, and releasing it runs the caller's allocator.
 *
 * ⚠ RELEASE EXACTLY ONCE.  A second release is undefined, exactly as calling
 * free() twice is.  The implementation carries a magic word that rejects a
 * pointer which never came from here, but it cannot make a double release safe
 * — by then the memory is gone and reading anything out of it, magic included,
 * is already undefined.  A guard that happens to work on an allocator which has
 * not yet reused the page is worth less than none.
 */
WR_API void wr_history_block_release(wr_history_block *block);

/* How many reservations are outstanding.  ⚠ Requests are SERIALISED: a second
 * `a1` cannot be issued until the first completes (api-request B16), so a
 * queued request's range may be evicted before it can run — which the library
 * warns about via WR_EV_HISTORY_EVICTION_RISK rather than silently returning a
 * holed set for the second shot. */
WR_API size_t wr_history_pending(const struct wr_session *session);

/*
 * C6 — a cheap SYNCHRONOUS availability query.  Answerable without performing a
 * pull and without blocking on the radio, so a consumer can resolve the whole
 * job — what window, what deadline, what to do if it is short — on one thread.
 *
 * ⚠ The bounds are an ESTIMATE, refined from what the device has actually
 * returned this connection (api-request B9).  §7.3's ~6000 samples / ~7.5 s was
 * measured ONCE, and it is not established
 * whether the buffer holds a fixed sample count, a fixed duration, or something
 * that varies with the live rate — and the three differ exactly during the
 * dense bursts, which is exactly when a consumer is pulling.  Treat the width
 * as an order of magnitude, not a contract.
 *
 * ⚠⚠ READ THE STATUS.  It is what separates a measurement of YOUR connection
 * from a number measured once on somebody else's:
 *
 *   WR_OK              — the width is what the device has ACTUALLY SERVED this
 *                        connection: the widest reach-back any pull achieved.
 *                        ⚠ It is therefore a LOWER BOUND, and no wider than the
 *                        widest window you have asked for — pull only 0.5 s
 *                        windows and the library can only ever verify 0.5 s,
 *                        whatever the buffer holds.  Under-claiming is
 *                        deliberate: a consumer that skips a pull because the
 *                        library said the data was there has lost the swing.
 *   WR_PENDING         — nothing has been SERVED yet, so no residency has been
 *                        verified.  `*out_range` carries the best estimate
 *                        available: WR_HISTORY_DEPTH_SEED_US, narrowed by any
 *                        span the device has already REFUSED.  ⚠ So the width
 *                        can move while the status does not — a `d0 03` lowers
 *                        the ceiling without anything having been delivered, and
 *                        the range is then an upper bound rather than a claim.
 *                        Useful as an order of magnitude, never as a reason to
 *                        skip a check.  One pull that delivers moves it to
 *                        WR_OK.
 *   WR_ERR_NO_STREAM   — §7.4: the device only buffers while streaming.
 *   WR_ERR_NO_FIT      — no mapping from an index to a host time.  ⚠ Expected
 *                        for the few tens of milliseconds after every pull:
 *                        §6.1.1's fit re-anchors at each bracket close and
 *                        cannot speak until the next live frame lands.
 *
 * The range is clamped to the stream's own start, so a stream that began 1.2 s
 * before impact cannot claim to reach back further (AR B10) — "we never
 * recorded that" is a different failure from eviction and reads as a device
 * fault if reported as one.
 *
 * ⚠ If the two halves of the bracket ever CROSS — a span served once and
 * refused later, at a narrower width — the narrower claim wins and
 * WR_WARN_HISTORY_DEPTH_CONFLICT fires.  That is §7.3's open question answering
 * itself: a fixed-duration buffer could not do it, a fixed-sample-count one can.
 *
 * A request for a window that has left the buffer is still answered whatever
 * this says — the device replies `d0 03` and the block comes back
 * WR_HIST_EVICTED.
 */
WR_API wr_status wr_history_resident_range(const struct wr_session *session,
                                           wr_time_range *out_range);

/*
 * True if the whole of [start,end) is inside the resident estimate.
 *
 * ⚠ ANSWERS ONLY FROM THE MEASURED CASE — wr_history_resident_range()'s WR_OK.
 * It is a bool with nowhere to put a caveat, so before the first pull of a
 * connection it is false: "we cannot say", which is the only answer that cannot
 * mislead a consumer into skipping a check.  False is never a reason not to
 * pull; it is a reason not to SKIP one.
 */
WR_API bool wr_history_coverage_available(const struct wr_session *session,
                                          wr_time_us start_us,
                                          wr_time_us end_us);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WRIST_HISTORY_H */
