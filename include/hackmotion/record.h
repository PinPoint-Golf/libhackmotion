/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/record.h — the optional `.hmwire` container, and the reconciliation
 * that reads one back against the specification.
 *
 * ============================================================================
 * ⚠ THIS IS NOT PART OF THE SANS-I/O CORE.  It is a separate target, it opens
 * files, and `tests/purity.cmake` deliberately does not look at it.  The core
 * copies wire chunks into a caller-supplied ring and the host drains them with
 * hm_session_poll_wire(); everything below is one host that does the draining.
 * ============================================================================
 *
 * RECORD THE WIRE BYTES, NOT THE DECODED SAMPLES (design §5.6, api-request
 * §2.9).  spec §12 still lists undecoded fields — two bytes of the battery
 * reply, two configuration bits, the status byte on §8.2's short-form
 * calibration result — the 64-byte calibration payload is
 * understood but decoded by nothing here, and §6.6's burst trigger and §10's
 * unexplained drift are open.  When any of those is settled a byte-level
 * recording can be RE-DECODED
 * with the fix applied; a sample-level one cannot, because it has already
 * discarded the bytes the fix would have interpreted differently.
 *
 * Two halves, and they are usable independently:
 *
 *   hm_recorder   chunks  →  a versioned file
 *   hm_replay     a file  →  chunks, byte-exact, in order
 *   hm_reconciler chunks  →  a report that names each specification claim, what
 *                            was measured, AND HOW MANY SAMPLES SAY SO
 *
 * ⚠ The third exists because every number in this library came from a document
 * and nothing here had met a sensor (design §11).  A capture validates
 * the frame layout, the scales, the ≈799.2 Hz rate, the 80.166 tick ratio, the
 * 59-tick skew and the burst distribution — all of which the next three phases
 * assume.  Finding a disagreement afterwards costs three phases.
 */
#ifndef HACKMOTION_RECORD_H
#define HACKMOTION_RECORD_H

#include <stdio.h>

#include "hackmotion/types.h"
#include "hackmotion/device.h"
#include "hackmotion/config.h"
#include "hackmotion/sample.h"
#include "hackmotion/clock.h"
#include "hackmotion/session.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* The container                                                             */
/* ------------------------------------------------------------------------ */
/*
 * A text header so file(1) and a human can identify it, then fixed-size
 * length-prefixed records.  design §5.6 sketched this; the exact layout is:
 *
 *     HMWIRE1\n
 *     device_id=<opaque>\n
 *     config=0x7e\n
 *     layout_version=1\n
 *     clock=monotonic_us\n
 *     byte_order=little\n
 *     identifiers=redacted\n
 *     \n
 *     <record><record>...
 *
 *     offset  size  field
 *       0       4   u32  length         payload bytes after this 20-byte header
 *       4       1   u8   direction      hm_wire_direction
 *       5       1   u8   flags          hm_wire_flag
 *       6       2   u16  reserved       written zero, ignored on read
 *       8       4   u32  sequence       the core's own chunk sequence
 *      12       8   i64  host_time_us
 *      20     len   payload
 *
 * ⚠ `sequence` is 4 bytes the design's sketch did not have, and it is here for
 * a reason rather than for symmetry: HM_WIRE_LOST says chunks were dropped
 * before this one but not HOW MANY, and a reader that renumbered from its own
 * ordinal would silently turn a lossy recording into a complete-looking one.
 * §5.6 has been amended to match.
 *
 * ⚠ Integers are LITTLE-ENDIAN, which is the opposite of the protocol inside
 * the payload — that is deliberate and it is why `byte_order=` is in the header
 * rather than assumed.  spec §1 says byte order is not uniform across this
 * protocol, and a container that left its own order to be guessed would be one
 * more place to guess.  The codec's big-endian readers are for the payload; the
 * container's own fields never go near them.
 */
#define HM_RECORD_MAGIC          "HMWIRE1"
#define HM_RECORD_HEADER_MAX      1024u  /* text header; refused above this   */
#define HM_RECORD_ENTRY_HEADER      20u

/* The `clock=` field.  The library never reads a clock, so this records which
 * one the HOST used — and a wall clock is a defect, not a variant (types.h). */
#define HM_RECORD_CLOCK_MONOTONIC "monotonic_us"
#define HM_RECORD_CLOCK_MAX         32

typedef struct hm_recording_info {
    /* Opaque, caller-chosen.  ⚠ Never a MAC (api-request §2.0.4). */
    char     device_id[HM_DEVICE_ID_MAX];

    /* The `a0 01 <cfg>` byte in force when the recording opened.  A stream
     * restart under a different configuration is recoverable from the recorded
     * write itself — which is the point of recording bytes. */
    uint8_t  config_bits;
    uint8_t  config_legacy;   /* the bare `82` start; carries no device clock  */
    uint8_t  reserved[2];

    uint32_t layout_version;  /* HM_SAMPLE_LAYOUT_VERSION at write time        */
    char     clock[HM_RECORD_CLOCK_MAX];

    /*
     * ⚠ Whether MAC (0x85) and serial (0x86) replies were kept.  Redaction is
     * the default and is marked per chunk with HM_WIRE_REDACTED so a reader
     * knows something was REMOVED rather than absent (design §5.6).
     */
    bool     identifiers_recorded;
} hm_recording_info;

/* Fills in the defaults: no device id, 0x7e, this layout version, monotonic,
 * identifiers redacted. */
HM_API hm_recording_info hm_recording_info_default(void);

/* --- Writer ------------------------------------------------------------- */
typedef struct hm_recorder hm_recorder;

/*
 * Creates or truncates `path` and writes the header.  Returns HM_ERR_INVALID_ARG
 * on a NULL argument, HM_ERR_NO_MEMORY, or HM_ERR_INVALID_STATE if the file
 * cannot be opened.
 */
HM_API hm_status hm_recorder_open(const char *path, const hm_recording_info *info,
                                  hm_recorder **out_recorder);

/* Appends `count` chunks in order.  A chunk whose length exceeds
 * HM_WIRE_CHUNK_MAX is refused with HM_ERR_INVALID_ARG and nothing is written. */
HM_API hm_status hm_recorder_write(hm_recorder *recorder, const hm_wire_chunk *chunks,
                                   size_t count);

HM_API uint64_t hm_recorder_chunks(const hm_recorder *recorder);
HM_API uint64_t hm_recorder_bytes(const hm_recorder *recorder);

/* Flushes, closes and frees.  Returns the first write error the recorder saw,
 * so a full disk cannot end a capture quietly. */
HM_API hm_status hm_recorder_close(hm_recorder *recorder);

/* --- Reader ------------------------------------------------------------- */
typedef struct hm_replay hm_replay;

/*
 * Opens and parses the header.  Refuses a wrong magic (HM_ERR_MALFORMED) and a
 * header longer than HM_RECORD_HEADER_MAX (HM_ERR_BUFFER_TOO_SMALL).
 *
 * ⚠ An UNKNOWN KEY in the header is ignored, not an error — the same rule spec
 * §5.1 gives for unknown message ids, and for the same reason: a recording
 * written by a later version must stay readable up to the part this version
 * understands.  A key it needs and cannot parse is still an error.
 */
HM_API hm_status hm_replay_open(const char *path, hm_replay **out_replay);

HM_API const hm_recording_info *hm_replay_info(const hm_replay *replay);

/*
 * The next chunk, byte-exact.  HM_OK, HM_DONE at end of file, HM_ERR_TRUNCATED
 * if the file ends mid-record, or HM_ERR_MALFORMED if a record claims a length
 * above HM_WIRE_CHUNK_MAX.
 *
 * ⚠ That last one is a refusal, not a clamp.  This is the one place a corrupt
 * or hostile file reaches a fixed-size buffer, and CONTRIBUTING.md invites
 * fuzzing exactly here.
 */
HM_API hm_status hm_replay_next(hm_replay *replay, hm_wire_chunk *out_chunk);

HM_API uint64_t hm_replay_chunks_read(const hm_replay *replay);
HM_API void     hm_replay_close(hm_replay *replay);

/* ------------------------------------------------------------------------ */
/* Reconciliation                                                            */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ "NO EVIDENCE" IS NOT "AGREEMENT".  A zero mismatch count out of zero
 * samples reads as success and is the failure this whole struct is shaped
 * against.  Every measured quantity below is an hm_stat carrying its own `n`,
 * every verdict has a distinct HM_CHECK_NO_EVIDENCE value, and the printed
 * report never states an estimate without the count behind it.
 */
typedef enum hm_check_verdict {
    HM_CHECK_NO_EVIDENCE = 0,  /* ⚠ nothing measured — NOT agreement           */
    HM_CHECK_MATCH,            /* within the specification's own stated spread */
    HM_CHECK_DIFFERS,          /* measured, and outside it — read the numbers  */
    HM_CHECK_NOTE              /* observed; the specification offers no figure */
} hm_check_verdict;

HM_API const char *hm_check_verdict_name(hm_check_verdict verdict);

/* Streaming mean/spread.  `sd` is the sample standard deviation, 0 when n < 2. */
typedef struct hm_stat {
    uint64_t n;
    double   mean;
    double   m2;     /* Welford's running sum of squared deviations */
    double   min;
    double   max;
} hm_stat;

HM_API void   hm_stat_reset(hm_stat *stat);
HM_API void   hm_stat_add(hm_stat *stat, double value);
HM_API double hm_stat_sd(const hm_stat *stat);

/* Index-step buckets, spec §6.6's own table.  The dense modes are the ones a
 * client gets wrong: 100 Hz is NOT the live ceiling. */
typedef enum hm_step_bucket {
    HM_STEP_NONPOSITIVE = 0, /* a repeat or a regression — should never happen */
    HM_STEP_DENSE,           /* 1-7:  bursts, present in EVERY session with motion */
    HM_STEP_8,               /* the nominal 100 Hz decimation                  */
    HM_STEP_9_31,
    HM_STEP_32,              /* the nominal 25 Hz decimation                   */
    HM_STEP_ABOVE_32,        /* a gap: dropped notification or a real pause    */
    HM_STEP_BUCKET_COUNT
} hm_step_bucket;

HM_API const char *hm_step_bucket_name(hm_step_bucket bucket);

/*
 * ⚠ The skew sample store.  §10.3's 59 ticks is claimed STABLE — "identical
 * median across the first and second halves of a 238 s session" — so the
 * reconciliation has to be able to split the session, which means keeping the
 * values rather than one running median.  It is capped, and a capture that hits
 * the cap says so: `skew_stored` below `skew_total` is a truncation, and a
 * truncation nobody reports reads as coverage.
 */
#define HM_RECONCILE_SKEW_MAX 262144u

/*
 * §6.5's claim about the two MCU timers, in ppm.
 *
 * ⚠ It is also the resolution a capture must REACH before it may claim to have
 * tested it.  §6.5's figure came from a 238 s session; the standard error on
 * the same measurement scales with the baseline and is ±1.26 ppm over 30 s.  A
 * capture whose own noise exceeds the claim reports no evidence, not a match —
 * otherwise every short capture "confirms" a number it could not have
 * contradicted.
 */
#define HM_ONE_RATE_CLAIM_PPM 2.0

typedef struct hm_reconcile_report {
    hm_recording_info info;

    /* --- provenance ---------------------------------------------------- */
    /* ⚠ Which configuration actually produced these frames.  Taken from the
     * recorded `a0 01 <cfg>` write where there is one, because that byte is on
     * the wire and the file header is only a claim about it. */
    hm_stream_config config;
    bool             config_from_stream;   /* false → fell back to the header */

    /* --- census -------------------------------------------------------- */
    uint64_t   chunks;
    uint64_t   host_writes;
    uint64_t   device_notifications;
    uint64_t   meta_chunks;
    uint64_t   chunks_after_loss;          /* carried HM_WIRE_LOST            */
    uint64_t   redacted_chunks;
    hm_time_us first_us, last_us, duration_us;
    uint64_t   message_count[256];         /* by device→host message id       */
    uint64_t   unknown_messages;
    uint64_t   decode_errors;
    uint32_t   codec_warnings;             /* OR over every decode (event.h)  */

    /* --- §6.3 framing --------------------------------------------------- */
    /* ⚠ Sized from the CONFIGURATION, not from the constants 47 and 93.  Bit 5
     * changes the block size, so a `5e` capture is 43/85 and a legacy `0x7f`
     * capture is 43 with one record only (§6.2, §6.3.1).  A checker that pinned
     * 47/93 would report every non-default capture as broken framing. */
    size_t   expected_len_one_record, expected_len_two_records;
    uint64_t notif_one_record, notif_two_records, notif_other_len;
    uint64_t records_live, records_history;

    /* --- §6.4 the quaternion norm, the cheapest structural check --------- */
    hm_stat  quat_norm[HM_UNIT_COUNT];
    uint64_t quat_norm_suspect;

    /* --- §6.6 the live rate is a continuum, not two states --------------- */
    uint64_t step[HM_STEP_BUCKET_COUNT];
    uint64_t steps_total;
    /*
     * ⚠ The step buckets lump 1-7 together as "dense", which HIDES the only
     * thing that proves the full internal rate was actually delivered: a RUN of
     * consecutive step-1 records.  §6.6 cites "eight consecutive steps of 1,
     * nine records at adjacent internal samples" as its evidence, so the run
     * length is the measurement, not the count.
     */
    uint32_t live_max_adjacent_run;      /* records, not steps: run + 1 */
    double   live_adjacent_peak_gyro_dps;
    double   dense_fraction;               /* HM_STEP_DENSE / steps_total     */
    uint32_t index_regressions;

    /* --- §6.5 the internal rate, fitted and never assumed ---------------- */
    hm_clock_snapshot fit;
    double   fitted_rate_hz;               /* 0 when the fit has nothing      */
    double   ppm_vs_nominal;               /* against 799.2                   */
    double   ppm_vs_800;                   /* ⚠ the cost of the round number  */

    /* --- §6.5 ticks per sample, per unit --------------------------------- */
    /*
     * ⚠ LEAST SQUARES OVER THE WHOLE STREAM, not hm_tick_unwrapper's own
     * `ratio`.  That one is a two-endpoint fit refitted on doubling, and it
     * exists to predict a tick value well enough to pick the right wrap out of
     * a ±32,768 budget — a job a 350 ppm error would still do perfectly.  Using
     * it as a precision instrument reported the two units 17.21 ppm apart on a
     * capture where they are 0.43 ppm apart, i.e. it manufactured a
     * specification violation out of an estimator that was working as designed.
     */
    double   ticks_per_index[HM_UNIT_COUNT];
    bool     tick_ratio_fitted[HM_UNIT_COUNT];
    uint64_t tick_fit_n;
    double   tick_rate_hz;                  /* ratio × fitted rate; ≈64,068   */

    /*
     * §6.5: "Both blocks of a record agree on it to within 2 ppm, confirming
     * the two MCU timers run at one rate."
     *
     * ⚠ Measured as the SLOPE OF (palm − arm) AGAINST INDEX, which is zero iff
     * the two run at one rate.  Differencing two independently fitted slopes
     * would subtract two noisy numbers; this cancels every common-mode term
     * before the fit instead of after it, and it is the same quantity the skew
     * stability below reports.
     *
     * ⚠ AND ITS STANDARD ERROR, because a ppm figure without one cannot test a
     * 2 ppm claim.  Measured at ±1.26 ppm over 30 s — so a short capture is
     * *unable* to test §6.5, and says so rather than passing.
     */
    double   rel_rate_ppm;
    double   rel_rate_ppm_sigma;
    bool     rel_rate_measured;

    /* ⚠ ±32768 is the budget and 0 is a coin toss, so an UNMEASURED margin
     * cannot be reported as 0 — it would read as the worst possible result.
     * This is the unwrapper's OWN health, which is what its ratio is for. */
    uint32_t worst_wrap_margin[HM_UNIT_COUNT];
    bool     wrap_margin_measured[HM_UNIT_COUNT];

    /* --- §10.3 inter-unit skew, palm − lower_arm ------------------------- */
    hm_stat  skew_ticks;
    uint64_t skew_total, skew_stored;       /* unequal ⇒ the store truncated  */
    double   skew_median_ticks;
    double   skew_median_first_half;
    double   skew_median_second_half;
    double   skew_median_us;

    /* --- §6.3 the physical check on which block is which ------------------ */
    /* Under rotation the palm sits at the larger radius and reads 31-51 m/s²
     * MORE.  Measured only where the motion is fast enough to separate them. */
    hm_stat  accel_mag_mps2[HM_UNIT_COUNT];
    hm_stat  accel_palm_minus_arm_fast;
    double   fast_gyro_threshold_dps;
    hm_stat  gyro_mag_dps[HM_UNIT_COUNT];
    double   gyro_peak_dps[HM_UNIT_COUNT];
    double   gyro_peak_fraction_of_full_scale;

    /* --- §6.4 pinned samples: int16 saturation is silent ------------------ */
    hm_pinned_counts pinned;

    /* --- §9.1 bring-up ---------------------------------------------------- */
    uint8_t  bringup[16];
    size_t   bringup_len;
    bool     bringup_matches_vendor;

    /* --- §9.2 the keepalive is mandatory ---------------------------------- */
    uint64_t   status_polls;
    hm_time_us max_host_write_gap_us;      /* ⚠ 300 s is the device's deadline */

    /* --- §6.1 start latency ----------------------------------------------- */
    hm_time_us stream_start_latency_us;    /* HM_TIME_UNKNOWN when unmeasured  */

    /*
     * ⚠ ARE THE ARRIVAL TIMESTAMPS IN THIS CAPTURE TRUSTWORTHY AT ALL?
     *
     * The longest run of live frames each arriving less than 2 ms after the
     * one before.  A few is ordinary BLE bunching within a connection event;
     * dozens means the host stalled and then dispatched a backlog, and every
     * frame in that burst carries the instant the host recovered rather than
     * the instant it arrived.
     *
     * ⚠ This is here because the cost was paid.  A capture whose harness
     * blocked its own event loop for 4.08 s had ~13 s of fabricated arrival
     * times.  The bug was found, measured and fixed — and then an analysis was
     * run over the SAME capture that matched calibration poses BY ARRIVAL
     * TIME, inside exactly that window, and got the two poses the wrong way
     * round.  Fixing a tool does not fix the data it has already produced.
     *
     * Anything keyed on when a frame arrived must be done in DEVICE time —
     * the sample index (§10.2) — whenever this count is large.
     */
    uint32_t max_frame_burst;
    /*
     * ⚠ The counter resets at every start but the crystal does not (§10), so
     * the fit above is PER STREAM while `records_live` counts them all.  With
     * more than one start in a capture, `fit` describes the last stream only
     * and the report says so rather than letting the two be read together.
     */
    uint32_t   stream_starts;

    /* --- §7 history ------------------------------------------------------- */
    uint32_t brackets_opened, brackets_closed;
    uint64_t device_errors;                /* 0xd0                             */
    uint64_t buttons;                      /* 0xfb                             */

    /*
     * ⚠ THE BUFFER IS MOTION-ADAPTIVE (§7.3), AND A RETRIEVAL THAT LOOKS
     * BROKEN IS USUALLY JUST A RETRIEVAL OVER A STATIONARY WRIST.
     *
     * Index step 8 (≈100 Hz) at rest, step 1 (the full ≈799.2 Hz) in fast
     * motion, measured across 25 retrievals and 17,739 steps.  A sensor on a
     * desk therefore replays evenly at one-in-eight with no error and nothing
     * on the wire to say so — correct behaviour, indistinguishable from a
     * broken full-rate path unless the angular rate is reported beside it.
     *
     * So the density is never reported alone.  index 0..8 counts each step,
     * [9] counts anything above 8 — which §7.3 says was never once observed.
     */
    uint64_t history_step_count[10];
    uint32_t history_modal_step;
    /* ⚠ The same measurement on the path the library's premise rests on. */
    uint32_t history_max_adjacent_run;
    hm_stat  history_gyro_dps;
    double   history_peak_gyro_dps;

    /*
     * ⚠ True only if some part of a retrieval came back dense WHILE MOVING
     * FAST — the one thing that tells a consumer its full-rate path works.
     * False means the capture did not test it, which is not the same as the
     * path being broken and is not the same as it being fine.
     */
    bool     history_exercised_full_rate;

    /*
     * ⚠ §7.5 CLAIMS A MID-STREAM RETRIEVAL COSTS NO RECORDING GAP.  MEASURED,
     * IT DOES.
     *
     * "A client that must stop to retrieve has a window in which the device
     * records nothing ... Issuing `a1` in place removes that window entirely."
     * Across six retrievals in one capture the sample counter advanced by 4-35
     * indices while the MCU tick timer advanced ~22,400 — about 350 ms, the
     * bracket's own duration.  The counter stalls; the timer does not.
     *
     * Both figures come from the same frames, so this is a device property and
     * not a host-timing artefact.  Excluding the six live pairs that straddle a
     * bracket moves ticks-per-index from 83.3988 to 80.1407 against §6.5's
     * ~80.14, and the whole-session rate from 768 Hz to 801.
     *
     * Two consequences the library must carry, and neither is cosmetic:
     *   - the index→host-time mapping gains a TIME OFFSET at every retrieval,
     *     so one line per stream is wrong across a pull;
     *   - the stall eats the tick-unwrap margin, which fell from 31,487 to
     *     9,372 of ±32,768 — §10.2's 8.4% of budget became 71%.
     *
     * ⚠ Measured from RAW tick differences, so it does not depend on the wrap
     * resolution whose stress it is reporting.  Valid only while a stall stays
     * under the 1.023 s tick wrap.
     *
     * ⚠ The figure is INCLUSIVE of the one live step that carries it — the
     * frames either side of the bracket are an ordinary live step apart, worth
     * 10-40 ms — so it overstates the stall by that much.  Stated here rather
     * than corrected for, because the correction would need the step to be
     * known and the raw measurement is the honest one.
     */
    uint32_t retrievals_measured;   /* brackets with live frames on both sides */
    uint32_t retrievals_stalled;
    hm_stat  retrieval_stall_ms;       /* recording time lost, per pull        */
    hm_stat  retrieval_indices_lost;
    /*
     * ⚠ THE RULE IS THAT THE GAP EQUALS THE PULL, at any size — not that small
     * pulls are exempt.  Measured 90-99% of the pull's own duration across a
     * 16× size range (25 ms to 406 ms), so a detector thresholding on absolute
     * shortfall reports a 25 ms pull as healthy and gets the count wrong.  This
     * is the fraction, and it is what the verdict tests.
     */
    hm_stat  retrieval_stall_fraction; /* lost ÷ the pull's own duration       */

    /*
     * ⚠ CUMULATIVE, because the consequence compounds and the per-pull figure
     * hides it.  A client that anchors once at stream start is out by 111,351
     * ticks — 3.4 whole wraps — after five pulls, and every history record it
     * dates after the first pull is then wrong by a multiple of 1.023 s.
     *
     * One stall is ~23,000 ticks of the ±32,768 that picks the right wrap: 72%,
     * where a pull-free gap uses 12.  Two pulls inside one live-frame gap
     * already exceed the budget (§10.2).
     */
    double   stall_total_ms;
    double   stall_total_ticks;
    double   stall_worst_ticks;

    /* --- verdicts, one per claim the specification actually states -------- */
    hm_check_verdict verdict_frame_length;
    hm_check_verdict verdict_quat_norm;
    hm_check_verdict verdict_rate;
    hm_check_verdict verdict_tick_ratio;
    hm_check_verdict verdict_skew;
    hm_check_verdict verdict_bursts;
    hm_check_verdict verdict_palm_is_second_block;
    hm_check_verdict verdict_bringup;
    hm_check_verdict verdict_keepalive;
    hm_check_verdict verdict_history_rate;
    hm_check_verdict verdict_retrieval_continuity;
} hm_reconcile_report;

typedef struct hm_reconciler hm_reconciler;

/*
 * `info` supplies the fallback configuration for frames that arrive before any
 * `a0 01 <cfg>` write appears in the recording — a capture that started
 * mid-stream, for instance.  Pass hm_replay_info()'s value.
 */
HM_API hm_status hm_reconcile_begin(const hm_recording_info *info,
                                    hm_reconciler **out_reconciler);

/* One chunk, in file order.  Cheap; the whole analysis is streaming. */
HM_API void hm_reconcile_observe(hm_reconciler *reconciler, const hm_wire_chunk *chunk);

/* Closes the accumulators and fills the report.  Idempotent. */
HM_API void hm_reconcile_finish(hm_reconciler *reconciler, hm_reconcile_report *out_report);

HM_API void hm_reconcile_free(hm_reconciler *reconciler);

/*
 * The whole report, one claim per line, each naming its specification section,
 * the expected figure, what was measured and the count behind it.
 *
 * ⚠ Never prints an estimate without its n, and prints "no evidence" rather
 * than a zero that would read as agreement.
 */
HM_API void hm_reconcile_print(const hm_reconcile_report *report, FILE *out);

/* Non-zero if any verdict came back HM_CHECK_DIFFERS — an exit code for a
 * script.  ⚠ HM_CHECK_NO_EVIDENCE is NOT a pass and is reported separately. */
HM_API int hm_reconcile_disagreements(const hm_reconcile_report *report);
HM_API int hm_reconcile_unmeasured(const hm_reconcile_report *report);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_RECORD_H */
