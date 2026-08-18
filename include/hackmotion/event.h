/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/event.h — everything the session has to say, as drained POD.
 *
 * ⚠ THE LIBRARY NEVER INVOKES A CONSUMER CALLBACK.  api-request §2.0.3.
 * Events are queued and the host drains them, which makes the teardown barrier
 * of §2.11 "stop draining" — a guarantee the consumer implements itself and
 * cannot get wrong — instead of something that has to hold across an arbitrary
 * call stack inside this library.
 *
 * Every event is a fixed-size POD with no pointers, so an event survives being
 * copied, queued, logged or sent across a thread boundary.
 */
#ifndef HACKMOTION_EVENT_H
#define HACKMOTION_EVENT_H

#include "hackmotion/types.h"
#include "hackmotion/device.h"
#include "hackmotion/clock.h"
#include "hackmotion/sample.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum hm_event_type {
    HM_EV_NONE = 0,

    /* Link -------------------------------------------------------------- */
    HM_EV_LINK_UP,            /* bring-up has started                         */
    HM_EV_LINK_DOWN,          /* with a classification — see hm_link_down     */
    HM_EV_MTU_REJECTED,       /* ⚠ negotiated MTU < 96; the session refuses to run */
    HM_EV_READY,              /* bring-up complete; safe to start a stream    */

    /* Device information ------------------------------------------------ */
    HM_EV_DEVICE_INFO,        /* 0x80 versions + 0x84 sensor map              */
    HM_EV_BATTERY,            /* 0x81 — arrives every 30 s as a side effect   */
    HM_EV_IDENTITY,           /* ⚠ 0x85 MAC / 0x86 serial — personal data     */

    /* Stream ------------------------------------------------------------ */
    HM_EV_STREAM_STARTED,     /* first 0x90 frame seen, NOT the a0 01 ack     */
    HM_EV_STREAM_STOPPED,
    HM_EV_STREAM_RESTARTED,   /* ⚠ reported, never silent — api-request B8    */

    /* Calibration ------------------------------------------------------- */
    HM_EV_CALIBRATION_PHASE,  /* the observable state machine, for a UI       */
    HM_EV_CALIBRATION_PRESENCE, /* the reference-pose angle — presence, NOT quality */

    /* History ----------------------------------------------------------- */
    HM_EV_HISTORY_STARTED,    /* `a1 02` accepted — the acceptance test       */
    HM_EV_HISTORY_PROGRESS,
    HM_EV_HISTORY_READY,      /* a block is collectable                       */
    HM_EV_HISTORY_BLIND_SPAN, /* live delivery was suspended for a retrieval  */
    HM_EV_HISTORY_EVICTION_RISK, /* a queued request may not survive the wait */

    /* Clock ------------------------------------------------------------- */
    HM_EV_CLOCK_UPDATED,      /* periodic; carries the current snapshot       */
    HM_EV_CLOCK_DEGRADED,     /* ⚠ residual spread widened, or fit degenerate */

    /* Device -> host oddments ------------------------------------------- */
    HM_EV_BUTTON,             /* 0xfb — an edge HINT; do not build a counter  */
    HM_EV_DEVICE_ERROR,       /* 0xd0 — one byte, and it means nothing specific */
    HM_EV_UNKNOWN_MESSAGE,    /* logged and ignored, per §5.1                 */
    HM_EV_PINNED_SAMPLES,     /* ⚠ silent int16 saturation was observed       */
    HM_EV_WARNING,

    HM_EVENT_TYPE_COUNT
} hm_event_type;

/* ------------------------------------------------------------------------ */
/* Link                                                                      */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ "LINK DROPPED" AND "DEVICE SLEPT" ARE DIFFERENT STATES (spec §9.6).
 *
 * A dropped link is an ordinary supervision timeout and reconnect usually
 * succeeds.  A slept or powered-off device needs A PHYSICAL BUTTON PRESS AND
 * NOTHING ELSE — retrying against it cannot succeed at any interval, burns host
 * battery, occupies the adapter, and shows the user a spinner for a problem
 * only they can fix.
 *
 * The library does not own the radio, so it does not retry.  It CLASSIFIES,
 * and the host acts.  The discriminators — how long the connection was idle,
 * whether the disconnect was clean, whether the device is still advertising —
 * are available to the library and to nobody above it.
 */
typedef enum hm_link_down_cause {
    HM_LINK_DOWN_UNKNOWN = 0,
    HM_LINK_DOWN_LOCAL_REQUEST,     /* we asked for it                         */
    HM_LINK_DOWN_SUPERVISION_TIMEOUT,
    HM_LINK_DOWN_REMOTE_CLOSED,
    HM_LINK_DOWN_TRANSPORT_ERROR,
    HM_LINK_DOWN_ADAPTER_GONE,
    /* Not a transport cause; the transport may report it if it knows. */
    HM_LINK_DOWN_CONNECTION_TAKEN   /* ⚠ the device accepts ONE connection and the
                                     * vendor app wins the race if it is running
                                     * (§2.2).  "Another application is using this
                                     * sensor" is a thing a user can act on, and a
                                     * retry loop against it is pure waste. */
} hm_link_down_cause;

typedef enum hm_recovery_advice {
    HM_RECOVER_UNKNOWN = 0,
    HM_RECOVER_RECONNECT_WITH_BACKOFF, /* ordinary link loss                    */
    HM_RECOVER_NEEDS_BUTTON_PRESS,     /* ⚠ do NOT retry — only the user can fix */
    HM_RECOVER_NEEDS_OTHER_APP_CLOSED,
    HM_RECOVER_DO_NOT_RETRY            /* we powered it off on purpose           */
} hm_recovery_advice;

typedef struct hm_link_down_event {
    uint8_t    cause;               /* hm_link_down_cause   */
    uint8_t    advice;              /* hm_recovery_advice   */
    uint8_t    calibration_invalidated; /* ⚠ always 1 — §8.3 */
    uint8_t    reserved;
    hm_time_us connected_for_us;
    hm_time_us since_last_device_byte_us;
    int64_t    suggested_retry_delay_us; /* 0 when advice says do not retry */
} hm_link_down_event;

/* ------------------------------------------------------------------------ */
/* Calibration                                                               */
/* ------------------------------------------------------------------------ */
typedef enum hm_calibration_phase {
    HM_CALP_IDLE = 0,
    /* ⚠ There is deliberately no AWAIT_STREAM phase.  §8.2: calibration is not
     * a standalone transaction — the device observes a continuous raise between
     * the markers — so hm_calibration_begin() REFUSES with HM_ERR_NO_STREAM
     * rather than entering a waiting state.  Under the one-stream cycle the
     * stream is open from just after HM_EV_READY and stays open (AR B8), so a
     * wait would never be entered anyway, and an unreachable phase is worse
     * than no phase. */
    HM_CALP_AWAIT_HORIZONTAL,  /* forearm horizontal, wrist straight          */
    HM_CALP_MARKING_POSE0,     /* `a2 00` written, awaiting `a2 01` reply     */
    HM_CALP_OBSERVING_RAISE,   /* the device is watching a CONTINUOUS raise   */
    HM_CALP_MARKING_POSE1,     /* `a2 01` written, awaiting `a2 01` reply     */
    HM_CALP_APPLYING,          /* awaiting `0x94`                             */
    HM_CALP_VERIFYING,         /* ⚠ transform APPLIED; presence not yet measured */
    /*
     * ⚠ COMPLETE MEANS THE DEVICE APPLIED THE TRANSFORM.  It does NOT mean the
     * calibration was checked, and it never means it was good: `0x94` is not a
     * verdict (§8.2).  Read hm_session_calibration_state() for what is actually
     * known — HM_CAL_UNKNOWN when the presence check was skipped or could not be
     * taken, and only HM_CAL_CALIBRATED when one was measured and passed.
     */
    HM_CALP_COMPLETE,
    HM_CALP_ABORTED,           /* a real state, not an error string           */
    HM_CALIBRATION_PHASE_COUNT
} hm_calibration_phase;

typedef enum hm_calibration_abort_reason {
    HM_CAL_ABORT_NONE = 0,
    /*
     * hm_calibration_abort().  ⚠ Also carried on a transition to COMPLETE, not
     * only to ABORTED: aborting at HM_CALP_VERIFYING declines the presence check
     * on a transform the device has ALREADY applied, and reporting that as
     * ABORTED would claim nothing happened.  See hm_calibration_abort().
     */
    HM_CAL_ABORT_CALLER,
    HM_CAL_ABORT_RAISE_TOO_SLOW,   /* CLIENT policy — the device imposes no deadline */
    HM_CAL_ABORT_STREAM_LOST,
    HM_CAL_ABORT_LINK_LOST,
    /*
     * The device did not answer within `policy.calibration_result_timeout_us` —
     * either a marker's `a2 01` reply or the `0x94` result.  ⚠ Which of the two
     * is in `previous_phase` on the event, so the one bound does not need two
     * reasons to stay legible.
     */
    HM_CAL_ABORT_NO_RESULT
} hm_calibration_abort_reason;

typedef struct hm_calibration_phase_event {
    uint8_t    phase;          /* hm_calibration_phase */
    uint8_t    previous_phase;
    uint8_t    abort_reason;   /* hm_calibration_abort_reason */
    uint8_t    reserved;
    hm_time_us elapsed_us;     /* since the routine began */
} hm_calibration_phase_event;

/*
 * ⚠ THIS IS A PRESENCE CHECK.  IT IS NOT A QUALITY SCORE, AND IT INVERTS.
 *
 * Spec §8.2, measured — three calibrations, one session, one mounting, each
 * read at the same reference pose:
 *
 *     correct routine (horizontal, then raised across the chest)   1.96°
 *     raise about the WRONG AXIS (straight up)                     6.10°
 *     NO RAISE AT ALL (pose 1 marked without moving)               0.70°
 *
 * The calibration carrying no axis information at all scored BEST, and not by
 * accident: the two-pose routine does two separate things — pose 0 ZEROES the
 * wrist, and the raise between the poses FINDS THE AXIS that separates flexion
 * from deviation.  This figure tests only the zeroing, and with nothing moving
 * between the markers the zero is perfect by construction while the anatomical
 * frame is left entirely undetermined.
 *
 * A library that selected between attempts on this number would systematically
 * prefer the worst one available.  So: never rank on it, never present it as a
 * score.  Its one sound use is catching "calibration never happened, or was
 * lost", where the gap is an order of magnitude (0.4-3.8° applied vs 15° after
 * a power cycle and 18.8° after a plain disconnect) — a failure that is
 * otherwise silent, permanent, and reachable mid-session without anyone doing
 * anything wrong.
 */
/* Classification thresholds.  ⚠ These are PRESENCE boundaries and must never
 * become quality boundaries — see above.  Drawn from §8.2's applied 0.36-3.80°
 * against uncalibrated 15.01° and post-disconnect 18.80°, so they sit in an
 * order-of-magnitude gap and have enormous margin. */
#define HM_PRESENCE_CALIBRATED_MAX_DEG  6.0f
#define HM_PRESENCE_ABSENT_MIN_DEG     10.0f

/* Longest run of live samples the presence check will consider. */
#define HM_PRESENCE_MAX_SAMPLES 64u

typedef struct hm_calibration_presence_event {
    float   relative_angle_deg;   /* at the reference pose */

    /*
     * ⚠ THE REFERENCE-POSE ANCHOR, IN TWO FORMS.  Both are kept because they
     * answer different questions and neither can be recovered once the pose has
     * passed.
     *
     * Why the library keeps them at all: the device applies calibration itself
     * and hands back quaternions in ITS anatomical frame (§8.1), and the
     * 64-byte result is deliberately undecoded.  A consumer with its own
     * anatomical convention has to solve the constant rotation between the two
     * frames empirically, and a pair at a pose the APPLICATION declared known is
     * exactly the anchor that solve needs.  Capturing it separately from the
     * live stream would anchor on an instant only NEARLY the one measured here,
     * and "nearly" becomes a fixed rotation error in every later reading.
     *
     * This is not a decomposition and not a frame choice.  It is the raw
     * measurement the library already took, kept instead of thrown away.
     */

    /*
     * (1) THE MEDOID RECORD — one real measured pair.
     *
     * ⚠ NOT the record whose relative angle is the median of the run.  It is
     * the record whose relative ROTATION is nearest the run's average rotation.
     * The distinction is load-bearing and hm_presence.c explains it: near zero,
     * a per-sample angle is dominated by Q14 quantisation, and `acos` rectifies
     * that noise so a median of angles biases high.  A run whose angle-median
     * and rotation-medoid are different records is easy to construct, and
     * test_presence.c constructs one.
     *
     * Because it is a real record: the two quaternions came off the wire
     * together, `skew_us` is a measurement rather than an average, and
     * hm_relative_angle_deg() of this pair reproduces `relative_angle_deg`
     * exactly.  Use it for the angle and its provenance.
     */
    float    q_lower_arm[4];      /* world→body, per §6.7 */
    float    q_palm[4];
    uint32_t sample_index;        /* which record the pair came from */
    int32_t  skew_us;             /* palm − lower_arm for that record */

    /*
     * (2) THE AVERAGED POSE — the better estimate of where the arm actually was.
     *
     * ⚠ Use THIS for a frame-reconciliation solve, not the medoid pair.  Two
     * error sources sit on an absolute orientation at the anchor, and
     * quantisation is the small one:
     *
     *     Q14 quantisation of an absolute rotation   ~0.007° per component
     *     a person holding a declared pose still     ~0.5-2°
     *
     * The wobble is one to two orders larger, and averaging over the run is
     * what removes it.  ⚠ The medoid cannot: it is selected on the RELATIVE
     * rotation, which is blind to a whole-arm movement that carries both units
     * together — precisely the motion that contaminates an absolute pose.  So
     * however centrally the medoid is chosen, it still holds whatever the
     * athlete was doing at that instant.
     *
     * Sign-aligned and normalised the same way as the relative average.  Over a
     * run of at most HM_PRESENCE_MAX_SAMPLES the heading drift of §6.2
     * (0.11-0.16°/min per unit) contributes under 0.01° and is ignored.
     */
    float    q_lower_arm_mean[4];
    float    q_palm_mean[4];

    /*
     * How still the pose actually was: the largest angular distance, in degrees,
     * between any sample's absolute rotation and the mean above, per unit
     * ([HM_UNIT_LOWER_ARM], [HM_UNIT_PALM]).
     *
     * ⚠ A mean without a spread is an estimate without evidence.  This is the
     * number that says whether the athlete held the pose or drifted through it,
     * and a consumer about to bake this anchor into every future reading should
     * look at it before trusting the mean.  The samples are gone afterwards, so
     * nothing else can reconstruct it.
     */
    float    pose_spread_deg[2];

    uint8_t state;                /* hm_calibration_state */
    uint8_t samples_used;
    uint8_t reserved[2];
} hm_calibration_presence_event;

/* ------------------------------------------------------------------------ */
/* History                                                                   */
/* ------------------------------------------------------------------------ */
typedef struct hm_history_progress_event {
    uint64_t request_id;
    uint32_t records_delivered;
    float    fraction;            /* of the requested index span, [0,1] */
    hm_time_us elapsed_us;
} hm_history_progress_event;

typedef struct hm_history_blind_span_event {
    uint64_t   request_id;
    hm_time_range span;           /* host time over which live delivery was suspended */
} hm_history_blind_span_event;

typedef struct hm_history_eviction_risk_event {
    uint64_t   request_id;
    hm_time_us queued_for_us;
    hm_time_us estimated_eviction_in_us;  /* may be negative: already at risk */
} hm_history_eviction_risk_event;

/* ------------------------------------------------------------------------ */
/* Misc payloads                                                             */
/* ------------------------------------------------------------------------ */
typedef struct hm_stream_event {
    uint64_t stream_id;
    uint8_t  config_bits;
    uint8_t  reserved[7];
} hm_stream_event;

typedef struct hm_battery_event {
    uint8_t  percent;
    uint8_t  reserved;
    uint16_t status_undecoded;    /* observed 2226-2235.  NOT millivolts. */
} hm_battery_event;

typedef struct hm_device_error_event {
    uint8_t code;  /* ⚠ only 0x03 has ever been seen, and it means nothing
                    * specific: measured across seven distinct causes — no data
                    * yet, malformed request, evicted range — every one returned
                    * d0 03.  The library classifies from its OWN state; no wire
                    * code can do it (spec §7.2). */
    uint8_t reserved[3];
    uint64_t request_id;          /* 0 if unsolicited */
} hm_device_error_event;

typedef struct hm_unknown_message_event {
    uint8_t message_id;
    uint8_t length;
    uint8_t first_bytes[6];       /* for a log line; the wire log has the rest */
} hm_unknown_message_event;

typedef struct hm_pinned_event {
    hm_pinned_counts counts;
    uint64_t         over_samples; /* how many samples the counts cover */
} hm_pinned_event;

typedef enum hm_warning_code {
    HM_WARN_NONE = 0,
    HM_WARN_SHORT_FRAME,
    /* ⚠ A notification whose length is not a whole number of messages.  Under
     * the one-call-one-notification contract this should never happen, so it is
     * the signature of a coalescing transport and is meant to be loud. */
    HM_WARN_TRAILING_BYTES,
    /*
     * More records in one notification than has ever been observed.  Decoded
     * and reported rather than silently truncated.
     *
     * ⚠ NOT a claim that the frame is malformed.  §6.3: the encoding puts NO
     * upper bound on records per notification — one or two is what fits a
     * 96-byte MTU, not a limit of the format — so a third is unobserved rather
     * than illegal, and it is decoded like any other.  The warning says the
     * stream has left the ground this specification was measured on.
     */
    HM_WARN_UNEXPECTED_RECORD_COUNT,
    HM_WARN_QUAT_NORM,             /* structural check failed — decode may be misaligned */
    HM_WARN_INDEX_REGRESSION,
    HM_WARN_TICK_PREDICTION_MARGIN,/* wrap resolution getting close to ambiguous */
    /* ⚠ No host→device write for policy.keepalive_alarm_us (120 s default)
     * against a 300 s idle shutdown — several polls missed, deadline in view. */
    HM_WARN_KEEPALIVE_LATE,
    /* No live frame for policy.live_gap_alarm_us (3 s default) while streaming.
     * ⚠ Suppressed inside a history bracket, where live is suspended by design. */
    HM_WARN_LIVE_GAP,
    /* `a0 01 7e` acknowledged but no 0x90 within policy.stream_start_timeout_us
     * (3 s default, against a measured 50-80 ms — §6.1). */
    HM_WARN_STREAM_START_TIMEOUT,
    /*
     * ⚠ `83` was queued and the device never answered it.  The same bound as the
     * start watchdog, from the same measurement, and it exists because STOPPING
     * was the ONE device-facing wait in design §5.7 that had no way out
     * (implementation-review I9).
     *
     * A device that drops the reply — or a host that stops draining
     * poll_writes(), so the `83` never leaves — left the session in STOPPING for
     * ever: start_stream() refused with HM_ERR_INVALID_STATE, every calibration
     * call refused with HM_ERR_NO_STREAM, history never issued, and nothing said
     * why.  The session now stops locally and reports this instead.
     */
    HM_WARN_STREAM_STOP_TIMEOUT,
    /*
     * ⚠ NOT "the reply had gaps" — every reply has gaps and they are not an
     * error (§7.3, design §8.7).  This fires only where the holing CANNOT be
     * the motion-adaptive buffer: §7.3 measured index step 8 as a hard floor
     * across 17,739 steps, so a gap wider than that is a delivery failure
     * rather than a still wrist.  `detail_i32` is the largest gap in indices.
     *
     * Warning on every reply would cry wolf on the normal shape of the data,
     * which is how a real one stops being read.  The block's `status`,
     * `delivered[]` and `density` carry the ordinary holing — and `density` is
     * the one that names it: an at-rest reply reads 0.125, one delivered index
     * in eight, which is this floor stated as a number rather than a warning.
     */
    HM_WARN_HISTORY_HOLED,
    /* The delivered set ran out before the requested range did — evidence
     * about how far back the buffer actually reaches.  `detail_i32` is the
     * number of requested indices that never arrived. */
    HM_WARN_HISTORY_SHORT,
    /*
     * ⚠ A record arrived inside our own bracket carrying an index OUTSIDE the
     * range we asked for.  §10.1 measured 4,182 mid-stream records with every
     * one inside the range, so this is a finding rather than a routine case —
     * and it is thrown away rather than mixed in, because live and history
     * frames are byte-identical and a stray record placed on the wrong
     * timeline is a swing aligned seconds off the video.  `detail_i32` counts
     * the records this event stands for.
     */
    HM_WARN_HISTORY_OUT_OF_RANGE,
    /*
     * ⚠ THE TWO HALVES OF §8.5's DEPTH BRACKET CROSSED, AND THAT IS A FINDING
     * ABOUT THE DEVICE RATHER THAN A FAULT IN THE SESSION.
     *
     * The library measures the depth as a bracket: the widest reach-back the
     * device actually served, and the narrowest it demonstrably failed to serve.
     * If the buffer held a fixed DURATION those two could never cross.  They
     * can, because §7.3 leaves open whether it holds a fixed duration or a fixed
     * sample COUNT — and under a fixed count the depth in TIME shrinks by up to
     * 8× when the wrist is moving, which is exactly when a consumer is pulling.
     *
     * `detail_i32` is the crossing in milliseconds (lower bound minus upper).
     * hm_history_resident_range() falls back to the narrower of the two, which
     * is the only claim both observations support.  Nothing is broken; the
     * open question of whether the buffer holds a fixed DURATION or a fixed
     * SAMPLE COUNT (§7.3) has just answered itself, and this is the channel it
     * answers through.
     */
    HM_WARN_HISTORY_DEPTH_CONFLICT,
    /* ⚠ The host clock went BACKWARDS.  The contract requires a monotonic clock
     * (types.h); a wall clock stepped by NTP or DST corrupts a capture in a way
     * that looks like a sensor fault.  The session clamps and continues, but it
     * says so — a consumer cannot fix what it cannot see, and this one is
     * entirely on their side of the boundary. */
    HM_WARN_HOST_CLOCK_REGRESSION,
    HM_WARN_NONSTANDARD_CONFIG,
    HM_WARN_LEGACY_STREAM,         /* 0x7f frames: no device clock, no history */
    /* ⚠ The platform reported a negotiated MTU of 0 — "I will not tell you".
     * §2.4's 96-byte floor is the only thing standing between a 93-byte frame
     * and a truncation that parses as garbage, and here it could not be
     * checked.  Proceeding is right (most stacks that report 0 have negotiated
     * a large MTU anyway) but doing so SILENTLY would put "we did not check" and
     * "we checked and it was fine" in the same recording. */
    HM_WARN_MTU_UNKNOWN,

    /* --- Calibration (§8.2, §8.3) --------------------------------------- */
    /*
     * ⚠ The presence check was ASKED FOR and could not be taken — too few live
     * samples reached the run, or the run held a quaternion that is not a
     * rotation.  `detail_i32` is how many samples were collected.
     *
     * This is the difference between "we checked and it was fine" and "we could
     * not check", which must never end up in the same recording.  The state
     * stays HM_CAL_UNKNOWN and `hm_calibration_presence_angle_deg()` stays NaN.
     */
    HM_WARN_PRESENCE_NOT_MEASURED,
    /*
     * ⚠ The reference-pose angle landed BETWEEN the two populations §8.2
     * measured — above HM_PRESENCE_CALIBRATED_MAX_DEG and below
     * HM_PRESENCE_ABSENT_MIN_DEG.  `detail_f64` carries the angle.
     *
     * It is evidence of neither state, so the calibration state is left exactly
     * as it was.  Guessing here is how a presence check becomes a quality score.
     */
    HM_WARN_CALIBRATION_INDETERMINATE,
    /*
     * The reference-pose angle sits in the uncalibrated population (§8.2:
     * 15.01° after a power cycle, 18.80° after a plain disconnect), so the
     * transform is absent or was lost.  `detail_f64` carries the angle, and the
     * state is driven to HM_CAL_UNCALIBRATED.
     */
    HM_WARN_CALIBRATION_ABSENT,
    /*
     * ⚠ A `0x94` arrived when no calibration was awaiting one — a result later
     * than the client's own bound, or one nothing here asked for.
     * `detail_i32` is the hm_calibration_phase it arrived in.
     *
     * It is reported rather than dropped because the device applies the
     * transform for every `a2 01` (§8.2) and re-references its stream when it
     * emits the result: from that instant the streamed quaternions are in the
     * anatomical frame whatever this library's state machine believes, so the
     * per-sample flag drops to HM_CAL_UNKNOWN with it.
     */
    HM_WARN_CALIBRATION_UNSOLICITED,
    /*
     * ⚠ The SHORT FORM of `0x94` arrived — the device answered the marker with
     * a status byte instead of the eight quaternions (§8.2).  `detail_i32`
     * carries that byte.
     *
     * It has never been seen on the wire, and what its values mean is not
     * known, so the library does not interpret it: the routine proceeds exactly
     * as it does for the long form, and the presence measurement — which tests
     * the device's own output rather than the message — decides. The byte is
     * reported so that it can never be swallowed by a library that could not
     * read it.
     */
    HM_WARN_CALIBRATION_STATUS_FORM,
    /*
     * ⚠ The `0x84` sensor map reports a sensor count this library's record
     * layout cannot decode.  `detail_i32` carries the count.
     *
     * §6.3's record is a header followed by one block PER SENSOR, so the record
     * size depends on the count; every layout this library knows has exactly
     * two.  A different count means the stream is being read at the wrong
     * offsets from the second block onwards, and it is reported when the MAP
     * arrives rather than left to the per-record quaternion-norm check, which
     * would fire on every record without ever naming the cause.
     */
    HM_WARN_SENSOR_COUNT_UNSUPPORTED,
    /*
     * ⚠ The device reports a product id other than HM_PRODUCT_ID_MEASURED — it
     * is not the hardware the specification was measured on.  `detail_i32`
     * carries the reported id.
     *
     * NOT an error, and NOT a refusal.  The protocol may well be identical;
     * "wG3" names a generation and later generations are expected.  What the
     * warning says is narrower and worth saying: every constant this library
     * carries — the ≈799.2 Hz sample rate, the tick rate, the ~7.5 s history
     * depth, the field scales, the meaning of each configuration bit — was
     * established on one product, and none of it has been checked here.
     *
     * It fires once, on the version reply, and lands in the wire log's
     * provenance so a capture says which hardware produced it instead of being
     * re-read years later under constants that never applied to it.
     *
     * ⚠ Feature gating does NOT use this.  The protocol version in the same
     * reply is what says which commands exist (§6.2, §7.1); that is generation-
     * independent and needs no product allowlist.
     */
    HM_WARN_UNVERIFIED_PRODUCT,
    HM_WARN_CODE_COUNT
} hm_warning_code;

typedef struct hm_warning_event {
    uint16_t code;                 /* hm_warning_code */
    uint16_t reserved;
    int32_t  detail_i32;
    double   detail_f64;
} hm_warning_event;

/* ------------------------------------------------------------------------ */
/* The event                                                                 */
/* ------------------------------------------------------------------------ */
typedef struct hm_event {
    uint16_t   type;               /* hm_event_type */
    uint16_t   reserved;
    uint32_t   sequence;           /* monotonic per session; gaps mean the queue overflowed */
    hm_time_us host_time_us;       /* when the session learned this, on the caller's clock */
    uint64_t   stream_id;          /* 0 outside a stream */

    union {
        hm_link_down_event             link_down;
        hm_device_info                 device_info;
        hm_battery_event               battery;
        hm_stream_event                stream;
        hm_calibration_phase_event     calibration_phase;
        hm_calibration_presence_event  calibration_presence;
        hm_history_progress_event      history_progress;
        hm_history_blind_span_event    history_blind_span;
        hm_history_eviction_risk_event history_eviction_risk;
        hm_clock_snapshot              clock;
        hm_device_error_event          device_error;
        hm_unknown_message_event       unknown_message;
        hm_pinned_event                pinned;
        hm_warning_event               warning;
        int32_t                        mtu;         /* HM_EV_MTU_REJECTED: negotiated value */
        /*
         * Deliberate ABI reservation, not a debug escape hatch: it fixes
         * sizeof(hm_event) so a future payload can be added without moving it
         * and without breaking a binding compiled against an older header.
         * 256 leaves ~39 % headroom over the largest real member
         * (hm_clock_snapshot).  Nothing writes through it.
         */
        uint8_t                        reserved_abi[256];
    } u;
} hm_event;

HM_API const char *hm_event_type_name(hm_event_type type);
HM_API const char *hm_warning_code_name(hm_warning_code code);
HM_API const char *hm_calibration_phase_name(hm_calibration_phase phase);
HM_API const char *hm_link_down_cause_name(hm_link_down_cause cause);
HM_API const char *hm_recovery_advice_name(hm_recovery_advice advice);

/*
 * ⚠ SAFE TO LOG?  api-request §2.13: the MAC and serial identify a specific
 * unit and a specific owner.  hm_event_is_sensitive() is true for exactly the
 * events carrying them, so a consumer's log sink can filter without having to
 * think about it, and the library's own default formatting redacts them.
 */
HM_API bool hm_event_is_sensitive(const hm_event *ev);

/* One-line, allocation-free rendering, with identifiers redacted unless
 * `include_identifiers` is true.  Returns the number of bytes that WOULD have
 * been written, snprintf-style. */
HM_API int hm_event_format(const hm_event *ev, char *out, size_t size,
                           bool include_identifiers);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_EVENT_H */
