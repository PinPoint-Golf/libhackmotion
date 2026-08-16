/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/types.h — scalar types, status codes and the time contract.
 *
 * Everything in libhackmotion is C11, POD and free of hidden allocation.
 * See docs/design.md §3 for the type-system rules these headers follow.
 */
#ifndef HACKMOTION_TYPES_H
#define HACKMOTION_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined(_WIN32) && defined(HM_SHARED)
#  if defined(HM_BUILDING_LIBRARY)
#    define HM_API __declspec(dllexport)
#  else
#    define HM_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define HM_API __attribute__((visibility("default")))
#else
#  define HM_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Time                                                                      */
/* ------------------------------------------------------------------------ */
/*
 * THE HOST CLOCK IS THE CALLER'S.  api-request §2.15 C8.
 *
 * Every `int64_t ... _us` in this API is microseconds on a clock the caller
 * chooses and the library never reads.  The library calls no clock function,
 * on any platform, ever — host time only ever enters through the `now_us` and
 * `host_recv_us` arguments the caller supplies.
 *
 * The clock MUST be monotonic.  A wall clock will be stepped by NTP or DST and
 * will corrupt a capture in a way that looks like a sensor fault.  PinPoint
 * Studio uses std::chrono::steady_clock; anything with the same properties
 * works, including a synthetic clock in a test.
 *
 * The epoch is arbitrary and never interpreted.  Only differences matter.
 */
typedef int64_t hm_time_us;

/* Returned wherever a timestamp is structurally unavailable — for example the
 * arrival instant of a history record, which carries no information (§10.1). */
#define HM_TIME_UNKNOWN  INT64_MIN

/* hm_session_next_due_us() returns this when the session has no pending
 * deadline.  A host may sleep indefinitely, waking on transport traffic. */
#define HM_TIME_NEVER    INT64_MAX

/* ------------------------------------------------------------------------ */
/* Status                                                                    */
/* ------------------------------------------------------------------------ */
/*
 * Non-negative values are success.  Negative values are failure.  Test with
 * `if (st < HM_OK)`, never `if (st != HM_OK)` — HM_PENDING is not an error.
 */
typedef enum hm_status {
    HM_OK                  = 0,
    HM_PENDING             = 1,  /* in flight; poll again later               */
    HM_DONE                = 2,  /* nothing further will arrive               */

    HM_ERR_INVALID_ARG     = -1,
    HM_ERR_INVALID_STATE   = -2,
    HM_ERR_NO_MEMORY       = -3,
    HM_ERR_BUFFER_TOO_SMALL= -4,
    HM_ERR_NOT_SUPPORTED   = -5,
    HM_ERR_TRUNCATED       = -6,  /* frame shorter than its message id implies */
    HM_ERR_MALFORMED       = -7,  /* frame is the right length and still wrong */
    HM_ERR_UNKNOWN_MESSAGE = -8,  /* id outside §5.1 — log and ignore, not an error */
    HM_ERR_NOT_ALLOWED     = -9,  /* command outside the §4 allowlist (see api-request §2.16) */
    HM_ERR_TIMEOUT         = -10,
    HM_ERR_CANCELLED       = -11,
    HM_ERR_LINK_DOWN       = -12,
    HM_ERR_MTU_TOO_SMALL   = -13, /* negotiated MTU < HM_MIN_ATT_MTU (§2.4)    */
    HM_ERR_NO_STREAM       = -14, /* operation requires an open stream         */
    HM_ERR_DEVICE_ERROR    = -15, /* the device replied 0xd0 (§7.2)            */
    HM_ERR_NO_FIT          = -16, /* clock fit has no usable observations yet  */
    HM_ERR_EVICTED         = -17, /* requested range is no longer in the buffer */
    HM_ERR_BUSY            = -18  /* a retrieval is already in flight          */
} hm_status;

/* Stable, allocation-free, never NULL.  Safe for logs. */
HM_API const char *hm_status_str(hm_status status);

/* ------------------------------------------------------------------------ */
/* UUID                                                                      */
/* ------------------------------------------------------------------------ */
/*
 * A 128-bit UUID in big-endian (RFC 4122 network) byte order — bytes[0] is the
 * most significant byte of time_low.  The library hands these out as data and
 * never resolves them itself; see api-request §2.0.5.
 */
typedef struct hm_uuid {
    uint8_t bytes[16];
} hm_uuid;

/* Parses the canonical 8-4-4-4-12 form, with or without braces, any case.
 * Returns HM_ERR_INVALID_ARG on anything else. */
HM_API hm_status hm_uuid_parse(const char *text, hm_uuid *out);

/* Writes the canonical lower-case 8-4-4-4-12 form.  `out` must hold at least
 * HM_UUID_STRING_SIZE bytes.  Returns `out`. */
#define HM_UUID_STRING_SIZE 37
HM_API char *hm_uuid_format(const hm_uuid *uuid, char *out, size_t out_size);

HM_API bool hm_uuid_equal(const hm_uuid *a, const hm_uuid *b);

/* ------------------------------------------------------------------------ */
/* Ranges                                                                    */
/* ------------------------------------------------------------------------ */
/* Inclusive on both ends; `first == last` is one sample. */
typedef struct hm_index_range {
    uint32_t first;
    uint32_t last;
} hm_index_range;

/* Half-open: [start_us, end_us).  Empty when start_us >= end_us. */
typedef struct hm_time_range {
    hm_time_us start_us;
    hm_time_us end_us;
} hm_time_range;

/* ------------------------------------------------------------------------ */
/* Allocator (optional)                                                      */
/* ------------------------------------------------------------------------ */
/*
 * The core never allocates unless the caller supplies memory or an allocator.
 * A caller that provides all the buffers in hm_session_config may leave this
 * zeroed and the library will not call malloc at all.
 */
typedef struct hm_allocator {
    void *(*alloc)(void *ctx, size_t size);
    void  (*free)(void *ctx, void *ptr);
    void  *ctx;
} hm_allocator;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_TYPES_H */
