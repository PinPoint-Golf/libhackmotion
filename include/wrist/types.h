/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wrist/types.h — scalar types, status codes and the time contract.
 *
 * Everything in libwrist is C11, POD and free of hidden allocation.
 * See docs/design.md §3 for the type-system rules these headers follow.
 */
#ifndef WRIST_TYPES_H
#define WRIST_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined(_WIN32) && defined(WR_SHARED)
#  if defined(WR_BUILDING_LIBRARY)
#    define WR_API __declspec(dllexport)
#  else
#    define WR_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#  define WR_API __attribute__((visibility("default")))
#else
#  define WR_API
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
typedef int64_t wr_time_us;

/* Returned wherever a timestamp is structurally unavailable — for example the
 * arrival instant of a history record, which carries no information (§10.1). */
#define WR_TIME_UNKNOWN  INT64_MIN

/* wr_session_next_due_us() returns this when the session has no pending
 * deadline.  A host may sleep indefinitely, waking on transport traffic. */
#define WR_TIME_NEVER    INT64_MAX

/* ------------------------------------------------------------------------ */
/* Status                                                                    */
/* ------------------------------------------------------------------------ */
/*
 * Non-negative values are success.  Negative values are failure.  Test with
 * `if (st < WR_OK)`, never `if (st != WR_OK)` — WR_PENDING is not an error.
 */
typedef enum wr_status {
    WR_OK                  = 0,
    WR_PENDING             = 1,  /* in flight; poll again later               */
    WR_DONE                = 2,  /* nothing further will arrive               */

    WR_ERR_INVALID_ARG     = -1,
    WR_ERR_INVALID_STATE   = -2,
    WR_ERR_NO_MEMORY       = -3,
    WR_ERR_BUFFER_TOO_SMALL= -4,
    WR_ERR_NOT_SUPPORTED   = -5,
    WR_ERR_TRUNCATED       = -6,  /* frame shorter than its message id implies */
    WR_ERR_MALFORMED       = -7,  /* frame is the right length and still wrong */
    WR_ERR_UNKNOWN_MESSAGE = -8,  /* id outside §5.1 — log and ignore, not an error */
    WR_ERR_NOT_ALLOWED     = -9,  /* command outside the §4 allowlist (see api-request §2.16) */
    WR_ERR_TIMEOUT         = -10,
    WR_ERR_CANCELLED       = -11,
    WR_ERR_LINK_DOWN       = -12,
    WR_ERR_MTU_TOO_SMALL   = -13, /* negotiated MTU < WR_MIN_ATT_MTU (§2.4)    */
    WR_ERR_NO_STREAM       = -14, /* operation requires an open stream         */
    WR_ERR_DEVICE_ERROR    = -15, /* the device replied 0xd0 (§7.2)            */
    WR_ERR_NO_FIT          = -16, /* clock fit has no usable observations yet  */
    WR_ERR_EVICTED         = -17, /* requested range is no longer in the buffer */
    WR_ERR_BUSY            = -18  /* a retrieval is already in flight          */
} wr_status;

/* Stable, allocation-free, never NULL.  Safe for logs. */
WR_API const char *wr_status_str(wr_status status);

/* ------------------------------------------------------------------------ */
/* UUID                                                                      */
/* ------------------------------------------------------------------------ */
/*
 * A 128-bit UUID in big-endian (RFC 4122 network) byte order — bytes[0] is the
 * most significant byte of time_low.  The library hands these out as data and
 * never resolves them itself; see api-request §2.0.5.
 */
typedef struct wr_uuid {
    uint8_t bytes[16];
} wr_uuid;

/* Parses the canonical 8-4-4-4-12 form, with or without braces, any case.
 * Returns WR_ERR_INVALID_ARG on anything else. */
WR_API wr_status wr_uuid_parse(const char *text, wr_uuid *out);

/* Writes the canonical lower-case 8-4-4-4-12 form.  `out` must hold at least
 * WR_UUID_STRING_SIZE bytes.  Returns `out`. */
#define WR_UUID_STRING_SIZE 37
WR_API char *wr_uuid_format(const wr_uuid *uuid, char *out, size_t out_size);

WR_API bool wr_uuid_equal(const wr_uuid *a, const wr_uuid *b);

/* ------------------------------------------------------------------------ */
/* Ranges                                                                    */
/* ------------------------------------------------------------------------ */
/* Inclusive on both ends; `first == last` is one sample. */
typedef struct wr_index_range {
    uint32_t first;
    uint32_t last;
} wr_index_range;

/* Half-open: [start_us, end_us).  Empty when start_us >= end_us. */
typedef struct wr_time_range {
    wr_time_us start_us;
    wr_time_us end_us;
} wr_time_range;

/* ------------------------------------------------------------------------ */
/* Allocator (optional)                                                      */
/* ------------------------------------------------------------------------ */
/*
 * The core never allocates unless the caller supplies memory or an allocator.
 * A caller that provides all the buffers in wr_session_config may leave this
 * zeroed and the library will not call malloc at all.
 */
typedef struct wr_allocator {
    void *(*alloc)(void *ctx, size_t size);
    void  (*free)(void *ctx, void *ptr);
    void  *ctx;
} wr_allocator;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* WRIST_TYPES_H */
