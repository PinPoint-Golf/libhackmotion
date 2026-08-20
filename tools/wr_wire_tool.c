/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wrwire — read a `.wrwire` capture back.
 *
 *   wrwire info FILE          the header and a census
 *   wrwire dump FILE          every chunk, decoded far enough to be readable
 *   wrwire verify FILE        container integrity: framing, ordering, loss
 *   wrwire reconcile FILE     ⚠ the capture against docs/specification.md
 *   wrwire allowlist          every command byte this library can ever send
 *   wrwire abi                every public struct's layout, as JSON
 *
 * ⚠ `allowlist` exists so that the capture front-end does not have to keep its
 * own copy of the list.  ⛔ `f0` reboots the sensor into firmware-update mode
 * through the ORDINARY data characteristic, so "which bytes may be written" is
 * a safety property, and a second copy of a safety property is a second thing
 * that can drift.  tools/wr_capture.py refuses to run if it cannot ask this.
 *
 * ⚠ `abi` exists for the same reason one register up: python/wrist
 * declares the public structs a second time, in ctypes, and a field one slot
 * out of place decodes every sample into plausible nonsense with no error.
 * wr_abi_check() compares struct SIZES only.  See tools/wr_abi_table.h.
 *
 * This tool opens files.  The core does not, and tests/purity.cmake keeps it
 * that way; nothing here links into the library a consumer embeds.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wrist/record.h"
#include "wrist/wrist.h"

#include "wr_codec.h"
#include "wr_abi_table.h"

static int usage(void)
{
    fputs("usage: wrwire <command> [args]\n"
          "\n"
          "  info FILE               header, duration, chunk census\n"
          "  dump FILE [--limit N] [--hex]\n"
          "                          one line per chunk, message id decoded\n"
          "  verify FILE             container integrity: framing, ordering, loss\n"
          "  reconcile FILE          the capture read back against the specification\n"
          "  allowlist               the command bytes this library can send, one per line\n"
          "  abi                     every public struct's size and field offsets, as JSON\n"
          "\n"
          "exit: 0 clean, 1 findings, 2 usage or I/O error\n",
          stderr);
    return 2;
}

static const char *direction_name(uint8_t d)
{
    switch ((wr_wire_direction)d) {
        case WR_WIRE_HOST_TO_DEVICE: return "H→D";
        case WR_WIRE_DEVICE_TO_HOST: return "D→H";
        case WR_WIRE_META:           return "meta";
        default:                     return "?";
    }
}

/* Enough of the message table to make a dump readable without duplicating the
 * decoder's job.  Anything absent prints as its raw id, which is what §5.1 says
 * a client should do with it anyway. */
static const char *message_name(uint8_t id)
{
    switch ((wr_msg_id)id) {
        case WR_MSG_ID_LEGACY_FRAME:   return "legacy frame";
        case WR_MSG_ID_VERSIONS:       return "versions";
        case WR_MSG_ID_STATUS:         return "status/battery";
        case WR_MSG_ID_STREAM_STARTED: return "stream started";
        case WR_MSG_ID_STREAM_STOPPED: return "stream stopped";
        case WR_MSG_ID_SENSOR_MAP:     return "sensor map";
        case WR_MSG_ID_MAC:            return "MAC";
        case WR_MSG_ID_SERIAL:         return "serial";
        case WR_MSG_ID_FRAME:          return "frame";
        case WR_MSG_ID_CALIBRATION:    return "calibration result";
        case WR_MSG_ID_START_ACK:      return "start ack";
        case WR_MSG_ID_HISTORY_MARK:   return "history marker";
        case WR_MSG_ID_CAL_ACK:        return "calibration ack";
        case WR_MSG_ID_DEVICE_ERROR:   return "DEVICE ERROR";
        case WR_MSG_ID_BUTTON:         return "button";
        default:                       return NULL;
    }
}

static void print_header(const wr_recording_info *info, FILE *out)
{
    fprintf(out, "  magic          %s\n", WR_RECORD_MAGIC);
    fprintf(out, "  device_id      %s\n", info->device_id[0] ? info->device_id : "(none)");
    if (info->config_legacy) {
        fprintf(out, "  config         legacy `82` — ⚠ no record header, no ticks, no history\n");
    } else {
        fprintf(out, "  config         0x%02x\n", (unsigned)info->config_bits);
    }
    fprintf(out, "  layout_version %u\n", (unsigned)info->layout_version);
    fprintf(out, "  clock          %s\n", info->clock);
    fprintf(out, "  identifiers    %s\n",
            info->identifiers_recorded ? "RECORDED" : "redacted");
}

/* ------------------------------------------------------------------------ */
static int cmd_info(const char *path)
{
    wr_replay *rp = NULL;
    wr_wire_chunk c;
    wr_status st;
    uint64_t by_dir[3] = {0, 0, 0};
    uint64_t payload = 0, lost = 0, redacted = 0;
    wr_time_us first = 0, last = 0;
    bool have_first = false;

    st = wr_replay_open(path, &rp);
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s\n", path, wr_status_str(st));
        return 2;
    }
    printf("%s\n", path);
    print_header(wr_replay_info(rp), stdout);

    while ((st = wr_replay_next(rp, &c)) == WR_OK) {
        if (c.direction < 3u) {
            by_dir[c.direction]++;
        }
        payload += c.length;
        if ((c.flags & WR_WIRE_LOST) != 0u) {
            lost++;
        }
        if ((c.flags & WR_WIRE_REDACTED) != 0u) {
            redacted++;
        }
        if (!have_first) {
            first = c.host_time_us;
            have_first = true;
        }
        last = c.host_time_us;
    }
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s after %llu chunk(s)\n", path, wr_status_str(st),
                (unsigned long long)wr_replay_chunks_read(rp));
        wr_replay_close(rp);
        return 1;
    }

    printf("  chunks         %llu  (%llu H→D, %llu D→H, %llu meta)\n",
           (unsigned long long)wr_replay_chunks_read(rp), (unsigned long long)by_dir[0],
           (unsigned long long)by_dir[1], (unsigned long long)by_dir[2]);
    printf("  payload        %llu bytes\n", (unsigned long long)payload);
    printf("  duration       %.3f s\n",
           have_first ? (double)(last - first) / 1e6 : 0.0);
    if (lost > 0u) {
        printf("  ⚠ loss         %llu chunk(s) followed a drop\n", (unsigned long long)lost);
    }
    if (redacted > 0u) {
        printf("  redacted       %llu chunk(s)\n", (unsigned long long)redacted);
    }
    wr_replay_close(rp);
    return 0;
}

/* ------------------------------------------------------------------------ */
static int cmd_dump(const char *path, uint64_t limit, bool hex)
{
    wr_replay *rp = NULL;
    wr_wire_chunk c;
    wr_status st;
    uint64_t n = 0;
    wr_time_us origin = 0;
    bool have_origin = false;

    st = wr_replay_open(path, &rp);
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s\n", path, wr_status_str(st));
        return 2;
    }

    while ((st = wr_replay_next(rp, &c)) == WR_OK) {
        const char *name;
        if (limit > 0u && n >= limit) {
            /* ⚠ Say what was dropped.  A silent truncation reads as "that was
             * the whole file". */
            printf("... stopped at --limit %llu; the file has more\n",
                   (unsigned long long)limit);
            break;
        }
        if (!have_origin) {
            origin = c.host_time_us;
            have_origin = true;
        }
        name = (c.length > 0u && c.direction == WR_WIRE_DEVICE_TO_HOST)
                   ? message_name(c.data[0])
                   : NULL;
        printf("%10.6f  #%-8u %-4s %3u B", (double)(c.host_time_us - origin) / 1e6, c.sequence,
               direction_name(c.direction), c.length);
        if (c.length > 0u) {
            printf("  %02x", c.data[0]);
            if (name != NULL) {
                printf(" %s", name);
            }
        }
        if ((c.flags & WR_WIRE_REDACTED) != 0u) {
            printf("  [REDACTED]");
        }
        if ((c.flags & WR_WIRE_LOST) != 0u) {
            printf("  [AFTER LOSS]");
        }
        if (hex) {
            printf("\n            ");
            for (uint16_t i = 0; i < c.length; ++i) {
                printf("%02x%s", c.data[i], ((i % 16u) == 15u && i + 1u < c.length)
                                                ? "\n            "
                                                : " ");
            }
        }
        printf("\n");
        n++;
    }
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s\n", path, wr_status_str(st));
        wr_replay_close(rp);
        return 1;
    }
    wr_replay_close(rp);
    return 0;
}

/* ------------------------------------------------------------------------ */
static int cmd_verify(const char *path)
{
    wr_replay *rp = NULL;
    wr_wire_chunk c;
    wr_status st;
    uint64_t findings = 0, n = 0;
    uint64_t regressions = 0, seq_gaps = 0, unmarked_gaps = 0;
    wr_time_us last_time = 0;
    uint32_t last_seq = 0;
    bool have_prev = false;

    st = wr_replay_open(path, &rp);
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s\n", path, wr_status_str(st));
        return 2;
    }

    while ((st = wr_replay_next(rp, &c)) == WR_OK) {
        if (have_prev) {
            if (c.host_time_us < last_time) {
                /* ⚠ The host clock must be monotonic (types.h).  A wall clock
                 * stepped by NTP or DST corrupts a capture in a way that looks
                 * like a sensor fault, and this is where it becomes visible. */
                regressions++;
            }
            if (c.sequence != last_seq + 1u) {
                seq_gaps++;
                if ((c.flags & WR_WIRE_LOST) == 0u) {
                    /* A gap the recording does not admit to. */
                    unmarked_gaps++;
                }
            }
        }
        last_time = c.host_time_us;
        last_seq = c.sequence;
        have_prev = true;
        n++;
    }

    printf("%s\n", path);
    print_header(wr_replay_info(rp), stdout);
    printf("  chunks         %llu\n", (unsigned long long)n);

    if (st < WR_OK) {
        printf("  FINDING        %s after %llu chunk(s)\n", wr_status_str(st),
               (unsigned long long)n);
        findings++;
    }
    if (regressions > 0u) {
        printf("  FINDING        %llu host-time regression(s) — ⚠ the capture's clock was not "
               "monotonic\n",
               (unsigned long long)regressions);
        findings++;
    }
    if (seq_gaps > 0u) {
        printf("  finding        %llu sequence gap(s), %llu of them not marked WR_WIRE_LOST\n",
               (unsigned long long)seq_gaps, (unsigned long long)unmarked_gaps);
        findings++;
    }
    if (findings == 0u) {
        printf("  ok             framing intact, host time monotonic, no unexplained gaps\n");
    }
    wr_replay_close(rp);
    return (findings == 0u) ? 0 : 1;
}

/* ------------------------------------------------------------------------ */
static int cmd_reconcile(const char *path)
{
    wr_replay *rp = NULL;
    wr_reconciler *rc = NULL;
    wr_reconcile_report report;
    wr_wire_chunk c;
    wr_status st;

    st = wr_replay_open(path, &rp);
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s\n", path, wr_status_str(st));
        return 2;
    }
    st = wr_reconcile_begin(wr_replay_info(rp), &rc);
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s\n", wr_status_str(st));
        wr_replay_close(rp);
        return 2;
    }

    while ((st = wr_replay_next(rp, &c)) == WR_OK) {
        wr_reconcile_observe(rc, &c);
    }
    if (st < WR_OK) {
        fprintf(stderr, "wrwire: %s: %s after %llu chunk(s) — the report below covers what "
                        "was readable\n",
                path, wr_status_str(st), (unsigned long long)wr_replay_chunks_read(rp));
    }

    wr_reconcile_finish(rc, &report);
    wr_reconcile_print(&report, stdout);

    {
        int differ = wr_reconcile_disagreements(&report);
        wr_reconcile_free(rc);
        wr_replay_close(rp);
        return (differ > 0) ? 1 : 0;
    }
}

/* ------------------------------------------------------------------------ */
static int cmd_allowlist(void)
{
    uint8_t bytes[32];
    size_t n = wr_command_allowlist(bytes, sizeof(bytes));

    if (n > sizeof(bytes)) {
        fprintf(stderr, "wrwire: allowlist is %zu bytes, buffer is %zu\n", n, sizeof(bytes));
        return 2;
    }
    for (size_t i = 0; i < n; ++i) {
        printf("%02x\n", bytes[i]);
    }
    /* ⛔ Belt and braces for a reader and for the caller's own assertion. */
    if (wr_command_is_allowed(0xf0u)) {
        fprintf(stderr, "wrwire: ⛔ 0xf0 is on the allowlist — refusing to vouch for it\n");
        return 2;
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
/* ⚠ Exit 1 on a self-check problem, not 0 with a warning.  A layout table
 * missing a field is not a finding about a capture — it is this tool having
 * stopped measuring, and the binding test downstream would then compare two
 * stale field sets and pass. */
static int cmd_abi(void)
{
    int problems = wr_abi_print_json(stdout, stderr);

    if (problems > 0) {
        fprintf(stderr,
                "wrwire: %d layout problem(s) — tools/wr_abi_table.c is out of step "
                "with the headers\n",
                problems);
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    if (argc < 2) {
        return usage();
    }
    if (strcmp(argv[1], "allowlist") == 0) {
        return cmd_allowlist();
    }
    if (strcmp(argv[1], "abi") == 0) {
        return cmd_abi();
    }
    if (strcmp(argv[1], "--version") == 0 || strcmp(argv[1], "-V") == 0) {
        printf("wrwire (libwrist %s)\n", wr_version_string());
        return 0;
    }
    if (argc < 3) {
        return usage();
    }

    if (strcmp(argv[1], "info") == 0) {
        return cmd_info(argv[2]);
    }
    if (strcmp(argv[1], "verify") == 0) {
        return cmd_verify(argv[2]);
    }
    if (strcmp(argv[1], "reconcile") == 0) {
        return cmd_reconcile(argv[2]);
    }
    if (strcmp(argv[1], "dump") == 0) {
        uint64_t limit = 0;
        bool hex = false;
        for (int i = 3; i < argc; ++i) {
            if (strcmp(argv[i], "--hex") == 0) {
                hex = true;
            } else if (strcmp(argv[i], "--limit") == 0 && i + 1 < argc) {
                limit = strtoull(argv[++i], NULL, 10);
            } else {
                return usage();
            }
        }
        return cmd_dump(argv[2], limit, hex);
    }
    return usage();
}
