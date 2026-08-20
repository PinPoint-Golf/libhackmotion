/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_gather_replay.c — drive the history gather with REAL bytes off a wG3.
 *
 * ⚠ THIS IS THE ONLY PLACE RETRIEVAL GETS VALIDATED, AND THE REASON IS §7.3.
 *
 * The device's buffer is MOTION-ADAPTIVE: index step 8 (≈100 Hz) while the
 * wrist is still, step 1 (the full ≈799.2 Hz) only in fast motion.  So a
 * synthetic full-rate reply in the test suite proves the gather HANDLES one and
 * nothing more — and at a desk a correct implementation and a broken one return
 * the same even one-in-eight.  The only evidence that the gather does the right
 * thing with a real reply is a real reply.
 *
 * What this does: replays a `.hmwire` capture through a real hm_session with
 * the gather live.  Where the recording contains an `a1`, it reserves THE SAME
 * SPAN IN HOST TIME through the session's own fit, so the reply the device
 * already gave lands inside a request the library believes in.  Everything
 * after that — the bracket discriminator, the stateless history unwrap,
 * coverage, density, the merge, the block — is the shipping code path.
 *
 * ⚠ IT RUNS IN ctest, over the fixtures in tests/fixtures/, AND IT ALSO HAS TO
 * WORK ON A CAPTURE NOBODY CURATED.  `.gitignore` still excludes `*.hmwire`
 * everywhere except that directory, so anyone who takes a recording off a
 * sensor points this at it directly — and a capture with no retrieval in it,
 * or one the session could not serve, must say NOTHING WAS CHECKED rather than
 * printing five green verdicts over zero samples.  That is why the exit codes
 * separate "passed" from "nothing to pass".
 *
 *   hm_gather_replay FILE [--json PATH]
 *
 * exit: 0 every check passed, 1 a check FAILED, 2 usage or I/O error,
 *       3 the capture is absent or contained no retrieval — nothing checked.
 *
 * ⚠ `--json` writes the per-block table to a SEPARATE FILE and leaves everything
 * on stdout byte-identical.  tools/hm_replay_py.py drives the same gather
 * through the Python binding and tests/test_python_replay.py compares the two
 * tables, so a binding that misreads a struct is caught by DISAGREEING WITH THIS
 * TOOL rather than by matching numbers somebody typed into a test.
 *
 * docs/capture-findings.md §4 records what it reported against swings.hmwire;
 * tests/fixtures/README.md says what each fixture is the control for.
 */
#include "hackmotion/hackmotion.h"
#include "hackmotion/history.h"
#include "hackmotion/record.h"
#include "hackmotion/session.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------ */
/* A check, with the count behind it                                         */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ A check that never ran and a check that passed are DIFFERENT ANSWERS, and
 * reporting them the same way is the failure this library exists to avoid.  So
 * every verdict carries how many observations it rests on, and zero of them is
 * NO EVIDENCE rather than a pass.
 */
typedef struct check {
    const char *what;
    uint64_t    observed;
    uint64_t    failed;
} check;

/*
 * The machine-readable side channel.  NULL unless --json was given, and when it
 * is given nothing on stdout changes — the three ctests over this tool pin that
 * output, and a flag that quietly reshaped it would be a flag that breaks them.
 *
 * ⚠ %.17g on every double, so a value round-trips EXACTLY through the text.  The
 * comparison downstream is equality, and a shortened float would turn a real
 * disagreement into a rounding argument.
 */
static FILE *g_json = NULL;
static bool  g_json_first = true;

static check g_dated = { "every sample dated by its block's own fit", 0, 0 };
static check g_ascending = { "samples ascending, strictly monotonic in index", 0, 0 };
static check g_history_src = { "every sample marked history, with no arrival time", 0, 0 };
static check g_floor = { "no delivered step above §7.3's floor of 8", 0, 0 };
static check g_overlap = { "live-vs-history agreement (§8.8)", 0, 0 };
/*
 * ⚠ INCREMENT 7'S OWN FAILURE MODE, AND IT IS WHY THIS CHECK EXISTS AT ALL.
 *
 * §8.5's depth bracket is learned from what the device returns.  A rule that
 * never fires leaves hm_history_resident_range() answering from
 * HM_HISTORY_DEPTH_SEED_US — §7.3's figure, measured ONCE after 20 s of
 * streaming on somebody else's session — and a consumer cannot tell that from a
 * measurement of their own connection.  The status is what separates them
 * (HM_OK measured, HM_PENDING seeded), and nothing in a fake-transport test can
 * check it honestly: a synthetic reply was told what to return.
 */
static check g_depth = { "buffer depth MEASURED, not seeded (§8.5)", 0, 0 };

static void check_note(check *c, bool ok)
{
    c->observed++;
    if (!ok) {
        c->failed++;
    }
}

/* `%-Ns` pads by BYTES, and these labels carry `§` — two bytes, one column — so
 * a table built with it comes out ragged.  Pad by code points instead. */
static void print_padded(const char *text, size_t width)
{
    size_t columns = 0u;

    fputs(text, stdout);
    for (const char *p = text; *p != '\0'; ++p) {
        if (((unsigned char)*p & 0xc0u) != 0x80u) {
            columns++; /* not a UTF-8 continuation byte */
        }
    }
    while (columns < width) {
        fputc(' ', stdout);
        columns++;
    }
}

static int check_print(const check *c)
{
    if (c->observed == 0u) {
        printf("  [ NONE ] ");
        print_padded(c->what, 50u);
        printf("⚠ NO EVIDENCE — not the same as agreement\n");
        return 0;
    }
    printf("  [%s] ", (c->failed == 0u) ? "  ok  " : " FAIL ");
    print_padded(c->what, 50u);
    printf("%llu checked, %llu failed\n", (unsigned long long)c->observed,
           (unsigned long long)c->failed);
    return (c->failed == 0u) ? 0 : 1;
}

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */
static hm_session *g_session;
static uint64_t    g_pending_id;
static unsigned    g_blocks;
static unsigned    g_status_count[HM_HISTORY_STATUS_COUNT];
/* ⚠ Seeded from NO READING rather than from 1.0.  A run with no block at all
 * used to print "worst density 1.0000" — a value manufactured by an initialiser,
 * printed in the same shape as a measurement. */
static bool        g_have_density;
static double      g_worst_density;

/* `density` is 1 / the median delivered index step (history.h), so the integer
 * comes straight back out of it.  0.0 is "not measurable" and stays that way. */
static const char *median_step_text(double density)
{
    static char buf[16];
    if (!(density > 0.0)) {
        return "?";
    }
    (void)snprintf(buf, sizeof(buf), "%.0f", 1.0 / density);
    return buf;
}
static uint64_t    g_overlap_samples;
static uint64_t    g_overlap_mismatches;

/* The depth reading owed to the block just collected.  ⚠ It cannot be taken at
 * collection time: §6.1.1 re-anchors the fit at every bracket close, so until
 * the next live frame lands there is no mapping from the buffer's head to a
 * host time and the query rightly refuses. */
static bool        g_depth_owed;
static hm_status   g_depth_status = HM_ERR_NO_FIT;
static hm_time_us  g_depth_width;
static unsigned    g_depth_measured;
static unsigned    g_depth_seeded;

static void depth_sample(void)
{
    hm_time_range range;
    hm_status     st;

    if (!g_depth_owed) {
        return;
    }
    st = hm_history_resident_range(g_session, &range);
    if (st != HM_OK && st != HM_PENDING) {
        /*
         * ⚠ NOT AN ANSWER, SO NOT AN OBSERVATION.  HM_ERR_NO_FIT is §6.1.1: the
         * fit re-anchored at the bracket close and has not seen a live frame
         * yet.  HM_ERR_NO_STREAM is a capture that ended with the pull.  Neither
         * says anything about the depth rule, and counting either as a failure
         * would be the same error this whole file exists to prevent — reporting
         * "never checked" as though it were "checked and wrong".
         */
        return;
    }
    g_depth_owed = false;
    g_depth_status = st;
    /* ⚠ ONE OBSERVATION PER PULL, and HM_PENDING is a FAILURE here rather than
     * an absence: a pull happened, so something should have been learned from
     * it.  A run that reports "no evidence" on this line has a depth rule that
     * never fires. */
    check_note(&g_depth, st == HM_OK);
    if (st == HM_OK) {
        g_depth_measured++;
        g_depth_width = range.end_us - range.start_us;
    } else if (st == HM_PENDING) {
        g_depth_seeded++;
    }
}

/* §7.3's own buckets: step 1 is a swing, step 8 is a still wrist, and nothing
 * above 8 was seen in 17,739 measured steps. */
static uint64_t g_step[6]; /* 1, 2, 3-5, 6-7, 8, >8 */
static const char *const k_step_name[6] = { "1  (≈799 Hz)", "2  (≈400 Hz)", "3-5",
                                            "6-7",          "8  (≈100 Hz)", ">8 ⚠" };

static void step_note(uint32_t step)
{
    size_t bucket;
    if (step <= 1u) {
        bucket = 0u;
    } else if (step == 2u) {
        bucket = 1u;
    } else if (step <= 5u) {
        bucket = 2u;
    } else if (step <= 7u) {
        bucket = 3u;
    } else if (step == 8u) {
        bucket = 4u;
    } else {
        bucket = 5u;
    }
    g_step[bucket]++;
    /* ⚠ The floor is what decides when a refill fires and when
     * HM_WARN_HISTORY_HOLED fires, so it is worth its own verdict. */
    check_note(&g_floor, step <= 8u);
}

static void drain(void)
{
    hm_write_request w[8];
    hm_event         ev[16];
    hm_sample        live[64];

    while (hm_session_poll_writes(g_session, w, 8u) > 0u) {
    }
    while (hm_session_poll_events(g_session, ev, 16u) > 0u) {
    }
    while (hm_session_poll_live(g_session, live, 64u) > 0u) {
    }
}

static void collect_if_ready(void)
{
    hm_history_block *b = NULL;

    if (g_pending_id == 0u) {
        return;
    }
    if (hm_history_collect(g_session, g_pending_id, &b) != HM_OK || b == NULL) {
        return;
    }
    g_pending_id = 0u;
    g_blocks++;
    g_depth_owed = true;
    if (b->status < (uint8_t)HM_HISTORY_STATUS_COUNT) {
        g_status_count[b->status]++;
    }

    /*
     * ⚠ THE MEDIAN STEP IS PRINTED BESIDE THE RATIO ON PURPOSE.  `density` is
     * its reciprocal, so a swing pull reads 1.000 and a still one reads 0.125 —
     * and six 1.000s in a row look exactly like the field that was pinned to 1.0
     * before implementation-review I1.  The integer says which it is, in §7.3's
     * own unit.  0.000 means NOT MEASURABLE, printed as "step ?".
     */
    printf("  block   %-9s n=%-5zu coverage=%.3f density=%.3f (step %s) "
           "achieved=%7.1f Hz gaps=%-4zu largest=%5u ms  overlap %u/%u  attempts=%u\n",
           hm_history_status_name((hm_history_status)b->status), b->sample_count,
           b->coverage_fraction, b->density, median_step_text(b->density), b->achieved_hz,
           b->gap_count, b->largest_gap_us / 1000u, b->live_overlap_mismatches,
           b->live_overlap_samples, b->attempts);
    if (b->coverage_overflowed != 0u) {
        printf("          ⚠ coverage storage overflowed: the interval list is a SUPERSET\n");
    }

    if (g_json != NULL) {
        fprintf(g_json,
                "%s\n    {\"status\": %u, \"status_name\": \"%s\", \"sample_count\": %zu, "
                "\"coverage_fraction\": %.17g, \"density\": %.17g, \"achieved_hz\": %.17g, "
                "\"largest_gap_us\": %u, \"gap_count\": %zu, \"delivered_count\": %zu, "
                "\"live_overlap_samples\": %u, \"live_overlap_mismatches\": %u, "
                "\"attempts\": %u, \"coverage_overflowed\": %u, "
                "\"requested_first\": %u, \"requested_last\": %u, "
                "\"first_index\": %u, \"last_index\": %u}",
                g_json_first ? "" : ",",
                (unsigned)b->status,
                hm_history_status_name((hm_history_status)b->status),
                b->sample_count, b->coverage_fraction, b->density, b->achieved_hz,
                b->largest_gap_us, b->gap_count, b->delivered_count,
                b->live_overlap_samples,
                b->live_overlap_mismatches, (unsigned)b->attempts,
                (unsigned)b->coverage_overflowed,
                b->requested_indices.first, b->requested_indices.last,
                b->sample_count ? b->samples[0].sample_index : 0u,
                b->sample_count ? b->samples[b->sample_count - 1u].sample_index : 0u);
        g_json_first = false;
    }

    if (!g_have_density || b->density < g_worst_density) {
        g_have_density = true;
        g_worst_density = b->density;
    }
    g_overlap_samples += b->live_overlap_samples;
    g_overlap_mismatches += b->live_overlap_mismatches;
    if (b->live_overlap_samples > 0u) {
        /* ⚠ Counted as ONE observation of the agreement, carrying its own
         * sample count — the block already reports how many indices it rests
         * on, and that number is what makes a zero mismatch count mean
         * anything at all. */
        check_note(&g_overlap, b->live_overlap_mismatches == 0u);
    }

    for (size_t i = 0; i < b->sample_count; ++i) {
        const hm_sample *sm = &b->samples[i];

        /* ⚠ The block is internally reproducible: the fit it carries is the one
         * that dated every sample in it (AR C9).  §6.1.1 is why that fit is
         * snapshotted when the window closed rather than at materialisation. */
        check_note(&g_dated, sm->host_time_us == hm_clock_to_host_us(&b->fit, sm->sample_index));
        /* Bulk arrival timestamps carry no information at all (§10.1). */
        check_note(&g_history_src, sm->source == (uint8_t)HM_SOURCE_HISTORY &&
                                       sm->host_recv_us == HM_TIME_UNKNOWN);
        if (i > 0u) {
            uint32_t prev = b->samples[i - 1u].sample_index;
            check_note(&g_ascending, sm->sample_index > prev);
            if (sm->sample_index > prev) {
                step_note(sm->sample_index - prev);
            }
        }
    }
    hm_history_block_release(b);
}

/* ------------------------------------------------------------------------ */
/* The replay                                                                */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THE RECORDING HOLDS WHAT WENT ON THE WIRE, AND THAT IS u16be (§7.1).  The
 * session works in UNWRAPPED indices, so a recorded `a1` pair has to be lifted
 * back before it can be turned into a host-time window — the other direction of
 * §7.4's "unwrap internally, re-wrap when asking".
 *
 * Lifted to the index NEAREST the head — §10.2's ±32,768 rule, which is the
 * same decision the library's own unwrapper makes.  Nearest rather than
 * at-or-below deliberately: `swings.hmwire`'s sixth pull asked 200 indices PAST
 * the head, and rounding that down a whole wrap would hide the very thing the
 * clamp below is there to report.  Under 65,536 it is the identity, which is
 * why `swings.hmwire` never needed it; `session1.hmwire`'s pull sits at 173 s,
 * past TWO wraps, and without this it maps to a host time ~170 s in the past.
 */
static uint32_t unwrap_near(uint32_t head, uint32_t raw16)
{
    int32_t delta = (int32_t)(int16_t)(uint16_t)((uint16_t)raw16 - (uint16_t)head);
    return (uint32_t)((int64_t)head + (int64_t)delta);
}

static void on_recorded_a1(const hm_wire_chunk *c, unsigned *reserved, unsigned *refused)
{
    hm_clock_snapshot  snap;
    hm_history_request req;
    uint32_t           first = (uint32_t)(((uint32_t)c->data[1] << 8) | c->data[2]);
    uint32_t           last = (uint32_t)(((uint32_t)c->data[3] << 8) | c->data[4]);
    uint64_t           id = 0u;
    hm_status          st;

    collect_if_ready();

    st = hm_session_clock(g_session, &snap);
    if (st != HM_OK) {
        /* Worth printing rather than skipping: it is how session1.hmwire — the
         * capture whose harness blocked its own event loop for 4.08 s against
         * §6.1's measured 50-80 ms — explains itself.  The session correctly
         * gave up on that stream, so no fit ever formed. */
        printf("  a1 [%u..%u]  no fit: %s (flags=0x%02x, observations=%d)\n", first, last,
               hm_status_str(st), (unsigned)snap.flags, snap.observations);
        (*refused)++;
        return;
    }

    last = unwrap_near(snap.last_index, last);
    /* `first` is lifted relative to `last`, not to the head, so a range that
     * straddles a wrap stays one ascending range rather than becoming a
     * 65,000-index one. */
    first = last - (uint32_t)(uint16_t)((uint16_t)last - (uint16_t)first);

    /*
     * ⚠ Clamped to the head the device had actually reached, and this models
     * what a library-driven consumer asks for rather than what the recording
     * harness asked for.  swings.hmwire contains one pull whose `a1` ran 200
     * indices PAST the head — a window whose last sample did not exist yet, so
     * hm_history_reserve() would rightly have held it (AR C1: no radio traffic
     * before the window can close) and the device answered it anyway with 27
     * records in a 25 ms bracket, which is §7.5's pull 6.
     */
    if (last > snap.last_index) {
        printf("  a1 [%u..%u]  ⚠ asked %u indices past the head; clamped to %u\n", first, last,
               last - snap.last_index, snap.last_index);
        last = snap.last_index;
    }

    memset(&req, 0, sizeof(req));
    /* ⚠ The one-microsecond nudge at each end is the half-open/inclusive
     * boundary of hm_clock_index_range_for_time(), pushed off the rounding edge
     * rather than sat on it. */
    req.window.start_us = hm_clock_to_host_us(&snap, first) - 1;
    req.window.end_us = hm_clock_to_host_us(&snap, last) + 1;
    req.deadline_us = c->host_time_us + (hm_time_us)60 * 1000 * 1000;
    req.max_attempts = 1u;

    st = hm_history_reserve(g_session, &req, &id);
    if (st != HM_OK) {
        printf("  a1 [%u..%u]  reserve refused: %s\n", first, last, hm_status_str(st));
        (*refused)++;
        return;
    }
    (*reserved)++;
    g_pending_id = id;
    printf("  a1 [%u..%u]  reserved id=%llu\n", first, last, (unsigned long long)id);
    drain();
}

static int usage(void)
{
    fputs("usage: hm_gather_replay FILE.hmwire [--json PATH]\n"
          "\n"
          "  Replays a capture through the history gather and reports what the\n"
          "  device's own records did to it.  ⚠ The only validation retrieval\n"
          "  gets: §7.3's buffer is motion-adaptive, so a synthetic reply proves\n"
          "  only that the code handles one.\n"
          "\n"
          "  --json PATH   also write the per-block table there, for comparison\n"
          "                against tools/hm_replay_py.py.  stdout is unchanged.\n"
          "\n"
          "exit: 0 checks passed, 1 a check FAILED, 2 usage or I/O error,\n"
          "      3 nothing was checked\n",
          stderr);
    return 2;
}

int main(int argc, char **argv)
{
    hm_session_config cfg = hm_session_config_default();
    hm_replay        *rp = NULL;
    hm_wire_chunk     c;
    unsigned          a1_seen = 0u;
    unsigned          reserved = 0u;
    unsigned          refused = 0u;
    int               failures = 0;
    FILE             *probe;
    const char       *json_path = NULL;

    if (argc < 2 || argv[1][0] == '-') {
        return usage();
    }
    if (argc == 4 && strcmp(argv[2], "--json") == 0) {
        json_path = argv[3];
    } else if (argc != 2) {
        return usage();
    }

    probe = fopen(argv[1], "rb");
    if (probe == NULL) {
        /* ⚠ NOT a silent success: a run that checked nothing has to say so in
         * the same breath, or it reads as a clean bill of health. */
        fprintf(stderr,
                "hm_gather_replay: %s not found.\n"
                "⚠ NOTHING WAS CHECKED.  tests/fixtures/ holds the tracked "
                "captures;\n"
                "   docs/capture-runbook.md covers taking a fresh one off a "
                "sensor.\n",
                argv[1]);
        return 3;
    }
    fclose(probe);

    /* ⚠ The live-vs-history digest ring is OFF by default, and with it off a
     * block reports live_overlap_samples = 0 — which means NO EVIDENCE, not
     * agreement.  This run wants the evidence. */
    cfg.memory.digest_ring_capacity = HM_DIGEST_RING_RECOMMENDED;
    /*
     * ⚠ THE STREAM-START BOUND IS RELAXED HERE, AND ONLY HERE.  §6.1 measures
     * the first frame arriving 50-80 ms after `a0`, and the library's 3 s
     * default is two orders of margin over that.  `session1.hmwire`'s first
     * frame arrived at 4.08 s — a property of the RECORDER, whose harness
     * blocked its own event loop, not of the device.  A replay tool that
     * enforced a live-session bound against a recording would discard the only
     * capture this project holds of a retrieval past two index wraps, and would
     * be enforcing it against the wrong party.  The bound itself is tested where
     * it belongs, in tests/test_session.c.
     */
    cfg.policy.stream_start_timeout_us = (hm_time_us)15 * 1000 * 1000;
    if (hm_session_create(&cfg, &g_session) != HM_OK) {
        return 2;
    }
    if (hm_replay_open(argv[1], &rp) != HM_OK) {
        fprintf(stderr, "hm_gather_replay: cannot read %s\n", argv[1]);
        hm_session_destroy(g_session);
        return 2;
    }

    /* ⚠ Opened HERE rather than at argument-parse time, and only once the run is
     * certain to happen.  Every early return above is a case where nothing will
     * be replayed, and a file left holding a half-written `{"blocks": [` is
     * worse than no file: the reader downstream fails on a parse error instead
     * of on the absence it should be reporting. */
    if (json_path != NULL) {
        g_json = fopen(json_path, "w");
        if (g_json == NULL) {
            fprintf(stderr, "hm_gather_replay: cannot write %s\n", json_path);
            hm_replay_close(rp);
            hm_session_destroy(g_session);
            return 2;
        }
        fprintf(g_json, "{\"blocks\": [");
    }

    printf("hm_gather_replay — %s\n"
           "the history gather, driven by a real device's own bytes\n\n",
           argv[1]);

    while (hm_replay_next(rp, &c) == HM_OK) {
        if (c.direction == (uint8_t)HM_WIRE_META) {
            /* tools/hm_capture.py writes the same short vocabulary the session
             * does, so one reader handles both. */
            if (c.length >= 7u && memcmp(c.data, "link_up", 7u) == 0) {
                hm_session_on_link_up(g_session, 247, c.host_time_us);
            } else if (c.length >= 12u && memcmp(c.data, "stream_start", 12u) == 0) {
                (void)hm_session_start_stream(g_session);
            }
            drain();
            continue;
        }
        if (c.direction == (uint8_t)HM_WIRE_HOST_TO_DEVICE) {
            hm_session_tick(g_session, c.host_time_us);
            drain();
            if (c.length == 5u && c.data[0] == 0xa1u) {
                a1_seen++;
                on_recorded_a1(&c, &reserved, &refused);
            }
            continue;
        }
        hm_session_on_bytes(g_session, c.data, c.length, c.host_time_us);
        drain();
        collect_if_ready();
        depth_sample();
    }
    collect_if_ready();
    depth_sample();

    if (g_pending_id != 0u) {
        printf("  ⚠ request %llu was never answered — pending %zu at end of capture\n",
               (unsigned long long)g_pending_id, hm_history_pending(g_session));
    }
    hm_replay_close(rp);

    printf("\nblocks\n------\n");
    if (g_blocks == 0u) {
        printf("  none — this capture contains no retrieval the session could serve\n");
    }
    for (int i = 0; i < (int)HM_HISTORY_STATUS_COUNT; ++i) {
        if (g_status_count[i] != 0u) {
            printf("  %-20s %u\n", hm_history_status_name((hm_history_status)i),
                   g_status_count[i]);
        }
    }
    /* ⚠ §8.7: every reply is holed and the holes are not an error.  A run with
     * no HM_HIST_COMPLETE in it is the expected shape, not a failure. */

    printf("\ndelivered index steps (§7.3)\n---------------------------\n");
    for (size_t i = 0; i < 6u; ++i) {
        if (g_step[i] != 0u) {
            printf("  %-14s %llu\n", k_step_name[i], (unsigned long long)g_step[i]);
        }
    }

    /*
     * ⚠ §8.5's bracket, and it is reported BEFORE it is trusted anywhere.  The
     * width below is a LOWER BOUND and is bounded in turn by the widest window
     * anyone asked for — this capture's pulls are 400 indices, so it can only
     * ever verify half a second whatever the buffer actually holds.  That is
     * the honest answer and it is much smaller than §7.3's seed, which is
     * exactly the point: the seed is a guess about somebody else's session.
     */
    printf("\nbuffer depth (§8.5)\n-------------------\n");
    if (g_depth_measured == 0u && g_depth_seeded == 0u) {
        printf("  no reading — no pull completed with a live frame after it\n");
    } else {
        printf("  readings       %u measured, %u still seeded\n", g_depth_measured,
               g_depth_seeded);
    }
    if (g_depth_measured > 0u) {
        printf("  verified reach-back  %.3f s   ⚠ a LOWER bound, and no wider than the\n"
               "                                 widest window asked for (seed is %.3f s)\n",
               (double)g_depth_width / 1e6, (double)HM_HISTORY_DEPTH_SEED_US / 1e6);
    }

    printf("\nverdicts\n--------\n");
    failures += check_print(&g_dated);
    failures += check_print(&g_ascending);
    failures += check_print(&g_history_src);
    failures += check_print(&g_floor);
    failures += check_print(&g_overlap);
    printf("           ");
    print_padded("...over", 50u);
    printf("%llu indices, %llu mismatched\n", (unsigned long long)g_overlap_samples,
           (unsigned long long)g_overlap_mismatches);
    failures += check_print(&g_depth);

    printf("\n  recorded a1 %u, reserved %u, refused %u; %u block(s); worst density ",
           a1_seen, reserved, refused, g_blocks);
    if (g_have_density) {
        printf("%.4f (step %s)\n", g_worst_density, median_step_text(g_worst_density));
    } else {
        printf("NO READING\n");
    }

    /*
     * ⚠ THE STEP HISTOGRAM GOES IN TOO, and it is not decoration.  It is
     * computed by walking the delivered samples, so it exercises the binding's
     * sample decode rather than only the block's scalars — a ctypes hm_sample
     * whose sample_index sat at the wrong offset would agree on every block
     * number and disagree here.
     */
    if (g_json != NULL) {
        fprintf(g_json, "\n  ],\n  \"steps\": {");
        for (size_t i = 0; i < 6u; ++i) {
            fprintf(g_json, "%s\"%zu\": %llu", i ? ", " : "", i,
                    (unsigned long long)g_step[i]);
        }
        fprintf(g_json, "},\n  \"a1_seen\": %u, \"reserved\": %u, \"refused\": %u,\n",
                a1_seen, reserved, refused);
        fprintf(g_json, "  \"blocks_total\": %u, \"depth_measured\": %u, "
                        "\"depth_seeded\": %u, \"depth_width_us\": %llu\n}\n",
                g_blocks, g_depth_measured, g_depth_seeded,
                (unsigned long long)g_depth_width);
        fclose(g_json);
        g_json = NULL;
    }

    hm_session_destroy(g_session);

    if (failures > 0) {
        return 1;
    }
    if (g_blocks == 0u) {
        fprintf(stderr, "⚠ NOTHING WAS CHECKED: no block was produced.\n");
        return 3;
    }
    return 0;
}
