/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_command.c — the allowlist, which is a safety property rather than a
 * behaviour.  api-request §2.16: "an API that cannot express the destructive
 * thing is worth more than a warning".
 */
#include "wr_test.h"
#include "wr_command.h"

/*
 * ⛔ THE TEST THAT MATTERS.  Spec §4.1: `f0` reboots the device into
 * firmware-update mode, and it reaches that mode through the ORDINARY data
 * characteristic — the same pipe every other command uses.  There must be no
 * path in this library that can emit it.
 */
WR_TEST(command_f0_can_never_be_emitted)
{
    wr_write_request w;
    const uint8_t f0[1] = {0xF0};

    WR_ASSERT(!wr_command_is_allowed(0xF0));
    WR_ASSERT_EQ(wr_command_emit(&w, f0, 1, false), WR_ERR_NOT_ALLOWED);
}

/*
 * The device accepts commands this library deliberately does not send (spec
 * §4.1).  Rather than enumerate them, sweep: EVERY value outside the allowlist
 * must be refused by the gate, at every length.
 *
 * ⚠ This is stronger than a list of the interesting ones, and it cannot rot.
 * A list has to be maintained against a moving allowlist and silently stops
 * covering anything the list forgot; the sweep covers all 245 by construction.
 */
WR_TEST(command_everything_outside_the_allowlist_is_refused)
{
    wr_write_request w;
    unsigned refused = 0;

    for (unsigned b = 0; b < 256u; ++b) {
        uint8_t payload[2] = {0, 0};
        if (wr_command_is_allowed((uint8_t)b)) {
            continue;
        }
        payload[0] = (uint8_t)b;
        WR_ASSERT_EQ(wr_command_emit(&w, payload, 2, false), WR_ERR_NOT_ALLOWED);
        ++refused;
    }
    /* ⚠ 256 minus the eleven on the list.  Asserted so that an allowlist which
     * quietly grew cannot leave this test passing over a smaller sweep. */
    WR_ASSERT_EQ(refused, 245u);

    /* Length is not what the gate keys on: a longer payload whose leading byte
     * is not a command id is refused just the same. */
    {
        const uint8_t wide[4] = {0x10, 0x11, 0x12, 0x13};
        WR_ASSERT_EQ(wr_command_emit(&w, wide, 4, false), WR_ERR_NOT_ALLOWED);
    }
}

/* The allowlist is exactly spec §4's client-facing command set. */
WR_TEST(command_allowlist_is_exactly_section_4)
{
    uint8_t expected[] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0xA0, 0xA1, 0xA2, 0xFA};
    uint8_t actual[32];
    size_t n = wr_command_allowlist(actual, sizeof(actual));

    WR_ASSERT_EQ(n, sizeof(expected));
    for (size_t i = 0; i < n; ++i) {
        WR_ASSERT_EQ(actual[i], expected[i]);
    }
    /* And nothing outside it, swept exhaustively over the byte space. */
    for (unsigned b = 0; b < 256u; ++b) {
        bool found = false;
        for (size_t i = 0; i < sizeof(expected); ++i) {
            if (expected[i] == (uint8_t)b) {
                found = true;
            }
        }
        WR_ASSERT_EQ(wr_command_is_allowed((uint8_t)b), found);
    }
}

/* §6.1 — `a0 01 7e`. */
WR_TEST(command_start_stream_encodes_the_observed_default)
{
    wr_write_request w;
    WR_ASSERT_EQ(wr_cmd_start_stream(&w, wr_stream_config_default()), WR_OK);
    WR_ASSERT_EQ(w.length, 3);
    WR_ASSERT_EQ(w.data[0], 0xA0);
    WR_ASSERT_EQ(w.data[1], 0x01);
    WR_ASSERT_EQ(w.data[2], 0x7E);
}

/* §4 — the legacy start takes no configuration byte at all. */
WR_TEST(command_legacy_start_is_a_bare_82)
{
    wr_write_request w;
    WR_ASSERT_EQ(wr_cmd_start_stream(&w, wr_stream_config_legacy()), WR_OK);
    WR_ASSERT_EQ(w.length, 1);
    WR_ASSERT_EQ(w.data[0], 0x82);
}

/* §7.1 — `a1 <first u16be> <last u16be>`, and `first` must be below `last`. */
WR_TEST(command_history_is_big_endian_and_ordered)
{
    wr_write_request w;
    WR_ASSERT_EQ(wr_cmd_history(&w, 0x1234, 0xABCD), WR_OK);
    WR_ASSERT_EQ(w.length, 5);
    WR_ASSERT_EQ(w.data[0], 0xA1);
    WR_ASSERT_EQ(w.data[1], 0x12);
    WR_ASSERT_EQ(w.data[2], 0x34);
    WR_ASSERT_EQ(w.data[3], 0xAB);
    WR_ASSERT_EQ(w.data[4], 0xCD);

    /* A reversed range is one of the seven distinct causes that all return the
     * same d0 03; refusing it here removes one thing to guess at later. */
    WR_ASSERT_EQ(wr_cmd_history(&w, 500, 100), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_cmd_history(&w, 100, 100), WR_ERR_INVALID_ARG);
}

/* §8.2 — the two pose markers, and nothing else. */
WR_TEST(command_calibration_markers)
{
    wr_write_request w;
    WR_ASSERT_EQ(wr_cmd_calibration_marker(&w, 0), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0xA2);
    WR_ASSERT_EQ(w.data[1], 0x00);
    WR_ASSERT_EQ(wr_cmd_calibration_marker(&w, 1), WR_OK);
    WR_ASSERT_EQ(w.data[1], 0x01);
    WR_ASSERT_EQ(wr_cmd_calibration_marker(&w, 2), WR_ERR_INVALID_ARG);
}

WR_TEST(command_single_byte_commands)
{
    wr_write_request w;
    WR_ASSERT_EQ(wr_cmd_versions(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0x80);
    WR_ASSERT_EQ(w.length, 1);
    WR_ASSERT_EQ(wr_cmd_status(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0x81);
    WR_ASSERT_EQ(wr_cmd_sensor_map(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0x84);
    WR_ASSERT_EQ(wr_cmd_mac(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0x85);
    WR_ASSERT_EQ(wr_cmd_serial(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0x86);
    WR_ASSERT_EQ(wr_cmd_stop_stream(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0x83);
    WR_ASSERT_EQ(wr_cmd_power_off(&w), WR_OK);
    WR_ASSERT_EQ(w.data[0], 0xFA);
}

WR_TEST(command_emit_rejects_oversized_and_empty_writes)
{
    wr_write_request w;
    uint8_t big[WR_MAX_COMMAND_LEN + 1];
    memset(big, 0x81, sizeof(big));
    WR_ASSERT_EQ(wr_command_emit(&w, big, sizeof(big), false), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_command_emit(&w, big, 0, false), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_command_emit(NULL, big, 1, false), WR_ERR_INVALID_ARG);
}

WR_TEST_MAIN()
