/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_device.c — discovery data, GATT identifiers, policy defaults and the
 * public ABI surface.  Nothing here talks to a radio, which is the point.
 */
#include "wr_test.h"
#include "wrist/wrist.h"
#include <stddef.h>

/* §2.3 — and ⚠ the data characteristic does NOT sit under the service's UUID
 * base, so a transport deriving one from the other by fragment substitution
 * cannot express this device (api-request §2.0.5). */
WR_TEST(device_gatt_uuids_match_the_specification)
{
    char text[WR_UUID_STRING_SIZE];

    WR_ASSERT_STR(wr_uuid_format(&WR_UUID_TRANSPARENT_UART_SERVICE, text, sizeof(text)),
                  "49535343-fe7d-4ae5-8fa9-9fafd205e455");
    WR_ASSERT_STR(wr_uuid_format(&WR_UUID_DATA_CHARACTERISTIC, text, sizeof(text)),
                  "413c3893-b7e8-4231-9673-7af7aed06ddc");
    WR_ASSERT_STR(wr_uuid_format(&WR_UUID_OTA_SERVICE_FORBIDDEN, text, sizeof(text)),
                  "1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0");
    WR_ASSERT_STR(wr_uuid_format(&WR_UUID_ISSC_PIPE_INERT, text, sizeof(text)),
                  "49535343-1e4d-4bd9-ba61-23c647249616");

    WR_ASSERT_MSG(!wr_uuid_equal(&WR_UUID_TRANSPARENT_UART_SERVICE,
                                 &WR_UUID_DATA_CHARACTERISTIC),
                  "service and characteristic share no base — do not derive one from the other");
}

WR_TEST(device_uuid_parse_round_trips)
{
    wr_uuid u;
    char text[WR_UUID_STRING_SIZE];

    WR_ASSERT_EQ(wr_uuid_parse("413C3893-B7E8-4231-9673-7AF7AED06DDC", &u), WR_OK);
    WR_ASSERT(wr_uuid_equal(&u, &WR_UUID_DATA_CHARACTERISTIC));
    WR_ASSERT_EQ(wr_uuid_parse("{413c3893-b7e8-4231-9673-7af7aed06ddc}", &u), WR_OK);
    WR_ASSERT(wr_uuid_equal(&u, &WR_UUID_DATA_CHARACTERISTIC));
    WR_ASSERT_STR(wr_uuid_format(&u, text, sizeof(text)),
                  "413c3893-b7e8-4231-9673-7af7aed06ddc");

    WR_ASSERT_EQ(wr_uuid_parse("nonsense", &u), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_uuid_parse("413c3893b7e842319673-7af7aed06ddc", &u), WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_uuid_parse("413c3893-b7e8-4231-9673-7af7aed06ddcXX", &u),
                 WR_ERR_INVALID_ARG);
    WR_ASSERT_EQ(wr_uuid_parse(NULL, &u), WR_ERR_INVALID_ARG);
}

/* §2.1 — the host scans; we supply the filter. */
WR_TEST(device_advertisement_matching)
{
    wr_uuid services[2];

    WR_ASSERT(wr_looks_like_sensor(WR_ADVERTISED_LOCAL_NAME, NULL, 0));
    WR_ASSERT(!wr_looks_like_sensor("HackMotion", NULL, 0));
    WR_ASSERT(!wr_looks_like_sensor(NULL, NULL, 0));

    /* Many stacks report no name; a service match is enough on its own. */
    services[0] = WR_UUID_ISSC_PIPE_INERT;
    services[1] = WR_UUID_TRANSPARENT_UART_SERVICE;
    WR_ASSERT(wr_looks_like_sensor(NULL, services, 2));
    WR_ASSERT(!wr_looks_like_sensor(NULL, services, 1));
}

/*
 * ⚠ §2.1 — THE GENERATION IS PART OF THE ADVERTISED NAME.  "wG3" is wrist,
 * generation 3, so a later sensor advertises a different string.  Matching the
 * exact name would hide it on every stack that reports no service UUIDs — and
 * the library's own contract says a name match alone has to be enough, because
 * many of them do exactly that.
 *
 * So the filter is the FAMILY prefix.  Being found is not being supported:
 * what this library can actually speak to is settled after link-up, by the MTU
 * floor and by the version reply.
 */
WR_TEST(device_discovery_finds_a_generation_this_library_has_never_seen)
{
    /* The measured device, and its siblings — past and future. */
    WR_ASSERT(wr_looks_like_sensor("HackMotion wG3", NULL, 0));
    WR_ASSERT(wr_looks_like_sensor("HackMotion wG4", NULL, 0));
    WR_ASSERT(wr_looks_like_sensor("HackMotion wG1", NULL, 0));
    /* Two digits, whenever that day comes: no arithmetic is done on the name. */
    WR_ASSERT(wr_looks_like_sensor("HackMotion wG12", NULL, 0));

    /* ⚠ And the prefix is still a discriminator, not a wildcard.  A HackMotion
     * product that is not a wrist sensor is NOT offered on its name. */
    WR_ASSERT(!wr_looks_like_sensor("HackMotion Launch Monitor", NULL, 0));
    WR_ASSERT(!wr_looks_like_sensor("HackMotion", NULL, 0));
    WR_ASSERT(!wr_looks_like_sensor("Hack", NULL, 0));
    WR_ASSERT(!wr_looks_like_sensor("", NULL, 0));
    /* Not a substring match: the name must BEGIN with the family. */
    WR_ASSERT(!wr_looks_like_sensor("My HackMotion wG3", NULL, 0));

    /* The measured name is still the documented one, and still matches. */
    WR_ASSERT_STR(WR_ADVERTISED_LOCAL_NAME, "HackMotion wG3");
    WR_ASSERT(wr_looks_like_sensor(WR_ADVERTISED_LOCAL_NAME, NULL, 0));
}

/* §2.4 / api-request §2.13 — the calibration result is 65 bytes and stream
 * notifications reach 93, so 96 is the floor and the library CHECKS it. */
WR_TEST(device_mtu_floor_covers_the_largest_message)
{
    WR_ASSERT(WR_MIN_ATT_MTU >= 93 + 3);
    WR_ASSERT_EQ(WR_MIN_ATT_MTU, 96);
}

/* §9.2 — the keepalive is a correctness requirement, not insurance. */
WR_TEST(device_timing_constants_match_the_specification)
{
    WR_ASSERT_EQ(WR_IDLE_SHUTDOWN_US, 300000000LL);
    WR_ASSERT_EQ(WR_KEEPALIVE_PERIOD_US, 30000000LL);
    WR_ASSERT(WR_KEEPALIVE_PERIOD_US * 2 < WR_IDLE_SHUTDOWN_US);
    WR_ASSERT_EQ(WR_SAMPLE_INDEX_MODULUS, 65536u);
    /* §6.5 — the counter wraps every 82.0 s at ≈799.2 Hz. */
    WR_ASSERT_NEAR((double)WR_SAMPLE_INDEX_MODULUS / WR_NOMINAL_SAMPLE_RATE_HZ, 82.0, 0.1);
    /* §7.6 — 3 s + 1.5 s against a ~7.5 s buffer leaves usable margin. */
    WR_ASSERT(WR_DEFAULT_PRE_ROLL_US + WR_DEFAULT_POST_ROLL_US < WR_HISTORY_DEPTH_SEED_US);
}

/*
 * ⚠ Every timing field must get a default, and the check is mechanical because
 * diligence has already failed here.  A field added to wr_session_policy but
 * forgotten in wr_session_policy_default() reads as 0, and 0 means "use the
 * default" — so the omission is invisible for every field whose handler honours
 * that convention, and a live bug for every field whose handler does not.
 *
 * Walking the struct as bytes rather than naming fields is deliberate: a named
 * list would need updating alongside the struct, which is the very thing that
 * keeps being forgotten.
 */
WR_TEST(device_every_policy_timing_field_has_a_default)
{
    wr_session_policy p = wr_session_policy_default();
    const wr_time_us *fields = (const wr_time_us *)(const void *)&p;
    size_t n = offsetof(wr_session_policy, accuracy_drift_us_per_s) / sizeof(wr_time_us);
    size_t zero_count = 0;

    /* Every wr_time_us field precedes accuracy_drift_us_per_s in the struct. */
    WR_ASSERT(n >= 9);
    for (size_t i = 0; i < n; ++i) {
        if (fields[i] == 0) {
            zero_count++;
        }
        WR_ASSERT_MSG(fields[i] >= 0, "no timing default may be negative");
    }
    WR_ASSERT_MSG(zero_count == 0,
                  "a timing policy field was added to the struct but not given a default");
}

/* The three triggers that were prose-only until the contract sweep found them. */
WR_TEST(device_alarm_thresholds_are_numbers_not_prose)
{
    wr_session_policy p = wr_session_policy_default();

    /* §6.1 — the first 0x90 arrives in 50-80 ms; 3 s is two orders of margin. */
    WR_ASSERT_EQ(p.stream_start_timeout_us, 3000000LL);

    /* ⚠ Must warn well before the 300 s idle shutdown, and after enough missed
     * 30 s polls that it means something. */
    WR_ASSERT(p.keepalive_alarm_us > 3 * WR_KEEPALIVE_PERIOD_US);
    WR_ASSERT_MSG(p.keepalive_alarm_us < WR_IDLE_SHUTDOWN_US / 2,
                  "the keepalive alarm must leave time to act");

    /* ⚠ Shorter than a full-depth pull, which is why it must be suppressed
     * inside a bracket rather than relying on the threshold. */
    WR_ASSERT(p.live_gap_alarm_us > 0);
    WR_ASSERT_MSG(p.live_gap_alarm_us < WR_HISTORY_DEPTH_SEED_US,
                  "a live-gap alarm longer than a pull would never fire; it is "
                  "suppressed during a bracket instead");

    WR_ASSERT(p.pinned_report_period_us > 0);
}

WR_TEST(device_policy_defaults_are_the_measured_ones)
{
    wr_session_policy p = wr_session_policy_default();
    WR_ASSERT_EQ(p.keepalive_period_us, WR_KEEPALIVE_PERIOD_US);
    /* ⚠ 2.2 ms per second of session, expressed in µs/s. */
    WR_ASSERT_NEAR(p.accuracy_drift_us_per_s, 2200.0, 1e-9);
    WR_ASSERT_EQ(p.history_pre_roll_us, WR_DEFAULT_PRE_ROLL_US);
    WR_ASSERT_EQ(p.history_post_roll_us, WR_DEFAULT_POST_ROLL_US);
    /* ⚠ identifiers stay out of recordings unless explicitly asked. */
    WR_ASSERT(!p.record_identifiers);
}

/* §7.6 — a request is sized around the event, and never around the buffer. */
WR_TEST(device_history_request_defaults_follow_section_7_6)
{
    wr_history_request r = wr_history_request_around(NULL, 1000000000LL);
    WR_ASSERT_EQ(r.window.start_us, 1000000000LL - 3000000LL);
    WR_ASSERT_EQ(r.window.end_us, 1000000000LL + 1500000LL);
    WR_ASSERT(r.window.end_us - r.window.start_us == 4500000LL);
    WR_ASSERT(r.refill_gaps);
    WR_ASSERT(r.max_attempts >= 1);
    WR_ASSERT(r.deadline_us > r.window.end_us);
}

/*
 * ⚠ The policy's pre/post-roll fields must actually reach the function that
 * documents them.  A policy field nothing can read is a policy field that does
 * not exist, and the previous signature could not see these at all.
 */
WR_TEST(device_history_request_honours_the_policy)
{
    wr_session_policy p = wr_session_policy_default();
    wr_history_request r;

    p.history_pre_roll_us = 2000000LL;
    p.history_post_roll_us = 500000LL;
    r = wr_history_request_around(&p, 1000000000LL);

    WR_ASSERT_EQ(r.window.start_us, 1000000000LL - 2000000LL);
    WR_ASSERT_EQ(r.window.end_us, 1000000000LL + 500000LL);
    /* The deadline scales with the window, since a pull takes about as long as
     * its span (§7.4). */
    WR_ASSERT_EQ(r.deadline_us, r.window.end_us + 2 * 2500000LL + 1000000LL);
}

/*
 * ⚠ A mandatory justification that is discarded is worse than none: it reads as
 * an audit trail without being one.  It must survive the copy into the block.
 */
WR_TEST(device_nonstandard_config_carries_its_justification)
{
    wr_stream_config c = wr_stream_config_nonstandard(0x5e, "decoding a 2026-08 archive");
    wr_stream_config copy = c; /* as wr_history_block carries it, by value */

    WR_ASSERT_STR(copy.justification, "decoding a 2026-08 archive");
    WR_ASSERT_EQ(copy.bits, 0x5e);
    WR_ASSERT(!wr_stream_config_is_observed_default(copy));

    /*
     * ⚠ Bound to a named local before the member is read, and that is required
     * rather than tidy: the temporary returned by these calls dies at the end of
     * the full expression, so the pointer WR_ASSERT_STR saves would dangle
     * before strcmp() ran (clang -Wdangling).  Undefined behaviour that gcc
     * happens not to diagnose.
     */
    {
        const wr_stream_config def = wr_stream_config_default();
        const wr_stream_config null_just = wr_stream_config_nonstandard(0x5e, NULL);
        const wr_stream_config empty_just = wr_stream_config_nonstandard(0x5e, "");

        /* The default carries none, because there is nothing to justify. */
        WR_ASSERT_STR(def.justification, "");
        /* And a caller who supplies nothing gets a recording that says so. */
        WR_ASSERT_STR(null_just.justification, "(unjustified)");
        WR_ASSERT_STR(empty_just.justification, "(unjustified)");
    }

    /* Over-long text is truncated, never overrun. */
    {
        char big[256];
        memset(big, 'x', sizeof(big) - 1);
        big[sizeof(big) - 1] = '\0';
        WR_ASSERT_EQ(strlen(wr_stream_config_nonstandard(0x5e, big).justification),
                     WR_CONFIG_JUSTIFICATION_MAX - 1);
    }
}

/* api-request §2.13 — identifiers are redacted from default formatting. */
WR_TEST(device_identity_events_are_marked_sensitive_and_redacted)
{
    wr_event ev;
    char line[256];

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)WR_EV_IDENTITY;
    memcpy(ev.u.device_info.serial, "WG3-00042", 10);
    memcpy(ev.u.device_info.mac, "01:23:45:67:89:AB", 18);

    WR_ASSERT(wr_event_is_sensitive(&ev));
    (void)wr_event_format(&ev, line, sizeof(line), false);
    WR_ASSERT(strstr(line, "WG3-00042") == NULL);
    WR_ASSERT(strstr(line, "01:23:45") == NULL);
    WR_ASSERT(strstr(line, "redacted") != NULL);

    (void)wr_event_format(&ev, line, sizeof(line), true);
    WR_ASSERT(strstr(line, "WG3-00042") != NULL);

    /*
     * ⚠ AND WR_EV_DEVICE_INFO SHIPS THE SAME STRUCT, identifiers included.
     * `enter_ready()` copies the whole `wr_device_info`, and bring-up does not
     * reach READY until the MAC and the serial have arrived — so this event
     * carries both on every normal connection (implementation-review I4).
     *
     * ⚠ It read as safe because wr_event_format() does not PRINT them for this
     * type, so the leak test below passed while the mechanism design §9.2 tells
     * a consumer to filter their log sink on was wrong.  Two safeguards, one
     * tested.  The question this asks is what the PRODUCER copies into the
     * union, not what the formatter renders.
     */
    ev.type = (uint16_t)WR_EV_DEVICE_INFO;
    WR_ASSERT_MSG(wr_event_is_sensitive(&ev),
                  "WR_EV_DEVICE_INFO carries the MAC and the serial by value");

    ev.type = (uint16_t)WR_EV_BATTERY;
    WR_ASSERT(!wr_event_is_sensitive(&ev));
}

/*
 * Every event must render to a log line without a crash, a truncation fault or
 * an identifier leak.  A logging path that only works for the events a
 * developer happened to hit is a logging path that fails during the incident.
 */
WR_TEST(device_every_event_type_formats_safely)
{
    char line[256];
    char tiny[8];

    for (int i = 0; i < (int)WR_EVENT_TYPE_COUNT; ++i) {
        wr_event ev;
        int n;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)i;
        memcpy(ev.u.device_info.serial, "WG3-00042", 10);
        memcpy(ev.u.device_info.mac, "01:23:45:67:89:AB", 18);

        n = wr_event_format(&ev, line, sizeof(line), false);
        WR_ASSERT(n > 0);
        WR_ASSERT(strlen(line) > 0);
        WR_ASSERT_MSG(strstr(line, "WG3-00042") == NULL,
                      "no event may leak a serial with identifiers disabled");
        WR_ASSERT_MSG(strstr(line, "01:23:45:67:89:AB") == NULL,
                      "no event may leak a MAC with identifiers disabled");

        /* snprintf semantics: report what WOULD have been written. */
        WR_ASSERT(wr_event_format(&ev, tiny, sizeof(tiny), false) > 0);
    }
    WR_ASSERT_EQ(wr_event_format(NULL, line, sizeof(line), false), -1);
    WR_ASSERT_EQ(wr_event_format(NULL, NULL, 0, false), -1);
}

/* Every enum name must be non-NULL and distinct enough to log. */
WR_TEST(device_enum_names_are_complete)
{
    for (int i = 0; i < (int)WR_EVENT_TYPE_COUNT; ++i) {
        WR_ASSERT_STR(wr_event_type_name((wr_event_type)i) ? "ok" : NULL, "ok");
        WR_ASSERT(strcmp(wr_event_type_name((wr_event_type)i), "invalid") != 0);
    }
    for (int i = 0; i < (int)WR_WARN_CODE_COUNT; ++i) {
        WR_ASSERT(strcmp(wr_warning_code_name((wr_warning_code)i), "invalid") != 0);
    }
    for (int i = 0; i < (int)WR_CALIBRATION_PHASE_COUNT; ++i) {
        WR_ASSERT(strcmp(wr_calibration_phase_name((wr_calibration_phase)i), "invalid") != 0);
    }
    for (int i = 0; i < (int)WR_HISTORY_STATUS_COUNT; ++i) {
        WR_ASSERT(strcmp(wr_history_status_name((wr_history_status)i), "invalid") != 0);
    }
    WR_ASSERT_STR(wr_unit_name(WR_UNIT_LOWER_ARM), "lower_arm");
    WR_ASSERT_STR(wr_unit_name(WR_UNIT_PALM), "palm");
    WR_ASSERT_STR(wr_channel_name(WR_CH_GYRO_Z), "gyro_z");
    WR_ASSERT_STR(wr_status_str(WR_ERR_NOT_ALLOWED), "command not on the allowlist");
}

/* api-request C11 — a POD layout with a version stamp, so a recording made
 * today is still decodable when the struct grows. */
WR_TEST(device_abi_surface_is_self_describing)
{
    wr_abi_sizes sizes;
    wr_abi_sizes_get(&sizes);

    WR_ASSERT_EQ(sizes.abi_version, WR_ABI_VERSION);
    WR_ASSERT_EQ(sizes.sample, sizeof(wr_sample));
    WR_ASSERT_EQ(sizes.sample_layout_version, WR_SAMPLE_LAYOUT_VERSION);
    WR_ASSERT_EQ(wr_abi_check(&sizes), WR_OK);

    sizes.sample += 8;
    WR_ASSERT_EQ(wr_abi_check(&sizes), WR_ERR_NOT_SUPPORTED);
    WR_ASSERT_EQ(wr_abi_check(NULL), WR_ERR_INVALID_ARG);
    WR_ASSERT_STR(wr_version_string(), WR_VERSION_STRING);
}

/* The public types must stay copyable POD: no pointers a consumer could be
 * handed and then have invalidated, and no padding surprises across a binding. */
WR_TEST(device_public_structs_are_flat_pod)
{
    wr_sample a, b;
    memset(&a, 0x5a, sizeof(a));
    b = a;
    WR_ASSERT_EQ(memcmp(&a, &b, sizeof(a)), 0);

    WR_ASSERT_EQ(sizeof(wr_write_request), 12);
    WR_ASSERT(sizeof(wr_unit_sample) % 8 == 0);
    WR_ASSERT(sizeof(wr_sample) % 8 == 0);
    WR_ASSERT(sizeof(wr_wire_chunk) >= WR_WIRE_CHUNK_MAX);
}

/* §6.2 / api-request §2.12 — the configuration type carries the warnings. */
WR_TEST(device_stream_config_describes_itself)
{
    char text[WR_CONFIG_DESCRIBE_SIZE];
    wr_stream_config d = wr_stream_config_default();
    wr_stream_config legacy = wr_stream_config_legacy();
    wr_stream_config odd = wr_stream_config_nonstandard(0x5e, "test");

    WR_ASSERT_EQ(d.bits, 0x7e);
    WR_ASSERT(wr_stream_config_is_observed_default(d));
    WR_ASSERT(wr_stream_config_is_time_alignable(d));
    WR_ASSERT_EQ(wr_stream_config_gyro_divisor(d), 8);
    WR_ASSERT_NEAR(wr_stream_config_gyro_full_scale_dps(d), 4096.0, 1e-9);

    WR_ASSERT(wr_stream_config_is_legacy(legacy));
    WR_ASSERT_MSG(!wr_stream_config_is_time_alignable(legacy),
                  "the legacy layout carries no device clock at all");
    WR_ASSERT_EQ(wr_stream_config_record_size(legacy), 42);
    WR_ASSERT_EQ(wr_stream_config_block_size(legacy), 21);

    WR_ASSERT(!wr_stream_config_is_observed_default(odd));
    WR_ASSERT(strstr(wr_stream_config_describe(odd, text, sizeof(text)), "NONSTANDARD") != NULL);
    WR_ASSERT(strstr(wr_stream_config_describe(d, text, sizeof(text)), "default") != NULL);
    /* Bit 6 clear halves the range, at which point a hand shake AND a golf
     * swing both clip (§6.4). */
    WR_ASSERT_NEAR(wr_stream_config_gyro_full_scale_dps(wr_stream_config_nonstandard(0x3e, "t")),
                   2048.0, 1e-9);
}

WR_TEST_MAIN()
