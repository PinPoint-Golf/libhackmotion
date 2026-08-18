/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * test_device.c — discovery data, GATT identifiers, policy defaults and the
 * public ABI surface.  Nothing here talks to a radio, which is the point.
 */
#include "hm_test.h"
#include "hackmotion/hackmotion.h"
#include <stddef.h>

/* §2.3 — and ⚠ the data characteristic does NOT sit under the service's UUID
 * base, so a transport deriving one from the other by fragment substitution
 * cannot express this device (api-request §2.0.5). */
HM_TEST(device_gatt_uuids_match_the_specification)
{
    char text[HM_UUID_STRING_SIZE];

    HM_ASSERT_STR(hm_uuid_format(&HM_UUID_TRANSPARENT_UART_SERVICE, text, sizeof(text)),
                  "49535343-fe7d-4ae5-8fa9-9fafd205e455");
    HM_ASSERT_STR(hm_uuid_format(&HM_UUID_DATA_CHARACTERISTIC, text, sizeof(text)),
                  "413c3893-b7e8-4231-9673-7af7aed06ddc");
    HM_ASSERT_STR(hm_uuid_format(&HM_UUID_OTA_SERVICE_FORBIDDEN, text, sizeof(text)),
                  "1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0");
    HM_ASSERT_STR(hm_uuid_format(&HM_UUID_ISSC_PIPE_INERT, text, sizeof(text)),
                  "49535343-1e4d-4bd9-ba61-23c647249616");

    HM_ASSERT_MSG(!hm_uuid_equal(&HM_UUID_TRANSPARENT_UART_SERVICE,
                                 &HM_UUID_DATA_CHARACTERISTIC),
                  "service and characteristic share no base — do not derive one from the other");
}

HM_TEST(device_uuid_parse_round_trips)
{
    hm_uuid u;
    char text[HM_UUID_STRING_SIZE];

    HM_ASSERT_EQ(hm_uuid_parse("413C3893-B7E8-4231-9673-7AF7AED06DDC", &u), HM_OK);
    HM_ASSERT(hm_uuid_equal(&u, &HM_UUID_DATA_CHARACTERISTIC));
    HM_ASSERT_EQ(hm_uuid_parse("{413c3893-b7e8-4231-9673-7af7aed06ddc}", &u), HM_OK);
    HM_ASSERT(hm_uuid_equal(&u, &HM_UUID_DATA_CHARACTERISTIC));
    HM_ASSERT_STR(hm_uuid_format(&u, text, sizeof(text)),
                  "413c3893-b7e8-4231-9673-7af7aed06ddc");

    HM_ASSERT_EQ(hm_uuid_parse("nonsense", &u), HM_ERR_INVALID_ARG);
    HM_ASSERT_EQ(hm_uuid_parse("413c3893b7e842319673-7af7aed06ddc", &u), HM_ERR_INVALID_ARG);
    HM_ASSERT_EQ(hm_uuid_parse("413c3893-b7e8-4231-9673-7af7aed06ddcXX", &u),
                 HM_ERR_INVALID_ARG);
    HM_ASSERT_EQ(hm_uuid_parse(NULL, &u), HM_ERR_INVALID_ARG);
}

/* §2.1 — the host scans; we supply the filter. */
HM_TEST(device_advertisement_matching)
{
    hm_uuid services[2];

    HM_ASSERT(hm_looks_like_hackmotion(HM_ADVERTISED_LOCAL_NAME, NULL, 0));
    HM_ASSERT(!hm_looks_like_hackmotion("HackMotion", NULL, 0));
    HM_ASSERT(!hm_looks_like_hackmotion(NULL, NULL, 0));

    /* Many stacks report no name; a service match is enough on its own. */
    services[0] = HM_UUID_ISSC_PIPE_INERT;
    services[1] = HM_UUID_TRANSPARENT_UART_SERVICE;
    HM_ASSERT(hm_looks_like_hackmotion(NULL, services, 2));
    HM_ASSERT(!hm_looks_like_hackmotion(NULL, services, 1));
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
HM_TEST(device_discovery_finds_a_generation_this_library_has_never_seen)
{
    /* The measured device, and its siblings — past and future. */
    HM_ASSERT(hm_looks_like_hackmotion("HackMotion wG3", NULL, 0));
    HM_ASSERT(hm_looks_like_hackmotion("HackMotion wG4", NULL, 0));
    HM_ASSERT(hm_looks_like_hackmotion("HackMotion wG1", NULL, 0));
    /* Two digits, whenever that day comes: no arithmetic is done on the name. */
    HM_ASSERT(hm_looks_like_hackmotion("HackMotion wG12", NULL, 0));

    /* ⚠ And the prefix is still a discriminator, not a wildcard.  A HackMotion
     * product that is not a wrist sensor is NOT offered on its name. */
    HM_ASSERT(!hm_looks_like_hackmotion("HackMotion Launch Monitor", NULL, 0));
    HM_ASSERT(!hm_looks_like_hackmotion("HackMotion", NULL, 0));
    HM_ASSERT(!hm_looks_like_hackmotion("Hack", NULL, 0));
    HM_ASSERT(!hm_looks_like_hackmotion("", NULL, 0));
    /* Not a substring match: the name must BEGIN with the family. */
    HM_ASSERT(!hm_looks_like_hackmotion("My HackMotion wG3", NULL, 0));

    /* The measured name is still the documented one, and still matches. */
    HM_ASSERT_STR(HM_ADVERTISED_LOCAL_NAME, "HackMotion wG3");
    HM_ASSERT(hm_looks_like_hackmotion(HM_ADVERTISED_LOCAL_NAME, NULL, 0));
}

/* §2.4 / api-request §2.13 — the calibration result is 65 bytes and stream
 * notifications reach 93, so 96 is the floor and the library CHECKS it. */
HM_TEST(device_mtu_floor_covers_the_largest_message)
{
    HM_ASSERT(HM_MIN_ATT_MTU >= 93 + 3);
    HM_ASSERT_EQ(HM_MIN_ATT_MTU, 96);
}

/* §9.2 — the keepalive is a correctness requirement, not insurance. */
HM_TEST(device_timing_constants_match_the_specification)
{
    HM_ASSERT_EQ(HM_IDLE_SHUTDOWN_US, 300000000LL);
    HM_ASSERT_EQ(HM_KEEPALIVE_PERIOD_US, 30000000LL);
    HM_ASSERT(HM_KEEPALIVE_PERIOD_US * 2 < HM_IDLE_SHUTDOWN_US);
    HM_ASSERT_EQ(HM_SAMPLE_INDEX_MODULUS, 65536u);
    /* §6.5 — the counter wraps every 82.0 s at ≈799.2 Hz. */
    HM_ASSERT_NEAR((double)HM_SAMPLE_INDEX_MODULUS / HM_NOMINAL_SAMPLE_RATE_HZ, 82.0, 0.1);
    /* §7.6 — 3 s + 1.5 s against a ~7.5 s buffer leaves usable margin. */
    HM_ASSERT(HM_DEFAULT_PRE_ROLL_US + HM_DEFAULT_POST_ROLL_US < HM_HISTORY_DEPTH_SEED_US);
}

/*
 * ⚠ Every timing field must get a default, and the check is mechanical because
 * diligence has already failed here.  A field added to hm_session_policy but
 * forgotten in hm_session_policy_default() reads as 0, and 0 means "use the
 * default" — so the omission is invisible for every field whose handler honours
 * that convention, and a live bug for every field whose handler does not.
 *
 * Walking the struct as bytes rather than naming fields is deliberate: a named
 * list would need updating alongside the struct, which is the very thing that
 * keeps being forgotten.
 */
HM_TEST(device_every_policy_timing_field_has_a_default)
{
    hm_session_policy p = hm_session_policy_default();
    const hm_time_us *fields = (const hm_time_us *)(const void *)&p;
    size_t n = offsetof(hm_session_policy, accuracy_drift_us_per_s) / sizeof(hm_time_us);
    size_t zero_count = 0;

    /* Every hm_time_us field precedes accuracy_drift_us_per_s in the struct. */
    HM_ASSERT(n >= 9);
    for (size_t i = 0; i < n; ++i) {
        if (fields[i] == 0) {
            zero_count++;
        }
        HM_ASSERT_MSG(fields[i] >= 0, "no timing default may be negative");
    }
    HM_ASSERT_MSG(zero_count == 0,
                  "a timing policy field was added to the struct but not given a default");
}

/* The three triggers that were prose-only until the contract sweep found them. */
HM_TEST(device_alarm_thresholds_are_numbers_not_prose)
{
    hm_session_policy p = hm_session_policy_default();

    /* §6.1 — the first 0x90 arrives in 50-80 ms; 3 s is two orders of margin. */
    HM_ASSERT_EQ(p.stream_start_timeout_us, 3000000LL);

    /* ⚠ Must warn well before the 300 s idle shutdown, and after enough missed
     * 30 s polls that it means something. */
    HM_ASSERT(p.keepalive_alarm_us > 3 * HM_KEEPALIVE_PERIOD_US);
    HM_ASSERT_MSG(p.keepalive_alarm_us < HM_IDLE_SHUTDOWN_US / 2,
                  "the keepalive alarm must leave time to act");

    /* ⚠ Shorter than a full-depth pull, which is why it must be suppressed
     * inside a bracket rather than relying on the threshold. */
    HM_ASSERT(p.live_gap_alarm_us > 0);
    HM_ASSERT_MSG(p.live_gap_alarm_us < HM_HISTORY_DEPTH_SEED_US,
                  "a live-gap alarm longer than a pull would never fire; it is "
                  "suppressed during a bracket instead");

    HM_ASSERT(p.pinned_report_period_us > 0);
}

HM_TEST(device_policy_defaults_are_the_measured_ones)
{
    hm_session_policy p = hm_session_policy_default();
    HM_ASSERT_EQ(p.keepalive_period_us, HM_KEEPALIVE_PERIOD_US);
    /* ⚠ 2.2 ms per second of session, expressed in µs/s. */
    HM_ASSERT_NEAR(p.accuracy_drift_us_per_s, 2200.0, 1e-9);
    HM_ASSERT_EQ(p.history_pre_roll_us, HM_DEFAULT_PRE_ROLL_US);
    HM_ASSERT_EQ(p.history_post_roll_us, HM_DEFAULT_POST_ROLL_US);
    /* ⚠ identifiers stay out of recordings unless explicitly asked. */
    HM_ASSERT(!p.record_identifiers);
}

/* §7.6 — a request is sized around the event, and never around the buffer. */
HM_TEST(device_history_request_defaults_follow_section_7_6)
{
    hm_history_request r = hm_history_request_around(NULL, 1000000000LL);
    HM_ASSERT_EQ(r.window.start_us, 1000000000LL - 3000000LL);
    HM_ASSERT_EQ(r.window.end_us, 1000000000LL + 1500000LL);
    HM_ASSERT(r.window.end_us - r.window.start_us == 4500000LL);
    HM_ASSERT(r.refill_gaps);
    HM_ASSERT(r.max_attempts >= 1);
    HM_ASSERT(r.deadline_us > r.window.end_us);
}

/*
 * ⚠ The policy's pre/post-roll fields must actually reach the function that
 * documents them.  A policy field nothing can read is a policy field that does
 * not exist, and the previous signature could not see these at all.
 */
HM_TEST(device_history_request_honours_the_policy)
{
    hm_session_policy p = hm_session_policy_default();
    hm_history_request r;

    p.history_pre_roll_us = 2000000LL;
    p.history_post_roll_us = 500000LL;
    r = hm_history_request_around(&p, 1000000000LL);

    HM_ASSERT_EQ(r.window.start_us, 1000000000LL - 2000000LL);
    HM_ASSERT_EQ(r.window.end_us, 1000000000LL + 500000LL);
    /* The deadline scales with the window, since a pull takes about as long as
     * its span (§7.4). */
    HM_ASSERT_EQ(r.deadline_us, r.window.end_us + 2 * 2500000LL + 1000000LL);
}

/*
 * ⚠ A mandatory justification that is discarded is worse than none: it reads as
 * an audit trail without being one.  It must survive the copy into the block.
 */
HM_TEST(device_nonstandard_config_carries_its_justification)
{
    hm_stream_config c = hm_stream_config_nonstandard(0x5e, "decoding a 2026-08 archive");
    hm_stream_config copy = c; /* as hm_history_block carries it, by value */

    HM_ASSERT_STR(copy.justification, "decoding a 2026-08 archive");
    HM_ASSERT_EQ(copy.bits, 0x5e);
    HM_ASSERT(!hm_stream_config_is_observed_default(copy));

    /*
     * ⚠ Bound to a named local before the member is read, and that is required
     * rather than tidy: the temporary returned by these calls dies at the end of
     * the full expression, so the pointer HM_ASSERT_STR saves would dangle
     * before strcmp() ran (clang -Wdangling).  Undefined behaviour that gcc
     * happens not to diagnose.
     */
    {
        const hm_stream_config def = hm_stream_config_default();
        const hm_stream_config null_just = hm_stream_config_nonstandard(0x5e, NULL);
        const hm_stream_config empty_just = hm_stream_config_nonstandard(0x5e, "");

        /* The default carries none, because there is nothing to justify. */
        HM_ASSERT_STR(def.justification, "");
        /* And a caller who supplies nothing gets a recording that says so. */
        HM_ASSERT_STR(null_just.justification, "(unjustified)");
        HM_ASSERT_STR(empty_just.justification, "(unjustified)");
    }

    /* Over-long text is truncated, never overrun. */
    {
        char big[256];
        memset(big, 'x', sizeof(big) - 1);
        big[sizeof(big) - 1] = '\0';
        HM_ASSERT_EQ(strlen(hm_stream_config_nonstandard(0x5e, big).justification),
                     HM_CONFIG_JUSTIFICATION_MAX - 1);
    }
}

/* api-request §2.13 — identifiers are redacted from default formatting. */
HM_TEST(device_identity_events_are_marked_sensitive_and_redacted)
{
    hm_event ev;
    char line[256];

    memset(&ev, 0, sizeof(ev));
    ev.type = (uint16_t)HM_EV_IDENTITY;
    memcpy(ev.u.device_info.serial, "WG3-00042", 10);
    memcpy(ev.u.device_info.mac, "01:23:45:67:89:AB", 18);

    HM_ASSERT(hm_event_is_sensitive(&ev));
    (void)hm_event_format(&ev, line, sizeof(line), false);
    HM_ASSERT(strstr(line, "WG3-00042") == NULL);
    HM_ASSERT(strstr(line, "01:23:45") == NULL);
    HM_ASSERT(strstr(line, "redacted") != NULL);

    (void)hm_event_format(&ev, line, sizeof(line), true);
    HM_ASSERT(strstr(line, "WG3-00042") != NULL);

    /*
     * ⚠ AND HM_EV_DEVICE_INFO SHIPS THE SAME STRUCT, identifiers included.
     * `enter_ready()` copies the whole `hm_device_info`, and bring-up does not
     * reach READY until the MAC and the serial have arrived — so this event
     * carries both on every normal connection (implementation-review I4).
     *
     * ⚠ It read as safe because hm_event_format() does not PRINT them for this
     * type, so the leak test below passed while the mechanism design §9.2 tells
     * a consumer to filter their log sink on was wrong.  Two safeguards, one
     * tested.  The question this asks is what the PRODUCER copies into the
     * union, not what the formatter renders.
     */
    ev.type = (uint16_t)HM_EV_DEVICE_INFO;
    HM_ASSERT_MSG(hm_event_is_sensitive(&ev),
                  "HM_EV_DEVICE_INFO carries the MAC and the serial by value");

    ev.type = (uint16_t)HM_EV_BATTERY;
    HM_ASSERT(!hm_event_is_sensitive(&ev));
}

/*
 * Every event must render to a log line without a crash, a truncation fault or
 * an identifier leak.  A logging path that only works for the events a
 * developer happened to hit is a logging path that fails during the incident.
 */
HM_TEST(device_every_event_type_formats_safely)
{
    char line[256];
    char tiny[8];

    for (int i = 0; i < (int)HM_EVENT_TYPE_COUNT; ++i) {
        hm_event ev;
        int n;
        memset(&ev, 0, sizeof(ev));
        ev.type = (uint16_t)i;
        memcpy(ev.u.device_info.serial, "WG3-00042", 10);
        memcpy(ev.u.device_info.mac, "01:23:45:67:89:AB", 18);

        n = hm_event_format(&ev, line, sizeof(line), false);
        HM_ASSERT(n > 0);
        HM_ASSERT(strlen(line) > 0);
        HM_ASSERT_MSG(strstr(line, "WG3-00042") == NULL,
                      "no event may leak a serial with identifiers disabled");
        HM_ASSERT_MSG(strstr(line, "01:23:45:67:89:AB") == NULL,
                      "no event may leak a MAC with identifiers disabled");

        /* snprintf semantics: report what WOULD have been written. */
        HM_ASSERT(hm_event_format(&ev, tiny, sizeof(tiny), false) > 0);
    }
    HM_ASSERT_EQ(hm_event_format(NULL, line, sizeof(line), false), -1);
    HM_ASSERT_EQ(hm_event_format(NULL, NULL, 0, false), -1);
}

/* Every enum name must be non-NULL and distinct enough to log. */
HM_TEST(device_enum_names_are_complete)
{
    for (int i = 0; i < (int)HM_EVENT_TYPE_COUNT; ++i) {
        HM_ASSERT_STR(hm_event_type_name((hm_event_type)i) ? "ok" : NULL, "ok");
        HM_ASSERT(strcmp(hm_event_type_name((hm_event_type)i), "invalid") != 0);
    }
    for (int i = 0; i < (int)HM_WARN_CODE_COUNT; ++i) {
        HM_ASSERT(strcmp(hm_warning_code_name((hm_warning_code)i), "invalid") != 0);
    }
    for (int i = 0; i < (int)HM_CALIBRATION_PHASE_COUNT; ++i) {
        HM_ASSERT(strcmp(hm_calibration_phase_name((hm_calibration_phase)i), "invalid") != 0);
    }
    for (int i = 0; i < (int)HM_HISTORY_STATUS_COUNT; ++i) {
        HM_ASSERT(strcmp(hm_history_status_name((hm_history_status)i), "invalid") != 0);
    }
    HM_ASSERT_STR(hm_unit_name(HM_UNIT_LOWER_ARM), "lower_arm");
    HM_ASSERT_STR(hm_unit_name(HM_UNIT_PALM), "palm");
    HM_ASSERT_STR(hm_channel_name(HM_CH_GYRO_Z), "gyro_z");
    HM_ASSERT_STR(hm_status_str(HM_ERR_NOT_ALLOWED), "command not on the allowlist");
}

/* api-request C11 — a POD layout with a version stamp, so a recording made
 * today is still decodable when the struct grows. */
HM_TEST(device_abi_surface_is_self_describing)
{
    hm_abi_sizes sizes;
    hm_abi_sizes_get(&sizes);

    HM_ASSERT_EQ(sizes.abi_version, HM_ABI_VERSION);
    HM_ASSERT_EQ(sizes.sample, sizeof(hm_sample));
    HM_ASSERT_EQ(sizes.sample_layout_version, HM_SAMPLE_LAYOUT_VERSION);
    HM_ASSERT_EQ(hm_abi_check(&sizes), HM_OK);

    sizes.sample += 8;
    HM_ASSERT_EQ(hm_abi_check(&sizes), HM_ERR_NOT_SUPPORTED);
    HM_ASSERT_EQ(hm_abi_check(NULL), HM_ERR_INVALID_ARG);
    HM_ASSERT_STR(hm_version_string(), HM_VERSION_STRING);
}

/* The public types must stay copyable POD: no pointers a consumer could be
 * handed and then have invalidated, and no padding surprises across a binding. */
HM_TEST(device_public_structs_are_flat_pod)
{
    hm_sample a, b;
    memset(&a, 0x5a, sizeof(a));
    b = a;
    HM_ASSERT_EQ(memcmp(&a, &b, sizeof(a)), 0);

    HM_ASSERT_EQ(sizeof(hm_write_request), 12);
    HM_ASSERT(sizeof(hm_unit_sample) % 8 == 0);
    HM_ASSERT(sizeof(hm_sample) % 8 == 0);
    HM_ASSERT(sizeof(hm_wire_chunk) >= HM_WIRE_CHUNK_MAX);
}

/* §6.2 / api-request §2.12 — the configuration type carries the warnings. */
HM_TEST(device_stream_config_describes_itself)
{
    char text[HM_CONFIG_DESCRIBE_SIZE];
    hm_stream_config d = hm_stream_config_default();
    hm_stream_config legacy = hm_stream_config_legacy();
    hm_stream_config odd = hm_stream_config_nonstandard(0x5e, "test");

    HM_ASSERT_EQ(d.bits, 0x7e);
    HM_ASSERT(hm_stream_config_is_observed_default(d));
    HM_ASSERT(hm_stream_config_is_time_alignable(d));
    HM_ASSERT_EQ(hm_stream_config_gyro_divisor(d), 8);
    HM_ASSERT_NEAR(hm_stream_config_gyro_full_scale_dps(d), 4096.0, 1e-9);

    HM_ASSERT(hm_stream_config_is_legacy(legacy));
    HM_ASSERT_MSG(!hm_stream_config_is_time_alignable(legacy),
                  "the legacy layout carries no device clock at all");
    HM_ASSERT_EQ(hm_stream_config_record_size(legacy), 42);
    HM_ASSERT_EQ(hm_stream_config_block_size(legacy), 21);

    HM_ASSERT(!hm_stream_config_is_observed_default(odd));
    HM_ASSERT(strstr(hm_stream_config_describe(odd, text, sizeof(text)), "NONSTANDARD") != NULL);
    HM_ASSERT(strstr(hm_stream_config_describe(d, text, sizeof(text)), "default") != NULL);
    /* Bit 6 clear halves the range, at which point a hand shake AND a golf
     * swing both clip (§6.4). */
    HM_ASSERT_NEAR(hm_stream_config_gyro_full_scale_dps(hm_stream_config_nonstandard(0x3e, "t")),
                   2048.0, 1e-9);
}

HM_TEST_MAIN()
