/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion/device.h — device constants, GATT identifiers and discovery
 * matching data.
 *
 * THE LIBRARY DOES NOT SCAN.  api-request §2.0.5: every consumer that matters
 * already owns a scanner, an adapter pool and a device registry, and a second
 * scanner inside the same process contends with the first for one adapter.
 * So this header publishes what a scanner needs as data, and the host scans.
 */
#ifndef HACKMOTION_DEVICE_H
#define HACKMOTION_DEVICE_H

#include "hackmotion/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Discovery (spec §2.1)                                                     */
/* ------------------------------------------------------------------------ */

/* The advertised local name, exact.  Spec §2.1. */
#define HM_ADVERTISED_LOCAL_NAME "HackMotion wG3"

/*
 * DISCOVERY IS A RACE.  The sensor advertises for only a few seconds after a
 * physical button press and then stops, so a scanner must already be running
 * when the button is pressed.  Arm the scanner first, then prompt the user.
 *
 * ⚠ If the sensor has been asleep, the first press only WAKES it — it does not
 * advertise on that press.  Tell the user to press, pause, then press again.
 * Without this the library looks broken on exactly the first-run path where
 * the user's confidence matters most.
 *
 * The device vibrates when the link comes up (§9.5); that is the user's
 * confirmation that the race was won, and the best thing for a UI to key on.
 */
#define HM_RECOMMENDED_SCAN_WINDOW_US ((hm_time_us)90 * 1000 * 1000)

/* ------------------------------------------------------------------------ */
/* GATT (spec §2.3)                                                          */
/* ------------------------------------------------------------------------ */
/*
 * The whole protocol is a byte stream inside one bidirectional characteristic.
 * It is NOT a characteristic per quantity, and the data characteristic does
 * NOT sit under the service's UUID base — a transport that derives one from
 * the other by fragment substitution cannot express this device (api-request
 * §2.0.5).  Resolve them independently.
 */
HM_API extern const hm_uuid HM_UUID_TRANSPARENT_UART_SERVICE; /* 49535343-fe7d-4ae5-8fa9-9fafd205e455 */
HM_API extern const hm_uuid HM_UUID_DATA_CHARACTERISTIC;      /* 413c3893-b7e8-4231-9673-7af7aed06ddc */

/*
 * ⛔ NEVER WRITE TO THESE.  The Microchip OTA/DFU service is the firmware
 * update path and a bad write bricks the device (spec §2.3).  They are
 * published only so a transport can refuse them by construction rather than
 * by documentation, per api-request §2.16.
 */
HM_API extern const hm_uuid HM_UUID_OTA_SERVICE_FORBIDDEN;    /* 1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0 */

/* Inert: the stock ISSC pipe characteristic accepts writes and never replies.
 * Published so a transport that finds it first does not pick it. */
HM_API extern const hm_uuid HM_UUID_ISSC_PIPE_INERT;          /* 49535343-1e4d-4bd9-ba61-23c647249616 */

/*
 * The calibration result is 65 bytes and stream notifications reach 93, both
 * beyond the 20-byte payload of the default ATT MTU (spec §2.4).
 *
 * No platform lets an application request an MTU through Qt, so the library
 * cannot ENSURE this — it CHECKS it on link-up and fails loudly with the
 * negotiated value (api-request §2.13).  The alternative is truncated frames
 * that parse as garbage on whichever platform eventually gets it wrong.
 */
#define HM_MIN_ATT_MTU 96

/* ------------------------------------------------------------------------ */
/* Advertisement matching                                                    */
/* ------------------------------------------------------------------------ */

/*
 * True if an advertisement looks like a HackMotion wG3.
 *
 * `local_name` may be NULL.  `advertised_services` may be NULL with
 * `service_count == 0` — many stacks report no service UUIDs in the
 * advertisement, so a name match alone is accepted.
 *
 * This is a filter for a scanner the HOST runs.  It performs no I/O.
 */
HM_API bool hm_looks_like_hackmotion(const char *local_name,
                                     const hm_uuid *advertised_services,
                                     size_t service_count);

/* ------------------------------------------------------------------------ */
/* Device identity                                                           */
/* ------------------------------------------------------------------------ */
/*
 * ⚠ THE LIBRARY'S DEVICE IDENTITY IS AN OPAQUE STRING, NEVER A MAC.
 * api-request §2.0.4: CoreBluetooth never exposes a Bluetooth address — macOS
 * gives a per-host, per-application UUID that is not stable across machines.
 * Any API shaped as connect(MacAddress) is unimplementable there.
 *
 * The device does report its own MAC over the wire (0x85), and that IS a
 * stable cross-platform identifier — but it arrives AFTER connection, so it
 * can identify a device in a recording and cannot be used to find one.
 */
#define HM_DEVICE_ID_MAX      64
#define HM_SERIAL_MAX         16
#define HM_MAC_STRING_MAX     18

typedef struct hm_device_info {
    /* From 0x80 (spec §5.2).  `valid` bits say which replies have arrived. */
    uint8_t hardware_major, hardware_minor;
    uint8_t protocol_major, protocol_minor;   /* gates features — §6.2, §7.1 */
    uint8_t firmware_major, firmware_minor;   /* the vendor calls this "embedded version" */
    uint8_t product_id;                       /* 0x14 == 20 on the device under test */

    /* From 0x84 (spec §5.4). */
    uint8_t sensor_count;                     /* 2 — why every record carries two blocks */
    uint8_t sensor_map_undecoded;             /* observed 0x01 */

    /* From 0x81 (spec §5.3). */
    uint8_t  battery_percent;
    uint16_t status_undecoded;                /* observed 2226-2235.  NOT millivolts. */

    /*
     * ⚠ IDENTIFIERS.  These name a specific unit and a specific owner.  They
     * are never written to the library's log or to a wire recording unless the
     * caller explicitly asks (api-request §2.13).  Treat them as personal data.
     */
    char mac[HM_MAC_STRING_MAX];              /* from 0x85, ASCII hex, colon-separated */
    char serial[HM_SERIAL_MAX];               /* from 0x86, ASCII */

    uint32_t valid;                           /* bitmask of hm_device_info_field */
} hm_device_info;

typedef enum hm_device_info_field {
    HM_INFO_VERSIONS   = 1u << 0,
    HM_INFO_SENSOR_MAP = 1u << 1,
    HM_INFO_BATTERY    = 1u << 2,
    HM_INFO_MAC        = 1u << 3,
    HM_INFO_SERIAL     = 1u << 4
} hm_device_info_field;

/* ------------------------------------------------------------------------ */
/* Device-imposed limits (spec §12)                                          */
/* ------------------------------------------------------------------------ */

/* The device powers itself down after 5 minutes idle, and the timer runs
 * whether or not a central is connected — an active stream does NOT prevent
 * it (§9.2).  What resets it is a host→device write. */
#define HM_IDLE_SHUTDOWN_US            ((hm_time_us)300 * 1000 * 1000)

/* The vendor app's poll period, measured to be sufficient (§9.2).  The library
 * owns this unconditionally and offers no way to switch it off. */
#define HM_KEEPALIVE_PERIOD_US         ((hm_time_us)30 * 1000 * 1000)

/* `fa` sends no acknowledgement and the link stays up for ~9 s afterwards
 * (§9.3).  A teardown path that reads the gap as failure and retries into it
 * will be wrong.
 *
 * 12 s is DELIBERATE MARGIN over a single 9 s measurement, not a transcription
 * of it.  Do not "correct" it to 9: waiting three seconds too long costs
 * nothing, and giving up three seconds too early reintroduces the retry this
 * constant exists to prevent. */
#define HM_POWER_OFF_LINGER_US         ((hm_time_us)12 * 1000 * 1000)

/* The sample counter wraps every 82.0 s at ≈799.2 Hz (§6.5). */
#define HM_SAMPLE_INDEX_MODULUS        65536u

/* ⚠ Measured ONCE, after 20 s of streaming (§7.3).  Whether the buffer holds a
 * fixed sample count, a fixed duration, or something that varies with the live
 * rate is NOT established, and the three differ exactly during the dense bursts
 * of §6.6 — which is exactly when a client is pulling.
 *
 * This is a seed for the first request of a connection, not a contract.  The
 * library refines it from what the device actually returns (api-request B9). */
#define HM_HISTORY_DEPTH_SEED_US       ((hm_time_us)7500 * 1000)

/* The vendor-recommended capture window (§7.6).  4.5 s against a ~7.5 s buffer
 * leaves usable margin.  Requesting the whole buffer comes back HOLED, not
 * clamped, at 33-58 % coverage. */
#define HM_DEFAULT_PRE_ROLL_US         ((hm_time_us)3000 * 1000)
#define HM_DEFAULT_POST_ROLL_US        ((hm_time_us)1500 * 1000)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HACKMOTION_DEVICE_H */
