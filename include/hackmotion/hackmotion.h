/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hackmotion.h — umbrella header for libhackmotion.
 *
 * A sans-I/O C11 library for HackMotion wrist sensors.
 *
 * ⚠ MEASURED ON ONE GENERATION.  "wG3" is wrist, generation 3, and every
 * constant in these headers was established on that product (id 0x14).  Later
 * generations are expected: discovery matches the family so they are still
 * found, and hm_session raises HM_WARN_UNVERIFIED_PRODUCT so a recording says
 * which hardware produced it.  Feature gating reads the PROTOCOL version, which
 * is generation-independent by design.
 *
 *   docs/specification.md   what the protocol is
 *   docs/api-request.md     what the first consuming application asked for
 *   docs/design.md          how this library answers both, and why
 *
 * The five facts about this device that the API deliberately makes hard to
 * ignore, because smoothing any of them away produces confidently wrong data
 * with no error anywhere:
 *
 *   1. The device's clock is authoritative and runs at ≈799.2 Hz, NOT 800.
 *   2. It applies its own calibration, and that calibration dies on a plain
 *      disconnect.  The one figure that looks like a quality score inverts.
 *   3. It is two sensors presented as one peripheral, 3-8 cm apart, with
 *      independent clocks 0.92 ms out, whose accelerations are SUPPOSED to
 *      disagree and whose quaternions map world → body.
 *   4. Its real data arrives after the fact, and nothing but a live clock fit
 *      can date it.
 *   5. It should not own the radio.
 */
#ifndef HACKMOTION_H
#define HACKMOTION_H

#include "hackmotion/version.h"
#include "hackmotion/types.h"
#include "hackmotion/device.h"
#include "hackmotion/config.h"
#include "hackmotion/sample.h"
#include "hackmotion/quat.h"
#include "hackmotion/clock.h"
#include "hackmotion/coverage.h"
#include "hackmotion/event.h"
#include "hackmotion/history.h"
#include "hackmotion/session.h"

#endif /* HACKMOTION_H */
