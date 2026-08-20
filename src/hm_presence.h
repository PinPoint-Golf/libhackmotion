/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_presence.h — internal.  The pure half of the calibration presence check.
 */
#ifndef HM_PRESENCE_H
#define HM_PRESENCE_H

#include "hackmotion/types.h"
#include "hackmotion/sample.h"
#include "hackmotion/event.h"

/*
 * ⚠ Returns HM_CAL_UNKNOWN for the band between the two populations §8.2
 * measured, and for NaN.  It never returns a score and never ranks attempts.
 */
hm_calibration_state hm_presence_classify(float angle_deg);

/*
 * Picks the reference-pose anchor out of a short run of live samples taken at a
 * pose the application declared known, and fills the event the session emits.
 *
 * Uses at most HM_PRESENCE_MAX_SAMPLES.
 *
 * ⚠ The record chosen is the MEDOID — the one whose relative ROTATION is
 * nearest the run's average rotation.  NOT the one whose relative ANGLE is the
 * median: near zero a per-sample angle is dominated by Q14 quantisation and
 * `acos` rectifies that noise, so a median of angles biases high.  hm_presence.c
 * has the arithmetic; test_presence.c pins the distinction with a run whose two
 * answers differ.
 *
 * Also fills the averaged absolute pose and its spread, which is what a frame
 * reconciliation actually wants — the medoid is selected on the relative
 * rotation and is therefore blind to a whole-arm wobble.
 */
hm_status hm_presence_select_reference(const hm_sample *samples, size_t count,
                                       hm_calibration_presence_event *out);

#endif /* HM_PRESENCE_H */
