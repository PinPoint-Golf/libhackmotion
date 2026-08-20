/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * wr_presence.h — internal.  The pure half of the calibration presence check.
 */
#ifndef WR_PRESENCE_H
#define WR_PRESENCE_H

#include "wrist/types.h"
#include "wrist/sample.h"
#include "wrist/event.h"

/*
 * ⚠ Returns WR_CAL_UNKNOWN for the band between the two populations §8.2
 * measured, and for NaN.  It never returns a score and never ranks attempts.
 */
wr_calibration_state wr_presence_classify(float angle_deg);

/*
 * Picks the reference-pose anchor out of a short run of live samples taken at a
 * pose the application declared known, and fills the event the session emits.
 *
 * Uses at most WR_PRESENCE_MAX_SAMPLES.
 *
 * ⚠ The record chosen is the MEDOID — the one whose relative ROTATION is
 * nearest the run's average rotation.  NOT the one whose relative ANGLE is the
 * median: near zero a per-sample angle is dominated by Q14 quantisation and
 * `acos` rectifies that noise, so a median of angles biases high.  wr_presence.c
 * has the arithmetic; test_presence.c pins the distinction with a run whose two
 * answers differ.
 *
 * Also fills the averaged absolute pose and its spread, which is what a frame
 * reconciliation actually wants — the medoid is selected on the relative
 * rotation and is therefore blind to a whole-arm wobble.
 */
wr_status wr_presence_select_reference(const wr_sample *samples, size_t count,
                                       wr_calibration_presence_event *out);

#endif /* WR_PRESENCE_H */
