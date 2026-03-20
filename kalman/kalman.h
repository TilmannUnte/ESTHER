/*
 * Copyright (c) 2025-2026 Tilmann Unte, Sebastian R. Stecher
 * SPDX-License-Identifier:  Apache-2.0
 */

/**
 * @file
 * Provides a simple 1D Kalman Filter used to track a close distance point in
 * front of the Magni robot, as detected by the SlamTech RPLidar.
 * The application layer is expected to call the kalman_filter_step function
 * periodically.
 *
 * Internally this Kalman Filter implementation uses fixed-point math. It requires
 * LibFixMath to function. Distance measurements are expected to be in
 * meters, velocity in m/s, and angles in radians.
 *
 * The Kalman Filter is incapable of quickly reacting to dynamic change within
 * the environment, as this is not modeled. 
 */
 
#ifndef KALMAN_H
#define KALMAN_H

#include <zephyr/kernel.h>
#include "fixmath.h"

struct kalman_config {
	fix16_t initial_state;
	fix16_t initial_pred;
	fix16_t initial_gain;
	fix16_t initial_state_uncertainty;
	fix16_t initial_pred_uncertainty;
	fix16_t initial_meas_uncertainty;
	fix16_t noise;
	/* TODO: Use recalibration tolerance and resistance to reset Kalman Filter in dynamic environment */
	/* fix16_t recalibration_tolerance; */
	/* fix16_t recalibration_resistance; */
};

struct kalman_data {
	fix16_t state;
	fix16_t prediction;
	fix16_t gain;
	fix16_t pred_uncertainty; // = covariance
	fix16_t state_uncertainty;
	fix16_t meas_uncertainty;
	uint64_t count;
	int64_t prev_timestamp;
	
	/* TODO: pose? takes three separate arguments, so not easily expressed as atomic_t */
};

struct kalman {
	const struct kalman_config *config;
	struct kalman_data *data;
};

extern const struct kalman_config kalman_config_0;

extern struct kalman_data kalman_data_0;

fix16_t kalman_step(struct kalman* kf, fix16_t dist, fix16_t angle, fix16_t vel);
int kalman_init(struct kalman* kf);
int kalman_reset(struct kalman* kf);

#endif /* KALMAN_H */
