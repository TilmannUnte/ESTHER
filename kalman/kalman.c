/*
 * Copyright (c) 2025-2026 Tilmann Unte, Sebastian R. Stecher
 * SPDX-License-Identifier:  Apache-2.0
*/

#include "kalman.h"
#include "fixmath.h"
#include "helpmath.h"

const struct kalman_config kalman_config_0 = {
	.initial_state = 0x00010000,
	.initial_pred = 0x00010000,
	.initial_gain = 0x00010000,
	.initial_state_uncertainty = 0x00010000,
	.initial_pred_uncertainty = 0x00010000,
	.initial_meas_uncertainty = 1, /* rounded up from 0.000009 */
	.noise = 1 /* rounded up from 0.000002 */
};

struct kalman_data kalman_data_0;

static void kalman_calc_gain(struct kalman *kf)
{
	struct kalman_data *data = kf->data;
	const struct kalman_config *cfg = kf->config;
	
	if (fix16_add(data->pred_uncertainty, data->meas_uncertainty) != 0) {
		data->gain = fix16_div(data->pred_uncertainty, fix16_add(data->pred_uncertainty
						, data->meas_uncertainty));
	} else {
		data->gain = cfg->initial_gain;
	}
}

static void kalman_state_update(struct kalman *kf, fix16_t meas)
{
	struct kalman_data *data = kf->data;
	
	data->state = fix16_add(data->prediction, fix16_mul(data->gain, fix16_sub(meas, data->prediction)));
}

static void kalman_state_uncertainty_update(struct kalman *kf)
{
	struct kalman_data *data = kf->data;
	
	data->state_uncertainty = fix16_mul(fix16_sub(fix16_one, data->gain), data->pred_uncertainty);
}

//TODO
static void kalman_state_extrapolate(struct kalman *kf, fix16_t secs_passed,
	fix16_t velocity, fix16_t angle)
{
	struct kalman_data *data = kf->data;

	fix16_t m_per_period = fix16_mul(secs_passed, velocity);
	
	if (rad_to_deg(angle) >= fix16_from_int(89) && rad_to_deg(angle) <= fix16_from_int(91)) {
		// robot is heading nearly straight into an obstacle
		data->prediction = fix16_sub(data->state, m_per_period);
	} else {
		// robot is passing the closest obstacle at an angle
		// adjust angle if it is not acute
		if (angle > PI_DIV_2) angle = fix16_sub(angle, PI_DIV_2);
		
		fix16_t adj_angle = fix16_sub(PI_DIV_2, angle);
		data->prediction = fix16_sqrt(
					fix16_pow(fix16_sub(fix16_mul(data->state, fix16_sin(adj_angle)), m_per_period), 2)
					+ fix16_pow(fix16_mul(data->state, fix16_cos(adj_angle)), 2)
					);
	}
}

static void kalman_pred_uncertainty_extrapolate(struct kalman *kf)
{
	struct kalman_data *data = kf->data;
	const struct kalman_config *cfg = kf->config;
	
	data->pred_uncertainty = fix16_add(data->state_uncertainty, cfg->noise);
}

fix16_t kalman_step(struct kalman* kf, fix16_t dist, fix16_t angle, fix16_t vel)
{
	if (kf == NULL) {
		return -EINVAL;
	}
	
	if (angle > THREE_PI_DIV_4 || angle < PI_DIV_4) {
		return -EINVAL;
	}
	
	struct kalman_data *data = kf->data;
	
	int64_t curr_timestamp = k_uptime_get();
	fix16_t secs_passed = (curr_timestamp - data->prev_timestamp)
				* fix16_one / 1000;
	
	kalman_calc_gain(kf);
	kalman_state_update(kf, dist);
	kalman_state_uncertainty_update(kf);
	kalman_state_extrapolate(kf, secs_passed, vel, angle);
	kalman_pred_uncertainty_extrapolate(kf);

	data->prev_timestamp = curr_timestamp;
	data->count++;
	
	// TODO: determine and report dynamic change, potentially reset filter
	return data->state;
}

int kalman_init(struct kalman* kf)
{
	if (kf == NULL) {
		return -EINVAL;
	}
	
	kf->data = &kalman_data_0;
	kf->config = &kalman_config_0;
	
	struct kalman_data *data = kf->data;
	const struct kalman_config *cfg = kf->config;

	data->state = cfg->initial_state;
	data->prediction =  cfg->initial_state;
	data->gain = cfg->initial_gain;
	data->pred_uncertainty = cfg->initial_pred_uncertainty;
	data->state_uncertainty = cfg->initial_state_uncertainty;
	data->meas_uncertainty = cfg->initial_meas_uncertainty;
	data->count = 0;
	data->prev_timestamp = k_uptime_get();
	
	return 0;
}

int kalman_reset(struct kalman* kf)
{
	if (kf == NULL) {
		return -EINVAL;
	}
	
	struct kalman_data *data = kf->data;
	const struct kalman_config *cfg = kf->config;

	data->gain = fix16_one;
	data->pred_uncertainty = cfg->initial_pred_uncertainty;
	data->state_uncertainty = cfg->initial_state_uncertainty;
	data->meas_uncertainty = cfg->initial_meas_uncertainty;
	
	return 0;
}
