/*
 * Copyright (c) 2025-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/
#include <zephyr/kernel.h>
#include <zephyr/timing/timing.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/drivers/watchdog.h>

#include "movement.h"
#include "ur_magni.h"
#include "fixmath.h"
#include "helpmath.h"

LOG_MODULE_REGISTER(movement);

struct movement_data movement_data_0 = {
	.current_pose = {.x = 0, .y = 0, .orientation = PI_DIV_2},
	.previous_pose = {.x = 0, .y = 0, .orientation = PI_DIV_2},
	.goal_pose = {.x = 0, .y = 0, .orientation = PI_DIV_2},
	.current_left = {.val1 = 0, .val2 = 0},
	.previous_left = {.val1 = 0, .val2 = 0},
	.current_right = {.val1 = 0, .val2 = 0},
	.previous_right = {.val1 = 0, .val2 = 0},
	//FIXME
	.movement_state = ATOMIC_INIT(MOVEMENT_STATE_IDLE),
	.force_stop = ATOMIC_INIT(false),
	.movement_watchdog = ATOMIC_INIT(-1),
};

const struct movement_config movement_config_0 = {
	.angular_tolerance = 3277, // 0.05
	.linear_tolerance = 3277, // 0.05
	.acceleration = 6554, // 0.1
	.min_linear_velocity = 6554, // 0.1
	.max_linear_velocity = 16385, // 0.25
	.min_angular_velocity = 6554, // 0.1
	.max_angular_velocity = 6554, // 0.1 // The turning accuracy is very low (~35° steps), so we limit the velocity very strictly
	.wheel_separation = 21627, // 0.33
	.origin_pose = {.x = 0, .y = 0, .orientation = PI_DIV_2},
	.speed_control_p_factor = 32768, // 0.5
	.speed_control_i_factor = 0,
	.speed_control_d_factor = 0,
};

#ifdef CONFIG_WATCHDOG
static void movement_watchdog_callback(int channel_id, void* user_data)
{
	k_tid_t thread = (k_tid_t) user_data;
	#ifdef CONFIG_THREAD_NAME
	const char* name = k_thread_name_get(thread);
	if (name != NULL) {
		LOG_DBG("%s deadline", name);
	} else {
		LOG_DBG("%p on chan%i deadline", thread, channel_id);
	}
	#else
	LOG_DBG("%p on chan %i deadline", thread, channel_id);
	#endif /* CONFIG_THREAD_NAME */
	//TODO: Potential to implement Kill Strategy
	//Caveat: all resources (e.g. mutexes) held by thread MUST be released first!
	//Thread then needs to be reinitialized
	//k_thread_abort(*thread);
}
#endif /* CONFIG_WATCHDOG */

static fix16_t calc_robot_orientation_rad(fix16_t delta_left, fix16_t delta_right, fix16_t wheel_sep) {
	return fix16_div(fix16_sub(delta_left, delta_right), wheel_sep);
}

static bool points_are_close(struct movement* self, struct movement_pose* a,
				struct movement_pose* b) {
	const struct movement_config* cfg = self->config;
	
	bool res = fix16_euclid_dist(a->x, a->y, b->x, b->y) < cfg->linear_tolerance;
	
	return res;
}


static void movement_thread_tracking(struct movement* self)
{
	struct movement_data* data = self->data;
	const struct movement_config* cfg = self->config;
	
	fix16_t position_step_delta = fix16_div(
						fix16_add(
							fix16_sub(sensor_value_to_fix16_t(&data->current_left),
							sensor_value_to_fix16_t(&data->previous_left)),
							fix16_sub(sensor_value_to_fix16_t(&data->current_right),
							sensor_value_to_fix16_t(&data->previous_right))),
						fix16_from_int(2));
	
	data->previous_pose.orientation = data->current_pose.orientation;
	//FIXME: normalize rad necessary?
	data->current_pose.orientation = fix16_sub(cfg->origin_pose.orientation,
					calc_robot_orientation_rad(
						sensor_value_to_fix16_t(&data->current_left),
						sensor_value_to_fix16_t(&data->current_right),
						cfg->wheel_separation));

	//FIXME: rad_abs_diff necessary? 
	fix16_t orientation_delta = fix16_sub(data->current_pose.orientation,
					data->previous_pose.orientation);
	
	fix16_t y_local_delta, x_local_delta;
	if (orientation_delta == 0) {
		// No measurable change in orientation
		LOG_DBG("Orientation Delta is 0");
		y_local_delta = position_step_delta;
		x_local_delta = 0;
	} else {
		// Robot travelled along an arc
		//FIXME: Some sources use sin(orientation_delta) instead of sin(orientation_delta/2), but that seems incorrect?
		y_local_delta = fix16_mul(
					fix16_mul(fix16_from_int(2), fix16_sin(fix16_div(orientation_delta, fix16_from_int(2)))),
					fix16_sub(fix16_div(position_step_delta, orientation_delta), fix16_div(cfg->wheel_separation, fix16_from_int(2)))
					);
		//FIXME: This is 0, because we can't track the x deviation (no corresponding encoder wheel)
		x_local_delta = fix16_mul(
					fix16_mul(fix16_from_int(2), fix16_sin(fix16_div(orientation_delta, fix16_from_int(2)))),
					fix16_add(fix16_div(0, orientation_delta), 0)
				);
	}
	
	//FIXME: normalize_rad necessary? my sources just use sub
	fix16_t avg_orientation = fix16_sub(data->current_pose.orientation, fix16_div(orientation_delta, fix16_from_int(2)));
	fix16_t x_global_delta, y_global_delta;
	
	x_global_delta = fix16_sub(fix16_mul(y_local_delta, fix16_cos(avg_orientation)),
			fix16_mul(x_local_delta, fix16_sin(avg_orientation)));
	y_global_delta = fix16_add(fix16_mul(y_local_delta, fix16_sin(avg_orientation)),
			fix16_mul(x_local_delta, fix16_cos(avg_orientation)));
	
	data->previous_pose.x = data->current_pose.x;
	data->previous_pose.y = data->current_pose.y;
	
	data->current_pose.x = fix16_add(data->current_pose.x, x_global_delta);
	data->current_pose.y = fix16_add(data->current_pose.y, y_global_delta);
	data->current_pose.orientation = normalize_rad(data->current_pose.orientation);
}

static void stop_movement(struct movement* self, k_timeout_t timeout) {
	struct sensor_value zero_vel = {.val1 = 0, .val2 = 0};

	ur_magni_set_velocity(self->data->magni, &zero_vel, &zero_vel, timeout);
}

static fix16_t movement_speed_controller(struct movement* self)
{
	struct movement_data *data = self->data;
	const struct movement_config *cfg = self->config;
	
	fix16_t error = fix16_sub(data->desired_vel,
				fix16_div(
					fix16_add(
						fix16_abs(sensor_value_to_fix16_t(&data->actual_vel_left)),
						fix16_abs(sensor_value_to_fix16_t(&data->actual_vel_right))
					),
					fix16_from_int(2)
				)
			);
	
	/* Expected time difference is one period duration, but due to jitter and mutexes there may be inconsistencies */
	int64_t curr_timestamp = k_uptime_get();
	fix16_t secs_passed = fix16_div(
				fix16_from_int((int) (curr_timestamp - data->prev_timestamp)),
				fix16_from_int(1000)
				);
	
	data->prev_timestamp = curr_timestamp;
	
	data->vel_p = error;
	data->vel_i = fix16_add(data->vel_i, fix16_mul(error, secs_passed));
	if (secs_passed != 0) {
		data->vel_d = fix16_div(fix16_sub(error, data->vel_error_prev), secs_passed);
	} else {
		data->vel_d = 0;
	}
	
	data->vel_error_prev = error;
	
	fix16_t pid_out = 0;
	pid_out = fix16_add(pid_out, fix16_mul(cfg->speed_control_p_factor, data->vel_p));
	pid_out = fix16_add(pid_out, fix16_mul(cfg->speed_control_i_factor, data->vel_i));
	pid_out = fix16_add(pid_out, fix16_mul(cfg->speed_control_d_factor, data->vel_d));
			
	LOG_DBG("Controller: desired: %i mm/s, error: %i mm/s, pid_out: %i mm/s", fix16_to_int(fix16_mul(data->desired_vel, fix16_from_int(1000))), fix16_to_int(fix16_mul(error, fix16_from_int(1000))), fix16_to_int(fix16_mul(pid_out, fix16_from_int(1000))));
	
	return pid_out;
}

static void movement_speed_controller_reset(struct movement* self)
{
	struct movement_data *data = self->data;

	data->desired_vel = 0;
	data->vel_p = 0;
	data->vel_i = 0;
	data->vel_d = 0;
	data->vel_error_prev = 0;
}

static fix16_t movement_speed_clamp(fix16_t in_vel, fix16_t min_vel, fix16_t max_vel)
{
	fix16_t out_vel = 0;
	
	if (in_vel > 0) {
		out_vel = min(in_vel, max_vel);
		
		if (out_vel < min_vel) out_vel = min_vel;
	} else if (in_vel < 0) {
		out_vel = max(in_vel, -max_vel);
		
		if (out_vel > -min_vel) out_vel = -min_vel;
	}
	
	return out_vel;
}

static void movement_thread_command(struct movement* self, fix16_t *vel_l, fix16_t *vel_r)
{
	//TODO: Backward movement not supported (in practice this was inaccurate!)
	//TODO: Final rotation correction not implemented (Ubiquity Robotics does this)
	
	struct movement_data *data = self->data;
	const struct movement_config *cfg = self->config;
	
	fix16_t set_vel;
	fix16_t goal_dist;
	fix16_t orientation_diff;
	
	enum movement_state state = atomic_get((const atomic_t*)&data->movement_state);
	
	orientation_diff = rad_abs_diff(data->goal_pose.orientation, data->current_pose.orientation);
	
	goal_dist = fix16_euclid_dist(data->current_pose.x, data->current_pose.y,
					data->goal_pose.x, data->goal_pose.y);
	
	switch (state) {
	case MOVEMENT_STATE_IDLE:
		*vel_l = 0;
		*vel_r = 0;
		break;
	case MOVEMENT_STATE_START:
		int direction = turning_direction(data->current_pose.orientation, data->goal_pose.orientation);
		LOG_DBG("New move start orientation: %i, goal: %i", fix16_to_int(rad_to_deg(data->current_pose.orientation)), fix16_to_int(rad_to_deg(data->goal_pose.orientation)));
		if (orientation_diff < cfg->angular_tolerance || direction == 0) {
			// Orientation difference is below threshold or a perfect circle
			LOG_DBG("New Move command does not require turning");
			state = MOVEMENT_STATE_FORWARD;
		} else if (direction == -1) {
			LOG_DBG("New move command starts with CW turn");
			state = MOVEMENT_STATE_TURNING_CW;
		} else if (direction == 1) {
			LOG_DBG("New move command starts with CCW turn");
			state = MOVEMENT_STATE_TURNING_CCW;
		} else {
			LOG_ERR("New move requires invalid turn");
		}
		
		*vel_l = 0;
		*vel_r = 0;

		break;
	case MOVEMENT_STATE_TURNING_CW:
		//LOG_DBG("orient diff: %i, prev: %i", fix16_to_int(rad_to_deg(orientation_diff)), fix16_to_int(rad_to_deg(data->prev_orientation_diff)));
		if (!data->orientation_overshot) {
			data->orientation_overshot = orientation_diff > data->prev_orientation_diff;
			if (data->orientation_overshot) LOG_DBG("Orientation was overshot");	
		}
		
		if (orientation_diff >= cfg->angular_tolerance &&
			!data->orientation_overshot) {
			data->desired_vel = cfg->max_angular_velocity;
		} else {
			data->desired_vel = 0;
		}
					
		set_vel = movement_speed_clamp(movement_speed_controller(self), cfg->min_angular_velocity, cfg->max_angular_velocity);
		if (set_vel == 0) {
			LOG_DBG("Finished Turning");
			state = MOVEMENT_STATE_FORWARD;
		}
			
		*vel_l = set_vel;
		*vel_r = -set_vel;
		
		break;
	case MOVEMENT_STATE_TURNING_CCW:
		//LOG_DBG("orient diff: %i, prev: %i", fix16_to_int(rad_to_deg(orientation_diff)), fix16_to_int(rad_to_deg(data->prev_orientation_diff)));
		if (!data->orientation_overshot) {
			data->orientation_overshot = orientation_diff > data->prev_orientation_diff;
			if (data->orientation_overshot) LOG_DBG("Orientation was overshot");	
		}
		
		if (orientation_diff >= cfg->angular_tolerance &&
			!data->orientation_overshot) {
			data->desired_vel = cfg->max_angular_velocity;
		} else {
			data->desired_vel = 0;
		}
					
		set_vel = movement_speed_clamp(movement_speed_controller(self), cfg->min_angular_velocity, cfg->max_angular_velocity);
		if (set_vel == 0) {
			LOG_DBG("Finished Turning");
			state = MOVEMENT_STATE_FORWARD;
		}
		
		*vel_l = -set_vel;
		*vel_r = set_vel;

		break;
	case MOVEMENT_STATE_FORWARD:
		if (!data->dist_overshot) data->dist_overshot = goal_dist > data->prev_goal_dist;
	
		if (goal_dist >= cfg->linear_tolerance &&
			!data->dist_overshot) {
			data->desired_vel = cfg->max_linear_velocity;
		} else {
			data->desired_vel = 0;
		}
		
		set_vel = movement_speed_clamp(movement_speed_controller(self), cfg->min_linear_velocity, cfg->max_linear_velocity);
		LOG_DBG("Dist: %i, Prev Dist: %i", fix16_to_int(fix16_mul(goal_dist, fix16_from_int(1000))), fix16_to_int(fix16_mul(data->prev_goal_dist, fix16_from_int(1000))));
		if (set_vel == 0) {
			LOG_DBG("Finished Moving Forward");
			state = MOVEMENT_STATE_IDLE;
		}
			
		*vel_r = set_vel;
		*vel_l = set_vel;

		break;
	}
	
	data->prev_orientation_diff = orientation_diff;
	data->prev_goal_dist = goal_dist;
	atomic_set(&data->movement_state, state);
}

//TODO: Watchdog
static void movement_thread(void* p1, void* p2, void* p3) {
	struct movement *self = p1;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	
	struct movement_data* data = self->data;
	const struct device* magni = data->magni;
	fix16_t vel_l = 0, vel_r = 0;
	int ret;
	
	ur_magni_sample_fetch_with_timeout(magni, K_FOREVER);
	sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_POSITION_LEFT, &data->current_left);
	sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_POSITION_RIGHT, &data->current_right);
	sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_VELOCITY_LEFT, &data->actual_vel_left);
	sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_VELOCITY_RIGHT, &data->actual_vel_right);
	
	data->prev_timestamp = k_uptime_get();

	#ifdef CONFIG_MOVEMENT_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&data->movement_timer);
	atomic_set(&data->movement_watchdog, task_wdt_add(MOVEMENT_THREAD_PERIOD_MSEC, movement_watchdog_callback, data->movement_tid));
	task_wdt_feed(atomic_get(&data->movement_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */
	
	for (;;) {
		#ifdef CONFIG_MOVEMENT_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->movement_timer);
		#else
		k_timer_status_sync(&data->movement_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->movement_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->movement_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->movement_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&data->movement_timer);
		#endif /* CONFIG_WATCHDOG */
		
		data->previous_left = data->current_left;
		data->previous_right = data->current_right;
		
		ret = ur_magni_sample_fetch_with_timeout(magni, sys_timepoint_timeout(deadline_tp));
		
		if (ret == 0) {
			sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_POSITION_LEFT, &data->current_left);
			sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_POSITION_RIGHT, &data->current_right);
			sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_VELOCITY_LEFT, &data->actual_vel_left);
			sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_VELOCITY_RIGHT, &data->actual_vel_right);
		} else {
			LOG_WRN("Can't read Magni position and velocity, skipping tracking update.");
		}
		
		LOG_DBG("Magni Position L: %i mm R: %i mm", (int) (sensor_value_to_micro(&data->current_left) / 1000), (int) (sensor_value_to_micro(&data->current_right) / 1000));
		
		if (k_mutex_lock(&data->pose_mutex, sys_timepoint_timeout(deadline_tp)) == 0) {
			// We do all pose manipulations in a single critical section, to not cause multiple waiting states
			if (ret == 0) movement_thread_tracking(self);
			LOG_DBG("Current Position: x: %i mm, y: %i mm, yaw: %i degrees", fix16_to_int(fix16_mul(data->current_pose.x, fix16_from_int(1000))), fix16_to_int(fix16_mul(data->current_pose.y, fix16_from_int(1000))), fix16_to_int(rad_to_deg(data->current_pose.orientation)));
			if (atomic_get(&data->force_stop)) {
				// Force Stopping invalidates the previous goal, the application is forced to calculate a new one
				data->goal_pose.x = data->current_pose.x;
				data->goal_pose.y = data->current_pose.y;
				data->goal_pose.orientation = data->current_pose.orientation;
				
				k_mutex_unlock(&data->pose_mutex);
				
				//FIXME: Sudden stopping leads to tumbling if 1m/s speed limit is ever extended!
				stop_movement(self, sys_timepoint_timeout(deadline_tp));
				movement_speed_controller_reset(self);
				atomic_set(&data->movement_state, MOVEMENT_STATE_IDLE);
				atomic_set(&data->force_stop, false);
				LOG_DBG("Movement has been forcibly stopped.");
			} else {				
				if (ret == 0) movement_thread_command(self, &vel_l, &vel_r);

				k_mutex_unlock(&data->pose_mutex);
				
				struct sensor_value sv_vel_l, sv_vel_r;
				sensor_value_from_fix16_t(&sv_vel_l, vel_l);
				sensor_value_from_fix16_t(&sv_vel_r, vel_r);
				LOG_DBG("Setting movement speed l: %i mm/s, r: %i mm/s", (int) (sensor_value_to_micro(&sv_vel_l) / 1000), (int) (sensor_value_to_micro(&sv_vel_r) / 1000));
				if (ret == 0) ur_magni_set_velocity(self->data->magni, &sv_vel_l, &sv_vel_r, sys_timepoint_timeout(deadline_tp));
			}
		} else {
			LOG_WRN("Movement thread timed out waiting on pose mutex.");
			//TODO: Reset speed after a timeout period?
		}

		#ifdef CONFIG_MOVEMENT_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif
		
		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&data->movement_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("Movement thread deadline");
		}
		#endif /* CONFIG_WATCHDOG */
	}
}

int movement_init(struct movement* self, const struct device* magni)
{
	if (self == NULL) {
		LOG_ERR("Initialization failed: no output pointer provided!");
		return -EINVAL;
	}
	if (magni == NULL) {
		LOG_ERR("Initialization failed: no magni device provided!");
		return -EINVAL;
	}
	
	self->data = &movement_data_0;
	self->config = &movement_config_0;
	
	struct movement_data* data = self->data;
	
	data->magni = magni;
	
	k_mutex_init(&data->pose_mutex);
	
	k_timer_init(&data->movement_timer, NULL, NULL);
	uint64_t period_ticks = k_ms_to_ticks_near64(MOVEMENT_THREAD_PERIOD_MSEC);
	k_timer_start(&data->movement_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));
	
	data->movement_tid = k_thread_create(&data->movement_thread,
				data->movement_stack,
				K_KERNEL_STACK_SIZEOF(data->movement_stack),
				movement_thread, (void*) self,
				NULL, NULL, /* unused parameters */
				MOVEMENT_THREAD_PRIORITY, K_ESSENTIAL,
				K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->movement_tid, "movement");
	#endif /* CONFIG_THREAD_NAME */
	
	return 0;
}

bool is_moving(struct movement *self)
{
	struct movement_data *data = self->data;
	
	enum movement_state state = atomic_get(&data->movement_state);
	return state != MOVEMENT_STATE_IDLE;
}

bool is_turning(struct movement *self)
{
	struct movement_data *data = self->data;
	
	enum movement_state state = atomic_get(&data->movement_state);
	return (state == MOVEMENT_STATE_START || state == MOVEMENT_STATE_TURNING_CW || state == MOVEMENT_STATE_TURNING_CCW);
}

int get_current_pose(struct movement* self, struct movement_pose* pose, k_timeout_t timeout)
{
	struct movement_data *data = self->data;
	
	if (k_mutex_lock(&data->pose_mutex, timeout) == 0) {
		pose->x = data->current_pose.x;
		pose->y = data->current_pose.y;
		pose->orientation = data->current_pose.orientation;
		
		k_mutex_unlock(&data->pose_mutex);
	} else {
		LOG_WRN("Couldn't read current pose due to mutex timeout.");
		return -ETIMEDOUT;
	}
	
	return 0;
}

int move_command(struct movement* self, fix16_t x, fix16_t y, k_timeout_t timeout)
{
	struct movement_data* data = self->data;

	if (is_moving(self)) {
		LOG_WRN("Issued new move command during ongoing movement!");
		return -EBUSY;
	}
	
	if (k_mutex_lock(&data->pose_mutex, timeout) == 0) {
		struct movement_pose new;
		new.x = x;
		new.y = y;
		new.orientation = normalize_rad(calculate_angle_rad(data->current_pose.x,
						data->current_pose.y, x, y));
		
		if (!points_are_close(self, &data->current_pose, &new)) {
			data->goal_pose.x = new.x;
			data->goal_pose.y = new.y;
			data->goal_pose.orientation = new.orientation;
			
			data->prev_goal_dist = fix16_euclid_dist(
					data->current_pose.x, data->current_pose.y,
					data->goal_pose.x, data->goal_pose.y
						);
						
			data->prev_orientation_diff = rad_abs_diff(
							data->current_pose.orientation,
							data->goal_pose.orientation
							);
			data->orientation_overshot = false;
			data->dist_overshot = false;
				
			atomic_set(&data->movement_state, MOVEMENT_STATE_START);
			LOG_DBG("Set to start new move command");
		} else {
			LOG_DBG("Rejected move command to very close position");
		}

		k_mutex_unlock(&data->pose_mutex);
	} else {
		LOG_WRN("Failed on move command due to mutex timeout.");
		return -ETIMEDOUT;
	}
	
	return 0;
}

void force_stop(struct movement* self)
{
	struct movement_data *data = self->data;
	
	atomic_set(&data->force_stop, true);
}
