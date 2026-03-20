/*
 * Copyright (c) 2025-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
 */

/**
 * @file
 * The Movement API takes over the speed control of the Magni robot.
 * This is done via a thread running at the same rate as the Magni driver's
 * velocity update thread.
 * 
 * The movement thread is capable of executing movement commands, either linear
 * or rotational. Only one command is executed at a time and there is no command
 * buffering. Therefore, applications programmers *must* make sure to only send
 * a new move command once the previous command has finished or been forcibly
 * halted.
 *
 * The movement thread will take care of accelerating and decelerating the robot
 * according to the remaining distance to the goal.
 *
 * Optionally, the movement thread can be informed about the distance to a nearby
 * obstacle (i.e. an obstacle in the path of the robot). It will then favor
 * obstacle avoidance over reaching the intended goal. In other words, it may
 * stop short of the desired goal, if it cannot be reached without risking
 * a collision.
 *
 * Movement along an arc is not inherently supported. However, deviations from
 * a perfectly straight linear movement are supported in the pose tracking.
 */

#ifndef __MOVEMENT_H__
#define __MOVEMENT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include "fixmath.h"

#define MOVEMENT_THREAD_STACK_SIZE 1024
#define MOVEMENT_THREAD_PERIOD_MSEC UR_MAGNI_VELOCITY_UPDATE_MSEC
#define MOVEMENT_THREAD_PRIORITY LOG2CEIL(MOVEMENT_THREAD_PERIOD_MSEC * NSEC_PER_MSEC) // = 26

enum movement_state {
	MOVEMENT_STATE_IDLE = 0,
	MOVEMENT_STATE_START,
	MOVEMENT_STATE_TURNING_CW,
	MOVEMENT_STATE_TURNING_CCW,
	MOVEMENT_STATE_FORWARD,
	// Backward not supported, tests showed poorer accuracy
};

struct movement_pose {
	fix16_t x;
	fix16_t y;
	fix16_t orientation;
};

struct movement_config {
	fix16_t angular_tolerance;
	fix16_t linear_tolerance;
	fix16_t acceleration;
	fix16_t min_linear_velocity;
	fix16_t max_linear_velocity;
	fix16_t min_angular_velocity;
	fix16_t max_angular_velocity;
	fix16_t speed_control_p_factor;
	fix16_t speed_control_i_factor;
	fix16_t speed_control_d_factor;
	fix16_t wheel_separation;
	struct movement_pose origin_pose;
	
/*	float minTurningVelocity,
	float maxTurningVelocity,
	float turningAcceleration,
	float maxLateralVelocity,
	
	float obstacleWaitThreshold,
	float forwardObstacleThreshold,
	float minSideDist,
	float localizationLatency,
	float runawayTimeoutSecs,
*/
};

struct movement_data {
	K_KERNEL_STACK_MEMBER(movement_stack, MOVEMENT_THREAD_STACK_SIZE);
	struct k_thread movement_thread;
	k_tid_t movement_tid;
	struct k_timer movement_timer;
	atomic_t movement_watchdog;
	
	const struct device* magni;
	
	struct movement_pose current_pose, previous_pose, goal_pose;
	struct k_mutex pose_mutex;
	struct sensor_value current_left, previous_left;
	struct sensor_value current_right, previous_right;
	struct sensor_value actual_vel_left, actual_vel_right;
	fix16_t prev_goal_dist;
	fix16_t prev_orientation_diff;
	bool orientation_overshot, dist_overshot;
	fix16_t desired_vel, vel_error_prev, vel_p, vel_i, vel_d;
	int64_t prev_timestamp;
	atomic_t force_stop, movement_state;
};

struct movement {
	const struct movement_config* config;
	struct movement_data* data;
};

extern struct movement_data movement_data_0;

extern const struct movement_config movement_config_0;

int movement_init(struct movement* self, const struct device* magni);

int move_command(struct movement* self, fix16_t x, fix16_t y, k_timeout_t timeout);

int get_current_pose(struct movement* self, struct movement_pose* pose, k_timeout_t);

bool is_moving(struct movement *self);

bool is_turning(struct movement *self);

void force_stop(struct movement* self);

#ifdef __cplusplus
}
#endif
#endif
