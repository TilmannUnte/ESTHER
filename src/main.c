/*
 * Copyright (c) 2021-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/timing/timing.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/drivers/watchdog.h>

#ifdef CONFIG_UR_MAGNI_DRIVER
#include "ur_magni.h"
#endif /* CONFIG_UR_MAGNI_DRIVER */
#ifdef CONFIG_RPLIDAR
#include "rplidar.h"
#endif /* CONFIG_RPLIDAR */
#ifdef CONFIG_LIBFIXMATH
#include "fixmath.h"
#endif /* CONFIG_LIBFIXMATH */
#ifdef CONFIG_HELPMATH
#include "helpmath.h"
#endif /* CONFIG_HELPMATH */
#ifdef CONFIG_KALMAN_FILTER
#include "kalman.h"
#endif /* CONFIG_KALMAN_FILTER */
#ifdef CONFIG_MOVEMENT
#include "movement.h"
#endif /* CONFIG_MOVEMENT */

#define STACK_SIZE 1024

#ifdef CONFIG_UR_MAGNI_DRIVER
#define MAGNI DT_INST(0, ur_magni)
#define MAGNI_THREAD_PRIORITY UR_MAGNI_VELOCITY_THREAD_PRIORITY // = 26
#define MAGNI_THREAD_PERIOD_MSEC UR_MAGNI_VELOCITY_UPDATE_MSEC
#endif /* CONFIG_UR_MAGNI_DRIVER */

#ifdef CONFIG_RC_RECEIVER_DRIVER
#define GEAR DT_NODELABEL(gear)
#endif /* CONFIG_RC_RECEIVER_DRIVER */

#ifdef CONFIG_RPLIDAR
#define RPLIDAR DT_INST(0, slamtech_rplidar)
#define RPLIDAR_FULL_ROTATION_MSEC 250
#define DISTANCE_MEASUREMENT_THREAD_PRIORITY LOG2CEIL(RPLIDAR_FULL_ROTATION_MSEC * NSEC_PER_MSEC) // = 28
#endif /* CONFIG_RPLIDAR */

LOG_MODULE_REGISTER(application);

#ifdef CONFIG_UR_MAGNI_DRIVER
atomic_t magni_watchdog = ATOMIC_INIT(-1);
#endif /* CONFIG_UR_MAGNI_DRIVER */

#ifdef CONFIG_RPLIDAR
struct lidar_measurement {
	fix16_t angle, dist;
};

struct lidar_data {
	const struct device *rplidar;
	struct k_timer timer;
	struct k_msgq *msgq;
	struct k_mutex *measurements_mutex;
	struct lidar_measurement* live_measurements;
	struct lidar_measurement* snap_measurements;
	struct lidar_measurement measurements[2][360];
	int64_t timestamp;
	uint16_t measurement_count;
	atomic_t rplidar_watchdog;
};

K_MUTEX_DEFINE(measurements_mutex);

struct lidar_data lidar_data_0 = {
	.live_measurements = NULL,
	.snap_measurements = NULL,
	.timestamp = -1,
	.measurement_count = 0,
	.rplidar_watchdog = ATOMIC_INIT(-1),
	.measurements_mutex = &measurements_mutex,
};

#endif /* CONFIG_RPLIDAR */

#ifdef CONFIG_WATCHDOG
static void main_watchdog_callback(int channel_id, void* user_data)
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

#ifdef CONFIG_UR_MAGNI_DRIVER
//FIXME: This thread has a mess of compile-time dependencies
static void magni_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	const struct device* magni = DEVICE_DT_GET(MAGNI);
	int ret;
	ret = device_is_ready(magni);
	__ASSERT(ret , "Can't find magni");
	
	#ifdef CONFIG_RC_RECEIVER_DRIVER
	const struct device *gear = DEVICE_DT_GET(GEAR);

	ret = device_is_ready(gear);
	__ASSERT(ret, "can't find RC gear channel");

	struct sensor_value gear_val;
	#endif /* CONFIG_RC_RECEIVER_DRIVER */
	
	#ifdef CONFIG_MOVEMENT
	struct movement movement;
	movement_init(&movement, magni);
	
	struct movement_pose current, goal;
	get_current_pose(&movement, &current, K_FOREVER);
	get_current_pose(&movement, &goal, K_FOREVER);
	LOG_DBG("start pose: x: %i mm, y: %i mm, yaw: %i millirad", fix16_to_int(fix16_mul(current.x, fix16_from_int(1000))), fix16_to_int(fix16_mul(current.y, fix16_from_int(1000))), fix16_to_int(fix16_mul(current.orientation, fix16_from_int(1000))));
	
	struct sensor_value vel_left, vel_right;
	fix16_t avg_vel;
	#ifdef CONFIG_RPLIDAR
	fix16_t dist_in_front = 0;
	fix16_t min_dist = fix16_from_int(12);
	const fix16_t dist_threshold = fix16_div(fix16_from_int(80), fix16_from_int(100));
	const fix16_t front_lower = deg_to_rad(fix16_from_int(83));
	const fix16_t front_upper = deg_to_rad(fix16_from_int(97));
	int64_t prev_timestamp = -1;
	bool new_data = false;
	bool move_command_delay = false;
	bool obstacle_reached = false;
	
	#ifdef CONFIG_KALMAN_FILTER
	struct kalman kalman;
	fix16_t kf_dist;
	kalman_init(&kalman);
	#endif /* CONFIG_KALMAN_FILTER */
	#endif /* CONFIG_RPLIDAR */
	#endif /* CONFIG_MOVEMENT */
	
	extern const k_tid_t magni_tcb;
	
	struct k_timer timer;
	k_timer_init(&timer, NULL, NULL);
	uint64_t period_ticks = k_ms_to_ticks_near64(MAGNI_THREAD_PERIOD_MSEC);
	k_timer_start(&timer,
		K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
		K_TICKS(period_ticks));

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif

	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&timer);
	atomic_set(&magni_watchdog, task_wdt_add(MAGNI_THREAD_PERIOD_MSEC, main_watchdog_callback, magni_tcb));
	task_wdt_feed(atomic_get(&magni_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */

	for(;;) {
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&timer);
		#else
		k_timer_status_sync(&timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&timer)));

		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(magni_tcb,
			k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&timer);
		#endif /* CONFIG_WATCHDOG */
		
	        //k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&timer)));

		#ifdef CONFIG_RC_RECEIVER_DRIVER
		ret = sensor_sample_fetch(gear);
		if (!ret) {
			sensor_channel_get(gear, SENSOR_CHAN_ALL, &gear_val);
		} else {
			LOG_WRN("Can't fetch new data from remote");
		}
		
		#ifdef CONFIG_MOVEMENT
		// Switch setting will continuously force stop all move commands
		if (gear_val.val2 < 1500) {
			LOG_INF("RC stop command received!");
			force_stop(movement);
		}
		#endif /* CONFIG_MOVEMENT */
		#endif /* CONFIG_RC_RECEIVER_DRIVER */
		
		#ifdef CONFIG_RPLIDAR
		// FIXME: directly accessing the global data is not great design
		new_data = false;
		if (k_mutex_lock(lidar_data_0.measurements_mutex, sys_timepoint_timeout(deadline_tp)) == 0) {
			
			if (lidar_data_0.timestamp > prev_timestamp) {
				dist_in_front = 0;
				min_dist = fix16_from_int(12);
				fix16_t avg_div = 0;
				
				for (int16_t i = 0; i < lidar_data_0.measurement_count; i++) {
					if (lidar_data_0.snap_measurements[i].angle >= front_lower && lidar_data_0.snap_measurements[i].angle <= front_upper) {
						dist_in_front = fix16_add(dist_in_front, lidar_data_0.snap_measurements[i].dist);
						avg_div = fix16_add(avg_div, fix16_from_int(1));
					}
					
					if (lidar_data_0.snap_measurements[i].dist < min_dist) {
						min_dist = lidar_data_0.snap_measurements[i].dist;
					}
				}
				
				if (avg_div != 0) {
					dist_in_front = fix16_div(dist_in_front, avg_div);
					new_data = true;
				}
				
				prev_timestamp = lidar_data_0.timestamp;
			}
			
			k_mutex_unlock(lidar_data_0.measurements_mutex);
		} else {
			LOG_ERR("Cannot access measurement mutex");
		}
		#endif /* CONFIG_RPLIDAR */
		
		#ifdef CONFIG_MOVEMENT
		get_current_pose(&movement, &current,
			sys_timepoint_timeout(deadline_tp));

		#ifdef CONFIG_RPLIDAR
		if (!is_moving(&movement)) {
			if (!move_command_delay) {
				if (obstacle_reached == false && new_data == true && dist_in_front > dist_threshold) { /* Move Forward a bit*/
					LOG_DBG("Dist in front is %i mm", fix16_to_int(fix16_mul(dist_in_front, fix16_from_int(1000))));
					fix16_t safe_dist = fix16_div(fix16_from_int(1), fix16_from_int(2));//fix16_sub(min_dist, dist_threshold);
					// make sure we're actually making useful progress with the next move				
					if (safe_dist > fix16_div(fix16_from_int(5), fix16_from_int(100))) {
						// lidar polar coordinates to local cartesian coordinates, then rotate and transform to produce global poal position
						fix16_t local_x, local_y;
						local_x = fix16_mul(safe_dist, fix16_cos(PI_DIV_2));
						local_y = fix16_mul(safe_dist, fix16_sin(PI_DIV_2));
				
						fix16_t global_x_delta, global_y_delta;
						global_x_delta = fix16_sub(fix16_mul(local_y, fix16_cos(current.orientation)),
									fix16_mul(local_x, fix16_sin(current.orientation)));
						global_y_delta = fix16_add(fix16_mul(local_y, fix16_sin(current.orientation)),
									fix16_mul(local_x, fix16_cos(current.orientation)));

						goal.x = current.x + global_x_delta;
						goal.y = current.y + global_y_delta;
						// goal.orientation is unused and we end up with whatever orientation the movement API requires to reach the goal position
						#ifdef CONFIG_RC_RECEIVER_DRIVER
						if (gear.val2 >= 1500) {
							move_command(&movement, goal.x, goal.y,
								sys_timepoint_timeout(deadline_tp));
							move_command_delay = true;
							obstacle_reached = true;
						}
						#else
						move_command(&movement, goal.x, goal.y,
							sys_timepoint_timeout(deadline_tp));
						move_command_delay = true;
						obstacle_reached = true;
						#endif /* CONFIG_RC_RECEIVER_DRIVER */
						
						#ifdef CONFIG_KALMAN_FILTER
						kalman_reset(&kalman);
						#endif /* CONFIG_KALMAN_FILTER */
					}
				} else if (obstacle_reached == true && new_data == true) { /* Turn to look for a way around an obstacle */
					
					//FIXME: Remove code duplication
					// We need a minimum amount of linear movement as pure rotations are not supported by our movement command library
					fix16_t safe_dist = fix16_div(fix16_from_int(6), fix16_from_int(100));
					// lidar polar coordinates to local cartesian coordinates, then rotate and transform to produce global poal position
					fix16_t local_x, local_y;
					local_x = fix16_mul(safe_dist, fix16_cos(180));
					local_y = fix16_mul(safe_dist, fix16_sin(180));
				
					fix16_t global_x_delta, global_y_delta;
					global_x_delta = fix16_sub(fix16_mul(local_y, fix16_cos(current.orientation)),
								fix16_mul(local_x, fix16_sin(current.orientation)));
					global_y_delta = fix16_add(fix16_mul(local_y, fix16_sin(current.orientation)),
								fix16_mul(local_x, fix16_cos(current.orientation)));

					goal.x = current.x + global_x_delta;
					goal.y = current.y + global_y_delta;
					// goal.orientation is unused and we end up with whatever orientation the movement API requires to reach the goal position
					#ifdef CONFIG_RC_RECEIVER_DRIVER
					if (gear.val2 >= 1500) {
						move_command(&movement, goal.x, goal.y,
							sys_timepoint_timeout(deadline_tp));
						move_command_delay = true;
						obstacle_reached = false;
					}
					#else
					move_command(&movement, goal.x, goal.y,
						sys_timepoint_timeout(deadline_tp));
					move_command_delay = true;
					obstacle_reached = false;
					#endif /* CONFIG_RC_RECEIVER_DRIVER */
					
					#ifdef CONFIG_KALMAN_FILTER
					kalman_reset(&kalman);
					#endif /* CONFIG_KALMAN_FILTER */
				} else {
					LOG_DBG("Waiting for distance data before making a move");
				}
			} else {
				// Whenever we have just finished a move command we allow for a delay to acquire fresher LiDAR data
				move_command_delay = false;
			}
		} else {
			if (!is_turning(&movement)) {
				ur_magni_sample_fetch_with_timeout(magni, K_FOREVER);
				sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_VELOCITY_LEFT, &vel_left);
				sensor_channel_get(magni, SENSOR_CHAN_UR_MAGNI_VELOCITY_RIGHT, &vel_right);
			
				avg_vel = fix16_div(fix16_add(sensor_value_to_fix16_t(&vel_left), sensor_value_to_fix16_t(&vel_right)), fix16_from_int(2));
			
				if (new_data == true) {
					#ifdef CONFIG_KALMAN_FILTER
					kf_dist = kalman_step(&kalman, dist_in_front, PI_DIV_2, avg_vel);
					
					if (kf_dist < dist_threshold) {
						force_stop(&movement);
						obstacle_reached = true;
						LOG_DBG("Distance under threshold");
					}
					
					//FIXME: Detecting a dynamic change is not reliable in a single iteration. Experiments with sonars had good results when resetting after 3 iterations of significant deviation, but the LiDAR is slower and more noisy in comparision.
					/*if (kf_dist > fix16_mul(dist_in_front, fix16_div(fix16_from_int(6), fix16_from_int(4)))) {
						LOG_INF("Sudden appearance of an obstacle, resetting Kalman");
						kalman_reset(&kalman);
						if (dist_in_front < dist_threshold) {
							force_stop(&movement);
							LOG_INF("Raw distance under threshold");
						}
					}*/
					#else
					if (dist_in_front < dist_threshold) {
						force_stop(&movement);
						obstacle_reached = true;
						LOG_DBG("Measured distance under threshold");
					}
					#endif /* CONFIG_KALMAN_FILTER */
				} else {
					LOG_DBG("Moving without new data on surroundings");
				}
			} else {
				#ifdef CONFIG_KALMAN_FILTER
				kalman_reset(&kalman);
				#endif /* CONFIG_KALMAN_FILTER */
			}
		}
		#endif /* CONFIG_RPLIDAR */
		#endif /* CONFIG_MOVEMENT */

		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif

		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&magni_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("Top-level magni deadline");
		}
		#endif /* CONFIG_WATCHDOG */
	}
}
#endif /* CONFIG_UR_MAGNI_DRIVER */

#ifdef CONFIG_RPLIDAR

static void lidar_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct lidar_data* data = p1;
	
	data->rplidar = DEVICE_DT_GET(RPLIDAR);
	int ret = device_is_ready(data->rplidar);
	__ASSERT(ret, "Lidar not found!");
	
	rplidar_measurement_node node;
	rplidar_result ans;
	
	extern const k_tid_t lidar_tcb;
	
	const fix16_t lower_range = fix16_div(fix16_from_int(15), fix16_from_int(100));
	const fix16_t upper_range = fix16_from_int(12);

	k_timer_init(&data->timer, NULL, NULL);
	uint64_t period_ticks = k_ms_to_ticks_near64(RPLIDAR_FULL_ROTATION_MSEC);
	k_timer_start(&data->timer,
		K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
		K_TICKS(period_ticks));

	if (k_mutex_lock(data->measurements_mutex, K_FOREVER) == 0) {
	
		data->measurement_count = 0;
		
		data->live_measurements = data->measurements[0];
		data->snap_measurements = data->measurements[1];
		
		data->timestamp = k_uptime_get();
		
		k_mutex_unlock(data->measurements_mutex);
	}
	
	//FIXME: Cold starting the LiDAR is very unreliable and takes a long time
	do {
		rplidar_reset(data->rplidar);
		k_timer_status_sync(&data->timer);
		ans = rplidar_start_scan(data->rplidar, &data->msgq);
	} while (RPLIDAR_IS_FAIL(ans));

	#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&data->timer);
	atomic_set(&data->rplidar_watchdog, task_wdt_add(RPLIDAR_FULL_ROTATION_MSEC, main_watchdog_callback, lidar_tcb));
	task_wdt_feed(atomic_get(&data->rplidar_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */
	
	for (;;) {
		#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->timer);
		#else
		k_timer_status_sync(&data->timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(lidar_tcb, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&data->timer);
		#endif /* CONFIG_WATCHDOG */
		
	        //k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&timer)));
		
		/*FIXME: some of these are now redundant, but the LiDAR is very finicky and they may help to verify that you got a good set of measurements */
		uint8_t start_bit_count = 0;
		uint16_t quality_sample_count = 0;
		
		fix16_t angle_in_rad_normalized;
		uint32_t angle_integral = 0;
		
		/* gather measurements from approx. full rotation starting at an arbitrary point */
		for (int16_t i = 0; i < 360; i++) {
			if (sys_timepoint_expired(deadline_tp)) {
				LOG_DBG("Lidar sample collection deadline reached or exceeded");
				break;
			}

			ret = k_msgq_get(data->msgq, &node, sys_timepoint_timeout(deadline_tp));
			if (ret == 0) {
				if (node.startBit == true) {
					start_bit_count++;
				}
				
				// Lidar scans in CW direction and reports degrees with 0° front facing
				// we transform it to polor coordinates (CCW) with 0 radians facing right (along positive X-axis)
				// adjust offset
				angle_in_rad_normalized = sensor_degrees_to_fix16_t_rad(&node.angle);
				angle_in_rad_normalized = fix16_sub(angle_in_rad_normalized, PI_DIV_2);
				if (angle_in_rad_normalized < 0) angle_in_rad_normalized = fix16_add(angle_in_rad_normalized, PI_MUL_2);
				// mirror rotation
				angle_in_rad_normalized = fix16_mul(angle_in_rad_normalized, fix16_from_int(-1));
				if (angle_in_rad_normalized != 0) angle_in_rad_normalized = fix16_add(angle_in_rad_normalized, PI_MUL_2);
			
				// We reduce the processing time downstream by only keeping data within the valid distance range
				// OPTIONAL: The .quality field provides 6-Bits of information on the strength of reflectance of each sample. We had confusing/inconsistent results when filtering data by .quality
				fix16_t dist = sensor_value_to_fix16_t(&node.distance);
				if (dist > lower_range && dist < upper_range) {
					data->live_measurements[quality_sample_count].dist = dist;
					data->live_measurements[quality_sample_count].angle = angle_in_rad_normalized;
					quality_sample_count++;
					//printk("%i;%i\n", fix16_to_int(rad_to_deg(angle_in_rad_normalized)), fix16_to_int(fix16_mul(dist, fix16_from_int(1000))));
				}
				
				uint16_t degrees = fix16_to_int(rad_to_deg(angle_in_rad_normalized)) % 360;
				angle_integral += degrees;
			} else if (ret == -EAGAIN){
				LOG_DBG("Lidar sample collection timeout");
				break;
			}
		}
		
		/* perform buffer exchange to hand over collected data to consuming threads */
		if (k_mutex_lock(data->measurements_mutex, sys_timepoint_timeout(deadline_tp)) == 0) {
			struct lidar_measurement* tmp;
			tmp = data->snap_measurements;
			data->snap_measurements = data->live_measurements;
			data->live_measurements = tmp;
			
			data->measurement_count = quality_sample_count;
			data->timestamp = k_uptime_get();
			k_mutex_unlock(data->measurements_mutex);
		} else {
			LOG_ERR("Cannot acquire measurement mutex to perform buffer swap!");
		}

		#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif
	
		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&data->rplidar_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("Top-level lidar deadline");
		}
		#endif /* CONFIG_WATCHDOG */
	}
}
#endif /* CONFIG_RPLIDAR */

#ifdef CONFIG_UR_MAGNI_DRIVER
K_THREAD_DEFINE(magni_tcb, STACK_SIZE, magni_thread, NULL, NULL, NULL, MAGNI_THREAD_PRIORITY, 0, 2000);
#endif /* CONFIG_UR_MAGNI_DRIVER */
//TODO Front Facing Sonar

#ifdef CONFIG_RPLIDAR
K_THREAD_DEFINE(lidar_tcb, STACK_SIZE, lidar_thread, &lidar_data_0, NULL, NULL, DISTANCE_MEASUREMENT_THREAD_PRIORITY, 0, 2000);
#endif /* CONFIG_RPLIDAR */
