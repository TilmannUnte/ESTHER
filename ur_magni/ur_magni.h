/*
 * Copyright (c) 2021-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
 */

/**
 * @file
 * This driver is a multi-threaded recreation of the official Magni ROS software
 * for the Zephyr RTOS.
 * It does not have ROS support! A seperate connection to uROS is a possible
 * future extension.
 *
 * It requires interrupt support and UART. Optionally it requires I2C support.
 *
 * Part of this driver uses Zephyr's generic sensor subsystem. This makes it easy
 * to use for application programmers since there are very few unique functions.
 * However, the Magni does not behave as a typical sensor. It constantly reports
 * it's status to the driver. When a sample is fetched by the application the most
 * recent status is captured and held seperately.
 *
 * The motor velocity is controlled by a dedicated function.
 *
 * The driver theoretically supports multiple instances, however it is unlikely
 * that this is ever practically useful, since the host system would need to be
 * physically connected to more than one Magni robot.
 */
#ifndef __UR_MAGNI_DRIVER_H__
#define __UR_MAGNI_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <zephyr/kernel.h> // stdint is not available for indexing without this
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/display.h>

#include "ur_magni_msg.h"

// native_sim (POSIX) breaks timing and may benefit from larger buffer sizes to compensate
#ifdef CONFIG_ARCH_POSIX
#define UR_MAGNI_SLAB_BLOCKS (2048 * 16)
#else
#define UR_MAGNI_SLAB_BLOCKS (4 * 16)
#endif /* CONFIG_ARCH_POSIX */
// These are set with -O0 and immediate logging in mind, which is very expensive
#define UR_MAGNI_UART_THREAD_STACK_SIZE 1024 //2048
#define UR_MAGNI_PARSER_THREAD_STACK_SIZE 1024 //2048
#define UR_MAGNI_VELOCITY_THREAD_STACK_SIZE 1024 //2048
#define UR_MAGNI_SYSTEM_THREAD_STACK_SIZE 1024 //2048
#define UR_MAGNI_JOINT_THREAD_STACK_SIZE 1024 //2048

#define UR_MAGNI_BAUDRATE 38400 // 9600 in some original sources?
// ceil(1/UR_MAGNI_BAUDRATE - 2/10 * UR_MAGNI_BAUDRATE) * USEC_PER_SEC))

#define UR_MAGNI_NSEC_PER_BYTE 260417
#define UR_MAGNI_NSEC_PER_MSG (UR_MAGNI_NSEC_PER_BYTE * 8)
//#define UR_MAGNI_USEC_PER_BYTE 261
//#define UR_MAGNI_USEC_PER_MSG (UR_MAGNI_USEC_PER_BYTE * 8)
#define UR_MAGNI_MSEC_PER_BYTE_CEIL 1
#define UR_MAGNI_MSEC_PER_MSG_CEIL 3
#define UR_MAGNI_MSEC_PER_MSG_FLOOR 2

// If harmonic periods are desired, the MSEC period can be used. The parser thread will activate slightly more often than strictly required.
#define UR_MAGNI_PARSER_NSEC (UR_MAGNI_NSEC_PER_MSG)
//#define UR_MAGNI_PARSER_USEC (UR_MAGNI_USEC_PER_MSG)
#define UR_MAGNI_PARSER_MSEC (UR_MAGNI_MSEC_PER_MSG_FLOOR)

#define UR_MAGNI_VELOCITY_UPDATE_MSEC \
(MSEC_PER_SEC / UR_MAGNI_VELOCITY_UPDATE_PER_SEC)
#define UR_MAGNI_SYSTEM_MAINTENANCE_MSEC (MSEC_PER_SEC * 60)
#define UR_MAGNI_JOINT_UPDATE_MSEC 250

#define UR_MAGNI_UART_RX_THREAD_PRIORITY LOG2CEIL(UR_MAGNI_NSEC_PER_BYTE / 2) // = 17
#define UR_MAGNI_UART_TX_THREAD_PRIORITY LOG2CEIL(UR_MAGNI_NSEC_PER_BYTE) // = 18
#define UR_MAGNI_PARSER_THREAD_PRIORITY LOG2CEIL(UR_MAGNI_PARSER_MSEC * NSEC_PER_MSEC) // = 21
#define UR_MAGNI_SYSTEM_THREAD_PRIORITY LOG2CEIL(UR_MAGNI_SYSTEM_MAINTENANCE_MSEC * NSEC_PER_MSEC) // = 36
#define UR_MAGNI_JOINT_THREAD_PRIORITY LOG2CEIL(UR_MAGNI_JOINT_UPDATE_MSEC * NSEC_PER_MSEC) // = 28
#define UR_MAGNI_VELOCITY_THREAD_PRIORITY LOG2CEIL(UR_MAGNI_VELOCITY_UPDATE_MSEC * NSEC_PER_MSEC) // = 26

#define UR_MAGNI_MIN_FW_VER 40

/**
 * @brief Extension for the standard sensor channels
 *
 * The sensor values reported by this driver do not align well with the standard
 * options of the generic sensor driver. Therefore it was decided to use an
 * extension to the standard list.
 */
enum ur_magni_sensor_channel {
	SENSOR_CHAN_UR_MAGNI_BATTERY_VOLTAGE = SENSOR_CHAN_PRIV_START,
	/** Reports the deviation from the desired position for the left motor, unit unclear */
	SENSOR_CHAN_UR_MAGNI_PID_ERROR_LEFT,
	/** Reports the deviation from the desired position for the right motor, unit unclear */
	SENSOR_CHAN_UR_MAGNI_PID_ERROR_RIGHT,
	/** Reports the current deviation from origin of the left motor, in m */
	SENSOR_CHAN_UR_MAGNI_POSITION_LEFT,
	/** Reports the current deviation from origin of the right motor, in m */
	SENSOR_CHAN_UR_MAGNI_POSITION_RIGHT,
	/** Reports the measured velocity of the left motor, in m/s */
	SENSOR_CHAN_UR_MAGNI_VELOCITY_LEFT,
	/** Reports the measured velocity of the right motor, in m/s */
	SENSOR_CHAN_UR_MAGNI_VELOCITY_RIGHT,
	/** Reports whether the left motor PWM limit has been reached.
	 * PWM limits ensure that the motor controller doesn't overheat.
	 */
	SENSOR_CHAN_UR_MAGNI_PWM_LIMIT_LEFT,
	/** Reports whether the right motor PWM limit has been reached.
	 * PWM limits ensure that the motor controller doesn't overheat.
	 */
	SENSOR_CHAN_UR_MAGNI_PWM_LIMIT_RIGHT,
	/** Reports a limit in the PID loop for the left motor */
	SENSOR_CHAN_UR_MAGNI_INTEGRAL_LIMIT_LEFT,
	/** Reports a limit in the PID loop for the right motor */
	SENSOR_CHAN_UR_MAGNI_INTEGRAL_LIMIT_RIGHT,
	/** Reports whether the speed limit has been reached on the left motor */
	SENSOR_CHAN_UR_MAGNI_VELOCITY_LIMIT_LEFT,
	/** Reports whether the speed limit has been reached on the right motor */
	SENSOR_CHAN_UR_MAGNI_VELOCITY_LIMIT_RIGHT,
	/** Reports whether a configuration register was set to an out of bounds value */
	SENSOR_CHAN_UR_MAGNI_PARAMETER_LIMIT,
	/** Reports the current in amps of the left motor */
	SENSOR_CHAN_UR_MAGNI_MOTOR_CURRENT_LEFT,
	/** Reports the current in amps of the right motor */
	SENSOR_CHAN_UR_MAGNI_MOTOR_CURRENT_RIGHT,
	/** Reports the PWM duty cycle of the left motor driver */
	SENSOR_CHAN_UR_MAGNI_MOTOR_PWM_LEFT,
	/** Reports the PWM duty cycle of the right motor driver */
	SENSOR_CHAN_UR_MAGNI_MOTOR_PWM_RIGHT,
	/** Reports time passed between two encoder ticks on left motor */
	SENSOR_CHAN_UR_MAGNI_ENCODER_TICK_INTERVAL_LEFT,
	/** Reports time passed between two encoder ticks on right motor */
	SENSOR_CHAN_UR_MAGNI_ENCODER_TICK_INTERVAL_RIGHT,
};

/**
 * @brief For internal use only
 *
 * Communication over the UART bus with the robot is handled with FIFO buffers.
 * These buffers are abstracted from the application using the driver and should
 * never be accessed directly outside of the driver.
 */
struct __aligned(16) ur_magni_fifo_item_t {
	void *fifo_reserved;
	char msg[8];
};

/**
 * @brief Structure for keeping a motor state
 *
 * The Magni robot has two independent motors. It reports their odometry values
 * on a regular basis. The driver will automatically update the @c position field.
 *
 * The @c vel_actual field contains the measured velocity of the robot.
 * The internal PID loop will aim to reach the @c vel_desired value.
 * This is set by the application. Internally, the driver and robot use rad/s.
 * On the application side, position and velocity are expressed in m/s.
 */
struct ur_magni_joint {
	struct sensor_value pos, vel_actual, vel_desired;
};

/**
 * @brief Structure to hold live diagnostics data
 *
 * This structure is used to hold live configuration and diagnostics data.
 * Unlike @see ur_magni_parameters the contents of this struct are expected to
 * change. A live instance governed by a mutex mechanism is used by driver
 * internals. It should not be accessed directly. Instead one should always
 * use the generic sensor driver model to work on a snapshot of the diagnostics.
 */
struct ur_magni_diagnostics {
	int32_t fw_ver, fw_date, fw_options;
	int32_t pid_p, pid_i, pid_d, pid_v, pid_c, pid_buf_size;
	int32_t max_pwm;
	struct sensor_value odom_freq_max, odom_freq_min;
	uint8_t limit_pwm_left, limit_pwm_right, limit_int_left, limit_int_right;
	uint8_t limit_vel_left, limit_vel_right, limit_param;
	struct sensor_value batt_v;
	uint8_t batt_low;
	uint8_t batt_crit;
	struct sensor_value amps_left, amps_right;
	int32_t pwm_left, pwm_right; //FIXME: original source lists this as "int", but I suspect it's actually unsigned
	//TODO: Main and Aux voltage diagnostics, currently unsupported in FW?
	uint8_t estop_motor_off;
	struct ur_magni_joint joint_left, joint_right;
	int16_t pid_error_left, pid_error_right;
	uint32_t odom_updates;
	uint16_t enc_tic_int_left, enc_tic_int_right;
	int32_t sys_events;
};

/**
 * @brief Structure to hold fixed configuration parameters
 *
 * This structure is used to initialize the Magni robot. The contents are not
 * supposed to change at runtime, however not all of them are defined at compile
 * time. In particular the option_switch value is read out from a GPIO expander
 * on the robot's I2C bus, provided that it is configured in KConfig.
 * Otherwise a standard value is used.
 *
 * For safety reasons the values for the deadman_timer and everything related to
 * the battery should not ever deviate from those used in the original source.
 *
 * The behavior of the robot's internal PID loop is not fully understood.
 * Changing the parameters may yield undesirable results.
 */
struct ur_magni_parameters {
	int32_t pid_p, pid_i, pid_d, pid_v, pid_c, pid_buf_size, pid_ctrl;
	int32_t board_ver, estop_detect, estop_pid_threshold;
	int32_t max_vel_fwd, max_vel_rev, max_pwm;
	int32_t deadman_timer;
	int32_t deadzone_enable;
	int32_t hw_options;
	int32_t option_switch;
	struct sensor_value batt_multiplier;
	struct sensor_value batt_offset;
	struct sensor_value batt_low_lvl;
	struct sensor_value batt_crit_lvl;
	struct sensor_value amps_multiplier;
	struct sensor_value amps_offset;
	struct sensor_value wheel_slip_threshold;
	struct sensor_value wheel_slip_nulling_period;
	struct sensor_value wheel_radius;
	struct sensor_value wheel_separation;
};

struct ur_magni_data {
	char *buf;
	atomic_t flush_req;
	size_t expected_rx;
	struct ur_magni_fifo_item_t *rdata;
	size_t expected_tx;
	struct ur_magni_fifo_item_t *tdata;
	struct k_mem_slab slab;
	struct k_fifo in_fifo;
	struct k_fifo out_fifo;
	struct ur_magni_diagnostics live_diags;
	struct ur_magni_diagnostics snap_diags;
	struct ur_magni_parameters parameters;
	struct k_mutex live_diags_mut;
	K_KERNEL_STACK_MEMBER(parser_stack, UR_MAGNI_PARSER_THREAD_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(joint_stack, UR_MAGNI_JOINT_THREAD_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(system_stack, UR_MAGNI_SYSTEM_THREAD_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(vel_stack, UR_MAGNI_VELOCITY_THREAD_STACK_SIZE);
	struct k_thread parser_thread, joint_thread, system_thread, vel_thread;
	k_tid_t parser_tid, joint_tid, system_tid, vel_tid;
	atomic_t parser_watchdog, joint_watchdog, system_watchdog, vel_watchdog;
	struct k_timer parser_timer, joint_timer, system_timer, vel_timer;
#ifndef UART_INTERRUPT_DRIVEN
	K_KERNEL_STACK_MEMBER(uart_rx_stack, UR_MAGNI_UART_THREAD_STACK_SIZE);
	K_KERNEL_STACK_MEMBER(uart_tx_stack, UR_MAGNI_UART_THREAD_STACK_SIZE);
	struct k_thread uart_rx_thread, uart_tx_thread;
	k_tid_t uart_rx_tid, uart_tx_tid;
	struct k_timer uart_rx_timer, uart_tx_timer;
#endif
};

struct ur_magni_config {
	const struct device *uart;
#ifdef CONFIG_UR_MAGNI_EXPANDER
	const struct device *i2c;
	const uint8_t expander_reg;
#endif /* CONFIG_UR_MAGNI_EXPANDER */
};

extern const struct ur_magni_diagnostics ur_magni_diagnostics_initial;
extern const struct ur_magni_parameters ur_magni_parameters_standard;

/**
 * @brief Set desired velocity for both motors
 *
 * @param dev device pointer
 * @param left_vel desired velocity for left motor in m/s
 * @param right_vel desired velocity for right motor in m/s
 * @param timeout scheduling timeout to wait for until the setting can be made
 * @return 0 on success, negative on error (i.e. LibMagni is busy)
 */
int ur_magni_set_velocity(const struct device *dev,
		struct sensor_value *left_vel, struct sensor_value *right_vel,
		k_timeout_t timeout);

int ur_magni_sample_fetch_with_timeout(const struct device *dev, k_timeout_t timeout);

#ifdef __cplusplus
}
#endif
#endif
