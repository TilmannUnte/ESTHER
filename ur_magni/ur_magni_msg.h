/*
 * Copyright (c) 2021-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
 */

/**
 * @file
 * This part of the Magni driver takes care creating, verifying and extracting
 * data from messages exchanged with the Magni motor controller over UART.
 *
 * The message protocol has been subject to frequent changes and revisions with
 * new firmware releases. Some of the official documentation is outdated and
 * does no longer apply. In particular be weary of a Markdown document in the
 * official github repo describing the protocol which hasn't been updated in
 * several years.
 * This part of the library is based on the official source code and
 * compatible with firmware version 40. Should changes be necessary, they must
 * be carefully validated against the official source yet again.
 *
 * Messages are 8 bytes in size. The first byte is always the special
 * @c MAGNI_MSG_HDR. It is followed by a byte that contains the
 * @c MAGNI_PRTCL_VER in the high nybble and a @see ur_magni_msgtype in the
 * lower nybble.
 *
 * Next follows a byte specifying a hardware register that is being accessed.
 * On @c WRITE and @c RESPONSE type messages the next four bytes are used to
 * encode a signed 32-bit value in big-endian. Some registers encode multiple
 * smaller values into this one 32-bit field.
 *
 * The message is terminated by a special checksum generated from its content.
 */

#ifndef UR_MAGNI_MSG_H_
#define UR_MAGNI_MSG_H_

#include <zephyr/kernel.h>
#include <stdint.h>
#include <zephyr/drivers/sensor.h>

#define UR_MAGNI_VELOCITY_UPDATE_PER_SEC 20

#define UR_MAGNI_MSG_SIZE 8
// The following two values may change in future Magni FW versions!
#define UR_MAGNI_MSG_HDR 0x7E
#define UR_MAGNI_PRTCL_VER 0x30

/**
 * @brief Message types for UART protocol
 *
 * The Magni robot uses four different message types encoded into the low nybble
 * of the second byte in every message.
 * The @c READ and @c WRITE types are used for register access.
 * The robot responds with the @c RESPONSE type. It frequently sends responses
 * despite not having received any requests. These include odometry updates for
 * example. The @c ERROR type is used to report back that a message could not be
 * parsed.
 */
enum ur_magni_msgtype {
	UR_MAGNI_MSG_READ = (0x0A),
	UR_MAGNI_MSG_WRITE = (0x0B),
	UR_MAGNI_MSG_RESPONSE = (0x0C),
	UR_MAGNI_MSG_ERROR = (0x0D)
};

/**
 * @brief Error codes for UART protocol
 *
 * Upon receiving a bad message the robot will return a message of type @c ERROR
 * with one of these error codes.
 */
enum ur_magni_error {
	UR_MAGNI_ERROR_NONE = 0,
	UR_MAGNI_ERROR_DELIMETER = 1,
	UR_MAGNI_ERROR_WRONG_PROTOCOL = 2,
	UR_MAGNI_ERROR_BAD_CHKSUM = 3,
	UR_MAGNI_ERROR_BAD_TYPE = 4,
	UR_MAGNI_ERROR_UNKNOWN_REG = 5
};

/**
 * @brief Registers for UART protocol
 *
 * The third byte of every message contains the Register identifier.
 * The register values have changed frequently over past firmware versions.
 * The list used in LibMagni is valid for v40 of the Magni firmware, which at
 * the time of writing is factory standard.
 *
 * @note The manufacturer's documentation is inconsistent and sometimes
 * outdated. For the most recent list of these registers one should always refer
 * to the newest version of the Magni ROS node software available on the official
 * Github landing.
 *
 * Very few registers are used in practice. The current and voltage measurements
 * are ignored in the original sources, only the battery voltage is monitored.
 *
 * Despite the fact that there are two sets of PID registers, only one is used
 * in original sources. The purpose of the @c RDY type registers is unclear.
 * The driver currently does not support live updates of the PID values, because
 * the internal functionality of the closed-source firmware is unknown.
 */
enum ur_magni_reg {
	UR_MAGNI_REG_STOP_START = 0x00,//**< deprecated
	UR_MAGNI_REG_BRAKE = 0x01,//**< state of braking system		RW
	UR_MAGNI_REG_SYS_EVENTS = 0x02,//**< reports system events	R
	UR_MAGNI_REG_PWM_L = 0x03,//**< PWM left			R
	UR_MAGNI_REG_PWM_R = 0x04,//**< PWM right			R
	UR_MAGNI_REG_DEPRECATED_5 = 0x05,//**< do not use
	UR_MAGNI_REG_DEPRECATED_6 = 0x06,//**< do not use
	UR_MAGNI_REG_PWM_BOTH = 0x07,//**< PWMs, 0xLLllRRrr		R
	UR_MAGNI_REG_TIC_INT_BOTH = 0x08,//**< Time between encoder tics, 0xLLllRRrr	R
	UR_MAGNI_REG_SPEED_RAMP_L = 0x09,//**< deprecated
	UR_MAGNI_REG_DRIVE_TYPE = 0x0A,//**< Type of wheel motor (v42)
	UR_MAGNI_REG_WHEEL_NULL_ERR = 0x0B,//**< Reset wheel PID targets
	UR_MAGNI_REG_WHEEL_DIR = 0x0C,//**< Setting wheel direction (v38)RW
	UR_MAGNI_REG_DEADMAN_TIMER = 0x0D,//**< watchdog, USE CAUTION	RW
	UR_MAGNI_REG_MOTOR_CURRENT_L = 0x0E,//**< motor current left mA	 R
	UR_MAGNI_REG_MOTOR_CURRENT_R = 0x0F,//**< motor current right mA R
	UR_MAGNI_REG_WHEEL_TYPE = 0x10,//**< setting wheel type	 	RW
	UR_MAGNI_REG_PID_ERROR_MAX = 0x11,//**< Limits for PID errors
	UR_MAGNI_REG_OPTION_SWITCH = 0x12,//**< For I2C switch values	RW
	UR_MAGNI_REG_PWM_OVERRIDE = 0x13,//**< usage unknown		RW
	UR_MAGNI_REG_PID_CONTROL = 0x14,//**< PID control switch	RW
	UR_MAGNI_REG_PID_V_RDY = 0x15,//**< prepared value		RW
	UR_MAGNI_REG_PID_P_RDY = 0x16,//**< prepared value		RW
	UR_MAGNI_REG_PID_I_RDY = 0x17,//**< prepared value		RW
	UR_MAGNI_REG_PID_D_RDY = 0x18,//**< prepared value		RW
	UR_MAGNI_REG_PID_C_RDY = 0x19,//**< prepared value		RW
	UR_MAGNI_REG_PID_V = 0x1A,//**< active value			RW
	UR_MAGNI_REG_PID_P = 0x1B,//**< active value			RW
	UR_MAGNI_REG_PID_I = 0x1C,//**< active value			RW
	UR_MAGNI_REG_PID_D = 0x1D,//**< active value			RW
	UR_MAGNI_REG_PID_C = 0x1E,//**< active value			RW
	UR_MAGNI_REG_LED_1 = 0x1F,//**< Debug LED 1 Bool		RW
	UR_MAGNI_REG_LED_2 = 0x20,//**< Debug LED 2 Bool		RW
	UR_MAGNI_REG_HW_VER = 0x21,//**< Setting board revision		RW
	UR_MAGNI_REG_FW_VER = 0x22,//**< firmware version		R
	UR_MAGNI_REG_BATT_VOLT = 0x23,//**< battery voltage ADC value	R
	UR_MAGNI_REG_5V_MAIN_CURRENT = 0x24,//**< current on 5V line mA	R
	UR_MAGNI_REG_MAIN_VOLT_TEST = 0x25,//**< check main line mV	R
	UR_MAGNI_REG_12V_MAIN_CURRENT = 0x26,//**< current on 12V line mA R
	UR_MAGNI_REG_AUX_VOLT_TEST = 0x27,//**< check aux line	 mV	R
	UR_MAGNI_REG_BATT_LOW = 0x28,//**< check for low battery	R
	UR_MAGNI_REG_VEL_BUF_LEN = 0x29,//**< velocity averaging buffer	RW
	UR_MAGNI_REG_VEL_BOTH = 0x2A,//**<  0xLLllRRrr	tics/100ms RW
	UR_MAGNI_REG_PID_BUF_LEN = 0x2B,//**< PID averaging buffer (<=100) RW
	UR_MAGNI_REG_LIMIT_REACHED = 0x2C,//**< reports on breached limits R
	UR_MAGNI_REG_PID_ERROR_BOTH = 0x2D,//**< PID error  0xLlRr	R
	UR_MAGNI_REG_ODOM_BOTH = 0x30,//**< 0xLLllRrr			R
	UR_MAGNI_REG_ROBOT_ID = 0x31,//**< always 0 for Magni		R
	UR_MAGNI_REG_MOTOR_POWER = 0x32,//**< 0x000l000r Bool	 	R
	UR_MAGNI_REG_ESTOP_ENABLE = 0x33, //**< status of estop function R
	UR_MAGNI_PID_MAX_ERROR = 0x34, //**< board ver 5.0 or below only R
	UR_MAGNI_REG_SPEED_MAX_FWD = 0x35, //**< sets velocity limit	RW
	UR_MAGNI_REG_SPEED_MAX_REV = 0x36, //**< sets velocity limit	RW
	UR_MAGNI_REG_MAX_PWM = 0x37, //**< Max PWM for PID		RW
	UR_MAGNI_REG_HW_OPTIONS = 0x38, //**< For non-standard HW configs R
	UR_MAGNI_REG_SPEED_DEADZONE = 0x39, //**< ignored speed values	RW
	UR_MAGNI_REG_FW_DATE = 0x3a, //**< build date of fw YYYYMMDD	R
	UR_MAGNI_REG_SELFTEST_START = 0x3b, //**< activate selftest	RW
	UR_MAGNI_REG_SELFTEST_RESULT = 0x3c //**< reports selftest results R
				//0x50-58 undocumented debug registers	R
};

/**
 * @brief Flag bits for register @c MAGNI_REG_PID_CONTROL
 */
enum ur_magni_pid_ctrl {
	UR_MAGNI_PID_CTRL_NORMAL = 0x0,
	UR_MAGNI_PID_CTRL_RESET = 0x1, //**< resets PID error
	UR_MAGNI_PID_CTRL_PWM_OVERRIDE = 0x2, //**< only used for testing
	UR_MAGNI_PID_CTRL_P_ONLY_ZERO_VEL = 0x4, //**< only use P on zero velocity
	UR_MAGNI_PID_CTRL_P_ONLY = 0x8, //**< only use P term, ignore I and D
	UR_MAGNI_PID_CTRL_SQ_ERR = 0x10, //**< squared error, better turns
	UR_MAGNI_PID_CTRL_ERR_MAX = 0x20, //**< PID error will be kept below max
	UR_MAGNI_PID_CTRL_P_BOOST = 0x40, //**< Higher gain for P term
	UR_MAGNI_PID_CTRL_P_TURBO = 0x80, //**< Additional gain for P term boost
	UR_MAGNI_PID_CTRL_SQ_ERR_AUTO = 0x100, //**< activate square error on turns automatically
	UR_MAGNI_PID_CTRL_P_BOOST_AUTO = 0x200, //**< activate P term boost on turns automatically
	UR_MAGNI_PID_CTRL_PID_VEL = 0x800, //**< activates PID velocity term if set
};

/**
 * @brief Flag bits for register @c MAGNI_REG_HW_OPTIONS
 *
 * There are several (proposed/custom) variants of the Magni robot.
 * @warning the driver is untested on non-standard Magni variants!
 * Odometry is expected to break if precision encoders are used.
 */
enum ur_magni_hw_option {
	UR_MAGNI_HW_OPTION_ENC_6_STATE = 0x01, //**< precision encoders
	UR_MAGNI_HW_OPTION_THIN_WHEEL = 0x02,
	UR_MAGNI_HW_OPTION_REV_DIR = 0x04,
	UR_MAGNI_HW_OPTION_4WD = 0x08,
	UR_MAGNI_HW_OPTION_STD_WHEEL = 0,
	UR_MAGNI_HW_OPTION_STD_DRIVE = 0, //**< 2 wheel-drive
	UR_MAGNI_HW_OPTION_STD_DIR = 0
};

enum ur_magni_motor {
	UR_MAGNI_MOTOR_1 = 1,
	UR_MAGNI_MOTOR_2 = 2
};

#define UR_MAGNI_SELFTEST_SHIFT 24
/**
 * @brief Flag bits for register @c MAGNI_REG_SELFTEST_RESULT
 */
enum ur_magni_selftest_state {
	UR_MAGNI_SELFTEST_IDLE = 0,
	UR_MAGNI_SELFTEST_MAIN_VOLT = 0x1,
	UR_MAGNI_SELFTEST_AUX_VOLT = 0x2,
	UR_MAGNI_SELFTEST_BATT_ERROR = 0x4,
	UR_MAGNI_SELFTEST_BATT_LOW = 0x8,
	UR_MAGNI_SELFTEST_MOTOR_PWR = 0x80,
	UR_MAGNI_SELFTEST_MOTOR_ENCS = 0x100,
	UR_MAGNI_SELFTEST_POWER_ON = 0x400,
	UR_MAGNI_SELFTEST_MOTOR_CURRENT_L = 0x1000,
	UR_MAGNI_SELFTEST_MOTOR_CURRENT_R = 0x2000,
	UR_MAGNI_SELFTEST_ENC_ERROR_L = 0x10000,
	UR_MAGNI_SELFTEST_ENC_ERROR_R = 0x20000,
	UR_MAGNI_SELFTEST_RUNNING = 0x800000,
	UR_MAGNI_SELFTEST_STATE = (0xF << UR_MAGNI_SELFTEST_SHIFT)
};

/**
 * @brief Bit Flags for register @c MAGNI_REG_SYS_EVENTS
 *
 * The firmware only reports a single event. The power on states can be read
 * periodically to check for a sudden reboot of the robot controller, possibly
 * due to power fluctuations or internal errors.
 * @note earlier firmwares don't report any events.
 */

enum ur_magni_sys_event {
	UR_MAGNI_SYS_EVENT_POWER_ON = 1
};

/**
 * @brief Bit Flags for register @c MAGNI_REG_LIMIT_REACHED
 *
 * There are four kinds of internal limits:
 * Motor PWM limits are reported when the pulse width is out of bounds.
 * Motor speed limits are reported when the desired velocity is above the preset
 * maximum values.
 * Parameter limits are reported when a @c WRITE access exceeds the internal
 * register size.
 * The exact origin of motor integral limits is not known. It's assumed to be
 * a PID related limit.
 */
enum ur_magni_limit {
	UR_MAGNI_LIMIT_MOTOR_PWM_L = 0x10,
	UR_MAGNI_LIMIT_MOTOR_PWM_R = 0x1,
	UR_MAGNI_LIMIT_MOTOR_INT_L = 0x20,
	UR_MAGNI_LIMIT_MOTOR_INT_R = 0x2,
	UR_MAGNI_LIMIT_MOTOR_SPEED_L = 0x40,
	UR_MAGNI_LIMIT_MOTOR_SPEED_R = 0x4,
	UR_MAGNI_LIMIT_PARAM = 0x80
};

extern const struct sensor_value ur_magni_ticks_per_rad;

extern const struct sensor_value ur_magni_qticks_per_rad;

/**
 * @brief Extracts a 32-bit signed value from a message.
 *
 * @param in_msg pointer to 8-byte array
 * @param[out] out_val pointer to output variable
 * @return zero on success, otherwise error code
 */
int ur_magni_msg_get_data(const char* const in_msg, int32_t* out_val);

/**
 * @brief Checks whether a message is valid
 *
 * This function should be called before any message is put on the UART bus
 * and it can also be used to check whether a received message is valid.
 * Specifically it checks whether the header is intact, the protocol value is
 * valid, the message type is supported, the register access is supported, and
 * lastly whether the checksum is correct.
 *
 * @param in_msg pointer to 8-byte array
 * @return non-zero on success
 */
int ur_magni_msg_valid(const char* const in_msg);

/**
 * @brief Constructs a new read access message
 *
 * @param[out] out_msg output pointer to 8-byte array
 * @param reg one of the magni registers from the specified enum
 * @return zero on success, otherwise error code
 */
int ur_magni_read(char* const out_msg, enum ur_magni_reg reg);

/**
 * @brief Constructs a speed write message
 *
 * @param[out] out_msg output pointer to 8-byte array
 * @param vel_left speed for left motor in radians/second
 * @param vel_right speed for right motor in radians/second
 * @return zero on success, otherwise error code
 * @warning does not check whether the input speed is less than current maximum
 * speed settings.
 */
int ur_magni_write_vel(char* const out_msg, struct sensor_value *vel_left,
		struct sensor_value *vel_right);

/**
 * @brief Constructs a board version write message
 *
 * The motor controller is not aware of the board revision it is built on top
 * of. During setup it needs to be configured correctly.
 * @warning Use this only if you know what you're doing
 *
 * @param[out] out_msg output pointer to 8-byte array
 * @param version numerical value of the board revision
 * @return zero on success, otherwise error code
 */
int ur_magni_write_hwver(char* const out_msg, uint8_t version);

/**
 * @brief Constructs a estop state write message
 *
 * The Estop feature is activated by the dedicated red button (or additional
 * attachments). This will stop the motors and the status change will be
 * reported. However, the state needs to be manually reset, preferably with some
 * delay, so that the robot will not instantly sharply accelerate.
 * @warning Use this only if you know what you're doing
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param estop_state input evaluated as boolean
 * @return zero on success, otherwise error code
 */
int ur_magni_write_estopstate(char* const out_msg, int estop_state);

/**
 * @brief Constructs an estop level write message
 *
 * @note The functionality of this feature is not fully understood.
 * In early magni versions the estop was linked to the PID regulation somehow.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param estop_level value for Estop PID Threshold
 * @return zero on success, otherwise error code
 */
int ur_magni_write_estoplevel(char* const out_msg, int estop_level);

/**
 * @brief Constructs a maximum forward speed write message
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param max_speed maximum forward speed
 * @return zero on success, otherwise error code
 */
int ur_magni_write_maxfwdvel(char* const out_msg, int max_speed);

/**
 * @brief Constructs a wheel type write message, should always be standard
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param wheel_type wheel type input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_wheeltype(char* const out_msg, int wheel_type);

/**
 * @brief Constructs a wheel direction write message, should always be forward
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param wheel_dir wheel direction input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_wheeldir(char* const out_msg, int wheel_dir);

/**
 * @brief Constructs a drive type write message, should always be standard
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param drive_type drive type input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_drivetype(char* const out_msg, int drive_type);

/**
 * @brief Constructs a write message to PID control register
 * 
 * @warning  PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param drive_type drive type input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_pidcontrol(char* const out_msg, int pid_control);

/**
 * @brief Constructs a write message to reset wheel setpoint based on current
 * position error. This allows to relieve stress in static situations where
 * wheels cannot slip.
 * 
 * @warning  PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param drive_type drive type input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_nullerror(char* const out_msg);

/**
 * @brief Constructs a system events write message
 *
 * This is only used to reset the system events register to 0.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param sys_events system events input value, should be 0
 * @return zero on success, otherwise error code
 */
int ur_magni_write_sysevents(char* const out_msg, int sys_events);

/**
 * @brief Constructs a maximum reverse speed write message
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param max_speed maximum reverse speed input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_maxrevvel(char* const out_msg, int max_speed);

/**
 * @brief Constructs a maximum PWM write message
 *
 * @warning PWM internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param max_pwm maximum PWM input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_maxpwm(char* const out_msg, int max_pwm);

/**
 * @brief Constructs a deadman timer write message
 *
 * The deadman timer is a watchdog functionality. The internals are not
 * documented.
 * @warning However, the original sources point out to use extreme caution with
 * this setting. Therefore only use the preset value.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param timeout deadman timer input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_deadman(char* const out_msg, int timeout);

/**
 * @brief Constructs a deadzone write message
 *
 * The deadzone is a region around a speed value of zero that will be ignored by
 * the motor controller. This can be useful if the motors fall out of calibration
 * and will slowly stir from their steady position over time.
 * However, normally this setting is not used.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param deadzone deadzone input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_deadzone(char* const out_msg, int deadzone);

/**
 * @brief Constructs a LED 1 write message
 *
 * Takes a boolean to set the status of an LED for debugging purposes.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param status evaluated as boolean input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_led1(char* const out_msg, int status);

/**
 * @brief Constructs a LED 2 write message
 *
 * Takes a boolean to set the status of an LED for debugging purposes.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param status evaluated as boolean input value
 * @return zero on success, otherwise error code
 */
int ur_magni_write_led2(char* const out_msg, int status);

/**
 * @brief Constructs a P value write message
 *
 * @warning PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param p_val P value input
 * @return zero on success, otherwise error code
 */
int ur_magni_write_p(char* const out_msg, int p_val);

/**
 * @brief Constructs a I value write message
 *
 * @warning PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param i_val I value input
 * @return zero on success, otherwise error code
 */
int ur_magni_write_i(char* const out_msg, int i_val);

/**
 * @brief Constructs a D value write message
 *
 * @warning PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param d_val D value input
 * @return zero on success, otherwise error code
 */
int ur_magni_write_d(char* const out_msg, int d_val);

/**
 * @brief Constructs a V value write message
 *
 * @warning PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param v_val V value input
 * @return zero on success, otherwise error code
 */
int ur_magni_write_v(char* const out_msg, int v_val);

/**
 * @brief Constructs a C value write message
 *
 * @warning PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param c_val C value input
 * @return zero on success, otherwise error code
 */
int ur_magni_write_c(char* const out_msg, int c_val);

/**
 * @brief Constructs a PID buffer size value write message
 *
 * @warning PID internals are not documented. Don't stray from preset values.
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param buf_size PID buffer size value input
 * @return zero on success, otherwise error code
 */
int ur_magni_write_bufsize(char* const out_msg, int buf_size);

/**
 * @brief Constructs an option switch write message
 *
 * The motor controller has very limited GPIO. Therefore the board has an
 * expander that is connected to the host MCU. The host MCU is required to read
 * the expander values and send them to the motor controller once upon initial
 * configuration.
 * @note The original sources indicate that this could be done periodically, but
 * doesn't actually do so currently
 *
 * @param[out] out_msg pointer to 8-byte array
 * @param option_switch values from GPIO expander read by host MCU
 * @return zero on success, otherwise error code
 */
int ur_magni_write_optionswitch(char* const out_msg, int option_switch);

#endif /* UR_MAGNI_MSG_H_ */
