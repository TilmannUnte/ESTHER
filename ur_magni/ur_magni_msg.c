/*
 * Copyright (c) 2021-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
 */
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ur_magni_msg.h"

LOG_MODULE_REGISTER(ur_magni_msg);

//TODO add support for higher accuracy encoder, as it is now the standard model
//TODO: this is based on 4.294 * 4.774556 * 2.0
const struct sensor_value ur_magni_ticks_per_rad = {
		.val1 = 41,//41
		.val2 = 3887//4000
};

// The following function has never been used in our project
/*
static void ur_magni_ticks_to_radians(int16_t ticks, struct sensor_value *out)
{
	int64_t bigticks = ticks * UR_MAGNI_VELOCITY_UPDATE_PER_SEC * 1000000;
	bigticks *= 1000000;
	bigticks /= sensor_value_to_micro(&ur_magni_ticks_per_rad) * 4;

	out->val1 = bigticks / 1000000;
	out->val2 = bigticks - out->val1 * 1000000;
}
*/

static int16_t ur_magni_radians_to_vel(struct sensor_value *in)
{
/* TODO for high precision encoders, the result needs to be halved */

	int64_t bigspeed = sensor_value_to_micro(in);
	bigspeed *= sensor_value_to_micro(&ur_magni_ticks_per_rad) * 4;
	bigspeed /= 1000000;
	bigspeed /= UR_MAGNI_VELOCITY_UPDATE_PER_SEC;
	bigspeed /= 1000000;

	int16_t speed = bigspeed;

	return speed;
}

static int ur_magni_msg_set_data(char* const out_msg, int32_t in_val)
{
	out_msg[3] = (in_val & 0xff000000) >> 24;
	out_msg[4] = (in_val & 0x00ff0000) >> 16;
	out_msg[5] = (in_val & 0x0000ff00) >> 8;
	out_msg[6] = (in_val & 0x000000ff);

	return 0;
}

static int ur_magni_chksum_generate(char* const inout_msg)
{
	char sum = 0;
	for (int i = 1; i < UR_MAGNI_MSG_SIZE - 1; i++) {
		sum += inout_msg[i];
	}
	inout_msg[UR_MAGNI_MSG_SIZE - 1] = 0xFF - sum;
	return 0;
}

static int ur_magni_chksum_valid(const char* const msg)
{
	char tmp[UR_MAGNI_MSG_SIZE];
	int ret = 0;

	for (size_t i = 0; i < UR_MAGNI_MSG_SIZE; i++) {
		tmp[i] = msg[i];
	}
	
	ret = ur_magni_chksum_generate(tmp);
	if (ret) return ret;
	if (tmp[UR_MAGNI_MSG_SIZE - 1] != msg[UR_MAGNI_MSG_SIZE - 1]) {
		LOG_HEXDUMP_WRN(msg, UR_MAGNI_MSG_SIZE, "Bad magni checksum!");
		return 0;
	}
	return 1;
}

static int ur_magni_type_valid(const char* const in_msg)
{
	switch (in_msg[1] & 0x0F) {
	case UR_MAGNI_MSG_READ:
	case UR_MAGNI_MSG_WRITE:
	case UR_MAGNI_MSG_RESPONSE:
	case UR_MAGNI_MSG_ERROR: return 1; break;
	default:
		LOG_WRN("Invalid magni message type!");
		return 0; break;
	}
}

static int ur_magni_header_valid(const char* const in_msg)
{
	return in_msg[0] == UR_MAGNI_MSG_HDR;
}

static int ur_magni_protocol_valid(const char* const in_msg)
{
	return (in_msg[1] & 0xF0) == UR_MAGNI_PRTCL_VER;
}

static int ur_magni_regaccess_valid(const char* const in_msg)
{
	if (in_msg[2] <= 0x3c) {
		if ((in_msg[1] & 0x0F) == UR_MAGNI_MSG_WRITE) {
			switch (in_msg[2]) {
			case UR_MAGNI_REG_BRAKE: return 1; break;
			case UR_MAGNI_REG_DRIVE_TYPE: return 1; break;
			case UR_MAGNI_REG_WHEEL_NULL_ERR: return 1; break;
			case UR_MAGNI_REG_WHEEL_DIR: return 1; break;
			case UR_MAGNI_REG_DEADMAN_TIMER: return 1; break;
			case UR_MAGNI_REG_WHEEL_TYPE: return 1; break;
			case UR_MAGNI_REG_PID_ERROR_MAX: return 1; break;
			case UR_MAGNI_REG_OPTION_SWITCH: return 1; break;
			case UR_MAGNI_REG_PWM_OVERRIDE: return 1; break;
			case UR_MAGNI_REG_PID_CONTROL: return 1; break;
			case UR_MAGNI_REG_PID_V_RDY: return 1; break;
			case UR_MAGNI_REG_PID_P_RDY: return 1; break;
			case UR_MAGNI_REG_PID_I_RDY: return 1; break;
			case UR_MAGNI_REG_PID_D_RDY: return 1; break;
			case UR_MAGNI_REG_PID_C_RDY: return 1; break;
			case UR_MAGNI_REG_PID_V: return 1; break;
			case UR_MAGNI_REG_PID_P: return 1; break;
			case UR_MAGNI_REG_PID_I: return 1; break;
			case UR_MAGNI_REG_PID_D: return 1; break;
			case UR_MAGNI_REG_PID_C: return 1; break;
			case UR_MAGNI_REG_LED_1: return 1; break;
			case UR_MAGNI_REG_LED_2: return 1; break;
			case UR_MAGNI_REG_HW_VER: return 1; break;
			case UR_MAGNI_REG_VEL_BUF_LEN: return 1; break;
			case UR_MAGNI_REG_VEL_BOTH: return 1; break;
			case UR_MAGNI_REG_PID_BUF_LEN: return 1; break;
			case UR_MAGNI_REG_ESTOP_ENABLE: return 1; break;
			case UR_MAGNI_REG_SPEED_MAX_FWD: return 1; break;
			case UR_MAGNI_REG_SPEED_MAX_REV: return 1; break;
			case UR_MAGNI_REG_MAX_PWM: return 1; break;
			case UR_MAGNI_REG_SPEED_DEADZONE: return 1; break;
			case UR_MAGNI_REG_SELFTEST_START: return 1; break;
			default:
				LOG_WRN("Illegal magni write to %x!", in_msg[2]);
				return 0; break;
			}
		} else { // is read, response, or error
			switch (in_msg[2]) {
			case UR_MAGNI_REG_DEPRECATED_5:
			case UR_MAGNI_REG_DEPRECATED_6:
				LOG_WRN("Read from deprecated reg %x", in_msg[2]);
				return 0; break;
			default: return 1; break; // others are valid
			}
		}
	} else {
		LOG_ERR("Invalid magni register number %x", in_msg[2]);
		return 0; // bad input register
	}
}

int ur_magni_msg_get_data(const char* const in_msg, int32_t* out_val)
{
	if (in_msg == NULL || out_val == NULL) {
		LOG_ERR("Tried to read data from NULL pointer!");
		return -EINVAL;
	}
	*out_val = 0;
	//FIXME: complains if in_msg == 0xFF, but I don't know how to improve it
	*out_val |= ((uint8_t) in_msg[3]) << 24;
	*out_val |= ((uint8_t) in_msg[4]) << 16;
	*out_val |= ((uint8_t) in_msg[5]) << 8;
	*out_val |= ((uint8_t) in_msg[6]);

	return 0;
}

int ur_magni_msg_valid(const char* const in_msg)
{
	if (in_msg == NULL) {
		LOG_ERR("Tried to read data from NULL pointer!");
		return -EINVAL;
	}
	return (ur_magni_chksum_valid(in_msg)
	&& ur_magni_type_valid(in_msg)
	&& ur_magni_header_valid(in_msg)
	&& ur_magni_protocol_valid(in_msg)
	&& ur_magni_regaccess_valid(in_msg));
}

int ur_magni_read(char* const out_msg, enum ur_magni_reg reg)
{
	// Does not verify the register value!
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_READ;
	out_msg[2] = reg;
	int ret = ur_magni_msg_set_data(out_msg, 0);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_vel(char* const out_msg, struct sensor_value *vel_left,
		struct sensor_value *vel_right)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	int16_t left, right;
	int32_t combined;
	left = ur_magni_radians_to_vel(vel_left);
	right = ur_magni_radians_to_vel(vel_right);
	combined = (left << 16) | (right & 0x0000ffff);

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_VEL_BOTH;
	int ret = ur_magni_msg_set_data(out_msg, combined);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_hwver(char* const out_msg, uint8_t version)
{
	/* Magni requires HW revision number to be communicated over serial
	 * Format 0x0000MMmm, where MM is major and mm is minor number */
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}
	if (version > 99) {
		LOG_WRN("Tried setting magni hardware version to bad value!");
		return -EINVAL;
	}
	int32_t combined = 0;
	uint8_t major, minor;
	major = version / 10;
	minor = version % 10;
	combined |= (major << 8);
	combined |= minor;

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_HW_VER;
	int ret = ur_magni_msg_set_data(out_msg, combined);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_estopstate(char* const out_msg, int estop_state)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_ESTOP_ENABLE;
	int ret = ur_magni_msg_set_data(out_msg, estop_state);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_estoplevel(char* const out_msg, int estop_level)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_PID_MAX_ERROR;
	int ret = ur_magni_msg_set_data(out_msg, estop_level);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_maxfwdvel(char* const out_msg, int max_vel)
{
	/*FIXME: It's unclear which formatting the speed value uses here
	 * The original source does not convert it from rad/s to ticks */
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_SPEED_MAX_FWD;
	int ret = ur_magni_msg_set_data(out_msg, max_vel);

	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_wheeltype(char* const out_msg, int wheel_type)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}
	if (wheel_type != UR_MAGNI_HW_OPTION_STD_WHEEL
	&& wheel_type != UR_MAGNI_HW_OPTION_THIN_WHEEL) {
		LOG_WRN("Tried setting magni wheeltype to nonsense value!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_WHEEL_TYPE;

	int ret = ur_magni_msg_set_data(out_msg, wheel_type);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_wheeldir(char* const out_msg, int wheel_dir)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}
	if (wheel_dir != UR_MAGNI_HW_OPTION_STD_DIR
	&& wheel_dir != UR_MAGNI_HW_OPTION_REV_DIR) {
		LOG_WRN("Tried setting magni wheel direction to bad value!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_WHEEL_DIR;

	int ret = ur_magni_msg_set_data(out_msg, wheel_dir);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_drivetype(char* const out_msg, int drive_type)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}
	if (drive_type != UR_MAGNI_HW_OPTION_STD_DRIVE
	&& drive_type != UR_MAGNI_HW_OPTION_4WD) {
		LOG_WRN("Tried setting magni drive type to bad value!");
		return -EINVAL;
	}
	
	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_DRIVE_TYPE;
	
	int ret = ur_magni_msg_set_data(out_msg, drive_type);
	if (ret != 0) return ret;
	
	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_pidcontrol(char* const out_msg, int pid_control)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}
	
	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_CONTROL;
	
	int ret = ur_magni_msg_set_data(out_msg, pid_control);
	if (ret != 0) return ret;
	
	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_nullerror(char* const out_msg)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}
	
	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_WHEEL_NULL_ERR;
	
	int ret = ur_magni_msg_set_data(out_msg,
			UR_MAGNI_MOTOR_1 | UR_MAGNI_MOTOR_2);
	if (ret != 0) return ret;
	
	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_sysevents(char* const out_msg, int sys_events)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_SYS_EVENTS;
	int ret = ur_magni_msg_set_data(out_msg, sys_events);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_maxrevvel(char* const out_msg, int max_vel)
{
	/*FIXME: It's unclear which formatting the speed value uses here
	 * The original source does not convert it from rad/s to ticks */

	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_SPEED_MAX_REV;
	int ret = ur_magni_msg_set_data(out_msg, max_vel);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_maxpwm(char* const out_msg, int max_pwm)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_MAX_PWM;
	int ret = ur_magni_msg_set_data(out_msg, max_pwm);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_deadman(char* const out_msg, int timeout)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_DEADMAN_TIMER;
	int ret = ur_magni_msg_set_data(out_msg, timeout);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_deadzone(char* const out_msg, int deadzone)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_SPEED_DEADZONE;
	int ret = ur_magni_msg_set_data(out_msg, deadzone);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_led1(char* const out_msg, int status)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_LED_1;
	int ret = ur_magni_msg_set_data(out_msg, status?1:0);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_led2(char* const out_msg, int status)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_LED_2;
	int ret = ur_magni_msg_set_data(out_msg, status?1:0);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_p(char* const out_msg, int p_val)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_P;
	int ret = ur_magni_msg_set_data(out_msg, p_val);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}
int ur_magni_write_i(char* const out_msg, int i_val)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_I;
	int ret = ur_magni_msg_set_data(out_msg, i_val);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_d(char* const out_msg, int d_val)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_D;
	int ret = ur_magni_msg_set_data(out_msg, d_val);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_v(char* const out_msg, int v_val)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_V;
	int ret = ur_magni_msg_set_data(out_msg, v_val);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_c(char* const out_msg, int c_val)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_C;
	int ret = ur_magni_msg_set_data(out_msg, c_val);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_bufsize(char* const out_msg, int buf_size)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_PID_BUF_LEN;
	int ret = ur_magni_msg_set_data(out_msg, buf_size);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}

int ur_magni_write_optionswitch(char* const out_msg, int option_switch)
{
	if (out_msg == NULL) {
		LOG_ERR("Tried to write data to NULL pointer!");
		return -EINVAL;
	}

	out_msg[0] = UR_MAGNI_MSG_HDR;
	out_msg[1] = UR_MAGNI_PRTCL_VER | UR_MAGNI_MSG_WRITE;
	out_msg[2] = UR_MAGNI_REG_OPTION_SWITCH;
	int ret = ur_magni_msg_set_data(out_msg, option_switch);
	if (ret != 0) return ret;

	return ur_magni_chksum_generate(out_msg);
}
