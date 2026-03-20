/*
 * Copyright (c) 2021-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/
#define DT_DRV_COMPAT ur_magni

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/irq_offload.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/drivers/watchdog.h>

#include <zephyr/timing/timing.h>
#include <zephyr/debug/thread_analyzer.h>

#include "ur_magni.h"
#include "ur_magni_msg.h"

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(watchdog0))
#define WDT_NODE DT_ALIAS(watchdog0)
#else
#define WDT_NODE DT_INVALID_NODE
#endif

LOG_MODULE_REGISTER(ur_magni);

const struct ur_magni_diagnostics ur_magni_diagnostics_initial = {
	.fw_ver = 0,
	.fw_date = 0,
	.fw_options = 0,
	.pid_p = 5000,//4000,//5100,
	.pid_i = 0,//5,//2,
	.pid_d = -110,//-200,//-111,
	.pid_v = 0, //0,
	.pid_c = 1000,
	.pid_buf_size = 70, //10,
	.max_pwm = 325,
	.odom_freq_max.val1 = 1000,
	.odom_freq_max.val2 = 0,
	.odom_freq_min.val1 = 50,
	.odom_freq_min.val2 = 0,
	.limit_pwm_left = 0,
	.limit_pwm_right = 0,
	.limit_int_left = 0,
	.limit_int_right = 0,
	.limit_vel_left = 0,
	.limit_vel_right = 0,
	.limit_param = 0,
	.batt_v.val1 = 0,
	.batt_v.val2 = 0,
	.batt_low = 0,
	.batt_crit = 0,
	.amps_left.val1 = 0,
	.amps_left.val2 = 0,
	.amps_right.val1 = 0,
	.amps_right.val2 = 0,
	.pwm_left = 0,
	.pwm_right = 0,
	.estop_motor_off = 0,
	.joint_left.pos.val1 = 0,
	.joint_left.pos.val2 = 0,
	.joint_left.vel_actual.val1 = 0,
	.joint_left.vel_actual.val2 = 0,
	.joint_left.vel_desired.val1 = 0,
	.joint_left.vel_desired.val2 = 0,
	.joint_right.pos.val1 = 0,
	.joint_right.pos.val2 = 0,
	.joint_right.vel_actual.val1 = 0,
	.joint_right.vel_actual.val2 = 0,
	.joint_right.vel_desired.val1 = 0,
	.joint_right.vel_desired.val2 = 0,
	.pid_error_left = 0,
	.pid_error_right = 0,
	.odom_updates = 0,
	.enc_tic_int_left = 0,
	.enc_tic_int_right = 0,
	.sys_events = 0,
};

const struct ur_magni_parameters ur_magni_parameters_standard = {
	.pid_p = 5000,//4000,//5100,
	.pid_i = 0,//5,//2,
	.pid_d = -110,//-200,//-111,
	.pid_v = 0,//0,
	.pid_c = 1000,
	.pid_buf_size = 70,//10,
	.pid_ctrl = 0,
	.board_ver = 53,
	.estop_detect = 1,
	.estop_pid_threshold = 1500,
	.max_vel_fwd = 104,
	.max_vel_rev = -104,
	.max_pwm = 325,
	.deadman_timer = 2400000,
	.deadzone_enable = 0,
	.hw_options = 0,
	.option_switch = 0,
	.batt_multiplier.val1 = 0,
	.batt_multiplier.val2 = 51270,
	.batt_offset.val1 = 0,
	.batt_offset.val2 = 0,
	.batt_low_lvl.val1 = 23,
	.batt_low_lvl.val2 = 200000,
	.batt_crit_lvl.val1 = 22,
	.batt_crit_lvl.val2 = 500000,
	.amps_multiplier.val1 = 1015,
	.amps_multiplier.val2 = 0,
	.amps_offset.val1 = 0,
	.amps_offset.val2 = 23800,
	.wheel_slip_threshold.val1 = 0,
	.wheel_slip_threshold.val2 = 80000,
	.wheel_slip_nulling_period.val1 = 2,
	.wheel_slip_nulling_period.val2 = 0,
	.wheel_radius.val1 = 0,
	.wheel_radius.val2 = 101500,
	.wheel_separation.val1 = 0,
	.wheel_separation.val2 = 330000
};

#ifdef CONFIG_UR_MAGNI_EXPANDER
static int ur_magni_i2c_init(const struct device *dev) {
	const struct ur_magni_config *cfg = dev->config;

	if (cfg->i2c == NULL) {
		LOG_ERR("I2C device not found!");
		return -ENXIO;
	}

	if (i2c_configure(cfg->i2c,
			I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER)
			!= 0) {
		LOG_ERR("Can't configure I2C device!");
		return -ENXIO;
	}
	LOG_INF("Initialized I2C connection!");
	return 0;
}

static int ur_magni_gpio_expander_read(const struct device *dev, uint8_t *out)
{
	const struct ur_magni_config *cfg = dev->config;

	uint8_t val;

	/* The expander has no internal state, so we just access it directly */
	if (i2c_read(cfg->i2c, &val, 1, cfg->expander_reg) != 0)
	{
		LOG_WRN("LibMagni: Couldn't read from I2C GPIO expander.");
		return -EIO;
	}

	*out = 0xff & ~val; /* The expander is inverting */
	return 0;
}
#endif /* CONFIG_UR_MAGNI_EXPANDER */

#ifdef CONFIG_WATCHDOG
static void ur_magni_watchdog_callback(int channel_id, void* user_data)
{
	struct ur_magni_data *data = user_data;
	k_tid_t thread = NULL;
	if (channel_id == atomic_get(&data->parser_watchdog)) {
		thread = data->parser_tid;
	} else if (channel_id == atomic_get(&data->system_watchdog)) {
		thread = data->system_tid;
	} else if (channel_id == atomic_get(&data->vel_watchdog)) {
		thread = data->vel_tid;
	} else if (channel_id == atomic_get(&data->joint_watchdog)) {
		thread = data->joint_tid;
	}
	#ifdef CONFIG_THREAD_NAME
	const char* name = k_thread_name_get(thread);
	if (name != NULL) {
		LOG_DBG("%s deadline", name);
	} else {
		LOG_DBG("%p on chan %i deadline", thread, channel_id);
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

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void ur_magni_uart_callback(const struct device* dev, void* user_data)
{
	/* Watch out: dev is the UART device, not the magni device*/
	ARG_UNUSED(dev);
	const struct device *magni = user_data;
	struct ur_magni_data *data = magni->data;
	const struct ur_magni_config *cfg = magni->config;
	int ret;

	if (uart_irq_update(cfg->uart)) {
		if (uart_irq_rx_ready(cfg->uart)) {
			if (atomic_get(&data->flush_req)) {
				char c = '\0';
				
				while (c != UR_MAGNI_MSG_HDR) {
					if  (uart_fifo_read(cfg->uart, &c, sizeof(char)) != sizeof(char)) {
						break;
					}
				}
				
				if (c == UR_MAGNI_MSG_HDR) {
					// we flushed to the start of the next msg
					atomic_set(&data->flush_req, false);
					//LOG_INF("Flushed");
					// reserve a slot, if we don't have one already
					if (data->expected_rx == 0) {
						ret = k_mem_slab_alloc(&data->slab, (void**) &data->rdata,
								K_NO_WAIT);
						__ASSERT(ret == 0, "Out of memory!\n");
					}
					// overwrite any old/broken msg contents with the new/correct one
					data->expected_rx = UR_MAGNI_MSG_SIZE * sizeof(char);
					data->expected_rx -= sizeof(char);
					data->rdata->msg[0] = UR_MAGNI_MSG_HDR;
				}
			}
			
			if (!atomic_get(&data->flush_req)) {
				// we are not flushing or just finished flushing
				if (data->expected_rx == 0) {
					data->expected_rx = UR_MAGNI_MSG_SIZE * sizeof(char);
					ret = k_mem_slab_alloc(&data->slab, (void**) &data->rdata,
								K_NO_WAIT);
					__ASSERT(ret == 0, "Out of memory!\n");
				}
				data->expected_rx -= uart_fifo_read(cfg->uart, &(data->rdata->msg[UR_MAGNI_MSG_SIZE - data->expected_rx]), data->expected_rx * sizeof(char));

				if (data->expected_rx == 0) {
					k_fifo_put(&data->in_fifo, data->rdata);
				}
			}
		}

		if (uart_irq_tx_ready(cfg->uart)) {
			if (data->expected_tx == 0) {
				data->tdata = k_fifo_get(&data->out_fifo, K_NO_WAIT);
				if (data->tdata) {
					data->expected_tx = UR_MAGNI_MSG_SIZE * sizeof(char);
				}
			}
			
			//FIXME: expected_tx should always be greater than 0 at this point, since we only go here when IRQ TX is enabled. However on some boards it is ALWAYS enabled...
			data->expected_tx -= uart_fifo_fill(cfg->uart,
				&(data->tdata->msg[UR_MAGNI_MSG_SIZE - data->expected_tx]), data->expected_tx * sizeof(char));
			if (data->expected_tx == 0 && data->tdata != NULL) {
				k_mem_slab_free(&data->slab,
					(void*) data->tdata);
				data->tdata = NULL;
				if (k_fifo_is_empty(&data->out_fifo)) {
					/* It is required to disable TX IRQs when
					there's no data, otherwise Interrupts will
					continuously fire and lock up the system */
					uart_irq_tx_disable(cfg->uart);
				}
			}
		}
	}
}
#else
static void ur_magni_uart_rx_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ur_magni_data *data = dev->data;
	const struct ur_magni_config *cfg = dev->config;

	int ret;

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif

	//FIXME: Watchdogs in Zephyr are limited to millisecond accuracy, therefore we have no watchdog for this thread
	for (;;) {
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->uart_rx_timer);
		#else
		k_timer_status_sync(&data->uart_rx_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->uart_rx_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->uart_rx_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->uart_rx_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */

		// Receive
		if (!atomic_get(&data->flush_req)) {
			if (data->expected_rx == 0) {
				data->expected_rx = UR_MAGNI_MSG_SIZE * sizeof(char);
				ret = k_mem_slab_alloc(&data->slab, (void**) &data->rdata,
							K_NO_WAIT);
				__ASSERT(ret == 0, "Out of memory!\n");
			}

			while (data->expected_rx > 0 && uart_poll_in(cfg->uart, &(data->rdata->msg[UR_MAGNI_MSG_SIZE - data->expected_rx])) == 0) {
				data->expected_rx -= sizeof(char);
			}

			if (data->expected_rx == 0) {
				k_fifo_put(&data->in_fifo, data->rdata);
				data->rdata = NULL;
			}
		} else {
			char c = '\0';
			
			while (uart_poll_in(cfg->uart, &c) == 0) {
				if (c == UR_MAGNI_MSG_HDR) break;
			}
			
				
			if (c == UR_MAGNI_MSG_HDR) {
				// we flushed to the start of the next msg
				atomic_set(&data->flush_req, false);
				LOG_DBG("Flushed");
				// reserve a slot, if we don't have one already
				if (data->expected_rx == 0) {
					ret = k_mem_slab_alloc(&data->slab, (void**) &data->rdata,
							K_NO_WAIT);
					__ASSERT(ret == 0, "Out of memory!\n");
				}
				// overwrite any old/broken msg contents with the new/correct one
				data->expected_rx = UR_MAGNI_MSG_SIZE * sizeof(char);
				data->expected_rx -= sizeof(char);
				data->rdata->msg[0] = UR_MAGNI_MSG_HDR;
			}
		}

		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif
	}
}

static void ur_magni_uart_tx_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ur_magni_data *data = dev->data;
	const struct ur_magni_config *cfg = dev->config;

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif

	//FIXME: Watchdogs in Zephyr are limited to millisecond accuracy, therefore we have no watchdog for this thread
	for (;;) {
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->uart_tx_timer);
		#else
		k_timer_status_sync(&data->uart_tx_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->uart_tx_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->uart_tx_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->uart_tx_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */

		// Transfer
		if (data->expected_tx == 0) {
			data->tdata = k_fifo_get(&data->out_fifo, K_NO_WAIT);
			if (data->tdata) {
				data->expected_tx = UR_MAGNI_MSG_SIZE * sizeof(char);
			}
		}
			
		if (data->expected_tx > 0) {
			// Unlike poll_in, poll_out is blocking and therefore we send one byte per job at most
			uart_poll_out(cfg->uart, data->tdata->msg[UR_MAGNI_MSG_SIZE - data->expected_tx]);

			data->expected_tx -= sizeof(char);
			if (data->expected_tx == 0 && data->tdata != NULL) {
				k_mem_slab_free(&data->slab,
					(void*) data->tdata);
				data->tdata = NULL;
			}
			#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
			deadline_miss = sys_timepoint_expired(deadline_tp);
			#endif
		} else {
			#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
			deadline_miss = false;
			#endif
			k_thread_suspend(data->uart_tx_tid);
		}
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int ur_magni_uart_init(const struct device *dev)
{
	const struct ur_magni_config *cfg = dev->config;
	int ret = 0;

	if (cfg->uart == NULL) {
		LOG_ERR("UART device not found!");
		return -ENXIO;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_rx_disable(cfg->uart);
	uart_irq_tx_disable(cfg->uart);
	
	uart_irq_callback_user_data_set(cfg->uart,
			ur_magni_uart_callback, (void*) dev);
	// Only activate TX when there's data, otherwise system will hang
	uart_irq_rx_enable(cfg->uart);
#else
	struct ur_magni_data *data = dev->data;

	k_timer_init(&data->uart_rx_timer, NULL, NULL);
	uint64_t period_ticks = k_ns_to_ticks_floor64(UR_MAGNI_NSEC_PER_BYTE / 2);
	k_timer_start(&data->uart_rx_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));

	data->uart_rx_tid = k_thread_create(&data->uart_rx_thread,
			data->uart_rx_stack,
			K_KERNEL_STACK_SIZEOF(data->uart_rx_stack),
			ur_magni_uart_rx_thread, (void*) dev, NULL, NULL,
			UR_MAGNI_UART_RX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->uart_rx_tid, "ur_magni_uart_rx");
	#endif /* CONFIG_THREAD_NAME */

	k_timer_init(&data->uart_tx_timer, NULL, NULL);
	period_ticks = k_ns_to_ticks_floor64(UR_MAGNI_NSEC_PER_BYTE);
	k_timer_start(&data->uart_tx_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));

	data->uart_tx_tid = k_thread_create(&data->uart_tx_thread,
			data->uart_tx_stack,
			K_KERNEL_STACK_SIZEOF(data->uart_tx_stack),
			ur_magni_uart_tx_thread, (void*) dev, NULL, NULL,
			UR_MAGNI_UART_TX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->uart_tx_tid, "ur_magni_uart_tx");
	#endif /* CONFIG_THREAD_NAME */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	LOG_INF("Initialized UART connection.");
	return ret;
}

static int ur_magni_msg_parse(const struct device *dev, const char* const msg)
{
	struct ur_magni_data *data = dev->data;

	int ret = ur_magni_msg_valid(msg);
	if (!ret) return -EBADMSG;

	uint8_t type = msg[1] & 0x0F;
	uint8_t reg = msg[2];
	int32_t val = 0;
	int16_t odom_l, odom_r, error_l, error_r, pwm_l, pwm_r;
	uint16_t tic_int_l, tic_int_r;
	int64_t big_val;

	ret = ur_magni_msg_get_data(msg, &val);
	if (ret != 0) return ret;

	ret = 0;
	if (type == UR_MAGNI_MSG_RESPONSE) {
		switch (reg) {
		case UR_MAGNI_REG_SYS_EVENTS:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received SYS_EVENTS");
			if ((val & UR_MAGNI_SYS_EVENT_POWER_ON) != 0) {
				data->live_diags.sys_events = val;
			}
			break;
		case UR_MAGNI_REG_FW_VER:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received FW_VER");
			data->live_diags.fw_ver = val;
			break;
		case UR_MAGNI_REG_FW_DATE:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received FW_DATE");
			data->live_diags.fw_date = val;
			break;
		case UR_MAGNI_REG_ODOM_BOTH:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received ODOM_BOTH");
			/* ODOM messages tell us how far the wheels have rotated
			 * messages are incremental and should not be lost */
			odom_l = (val >> 16) & 0xffff;
			odom_r = val & 0xffff;

			//TODO: add support for high precision encoders

			//FIXME: * and / order and overflow likelyhood?
			big_val = odom_l * 1000000;
			big_val *= 1000000;
			big_val /= sensor_value_to_micro(&ur_magni_ticks_per_rad);
			big_val += sensor_value_to_micro(&data->live_diags.joint_left.pos);
			sensor_value_from_micro(&data->live_diags.joint_left.pos, big_val);

			big_val = odom_r * 1000000;
			big_val *= 1000000;
			big_val /= sensor_value_to_micro(&ur_magni_ticks_per_rad);
			big_val += sensor_value_to_micro(&data->live_diags.joint_right.pos);
			sensor_value_from_micro(&data->live_diags.joint_right.pos, big_val);

			data->live_diags.odom_updates++;
			break;
		case UR_MAGNI_REG_PID_ERROR_BOTH:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received PID_ERROR_BOTH");
			// PID Errors means deviation from target, not a fault
			error_l = (val >> 16) & 0xffff;
			error_r = val & 0xffff;
			data->live_diags.pid_error_left = error_l;
			data->live_diags.pid_error_right = error_r;
			break;
		case UR_MAGNI_REG_PWM_BOTH:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received PWM_BOTH");
			pwm_l = (val >> 16) & 0xffff;
			pwm_r = val & 0xffff;
			data->live_diags.pwm_left = pwm_l;
			data->live_diags.pwm_right = pwm_r;
			break;
		case UR_MAGNI_REG_MOTOR_CURRENT_L:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received MOTOR_CURRENT_L");
			//FIXME: on the motor amps calculations, the offset is removed before the multiplier, but the batt voltage are the other way around?
			big_val = (val & 0xffff) * 1000000;
			big_val -= sensor_value_to_micro(&data->parameters.amps_offset);
			big_val *= sensor_value_to_micro(&data->parameters.amps_multiplier);
			big_val /= 1000000;
			sensor_value_from_micro(&data->live_diags.amps_left, big_val);
			
			break;
		case UR_MAGNI_REG_MOTOR_CURRENT_R:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received MOTOR_CURRENT_R");
			big_val = (val & 0xffff) * 1000000;
			big_val -= sensor_value_to_micro(&data->parameters.amps_offset);
			big_val *= sensor_value_to_micro(&data->parameters.amps_multiplier);
			big_val /= 1000000;
			sensor_value_from_micro(&data->live_diags.amps_right, big_val);
			
			break;
		case UR_MAGNI_REG_HW_OPTIONS:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received HW_OPTIONS");
			//FIXME is this the correct assignment?
			//names come from original source
			data->live_diags.fw_options = val;
			// TODO change encoder value, wheel type and direction
			break;
		case UR_MAGNI_REG_LIMIT_REACHED:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Got informed about limit break");
			// Limits are reset when the application fetches them
			if (val & UR_MAGNI_LIMIT_MOTOR_PWM_L) {
				data->live_diags.limit_pwm_left = 1;
			}
			if (val & UR_MAGNI_LIMIT_MOTOR_PWM_R) {
				data->live_diags.limit_pwm_right = 1;
			}
			if (val & UR_MAGNI_LIMIT_MOTOR_INT_L) {
				data->live_diags.limit_int_left = 1;
			}
			if (val & UR_MAGNI_LIMIT_MOTOR_INT_R) {
				data->live_diags.limit_int_right = 1;
			}
			if (val & UR_MAGNI_LIMIT_MOTOR_SPEED_L) {
				data->live_diags.limit_vel_left = 1;
			}
			if (val & UR_MAGNI_LIMIT_MOTOR_SPEED_R) {
				data->live_diags.limit_vel_right = 1;
			}
			if (val & UR_MAGNI_LIMIT_PARAM) {
				data->live_diags.limit_param = 1;
			}
			break;
		case UR_MAGNI_REG_BATT_VOLT:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received BATT_VOLT");
			//FIXME: on the motor amps calculations, the offset is removed before the multiplier, but the batt voltage are the other way around?
			//TODO act on low and critical voltage levels
				big_val = val * 1000000;
				big_val *= sensor_value_to_micro(&data->parameters.batt_multiplier);
				big_val /= 1000000;
				big_val += sensor_value_to_micro(&data->parameters.batt_offset);
				sensor_value_from_micro(&data->live_diags.batt_v, big_val);

			if (big_val < sensor_value_to_micro(&data->parameters.batt_low_lvl)) {
				LOG_WRN("Battery is low.");
				data->live_diags.batt_low = 1;
			}
			if (big_val < sensor_value_to_micro(&data->parameters.batt_crit_lvl)) {
				LOG_ERR("Battery is critical!");
				data->live_diags.batt_crit = 1;
			}
			break;
		case UR_MAGNI_REG_MOTOR_POWER:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received MOTOR_POWER");
			//NOTE: this only works for board versions 50 and beyond
			if (val) {
				data->live_diags.estop_motor_off = 0;
			} else {
				LOG_WRN("Estop is activated.");
				data->live_diags.estop_motor_off = 1;
			}
			break;
		case UR_MAGNI_REG_TIC_INT_BOTH:
			LOG_HEXDUMP_DBG(&val, sizeof(int32_t), "Received TIC_INT_BOTH");
			//NOTE: This does not work on older Firmware versions
			tic_int_l = (val >> 16) & 0xffff;
			tic_int_r = val & 0xffff;
			data->live_diags.enc_tic_int_left = tic_int_l;
			data->live_diags.enc_tic_int_right = tic_int_r;
			break;
		case UR_MAGNI_REG_5V_MAIN_CURRENT:
		case UR_MAGNI_REG_MAIN_VOLT_TEST:
		case UR_MAGNI_REG_12V_MAIN_CURRENT:
		case UR_MAGNI_REG_AUX_VOLT_TEST:
			LOG_DBG("Received MAIN_CURRENT or VOLT_TEST");
			/* NOTE: these are reported frequently, but they seem
			 * like unfinished features.
			 */
			break;
		default:
			LOG_WRN("unhandled response %x", reg);
			break;
		}
	} else if (type == UR_MAGNI_MSG_ERROR) {
		switch (val) {
		case UR_MAGNI_ERROR_NONE:
			LOG_ERR("Received NONE error.");
			break;
		case UR_MAGNI_ERROR_DELIMETER:
			LOG_ERR("Received DELIMETER error.");
			break;
		case UR_MAGNI_ERROR_WRONG_PROTOCOL:
			LOG_ERR("Received WRONG PROTOCOL error.");
			break;
		case UR_MAGNI_ERROR_BAD_CHKSUM:
			LOG_ERR("Received BAD CHECKSUM error.");
			break;
		case UR_MAGNI_ERROR_BAD_TYPE:
			LOG_ERR("Received BAD TYPE error.");
			break;
		case UR_MAGNI_ERROR_UNKNOWN_REG:
			LOG_ERR("Received UNKNOWN REGISTER error.");
			break;
		default:
			LOG_ERR("Received unknown error code %d.", val);
			break;
		}
		ret = -EBADMSG;
	}
	return ret;
}

static void ur_magni_parser_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ur_magni_data *data = dev->data;

	char in_msg[8];
	struct ur_magni_fifo_item_t *e = NULL;
	int ret = 0;
	
	/*timing_t start_time, end_time;
	uint64_t total_cycles, total_ns;
	
	timing_init();
	timing_start();
	start_time = timing_counter_get();*/

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&data->parser_timer);
	atomic_set(&data->parser_watchdog, task_wdt_add(UR_MAGNI_PARSER_MSEC, ur_magni_watchdog_callback, data));
	task_wdt_feed(atomic_get(&data->parser_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */
	
	for (;;) {
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->parser_timer);
		#else
		k_timer_status_sync(&data->parser_timer);
		#endif


		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->parser_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->parser_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->parser_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&data->parser_timer);
		#endif /* CONFIG_WATCHDOG */

		//k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&data->parser_timer)));
		
		if (!k_fifo_is_empty(&data->in_fifo))  {
			e = k_fifo_get(&data->in_fifo, sys_timepoint_timeout(deadline_tp));

			if (e != NULL) {
				LOG_DBG("Received MSG");
				for (size_t i = 0; i < UR_MAGNI_MSG_SIZE; i++) {
					in_msg[i] = e->msg[i];
				}
				k_mem_slab_free(&data->slab, (void*) e);

				if (k_mutex_lock(&data->live_diags_mut,
					sys_timepoint_timeout(deadline_tp)) == 0) {
				
					ret = ur_magni_msg_parse(dev, in_msg);
					k_mutex_unlock(&data->live_diags_mut);
				
					if (ret == -EBADMSG) {
						/* potential data corruption*/
						/*LOG_HEXDUMP_WRN(in_msg,
							UR_MAGNI_MSG_SIZE, "Received broken message"); */
						atomic_set(&data->flush_req, true);
						LOG_WRN("Received broken message, flushing UART in FIFO.");
					}
				} else {
					LOG_ERR("Parser thread failed on mutex lock");
				}
			} else {
				LOG_DBG("No message to parse");
			}
		}

		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif

		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&data->parser_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("Parser deadline");
		}
		#endif /* CONFIG_WATCHDOG */
	}
}

static int ur_magni_uart_transfer(const struct device *dev,
	const char* const msg, k_timeout_t timeout)
{
	struct ur_magni_data *data = dev->data;
	int ret;
	
	struct ur_magni_fifo_item_t *tdata;
	ret = k_mem_slab_alloc(&data->slab, (void**) &tdata, timeout);
	__ASSERT(ret == 0, "Out of memory!\n");
	
	for (size_t i = 0; i < UR_MAGNI_MSG_SIZE; i++) {
		tdata->msg[i] = msg[i];
	}
	
	LOG_HEXDUMP_DBG(msg, UR_MAGNI_MSG_SIZE, "Transfering message to Magni");
	k_fifo_put(&data->out_fifo, tdata);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	const struct ur_magni_config *cfg = dev->config;
	uart_irq_tx_enable(cfg->uart);
#else
	k_thread_resume(data->uart_tx_tid);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int ur_magni_pid_full_update(const struct device *dev,
		int p_val, int i_val, int d_val,
		int v_val, int c_val, int buf_size, int max_pwm, k_timeout_t timeout)
{
	// FIXME: Older FW versions don't support all settings
	// Live updating PID (and adjacent) parameters is untested and risky

	int ret = 0;
	char msg[8];

	LOG_INF("P: %d", p_val);
	if(ur_magni_write_p(msg, p_val)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	LOG_INF("I: %d", i_val);
	if(ur_magni_write_i(msg, i_val)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;


	LOG_INF("D: %d", d_val);
	if(ur_magni_write_d(msg, d_val)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;


	LOG_INF("V: %d", v_val);
	if(ur_magni_write_v(msg, v_val)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;


	LOG_INF("C: %d", c_val);
	if(ur_magni_write_c(msg, c_val)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;


	LOG_INF("bufsize: %d", buf_size);
	if(ur_magni_write_bufsize(msg, buf_size)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;


	LOG_INF("maxpwm: %d", max_pwm);
	if(ur_magni_write_maxpwm(msg, max_pwm)) return -EINVAL;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	return 0;
}

//FIXME: This waits on a mutex, but is only used internally
static int ur_magni_controller_init(const struct device *dev, k_timeout_t timeout)
{
	struct ur_magni_data *data = dev->data;
	int ret = 0;
	char msg[8];
	//TODO: Add Support for thin wheels
	LOG_INF("Setting wheel type to: std");
	if (ur_magni_write_wheeltype(msg, UR_MAGNI_HW_OPTION_STD_WHEEL)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;
	
	//TODO: Add support for 4-wheel drive type
	//FIXME: Technically this is only for firmware v42 and above
	//However, the original software sets it for every version during boot
	LOG_INF("Setting drive type to: std");
	if (ur_magni_write_drivetype(msg, UR_MAGNI_HW_OPTION_STD_DRIVE)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	//TODO: Add Support for reverse direction
	LOG_INF("Setting wheel direction to: std");
	if (ur_magni_write_wheeldir(msg, UR_MAGNI_HW_OPTION_STD_DIR)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	//TODO: Read board version, instead of hardcoding it
	// The original source also doesn't do this, not sure it's possible yet
	LOG_INF("Setting board version to: %d", data->parameters.board_ver);
	if(ur_magni_write_hwver(msg, data->parameters.board_ver)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	LOG_INF("Setting option switch to: %d", data->parameters.option_switch);
	if (ur_magni_write_optionswitch(msg, data->parameters.option_switch)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	LOG_INF("Clearing system events.");
	if(ur_magni_write_sysevents(msg, 0)) return -EBADMSG;
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	if (k_mutex_lock(&data->live_diags_mut, timeout) == 0) {
		data->live_diags.sys_events = 0;
		k_mutex_unlock(&data->live_diags_mut);
	} else {
		LOG_ERR("Could not lock live diags mutex for initialization");
	}

	LOG_INF("Initializing E-Stop state to: %d", data->parameters.estop_detect);
	if(ur_magni_write_estopstate(msg, data->parameters.estop_detect)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	LOG_INF("Initializing E-Stop Threshold to: %d", data->parameters.estop_pid_threshold);
	if(ur_magni_write_estoplevel(msg, data->parameters.estop_pid_threshold))
	{
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	LOG_INF("Setting speed limitations to: %d and %d", data->parameters.max_vel_fwd, data->parameters.max_vel_rev);
	if(ur_magni_write_maxfwdvel(msg, data->parameters.max_vel_fwd)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	if(ur_magni_write_maxrevvel(msg, data->parameters.max_vel_rev)) {
		return -EBADMSG;
	}
	ret = ur_magni_uart_transfer(dev, msg, timeout);
	if (ret != 0) return ret;

	LOG_INF("Initializing PID values.");
	if(ur_magni_pid_full_update(
			dev,
			data->parameters.pid_p,
			data->parameters.pid_i,
			data->parameters.pid_d,
			data->parameters.pid_v,
			data->parameters.pid_c,
			data->parameters.pid_buf_size,
			data->parameters.max_pwm, timeout)) return -EBADMSG;


	LOG_INF("Achieved full initialization.");

	return 0;
}

static void ur_magni_system_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ur_magni_data *data = dev->data;

	char battery[8], sysevents[8], setwheel[8], setdrive[8];
	ur_magni_read(battery, UR_MAGNI_REG_BATT_VOLT);
	ur_magni_read(sysevents, UR_MAGNI_REG_SYS_EVENTS);
	ur_magni_write_wheeltype(setwheel, UR_MAGNI_HW_OPTION_STD_WHEEL);
	ur_magni_write_drivetype(setdrive, UR_MAGNI_HW_OPTION_STD_DRIVE);

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif

	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&data->system_timer);
	atomic_set(&data->system_watchdog, task_wdt_add(UR_MAGNI_SYSTEM_MAINTENANCE_MSEC, ur_magni_watchdog_callback, data));
	task_wdt_feed(atomic_get(&data->system_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */
	
	/*timing_t start_time, end_time;
	uint64_t total_cycles, total_ns;
	
	timing_init();
	timing_start();
	start_time = timing_counter_get();*/

	for (;;) {
		/*end_time = timing_counter_get();
		timing_stop();
		total_cycles = timing_cycles_get(&start_time, &end_time);
		total_ns = timing_cycles_to_ns(total_cycles);
		LOG_INF("System Thread Execution Time: %llu ns", total_ns);*/
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->system_timer);
		#else
		k_timer_status_sync(&data->system_timer);
		#endif
		
		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->system_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->system_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->system_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&data->system_timer);
		#endif /* CONFIG_WATCHDOG */
		
		//k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&data->system_timer)));
		
		/*timing_start();
		start_time = timing_counter_get();*/

		/* check for reboot event (power fluctuation or error) */
		if (k_mutex_lock(&data->live_diags_mut,
				sys_timepoint_timeout(deadline_tp)) == 0) {
			if ((data->live_diags.sys_events
					& UR_MAGNI_SYS_EVENT_POWER_ON) != 0) {
				data->live_diags.sys_events = 0;
				k_mutex_unlock(&data->live_diags_mut);
				LOG_WRN("magni suddenly reset!");
				ur_magni_controller_init(dev, sys_timepoint_timeout(deadline_tp));
			} else {
				// nothing to do
				k_mutex_unlock(&data->live_diags_mut);
			}
		} else {
			LOG_WRN("System thread failed on mutex lock");
		}

		/* Request updates for battery and system events */
		ur_magni_uart_transfer(dev, battery, sys_timepoint_timeout(deadline_tp));
		ur_magni_uart_transfer(dev, sysevents, sys_timepoint_timeout(deadline_tp));
		/* For safety reasons we ensure that wheeltype and drivetype are
		 * set correctly. An incorrect setting could be catastrophic.
		 * Drivetype register has no functionality on older FWs. */
		ur_magni_uart_transfer(dev, setwheel, sys_timepoint_timeout(deadline_tp));
		
		if (k_mutex_lock(&data->live_diags_mut,
				sys_timepoint_timeout(deadline_tp)) == 0) {
			if (data->live_diags.fw_ver >= 42) {
				k_mutex_unlock(&data->live_diags_mut);
				ur_magni_uart_transfer(dev, setdrive, sys_timepoint_timeout(deadline_tp));
			} else {
				// nothing to do
				k_mutex_unlock(&data->live_diags_mut);
			}
		} else {
			LOG_WRN("System thread failed on mutex lock");
		}

		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif

		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&data->system_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("System deadline");
		}
		#endif /* CONFIG_WATCHDOG */

		#ifdef CONFIG_THREAD_ANALYZER
		thread_analyzer_print(0);
		#endif /* CONFIG_THREAD_ANALYZER */
	}
}

static void ur_magni_vel_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ur_magni_data *data = dev->data;

	char vel[8];
	/* the delay ensures that the robot won't instantly jump into action
	 * when estop is deactivated */
	uint8_t estop_delay = 0;
	struct sensor_value vel_left, vel_right;

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&data->vel_timer);
	atomic_set(&data->vel_watchdog, task_wdt_add(UR_MAGNI_VELOCITY_UPDATE_MSEC, ur_magni_watchdog_callback, data));
	task_wdt_feed(atomic_get(&data->vel_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */
	
	/*timing_t start_time, end_time;
	uint64_t total_cycles, total_ns;
	
	timing_init();
	timing_start();
	start_time = timing_counter_get();*/

	for (;;) {
		/*end_time = timing_counter_get();
		timing_stop();
		total_cycles = timing_cycles_get(&start_time, &end_time);
		total_ns = timing_cycles_to_ns(total_cycles);
		LOG_INF("Velocity Thread Execution Time: %llu ns", total_ns);*/
		
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->vel_timer);
		#else
		k_timer_status_sync(&data->vel_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->vel_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->vel_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->vel_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&data->vel_timer);
		#endif /* CONFIG_WATCHDOG */
		
	       // k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&data->vel_timer)));
		/*timing_start();
		start_time = timing_counter_get();*/
		
		if (k_mutex_lock(&data->live_diags_mut,
				sys_timepoint_timeout(deadline_tp)) == 0) {
			if (data->live_diags.estop_motor_off) {
				k_mutex_unlock(&data->live_diags_mut);
				
				/* retain internal velocity state,
				 * but reset motor control.
				 */
				//LOG_INF("Estop was activated");
				vel_left.val1 = 0;
				vel_left.val2 = 0;
				vel_right.val1 = 0;
				vel_right.val2 = 0;
				ur_magni_write_vel(vel, &vel_left,
							&vel_right);
							
				ur_magni_uart_transfer(dev, vel, sys_timepoint_timeout(deadline_tp));
				estop_delay = 8;
			} else {
				// make copies, so we can unlock early
				vel_left =
				data->live_diags.joint_left.vel_desired;
				vel_right =
				data->live_diags.joint_right.vel_desired;
				k_mutex_unlock(&data->live_diags_mut);
				
				if (estop_delay == 0) {
					ur_magni_write_vel(vel, &vel_left,
								&vel_right);
					
					ur_magni_uart_transfer(dev, vel, sys_timepoint_timeout(deadline_tp));

				} else {
					estop_delay -= 1;
					vel_left.val1 = 0;
					vel_left.val2 = 0;
					vel_right.val1 = 0;
					vel_right.val2 = 0;
					ur_magni_write_vel(vel, &vel_left,
							&vel_right);
		
					ur_magni_uart_transfer(dev, vel, sys_timepoint_timeout(deadline_tp));
				}
			}
		} else {
			LOG_ERR("Couldn't aquire Mutex to read estop state");
		}
		
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif

		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&data->vel_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("Velocity deadline");
		}
		#endif /* CONFIG_WATCHDOG */
	}
}

static int ur_magni_set_actual_velocity(const struct device *dev,
		struct sensor_value *vel_left, struct sensor_value *vel_right,
		k_timeout_t timeout)
{
	struct ur_magni_data *data = dev->data;

	/* Only used to store the measured actual velocity of the motors */
	if (k_mutex_lock(&data->live_diags_mut, timeout) == 0) {
		data->live_diags.joint_left.vel_actual = *vel_left;
		data->live_diags.joint_right.vel_actual = *vel_right;
		k_mutex_unlock(&data->live_diags_mut);
		return 0;
	}
	return -ETIMEDOUT;
}

static void ur_magni_joint_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct ur_magni_data *data = dev->data;
	struct sensor_value prev_pos_left, prev_pos_right, vel_left, vel_right,
				pos_left, pos_right, elapsed_secs,
				vel_desired_left, vel_desired_right;
	int64_t big_vel_desired_left, big_vel_desired_right, big_wheelslip_thresh,
			big_wheelslip_null_period, big_val, last_joint_update,
			zero_vel_duration = 0;
	char nullerror_msg[8];
	bool standstill_left, standstill_right;
	
	ur_magni_write_nullerror(nullerror_msg);

	pos_left.val1 = 0;
	pos_left.val2 = 0;
	pos_right.val1 = 0;
	pos_right.val2 = 0;
	
	big_wheelslip_thresh = sensor_value_to_micro(&data->parameters.wheel_slip_threshold);
	big_wheelslip_null_period = sensor_value_to_micro(&data->parameters.wheel_slip_nulling_period);
	
	/* We allow waiting as this is initialization */
	uint64_t period_ticks = k_ms_to_ticks_near64(UR_MAGNI_JOINT_UPDATE_MSEC);
	if (k_mutex_lock(&data->live_diags_mut,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks))) == 0) {
		prev_pos_left = data->live_diags.joint_left.pos;
		prev_pos_right = data->live_diags.joint_right.pos;
		k_mutex_unlock(&data->live_diags_mut);
	} else {
		LOG_ERR("Can't get initial position to start joint thread.");
		k_oops();
		return;

	}

	#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	#ifdef CONFIG_WATCHDOG
	k_timer_status_sync(&data->joint_timer);
	atomic_set(&data->joint_watchdog, task_wdt_add(UR_MAGNI_JOINT_UPDATE_MSEC, ur_magni_watchdog_callback, data));
	task_wdt_feed(atomic_get(&data->joint_watchdog));
	#else
	int64_t deadline;
	#endif /* CONFIG_WATCHDOG */

	last_joint_update = k_uptime_get();
	
	/*timing_t start_time, end_time;
	uint64_t total_cycles, total_ns;
	
	timing_init();
	timing_start();
	start_time = timing_counter_get();*/

	for (;;) {
		/*end_time = timing_counter_get();
		timing_stop();
		total_cycles = timing_cycles_get(&start_time, &end_time);
		total_ns = timing_cycles_to_ns(total_cycles);
		LOG_INF("Odometrie Thread Execution Time: %llu ns", total_ns);*/
		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->joint_timer);
		#else
		k_timer_status_sync(&data->joint_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->joint_timer)));
		
		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->joint_tid, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->joint_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */
		
		#ifndef CONFIG_WATCHDOG
		deadline = k_uptime_get() + k_timer_remaining_get(&data->joint_timer);
		#endif /* CONFIG_WATCHDOG */
		
		//k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&data->joint_timer)));
		
		/*timing_start();
		start_time = timing_counter_get();*/

		vel_left.val1 = 0;
		vel_left.val2 = 0;
		vel_right.val1 = 0;
		vel_right.val2 = 0;
		
		if (k_mutex_lock(&data->live_diags_mut,
				sys_timepoint_timeout(deadline_tp)) == 0) {
			pos_left = data->live_diags.joint_left.pos;
			pos_right = data->live_diags.joint_right.pos;
			vel_desired_left = data->live_diags.joint_left.vel_desired;
			vel_desired_right = data->live_diags.joint_right.vel_desired;
			k_mutex_unlock(&data->live_diags_mut);
			
			// calculate actual velocity based on position change over time
			big_val = (k_uptime_get() - last_joint_update)
					* 1000000 / 1000;
			sensor_value_from_micro(&elapsed_secs, big_val);
			last_joint_update = k_uptime_get();

			if (sensor_value_to_micro(&elapsed_secs) != 0) {
				big_val = ((sensor_value_to_micro(&pos_left)
				- sensor_value_to_micro(&prev_pos_left))
				* 1000000)
				/ sensor_value_to_micro(&elapsed_secs);
				sensor_value_from_micro(&vel_left, big_val);

				big_val = ((sensor_value_to_micro(&pos_right)
				- sensor_value_to_micro(&prev_pos_right))
				* 1000000)
				/ sensor_value_to_micro(&elapsed_secs);
				sensor_value_from_micro(&vel_right, big_val);
				
				ur_magni_set_actual_velocity(dev, &vel_left, &vel_right, sys_timepoint_timeout(deadline_tp));
			} else {
				LOG_WRN("Division-by-Zero avoided: no time elapsed between two consecutive joint thread jobs!");
			}
		        
			prev_pos_left = pos_left;
			prev_pos_right = pos_right;
			
			// Check for near standstill
			big_vel_desired_left = sensor_value_to_micro(&vel_desired_left);
			big_vel_desired_right = sensor_value_to_micro(&vel_desired_right);
			
			standstill_left = llabs(big_vel_desired_left) <
						llabs(big_wheelslip_thresh);
					
			standstill_right = llabs(big_vel_desired_right) <
						llabs(big_wheelslip_thresh);
			
			if (standstill_left && standstill_right) {
				zero_vel_duration += UR_MAGNI_JOINT_UPDATE_MSEC;
				// Silence motors when standing still for longer
				if (zero_vel_duration > big_wheelslip_null_period) {
					ur_magni_uart_transfer(dev, nullerror_msg, sys_timepoint_timeout(deadline_tp));
					zero_vel_duration = 0;
				}
			} else {
				zero_vel_duration = 0;
			}
		} else {
			LOG_WRN("Failed to update joints!");
		}

		#ifdef CONFIG_UR_MAGNI_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif

		#ifdef CONFIG_WATCHDOG
		task_wdt_feed(atomic_get(&data->joint_watchdog));
		#else
		if (deadline < k_uptime_get()) {
			LOG_DBG("Joint deadline");
		}
		#endif /* CONFIG_WATCHDOG */
	}
}

static int ur_magni_init(const struct device* dev)
{
	struct ur_magni_data *data = dev->data;

	const int tries = 1024;
	char msg[8];
	int i, ret;
	uint8_t option_switch = 0;

	data->live_diags = ur_magni_diagnostics_initial;
	data->snap_diags = ur_magni_diagnostics_initial;
	data->parameters = ur_magni_parameters_standard;

	LOG_DBG("size: %i", sizeof(struct ur_magni_fifo_item_t));

	ret = k_mem_slab_init(&data->slab, data->buf,
		sizeof(struct ur_magni_fifo_item_t), UR_MAGNI_SLAB_BLOCKS);
	__ASSERT(ret == 0, "Can't allocate memory for slab.");

	k_fifo_init(&data->in_fifo);
	k_fifo_init(&data->out_fifo);

	ret = k_mutex_init(&data->live_diags_mut);
	__ASSERT(ret == 0, "Can't initialize mutex.");

	k_timer_init(&data->parser_timer, NULL, NULL);
	k_timer_init(&data->joint_timer, NULL, NULL);
	k_timer_init(&data->vel_timer, NULL, NULL);
	k_timer_init(&data->system_timer, NULL, NULL);
	/* timers and therefore periodic task activation is intended to be in sync.
	 * however jitter can never be fully eliminated! */
	uint64_t period_ticks = k_ms_to_ticks_near64(UR_MAGNI_PARSER_MSEC);
	k_timer_start(&data->parser_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));
	period_ticks = k_ms_to_ticks_near64(UR_MAGNI_JOINT_UPDATE_MSEC);
	k_timer_start(&data->joint_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));
	period_ticks = k_ms_to_ticks_near64(UR_MAGNI_VELOCITY_UPDATE_MSEC);
	k_timer_start(&data->vel_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));
	period_ticks = k_ms_to_ticks_near64(UR_MAGNI_SYSTEM_MAINTENANCE_MSEC);
	k_timer_start(&data->system_timer,
			K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)),
			K_TICKS(period_ticks));

#ifdef CONFIG_UR_MAGNI_EXPANDER
	ret = ur_magni_i2c_init(dev);
	__ASSERT(ret == 0, "Can't initialize I2C connection.");
	ret = ur_magni_gpio_expander_read(dev, &option_switch);
	__ASSERT(ret == 0, "Can't access I2C GPIO expander.");
	LOG_HEXDUMP_INF(&option_switch, 1, "Got Magni HW options");
#else
	LOG_INF("No I2C connection provided, setting option switch to default");
	option_switch = 0x02;
#endif /* CONFIG_UR_MAGNI_EXPANDER */

	data->expected_rx = 0;
	data->expected_tx = 0;
	data->rdata = NULL;
	data->tdata = NULL;

	atomic_set(&data->flush_req, true);
	ret = ur_magni_uart_init(dev);
	__ASSERT(ret == 0, "Can't initialize UART connection.");
	/* We allow waiting in this fuction as it is initialization */
	/* magni can take some time to warm up */
	/* FIXME: always fails to connect on cold-boot and requires soft reset */
	for (i = 0; i < tries; i++) {
		if (!atomic_get(&data->flush_req)) {
			break;
		}
		k_sleep(K_NSEC(UR_MAGNI_NSEC_PER_MSG));
	}
	__ASSERT(ret == 0, "Can't flush UART connection initially.");

	data->parameters.option_switch = option_switch;
	
#ifdef CONFIG_WATCHDOG		
	const struct device *const hw_wdt_dev = DEVICE_DT_GET_OR_NULL(WDT_NODE);
	ret = task_wdt_init(hw_wdt_dev);
	__ASSERT(ret == 0, "Can't initialize hardware watchdog timer");
	
	/* marks all watchdog channels as uninitialized */
	atomic_set(&data->parser_watchdog, -1);
	atomic_set(&data->system_watchdog, -1);
	atomic_set(&data->vel_watchdog, -1);
	atomic_set(&data->joint_watchdog, -1);
#endif /* CONFIG_WATCHDOG */

	data->parser_tid = k_thread_create(&data->parser_thread,
			data->parser_stack,
			K_KERNEL_STACK_SIZEOF(data->parser_stack),
			ur_magni_parser_thread, (void*) dev, NULL, NULL,
			UR_MAGNI_PARSER_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->parser_tid, "ur_magni_parser");
	#endif /* CONFIG_THREAD_NAME */
	
	// Request FW Version
	LOG_INF("Requesting firmware version.");
	ret = ur_magni_read(msg, UR_MAGNI_REG_FW_VER);
	__ASSERT(ret == 0, "Can't build FW request message for setup.");
	ret = ur_magni_uart_transfer(dev, msg, K_FOREVER);
	__ASSERT(ret == 0, "Can't transfer FW request for setup.");

	for (i = 0; i < tries; i++) {
		if (k_mutex_lock(&data->live_diags_mut,
				K_NSEC(UR_MAGNI_NSEC_PER_MSG)) == 0) {
			if (data->live_diags.fw_ver !=
					ur_magni_diagnostics_initial.fw_ver) {
				LOG_INF("Got Magni FW version: %i", data->live_diags.fw_ver);
				k_mutex_unlock(&data->live_diags_mut);
				break;
			}
			k_mutex_unlock(&data->live_diags_mut);
		}
		k_sleep(K_NSEC(UR_MAGNI_NSEC_PER_MSG));
	}
	__ASSERT(i != tries, "Couldn't read out Magni's FW version!");

	// Request FW build date
	LOG_INF("Requesting firmware date.");
	ret = ur_magni_read(msg, UR_MAGNI_REG_FW_DATE);
	__ASSERT(ret == 0, "Can't build FW date request message for setup.");
	ret = ur_magni_uart_transfer(dev, msg, K_FOREVER);
	__ASSERT(ret == 0, "Can't transfer FW date request for setup.");

	for (i = 0; i < tries; i++) {
		if (k_mutex_lock(&data->live_diags_mut,
				K_NSEC(UR_MAGNI_NSEC_PER_MSG)) == 0) {
			if (data->live_diags.fw_date !=
					ur_magni_diagnostics_initial.fw_date) {
				LOG_INF("Got Magni FW date: %i", data->live_diags.fw_date);
				k_mutex_unlock(&data->live_diags_mut);
				break;
			}
			k_mutex_unlock(&data->live_diags_mut);
		}
		k_sleep(K_NSEC(UR_MAGNI_NSEC_PER_MSG));
	}
	__ASSERT(i != tries, "Couldn't read out FW build date for setup.");

	ret = ur_magni_controller_init(dev, K_FOREVER);
	__ASSERT(ret == 0, "Couldn't initialize the motor controller.");

	data->system_tid = k_thread_create(&data->system_thread,
			data->system_stack,
			K_KERNEL_STACK_SIZEOF(data->system_stack),
			ur_magni_system_thread, (void*) dev, NULL, NULL,
			UR_MAGNI_SYSTEM_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->system_tid, "ur_magni_system");
	#endif /* CONFIG_THREAD_NAME */
	data->vel_tid = k_thread_create(&data->vel_thread,
			data->vel_stack,
			K_KERNEL_STACK_SIZEOF(data->vel_stack),
			ur_magni_vel_thread, (void*) dev, NULL, NULL,
			UR_MAGNI_VELOCITY_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->vel_tid, "ur_magni_velocity");
	#endif /* CONFIG_THREAD_NAME */
	data->joint_tid = k_thread_create(&data->joint_thread,
			data->joint_stack,
			K_KERNEL_STACK_SIZEOF(data->joint_stack),
			ur_magni_joint_thread, (void*) dev, NULL, NULL,
			UR_MAGNI_JOINT_THREAD_PRIORITY, 0, K_NO_WAIT);
	#ifdef CONFIG_THREAD_NAME
	k_thread_name_set(data->joint_tid, "ur_magni_joint");
	#endif /* CONFIG_THREAD_NAME */

	return 0;
}

static int ur_magni_channel_get(const struct device *dev,
		enum sensor_channel chan, struct sensor_value *val)
{
	struct ur_magni_data *data = dev->data;
	enum ur_magni_sensor_channel ur_magni_chan = (enum ur_magni_sensor_channel) chan;
	int64_t big_val;
	int ret = 0;

	switch (ur_magni_chan) {
	case SENSOR_CHAN_UR_MAGNI_BATTERY_VOLTAGE:
		*val = data->snap_diags.batt_v;
		break;
	case SENSOR_CHAN_UR_MAGNI_PID_ERROR_LEFT:
		val->val1 = data->snap_diags.pid_error_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_PID_ERROR_RIGHT:
		val->val1 = data->snap_diags.pid_error_right;
		break;
	// TODO is there a conversion from rad to deg or viceversa missing?
	case SENSOR_CHAN_UR_MAGNI_POSITION_LEFT:
		big_val = sensor_value_to_micro(&data->snap_diags.joint_left.pos);
		big_val /= 1000000;
		big_val *= sensor_value_to_micro(&data->parameters.wheel_radius);
		sensor_value_from_micro(val, big_val);
		break;
	case SENSOR_CHAN_UR_MAGNI_POSITION_RIGHT:
		big_val = sensor_value_to_micro(&data->snap_diags.joint_right.pos);
		big_val /= 1000000;
		big_val *= sensor_value_to_micro(&data->parameters.wheel_radius);
		sensor_value_from_micro(val, big_val);
		break;
	case SENSOR_CHAN_UR_MAGNI_VELOCITY_LEFT:
		big_val = sensor_value_to_micro(&data->snap_diags.joint_left.vel_actual);
		big_val /= 1000000;
		big_val *= sensor_value_to_micro(&data->parameters.wheel_radius);
		sensor_value_from_micro(val, big_val);
		break;
	case SENSOR_CHAN_UR_MAGNI_VELOCITY_RIGHT:
		big_val = sensor_value_to_micro(&data->snap_diags.joint_right.vel_actual);
		big_val /= 1000000;
		big_val *= sensor_value_to_micro(&data->parameters.wheel_radius);
		sensor_value_from_micro(val, big_val);
		break;
	case SENSOR_CHAN_UR_MAGNI_PWM_LIMIT_LEFT:
		val->val1 = data->snap_diags.limit_pwm_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_PWM_LIMIT_RIGHT:
		val->val1 = data->snap_diags.limit_pwm_right;
		break;
	case SENSOR_CHAN_UR_MAGNI_INTEGRAL_LIMIT_LEFT:
		val->val1 = data->snap_diags.limit_int_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_INTEGRAL_LIMIT_RIGHT:
		val->val1 = data->snap_diags.limit_int_right;
		break;
	case SENSOR_CHAN_UR_MAGNI_VELOCITY_LIMIT_LEFT:
		val->val1 = data->snap_diags.limit_vel_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_VELOCITY_LIMIT_RIGHT:
		val->val1 = data->snap_diags.limit_vel_right;
		break;
	case SENSOR_CHAN_UR_MAGNI_PARAMETER_LIMIT:
		val->val1 = data->snap_diags.limit_param;
		break;
	case SENSOR_CHAN_UR_MAGNI_MOTOR_CURRENT_LEFT:
		*val = data->snap_diags.amps_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_MOTOR_CURRENT_RIGHT:
		*val = data->snap_diags.amps_right;
		break;
	case SENSOR_CHAN_UR_MAGNI_MOTOR_PWM_LEFT:
		val->val1 = data->snap_diags.pwm_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_MOTOR_PWM_RIGHT:
		val->val1 = data->snap_diags.pwm_right;
		break;
	case SENSOR_CHAN_UR_MAGNI_ENCODER_TICK_INTERVAL_LEFT:
		val->val1 = data->snap_diags.enc_tic_int_left;
		break;
	case SENSOR_CHAN_UR_MAGNI_ENCODER_TICK_INTERVAL_RIGHT:
		val->val1 = data->snap_diags.enc_tic_int_right;
		break;
	default:
		LOG_WRN("unsupported sensor channel.");
		ret = -ENOTSUP;
	}

	return ret;
}


static int ur_magni_sample_fetch_chan_with_timeout(const struct device *dev,
		enum sensor_channel chan, k_timeout_t timeout)
{
	struct ur_magni_data *data = dev->data;
	enum ur_magni_sensor_channel ur_magni_chan = (enum ur_magni_sensor_channel) chan;
	int ret = 0;
	// FIXME: timeout for waiting should be available as a parameter here
	if (k_mutex_lock(&data->live_diags_mut,
		timeout) == 0) {
		if (chan == SENSOR_CHAN_ALL) {
			data->snap_diags = data->live_diags;
			// Limits need to be reset manually
			data->live_diags.limit_pwm_left = 0;
			data->live_diags.limit_pwm_right = 0;
			data->live_diags.limit_int_left = 0;
			data->live_diags.limit_int_right = 0;
			data->live_diags.limit_vel_left = 0;
			data->live_diags.limit_vel_right = 0;
			data->live_diags.limit_param = 0;
		} else {
			switch (ur_magni_chan) {
			case SENSOR_CHAN_UR_MAGNI_BATTERY_VOLTAGE:
				data->snap_diags.batt_v =
					data->live_diags.batt_v;
				break;
			case SENSOR_CHAN_UR_MAGNI_PID_ERROR_LEFT:
				data->snap_diags.pid_error_left =
					data->live_diags.pid_error_left;
				break;
			case SENSOR_CHAN_UR_MAGNI_PID_ERROR_RIGHT:
				data->snap_diags.pid_error_right =
					data->live_diags.pid_error_right;
				break;
			case SENSOR_CHAN_UR_MAGNI_POSITION_LEFT:
				data->snap_diags.joint_left.pos =
					data->live_diags.joint_left.pos;
				break;
			case SENSOR_CHAN_UR_MAGNI_POSITION_RIGHT:
				data->snap_diags.joint_right.pos =
					data->live_diags.joint_right.pos;
				break;
			case SENSOR_CHAN_UR_MAGNI_VELOCITY_LEFT:
				data->snap_diags.joint_left.vel_actual =
					data->live_diags.joint_left.vel_actual;
				break;
			case SENSOR_CHAN_UR_MAGNI_VELOCITY_RIGHT:
				data->snap_diags.joint_right.vel_actual =
					data->live_diags.joint_right.vel_actual;
				break;
			case SENSOR_CHAN_UR_MAGNI_PWM_LIMIT_LEFT:
				data->snap_diags.limit_pwm_left =
					data->live_diags.limit_pwm_left;
				// Limits need to be reset manually
				data->live_diags.limit_pwm_left = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_PWM_LIMIT_RIGHT:
				data->snap_diags.limit_pwm_right =
					data->live_diags.limit_pwm_right;
				// Limits need to be reset manually
				data->live_diags.limit_pwm_right = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_INTEGRAL_LIMIT_LEFT:
				data->snap_diags.limit_int_left =
					data->live_diags.limit_int_left;
				// Limits need to be reset manually
				data->live_diags.limit_int_left = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_INTEGRAL_LIMIT_RIGHT:
				data->snap_diags.limit_int_right =
					data->live_diags.limit_int_right;
				// Limits need to be reset manually
				data->live_diags.limit_int_right = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_VELOCITY_LIMIT_LEFT:
				data->snap_diags.limit_vel_left =
					data->live_diags.limit_vel_left;
				// Limits need to be reset manually
				data->live_diags.limit_vel_left = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_VELOCITY_LIMIT_RIGHT:
				data->snap_diags.limit_vel_right =
					data->live_diags.limit_vel_right;
				// Limits need to be reset manually
				data->live_diags.limit_vel_right = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_PARAMETER_LIMIT:
				data->snap_diags.limit_param =
					data->live_diags.limit_param;
				// Limits need to be reset manually
				data->live_diags.limit_param = 0;
				break;
			case SENSOR_CHAN_UR_MAGNI_MOTOR_CURRENT_LEFT:
				data->snap_diags.amps_left =
					data->live_diags.amps_left;
				break;
			case SENSOR_CHAN_UR_MAGNI_MOTOR_CURRENT_RIGHT:
				data->snap_diags.amps_right =
					data->live_diags.amps_right;
				break;
			case SENSOR_CHAN_UR_MAGNI_MOTOR_PWM_LEFT:
				data->snap_diags.pwm_left =
					data->live_diags.pwm_left;
				break;
			case SENSOR_CHAN_UR_MAGNI_MOTOR_PWM_RIGHT:
				data->snap_diags.pwm_right =
					data->live_diags.pwm_right;
				break;
			case SENSOR_CHAN_UR_MAGNI_ENCODER_TICK_INTERVAL_LEFT:
				data->snap_diags.enc_tic_int_left =
					data->live_diags.enc_tic_int_left;
				break;
			case SENSOR_CHAN_UR_MAGNI_ENCODER_TICK_INTERVAL_RIGHT:
				data->snap_diags.enc_tic_int_right =
					data->live_diags.enc_tic_int_right;
				break;
			default:
				LOG_WRN("unsupported sensor channel.");
				ret = -ENOTSUP;
			}
		}
		k_mutex_unlock(&data->live_diags_mut);
	} else {
		LOG_WRN("couldn't acquire live diagnostics mutex.");
		ret = -ETIMEDOUT;
	}

	return ret;
}

static int ur_magni_sample_fetch_chan_no_wait(const struct device *dev, enum sensor_channel chan)
{
	return ur_magni_sample_fetch_chan_with_timeout(dev, chan, K_NO_WAIT);
}

int ur_magni_sample_fetch_with_timeout(const struct device *dev, k_timeout_t timeout)
{
	return ur_magni_sample_fetch_chan_with_timeout(dev, SENSOR_CHAN_ALL, timeout);
}

static const struct sensor_driver_api ur_magni_api = {
	.sample_fetch = ur_magni_sample_fetch_chan_no_wait,
	.channel_get = ur_magni_channel_get
};

int ur_magni_set_velocity(const struct device *dev,
		struct sensor_value *left_vel, struct sensor_value *right_vel,
		k_timeout_t timeout)
{
	struct sensor_value left_vel_rad, right_vel_rad;
	struct ur_magni_data *data = dev->data;
	int64_t big_val;
	
	big_val = sensor_value_to_micro(left_vel);
	big_val *= 1000000;
	big_val /= sensor_value_to_micro(&data->parameters.wheel_radius);
	sensor_value_from_micro(&left_vel_rad, big_val);
	
	big_val = sensor_value_to_micro(right_vel);
	big_val *= 1000000;
	big_val /= sensor_value_to_micro(&data->parameters.wheel_radius);
	sensor_value_from_micro(&right_vel_rad, big_val);
	
	if (k_mutex_lock(&data->live_diags_mut,
			timeout) == 0) {
		data->live_diags.joint_left.vel_desired = left_vel_rad;
		data->live_diags.joint_right.vel_desired = right_vel_rad;
		k_mutex_unlock(&data->live_diags_mut);
		return 0;
	}
	return -ETIMEDOUT;
}

/* NOTE: The following macro will be inserted into a struct definition.
 * It was setup to already contain commas at the end of each line! */
#ifdef CONFIG_UR_MAGNI_EXPANDER
#define EXPANDER_DEVICE(i)                                                     \
.i2c = DEVICE_DT_GET(DT_INST_PHANDLE(i, i2c_bus)),                             \
.expander_reg = DT_REG_ADDR(DT_CHILD(DT_INST_PHANDLE(i, i2c_bus), pcf8574_20)),
#else
#define EXPANDER_DEVICE(i)
#endif /* CONFIG_UR_MAGNI_EXPANDER */

//FIXME: replace magic number in __aligned() block, depends on fifo_item_t
#define UR_MAGNI_INIT(i)                                                       \
char __aligned(16)                                                             \
ur_magni_buf_##i[UR_MAGNI_SLAB_BLOCKS * sizeof(struct ur_magni_fifo_item_t)];  \
                                                                               \
static struct ur_magni_data ur_magni_data_##i = {                              \
	.buf = ur_magni_buf_##i,                                               \
};                                                                             \
                                                                               \
static const struct ur_magni_config ur_magni_config_##i = {                    \
	.uart = DEVICE_DT_GET(DT_INST_BUS(i)),                                 \
	EXPANDER_DEVICE(i)                                                     \
};                                                                             \
                                                                               \
DEVICE_DT_INST_DEFINE(i, &ur_magni_init, NULL, &ur_magni_data_##i,             \
	&ur_magni_config_##i, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
	&ur_magni_api);                                                        \

DT_INST_FOREACH_STATUS_OKAY(UR_MAGNI_INIT)
