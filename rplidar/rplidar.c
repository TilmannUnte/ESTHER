/*
 * Copyright (c) 2022-2026 Vinzenz Malke, Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/

#include "rplidar.h"
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/drivers/watchdog.h>

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rplidar, CONFIG_SENSOR_LOG_LEVEL);
#define DT_DRV_COMPAT slamtech_rplidar


static rplidar_result _rplidar_stop_scan(const struct device * dev);
static rplidar_result _rplidar_reset(const struct device * dev);
static rplidar_result _rplidar_get_deviceinfo(const struct device * dev, rplidar_resp_device_info_t * info);
static rplidar_result _rplidar_get_devicehealth(const struct device * dev, rplidar_resp_device_health_t * health);
static rplidar_result _rplidar_get_samplerate(const struct device * dev, rplidar_resp_samplerate_t * samplerate);


static rplidar_result _rplidar_start_scan(const struct device * dev, struct k_msgq ** msgq, uint8_t type);
static rplidar_result _rplidar_get_config(const struct device * dev, int type, void * request_data, void * response);

static void _rplidar_fill_config_send_buffer(const struct device * dev, int type, void * request_data);
static void _rplidar_append_checksum(const struct device * dev, uint8_t bytes);

static rplidar_result _rplidar_check_health(const struct device *dev);
static rplidar_result _rplidar_check_response_header(const struct device * dev, uint8_t type);

static rplidar_result _rplidar_send_command(const struct device * dev, uint8_t cmd, uint8_t payloadsize);
static rplidar_result _rplidar_send_command_no_response(const struct device * dev, uint8_t cmd);
static rplidar_result _rplidar_receive_data(const struct device * dev, uint8_t datalength);
static void _rplidar_uart_flush(const struct device *uart_dev);

static rplidar_result _rplidar_take_rxtx(const struct device *dev);
static void _rplidar_give_rxtx(const struct device *dev);
static rplidar_result _rplidar_give_rxtx_answer(const struct device *dev, rplidar_result ans);

static int _rplidar_node_from_response(rplidar_resp_measurement_node_t * node, rplidar_measurement_node* m_node);
static void _rplidar_unpacker_thread(void * p1, void * p2, void * p3) ;

#ifdef CONFIG_RPLIDAR_EXPRESS_MODE
static void _rplidar_fill_express_send_buffer(const struct device * dev);
static void _rplidar_nodes_from_express(const struct device * dev, rplidar_resp_express_measurement_t * express_resp, k_timeout_t timeout);
#endif

rplidar_result rplidar_start_scan(const struct device * dev, struct k_msgq ** msgq) 
{
    rplidar_result res = _rplidar_start_scan(dev, msgq, RPLIDAR_SCAN_MODE_DEFAULT);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed starting default scan with error code: 0x%X", res);
    return res;
}

rplidar_result rplidar_start_scan_force(const struct device * dev, struct k_msgq ** msgq) 
{
    rplidar_result res = _rplidar_start_scan(dev, msgq, RPLIDAR_SCAN_MODE_FORCE);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed starting forced scan with error code: 0x%X", res);
    return res;
}

#ifdef CONFIG_RPLIDAR_EXPRESS_MODE
rplidar_result rplidar_start_scan_express(const struct device * dev, struct k_msgq ** msgq) 
{
    rplidar_result res =  _rplidar_start_scan(dev, msgq, RPLIDAR_SCAN_MODE_EXPRESS);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed starting express scan with error code: 0x%X", res);
    return res;
}
#endif

rplidar_result rplidar_stop_scan(const struct device * dev) 
{
    rplidar_result res = _rplidar_stop_scan(dev);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed stopping current scan with error code: 0x%X", res);
    return res;
} 

static rplidar_result _rplidar_stop_scan(const struct device * dev) 
{
    const struct rplidar_cfg *cfg = dev->config;
    struct rplidar_data * data = dev->data;
	int res;
    rplidar_result ans;

    #ifdef CONFIG_RPLIDAR_MOTOR_CTRL
    pwm_set_dt(&cfg->moto_pwm, cfg->moto_pwm.period, 0);
    #endif /* CONFIG_RPLIDAR_MOTOR_CTRL */

    res = k_sem_take(&data->tx_ready_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (res) return RPLIDAR_RESULT_TX_HANDLER_BUSY;

    ans = _rplidar_send_command_no_response(dev, RPLIDAR_CMD_STOP);
    k_sem_give(&data->tx_ready_sem);
    if (RPLIDAR_IS_FAIL(ans)) return ans;    

    data->is_scanning = false;
    #ifdef CONFIG_WATCHDOG
    /* tries to make sure that the watchdog does not interrupt at a bad moment */
    task_wdt_feed(atomic_get(&data->unpacker_watchdog));
    #endif
    k_thread_suspend(data->unpacker_thread_id);
    #ifdef CONFIG_WATCHDOG
    task_wdt_delete(atomic_get(&data->unpacker_watchdog));
    atomic_set(&data->unpacker_watchdog, -1);
    #endif
    uart_irq_rx_disable(cfg->uart_dev);
    k_sem_give(&data->rx_ready_sem);

    if (data->scan_mode == RPLIDAR_SCAN_MODE_FORCE) { 
        /* 
        SUPER HACKY WORKAROUND which flushes the UART (the LIDARS Output UART)
        weirdly neither stop nor reset will clear this
        an OSCILLOSCOPE might give more insight...        
        */
        rplidar_resp_device_info_t info;
        rplidar_get_deviceinfo(dev, &info);
    }

    return RPLIDAR_RESULT_OK;
}

rplidar_result rplidar_reset(const struct device * dev) 
{
    rplidar_result res = _rplidar_reset(dev);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed resetting with error code: 0x%X", res);
    return res;
} 

void rplidar_info_log(const struct device *rpLidar)
{
	if (!device_is_ready(rpLidar)){
		LOG_ERR("Lidar not found!");
		return;
	}

	rplidar_result ans;
	rplidar_resp_device_info_t info;

	ans = rplidar_get_deviceinfo(rpLidar, &info);
	if (RPLIDAR_IS_FAIL(ans)) {
		LOG_ERR("Failed getting RPLidar deviceinfo with Error: 0x%X", ans);
	} else {
		LOG_INF("RPLidar Model Number: 0x%X", info.model);
		LOG_INF("RPLidar Firmware Version: 0x%04X", info.firmware_version);
		LOG_INF("RPLidar Hardware Version: 0x%02X", info.hardware_version);
		LOG_HEXDUMP_INF(info.serialnumber, 16, "RPLidar Serial Number");
	}
	
	uint16_t number_of_modes;
	ans = rplidar_get_scan_mode_count(rpLidar, &number_of_modes);
	if (RPLIDAR_IS_FAIL(ans)) {
		LOG_ERR("Failed getting RPLidar number of modes with Error: 0x%X", ans);
	} else {
		LOG_INF("RPLidar Number of Modes: %u", number_of_modes);
	}

	uint32_t rate;
	uint32_t distance;
	uint8_t ans_type;
	char name[6] = "Empty";	

	for (uint16_t i = 0; i < number_of_modes; i++) {
		ans = rplidar_get_scan_mode_us_per_sample(rpLidar, i, &rate);
		if (RPLIDAR_IS_FAIL(ans)) {
			LOG_ERR("Failed getting RPLidar scan mode us, with ErrorCode: 0x%X", ans);
			rate = 0;
		}
		ans = rplidar_get_scan_mode_max_distance(rpLidar, i, &distance);
		if (RPLIDAR_IS_FAIL(ans)) {
			LOG_ERR("Failed getting RPLidar scan mode distance, with ErrorCode: 0x%X", ans);
			distance = 0;
		}
		ans = rplidar_get_scan_mode_ans_type(rpLidar, i, &ans_type);
		if (RPLIDAR_IS_FAIL(ans)) {
			LOG_ERR("Failed getting RPLidar scan mode anstype, with ErrorCode: 0x%X", ans);
			ans_type = 0x0;
		}
		ans = rplidar_get_scan_mode_name(rpLidar, i, name);
		if (RPLIDAR_IS_FAIL(ans)) {
			LOG_ERR("Failed getting RPLidar scan mode name, with ErrorCode: 0x%X", ans);
			strncpy(name, "Empty", 6);
		}
		
		LOG_INF("RPLidar Mode %u (%s): %uuS per Sample, %um max Distance, 0x%02X answer Type", i, name, rate, distance, ans_type);
		//k_msleep(1);
	}

	uint16_t scan_mode_typical;
	ans = rplidar_get_scan_mode_typical(rpLidar, &scan_mode_typical);
	if (RPLIDAR_IS_FAIL(ans)) {
		LOG_ERR("Failed getting RPLidar typical scan mode with Error: 0x%X\n", ans);
	} else {
		LOG_INF("RPLidar typical scan mode: %u\n", scan_mode_typical);
	}
}

static rplidar_result _rplidar_reset(const struct device * dev)
{
    struct rplidar_data * data = dev->data;
    int res;
    rplidar_result ans;

    res = k_sem_take(&data->tx_ready_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (res) return RPLIDAR_RESULT_TX_HANDLER_BUSY;

    ans = _rplidar_send_command_no_response(dev, RPLIDAR_CMD_RESET);
    k_sem_give(&data->tx_ready_sem);
    if (RPLIDAR_IS_FAIL(ans)) return ans;

    if (data->is_scanning) {      
        data->is_scanning = false;
        #ifdef CONFIG_WATCHDOG
        /* tries to make sure that the watchdog does not interrupt at a bad moment */
        task_wdt_feed(atomic_get(&data->unpacker_watchdog));
        #endif
        k_thread_suspend(data->unpacker_thread_id); 
        #ifdef CONFIG_WATCHDOG
        task_wdt_delete(atomic_get(&data->unpacker_watchdog));
        atomic_set(&data->unpacker_watchdog, -1);
        #endif
	#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        const struct rplidar_cfg *cfg = dev->config;
        uart_irq_rx_disable(cfg->uart_dev);
	#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
        k_sem_give(&data->rx_ready_sem);
    }
    return RPLIDAR_RESULT_OK;
}

rplidar_result rplidar_get_deviceinfo(const struct device * dev, rplidar_resp_device_info_t * info) 
{
    rplidar_result res = _rplidar_get_deviceinfo(dev, info);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed retriving device info with error code: 0x%X", res);
    return res;
} 

static rplidar_result _rplidar_get_deviceinfo(const struct device * dev, rplidar_resp_device_info_t * info) 
{
    struct rplidar_data * data = dev->data;
    rplidar_result ans;

    ans = _rplidar_take_rxtx(dev);
    if (RPLIDAR_IS_FAIL(ans)) return ans;

    ans = _rplidar_send_command(dev, RPLIDAR_CMD_GET_INFO, 0);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
    
    ans = _rplidar_check_response_header(dev, RPLIDAR_RESP_TYPE_INFO);
    if (RPLIDAR_IS_FAIL(ans))  return _rplidar_give_rxtx_answer(dev, ans);

    ans = _rplidar_receive_data(dev, sizeof(rplidar_resp_device_info_t));
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);

    memcpy((uint8_t*)info, &data->rd_data[RPLIDAR_RECV_BUF_LEN - sizeof(rplidar_resp_device_info_t)], sizeof(rplidar_resp_device_info_t));
    _rplidar_give_rxtx(dev);
    return RPLIDAR_RESULT_OK;
}

rplidar_result rplidar_get_devicehealth(const struct device * dev, rplidar_resp_device_health_t * health)
{
    rplidar_result res = _rplidar_get_devicehealth(dev, health);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed retriving device health with error code: 0x%X", res);
    return res;
} 

static rplidar_result _rplidar_get_devicehealth(const struct device * dev, rplidar_resp_device_health_t * health)
{
    struct rplidar_data * data = dev->data;
    rplidar_result ans;
    
    ans = _rplidar_take_rxtx(dev);
    if (RPLIDAR_IS_FAIL(ans)) return ans;
    
    ans = _rplidar_send_command(dev, RPLIDAR_CMD_GET_HEALTH,0);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
    
    ans = _rplidar_check_response_header(dev, RPLIDAR_RESP_TYPE_HEALTH);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);

    ans = _rplidar_receive_data(dev, sizeof(rplidar_resp_device_health_t));
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);

    memcpy((uint8_t*)health, &data->rd_data[RPLIDAR_RECV_BUF_LEN - sizeof(rplidar_resp_device_health_t)], sizeof(rplidar_resp_device_health_t));
    
    _rplidar_give_rxtx(dev);
    return RPLIDAR_RESULT_OK;
}

rplidar_result rplidar_get_samplerate(const struct device * dev, rplidar_resp_samplerate_t * samplerate) 
{
    rplidar_result res = _rplidar_get_samplerate(dev, samplerate);
    if (RPLIDAR_IS_FAIL(res)) LOG_WRN("Failed retriving samplerate with error code: 0x%X", res);
    return res;
} 

static rplidar_result _rplidar_get_samplerate(const struct device * dev, rplidar_resp_samplerate_t * samplerate) 
{
    struct rplidar_data * data = dev->data;
    rplidar_result ans;

    ans = _rplidar_take_rxtx(dev);
    if (RPLIDAR_IS_FAIL(ans)) return ans;

    ans = _rplidar_send_command(dev, RPLIDAR_CMD_GET_SAMPLERATE, 0);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
    
    ans = _rplidar_check_response_header(dev, RPLIDAR_RESP_TYPE_SAMPLERATE);
    if (RPLIDAR_IS_FAIL(ans))  return _rplidar_give_rxtx_answer(dev, ans);

    ans = _rplidar_receive_data(dev, sizeof(rplidar_resp_samplerate_t));
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);

    memcpy((uint8_t*)samplerate, &data->rd_data[RPLIDAR_RECV_BUF_LEN - sizeof(rplidar_resp_samplerate_t)], sizeof(rplidar_resp_samplerate_t));
    _rplidar_give_rxtx(dev);
    return RPLIDAR_RESULT_OK;
}


rplidar_result rplidar_get_scan_mode_count(const struct device * dev, uint16_t * count) {
    rplidar_result ans;
    ans = _rplidar_get_config(dev, RPLIDAR_CONF_SCAN_MODE_COUNT, NULL, (void * )count );
    return ans;
}

rplidar_result rplidar_get_scan_mode_us_per_sample(const struct device * dev, uint16_t mode_id, uint32_t * rate) 
{
    rplidar_result ans;
    uint32_t response = 0;
    ans = _rplidar_get_config(dev, RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, (void *)&mode_id, (void * )&response );
    *rate = (uint32_t)(response >> 8);
    return ans;

}

rplidar_result rplidar_get_scan_mode_max_distance(const struct device * dev, uint16_t mode_id, uint32_t * distance) 
{
    rplidar_result ans;
    uint32_t response = 0;
    ans = _rplidar_get_config(dev, RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, (void *)&mode_id, (void * )&response );
    *distance = (uint32_t)(response >> 8);
    return ans;

}

rplidar_result rplidar_get_scan_mode_ans_type(const struct device * dev, uint16_t mode_id, uint8_t * type)
{
    rplidar_result ans;
    ans = _rplidar_get_config(dev, RPLIDAR_CONF_SCAN_MODE_ANS_TYPE, (void *)&mode_id, (void * )type );
    return ans;
}

rplidar_result rplidar_get_scan_mode_typical(const struct device * dev, uint16_t * index) 
{
    rplidar_result ans;
    ans = _rplidar_get_config(dev, RPLIDAR_CONF_SCAN_MODE_TYPICAL, NULL, (void * )index );
    return ans;
}

rplidar_result rplidar_get_scan_mode_name(const struct device * dev, uint16_t mode_id, uint8_t * name)
{
    rplidar_result ans;
    ans = _rplidar_get_config(dev, RPLIDAR_CONF_SCAN_MODE_NAME, (void *)&mode_id, (void * )name );
    name[5] = 0x00;
    return ans;
}

rplidar_result rplidar_start_motor(const struct device *dev) 
{
    #ifdef CONFIG_RPLIDAR_MOTOR_CTRL
    const struct rplidar_cfg *cfg = dev->config;
    pwm_set_dt(&cfg->moto_pwm, cfg->moto_pwm.period, (cfg->moto_pwm.period*RPLIDAR_MOTOR_SPEED_PERCENT)/100);
    #else
    ARG_UNUSED(dev);
    #endif /* CONFIG_RPLIDAR_MOTOR_CTRL */
    return RPLIDAR_RESULT_OK;
}

rplidar_result rplidar_stop_motor(const struct device *dev)
{
    #ifdef CONFIG_RPLIDAR_MOTOR_CTRL
    const struct rplidar_cfg *cfg = dev->config;
    pwm_set_dt(&cfg->moto_pwm, cfg->moto_pwm.period, 0);
    #else
    ARG_UNUSED(dev);
    #endif /* CONFIG_RPLIDAR_MOTOR_CTRL */
    return RPLIDAR_RESULT_OK;
}

#ifdef CONFIG_WATCHDOG
static void rplidar_watchdog_callback(int channel_id, void* user_data)
{
	struct rplidar_data *data = user_data;
	k_tid_t thread = data->unpacker_thread_id;
	
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
	//k_thread_abort(*thread);
}
#endif /* CONFIG_WATCHDOG */

static rplidar_result _rplidar_start_scan(const struct device * dev, struct k_msgq ** msgq, uint8_t mode) 
{
    const struct rplidar_cfg *cfg = dev->config;
    struct rplidar_data * data = dev->data;
    rplidar_result ans;
    
    #ifdef CONFIG_WATCHDOG
    k_timer_status_sync(&data->unpacker_timer);
    atomic_set(&data->unpacker_watchdog, task_wdt_add(RPLIDAR_UNPACKER_PERIOD_MSEC, rplidar_watchdog_callback, data));
    task_wdt_feed(atomic_get(&data->unpacker_watchdog));
    #endif
    
    ans = _rplidar_check_health(dev);
    if (RPLIDAR_IS_FAIL(ans)) return ans;

    ans = _rplidar_take_rxtx(dev);
    if (RPLIDAR_IS_FAIL(ans)) return ans;
    
    uint8_t payloadsize = 0;
    #ifdef CONFIG_RPLIDAR_EXPRESS_MODE
    if (mode == RPLIDAR_SCAN_MODE_EXPRESS) {_rplidar_fill_express_send_buffer(dev); payloadsize = 7;}
    #endif
    ans = _rplidar_send_command(dev, rplidar_scan_modes[mode][RPLIDAR_SCAN_MODE_COMMAND], payloadsize);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
    
    ans = _rplidar_check_response_header(dev, rplidar_scan_modes[mode][RPLIDAR_SCAN_MODE_RESPONSE_TYPE]);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
    #ifdef CONFIG_RPLIDAR_MOTOR_CTRL
    pwm_set_dt(&cfg->moto_pwm, cfg->moto_pwm.period, (cfg->moto_pwm.period*RPLIDAR_MOTOR_SPEED_PERCENT)/100);
    #endif /* CONFIG_RPLIDAR_MOTOR_CTRL */
    *msgq = &data->scanner_msgq;

    data->scan_mode = mode;  
    data->is_scanning = true; 
    data->expected_rx_size = rplidar_scan_modes[mode][RPLIDAR_SCAN_MODE_RESPONSE_SIZE];
    
    #ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_rx_enable(cfg->uart_dev);
    #endif /* CONFIG_UART_INTERRUPT_DRIVEN */
    #ifdef CONFIG_WATCHDOG
    task_wdt_feed(atomic_get(&data->unpacker_watchdog));
    #endif /* CONFIG_WATCHDOG */
    k_thread_resume(data->unpacker_thread_id);

    k_sem_give(&data->tx_ready_sem);
    return RPLIDAR_RESULT_OK;
}

static rplidar_result _rplidar_get_config(const struct device * dev, int type, void * request_data, void * response)
{
    struct rplidar_data * data = dev->data;
    rplidar_result ans;
    
    ans = _rplidar_take_rxtx(dev);
    if (RPLIDAR_IS_FAIL(ans)) return ans;


    _rplidar_fill_config_send_buffer(dev, type, request_data);
    ans = _rplidar_send_command(dev, RPLIDAR_CMD_GET_CONFIG, 2 + RPLIDAR_CONFIG_REQUEST_TYPE_SIZE + rplidar_conf_types[type][RPLIDAR_CONF_REQUESTPAYLOADSIZE]);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);

    ans = _rplidar_check_response_header(dev, RPLIDAR_RESP_TYPE_CONF);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
   
    ans = _rplidar_receive_data(dev, RPLIDAR_CONFIG_RESPONSE_TYPE_SIZE + rplidar_conf_types[type][RPLIDAR_CONF_RESPONSEPAYLOADSIZE]);
    if (RPLIDAR_IS_FAIL(ans)) return _rplidar_give_rxtx_answer(dev, ans);
    	
    memcpy((uint8_t*)response, &data->rd_data[RPLIDAR_RECV_BUF_LEN - rplidar_conf_types[type][RPLIDAR_CONF_RESPONSEPAYLOADSIZE]], rplidar_conf_types[type][RPLIDAR_CONF_RESPONSEPAYLOADSIZE]);
    
    _rplidar_give_rxtx(dev); 
    return RPLIDAR_RESULT_OK;
}

#ifdef CONFIG_RPLIDAR_EXPRESS_MODE
static void _rplidar_fill_express_send_buffer(const struct device * dev)
{
    struct rplidar_data * data = dev->data;
    // this is precomputed as shown in protocol
    uint8_t index = RPLIDAR_SEND_BUF_LEN - 7;
    data->sd_data[index++] =  0x05;
    data->sd_data[index++] =  0x00;
    data->sd_data[index++] =  0x00;
    data->sd_data[index++] =  0x00;
    data->sd_data[index++] =  0x00;
    data->sd_data[index++] =  0x00;
    data->sd_data[index++] =  0x22;
}
#endif

static void _rplidar_fill_config_send_buffer(const struct device * dev, int type, void * request_data)
{
    struct rplidar_data * data = dev->data;
    uint8_t request_type = rplidar_conf_types[type][RPLIDAR_CONF_REQUESTTYPE];
    uint8_t request_payloadsize = rplidar_conf_types[type][RPLIDAR_CONF_REQUESTPAYLOADSIZE];
    uint8_t index = RPLIDAR_SEND_BUF_LEN - 1 - 1 - request_payloadsize - RPLIDAR_CONFIG_REQUEST_TYPE_SIZE;
    data->sd_data[index++] = request_payloadsize + RPLIDAR_CONFIG_REQUEST_TYPE_SIZE;
    data->sd_data[index++] = request_type;
    index += 3; // Type should be uint32_t but only uint8_t needed
    for (uint8_t i = 0; i < request_payloadsize; i++) {
        data->sd_data[index++] =  ((uint8_t *)request_data)[i];
    }
    _rplidar_append_checksum(dev, request_payloadsize + RPLIDAR_CONFIG_REQUEST_TYPE_SIZE + 1);
}

static void _rplidar_append_checksum(const struct device * dev, uint8_t bytes)
{
    struct rplidar_data * data = dev->data;
    uint8_t checksum = 0x00;
    checksum ^= 0xA5;
    checksum ^= 0x84; //command type... this has to get here somehow
    for (uint8_t i = 0; i < bytes; i++) {
        checksum ^= data->sd_data[RPLIDAR_SEND_BUF_LEN - 1 - i - 1];
    }
    data->sd_data[RPLIDAR_SEND_BUF_LEN -1] = checksum;
}

static rplidar_result _rplidar_check_health(const struct device *dev) 
{
    rplidar_resp_device_health_t health;
    rplidar_result ans;
    ans = _rplidar_get_devicehealth(dev, &health);
    if (RPLIDAR_IS_FAIL(ans)) return ans;

    if (health.status) return RPLIDAR_RESULT_DEVICE_UNHEALTHY;
    return RPLIDAR_RESULT_OK;
}

static rplidar_result _rplidar_check_response_header(const struct device * dev, uint8_t type) 
{
    struct rplidar_data * data = dev->data;
    uint8_t response_header_index = RPLIDAR_RECV_BUF_LEN -  sizeof(rplidar_resp_header_t);
    rplidar_resp_header_t * response_header = (rplidar_resp_header_t*)&data->rd_data[response_header_index];
    if (response_header->startFlag1 != RPLIDAR_RESPONSE_SFLAG1) return RPLIDAR_RESULT_INVALID_RESPONSE_HEADER;
    if (response_header->startFlag2 != RPLIDAR_RESPONSE_SFLAG2) return RPLIDAR_RESULT_INVALID_RESPONSE_HEADER;  
    if (response_header->type != type) return RPLIDAR_RESULT_INVALID_RESPONSE_TYPE;
    return RPLIDAR_RESULT_OK;
}

/*
    Sends out command which and waits for header.
    Expects payload to be already in tx buffer
*/
static rplidar_result _rplidar_send_command(const struct device * dev, uint8_t cmd, uint8_t payloadsize) 
{
    struct rplidar_data * data = dev->data;
    int ret; 

    uint8_t idx = RPLIDAR_SEND_BUF_LEN - payloadsize;
    data->sd_data[--idx] = cmd;
    data->sd_data[--idx] = RPLIDAR_REQUEST_SFLAG;
    data->expected_tx_size = sizeof(RPLIDAR_REQUEST_SFLAG) + sizeof(cmd) + payloadsize; 
    for (int i = 0; i < RPLIDAR_RECV_BUF_LEN; i++) data->rd_data[i] = 0;
    data->expected_rx_size = sizeof(rplidar_resp_header_t);

    /* Useful for debugging
	printk("Send Data: 0x");
	for (int i= 0; i < RPLIDAR_SEND_BUF_LEN; i++) {
		printk("%02X", data->sd_data[i]);
	}
	printk(" sending last 0x%X Bytes\n", data->expected_tx_size);
	*/
   
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    const struct rplidar_cfg *cfg = dev->config;
    uart_irq_tx_enable(cfg->uart_dev);
#else
    k_thread_resume(data->uart_tx_thread_id);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
    ret = k_sem_take(&data->tx_done_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (ret) return RPLIDAR_RESULT_TX_TIMEOUT;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_rx_enable(cfg->uart_dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
    ret = k_sem_take(&data->rx_done_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (ret)  return RPLIDAR_RESULT_RX_TIMEOUT;  
    return RPLIDAR_RESULT_OK;
}

/*
    Send command which doesnt have any response (no Header)
    Only Stop and Reset do so.

*/
static rplidar_result _rplidar_send_command_no_response(const struct device * dev, uint8_t cmd) 
{
    struct rplidar_data * data = dev->data;
    int ret;

    uint8_t idx = RPLIDAR_SEND_BUF_LEN;
    data->sd_data[--idx] = cmd;
    data->sd_data[--idx] = RPLIDAR_REQUEST_SFLAG;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    const struct rplidar_cfg *cfg = dev->config;
    uart_irq_tx_enable(cfg->uart_dev);
#else
    k_thread_resume(data->uart_tx_thread_id);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
    ret = k_sem_take(&data->tx_done_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (ret) return RPLIDAR_RESULT_TX_TIMEOUT;
    return RPLIDAR_RESULT_OK;
}

static rplidar_result _rplidar_receive_data(const struct device * dev, uint8_t datalength)
{
    struct rplidar_data * data = dev->data;
    int ret;

    data->expected_rx_size = datalength;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    const struct rplidar_cfg *cfg = dev->config;
    uart_irq_rx_enable(cfg->uart_dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
    ret = k_sem_take(&data->rx_done_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (ret) return RPLIDAR_RESULT_RX_TIMEOUT;    
    return RPLIDAR_RESULT_OK;
}

static void _rplidar_uart_flush(const struct device *uart_dev)
{
    uint8_t c;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    while (uart_fifo_read(uart_dev, &c, 1) > 0) {
        continue;
    }
#else
    while (uart_poll_in(uart_dev, &c) == 0) {
        continue;
    }
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
}

static rplidar_result _rplidar_take_rxtx(const struct device *dev)
{
    struct rplidar_data * data = dev->data;
    const struct rplidar_cfg *cfg = dev->config;

    int ans;
    ans = k_sem_take(&data->tx_ready_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (ans) return RPLIDAR_RESULT_TX_HANDLER_BUSY;
    ans = k_sem_take(&data->rx_ready_sem, RPLIDAR_DEFAULT_TIMEOUT);
    if (ans) {
        k_sem_give(&data->rx_ready_sem);
        return RPLIDAR_RESULT_RX_HANDLER_BUSY;
    }
    _rplidar_uart_flush(cfg->uart_dev);   
    k_sem_reset(&data->rx_done_sem);
    k_sem_reset(&data->tx_done_sem);
    return RPLIDAR_RESULT_OK;
}

static void _rplidar_give_rxtx(const struct device *dev)
{
    struct rplidar_data * data = dev->data;
    k_sem_give(&data->tx_ready_sem);
    k_sem_give(&data->rx_ready_sem);
}

static rplidar_result _rplidar_give_rxtx_answer(const struct device *dev, rplidar_result ans)
{
    _rplidar_give_rxtx(dev);
    return ans;
}

static int _rplidar_node_from_response(rplidar_resp_measurement_node_t * node, rplidar_measurement_node* m_node) 
{
    /* Check transmission packet integrity by compared sync bit and inverted sync bit (always different) as well as check bit (always 1) */
    if ((node->start_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) == ((node->start_quality & RPLIDAR_RESP_MEASUREMENT_INVERTSYNCBIT) >> RPLIDAR_RESP_MEASUREMENT_INVERTSYNCSHIFT)
         || (node->angle_q6_check & RPLIDAR_RESP_MEASUREMENT_CHECKBIT) != 0x1) {
        return -EINVAL;
    }
    
    sensor_value_from_micro(&m_node->distance, (int64_t) node->distance_q2 * (1000000 / 4 / 1000));
    sensor_value_from_micro(&m_node->angle, (int64_t) (node->angle_q6_check >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) * (1000000 / 64));
    if (sensor_value_to_micro(&m_node->angle) >= 360000000) sensor_value_from_micro(&m_node->angle, sensor_value_to_micro(&m_node->angle) - 360000000);
    m_node->quality = (node->start_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    m_node->startBit = (node->start_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT)?true:false;
    
    return 0;
}

#ifdef CONFIG_RPLIDAR_EXPRESS_MODE
static void _rplidar_nodes_from_express(const struct device * dev, rplidar_resp_express_measurement_t * express_resp, k_timeout_t timeout)
{
    struct rplidar_data * data = dev->data;

    if (express_resp->sync_check_1 >> 4 != 0xA || express_resp->sync_check_2 >> 4 != 0x5) return;
    int64_t start_angle_micro = (express_resp->start_start_angle_q6 & 0x7FFF) * (1000000 / 64); //bitmask start bit
    int64_t angle_diff_micro = sensor_value_to_micro(&data->express_angle_diff);
    angle_diff_micro = angle_diff_micro / 32;
    
    for (int i = 0; i < 16; i++) {          
        uint16_t b1 = express_resp->cabins[i].distance1_dPhi1;
        uint16_t b2 = express_resp->cabins[i].distance2_dPhi2;
        uint8_t  b3 = express_resp->cabins[i].dPhi1_dPhi2;

        int64_t d1_micro = (b1 >> 2) * (1000000 / 1000);
        int64_t d2_micro = (b2 >> 2) * (1000000 / 1000);

        uint8_t dPhi1 = (b3 >> 4 ) | ((b1 & 0x3) << 4);
        uint8_t dPhi2 = (b3 & 0xF) | ((b2 & 0x3) << 4);

        int64_t compPhi1_micro = (dPhi1 & 0x1F) * (1000000 / 3);
        int64_t compPhi2_micro = (dPhi2 & 0x1F) * (1000000 / 3);
        compPhi1_micro = (dPhi1 & 0x20) ? -compPhi1_micro : compPhi1_micro;
        compPhi2_micro = (dPhi2 & 0x20) ? -compPhi2_micro : compPhi2_micro;

        int64_t angle1_micro = start_angle_micro + angle_diff_micro * (((i+1) * 2) -1 ) - compPhi1_micro;
        int64_t angle2_micro = start_angle_micro + angle_diff_micro * (((i+1) * 2)    ) - compPhi2_micro;
        
        rplidar_measurement_node node_1;
        rplidar_measurement_node node_2;
            
        sensor_value_from_micro(&node_1.distance, d1_micro);
        sensor_value_from_micro(&node_2.distance, d2_micro);

        if (angle1_micro >= 360000000) angle1_micro -= 360000000;
        sensor_value_from_micro(&node_1.angle, angle1_micro);
        if (angle2_micro >= 360000000) angle2_micro -= 360000000;
        sensor_value_from_micro(&node_2.angle, angle2_micro);

        
        //FIXME: adjust timeout between all the calls
        if (d1_micro) {
            if (k_msgq_put(&data->scanner_msgq, &node_1, timeout) != 0) {
                LOG_DBG("Purging scanner queue");
	        k_msgq_purge(&data->scanner_msgq);
	        k_msgq_put(&data->scanner_msgq, &node_1, timeout);
            }
        }
        
        if (d2_micro) {
            if (k_msgq_put(&data->scanner_msgq, &node_2, timeout) != 0) {
                LOG_DBG("Purging scanner queue");
                k_msgq_purge(&data->scanner_msgq);
                k_msgq_put(&data->scanner_msgq, &node_2, timeout);
            }
        }
    }
}
#endif

static void _rplidar_unpacker_thread(void * p1, void * p2, void * p3) 
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    const struct device *dev = p1;
    struct rplidar_data * data = dev->data;

    int res;
    
    #ifdef CONFIG_WATCHDOG
    task_wdt_feed(atomic_get(&data->unpacker_watchdog));
    #else
    int64_t deadline;
    #endif /* CONFIG_WATCHDOG */

    #ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
    bool deadline_miss = false;
    #endif
    
    while (1) {
        #ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
        if (deadline_miss == false) k_timer_status_sync(&data->unpacker_timer);
        #else
        k_timer_status_sync(&data->unpacker_timer);
        #endif

	k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->unpacker_timer)));
        
        #ifdef CONFIG_SCHED_DEADLINE
        k_thread_deadline_set(data->unpacker_thread_id, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->unpacker_timer)));
        #endif /* CONFIG_SCHED_DEADLINE */
        
        #ifndef CONFIG_WATCHDOG
        deadline = k_uptime_ticks() + k_timer_remaining_ticks(&data->unpacker_timer);
        #endif

        //k_timepoint_t end = sys_timepoint_calc(K_MSEC(k_timer_remaining_get(&data->unpacker_timer)));
        
        for (uint8_t i = 0; i < (RPLIDAR_TRANSMISSIONS_PER_MSEC * RPLIDAR_UNPACKER_PERIOD_MSEC); i++) {
            if (data->scan_mode == RPLIDAR_SCAN_MODE_DEFAULT || data->scan_mode == RPLIDAR_SCAN_MODE_FORCE) {
                rplidar_resp_measurement_node_t node;
                res = k_msgq_get(&data->transfer_msgq, &node, sys_timepoint_timeout(deadline_tp));       
                if (res) break;
                rplidar_measurement_node m_node;
                res = _rplidar_node_from_response(&node, &m_node);
                if (res) continue; /* discard broken transmission and move on to the next one */
                if (k_msgq_put(&data->scanner_msgq, &m_node, sys_timepoint_timeout(deadline_tp)) != 0) {
                    LOG_DBG("Purging scanner queue");
                    k_msgq_purge(&data->scanner_msgq);
                    k_msgq_put(&data->scanner_msgq, &m_node, sys_timepoint_timeout(deadline_tp));
                }
            }
            #ifdef CONFIG_RPLIDAR_EXPRESS_MODE
            else if (data->scan_mode == RPLIDAR_SCAN_MODE_EXPRESS) {
                 rplidar_resp_express_measurement_t measurement;
                 res = k_msgq_get(&data->express_transfer_msgq, &measurement, sys_timepoint_timeout(deadline_tp));
                 if (res) break;
                 _rplidar_nodes_from_express(dev, &measurement, sys_timepoint_timeout(deadline_tp));
            }
            #endif
        }

        #ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
        deadline_miss = sys_timepoint_expired(deadline_tp);
        #endif

        #ifdef CONFIG_WATCHDOG
        task_wdt_feed(atomic_get(&data->unpacker_watchdog));
        #else
        if (deadline < k_uptime_ticks()) {
            LOG_DBG("Unpacker deadline");
        }
        #endif /* CONFIG_WATCHDOG */
    }
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void _rplidar_uart_isr(const struct device *uart_dev, void *user_data) {
    const struct device *dev = user_data;
    struct rplidar_data *data = dev->data;
    ARG_UNUSED(user_data);

    if (uart_dev == NULL) {
        return;
    }

    // Must be called -> API
    if (!uart_irq_update(uart_dev)) {
        return;
    }


    // Check if we Receive data (only one Transmission is handled at one time (Semaphors))
    if (uart_irq_rx_ready(uart_dev)) {
        // Read bytes
        data->expected_rx_size -= uart_fifo_read(uart_dev, 
                                                   &data->rd_data[RPLIDAR_RECV_BUF_LEN - data->expected_rx_size],
                                                   data->expected_rx_size);
        // If all bytes were received
        if (data->expected_rx_size == 0) {
            if (data->is_scanning) {
                uint8_t node_index = RPLIDAR_RECV_BUF_LEN  - rplidar_scan_modes[data->scan_mode][RPLIDAR_SCAN_MODE_RESPONSE_SIZE];
                if (data->scan_mode == RPLIDAR_SCAN_MODE_DEFAULT || data->scan_mode == RPLIDAR_SCAN_MODE_FORCE) {
                    if (k_msgq_put(&data->transfer_msgq, (rplidar_resp_measurement_node_t *)&data->rd_data[node_index], K_NO_WAIT) != 0) {
                        LOG_WRN("Purging transfer queue");
                        k_msgq_purge(&data->transfer_msgq);
                        k_msgq_put(&data->transfer_msgq, (rplidar_resp_measurement_node_t *)&data->rd_data[node_index], K_NO_WAIT);
                    }             
                }
                #ifdef CONFIG_RPLIDAR_EXPRESS_MODE
                else if (data->scan_mode == RPLIDAR_SCAN_MODE_EXPRESS) {
                    uint16_t start_start_angle_q6  = ((rplidar_resp_express_measurement_t *)&data->rd_data[node_index])->start_start_angle_q6;
                    int64_t start_angle_micro = (start_start_angle_q6 & 0x7FFF) * (1000000 / 64); //bitmask start bit
                    int64_t angle_diff_micro = start_angle_micro >= sensor_value_to_micro(&data->express_old_start_angle) ?  start_angle_micro - sensor_value_to_micro(&data->express_old_start_angle) : 360000000 + start_angle_micro - sensor_value_to_micro(&data->express_old_start_angle);
                    sensor_value_from_micro(&data->express_old_start_angle, start_angle_micro);
                    sensor_value_from_micro(&data->express_angle_diff, angle_diff_micro);
                
                    if (k_msgq_put(&data->express_transfer_msgq, (rplidar_resp_express_measurement_t *)&data->rd_data[node_index], K_NO_WAIT) != 0) {
                        LOG_WRN("Purging express transfer queue");
                        k_msgq_purge(&data->express_transfer_msgq);
                        k_msgq_put(&data->express_transfer_msgq, (rplidar_resp_express_measurement_t *)&data->rd_data[node_index], K_NO_WAIT);
                    }
                }
                #endif
                data->expected_rx_size = rplidar_scan_modes[data->scan_mode][RPLIDAR_SCAN_MODE_RESPONSE_SIZE];
            } else {
                uart_irq_rx_disable(uart_dev);
                k_sem_give(&data->rx_done_sem);
            }
        }
    }

    // If we can transmit data
    if (uart_irq_tx_ready(uart_dev)) {
        data->expected_tx_size -= uart_fifo_fill(uart_dev, &data->sd_data[RPLIDAR_SEND_BUF_LEN - data->expected_tx_size], data->expected_tx_size);

        // If all bytes were sent out
        if (data->expected_tx_size == 0) {
            uart_irq_tx_disable(uart_dev);
            for (int i = 0; i < RPLIDAR_SEND_BUF_LEN; i++) {
                data->sd_data[i] = 0x00;
            }
            k_sem_give(&data->tx_done_sem);
        }
    }
}
#else

static void _rplidar_uart_rx_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	const struct device* dev = p1;
	struct rplidar_data *data = dev->data;
	const struct rplidar_cfg *cfg = dev->config;

	#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	//FIXME: Zephyr Watchdogs are limited to millisecond accuracy, this thread has a smaller period so it does not yet have a deadline watchdog
	for(;;) {
		#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->uart_rx_timer);
		#else
		k_timer_status_sync(&data->uart_rx_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->uart_rx_timer)));

		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->uart_rx_thread_id, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->uart_rx_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */

		// Receive
		// Under normal operating conditions, only one byte will be received per job
		while (data->expected_rx_size > 0) {
			if (uart_poll_in(cfg->uart_dev, &data->rd_data[RPLIDAR_RECV_BUF_LEN - data->expected_rx_size]) != 0) {
				break;
			}
			data->expected_rx_size--;
		}

		// If all bytes were received
		if (data->expected_rx_size == 0) {
			if (data->is_scanning) {
				uint8_t node_index = RPLIDAR_RECV_BUF_LEN  - rplidar_scan_modes[data->scan_mode][RPLIDAR_SCAN_MODE_RESPONSE_SIZE];
				if (data->scan_mode == RPLIDAR_SCAN_MODE_DEFAULT || data->scan_mode == RPLIDAR_SCAN_MODE_FORCE) {
					if (k_msgq_put(&data->transfer_msgq, (rplidar_resp_measurement_node_t *)&data->rd_data[node_index], K_NO_WAIT) != 0) {
						LOG_WRN("Purging transfer queue");
						k_msgq_purge(&data->transfer_msgq);
						k_msgq_put(&data->transfer_msgq, (rplidar_resp_measurement_node_t *)&data->rd_data[node_index], K_NO_WAIT);
					}
				}
                                #ifdef CONFIG_RPLIDAR_EXPRESS_MODE
                                else if (data->scan_mode == RPLIDAR_SCAN_MODE_EXPRESS) {
					uint16_t start_start_angle_q6  = ((rplidar_resp_express_measurement_t *)&data->rd_data[node_index])->start_start_angle_q6;
					int64_t start_angle_micro = (start_start_angle_q6 & 0x7FFF) * (1000000 / 64); //bitmask start bit
					int64_t angle_diff_micro = start_angle_micro >= sensor_value_to_micro(&data->express_old_start_angle) ?  start_angle_micro - sensor_value_to_micro(&data->express_old_start_angle) : 360000000 + start_angle_micro - sensor_value_to_micro(&data->express_old_start_angle);
					sensor_value_from_micro(&data->express_old_start_angle, start_angle_micro);
					sensor_value_from_micro(&data->express_angle_diff, angle_diff_micro);

					if (k_msgq_put(&data->express_transfer_msgq, (rplidar_resp_express_measurement_t *)&data->rd_data[node_index], K_NO_WAIT) != 0) {
						LOG_WRN("Purging express transfer queue");
						k_msgq_purge(&data->express_transfer_msgq);
						k_msgq_put(&data->express_transfer_msgq, (rplidar_resp_express_measurement_t *)&data->rd_data[node_index], K_NO_WAIT);
					}
				}
                                #endif                 
				data->expected_rx_size = rplidar_scan_modes[data->scan_mode][RPLIDAR_SCAN_MODE_RESPONSE_SIZE];
			} else {
				k_sem_give(&data->rx_done_sem);
			}
		}

		#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
		deadline_miss = sys_timepoint_expired(deadline_tp);
		#endif
	}
}

static void _rplidar_uart_tx_thread(void* p1, void* p2, void* p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);
	const struct device* dev = p1;
	struct rplidar_data *data = dev->data;
	const struct rplidar_cfg *cfg = dev->config;

	#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
	bool deadline_miss = false;
	#endif
	
	//FIXME: Zephyr Watchdogs are limited to millisecond accuracy, this thread has a smaller period so it does not yet have a deadline watchdog
	for(;;) {
		#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
		if (deadline_miss == false) k_timer_status_sync(&data->uart_tx_timer);
		#else
		k_timer_status_sync(&data->uart_tx_timer);
		#endif

		k_timepoint_t deadline_tp = sys_timepoint_calc(K_TICKS(k_timer_remaining_ticks(&data->uart_tx_timer)));

		#ifdef CONFIG_SCHED_DEADLINE
		k_thread_deadline_set(data->uart_tx_thread_id, k_ticks_to_cyc_floor32(k_timer_remaining_ticks(&data->uart_tx_timer)));
		#endif /* CONFIG_SCHED_DEADLINE */

		// Transfer
		if (data->expected_tx_size > 0) {
			uart_poll_out(cfg->uart_dev, data->sd_data[RPLIDAR_SEND_BUF_LEN - data->expected_tx_size]);
			data->expected_tx_size--;
		}

		// If all bytes were sent out
		if (data->expected_tx_size == 0) {
			for (int i = 0; i < RPLIDAR_SEND_BUF_LEN; i++) {
				data->sd_data[i] = 0x00;
			}

			k_sem_give(&data->tx_done_sem);

			#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
			deadline_miss = false;
			#endif
			k_thread_suspend(data->uart_tx_thread_id);
		} else {
			#ifdef CONFIG_RPLIDAR_DEADLINE_MISS_STRATEGY_QUEUE
			deadline_miss = sys_timepoint_expired(deadline_tp);
			#endif
		}
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int rplidar_init(const struct device *dev)
{
    struct rplidar_data *data = dev->data;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    const struct rplidar_cfg *cfg = dev->config;
    uart_irq_rx_disable(cfg->uart_dev);
    uart_irq_tx_disable(cfg->uart_dev);

    uart_irq_callback_user_data_set(cfg->uart_dev, cfg->cb, (void *)dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
    k_sem_init(&data->rx_ready_sem, 1, 1);
    k_sem_init(&data->tx_ready_sem, 1, 1);

    k_sem_init(&data->rx_done_sem, 0, 1);
    k_sem_init(&data->tx_done_sem, 0, 1);

    k_msgq_init(&data->scanner_msgq, (uint8_t *)data->scanner_msgq_buffer, NODE_ALIGN, SCANNER_BUFFER_SIZE);
    k_msgq_init(&data->transfer_msgq, (uint8_t *)data->transfer_msgq_buffer, sizeof(rplidar_resp_measurement_node_t), TRANSFER_BUFFER_SIZE);
    #ifdef CONFIG_RPLIDAR_EXPRESS_MODE
    k_msgq_init(&data->express_transfer_msgq, (uint8_t *)data->express_transfer_msgq_buffer, sizeof(rplidar_resp_express_measurement_t), EXPRESS_TRANSFER_BUFFER_SIZE);
    #endif

    k_timer_init(&data->unpacker_timer, NULL, NULL);
    uint64_t period_ticks = k_ms_to_ticks_near64(RPLIDAR_UNPACKER_PERIOD_MSEC);
    k_timer_start(&data->unpacker_timer, K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)), K_TICKS(period_ticks));

#ifndef CONFIG_UART_INTERRUPT_DRIVEN
    k_timer_init(&data->uart_rx_timer, NULL, NULL);
    period_ticks = k_ns_to_ticks_floor64(RPLIDAR_UART_PERIOD_NSEC / 2);
    k_timer_start(&data->uart_rx_timer, K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)), K_TICKS(period_ticks));

    data->uart_rx_thread_id = k_thread_create(&data->uart_rx_thread, data->uart_rx_stack, K_KERNEL_STACK_SIZEOF(data->uart_rx_stack),
                    _rplidar_uart_rx_thread, (void*)dev, NULL, NULL, RPLIDAR_UART_RX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
    #ifdef CONFIG_THREAD_NAME
    k_thread_name_set(data->uart_rx_thread_id, "rplidar_uart_rx");
    #endif /* CONFIG_THREAD_NAME */

    k_timer_init(&data->uart_tx_timer, NULL, NULL);
    period_ticks = k_ns_to_ticks_floor64(RPLIDAR_UART_PERIOD_NSEC);
    k_timer_start(&data->uart_tx_timer, K_TICKS(period_ticks - (k_uptime_ticks() % period_ticks)), K_TICKS(period_ticks));

    data->uart_tx_thread_id = k_thread_create(&data->uart_tx_thread, data->uart_tx_stack, K_KERNEL_STACK_SIZEOF(data->uart_tx_stack),
                    _rplidar_uart_tx_thread, (void*)dev, NULL, NULL, RPLIDAR_UART_TX_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
    #ifdef CONFIG_THREAD_NAME
    k_thread_name_set(data->uart_tx_thread_id, "rplidar_uart_tx");
    #endif /* CONFIG_THREAD_NAME */
    k_thread_suspend(data->uart_tx_thread_id);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

    data->unpacker_thread_id = k_thread_create(&data->unpacker_thread, data->unpacker_stack, K_KERNEL_STACK_SIZEOF(data->unpacker_stack), 
                    _rplidar_unpacker_thread, (void*)dev, NULL, NULL, RPLIDAR_UNPACKER_THREAD_PRIORITY, K_ESSENTIAL, K_NO_WAIT);
    #ifdef CONFIG_THREAD_NAME
    k_thread_name_set(data->unpacker_thread_id, "rplidar_unpacker");
    #endif /* CONFIG_THREAD_NAME */
    #ifdef CONFIG_WATCHDOG
    atomic_set(&data->unpacker_watchdog, -1);
    #endif
    k_thread_suspend(data->unpacker_thread_id);
    
    data->is_scanning = false;    
    return 0;
}
/* NOTE: The following macro will be inserted into a struct definition.
 * It was setup to already contain commas at the end of each line! */
#ifdef CONFIG_RPLIDAR_MOTOR_CTRL
#define PWM_DEVICE(inst) .moto_pwm = PWM_DT_SPEC_GET(DT_INST(inst, DT_DRV_COMPAT)),
#else
#define PWM_DEVICE(inst)
#endif /* CONFIG_RPLIDAR_MOTOR_CTRL */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_ISR .cb = _rplidar_uart_isr,
#else
#define UART_ISR
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define RPLIDAR_INIT(inst)                                              \
    static struct rplidar_data rplidar_data_##inst;                     \
    static const struct rplidar_cfg rplidar_config_##inst = {           \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                   \
        UART_ISR                                                        \
        PWM_DEVICE(inst)                                                \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst, rplidar_init, NULL, &rplidar_data_##inst, &rplidar_config_##inst,  \
        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);                                           \

DT_INST_FOREACH_STATUS_OKAY(RPLIDAR_INIT);

// PWM_DT_SPEC_GET(DT_LABEL(PWMMOTO)),    

/*        .moto_pwm = PWM_DT_SPEC_GET(DT_INST(inst, pwm4)),                
*/
