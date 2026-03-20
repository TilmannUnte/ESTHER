#pragma once

/*
 * Copyright (c) 2022-2026 Vinzenz Malke, Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/

/**
* @mainpage 
* The RoboPeak Lidar is a low cost LIDAR sensor by Slamtec.
*
* This Sensor doesn't use the zephyr sensor api, because of it's working principle.
* 
* The most generic form of measuring is as follows:
* @code
* static const struct device *const rpLidar = DEVICE_DT_GET(DT_NODELABEL(rplidar));
* struct k_msgq * msgq = NULL;
* rplidar_measurement_node m_node;
*	rplidar_start_scan(rpLidar, &msgq);
* while (1) k_msgq_get(msgq, &m_node,K_FOREVER );
* @endcode
*/

#include "inc/rplidar_cmd.h"
#include "inc/rplidar_types.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/types.h>

#define RPLIDAR_DEFAULT_TIMEOUT K_MSEC(500) //500
#define RPLIDAR_DOUBLE_TIMEOUT K_MSEC(1000) //500
#define RPLIDAR_LONG_TIMEOUT  K_MSEC(3000) // 1000

#define RPLIDAR_RECV_BUF_LEN 100 // biggest is express mode
#define RPLIDAR_SEND_BUF_LEN 10

// Bounding the period for the unpacker task is non-trivial
// Messages from the LiDAR have a variety of sizes, depending on mode.
// Normal operation yields a sample roughly every 0.5ms. Express mode delivers two samples at once with the same message delivery rate.
// We process multiple of either kind of message for each job of the unpacker thread.
#define RPLIDAR_UNPACKER_PERIOD_MSEC 4
#define RPLIDAR_TRANSMISSIONS_PER_MSEC 2
#define RPLIDAR_UART_PERIOD_NSEC 86806

#define RPLIDAR_THREAD_STACK_SIZE 1024//2048

//TODO move to KConfig
#define RPLIDAR_UART_RX_THREAD_PRIORITY LOG2CEIL(RPLIDAR_UART_PERIOD_NSEC / 2) // = 16
#define RPLIDAR_UART_TX_THREAD_PRIORITY LOG2CEIL(RPLIDAR_UART_PERIOD_NSEC) // = 17
#define RPLIDAR_UNPACKER_THREAD_PRIORITY LOG2CEIL(RPLIDAR_UNPACKER_PERIOD_MSEC * NSEC_PER_MSEC) // = 22

//TODO move to DeviceTree
#define RPLIDAR_MOTOR_SPEED_PERCENT 78 // must be experimentally determined, likely depends on your specific setup. You want to optimize for 360 measurements for every rotation. Blindly setting this to 100% will perform worse!

#define NODE_ALIGN 32
#define TRANSFER_NODE_ALIGN 8
#define TRANSFER_EXPRESS_ALIGN 128

#define SCANNER_BUFFER_SIZE (RPLIDAR_TRANSMISSIONS_PER_MSEC * RPLIDAR_UNPACKER_PERIOD_MSEC)

// native_sim (POSIX) breaks timing and benefits from MUCH larger buffer sizes to compensate
#ifdef CONFIG_ARCH_POSIX
#define TRANSFER_BUFFER_SIZE (RPLIDAR_TRANSMISSIONS_PER_MSEC * RPLIDAR_UNPACKER_PERIOD_MSEC * 512)
#define EXPRESS_TRANSFER_BUFFER_SIZE (RPLIDAR_TRANSMISSIONS_PER_MSEC * RPLIDAR_UNPACKER_PERIOD_MSEC * 512)
#else
#define TRANSFER_BUFFER_SIZE (2 * RPLIDAR_TRANSMISSIONS_PER_MSEC * RPLIDAR_UNPACKER_PERIOD_MSEC)
#define EXPRESS_TRANSFER_BUFFER_SIZE (2 * RPLIDAR_TRANSMISSIONS_PER_MSEC * RPLIDAR_UNPACKER_PERIOD_MSEC)
#endif /* CONFIG_ARCH_POSIX */
/*!
* @brief Output data structure for measurements
*/
typedef struct rplidar_measurement_node
{
    struct sensor_value distance;  ///< distance in m
    struct sensor_value angle;     ///< angle in degrees
    uint8_t quality;  ///< quality of measurement
    _Bool startBit;    ///< set, if datapoint is the first in new rotation
}__attribute__((aligned(NODE_ALIGN))) rplidar_measurement_node;

/*!
* @brief Starts scanning operation in default mode.
*
* Outputs roughly 2000sps.
* The msgq will be filled with rplidar_measurement_node 's
*
* @param dev device pointer
* @param msgq pointer which will be bent to output msgq
*/
rplidar_result rplidar_start_scan(const struct device * dev, struct k_msgq ** msgq);

/*!
* @brief Starts scanning operation in force mode.
*
* The Lidar will not wait until the rotation is stabilised before measuring.
* Outputs roughly 2000sps.
* The msgq will be filled with rplidar_measurement_node 's
*
* Should be used for debugging only.
*
* @param dev device pointer
* @param msgq pointer which will be bent to output msgq
*/
rplidar_result rplidar_start_scan_force(const struct device * dev, struct k_msgq ** msgq);


/*!
* @brief Starts scanning operation in express mode.
*
* Outputs roughly 4000sps.
* The msgq will be filled with rplidar_measurement_node 's
* (Qulity and Startbit are not set in this mode)
*
* @note Only supported on firmware version > 1.17 
*
* @param dev device pointer
* @param msgq pointer which will be bent to output msgq
*/
#ifdef CONFIG_RPLIDAR_EXPRESS_MODE
rplidar_result rplidar_start_scan_express(const struct device * dev, struct k_msgq ** msgq);
#endif

/*!
* @brief Stops all scanning Operations
* 
* @param dev device pointer
*/
rplidar_result rplidar_stop_scan(const struct device * dev);

/*!
* @brief Resets/reboots the RPLIDAR Core
*
* @param dev device pointer
*/
rplidar_result rplidar_reset(const struct device * dev);

/*!
* @brief Get RPLidar device info
*
* @param dev device pointer
* @param info Adress to which the device health will be written
*/
rplidar_result rplidar_get_deviceinfo(const struct device * dev, rplidar_resp_device_info_t * info);

/*!
* @brief Get RPLidar device health
* 
* @param dev device pointer
* @param health Adress to which the device info will be written
*/
rplidar_result rplidar_get_devicehealth(const struct device * dev, rplidar_resp_device_health_t * health);

/*!
* @brief Get RPLidar samplingtime 
*
* @note Uses GET_SAMPLERATE request, which is only available if firmware version > 1.17 
*
* @param dev device pointer
* @param samplerate Adress to which the device samplerate will be written
*/
rplidar_result rplidar_get_samplerate(const struct device * dev, rplidar_resp_samplerate_t * samplerate);

/*!
* @brief Get amount of scan modes available 
* 
* @note Uses GET_LIDAR_CONF request, which is only available if firmware version > 1.24
* 
* @param dev device pointer
* @param count Adress to which the scan mode count will be written 
*/
rplidar_result rplidar_get_scan_mode_count(const struct device * dev, uint16_t * count);

/*!
* @brief Get sampletime in ms for specific scan mode 
* 
* @note Uses GET_LIDAR_CONF request, which is only available if firmware version > 1.24
* 
* @param dev device pointer
* @param mode_id The id of the mode that gets checked
* @param rate Adress to which the sampletime will be written 
*/
rplidar_result rplidar_get_scan_mode_us_per_sample(const struct device * dev, uint16_t mode_id, uint32_t * rate);

/*!
* @brief Get maximum distance of specific scan mode
* 
* @note Uses GET_LIDAR_CONF request, which is only available if firmware version > 1.24
* 
* @param dev device pointer
* @param mode_id The id of the mode that gets checked
* @param distance Adress to which maximum distance will be written (in meters)
*/
rplidar_result rplidar_get_scan_mode_max_distance(const struct device * dev, uint16_t mode_id, uint32_t  * distance);

/*!
* @brief Get answer type of specific scan mode
* 
* @note Uses GET_LIDAR_CONF request, which is only available if firmware version > 1.24
* 
* @param dev device pointer
* @param mode_id The id of the mode that gets checked
* @param type Adress to which answere type will be written
*/
rplidar_result rplidar_get_scan_mode_ans_type(const struct device * dev, uint16_t mode_id, uint8_t * type);

/*!
* @brief Get the default scan mode used by lidar
* 
* @note Uses GET_LIDAR_CONF request, which is only available if firmware version > 1.24
* 
* @param dev device pointer
* @param index Adress to which the scan mode index will be written
*/
rplidar_result rplidar_get_scan_mode_typical(const struct device * dev, uint16_t * index);


/*!
* @brief Get first 5 letters of the scan modes name
* 
* @note Uses GET_LIDAR_CONF request, which is only available if firmware version > 1.24
* 
* @param dev device pointer
* @param mode_id The id of the mode that gets checked
* @param name Adress to which the name will be written (uint8_t array of size 6 (0 byte))
*/
rplidar_result rplidar_get_scan_mode_name(const struct device * dev, uint16_t mode_id, uint8_t * name);


/*!
* @brief Starts motor of Lidar
* For debugging purposes only.
* @param dev device pointer
*/
rplidar_result rplidar_start_motor(const struct device *dev);

/*!
* @brief Stops motor of Lidar
* For debugging purposes only.
* @param dev device pointer
*/
rplidar_result rplidar_stop_motor(const struct device *dev);

/*!
* @brief Log all LiDAR info
* For debugging purposes only, requires full Logging output to be enabled.
* @param rpLidar device pointer
*/
void rplidar_info(const struct device *rpLidar);

struct rplidar_data 
{
    uint16_t data;

    uint8_t expected_rx_size;
    uint8_t expected_tx_size;
    bool has_response;

    uint8_t rd_data[RPLIDAR_RECV_BUF_LEN];
    uint8_t sd_data[RPLIDAR_SEND_BUF_LEN];

    struct k_sem rx_ready_sem;
    struct k_sem rx_done_sem;  
    struct k_sem tx_ready_sem;
    struct k_sem tx_done_sem; 
    
    struct k_timer unpacker_timer;
    atomic_t unpacker_watchdog;

    K_KERNEL_STACK_MEMBER(unpacker_stack, RPLIDAR_THREAD_STACK_SIZE);
    struct k_thread unpacker_thread;
    k_tid_t unpacker_thread_id;

#ifndef CONFIG_UART_INTERRUPT_DRIVEN
    K_KERNEL_STACK_MEMBER(uart_rx_stack, RPLIDAR_THREAD_STACK_SIZE);
    K_KERNEL_STACK_MEMBER(uart_tx_stack, RPLIDAR_THREAD_STACK_SIZE);
    struct k_thread uart_rx_thread;
    struct k_thread uart_tx_thread;
    k_tid_t uart_rx_thread_id;
    k_tid_t uart_tx_thread_id;
    struct k_timer uart_rx_timer;
    struct k_timer uart_tx_timer;
#endif

    uint8_t __aligned(NODE_ALIGN) scanner_msgq_buffer[SCANNER_BUFFER_SIZE * sizeof(rplidar_measurement_node)];
    struct k_msgq scanner_msgq;

    uint8_t __aligned(TRANSFER_NODE_ALIGN) transfer_msgq_buffer[TRANSFER_BUFFER_SIZE * TRANSFER_NODE_ALIGN];
    struct k_msgq transfer_msgq;

#ifdef CONFIG_RPLIDAR_EXPRESS_MODE
    uint8_t __aligned(TRANSFER_EXPRESS_ALIGN) express_transfer_msgq_buffer[EXPRESS_TRANSFER_BUFFER_SIZE * TRANSFER_EXPRESS_ALIGN];
    struct k_msgq express_transfer_msgq;
    struct sensor_value express_old_start_angle;
    struct sensor_value express_angle_diff;
#endif

    bool is_scanning;
    uint8_t scan_mode;
};

struct rplidar_cfg {
  const struct device *uart_dev;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
  uart_irq_callback_user_data_t cb;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#ifdef CONFIG_RPLIDAR_MOTOR_CTRL
  const struct pwm_dt_spec moto_pwm;
#endif /* CONFIG_RPLIDAR_MOTOR_CTRL */
};
