/*
 * Copyright (c) 2022 Vinzenz Malke
 * SPDX-License-Identifier:  Apache-2.0
*/

#pragma once
#include <zephyr/types.h>


// Request Protocol

#define RPLIDAR_REQUEST_SFLAG           ((uint8_t)0xA5)


// Response Protocol

#define RPLIDAR_RESPONSE_SFLAG1         0xA5
#define RPLIDAR_RESPONSE_SFLAG2         0x5A

#define RPLIDAR_RESPONSE_MODE_SINGLE    0x0
#define RPLIDAR_RESPONSE_MODE_MULTI     0x1


// Response Types

#define RPLIDAR_RESP_TYPE_MEASUREMENT   0x81 // answer to: SCAN, FORCE_SCAN
#define RPLIDAR_RESP_TYPE_EM_LEGACY     0x82 // answer to: EXPRESS_SCAN (Legacy) 
#define RPLIDAR_RESP_TYPE_EM_EXTENDED   0x84 // answer to: EXPRESS_SCAN (Extended)
#define RPLIDAR_RESP_TYPE_EM_DENSE      0x85 // answer to: EXPRESS_SCAN (Dense)

#define RPLIDAR_RESP_TYPE_INFO          0x4
#define RPLIDAR_RESP_TYPE_HEALTH        0x6
#define RPLIDAR_RESP_TYPE_SAMPLERATE    0x15
#define RPLIDAR_RESP_TYPE_CONF          0x20


// Request

typedef struct _rplidar_request_packet_t {
    uint8_t startFlag;                        // must be RPLIDAR_REQUEST_SFLAG
    uint8_t cmd;
    uint8_t size;
    uint8_t data[0];
} __attribute__((packed)) rplidar_request_packet_t;


// Response Header
typedef struct _rplidar_resp_header_t {
    uint8_t     startFlag1; // must be RPLIDAR_RESPONSE_SFLAG1
    uint8_t     startFlag2; // must be RPLIDAR_RESPONSE_SFLAG2
    uint32_t    length:30;
    uint32_t    sendMode:2;
    uint8_t     type;
} __attribute__((packed)) rplidar_resp_header_t;

// Response types

typedef struct _rplidar_resp_measurement_node_t {
    uint8_t     start_quality;  // startbit: 1; inverse_start: 1; quality: 6
    uint16_t    angle_q6_check; // angle_q6: 15; checkbit: 1
    uint16_t    distance_q2; 
} __attribute__((packed)) rplidar_resp_measurement_node_t;

typedef struct _rplidar_resp_measurement_cabin_t {
    uint16_t    distance1_dPhi1; // distance1: 14, dPhi: 2
    uint16_t    distance2_dPhi2; // distance1: 14, dPhi: 2
    uint8_t     dPhi1_dPhi2;     // dPhi1: 4, dPhi2: 4
} __attribute__((packed)) rplidar_resp_measurement_cabin_t;


typedef struct _rplidar_resp_express_measurement_t {
    uint8_t     sync_check_1;   // syncflag1 (0xA): 4, checksum: 4
    uint8_t     sync_check_2;   // syncflag2 (0x5): 4, checksum: 4
    uint16_t    start_start_angle_q6; // startFlag: 1, start_angle: 15 
    rplidar_resp_measurement_cabin_t cabins[16]; // 16 cabins 
} __attribute__((packed)) rplidar_resp_express_measurement_t;


/*!
* @brief RPLidar device info
*/
typedef struct rplidar_resp_device_info_t {
    uint8_t     model;              ///< RPLIDAR model ID
    uint16_t    firmware_version;   ///< Firmware version number (8 bits integer part, 8 bits decimal part)
    uint8_t     hardware_version;   ///< Hardware version number
    uint8_t     serialnumber[16];   ///< 128bit unique serial number (LSB)
} __attribute__((packed)) rplidar_resp_device_info_t;

/*!
* @brief RPLidar device health
*/
typedef struct rplidar_resp_device_health_t {
    uint8_t     status;         ///< RPLidar Health State (0: Good, 1: Warning, 2:Error)
    uint16_t    error_code;     ///< The related error code that caused a warning/error
} __attribute__((packed)) rplidar_resp_device_health_t;

/*!
* @brief RPLidar samplerate
*/
typedef struct rplidar_resp_samplerate_t {
    uint16_t    t_standard;     ///< The time used - in scan mode - when RPLIDAR takes a single laser ranging (in microseconds)
    uint16_t    t_express;      ///< The time used - in express mode - when RPLIDAR takes a single laser ranging (in microseconds)
} __attribute__((packed)) rplidar_resp_samplerate_t;
