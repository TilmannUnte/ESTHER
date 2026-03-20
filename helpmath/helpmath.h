/*
 * Copyright (c) 2025-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
 */

/**
 * @file
 * Helpmath is a library used to convert between Zephyr's struct sensor_value
 * and libfixmath fix16_t types. Accuracy loss is expected and ignored during
 * execution.
 *
 * This library may end up receiving more small helper functions if deemed
 * necessary. Look into the Zephyr API first, it provides a lot of helpers!
 */

#ifndef __HELPMATH_H__
#define __HELPMATH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include "fixmath.h"

/* Just for simplicity, no additional accuracy */
static const fix16_t PI_DIV_2 = 102944;
static const fix16_t PI_MUL_2 = 411774;
static const fix16_t PI_DIV_6_MUL_2 = 68629;
static const fix16_t PI_DIV_6_MUL_4 = 137258;

fix16_t sensor_value_to_fix16_t(struct sensor_value *in);
int sensor_value_from_fix16_t(struct sensor_value *out, fix16_t in);

fix16_t rad_to_deg(fix16_t r);
fix16_t deg_to_rad(fix16_t d);
fix16_t sensor_degrees_to_fix16_t_rad(struct sensor_value *in);
fix16_t normalize_rad(fix16_t r);
fix16_t rad_abs_diff(fix16_t a, fix16_t b);
fix16_t calculate_angle_rad(fix16_t x1, fix16_t y1, fix16_t x2, fix16_t y2);
int turning_direction(fix16_t start, fix16_t goal);
/* roots and logs are not suppoted by this pow function */
fix16_t fix16_pow(fix16_t base, uint8_t exp);
fix16_t fix16_abs(fix16_t in);
fix16_t fix16_euclid_dist(fix16_t x1, fix16_t y1, fix16_t x2, fix16_t y2);
#ifdef __cplusplus
}
#endif
#endif
