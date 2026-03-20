/*
 * Copyright (c) 2025-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/

#include <zephyr/logging/log.h>

#include "helpmath.h"

LOG_MODULE_REGISTER(helpmath);

fix16_t sensor_value_to_fix16_t(struct sensor_value *in)
{
	fix16_t out = 0;
	
	int64_t big = sensor_value_to_micro(in);
	big *= fix16_one;
	big /= 1000000;
	
	if (big > fix16_max) {
		out = fix16_max;
	} else if (big < fix16_min) {
		out = fix16_min;
	} else {
		out = (fix16_t) big;
	}
	
	return out;
}

int sensor_value_from_fix16_t(struct sensor_value *out, fix16_t in)
{
	int64_t big = in;
	
	big *= 1000000;
	big /= fix16_one;
	
	return sensor_value_from_micro(out, big);
}

fix16_t rad_to_deg(fix16_t r)
{
	return fix16_div(fix16_mul(r, fix16_from_int(180)), fix16_pi);
}

fix16_t deg_to_rad(fix16_t d)
{
	return fix16_div(fix16_mul(d, fix16_pi), fix16_from_int(180));
}


fix16_t sensor_degrees_to_fix16_t_rad(struct sensor_value *in)
{
	fix16_t deg = sensor_value_to_fix16_t(in);
	
	return deg_to_rad(deg);
}

fix16_t normalize_rad(fix16_t r)
{
	while (r > PI_MUL_2) r = fix16_sub(r, PI_MUL_2);
	
	while (r < 0) r = fix16_add(r, PI_MUL_2);
	
	return r;
}

fix16_t rad_abs_diff(fix16_t a, fix16_t b)
{
	a = normalize_rad(a);
	b = normalize_rad(b);

	fix16_t diff = normalize_rad(fix16_sub(a, b));
	return min(diff, fix16_sub(PI_MUL_2, diff));
}

fix16_t calculate_angle_rad(fix16_t x1, fix16_t y1, fix16_t x2, fix16_t y2)
{
	return fix16_atan2(
			fix16_sub(y2, y1),
			fix16_sub(x2, x1)
		);
}

int turning_direction(fix16_t start, fix16_t goal)
{
	int dir;
	
	if (start < goal) {
		if (fix16_abs(fix16_sub(start, goal)) < fix16_pi) {
			dir = 1;
		} else {
			dir = -1;
		}
	} else {
		if (fix16_abs(fix16_sub(start, goal)) < fix16_pi) {
			dir = -1;
		} else {
			dir = 1;
		}
	}
	
	return dir;
}

fix16_t fix16_pow(fix16_t base, uint8_t exp)
{
	fix16_t out = fix16_one;
	for (; exp > 0; exp--) {
		out = fix16_mul(out, base);
	}
	
	return out;
}

fix16_t fix16_abs(fix16_t in)
{
	return in < 0 ? -in : in;
}

fix16_t fix16_euclid_dist(fix16_t x1, fix16_t y1, fix16_t x2, fix16_t y2)
{
	fix16_t a = fix16_pow(fix16_sub(x1, x2), 2);
	fix16_t b = fix16_pow(fix16_sub(y1, y2), 2);
	
	return fix16_sqrt(fix16_add(a, b));
}

