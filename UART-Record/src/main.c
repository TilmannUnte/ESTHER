/*
 * Copyright (c) 2025-2026 Tilmann Unte
 * SPDX-License-Identifier:  Apache-2.0
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "native_rtc.h"

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

struct patch_info {
	const uint8_t * const name;
	const struct device *dev;
	FILE *fp;
};

#define DEV_CONSOLE DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
#define DEV_MAGNI   DEVICE_DT_GET(DT_NODELABEL(uart0))
#define DEV_LIDAR   DEVICE_DT_GET(DT_NODELABEL(uart2))

struct patch_info patch_magni = {
	.name = "magni",
	.dev = DEV_MAGNI,
	.fp = NULL,
};

struct patch_info patch_lidar = {
	.name = "lidar",
	.dev = DEV_LIDAR,
	.fp = NULL,
};

static void receive(struct patch_info *patch)
{
	uint8_t value;
	bool first_iteration = true;
	
	while (uart_poll_in(patch->dev, &value) == 0) {
		uart_poll_out(patch->dev, value);
		// output current buffer content in the form "[timestamp]: [byte1], [...], [byten]\n"
		if (first_iteration) {
			int64_t us = native_rtc_gettime_us(RTC_CLOCK_BOOT);
			fprintf(patch->fp, "%lld:", us);
			first_iteration = false;
		}
		
		fprintf(patch->fp, " %02hhX", value);
	}

	if (!first_iteration) fprintf(patch->fp, "\n");
}

static void signal_handler(int signal)
{
	if (signal == SIGINT) {
		fflush(patch_magni.fp);
		fclose(patch_magni.fp);
		fflush(patch_lidar.fp);
		fclose(patch_lidar.fp);
		exit(0);
	}
}

int main(void)
{
	char name[10];
	FILE *test;
	
	for (int i = 0; i < 1000; i++) {
		snprintf(name, 10, "magni%i", i);
		if ((test = fopen(name, "r")) != NULL) {
			/* Filename already exists, skip this one */
			fclose(test);
		} else {
			patch_magni.fp = fopen(name, "w");
			snprintf(name, 10, "lidar%i", i);
			patch_lidar.fp = fopen(name, "w");
			
			break;
		}
	}
	
	if (patch_magni.fp == NULL || patch_lidar.fp == NULL) return -1;
	
	if (signal(SIGINT, signal_handler) == SIG_ERR) {
		return -1;
	}
	
	for (;;) {
		// sleep is mandatory, otherwise the scheduler is never called and the simulated system clock will never advance
		k_sleep(K_TICKS(1));
		receive(&patch_magni);
		receive(&patch_lidar);
	}

	return 0;
}
