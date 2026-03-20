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

static void transmit(struct patch_info *patch)
{
	uint8_t value;
	//printk("%s:", patch->name);
	while(fgetc(patch->fp) == ' ') {
		if (fscanf(patch->fp, "%02hhX", &value) == 1) {
			uart_poll_out(patch->dev, value);
			//printk("%02hhX ", value);
		}
	}
	//printk("\n");
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

int main(int argc, char* argv[])
{
	int count = 0;
	char name[10];
	FILE *test;
	
	/* An input file number value may be provided as command line argument */
	if (argc > 1) {
		count = atoi(argv[1]);
		if (count > 1000 || count < 0) return -1;
	}
	
	for (int i = count; i < 1000; i++) {
		snprintf(name, 10, "magni%i", i);
		if ((test = fopen(name, "r")) == NULL) {
			/* Filename does not exist, skip this one */
			fclose(test);
		} else {
			patch_magni.fp = fopen(name, "r");
			snprintf(name, 10, "lidar%i", i);
			patch_lidar.fp = fopen(name, "r");
			
			break;
		}
	}
	
	if (patch_magni.fp == NULL || patch_lidar.fp == NULL) return -1;
	
	if (signal(SIGINT, signal_handler) == SIG_ERR) {
		return -1;
	}

	int64_t current_time, magni_time = 0, lidar_time = 0;
	bool first_iteration = true;
	int ret; 
	
	for (;;) {
		/* will continue until SIGINT (Ctrl+C) is caught or EOF is reached*/

		if (magni_time == 0) {
			ret = fscanf(patch_magni.fp, "%lld:", &magni_time);
			if (ret != 1) {
				printk("alignment error on magni\n");
				break;
			}
		}

		if (lidar_time == 0) {
			ret = fscanf(patch_lidar.fp, "%lld:", &lidar_time);
			if (ret != 1) {	
				printk("alignment error on lidar\n");
				break;
			}
		}
		
		if (first_iteration) {
			printk("current time: %lld, magni start: %lld, lidar start: %lld\n", native_rtc_gettime_us(RTC_CLOCK_REALTIME), magni_time, lidar_time);
			int64_t start_time = min(magni_time, lidar_time);
			current_time = native_rtc_gettime_us(RTC_CLOCK_BOOT); // expected to be 0 since we have never waited
			native_rtc_offset(start_time);
			first_iteration = false;
			printk("start time: %lld\n", native_rtc_gettime_us(RTC_CLOCK_REALTIME));
		}
		
		current_time = native_rtc_gettime_us(RTC_CLOCK_REALTIME);
		
		if (magni_time != 0 && current_time >= magni_time) {
			transmit(&patch_magni);
			magni_time = 0;
		}
		if (lidar_time != 0 && current_time >= lidar_time) {
			transmit(&patch_lidar);
			lidar_time = 0;
		}
		
		// waiting is mandatory, otherwise the scheduler will never be called and the simulated system clock does not advance
		k_sleep(K_TICKS(1));
	}

	return 0;
}
