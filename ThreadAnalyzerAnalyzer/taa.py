# Copyright (c) 2026 Tilmann Unte
# SPDX-License-Identifier:  Apache-2.0

from os import listdir
from math import inf, isnan, ceil
import re
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import scienceplots
import numpy as np

plt.style.use(['science', 'ieee'])
#plt.rc('text', usetex=True)
#plt.rcParams['font.size'] = 10

input_dir = "./"
file_list = listdir(input_dir)
for input_file in file_list:
	input_file = str(input_file)
	if ".txt" not in input_file:
		file_list.remove(input_file)
	if ".pdf" in input_file:
		file_list.remove(input_file)
file_list = sorted(file_list)

xmc45_hz = 120000000
l475_hz = 80000000
stm32f4_hz = 168000000
bluepill_hz = 72000000
esp32_hz = 240000000 
esp32c6_hz = 160000000
portenta_h7_hz = 400000000

# color-blind friendly palette, from cold to warm-toned
colorsGlobal = ['#332288', '#88CCEE', '#44AA99', '#117733', '#999933', '#DDCC77', '#CC6677', '#882255', '#AA4499', '#EE3377', '#CC3311', '#EE7733']

# various gray tones
colorMisc =  ['#DDDDDD']
colorMisc2 =   ['#BBBBBB']
colorMisc3 = ['#111111']

colorMatch = {
	"portenta": "#ddcc77", #999933
	"l475": "#88ccee",
	"esp32": "#cc3311",
	"stm32f4": "#117733",
	"min_dev": "#332288",
	"xmc45": "#44AA99",
}

hatchMatch = {
	"Robot UART RX":"//",
	"Robot UART TX":"\\\\",
	"Robot Parser":"||",
	"Robot Velocity":"..",
	"Robot Odometry":"oo",
	"Robot System":"OO",
	"LiDAR UART RX":"++",
	"LiDAR UART TX":"xx",
	"LiDAR Parser":"**",
	"Robot Movement":"O.",
	"LiDAR Sweep":"*-",
	"Application":"o-",
	"Logging":"\\|",
	"Overhead":"-\\",
}

periodic_index = ["Robot UART RX", "Robot UART TX","Robot Parser", "Robot Velocity", "Robot Odometry", "Robot System","LiDAR UART RX", "LiDAR UART TX", "LiDAR Parser", "Robot Movement", "LiDAR Sweep", "Application", "Logging", "Overhead"]
periodic_fpps_mean = pd.DataFrame({"strictly periodic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0], "periodic & sporadic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, index=periodic_index)
periodic_fpps_std = pd.DataFrame({"strictly periodic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0], "periodic & sporadic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, index=periodic_index)
periodic_edf_mean = pd.DataFrame({"strictly periodic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0], "periodic & sporadic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, index=periodic_index)
periodic_edf_std = pd.DataFrame({"strictly periodic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0], "periodic & sporadic":[0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, index=periodic_index)

colorStrict = ["#999933"]
colorSporadic = ["#ddcc77"]

bargraph_index = ["Robot Parser", "Robot Velocity", "Robot Odometry", "Robot System", "LiDAR Parser", "Robot Movement", "LiDAR Sweep", "Application", "Logging", "Overhead"]
fpps_mean = pd.DataFrame({"min_dev":[0,0,0,0,0,0,0,0,0,0], "l475":[0,0,0,0,0,0,0,0,0,0], "stm32f4":[0,0,0,0,0,0,0,0,0,0], "xmc45":[0,0,0,0,0,0,0,0,0,0], "esp32":[0,0,0,0,0,0,0,0,0,0], "portenta":[0,0,0,0,0,0,0,0,0,0]}, index=bargraph_index)
edf_mean = pd.DataFrame({"l475":[0,0,0,0,0,0,0,0,0,0], "stm32f4":[0,0,0,0,0,0,0,0,0,0], "xmc45":[0,0,0,0,0,0,0,0,0,0], "esp32":[0,0,0,0,0,0,0,0,0,0], "portenta":[0,0,0,0,0,0,0,0,0,0]}, index=bargraph_index)
fpps_std = pd.DataFrame({"min_dev":[0,0,0,0,0,0,0,0,0,0], "l475":[0,0,0,0,0,0,0,0,0,0], "stm32f4":[0,0,0,0,0,0,0,0,0,0], "xmc45":[0,0,0,0,0,0,0,0,0,0], "esp32":[0,0,0,0,0,0,0,0,0,0], "portenta":[0,0,0,0,0,0,0,0,0,0]}, index=bargraph_index)
edf_std = pd.DataFrame({"l475":[0,0,0,0,0,0,0,0,0,0], "stm32f4":[0,0,0,0,0,0,0,0,0,0], "xmc45":[0,0,0,0,0,0,0,0,0,0], "esp32":[0,0,0,0,0,0,0,0,0,0], "portenta":[0,0,0,0,0,0,0,0,0,0]}, index=bargraph_index)

for name in file_list:

	data = pd.DataFrame(
		[[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]],
		index = ["Robot UART RX", "Robot UART TX", "Robot Parser", "Robot Velocity", "Robot Odometry", "Robot System", "LiDAR UART RX", "LiDAR UART TX", "LiDAR Parser", "Robot Movement", "LiDAR Sweep", "Application", "Logging", "Overhead", "Idle"],
		columns = ["Rate", "Hyperperiod0", "Hyperperiod1", "Hyperperiod2", "Hyperperiod3", "Hyperperiod4", "Hyperperiod5", "Hyperperiod6", "Hyperperiod7", "Hyperperiod8", "Hyperperiod9", "Hyperperiod10", "Hyperperiod11", "Hyperperiod12", "Hyperperiod13", "Hyperperiod14", "Mean", "StD", "Job"],
	)

	data.loc["Robot Movement", "Rate"] = 50
	data.loc["Application", "Rate"] = 50
	data.loc["LiDAR Sweep", "Rate"] = 250
	data.loc["LiDAR Parser", "Rate"] = 4
	data.loc["LiDAR UART RX", "Rate"] = 0.04
	data.loc["LiDAR UART TX", "Rate"] = 0.08
	data.loc["Robot UART RX", "Rate"] = 0.13
	data.loc["Robot UART TX", "Rate"] = 0.26
	data.loc["Robot Odometry", "Rate"] = 250
	data.loc["Robot Velocity", "Rate"] = 50
	data.loc["Robot System", "Rate"] = 60000
	data.loc["Robot Parser", "Rate"] = 2

	f = open(input_dir + name, "r")

	print(name)

	clock_speed = 0
	if "xmc45" in name:
		clock_speed = xmc45_hz
	elif "l475" in name:
		clock_speed = l475_hz
	elif "stm32f4" in name:
		clock_speed = stm32f4_hz
	elif "min_dev" in name:
		clock_speed = bluepill_hz
	elif "esp32_" in name:
		clock_speed = esp32_hz
	elif "esp32c6" in name:
		clock_speed = esp32c6_hz
	elif "portenta" in name:
		clock_speed = portenta_h7_hz
	else:
		clock_speed = 0
	
	ms_per_min = 60000
	
	current_thread = ""
	current_period = -1
	wc_errors_per_period = 0
	errors_per_period = 0

	for line in f.readlines():
		if current_period != -1 and "checksum" in line:
			errors_per_period += 1

		if "min_dev" not in name and "thread_analyzer:" not in line:
			continue

		if "movement" in line:
			wc_errors_per_period = max(wc_errors_per_period, errors_per_period)
			errors_per_period = 0
			if current_period == 14:
				break
			else:
				current_period += 1
			current_thread = "Robot Movement"
		elif "magni_tcb" in line:
			current_thread = "Application"
		elif "lidar_tcb" in line:
			current_thread = "LiDAR Sweep"
		elif "rplidar_unpacker" in line:
			current_thread = "LiDAR Parser"
		elif "ur_magni_joint" in line:
			current_thread = "Robot Odometry"
		elif "ur_magni_velocity" in line:
			current_thread = "Robot Velocity"
		elif "ur_magni_system" in line:
			current_thread = "Robot System"
		elif "ur_magni_parser" in line:
			current_thread = "Robot Parser"
		elif "logging" in line:
			current_thread = "Logging"
		elif "idle" in line:
			current_thread = "Idle"
		elif "rplidar_uart_tx" in line:
			current_thread = "LiDAR UART TX"
		elif "rplidar_uart_rx" in line:
			current_thread = "LiDAR UART RX"
		elif "ur_magni_uart_tx" in line:
			current_thread = "Robot UART TX"
		elif "ur_magni_uart_rx" in line:
			current_thread = "Robot UART RX"

		if "Total CPU cycles used:" in line:
			total_cycles = int(re.compile("(\\d+)").match(line.split(' ')[-1]).group(1))
			#print(str(current_period) + " " + current_thread + " " + str(total_cycles))
			data.loc[current_thread, "Hyperperiod" + str(current_period)] = total_cycles

	print("Worst case messages lost per hyperperiod: " + str(wc_errors_per_period))

	period_amount = current_period + 1
	
	# Determine execution time totals per Hyperperiod (as opposed to: since boot)
	while current_period > 0:
		for thread in data.index:
			if "Overhead" not in thread:
				data.loc[thread, "Hyperperiod" + str(current_period)] -= data.loc[thread, "Hyperperiod" + str(current_period - 1)]
		current_period -= 1
	
	# Determine Overhead based on hyperperiod duration - all thread activity
	for i in range(period_amount):
		overhead_cycles = clock_speed * 60

		for thread in data.index:
			if "Overhead" not in thread:
				overhead_cycles -= data.loc[thread, "Hyperperiod" + str(i)]

		data.loc["Overhead", "Hyperperiod" + str(i)] = overhead_cycles

	# Determine absolute mean and std of warm hyperperiods, and approximates average execution time of each job of each thread
	for thread in data.index:
		data.loc[thread, "Mean"] = np.mean(data.loc[thread, "Hyperperiod1":"Hyperperiod14"])
		data.loc[thread, "StD"] = np.std(data.loc[thread, "Hyperperiod1":"Hyperperiod14"])

		if data.loc[thread, "Rate"] != 0:
			data.loc[thread, "Job"] = (data.loc[thread, "Mean"] / (ms_per_min / data.loc[thread, "Rate"]) / clock_speed) * 1000

	# Determine Utilization
	for period in range(period_amount):
		for thread in data.index:
			data.loc[thread, "Hyperperiod" + str(period)] = (data.loc[thread, "Hyperperiod" + str(period)] / (clock_speed * 60)) * 100

	for thread in data.index:
		data.loc[thread, "Mean"] = np.mean(data.loc[thread, "Hyperperiod1":"Hyperperiod14"])
		data.loc[thread, "StD"] = np.std(data.loc[thread, "Hyperperiod1":"Hyperperiod14"])

	# Discard invalid or unreliable data
	if wc_errors_per_period < 10 and "esp32c6" not in name and "skipnext" not in name:
		no_idle = data.drop("Idle", inplace=False)
		if "polling" not in name:
			no_idle.drop(["LiDAR UART TX", "LiDAR UART RX", "Robot UART TX", "Robot UART RX"], inplace=True)
			if "min_dev" in name:
				no_idle.drop("Logging", inplace=True)
		#elif "suspend" in name:
			#no_idle.drop("LiDAR UART TX", inplace=True)

		print(no_idle["Job"])
		
		if "isr" in name:
			if "edf" in name:
				if "min_dev" in name:
					edf_mean["min_dev"] = no_idle["Mean"]
					edf_std["min_dev"] = no_idle["StD"]
				elif "l475" in name:
					edf_mean["l475"] = no_idle["Mean"]
					edf_std["l475"] = no_idle["StD"]
				elif "stm32f4" in name:
					edf_mean["stm32f4"] = no_idle["Mean"]
					edf_std["stm32f4"] = no_idle["StD"]
				elif "xmc45" in name:
					edf_mean["xmc45"] = no_idle["Mean"]
					edf_std["xmc45"] = no_idle["StD"]
				elif "esp32" in name:
					edf_mean["esp32"] = no_idle["Mean"]
					edf_std["esp32"] = no_idle["StD"]
				elif "portenta" in name:
					edf_mean["portenta"] = no_idle["Mean"]
					edf_std["portenta"] = no_idle["StD"]
			elif "fpps" in name:
				if "min_dev" in name:
					fpps_mean["min_dev"] = no_idle["Mean"]
					fpps_std["min_dev"] = no_idle["StD"]
				elif "l475" in name:
					fpps_mean["l475"] = no_idle["Mean"]
					fpps_std["l475"] = no_idle["StD"]
				elif "stm32f4" in name:
					fpps_mean["stm32f4"] = no_idle["Mean"]
					fpps_std["stm32f4"] = no_idle["StD"]
				elif "xmc45" in name:
					fpps_mean["xmc45"] = no_idle["Mean"]
					fpps_std["xmc45"] = no_idle["StD"]
				elif "esp32" in name:
					fpps_mean["esp32"] = no_idle["Mean"]
					fpps_std["esp32"] = no_idle["StD"]
				elif "portenta" in name:
					fpps_mean["portenta"] = no_idle["Mean"]
					fpps_std["portenta"] = no_idle["StD"]
		elif "polling" in name:
			if "fpps" in name and "portenta" in name:
				if "suspend" in name:
					periodic_fpps_mean["periodic & sporadic"] = no_idle["Mean"]
					periodic_fpps_std["periodic & sporadic"] = no_idle["StD"]
				else:
					periodic_fpps_mean["strictly periodic"] = no_idle["Mean"]
					periodic_fpps_std["strictly periodic"] = no_idle["StD"]
		#if "polling" in name:
		#	fig, ax = plt.subplots()
		#	fig.set_figwidth(3.5)
		#	bars = ax.barh(no_idle.index, no_idle["Mean"], xerr=no_idle["StD"], align="center", color=[colorMatch["portenta"]])
		#	ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
		#	ax.invert_yaxis()
		#	ax.set_xlim(0,25)
		#	ax.set_xlabel("Mean Utilization\n(Total: <" + str(int(ceil(sum(no_idle["Mean"])))) + "%)")
		#	#ax.set_title(name)
		#	plt.savefig(name + ".pdf", bbox_inches= "tight", format="pdf")
		#	#plt.show()
#print(periodic_fpps_mean.sum())
#print(fpps_std)
print(fpps_mean.sum())
print(edf_mean.sum())

produce_figures = True
if produce_figures is True:
	### Polling bar plot
	ind = np.arange(len(periodic_fpps_mean))
	print(ind)
	width = 0.4

	fig, ax = plt.subplots()
	# plt.barh(periodic_index,periodic_fpps_mean["strictly periodic"],xerr=periodic_fpps_std["strictly periodic"],color=colorStrict, label="strictly periodic")

	br0 = np.arange(len(periodic_index))
	br1 = [x + 0.2 for x in br0]
	br2 = [x + width for x in br1]

	bars = ax.barh(br1, periodic_fpps_mean["strictly periodic"], width, xerr=periodic_fpps_std["strictly periodic"], color=colorStrict, label="strictly periodic")
	ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) for x in bars.datavalues], fontsize = 8)
	bars = ax.barh(br2, periodic_fpps_mean["periodic & sporadic"], width, xerr=periodic_fpps_std["periodic & sporadic"], color=colorSporadic, label="periodic \& sporadic")
	ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) for x in bars.datavalues], fontsize = 8)
	# ax.set(yticks=ind + (2 * width)/3, yticklabels=periodic_index, ylim=[3*width - 1, len(periodic_fpps_mean)])
	plt.yticks([r + width for r in range(len(periodic_index))], periodic_index)
	fig.set_figwidth(7.16)
	fig.set_figheight(7.16/5.0*3.0)
	ax.legend()
	ax.invert_yaxis()
	ax.spines['top'].set_visible(False)
	ax.spines['right'].set_visible(False)
	plt.tick_params(axis='y', which="minor", left=False, right=False)
	plt.tick_params(axis='y', which="major", left=True, right=False)
	#ax.set_title("FPP Scheduling")
	ax.set_xlabel("Mean Utilization \%")
	plt.savefig("periodic.pdf", bbox_inches= "tight", format="pdf")


	### ISR EDF Plot
	ind = np.arange(len(edf_mean))
	width = 0.15

	fig, ax = plt.subplots()
	bars = ax.barh(ind + width/2, edf_mean["min_dev"], width, xerr=edf_std["min_dev"], color=[colorMatch["min_dev"]], label="min_dev")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 1 * width + width/2, edf_mean["l475"], width, xerr=edf_std["l475"], color=[colorMatch["l475"]], label="l475")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 2 * width + width/2, edf_mean["stm32f4"], width, xerr=edf_std["stm32f4"], color=[colorMatch["stm32f4"]], label="stm32f4")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 3 * width + width/2, edf_mean["xmc45"], width, xerr=edf_std["xmc45"], color=[colorMatch["xmc45"]], label="xmc45")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 4 * width + width/2, edf_mean["esp32"], width, xerr=edf_std["esp32"], color=[colorMatch["esp32"]], label="esp32")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 5 * width + width/2, edf_mean["portenta"], width, xerr=edf_std["portenta"], color=[colorMatch["portenta"]], label="portenta")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	ax.set(yticks=ind + (6 * width)/2, yticklabels=bargraph_index, ylim=[6*width - 1, len(edf_mean)])
	fig.set_figwidth(7.16)
	ax.legend()
	ax.set_xlim(right=15)
	ax.invert_yaxis()
	ax.spines['top'].set_visible(False)
	ax.spines['right'].set_visible(False)
	#ax.yaxis.set_minor_locator(MultipleLocator(0.5))
	plt.tick_params(axis='y', which="major", left=True, right=False)
	plt.tick_params(axis='y', which="minor", left=False, right=False)
	#ax.set_title("EDF Scheduling")
	ax.set_xlabel("Mean Utilization \%")
	plt.savefig("edf.pdf", bbox_inches= "tight", format="pdf")

	### ISR FPPS Plot
	ind = np.arange(len(fpps_mean))
	width = 0.15

	fig, ax = plt.subplots()
	bars = ax.barh(ind + width/2, fpps_mean["min_dev"], width, xerr=fpps_std["min_dev"], color=[colorMatch["min_dev"]], label="min_dev")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + width + width/2, fpps_mean["l475"], width, xerr=fpps_std["l475"], color=[colorMatch["l475"]], label="l475")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 2 * width + width/2, fpps_mean["stm32f4"], width, xerr=fpps_std["stm32f4"], color=[colorMatch["stm32f4"]], label="stm32f4")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 3 * width + width/2, fpps_mean["xmc45"], width, xerr=fpps_std["xmc45"], color=[colorMatch["xmc45"]], label="xmc45")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 4 * width + width/2, fpps_mean["esp32"], width, xerr=fpps_std["esp32"], color=[colorMatch["esp32"]], label="esp32")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	bars = ax.barh(ind + 5 * width + width/2, fpps_mean["portenta"], width, xerr=fpps_std["portenta"], color=[colorMatch["portenta"]], label="portenta")
	#ax.bar_label(bars, labels=[str(int(x * 1000) / 1000.0) if x < 1 else "" for x in bars.datavalues])
	ax.set(yticks=ind + (6 * width)/2, yticklabels=bargraph_index, ylim=[6*width - 1, len(fpps_mean)])
	fig.set_figwidth(7.16)
	ax.legend()
	ax.invert_yaxis()
	ax.set_xlim(right=15)
	ax.spines['top'].set_visible(False)
	ax.spines['right'].set_visible(False)
	#ax.yaxis.set_minor_locator(MultipleLocator(0.5))
	plt.tick_params(axis='y', which="major", left=True, right=False)
	plt.tick_params(axis='y', which="minor", left=False, right=False)
	#ax.set_title("FPP Scheduling")
	ax.set_xlabel("Mean Utilization \%")
	plt.savefig("fpp.pdf", bbox_inches= "tight", format="pdf")
