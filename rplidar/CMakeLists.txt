# Copyright (c) 2022 Vinzenz Malke
# SPDX-License-Identifier:  Apache-2.0

if (CONFIG_RPLIDAR)
zephyr_include_directories(.)
zephyr_library()
zephyr_library_sources(
    rplidar.c
)
endif()
