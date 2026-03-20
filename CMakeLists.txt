# Copyright (c) 2021-2026 Tilmann Unte
# SPDX-License-Identifier:  Apache-2.0

list(APPEND ZEPHYR_EXTRA_MODULES
   ${CMAKE_CURRENT_SOURCE_DIR}/ur_magni
   ${CMAKE_CURRENT_SOURCE_DIR}/rplidar
   ${CMAKE_CURRENT_SOURCE_DIR}/libfixmath
   ${CMAKE_CURRENT_SOURCE_DIR}/helpmath
   ${CMAKE_CURRENT_SOURCE_DIR}/kalman
   ${CMAKE_CURRENT_SOURCE_DIR}/movement
)

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(magni_sample)

target_sources(app PRIVATE
  src/main.c
  )
