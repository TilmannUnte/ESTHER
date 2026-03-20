# Copyright (c) 2025-2026 Tilmann Unte
# SPDX-License-Identifier:  Apache-2.0

if(CONFIG_MOVEMENT)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    movement.c
    )
endif()
