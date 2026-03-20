# Copyright (c) 2025-2026 Tilmann Unte
# SPDX-License-Identifier:  Apache-2.0

if(CONFIG_HELPMATH)
  # headers to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    helpmath.c
    )
endif()
