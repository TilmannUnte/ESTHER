# Copyright (c) 2021-2026 Tilmann Unte
# SPDX-License-Identifier:  Apache-2.0
if(CONFIG_UR_MAGNI_DRIVER)

  # Add headers to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    ur_magni.c
    ur_magni_msg.c
    )
endif()
