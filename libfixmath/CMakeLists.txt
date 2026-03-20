# Copyright (c) 2025-2026 Tilmann Unte
# SPDX-License-Identifier:  Apache-2.0

if(CONFIG_LIBFIXMATH)
  # headers to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    fix16.c
    fix16_exp.c
    fix16_sqrt.c
    fix16_trig.c
    fract32.c
    uint32.c
    )
endif()
