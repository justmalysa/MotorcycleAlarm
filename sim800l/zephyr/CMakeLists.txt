# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_include_directories(${ZEPHYR_BASE}/drivers/modem)
zephyr_library()
zephyr_library_sources(sim800l.c)