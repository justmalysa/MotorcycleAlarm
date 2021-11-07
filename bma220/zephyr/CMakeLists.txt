# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_library()
zephyr_library_sources(bma220.c)
zephyr_library_sources_ifdef(CONFIG_BMA220_TRIGGER bma220_trigger.c)