#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

FILE(GLOB app_sources src/*.c)
# NORDIC SDK APP START
target_sources(app PRIVATE ${app_sources})
# NORDIC SDK APP END

# USB HID support - include common USB sample helpers if using new stack
set(SAMPLE_USBD_DIR ${ZEPHYR_BASE}/samples/subsys/usb/common)
target_include_directories(app PRIVATE ${SAMPLE_USBD_DIR})
target_sources_ifdef(CONFIG_USB_DEVICE_STACK_NEXT app PRIVATE
	${SAMPLE_USBD_DIR}/sample_usbd_init.c
)
