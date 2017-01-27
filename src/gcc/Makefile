# -*- mode: makefile; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
#
#/*
# * @(#)Makefile
# *
# * Copyright (c) 2014 - 2015 Argusat Limited
# * 10 Underwood Road.  Southampton.  UK
# * All rights reserved.
# *
# * This software is the confidential and proprietary information of 
# * Argusat Limited. ("Confidential Information").  You shall not
# * disclose such Confidential Information and shall use it only in
# * accordance with the terms of the license agreement you entered into
# * with Argusat Limited.
# */

# List all source files the application uses.
APPLICATION_SRCS = $(notdir $(wildcard ../*.c)) \
    $(notdir $(wildcard ../ble_services/*.c))   \
    nrf_delay.c              \
    ble_srv_common.c         \
    ble_conn_params.c        \
    ble_flash.c              \
    ble_radio_notification.c \
    ble_sensorsim.c          \
    ble_stack_handler.c      \
    ble_bondmngr.c           \
    ble_advdata.c            \
    ble_bas.c                \
    ble_dis.c                \
    ble_ias.c                \
    ble_ess.c                \
    twi_hw_master_sd.c       \
    nrf_pwm.c                \
    app_timer.c              \
    app_gpiote.c             \
    app_button.c             \
    sht2x.c                  \
    main.c

#    ../nrf51822-i2c-sensors/nrf51822/I2Cdev/I2Cdev.cpp \
#    ../nrf51822-i2c-sensors/nrf51822/LM73/LM73.cpp

# Use shell to find name of root folder. Possible but horrible to do in make.
PROJECT_NAME = $(shell basename "$(realpath ../)")

DEVICE := NRF51
BOARD := BOARD_NRF6310
SEGGER_SERIAL = 518002256

USE_SOFTDEVICE := S110
#SOFTDEVICE=../s110_v7/s110_nrf51822_7.1.0_softdevice.hex
SOFTDEVICE=../s110_v5.2.1/s110_nrf51822_5.2.1_softdevice.hex

SOURCE_PATHS = ./ble_services \
    ../nrf51-sht2x-library    \
    ../nrf51-pwm-library

LIBRARY_PATHS = ./ble_services \
    ../nrf51-sht2x-library     \
    ../nrf51-pwm-library

SDK_PATH = ../../nordic_sdk/nrf51_sdk_4_4_2/nrf51822/
TEMPLATE_PATH = ../nrf51-pure-gcc-setup/template/

CFLAGS = -Os -Wno-unused-local-typedefs -I.

TERMINAL = ../term.sh
GDB_PORT_NUMBER = 2331

include $(TEMPLATE_PATH)Makefile
