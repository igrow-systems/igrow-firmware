/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

#ifndef PEBBLE_ERROR_CODES_H__
#define PEBBLE_ERROR_CODES_H__

/*
 * @(#)main.c        
 *
 * Copyright (c) 2015 Argusat Limited
 * 10 Underwood Road,  Southampton.  UK
 * All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * Argusat Limited. ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with Argusat Limited.
 */


/** @file
 *
 * @defgroup igrow-nordic_pebble_errorcodes error_codes.h
 * @{
 * @ingroup igrow-nordic_pebble
 * @brief Error codes for iGrow Pebble
 *
 * This file contains library code for communicating with Sensiron's
 * range of humidity and temperature sensors.
 */

#include <stdint.h>
#include <stdbool.h>

#include "typedefs.h"

// sensor command
typedef enum {
  PEBBLE_TWI_INIT_FAILED                = 0x01, // Failed to bring up the TWI
  PEBBLE_SHT2X_INIT_FAILED              = 0x02, // Failed to bring up the SHT2x device
  PEBBLE_SHT2X_MEASURE_TEMP_FAILED      = 0x03, // Failed to read temperature from the SHT2x device
  PEBBLE_SHT2X_MEASURE_HUMIDITY_FAILED  = 0x04, // Failed to read humidity from the SHT2x device
  PEBBLE_SHT2X_TEMP_OUT_OF_RANGE        = 0x05  // Unable to represent the temperature as a 16 bit signed integer (! unlikely)
} pebble_error_code_t;


#endif // PEBBLE_ERROR_CODES_H__
