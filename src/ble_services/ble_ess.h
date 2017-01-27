/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/*
 * @(#)ble_ess.h
 *
 * Copyright (c) 2015 - 2016 Argusat Limited
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
 * @defgroup ble_sdk_srv_ess Environmental Sensing Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Environmental Sensing Service module.
 *
 * @details This module implements the Environmental Sensing Service.
 *
 *          If an event handler is supplied by the application, the Environmental Sensing 
 *          Service will generate Environmental Sensing Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Environmental Sensing Service
 *       module by calling ble_ess_on_ble_evt() from the @ref softdevice_handler function.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_ESS_H__
#define BLE_ESS_H__

#include <stdint.h>
#include <stdbool.h>
#include <ble.h>

#include "ble_srv_common.h"
#include "ble_date_time.h"

#define BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE                      0x181A     /**< Environmental Sensing Service UUID. */

// Sampling Function
#define BLE_ESS_SAMPLING_FUNCTION_UNSPECIFIED      0x00
#define BLE_ESS_SAMPLING_FUNCTION_INSTANTANEOUS    0x01
#define BLE_ESS_SAMPLING_FUNCTION_ARITHMETIC_MEAN  0x02
#define BLE_ESS_SAMPLING_FUNCTION_RMS              0x03
#define BLE_ESS_SAMPLING_FUNCTION_MAXIMUM          0x04
#define BLE_ESS_SAMPLING_FUNCTION_MINIMUM          0x05
#define BLE_ESS_SAMPLING_FUNCTION_ACCUMULATED      0x06
#define BLE_ESS_SAMPLING_FUNCTION_COUNT            0x07

// Application

// Characteristic UUIDs

#define BLE_UUID_APPARENT_WIND_DIRECTION_CHAR                0x2A73     /**< Apparent Wind Direction characteristic UUID. */
#define BLE_UUID_APPARENT_WIND_SPEED_CHAR                    0x2A72     /**< Apparent Wind Speed characteristic UUID. */
#define BLE_UUID_DEW_POINT_CHAR                              0x2A7B     /**< Dew Point characteristic UUID. */
#define BLE_UUID_ELEVATION_CHAR                              0x2A6C     /**< Elevation characteristic UUID. */
#define BLE_UUID_GUST_FACTOR_CHAR                            0x2A74     /**< Gust Factor characteristic UUID. */
#define BLE_UUID_HEAT_INDEX_CHAR                             0x2A7A     /**< Heat Index characteristic UUID. */
#define BLE_UUID_HUMIDITY_CHAR                               0x2A6F     /**< Humidity characteristic UUID. */
#define BLE_UUID_IRRADIANCE_CHAR                             0x2A77     /**< Irradiance characteristic UUID. */
#define BLE_UUID_POLLEN_CONCENTRATION_CHAR                   0x2A75     /**< Pollen Concentration characteristic UUID. */
#define BLE_UUID_RAINFALL_CHAR                               0x2A78     /**< Rainfall characteristic UUID. */
#define BLE_UUID_PRESSURE_CHAR                               0x2A6D     /**< Pressure characteristic UUID. */
#define BLE_UUID_TEMPERATURE_CHAR                            0x2A6E     /**< Temperature characteristic UUID. */
#define BLE_UUID_TRUE_WIND_DIRECTION_CHAR                    0x2A71     /**< True Wind Direction characteristic UUID. */
#define BLE_UUID_TRUE_WIND_SPEED_CHAR                        0x2A70     /**< True Wind Speed characteristic UUID. */
#define BLE_UUID_UV_INDEX_CHAR                               0x2A76     /**< UV Index characteristic UUID. */
#define BLE_UUID_WIND_CHILL_CHAR                             0x2A79     /**< Wind Chill characteristic UUID. */
#define BLE_UUID_BAROMETRIC_PRESSURE_TREND_CHAR              0x2AA3     /**< Barometric Pressure Trend characteristic UUID. */
#define BLE_UUID_MAGNETIC_DECLINATION_CHAR                   0x2A2C     /**< Magnetic Declination characteristic UUID. */
#define BLE_UUID_MAGNETIC_FLUX_DENSITY_2D_CHAR               0x2AA0     /**< Magnetic Flux Density 2D characteristic UUID. */
#define BLE_UUID_MAGNETIC_FLUX_DENSITY_3D_CHAR               0x2AA1     /**< Magnetic Flux Density 3D characteristic UUID. */
#define BLE_UUID_DESC_VALUE_CHANGED_CHAR                     0x2A7D     /**< Descriptor Value Changed characteristic UUID. */

// Descriptor UUIDs

#define BLE_UUID_ES_MEASUREMENT_DESC                0x290C     /**< Environmental Sensing Measurement descriptor UUID. */
#define BLE_UUID_ES_CONFIGURATION_DESC              0x290B     /**< Environmental Sensing Configuration descriptor UUID. */
#define BLE_UUID_ES_TRIGGER_SETTING_DESC            0x2AA1     /**< Environmental Sensing Trigger Setting descriptor UUID. */
#define BLE_UUID_CHAR_USER_DESCRIPTION_DESC         0x2901     /**< Characteristic User Description descriptor UUID. Careful!  This is defined in ble_type.h in the Nordic SDK... */
#define BLE_UUID_VALID_RANGE_DESC                   0x2906     /**< Valid Range descriptor UUID. */

/**@brief Environmental Sensing Service event type. */
typedef enum
{
    BLE_ESS_EVT_INDICATION_ENABLED,                                         /**< Environmental Sensing value indication enabled event. */
    BLE_ESS_EVT_INDICATION_DISABLED,                                        /**< Environmental Sensing value indication disabled event. */
    BLE_ESS_EVT_INDICATION_CONFIRMED,                                       /**< Confirmation of a temperature measurement indication has been received. */
    BLE_ESS_EVT_NOTIFICATION_ENABLED,                                       /**< Environmental Sensing value notification enabled event. */
    BLE_ESS_EVT_NOTIFICATION_DISABLED,                                      /**< Environmental Sensing value notification disabled event. */
    BLE_ESS_EVT_DESCRIPTOR_VALUE_CHANGED                                    /**< A descriptor's value has changed.  */
} ble_ess_evt_type_t;

/**@brief Environmental Sensing Service ES Measurement Descriptor Sampling Function. */
typedef enum
{
    BLE_ESS_SMPF_UNSPECIFIED,                                                /**< Unspecified */
    BLE_ESS_SMPF_INSTANTANEOUS,                                              /**< Instataneous */
    BLE_ESS_SMPF_ARITHMETIC_MEAN,                                            /**< Arithmetic Mean */
    BLE_ESS_SMPF_RMS,                                                        /**< RMS */
    BLE_ESS_SMPF_MAXIMUM,                                                    /**< Maximum */
    BLE_ESS_SMPF_MINIMUM,                                                    /**< Minimum */
    BLE_ESS_SMPF_ACCUMULATED,                                                /**< Accumulated */
    BLE_ESS_SMPF_COUNT                                                      /**< Count */
                                                                            /**< 0x08 - 0xFF RFU */
} ble_ess_smpf_type_t;

/**@brief Environmental Sensing Service ES Measurement Descriptor Application. */
typedef enum
{
  BLE_ESS_APPLICATION_UNSPECIFIED,                          // 0x00
	BLE_ESS_APPLICATION_AIR,                                  // 0x01
	BLE_ESS_APPLICATION_WATER,                                // 0x02
	BLE_ESS_APPLICATION_BAROMETRIC,                           // 0x03
	BLE_ESS_APPLICATION_SOIL,                                 // 0x04
	BLE_ESS_APPLICATION_INFRARED,                             // 0x05
	BLE_ESS_APPLICATION_MAP_DATABASE,                         // 0x06
	BLE_ESS_APPLICATION_BAROMETRIC_ELEVATION_SOURCE,          // 0x07
	BLE_ESS_APPLICATION_GPS_ONLY_ELEVATION_SOURCE,            // 0x08
	BLE_ESS_APPLICATION_GPS_AND_MAP_DATABASE_ELEVATION_SOURCE,// 0x09
	BLE_ESS_APPLICATION_VERTICAL_DATUM_ELEVATION_SOURCE,      // 0x0A
	BLE_ESS_APPLICATION_ONSHORE,                              // 0x0B
	BLE_ESS_APPLICATION_ONBOARD_VESSEL_OR_VEHICLE,            // 0x0C
	BLE_ESS_APPLICATION_FRONT,                                // 0x0D
	BLE_ESS_APPLICATION_REAR,                                 // 0x0E
	BLE_ESS_APPLICATION_UPPER,                                // 0x0F
	BLE_ESS_APPLICATION_LOWER,                                // 0x10
	BLE_ESS_APPLICATION_PRIMARY,                              // 0x11
	BLE_ESS_APPLICATION_SECONDARY,                            // 0x12
	BLE_ESS_APPLICATION_OUTDOOR,                              // 0x13
	BLE_ESS_APPLICATION_INDOOR,                               // 0x14
	BLE_ESS_APPLICATION_TOP,                                  // 0x15
	BLE_ESS_APPLICATION_BOTTOM,                               // 0x16
	BLE_ESS_APPLICATION_MAIN,                                 // 0x17
	BLE_ESS_APPLICATION_BACKUP,                               // 0x18
	BLE_ESS_APPLICATION_AUXILLIARY,                           // 0x19
	BLE_ESS_APPLICATION_SUPPLEMENTARY,                        // 0x1A
	BLE_ESS_APPLICATION_INSIDE,                               // 0x1B
	BLE_ESS_APPLICATION_OUTSIDE,                              // 0x1C
	BLE_ESS_APPLICATION_LEFT,                                 // 0x1D
	BLE_ESS_APPLICATION_RIGHT,                                // 0x1E
	BLE_ESS_APPLICATION_INTERNAL,                             // 0x1F
	BLE_ESS_APPLICATION_EXTERNAL,                             // 0x20
	BLE_ESS_APPLICATION_SOLAR                                 // 0x21
} ble_ess_application_type_t;

/**@brief Environmental Sensing Service ES Trigger Setting Descriptor Condition. */
typedef enum
{
    BLE_ESS_CONDITION_INACTIVE,                                              /**< Trigger Inactive */
    BLE_ESS_CONDITION_FIXED_INTERVAL,                                        /**< Fixed Interval */
    BLE_ESS_CONDITION_MIN_INTERVAL,                                          /**< Minimum Interval */
    BLE_ESS_CONDITION_VALUE_CHANGED,                                         /**< Value Changed */
    BLE_ESS_CONDITION_LT,                                                    /**< Less Than */
    BLE_ESS_CONDITION_LTE,                                                   /**< Less Than or Equal */
    BLE_ESS_CONDITION_GT,                                                    /**< Greater Than */
    BLE_ESS_CONDITION_GTE,                                                   /**< Greater Than or Equal */
    BLE_ESS_CONDITION_EQ,                                                    /**< Equal */
    BLE_ESS_CONDITION_NE                                                     /**< Not Equal */
                                                                             /**< 0x0A - 0xFF RFU */
} ble_ess_condition_type_t;

/**@brief Environmental Sensing Service ES Cofiguration Descriptor Trigger Logic. */
typedef enum
{
    BLE_ESS_TRIG_LOGIC_AND,                                              /**< Boolean AND */
    BLE_ESS_TRIG_LOGIC_OR                                                /**< Boolean OR */
                                                                         /**< 0x02 - 0xFF RFU */
} ble_ess_trig_logic_type_t;

/**@brief Environmental Sensing Service event type. */
typedef struct
{
    ble_ess_evt_type_t evt_type;                                            /**< Type of event. */
} ble_ess_evt_t;


// Forward declaration of the ble_ess_t type. 
typedef struct ble_ess_s ble_ess_t;

/**@brief Environmental Sensing Service event handler type. */
typedef void (*ble_ess_evt_handler_t) (ble_ess_t * p_ess, ble_ess_evt_t * p_evt);

/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
{
  int8_t  exponent;                                                         /**< Base 10 exponent */
  int32_t mantissa;                                                         /**< Mantissa, should be using only 24 bits */
} ieee_float32_t;

/**@brief Environmental Sensing Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
    ble_ess_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Environmental Sensing Service. */
    ble_srv_cccd_security_mode_t ess_temperature_attr_md;                   /**< Initial security level for environmental sensing temperature attribute */
    int16_t                      init_temperature;                          /**< Initial value for environmental sensing temperature attribute */
    int16_t                      init_humidity;                             /**< Initial value for environmental sensing temperature attribute */
    ble_srv_cccd_security_mode_t ess_dvc_attr_md;                           /**< Initial security level for environmental sensing Descriptor Value Changed attribute */
} ble_ess_init_t;

/**@brief Environmental Sensing Service structure. This contains various status information for
 *        the service. */
typedef struct ble_ess_s
{
    ble_ess_evt_handler_t        evt_handler;                               /**< Event handler to be called for handling events in the Environmental Sensing Service. */
    uint16_t                     service_handle;                            /**< Handle of Environmental Sensing Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     temperature_handles;                       /**< Handles related to the Environmental Sensing Temperature Type characteristic. */
    ble_gatts_char_handles_t     humidity_handles;                          /**< Handles related to the Environmental Sensing Humidity characteristic. */
    ble_gatts_char_handles_t     irradiance_handles;                        /**< Handles related to the Environmental Sensing Irradiance characteristic. */
    ble_gatts_char_handles_t     dvc_handles;                               /**< Handles related to the Descriptor Value Changed characteristic. */
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    int16_t                      current_temperature;                       /**< Current temperature in degrees celcius E^2  */
} ble_ess_t;

/**@brief Descriptor Value Changed Characteristic structure. This contains an Environmental Sensing
 *        Descriptor Value Changed Characteristic. */
typedef struct ble_ess_dvc_char_s
{
    uint16_t                     flags;                                     /**< See ESS Bluetooth Specification 3.2.1.1. */
    ble_uuid_t                   characteristic_uuid;                       /**< The UUID of the characteristic with affected descriptor. */
} ble_ess_dvc_char_t;

/**@brief ES Measurement Descriptor structure. This contains an Environmental Sensing
 *        Measurement Descriptor. */
typedef struct ble_ess_meas_desc_s
{
    uint16_t                     flags;                                     /**< Reserved for Future Use or RFU */
    ble_ess_smpf_type_t          sampling_function;                         /**< Sampling Function used to compute value of ESS Characteristic */
    uint32_t                     measurement_period;                        /**< 0x000000 - Not in use. 0x000001 - 0xFFFFFF Time period in seconds */
    uint32_t                     internal_update_interval;                  /**< 0x000000 - Not in use. 0x000001 - 0xFFFFFF Time period in seconds */
    ble_ess_application_type_t   application;                               /**< Specific application of ESS Characteristic */
    uint8_t                      measurement_uncertainty;                   /**< Measurement Uncertainty, 0x00 - 0xFE Maximum error expressed as percentage of reported value. 0xFF Information not available */
} ble_ess_meas_desc_t;

/**@brief ES Trigger Setting Descriptor structure. This contains an Environmental Sensing
 *        Trigger Setting Descriptor. */
typedef struct ble_ess_trig_set_desc_s
{
  ble_ess_condition_type_t condition;                                 /**< Condition to trigger data transmission */
  uint16_t                 operand;                                   /**< Operand.  Format and size as per ES Characteristic */
} ble_ess_trig_set_desc_t;

/**@brief ES Configuration Descriptor structure.  */
typedef struct ble_ess_config_desc_s
{
  ble_ess_trig_logic_type_t trigger_logic;                         /**<  */
} ble_ess_config_desc_t;

/**@brief Characteristic User Description Descriptor structure. 
 *         */
typedef struct ble_ess_char_user_desc_desc_s
{
  ble_ess_trig_logic_type_t trigger_logic;                         /**<  */
} ble_ess_char_user_desc_desc_t;



/**@brief Function for initializing the Environmental Sensing Service.
 *
 * @param[out]  p_ess       Environmental Sensing Service structure. This structure will have to
 *                          be supplied by the application. It will be initialized by this function,
 *                          and will later be used to identify this particular service instance.
 * @param[in]   p_ess_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Environmental Sensing Service.
 *
 * @param[in]   p_ess      Environmental Sensing Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ess_on_ble_evt(ble_ess_t * p_ess, ble_evt_t * p_ble_evt);

/**@brief Function for sending evironmental measurement if notification has been enabled.
 *
 * @details The application calls this function after having performed a Environmental Sensing
 *          measurement. If indication has been enabled, the measurement data is encoded and
 *          sent to the client.
 *
 * @param[in]   p_ess           Environmental Sensing Service structure.
 * @param[in]   ess_char_handle Handle of ESS Characteristic to update.
 * @param[in]   p_ess_meas      Pointer to new environmental sensing measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ess_measurement_update(ble_ess_t * p_ess, uint16_t ess_char_handle, int16_t * p_ess_meas);

/**@brief Function for checking if indication of Descriptor Value Changed is currently enabled.
 *
 * @param[in]   p_ess                  Environmental Sensing Service structure.
 * @param[in]   ess_char_handle        Handle of ESS Characteristic to enable indications for.
 * @param[out]  p_indication_enabled   TRUE if indication is enabled, FALSE otherwise.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ess_is_indication_enabled(ble_ess_t * p_ess, uint16_t ess_char_handle, bool * p_indication_enabled);

/**@brief Function for checking if notification of Temperature Measurement is currently enabled.
 *
 * @param[in]   p_ess                  Environmental Sensing Service structure.
 * @param[in]   ess_char_handle        Handle of ESS Characteristic to enable notifications for.
 * @param[out]  p_indication_enabled   TRUE if indication is enabled, FALSE otherwise.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_ess_is_notification_enabled(ble_ess_t * p_ess, uint16_t ess_char_handle, bool * p_notification_enabled);

#endif // BLE_ESS_H__

/** @} */
