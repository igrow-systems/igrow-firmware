/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/*
 * @(#)main.c        
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
 * @defgroup igrow-nordic_pebble_main main.c
 * @{
 * @ingroup igrow-nordic_pebble_main
 * @brief iGrow 101 Bluetooth Smart humidity, sunlight and temperature logger.
 *
 * This file contains the source code for the iGrow Pebble 
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "ble_bondmngr.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_pwm.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_ess.h"
#include "ble_dis.h"
#include "ble_ias.h"
#include "ble_conn_params.h"
#ifdef HW_TARGET_NRF6310
  #include "ble_nrf6310_pins.h"
#else
  #include "igrow_101_pins.h"
#endif
#include "ble_sensorsim.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "app_button.h"
//#include "ble_error_log.h"
#include "ble_radio_notification.h"
#include "ble_flash.h"
#include "ble_debug_assert_handler.h"
#include "twi_master.h"
#include "sht2x.h"
#include "error_codes.h"

#include <limits.h>

#define DEVICE_NAME                          "iGrow Pebble"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                    "Argusat"                                  /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                            "PEBBLE101"                                /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                      0x1122334455                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                        0x667788                                   /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                     40                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS                 3                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define TEMP_TYPE_AS_CHARACTERISTIC          0                                          /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

#define MIN_CELCIUS_DEGREES                  3688                                       /**< Minimum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define MAX_CELCIUS_DEGRESS                  3972                                       /**< Maximum temperature in celcius for use in the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */
#define CELCIUS_DEGREES_INCREMENT            36                                         /**< Value by which temperature is incremented/decremented for each call to the simulated measurement function (multiplied by 100 to avoid floating point arithmetic). */

#define USE_SIMULATED_SENSORS                0                                         /**< Use simulated sensor measurements instead of hardware sensors.  Enables testing when running on dev kit or similar hardware without hardware sensors. */



#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds) */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS                 1                                          /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY               APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT                    30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define FLASH_PAGE_SYS_ATTR                 (BLE_FLASH_PAGE_END - 3)                    /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                     (BLE_FLASH_PAGE_END - 1)                    /**< Flash page used for bond manager bonding information. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_gap_sec_params_t                  m_sec_params;                              /**< Security requirements for this application. */
static ble_gap_adv_params_t                  m_adv_params;                              /**< Parameters to be passed to the stack when starting advertising. */
static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
static ble_ess_t                             m_ess;                                     /**< Structure used to identify the environmental sensing service. */
static ble_ias_t                             m_ias;                                     /**< Structure used to identify the Immediate Alert service. */
static ble_sensorsim_cfg_t                   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t                 m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static bool                                  m_ess_dvc_ind_conf_pending = false;        /**< Flag to keep track of when an indication confirmation is pending. */
static bool                                  m_is_advertising = false;                  /**< Flag to keep track of whether we are advertising or not. */

static ble_sensorsim_cfg_t                   m_temp_celcius_sim_cfg;                    /**< Temperature simulator configuration. */
static ble_sensorsim_state_t                 m_temp_celcius_sim_state;                  /**< Temperature simulator state. */

static app_timer_id_t                        m_battery_timer_id;                        /**< Battery timer. */
static ble_advdata_t                         m_advdata;                                 /**< Advertising Data */
static uint8_t                               m_adv_flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
static ble_advdata_service_data_t            m_adv_service_data;                        /**< Advertising additional Service Data */
static uint8_array_t                         m_ess_service_data_array;                  /**< PRN to indicate to Collector we have updated measurements */
static uint8_t                               m_ess_service_data_raw[2];                 /**< PRN to indicate to Collector we have updated measurements */


/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset
    //NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}

/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
}

static void advertising_update_service_data(void)
{
    uint32_t err_code;
    uint8_t num_rand_bytes_available;

    err_code = sd_rand_application_bytes_available_get(&num_rand_bytes_available);
    APP_ERROR_CHECK(err_code);

    if (num_rand_bytes_available > 1)
    {
        err_code = sd_rand_application_vector_get(m_ess_service_data_raw, 2);
        APP_ERROR_CHECK(err_code);
           
        err_code = ble_advdata_set(&m_advdata, NULL);
        APP_ERROR_CHECK(err_code);

    } else {

    }

}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;
    float    temp_celsius;
    float    humidity_rh;
    nt16     raw_temp;
    nt16     raw_humidity;

    bool result = sht2x_measure(SHT2X_TEMP, &raw_temp);
    if (!result)
    {
      APP_ERROR_HANDLER(PEBBLE_SHT2X_MEASURE_TEMP_FAILED);
    }
    temp_celsius = sht2x_calc_temp_celsius(raw_temp.u16);

    result = sht2x_measure(SHT2X_HUMIDITY, &raw_humidity);
    if (!result)
    {
      APP_ERROR_HANDLER(PEBBLE_SHT2X_MEASURE_HUMIDITY_FAILED);
    }
    humidity_rh = sht2x_calc_humidity_rh(raw_humidity.u16);
    // Update current service value
    long temp_celsius_long = temp_celsius >= 0 ? (long)(temp_celsius+0.5) : (long)(temp_celsius-0.5);
    if (temp_celsius_long <= INT_MAX && temp_celsius_long >= INT_MIN)
    {

        err_code = ble_ess_measurement_update(&m_ess, BLE_UUID_TEMPERATURE_CHAR, (int16_t*)&temp_celsius_long);

        switch (err_code)
        {
            case NRF_SUCCESS:

                break;
                
            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            case NRF_ERROR_DATA_SIZE:

                break;
            default:
                APP_ERROR_HANDLER(err_code);
        }
        // Update the historical data service
    }
    else
    {
        //
        APP_ERROR_HANDLER(PEBBLE_SHT2X_TEMP_OUT_OF_RANGE);
    }

    battery_level = (uint8_t)ble_sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
    
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }

    if (m_is_advertising)
    {
        advertising_stop();
    }
    
    advertising_update_service_data();

    if (m_is_advertising)
    {
        advertising_start();
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for populating simulated health thermometer measurement.
 */
static void ess_sim_measurement(int16_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };
    
    uint32_t celciusX100;
        
    celciusX100 = ble_sensorsim_measure(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);

    // +ve values of temperature only!
    *p_meas = (int16_t)celciusX100;

    // update simulated time stamp
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    GPIO_LED_CONFIG(ADVERTISING_LED_PIN_NO);
    GPIO_LED_CONFIG(CONNECTED_LED_PIN_NO);
    GPIO_LED_CONFIG(ASSERT_LED_PIN_NO);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;
    
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
    APP_ERROR_CHECK(err_code);
    
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    
    ble_uuid_t adv_uuids[] = 
    {
        {BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE, BLE_UUID_TYPE_BLE}
        //        {BLE_UUID_IMMEDIATE_ALERT_SERVICE,       BLE_UUID_TYPE_BLE},
        //        {BLE_UUID_BATTERY_SERVICE,               BLE_UUID_TYPE_BLE}, 
        //        {BLE_UUID_DEVICE_INFORMATION_SERVICE,    BLE_UUID_TYPE_BLE}
    };

    m_ess_service_data_raw[0] = (uint8_t)(0x08);
    m_ess_service_data_raw[1] = (uint8_t)(0x80);

    m_ess_service_data_array.size = 2;
    m_ess_service_data_array.p_data = m_ess_service_data_raw;

    m_adv_service_data.service_uuid = BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE;
    m_adv_service_data.data = m_ess_service_data_array;

    // Build and set advertising data
    memset(&m_advdata, 0, sizeof(m_advdata));
    
    m_advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    m_advdata.include_appearance      = true;
    m_advdata.flags.size              = sizeof(m_adv_flags);
    m_advdata.flags.p_data            = &m_adv_flags;
    m_advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    m_advdata.uuids_complete.p_uuids  = adv_uuids;
    m_advdata.p_service_data_array    = &m_adv_service_data;
    m_advdata.service_data_count      = 1;    

    err_code = ble_advdata_set(&m_advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising)
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

void set_frequency_and_duty_cycle(uint32_t frequency, uint32_t duty_cycle_percent)
{
  //nrf_pwm_set_max_value((16000000 + (frequency / 2)) / frequency);
  //    nrf_pwm_set_value(0, (16000000 / frequency) * duty_cycle_percent / 100);

}

/**@brief Function for signaling alert event from Immediate Alert or Link Loss services.
 *
 * @param[in]   alert_level  Requested alert level.
 */
static void alert_signal(uint8_t alert_level)
{
    switch (alert_level)
    {
        case BLE_CHAR_ALERT_LEVEL_NO_ALERT:

            break;

        case BLE_CHAR_ALERT_LEVEL_MILD_ALERT:
            break;

        case BLE_CHAR_ALERT_LEVEL_HIGH_ALERT:
          
            break;
            
        default:
            break;
    }
}

static void play_sample(void)
{
  for (int i = 0; i < 128; ++i)
  {
    //      nrf_pwm_set_value(0, 50);
    //      nrf_delay_us(256);
      //      nrf_pwm_set_value(0, 0);
    //      nrf_delay_us(3840);
  }
  //  set_frequency_and_duty_cycle((uint32_t)(440.0f + 0.5f), 50);
  //  nrf_delay_ms(1000);
  //  set_frequency_and_duty_cycle((uint32_t)(440.0f + 0.5f), 0);
}

/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(ble_ess_t* p_ess)
{
    int16_t simulated_meas = 137;
    uint32_t err_code;
    uint16_t ess_char_handle;

    ess_char_handle = p_ess->temperature_handles.value_handle;

    //    if (!m_ess_dvc_ind_conf_pending)
    //    {
        ess_sim_measurement(&simulated_meas);
        
        err_code = ble_ess_measurement_update(&m_ess, ess_char_handle, &simulated_meas);
        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
              //                m_ess_dvc_ind_conf_pending = true;
                break;
                
            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            case NRF_ERROR_DATA_SIZE:

                break;
            default:
                APP_ERROR_HANDLER(err_code);
        }
        //    }
        //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
    play_sample();

}


/**@brief Function for handling the Evironmental Sensing Service events.
 *
 * @details This function will be called for all Environmental Sensing Service events which are passed
 *          to the application.
 *
 * @param[in]   p_ess   Environmental Sensing Service stucture.
 * @param[in]   p_evt   Event received from the Environmental Sensing Service.
 */
static void on_ess_evt(ble_ess_t * p_ess, ble_ess_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_ESS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
          //            temperature_measurement_send();
            break;
        case BLE_ESS_EVT_INDICATION_DISABLED:
            // Indication has been disabled
            break;
        case BLE_ESS_EVT_NOTIFICATION_ENABLED:
            // Notification has been enabled, send a single temperature measurement
            temperature_measurement_send(p_ess);
            break;
        case BLE_ESS_EVT_NOTIFICATION_DISABLED:
            // Notification has been disabled
            break;
        case BLE_ESS_EVT_INDICATION_CONFIRMED:
            m_ess_dvc_ind_conf_pending = false;
            break;

        default:
            break;
    }
}

/**@brief Function for handling Immediate Alert events.
 *
 * @details This function will be called for all Immediate Alert events which are passed to the
 *          application.
 *
 * @param[in]   p_ias  Immediate Alert stucture.
 * @param[in]   p_evt  Event received from the Immediate Alert service.
 */
static void on_ias_evt(ble_ias_t * p_ias, ble_ias_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_IAS_EVT_ALERT_LEVEL_UPDATED:
            alert_signal(p_evt->params.alert_level);
            break;
            
        default:
            break;
    }
}


/**@brief Function for initializing the Immediate Alert Service.
 */
static void ias_init(void)
{
    uint32_t       err_code;
    ble_ias_init_t ias_init_obj;

    memset(&ias_init_obj, 0, sizeof(ias_init_obj));
    ias_init_obj.evt_handler = on_ias_evt;
    
    err_code = ble_ias_init(&m_ias, &ias_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Environmental Sensing Service
 */
static void ess_init(void)
{
    uint32_t       err_code;
    ble_ess_init_t ess_init_obj;

    memset(&ess_init_obj, 0, sizeof(ess_init_obj));
    ess_init_obj.evt_handler = on_ess_evt;
    
    // Here the sec level for the Environmental Sensing Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&ess_init_obj.ess_temperature_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ess_init_obj.ess_temperature_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&ess_init_obj.ess_temperature_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&ess_init_obj.ess_dvc_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&ess_init_obj.ess_dvc_attr_md.write_perm);

    err_code = ble_ess_init(&m_ess, &ess_init_obj);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Envrionmental Sensing, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_bas_init_t   bas_init;
    ble_dis_init_t   dis_init;
    ble_dis_sys_id_t sys_id;
    
    // Initialize Battery Service
    memset(&bas_init, 0, sizeof(bas_init));
    
    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    
    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Immediate Alert Service
    ias_init();

    // Initialize Envionmetal Sensing Service
    ess_init();
}

void pwm_init()
{
    nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;
    
    pwm_config.mode             = PWM_MODE_MTR_255;
    pwm_config.num_channels     = 1;
    pwm_config.gpio_num[0]      = PIEZO_PIN_NO;
    
    // Initialize the PWM library
    nrf_pwm_init(&pwm_config);    
}

/**@brief Function for initializing the sensor simulators.
 */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    // Temperature is in celcius (it is multiplied by 100 to avoid floating point arithmetic).
    m_temp_celcius_sim_cfg.min           = MIN_CELCIUS_DEGREES;
    m_temp_celcius_sim_cfg.max           = MAX_CELCIUS_DEGRESS;
    m_temp_celcius_sim_cfg.incr          = CELCIUS_DEGREES_INCREMENT;
    m_temp_celcius_sim_cfg.start_at_max  = false;

    ble_sensorsim_init(&m_temp_celcius_sim_state, &m_temp_celcius_sim_cfg);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_is_advertising = false;
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            // Start detecting button presses
            err_code = app_button_enable();
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            m_ess_dvc_ind_conf_pending = false;

            // Stop detecting button presses when not connected
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            // Since we are not in a connection and have not started advertising, store bonds
            err_code = ble_bondmngr_bonded_masters_store();
            APP_ERROR_CHECK(err_code);

            m_is_advertising = true;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                GPIO_WAKEUP_BUTTON_CONFIG(SEND_MEAS_BUTTON_PIN_NO);
                GPIO_WAKEUP_BUTTON_CONFIG(BONDMNGR_DELETE_BUTTON_PIN_NO);

                err_code = sd_power_system_off();    
            }
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            }
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_ess_on_ble_evt(&m_ess, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_bondmngr_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
  BLE_STACK_HANDLER_INIT(//NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION,
                         NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM,
                         //                         NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}


/**@brief Function for handling button events.
 *
 * @param[in]   pin_no   The pin number of the button pressed.
 */
static void button_event_handler(uint8_t pin_no)
{
    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
    switch (pin_no)
    {
        case SEND_MEAS_BUTTON_PIN_NO:
          temperature_measurement_send(&m_ess);
            break;
            
        default:
            APP_ERROR_HANDLER(pin_no);
    }
}


/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for initializing button module.
 */
static void buttons_init(void)
{
    static app_button_cfg_t buttons[] =
    {
        {SEND_MEAS_BUTTON_PIN_NO,       false, NRF_GPIO_PIN_NOPULL, button_event_handler},
        {BONDMNGR_DELETE_BUTTON_PIN_NO, false, NRF_GPIO_PIN_PULLUP, NULL}
    };
    
    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]),                            BUTTON_DETECTION_DELAY, false);
}


/**@brief Function for handling a Bond Manager error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the Bond Manager events.
 *
 * @param[in]   p_evt   Data associated to the bond manager event.
 */
static void bond_evt_handler(ble_bondmngr_evt_t * p_evt)
{
    uint32_t err_code;
    bool     is_notification_enabled;
    uint16_t ess_char_handle;

    ess_char_handle = BLE_GATT_HANDLE_INVALID;

    switch (p_evt->evt_type)
    {
        case BLE_BONDMNGR_EVT_ENCRYPTED:
            // Send a single temperature measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_ess_on_ble_evt() is called before
            //       ble_bondmngr_on_ble_evt() in ble_evt_dispatch().
            err_code = ble_ess_is_notification_enabled(&m_ess, ess_char_handle, &is_notification_enabled);
            APP_ERROR_CHECK(err_code);
            if (is_notification_enabled)
            {
                temperature_measurement_send(&m_ess);
            }
            break;
            
        default:
            break;
    }
}


/**@brief Function for the Bond Manager initialization.
 */
static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bond_init_data;
    bool                bonds_delete;

    // Clear all bonded masters if the Bonds Delete button is pushed
    err_code = app_button_is_pushed(BONDMNGR_DELETE_BUTTON_PIN_NO, &bonds_delete);
    APP_ERROR_CHECK(err_code);

    // Initialize the Bond Manager
    bond_init_data.flash_page_num_bond     = FLASH_PAGE_BOND;
    bond_init_data.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bond_init_data.evt_handler             = bond_evt_handler;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;

    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Radio Notification events.
 */
static void radio_notification_init(void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init(NRF_APP_PRIORITY_HIGH,
                                           NRF_RADIO_NOTIFICATION_DISTANCE_4560US,
                                           ble_flash_on_radio_active_evt);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_event_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool sht2x_init_succeeded;
    // Initialize.
    leds_init();
    timers_init();
    gpiote_init();
    buttons_init();
    if (!twi_master_init())
    {
        APP_ERROR_HANDLER(PEBBLE_TWI_INIT_FAILED);
    }

    sht2x_init_succeeded = sht2x_init();
    // If both failed to initialized, halt here.
    if (!sht2x_init_succeeded)
    {
        APP_ERROR_HANDLER(PEBBLE_SHT2X_INIT_FAILED);
    }

    ble_stack_init();
    bond_manager_init();
    gap_params_init();
    advertising_init();
    services_init();
    //    pwm_init();
    sensor_sim_init();
    conn_params_init();
    sec_params_init();
    radio_notification_init();

    // Start execution.
    application_timers_start();
    m_is_advertising = true;
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/** 
 * @}
 */
