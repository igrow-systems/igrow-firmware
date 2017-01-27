/* -*- mode: c; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

/*
 * @(#)ble_ess.c
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
 * @defgroup igrow-nordic_pebble_main main.c
 * @{
 * @ingroup igrow-nordic_pebble_main
 * @brief iGrow 101 Bluetooth Smart humidity, sunlight and temperature logger.
 *
 * This file contains the source code for the iGrow Pebble 
 * This application uses the @ref srvlib_conn_params module.
 */

#include "ble_ess.h"

#include "typedefs.h"  // from nrf-sht2x-library for nt16

#include "nordic_common.h"
#include "ble_l2cap.h"
#include "ble_srv_common.h"
#include "app_util.h"

#include <string.h>

#define OPCODE_LENGTH 1                                                    /**< Length of opcode inside Environmental Sensing Measurement packet. */
#define HANDLE_LENGTH 2                                                    /**< Length of handle inside Environmental Sensing Measurement packet. */
#define MAX_ESM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)  /**< Maximum size of a transmitted Envrionmental Sensing Service characteristic value. */

// Environmental Sensing Descriptor Value Changed flag bits
#define ESS_DVC_FLAG_SOURCE_BIT         (0x01 << 0)  /**< Source bit. 0: Server 1: Client */
#define ESS_DVC_FLAG_TRIG_SET_BIT       (0x01 << 1)  /**< ES Trigger Setting */
#define ESS_DVC_FLAG_CONFIG_BIT         (0x01 << 2)  /**< ES Configuration */
#define ESS_DVC_FLAG_MEAS_BIT           (0x01 << 3)  /**< ES Measurement */
#define ESS_DVC_FLAG_CHAR_USER_DESC_BIT (0x01 << 4)  /**< Characteristic User Description */
/**< Bits 5 - 15 Reserved for Future Use */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)
{
    p_ess->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ess->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling write events to the Environmental Sensing Service Measurement CCCD.
 *
 * @param[in]   p_ess         Environmental Sensing Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
static void on_cccd_write(ble_ess_t * p_ess, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update indication state
        if (p_ess->evt_handler != NULL)
        {
            ble_ess_evt_t evt;

            if (p_evt_write->handle == p_ess->temperature_handles.cccd_handle)
            {

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_ESS_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_ESS_EVT_NOTIFICATION_DISABLED;
                }
            }
            else if (p_evt_write->handle == p_ess->dvc_handles.cccd_handle)
            {
                if (ble_srv_is_indication_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_ESS_EVT_INDICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_ESS_EVT_INDICATION_DISABLED;
                }
            }

            p_ess->evt_handler(p_ess, &evt);
        }
    }
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_ess->temperature_handles.cccd_handle
        || p_evt_write->handle == p_ess->dvc_handles.cccd_handle)
    {
        on_cccd_write(p_ess, p_evt_write);
    }
}


/**@brief Function for handling the HVC event.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in]   p_ess       Environmental Sensing Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_hvc(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_hvc_t * p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;

    if (p_hvc->handle == p_ess->temperature_handles.value_handle)
    {
        ble_ess_evt_t evt;

        evt.evt_type = BLE_ESS_EVT_INDICATION_CONFIRMED;
        p_ess->evt_handler(p_ess, &evt);
    }
}


void ble_ess_on_ble_evt(ble_ess_t * p_ess, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_ess, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ess, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_ess, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_hvc(p_ess, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Environmental Sensing Measurement.
 *
 * @param[in]   p_ess              Environmental Sensing Service structure.
 * @param[in]   p_ess_meas         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t ess_measurement_encode(ble_ess_t * p_ess,
                                      int16_t   * p_ess_meas,
                                      uint8_t   * p_encoded_buffer)
{

    uint8_t  len   = 2;
    nt16 encoded_temp;

    encoded_temp.i16 = *p_ess_meas;

    p_encoded_buffer[0] = encoded_temp.s16.u8L;
    p_encoded_buffer[1] = encoded_temp.s16.u8H;

    return len;
}


/**@brief Function for adding Environmental Sensing Measurement characteristics.
 *
 * @param[in]   p_ess        Environmental Sensing Service structure.
 * @param[in]   p_ess_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
/*
static uint32_t ess_measurement_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    ble_ess_meas_t      initial_esm;
    uint8_t             encoded_esm[MAX_ESM_LEN];

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    cccd_md.write_perm = p_ess_init->ess_meas_attr_md.cccd_write_perm;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.indicate = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_MEASUREMENT_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_ess_init->ess_meas_attr_md.read_perm;
    attr_md.write_perm = p_ess_init->ess_meas_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    memset(&initial_esm, 0, sizeof(initial_esm));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = ess_measurement_encode(p_ess, &initial_htm, encoded_esm);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_HTM_LEN;
    attr_char_value.p_value   = encoded_esm;

    return sd_ble_gatts_characteristic_add(p_ess->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ess->meas_handles);
}
*/

/**@brief Function for adding Temperature characteristics.
 *
 * @param[in]   p_ess        Environmental Sensing Service structure.
 * @param[in]   p_ess_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ess_temperature_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    ble_gatts_char_pf_t pres_fmt;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t cccd_md;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    int16_t             init_value_temperature;
    uint8_t             init_value_encoded[MAX_ESM_LEN];

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_ess_init->ess_temperature_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = &cccd_md;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TEMPERATURE_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_ess_init->ess_temperature_attr_md.read_perm;
    attr_md.write_perm = p_ess_init->ess_temperature_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    init_value_temperature = p_ess_init->init_temperature;
    p_ess->current_temperature = p_ess_init->init_temperature;

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = ess_measurement_encode(p_ess, &init_value_temperature, init_value_encoded);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = MAX_ESM_LEN;
    attr_char_value.p_value   = init_value_encoded;

    return sd_ble_gatts_characteristic_add(p_ess->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ess->temperature_handles);
}

/**@brief Function for adding Descriptor Value Changed characteristic.
 *
 * @param[in]   p_ess        Environmental Sensing Service structure.
 * @param[in]   p_ess_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ess_dvc_char_add(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    int16_t             init_value_temperature;
    int16_t             init_value_encoded[MAX_ESM_LEN];

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_ess_init->ess_dvc_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.indicate = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = NULL;
    char_md.p_sccd_md           = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DESC_VALUE_CHANGED_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_ess_init->ess_dvc_attr_md.read_perm;
    attr_md.write_perm = p_ess_init->ess_dvc_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    init_value_temperature = p_ess_init->init_temperature;
    init_value_encoded[0] = init_value_temperature;

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint16_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint16_t);
    attr_char_value.p_value   = (uint8_t *)init_value_encoded;

    return sd_ble_gatts_characteristic_add(p_ess->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ess->dvc_handles);
}


uint32_t ble_ess_init(ble_ess_t * p_ess, const ble_ess_init_t * p_ess_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_ess->evt_handler = p_ess_init->evt_handler;
    p_ess->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ENVIRONMENTAL_SENSING_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ess->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add measurement characteristic
    /*
    err_code = ess_measurement_char_add(p_ess, p_ess_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    */

    // Add temperature characteristic
    err_code = ess_temperature_char_add(p_ess, p_ess_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    /*
    // Add humidity characteristic
    err_code = ess_humidity_char_add(p_ess, p_ess_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add irradiance characteristic
    err_code = ess_irradiance_char_add(p_ess, p_ess_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    */

    err_code = ess_dvc_char_add(p_ess, p_ess_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_ess_measurement_update(ble_ess_t * p_ess, uint16_t ess_char_handle, int16_t * p_ess_meas)
{
    uint32_t err_code;

    // Send value if connected
    if (p_ess->conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        uint8_t                encoded_ess_meas[MAX_ESM_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = ess_measurement_encode(p_ess, p_ess_meas, encoded_ess_meas);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = ess_char_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_ess_meas;

        err_code = sd_ble_gatts_hvx(p_ess->conn_handle, &hvx_params);
        if ((err_code == NRF_SUCCESS) && (hvx_len != len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

uint32_t ble_ess_is_indication_enabled(ble_ess_t * p_ess, uint16_t ess_char_handle, bool * p_indication_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(p_ess->dvc_handles.cccd_handle,
                                      0,
                                      &len,
                                      cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_indication_enabled = ble_srv_is_indication_enabled(cccd_value_buf);
    }
    return err_code;
}

uint32_t ble_ess_is_notification_enabled(ble_ess_t * p_ess, uint16_t ess_char_handle, bool * p_indication_enabled)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    uint16_t len = BLE_CCCD_VALUE_LEN;

    err_code = sd_ble_gatts_value_get(p_ess->temperature_handles.cccd_handle,
                                      0,
                                      &len,
                                      cccd_value_buf);
    if (err_code == NRF_SUCCESS)
    {
        *p_indication_enabled = ble_srv_is_notification_enabled(cccd_value_buf);
    }
    return err_code;
}

