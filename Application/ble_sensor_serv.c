/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "nrf_soc.h"
//#include "nrf_gpio.h"
//#include "custom_board.h"

#include "ble_controller_serv.h"
#include "ble.h"
//#include "ble_srv_common.h"
#include "ble_sensor_serv.h"
#define INVALID_BATTERY_LEVEL  255


ble_sensor_t 	 m_sensor;




static uint32_t acel_x_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_acel_x_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->acel_x_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_ACEL_X;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->acel_x_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->acel_x_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_acel_x_level = p_sensor_init->initial_acel_x_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 2;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 2;//sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_acel_x_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->acel_x_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->acel_x_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->acel_x_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t acel_x_update_char(ble_sensor_t * p_sensor, int16_t acel_x_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (acel_x_level != p_sensor->acel_x_last)
    {
        uint16_t len = sizeof(acel_x_level);

        // Save new rssi value
        p_sensor->acel_x_last = acel_x_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(acel_x_level);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&acel_x_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->acel_x_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(acel_x_level);

            hvx_params.handle   = p_sensor->acel_x_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&acel_x_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}



static uint32_t acel_y_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_acel_y_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->acel_y_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_ACEL_Y;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->acel_y_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->acel_y_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_acel_y_level = p_sensor_init->initial_acel_y_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_acel_y_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->acel_y_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->acel_y_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->acel_y_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t acel_y_update_char(ble_sensor_t * p_sensor, int16_t acel_y_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (acel_y_level != p_sensor->acel_y_last)
    {
        uint16_t len = sizeof(acel_y_level);

        // Save new rssi value
        p_sensor->acel_y_last = acel_y_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(acel_y_level);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&acel_y_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->acel_y_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(acel_y_level);

            hvx_params.handle   = p_sensor->acel_y_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&acel_y_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}







static uint32_t acel_z_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_acel_z_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->acel_z_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_ACEL_Z;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->acel_z_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->acel_z_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_acel_z_level = p_sensor_init->initial_acel_z_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 2;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 2;//sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_acel_z_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->acel_z_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->acel_z_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->acel_z_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t acel_z_update_char(ble_sensor_t * p_sensor, int16_t acel_z_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (acel_z_level != p_sensor->acel_z_last)
    {
        uint16_t len = sizeof(acel_z_level);

        // Save new rssi value
        p_sensor->acel_z_last = acel_z_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(acel_z_level);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&acel_z_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->acel_z_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(acel_z_level);

            hvx_params.handle   = p_sensor->acel_z_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&acel_z_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}



/**@brief Function for adding Connection Mode Settings characteristics.
 *
 * @param[in]   p_extra        BLE Settings Service structure.
 * @param[in]   p_settings_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t gyro_x_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_gyro_x_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->gyro_x_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_GYRO_X;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->gyro_x_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->gyro_x_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_gyro_x_level = p_sensor_init->initial_gyro_x_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 2;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 2;//sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_gyro_x_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->gyro_x_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->gyro_x_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->gyro_x_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t gyro_x_update_char(ble_sensor_t * p_sensor, int16_t gyro_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (gyro_level != p_sensor->gyro_x_last)
    {
        uint16_t len = sizeof(gyro_level);

        // Save new rssi value
        p_sensor->gyro_x_last = gyro_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(gyro_level);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&gyro_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->gyro_x_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(gyro_level);

            hvx_params.handle   = p_sensor->gyro_x_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&gyro_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}


static uint32_t gyro_y_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_gyro_y_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->gyro_y_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_GYRO_Y;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->gyro_y_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->gyro_y_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_gyro_y_level = p_sensor_init->initial_gyro_y_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 2;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 2;//sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_gyro_y_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->gyro_y_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->gyro_y_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->gyro_y_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t gyro_y_update_char(ble_sensor_t * p_sensor, int16_t gyro_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (gyro_level != p_sensor->gyro_y_last)
    {
        uint16_t len = sizeof(gyro_level);

        // Save new rssi value
        p_sensor->gyro_y_last = gyro_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(gyro_level);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&gyro_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->gyro_y_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(gyro_level);

            hvx_params.handle   = p_sensor->gyro_y_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&gyro_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}



static uint32_t gyro_z_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_gyro_z_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->gyro_z_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_GYRO_Z;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->gyro_z_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->gyro_z_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_gyro_z_level = p_sensor_init->initial_gyro_z_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_gyro_z_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->gyro_z_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->gyro_z_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->gyro_z_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t gyro_z_update_char(ble_sensor_t * p_sensor, int16_t gyro_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (gyro_level != p_sensor->gyro_z_last)
    {
        uint16_t len = sizeof(gyro_level);

        // Save new rssi value
        p_sensor->gyro_z_last = gyro_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(gyro_level);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&gyro_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->gyro_z_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(gyro_level);

            hvx_params.handle   = p_sensor->gyro_z_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&gyro_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}



static uint32_t ground_station_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_gnd_station_level;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    // Add Battery Level characteristic
    if (p_sensor->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_sensor_init->ground_station_char_attr_md.cccd_write_perm;
        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

	ble_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
    ble_uuid.uuid = UUID_CHAR_GND_STATION;
    sd_ble_uuid_vs_add(&m_base_uuid128, &ble_uuid.type);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_sensor_init->ground_station_char_attr_md.read_perm;
    attr_md.write_perm = p_sensor_init->ground_station_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_gnd_station_level = p_sensor_init->initial_ground_station_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = &initial_gnd_station_level;

    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_sensor->ground_station_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_sensor_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_sensor_init->ground_station_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth    = 0;
        attr_md.wr_auth    = 0;
        attr_md.vlen       = 0;

        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid       = &ble_uuid;
        attr_char_value.p_attr_md    = &attr_md;
        attr_char_value.init_len     = init_len;
        attr_char_value.init_offs    = 0;
        attr_char_value.max_len      = attr_char_value.init_len;
        attr_char_value.p_value      = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_sensor->ground_station_handles.value_handle,
                                               &attr_char_value,
                                               &p_sensor->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}


uint32_t ground_station_update_char(ble_sensor_t * p_sensor, uint8_t ground_station_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (ground_station_level != p_sensor->ground_station_last)
    {
        uint16_t len = sizeof(int8_t);

        // Save new rssi value
        p_sensor->ground_station_last = ground_station_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(uint8_t);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&ground_station_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->ground_station_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(int8_t);

            hvx_params.handle   = p_sensor->ground_station_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&ground_station_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}


static uint32_t pressure_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_pressure_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->pressure_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_PRESSURE;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->pressure_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->pressure_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_pressure_level = p_sensor_init->initial_pressure_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = sizeof(uint32_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = sizeof(uint32_t);
	    attr_char_value.p_value      = &initial_pressure_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->pressure_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->pressure_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->pressure_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t pressure_update_char(ble_sensor_t * p_sensor, uint32_t pressure_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (pressure_level != p_sensor->pressure_last)
    {
        uint16_t len = sizeof(uint32_t);

        // Save new rssi value
        p_sensor->pressure_last = pressure_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(uint32_t);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&pressure_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->pressure_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(pressure_level);

            hvx_params.handle   = p_sensor->pressure_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&pressure_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}




static uint32_t temp_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_temp_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->temp_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_TEMP;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->temp_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->temp_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_temp_level = p_sensor_init->initial_temp_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = sizeof(int8_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = sizeof(int8_t);
	    attr_char_value.p_value      = &initial_temp_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->temp_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->temp_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->temp_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t temp_update_char(ble_sensor_t * p_sensor, int8_t temp_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (temp_level != p_sensor->temp_last)
    {
        uint16_t len = sizeof(int8_t);

        // Save new rssi value
        p_sensor->temp_last = temp_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(int8_t);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&temp_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->temp_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(temp_level);

            hvx_params.handle   = p_sensor->temp_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&temp_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}

static uint32_t light_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_light_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->light_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_LIGHT;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->light_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->light_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_light_level = p_sensor_init->initial_light_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_light_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->light_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->light_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->light_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t light_update_char(ble_sensor_t * p_sensor, uint16_t light_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (light_level != p_sensor->light_last)
    {
        uint16_t len = sizeof(light_level);

        // Save new rssi value
        p_sensor->light_last = light_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(uint16_t);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&light_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->light_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(light_level);

            hvx_params.handle   = p_sensor->light_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&light_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}




static uint32_t altitude_char_add(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
	   uint32_t            err_code;
	    ble_gatts_char_md_t char_md;
	    ble_gatts_attr_md_t cccd_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_uuid_t          service_uuid;
	    ble_gatts_attr_md_t attr_md;
	    uint8_t             initial_altitude_level;
	    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
	    uint8_t             init_len;

	    // Add Battery Level characteristic
	    if (p_sensor->is_notification_supported)
	    {
	        memset(&cccd_md, 0, sizeof(cccd_md));

	        // According to BAS_SPEC_V10, the read operation on cccd should be possible without
	        // authentication.
	        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	        cccd_md.write_perm = p_sensor_init->altitude_char_attr_md.cccd_write_perm;
	        cccd_md.vloc = BLE_GATTS_VLOC_STACK;
	    }

	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.read   = 1;
	    char_md.char_props.notify = (p_sensor->is_notification_supported) ? 1 : 0;
	    char_md.p_char_user_desc  = NULL;
	    char_md.p_char_pf         = NULL;
	    char_md.p_user_desc_md    = NULL;
	    char_md.p_cccd_md         = (p_sensor->is_notification_supported) ? &cccd_md : NULL;
	    char_md.p_sccd_md         = NULL;

		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    service_uuid.uuid = UUID_CHAR_ALTITUDE;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

	    memset(&attr_md, 0, sizeof(attr_md));

	    attr_md.read_perm  = p_sensor_init->altitude_char_attr_md.read_perm;
	    attr_md.write_perm = p_sensor_init->altitude_char_attr_md.write_perm;
	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    initial_altitude_level = p_sensor_init->initial_altitude_level;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &service_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = sizeof(uint16_t);
	    attr_char_value.p_value      = &initial_altitude_level;

	    err_code = sd_ble_gatts_characteristic_add(p_sensor->service_handle, &char_md,
	                                               &attr_char_value,
	                                               &p_sensor->altitude_handles);
	    if (err_code != NRF_SUCCESS)
	    {
	        return err_code;
	    }

	    if (p_sensor_init->p_report_ref != NULL)
	    {
	        // Add Report Reference descriptor
	        BLE_UUID_BLE_ASSIGN(service_uuid, BLE_UUID_REPORT_REF_DESCR);

	        memset(&attr_md, 0, sizeof(attr_md));

	        attr_md.read_perm = p_sensor_init->altitude_report_read_perm;
	        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

	        attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	        attr_md.rd_auth    = 0;
	        attr_md.wr_auth    = 0;
	        attr_md.vlen       = 0;

	        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_sensor_init->p_report_ref);

	        memset(&attr_char_value, 0, sizeof(attr_char_value));

	        attr_char_value.p_uuid       = &service_uuid;
	        attr_char_value.p_attr_md    = &attr_md;
	        attr_char_value.init_len     = init_len;
	        attr_char_value.init_offs    = 0;
	        attr_char_value.max_len      = attr_char_value.init_len;
	        attr_char_value.p_value      = encoded_report_ref;

	        err_code = sd_ble_gatts_descriptor_add(p_sensor->altitude_handles.value_handle,
	                                               &attr_char_value,
	                                               &p_sensor->report_ref_handle);
	        if (err_code != NRF_SUCCESS)
	        {
	            return err_code;
	        }
	    }
	    else
	    {
	        p_sensor->report_ref_handle = BLE_GATT_HANDLE_INVALID;
	    }

	    return NRF_SUCCESS;
}
uint32_t altitude_update_char(ble_sensor_t * p_sensor, uint16_t altitude_level)
{
    uint32_t err_code = NRF_SUCCESS;


    if (altitude_level != p_sensor->altitude_last)
    {
        uint16_t len = sizeof(uint16_t);

        // Save new rssi value
        p_sensor->altitude_last = altitude_level;

        ble_gatts_value_t gatts_value;

		// Initialize value struct.
		memset(&gatts_value, 0, sizeof(gatts_value));

		gatts_value.len     = sizeof(uint16_t);
		gatts_value.offset  = 0;
		gatts_value.p_value = (uint8_t*)&altitude_level;
		// Update database
		err_code= sd_ble_gatts_value_set(p_sensor->conn_handle, p_sensor->altitude_handles.value_handle,&gatts_value);
       if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sensor->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sensor->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(altitude_level);

            hvx_params.handle   = p_sensor->altitude_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = (uint8_t*)&altitude_level;

            err_code = sd_ble_gatts_hvx(p_sensor->conn_handle, &hvx_params);

        }
        else
        {

            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}

uint32_t ble_sensor_serv_init(ble_sensor_t * p_sensor, const ble_sensor_init_t * p_sensor_init)
{
    uint32_t   err_code;
	ble_uuid_t  service_uuid;


    // Initialize service structure
    if (p_sensor_init->evt_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }






    // Initialize service structure
    p_sensor->evt_handler               = p_sensor_init->evt_handler;
    p_sensor->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_sensor->is_notification_supported = p_sensor_init->support_notification;
    p_sensor->gyro_x_last 				= 0;
    p_sensor->gyro_y_last 				= 0;
    p_sensor->gyro_z_last 				= 0;
    p_sensor->acel_x_last 				= 0;
    p_sensor->acel_y_last 				= 0;
    p_sensor->acel_z_last 				= 0;
    p_sensor->ground_station_last 		= 0;
    p_sensor->light_last 				= 0;
    p_sensor->temp_last 				= 0;
    p_sensor->pressure_last 			= 0;
    p_sensor->altitude_last 			= 0;

    // Add service
    //BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEART_RATE_SERVICE); //TO BE CHANGED!!!
	service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
    service_uuid.uuid = UUID_SERV_SENSOR;
    err_code = sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_sensor->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
/*
		// Add Advertising Interval characteristics
	err_code = acel_x_char_add(p_sensor, p_sensor_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = acel_y_char_add(p_sensor, p_sensor_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

		// Add TXPower characteristics
	err_code = acel_z_char_add(p_sensor, p_sensor_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}





		// Add Connection Mode characteristic
    err_code = gyro_x_char_add(p_sensor, p_sensor_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// Add button level characteristic
	err_code = gyro_y_char_add(p_sensor, p_sensor_init);
		if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}

		// Add Activate Bootloader characteristic
    err_code = gyro_z_char_add(p_sensor, p_sensor_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
*/
    		// Add battery level characteristic
		err_code = light_char_add(p_sensor, p_sensor_init);
			if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}

    			// Add battery level characteristic
			err_code = ground_station_char_add(p_sensor, p_sensor_init);
				if (err_code != NRF_SUCCESS)
			{
				return err_code;
			}


	// Add battery level characteristic
	err_code = temp_char_add(p_sensor, p_sensor_init);
		if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}



		// Add battery level characteristic
		err_code = pressure_char_add(p_sensor, p_sensor_init);
			if (err_code != NRF_SUCCESS)
		{
			return err_code;
		}

/*
			// Add battery level characteristic
			err_code = altitude_char_add(p_sensor, p_sensor_init);
				if (err_code != NRF_SUCCESS)
			{
				return err_code;
			}
*/



	return NRF_SUCCESS;
}


static void on_connect(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
    p_sensor->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sensor->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{


	  ble_gatts_evt_write_t * p_evt_write;




	  if (p_sensor->is_notification_supported)
    {
	        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	        if ((p_evt_write->handle == p_sensor->gyro_x_handles.cccd_handle) && (p_evt_write->len == 2))
		{
			// CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
			{
				ble_sensor_evt_t evt;
				if (ble_srv_is_notification_enabled(p_evt_write->data))
				{
					evt.evt_type = CHAR_GYRO_X_SUBSCRIBED;
				}
				else
				{
					evt.evt_type = CHAR_GYRO_X_UNSUBSCRIBED;

				}
				p_sensor->evt_handler(p_sensor, &evt);
			}
		}


	        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	        if ((p_evt_write->handle == p_sensor->gyro_y_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_GYRO_Y_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_GYRO_Y_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }
	        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	        if ((p_evt_write->handle == p_sensor->gyro_z_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_GYRO_Z_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_GYRO_Z_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }
        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->acel_x_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_ACEL_X_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_ACEL_X_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }
        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->acel_y_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_ACEL_Y_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_ACEL_Y_SUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }
        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->acel_z_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_ACEL_Z_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_ACEL_Z_SUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }

        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->temp_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_TEMP_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_TEMP_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }
        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->light_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_LIGHT_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_LIGHT_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }

        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->ground_station_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_GND_STATION_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_GND_STATION_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }


        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        if ((p_evt_write->handle == p_sensor->pressure_handles.cccd_handle) && (p_evt_write->len == 2))
        {
            // CCCD written, call application event handler
			if (p_sensor->evt_handler != NULL)
            {
				ble_sensor_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CHAR_PRESSURE_SUBSCRIBED;
                }
                else
                {
                    evt.evt_type = CHAR_PRESSURE_UNSUBSCRIBED;
                }

				p_sensor->evt_handler(p_sensor, &evt);
            }
        }



        p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
             if ((p_evt_write->handle == p_sensor->altitude_handles.cccd_handle) && (p_evt_write->len == 2))
             {
                 // CCCD written, call application event handler
     			if (p_sensor->evt_handler != NULL)
                 {
     				ble_sensor_evt_t evt;

                     if (ble_srv_is_notification_enabled(p_evt_write->data))
                     {
                         evt.evt_type = CHAR_ALTITUDE_SUBSCRIBED;
                     }
                     else
                     {
                         evt.evt_type = CHAR_ALTITUDE_UNSUBSCRIBED;
                     }

     				p_sensor->evt_handler(p_sensor, &evt);
                 }
             }


    }
}


void ble_sensor_on_ble_evt(ble_sensor_t * p_sensor, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sensor, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sensor, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sensor, p_ble_evt);
            break;

        default:
            break;
    }
}

