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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "nordic_common.h"
#include "app_util.h"
#include "nrf_soc.h"
#include "ble_controller_serv.h"

ble_controller_t    m_controller;                                           /**< Structure used to identify the Beeps service. */







static uint32_t ctrl_pwm1_char_add(ble_controller_t * p_controller)
{
	ble_gatts_char_md_t char_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_gatts_attr_md_t attr_md;
	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.write = 1;
	    char_md.p_char_user_desc         = NULL;
	    char_md.p_char_pf                = NULL;
	    char_md.p_user_desc_md           = NULL;
	    char_md.p_cccd_md                = NULL;
	    char_md.p_sccd_md                = NULL;

		ble_uuid_t  char_uuid;
		char_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    char_uuid.uuid = UUID_CHAR_PWM1;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &char_uuid.type);
	    memset(&attr_md, 0, sizeof(attr_md));

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &char_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 1;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 1;//sizeof(uint16_t);

	    return sd_ble_gatts_characteristic_add(p_controller->service_handle,
	                                           &char_md,
	                                           &attr_char_value,
	                                           &p_controller->controller_pwm1_handles);
}


static void on_controller_pwm1_write(ble_controller_t * p_controller, ble_gatts_evt_write_t * p_evt_write)
{
	   if ((p_evt_write->len >= 1) && (p_controller->evt_handler != NULL))
	   {
		   ble_controller_evt_t evt;

		   evt.evt_type = PWM1_VAL;
	        evt.pwm1_val = p_evt_write->data[0];
	        p_controller->evt_handler(p_controller, &evt);
	    }

}


static uint32_t ctrl_pwm2_char_add(ble_controller_t * p_controller)
{
	ble_gatts_char_md_t char_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_gatts_attr_md_t attr_md;
	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.write = 1;
	    char_md.p_char_user_desc         = NULL;
	    char_md.p_char_pf                = NULL;
	    char_md.p_user_desc_md           = NULL;
	    char_md.p_cccd_md                = NULL;
	    char_md.p_sccd_md                = NULL;

		ble_uuid_t  char_uuid;
		char_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    char_uuid.uuid = UUID_CHAR_PWM2;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &char_uuid.type);
	    memset(&attr_md, 0, sizeof(attr_md));

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &char_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 1;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 1;//sizeof(uint16_t);

	    return sd_ble_gatts_characteristic_add(p_controller->service_handle,
	                                           &char_md,
	                                           &attr_char_value,
	                                           &p_controller->controller_pwm2_handles);
}


static void on_controller_pwm2_write(ble_controller_t * p_controller, ble_gatts_evt_write_t * p_evt_write)
{
	   if ((p_evt_write->len >= 1) && (p_controller->evt_handler != NULL))
	   {
		   ble_controller_evt_t evt;

		   evt.evt_type = PWM2_VAL;
	        evt.pwm2_val = p_evt_write->data[0];
	        p_controller->evt_handler(p_controller, &evt);
	    }

}



static uint32_t ctrl_pwm3_char_add(ble_controller_t * p_controller)
{
	ble_gatts_char_md_t char_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_gatts_attr_md_t attr_md;
	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.write = 1;
	    char_md.p_char_user_desc         = NULL;
	    char_md.p_char_pf                = NULL;
	    char_md.p_user_desc_md           = NULL;
	    char_md.p_cccd_md                = NULL;
	    char_md.p_sccd_md                = NULL;

		ble_uuid_t  char_uuid;
		char_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    char_uuid.uuid = UUID_CHAR_PWM3;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &char_uuid.type);
	    memset(&attr_md, 0, sizeof(attr_md));

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &char_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 1;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 1;//sizeof(uint16_t);

	    return sd_ble_gatts_characteristic_add(p_controller->service_handle,
	                                           &char_md,
	                                           &attr_char_value,
	                                           &p_controller->controller_pwm3_handles);
}

static void on_controller_pwm3_write(ble_controller_t * p_controller, ble_gatts_evt_write_t * p_evt_write)
{
	   if ((p_evt_write->len >= 1) && (p_controller->evt_handler != NULL))
	   {
		   ble_controller_evt_t evt;

		   evt.evt_type = PWM3_VAL;
	        evt.pwm3_val = p_evt_write->data[0];
	        p_controller->evt_handler(p_controller, &evt);
	    }

}






static uint32_t ctrl_pwm4_char_add(ble_controller_t * p_controller)
{
	ble_gatts_char_md_t char_md;
	    ble_gatts_attr_t    attr_char_value;
	    ble_gatts_attr_md_t attr_md;
	    memset(&char_md, 0, sizeof(char_md));

	    char_md.char_props.write = 1;
	    char_md.p_char_user_desc         = NULL;
	    char_md.p_char_pf                = NULL;
	    char_md.p_user_desc_md           = NULL;
	    char_md.p_cccd_md                = NULL;
	    char_md.p_sccd_md                = NULL;

		ble_uuid_t  char_uuid;
		char_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
	    char_uuid.uuid = UUID_CHAR_PWM4;
	    sd_ble_uuid_vs_add(&m_base_uuid128, &char_uuid.type);
	    memset(&attr_md, 0, sizeof(attr_md));

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	    attr_md.rd_auth    = 0;
	    attr_md.wr_auth    = 0;
	    attr_md.vlen       = 0;

	    memset(&attr_char_value, 0, sizeof(attr_char_value));

	    attr_char_value.p_uuid       = &char_uuid;
	    attr_char_value.p_attr_md    = &attr_md;
	    attr_char_value.init_len     = 1;//sizeof(uint16_t);
	    attr_char_value.init_offs    = 0;
	    attr_char_value.max_len      = 1;//sizeof(uint16_t);

	    return sd_ble_gatts_characteristic_add(p_controller->service_handle,
	                                           &char_md,
	                                           &attr_char_value,
	                                           &p_controller->controller_pwm4_handles);
}

static void on_controller_pwm4_write(ble_controller_t * p_controller, ble_gatts_evt_write_t * p_evt_write)
{
	   if ((p_evt_write->len >= 1) && (p_controller->evt_handler != NULL))
	   {
		   ble_controller_evt_t evt;

		   evt.evt_type = PWM4_VAL;
	        evt.pwm4_val = p_evt_write->data[0];
	        p_controller->evt_handler(p_controller, &evt);
	    }

}




uint32_t ble_controller_serv_init(ble_controller_t * p_controller, const ble_controller_init_t * p_controller_init)
{
    uint32_t   err_code;

    // Initialize service structure
    if (p_controller_init->evt_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    p_controller->evt_handler 				= p_controller_init->evt_handler;
    p_controller->conn_handle               	= BLE_CONN_HANDLE_INVALID;
//    p_controller->is_notification_supported 	= p_controller_init->support_notification;
		
    // Add service
		ble_uuid_t  service_uuid;
		service_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN + ITC_VENDOR_UUID_INDEX;  // Position of the Vender specific ID base in m_base_uuid128.
    service_uuid.uuid = UUID_SERV_CONTROLLER;
    err_code = sd_ble_uuid_vs_add(&m_base_uuid128, &service_uuid.type);
		
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_controller->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }



		err_code = ctrl_pwm1_char_add(p_controller);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

		err_code = ctrl_pwm2_char_add(p_controller);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		

    err_code = ctrl_pwm3_char_add(p_controller);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = ctrl_pwm4_char_add(p_controller);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
	return NRF_SUCCESS;
}
/**@brief Function for handling the Write event.
 *
 * @param[in]   p_beep       Immediate Alert Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_controller_t * p_controller, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

		//read event from mobile under the Beep Service
    if (p_evt_write->handle == p_controller->controller_pwm3_handles.value_handle)
    {
        on_controller_pwm3_write(p_controller, p_evt_write);
    }
		else if (p_evt_write->handle == p_controller->controller_pwm4_handles.value_handle)
    {
        on_controller_pwm4_write(p_controller, p_evt_write);
    }
		else if (p_evt_write->handle == p_controller->controller_pwm1_handles.value_handle)
    {
        on_controller_pwm1_write(p_controller, p_evt_write);
    }
		else if (p_evt_write->handle == p_controller->controller_pwm2_handles.value_handle)
    {
        on_controller_pwm2_write(p_controller, p_evt_write);
    }

}




/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_controller_t * p_controller, ble_evt_t * p_ble_evt)
{
    p_controller->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_controller_t * p_controller, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_controller->conn_handle = BLE_CONN_HANDLE_INVALID;
}



void ble_controller_on_ble_evt(ble_controller_t * p_controller, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_controller, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_controller, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_controller, p_ble_evt);
            break;

        default:
            break;
    }
}




