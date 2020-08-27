#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "mainfile.h"
#include "boards.h"
#include "ble_bas.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "app_uart.h"
#include "ble_nus.h"
#include "ble_uart.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "softdevice_handler.h"
#include "timers.h"
#include "ble_controller_serv.h"
#include "ble_sensor_serv.h"
#include "ble_dis.h"
#include "device_manager.h"
#include "ble_conn_params.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "ble_advertising.h"
#include "ble_flash.h"
#include "ble_radio_notification.h"
#include "app_util_platform.h"
#include "button.h"
#include "nrf_adc.h"
#include "state_machine.h"
#include "pstorage.h"
#include "bluetooth.h"
#include "nrf_delay.h"
#include "adc.h"
#include "nrf_pwm.h"

#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific).*/


#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */


 #define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
 #define PERIPHERAL_LINK_COUNT           1                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/


#define TIMER_RSSI_TIME_BASE	100									//ms


static char buf_aux[100];
static MY_BLE_DEV 						my_ble;
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;     /**< Handle of the current connection. */

static dm_application_instance_t        m_app_handle;                                /**< Application identifier allocated by device manager */

static bool                            	m_memory_access_in_progress = false;              /**< Flag to keep track of ongoing operations on persistent memory. */

static int8_t raw_rssi=0;
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */

static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */


static ble_advertising_mode_t adv_pendent_mode=BLE_SLOW_ADV;



static ble_gap_addr_t addr_connected;



#ifdef _ADV_WHITE_LIST_
static dm_handle_t                      m_bonded_peer_handle;                       /**< Device reference handle to the current connected peer. */
#endif
static uint8_t                           m_direct_adv_cnt;                              /**< Counter of direct advertisements. */

static bool flag_detect=false;







/**@brief Function for stopping advertising.
 */

static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */

static void app_context_load(dm_handle_t const *p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */

static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        //APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}

/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for initializing the Radio Notification event.
 */




/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
void battery_perc_update(unsigned char val)
{
    uint32_t err_code;
    if(my_ble.connected)
    {
	//sprintf(buf_aux,"BAT_VAL: %u",val);
	//nus_printStr(buf_aux);
    }
    err_code = ble_bas_battery_level_update(&m_bas,val);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }

}

static void radio_notification_init(void)
{
    uint32_t err_code;

    err_code = ble_radio_notification_init(APP_IRQ_PRIORITY_HIGHEST,
                                           NRF_RADIO_NOTIFICATION_DISTANCE_4560US,
                                           ble_flash_on_radio_active_evt);
    APP_ERROR_CHECK(err_code);

}





/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */

void on_ble_evt(ble_evt_t * p_ble_evt)
{
  uint32_t        err_code = NRF_SUCCESS;
  int8_t tmp,i;

  ble_gap_addr_t peer_address;


  switch (p_ble_evt->header.evt_id)
   {
        case BLE_GAP_EVT_CONNECTED:
           	m_conn_handle=p_ble_evt->evt.gap_evt.conn_handle;
           	addr_connected = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
        	//timer_fake_sensors_start(1000);


           	my_ble.connected=true;
			my_ble.rssi.subscribed=false;
			//timer_adv2_start(5000);
			
   			timer_poff_stop();
   			advertising_start(BLE_NO_ADV);
   			advertising_start(BLE_CONNECTED_ADV);
			timer_send_nus_start(1000);
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			my_ble.connected=false;
			advertising_start(BLE_NO_ADV);
			advertising_start(BLE_FAST_ADV);

			m_conn_handle=BLE_CONN_HANDLE_INVALID;



			break;
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//ALTERADO 02-07-15 by Andr�
//			err_code = sd_ble_gap_sec_params_reply(m_conn_handle,BLE_GAP_SEC_STATUS_SUCCESS, &m_sec_params,NULL);
//			APP_ERROR_CHECK(err_code);
            break;
        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
               	advertising_start(my_ble.adv.next_mode);
            break;
        case BLE_GATTC_EVT_TIMEOUT:
       	//No break
        case BLE_GATTS_EVT_TIMEOUT:
        	// Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);

            break;
		case BLE_GAP_EVT_RSSI_CHANGED:							// NAO existe no NORDIC PROXIMITY mas existe no LAPA R1
			if(my_ble.connected)
				if(my_ble.rssi.en)
				{
				//	raw_rssi =p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;

					if(my_ble.rssi.subscribed)
					{
//29-11-16////////////////////////////////////////////////////////////////
						//ble_rssi_update(&m_beep, (int8_t)raw_rssi);
//////////////////////////////////////////////////////////////////////////
					}
				}
			break;
		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		        break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling Beep events.
 *
 * @details This function will be called for all Immediate Alert events which are passed to the
 *          application.
 *
 * @param[in]   p_beep  Beep structure.
 * @param[in]   p_evt  Event received from the Beep service.
 */
void on_controller_evt(ble_controller_t * p_controller, ble_controller_evt_t * p_evt)
{
	uint32_t err_code;
	uint8_t i=0;

	switch(p_evt->evt_type)
	{
		case PWM1_VAL:
			sprintf(buf_aux,"PWM1_VAL: %u",p_evt->pwm1_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);
			break;
		case PWM2_VAL:
			sprintf(buf_aux,"PWM2_VAL: %u",p_evt->pwm2_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);
			break;
		case PWM3_VAL:

			sprintf(buf_aux,"PWM3_VAL: %u",p_evt->pwm3_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);

			break;
		case PWM4_VAL:
			sprintf(buf_aux,"PWM4_VAL: %u",p_evt->pwm4_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);

			break;
		default:
			break;
	}
}


/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in]   p_bas  Battery Service structure.
 * @param[in]   p_evt  Event received from the Battery Service.
 */

void on_sensor_evt(ble_sensor_t * p_extra, ble_sensor_evt_t *p_evt)
{
	//timer_fake_sensors_start(1000);


		switch (p_evt->evt_type)
		{
			case CHAR_ACEL_X_SUBSCRIBED:
				nus_printStr("ACEL_X_SUB");
				my_ble.acel_x.subscribed=true;
				break;
			case CHAR_ACEL_X_UNSUBSCRIBED:
				nus_printStr("ACEL_X_UNSUB");
				my_ble.acel_x.subscribed=false;

				break;
			case CHAR_ACEL_Y_SUBSCRIBED:
				nus_printStr("ACEL_Y_SUB");
				my_ble.acel_y.subscribed=true;

				break;
			case CHAR_ACEL_Y_UNSUBSCRIBED:
				nus_printStr("ACEL_Y_UNSUB");
				my_ble.acel_y.subscribed=false;

				break;
			case CHAR_ACEL_Z_SUBSCRIBED:
				my_ble.acel_z.subscribed=true;
				nus_printStr("ACEL_Z_SUB");

				break;
			case CHAR_ACEL_Z_UNSUBSCRIBED:
				my_ble.acel_z.subscribed=false;
				nus_printStr("ACEL_X_UNSUB");

				break;

			case CHAR_GYRO_X_SUBSCRIBED:
				my_ble.gyro_x.subscribed=true;
				nus_printStr("GYRO_X_SUB");

				break;
			case CHAR_GYRO_X_UNSUBSCRIBED:
				my_ble.gyro_x.subscribed=false;
				nus_printStr("GYRO_X_UNSUB");

				break;
			case CHAR_GYRO_Y_SUBSCRIBED:
				my_ble.gyro_y.subscribed=true;
				nus_printStr("GYRO_Y_SUB");

				break;
			case CHAR_GYRO_Y_UNSUBSCRIBED:
				my_ble.gyro_y.subscribed=false;
				nus_printStr("GYRO_Y_UNSUB");

				break;
			case CHAR_GYRO_Z_SUBSCRIBED:
				my_ble.gyro_z.subscribed=true;
				nus_printStr("GYRO_Z_SUB");

				break;
			case CHAR_GYRO_Z_UNSUBSCRIBED:
				my_ble.gyro_z.subscribed=false;
				nus_printStr("GYRO_Z_UNSUB");

				break;

			case CHAR_LIGHT_SUBSCRIBED:
				my_ble.light.subscribed=true;
			nus_printStr("LIGHT_SUB");

				break;
			case CHAR_LIGHT_UNSUBSCRIBED:
				my_ble.light.subscribed=false;
				nus_printStr("LIGHT_UNSUB");

				break;

			case CHAR_GND_STATION_SUBSCRIBED:
				nus_printStr("GND_STATION_SUB");
				my_ble.gnd_station.subscribed=true;
			break;
			case CHAR_GND_STATION_UNSUBSCRIBED:
				nus_printStr("GND_STATION_UNSUB");
				my_ble.gnd_station.subscribed=false;

				break;

			case CHAR_PRESSURE_SUBSCRIBED:
				nus_printStr("PRESSURE_SUB");
				my_ble.pressure.subscribed=true;
			break;
			case CHAR_PRESSURE_UNSUBSCRIBED:
				nus_printStr("PRESSURE_UNSUB");
				my_ble.pressure.subscribed=false;

				break;
			case CHAR_TEMP_SUBSCRIBED:
				nus_printStr("TEMP_SUB");
				my_ble.temp.subscribed=true;
			break;
			case CHAR_TEMP_UNSUBSCRIBED:
				nus_printStr("TEMP_UNSUB");
				my_ble.temp.subscribed=false;

				break;
			case CHAR_ALTITUDE_SUBSCRIBED:
				nus_printStr("ALTITUDE_SUB");
				my_ble.altitude.subscribed=true;

			break;
			case CHAR_ALTITUDE_UNSUBSCRIBED:
				nus_printStr("ALTITUDE_UNSUB");
				my_ble.altitude.subscribed=false;

				break;

			default:
				// No implementation needed.
			break;
		}
}






void serv_controller_init(void)
{
	uint32_t       			err_code;
    ble_controller_init_t 	controller_init_obj;
    memset(&controller_init_obj, 0, sizeof(controller_init_obj));


    controller_init_obj.evt_handler 			= on_controller_evt;
    controller_init_obj.support_notification 	= true;
    controller_init_obj.p_report_ref       		= NULL;
    err_code = ble_controller_serv_init(&m_controller, &controller_init_obj);
    APP_ERROR_CHECK(err_code);

}







void serv_sensor_init(void)
{
    uint32_t       err_code;
    ble_sensor_init_t sensor_init_obj;

    memset(&sensor_init_obj, 0, sizeof(sensor_init_obj));
    sensor_init_obj.evt_handler          = on_sensor_evt;

	sensor_init_obj.initial_acel_x_level=0;
	sensor_init_obj.initial_acel_y_level=0;
	sensor_init_obj.initial_acel_z_level=0;

	sensor_init_obj.initial_gyro_x_level=0;
	sensor_init_obj.initial_gyro_y_level=0;
	sensor_init_obj.initial_gyro_z_level=0;
	sensor_init_obj.initial_temp_level=0;
	sensor_init_obj.initial_light_level=0;


	sensor_init_obj.initial_ground_station_level=0;
	sensor_init_obj.initial_pressure_level=0;
	sensor_init_obj.initial_altitude_level=0;

	sensor_init_obj.initial_batt_level=0;

	sensor_init_obj.support_notification=true;
	sensor_init_obj.p_report_ref       	= NULL;




    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_x_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_x_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_x_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.acel_x_char_attr_md.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_y_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_y_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_y_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.acel_y_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_z_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_z_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_z_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.acel_z_char_attr_md.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_x_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_x_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_x_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.gyro_x_char_attr_md.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_y_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_y_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_y_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.gyro_y_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_z_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_z_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_z_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.gyro_z_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.temp_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.temp_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.temp_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.temp_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.light_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.light_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.light_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.light_char_attr_md.write_perm);



    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.ground_station_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.ground_station_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.ground_station_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.ground_station_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.pressure_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.pressure_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.pressure_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.pressure_char_attr_md.write_perm);



    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.altitude_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.altitude_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.altitude_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.altitude_char_attr_md.write_perm);


    err_code = ble_sensor_serv_init(&m_sensor, &sensor_init_obj);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    memset(my_ble.dev_name,'\0',sizeof(my_ble.dev_name));
///////////////////////////////////////
	my_ble.mac_add[5]=NRF_FICR->DEVICEADDR[1]>>8;
	my_ble.mac_add[4]=NRF_FICR->DEVICEADDR[1]>>0;
	my_ble.mac_add[3]=NRF_FICR->DEVICEADDR[0]>>24;
	my_ble.mac_add[2]=NRF_FICR->DEVICEADDR[0]>>16;
	my_ble.mac_add[1]=NRF_FICR->DEVICEADDR[0]>>8;
	my_ble.mac_add[0]=NRF_FICR->DEVICEADDR[0];
/*
	 //This is my code to change MAC address
	  ble_gap_addr_t new_add ;
	  new_add.addr_type=BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
	  int i=0;
	  memcpy(new_add.addr,my_ble.mac_add,sizeof(new_add.addr));
	  for(i=0;i<2;i++)
	  {
	    new_add.addr[i]=0xff;
	  }
	  new_add.addr[5] |= 0xc0;// 2 MSBit must be '11' for RANDOM_STATIC address, see v4.0, Vol 3, Part C, chapter 10.8
	 err_code=sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &new_add) ;
	    APP_ERROR_CHECK(err_code);
*/


	sprintf((char *)my_ble.dev_name,"P%02x%02x%02x",my_ble.mac_add[5],my_ble.mac_add[4],my_ble.mac_add[3]);
    err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)my_ble.dev_name,strlen((const char *)my_ble.dev_name));
    APP_ERROR_CHECK(err_code);
//////////////////////////////////////////////////////////////////////////////////////
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_TAG);
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MSEC_TO_UNITS((uint16_t) MIN_CONN_INTERVAL_DEFAULT_MS, UNIT_1_25_MS);//MIN_CONN_INTERVAL_DEFAULT;
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS((uint16_t) MAX_CONN_INTERVAL_DEFAULT_MS, UNIT_1_25_MS);//MAX_CONN_INTERVAL_DEFAULT;
    gap_conn_params.slave_latency     = SLAVE_LATENCY_DEFAULT;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT_DEFAULT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
	//if(err_code == NRF_SUCCESS)
	//	ble_extra_max_conn_interval_default_set(&m_sensor, MAX_CONN_INTERVAL_DEFAULT_MS);
    tx_power_set(TX_POWER_LEVEL_DEFAULT);
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;

	cp_init.evt_handler                    = NULL;
	cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}




/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	dm_ble_evt_handler(p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);

    ble_conn_params_on_ble_evt(p_ble_evt);

    ble_sensor_on_ble_evt(&m_sensor, p_ble_evt);
	ble_controller_on_ble_evt(&m_controller, p_ble_evt);

    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */


	on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */

void sys_evt_dispatch(uint32_t sys_evt)
{
	pstorage_sys_event_handler(sys_evt); //Add this line
     ble_advertising_on_sys_evt(sys_evt);
	 on_sys_evt(sys_evt);

}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(int flag)
{
	 uint32_t err_code;

	 //uint32_t app_ram_base;



	    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

	    // Initialize the SoftDevice handler module.
	    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	    ble_enable_params_t ble_enable_params;


	    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
	    APP_ERROR_CHECK(err_code);
	    ble_enable_params.gatts_enable_params.service_changed = 1;
		ble_enable_params.gatts_enable_params.attr_tab_size= BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
	    ble_enable_params.common_enable_params.vs_uuid_count = 6;//4; 								//NUM of custom UUIDs
		ble_enable_params.common_enable_params.p_conn_bw_counts=NULL;


	    // Check the ram settings against the used number of links
	    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

	    // Enable BLE stack.
	    err_code = softdevice_enable(&ble_enable_params);
	    APP_ERROR_CHECK(err_code);


	    #if (SCAN_REQUEST_NOTIFICATION_EN==true)
		const ble_opt_t scan_req_opt=
			{
				.gap_opt =
				{
					.scan_req_report =
					{
						.enable=1
					}
				}
			};
		sd_ble_opt_set(BLE_GAP_OPT_SCAN_REQ_REPORT, &scan_req_opt);
		#endif

	    // Register with the SoftDevice handler module for BLE events.
	    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	    APP_ERROR_CHECK(err_code);

	    // Register with the SoftDevice handler module for BLE events.
	    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	    APP_ERROR_CHECK(err_code);




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


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


void timer_poff_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);



}



void tx_power_set(tx_power_t tx_power_level)
{
	const int8_t 				TX_POWER_LIST[8] 			= {4, 0, -4, -8, -12, -16, -20, -30};
	int8_t tx_power;
	switch(tx_power_level)
	{
	case TX_PLUS_4_dBm:
		tx_power = TX_POWER_LIST[TX_PLUS_4_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: +4dBm\n");
		#endif
		break;
	case TX_0_dBm:
		tx_power = TX_POWER_LIST[TX_0_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: 0dBm\n");
		#endif
		break;
	case TX_MINUS_4_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_4_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -4dBm\n");
		#endif
		break;
	case  TX_MINUS_8_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_8_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -8dBm\n");
		#endif
		break;
	case TX_MINUS_12_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_12_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -12dBm\n");
		#endif
		break;
	case TX_MINUS_16_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_16_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -16dBm\n");
		#endif
		break;
	case TX_MINUS_20_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_20_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -20dBm\n");
		#endif
		break;
	case TX_MINUS_30_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_30_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -30dBm\n");
		#endif
		break;
	default:
		#ifdef _UART_ENABLE_
		printf((const char *)"ERROR: TX PWR fail\n");
		#endif
		tx_power = TX_POWER_LIST[TX_MINUS_30_dBm];
			break;
	}
	uint32_t err_code = sd_ble_gap_tx_power_set(tx_power);
	APP_ERROR_CHECK(err_code);
	if(err_code == NRF_SUCCESS)
	{
		//ble_extra_tx_power_level_set(&m_sensor, tx_power_level);
		//ble_extra_connection_mode_set(&m_sensor, (my_ble.conn_params.requested_max_conn_interval << 16 | tx_power_level << 0));
	}



}




void gap_conn_params_and_tx_power_change(int16_t max_connection_interval, uint8_t tx_power_level)
{
	gap_conn_params_change(max_connection_interval);
	tx_power_set(tx_power_level);
}



/**@brief Function for the GAP Connection Parameters Change.
 *
 * @details This function is used to change the connection parameters
 *
 */
void gap_conn_params_change(uint16_t max_connection_interval)
{
	ble_gap_conn_params_t  updated_cnxn_param;
	uint32_t               err_code;
	
	updated_cnxn_param.min_conn_interval =  MSEC_TO_UNITS((uint16_t)(((uint16_t)max_connection_interval)-((uint16_t)25)), UNIT_1_25_MS);
	updated_cnxn_param.max_conn_interval =  MSEC_TO_UNITS((uint16_t) max_connection_interval, UNIT_1_25_MS);
	updated_cnxn_param.slave_latency     =  SLAVE_LATENCY_DEFAULT;
	updated_cnxn_param.conn_sup_timeout  =  CONN_SUP_TIMEOUT_DEFAULT;
		
	// Initialize and set-up connection parameter negotiation module
	// if this method is called upon connection it will immediately slow down
	err_code = ble_conn_params_change_conn_params(&updated_cnxn_param);
	if(err_code == NRF_SUCCESS)
	{
		// Procedure request succeeded. Connection parameters will be negotiated as requested.
		// BLE_CONN_PARAMS_EVT_SUCCEEDED will be notified if parameter negotiation is successful.
		//my_ble.conn_params.requested_conn_params_change = true;
		my_ble.conn_params.requested_max_conn_interval = max_connection_interval;
	}
	else
	{
			// Procedure request failed.
	}
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
          //  err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
           // APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
           // sleep_mode_enter();
            break;
        case BLE_ADV_EVT_SLOW:
            break;
        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *
 * @param[in]  adv_flags  Indicates which type of advertisement to use, see @ref BLE_GAP_DISC_MODES.
 *
 */
void advertising_init(uint8_t adv_flags)
{
     uint32_t      					err_code;
    ble_advdata_t 					advdata;
    ble_advdata_manuf_data_t        manuf_data;
    uint16_t sdata =  				battery_level()<<8;
    manuf_data.data.p_data  		= (uint8_t*)&sdata;
    manuf_data.data.size    		= sizeof(sdata);
	ble_uuid_t adv_uuids[] 			=	{	\
											{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}, \
    										{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}, \
    										{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},  \
    									};
	/*{UUID_SERV_SENSOR, BLE_UUID_TYPE_BLE}, \*/

   // my_ble.next_adv_mode=BLE_NO_ADV;
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          		= BLE_ADVDATA_SHORT_NAME;
    advdata.short_name_len = 7; // Advertise only first 7 letters of name
    advdata.include_appearance 		= false;
	advdata.flags              		= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
 	advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
	advdata.service_data_count   	= 1;
    advdata.p_manuf_specific_data 	= &manuf_data;
    int8_t tx_power                 = -4;
    advdata.p_tx_power_level        = &tx_power;



    ble_adv_modes_config_t options = {0};
    /*
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL_SLOW;
    options.ble_adv_fast_timeout  = 180;//APP_ADV_TIMEOUT_IN_SECONDS;
*/
/*

    ble_advdata_manuf_data_t        manuf_data_response;
    uint8_t                         data_response[] = "Many_bytes_of_data";
    manuf_data_response.data.p_data              = data_response;
    manuf_data_response.data.size                = sizeof(data_response);

    ble_advdata_t   advdata_response;// Declare and populate a scan response packet

     // Always initialize all fields in structs to zero or you might get unexpected behaviour
     memset(&advdata_response, 0, sizeof(advdata_response));
     // Populate the scan response packet
     advdata_response.name_type               = BLE_ADVDATA_NO_NAME;
     advdata_response.p_manuf_specific_data   = &manuf_data_response;

    err_code = ble_advertising_init(&advdata, &advdata_response, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
*/
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);



	my_ble.adv.interval 				= APP_ADV_INTERVAL_SLOW;


}





/**@brief Function for starting advertising.
 */
void advertising_start(ble_advertising_mode_t mode)
{
  
    //ble_gap_whitelist_t  whitelist;
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    ble_gap_addr_t       peer_address;

    // Initialize advertising parameters with defaults values
    memset(&adv_params, 0, sizeof(adv_params));
    // Configure advertisement according to current advertising state
	my_ble.adv.current_mode=mode;

	switch (my_ble.adv.current_mode)
    {
        case BLE_NO_ADV:
			my_ble.adv.next_mode=BLE_NO_ADV;
			err_code=sd_ble_gap_adv_stop();
			APP_ERROR_CHECK(err_code);
			break;


        case BLE_DIRECTED_ADV:
		   adv_params.p_peer_addr = &peer_address;
		   adv_params.type        = BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
		   adv_params.timeout     = 0;

		   m_direct_adv_cnt--;
		   if (m_direct_adv_cnt == 0)
		   {
			   my_ble.adv.next_mode  = BLE_FAST_ADV_WHITELIST;
		   }
             break;

        case BLE_SLOW_ADV:
			adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND; //Connectable undirected -> any central can connect to device
			adv_params.p_peer_addr = NULL;
			adv_params.fp          = BLE_GAP_ADV_FP_ANY; //Allow scan requests and connect requests from any device
			adv_params.p_whitelist = NULL;
            adv_params.interval = my_ble.adv.interval;
            adv_params.timeout  = APP_SLOW_ADV_TIMEOUT_S;//my_ble.adv.timeout_slow;
			my_ble.adv.next_mode=BLE_FAST_ADV;
		    // Start advertising
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
		    err_code = sd_ble_gap_adv_start(&adv_params);
		    APP_ERROR_CHECK(err_code);
			break;

        case BLE_FAST_ADV:
			#ifdef _UART_ENABLE_
			printf((const char *)"BLE_FAST_ADV!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
			#endif
			adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND; //Connectable undirected -> any central can connect to device
			adv_params.p_peer_addr = NULL;
			adv_params.fp          = BLE_GAP_ADV_FP_ANY; //Allow scan requests and connect requests from any device
			adv_params.p_whitelist = NULL;
			adv_params.interval = APP_ADV_INTERVAL_FAST; 				//my_ble.adv.interval_very_fast;
			adv_params.timeout  = APP_FAST_ADV_TIMEOUT_S;		//my_ble.adv.timeout_very_fast_default;
			my_ble.adv.next_mode= BLE_SLOW_ADV;
		    // Start advertising
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
		    err_code = sd_ble_gap_adv_start(&adv_params);
		    APP_ERROR_CHECK(err_code);
			break;

        case BLE_BUTTON_ADV:
			#ifdef _UART_ENABLE_
			printf((const char *)"BLE_BUTTON_ADV\n");
			#endif
			adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND; //Connectable undirected -> any central can connect to device
			adv_params.p_peer_addr = NULL;
			adv_params.fp          = BLE_GAP_ADV_FP_ANY; //Allow scan requests and connect requests from any device
			adv_params.p_whitelist = NULL;
			adv_params.interval = APP_ADV_INTERVAL_BUTTON; 				//my_ble.adv.interval_very_fast;
			adv_params.timeout  = APP_BUTTON_ADV_TIME_S;		//my_ble.adv.timeout_very_fast_default;
			my_ble.adv.next_mode= BLE_SLOW_ADV;
		    // Start advertising
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
		    err_code = sd_ble_gap_adv_start(&adv_params);
		    APP_ERROR_CHECK(err_code);
			break;

        case BLE_SAFETY_MODE_ADV:
			#ifdef _UART_ENABLE_
			printf((const char *)"BLE_SAFETY_MODE_ADV\n");
			#endif
			adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND; //Connectable undirected -> any central can connect to device
			adv_params.p_peer_addr = NULL;
			adv_params.fp          = BLE_GAP_ADV_FP_ANY; //Allow scan requests and connect requests from any device
			adv_params.p_whitelist = NULL;
			adv_params.interval = APP_ADV_INTERVAL_SAFETY_MODE; 				//my_ble.adv.interval_very_fast;
			adv_params.timeout  = APP_SAFETY_MODE_ADV_TIMEOUT_S;	//my_ble.adv.timeout_very_fast_safetymode;
			my_ble.adv.next_mode= BLE_SLOW_ADV;
		    // Start advertising
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
		    err_code = sd_ble_gap_adv_start(&adv_params);
		    APP_ERROR_CHECK(err_code);
			break;

        case BLE_CONNECTED_ADV:
			#ifdef _UART_ENABLE_
			printf((const char *)"BLE_ADV2\n");
			#endif
        	adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND; //Connectable undirected -> any central can connect to device
			adv_params.p_peer_addr = NULL;
			adv_params.fp          = BLE_GAP_ADV_FP_ANY; //Allow scan requests and connect requests from any device
			adv_params.p_whitelist = NULL;
			adv_params.interval = APP_ADV_INTERVAL_CONNECTED; 				//my_ble.adv.interval_very_fast;
		    adv_params.timeout  = APP_CONNECTED_ADV_TIMEOUT_S;//my_ble.adv.timeout_slow;
			my_ble.adv.next_mode= BLE_SLOW_ADV;
		    // Start advertising
		    advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
		    err_code = sd_ble_gap_adv_start(&adv_params);
		    APP_ERROR_CHECK(err_code);
			break;

        case BLE_ACTIVE_SCAN_ADV:
 			#ifdef _UART_ENABLE_
 			printf((const char *)"BLE ACTIVE SCAN ADV %lu ms\n",time_stamp);
 			time_stamp=0;

 			#endif
 			adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND; //Connectable undirected -> any central can connect to device
 			adv_params.p_peer_addr = NULL;
 			adv_params.fp          = BLE_GAP_ADV_FP_ANY; //Allow scan requests and connect requests from any device
 			adv_params.p_whitelist = NULL;
 			adv_params.interval = APP_ADV_INTERVAL_ACTIVE_SCAN; 				//my_ble.adv.interval_very_fast;
 			adv_params.timeout  = APP_ACTIVE_SCAN_ADV_DURATION_S;		//my_ble.adv.timeout_very_fast_default;
 			my_ble.adv.next_mode= BLE_SLOW_ADV;
 		    // Start advertising
            advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
 		    err_code = sd_ble_gap_adv_start(&adv_params);
 		    APP_ERROR_CHECK(err_code);
 			break;

        default:
            // No implementation needed.
            break;
    }
}

//Device Information Service Initialization.
void dis_init(void)
{
	uint32_t       err_code;
	ble_dis_init_t   dis_init;
		
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,     	 FIRMWARE_REV);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,     	 HARDWARE_REV);	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

}
//Battery Status Service Initialization
void bas_init(void)
{
	uint32_t       err_code;
	ble_bas_init_t bas_init;
    // Initialize Battery Service.
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

}
//Device Firmware Update Initialization 
void dfu_init(void)
{
	uint32_t       err_code;
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);

}



/**@brief Function for initializing the services that will be used by the application.
 */
void services_init(void)
{

	dis_init(); 				//Device Information Service Initialization.
	nus_services_init(); 		//uart service initialization
	bas_init();					//Battery Status Service Initialization
	dfu_init();					//Device Firmware Update Initialization 
//	serv_controller_init();		//Custom Service Controller Initialization
//	serv_sensor_init();			//Custom Service Sensor Initialization

}

void bluetooth_init(void)
{
	my_ble.connected=false;
	ble_stack_init(1);	//Bluetooth Stack Init
	radio_notification_init();
}







void bluetooth_read(MY_BLE_DEV *dev)
{
	memcpy(dev,&my_ble,sizeof(MY_BLE_DEV));
}



void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
			#ifdef _DEVICE_MANAGER_EN_
        	if (m_memory_access_in_progress)
            {
				#ifdef _UART_ENABLE_
				printf("NRF_EVT_FLASH_OPERATION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
				#endif
                m_memory_access_in_progress = false;
                advertising_start(adv_pendent_mode);	//Recupera o advertise que não executou enquanto escrevia na flash
            }
			#endif
        	break;
        default:
            // No implementation needed.
            break;
    }
}

void battery_adv_update(void)
{
	if(my_ble.connected==false)
	{
		advertising_start(BLE_NO_ADV);		//Advertise Stop
		advertising_start(BLE_SLOW_ADV);	//Advertise re-start in SLOW ADV
	}
}

void button_adv_set(void)
{
	uint8_t p_data[2];
	uint32_t err_code;
	if(my_ble.connected==false)
	{
		advertising_start(BLE_NO_ADV);
		advertising_start(BLE_BUTTON_ADV);
		p_data[0]=0xAA;
		p_data[1]=0x55;
		//err_code=sd_ble_gap_adv_data_set(p_data,2,NULL,0);
	    //APP_ERROR_CHECK(err_code);
		nrf_gpio_pin_set(BSP_LED_0);					//SET OFF
		nrf_gpio_pin_set(BSP_LED_1);					//SET OFF
		nrf_gpio_pin_set(BSP_LED_2);					//SET OFF
	}
}





/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */

static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

    switch (p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
           // m_bonded_peer_handle = (*p_handle);
            break;
        case DM_EVT_LINK_SECURED:
        	app_context_load(p_handle);
        	break;
    }
    return NRF_SUCCESS;



}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
void device_manager_init(bool erase_bonds)
{

    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;


    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);

}


/*

void timer_pwm_off_handler(void * p_context)
{

    nrf_pwm_set_value(0, 0);
	nrf_pwm_set_enabled(false);
    apply_pan73_workaround(PWM_TIMER, false);
    PWM_TIMER->TASKS_STOP = 1;

}
*/
void timer_send_nus_handler(void * p_context)
{
	uint16_t tmp;
	float x,y,z;

	static uint16_t prev_ps;
	static uint16_t prev_battery;
	
	if(my_ble.connected)
	{	
		timer_send_nus_start(1000);

	//buffer de NUS print e de 20bytes

		sprintf(buf_aux,"%.1fmbar/%.1fm",my_ble.pressure.pressure/100.0, my_ble.altitude.altitude);
		nus_printStr(buf_aux);
		read_temp(&x);		
		sprintf(buf_aux,"T1: %.1fC / %.1fC",my_ble.temp.temp,x);
		nus_printStr(buf_aux);
		sprintf(buf_aux,"AL: %ulm / %u",my_ble.light.light, my_ble.proximity.proximity);
		nus_printStr(buf_aux);
		read_acc(&x,&y,&z);
		sprintf(buf_aux,"A: %.1f/%.1f/%.1f",x,y,z);
		nus_printStr(buf_aux);
/*		read_gyro(&x,&y,&z);
		sprintf(buf_aux,"x: %.1f",x);
		nus_printStr(buf_aux);
		sprintf(buf_aux,"y: %.1f",y);
		nus_printStr(buf_aux);
		sprintf(buf_aux,"z: %.1f",z);
		nus_printStr(buf_aux);
*/
/*
		read_angle(&x,&y);
		sprintf(buf_aux,"x: %.1f",x);
		nus_printStr(buf_aux);
		sprintf(buf_aux,"y: %.1f",y);
		nus_printStr(buf_aux);

*/



		read_gyro(&x,&y,&z);
		sprintf(buf_aux,"Gx: %.2f",x);
		nus_printStr(buf_aux);
		sprintf(buf_aux,"Gy: %.2f",y);
		nus_printStr(buf_aux);
		sprintf(buf_aux,"Gz: %.2f",z);
		nus_printStr(buf_aux);
		
/*
		read_temp(&x);
		sprintf(buf_aux,"T2:%.2fC",x);
		nus_printStr(buf_aux);


		if(prev_battery!=battery_level())
		{
			prev_battery=battery_level();
			
			sprintf(buf_aux,"BATT: %u\%",battery_level());
			nus_printStr(buf_aux);
		}	
		
		if(prev_ps!=my_ble.proximity.proximity)
		{
			prev_ps=my_ble.proximity.proximity;
		}	
*/
/*
		
*/
		nus_printStr("-----------------");
		}

}


void timer_mpu6050_handler(void * p_context)
{
	timer_mpu6050_start(1); //1KHz

	if(my_ble.connected)
		mpu6050_enable();
	else
		mpu6050_disable();
    mpu6050_loop();

}

void timer_ps_handler(void * p_context)
{
	timer_ps_start(1000);

	my_ble.proximity.proximity=ap3216c_read_ps_val();
	my_ble.light.light=ap3216c_read_als_val();
	ap3216c_start_once_als_ps();
}

void timer_bmp180_handler(void * p_context)
{
		
		timer_bmp180_start(1000);
		my_ble.temp.temp = bmp180GetTemperature(bmp180ReadUT()); //MUST be called first
		my_ble.pressure.pressure = bmp180GetPressure(bmp180ReadUP());
		my_ble.pressure.atm = my_ble.pressure.pressure / 101325; // "standard atmosphere"
		my_ble.altitude.altitude = calcAltitude(my_ble.pressure.pressure); //Uncompensated caculation - in Meters




		


}


