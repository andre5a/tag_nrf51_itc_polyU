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


#ifndef BLE_SENSOR_SERV_H__
#define BLE_SENSOR_SERV_H__

#include "ble_srv_common.h"
#include "nrf_soc.h"


#define UUID_SERV_SENSOR		 								0x1500													/**< Proprietary UUID for local service. */
#define UUID_CHAR_ACEL_X			 							0x1501													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_ACEL_Y									 	0x1502													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_ACEL_Z										0x1503													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_GYRO_X										0x1504													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_GYRO_Y										0x1505													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_GYRO_Z					 					0x1506													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_TEMP						 					0x1507													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_LIGHT						 					0x1508													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_PRESSURE					 					0x1509													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_GND_STATION					 				0x150A													/**< Proprietary UUID for local characteristic. */
#define UUID_CHAR_FW_UPDATE										0x150B
#define UUID_CHAR_ALTITUDE					 					0x150C													/**< Proprietary UUID for local characteristic. */

typedef enum
{
		CHAR_ACEL_X_SUBSCRIBED=0,
		CHAR_ACEL_X_UNSUBSCRIBED=1,
		CHAR_ACEL_Y_SUBSCRIBED=2,
		CHAR_ACEL_Y_UNSUBSCRIBED=3,
		CHAR_ACEL_Z_SUBSCRIBED=4,
		CHAR_ACEL_Z_UNSUBSCRIBED=5,

		CHAR_GYRO_X_SUBSCRIBED=6,
		CHAR_GYRO_X_UNSUBSCRIBED=7,
		CHAR_GYRO_Y_SUBSCRIBED=8,
		CHAR_GYRO_Y_UNSUBSCRIBED=9,
		CHAR_GYRO_Z_SUBSCRIBED=10,
		CHAR_GYRO_Z_UNSUBSCRIBED=11,



		CHAR_GND_STATION_SUBSCRIBED=12,
		CHAR_GND_STATION_UNSUBSCRIBED=13,

		CHAR_PRESSURE_SUBSCRIBED=14,
		CHAR_PRESSURE_UNSUBSCRIBED=15,

		CHAR_TEMP_SUBSCRIBED=16,
		CHAR_TEMP_UNSUBSCRIBED=17,

		CHAR_LIGHT_SUBSCRIBED=18,
		CHAR_LIGHT_UNSUBSCRIBED=19,

		CHAR_ALTITUDE_SUBSCRIBED=20,
		CHAR_ALTITUDE_UNSUBSCRIBED=21,


} ble_extra_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
	ble_extra_evt_type_t evt_type;                                  /**< Type of event. */
} ble_sensor_evt_t;

// Forward declaration of the ble_extra_t type. 
typedef struct ble_sensor_s ble_sensor_t;

/**@brief Battery Service event handler type. */
typedef void (*ble_sensor_evt_handler_t) (ble_sensor_t * p_sensor, ble_sensor_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
	ble_sensor_evt_handler_t   				evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    bool                       				support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_srv_report_ref_t *     				p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */

    ble_srv_cccd_security_mode_t  			acel_x_char_attr_md;
	ble_srv_cccd_security_mode_t  			acel_y_char_attr_md;
	ble_srv_cccd_security_mode_t  			acel_z_char_attr_md;
	ble_srv_cccd_security_mode_t  			gyro_x_char_attr_md;
	ble_srv_cccd_security_mode_t  			gyro_y_char_attr_md;
	ble_srv_cccd_security_mode_t  			gyro_z_char_attr_md;
	ble_srv_cccd_security_mode_t  			ground_station_char_attr_md;
	ble_srv_cccd_security_mode_t  			pressure_char_attr_md;
	ble_srv_cccd_security_mode_t  			temp_char_attr_md;
	ble_srv_cccd_security_mode_t  			light_char_attr_md;
	ble_srv_cccd_security_mode_t  			altitude_char_attr_md;

	ble_gap_conn_sec_mode_t       			acel_x_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			acel_y_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			acel_z_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			gyro_x_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			gyro_y_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			gyro_z_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			ground_station_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			pressure_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			temp_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			light_report_read_perm; /**< Initial security level for battery report read attribute */
	ble_gap_conn_sec_mode_t       			altitude_report_read_perm;

	int16_t 								initial_acel_x_level;
	int16_t 								initial_acel_y_level;
	int16_t 								initial_acel_z_level;
	int16_t 								initial_gyro_x_level;
	int16_t 								initial_gyro_y_level;
	int16_t 								initial_gyro_z_level;
	int16_t 								initial_batt_level;
	uint8_t 								initial_ground_station_level;
	uint32_t 								initial_pressure_level;
	int8_t 									initial_temp_level;
	uint16_t 								initial_light_level;
	uint16_t 								initial_altitude_level;

} ble_sensor_init_t;

/**@brief Extra Service structure. This contains various status information for the service. */
typedef struct ble_sensor_s
{
	ble_sensor_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    uint16_t                    service_handle;                 /**< Handle of Battery Service (as provided by the BLE stack). */
    uint16_t                    report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint16_t                    conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                        is_notification_supported;      /**< TRUE if notification of Battery Level is supported. */
	ble_gatts_char_handles_t 	acel_x_handles;
	ble_gatts_char_handles_t 	acel_y_handles;
	ble_gatts_char_handles_t 	acel_z_handles;
	ble_gatts_char_handles_t  	gyro_x_handles;      /**< Advertising Timeout Slow Handles related to the BLE Settings characteristic. */
	ble_gatts_char_handles_t 	gyro_y_handles;
	ble_gatts_char_handles_t	gyro_z_handles;
	ble_gatts_char_handles_t	ground_station_handles;
	ble_gatts_char_handles_t	pressure_handles;
	ble_gatts_char_handles_t	temp_handles;
	ble_gatts_char_handles_t	light_handles;
	ble_gatts_char_handles_t	altitude_handles;

	int16_t                    	gyro_x_last;
    int16_t                    	gyro_y_last;
    int16_t                    	gyro_z_last;
    int16_t                    	acel_x_last;
    int16_t                    	acel_y_last;
    int16_t                    	acel_z_last;
    int8_t                    	ground_station_last;
    uint32_t                    	pressure_last;
    int8_t                    	temp_last;
    uint16_t                    	light_last;
    uint16_t                    	altitude_last;

} ble_sensor_t;

extern ble_sensor_t    m_sensor;
/**@brief Function for initializing the Battery Service.
 *
 * @param[out]  p_bas       Battery Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_bas_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
 uint32_t ble_sensor_serv_init(ble_sensor_t * p_extra, const ble_sensor_init_t * p_extra_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_bas_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_bas      Battery Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_sensor_on_ble_evt(ble_sensor_t * p_extra, ble_evt_t * p_ble_evt);
//uint32_t battery_level_update(ble_sensor_t * p_sensor, uint16_t battery_level);

uint32_t acel_x_update_char(ble_sensor_t * p_extra, int16_t button_test);
uint32_t acel_y_update_char(ble_sensor_t * p_extra, int16_t button_test);
uint32_t acel_z_update_char(ble_sensor_t * p_extra, int16_t button_test);

uint32_t gyro_x_update_char(ble_sensor_t * p_extras, int16_t button_test);
uint32_t gyro_y_update_char(ble_sensor_t * p_sensor, int16_t button_test);
uint32_t gyro_z_update_char(ble_sensor_t * p_extras, int16_t button_test);
uint32_t ground_station_update_char(ble_sensor_t * p_sensor, uint8_t ground_station_level);
uint32_t pressure_update_char(ble_sensor_t * p_sensor, uint32_t pressure_level);
uint32_t light_update_char(ble_sensor_t * p_sensor, uint16_t light_level);
uint32_t temp_update_char(ble_sensor_t * p_sensor, int8_t temp_level);
uint32_t altitude_update_char(ble_sensor_t * p_sensor, uint16_t altitude_level);

#endif // BLE_BAS_H__

/** @} */
