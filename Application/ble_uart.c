/*
 * ble_uart.c
 *
 *  Created on: 23/04/2017
 *      Author: andre
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "ble_nus.h"
#include "app_uart.h"
#include "ble_uart.h"


ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */

volatile uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
volatile uint16_t rx_rd_ptr=0;
volatile uint16_t rx_wr_ptr=0;
volatile uint16_t rx_cnt=0;

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	uint8_t data;
	for(uint32_t i=0;i<length;i++)
		nus_put_buff(p_data[i]);

	while(nus_rx_cnt())
	{
		if(nus_gchar(&data))
		nus_pchar(data);
	}
}

/**@snippet [Handling the data received over BLE] */



void nus_print(unsigned char *data_in, uint16_t len)
{

    uint32_t       err_code;


	err_code = ble_nus_string_send(&m_nus, data_in, len);
	if (err_code != NRF_ERROR_INVALID_STATE)
	{
		APP_ERROR_CHECK(err_code);
	}


}

void nus_printStr(char *str)
{

	uint16_t len=strlen(str);
	nus_print((unsigned char*)str, len);
/*	while(*str!='\0')
	{
		nus_pchar(*str);
		str++;

	}
	*/
}

void nus_pchar(unsigned char character)
{

	nus_print(&character, 1);

}

void nus_put_buff(unsigned char character)
{
	data_array[rx_wr_ptr]=character;
	if((++rx_wr_ptr)>=BLE_NUS_MAX_DATA_LEN)
		rx_wr_ptr=0;
	if((++rx_cnt)>=BLE_NUS_MAX_DATA_LEN)
		rx_cnt=0;
}


bool nus_gchar(unsigned char *character)
{
	if(rx_cnt)
	{
		*character=data_array[rx_rd_ptr];
		if((++rx_rd_ptr)>=BLE_NUS_MAX_DATA_LEN)
			rx_rd_ptr=0;
		rx_cnt--;
		return true;
	}
	return false;
}

uint16_t nus_rx_cnt(void)
{
	return rx_cnt;
}


/**@brief Function for initializing services that will be used by the application.
 */
void nus_services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

