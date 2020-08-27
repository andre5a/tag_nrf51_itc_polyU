/*
 * ble_uart.h
 *
 *  Created on: 23/04/2017
 *      Author: andre
 */

#ifndef BLE_UART_H_
#define BLE_UART_H_

extern ble_nus_t                        m_nus;

void nus_print(unsigned char *data_in, uint16_t len);
void nus_pchar(unsigned char character);
void nus_services_init(void);

void nus_put_buff(unsigned char character);

bool nus_gchar(unsigned char *character);

uint16_t nus_rx_cnt(void);

void nus_printStr(char *str);


#endif /* BLE_UART_H_ */
