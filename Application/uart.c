#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "softdevice_handler.h"
#include "app_uart.h"
#include "nrf51_bitfields.h"

#include "app_error.h"
#include "boards.h"
#include "uart.h"

#ifdef UART_EN
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}



void uart_init(void)
{
	uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };
		//nrf_gpio_cfg_output(SER_APP_TX_PIN);
		NRF_UART0->ENABLE = 1;		//UART ENABLED
		NRF_UART0->TASKS_STARTTX = 1;
		NRF_UART0->TASKS_STARTRX = 1; //RX Enabled

		APP_UART_FIFO_INIT(&comm_params, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, uart_error_handle, APP_IRQ_PRIORITY_LOW, err_code);

		APP_ERROR_CHECK(err_code);


}


#endif



