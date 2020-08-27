#include <stdbool.h>
#include "mainfile.h"
#include "pstorage.h"

/**@brief Function for initializing low frequency clock.
 */
void clock_initialization(void)
{
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }





}
/**@brief Pin I/O Board Init
 *
 *
 */
void init_io(void)
{
	//////////////////////////////////////////////////////////////////////////////////////////////////
	/*Inicializar (todos) os pinos como entradas com (pull-up ou pull-down) ou saídas (low/Hi level)*/
	nrf_gpio_cfg_input(1, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(2, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(3, NRF_GPIO_PIN_PULLDOWN);

	nrf_gpio_cfg_input(7, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(8, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(9, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(10, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(11, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(12, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(13, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(14, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(15, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(16, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(17, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(18, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(19, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(20, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(21, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(22, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(23, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(24, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(25, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(28, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(29, NRF_GPIO_PIN_PULLDOWN);
	//////////////////////////////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////////////////////////
	nrf_gpio_cfg_output(BSP_LED_0);					//OUTUPUT DIRECTION
	nrf_gpio_pin_set(BSP_LED_0);						//SET OFF

	nrf_gpio_cfg_output(BSP_LED_1);					//OUTUPUT DIRECTION
	nrf_gpio_pin_clear(BSP_LED_1);					//SET OFF

	nrf_gpio_cfg_output(BSP_LED_2);					//OUTUPUT DIRECTION
	nrf_gpio_pin_clear(BSP_LED_2);					//SET OFF

	nrf_gpio_cfg_output(BSP_LED_2);				//OUTUPUT DIRECTION
	nrf_gpio_pin_clear(BSP_LED_2);					//SET OFF

	//nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP); //INPUT DIRECTION
	// Configure button with sense level low as wakeup source.
	//nrf_gpio_cfg_sense_input(BUTTON_2,NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
	//////////////////////////////////////////////////////////////////////////////////////////////////

}

/**@brief Init global do sistema
 *
 *
 */

void init(void)
{
	DEVICE lapa;
	uint32_t err_code;
	// Initialize.

	//err_code = NRF_LOG_INIT();
	//APP_ERROR_CHECK(err_code);

	init_io();										//Init IO PINS

	NRF_POWER->TASKS_LOWPWR = 1;
	NRF_SPI0->ENABLE = 0;							//SPI Disabled
	NRF_UART0->TASKS_STOPTX = 1;
	NRF_UART0->TASKS_STOPRX = 1;
	NRF_UART0->ENABLE = 0;							//UART Disabled

	clock_initialization();	  						// Initialize Clock Sys.

	timers_init();									//Inicia timers de baixa frequencia

#ifdef UART_EN
	uart_init();
	//printf("UART\n");

#endif

	bluetooth_init();								//Inicia stack
/*!
 * The SoftDevice and scheduler must be initialized first before initializing the Persistent Storage Module.
 * Modules that use this storage module shall then be initialized after the storage module.
 * Block size and offset in load, store and update shall be a multiple of word size (4 bytes).
 * In case blocks span across multiple flash pages update and clear of blocks might not work as expected.
 * The application is expected to ensure that when a System OFF is issued, flash access is not ongoing or queued with the module.
 * Writing data to blocks that already have data stored is unreliable. The application must clear the blocks before writing fresh data to it.
 * No intermediate copy of source data to be stored in flash is made. Application cannot free or reuse the memory that is the source of data until this operation is complete.
 * Completion of this operation is notified through the event notification callback.
 * */
	device_manager_init(true);
	gap_params_init();								//Generic Profile Init
//	get_encrypted_name_for_authentication();		//Initialização da chave de autenticação para app mobile


	services_init();								//Inicialização dos serviços Ble
	advertising_init(BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE);	//Inicialização das flags de advertise

	conn_params_init();								//Inicialiazção dos parametros de ligação bluetooth

	pwm_init();										//Inicialização do periférico bluetooth
	adc_configure();								//Inicialização do periférico ADC/Battery Reader
	nrf_pwm_set_enabled(false);

	advertising_start(BLE_SLOW_ADV);							//Faz Slow Advertise


	if(twi_master_init())
	{
		if(mpu6050_init(0x69)) //0x69 MPU6050
		{
			mpu6050_setup();
			nrf_gpio_pin_set(BSP_LED_0);					//SET OFF
			timer_mpu6050_start(1000);
		}

		nrf_gpio_pin_set(BSP_LED_1);					//SET OFF
		if(ap3216c_init(0x1E))		//0x1E AP3216C
		{
			//init_ext_interrupt();
			ap3216c_setup();
			//nrf_gpio_pin_set(BSP_LED_1);					//SET OFF
			//ap3216c_register_write(0x00,0x07);
			timer_ps_start(1000);
		}

		if(bmp180_init(0x77)) //0x77 BMP180
		{
			bmp180_cal();
			nrf_gpio_pin_set(BSP_LED_2);					//SET OFF
			timer_bmp180_start(1000);
		}

	}

//	nrf_pwm_set_enabled(true);

//	nrf_pwm_set_value(0,128);


	button_init();													//Inicializa frequencia de amostragem do estado do botão -> modo operacional
	battery_start_read();											//Inicialização da Thread para leitura da voltagem na bateria
//	timer_active_scan_start(1000);
//	timer_ps_start(500);


	

}

