#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_button.h"
#include "ble_hci.h"
#include "app_error.h"
#include "softdevice_handler.h"

#include "timers.h"
#include "pm.h"
#include "ble_controller_serv.h"

#include "bluetooth.h"

#include "state_machine.h"
#include "button.h"

#define NUM_BUTTONS					1

#define BUTTON_NO_PRESS_CHAR_DATA		0x00
#define BUTTON_SHORT_PRESS_CHAR_DATA	0x10
#define BUTTON_LONG_PRESS_CHAR_DATA		0x20
#define BUTTON_DOUBLE_PRESS_CHAR_DATA	0x30
#define BUTTON_SAFETY_FAR_CHAR_DATA		0x40
//#define BUTTON_SAFETY_DATA				0x50



static STATE_MACHINE 				button_st[NUM_BUTTONS];
static uint16_t 					button_tout[NUM_BUTTONS]={0};//,0};
static uint16_t						button_count[NUM_BUTTONS]={0};//,0};

static bool button_busy_flag=false;
static uint8_t button=0;

static void button_state_handler(uint8_t pin_no, uint8_t button_action);
static const app_button_cfg_t app_buttons[NUM_BUTTONS]=   {{BUTTON_1, false, BUTTON_PULL, button_state_handler}};//,{BUTTON_2, false, BUTTON_PULL, button_state_handler}};
void button_core(unsigned char button_num);






#define BUTTON_INIT 					0
#define BUTTON_PRESSED1_INIT			1
#define BUTTON_PRESSED1					2
#define BUTTON_UNPRESSED_INIT 			3
#define BUTTON_UNPRESSED 				4
#define BUTTON_PRESSED2_INIT			5
#define BUTTON_PRESSED2					6
#define BUTTON_VALIDATED				7
#define BUTTON_WAIT_END					8

/*
#define BUTTON_TIME_10MS_PRESSED		1
#define BUTTON_TIME_10MS_UNPRESSED		0
#define BUTTON_TOUT_10MS				10//50
*/
#define BUTTON_LONG_TIME_PRESSED		30//75

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void button_state_handler(uint8_t pin_no, uint8_t button_action)
{
	if(button_busy()==false)
		if (button_action == APP_BUTTON_PUSH)
		{
			switch(pin_no)
			{
			case BUTTON_1:
				button=0;
				button_core(button);
			break;
			default:
				break;

			}
		}

}
void button_core(unsigned char button_num)
{

	switch(button_st[button_num].cstate)
	{
		case BUTTON_INIT:
			button_st[button_num].nxstate=BUTTON_INIT;
			button_tout[button_num]=0;
			button_count[button_num]=0;
			button_busy_flag=false;
			timer_button_state_stop();
			nrf_gpio_pin_set(LED_RGB_RED);					//SET OFF
			nrf_gpio_pin_set(LED_RGB_GREEN);					//SET OFF
			nrf_gpio_pin_set(LED_RGB_BLUE);					//SET OFF
			switch(button_num)
			{
			case 0:
				if(!nrf_gpio_pin_read(BUTTON_1))
				{

					button_busy_flag=true;
					timer_button_state_start(25);
					button_st[button_num].nxstate=BUTTON_PRESSED1_INIT;
					nrf_gpio_pin_clear(LED_RGB_GREEN);					//SET ON

				}
			break;
			}
			break;
		case BUTTON_PRESSED1_INIT:
			button_tout[button_num]=0;
			button_count[button_num]=0;
			button_st[button_num].nxstate=BUTTON_PRESSED1;
			//no break
		case BUTTON_PRESSED1:
			timer_button_state_start(25);
			button_st[button_num].nxstate=BUTTON_UNPRESSED_INIT;
			switch(button_num)
			{
			case 0:
				if(!nrf_gpio_pin_read(BUTTON_1))
				{
					button_st[button_num].nxstate=BUTTON_PRESSED1;
					if((++button_count[button_num])>=BUTTON_LONG_TIME_PRESSED)
					{
						button_st[button_num].nxstate=BUTTON_WAIT_END;
						button1_long_press_evt();
					}
				}
				break;
			case 1:
				if(!nrf_gpio_pin_read(BUTTON_1))
				{
					button_st[button_num].nxstate=BUTTON_PRESSED1;
					if((++button_count[button_num])>=BUTTON_LONG_TIME_PRESSED)
					{
						button_st[button_num].nxstate=BUTTON_WAIT_END;
						button1_long_press_evt();
					}
				}
				break;

			}
			break;
		case BUTTON_UNPRESSED_INIT:
			button_tout[button_num]=0;
			button_count[button_num]=0;
			button_st[button_num].nxstate=BUTTON_UNPRESSED;
			//no break
		case BUTTON_UNPRESSED:
			timer_button_state_start(25);
			button_st[button_num].nxstate=BUTTON_PRESSED2_INIT;
			switch(button_num)
			{
			case 0:
				if(nrf_gpio_pin_read(BUTTON_1))
				{
					button_st[button_num].nxstate=BUTTON_UNPRESSED;
					if((++button_count[button_num])>=15)
					{
						button_st[button_num].nxstate=BUTTON_WAIT_END;
						button1_short_press_evt();
					}
				}
				break;
			case 1:

				break;
			}
			break;
		case BUTTON_WAIT_END:
			timer_button_state_start(25);
			switch(button_num)
			{
			case 0:
			if(nrf_gpio_pin_read(BUTTON_1))
				button_st[button_num].nxstate=BUTTON_INIT;
			break;
			}
			break;
		default:
			button_st[button_num].nxstate=BUTTON_INIT;

			break;
	}
	button_st[button_num].pvstate=button_st[button_num].cstate;
	button_st[button_num].cstate=button_st[button_num].nxstate;


}

void timer_button_state_handler(void * p_context)
{

	button_core(button);

}


bool button_busy(void)
{
	//if(button_st.cstate!=BUTTON_INIT)
	if(button_busy_flag)
		return true;
	return false;

}




void button1_short_press_evt(void)
{
	MY_BLE_DEV blt_dev;
	bluetooth_read(&blt_dev);
	#ifdef _UART_ENABLE_
	printf("Button: short press\r\n");
	#endif
	//else
	//{
		button_adv_set();
	//}

}

void button1_long_press_evt(void)
{

	MY_BLE_DEV blt_dev;
		bluetooth_read(&blt_dev);

	if((blt_dev.connected==true))
	{
//		button_char_update(BUTTON_LONG_SIG);
	}
	else
	{
	//	if(blt_dev.connected==false)
	//		button_adv_set();
	}

	//activate GPREGRET register for activating bootloader after system reset
	//sd_power_gpregret_set(1);
	//system reset
	//sd_nvic_SystemReset();


#ifdef _UART_ENABLE_
printf("Button: long press\r\n");
#endif

}
void button1_double_press_evt(void)
{
	MY_BLE_DEV blt_dev;

	bluetooth_read(&blt_dev);
//	if(blt_dev.connected==true)
//		button_char_update(BUTTON_DOUBLE_SIG);



#ifdef _UART_ENABLE_
printf("Button: double press\r\n");
#endif

}


void button_init(void)
{
   uint32_t err_code = app_button_init((app_button_cfg_t *)app_buttons,NUM_BUTTONS, 5);//APP_TIMER_TICKS(50, APP_TIMER_PRESCALER/2));

    if (err_code == NRF_SUCCESS)
    {
        err_code = app_button_enable();
    }
}

