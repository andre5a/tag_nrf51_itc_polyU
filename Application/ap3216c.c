/*
 * ap3216c.c
 *
 *  Created on: 11/06/2017
 *      Author: andre
 */


#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"
#include "boards.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "ap3216c.h"
#include "nordic_common.h"
#include "nrf_gpio.h"


#include <string.h>
#include "app_uart.h"
#include "ble_nus.h"
#include "ble_uart.h"


#define ADDRESS_WHO_AM_I          (0x00U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0x00U) // !<

//static STATE_MACHINE 				ap3216c_st;


static bool int_flag=false;
static char buf_aux[100];


static const uint8_t expected_who_am_i = 0x00U; // !< Expected value to get from WHO_AM_I register.
static uint8_t       m_device_address;          // !< Device address in bits [7:1]

bool ap3216c_init(uint8_t device_address)
{
    bool transfer_succeeded = true;

    m_device_address = (uint8_t)(device_address << 1);

    // Read and verify product ID
    transfer_succeeded &= ap3216c_verify_product_id();

    return transfer_succeeded;
}

bool ap3216c_verify_product_id(void)
{
    uint8_t who_am_i;

    if (ap3216c_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1))
    {
        if (who_am_i != expected_who_am_i)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool ap3216c_register_write(uint8_t register_address, uint8_t value)
{
    uint8_t w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_master_transfer(m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}

bool ap3216c_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}

void ap3216c_write16(uint8_t regAddress, uint16_t value) {
    uint8_t w2_data[3];

    w2_data[0] = regAddress;
    w2_data[1] = value & 0x00FF;
    w2_data[2] = value>>8;
    twi_master_transfer(m_device_address, w2_data, 3, TWI_ISSUE_STOP);

}
uint16_t ap3216c_read16(uint8_t regAddress) {
    uint8_t w2_data[3];

    ap3216c_register_read(regAddress, w2_data, 2);
return (uint16_t)(w2_data[1]<<8 | w2_data[0]<<0);
}




void ap3216c_write(uint8_t regAddress, uint8_t value) {
	ap3216c_register_write(regAddress,value);
}

uint8_t ap3216c_read(uint8_t regAddress) {
	uint8_t val=0;
	ap3216c_register_read(regAddress, &val, 1);
  return val;
}

void ap3216c_write_ps_thh(uint16_t value) {
uint16_t tmp;

	tmp=value>>2;
	tmp=tmp<<8;
	tmp|=value &0x0003;
	ap3216c_write16(REG_PS_THR_H, tmp);

}


uint16_t ap3216c_read_ps_val(void) {
uint16_t tmp;
uint16_t val;
	tmp=ap3216c_read16(REG_DATA_PS);
	val=tmp &0x000F;
	tmp=tmp&0xFF00;
	tmp=tmp>>4;
	val|=tmp;
	val&=0x3FF;
  return val;
}
uint16_t ap3216c_read_als_val(void) {
uint16_t val;
	val=ap3216c_read16(REG_DATA_ALS);
  return val;
}


void ap3216c_write_ps_thl(uint16_t value) {
uint16_t tmp;

	tmp=value>>2;
	tmp=tmp<<8;
	tmp|=value &0x0003;
	ap3216c_write16(REG_PS_THR_L, tmp);

}

void ap3216c_start_once_als_ps(void)
{
	ap3216c_write(REG_SYS_CONF, 0x07); //Read command

}


void ap3216c_setup(void)
{
	ap3216c_write(REG_SYS_CONF, 0x04); //Reset
	nrf_delay_ms(10);
	ap3216c_write(REG_ALS_CONF,0B00<<4|0B0000<<0);
	ap3216c_write(REG_ALS_CAL,0x40);
	ap3216c_write16(REG_ALS_THR_L, 0x0000);
	ap3216c_write16(REG_ALS_THR_H, 0xFFFF);

	ap3216c_write(REG_PS_CONF,0b1000<<BIT_PS_INTEGRATION |2<<BIT_PS_GAIN | 3<<BIT_PS_INT_FILT);
	ap3216c_write(REG_INT_CLEAR_MNR, 0x00);
	ap3216c_write(REG_PS_LED_CTR,0b01<<BIT_LED_PULSE | 0b01<<BIT_LED_DRIVE);
	ap3216c_write(REG_PS_INT_MODE,0<<0);
	ap3216c_write(REG_PS_MEAN_T,0x03);
	ap3216c_write(REG_PS_LED_W,0x07);
	ap3216c_write16(REG_PS_CAL, 0x0000);
	ap3216c_write_ps_thh(1000);
	ap3216c_write_ps_thl(100);
	ap3216c_write(REG_SYS_CONF, 0x00); //Power Down

}


void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_HITOLO:
            //do something
        	//if(int_flag==false)
        	{
        	int_flag=true;
        	nrf_gpio_pin_toggle(BSP_LED_2);
        	}
        	break;
        default:
            //do nothing
            break;
    }

}

void init_ext_interrupt(void)
{
/*
	    ret_code_t ret_code;

	    ret_code = nrf_drv_gpiote_init();
	    APP_ERROR_CHECK(ret_code);

	    nrf_drv_gpiote_in_config_t config =
	    {
	        .sense = NRF_GPIOTE_POLARITY_HITOLO,
	        .pull = NRF_GPIO_PIN_PULLUP, // NRF_GPIO_PIN_NOPULL ,
	        .is_watcher = false,
	        .hi_accuracy = false
	    };


	    ret_code = nrf_drv_gpiote_in_init(AP3216C_INT_PIN_NUMBER, &config, gpiote_evt_handler);
	    APP_ERROR_CHECK(ret_code);

	    nrf_drv_gpiote_in_event_enable(AP3216C_INT_PIN_NUMBER,true);

*/
}
