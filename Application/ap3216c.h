/*
 * ap3216c.h
 *
 *  Created on: 11/06/2017
 *      Author: andre
 */

#ifndef AP3216C_H_
#define AP3216C_H_


#define REG_SYS_CONF	0x00
#define REG_INT_STATUS 	0x01
#define REG_INT_CLEAR_MNR 	0x02
#define REG_DATA_ALS 	0x0C
#define REG_DATA_PS 	0x0E
#define REG_ALS_CONF 	0x10
#define REG_ALS_CAL 	0x19
#define REG_ALS_THR_L 	0x1A
#define REG_ALS_THR_H 	0x1C
#define REG_PS_CONF 	0x20
#define REG_PS_LED_CTR 	0x21
#define REG_PS_INT_MODE 	0x22
#define REG_PS_MEAN_T 	0x23
#define REG_PS_LED_W 	0x24
#define REG_PS_CAL 	0x28
#define REG_PS_THR_L 	0x2A
#define REG_PS_THR_H 	0x2C

#define BIT_PS_INT_FILT		0
#define BIT_PS_GAIN			2
#define BIT_PS_INTEGRATION	4

#define BIT_LED_PULSE		4
#define BIT_LED_DRIVE		0

bool ap3216c_init(uint8_t device_address);
bool ap3216c_verify_product_id(void);
bool ap3216c_register_write(uint8_t register_address, uint8_t value);
bool ap3216c_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes);
/*lint --flb "Leave library region" */
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void init_ext_interrupt(void);
void ap3216c_loop(void);



uint16_t ap3216c_read_ps_val(void);
uint16_t ap3216c_read_als_val(void);
void ap3216c_write_ps_thl(uint16_t value);
void ap3216c_start_once_als_ps(void);
void ap3216c_setup(void);



#endif /* AP3216C_H_ */
