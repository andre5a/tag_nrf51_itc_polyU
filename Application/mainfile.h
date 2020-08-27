#ifndef MAINFILE_H__
#define MAINFILE_H__
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "nrf_delay.h"
#include "app_error.h"
#include "boards.h"
//#include "radio_test.h"
#include "nrf_drv_gpiote.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
#include "ble_conn_params.h"
#include "nrf_pwm.h"
#include "pwm.h"
#include "timers.h"
#include "bluetooth.h"
#include "adc.h"
#include "pm.h"
#include "uart.h"
#include "button.h"
#include "flash.h"
#include "pstorage.h"
#include "twi_master.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "ap3216c.h"
#include "door.h"
void init(void);

#endif

