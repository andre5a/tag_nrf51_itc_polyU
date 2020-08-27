#include <stdint.h>
#include <string.h>
#include "nrf51.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "app_timer.h"
#include "boards.h"
#include "nrf_sdm.h"
//#include "nrf_gpio.h"
#include "app_timer.h"
#include "timers.h"

//static TIMER_ID timer_id;

APP_TIMER_DEF(m_mpu6050_id);                                               /**< Battery measurement timer. */
APP_TIMER_DEF(m_battery_id);                                               /**< Battery measurement timer. */
APP_TIMER_DEF(m_poff_id);                                               /**< Battery measurement timer. */
APP_TIMER_DEF(m_button_state_id);                                               /**< Battery measurement timer. */
APP_TIMER_DEF(m_send_nus_id);                                               /**< Battery measurement timer. */
APP_TIMER_DEF(m_bmp180_id);                                               /**< Battery measurement timer. */
APP_TIMER_DEF(m_ps_id);                                               /**< Battery measurement timer. */


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
	    // Create battery timer
    err_code = app_timer_create(&m_battery_id, APP_TIMER_MODE_SINGLE_SHOT, battery_base_time_handler);
    APP_ERROR_CHECK(err_code);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

    err_code = app_timer_create(&m_poff_id, APP_TIMER_MODE_SINGLE_SHOT, timer_poff_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_button_state_id, APP_TIMER_MODE_SINGLE_SHOT, timer_button_state_handler);
    APP_ERROR_CHECK(err_code);


    err_code = app_timer_create(&m_mpu6050_id, APP_TIMER_MODE_SINGLE_SHOT, timer_mpu6050_handler);
    APP_ERROR_CHECK(err_code);


   // err_code = app_timer_create(&timer_id.active_scan_enable, APP_TIMER_MODE_SINGLE_SHOT, timer_active_scan_handler);
   // APP_ERROR_CHECK(err_code);


    err_code = app_timer_create(&m_send_nus_id, APP_TIMER_MODE_SINGLE_SHOT, timer_send_nus_handler);
    APP_ERROR_CHECK(err_code);


    err_code = app_timer_create(&m_bmp180_id, APP_TIMER_MODE_SINGLE_SHOT, timer_bmp180_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_ps_id, APP_TIMER_MODE_SINGLE_SHOT, timer_ps_handler);
    APP_ERROR_CHECK(err_code);



}



void timer_ps_start(uint32_t time_ms)
{
		uint32_t err_code;
		// Start battery timer
   err_code = app_timer_start(m_ps_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
   APP_ERROR_CHECK(err_code);
}




void timer_ps_stop(void)
{
	uint32_t err_code;
	//stop authentication timer
    err_code = app_timer_stop(m_ps_id);
    APP_ERROR_CHECK(err_code);
}


/////////////////////////////////////////////////////////////////////////////////////

void timer_bmp180_start(uint32_t time_ms)
{
		uint32_t err_code;
		// Start battery timer
   err_code = app_timer_start(m_bmp180_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
   APP_ERROR_CHECK(err_code);
}




void timer_bmp180_stop(void)
{
	uint32_t err_code;
	//stop authentication timer
    err_code = app_timer_stop(m_bmp180_id);
    APP_ERROR_CHECK(err_code);
}

void battery_timer_start(uint32_t time_ms)
{
		uint32_t err_code;
		// Start battery timer
   err_code = app_timer_start(m_battery_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
   APP_ERROR_CHECK(err_code);
}




void battery_timer_stop(void)
{
	uint32_t err_code;
	//stop authentication timer
    err_code = app_timer_stop(m_battery_id);
    APP_ERROR_CHECK(err_code);
}




void timer_button_state_stop(void)
{
	uint32_t err_code;

	err_code =	app_timer_stop(m_button_state_id);
	APP_ERROR_CHECK(err_code);
}



void timer_button_state_start(uint32_t time_ms)
{
	uint32_t err_code;
    // Start application timers.
	err_code = app_timer_start(m_button_state_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
		APP_ERROR_CHECK(err_code);
}


void timer_poff_start(uint32_t time_ms)
{
	uint32_t err_code;

	//start timer for verifying authentication
		err_code = app_timer_start(m_poff_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
		APP_ERROR_CHECK(err_code);
}

void timer_poff_stop(void)
{
	uint32_t err_code;
	 err_code =		app_timer_stop(m_poff_id);
     APP_ERROR_CHECK(err_code);
}


void timer_mpu6050_start(uint32_t time_ms)
{
	uint32_t err_code;

	//start timer for verifying authentication
		err_code = app_timer_start(m_mpu6050_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
		APP_ERROR_CHECK(err_code);
}

void timer_mpu6050_stop(void)
{
	uint32_t err_code;
	 err_code =		app_timer_stop(m_mpu6050_id);
     APP_ERROR_CHECK(err_code);
}


void timer_send_nus_start(uint32_t time_ms)
{
	uint32_t err_code;

	//start timer for verifying authentication
		err_code = app_timer_start(m_send_nus_id, APP_TIMER_TICKS(time_ms, APP_TIMER_PRESCALER), NULL);
		APP_ERROR_CHECK(err_code);
}

void timer_send_nus_stop(void)
{
	uint32_t err_code;
	 err_code =		app_timer_stop(m_send_nus_id);
     APP_ERROR_CHECK(err_code);
}



void set_tout_ms(uint32_t *count,uint32_t tout_ms,uint32_t base_time_ms)
{
	(*count)=tout_ms/base_time_ms;
}

uint32_t tout_run(uint32_t *count)
{
	if((*count)>0)
	(*count)--;
	return (*count);
}
