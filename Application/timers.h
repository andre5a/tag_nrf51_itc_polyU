#ifndef TIMERS_H__
#define TIMERS_H__
#include "app_timer.h"


#define APP_TIMER_PRESCALER             	0//32->(1ms) //3276->(100ms) 327->(10ms) 163->(5ms)   4095   /**< Value of the RTC1 PRESCALER register. */
																								//1 for Battery measurement,
																								//1 for authentication Timeout
																								//1 for safety mode
																								//1 for sound_led,
																								//1 for button_state
																								//1 for button_char_update
																								//1 for system off
																								//1 for tx_power safety mode
																								//1 for advertising2 start

																								//1 for connection parameters module,
																								//1 for BLE STACK
																								//1 gpiote

#define APP_TIMER_OP_QUEUE_SIZE          	6                                                /**< Size of timer operation queues. */
#define POFF_DEFAULT_TIMOUT					60000	//ms


#define BATTERY_LEVEL_MEAS_INTERVAL       APP_TIMER_TICKS(120000, APP_TIMER_PRESCALER)      	/**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */
//#define SAFETY_MODE_WAITING_INTERVAL 	 APP_TIMER_TICKS(14000, APP_TIMER_PRESCALER)	 		//7.1secs

/*< Number of attempts before giving up the connection parameter negotiation. */
//


void timers_init(void);

void timer_ps_start(uint32_t time_ms);
void timer_ps_stop(void);
void timer_ps_handler(void * p_context);



void timer_bmp180_start(uint32_t time_ms);
void timer_bmp180_stop(void);
void timer_bmp180_handler(void * p_context);


void battery_timer_start(uint32_t timer);
void battery_timer_stop(void);
void battery_base_time_handler(void * p_context);


//void timer_button_adv_handler(void * p_context);

void timer_button_state_start(uint32_t time_ms);
void timer_button_state_stop(void);
void timer_button_state_handler(void * p_context);

void timer_poff_start(uint32_t time_ms);
void timer_poff_stop(void);
void timer_poff_handler(void * p_context);

void timer_button_adv_start(uint32_t time_ms);
void timer_button_adv_stop(void);





void timer_mpu6050_start(uint32_t time_ms);
void timer_mpu6050_stop(void);
void timer_mpu6050_handler(void * p_context);


void timer_send_nus_start(uint32_t time_ms);
void timer_send_nus_stop(void);
void timer_send_nus_handler(void * p_context);



void set_tout_ms(uint32_t *count,uint32_t tout_ms,uint32_t base_time_ms);
uint32_t tout_run(uint32_t *count);

#endif

