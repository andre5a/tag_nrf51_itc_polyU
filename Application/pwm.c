#include <stdio.h>
#include <stdint.h>
#include "nrf51.h"
#include "nrf_pwm.h"
#include "boards.h"

#include "pwm.h"


	#define PWM1_PIN		21
	#define PWM2_PIN		22
	#define PWM3_PIN		23
	#define PWM4_PIN		24

void pwm_init(void)
{
    nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;
    
    pwm_config.mode             = PWM_MODE_MTR_255;
    pwm_config.num_channels     = 4;
    pwm_config.gpio_num[0]      = BSP_LED_0;
    pwm_config.gpio_num[1]      = BSP_LED_1;
    pwm_config.gpio_num[2]      = BSP_LED_2;
    pwm_config.gpio_num[3]      = PWM4_PIN;
    
    // Initialize the PWM library
    nrf_pwm_init(&pwm_config);    
}

void set_frequency_and_duty_cycle(uint32_t frequency, uint32_t duty_cycle_percent)
{
    nrf_pwm_set_max_value((16000000 + (frequency / 2)) / frequency);
    nrf_pwm_set_value(0, (16000000 / frequency) * duty_cycle_percent / 100);
}
