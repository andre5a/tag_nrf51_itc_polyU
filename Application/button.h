#ifndef BUTTON_LED_H__
#define BUTTON_LED_H__

//#define _BUTTON_NORMAL_PRESS_
//#define _BUTTON_DOUBLE_PRESS_
//#define _BUTTON_LONG_PRESS_



#define APP_GPIOTE_MAX_USERS              1                                                 /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY            APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)          /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */



typedef enum
{
    BUTTON_OFF_SIG=0,
    BUTTON_SHORT_SIG=1,
    BUTTON_LONG_SIG=2,
    BUTTON_DOUBLE_SIG=3,
    SAFETY_FAR_SIG=4

} button_signaling_t;





void timer_button_state(void);
void timer_button_state_handler(void * p_context);
void button_events_start(void);
bool button_busy(void);

void button1_short_press_evt(void);
void button1_long_press_evt(void);
void button1_double_press_evt(void);
void button_adv_set(void);


void button_init(void);


bool button_test_is_subscribe(void);
void button_test_subscribe_set(bool flag);

#endif


