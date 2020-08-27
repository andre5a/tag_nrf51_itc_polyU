#include <stdio.h>
#include <stdbool.h>
#include "door.h"

#define MAX_CURRENT 1000
#define CLOSE_DIR true
#define OPEN_DIR false

#define TIME_3S     3
#define TIME_5S     5
#define MOTOR_TIMEOUT_S     30

#define MOTOR_DIR_OPEN      0
#define MOTOR_WAIT_OPEN     1
#define M1_DIR_CLOSE        2
#define M2_DIR_CLOSE        3
#define M2_DIR_OPEN1        6
#define M1_DIR_OPEN1        7
#define M1_DIR_CLOSE1       8
#define M2_DIR_CLOSE1       9
#define TIME_WRITE_EEP      10
#define CAL_END             11

enum door_states{                       \
                    GATE_INIT=0,        \
                    GATE_IDLE,          \
                    GATE_CAL,           \
                    GATE_OC,            \
                    GATE_CHECK_BUONDARY,\
                    GATE_STOP,          \
                    GATE_OBST,          \
                    GATE_REV,           \
                    GATE_TIMEOUT,   \                    
                    GATE_STORE_KEY_EEP, \
                 };

enum door_states nstate;
enum door_states cstate;

static time_t time_remaining_m1;
static time_t time_remaining_m2;

static time_t m1_time_run;
static time_t m2_time_run;

static bool door_dir=CLOSE_DIR;

enum door_status{                  \
                    DOOR_OPENED=0,   \
                    DOOR_CLOSED,    \
                };
enum door_status door_sta;


static uint16_t current_ma=0;



static uint8_t cal_cstate=0;
static uint8_t cal_nstate=0;
static time_t m1_time_open_meas_sec=0;
static time_t m1_time_close_meas_sec=0;
static time_t m2_time_open_meas_sec=0;
static time_t m2_time_close_meas_sec=0;


enum motor_states{                  \
                    MOTOR_STOP=0,   \
                    MOTOR_OPEN,     \
                    MOTOR_CLOSE,    \
                };

enum motor_states m1_dir;
enum motor_states m2_dir;
static time_t m1_time_pos=0;
static time_t m2_time_pos=0;
static time_t motor_timeout=0;

/////
enum m_states{M_INIT,M_IDLE,M_WAIT,M_RUN,M_FIN,M_PINCH,M_FAIL};

enum m_states m1_cstate;
enum m_states m1_nstate;
enum m_states m2_cstate;
enum m_states m2_nstate;


static bool m1_trigger=false;
static bool m2_trigger=false;



static bool lamp_on=false;

time_t read_eep_time(void)
{
    time_t val;
      /* code */
    return val;
}


uint8_t rf_sig(void)
{

    return 0;
}


bool learning_button_pressed(void)
{

    return false;
}

bool OC_button_pressed(void)
{

    return false;
}


bool key_button_pressed(void)
{

    return false;
}

uint16_t read_current_m1(void)
{
    uint16_t current=0;
    return current;
}

uint16_t read_current_m2(void)
{
    uint16_t current=0;
    return current;
}


bool ir_obst_detect(void)
{

    return false;
}




uint8_t door_learning(void)
{

            bool cal_finished=false;


            switch(cal_cstate)
            {
                case MOTOR_DIR_OPEN:
                    m1_time_open_meas_sec=0;
                    m1_time_close_meas_sec=0;
                    m2_time_open_meas_sec=0;
                    m2_time_close_meas_sec=0;

                    m1_dir=MOTOR_OPEN;
                    m2_dir=MOTOR_OPEN;
                    cal_nstate=MOTOR_WAIT_OPEN;
                    break;
                case MOTOR_WAIT_OPEN:                  
                    if(read_current_m1()>MAX_CURRENT)
                    {
                        m1_dir=MOTOR_STOP;
                    } 
                    if(read_current_m2()>MAX_CURRENT)
                    {
                        m2_dir=MOTOR_STOP;
                    } 
                    if(m1_dir==MOTOR_STOP && m2_dir==MOTOR_STOP)
                        cal_nstate=M1_DIR_CLOSE;
                    break;
                case M1_DIR_CLOSE:
                    m1_dir=MOTOR_CLOSE;
                    if(read_current_m1()>MAX_CURRENT)
                    {
                        m1_dir=MOTOR_STOP;
                        cal_nstate=M2_DIR_CLOSE;

                    } 
                    break;


                case M2_DIR_CLOSE:
                    m2_dir=MOTOR_CLOSE;
                    if(read_current_m2()>MAX_CURRENT)
                    {
                        m2_dir=MOTOR_STOP;
                        cal_nstate=M2_DIR_OPEN1;

                    } 

                    break;

                case M2_DIR_OPEN1:
                    m2_dir=MOTOR_OPEN;
                    m2_time_open_meas_sec++;
                    if(read_current_m2()>MAX_CURRENT)
                    { 
                         m2_dir=MOTOR_STOP;
                        cal_nstate=M1_DIR_OPEN1; 
                    }                
                    break;

                case M1_DIR_OPEN1:
                    m1_dir=MOTOR_OPEN;
                    m1_time_open_meas_sec++;
                    if(read_current_m1()>MAX_CURRENT)
                    { 
                         m1_dir=MOTOR_STOP;
                        cal_nstate=M1_DIR_CLOSE1; 
                    }                
                    break;

                case M1_DIR_CLOSE1:
                    m1_dir=MOTOR_CLOSE;
                    m1_time_close_meas_sec++;
                    if(read_current_m1()>MAX_CURRENT)
                    { 
                        m1_dir=MOTOR_STOP;
                        cal_nstate=M2_DIR_CLOSE1; 
                    }                
                    break;

                case M2_DIR_CLOSE1:
                    m2_dir=MOTOR_CLOSE;
                    m2_time_close_meas_sec++;
                    if(read_current_m2()>MAX_CURRENT)
                    { 
                        m2_dir=MOTOR_STOP;
                        cal_nstate=TIME_WRITE_EEP; 
                    }                
                    break;
                case TIME_WRITE_EEP:
                   // cal_nstate=MOTOR_DIR_OPEN; 
                    //if(m1_time_close_meas_sec==m1_time_open_meas_sec)
                    //    if(m2_time_close_meas_sec==m2_time_open_meas_sec)
                       {
                           //WRITE TIME IN EEPROM
                           cal_nstate=CAL_END; 
                       } 
                       
                    break;
                case CAL_END:
                    cal_finished=true;
                    break;


            } 
            cal_cstate=cal_nstate;

    return cal_finished;
}


void door_machine_st(void)
{
    switch (cstate)
    {
        case GATE_INIT:
            /* constant-expression */
            /* code */
            m1_dir=MOTOR_STOP;
            m2_dir=MOTOR_STOP;
            lamp_on = false;
            door_dir=CLOSE_DIR;
            door_sta=DOOR_OPENED;           
            nstate=GATE_IDLE;
            break;
        case GATE_IDLE:
            /* constant-expression */
            /* code */
            nstate=GATE_IDLE;

            lamp_on = false;
 
            if(rf_sig() || OC_button_pressed())
            {
                nstate=GATE_OC;
            } 
            if(learning_button_pressed())
            {
                nstate=GATE_CAL;
            } 
            if(ir_obst_detect())
            {
                nstate=GATE_IDLE;
            } 

            if(key_button_pressed())
            {
                nstate=GATE_STORE_KEY_EEP;
            } 

            break;
        case GATE_CAL:
            /* constant-expression */
            /* code */
    
            if(door_learning())
                nstate=GATE_IDLE;
            if(rf_sig() || OC_button_pressed())
            {
                m1_dir=MOTOR_STOP;
                m2_dir=MOTOR_STOP;
                nstate=GATE_STOP;
            }                 
            break;

        case GATE_OC:
            /* constant-expression */
            /* code */
            motor_timeout=0;
            if(door_dir==CLOSE_DIR)
                m1_dir=MOTOR_CLOSE;
            
            if(door_dir==OPEN_DIR)
                m2_dir=MOTOR_OPEN;
            nstate=GATE_CHECK_BUONDARY;


        case GATE_CHECK_BUONDARY:
            lamp_on=~lamp_on;


            if(door_dir==CLOSE_DIR)
            { 
                if(m2_time_pos>m1_time_pos)  //Motor2 só ativa depois do Motor1
                {
                        m2_dir=MOTOR_CLOSE;        
                }                         
            }
            else
            {                
                if(m1_time_pos<m2_time_pos)  //Motor1 só ativa depois do Motor2
                { 
                        m1_dir=MOTOR_OPEN;
                } 
            }

            if(motor_timeout>=MOTOR_TIMEOUT_S)
                nstate=GATE_TIMEOUT;

          //  if((m1_time_pos==0)  || (m1_time_pos>= read_eep_time()))
          //      nstate=GATE_TIMEOUT;

          //  if((m2_time_pos==0) || (m2_time_pos>= read_eep_time()))
          //      nstate=GATE_TIMEOUT;


            if(read_current_m1()>MAX_CURRENT)
            {
                m1_dir=MOTOR_STOP;
                if(door_dir==CLOSE_DIR)
                {    
                    if(m1_time_pos>TIME_3S)
                        nstate=GATE_OBST;
                }
                else
                {
                    if(m1_time_pos<(read_eep_time()-TIME_3S))
                        nstate=GATE_OBST;
                 }        
            }
            


            if(read_current_m2()>MAX_CURRENT)
            {
                m2_dir=MOTOR_STOP;
                if(door_dir==CLOSE_DIR)
                {    
                    if(m2_time_pos>TIME_3S)
                        nstate=GATE_OBST;
                }
                else
                {
                    if(m2_time_pos<(read_eep_time()-TIME_3S))
                        nstate=GATE_OBST;
                }
            }

            if((m1_dir==MOTOR_STOP) && (m2_dir==MOTOR_STOP))
                nstate=GATE_STOP;


        if(rf_sig() || OC_button_pressed())
            {
                m1_dir=MOTOR_STOP;
                m2_dir=MOTOR_STOP;
                nstate=GATE_STOP;
            } 

            break;
        case GATE_OBST:
            /* constant-expression */
            /* code */
            lamp_on=~lamp_on;
            m1_dir=MOTOR_STOP;
            m2_dir=MOTOR_STOP;  
            if(door_dir==CLOSE_DIR)
                nstate=GATE_REV;
            if(rf_sig() || OC_button_pressed())
            {
                nstate=GATE_STOP;
            }             
            break;
        case GATE_REV:
            lamp_on=~lamp_on;
            m1_dir=MOTOR_STOP;
            m2_dir=MOTOR_STOP;        
            door_dir=OPEN_DIR;
            nstate=GATE_OC;
            break;
        case GATE_STOP:
            /* constant-expression */
            /* code */
            door_dir=~door_dir;
            lamp_on=false;            
            m1_dir=MOTOR_STOP;
            m2_dir=MOTOR_STOP;           
            nstate=GATE_IDLE;
            break;            
        case GATE_TIMEOUT:
            lamp_on=~lamp_on;
            m1_dir=MOTOR_STOP;
            m2_dir=MOTOR_STOP;  
            if(rf_sig() || OC_button_pressed())
            {
                nstate=GATE_STOP;
            }            
            break;
                    
        case GATE_STORE_KEY_EEP:
            nstate=GATE_IDLE;
            break;

        default:
            nstate=GATE_INIT;
            break;
    }
        cstate=nstate;

    motor_timeout++; 
    switch (m1_dir)
    {
        case MOTOR_OPEN:
            if(m1_time_pos<0xFFFF)
                m1_time_pos++;
            break;
        case MOTOR_CLOSE:
            if(m1_time_pos>0)
                m1_time_pos--;
            break;
        case MOTOR_STOP:
            break;

        default:
            break;
    }

    switch (m2_dir)
    {
        case MOTOR_OPEN:
            if(m2_time_pos<0xFFFF)
                m2_time_pos++;
            break;
        case MOTOR_CLOSE:
            if(m2_time_pos>0)
                m2_time_pos--;
            break;
        case MOTOR_STOP:
            break;

        default:
            break;
    }


}
