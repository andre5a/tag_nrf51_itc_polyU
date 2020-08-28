/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include "twi_master.h"
#include "mpu6050.h"
#include "state_machine.h"
#include "nrf_delay.h"
#include <string.h>
#include "app_uart.h"
#include "ble_nus.h"
#include "ble_uart.h"
#include "quaternion.h"
#include "sensor_processing_lib.h"
#include "vector_3d.h"

/*lint ++flb "Enter library region" */



#define SM_INIT              0
#define SM_SETUP             1
#define SM_IMU_CAL           2
#define SM_IMU_LOW_PWR_ACCEL 3
#define SM_RUN_LOW_PWR       4
#define SM_SLEEP             5
#define SM_IDLE              6
#define SM_RUN               7
#define SM_WAIT_RESET        8


#define ADDRESS_SIGNAL_PATH_RESET (0x68U) // !<

static const uint8_t expected_who_am_i = 0x68U; // !< Expected value to get from WHO_AM_I register.
static uint8_t       m_device_address;          // !< Device address in bits [7:1]


static STATE_MACHINE 				mpu6050_st;


float rad_to_deg = 180/M_PI; 

static char buf_aux[100];

static float dpsPerDigit, rangePerDigit, accSensibility;
static float Acc_angle_error_x, Acc_angle_error_y, Acc_angle_x, Acc_angle_y;

static float Total_angle_x, Total_angle_y;
static	float actualThreshold;
static	bool useCalibrate;


static		Vector na, ng; // Normalized vectors
static		Vector tg, dg; // Threshold and Delta for Gyro
static		Vector th;     // Threshold
static		Activites a;   // Activities
static      float tempc;
static      float roll, pitch, yaw;

		typedef struct gyros
		{
		    int16_t XAxis;
		    int16_t YAxis;
		    int16_t ZAxis;
		}giro;

static giro ra,rg;

static bool mpu6050_en=false;
static uint32_t time_tick=0;
static uint32_t time_now, time_prev;

static float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
static float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
//static  int16_t cnt_tick;
 
static float delta,wx,wy,wz;
static euler_angles angles;
static vector_ijk fused_vector;
static Quaternion q_acc;



bool mpu6050_init(uint8_t device_address)
{
    bool transfer_succeeded = true;

    m_device_address = (uint8_t)(device_address << 1);

    // Do a reset on signal paths
    uint8_t reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths.
    transfer_succeeded &= mpu6050_register_write(ADDRESS_SIGNAL_PATH_RESET, reset_value);

    // Read and verify product ID
    transfer_succeeded &= mpu6050_verify_product_id();

    return transfer_succeeded;
}

bool mpu6050_verify_product_id(void)
{
    uint8_t who_am_i;

    if (mpu6050_register_read(MPUREG_WHOAMI, &who_am_i, 1))
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

bool mpu6050_register_write(uint8_t register_address, uint8_t value)
{
    uint8_t w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_master_transfer(m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}

bool mpu6050_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}
void mpu6050_setup(void)
{
    mpu6050_register_write(MPUREG_PWR_MGMT_1, 1<<BIT_POS_DEVICE_RESET| 0<<BIT_POS_SLEEP |0<<BIT_POS_CYCLE | 0<<BIT_POS_TEMP_DIS);// MPU Reset
    nrf_delay_ms(100);
    mpu6050_register_write(MPUREG_PWR_MGMT_1, 0<<BIT_POS_DEVICE_RESET|0<<BIT_POS_SLEEP |0<<BIT_POS_CYCLE | 0<<BIT_POS_TEMP_DIS | MPU6050_CLOCK_PLL_XGYRO<<BIT_POS_CLKSEL);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_PWR_MGMT_2, 0<<BIT_POS_LP_WAKE_CTRL| 0<<BIT_POS_STBY_AX | 0<<BIT_POS_STBY_AY | 0<<BIT_POS_STBY_AZ| 0<<BIT_POS_STBY_GX | 0<<BIT_POS_STBY_GY | 0<<BIT_POS_STBY_GZ);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_CONFIG, 3<BIT_POS_DLPF_CFG);     //BIT_POS_DLPF_CFG = 0 || 7 means disabeled->GIRO Sample output 8KHz  
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_SMPLRT_DIV, 99);//Use a 0x00-1KHz 0x04-200Hz sample rate
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_INT_ENABLE, 0<<BIT_POS_DATA_RDY_EN | 1<< BIT_POS_FIFO_OFLOW_EN);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_GYRO_CONFIG, MPU6050_SCALE_2000DPS<<BIT_POS_FS_SEL);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_ACCEL_CONFIG, MPU6050_RANGE_2G<<BIT_POS_AFS_SEL);
    nrf_delay_ms(10);
    mpu6050_setScale(MPU6050_SCALE_2000DPS);
    nrf_delay_ms(10);
    mpu6050_setRange(MPU6050_RANGE_2G);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_I2C_MST_CTRL,0<<BIT_POS_SLV3_FIFO_EN);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_FIFO_EN,1<<BIT_POS_TEMP_FIFO_EN | 1<<BIT_POS_XG_FIFO_EN | 1<<BIT_POS_YG_FIFO_EN | 1<<BIT_POS_ZG_FIFO_EN | 1<<BIT_POS_ACCEL_FIFO_EN);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_USER_CTRL,  0<<BIT_POS_FIFO_EN |0<<BIT_POS_FIFO_RESET |0<<BIT_POS_SIG_COND_RESET);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_SIGNAL_PATH_RESET, 0<<BIT_POS_GYRO_RESET| 0<<BIT_POS_ACCEL_RESET |0<<BIT_POS_TEMP_RESET);
    nrf_delay_ms(10);
    mpu6050_register_write(MPUREG_PWR_MGMT_1, 0<<BIT_POS_DEVICE_RESET| 1<<BIT_POS_SLEEP |0<<BIT_POS_CYCLE | 0<<BIT_POS_TEMP_DIS); //Sleep
    nrf_delay_ms(10);
 
}


void mpu6050_loop(void)
{
	uint8_t status=0;
	uint8_t data[100];
	int16_t gX,gY,gZ;
	int16_t aX,aY,aZ;
	int16_t temp;
    clock_t  elapsedTime;
    uint16_t fifo_count;
    uint16_t packet_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};


     time_tick++;


	switch(mpu6050_st.cstate)
	{
		case SM_INIT:
		    mpu6050_st.nxstate=SM_SETUP;
			//mpu6050_setup();
          
            accAngleX=0;
            accAngleY=0;
            gyroAngleX=0; 
            gyroAngleY=0; 
            gyroAngleZ=0;
            AccErrorX=0; 
            AccErrorY=0; 
            GyroErrorX=0; 
            GyroErrorY=0; 
            GyroErrorZ=0;
            roll=0;
            pitch=0; 
            yaw=0;


            fused_vector = vector_3d_initialize(0.0,0.0,-1.0);
            q_acc = quaternion_initialize(1.0,0.0,0.0,0.0);
			break;

		case SM_SETUP:
			mpu6050_st.nxstate=SM_IMU_CAL;
            mpu6050_register_write(MPUREG_PWR_MGMT_1, 0<<BIT_POS_DEVICE_RESET| 0<<BIT_POS_SLEEP |0<<BIT_POS_CYCLE | 0<<BIT_POS_TEMP_DIS); //Wake-up
            mpu6050_register_write(MPUREG_USER_CTRL,  1<<BIT_POS_FIFO_EN |0<<BIT_POS_FIFO_RESET |0<<BIT_POS_SIG_COND_RESET);            
            //cnt_measures=0;
		//	break;
        case SM_IMU_CAL:
            mpu6050_st.nxstate=SM_RUN;
            mpu6050_register_read(MPUREG_FIFO_COUNTH,data,2);
            fifo_count=data[0];
            fifo_count=fifo_count<<8;
            fifo_count|=data[1];
            packet_count=fifo_count/14;  //14bytes per frame  Ax[2],Ay[2],Az[2],Gx[2],Gy[2],Gz[2],Temp[2]

            while(packet_count>0)
            { 
                packet_count--;
                mpu6050_register_read(MPUREG_FIFO_R_W, data, 14); 

                aX=data[0];
                aX=aX<<8;
                aX|=data[1];

                aY=data[2];
                aY=aY<<8;
                aY|=data[3];

                aZ=data[4];
                aZ=aZ<<8;
                aZ|=data[5];

                temp=data[6];
                temp=temp<<8;
                temp|=data[7];

                gX=data[8];
                gX=gX<<8;
                gX|=data[9];

                gY=data[10];
                gY=gY<<8;
                gY|=data[11];

                gZ=data[12];
                gZ=gZ<<8;
                gZ|=data[13];

                accel_bias[0]+=(int32_t) aX;
                accel_bias[1]+=(int32_t) aY;
                accel_bias[2]+=(int32_t) aZ;
                gyro_bias[0]+=(int32_t) gX;
                gyro_bias[1]+=(int32_t) gY;
                gyro_bias[2]+=(int32_t) gZ;

            } 
            packet_count=fifo_count/14;

            AccErrorX=accel_bias[0]/packet_count;
            AccErrorY=accel_bias[1]/packet_count;
            AccErrorZ=accel_bias[2]/packet_count;
            GyroErrorX=gyro_bias[0]/packet_count;
            GyroErrorY=gyro_bias[1]/packet_count;
            GyroErrorZ=gyro_bias[2]/packet_count;


            if(AccErrorZ > 0L) 
            {
                AccErrorZ= AccErrorZ - accSensibility;
            }  // Remove gravity from the z-axis accelerometer bias calculation
            else 
            {
                 AccErrorZ= AccErrorZ + accSensibility;
            }




            AccErrorX *= rangePerDigit;  // Sensitivity 
            AccErrorY *= rangePerDigit;  // Sensitivity
            AccErrorZ *= rangePerDigit;  // Sensitivity
   
    Acc_angle_error_x =  ((atan((AccErrorY)/sqrt(pow((AccErrorX),2) + pow((AccErrorZ),2)))*rad_to_deg));
    Acc_angle_error_y =  ((atan(-1*(AccErrorX)/sqrt(pow((AccErrorY),2) + pow((AccErrorZ),2)))*rad_to_deg)); 



            GyroErrorX *=  dpsPerDigit;
            GyroErrorY *=  dpsPerDigit;
            GyroErrorZ *=  dpsPerDigit;
            
        //    break;
        case SM_RUN:
            if(mpu6050_en==false)
			    mpu6050_st.nxstate=SM_SLEEP;
            if(mpu6050_getIntStatus() & 1<<BIT_POS_FIFO_OFLOW_INT)
           {
               nus_printStr("FIFO OVERFLOW!");
           }                
            mpu6050_register_read(MPUREG_FIFO_COUNTH,data,2);
            fifo_count=data[0];
            fifo_count=fifo_count<<8;
            fifo_count|=data[1];
            packet_count=fifo_count/14;  //14bytes per frame  Ax[2],Ay[2],Az[2],Gx[2],Gy[2],Gz[2],Temp[2]

            while(packet_count>0)
            { 
                packet_count--;
                mpu6050_register_read(MPUREG_FIFO_R_W, data, 14); 
                aX=data[0];
                aX=aX<<8;
                aX|=data[1];

                aY=data[2];
                aY=aY<<8;
                aY|=data[3];

                aZ=data[4];
                aZ=aZ<<8;
                aZ|=data[5];

                temp=data[6];
                temp=temp<<8;
                temp|=data[7];

                gX=data[8];
                gX=gX<<8;
                gX|=data[9];

                gY=data[10];
                gY=gY<<8;
                gY|=data[11];

                gZ=data[12];
                gZ=gZ<<8;
                gZ|=data[13];

                tempc=temp;
                tempc/=340.0;
                tempc+=36.53;

                na.XAxis = aX*rangePerDigit- AccErrorX;  
                na.YAxis = aY*rangePerDigit- AccErrorY;  
                na.ZAxis = aZ*rangePerDigit- AccErrorZ;  

                ng.XAxis = gX * dpsPerDigit -GyroErrorX;
                ng.YAxis = gY * dpsPerDigit -GyroErrorY;
                ng.ZAxis = gZ * dpsPerDigit -GyroErrorZ;


                gyroAngleX+= ng.XAxis/20;
                gyroAngleY+= ng.YAxis/20;  
                gyroAngleZ+= ng.ZAxis/20;
/*


    Acc_angle_x = (atan((na.YAxis)/sqrt(pow((na.XAxis),2) + pow((na.ZAxis),2)))*rad_to_deg) - Acc_angle_error_x;
    Acc_angle_y = (atan(-1*(na.XAxis)/sqrt(pow((na.YAxis),2) + pow((na.ZAxis),2)))*rad_to_deg) - Acc_angle_error_y;    


    //////////////////////////////////////Total angle and filter/////////////////////////////////////
    Total_angle_x = 0.98 *(Total_angle_x + gyroAngleX) + 0.02*Acc_angle_x;
    Total_angle_y = 0.98 *(Total_angle_y + gyroAngleY) + 0.02*Acc_angle_y;
*/


/*

                time_now = time_tick;
                delta = (float)((time_now - time_prev)/1000.0f);///1000000.0f) ; // set integration time by time elapsed since last filter update
                time_prev = time_now;
*/
                fused_vector = update_fused_vector(fused_vector,na.XAxis,na.YAxis,na.ZAxis,ng.XAxis,ng.YAxis,ng.ZAxis,1.0/20.0);
                
                q_acc = quaternion_from_accelerometer(fused_vector.a,fused_vector.b,fused_vector.c);
                angles = quaternion_to_euler_angles(q_acc);

            }
            
            break;
      
        case SM_SLEEP:
        	mpu6050_register_write(MPUREG_PWR_MGMT_1, 1<<BIT_POS_SLEEP |0<<BIT_POS_CYCLE | 1<<BIT_POS_TEMP_DIS);
			mpu6050_st.nxstate=SM_IDLE;

        case SM_IDLE:
            if(mpu6050_en)
        	    mpu6050_st.nxstate=SM_SETUP;

            break;
		default:
			mpu6050_st.cstate=SM_INIT;
			break;
	}


	mpu6050_st.cstate=mpu6050_st.nxstate;
}

  
  

void read_acc(float *x, float *y, float *z)
{
    *x=na.XAxis;
    *y=na.YAxis;
    *z=na.ZAxis;
}

void read_gyro(float *x, float *y, float *z)
{


/*
    *x=gyroAngleX;
    *y=gyroAngleY;
    *z=gyroAngleZ;
 
*/ 
/*               *x=gyro_angleX;
                *y=gyro_angleY;
                *z=gyro_angleZ;
*/


/*
*x=roll;
*y=pitch;
*z=yaw;
*/
/*
*x=q[0];
*y=q[1];
*z=q[2];
*/

*x=angles.roll;
*y=angles.pitch;
*z=angles.yaw;


}

void read_angle(float *x, float *y)
{
*x=Total_angle_x;
*y=Total_angle_y;
  
}


void read_temp(float *temp)
{
    *temp=tempc;
}

void mpu6050_enable(void)
{
    mpu6050_en=true;
}

void mpu6050_disable(void)
{
    mpu6050_en=false;
}


bool mpu6050_start(mpu6050_dps_t scale, mpu6050_range_t range)
{
    // Set Address
mpu6050_st.cstate=mpu6050_st.nxstate=SM_INIT;

    // Reset calibrate values
    dg.XAxis = 0;
    dg.YAxis = 0;
    dg.ZAxis = 0;
    useCalibrate = false;

    // Reset threshold values
    tg.XAxis = 0;
    tg.YAxis = 0;
    tg.ZAxis = 0;
    actualThreshold = 0;

    // Disable Sleep Mode
    mpu6050_setSleepEnabled(false);

    // Set Clock Source
    mpu6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    //mpu6050_setClockSource(MPU6050_CLOCK_INTERNAL_8MHZ);
    // Set Scale & Range
    mpu6050_setScale(scale);
    mpu6050_setRange(range);
   // mpu6050_setDLPFMode(7);

	mpu6050_register_write(MPUREG_SMPLRT_DIV, 19);

    return true;
}

void mpu6050_setScale(mpu6050_dps_t scale)
{
    uint8_t value;

    switch (scale)
    {
	case MPU6050_SCALE_250DPS:
	    dpsPerDigit = 250.0/32768.0;
	    break;
	case MPU6050_SCALE_500DPS:
	    dpsPerDigit = 500.0/32768.0;
	    break;
	case MPU6050_SCALE_1000DPS:
	    dpsPerDigit = 1000.0/32768.0;
	    break;
	case MPU6050_SCALE_2000DPS:
	    dpsPerDigit = 2000.0/32768.0;
	    break;
	default:
	    break;
    }

    value = mpu6050_readRegister8(MPUREG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (scale << 3);
    mpu6050_writeRegister8(MPUREG_GYRO_CONFIG, value);
}

mpu6050_dps_t mpu6050_getScale(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_GYRO_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_dps_t)value;
}

void mpu6050_setRange(mpu6050_range_t range)
{
    uint8_t value;

    switch (range)
    {
	case MPU6050_RANGE_2G:
	    rangePerDigit = 2.0/32768.0;
        accSensibility = 32768.0/2.0;
	    break;
	case MPU6050_RANGE_4G:
	    rangePerDigit = 4.0/32768.0;
        accSensibility = 32768.0/4.0;
	    break;
	case MPU6050_RANGE_8G:
	    rangePerDigit =  8.0/32768.0;
	        accSensibility = 32768.0/8.0;
        break;
	case MPU6050_RANGE_16G:
	    rangePerDigit =  16.0/32768.0;
        accSensibility = 32768.0/16.0;	    
        break;
	default:
	    break;
    }

    value = mpu6050_readRegister8(MPUREG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    mpu6050_writeRegister8(MPUREG_ACCEL_CONFIG, value);
}

mpu6050_range_t mpu6050_getRange(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_ACCEL_CONFIG);
    value &= 0b00011000;
    value >>= 3;
    return (mpu6050_range_t)value;
}

void mpu6050_setDHPFMode(mpu6050_dhpf_t dhpf)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_ACCEL_CONFIG);
    value &= 0b11111000;
    value |= dhpf;
    mpu6050_writeRegister8(MPUREG_ACCEL_CONFIG, value);
}

void mpu6050_setDLPFMode(mpu6050_dlpf_t dlpf)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_CONFIG);
    value &= 0b11111000;
    value |= dlpf;
    mpu6050_writeRegister8(MPUREG_CONFIG, value);
}

void mpu6050_setClockSource(mpu6050_clockSource_t source)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_PWR_MGMT_1);
    value &= 0b11111000;
    value |= source;
    mpu6050_writeRegister8(MPUREG_PWR_MGMT_1, value);
}

mpu6050_clockSource_t mpugetClockSource(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_PWR_MGMT_1);
    value &= 0b00000111;
    return (mpu6050_clockSource_t)value;
}

bool mpu6050_getSleepEnabled(void)
{
    return mpu6050_readRegisterBit(MPUREG_PWR_MGMT_1, 6);
}

void mpu6050_setSleepEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPUREG_PWR_MGMT_1, 6, state);
}

bool mpu6050_getIntZeroMotionEnabled(void)
{
    return mpu6050_readRegisterBit(MPUREG_INT_ENABLE, 5);
}

void mpu6050_setIntZeroMotionEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPUREG_INT_ENABLE, 5, state);
}

bool mpu6050_getIntMotionEnabled(void)
{
    return mpu6050_readRegisterBit(MPUREG_INT_ENABLE, 6);
}

void mpu6050_setIntMotionEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPUREG_INT_ENABLE, 6, state);
}

bool mpu6050_getIntFreeFallEnabled(void)
{
    return mpu6050_readRegisterBit(MPUREG_INT_ENABLE, 7);
}

void mpu6050_setIntFreeFallEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPUREG_INT_ENABLE, 7, state);
}

uint8_t mpu6050_getMotionDetectionThreshold(void)
{
    return mpu6050_readRegister8(MPUREG_PWR_MGMT_1);
}

void mpu6050_setMotionDetectionThreshold(uint8_t threshold)
{
    mpu6050_writeRegister8(MPUREG_PWR_MGMT_1, threshold);
}

uint8_t mpu6050_getMotionDetectionDuration(void)
{
    return mpu6050_readRegister8(MPUREG_MOT_DURATION);
}

void mpu6050_setMotionDetectionDuration(uint8_t duration)
{
    mpu6050_writeRegister8(MPUREG_MOT_DURATION, duration);
}

uint8_t mpu6050_getZeroMotionDetectionThreshold(void)
{
    return mpu6050_readRegister8(MPUREG_ZMOT_THRESHOLD);
}

void mpu6050_setZeroMotionDetectionThreshold(uint8_t threshold)
{
    mpu6050_writeRegister8(MPUREG_ZMOT_THRESHOLD, threshold);
}

uint8_t mpu6050_getZeroMotionDetectionDuration(void)
{
    return mpu6050_readRegister8(MPUREG_ZMOT_DURATION);
}

void mpu6050_setZeroMotionDetectionDuration(uint8_t duration)
{
    mpu6050_writeRegister8(MPUREG_ZMOT_DURATION, duration);
}

uint8_t mpu6050_getFreeFallDetectionThreshold(void)
{
    return mpu6050_readRegister8(MPUREG_FF_THRESHOLD);
}

void mpu6050_setFreeFallDetectionThreshold(uint8_t threshold)
{
    mpu6050_writeRegister8(MPUREG_FF_THRESHOLD, threshold);
}

uint8_t mpu6050_getFreeFallDetectionDuration(void)
{
    return mpu6050_readRegister8(MPUREG_FF_DURATION);
}

void mpu6050_setFreeFallDetectionDuration(uint8_t duration)
{
    mpu6050_writeRegister8(MPUREG_FF_DURATION, duration);
}

bool mpu6050_getI2CMasterModeEnabled(void)
{
    return mpu6050_readRegisterBit(MPUREG_USER_CTRL, 5);
}

void mpu6050_setI2CMasterModeEnabled(bool state)
{
    mpu6050_writeRegisterBit(MPUREG_USER_CTRL, 5, state);
}

void mpu6050_setI2CBypassEnabled(bool state)
{
    return mpu6050_writeRegisterBit(MPUREG_INT_PIN_CFG, 1, state);
}

bool mpu6050_getI2CBypassEnabled(void)
{
    return mpu6050_readRegisterBit(MPUREG_INT_PIN_CFG, 1);
}

void mpu6050_setAccelPowerOnDelay(mpu6050_onDelay_t delay)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    mpu6050_writeRegister8(MPUREG_MOT_DETECT_CTRL, value);
}

mpu6050_onDelay_t mpu6050_getAccelPowerOnDelay(void)
{
    uint8_t value;
    value = mpu6050_readRegister8(MPUREG_MOT_DETECT_CTRL);
    value &= 0b00110000;
    return (mpu6050_onDelay_t)(value >> 4);
}

uint8_t mpu6050_getIntStatus(void)
{
    return mpu6050_readRegister8(MPUREG_INT_STATUS);
}

Activites mpu6050_readActivites(void)
{
    uint8_t data = mpu6050_readRegister8(MPUREG_INT_STATUS);

    a.isOverflow = ((data >> 4) & 1);
    a.isFreeFall = ((data >> 7) & 1);
    a.isInactivity = ((data >> 5) & 1);
    a.isActivity = ((data >> 6) & 1);
    a.isDataReady = ((data >> 0) & 1);

    data = mpu6050_readRegister8(MPUREG_MOT_DETECT_STATUS);

    a.isNegActivityOnX = ((data >> 7) & 1);
    a.isPosActivityOnX = ((data >> 6) & 1);

    a.isNegActivityOnY = ((data >> 5) & 1);
    a.isPosActivityOnY = ((data >> 4) & 1);

    a.isNegActivityOnZ = ((data >> 3) & 1);
    a.isPosActivityOnZ = ((data >> 2) & 1);

    return a;
}

void mpu6050_readRawAccel(void)
{
uint8_t data[6];
	mpu6050_register_read(MPUREG_ACCEL_XOUT_H, data, 6);
	uint8_t xha = data[0];
	uint8_t xla = data[1];
	uint8_t yha = data[2];
	uint8_t yla = data[3];
	uint8_t zha = data[4];
	uint8_t zla = data[5];

    ra.XAxis = xha;
    ra.XAxis= ra.XAxis<<8;
    ra.XAxis|= xla;

    ra.YAxis = yha;
    ra.YAxis= ra.YAxis<<8;
    ra.YAxis|= yla;

    ra.ZAxis = zha;
    ra.ZAxis= ra.ZAxis<<8;
    ra.ZAxis|= zla;

}

Vector mpu6050_readNormalizeAccel(void)
{
    mpu6050_readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;

    return na;
}

Vector mpu6050_readScaledAccel(void)
{
    mpu6050_readRawAccel();

    na.XAxis = ra.XAxis * rangePerDigit;
    na.YAxis = ra.YAxis * rangePerDigit;
    na.ZAxis = ra.ZAxis * rangePerDigit;

    return na;
}


void mpu6050_readRawGyro(void)
{

	uint8_t data[6];
	memset(data,0,sizeof(data));
		mpu6050_register_read(MPUREG_GYRO_XOUT_H, data, 6);
		uint8_t xha = data[0];
		uint8_t xla = data[1];
		uint8_t yha = data[2];
		uint8_t yla = data[3];
		uint8_t zha = data[4];
		uint8_t zla = data[5];



    rg.XAxis = xha;
    rg.XAxis= rg.XAxis<<8;
    rg.XAxis|= xla;

    rg.YAxis = yha;
    rg.YAxis= rg.YAxis<<8;
    rg.YAxis|= yla;

    rg.ZAxis = zha;
    rg.ZAxis= rg.ZAxis<<8;
    rg.ZAxis|= zla;



}

Vector mpu6050_readNormalizeGyro(void)
{
    mpu6050_readRawGyro();

    if (useCalibrate)
    {
		ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
		ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
		ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else
    {
		ng.XAxis = rg.XAxis * dpsPerDigit;
		ng.YAxis = rg.YAxis * dpsPerDigit;
		ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }

    if (actualThreshold)
    {
		if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
		if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
		if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }

    return ng;
}

float mpu6050_readTemperature(void)
{
    int16_t T;
    T = mpu6050_readRegister16(MPUREG_TEMP_OUT_H);
    return (float)T/340 + 36.53;
}

int16_t mpu6050_getGyroOffsetX(void)
{
    return mpu6050_readRegister16(MPUREG_GYRO_XOFFS_H);
}

int16_t mpu6050_getGyroOffsetY(void)
{
    return mpu6050_readRegister16(MPUREG_GYRO_YOFFS_H);
}

int16_t mpu6050_getGyroOffsetZ(void)
{
    return mpu6050_readRegister16(MPUREG_GYRO_ZOFFS_H);
}

void mpu6050_setGyroOffsetX(int16_t offset)
{
    mpu6050_writeRegister16(MPUREG_GYRO_XOFFS_H, offset);
}

void mpu6050_setGyroOffsetY(int16_t offset)
{
    mpu6050_writeRegister16(MPUREG_GYRO_YOFFS_H, offset);
}

void mpu6050_setGyroOffsetZ(int16_t offset)
{
    mpu6050_writeRegister16(MPUREG_GYRO_ZOFFS_H, offset);
}

int16_t mpu6050_getAccelOffsetX(void)
{
    return mpu6050_readRegister16(MPUREG_ACCEL_XOFFS_H);
}

int16_t mpu6050_getAccelOffsetY(void)
{
    return mpu6050_readRegister16(MPUREG_ACCEL_YOFFS_H);
}

int16_t mpu6050_getAccelOffsetZ(void)
{
    return mpu6050_readRegister16(MPUREG_ACCEL_ZOFFS_H);
}

void mpu6050_setAccelOffsetX(int16_t offset)
{
    mpu6050_writeRegister16(MPUREG_ACCEL_XOFFS_H, offset);
}

void mpu6050_setAccelOffsetY(int16_t offset)
{
    mpu6050_writeRegister16(MPUREG_ACCEL_YOFFS_H, offset);
}

void mpu6050_setAccelOffsetZ(int16_t offset)
{
    mpu6050_writeRegister16(MPUREG_ACCEL_ZOFFS_H, offset);
}

// Calibrate algorithm
void mpu6050_calibrateGyro(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
		mpu6050_readRawGyro();
		sumX += rg.XAxis;
		sumY += rg.YAxis;
		sumZ += rg.ZAxis;

		sigmaX += rg.XAxis * rg.XAxis;
		sigmaY += rg.YAxis * rg.YAxis;
		sigmaZ += rg.ZAxis * rg.ZAxis;

		nrf_delay_ms(5);
    }

    // Calculate delta vectors
    dg.XAxis = sumX / samples;
    dg.YAxis = sumY / samples;
    dg.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
    	mpu6050_setThreshold(actualThreshold);
    }
}

// Get current threshold value
uint8_t mpu6050_getThreshold(void)
{
    return actualThreshold;
}

// Set treshold value
void mpu6050_setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
		// If not calibrated, need calibrate
		if (!useCalibrate)
		{
			mpu6050_calibrateGyro(50);
		}

		// Calculate threshold vectors
		tg.XAxis = th.XAxis * multiple;
		tg.YAxis = th.YAxis * multiple;
		tg.ZAxis = th.ZAxis * multiple;
    } else
    {
		// No threshold
		tg.XAxis = 0;
		tg.YAxis = 0;
		tg.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

// Fast read 8-bit from register
uint8_t mpu6050_fastRegister8(uint8_t reg)
{
	uint8_t value;
    mpu6050_register_read(reg, &value, 1);
    return value;
}

// Read 8-bit from register
uint8_t mpu6050_readRegister8(uint8_t reg)
{


	uint8_t value;
    mpu6050_register_read(reg, &value, 1);
    return value;
}

// Write 8-bit to register
void mpu6050_writeRegister8(uint8_t reg, uint8_t value)
{
	mpu6050_register_write(reg,value);


}

int16_t mpu6050_readRegister16(uint8_t reg)
{

	uint8_t data[2];
	uint16_t value;
	mpu6050_register_read(reg, data,2);
    value = data[0] << 8 | data[1];
    return value;
}

void mpu6050_writeRegister16(uint8_t reg, int16_t value)
{

	mpu6050_register_write(reg, value>>8);
	mpu6050_register_write(reg, value & 0xff);


}

// Read register bit
bool mpu6050_readRegisterBit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = mpu6050_readRegister8(reg);
    return ((value >> pos) & 1);
}

// Write register bit
void mpu6050_writeRegisterBit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = mpu6050_readRegister8(reg);

    if (state)
    {
        value |= (1 << pos);
    } else
    {
        value &= ~(1 << pos);
    }

    mpu6050_writeRegister8(reg, value);
}

/*lint --flb "Leave library region" */
