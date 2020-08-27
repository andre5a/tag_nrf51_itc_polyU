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

#ifndef MPU6050_H
#define MPU6050_H

/*lint ++flb "Enter library region" */

#include <stdbool.h>
#include <stdint.h>



// MPU 6050 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2	0x1D
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_PRODUCT_ID		0x0C
#define MPUREG_TRIM1			0x0D
#define MPUREG_TRIM2			0x0E
#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10
#define MPUREG_MOT_DURATION 	0x20
#define MPUREG_ZMOT_THRESHOLD	0x21
#define MPUREG_ZMOT_DURATION	0x22
#define MPUREG_FF_THRESHOLD		0x1D
#define MPUREG_FF_DURATION		0x1E
#define MPUREG_MOT_DETECT_CTRL	0x69
#define MPUREG_MOT_DETECT_STATUS	0x61
#define MPUREG_GYRO_XOFFS_H		0x13
#define MPUREG_GYRO_YOFFS_H		0x15
#define MPUREG_GYRO_ZOFFS_H		0x17
#define MPUREG_ACCEL_XOFFS_H	0x06
#define MPUREG_ACCEL_YOFFS_H	0x08
#define MPUREG_ACCEL_ZOFFS_H	0x0A
#define MPUREG_SIGNAL_PATH_RESET		0x68

/*
// Configuration bits MPU 6050
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_ACCEL_CONFIG_2G 	0x00
#define BITS_ACCEL_CONFIG_4G 	0x08
#define BITS_ACCEL_CONFIG_8G 	0x10
#define BITS_ACCEL_CONFIG_16G 	0x18
#define BIT_INT_ANYRD_2CLEAR	0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01
#define BITS_USER_CTRL_FIFO_RST 0x04
#define BITS_USER_CTRL_I2C_MST_EN 0x20
#define BITS_USER_CTRL_FIFO_EN 0x40
#define BITS_FIFO_ENABLE_TEMP_OUT 0x80
#define BITS_FIFO_ENABLE_GYRO_XOUT 0x40
#define BITS_FIFO_ENABLE_GYRO_YOUT 0x20
#define BITS_FIFO_ENABLE_GYRO_ZOUT 0x10
#define BITS_FIFO_ENABLE_ACCEL 0x08
#define BITS_INT_STATUS_FIFO_OVERFLOW 0x10
#define BITS_DLPF_CFG_260HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_94HZ		0x02
#define BITS_DLPF_CFG_44HZ		0x03
#define BITS_DLPF_CFG_21HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06

*/
#define BIT_POS_STBY_GX	2
#define BIT_POS_STBY_GY	1
#define BIT_POS_STBY_GZ	0

#define BIT_POS_STBY_AX	5
#define BIT_POS_STBY_AY	4
#define BIT_POS_STBY_AZ	3
#define BIT_POS_LP_WAKE_CTRL 6

#define BIT_POS_TEMP_DIS 3
#define BIT_POS_CYCLE    5
#define BIT_POS_SLEEP    6
#define BIT_POS_DEVICE_RESET    7

#define BIT_POS_CLKSEL  0

#define BIT_POS_DATA_RDY_EN 0
#define BIT_POS_FIFO_OFLOW_EN 4

#define BIT_POS_DATA_RDY_INT    0
#define BIT_POS_FIFO_OFLOW_INT  4

#define BIT_POS_DLPF_CFG        0
#define BIT_POS_FS_SEL          3
#define BIT_POS_AFS_SEL         3

#define BIT_POS_FIFO_EN         6

#define BIT_POS_ACCEL_FIFO_EN   3
#define BIT_POS_ZG_FIFO_EN      4
#define BIT_POS_YG_FIFO_EN      5
#define BIT_POS_XG_FIFO_EN      6
#define BIT_POS_TEMP_FIFO_EN    7
#define BIT_POS_SLV3_FIFO_EN    5

#define BIT_POS_GYRO_RESET      2
#define BIT_POS_ACCEL_RESET     1
#define BIT_POS_TEMP_RESET      0

#define BIT_POS_SIG_COND_RESET  0
#define BIT_POS_FIFO_RESET      2
#define BIT_POS_FIFO_EN         6

typedef struct Vectors
{
    float XAxis;
    float YAxis;
    float ZAxis;
}Vector;


typedef struct Activitie
{
    bool isOverflow;
    bool isFreeFall;
    bool isInactivity;
    bool isActivity;
    bool isPosActivityOnX;
    bool isPosActivityOnY;
    bool isPosActivityOnZ;
    bool isNegActivityOnX;
    bool isNegActivityOnY;
    bool isNegActivityOnZ;
    bool isDataReady;
}Activites;



typedef enum
{
    MPU6050_CLOCK_KEEP_RESET      = 0b111,
    MPU6050_CLOCK_EXTERNAL_19MHZ  = 0b101,
    MPU6050_CLOCK_EXTERNAL_32KHZ  = 0b100,
    MPU6050_CLOCK_PLL_ZGYRO       = 0b011,
    MPU6050_CLOCK_PLL_YGYRO       = 0b010,
    MPU6050_CLOCK_PLL_XGYRO       = 0b001,
    MPU6050_CLOCK_INTERNAL_8MHZ   = 0b000
} mpu6050_clockSource_t;

typedef enum
{
    MPU6050_SCALE_2000DPS         = 0b11,
    MPU6050_SCALE_1000DPS         = 0b10,
    MPU6050_SCALE_500DPS          = 0b01,
    MPU6050_SCALE_250DPS          = 0b00
} mpu6050_dps_t;

typedef enum
{
    MPU6050_RANGE_16G             = 0b11,
    MPU6050_RANGE_8G              = 0b10,
    MPU6050_RANGE_4G              = 0b01,
    MPU6050_RANGE_2G              = 0b00,
} mpu6050_range_t;

typedef enum
{
    MPU6050_DELAY_3MS             = 0b11,
    MPU6050_DELAY_2MS             = 0b10,
    MPU6050_DELAY_1MS             = 0b01,
    MPU6050_NO_DELAY              = 0b00,
} mpu6050_onDelay_t;

typedef enum
{
    MPU6050_DHPF_HOLD             = 0b111,
    MPU6050_DHPF_0_63HZ           = 0b100,
    MPU6050_DHPF_1_25HZ           = 0b011,
    MPU6050_DHPF_2_5HZ            = 0b010,
    MPU6050_DHPF_5HZ              = 0b001,
    MPU6050_DHPF_RESET            = 0b000,
} mpu6050_dhpf_t;

typedef enum
{
    MPU6050_DLPF_6                = 0b110,
    MPU6050_DLPF_5                = 0b101,
    MPU6050_DLPF_4                = 0b100,
    MPU6050_DLPF_3                = 0b011,
    MPU6050_DLPF_2                = 0b010,
    MPU6050_DLPF_1                = 0b001,
    MPU6050_DLPF_0                = 0b000,
} mpu6050_dlpf_t;




void mpu6050_enable(void);


void mpu6050_disable(void);


/** @file
* @brief MPU6050 gyro/accelerometer driver.
*
*
* @defgroup nrf_drivers_mpu6050 MPU6050 gyro/accelerometer driver
* @{
* @ingroup nrf_drivers
* @brief MPU6050 gyro/accelerometer driver.
*/

/**
 * @brief Function for initializing MPU6050 and verifies it's on the bus.
 *
 * @param device_address Device TWI address in bits [6:0].
 * @return
 * @retval true MPU6050 found on the bus and ready for operation.
 * @retval false MPU6050 not found on the bus or communication failure.
 */
bool mpu6050_init(uint8_t device_address);

/**
  @brief Function for writing a MPU6050 register contents over TWI.
  @param[in]  register_address Register address to start writing to
  @param[in] value Value to write to register
  @retval true Register write succeeded
  @retval false Register write failed
*/
bool mpu6050_register_write(uint8_t register_address, const uint8_t value);

/**
  @brief Function for reading MPU6050 register contents over TWI.
  Reads one or more consecutive registers.
  @param[in]  register_address Register address to start reading from
  @param[in]  number_of_bytes Number of bytes to read
  @param[out] destination Pointer to a data buffer where read data will be stored
  @retval true Register read succeeded
  @retval false Register read failed
*/
bool mpu6050_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);

/**
  @brief Function for reading and verifying MPU6050 product ID.
  @retval true Product ID is what was expected
  @retval false Product ID was not what was expected
*/
bool mpu6050_verify_product_id(void);

/**
 *@}
 **/

/*lint --flb "Leave library region" */



void mpu6050_loop(void);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);

void read_acc(float *x, float *y, float *z);
void read_angle(float *x, float *y);

void read_gyro(float *x, float *y, float *z);
void read_temp(float *temp);

void mpu6050_setup(void);


bool mpu6050_start(mpu6050_dps_t scale, mpu6050_range_t range);

void mpu6050_setScale(mpu6050_dps_t scale);
mpu6050_dps_t mpu6050_getScale(void);

void mpu6050_setRange(mpu6050_range_t range);

mpu6050_range_t mpu6050_getRange(void);
void mpu6050_setDHPFMode(mpu6050_dhpf_t dhpf);
void mpu6050_setDLPFMode(mpu6050_dlpf_t dlpf);


void mpu6050_setClockSource(mpu6050_clockSource_t source);

mpu6050_clockSource_t mpugetClockSource(void);
bool mpu6050_getSleepEnabled(void);

void mpu6050_setSleepEnabled(bool state);

bool mpu6050_getIntZeroMotionEnabled(void);

void mpu6050_setIntZeroMotionEnabled(bool state);

bool mpu6050_getIntMotionEnabled(void);

void mpu6050_setIntMotionEnabled(bool state);
bool mpu6050_getIntFreeFallEnabled(void);

void mpu6050_setIntFreeFallEnabled(bool state);

uint8_t mpu6050_getMotionDetectionThreshold(void);

void mpu6050_setMotionDetectionThreshold(uint8_t threshold);

uint8_t mpu6050_getMotionDetectionDuration(void);

void mpu6050_setMotionDetectionDuration(uint8_t duration);

uint8_t mpu6050_getZeroMotionDetectionThreshold(void);

void mpu6050_setZeroMotionDetectionThreshold(uint8_t threshold);

uint8_t mpu6050_getZeroMotionDetectionDuration(void);

void mpu6050_setZeroMotionDetectionDuration(uint8_t duration);

uint8_t mpu6050_getFreeFallDetectionThreshold(void);

void mpu6050_setFreeFallDetectionThreshold(uint8_t threshold);

uint8_t mpu6050_getFreeFallDetectionDuration(void);

void mpu6050_setFreeFallDetectionDuration(uint8_t duration);

bool mpu6050_getI2CMasterModeEnabled(void);

void mpu6050_setI2CMasterModeEnabled(bool state);

void mpu6050_setI2CBypassEnabled(bool state);
bool mpu6050_getI2CBypassEnabled(void);

void mpu6050_setAccelPowerOnDelay(mpu6050_onDelay_t delay);

mpu6050_onDelay_t mpu6050_getAccelPowerOnDelay(void);

uint8_t mpu6050_getIntStatus(void);

Activites mpu6050_readActivites(void);

void mpu6050_readRawAccel(void);

Vector mpu6050_readNormalizeAccel(void);

Vector mpu6050_readScaledAccel(void);


void mpu6050_readRawGyro(void);

Vector mpu6050_readNormalizeGyro(void);

float mpu6050_readTemperature(void);

int16_t mpu6050_getGyroOffsetX(void);

int16_t mpu6050_getGyroOffsetY(void);

int16_t mpu6050_getGyroOffsetZ(void);
void mpu6050_setGyroOffsetX(int16_t offset);

void mpu6050_setGyroOffsetY(int16_t offset);

void mpu6050_setGyroOffsetZ(int16_t offset);

int16_t mpu6050_getAccelOffsetX(void);
int16_t mpu6050_getAccelOffsetY(void);

int16_t mpu6050_getAccelOffsetZ(void);

void mpu6050_setAccelOffsetX(int16_t offset);

void mpu6050_setAccelOffsetY(int16_t offset);

void mpu6050_setAccelOffsetZ(int16_t offset);

// Calibrate algorithm
void mpu6050_calibrateGyro(uint8_t samples);

// Get current threshold value
uint8_t mpu6050_getThreshold(void);

// Set treshold value
void mpu6050_setThreshold(uint8_t multiple);

// Fast read 8-bit from register
uint8_t mpu6050_fastRegister8(uint8_t reg);

// Read 8-bit from register
uint8_t mpu6050_readRegister8(uint8_t reg);

// Write 8-bit to register
void mpu6050_writeRegister8(uint8_t reg, uint8_t value);

int16_t mpu6050_readRegister16(uint8_t reg);

void mpu6050_writeRegister16(uint8_t reg, int16_t value);

// Read register bit
bool mpu6050_readRegisterBit(uint8_t reg, uint8_t pos);

// Write register bit
void mpu6050_writeRegisterBit(uint8_t reg, uint8_t pos, bool state);
#endif /* MPU6050_H */
