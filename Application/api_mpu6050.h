/*
 * api_mpu6050.h
 *
 *  Created on: 13/06/2017
 *      Author: andre
 */

#ifndef API_MPU6050_H_
#define API_MPU6050_H_

#include <stdint.h>
#include <string.h>

struct fifo_packet {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};

#define MPU6050_ONE_G	9.80665f

#define MIN(_x, _y) (_x) > (_y) ? (_y) : (_x)

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// LSB sensitivity from the datasheet is 16.4 LSB/degree/s at +-2000 degrees/s
// and deg to rad conversion
#define GYRO_RAW_TO_RAD_S 	 (M_PI_F / 180.0f / 16.4f)

#define DIR_READ			0x80
#define DIR_WRITE			0x00


// Length of the FIFO used by the sensor to buffer unread
// sensor data. The FIFO size of the MPU6050 is 1024 bytes.
// However, we sample and read with 1 kHz, so that most often
// there is just one packet in the FIFO.
// Including some slack, we allocate memory for four packets.
#define MPU_MAX_LEN_FIFO_IN_BYTES (4 * sizeof(fifo_packet))

// Uncomment to allow additional debug output to be generated.
// #define MPU6050_DEBUG 1



#endif /* API_MPU6050_H_ */
