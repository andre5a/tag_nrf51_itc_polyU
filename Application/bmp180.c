/*
 * bmp180.c
 *
 *  Created on: 11/06/2017
 *      Author: andre
 */
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "twi_master.h"
#include "nrf_delay.h"


#include "bmp180.h"

#define ADDRESS_WHO_AM_I          (0xD0U) // !< WHO_AM_I register identifies the device. Expected value is 0x68.
#define ADDRESS_SIGNAL_PATH_RESET (0xE0U) // !<

const unsigned char OSS = 0;  // Oversampling Setting

int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;


// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;


static const uint8_t expected_who_am_i = 0x55U; // !< Expected value to get from WHO_AM_I register.
static uint8_t       m_device_address;          // !< Device address in bits [7:1]

bool bmp180_init(uint8_t device_address)
{
    bool transfer_succeeded = true;

    m_device_address = (uint8_t)(device_address << 1);

    // Do a reset on signal paths
   // uint8_t reset_value = 0x04U | 0x02U | 0x01U; // Resets gyro, accelerometer and temperature sensor signal paths.
   // transfer_succeeded &= bmp180_register_write(ADDRESS_SIGNAL_PATH_RESET, reset_value);

    // Read and verify product ID
    transfer_succeeded &= bmp180_verify_product_id();

    return transfer_succeeded;
}

bool bmp180_verify_product_id(void)
{
    uint8_t who_am_i;

    if (bmp180_register_read(ADDRESS_WHO_AM_I, &who_am_i, 1))
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

bool bmp180_register_write(uint8_t register_address, uint8_t value)
{
    uint8_t w2_data[2];

    w2_data[0] = register_address;
    w2_data[1] = value;
    return twi_master_transfer(m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}

bool bmp180_register_read(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes)
{
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
    return transfer_succeeded;
}

int16_t bmp180_readInt(uint8_t register_address)
{
	int16_t result;
	uint8_t data[2];
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, data, 2, TWI_ISSUE_STOP);
    result=data[0];
    result=result<<8;
    result|=data[1];
    return result;
}

int8_t bmp180_read(uint8_t register_address)
{
	uint8_t data;
    bool transfer_succeeded;
    transfer_succeeded  = twi_master_transfer(m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
    transfer_succeeded &= twi_master_transfer(m_device_address|TWI_READ_BIT, &data, 1, TWI_ISSUE_STOP);
    return data;
}


void bmp180_cal(void)
{
	/* read calibration data */
	ac1=bmp180_readInt(BMP180_CAL_AC1);
	ac2=bmp180_readInt(BMP180_CAL_AC2);
	ac3=bmp180_readInt(BMP180_CAL_AC3);
	ac4=bmp180_readInt(BMP180_CAL_AC4);
	ac5=bmp180_readInt(BMP180_CAL_AC5);
	ac6=bmp180_readInt(BMP180_CAL_AC6);

	b1=bmp180_readInt(BMP180_CAL_B1);
	b2=bmp180_readInt(BMP180_CAL_B2);

	mb=bmp180_readInt(BMP180_CAL_MB);
	mc=bmp180_readInt(BMP180_CAL_MC);
	md=bmp180_readInt(BMP180_CAL_MD);

}


// Calculate temperature in deg C
float bmp180GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp180GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}



// Read the uncompensated temperature value
unsigned int bmp180ReadUT(void){



	unsigned int ut;


  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
	bmp180_register_write(0xF4, 0x2E);

  // Wait at least 4.5ms
  nrf_delay_ms(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp180_readInt(0xF6);
  return ut;
}


// Read the uncompensated pressure value
unsigned long bmp180ReadUP(void){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting

	bmp180_register_write(0xF4, 0x34 + (OSS<<6));


  // Wait for conversion, delay time dependent on OSS
  nrf_delay_ms(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp180_read(0xF6);
  lsb = bmp180_read(0xF7);
  xlsb = bmp180_read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  //C = C /0.0000225577;
  C=C*44330;
  return C;
/*
float R=8,3144621;
float T0=288.15;
float g=9.80665;
float M=0.0289644;
float P0=101325;

float A=101325/pressure;
float B=8434.6598;
float C=log(A);
	A=B*C;
return A;
*/
}

/*lint --flb "Leave library region" */
