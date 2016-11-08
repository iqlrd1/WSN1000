/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      BMP180_temperature_sensor.h
 *
 *  DESCRIPTION
 *      Header file for BMP180 Temperature Sensor
 *
 *****************************************************************************/

#ifndef __SENSOR_BMP180_BAROMETER_H__
#define __SENSOR_BMP180_BAROMETER_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <types.h>
#include <time.h>
/*============================================================================*
 *  local Header Files
 *============================================================================*/
//#include "user_config.h"
#include "iic.h"

#ifdef ENABLE_BMP180 //

/*============================================================================*
 *  Public Definitions
 *============================================================================*/
/* Temperature sensor BMP180 specific definitions */
#define BMP180_I2C_ADDRESS (0xEE)
#define BMP180_MANU_ID     (0x55)  //出厂制造商固定ID

short ac1,ac2,ac3,b1,b2;//EEPROM的11个校正系数
unsigned short ac4,ac5,ac6;
short mb,mc,md;
long temperature1,pressure;//最终的温度和气压值


/* BMP180 I2C registers */
#define REG_AC1            (0xAA)  //便于理解，提高可阅读性
#define REG_AC2            (0xAC)
#define REG_AC3            (0xAE)
#define REG_AC4            (0xB0)
#define REG_AC5            (0xB2)
#define REG_AC6            (0xB4)
#define REG_B1             (0xB6)
#define REG_B2             (0xB8)
#define REG_MB             (0xBA)
#define REG_MC             (0xBC)
#define REG_MD             (0xBE)

#define REG_OUT_XLSB       (0xF8)
#define REG_OUT_LSB        (0xF7)
#define REG_OUT_MSB        (0xF6)
#define REG_CTRL_MEAS      (0xF4)
#define REG_ID        (0xD0)/**/
#define OSS 0              //oversampling setting(pressure):ultra low power mode

/* Configuration register bit fields */
/* Enable/disable event assertion in out of bounds
 * condition
 */
#define BMP180_ENABLE_EVENT            (0x00)
#define BMP180_DISABLE_EVENT           (0x80)

/* Mode */
#define CONTINUOUS_CONV                 (0x00)
#define STANDBY_MODE                    (0x40)

/* Temperature resolution bits */
#define TRES_12_BITS                    (0x03 << 3)
#define TRES_11_BITS                    (0x01 << 3)
#define TRES_10_BITS                    (0x00 << 3)
#define TRES_09_BITS                    (0x02 << 3)

/* Fractional Mask depending upon the resolution */
#define TRES_09_BITS_FRACTIONAL_MASK    (0x80)
#define TRES_10_BITS_FRACTIONAL_MASK    (0xC0)
#define TRES_11_BITS_FRACTIONAL_MASK    (0xE0)
#define TRES_12_BITS_FRACTIONAL_MASK    (0xF0)

/* Right Shift value for Fractional Part.
 * Resolution      : 9/10/11/12 bits.
 * Integer Part    : 8-bits.
 * Fractional Part : Resolution - Integer Part
 */
#define TRES_BITS_SHIFT(res)            (16 - (res))

/* Status bits  */
#define STATUS_BUSY_BITMASK             (0x80)
#define STATUS_TEMP_HI_BITMASK          (0x40)
#define STATUS_TEMP_LO_BITMASK          (0x20)

/* Maximum conversion time in milliseconds
 * Conversion times for the BMP180 sensor
 *     9-bit Typical: 10.5ms Max: 14 ms
 *    10-bit Typical: 21ms   Max: 28 ms
 *    11-bit Typical: 42ms   Max: 56 ms
 *    12-bit Typical: 84ms   Max: 112 ms
 */
#define MAX_CONVERSION_TIME             (112)

/* Invalid temperature. */
#define INVALID_TEMPERATURE             ((int16)0xFFFF)

/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/
/* This function initialises the temperature sensor BMP180 */
extern bool BMP180_Init(void);

/* This function reads data from the temperature sensor BMP180 */
extern bool BMP180_ReadTemperature(uint16 *temp,uint32 *pres);

/* This function calibrates the temperature sensor BMP180. */
extern void BMP180_Calibration(void);

extern void printDecimal(long val);

/* This function implements the callback function for read operation.*/
extern void BMP180_ReadCallback(void);

/* This function shuts down the temperature sensor BMP180 */
extern void BMP180_ShutDown(void);

/* This function handles the interrupt from temperature sensor BMP180 */
extern void BMP180_InterruptHandler(void);

#endif /* TEMPERATURE_SENSOR_BMP180 */
#endif /* __BMP180_TEMPERATURE_SENSOR_H__ */

