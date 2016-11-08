/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      Si7034_temperature_sensor.h
 *
 *  DESCRIPTION
 *      Header file for Si7034 Temperature Sensor
 *
 *****************************************************************************/

#ifndef __SENSOR_Si7034_H__
#define __SENSOR_Si7034_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <types.h>
#include <time.h>
/*============================================================================*
 *  local Header Files
 *============================================================================*/
#include "user_config.h"
#include "i2c_sim.h"
//#include "i2c_comms.h"

#ifdef ENABLE_SI7034

/*============================================================================*
 *  Public Definitions
 *============================================================================*/
/* Temperature sensor Si7034 specific definitions */
#define Si7034_I2C_ADDRESS             (0xe0)

uint8 temp_msb,temp_lsb,t_checksum;
uint8 humi_msb,humi_lsb,h_checksum;
int16 temperature;uint16 humidity;//全局变量
uint8 fw_revision;


/* Si7034 I2C registers */
//#define REG_TEMP_MSB                    (0x00)
#define REG_STATUS                      (0x01)
#define REG_TEMP_LSB                    (0x02)
#define REG_CONFIG                      (0x03)
#define REG_CONV_RATE                   (0x04)
#define REG_TEMP_HI_LIM_MSB             (0x05)
#define REG_TEMP_HI_LIM_LSB             (0x06)
#define REG_TEMP_LO_LIM_MSB             (0x07)
#define REG_TEMP_LO_LIM_LSB             (0x08)
#define REG_ONE_SHOT_READ               (0x0F)
#define REG_THERM_LIM                   (0x20)
#define REG_THERM_HYS                   (0x21)
#define REG_I2C_TIMEOUT                 (0x22)
#define REG_PID                         (0xFD)
//#define REG_MANU_ID                     (0xFE)
#define REG_REVISION                    (0xFF)

/* Configuration register bit fields */
/* Enable/disable event assertion in out of bounds
 * condition
 */
#define Si7034_ENABLE_EVENT            (0x00)
#define Si7034_DISABLE_EVENT           (0x80)

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
 * Conversion times for the Si7034 sensor
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
/* This function initialises the temperature sensor Si7034 */
extern bool Si7034_Init(void);

/* This function calibrates the temperature sensor Si7034. */
extern void Si7034_Calibration(void);

/* This function reads data from the temperature sensor Si7034 */
extern bool Si7034_ReadTemperature(uint16 *temp,uint16 *humi);
extern uint8 cal_crc(uint8 *ptr, uint8 n);/*数据CRC8-MAXIUM校验*/
extern void printDec(long val);


/* This function implements the callback function for read operation.*/
extern void Si7034_ReadCallback(void);

/* This function shuts down the temperature sensor Si7034 */
extern void Si7034_ShutDown(void);

/* This function handles the interrupt from temperature sensor Si7034 */
extern void Si7034_InterruptHandler(void);

#endif /* TEMPERATURE_SENSOR_Si7034 */
#endif /* __Si7034_TEMPERATURE_SENSOR_H__ */

