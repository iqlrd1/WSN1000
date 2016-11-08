/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      tempsensor_hw.h
 *
 *  DESCRIPTION
 *      This file defines a interface to abstract hardware specifics of
 *      temperature sensor.
 *
 *  NOTES
 *
 ******************************************************************************/
#ifndef __TEMPSENSOR_HW_H__
#define __TEMPSENSOR_HW_H__

/*============================================================================*
 *  Public Definitions
 *============================================================================*/
/* Converting 273.15 to 1/32kelvin results in 8740.8. Add the 0.025 to 
 * compensate loss due to integer division.
 */
#define INTEGER_DIV_LOSS_FACTOR (0.025)

/* Celsius to 1/32 kelvin conversion factor = (273.15 * 32) */
#define CELSIUS_TO_KELVIN_FACTOR \
                             (uint16)((273.15 + INTEGER_DIV_LOSS_FACTOR) * 32)

/* Number of Timers required for temperature sensor. */
#define NUM_TEMP_SENSOR_TIMERS      (1)

/* Function pointer for Temperature Sensor Event Callback. */
typedef void (*TEMPSENSOR_EVENT_HANDLER_T)(int16 temp);

/* Temperature Sensor Hardware Initialization function. */
extern bool TempSensorHardwareInit(TEMPSENSOR_EVENT_HANDLER_T handler);

/* This function initiates a Read temperature operation.
 * Temperature is reported in the Event Handler registered.
 */
extern bool TempSensorRead(void);

#endif /* __TEMPSENSOR_HW_H__ */
