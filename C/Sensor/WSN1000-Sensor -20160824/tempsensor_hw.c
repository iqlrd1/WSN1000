/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      tempsensor_hw.c
 *
 *  DESCRIPTION
 *      This file implements the abstraction for temperature sensor hardware
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>
#include <timer.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
#include "tempsensor_hw.h"
#ifdef TEMPERATURE_SENSOR_STTS751
#include "stts751_temperature_sensor.h"
#endif /* TEMPERATURE_SENSOR_STTS751 */

/*============================================================================*
 *  Private data
 *============================================================================*/
/* Temperature sensor read delay after issuing read command. */
#ifdef TEMPERATURE_SENSOR_STTS751
#define TEMP_READ_DELAY         (MAX_CONVERSION_TIME * MILLISECOND)
#endif /* TEMPERATURE_SENSOR_STTS751 */

/* Event handler to be called after temperature is read from sensor. */
static TEMPSENSOR_EVENT_HANDLER_T eventHandler;

/* Timer ID for temperature read delay. */
static timer_id read_delay_tid = TIMER_INVALID;

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      tempSensorReadyToRead
 *
 *  DESCRIPTION
 *      This function is called after a duration of temperature read delay,
 *      once a read is initiated.
 *
 *  RETURNS
 *      TRUE if initialization is sucessful.
 *
 *----------------------------------------------------------------------------*/
static void tempSensorReadyToRead(timer_id tid)
{
    int16 temp;

    if (tid == read_delay_tid)
    {
        read_delay_tid = TIMER_INVALID;

#ifdef TEMPERATURE_SENSOR_STTS751
        /* Read the temperature. */
        STTS751_ReadTemperature(&temp);
        if (temp != INVALID_TEMPERATURE)
        {
            /* Convert temperature in to 1/32 degree Centigrade units */
            temp = (temp << 1);

            temp += CELSIUS_TO_KELVIN_FACTOR;
        }
#endif /* TEMPERATURE_SENSOR_STTS751 */

        /* Report the temperature read. */
        eventHandler(temp);
    }
}

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      TempSensorHardwareInit
 *
 *  DESCRIPTION
 *      This function initialises the temperature sensor hardware.
 *
 *  RETURNS
 *      TRUE if initialization is sucessful.
 *
 *----------------------------------------------------------------------------*/
extern bool TempSensorHardwareInit(TEMPSENSOR_EVENT_HANDLER_T handler)
{
    bool status = FALSE;

    read_delay_tid = TIMER_INVALID;

    if (NULL != handler)
    {
        eventHandler = handler;
        #ifdef TEMPERATURE_SENSOR_STTS751
                status = STTS751_Init();
        #endif /* TEMPERATURE_SENSOR_STTS751 */
    }

    return status;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      TempSensorRead
 *
 *  DESCRIPTION
 *      This function sends a temperature read command to the sensor.
 *      Temperature will be reported in the registered event handler.
 *
 *  RETURNS
 *      TRUE command is sent.
 *
 *----------------------------------------------------------------------------*/
extern bool TempSensorRead(void)
{
    bool status = FALSE;

    /* Return FALSE if already a read is in progress. */
    if (TIMER_INVALID == read_delay_tid)
    {
#ifdef TEMPERATURE_SENSOR_STTS751
        status = STTS751_InitiateOneShotRead();
#endif /* TEMPERATURE_SENSOR_STTS751 */
    }

    /* Command is issued without failure, start the delay timer. */
    if (status)
    {
        read_delay_tid = TimerCreate(TEMP_READ_DELAY, TRUE,
                                     tempSensorReadyToRead);
    }

    return status;
}

