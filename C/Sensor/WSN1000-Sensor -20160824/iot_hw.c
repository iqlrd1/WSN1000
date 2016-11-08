/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      iot_hw.c
 *
 *  DESCRIPTION
 *      This file implements the LED Controller of IOT hardware
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <pio.h>            /* Programmable I/O configuration and control */

/*============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "iot_hw.h"

/*============================================================================*
 *  Private data
 *============================================================================*/
/* Colour depth in bits, as passed by application. */
#define LIGHT_INPUT_COLOR_DEPTH  (8)
/* Colour depth in bits, mapped to actual hardware. */
#define LIGHT_MAPPED_COLOR_DEPTH (6)
/* Colour depth lost due to re-quantization of levels. */
#define QUANTIZATION_ERROR       (LIGHT_INPUT_COLOR_DEPTH -\
                                  LIGHT_MAPPED_COLOR_DEPTH)
/* Maximum colour level supported by mapped colour depth bits. */
#define COLOR_MAX_VALUE          ((0x1 << LIGHT_MAPPED_COLOR_DEPTH) - 1)

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      IOTLightControlDeviceInit
 *
 *  DESCRIPTION
 *      This function initialises the Red, Green and Blue LED lines.
 *      Configure the IO lines connected to Temperature Sensor as inputs.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void IOTLightControlDeviceInit(void)
{
    /* Configure the LED_PIO_RED PIO as output PIO */
    PioSetDir(LED_PIO_RED, PIO_DIRECTION_OUTPUT);

    /* Configure the LED_PIO_GREEN PIO as output PIO */
    PioSetDir(LED_PIO_GREEN, PIO_DIRECTION_OUTPUT);

    /* Configure the LED_PIO_BLUE PIO as output PIO */
    PioSetDir(LED_PIO_BLUE, PIO_DIRECTION_OUTPUT);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IOTLightControlDevicePower
 *
 *  DESCRIPTION
 *      This function sets power state of LED.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void IOTLightControlDevicePower(bool power_on)
{
    if (power_on == TRUE)
    {
        /* Configure the LED's */

        /* Connect PWM0 output to LED_PIO_RED */
        PioSetMode(LED_PIO_RED, pio_mode_pwm0);

        /* Connect PWM1 output to LED_PIO_GREEN */
        PioSetMode(LED_PIO_GREEN, pio_mode_pwm1);

         /* Connect PWM2 output to LED_PIO_BLUE */
        PioSetMode(LED_PIO_BLUE, pio_mode_pwm2);

        /* Enable the PIO's */
        PioEnablePWM(LED_PWM_RED, TRUE);
        PioEnablePWM(LED_PWM_GREEN, TRUE);
        PioEnablePWM(LED_PWM_BLUE, TRUE);

        /* Initialise the LED to off */
        IOTLightControlDeviceSetColor(0, 0, 0);
    }
    else
    {
         /* When power off is selected, disable all PWMs and
          * set all PIOs to HIGH, as IOT board uses common anode LED.
          */
         PioSetMode(LED_PIO_RED, pio_mode_user);
         PioSetMode(LED_PIO_GREEN, pio_mode_user);
         PioSetMode(LED_PIO_BLUE, pio_mode_user);

         PioSet(LED_PIO_RED, 1);
         PioSet(LED_PIO_GREEN, 1);
         PioSet(LED_PIO_BLUE, 1);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IOTLightControlDeviceSetLevel
 *
 *  DESCRIPTION
 *      This function sets the brightness level.
 *      Convert Level to equal values of RGB.
 *      Note that linear translation has been assumed for now.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void IOTLightControlDeviceSetLevel(uint8 level)
{
    /* Maps level to equal values of Red, Green and Blue */
    IOTLightControlDeviceSetColor(level, level, level);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IOTLightControlDeviceSetColor
 *
 *  DESCRIPTION
 *      This function sets the colour as passed in argument values.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void IOTLightControlDeviceSetColor(uint8 red, uint8 green, uint8 blue)
{
    /* When level is Lowest (0-3) simply disable PWM to avoid flicker */
    if ((red >> QUANTIZATION_ERROR) == 0)
    {
         PioSetMode(LED_PIO_RED, pio_mode_user);
         PioSet(LED_PIO_RED, 1);
    }
    else
    {
        PioSetMode(LED_PIO_RED, pio_mode_pwm0);
    }

    if ((green >> QUANTIZATION_ERROR) == 0)
    {
         PioSetMode(LED_PIO_GREEN, pio_mode_user);
         PioSet(LED_PIO_GREEN, 1);
    }
    else
    {
        PioSetMode(LED_PIO_GREEN, pio_mode_pwm1);
    }

    if ((blue >> QUANTIZATION_ERROR) == 0)
    {
         PioSetMode(LED_PIO_BLUE, pio_mode_user);
         PioSet(LED_PIO_BLUE, 1);
    }
    else
    {
        PioSetMode(LED_PIO_BLUE, pio_mode_pwm2);
    }

    /* Invert values as its a pull down */
    red   = COLOR_MAX_VALUE - (red >> QUANTIZATION_ERROR);
    green = COLOR_MAX_VALUE - (green >> QUANTIZATION_ERROR);
    blue  = COLOR_MAX_VALUE - (blue >> QUANTIZATION_ERROR);

    PioConfigPWM(LED_PWM_RED, pio_pwm_mode_push_pull,
                 red, (COLOR_MAX_VALUE - red), 1U,
                 red, (COLOR_MAX_VALUE - red), 1U, 0U);
    PioConfigPWM(LED_PWM_GREEN, pio_pwm_mode_push_pull,
                 green, (COLOR_MAX_VALUE - green), 1U,
                 green, (COLOR_MAX_VALUE - green), 1U, 0U);
    PioConfigPWM(LED_PWM_BLUE, pio_pwm_mode_push_pull,
                 blue, (COLOR_MAX_VALUE - blue), 1U,
                 blue, (COLOR_MAX_VALUE - blue), 1U, 0U);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IOTLightControlDeviceBlink
 *
 *  DESCRIPTION
 *      This function sets colour and blink time for LEDs.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void IOTLightControlDeviceBlink(uint8 red, uint8 green, uint8 blue,
                                       uint8 on_time, uint8 off_time)
{
    IOTLightControlDevicePower(TRUE);

    IOTLightControlDeviceSetColor(red, green, blue);

    /* Invert the On and Off times as LEDs on
     * IOT board are Common-Anode type
     */
    PioConfigPWM(LED_PWM_RED, pio_pwm_mode_push_pull,
                 red, 0, off_time, 0, red, on_time, 0U);

    PioConfigPWM(LED_PWM_GREEN, pio_pwm_mode_push_pull,
                 green, 0, off_time, 0, green, on_time, 0U);

    PioConfigPWM(LED_PWM_BLUE, pio_pwm_mode_push_pull,
                 blue, 0, off_time, 0, blue, on_time, 0U);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      IOTSwitchInit
 *
 *  DESCRIPTION
 *      This function sets GPIO to switch mode.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void IOTSwitchInit(void)
{
    /* Set-up the PIOs for the switch */
    /*PioSetDir(SW2_PIO, PIO_DIRECTION_INPUT);
    PioSetMode(SW2_PIO, pio_mode_user);

    PioSetDir(SW3_PIO, PIO_DIRECTION_INPUT);
    PioSetMode(SW3_PIO, pio_mode_user);

    PioSetDir(SW4_PIO, PIO_DIRECTION_INPUT);
    PioSetMode(SW4_PIO, pio_mode_user);

    PioSetPullModes(BUTTONS_BIT_MASK , pio_mode_strong_pull_up);

    PioSetEventMask(BUTTONS_BIT_MASK , pio_event_mode_both);
    
    PioSetDir(SW4_PIO, PIO_DIRECTION_INPUT);
    PioSetMode(SW4_PIO, pio_mode_user);
    PioSetPullModes(PIO_BIT_MASK(SW4_PIO) , pio_mode_strong_pull_up);

    PioSetEventMask(PIO_BIT_MASK(SW4_PIO) , pio_event_mode_both);*/
        
    /*20160706*/
    PioSetMode(SEND_IND,pio_mode_user);
    PioSetDir(SEND_IND,1);
    PioSetPullModes((1L<<SEND_IND),pio_mode_strong_pull_up);
     
    /*20160801*/
    PioSetMode(PUSH_BUTTON,pio_mode_user);
    PioSetDir(PUSH_BUTTON,0);
    PioSetPullModes((1L<<PUSH_BUTTON),pio_mode_strong_pull_up);
    PioSetEventMask((1L<<PUSH_BUTTON),pio_event_mode_both);
}

