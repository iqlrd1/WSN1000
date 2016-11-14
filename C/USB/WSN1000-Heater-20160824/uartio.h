/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      uartio.h
 *
 *  DESCRIPTION
 *      UART IO header
 *
 ******************************************************************************/

#ifndef __UARTIO_H__
#define __UARTIO_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
 
#include <types.h>          /* Commonly used type definitions */
#include <sys_events.h>     /* System event definitions and declarations */
#include <sleep.h>          /* Control the device sleep states */
#include <timer.h>
#include <debug.h>

#include "csr_mesh_heater.h"
#include "app_data_stream.h"

/*20160727*/
bool idreq;
bool continueToRecv;
/*20161109*/
bool receiveDevidInProgress;
/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/
/*20160613*/
uint16 inputNumberOfSensor;
/* Transmit waiting data over UART */
void sendPendingData(void);
/*20160727*/
void handleDevidRecv(void);
/*20160830*/
#ifdef DEBUG_ENABLE
void printInDecimal(uint32 val);/* Print a number in decimal. */
#endif
/*20160830*/
void delAllStoreDedvidNVM(void);
void delDevidNVM(uint16 devid);
/*----------------------------------------------------------------------------*
 *  NAME
 *      Start
 *
 *  DESCRIPTION
 *      Run the startup routine.
 *
 * PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
extern void uartInit(void);

#endif /* __UARTIO_H__ */
