/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 * FILE
 *    i2c_comms.h
 *
 *  DESCRIPTION
 *    Header file for the I2C procedures
 *
 ******************************************************************************/
#ifndef _I2C_SIM_H_
#define _I2C_SIM_H_

/*=============================================================================
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>     //IO¿ÚÄ£Äâ
#include <types.h>
#include <debug.h>
#include <status.h>
#include <time.h>


#define SDA 9
#define SCL 10
#define ACK  0
#define NACK 1
#define bitmask(x) (1L<<x)
/*=============================================================================
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
sys_status state;//
/*=============================================================================
 *  Public function prototypes
 *============================================================================*/

void pio_configure(void);
void I2c_start(void);
void I2c_stop(void);
bool I2c_receiveack(void);
void I2c_sendack(bool ack);
void I2c_sendbyte(uint8 dat);
uint8 I2c_receivebyte(void);

extern void I2cWriteCommands(uint8 slave_addr,uint8 num,uint8 instruction[]);

#endif /* _I2C_COMMS_H */
