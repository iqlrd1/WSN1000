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
#ifndef IIC_H
#define IIC_H

/*=============================================================================
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>     //BMP180 IO¿ÚÄ£Äâ
#include <types.h>
#include <debug.h>
#include <status.h>

#define SDA1 9
#define SCL1 10
#define ACK  0
#define NACK 1
#define bitmask1(x) (1L<<x)
/*=============================================================================
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
//sys_status state;
/*=============================================================================
 *  Public function prototypes
 *============================================================================*/
void pioConfigure(void);
void I2cStart(void);
void I2cStop(void);
bool I2cReceiveack(void);
void I2cSendack(bool ack);
void I2cSendbyte(uint8 dat);
uint8 I2cReceivebyte(void);

void I2cWriteRegister(uint8 slave_addr,uint8 reg_addr,uint8 instword);
void I2cReadRegister(uint8 slave_addr,uint8 reg_addr,uint8 *p_buffer);
void I2cReadRegisters(uint8 slave_addr,uint8 reg_addr,uint8 p_buffer[],uint8 num);
uint16 I2cReadWord(uint8 slave_addr,uint8 reg_addr,uint8 *p_buffer);

#endif /* _I2C_COMMS_H */
