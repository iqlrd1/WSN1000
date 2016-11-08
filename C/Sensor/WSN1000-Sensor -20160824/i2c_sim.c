/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *    i2c_comms.c
 *
 *  DESCRIPTION
 *    This file defines different I2C procedures.
 *
 ******************************************************************************/

/*=============================================================================
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>
#include <types.h>
#include <i2c_sim.h>

/*=============================================================================
 *  Local Header Files
 *============================================================================*/
#include "i2c_comms.h"
#include "user_config.h"

/*=============================================================================
 *  Public function definitions
 *============================================================================*/

void pio_configure()/*IO口配置时产生一下降沿？？*/
{   PioSetModes(bitmask(SDA)|bitmask(SCL),pio_mode_user);
    PioSetDirs(bitmask(SCL)|bitmask(SCL),bitmask(SCL)|bitmask(SCL));//SDA方向无定
    PioSetPullModes(bitmask(SDA)|bitmask(SCL),pio_mode_strong_pull_down);
}

void I2c_start()
{   PioSetDir(SDA,1);////output(默认就是输出)
    PioSet(SCL,1);TimeDelayUSec(5);
    PioSet(SDA,1);TimeDelayUSec(5);//>4.7us
    PioSet(SDA,0);TimeDelayUSec(5);//>4us,产生下降沿
    PioSet(SCL,0);
    //DebugWriteString("Start signal. ");
}

void I2c_stop()
{   PioSetDir(SDA,1);////output
    PioSet(SCL,1);PioSet(SDA,0);
    TimeDelayUSec(5);//>4us
    PioSet(SDA,1);
    TimeDelayUSec(15);//>4.7us,产生上升沿
    //PioSet(SCL,0);
}

void I2c_sendbyte(uint8 dat)
{   uint16 i=0;PioSetDir(SDA,1);////output
    PioSet(SCL,0);    
    for(;i<8;i++) {
        PioSet(SDA,(bool)(dat&0x80));
        PioSet(SCL,1);//8个时钟
        dat<<=1;TimeDelayUSec(12);
        PioSet(SCL,0);TimeDelayUSec(12);
    }
    //DebugWriteString("Entering I2c_sendbyte\r\n");
}
bool I2c_receiveack()//从机回应信号
{   bool ack;
    /*PioSetDir(SDA,1);//先输出，再输入接收
    PioSet(SCL,0);PioSet(SDA,1);*///释放总线等待ACK/NACK的到来
    //PioSetDir(SDA,0);//input
    PioSet(SCL,1);TimeDelayUSec(12);
    ack=PioGet(SDA);
    PioSet(SCL,0);TimeDelayUSec(12);//第9个时钟
    return ack;
    //DebugWriteString("Entering I2c_receiveack\r\n");
}

uint8 I2c_receivebyte()
{   
    //PioSetDir(SDA,1);//output
    //PioSet(SDA,1);
    PioSetDir(SDA,0);//input
    uint8 j,dat=0;
    for(j=0;j<8;j++) {TimeDelayUSec(12);
        //PioSet(SCL,0);
        dat<<=1;
        PioSet(SCL,1);TimeDelayUSec(12);
        if(PioGet(SDA)) dat+=1;//高位先出(这一句夹在下降沿之间，否则后续字节接收不到)       
        PioSet(SCL,0);              
    }
    return dat;
}
void I2c_sendack(bool ack)
{   PioSetDir(SDA,1);//PioSet(SCL,0);
    PioSet(SDA,ack);
    PioSet(SCL,1);TimeDelayUSec(12);/**/    
    PioSet(SCL,0);TimeDelayUSec(12);
    //PioSet(SDA,0);
}

void I2cWriteCommands(uint8 slave_addr,uint8 num,uint8 instruction[])
{   uint8 n;
    I2c_start();  //S
    I2c_sendbyte(slave_addr);//写指令字：访问IIC器件地址
    I2c_receiveack();
    for(n=0;n<num;n++)
    {   I2c_sendbyte(instruction[n]);
        I2c_receiveack();
    }              
}