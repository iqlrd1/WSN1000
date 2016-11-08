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
#include <time.h>
/*=============================================================================
 *  Local Header Files
 *============================================================================*/
#include "iic.h"

/*=============================================================================
 *  Public function definitions
 *============================================================================*/
void pioConfigure()/*IO口配置时产生一下降沿？？*/
{   PioSetModes(bitmask1(SDA1)|bitmask1(SCL1),pio_mode_user);
    PioSetDirs(bitmask1(SCL1)|bitmask1(SCL1),bitmask1(SCL1)|bitmask1(SCL1));//SDA1方向无定
    PioSetPullModes(bitmask1(SDA1)|bitmask1(SCL1),pio_mode_strong_pull_down);
}

void I2cStart()
{   PioSetDir(SDA1,1);////output(默认就是输出)
    PioSet(SCL1,1);TimeDelayUSec(5);
    PioSet(SDA1,1);TimeDelayUSec(5);//>4.7us
    PioSet(SDA1,0);TimeDelayUSec(5);//>4us,产生下降沿
    PioSet(SCL1,0);
    //DebugWriteString("Start signal. ");
}

void I2cStop()
{   PioSetDir(SDA1,1);////output
    PioSet(SCL1,1);PioSet(SDA1,0);
    TimeDelayUSec(5);//>4us
    PioSet(SDA1,1);
    TimeDelayUSec(15);//>4.7us,产生上升沿
    //PioSet(SCL1,0);
}

void I2cSendbyte(uint8 dat)
{   uint16 i=0;PioSetDir(SDA1,1);////output
    PioSet(SCL1,0);    
    for(;i<8;i++) {
        PioSet(SDA1,(bool)(dat&0x80));
        PioSet(SCL1,1);//8个时钟
        dat<<=1;TimeDelayUSec(12);
        PioSet(SCL1,0);TimeDelayUSec(12);
    }
    //DebugWriteString("Entering I2cSendbyte\r\n");
}
bool I2cReceiveack()//从机回应信号
{   bool ack;
    /*PioSetDir(SDA1,1);//先输出，再输入接收
    PioSet(SCL1,0);PioSet(SDA1,1);*///释放总线等待ACK/NACK的到来
    //PioSetDir(SDA1,0);//input
    PioSet(SCL1,1);TimeDelayUSec(12);
    ack=PioGet(SDA1);
    PioSet(SCL1,0);TimeDelayUSec(12);//第9个时钟
    return ack;
    //DebugWriteString("Entering I2cReceiveack\r\n");
}

uint8 I2cReceivebyte()
{   
    //PioSetDir(SDA1,1);//output
    //PioSet(SDA1,1);
    PioSetDir(SDA1,0);//input
    uint8 j,dat=0;
    for(j=0;j<8;j++) {TimeDelayUSec(12);
        //PioSet(SCL1,0);
        dat<<=1;
        PioSet(SCL1,1);TimeDelayUSec(12);
        if(PioGet(SDA1)) dat+=1;//高位先出(这一句夹在下降沿之间，否则后续字节接收不到)       
        PioSet(SCL1,0);              
    }
    return dat;
}
void I2cSendack(bool ack)
{   PioSetDir(SDA1,1);//PioSet(SCL1,0);
    PioSet(SDA1,ack);
    PioSet(SCL1,1);TimeDelayUSec(12);/**/    
    PioSet(SCL1,0);TimeDelayUSec(12);
    //PioSet(SDA1,0);
}

void I2cWriteRegister(uint8 slave_addr,uint8 reg_addr,uint8 instword)
{   bool a,b,c=0;
    I2cStart();          //S
    I2cSendbyte(slave_addr);a=I2cReceiveack();
    if(a==NACK) I2cStop();
    I2cSendbyte(reg_addr);  b=I2cReceiveack();
    if(b==NACK) I2cStop();
    I2cSendbyte(instword);//指令控制字
    c=I2cReceiveack();/*等待最后一字节的应答(把握),此处无误1*/
    I2cStop();           //P  
    //DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
    //DebugWriteString("Entering I2cWriteRegister.\r\n");
}

void I2cReadRegister(uint8 slave_addr,uint8 reg_addr,uint8 *p_buffer)
{   I2cStart();bool a,b,c=0;
    I2cSendbyte(slave_addr);//写
    a=I2cReceiveack();
    I2cSendbyte(reg_addr);
    b=I2cReceiveack();
    I2cStart();  //Sr
    I2cSendbyte(slave_addr+1);//读
    c=I2cReceiveack();/*此处也有问题:却能出来结果？？2*/
    *p_buffer=I2cReceivebyte();//数据到来：获取到一字节并存放其中
    I2cSendack(NACK);//MNACK终止读入
    I2cStop();
    //DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
}

void I2cReadRegisters(uint8 slave_addr,uint8 reg_addr,uint8 p_buffer[],uint8 num)
{   I2cStart();bool a,b,c=0;
    I2cSendbyte(slave_addr);//写
    a=I2cReceiveack();
    if(a==NACK) I2cStop();
    I2cSendbyte(reg_addr);
    b=I2cReceiveack();
    if(b==NACK) I2cStop();
    I2cStart();
    I2cSendbyte(slave_addr+1);//指令字
    c=I2cReceiveack();/*此处应答信号有差,第二个字节接收累积出错3*/
    //if(c==NACK) I2cStop();/*又是此处回应信号出问题（逻辑误判:波形非高非低）*/
    uint8 count;
    for(count=0;count<num-1;count++)
    {   p_buffer[count]=I2cReceivebyte();//该固件库函数能控制寄存器地址的自增保证连续读取
        I2cSendack(ACK);
    }
    p_buffer[num-1]=I2cReceivebyte();//此函数有些智能：寄存器地址跟着自增-------》
    I2cSendack(NACK);//最后一字节与前面num-1个字节处理不同，单独对待
    I2cStop();
    for(count=0;count<=num-1;count++)
    {   DebugWriteString(" ");
        DebugWriteUint8(p_buffer[count]);       
    }/**/
    DebugWriteString("\r\n");
    //DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
}

uint16 I2cReadWord(uint8 slave_addr,uint8 reg_addr,uint8 *p_buffer)
{   I2cStart();bool a,b,c=0;//        
    I2cSendbyte(slave_addr);//写:指明访问地址
    a=I2cReceiveack();
    if(a==NACK) I2cStop();//
    I2cSendbyte(reg_addr);
    b=I2cReceiveack();
    if(b==NACK) I2cStop();//
    I2cStart();
    I2cSendbyte(slave_addr+1);//指令字（改向）
    c=I2cReceiveack();/*此处应答信号有差,第二个字节接收累积出错4*/
    //if(c==NACK) {I2cStop();DebugWriteString("Force to end\r\n");}
    *p_buffer=I2cReceivebyte();
    I2cSendack(ACK);
    *(p_buffer+1)=I2cReceivebyte();
    I2cSendack(NACK);/*MNACK*/
    I2cStop();
    
    //DebugWriteUint8(*p_buffer);DebugWriteUint8(*(p_buffer+1));
    uint16 word;
    word=((uint16)(*p_buffer))<<8|(*(p_buffer+1));//处理 
    /*DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
    DebugWriteString("\r\n");*/
    return word;
}