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
void pioConfigure()/*IO������ʱ����һ�½��أ���*/
{   PioSetModes(bitmask1(SDA1)|bitmask1(SCL1),pio_mode_user);
    PioSetDirs(bitmask1(SCL1)|bitmask1(SCL1),bitmask1(SCL1)|bitmask1(SCL1));//SDA1�����޶�
    PioSetPullModes(bitmask1(SDA1)|bitmask1(SCL1),pio_mode_strong_pull_down);
}

void I2cStart()
{   PioSetDir(SDA1,1);////output(Ĭ�Ͼ������)
    PioSet(SCL1,1);TimeDelayUSec(5);
    PioSet(SDA1,1);TimeDelayUSec(5);//>4.7us
    PioSet(SDA1,0);TimeDelayUSec(5);//>4us,�����½���
    PioSet(SCL1,0);
    //DebugWriteString("Start signal. ");
}

void I2cStop()
{   PioSetDir(SDA1,1);////output
    PioSet(SCL1,1);PioSet(SDA1,0);
    TimeDelayUSec(5);//>4us
    PioSet(SDA1,1);
    TimeDelayUSec(15);//>4.7us,����������
    //PioSet(SCL1,0);
}

void I2cSendbyte(uint8 dat)
{   uint16 i=0;PioSetDir(SDA1,1);////output
    PioSet(SCL1,0);    
    for(;i<8;i++) {
        PioSet(SDA1,(bool)(dat&0x80));
        PioSet(SCL1,1);//8��ʱ��
        dat<<=1;TimeDelayUSec(12);
        PioSet(SCL1,0);TimeDelayUSec(12);
    }
    //DebugWriteString("Entering I2cSendbyte\r\n");
}
bool I2cReceiveack()//�ӻ���Ӧ�ź�
{   bool ack;
    /*PioSetDir(SDA1,1);//����������������
    PioSet(SCL1,0);PioSet(SDA1,1);*///�ͷ����ߵȴ�ACK/NACK�ĵ���
    //PioSetDir(SDA1,0);//input
    PioSet(SCL1,1);TimeDelayUSec(12);
    ack=PioGet(SDA1);
    PioSet(SCL1,0);TimeDelayUSec(12);//��9��ʱ��
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
        if(PioGet(SDA1)) dat+=1;//��λ�ȳ�(��һ������½���֮�䣬��������ֽڽ��ղ���)       
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
    I2cSendbyte(instword);//ָ�������
    c=I2cReceiveack();/*�ȴ����һ�ֽڵ�Ӧ��(����),�˴�����1*/
    I2cStop();           //P  
    //DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
    //DebugWriteString("Entering I2cWriteRegister.\r\n");
}

void I2cReadRegister(uint8 slave_addr,uint8 reg_addr,uint8 *p_buffer)
{   I2cStart();bool a,b,c=0;
    I2cSendbyte(slave_addr);//д
    a=I2cReceiveack();
    I2cSendbyte(reg_addr);
    b=I2cReceiveack();
    I2cStart();  //Sr
    I2cSendbyte(slave_addr+1);//��
    c=I2cReceiveack();/*�˴�Ҳ������:ȴ�ܳ����������2*/
    *p_buffer=I2cReceivebyte();//���ݵ�������ȡ��һ�ֽڲ��������
    I2cSendack(NACK);//MNACK��ֹ����
    I2cStop();
    //DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
}

void I2cReadRegisters(uint8 slave_addr,uint8 reg_addr,uint8 p_buffer[],uint8 num)
{   I2cStart();bool a,b,c=0;
    I2cSendbyte(slave_addr);//д
    a=I2cReceiveack();
    if(a==NACK) I2cStop();
    I2cSendbyte(reg_addr);
    b=I2cReceiveack();
    if(b==NACK) I2cStop();
    I2cStart();
    I2cSendbyte(slave_addr+1);//ָ����
    c=I2cReceiveack();/*�˴�Ӧ���ź��в�,�ڶ����ֽڽ����ۻ�����3*/
    //if(c==NACK) I2cStop();/*���Ǵ˴���Ӧ�źų����⣨�߼�����:���ηǸ߷ǵͣ�*/
    uint8 count;
    for(count=0;count<num-1;count++)
    {   p_buffer[count]=I2cReceivebyte();//�ù̼��⺯���ܿ��ƼĴ�����ַ��������֤������ȡ
        I2cSendack(ACK);
    }
    p_buffer[num-1]=I2cReceivebyte();//�˺�����Щ���ܣ��Ĵ�����ַ��������-------��
    I2cSendack(NACK);//���һ�ֽ���ǰ��num-1���ֽڴ���ͬ�������Դ�
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
    I2cSendbyte(slave_addr);//д:ָ�����ʵ�ַ
    a=I2cReceiveack();
    if(a==NACK) I2cStop();//
    I2cSendbyte(reg_addr);
    b=I2cReceiveack();
    if(b==NACK) I2cStop();//
    I2cStart();
    I2cSendbyte(slave_addr+1);//ָ���֣�����
    c=I2cReceiveack();/*�˴�Ӧ���ź��в�,�ڶ����ֽڽ����ۻ�����4*/
    //if(c==NACK) {I2cStop();DebugWriteString("Force to end\r\n");}
    *p_buffer=I2cReceivebyte();
    I2cSendack(ACK);
    *(p_buffer+1)=I2cReceivebyte();
    I2cSendack(NACK);/*MNACK*/
    I2cStop();
    
    //DebugWriteUint8(*p_buffer);DebugWriteUint8(*(p_buffer+1));
    uint16 word;
    word=((uint16)(*p_buffer))<<8|(*(p_buffer+1));//���� 
    /*DebugWriteUint8(a);DebugWriteUint8(b);DebugWriteUint8(c);
    DebugWriteString("\r\n");*/
    return word;
}