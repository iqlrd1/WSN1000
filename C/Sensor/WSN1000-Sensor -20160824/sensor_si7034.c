/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      Si7034_temperature_sensor.c
 *
 *  DESCRIPTION
 *      This file implements the procedure for communicating with a Si7034
 *      Sensor.
 *
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <types.h>
#include <macros.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
#include "sensor_si7034.h"
#include "app_debug.h"

#ifdef ENABLE_SI7034

uint8 electronic_id_code1[2]={0xfa,0x0f};
uint8 electronic_id_code2[2]={0xfc,0xc9};
uint8 soft_reset_code[1]={0xfe};
uint8 TH_normal_hold_code[2]={0x7c,0xa2};
uint8 TH_normal_no_hold_code[2]={0x78,0x66};
uint8 TH_fast_hold_code[2]={0x64,0x58};
uint8 TH_fast_no_hold_code[2]={0x60,0x9c};
uint8 query_device_no_ack_code[2]={0x80,0x5d};
uint8 query_device_ack_code[2]={0xef,0xc8};
uint8 write_heater_reg_code[1]={0xe6};
uint8 read_heater_reg_code[1]={0xe7};
uint8 fw_revision_code[2]={0x84,0xf1};   //firmware revision
/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      Si7034_Init
 *
 *  DESCRIPTION
 *      This function initialises the temperature sensor Si7034
 *
 *  RETURNS
 *      TRUE if success.
 *
 *---------------------------------------------------------------------------*/
extern bool Si7034_Init(void)
{
    bool  success = FALSE;

        /* Initialise I2C communication. */
        pio_configure();
        
        I2cWriteCommands(Si7034_I2C_ADDRESS,1,soft_reset_code);//software reset
        TimeDelayUSec(2000);
        I2c_stop();
        DebugWriteString("Sensor initialization accomplished.\r\n");/**/        
        success=TRUE;

    return success;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      Si7034_Calibration
 *
 *  DESCRIPTION
 *      This function calibrates the temperature sensor Si7034.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void Si7034_Calibration(void)
{   uint8 crc_t[2]={temp_msb,temp_lsb};
    uint8 crc_h[2]={humi_msb,humi_lsb};
    uint8 crc8_t,crc8_h;
    crc8_t=cal_crc(crc_t,2);//存放crc8校验值
    crc8_h=cal_crc(crc_h,2);
    if((crc8_t=t_checksum))
    {   temperature=(((int16)temp_msb)<<8)|(int16)temp_lsb;
        temperature=-45+175*(long)temperature/65536;//(slope)+(offset)
        DebugWriteUint16(temperature);DebugWriteString("  ");
        printDec(temperature);//DebugWriteString(".");
        
        //printInDecimal((humidity)*10%10);
        DebugWriteString("℃\r\n");
    }
    if((crc8_h=h_checksum))
    {   humidity=((uint16)humi_msb)<<8|(uint16)humi_lsb;
        humidity=(100*(long)humidity/65536);
        DebugWriteUint16(humidity);DebugWriteString("  ");
        printDec(humidity);DebugWriteString("%\r\n");
    }
    //DebugWriteString("Check passed and get result!\r\n");    
}    


/*----------------------------------------------------------------------------*
 *  NAME
 *      Si7034_ReadTemperature
 *
 *  DESCRIPTION
 *      This function read the temperature from sensor Si7034.
 *
 *  RETURNS
 *      TRUE if read succeeds.
 *
 *----------------------------------------------------------------------------*/
extern bool Si7034_ReadTemperature(uint16 *temp,uint16 *humi)
{
    bool  success = FALSE;

    /* Set temperature to Invalid value */
    *temp = INVALID_TEMPERATURE;

        /* Initialise I2C. */
        pio_configure();

        I2cWriteCommands(Si7034_I2C_ADDRESS,2,TH_normal_no_hold_code);
        I2c_start();    //重启Sr
        I2c_sendbyte(Si7034_I2C_ADDRESS+1);  //读第一次
        TimeDelayUSec(9500);     /*SNACK：wait conversion time:9.5ms*/    
        I2c_start();
        I2c_sendbyte(Si7034_I2C_ADDRESS+1);  //读第二次
        I2c_receiveack();/**/
        temp_msb=I2c_receivebyte();  I2c_sendack(ACK);//连续读取6个字节   
        temp_lsb=I2c_receivebyte();  I2c_sendack(ACK);    
        t_checksum=I2c_receivebyte();I2c_sendack(ACK);   
        humi_msb=I2c_receivebyte();  I2c_sendack(ACK);
        humi_lsb=I2c_receivebyte();  I2c_sendack(ACK);
        h_checksum=I2c_receivebyte();I2c_sendack(NACK);//主机终止
        I2c_stop();    //P
        /*DebugWriteString("Get raw data:\r\n");
        DebugWriteUint8(temp_msb);DebugWriteString(" ");
        DebugWriteUint8(temp_lsb);DebugWriteString(" ");
        DebugWriteUint8(t_checksum);DebugWriteString(" \r\n");
        DebugWriteUint8(humi_msb);DebugWriteString(" ");
        DebugWriteUint8(humi_lsb);DebugWriteString(" ");
        DebugWriteUint8(h_checksum);DebugWriteString("\r\n"); */       
        
        Si7034_Calibration();
        //*temp=((temperature&0xff00))>>4|((temperature&0xff)<<4);/*接收格式如此*/
        *temp=temperature;
        *humi=humidity;
        success=TRUE;

    return success;
}

extern uint8 cal_crc(uint8 *ptr, uint8 n)
{	uint8 i;
	uint8 crc8= 0xff;
	while(n--)
	{	for(i=0x80;i!=0;i>>=1)
		{	crc8<<=1;//MSB first
			if((crc8&0x100)!=0) crc8^=0x31;/*CRC8-MAXIUM:g(x)=x8+x5+x4+1*/
			if((*ptr&i)!=0) crc8=crc8^(0x100^0x31);
		}
		ptr++;
	}
	return crc8;
}

extern void printDec(long val)
{   if(val >= 10)
    {   printDec(val/10);
    }
    DebugWriteChar(('0' + (val%10)));//自身反复循环：内层为最高位，外层为最低位
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      Si7034_ReadCallback
 *
 *  DESCRIPTION
 *      This function implements the callback function for read operation.
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/
extern void Si7034_ReadCallback(void)
{
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      Si7034_ShutDown
 *
 *  DESCRIPTION
 *      This function shuts down the temperature sensor Si7034
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/
extern void Si7034_ShutDown(void)
{
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      Si7034_InterruptHandler
 *
 *  DESCRIPTION
 *      This function handles the interrupt from temperature sensor Si7034
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/
extern void Si7034_InterruptHandler(void)
{
}

#endif /* TEMPERATURE_SENSOR_Si7034 */

