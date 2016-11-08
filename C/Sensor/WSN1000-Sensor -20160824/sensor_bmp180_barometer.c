/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      BMP180_temperature1_sensor.c
 *
 *  DESCRIPTION
 *      This file implements the procedure for communicating with a BMP180
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
#include "stts751_temperature_sensor.h"
#include "sensor_bmp180_barometer.h"
#include "iic.h"
#include "app_debug.h"

#ifdef ENABLE_BMP180//

static int16 ut;
static uint32 up;//存放原始数据
/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/
/*----------------------------------------------------------------------------*
 *  NAME
 *      BMP180_Init
 *
 *  DESCRIPTION
 *      This function initialises the temperature1 sensor BMP180
 *
 *  RETURNS
 *      TRUE if success.
 *
 *---------------------------------------------------------------------------*/
extern bool BMP180_Init(void)
{   uint8 manu_id=0;//uint8 b2_msb=0;
    uint8 coeff[22];
    bool  success = FALSE;
 
        pioConfigure();TimeDelayUSec(20);/*待其配置完成*/
        
        I2cReadRegister(BMP180_I2C_ADDRESS,REG_ID,&manu_id);
        DebugWriteUint8(manu_id);DebugWriteString(" manufactor ID\r\n");
        if(manu_id==BMP180_MANU_ID)
        {              
            ac1=I2cReadWord(BMP180_I2C_ADDRESS,REG_AC1,coeff);//无符号→有符号
            DebugWriteUint16(ac1);DebugWriteString("←ac1  ");
            
            ac2=I2cReadWord(BMP180_I2C_ADDRESS,REG_AC2,coeff+2);          
            printDecimal(ac2);DebugWriteString("←ac2 |");
            DebugWriteUint16(ac2);DebugWriteString("←ac2  ");
            
            ac3=I2cReadWord(BMP180_I2C_ADDRESS,REG_AC3,coeff+4);
            printDecimal(ac3);DebugWriteString("←ac3 |");
            DebugWriteUint16(ac3);DebugWriteString("←ac3  ");
            
            ac4=I2cReadWord(BMP180_I2C_ADDRESS,REG_AC4,coeff+6);           
            DebugWriteUint16(ac4);DebugWriteString("←ac4  ");
            
            ac5=I2cReadWord(BMP180_I2C_ADDRESS,REG_AC5,coeff+8);
            DebugWriteUint16(ac5);DebugWriteString("←ac5  ");
            
            ac6=I2cReadWord(BMP180_I2C_ADDRESS,REG_AC6,coeff+10);
            DebugWriteUint16(ac6);DebugWriteString("←ac6\r\n");
            
            b1 =I2cReadWord(BMP180_I2C_ADDRESS,REG_B1,coeff+12);
            DebugWriteUint16(b1);DebugWriteString("←b1  ");/**/
            
            b2 =I2cReadWord(BMP180_I2C_ADDRESS,REG_B2,coeff+14);
            DebugWriteUint16(b2);DebugWriteString("←b2  ");
            
            mb =I2cReadWord(BMP180_I2C_ADDRESS,REG_MB,coeff+16);
            printDecimal(mb);DebugWriteString("←mb |");
            DebugWriteUint16(mb);DebugWriteString("←mb  ");
            
            mc =I2cReadWord(BMP180_I2C_ADDRESS,REG_MC,coeff+18);
            printDecimal(mc);DebugWriteString("←mc |");
            DebugWriteUint16(mc);DebugWriteString("←mc  ");
            
            md =I2cReadWord(BMP180_I2C_ADDRESS,REG_MD,coeff+20);
            //if((md&0x8000)==0x8000) md=(~md+1)*(-1);
            DebugWriteUint16(md);DebugWriteString("←md\r\n");/**/
            
            //I2cReadRegister(BMP180_I2C_ADDRESS,REG_B2,coeff);
            //printDecimal(b2_msb);DebugWriteString("←b2_msb  ");
        }        
        success=TRUE;

    return success;
}  

/*----------------------------------------------------------------------------*
 *  NAME
 *      BMP180_Readtemperature1
 *
 *  DESCRIPTION
 *      This function read the temperature1 from sensor BMP180.
 *
 *  RETURNS
 *      TRUE if read succeeds.
 *
 *----------------------------------------------------------------------------*/
extern bool BMP180_ReadTemperature(uint16 *temp,uint32 *pres)
{   uint8 tempdata[2],pressdata[3];   
    
    //uint8 i;
    bool  success = FALSE;

    /* Set temperature1 to Invalid value */
    *temp = INVALID_TEMPERATURE;

        pioConfigure();

        I2cWriteRegister(BMP180_I2C_ADDRESS,REG_CTRL_MEAS,0x2E);//启动温度ADC转换
        TimeDelayUSec(4500);//等待转换完成：4.5ms
        I2cReadRegisters(BMP180_I2C_ADDRESS,REG_OUT_MSB,tempdata,2);//高位先出
        ut=tempdata[1] | ((uint16)(tempdata[0]))<<8;
        /*DebugWriteString("   Get ut data:");DebugWriteUint32(ut);
        for(i=0;i<2;i++) {DebugWriteString(" ");
            DebugWriteUint8(tempdata[i]);DebugWriteString(" ");}
        DebugWriteString("\r\n");*/    
        
        I2cWriteRegister(BMP180_I2C_ADDRESS,REG_CTRL_MEAS,0x34);//启动气压ADC转换
        TimeDelayUSec(4500);//等待转换完成：4.5ms
        I2cReadRegisters(BMP180_I2C_ADDRESS,REG_OUT_MSB,pressdata,3);//高位先出
        up=(uint32)(((pressdata[2]) | ((uint32)(pressdata[1]))<<8 \
             |(((uint32)(pressdata[0]))<<16))>>(8-OSS));
        /*DebugWriteString(" Get up data:");DebugWriteUint32(up);
        for(i=0;i<3;i++) {DebugWriteString(" ");
            DebugWriteUint8(pressdata[i]);DebugWriteString(" ");}
        DebugWriteString("\r\n");*/
        
        BMP180_Calibration();
        //*temp=((temperature1&0xff00))>>4|((temperature1&0xff)<<4);/*接收格式如此*/
        /* Release the I2C bus */
        *temp=temperature1;
        *pres=pressure;

        success=TRUE;

    return success;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BMP180_Calibration
 *
 *  DESCRIPTION
 *      This function calibrates the temperature1 sensor BMP180.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void BMP180_Calibration(void)
{   long x1,x2,x3,b3,b5,b6;
    unsigned long b4,b7,p;    

    x1 = ((long)ut -(long)ac6)*(long)ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + (long)md);
	b5 = x1 + x2;
	temperature1 = (b5 + 8) >> 4;
    DebugWriteString("Maintain final temperature1 data:");
    DebugWriteUint16((uint16)temperature1);DebugWriteString(" ");
    printDecimal(temperature1/10);DebugWriteString(".");
    printDecimal(temperature1%10);DebugWriteString("℃\r\n");
    //temperature1/=10;//此处除以10就丢失掉小数部分
    
    b6 = b5 - 4000;
	x1 = ((long)b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	b3 = ((((long)ac1 * 4 + x3)<<OSS) + 2)/4;
	x1 = (long)ac3 * b6 >> 13;
	x2 = ((long)b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long) (x3 + 32768)) >> 15;
	b7 = ((unsigned long) up - b3) * (50000 >> OSS);
	if( b7 < 0x80000000) p = (b7 * 2) / b4 ;
    else  p = (b7 / b4) * 2;
	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	pressure = p + ((x1 + x2 + 3791) >> 4);
    DebugWriteString("Maintain final pressure data:");
    DebugWriteUint32(pressure);DebugWriteString(" "); 
    printDecimal(pressure);DebugWriteString("Pa\r\n");  
}  

extern void printDecimal(long val)
{   if(val < 0)
    {   val=-val;DebugWriteString("-");
    }
    if(val >= 10)
    {   printDecimal(val/10);
    }
    DebugWriteChar(('0' + (val%10)));//自身反复循环：内层为最高位，外层为最低位
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BMP180_ReadCallback
 *
 *  DESCRIPTION
 *      This function implements the callback function for read operation.
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/
extern void BMP180_ReadCallback(void)
{
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BMP180_ShutDown
 *
 *  DESCRIPTION
 *      This function shuts down the temperature1 sensor BMP180
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/
extern void BMP180_ShutDown(void)
{
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      BMP180_InterruptHandler
 *
 *  DESCRIPTION
 *      This function handles the interrupt from temperature1 sensor BMP180
 *
 *  RETURNS
 *      Nothing
 *
 *---------------------------------------------------------------------------*/
extern void BMP180_InterruptHandler(void)
{
}

#endif /* temperature1_SENSOR_BMP180 */

