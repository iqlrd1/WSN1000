#include "sensor_bme280.h"
#include <spi.h>
#include <debug.h>
#include <types.h>

extern void printInDecimal(long val)
{   if(val<0)
    {   DebugWriteChar('-');
        val=-val;
    }
    if(val >= 10)
    {   printInDecimal(val/10);
    }
    DebugWriteChar(('0' + (val%10)));/*自身反复循环嵌套：内层为最高位，外层为最低位*/
}

extern void printUnDecimal(long val)
{   
    if(val >= 10)
    {   printInDecimal(val/10);
    }
    DebugWriteChar(('0' + (val%10)));/*自身反复循环嵌套：内层为最高位，外层为最低位*/
}

void BME280_Trigger(void)
{
    bool ret_status;
    ret_status=SpiInit(10,11,4,9);
    //MOSI MISO SLK CS#

    /*Weather monitoring*/
    SpiWriteRegister(REG_CONFIG &0x7f,0x00);//t_sb:0.5ms,IIR:off,SPI3W_en=0
    SpiWriteRegister(REG_CTRL_MEAS &0x7f,0x26);//osr_t:x1,osr_p:x1,mode:forced
    SpiWriteRegister(REG_CTRL_HUM &0x7f,0x01); //osr_h:x1 
}

void BME280_Init(void)
{   
    uint8 chip_id;
    uint8 coeff[33];
    bool ret_status;
    ret_status=SpiInit(10,11,4,9);
    //MOSI MISO SLK CS#
    
    SpiWriteRegister(REG_RESET &0x7f,RESET);//R/W#=0
    /*Weather monitoring*/
    SpiWriteRegister(REG_CONFIG &0x7f,0x00);//t_sb:0.5ms,IIR:off,SPI3W_en=0
    SpiWriteRegister(REG_CTRL_MEAS &0x7f,0x26);//osr_t:x1,osr_p:x1,mode:forced
    SpiWriteRegister(REG_CTRL_HUM &0x7f,0x01); //osr_h:x1
 
    chip_id=SpiReadRegister(REG_CHIPID);

    if(chip_id==CHIPID) 
    {   
        DebugWriteUint8(chip_id);DebugWriteString("<-Get chip id\r\n");
        SpiReadRegisterBurst(REG_DIG_T1,coeff,24,FALSE);
        dig_T1=coeff[0]|(((uint16)coeff[1])<<8);//LSBFIRST
        dig_T2=coeff[2]|(((uint16)coeff[3])<<8);
        dig_T3=coeff[4]|(((uint16)coeff[5])<<8);
        dig_P1=coeff[6]|(((uint16)coeff[7])<<8);
        dig_P2=coeff[8]|(((uint16)coeff[9])<<8);
        dig_P3=coeff[10]|(((uint16)coeff[11])<<8);
        dig_P4=coeff[12]|(((uint16)coeff[13])<<8);
        dig_P5=coeff[14]|(((uint16)coeff[15])<<8);
        dig_P6=coeff[16]|(((uint16)coeff[17])<<8);
        dig_P7=coeff[18]|(((uint16)coeff[19])<<8);
        dig_P8=coeff[20]|(((uint16)coeff[21])<<8);
        dig_P9=coeff[22]|(((uint16)coeff[23])<<8);

        dig_H1=SpiReadRegister(REG_DIG_H1);               //uint8
        dig_H3=SpiReadRegister(REG_DIG_H3);
        dig_H6=SpiReadRegister(REG_DIG_H6);
        
        SpiReadRegisterBurst(REG_DIG_H2,coeff+24,2,FALSE);//uint16
        dig_H2=coeff[24]|(((uint16)coeff[25])<<8);

        coeff[26]=SpiReadRegister(REG_DIG_H4);            //uint12
        SpiReadRegisterBurst(REG_DIG_H5,coeff+27,2,FALSE);
        dig_H4=(((uint16)coeff[26])<<4)|(coeff[27]&0x0f);//MSBFIRST
        //dig_H4=(((uint16)coeff[26])<<8|(coeff[27]<<4))>>4;
        dig_H5=(((uint16)coeff[28])<<4)|((coeff[27]>>4)&0x0f);//LSBFIRST
    }
    printInDecimal(dig_T1);DebugWriteString("←dig_T1 ");
    DebugWriteUint16(dig_T1);DebugWriteString("\r\n");
    printInDecimal(dig_T2);DebugWriteString("←dig_T2 ");
    DebugWriteUint16(dig_T2);DebugWriteString("\r\n");
    printInDecimal(dig_T3);DebugWriteString("←dig_T3 ");
    DebugWriteUint16(dig_T3);DebugWriteString("\r\n");
    
    printInDecimal(dig_P1);DebugWriteString("←dig_P1 ");
    DebugWriteUint16(dig_P1);DebugWriteString("\r\n");    
    printInDecimal(dig_P2);DebugWriteString(" ←dig_P2 ");
    DebugWriteUint16(dig_P2);DebugWriteString("\r\n"); 
    printInDecimal(dig_P3);DebugWriteString("←dig_P3 ");
    DebugWriteUint16(dig_P3);DebugWriteString("\r\n"); 
    printInDecimal(dig_P4);DebugWriteString(" ← dig_P4 ");
    DebugWriteUint16(dig_P4);DebugWriteString("\r\n"); 
    printInDecimal(dig_P5);DebugWriteString(" ←dig_P5 ");
    DebugWriteUint16(dig_P5);DebugWriteString("\r\n"); 
    printInDecimal(dig_P6);DebugWriteString("←dig_P6 ");
    DebugWriteUint16(dig_P6);DebugWriteString("\r\n");
    printInDecimal(dig_P7);DebugWriteString("←dig_P7 ");
    DebugWriteUint16(dig_P7);DebugWriteString("\r\n");
    printInDecimal(dig_P8);DebugWriteString(" ←dig_P8 ");
    DebugWriteUint16(dig_P8);DebugWriteString("\r\n");
    printInDecimal(dig_P9);DebugWriteString(" ←dig_P9 ");    
    DebugWriteUint16(dig_P9);DebugWriteString("\r\n"); 
    
    printInDecimal(dig_H1);DebugWriteString(" ←dig_H1 ");
    DebugWriteUint16(dig_H1);DebugWriteString("\r\n"); 
    printInDecimal(dig_H2);DebugWriteString("←dig_H2 ");
    DebugWriteUint16(dig_H2);DebugWriteString("\r\n"); 
    printInDecimal(dig_H3);DebugWriteString(" ←dig_H3 ");
    DebugWriteUint16(dig_H3);DebugWriteString("\r\n");     
    printInDecimal(dig_H4);DebugWriteString("←dig_H4 ");
    DebugWriteUint16(dig_H4);DebugWriteString("\r\n"); 
    printInDecimal(dig_H5);DebugWriteString(" ←dig_H5 ");
    DebugWriteUint16(dig_H5);DebugWriteString("\r\n"); 
    printInDecimal(dig_H6);DebugWriteString("← dig_H6 ");
    DebugWriteUint16(dig_H6);DebugWriteString("\r\n");/**/
}

void BME280_Getdata()
{   
    uint8 dat[8]={0,0,0,0,0,0,0,0};uint16 i;
    SpiReadRegisterBurst(REG_PRESS_MSB,dat,8,FALSE);//MSBFIRST
    //up=((((uint32)dat[0])<<12)|(((uint32)dat[1])<<4)|(dat[2]>>4));
    up=((((uint32)dat[0])<<16)|(((uint32)dat[1])<<8)|dat[2])>>4;
    //ut=((((uint32)dat[3])<<12)|(((uint32)dat[4])<<4)|(dat[5]>>4));
    ut=(int32)(((((uint32)dat[3])<<16)|(((uint32)dat[4])<<8)|dat[5])>>4);
    uh=((((uint16)dat[6])<<8)|dat[7]);/**/
    for(i=0;i<8;i++) {DebugWriteUint16(dat[i]);DebugWriteString(" ");}
    DebugWriteString("\r\n");
    DebugWriteUint32(ut);DebugWriteString("←ut ");printInDecimal(ut);
    DebugWriteString("\r\n");
    DebugWriteUint32(up);DebugWriteString(" ←up ");printInDecimal(up);
    DebugWriteString("\r\n");
    DebugWriteUint32(uh);DebugWriteString("←uh ");printInDecimal(uh);
    DebugWriteString("\r\n");
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
uint8 BME280_compensation_int32(uint32 *T,uint32 *p,uint32 *h)
{
	int32 var1, var2, v_x1_u32r, t_fine; // T,
    var1=var2=0;//uint32 h;
    
    /*long ut=0;
    long up=0,uh=0;*/
    BME280_Getdata();
    
    var1 = ( ((ut>>3)-((int32)dig_T1<<1)) * ((int32)dig_T2) ) >>11;
    var2 = ( ((((ut>>4)-((int32)dig_T1)) * ((ut>>4)-((int32)dig_T1)))>>12) * \
           ((int32)dig_T3) )>>14;
	t_fine = var1 + var2;
	*T = (t_fine * 5 + 128) >> 8;
	DebugWriteString("Maintain final temperature data:");
    DebugWriteUint32((unsigned long)*T);DebugWriteString(" ");
    printInDecimal(*T/100);DebugWriteString(".");
    printUnDecimal(*T%100);DebugWriteString("℃\r\n");
    
    // Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
    var1 = (((int32)t_fine)>>1) - (int32)64000;
    var2 = (((var1>>2)*(var1>>2))>>11) * ((int32)dig_P6);
	var2 = var2 + ((var1*((int32)dig_P5))<<1);
	var2 = (var2>>2)+(((int32)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32)dig_P1))>>15);
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	*p = (((uint32)(((int32)1048576)-up)-(var2>>12)))*3125;
	if (*p < 0x80000000)
	{
		*p = (*p << 1) / ((uint32)var1);
	}
	else
	{
		*p = (*p / (uint32)var1) * 2;
	}
	var1 = (((int32)dig_P9) * ((int32)(((*p>>3) * (*p>>3))>>13)))>>12;
	var2 = (((int32)(*p>>2)) * ((int32)dig_P8))>>13;
	*p = (uint32)((int32)*p + ((var1 + var2 + dig_P7) >> 4));
    *p= (unsigned long)*p;
	DebugWriteString("Maintain final pressure data:   ");
    DebugWriteUint32(*p);DebugWriteString(" ");
    printInDecimal(*p);DebugWriteString("Pa\r\n");
    
    // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46.333 %RH
    v_x1_u32r =t_fine-((int32)76800);
    v_x1_u32r = (( ((uh<<14)-(((int32)dig_H4)<<20)-(((int32)dig_H5)*v_x1_u32r))+\
                ((int32)16384) )>>15) * (((((((v_x1_u32r*((int32)dig_H6))>>10)*(((v_x1_u32r*((int32)dig_H3))>>11)+((int32)32768)))>>10)+\
                (int32)2097152)*((int32)dig_H2)+8192)>>14);
                
    v_x1_u32r = (v_x1_u32r-(((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*((int32)dig_H1))>>4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    
    *h=(uint32)(v_x1_u32r>>12);
    DebugWriteString("Maintain final humidity data:   ");
    DebugWriteUint32(*h);DebugWriteString(" ");
    printInDecimal(*h/1024);DebugWriteString(".");
    printInDecimal((*h%1024)*1000/1024);DebugWriteString("%\r\n");
    return 1;
}
