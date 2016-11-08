#ifndef BME280_H
#define BME280_H

#include <types.h>

#define CHIPID          0X60
#define RESET           0XB0

#define REG_HUM_LSB     0XFE
#define REG_HUM_MSB     0XFD
#define REG_TEMP_XLSB   0XFC
#define REG_TEMP_LSB    0XFB
#define REG_TEMP_MSB    0XFA
#define REG_PRESS_XLSB  0XF9
#define REG_PRESS_LSB   0XF8
#define REG_PRESS_MSB   0XF7

#define REG_CONFIG      0XF5
#define REG_CTRL_MEAS   0XF4
#define REG_STATUS      0XF3
#define REG_CTRL_HUM    0XF2

#define REG_RESET       0XE0
#define REG_CHIPID      0XD0

#define REG_DIG_T1      0X88
#define REG_DIG_T2      0X8A
#define REG_DIG_T3      0X8C
#define REG_DIG_P1      0X8E
#define REG_DIG_P2      0X90
#define REG_DIG_P3      0X92
#define REG_DIG_P4      0X94
#define REG_DIG_P5      0X96
#define REG_DIG_P6      0X98
#define REG_DIG_P7      0X9A
#define REG_DIG_P8      0X9C
#define REG_DIG_P9      0X9E

#define REG_DIG_H1      0XA1    //uint8
#define REG_DIG_H2      0XE1
#define REG_DIG_H3      0XE3    //uint8
#define REG_DIG_H4      0XE4    //uint12:0XE5[3:0]-dig_H4[3:0]
#define REG_DIG_H5      0XE5    //uint12:0XE5[7:4]-dig_H5[3:0]
#define REG_DIG_H6      0XE7    //uint8

unsigned short dig_T1,dig_P1;
short dig_T2,dig_T3;
short dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;//EEPROM的18个校正系数
uint8 dig_H1,dig_H3;
int8 dig_H6;
short dig_H2,dig_H4,dig_H5;//dig_H4,dig_H5实则为uint12
long ut;
long up,uh;//unsigned 

extern void printInDecimal(long val);
extern void printUnDecimal(long val);
void BME280_Trigger(void);
void BME280_Init(void);
void BME280_Getdata(void);
uint8 BME280_compensation_int32(uint32 *T,uint32 *p,uint32 *h);//int32 ut,uint32 up,uint32 uh
#endif
 