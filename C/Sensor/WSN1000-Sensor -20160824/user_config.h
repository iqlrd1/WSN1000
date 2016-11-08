/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 * FILE
 *      user_config.h
 *
 * DESCRIPTION
 *      This file contains definitions which will enable customization of the
 *      application.
 *
 ******************************************************************************/

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

/*=============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Application version */
#define APP_MAJOR_VERSION   (1)
#define APP_MINOR_VERSION   (3)

/* Application NVM version. This version is used to keep the compatibility of
 * NVM contents with the application version. This value needs to be modified
 * only if the new version of the application has a different NVM structure
 * than the previous version (such as number of groups supported) that can
 * shift the offsets of the currently stored parameters.
 * If the application NVM version has changed, it could still read the values
 * from the old Offsets and store into new offsets.
 * This application currently erases all the NVM values if the NVM version has
 * changed.
 */
#define APP_NVM_VERSION                (2)

#define CSR_MESH_SENSOR_PID            (0x1062)

/* Vendor ID for CSR */
#define APP_VENDOR_ID                  (0x0A12)

/* Product ID. */
#define APP_PRODUCT_ID                 (CSR_MESH_SENSOR_PID)

/* Version Number. */
#define APP_VERSION            (((uint32)(APP_PRODUCT_ID   & 0xFF) << 24) | \
                               ((uint32)(APP_NVM_VERSION   & 0xFF) << 16) | \
                               ((uint32)(APP_MAJOR_VERSION & 0xFF) << 8)  | \
                               ((uint32)(APP_MINOR_VERSION & 0xFF)))

/* The IOT board has 2 bottons to control the desired temperature of one group.
 * Hence setting the number of sensor model groups supported as 1.
 */
#define NUM_SENSOR_ACTUATOR_MODEL_GROUPS  (4)

/* Number of attention model groups. */
#define NUM_ATTENTION_MODEL_GROUPS     (4)

/* Number of data model groups */
#define NUM_DATA_MODEL_GROUPS          (4)

/* Macro to enable GATT OTA SERVICE */
/* Over-the-Air Update is supported only on EEPROM */
#ifdef NVM_TYPE_EEPROM

//#define ENABLE_GATT_OTA_SERVICE

/* Enable firmware model support */
//#define ENABLE_FIRMWARE_MODEL

#endif /* NVM_TYPE_EEPROM */

/* Enable Static Random Address. */
/* #define USE_STATIC_RANDOM_ADDRESS */

/* Enables Authorization Code on Device. */
/* #define USE_AUTHORIZATION_CODE */

/* STTS751 Temperature Sensor. */
#define TEMPERATURE_SENSOR_STTS751

/* Enable battery model support */
#define ENABLE_BATTERY_MODEL /* *///20160704

/* Enable data model support */
#define ENABLE_DATA_MODEL

/* Enable application debug logging on UART */
#define DEBUG_ENABLE

/* Temperature Sensor Parameters. */
/* Temperature Sensor Sampling Interval */
#define TEMPERATURE_SAMPLING_INTERVAL  (15 * SECOND)

/* Temperature Controller. */
#define ENABLE_TEMPERATURE_CONTROLLER

/* Enable the Acknowledge mode */
/* #define ENABLE_ACK_MODE */

/* Default repeat interval in seconds. This enables the sensor periodically
 * sending the temperature every repeat interval. Value range 0-255. The
 * interval within 1-30 seconds is considered to be min of 30 due to the 
 * retransmissions.
 */
#define DEFAULT_REPEAT_INTERVAL        (0)

/* Default rx duty cycle.in percentage */
#define DEFAULT_RX_DUTY_CYCLE          (2)
#define CUSTOM_RX_DUTY_CYCLE            70
#define OFF_RX_DUTY_CYCLE               1

/* Rx duty cycle in percentage when device set to active scan mode. The device  
 * is present in this mode before grouping and on attention or data stream in 
 * progress.
 */
#define HIGH_RX_DUTY_CYCLE             (100)
/*new code20160512*///自定义开始数据流指令
#define REQ_ID_CMD      0XF1   //heater->sensor
//#define ID_CMD_CFM      0xF2   //heater<-sensor
//#define SEND_ID         0XF3   //heater<-sensor
//#define SEND_ID_CFM     0XF4   //heater->sensor
#define REGISTERED      0XF5   //heater->sensor
//#define REGISTED_CFM    0XF6   //heater<-sensor
/* Number of msgs to be added in the transmit queue in one shot.
 * supported values 1-5
 */
#define TRANSMIT_MSG_DENSITY           (4)

/* Number of msgs to be retransmitted per temp change */
//#define NUM_OF_RETRANSMISSIONS         (60)
/*test code*/
#define NUM_OF_RETRANSMISSIONS         (60)


/* Temperature change in 1/32 kelvin units. If temp changes more than this the
 * sensor would write the temp change onto the group.
 */
#define TEMPERATURE_CHANGE_TOLERANCE   (32)

/*test code 20160418*/
#define PRESET_DEVID

/*test code 20160421*/
//#define ENABLE_BMP180
/*test code 20160422*/
//#define ENABLE_SI7034
/*20160704*/
#define ENABLE_BME280

#endif /* __USER_CONFIG_H__ */
