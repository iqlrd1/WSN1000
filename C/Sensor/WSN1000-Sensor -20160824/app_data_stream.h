/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      app_data_stream.h
 *
 *  DESCRIPTION
 *      Header definitions for application data stream implementation
 *
 *****************************************************************************/

#ifndef __APP_DATA_STREAM_H__
#define __APP_DATA_STREAM_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <bt_event_types.h>
#include <pio.h>
/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include "user_config.h"
#include "iot_hw.h"
/*============================================================================*
 *  CSRmesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <stream_model.h>
#include <debug.h>
#include <timer.h>

#ifdef ENABLE_BME280
    #include "sensor_bme280.h"
#endif

#ifdef ENABLE_DATA_MODEL

//20160520
bool streamCFM;
timer_id register_tid;
/*20160815*/
uint16 ledFlashtid;
/*20160819*/
bool ledFlashFlag;

/*20160705*/
#define SENSOR_STATUS_INIT          0X0
#define SENSOR_STATUS_INIT_PRESSED  0X1
#define SENSOR_STATUS_REGISTER      0X2
#define SENSOR_STATUS_NORMAL        0X3
#define SENSOR_STATUS_ERROR         0X4
uint16 currentSensorStatus;
/*============================================================================*
 *  Public Definitions
 *============================================================================*/

/* Application protocol codes used to exchange device data over the
 * data stream model
 */
typedef enum
{
    CSR_DEVICE_INFO_REQ = 0x01,
    CSR_DEVICE_INFO_RSP = 0x02,
    CSR_DEVICE_INFO_SET = 0x03,
    CSR_DEVICE_INFO_RESET = 0x04,
    CUSTOM_REQ_DEVID      = 0X50, //custom code start from 0x50
    CUSTOM_REC_DEVID      = 0X51,                       
    CUSTOM_REQ_DATA       = 0X52,
    CUSTOM_SET_MESH_PERIOD= 0X53
}APP_DATA_STREAM_CODE_T;


//20160602
#define CANNOT_REC_DEVID_CMD    0x70
#define SEND_DEVID_UNSUCCESS    0x71

/*new coe20160422*/
bool datasentflag;
timer_id tx_id;/*new code20160512*/
/*20160819*/
bool flagCustomReqDataRec;
/*20160823*/
uint8 meshONNotSleepTime,meshOFFSleepTime;
uint16 deviceid_src;

extern void warning(void);
/*20160602*/
extern void errorHandler(uint16 error_code);
void turnONMeshTimerHandler(timer_id tid);/*20160810*/
/*20160826*/
void sendDataStreamWithCFM(uint16 dat_len,uint16 devid_dest,uint16 counter);
/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/
/* Initialises the application data stream protocol */
extern void AppDataStreamInit(uint16 *group_id_list, uint16 num_groups);

/* This function handles the CSR_MESH_DATA_STREAM_FLUSH message.*/
extern void handleCSRmeshDataStreamFlushInd(CSR_MESH_STREAM_EVENT_T *p_event);

/* This function handles the CSR_MESH_DATA_BLOCK_IND message */
extern void handleCSRmeshDataBlockInd(CSR_MESH_STREAM_EVENT_T *p_event);

/* This function handles the CSR_MESH_DATA_STREAM_DATA_IND message */
extern void handleCSRmeshDataStreamDataInd(CSR_MESH_STREAM_EVENT_T *p_event);

/* This function handles the CSR_MESH_DATA_STREAM_SEND_CFM message */
extern void handleCSRmeshDataStreamSendCfm(CSR_MESH_STREAM_EVENT_T *p_event);

#endif /* ENABLE_DATA_MODEL */

#endif /* __APP_DATA_STREAM_H__ */

