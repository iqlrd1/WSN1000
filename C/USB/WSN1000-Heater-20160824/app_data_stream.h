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
#include <debug.h>
#include <nvm.h>
#include <pio.h>
/*============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include "user_config.h"
#include "csr_mesh_heater.h"
#include "iot_hw.h"
#include "uartio.h"
/*============================================================================*
 *  CSRmesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <stream_model.h>

#ifdef ENABLE_DATA_MODEL
#define PENDING_INPUT 1
#define FINISHED_INPUT 0

/*20160705*/
#define HEATER_STATUS_INIT  0X0
#define HEATER_STATUS_SEARCH    0X1
#define HEATER_STATUS_REGISTER  0X2
#define HEATER_STATUS_NORMAL    0X3
uint16 currentHeaterStatus;
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
/*20160531*/
typedef enum
{   heater_status_init=0X0000,
    heater_status_search,
    heater_status_register,
    heater_status_normal,
}HEATER_STATUS_T;
HEATER_STATUS_T heater_status;

/*new code20160512*/
//static bool regist_cfm=0;
//static uint16 cfm_case=0;
/*20160613*/
timer_id receive_delay_tid;
/*20160823*/
uint16 meshONNotSleepTimeMin,meshOFFSleepTimeMin;
uint16 clockMeshON_tid,clockMeshOFF_tid;

void streamSendRetryTimer1(timer_id tid);
void sendNextPacket1(void);
extern void sendCustomCmd(uint16 intervalms,uint16 retryCounter,uint8 customData[],\
                          uint16 dataLength,uint16 customCode, uint16 targetDevid);/*20160607*/
void sendCustomCmd1(uint16 customCmd, uint16 targetDevid);/*20160727*/
/*20160601*/
extern void recDevIDTimerHandler(timer_id tid);
extern void warning(void);
/*20160612*/
bool storeDevIDtoNVM(uint16 devid);
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

