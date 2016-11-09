/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *    app_data_stream.c
 *
 * DESCRIPTION
 *    This file implements a simple protocol over the data model to exchange
 *    device information.
 *    The protocol:
 *       | CODE | LEN (1 or 2 Octets| Data (LEN Octets)|
 *       CODE Defined by APP_DATA_STREAM_CODE_T
 *       LEN - if MS Bit of First octet is 1, then len is 2 octets
 *       if(data[0] & 0x80) LEN = data[0]
 *
 ******************************************************************************/

/*=============================================================================*
 *  SDK Header Files
 *============================================================================*/
#include <gatt.h>
#include <timer.h>
#include <mem.h>
#include "app_debug.h"

/*============================================================================*
 *  CSR Mesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <stream_model.h>

/*=============================================================================*
 *  Local Header Files
*============================================================================*/
#include "app_data_stream.h"
#include "csr_mesh_heater.h"
#ifdef  ENABLE_DATA_MODEL
/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

#define DEVICE_INFO_STRING          "CSRmesh Heater application\r\n" \
                                    "Supported Models:\r\n" \
                                    "  Sensor Model\r\n" \
                                    "  Bearer Model\r\n" \
                                    "  Attention Model\r\n" \
                                    "  Data Model"

/* Data stream send retry wait time */
#define STREAM_SEND_RETRY_TIME            (500 * MILLISECOND)

/* Data stream received timeout value */
#define RX_STREAM_TIMEOUT                 (5 * SECOND) //20160520

/* Max number of retries */
#define MAX_SEND_RETRIES                  (30)

/*=============================================================================*
 *  Private Data
 *============================================================================*/
/* String to give a brief description of the application */
static uint8 device_info[256];

/* Device info length */
static uint8 device_info_length;

/* Stream bytes sent tracker */
static uint16 tx_stream_offset = 0;

/* Stream send retry timer */
static timer_id stream_send_retry_tid = TIMER_INVALID;

/* Stream send retry counter */
static uint16 stream_send_retry_count = 0;

/* Current Rx Stream offset */
static uint16 rx_stream_offset = 0;

/* Rx Stream status flag */
static bool rx_stream_in_progress = FALSE;

/* Rx stream timeout tid */
static timer_id rx_stream_timeout_tid;

static APP_DATA_STREAM_CODE_T current_stream_code;

bool cmdCFM=FALSE;
//bool cmdAdvert=FALSE;/*20160531*/
uint8 id_cmd[3];
uint16 sendCustomCmdRetryCounter=0;
uint16 timecounter=0;
uint16 sendCustomCmdIntervalms;/*20160607*/
/*20160824*/
uint16 devid_target;
/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/
static void streamSendRetryTimer(timer_id tid);
static void sendNextPacket(void);

/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/
 void warning()
{   PioConfigPWM(LED_PWM_RED, pio_pwm_mode_push_pull,
                 50, 0, 100, 0, 50, 100, 0U);//LED blink warning
}  
/*----------------------------------------------------------------------------*
 *  NAME
 *      streamSendRetryTimer
 *
 *  DESCRIPTION
 *      Timer handler to retry sending next packet
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void streamSendRetryTimer(timer_id tid)
{
    if( tid == stream_send_retry_tid )
    {
        stream_send_retry_tid = TIMER_INVALID;
        stream_send_retry_count++;
        if( stream_send_retry_count < MAX_SEND_RETRIES )
        {
            StreamResendLastData();
            stream_send_retry_tid =  TimerCreate(STREAM_SEND_RETRY_TIME, TRUE,
                                                          streamSendRetryTimer);
        }
        else
        {
            stream_send_retry_count = 0;
            StreamFlush();
            /* Set the mesh scan back to low duty cycle if the device is already 
             * configured.
             */
            EnableHighDutyScanMode(TRUE); //20160808 change from false to true
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      rxStreamTimeoutHandler
 *
 *  DESCRIPTION
 *      Timer handler to handle rx stream timeout
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

static void rxStreamTimeoutHandler(timer_id tid)
{    
    if( tid == rx_stream_timeout_tid )
    {
        TimerDelete(rx_stream_timeout_tid);
        /* Reset the stream */
        rx_stream_timeout_tid = TIMER_INVALID;
        rx_stream_in_progress = FALSE;
        StreamReset();
        /* Set the mesh scan back to low duty cycle if the device is already 
         * configured.
         */
        EnableHighDutyScanMode(TRUE); //20160808 change from false to true
    }   
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      sendNextPacket
 *
 *  DESCRIPTION
 *      Forms a stream data packet with the current counter and sends it to
 *      the stream receiver
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void sendNextPacket(void)
{
    uint16 data_pending, len;

    /* Stop retry timer */
    stream_send_retry_count = 0;
    TimerDelete(stream_send_retry_tid);
    stream_send_retry_tid = TIMER_INVALID;

    data_pending = device_info_length+2 - tx_stream_offset;

    if( data_pending )
    {
        len = (data_pending > STREAM_DATA_BLOCK_SIZE_MAX)? 
                                    STREAM_DATA_BLOCK_SIZE_MAX:data_pending;

        /* Send the next packet */
        StreamSendData(&device_info[tx_stream_offset], len);
        tx_stream_offset += len;

        stream_send_retry_tid = TimerCreate(100 * MILLISECOND, TRUE,
                                                       streamSendRetryTimer);
    }
    else
    {
        /* Send flush to indicate end of stream */
        StreamFlush();
        /* Set the mesh scan back to low duty cycle if the device is already 
         * configured.
         */
        EnableHighDutyScanMode(TRUE); //20160808 change from false to true
    }
}
/*20160520*/
void sendNextPacket1(void)  
{   if(inputNumberOfSensor!=0) saveInputSensorNumberToNvm();
    id_cmd[0]=CUSTOM_REC_DEVID;
    id_cmd[1]=REQ_ID_CMD;

    /* Stop retry timer */
    stream_send_retry_count = 0;
    TimerDelete(stream_send_retry_tid);
    stream_send_retry_tid = TIMER_INVALID;
    StreamStartSender(0x0000);//broadcast 
    StreamSendData(id_cmd,2);
    /*20160524*/
    /*DebugWriteString(" cmdCFM1:");DebugWriteUint8(cmdCFM);*/
    cmdCFM=FALSE;//
    stream_send_retry_tid = TimerCreate(100 * MILLISECOND, TRUE,
                                        streamSendRetryTimer1);
}

void sendCustomCmd(uint16 intervalms,uint16 retryCounter,uint8 customData[], \
                   uint16 dataLength,uint16 customCode, uint16 targetDevid)
{   
    EnableHighDutyScanMode(TRUE);//20160819
    sendCustomCmdRetryCounter=retryCounter;
    sendCustomCmdIntervalms=intervalms;
    devid_target=targetDevid;
    
    id_cmd[0]=customCode; //send cmd
    uint16 i;
    for(i=0;i<dataLength;i++)        
        id_cmd[i+1]=customData[i];
   
    /* Stop retry timer */
    stream_send_retry_count = 0;
    stream_send_retry_tid = TIMER_INVALID;
    //StreamStartSender(targetDevid);
    //StreamSendData(id_cmd,2);
    StreamSendDataBlock(devid_target,id_cmd,dataLength+1);//20160824

    cmdCFM=FALSE;
    
    stream_send_retry_tid = 
            TimerCreate(sendCustomCmdIntervalms * MILLISECOND,
                        TRUE, streamSendRetryTimer1);
}

/*20160520*/
void streamSendRetryTimer1(timer_id tid)
{     
    TimerDelete(stream_send_retry_tid);
    stream_send_retry_tid = TIMER_INVALID;
    
    stream_send_retry_count++;

    if( timecounter<=sendCustomCmdRetryCounter )    //(cmdCFM==TRUE)
    {            
        //StreamResendLastData();
        //StreamStartSender(0x0000);//broadcast
        //StreamSendData(id_cmd,2);
        StreamSendDataBlock(devid_target,id_cmd,2);
        timecounter++;
        DebugWriteString(" resend-cmd");

        stream_send_retry_tid = TimerCreate(sendCustomCmdIntervalms*MILLISECOND, 
                                            TRUE,streamSendRetryTimer1);
    }
    else
    {
        stream_send_retry_count = 0;
        timecounter=0;//20160808
        //StreamFlush();
    }
}


void sendCustomCmd1(uint16 customCmd, uint16 targetDevid)
{      
    id_cmd[0]=CUSTOM_REQ_DEVID; //send cmd code
    id_cmd[1]=customCmd;
    //id_cmd[1]=REQ_ID_CMD;

    StreamStartSender(targetDevid);
    StreamSendData(id_cmd,2);
}
/*=============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppDataStreamInit
 *
 *  DESCRIPTION
 *      This function initializes the stream Model.
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
extern void AppDataStreamInit(uint16 *group_id_list, uint16 num_groups)
{   //var=1;
    /* Model initialisation */
    StreamModelInit(group_id_list, num_groups);

    /* Reset timers */
    stream_send_retry_tid = TIMER_INVALID;
    rx_stream_timeout_tid = TIMER_INVALID;

    /* Reset the device info */
    device_info_length = sizeof(DEVICE_INFO_STRING);
    device_info[0] = CSR_DEVICE_INFO_RSP;
    device_info[1] = device_info_length;

    MemCopy(&device_info[2], DEVICE_INFO_STRING, sizeof(DEVICE_INFO_STRING));
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleCSRmeshDataStreamFlushInd
 *
 *  DESCRIPTION
 *      This function handles the CSR_MESH_DATA_STREAM_FLUSH message.
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/

extern void handleCSRmeshDataStreamFlushInd(CSR_MESH_STREAM_EVENT_T *p_event)
{
    //if(0) rxStreamTimeoutHandler(0);  
    DebugWriteString("\r\nFLUSH.");
    rx_stream_offset = 0;

    if( rx_stream_in_progress == FALSE )
    {
        /* Change the Rx scan duty cycle to active at start of data stream */
        EnableHighDutyScanMode(TRUE);
        /* Start the stream timeout timer */
        TimerDelete(rx_stream_timeout_tid);
        rx_stream_timeout_tid = TimerCreate(RX_STREAM_TIMEOUT, TRUE,
                                                        rxStreamTimeoutHandler);
    }
    else//rx_stream_in_progress==TRUE
    {
        /* End of stream */
        rx_stream_in_progress = FALSE;
        TimerDelete(rx_stream_timeout_tid);
        rx_stream_timeout_tid = TIMER_INVALID;
        /* Set the mesh scan back to low duty cycle if the device is already 
         * configured.
         */
        EnableHighDutyScanMode(TRUE); //20160808 change from false to true
    }  
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleCSRmeshDataBlockInd
 *
 *  DESCRIPTION
 *      This function handles the CSR_MESH_DATA_BLOCK_IND message
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
extern void handleCSRmeshDataBlockInd(CSR_MESH_STREAM_EVENT_T *p_event)
{
    switch(p_event->data[0])
    {
        case CSR_DEVICE_INFO_REQ:
        {
            /* Change the Rx scan duty cycle to active at the start of stream */
            EnableHighDutyScanMode(TRUE);
            /* Set the source device ID as the stream target device */
            StreamStartSender(p_event->common_data.source_id );
            tx_stream_offset = 0;
            /* Set the opcode to CSR_DEVICE_INFO_RSP */
            device_info[0] = CSR_DEVICE_INFO_RSP;

            /* start sending the data */
            sendNextPacket();
        }
        break;

        case CSR_DEVICE_INFO_RESET:
        {
            /* Reset the device info */
            device_info_length = sizeof(DEVICE_INFO_STRING);
            device_info[0] = CSR_DEVICE_INFO_RSP;
            device_info[1] = device_info_length;
            MemCopy(&device_info[2], DEVICE_INFO_STRING,
                                                   sizeof(DEVICE_INFO_STRING));
        }
        break;

        default:
        break;
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleCSRmeshDataStreamDataInd
 *
 *  DESCRIPTION
 *      This function handles the CSR_MESH_DATA_STREAM_DATA_IND message
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
extern void handleCSRmeshDataStreamDataInd(CSR_MESH_STREAM_EVENT_T *p_event)
{   //DebugWriteString("handleCSRmeshDataStreamDataInd\r\n");
    /* Restart the stream timeout timer */
    TimerDelete(rx_stream_timeout_tid);
    rx_stream_timeout_tid = TimerCreate(RX_STREAM_TIMEOUT, TRUE,
                                                    rxStreamTimeoutHandler);

    /* Set stream in progress flag to TRUE */
    rx_stream_in_progress = TRUE;

    if( rx_stream_offset == 0 )
    {
        /* If the stream offset is 0. The data[0] will be the CODE */
        switch(p_event->data[0])
        {
            case CSR_DEVICE_INFO_REQ:
            {   DebugWriteString("CSR_DEVICE_INFO_REQ\r\n");
                /* Change the Rx scan duty cycle to active at start of stream */
                EnableHighDutyScanMode(TRUE);
                /* Set the source device ID as the stream target device */
                StreamStartSender(p_event->common_data.source_id);
                tx_stream_offset = 0;
                /* Set the stream code to CSR_DEVICE_INFO_RSP */
                device_info[0] = CSR_DEVICE_INFO_RSP;
                /* Start sending */
                sendNextPacket();
            }
            break;
            
            case CSR_DEVICE_INFO_SET:
            {       
                DebugWriteString("CSR_DEVICE_INFO_SET\r\n");//?
                /* CSR_DEVICE_INFO_SET is received. Store the code, length and
                 * the data into the device_info array in the format received
                 */
                current_stream_code = CSR_DEVICE_INFO_SET;
                device_info_length = p_event->data[1];
                MemCopy(device_info, p_event->data, p_event->data_len);
                rx_stream_offset = p_event->data_len;
                /* Change the Rx scan duty cycle to active at start of stream */
                EnableHighDutyScanMode(TRUE);
                
            }
            break;
            
            case CSR_DEVICE_INFO_RESET:
            {   DebugWriteString("CSR_DEVICE_INFO_RESET\r\n");
                /* Reset the device info */
                device_info_length = sizeof(DEVICE_INFO_STRING);
                device_info[0] = CSR_DEVICE_INFO_RSP;
                device_info[1] = device_info_length;
                MemCopy(&device_info[2], DEVICE_INFO_STRING,
                                                    sizeof(DEVICE_INFO_STRING));
            }
            break;

            case CUSTOM_REC_DEVID:            
            {
                if(receiveDevidInProcess==TRUE)  //20161109
                {
                    uint16 rx_data;                               
                    MemCopyPack(&rx_data,p_event->data+1,2);
                    storeDevIDtoNVM(rx_data);
                    DebugWriteString("\r\nReceived DevID:");
                    DebugWriteUint16(rx_data);
                    //StreamReset();//20160816
                }
            }
            break;
        }
    }
    else
    {
        if( current_stream_code == CSR_DEVICE_INFO_SET
            && rx_stream_offset + p_event->data_len < sizeof(device_info) )
        {
            MemCopy(&device_info[rx_stream_offset], p_event->data,
                                                            p_event->data_len);
            rx_stream_offset += p_event->data_len;
        }

        /* No other CODE is handled currently */
    }
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleCSRmeshDataStreamSendCfm
 *
 *  DESCRIPTION
 *      This function handles the CSR_MESH_DATA_STREAM_SEND_CFM message
 *
 *  RETURNS
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
extern void handleCSRmeshDataStreamSendCfm(CSR_MESH_STREAM_EVENT_T *p_event)
{   //DebugWriteString("handleCSRmeshDataStreamSendCfm\r\n");
    uint16 rxdat;//uint16 i;
     MemCopyPack(&rxdat,p_event->data+1,2);
     
     DebugWriteString(" -");
     DebugWriteUint16(rxdat);
     DebugWriteString(" cfm\r\n");

     //id_cfm=0;//表明id cfm发送和接受成功:阻断继续发id cmd指令
    cmdCFM=TRUE;
    //sendCustomCmd(TIMER_RETRY_TIME_LENGTH,TIMER_RETRY_TIME_COUNT,REQ_ID_CMD,0x0000);//
}
#endif /* ENABLE_DATA_MODEL */

