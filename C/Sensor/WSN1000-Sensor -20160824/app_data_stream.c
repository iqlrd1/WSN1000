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
#include <debug.h>
#include <random.h>     //20160815
#include <sleep.h>      //20160816
/*============================================================================*
 *  CSR Mesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <stream_model.h>

/*=============================================================================*
 *  Local Header Files
*============================================================================*/
#include "app_data_stream.h"
#include "csr_mesh_tempsensor.h"
#ifdef  ENABLE_DATA_MODEL
/*=============================================================================*
 *  Private Definitions
 *============================================================================*/

#define DEVICE_INFO_STRING      "CSRmesh Temp Sensor application\r\n" \
                                "Supported Models:\r\n" \
                                "  Sensor Model\r\n" \
                                "  Bearer Model\r\n" \
                                "  Attention Model\r\n" \
                                "  Data Model"

/* Data stream send retry wait time */
//#define STREAM_SEND_TIME_INTERVAL         (500 * MILLISECOND)
#define STREAM_SEND_TIME_INTERVAL         0                               

/* Data stream received timeout value */
#define RX_STREAM_TIMEOUT                 (5 * SECOND)

/* Max number of retries */
#define MAX_SEND_RETRIES                  (30)
/*20160530*/                                
uint8 count=5;//

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
static uint16 streamSendRetryCount = 0;
static uint16 streamSendRetryCountCheck = 0;

/* Current Rx Stream offset */
static uint16 rx_stream_offset = 0;

/* Rx Stream status flag */
static bool rx_stream_in_progress = FALSE;

/* Rx stream timeout tid */
static timer_id rx_stream_timeout_tid;

static APP_DATA_STREAM_CODE_T current_stream_code;

uint8 dat[8];/*new code20160603*/
uint16 targetDevid;
uint16 stream_len=0;

uint16 deviceid_local;

   
/*=============================================================================*
 *  Private Function Prototypes
 *============================================================================*/
static void streamSendRetryTimer(timer_id tid);
void streamSendRetryTimerHandler(timer_id tid); //20160603

/*20160819*/
void debounceTimerHandlerCUSTOMREQDATA(timer_id tid);
/*=============================================================================*
 *  Private Function Implementations
 *============================================================================*/
/*20160530*/
extern void warning()
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
        streamSendRetryCount++;
        if( streamSendRetryCount < MAX_SEND_RETRIES )
        {
            StreamResendLastData();
            stream_send_retry_tid = TimerCreate(STREAM_SEND_TIME_INTERVAL, 
                                                TRUE, streamSendRetryTimer);
        }
        else
        {
            streamSendRetryCount = 0;
            StreamFlush();
            /* Set the mesh scan back to low duty cycle if the device is already 
             * configured.
             */
            EnableHighDutyScanMode(FALSE);
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
        /* Reset the stream */
        rx_stream_timeout_tid = TIMER_INVALID;
        rx_stream_in_progress = FALSE;
        StreamReset();
        /* Set the mesh scan back to low duty cycle if the device is already 
         * configured.
         */
        EnableHighDutyScanMode(FALSE);
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
    streamSendRetryCount = 0;
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
        EnableHighDutyScanMode(FALSE);
    }
}

void sendDataStreamWithCFM(uint16 dat_len,uint16 devid_dest, uint16 counter)
{   
    targetDevid = devid_dest;
    stream_len  = dat_len;
    streamSendRetryCount = counter;
    streamSendRetryCountCheck=0;
    streamCFM=FALSE;
    
    /* Stop retry timer */
    TimerDelete(stream_send_retry_tid); //reset timer
    stream_send_retry_tid = TIMER_INVALID; //reset timer
    
    /*20060803*/
    TimerDelete(ledFlashtid);//idle state indication
    ledFlashtid=TIMER_INVALID;
    PioSet(SEND_IND,0);//user mode:OFF
    PioSetMode(SEND_IND,pio_mode_pwm1);//fast bilnk ind 
    PioConfigPWM(1,pio_pwm_mode_push_pull, 1, 200, 1,
                                           200, 1, 1, 5);
    PioEnablePWM(1,TRUE); 
    
    StreamStartSender(targetDevid); //set target device id
    StreamFlush();
    StreamSendData(dat,stream_len); //function to send the data

    stream_send_retry_tid = TimerCreate(100 * MILLISECOND, TRUE,
                                        streamSendRetryTimerHandler);//retry after 100ms call streamSendRetryTimerHandler
}

void streamSendRetryTimerHandler(timer_id tid)//20160520 //send retry timer handler
{
    stream_send_retry_tid = TIMER_INVALID; //reset timer
    static uint32 random32;//20160815
    streamSendRetryCountCheck++; //inc retry counter
    
    if(streamSendRetryCountCheck>streamSendRetryCount)//directly send data without cfm
    {
        /*errorHandler(SEND_DEVID_UNSUCCESS);
        currentSensorStatus=SENSOR_STATUS_ERROR;
        saveSensorStatusToNVM();*/
        if(currentSensorStatus==SENSOR_STATUS_REGISTER)
        {               
            streamSendRetryCount = 0; //reset send retry counter
            streamSendRetryCountCheck = 0;
            TimerDelete(stream_send_retry_tid);
            stream_send_retry_tid=TIMER_INVALID;
            streamCFM=FALSE;//20160816
            
            currentSensorStatus=SENSOR_STATUS_NORMAL;//change status and start normal status timer
            //saveSensorStatusToNVM();
            PioEnablePWM(1,FALSE);//LED OFF ind
            
            PioSetModes(0x0e10,pio_mode_user);//0x1110 0001 0000
            PioSetDirs(0x0e10,0x0e10);
            PioSetPullModes(0x0e10,pio_mode_strong_pull_up);    
            CsrMeshEnableListening(FALSE);
            SleepModeChange(sleep_mode_deep);//20160816           
        }        
    }
    else  
        if(streamCFM==FALSE)  //20160816
    {
        StreamStartSender(targetDevid);
        StreamSendData(dat,stream_len);
        
        random32=(Random32()%20)*400;//(0:400:8000]
        stream_send_retry_tid =  
                TimerCreate(( STREAM_SEND_TIME_INTERVAL+random32 )*MILLISECOND, 
                            TRUE,streamSendRetryTimerHandler);
    }
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
{
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
{   DebugWriteString("\r\nflush ind.\r\n");
    rx_stream_offset = 0;

    if( rx_stream_in_progress == FALSE )
    {
        /* Change the Rx scan duty cycle to active at the start of data stream */
        EnableHighDutyScanMode(TRUE);
        /* Start the stream timeout timer */
        TimerDelete(rx_stream_timeout_tid);
        rx_stream_timeout_tid = TimerCreate(RX_STREAM_TIMEOUT, TRUE,
                                                        rxStreamTimeoutHandler);
    }
    else
    {
        /* End of stream */
        rx_stream_in_progress = FALSE;
        TimerDelete(rx_stream_timeout_tid);
        rx_stream_timeout_tid = TIMER_INVALID;
        /* Set the mesh scan back to low duty cycle if the device is already 
         * configured.
         */
        EnableHighDutyScanMode(FALSE);
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
    deviceid_src =p_event->common_data.source_id;//heater ID                      
    deviceid_local=g_node_data.device_uuid.uuid[0];//sensor ID;
    
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

        case CUSTOM_REQ_DATA:
        {
            if(flagCustomReqDataRec==FALSE)
            {
                DebugWriteString("CUSTOM_REQ_DATA");
                BME280_Trigger();
                EnableHighDutyScanMode(TRUE);
                SleepModeChange(sleep_mode_never);
                //recCmdOnceLock++;//20160819
                TimerCreate(3*SECOND,TRUE,debounceTimerHandlerCUSTOMREQDATA);
                TimerCreate(TIMER_SENSOR_READ_INIT_INTERVAL,TRUE,timerHandleReadBME280Data);
                flagCustomReqDataRec=TRUE;
            }
            //TimerCreate(2*SECOND,TRUE,turnOFFMeshTimerHandler);
        }
        break;
        
        case CUSTOM_REQ_DEVID:
        {                  
            uint8 dat1[4];//½ÓÊÕÖ¸Áî
            MemCopy(&dat1,&(p_event->data[1]),3);

            /*new code20160512*/
            if((currentSensorStatus==SENSOR_STATUS_INIT))//&&(dat1==REQ_ID_CMD)
            {           
                dat[0]=CUSTOM_REC_DEVID;
                dat[1]=(uint8)(deviceid_local&0xff);
                dat[2]=(uint8)((deviceid_local>>8)&0xff);             
                currentSensorStatus=SENSOR_STATUS_REGISTER; 
                //saveSensorStatusToNVM();/**/               
                TimerCreate(10*SECOND,TRUE,turnONMeshTimerHandler);//20160830
                sendDataStreamWithCFM(3,deviceid_src,100);//temporarily remove cfm
                meshONNotSleepTime=dat1[1];
                meshOFFSleepTime  =dat1[2];
            }
        }
        break;
        
       /*case CUSTOM_SET_MESH_PERIOD:
        {
            uint8 meshTimeMin;
            MemCopy(&meshTimeMin,p_event->data+1,1);//uint8->uint8
            meshONNotSleepTime=meshTimeMin;//MINUTE
            MemCopy(&meshTimeMin,p_event->data+2,1);            
            meshOFFSleepTime  =meshTimeMin;
            TimerCreate(10*SECOND,TRUE,turnONMeshTimerHandler);
        }
        break;*/
        
        default:
        break;
    }
}

void debounceTimerHandlerCUSTOMREQDATA(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;        
    flagCustomReqDataRec=FALSE;
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
{  
    TimerDelete(rx_stream_timeout_tid);
    rx_stream_timeout_tid = TimerCreate(RX_STREAM_TIMEOUT, TRUE,
                                               rxStreamTimeoutHandler);
    /*20160603*/
    //targetDevid==p_event->common_data.source_id;
    deviceid_src =p_event->common_data.source_id;//heater ID                      
    deviceid_local=g_node_data.device_uuid.uuid[0];//sensor ID;
    
    /* Set stream_in_progress flag to TRUE */
    rx_stream_in_progress = TRUE;

    if( rx_stream_offset == 0 )
    {
        /* If the stream offset is 0. The data[0] will be the CODE */
        switch(p_event->data[0])
        {
            case CSR_DEVICE_INFO_REQ:
            {
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

            case CSR_DEVICE_INFO_SET:
            {
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
extern void handleCSRmeshDataStreamSendCfm(CSR_MESH_STREAM_EVENT_T *p_event) //handler for receving CFM signal from traget dev(receiver)
{     
    streamCFM=TRUE; 
    
    if(currentSensorStatus==SENSOR_STATUS_REGISTER)
    {   
        
        streamSendRetryCount = 0; //reset send retry counter
        streamSendRetryCountCheck = 0;
        TimerDelete(stream_send_retry_tid);
        stream_send_retry_tid=TIMER_INVALID;
        
        currentSensorStatus=SENSOR_STATUS_NORMAL;//change status and start normal status timer
        //saveSensorStatusToNVM();
        
        PioEnablePWM(1,FALSE);//LED OFF ind
        PioSetMode(SEND_IND,pio_mode_user);
        if(ledFlashFlag==0)                   
            PioSet(SEND_IND,0);//OFF
        else 
            PioSet(SEND_IND,1);//ON 

    }/**/
}
#endif /* ENABLE_DATA_MODEL */

