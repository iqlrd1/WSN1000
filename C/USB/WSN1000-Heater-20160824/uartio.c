/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2012-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 *  FILE
 *      uartio.c
 *
 *  DESCRIPTION
 *      UART IO implementation.
 *
 ******************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <uart.h>           /* Functions to interface with the chip's UART */

/*============================================================================*
 *  Local Header Files
 *============================================================================*/

#include "uartio.h"         /* Header file to this source file */
#include "byte_queue.h"     /* Byte queue API */
#include "mem.h"
#include <string.h>
#include <timer.h>
/*============================================================================*
 *  Private Data
 *============================================================================*/
 
 /* The application is required to create two buffers, one for receive, the
  * other for transmit. The buffers need to meet the alignment requirements
  * of the hardware. See the macro definition in uart.h for more details.
  */

#define RX_BUFFER_SIZE      UART_BUF_SIZE_BYTES_64
#define TX_BUFFER_SIZE      UART_BUF_SIZE_BYTES_64

/* Create 64-byte receive buffer for UART data */
UART_DECLARE_BUFFER(rx_buffer, RX_BUFFER_SIZE);

/* Create 64-byte transmit buffer for UART data */
UART_DECLARE_BUFFER(tx_buffer, TX_BUFFER_SIZE);
/*20160612*/
uint16 txDataLength=0;
timer_id endOfCmd_tid=TIMER_INVALID;
/*20160725*/
bool eof=FALSE;
char start[7] ="STRREC|";
char stop [7] ="STPREC|";
char conreq[7]="CONREQ|";
//char switchPeriod[]="ON";
//char search[9]="SEARCH60|";
char sonxoffx[]="SO";//20160829
char devidreq [9] ="DEVIDREQ|";
char devidread[10]="DEVIDREAD|";

char searchstop[11]="SEARCHSTOP|";
char delete[3]="DEL";
char removeall [10]="REMOVEALL|";

char getdata[]="GETDATA|";
char getdata0[1]="G";
/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

/* UART receive callback to receive serial commands */
static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_req_data_length);

/* UART transmit callback when a UART transmission has finished */
static void uartTxDataCallback(void);

/* Transmit waiting data over UART */
//static void sendPendingData(void);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      uartRxDataCallback
 *
 *  DESCRIPTION
 *      This is an internal callback function (of type uart_data_in_fn) that
 *      will be called by the UART driver when any data is received over UART.
 *      See DebugInit in the Firmware Library documentation for details.
 *
 * PARAMETERS
 *      p_rx_buffer [in]   Pointer to the receive buffer (uint8 if 'unpacked'
 *                         or uint16 if 'packed' depending on the chosen UART
 *                         data mode - this application uses 'unpacked')
 *
 *      length [in]        Number of bytes ('unpacked') or words ('packed')
 *                         received
 *
 *      p_additional_req_data_length [out]
 *                         Number of additional bytes ('unpacked') or words
 *                         ('packed') this application wishes to receive
 *
 * RETURNS
 *      The number of bytes ('unpacked') or words ('packed') that have been
 *      processed out of the available data.
 *----------------------------------------------------------------------------*/
static uint16 uartRxDataCallback(void   *p_rx_buffer,
                                 uint16  length,
                                 uint16 *p_additional_req_data_length)
{
    /*20160725*/
    uint8 *byte=(uint8 *)p_rx_buffer;
    if ( length > 0 )
    {
        if(*byte=='|') eof=TRUE;
        
        /* First copy all the bytes received into the byte queue */
        BQForceQueueBytes((const uint8 *)p_rx_buffer, length);//入队列
        txDataLength++;
    }
    
    /* Send any pending data waiting to be sent */
    //sendPendingData();
    /*20160612*/
                   
    uint8 cmdData[1];    
        if(eof==TRUE)
        {            
            if( MemCmp(conreq,g_queue,sizeof(conreq)/sizeof(char))==0 )//CONREQ
            {
                DebugWriteString("[CONREQOK]\r\n");    
            }
            else if(MemCmp(start,g_queue,sizeof(start)/sizeof(uint8))==0)//STRREC
            {
                continueToRecv=TRUE;//start receive
                //sendPendingData();DebugWriteString("\r\n");      
            }
            else if(MemCmp(stop,g_queue,sizeof(stop)/sizeof(uint8))==0)//STPREC
            {   
                continueToRecv=FALSE;
                //sendPendingData();DebugWriteString("\r\n");
            }            
            else if( MemCmp(devidreq,g_queue,sizeof(devidreq)/sizeof(uint8)) ==0 )//DEVIDREQ
            {
                idreq=FALSE;
                handleDevidRecv();
                //DebugWriteString("[DEVIDREQ50]\r\n");
            }
            else if( MemCmp(devidread,g_queue,sizeof(devidread)/sizeof(uint8)) ==0 )//DEVIDREAD
            {           
                idreq=TRUE;//not idreq but idread
                handleDevidRecv();
                //DebugWriteString("\r\ndevidread");
            }
            else if( MemCmp(sonxoffx,g_queue,2) ==0)                                //SOxFx
            {
                TimerDelete(clockMeshON_tid);
                TimerDelete(clockMeshOFF_tid);
                uint8 sendData[3];char timeChar[2];
                sendData[0]=REQ_ID_CMD;
                
                StrNCopy(timeChar,g_queue+2,2);
                sendData[1]=string2int(timeChar,2);//meshONTime(ascii->uint8)
                StrNCopy(timeChar,g_queue+5,2);
                sendData[2]=string2int(timeChar,2);//meshOFFTime
                
                meshONNotSleepTimeMin=sendData[1];
                meshOFFSleepTimeMin  =sendData[2];//local clock
                
                sendCustomCmd(500,10,sendData,3,CUSTOM_REQ_DEVID,0x0000);//broadcast
                DebugWriteString("[SEARCHON");printInDecimal(sendData[1]);
                DebugWriteString("OFF");printInDecimal(sendData[2]);
                DebugWriteString("]\r\n");
                TimerCreate(10*SECOND,TRUE,clockMeshONTimerHandler);
            }
            /*else if( MemCmp(search,g_queue,sizeof(search)/sizeof(uint8)) ==0 )  //SEARCH
            {
                cmdData[0]=REQ_ID_CMD;
                //uint16 broadcastCmdCount=3;                
                //while(broadcastCmdCount--)
                    sendCustomCmd(500,10,cmdData,1,CUSTOM_REQ_DEVID,0x0000);//broadcast
                DebugWriteString("[SEARCHSTART]\r\n");
            }
            else if(MemCmp(switchPeriod,g_queue,2)==0)                          //ONxOFFx
            {
                uint8 meshTogglePeriod[4];char timeChar[2];
                
                StrNCopy(timeChar,g_queue+2,2);
                meshTogglePeriod[0]=string2int(timeChar,2);//meshONTime(uint16->uint8)
                StrNCopy(timeChar,g_queue+7,2);
                meshTogglePeriod[1]=string2int(timeChar,2);//meshOFFTime            
                             
                meshONNotSleepTimeMin=meshTogglePeriod[0];
                meshOFFSleepTimeMin  =meshTogglePeriod[1];
                
                sendCustomCmd(100,3,meshTogglePeriod,2,CUSTOM_SET_MESH_PERIOD,0x0000);//target:all
                TimerCreate(TIMER_START_MESHSWITCH,TRUE,clockMeshONtimerHandler);//20160823
                DebugWriteString("[$READY]");
            }*/
            else if( MemCmp(removeall,g_queue,sizeof(removeall)/sizeof(uint8)) ==0)  //REMOVEALL
            {
                delAllStoreDedvidNVM();
                DebugWriteString("[REMOVEALLOK]\r\n");
            }
            else if( MemCmp(delete,g_queue,3) ==0)                              //DELETE
            {
                char devidchar[4];uint16 devid;
                StrNCopy(devidchar,g_queue+3,4);
                devid=str2inthex(devidchar,4);
                delDevidNVM(devid);
                DebugWriteString("[DELETE");DebugWriteUint16(devid);
                DebugWriteString("]\r\n");
            }
                
            else if(MemCmp(getdata,g_queue,sizeof(getdata)/sizeof(uint8)) ==0)  //GETDATA
            {   
                cmdData[0]=REQ_SENSOR_DATA;
                continueToRecv=TRUE;//open SENSOR_VALUE receive locked conditions
                sendCustomCmd(100,3,cmdData,1,CUSTOM_REQ_DATA,0x0000);//target:all
                DebugWriteString("[GETDATA]\r\n");
            }
            //else if( (txDataLength>(sizeof(getdata)+1)) )                  //Gx
            else if( MemCmp(getdata0,g_queue,sizeof(getdata0)/sizeof(uint8)) ==0)
            {               
                char devid[4];
                uint16 devid_recver=0;
                
                cmdData[0]=REQ_SENSOR_DATA;
                StrNCopy(devid,g_queue+1,4);
                devid_recver=str2inthex(devid,4);
                
                continueToRecv=TRUE;
                sendCustomCmd(100,5,cmdData,1,CUSTOM_REQ_DATA,devid_recver);
                DebugWriteString("[GETDATA");DebugWriteUint16(devid_recver);
                DebugWriteString("]\r\n");
            }            
                        
            else if( MemCmp(searchstop,g_queue,sizeof(searchstop)/sizeof(uint8)) ==0 )//SEARCHSTOP
            {
                DebugWriteString("[SEARCHSTARTSTOP]\r\n");
            }
            
            MemSet(g_queue,0x0,20);
            BQClearBuffer();
            txDataLength=0;
            eof=FALSE;            
        }


    *p_additional_req_data_length = (uint16)1;
    
    /* Return the number of bytes that have been processed */
    return length;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      uartTxDataCallback
 *
 *  DESCRIPTION
 *      This is an internal callback function (of type uart_data_out_fn) that
 *      will be called by the UART driver when data transmission over the UART
 *      is finished. See DebugInit in the Firmware Library documentation for
 *      details.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void uartTxDataCallback(void)
{
    /* Send any pending data waiting to be sent */
    sendPendingData();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      sendPendingData
 *
 *  DESCRIPTION
 *      Send buffered data over UART that was waiting to be sent. Perform some
 *      translation to ensured characters are properly displayed.
 *
 * PARAMETERS
 *      None
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void sendPendingData(void)
{   
    /* Loop until the byte queue is empty */
    while (BQGetDataSize() > 0)
    {
        uint8 byte = '\0';
        
        /* Read the next byte in the queue */
        if (BQPeekBytes(&byte, 1) > 0)
        {
            bool ok_to_commit = FALSE;
            
            /* Check if Enter key was pressed */
            if (byte == '\r')
            {
                /* Echo carriage return and newline */
                const uint8 data[] = {byte, '\n'};//格式问题
                //缓冲区到UART发送缓冲中
                ok_to_commit = UartWrite(data, sizeof(data)/sizeof(uint8));
            }
            else if (byte == '\b')
            /* If backspace key was pressed */
            {
                /* Issue backspace, overwrite previous character on the
                 * terminal, then issue another backspace
                 */
                const uint8 data[] = {byte, ' ', byte};//何解？？
                
                ok_to_commit = UartWrite(data, sizeof(data)/sizeof(uint8));
            }
            else
            {
                /* Echo the character */
                ok_to_commit = UartWrite(&byte, 1);
                //
            }

            if (ok_to_commit)
            {
                /* Now that UART driver has accepted this data
                 * remove the data from the buffer
                 */
                BQCommitLastPeek();
            }
            else
            {
                /* If UART doesn't have enough space available to accommodate
                 * this data, postpone sending data and leave it in the buffer
                 * to try again later.
                 */
                break;
            }
        }
        else
        {
            /* Couldn't read data for some reason. Postpone sending data and
             * try again later.
            */
            break;
        }
    }
}


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      Start
 *
 *  DESCRIPTION
 *      Run the startup routine.
 *
 * PARAMETERS
 *      last_sleep_state [in]   Last sleep state
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
void uartInit(void)
{
    /* Initialise UART and configure with default baud rate and port
     * configuration
     */
    UartInit(uartRxDataCallback,
             uartTxDataCallback,
             rx_buffer, RX_BUFFER_SIZE,//64
             tx_buffer, TX_BUFFER_SIZE,//64
             uart_data_unpacked);
    
    /* Enable UART */
    UartEnable(TRUE);

    /* UART receive threshold is set to 1 byte, so that every single byte
     * received will trigger the receiver callback */
    UartRead(1, 0);

    /* Send clear screen command over UART */
}
