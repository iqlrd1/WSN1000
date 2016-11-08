/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      csr_mesh_heater.c
 *
 *  DESCRIPTION
 *      This file defines a CSRmesh Heater application
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <pio.h>
#include <mem.h>
#include <Sys_events.h>
#include <Sleep.h>
#include <timer.h>
#include <security.h>
#include <gatt.h>
#include <gatt_prim.h>
#include <panic.h>
#include <nvm.h>
#include <buf_utils.h>
#include <random.h>
#include <config_store.h>
#include <debug.h>
#include <gatt_uuid.h>

/*============================================================================*
 *  CSRmesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <bearer_model.h>
#include <sensor_model.h>
#include <attention_model.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
#include "app_debug.h"
#include "app_gatt.h"
#include "mesh_control_service.h"
#include "gap_service.h"
#include "csr_mesh_heater.h"
#include "csr_mesh_heater_gatt.h"
#include "app_gatt_db.h"
#include "nvm_access.h"
#include "iot_hw.h"
#include "appearance.h"

#ifdef ENABLE_DATA_MODEL
#include "app_data_stream.h"
#endif /* ENABLE_DATA_MODEL */

#if defined (ENABLE_GATT_OTA_SERVICE) || defined (ENABLE_FIRMWARE_MODEL)
#include <csr_ota.h>
#include "csr_ota_service.h"
#include "gatt_service.h"
#endif /* ENABLE_GATT_OTA_SERVICE || ENABLE_FIRMWARE_MODEL */

#ifdef ENABLE_FIRMWARE_MODEL
#include <firmware_model.h>
#endif

#ifdef ENABLE_BATTERY_MODEL
#include "battery_hw.h"
#include "battery_model.h"
#endif /* ENBLE_BATTERY_MODEL */
//20160612
#include "uartio.h"
/*20160711*/
#define REC_IND     10
/*============================================================================*
 *  Private Data Types
 *============================================================================*/
#ifdef ENABLE_ACK_MODE
typedef struct 
{
    uint16 dev_id;
    uint16 tid;
}DEVICE_INFO_T;
#endif /* ENABLE_ACK_MODE */

static uint8 mode_selection=0;//default:

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

/* Maximum number of timers */
#define MAX_APP_TIMERS                 (10 + MAX_CSR_MESH_TIMERS+1)


/* CSRmesh device UUID size */
#define DEVICE_UUID_SIZE_WORDS         (8)

/* CSRmesh Authorization Code Size in Words */
#define DEVICE_AUTHCODE_SIZE_IN_WORDS  (4)

/* Default device UUID */
/*#define DEFAULT_UUID                   {0x3210, 0x7654, 0xBA98, 0xFEDC,\
                                         0xCDEF, 0x89AB, 0x4567, 0x0123}*/
#define DEFAULT_UUID                {0x8999,0x7,0x6,0x5,0x4,0x3,0x2,0x1}
/* Default Authorisation code */
#define DEFAULT_AUTH_CODE              {0xCDEF, 0x89AB, 0x4567, 0x0123}

/* CS Key for mesh advertising interval */
#define CSKEY_INDEX_CSRMESH_ADV_INTERVAL \
                                       (0)

/* CS Key for mesh advertising time */
#define CSKEY_INDEX_CSRMESH_ADV_TIME   (1)

/* CS Key for user flags */
#define CSKEY_INDEX_USER_FLAGS         (2)

/* Used for generating Random UUID */
#define RANDOM_UUID_ENABLE_MASK        (0x0001)

/* Used for permanently Enabling/Disabling Relay */
#define RELAY_ENABLE_MASK              (0x0002)

/* Used for permanently Enabling/Disabling Bridge */
#define BRIDGE_ENABLE_MASK             (0x0004)

/* Advertisement Timer for sending device identification */
#define DEVICE_ID_ADVERT_TIMER_ID      (5 * SECOND)

/* Slave device is not allowed to transmit another Connection Parameter
 * Update request till time TGAP(conn_param_timeout). Refer to section 9.3.9.2,
 * Vol 3, Part C of the Core 4.0 BT spec. The application should retry the
 * 'connection parameter update' procedure after time TGAP(conn_param_timeout)
 * which is 30 seconds.
 */
#define GAP_CONN_PARAM_TIMEOUT         (30 * SECOND)


/* TGAP(conn_pause_peripheral) defined in Core Specification Addendum 3 Revision
 * 2. A Peripheral device should not perform a Connection Parameter Update proc-
 * -edure within TGAP(conn_pause_peripheral) after establishing a connection.
 */
#define TGAP_CPP_PERIOD                (5 * SECOND)

/* TGAP(conn_pause_central) defined in Core Specification Addendum 3 Revision 2.
 * After the Peripheral device has no further pending actions to perform and the
 * Central device has not initiated any other actions within TGAP(conn_pause_ce-
 * -ntral), then the Peripheral device may perform a Connection Parameter Update
 * procedure.
 */
#define TGAP_CPC_PERIOD                (1 * SECOND)

#ifdef ENABLE_FIRMWARE_MODEL
/* OTA Reset Defer Duration */
#define OTA_RESET_DEFER_DURATION       (500 * MILLISECOND)
#endif

/* Magic value to check the sanity of NVM region used by the application. This 
 * value should be unique for each application as the NVM layout changes for
 * every application.
 */
#define NVM_SANITY_MAGIC               (0xAB87)

/*Number of IRKs that application can store */
#define MAX_NUMBER_IRK_STORED           (1)

/* NVM offset for the application NVM version */
#define NVM_OFFSET_APP_NVM_VERSION     (0)

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_SANITY_WORD         (NVM_OFFSET_APP_NVM_VERSION + 1)

/* NVM offset for NVM device l_press */
#define NVM_OFFSET_DEVICE_UUID         (NVM_OFFSET_SANITY_WORD + 1)

/* NVM Offset for Authorization Code */
#define NVM_OFFSET_DEVICE_AUTHCODE     (NVM_OFFSET_DEVICE_UUID + \
                                        DEVICE_UUID_SIZE_WORDS)

#define NVM_OFFSET_NETWORK_KEY         (NVM_OFFSET_DEVICE_AUTHCODE + \
                                        sizeof(CSR_MESH_AUTH_CODE_T))

#define NVM_OFFSET_DEVICE_ID           (NVM_OFFSET_NETWORK_KEY + \
                                        sizeof(CSR_MESH_NETWORK_KEY_T))

#define NVM_OFFSET_SEQUENCE_NUMBER     (NVM_OFFSET_DEVICE_ID + 1)

#define NVM_OFFSET_DEVICE_ETAG         (NVM_OFFSET_SEQUENCE_NUMBER + 2)
/*201606007*/
#define NVM_SENSOR_NUMBER_OFFSET     (NVM_OFFSET_DEVICE_ETAG+sizeof(CSR_MESH_ETAG_T))
#define NVM_SENSORS_DEVID_OFFSET     (NVM_SENSOR_NUMBER_OFFSET+sizeof(uint16))
/*20160616*/
#define NVM_HEATER_STATUS_OFFSET    (NVM_SENSORS_DEVID_OFFSET+ \
                                        (NUM_OF_DEVID_IN_NVM*sizeof(uint16)))
/*20160615*/
#define NVM_DEVID_ADDR_INDEX_OFFSET  (NVM_HEATER_STATUS_OFFSET+sizeof(uint16))

#define NVM_OFFSET_ASSOCIATION_STATE   (NVM_DEVID_ADDR_INDEX_OFFSET+(sizeof(uint16)))

/*#define NVM_OFFSET_ASSOCIATION_STATE   (NVM_OFFSET_DEVICE_ETAG + \
                                        sizeof(CSR_MESH_ETAG_T))*/

/* NVM Offset for Sensor Model Groups. */
#define NVM_OFFSET_SENSOR_MODEL_GROUPS (NVM_OFFSET_ASSOCIATION_STATE + 1)

/* NVM Offset for Attention Model Groups. */
#define NVM_OFFSET_ATTENTION_MODEL_GROUPS \
                                       (NVM_OFFSET_SENSOR_MODEL_GROUPS + \
                                        sizeof(sensor_model_groups))

#ifdef ENABLE_DATA_MODEL                                        
#define NVM_OFFSET_DATA_MODEL_GROUPS \
                                       (NVM_OFFSET_ATTENTION_MODEL_GROUPS + \
                                        sizeof(attention_model_groups))

/* NVM Offset for the Bearer State Data. */
#define NVM_BEARER_DATA_OFFSET         (NVM_OFFSET_DATA_MODEL_GROUPS + \
                                        sizeof(data_model_groups))

#else

/* NVM Offset for the Bearer State Data. */
#define NVM_BEARER_DATA_OFFSET         (NVM_OFFSET_ATTENTION_MODEL_GROUPS + \
                                        sizeof(attention_model_groups))
#endif

/* NVM Offset for sensor State Data */
#define NVM_SENSOR_STATE_OFFSET      (NVM_BEARER_DATA_OFFSET + \
                                        sizeof(BEARER_MODEL_STATE_DATA_T))

/* Size of sensor State data to be stored in NVM */
#define SENSOR_SAVED_STATE_SIZE      (2 * sizeof(uint16))
//#define SENSOR_SAVED_STATE_SIZE      ( sizeof(uint16))/*20160530*/
/* Get NVM Offset of a sensor from it's index. */
/*#define GET_SENSOR_NVM_OFFSET(idx)   (NVM_SENSOR_STATE_OFFSET + \
                                        ((idx) * (SENSOR_SAVED_STATE_SIZE)))*/
#define GET_DEVICE_NVM_OFFSET(idx)   (NVM_SENSOR_STATE_OFFSET + \
                                        ((idx) * (SENSOR_SAVED_STATE_SIZE)))
/* NVM Offset for Application data */
#define NVM_MAX_APP_MEMORY_WORDS     (NVM_SENSOR_STATE_OFFSET + \
                                        (NUM_SENSORS_SUPPORTED * \
                                         SENSOR_SAVED_STATE_SIZE))
/* Number of Supported sensor types. */
#define NUM_SENSORS_SUPPORTED        (4)

/* Current Air temperature sensor Index */
#define CURRENT_AIR_TEMP             (0)

/* Desired Air temperature sensor Index */
#define CURRENT_AIR_TEMP           (0)  /*20160704*/
#define CURRENT_AIR_HUMI           (1) //20160808
#define CURRENT_PRESSURE_LOW       (2)
#define CURRENT_PRESSURE_HIGH      (3)

//#define CURRENT_BATTERY            (1)  //CURRENT_AIR_HUMI
//#define CURRENT_HUMI_LOW           (4)
//#define CURRENT_HUMI_HIGH          (5)

/* Max sensor type supp in this app sensor_type_desired_air_temperature = 3*/
#define SENSOR_TYPE_SUPPORTED_MAX    sensor_type_barometric_pressure

#ifdef ENABLE_ACK_MODE
/* Maximum devices for which ackowledgements could be sent */
#define MAX_ACK_DEVICES              (5)

/* Maximum retransmit count */
#define MAX_RETRANSMIT_COUNT         (MAX_RETRANSMISSION_TIME / \
                                      RETRANSMIT_INTERVAL)

/* The broadcast id for MESH is defined as 0 */
#define CSR_MESH_BROADCAST_ID        (0)
#endif /* ENABLE_ACK_MODE */

//20160608
#define MAX_REC_DEVID_COUNT           (3)

#define TEMP_DATA_RECVD     0x01
#define HUMI_DATA_RECVD     0X02
#define PRESS_HIGH_RECVD    0X04
#define PRESS_LOW_RECVD     0x08
#define ALL_DATA_RECVD      0X0F
#define NO_DATA_RECVD       0X00

//#define BATT_DATA_RECVD     0x02
//#define HUMI_HIGH_RECVD     0X10
//#define HUMI_LOW_RECVD      0X20
/*============================================================================*
 *  Public Data
 *============================================================================*/

/* CSRmesh Heater Application data instance */
CSRMESH_HEATER_DATA_T g_heater_app_data;

/* CSRmesh network related node data */
CSR_MESH_NODE_DATA_T g_node_data ={ 
    .device_uuid.uuid = DEFAULT_UUID,
    .auth_code.auth_code = DEFAULT_AUTH_CODE,
};

/* Application VID,PID and Version. */
CSR_MESH_VID_PID_VERSION_T vid_pid_info =
{
    .vendor_id  = APP_VENDOR_ID,
    .product_id = APP_PRODUCT_ID,
    .version    = APP_VERSION,
};

/* Attention timer id */
static timer_id attn_tid = TIMER_INVALID;
/*20160608*/
timer_id sensor_data_incomplete_tid;
/*20160613*/
timer_id no_data_check_tid=TIMER_INVALID;
/*20160623*/
timer_id info_tid=TIMER_INVALID;

#ifdef ENABLE_FIRMWARE_MODEL
/* Firmware Reset Delay Timer Id */
static timer_id ota_rst_tid = TIMER_INVALID;
#endif /* ENABLE_FIRMWARE_MODEL */

/* Device Apprearance. */
CSR_MESH_APPEARANCE_T device_appearance = {APPEARANCE_ORG_BLUETOOTH_SIG,
                                           APPEARANCE_CSR_MESH_HEATER_VALUE};

/* Device Short name */
uint8 short_name[9] = "Heater";

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Declare space for application timers. */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];

#ifdef ENABLE_ACK_MODE
/* Retransmit Timer ID. */
static timer_id retransmit_tid = TIMER_INVALID;

/* Write Value Msg Retransmit counter */
static uint16   ack_retransmit_count = 0;
#endif /* ENABLE_ACK_MODE */

/* Temperature Value in 1/32 kelvin units. */
static uint16 current_air_temp;
/* Current Desired Air Temperature. */
//static SENSOR_FORMAT_TEMPERATURE_T current_desired_air_temp;
//static uint16 current_battery_level;
//static uint16 current_humi_high;
//static uint16 current_humi_low;

static uint16 current_air_humi;
static uint16 current_press_low;
//static uint16 current_press_high;//当前温湿度、气压数据
//static uint32 current_air_press;//20160809
static uint16 current_air_press;

static SENSOR_FORMAT_TEMPERATURE_T recvd_air_temp=0;//20160704
static SENSOR_FORMAT_TEMPERATURE_T recvd_air_humi=0;//20160808
static SENSOR_FORMAT_TEMPERATURE_T recvd_air_press=0;//20160809

//static SENSOR_FORMAT_TEMPERATURE_T recvd_press_low=0;
//static SENSOR_FORMAT_TEMPERATURE_T recvd_press_high=0;//方便初始化 

//static SENSOR_FORMAT_TEMPERATURE_T recvd_bat_level=0;
//static SENSOR_FORMAT_TEMPERATURE_T recvd_humi_low=0;
//static SENSOR_FORMAT_TEMPERATURE_T recvd_humi_high=0;
/* Supported sensor type state */
static SENSOR_STATE_DATA_T sensor_state[NUM_SENSORS_SUPPORTED];

#ifdef ENABLE_DATA_MODEL
static uint16 data_model_groups[NUM_DATA_MODEL_GROUPS];
#endif /* ENABLE_DATA_MODEL */

/* sensor Model Grouping Data. */
static uint16 sensor_model_groups[NUM_SENSOR_MODEL_GROUPS];

/* Attention Model Grouping Data. */
static uint16 attention_model_groups[NUM_ATTENTION_MODEL_GROUPS];

/* Read value transmit counter */
static uint16 read_value_transmit_count = 0;

/* Read Value Timer ID. */
static timer_id read_val_tid = TIMER_INVALID;

/* To send the MASP associate to NW msg and Dev appearance msg alternatively */
static bool send_dev_appearance = FALSE;

/* store the bearer relay active value during connection and restore it back 
 * after the disconnection
 */
static uint16 bearer_relay_active;

/* store the promiscuous value during connection and restore it back after the 
 * disconnection
 */
static uint16 bearer_promiscuous;

#ifdef ENABLE_ACK_MODE
/* Array of structure holding the device id and the transaction id to be sent*/
static DEVICE_INFO_T sensor_dev_ack[MAX_ACK_DEVICES];
#endif /* ENABLE_ACK_MODE */

//20160519
static bool id_cfm=0;//自启动
/*20160607*/
uint16 receiveIDcount=MAX_REC_DEVID_COUNT;
/*20160608*/
uint16 dataReceivedFlag=0;
/*20160708*/
//static bool outputSensorOneTimeFlag=TRUE;//make the same device only output one time
/*20160615*/
uint16  endOfDevidNvmAddr=0;
uint16 numberOfRegSensor=0;
static uint16 nvmBuffer[NUM_OF_DEVID_IN_NVM];
static uint16 recSensorDataDevid;//20160617
/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/
static void appDataInit(void);
static void readPersistentStore(void);
static void handleSignalLmEvConnectionComplete(
                                     LM_EV_CONNECTION_COMPLETE_T *p_event_data);
static void handleSignalLmConnectionUpdate(
                                     LM_EV_CONNECTION_UPDATE_T* p_event_data);
static void handleGapCppTimerExpiry(timer_id tid);
static void deviceIdAdvertTimeoutHandler(timer_id tid);
//static void updateHeaterStatus(void);

static void startReadValueTimer(void);
static void readValTimerHandler(timer_id tid);
static void readCurrentTempFromGroup(void);

//void devicecpy(DEVICE_T *dest,DEVICE_T *src);
void printTimeAndData(timer_id tid);
void printCurrentTime(void);


uint16 getNumberOfRegSensor(void);
void bubble_sorting(uint16 a[],uint16 n);
void listAllDetectedDevice(void);
void handleSensorDataIncomplete(timer_id tid);
void sendcmdTimerHandler(timer_id tid);
/*20160531*/
void handleRegisterMode(uint16 heater_status0);


#ifdef DEBUG_ENABLE
/* UART receive callback to receive serial commands */
//20160612
//static uint16 uartRxDataCallback(void   *p_rx_buffer,
//                                 uint16  length,
//                                 uint16 *p_req_data_length);

//static uint16 UartDataRxCallback (void* p_data, uint16 data_count,
//                                  uint16* p_num_additional_words);

//static uint16 UartDataRxCallback1 (void* p_data, uint16 data_count,
//                                  uint16* p_num_additional_words);

#else

#define printInDecimal(n)

#endif /* DEBUG_ENABLE */

#ifdef ENABLE_ACK_MODE
static void sendValueAck(void);
static void retransmitIntervalTimerHandler(timer_id tid);
static void startRetransmitTimer(void);
static void addDeviceToSensorList(uint16 dev_id, uint8 tid);
static void resetDeviceList(void);
#endif /* ENABLE_ACK_MODE */

/*test code 20160419*/
#ifdef PRESET_DEVID
static void presetDevid(void);
#endif

//20160608
void delnvmBuffer(uint16 devid);
void errorHandle(error_code err,uint16 param);
/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/
/*void devicecpy(DEVICE_T *dest,DEVICE_T *src)
{   dest->deviceid=src->deviceid;//
    //dest->temp=src->temp;
    //dest->humi=src->humi;
    //dest->l_press=src->l_press;
    //dest->h_press=src->h_press;
    //dest->press  =src->press;
}*/
void printTimeAndData(timer_id tid)
{    
    timer_id tid0=tid;
    TimerDelete(tid0);
    
    //printCurrentTime();
    uint16 i;
    if(inputNumberOfSensor!=0){//
        if(mode_selection==0)  /*LIST_FORMAT*/
        {   DEBUG_STR("---------------------------------\r\n");
            for(i=1;i<=inputNumberOfSensor;i++)
            {   //轮询
                //if(g_device[i].deviceid!=0)
                {   DEBUG_STR(" Device ID:");
                    //DEBUG_U16(g_device[i].deviceid);
                    DEBUG_STR("\r\nRECEIVED CURRENT TEMP : ");
                    printInDecimal(current_air_temp);DEBUG_STR(" ℃\r\n");
                    
                    DEBUG_STR("RECEIVED CURRENT HUMI : ");
                    printInDecimal(current_air_humi);DEBUG_STR(" %\r\n");
                                    
                    DEBUG_STR("RECEIVED CURRENT PRES : ");
                    printInDecimal(current_air_press);DEBUG_STR(" Pa ");
                    //DEBUG_U16(g_device[i].h_press);DEBUG_STR(" ");
                    //DEBUG_U16(g_device[i].l_press);
                    DEBUG_STR("\r\n");
                }
            }
        }
        if(mode_selection==1)  /*CSV_FORMAT*/
        {   
            for(i=1;i<inputNumberOfSensor+1;i++)
            {   //轮询
                //if(g_device[i].deviceid!=0){            
                    DEBUG_STR("Device ID,");
                    //DEBUG_U16(g_device[i].deviceid);
                    
                    DEBUG_STR(",Temp,");
                    printInDecimal(current_air_temp);DEBUG_STR("℃,");
                    
                    DEBUG_STR("Humi,");        
                    printInDecimal(current_air_humi);DEBUG_STR("%,");
                    
                    DEBUG_STR("Press,");        
                    printInDecimal(current_air_press);DEBUG_STR("Pa\r\n");                         
                //}
            }
        }
    }
    //time_dat_tid=TimerCreate(10*SECOND,TRUE,printTimeAndData);//闭环自循环
}

#if 0
//20160622
static void infoTimerHandler(timer_id tid)
{
    DEBUG_STR("\r\nPlease input sensors number:");
    if(currentHeaterStatus==HEATER_STATUS_INIT) 
        info_tid=TimerCreate(5*SECOND,TRUE,infoTimerHandler);
    else TimerDelete(info_tid);
}
#endif

void sendcmdTimerHandler(timer_id tid)
{   
    TimerDelete(tid);
    uint16 req_id_cmd=REQ_ID_CMD;//指示sensor传送device ID
    //uint16 registered=REGISTERED;//指示sensor已搜索成功
    uint8 cmd[2];//uint8 id_cmd[2],regist_cmd[2];
    DEBUG_STR("Start send id cmd ");
    if(id_cfm==1){
        DEBUG_STR("send id cmd \r\n");
        StreamFlush();    
        StreamStartSender(0x0000);//broadcast(send cmd to sensors)
        cmd[0]=(uint8)(req_id_cmd&0xff);cmd[1]=(uint8)((req_id_cmd>>8)&0xff);//先低位
        //cmd[1]=(uint8)(req_id_cmd&0xff);cmd[0]=(uint8)((req_id_cmd>>8)&0xff);//先高位
        StreamSendData(cmd,2);//start id transmission 
        TimerCreate(1*SECOND,TRUE,sendcmdTimerHandler);
    }
}

#if 0
void printCurrentTime(void)
{   
    /* Read current system time */
    const uint32 now = TimeGet32();    
    /* Report current system time */
    DebugWriteString("\n\nCurrent system time: ");
    writeASCIICodedNumber(now / MINUTE);
    DebugWriteString("m ");
    writeASCIICodedNumber((now % MINUTE)/SECOND);
    DebugWriteString("s------------\r\n");  
}
#endif
static void lStrip(void)
{
    uint8 byte = '\0';
    
    /* Loop until the byte queue is empty */
    while (BQPeekBytes(&byte, 1) > 0)
    {
        /* Is the next byte in the queue a white space character? */
        if ((byte == ' ') || (byte == '\t') ||
             (byte == '\r') || (byte == '\n'))
        {
            /* Clear it and continue reading */
            BQCommitLastPeek();
        }
        else
        {
            /* Jump out of the loop */
            break;
        }
    }
}
uint16 string2int(char *str,uint16 data_count)  //uint8
{
    uint8  byte = '\0';         /* Next byte read from queue */
    uint16 total = 0;           /* Running total */
    /* Strip any leading white space from the byte queue */
    lStrip();
    //if ( data_count > 0 )
    //{
        /* First copy all the bytes received into the byte queue */
    //    BQForceQueueBytes((const uint8 *)str, data_count);
    //}
    
    /* Send any pending data waiting to be sent */
    //sendPendingData();
    /* Loop until the byte queue is empty */    
    while (*str > 0)
    {   
        byte=*str;
        //DEBUG_U8(byte);
        //DEBUG_STR(" <-string\r\n");
        /* If the next byte in the queue is a valid ASCII digit */
        if ((byte >= '0') && (byte <= '9'))
        {
            /* Convert ASCII decimal value into uint8 */
            const uint8 num_byte = byte - '0';

            /* Append decimal value to current result */
            total = 10 * total + num_byte;

            /* Remove this byte from the queue */
            //BQCommitLastPeek();
        }
        else /* If the next byte in the queue is not a valid ASCII digit */
        {
            /* Finish parsing and exit */
            break;
        }
        str++;
    }
    return total;
}

uint16 str2inthex(char *str,uint16 data_count)
{
    uint8  byte = '\0';
    uint16 total = 0;
    
    while (*str > 0)
    {   
        byte=*str;

        if ((byte >= '0') && (byte <= '9'))
        {
            const uint8 num_byte = byte - '0';
            total = (total<<4) + num_byte;
        }
        else if( (byte>='a')&&(byte<='z') )
        {
            const uint8 num_byte = byte - 0x57;
            total = (total<<4) + num_byte;
        }
        else if( (byte>='A')&&(byte<='Z') )
        {
            const uint8 num_byte = byte - 0x37;
            total = (total<<4) + num_byte;
        }
        else
        {
            break;
        }
        str++;
    }
    return total;
}

uint16 getNumberOfRegSensor(void)
{   
    uint16 n;
    uint16 deviceid;
    //uint16 numberOfRegSensor=0;
    /*do(n<inputNumberOfSensor)
    {   Nvm_Read(&deviceid,sizeof(deviceid),NVM_SENSORS_DEVID_OFFSET+n);
        n++;
    }
    while(deviceid!=0xffff)
    {   numberOfRegSensor++;
    }*/ 
    for(n=0;n<NUM_OF_DEVID_IN_NVM;n++)   
    {   
        Nvm_Read(&deviceid,sizeof(deviceid),NVM_SENSORS_DEVID_OFFSET+n);        
          if(deviceid!=0xffff)
                numberOfRegSensor++;
    }   
    return endOfDevidNvmAddr;//numberOfRegSensor
}

void saveInputSensorNumberToNvm(void)//input intended
{   
    Nvm_Write(&inputNumberOfSensor,1,NVM_SENSOR_NUMBER_OFFSET);
}

extern void saveReceiveDeviceIDToNvm(uint16 *buffer,uint16 length)
{   NvmWrite(buffer,length,NVM_SENSORS_DEVID_OFFSET);
}

/*20160726*/
void saveHeaterStatusToNVM()
{
    Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);
}

void recDevIDTimerHandler(timer_id tid)
{   
    TimerDelete(receive_delay_tid);
    receive_delay_tid=TIMER_INVALID;
    
    uint8 cmdData[1]={REQ_ID_CMD};
    if(receiveIDcount>0)
    {          
        receiveIDcount--;
        DEBUG_STR("\r\nSeaching...Remain");
        DEBUG_U16(receiveIDcount);
        DEBUG_STR("min\r\n");
        sendCustomCmd(TIMER_RETRY_TIME_LENGTH,TIMER_RETRY_TIME_COUNT,cmdData,1,CUSTOM_REQ_DEVID,0x0000);
    }
    else 
    {   
       /* uint16 i;
        uint16 test_devid=0xABC0;
        for(i=0;i<NUM_OF_DEVID_IN_NVM;i++)            
            storeDevIDtoNVM(test_devid+i);*/
        
        currentHeaterStatus=HEATER_STATUS_REGISTER;
        Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);//20160616
        handleRegisterMode(currentHeaterStatus);
        receiveIDcount=MAX_REC_DEVID_COUNT;
    }
}

void handleRegisterMode(uint16 heater_status0)
{      
    /*uint16 missing_num=0;

    DEBUG_STR("\r\nUser input total sensor number:");
    printInDecimal(inputNumberOfSensor);DEBUG_STR("\r\n");*/
   
    if(heater_status0==HEATER_STATUS_REGISTER)
    {   
        /*DEBUG_STR("Sucessfully regsistered sensor number:");
        printInDecimal(endOfDevidNvmAddr);
        DEBUG_STR("\r\n");
        
        missing_num=inputNumberOfSensor-endOfDevidNvmAddr;
        DEBUG_STR("Missing sensor number   :");
        printInDecimal(missing_num);
        DEBUG_STR("\r\n");
        
        if(missing_num!=0) 
            listAllDetectedDevice();//模块耦合
        */
        /*20150726*/
        if(idreq==TRUE)
        {
            DEBUG_STR("[DEVIDREQ");
            printInDecimal(endOfDevidNvmAddr);DEBUG_STR("]\r\n");
        }
        else listAllDetectedDevice();/**/
        //currentHeaterStatus=HEATER_STATUS_NORMAL;//first enter,update dynamic record table
        //Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);//20160616           
    }
}
/*20160727*/
void handleDevidRecv()
{
    uint16 regNumberOfSensor=0;
    if(idreq==FALSE)
    {
        DEBUG_STR("[DEVIDREQ");
        regNumberOfSensor=getNumberOfRegSensor();
        Nvm_Write(&regNumberOfSensor,1,NVM_SENSOR_NUMBER_OFFSET);
        printInDecimal(regNumberOfSensor);DEBUG_STR("]\r\n");
        
        DEBUG_STR("[ADDR");
        printInDecimal(endOfDevidNvmAddr);
        DEBUG_STR("]\r\n");
    }
    else listAllDetectedDevice();/**/
}

void bubble_sorting(uint16 a[],uint16 n)
{   
    uint16 i,j;int temp;
    
    for(i=0;i<n-1;i++)
    {   
        for(j=0;j<n-1;j++)
        {   
            if(a[j]>a[j+1]) 
            {   temp=a[j];
                a[j]=a[j+1];
                a[j+1]=temp;
            }
        }
    }
}
void listAllDetectedDevice()
{   
    uint16 k,regSensorNumber=0;
    MemSet((void *)nvmBuffer,0xffff,NUM_OF_DEVID_IN_NVM); //20160615 clear NVM buffer before use
    Nvm_Read(nvmBuffer,endOfDevidNvmAddr,NVM_SENSORS_DEVID_OFFSET);
    regSensorNumber=getNumberOfRegSensor();
    bubble_sorting(nvmBuffer,endOfDevidNvmAddr+1);//order
    DEBUG_STR("[IDS");//20160727
    
    if(endOfDevidNvmAddr>=2)
    {
        for(k=0;k<=endOfDevidNvmAddr-2;k++)
        //for(k=0;k<=regSensorNumber-2;k++)
        {
            DEBUG_U16(nvmBuffer[k]);
            DEBUG_STR("/");
        }
        DEBUG_U16(nvmBuffer[endOfDevidNvmAddr-1]);
        DEBUG_STR("]\r\n");
    }
    else
    {
        DEBUG_U16(nvmBuffer[endOfDevidNvmAddr-1]);
        DEBUG_STR("]\r\n");
    }

}


void handleSensorDataIncomplete(timer_id tid)
{   
    DEBUG_STR("RECEIVE INCOMPLETE SENSOR DATA!!\r\n");
    timer_id tid0=tid;
    TimerDelete(tid0);
    tid0=TIMER_INVALID;
    warning();//模块耦合
}
#if 0
static bool verifyData(void)
{   
    if(
       (current_air_temp>TEMP_HIGHEST)||(current_air_temp<TEMP_LOWEST)||
       (current_air_humi>HUMI_HIGHEST)||(current_air_humi<HUMI_LOWEST)||
       (current_air_press>PRESS_HIGHEST)||(current_air_press<PRESS_LOWEST))
    {   
        return 1;
    }
    return 0;
}
#endif

#if 0
static void periodicCheckNoDataDevices(timer_id tid)
{   
    TimerDelete(tid);
    uint16 i;
    MemSet((void *)nvmBuffer,0xffff,NUM_OF_DEVID_IN_NVM); //20160615 clear NVM buffer before use
    Nvm_Read(nvmBuffer,NUM_OF_DEVID_IN_NVM,NVM_SENSORS_DEVID_OFFSET);
    
    for(i=0;i<NUM_OF_DEVID_IN_NVM;i++)
    {
       if(nvmBuffer[i]==recSensorDataDevid)
           if(dataReceivedFlag==NO_DATA_RECVD)
           {
               DEBUG_STR("Device ");DEBUG_U16(recSensorDataDevid);
               DEBUG_STR(" has not received any data!!\r\n");
               errorHandle(ERR_CODE_SENSOR_NO_REPONSE,nvmBuffer[i]);
           }
    }
    no_data_check_tid=TimerCreate(30*SECOND,TRUE,periodicCheckNoDataDevices);
}
#endif

void errorHandle(error_code err,uint16 param)
{   /*
    switch(err)
    {
        case ERR_CODE_SENSOR_NO_REPONSE:
        {           
        }
        case ERR_CODE_DATA_OUTOF_RANGE:
        {
        }
        default:
        break;
    }*/
}
bool storeDevIDtoNVM(uint16 devid)
{   
    uint16 i;//20160616

    MemSet((void *)nvmBuffer,0xffff,NUM_OF_DEVID_IN_NVM); //20160615 clear NVM buffer before use
    
    Nvm_Read(nvmBuffer,endOfDevidNvmAddr,NVM_SENSORS_DEVID_OFFSET);

    for(i=0;i<endOfDevidNvmAddr;i++)
        if(nvmBuffer[i]==devid)
            return 0;
    
    Nvm_Write(&devid,1,NVM_SENSORS_DEVID_OFFSET+endOfDevidNvmAddr);
    
    endOfDevidNvmAddr++;

    Nvm_Write(&endOfDevidNvmAddr,1,NVM_DEVID_ADDR_INDEX_OFFSET);    
    
    return 1;       
}


//20160608
void delnvmBuffer(uint16 devid)
{
    uint16 i;
    for(i=0;i<=NUM_OF_DEVID_IN_NVM;i++)
    {
        if(nvmBuffer[i]==devid)
            nvmBuffer[i]=0;
    }
}
void delDevidNVM(uint16 devid)
{
    uint16 i;
    MemSet((void *)nvmBuffer,0xffff,NUM_OF_DEVID_IN_NVM); 
    Nvm_Read(nvmBuffer,endOfDevidNvmAddr,NVM_SENSORS_DEVID_OFFSET);
    for(i=0;i<endOfDevidNvmAddr;i++)
        if(nvmBuffer[i]==devid) nvmBuffer[i]=0xffff;
    Nvm_Write(nvmBuffer,endOfDevidNvmAddr,NVM_SENSORS_DEVID_OFFSET);
}
/*20160830*/
void delAllStoreDedvidNVM(void)
{
    uint16 regSensorNumber=getNumberOfRegSensor();
    //printInDecimal(regSensorNumber);DebugWriteString("totalDevice\r\n");
    uint16 i,nvm_erase=0xffff;
    for(i=0;i<regSensorNumber;i++)
        Nvm_Write(&nvm_erase,1,NVM_SENSORS_DEVID_OFFSET+i);
    endOfDevidNvmAddr=0; 
}   
/*20160708*/
#if 0//
static void avoidRepeatPrintSameDevTimerHandler(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;
    outputSensorOneTimeFlag=TRUE;
}
#endif//
/*20160711*/
#if 0//
static void recDataInd(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;
    PioSet(REC_IND,1);//off
}
#endif//

/*20160817*/
void clockMeshONTimerHandler(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;
    SleepModeChange(sleep_mode_never);
    CsrMeshEnableListening(TRUE);

    DEBUG_STR("[$READY]");
    PioSetMode(MESHON_IND,pio_mode_user);
    PioSet(MESHON_IND,0);//ON
    /*20160823*/
    clockMeshON_tid=TimerCreate(meshONNotSleepTimeMin*MINUTE,TRUE,clockMeshOFFTimerHandler);//SECOND
    //TimerCreate(TIMER_HEATER_MESHON,TRUE,clockMeshOFFTimerHandler);
}

void clockMeshOFFTimerHandler(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;
    SleepModeChange(sleep_mode_never);
    CsrMeshEnableListening(TRUE);

    DEBUG_STR("[$END]");
    PioSetMode(MESHON_IND,pio_mode_user);
    PioSet(MESHON_IND,1);//OFF
    /*20160823*/
    clockMeshOFF_tid=TimerCreate(meshOFFSleepTimeMin*MINUTE,TRUE,clockMeshONTimerHandler);//SECOND
}    
    
#ifdef USE_STATIC_RANDOM_ADDRESS
/*-----------------------------------------------------------------------------*
 *  NAME
 *      generateStaticRandomAddress
 *
 *  DESCRIPTION
 *      This function generates a static random address.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void generateStaticRandomAddress(BD_ADDR_T *addr)
{
    uint16 temp[3];
    uint16 idx = 0;

    if (!addr) return;

    for (idx = 0; idx < 3;)
    {
        temp[idx] = Random16();
        if ((temp[idx] != 0) && (temp[idx] != 0xFFFF))
        {
            idx++;
        }
    }

    addr->lap = ((uint32)(temp[1]) << 16) | (temp[0]);
    addr->lap &= 0x00FFFFFFUL;
    addr->uap = (temp[1] >> 8) & 0xFF;
    addr->nap = temp[2];

    addr->nap &= ~BD_ADDR_NAP_RANDOM_TYPE_MASK;
    addr->nap |=  BD_ADDR_NAP_RANDOM_TYPE_STATIC;
}
#endif /* USE_STATIC_RANDOM_ADDRESS */

#ifdef ENABLE_FIRMWARE_MODEL
/*-----------------------------------------------------------------------------*
 *  NAME
 *      issueOTAReset
 *
 *  DESCRIPTION
 *      This function issues an OTA Reset.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void issueOTAReset(timer_id tid)
{
    if (ota_rst_tid == tid)
    {
        ota_rst_tid = TIMER_INVALID;

        /* Issue OTA Reset. */
        OtaReset();
    }
}
#endif /* ENABLE_FIRMWARE_MODEL */

#ifdef DEBUG_ENABLE
/*----------------------------------------------------------------------------*
 *  NAME
 *      printInDecimal
 *
 *  DESCRIPTION
 *      This function prints an UNSIGNED integer in decimal.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void printInDecimal(uint32 val)
{
    if(val >= 10)
    {
        printInDecimal(val/10);
    }
    DebugWriteChar(('0' + (val%10)));
}
#endif /* DEBUG_ENABLE */

/*----------------------------------------------------------------------------*
 *  NAME
 *      readSensorDataFromNVM
 *
 *  DESCRIPTION
 *      This function reads sensor data from NVM into state variable.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void readSensorDataFromNVM(uint16 idx)
{
    Nvm_Read((uint16 *)sensor_state[idx].value, sizeof(uint16),
                                      (GET_DEVICE_NVM_OFFSET(idx)));
}
//#if 0
static uint16 readDevIDFromNVM(uint16 idx)
{   uint16 devid=0;
    Nvm_Read(&devid, sizeof(uint16),/*20160531*/
              NVM_SENSORS_DEVID_OFFSET+idx);
    return devid;
}
//#endif
/*----------------------------------------------------------------------------*
 *  NAME
 *      writeSensorDataToNVM
 *
 *  DESCRIPTION
 *      This function writes sensor data from state variable into NVM.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void writeSensorDataToNVM(uint16 idx)
{
    Nvm_Write((uint16 *)sensor_state[idx].value, 
              sizeof(uint16),
              (GET_DEVICE_NVM_OFFSET(idx)));
}

#ifdef ENABLE_ACK_MODE
/*----------------------------------------------------------------------------*
 *  NAME
 *      sendValueAck
 *
 *  DESCRIPTION
 *      This function sends the sensor value acknowledgement message back to the
 *      device.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void sendValueAck(void)
{
    uint16 index;
    sensor_type_t type1[2] = {sensor_type_internal_air_temperature,
                              sensor_type_internal_humidity                              
                             };//1,4,
    sensor_type_t type2[2] = {sensor_type_desired_air_temperature,
                              sensor_type_barometric_pressure
                             };//3,58
    /*
    sensor_type_t type[4] = {sensor_type_internal_air_temperature,
                              sensor_type_desired_air_temperature,
                              sensor_type_internal_humidity,
                              sensor_type_barometric_pressure
                             };//1,3,4,58
    */
    for(index = 0; index <= MAX_ACK_DEVICES; index++)
    {
        if(sensor_dev_ack[index].dev_id != CSR_MESH_BROADCAST_ID)
        {
            /* Retransmitting the same message on every transmission interval 
             * can be configured to increase the possibility of the msg to reach 
             * to the scanning devices with more robustness as they are scanning 
             * at low duty cycles.
             */
            SensorValue(sensor_dev_ack[index].dev_id, 
                        type1,
                        2,
                        sensor_dev_ack[index].tid,
                        &g_heater_app_data.sensor_data);
            SensorValue(sensor_dev_ack[index].dev_id, 
                        type2,
                        2,
                        sensor_dev_ack[index].tid,
                        &g_heater_app_data.sensor_data);
            /*SensorValue(sensor_dev_ack[index].dev_id, 
                        type,
                        4,
                        sensor_dev_ack[index].tid,
                        &g_heater_app_data.sensor_data);*/
            
            DEBUG_STR("Sends sensor values to the destination device.\r\n");
        }
    }
    DEBUG_STR(" Acknowledge AIR HUMI : ");
    printInDecimal(current_air_humi);DEBUG_STR(" %\r\n");

    DEBUG_STR(" Acknowledge AIR TEMP : ");
    printInDecimal(current_air_temp);DEBUG_STR(" ℃\r\n");   
    
    DEBUG_STR(" Acknowledge AIR PRESS : ");
    printInDecimal(current_air_press);DEBUG_STR(" Pa\r\n");
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      retransmitIntervalTimerHandler
 *
 *  DESCRIPTION
 *      This function expires when the next message needs to be transmitted 
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void retransmitIntervalTimerHandler(timer_id tid)
{
    if (tid == retransmit_tid)
    {   
        TimerDelete(retransmit_tid);
        retransmit_tid = TIMER_INVALID;

        /* transmit the pending message to all the groups*/
        sendValueAck();

        ack_retransmit_count --;

        /* start a timer to send the broadcast ack data */
        startRetransmitTimer();
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      startRetransmitTimer
 *
 *  DESCRIPTION
 *      This function starts the broadcast timer for retransmission.
 *
 *  RETURNS
 *      None
 *
 *----------------------------------------------------------------------------*/
static void startRetransmitTimer(void)
{
    if(retransmit_tid == TIMER_INVALID && ack_retransmit_count > 0)
    {
        retransmit_tid = TimerCreate(RETRANSMIT_INTERVAL, TRUE,
                                     retransmitIntervalTimerHandler);
    }

    /* reset the device id and the tid in the device list as we have sent all
     * the ack to the devices
     */
    if(ack_retransmit_count == 0)
    {
        resetDeviceList();
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      addDeviceToSensorList
 *
 *  DESCRIPTION
 *      This function adds the device id and the transaction id onto the 
 *      list.
 *
 *  RETURNS
 *      None
 *
 *----------------------------------------------------------------------------*/
static void addDeviceToSensorList(uint16 dev_id, uint8 tid)
{
    uint8 index;
    bool dev_found = FALSE;

    /* Check whether the device is already present in the list as we are 
     * sending the acknowledgements in which case we can refresh the latest
     * tid for that device.
     */
    for(index = 0; index < MAX_ACK_DEVICES; index++)
    {
        if(sensor_dev_ack[index].dev_id == dev_id)
        {
            dev_found = TRUE;
            sensor_dev_ack[index].tid = tid;
            break;
        }
    }
    /* If the device is not found then add the device onto the device list db */
    if(!dev_found)
    {
        for(index = 0; index < MAX_ACK_DEVICES; index++)
        {
            if(sensor_dev_ack[index].dev_id == CSR_MESH_BROADCAST_ID)
            {
                sensor_dev_ack[index].dev_id = dev_id;
                sensor_dev_ack[index].tid = tid;
                break;
            }
        }
        /* If the device list is full then just replace the oldest device id as
         * we would have sent the maximum acknowledgements to the first dev
         * added in the list.
         */
        if(index == MAX_ACK_DEVICES - 1)
        {
            sensor_dev_ack[0].dev_id = dev_id;
            sensor_dev_ack[0].tid = tid;
        }
    }
    ack_retransmit_count = MAX_RETRANSMIT_COUNT;

    /* start a timer to send the broadcast ack data */
    TimerDelete(retransmit_tid);
    retransmit_tid = TIMER_INVALID;
    startRetransmitTimer();
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      resetDeviceList
 *
 *  DESCRIPTION
 *      The function resets the device id and the ack flag of complete db
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void resetDeviceList(void)
{
    uint16 idx;

    for(idx=0; idx < MAX_ACK_DEVICES; idx++)
    {
        sensor_dev_ack[idx].dev_id = CSR_MESH_BROADCAST_ID;
        sensor_dev_ack[idx].tid = 0;
    }
}
#endif /* ENABLE_ACK_MODE */
/*----------------------------------------------------------------------------*
 *  NAME
 *      updateHeaterStatus
 *
 *  DESCRIPTION
 *      This function updates the Heater Status from ON or OFF
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
#if 0

static void updateHeaterStatus(void)//状态切换
{
    if( current_desired_air_temp > current_air_temp )
    {
        if( g_heater_app_data.status == heater_off )
        {
            /* Print only if the status changed */
            DEBUG_STR("HEATER ON\r\n");
            g_heater_app_data.status = heater_on;
        }
        /* Indicate the Heater ON status by glowing red LED */
        IOTLightControlDeviceSetColor(255,0,0);
    }
    else
    {
        if( g_heater_app_data.status == heater_on )
        {
            /* Print if the status changed */
            DEBUG_STR("HEATER OFF\r\n");
            g_heater_app_data.status = heater_off;
        }
        /* Turn off the red LED to indicate Heating status */
        IOTLightControlDevicePower(FALSE);
    }
}
#endif
/*----------------------------------------------------------------------------*
 *  NAME
 *      readCurrentTempFromGroup
 *
 *  DESCRIPTION
 *      This function reads the current temperature from the supported group
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void readCurrentTempFromGroup(void)
{   
    sensor_type_t type1[2] = {sensor_type_internal_air_temperature,
                              sensor_type_internal_humidity                              
                             };//1,4
    sensor_type_t type2[2] = {sensor_type_desired_air_temperature,
                              sensor_type_barometric_pressure
                             };//3,58 
    
    /*sensor_type_t type[4] = {sensor_type_internal_air_temperature,
                              sensor_type_desired_air_temperature,
                              sensor_type_internal_humidity,
                              sensor_type_barometric_pressure
                             };//1,3,4,58
    */uint16 index;

    for(index = 0; index < NUM_SENSOR_MODEL_GROUPS; index++)
    {
        if(sensor_model_groups[index] != 0)
        {   
            SensorReadValue(sensor_model_groups[index],//group_id 
                            type1,
                            2);
            SensorReadValue(sensor_model_groups[index],//group_id 
                            type2,
                            2);/*
            SensorReadValue(sensor_model_groups[index],//group_id 
                            type,
                            4);*/      
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      startReadValueTimer
 *
 *  DESCRIPTION
 *      This function starts the broadcast timer for retransmission.
 *
 *  RETURNS
 *      None
 *
 *----------------------------------------------------------------------------*/
static void startReadValueTimer(void)
{
    if(read_val_tid == TIMER_INVALID && read_value_transmit_count > 0)
    {
        read_val_tid = TimerCreate(RETRANSMIT_INTERVAL, TRUE,
                                   readValTimerHandler);
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      readValTimerHandler
 *
 *  DESCRIPTION
 *      This function expires when the next message needs to be transmitted 
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void readValTimerHandler(timer_id tid)
{
        TimerDelete(read_val_tid);
        read_val_tid = TIMER_INVALID;

        /* Read the internal and desired temp from the group */
        readCurrentTempFromGroup();

        read_value_transmit_count --;

        /* start a timer to read the temp from the group */
        startReadValueTimer();
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      IsHeaterConfigured
 *
 *  DESCRIPTION
 *      This below function returns whether the heater is configured or not
 *
 *  RETURNS/MODIFIES
 *      TRUE if the heater has been grouped otherwise returns FALSE
 *
 *----------------------------------------------------------------------------*/
static bool IsHeaterConfigured(void)
{
    uint16 index;

    for(index = 0; index < NUM_SENSOR_MODEL_GROUPS; index++)
    {
        if(sensor_model_groups[index] != 0)
        {
            return TRUE;
        }
    }
    return FALSE;
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      deviceIdAdvertTimeoutHandler
 *
 *  DESCRIPTION
 *      This function handles the Device ID advertise timer event.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void deviceIdAdvertTimeoutHandler(timer_id tid)
{
    if(tid == g_heater_app_data.mesh_device_id_advert_tid)
    {
        TimerDelete(g_heater_app_data.mesh_device_id_advert_tid);
        g_heater_app_data.mesh_device_id_advert_tid = TIMER_INVALID;
        /* Start the timer only if the device is not associated */
        if(g_heater_app_data.assoc_state == app_state_not_associated)
        {
            /* Generate a random delay between 0 to 4095 ms */
            uint32 random_delay = ((uint32)(Random16() & 0x0FFF))*(MILLISECOND);

            if(send_dev_appearance == FALSE)
            {
                /* Send the device ID advertisements */
                CsrMeshAssociateToANetwork();
                send_dev_appearance = TRUE;
            }
            else
            {
                /* Send the device appearance */
                CsrMeshAdvertiseDeviceAppearance(&device_appearance, 
                                                 short_name, 
                                                 sizeof(short_name));
                send_dev_appearance = FALSE;
            }

            g_heater_app_data.mesh_device_id_advert_tid = TimerCreate(
                                       DEVICE_ID_ADVERT_TIMER_ID + random_delay,
                                       TRUE,
                                       deviceIdAdvertTimeoutHandler);
        }
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      appDataInit
 *
 *  DESCRIPTION
 *      This function is called to initialise CSRmesh Heater application data 
 *      structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void appDataInit(void)
{
    /* Reset/Delete all the timers */
    TimerDelete(g_heater_app_data.gatt_data.app_tid);
    g_heater_app_data.gatt_data.app_tid = TIMER_INVALID;

    TimerDelete(g_heater_app_data.gatt_data.con_param_update_tid);
    g_heater_app_data.gatt_data.con_param_update_tid = TIMER_INVALID;
    g_heater_app_data.gatt_data.cpu_timer_value = 0;

    g_heater_app_data.gatt_data.st_ucid = GATT_INVALID_UCID;

    g_heater_app_data.gatt_data.advert_timer_value = 0;

    /* Reset the connection parameter variables. */
    g_heater_app_data.gatt_data.conn_interval = 0;
    g_heater_app_data.gatt_data.conn_latency = 0;
    g_heater_app_data.gatt_data.conn_timeout = 0;

    /* Initialise GAP Data structure */
    GapDataInit();

    /* Initialise the Mesh Control Service Data Structure */
    MeshControlServiceDataInit();

#ifdef ENABLE_GATT_OTA_SERVICE
    /* Initialise GATT Data structure */
    GattDataInit();

    /* Initialise the CSR OTA Service Data */
    OtaDataInit();
#endif /* ENABLE_GATT_OTA_SERVICE */
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      initiateAssociation
 *
 *  DESCRIPTION
 *      This function starts timer to send CSRmesh Association Messages
 *      and also gives visual indication that Heater is not associated.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void initiateAssociation(void)
{
    /* Blink light to indicate that it is not associated */
    IOTLightControlDeviceBlink(0, 0, 127, 32, 32);

    /* Send the device ID advertisements */
    CsrMeshAssociateToANetwork();
    send_dev_appearance = TRUE;

    /* Start a timer to send Device ID messages periodically to get
     * associated to a network
     */
    g_heater_app_data.mesh_device_id_advert_tid = TimerCreate(
                                               DEVICE_ID_ADVERT_TIMER_ID,
                                               TRUE,
                                               deviceIdAdvertTimeoutHandler);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      readPersistentStore
 *
 *  DESCRIPTION
 *      This function is used to Initialise and read NVM data
 *
 *  RETURNS/MODIFIES
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void readPersistentStore(void)
{
    /* NVM offset for supported services */
    uint16 nvm_offset = 0;
    uint16 nvm_sanity = 0xffff;
    bool nvm_start_fresh = FALSE;
    uint16 app_nvm_version;
    uint16 i;

    nvm_offset = NVM_MAX_APP_MEMORY_WORDS;

    /* Read the Application NVM version */
    Nvm_Read(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);

    if( app_nvm_version != APP_NVM_VERSION )    //first download
    {
        /* The NVM structure could have changed
         * with a new version of the application, due to NVM values getting
         * removed or added. Currently this application clears all the 
         * application and CSRmesh NVM values and writes to NVM
         */
#ifdef NVM_TYPE_EEPROM
        uint16 eeprom_erase = 0xFFFF;
        /* Erase a block in EEPROM to remove all sanity words */
        for (i = 0; i < 128; i++)
        {
            Nvm_Write(&eeprom_erase, 0x1, i);
        }
#elif NVM_TYPE_FLASH
        NvmErase(FALSE);
#endif /* NVM_TYPE_EEPROM */

        /* Save new version of the NVM */
        app_nvm_version = APP_NVM_VERSION;
        Nvm_Write(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);
    }
    
    /* Read the NVM sanity word to check if the NVM validity */
    Nvm_Read(&nvm_sanity, sizeof(nvm_sanity),
             NVM_OFFSET_SANITY_WORD);  
    
    /* Initialise the paired flag to false */
    g_heater_app_data.gatt_data.paired = FALSE;

    if(nvm_sanity == NVM_SANITY_MAGIC)  //second:power-down/reset
    {
        /* Read association state from NVM */
        Nvm_Read((uint16 *)&g_heater_app_data.assoc_state,
                sizeof(g_heater_app_data.assoc_state),
                NVM_OFFSET_ASSOCIATION_STATE);

        /* Read Bearer Model Data from NVM */
        Nvm_Read((uint16 *)&g_heater_app_data.bearer_data,
                 sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

        /* Read sensor State Data. */
        for (i = 0; i < NUM_SENSORS_SUPPORTED; i++)
        {
            readSensorDataFromNVM(i);
        }

        /* Read assigned Groups IDs for sensor model from NVM */
        Nvm_Read((uint16 *)sensor_model_groups, sizeof(sensor_model_groups),
                NVM_OFFSET_SENSOR_MODEL_GROUPS);

        /* Read assigned Group IDs for Attention model from NVM */
        Nvm_Read((uint16 *)attention_model_groups, 
                                            sizeof(attention_model_groups),
                                            NVM_OFFSET_ATTENTION_MODEL_GROUPS);
#ifdef ENABLE_DATA_MODEL
        /* Read assigned Group IDs for data stream model from NVM */
        Nvm_Read((uint16 *)data_model_groups, sizeof(data_model_groups),
                                            NVM_OFFSET_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

        /* If NVM in use, read device name and length from NVM */
        GapReadDataFromNVM(&nvm_offset);

    }
    else /* NVM Sanity check failed means either the device is being brought up
          * for the first time or memory has got corrupted in which case
          * discard the data and start fresh.
          */
    {
        /* Read Configuration flags from User CS Key */
        uint16 cskey_flags = CSReadUserKey(CSKEY_INDEX_USER_FLAGS);
        nvm_start_fresh = TRUE;

        nvm_sanity = NVM_SANITY_MAGIC;

        /* Write NVM Sanity word to the NVM */
        Nvm_Write(&nvm_sanity, sizeof(nvm_sanity),
                  NVM_OFFSET_SANITY_WORD);
        /*20160615*/
        endOfDevidNvmAddr=0;
        Nvm_Write(&endOfDevidNvmAddr,1,NVM_DEVID_ADDR_INDEX_OFFSET);
        currentHeaterStatus=HEATER_STATUS_INIT;//20160616       
        Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);

        if (cskey_flags & RANDOM_UUID_ENABLE_MASK)
        {
            /* The flag is set so generate a random UUID NVM */
            for( i = 0 ; i < 8 ; i++)
            {
                g_node_data.device_uuid.uuid[i] = Random16();//随机产生
            }
        }

        /* Update Bearer Model Data from CSKey flags for the first time. */
        g_heater_app_data.bearer_data.bearerPromiscuous = 0x0000;
        g_heater_app_data.bearer_data.bearerEnabled     = BLE_BEARER_MASK;
        g_heater_app_data.bearer_data.bearerRelayActive = 0x0000;

        if (cskey_flags & RELAY_ENABLE_MASK)
        {
            g_heater_app_data.bearer_data.bearerRelayActive |= BLE_BEARER_MASK;
            g_heater_app_data.bearer_data.bearerPromiscuous |= BLE_BEARER_MASK;
        }

        if (cskey_flags & BRIDGE_ENABLE_MASK)
        {
            g_heater_app_data.bearer_data.bearerEnabled     
                                                |= BLE_GATT_SERVER_BEARER_MASK;
            g_heater_app_data.bearer_data.bearerRelayActive 
                                                |= BLE_GATT_SERVER_BEARER_MASK;
            g_heater_app_data.bearer_data.bearerPromiscuous 
                                                |= BLE_GATT_SERVER_BEARER_MASK;
        }
        //DEBUG_STR("SENSORS_DB_OFFSET:");DEBUG_U16(NVM_ALL_SENSORS_DB);
        //DEBUG_STR("\r\nASSOCIOATION_STATE:");DEBUG_U16(NVM_OFFSET_ASSOCIATION_STATE);
        /*uint16 testnvm0=0x7890;
        uint16 testnvm1=0x5678;
        Nvm_Write(&testnvm0,1, NVM_SENSORS_DEVID_OFFSET);
        Nvm_Write(&testnvm1,1, NVM_ALL_SENSORS_DB);*/
        
        /* Update Bearer Model Data to NVM */
        Nvm_Write((uint16 *)&g_heater_app_data.bearer_data,
                  sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

        /* Write to NVM */
        Nvm_Write(g_node_data.device_uuid.uuid, DEVICE_UUID_SIZE_WORDS,
                  NVM_OFFSET_DEVICE_UUID);

#ifdef USE_AUTHORIZATION_CODE
        /* Write Authorization Code to NVM */
        Nvm_Write(g_node_data.auth_code.auth_code,DEVICE_AUTHCODE_SIZE_IN_WORDS,
                  NVM_OFFSET_DEVICE_AUTHCODE);
#endif /* USE_AUTHORIZATION_CODE */

        /* The device will not be associated as it is coming up for the
         * first time
         */
        g_heater_app_data.assoc_state = app_state_not_associated;

        /* Write association state to NVM */
        Nvm_Write((uint16 *)&g_heater_app_data.assoc_state,
                 sizeof(g_heater_app_data.assoc_state),
                 NVM_OFFSET_ASSOCIATION_STATE);

        /* Write sensor State data to NVM. */
        for (i = 0; i < NUM_SENSORS_SUPPORTED; i++)
        {
            writeSensorDataToNVM(i);
        }

        MemSet(sensor_model_groups, 0x0000, sizeof(sensor_model_groups)); 

        /* Write assigned Groups IDs for sensor model from NVM */
        Nvm_Write(sensor_model_groups, sizeof(sensor_model_groups),
                                             NVM_OFFSET_SENSOR_MODEL_GROUPS);

        MemSet(attention_model_groups, 0x0000, sizeof(attention_model_groups)); 

        /* Write assigned Groups IDs for Attention model from NVM */
        Nvm_Write(attention_model_groups, sizeof(attention_model_groups),
                                             NVM_OFFSET_ATTENTION_MODEL_GROUPS);

/* Data stream model */
#ifdef ENABLE_DATA_MODEL
        MemSet(data_model_groups, 0x0000, sizeof(data_model_groups));
        Nvm_Write((uint16 *)data_model_groups, sizeof(data_model_groups),
                                        NVM_OFFSET_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

        /* If fresh NVM, write device name and length to NVM for the
         * first time.
         */
        GapInitWriteDataToNVM(&nvm_offset);
    }

    
    Nvm_Read((uint16 *)&g_heater_app_data.assoc_state,1, 
                                                  NVM_OFFSET_ASSOCIATION_STATE);

    /* Read the UUID from NVM */
    Nvm_Read(g_node_data.device_uuid.uuid, DEVICE_UUID_SIZE_WORDS, 
                                                        NVM_OFFSET_DEVICE_UUID);

#ifdef USE_AUTHORIZATION_CODE
    /* Read Authorization Code from NVM */
    Nvm_Read(g_node_data.auth_code.auth_code, DEVICE_AUTHCODE_SIZE_IN_WORDS,
                                                    NVM_OFFSET_DEVICE_AUTHCODE);
    g_node_data.use_authorisation = TRUE;
#endif /* USE_AUTHORIZATION_CODE */

    if(g_heater_app_data.assoc_state == app_state_associated)
    {
        g_node_data.associated = TRUE;
        /* Read node data from NVM */
        /* Network key */
        Nvm_Read(g_node_data.nw_key.key, sizeof(CSR_MESH_NETWORK_KEY_T),
                                                        NVM_OFFSET_NETWORK_KEY);
        /* Device ID */
        Nvm_Read(&g_node_data.device_id, 1, NVM_OFFSET_DEVICE_ID);
        /* Sequence Number */
        Nvm_Read((uint16 *)&g_node_data.seq_number, 2,
                                                    NVM_OFFSET_SEQUENCE_NUMBER);
        /* Last ETag */
        Nvm_Read(g_node_data.device_ETag.ETag, sizeof(CSR_MESH_ETAG_T),
                                                    NVM_OFFSET_DEVICE_ETAG);
        /* As association is already complete set LE bearer to non-promiscuous*/
        g_heater_app_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
    }

}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      attnTimerHandler
 *
 *  DESCRIPTION
 *      This function handles Attention time-out.
 *
 *  RETURNS/MODIFIES
 *      Nothing
 *
 *----------------------------------------------------------------------------*/
static void attnTimerHandler(timer_id tid)
{
    if (attn_tid == tid)
    {
        attn_tid = TIMER_INVALID;

        /* Set back the scan to low duty cycle only if the device has
         * already been grouped.
         */
        EnableHighDutyScanMode(TRUE); //20160808 change from false to true

        /* Restore Light State */
        if(g_heater_app_data.assoc_state == app_state_associated)
        {
            if( g_heater_app_data.status == heater_off)
            {
                /* Heating is OFF turn off red LED */
                IOTLightControlDeviceSetColor(0,0,0);

                IOTLightControlDevicePower(FALSE);
            }
            else
            {
                /* Heating is ON so turn on red LED */
                IOTLightControlDeviceSetColor(255,0,0);
            }
        }
        else
        {
            /* Restart association blink */
            IOTLightControlDeviceBlink(0, 0, 127, 32, 32);
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      requestConnParamUpdate
 *
 *  DESCRIPTION
 *      This function is used to send L2CAP_CONNECTION_PARAMETER_UPDATE_REQUEST
 *      to the remote device when an earlier sent request had failed.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void requestConnParamUpdate(timer_id tid)
{
    /* Application specific preferred parameters */
    ble_con_params app_pref_conn_param;

    if(g_heater_app_data.gatt_data.con_param_update_tid == tid)
    {
        g_heater_app_data.gatt_data.con_param_update_tid = TIMER_INVALID;
        g_heater_app_data.gatt_data.cpu_timer_value = 0;

        /*Handling signal as per current state */
        switch(g_heater_app_data.state)
        {
            case app_state_connected:
            {
                /* Increment the count for Connection Parameter Update
                 * requests
                 */
                ++ g_heater_app_data.gatt_data.num_conn_update_req;

                /* If it is first or second request, preferred connection
                 * parameters should be request
                 */
                if(g_heater_app_data.gatt_data.num_conn_update_req == 1 ||
                   g_heater_app_data.gatt_data.num_conn_update_req == 2)
                {
                    app_pref_conn_param.con_max_interval =
                                                PREFERRED_MAX_CON_INTERVAL;
                    app_pref_conn_param.con_min_interval =
                                                PREFERRED_MIN_CON_INTERVAL;
                    app_pref_conn_param.con_slave_latency =
                                                PREFERRED_SLAVE_LATENCY;
                    app_pref_conn_param.con_super_timeout =
                                                PREFERRED_SUPERVISION_TIMEOUT;
                }
                /* If it is 3rd or 4th request, APPLE compliant parameters
                 * should be requested.
                 */
                else if(g_heater_app_data.gatt_data.num_conn_update_req == 3 ||
                        g_heater_app_data.gatt_data.num_conn_update_req == 4)
                {
                    app_pref_conn_param.con_max_interval =
                                                APPLE_MAX_CON_INTERVAL;
                    app_pref_conn_param.con_min_interval =
                                                APPLE_MIN_CON_INTERVAL;
                    app_pref_conn_param.con_slave_latency =
                                                APPLE_SLAVE_LATENCY;
                    app_pref_conn_param.con_super_timeout =
                                                APPLE_SUPERVISION_TIMEOUT;
                }

                /* Send Connection Parameter Update request using application
                 * specific preferred connection parameters
                 */

                if(LsConnectionParamUpdateReq(
                                    &g_heater_app_data.gatt_data.con_bd_addr,
                                    &app_pref_conn_param) != ls_err_none)
                {
                    ReportPanic(app_panic_con_param_update);
                }
            }
            break;

            default:
                /* Ignore in other states */
            break;
        }

    } /* Else ignore the timer */

}


/*-----------------------------------------------------------------------------*
 *  NAME
 *      handleGapCppTimerExpiry
 *
 *  DESCRIPTION
 *      This function handles the expiry of TGAP(conn_pause_peripheral) timer.
 *      It starts the TGAP(conn_pause_central) timer, during which, if no activ-
 *      -ity is detected from the central device, a connection parameter update
 *      request is sent.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleGapCppTimerExpiry(timer_id tid)
{
    if(g_heater_app_data.gatt_data.con_param_update_tid == tid)
    {
        g_heater_app_data.gatt_data.con_param_update_tid =
                           TimerCreate(TGAP_CPC_PERIOD, TRUE,
                                       requestConnParamUpdate);
        g_heater_app_data.gatt_data.cpu_timer_value = TGAP_CPC_PERIOD;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appAdvertisingExit
 *
 *  DESCRIPTION
 *      This function is called while exiting app_state_advertising state.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void appAdvertisingExit(void)
{
    /* Stop on-going advertisements */
    GattStopAdverts();
        /* Cancel advertisement timer */
        TimerDelete(g_heater_app_data.gatt_data.app_tid);
        g_heater_app_data.gatt_data.app_tid = TIMER_INVALID;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAddDBCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_ADD_DB_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattAddDBCfm(GATT_ADD_DB_CFM_T *p_event_data)
{
    switch(g_heater_app_data.state)
    {
        case app_state_init:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* If GATT bearer is enabled move to advertisement state 
                 * otherwise move to idle state. The advertisement would be 
                 * triggerred once the GATT bearer is enabled again.
                 */
                if(g_heater_app_data.bearer_data.bearerEnabled & 
                                                BLE_GATT_SERVER_BEARER_MASK)
                {
                    AppSetState(app_state_advertising);
                }
                else
                {
                    AppSetState(app_state_idle);
                }
            }
            else
            {
                /* Don't expect this to happen */
                ReportPanic(app_panic_db_registration);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattCancelConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CANCEL_CONNECT_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattCancelConnectCfm(void)
{
    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_advertising:
        {
            /* Do nothing here */
        }
        break;

        case app_state_connected:
            /* The CSRmesh could have been sending data on
             * advertisements so do not panic
             */
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*---------------------------------------------------------------------------
 *
 *  NAME
 *      handleSignalLmEvConnectionComplete
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_COMPLETE.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void handleSignalLmEvConnectionComplete(
                                     LM_EV_CONNECTION_COMPLETE_T *p_event_data)
{
    /* Store the connection parameters. */
    g_heater_app_data.gatt_data.conn_interval = 
                                        p_event_data->data.conn_interval;
    g_heater_app_data.gatt_data.conn_latency  = 
                                        p_event_data->data.conn_latency;
    g_heater_app_data.gatt_data.conn_timeout  = 
                                        p_event_data->data.supervision_timeout;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattConnectCfm
 *
 *  DESCRIPTION
 *      This function handles the signal GATT_CONNECT_CFM
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattConnectCfm(GATT_CONNECT_CFM_T* p_event_data)
{
    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_advertising:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* Store received UCID */
                g_heater_app_data.gatt_data.st_ucid = p_event_data->cid;

                /* Store connected BD Address */
                g_heater_app_data.gatt_data.con_bd_addr = p_event_data->bd_addr;

                /* Store the bearer relay active and promiscuous onto global 
                 * as they need to be reverted after disconnection.
                 */
                bearer_relay_active = 
                    g_heater_app_data.bearer_data.bearerRelayActive;

                bearer_promiscuous = 
                    g_heater_app_data.bearer_data.bearerPromiscuous;

                g_heater_app_data.bearer_data.bearerRelayActive = 
                    BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK;

                g_heater_app_data.bearer_data.bearerPromiscuous = 
                    BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK;

                /* When device is connected as bridge enable the BLE and GATT 
                 * bearer relays otherwise mesh messages sent by control device 
                 * over GATT will not be forwarded on mesh.
                 */
                CsrMeshRelayEnable(
                              g_heater_app_data.bearer_data.bearerRelayActive);

                /* Enable the promiscuous mode on both the bearers which makes
                 * sure the connected control device can control any mesh n/w.
                 */
                CsrMeshEnablePromiscuousMode(
                              g_heater_app_data.bearer_data.bearerPromiscuous);

                /* Enter connected state */
                AppSetState(app_state_connected);

                /* Inform CSRmesh that we are connected now */
                CsrMeshHandleDataInConnection(
                                g_heater_app_data.gatt_data.st_ucid,
                                g_heater_app_data.gatt_data.conn_interval);

                /* Since CSRmesh Heater application does not mandate
                 * encryption requirement on its characteristics, so the
                 * remote master may or may not encrypt the link. Start a
                 * timer  here to give remote master some time to encrypt
                 * the link and on expiry of that timer, send a connection
                 * parameter update request to remote side.
                 */

                /* Don't request security as this causes connection issues
                 * with Android 4.4
                 *
                 *  SMRequestSecurityLevel(
                                    &g_heater_app_data.gatt_data.con_bd_addr);
                 */

                /* If the current connection parameters being used don't
                 * comply with the application's preferred connection
                 * parameters and the timer is not running and, start timer
                 * to trigger Connection Parameter Update procedure
                 */
                if((g_heater_app_data.gatt_data.con_param_update_tid ==
                                                        TIMER_INVALID) &&
                   (g_heater_app_data.gatt_data.conn_interval <
                                             PREFERRED_MIN_CON_INTERVAL ||
                    g_heater_app_data.gatt_data.conn_interval >
                                             PREFERRED_MAX_CON_INTERVAL
#if PREFERRED_SLAVE_LATENCY
                    || g_heater_app_data.gatt_data.conn_latency <
                                             PREFERRED_SLAVE_LATENCY
#endif
                   )
                  )
                {
                    /* Set the number of conn update attempts to zero */
                    g_heater_app_data.gatt_data.num_conn_update_req = 0;

                    /* The application first starts a timer of
                     * TGAP_CPP_PERIOD. During this time, the application
                     * waits for the peer device to do the database
                     * discovery procedure. After expiry of this timer, the
                     * application starts one more timer of period
                     * TGAP_CPC_PERIOD. If the application receives any
                     * GATT_ACCESS_IND during this time, it assumes that
                     * the peer device is still doing device database
                     * discovery procedure or some other configuration and
                     * it should not update the parameters, so it restarts
                     * the TGAP_CPC_PERIOD timer. If this timer expires, the
                     * application assumes that database discovery procedure
                     * is complete and it initiates the connection parameter
                     * update procedure.
                     */
                    g_heater_app_data.gatt_data.con_param_update_tid =
                                      TimerCreate(TGAP_CPP_PERIOD, TRUE,
                                                  handleGapCppTimerExpiry);
                    g_heater_app_data.gatt_data.cpu_timer_value =
                                                        TGAP_CPP_PERIOD;
                }
                  /* Else at the expiry of timer Connection parameter
                   * update procedure will get triggered
                   */
                }
            else
            {
                /* If GATT bearer is enabled move to advertisement state 
                 * otherwise move to idle state. The advertisement would be 
                 * triggerred once the GATT bearer is enabled again.
                 */
                if(g_heater_app_data.bearer_data.bearerEnabled & 
                                                BLE_GATT_SERVER_BEARER_MASK)
                {
                    AppSetState(app_state_advertising);
                }
                else
                {
                    AppSetState(app_state_idle);
                }
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalSmSimplePairingCompleteInd
 *
 *  DESCRIPTION
 *      This function handles the signal SM_SIMPLE_PAIRING_COMPLETE_IND
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalSmSimplePairingCompleteInd(
                                 SM_SIMPLE_PAIRING_COMPLETE_IND_T *p_event_data)
{
    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_connected:
        {
            if(p_event_data->status == sys_status_success)
            {
                /* Device is paired */
                g_heater_app_data.gatt_data.paired = TRUE;
            }
            else
            {
                /* Pairing has failed.disconnect the link.*/
                AppSetState(app_state_disconnecting);
            }
        }
        break;

        default:
            /* Firmware may send this signal after disconnection. So don't
             * panic but ignore this signal.
             */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsConnParamUpdateCfm
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_CFM.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLsConnParamUpdateCfm(
                            LS_CONNECTION_PARAM_UPDATE_CFM_T *p_event_data)
{
    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_connected:
        {
            /* Received in response to the L2CAP_CONNECTION_PARAMETER_UPDATE
              * request sent from the slave after encryption is enabled. If
              * the request has failed, the device should again send the same
              * request only after Tgap(conn_param_timeout). Refer
              * Bluetooth 4.0 spec Vol 3 Part C, Section 9.3.9 and profile spec.
              */
            if ((p_event_data->status != ls_err_none) &&
                (g_heater_app_data.gatt_data.num_conn_update_req <
                                        MAX_NUM_CONN_PARAM_UPDATE_REQS))
            {
                /* Delete timer if running */
                TimerDelete(g_heater_app_data.gatt_data.con_param_update_tid);

                g_heater_app_data.gatt_data.con_param_update_tid =
                                 TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                             TRUE, requestConnParamUpdate);
                g_heater_app_data.gatt_data.cpu_timer_value =
                                             GAP_CONN_PARAM_TIMEOUT;
            }
        }
        break;

        default:
            /* Control should never come here but in one of the odd cases when 
             * the master is disconnecting during the connection param update 
             * the above msg is received after the disconnection complete from  
             * the firmware. Hence ignoring the signal is other states too.
             */
            /* ReportPanic(app_panic_invalid_state); */
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmConnectionUpdate
 *
 *  DESCRIPTION
 *      This function handles the signal LM_EV_CONNECTION_UPDATE.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLmConnectionUpdate(
                                   LM_EV_CONNECTION_UPDATE_T* p_event_data)
{
    switch(g_heater_app_data.state)
    {
        case app_state_connected:
        case app_state_disconnecting:
        {
            /* Store the new connection parameters. */
            g_heater_app_data.gatt_data.conn_interval =
                                            p_event_data->data.conn_interval;
            g_heater_app_data.gatt_data.conn_latency =
                                            p_event_data->data.conn_latency;
            g_heater_app_data.gatt_data.conn_timeout =
                                        p_event_data->data.supervision_timeout;

            CsrMeshHandleDataInConnection(g_heater_app_data.gatt_data.st_ucid,
                                    g_heater_app_data.gatt_data.conn_interval);

            DEBUG_STR("Parameter Update Complete: ");
            DEBUG_U16(g_heater_app_data.gatt_data.conn_interval);
            DEBUG_STR("\r\n");
        }
        break;

        default:
            /* Control should never come here but in one of the odd cases when 
             * the master is disconnecting during the connection param update 
             * the above msg is received after the disconnection complete from  
             * the firmware. Hence ignoring the signal is other states too.
             */
            /* ReportPanic(app_panic_invalid_state); */
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLsConnParamUpdateInd
 *
 *  DESCRIPTION
 *      This function handles the signal LS_CONNECTION_PARAM_UPDATE_IND.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLsConnParamUpdateInd(
                                 LS_CONNECTION_PARAM_UPDATE_IND_T *p_event_data)
{
    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_connected:
        {
            /* Delete timer if running */
            TimerDelete(g_heater_app_data.gatt_data.con_param_update_tid);
            g_heater_app_data.gatt_data.con_param_update_tid = TIMER_INVALID;
            g_heater_app_data.gatt_data.cpu_timer_value = 0;

            /* The application had already received the new connection
             * parameters while handling event LM_EV_CONNECTION_UPDATE.
             * Check if new parameters comply with application preferred
             * parameters. If not, application shall trigger Connection
             * parameter update procedure
             */

            if(g_heater_app_data.gatt_data.conn_interval <
                                                PREFERRED_MIN_CON_INTERVAL ||
               g_heater_app_data.gatt_data.conn_interval >
                                                PREFERRED_MAX_CON_INTERVAL
#if PREFERRED_SLAVE_LATENCY
               || g_heater_app_data.gatt_data.conn_latency <
                                                PREFERRED_SLAVE_LATENCY
#endif
              )
            {
                /* Set the num of conn update attempts to zero */
                g_heater_app_data.gatt_data.num_conn_update_req = 0;

                /* Start timer to trigger Connection Parameter Update
                 * procedure
                 */
                g_heater_app_data.gatt_data.con_param_update_tid =
                                TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                            TRUE, requestConnParamUpdate);
                g_heater_app_data.gatt_data.cpu_timer_value =
                                                        GAP_CONN_PARAM_TIMEOUT;
            }
        }
        break;

        default:
            /* Control should never come here but in one of the odd cases when 
             * the master is disconnecting during the connection param update 
             * the above msg is received after the disconnection complete from  
             * the firmware. Hence ignoring the signal is other states too.
             */
            /* ReportPanic(app_panic_invalid_state); */
        break;
    }

}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalGattAccessInd
 *
 *  DESCRIPTION
 *      This function handles GATT_ACCESS_IND message for attributes
 *      maintained by the application.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalGattAccessInd(GATT_ACCESS_IND_T *p_event_data)
{

    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_connected:
        {
            /* GATT_ACCESS_IND indicates that the central device is still disco-
             * -vering services. So, restart the connection parameter update
             * timer
             */
             if(g_heater_app_data.gatt_data.cpu_timer_value==TGAP_CPC_PERIOD &&
                g_heater_app_data.gatt_data.con_param_update_tid!=TIMER_INVALID)
             {
                TimerDelete(g_heater_app_data.gatt_data.con_param_update_tid);
                g_heater_app_data.gatt_data.con_param_update_tid =
                                    TimerCreate(TGAP_CPC_PERIOD,
                                                TRUE, requestConnParamUpdate);
             }

            /* Received GATT ACCESS IND with write access */
            if(p_event_data->flags & ATT_ACCESS_WRITE)
            {
                /* If only ATT_ACCESS_PERMISSION flag is enabled, then the
                 * firmware is asking the app for permission to go along with
                 * prepare write request from the peer. Allow it.
                 */
                if(((p_event_data->flags) &
                   (ATT_ACCESS_PERMISSION | ATT_ACCESS_WRITE_COMPLETE))
                                                    == ATT_ACCESS_PERMISSION)
                {
                    GattAccessRsp(p_event_data->cid, p_event_data->handle,
                                  sys_status_success, 0, NULL);
                }
                else
                {
                    HandleAccessWrite(p_event_data);
                }
            }
            else if(p_event_data->flags & ATT_ACCESS_WRITE_COMPLETE)
            {
                GattAccessRsp(p_event_data->cid, p_event_data->handle,
                                          sys_status_success, 0, NULL);
            }
            /* Received GATT ACCESS IND with read access */
            else if(p_event_data->flags ==
                                    (ATT_ACCESS_READ | ATT_ACCESS_PERMISSION))
            {
                HandleAccessRead(p_event_data);
            }
            else
            {
                GattAccessRsp(p_event_data->cid, p_event_data->handle,
                              gatt_status_request_not_supported,
                              0, NULL);
            }
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      handleSignalLmDisconnectComplete
 *
 *  DESCRIPTION
 *      This function handles LM Disconnect Complete event which is received
 *      at the completion of disconnect procedure triggered either by the
 *      device or remote host or because of link loss
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void handleSignalLmDisconnectComplete(
                HCI_EV_DATA_DISCONNECT_COMPLETE_T *p_event_data)
{

    /* Reset the connection parameter variables. */
    g_heater_app_data.gatt_data.conn_interval = 0;
    g_heater_app_data.gatt_data.conn_latency = 0;
    g_heater_app_data.gatt_data.conn_timeout = 0;

    CsrMeshHandleDataInConnection(GATT_INVALID_UCID, 0);

    /* Restore the relay and the promiscuous settings to the last set values */
    g_heater_app_data.bearer_data.bearerRelayActive = bearer_relay_active;
    g_heater_app_data.bearer_data.bearerPromiscuous = bearer_promiscuous;

    CsrMeshRelayEnable(g_heater_app_data.bearer_data.bearerRelayActive);
    CsrMeshEnablePromiscuousMode(
                            g_heater_app_data.bearer_data.bearerPromiscuous);


#ifdef ENABLE_GATT_OTA_SERVICE
    if(OtaResetRequired())
    {
        OtaReset();
    }
#endif /* ENABLE_GATT_OTA_SERVICE */

    /*Handling signal as per current state */
    switch(g_heater_app_data.state)
    {
        case app_state_connected:
        case app_state_disconnecting:
        {
            /* Connection is terminated either due to Link Loss or
             * the local host terminated connection. In either case
             * Initialise the app data and go to fast advertising.
             */
            appDataInit();            

            /* If GATT bearer is enabled move to advertisement state 
             * otherwise move to idle state. The advertisement would be 
             * triggerred once the GATT bearer is enabled again.
             */
            if(g_heater_app_data.bearer_data.bearerEnabled & 
                                            BLE_GATT_SERVER_BEARER_MASK)
            {
                AppSetState(app_state_advertising);
            }
            else
            {
                AppSetState(app_state_idle);
            }
            
        }
        break;

        default:
            /* Control should never come here */
            ReportPanic(app_panic_invalid_state);
        break;
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      handleCsrMeshGroupSetMsg
 *
 *  DESCRIPTION
 *      This function handles the CSRmesh Group Assignment message. Stores
 *      the group_id at the given index for the model
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static bool handleCsrMeshGroupSetMsg(uint8 *msg, uint16 len)
{
    CSR_MESH_MODEL_TYPE_T model = msg[0];
    uint8 index = msg[1];
    uint16 group_id = msg[3] + (msg[4] << 8);
    bool update_lastetag = TRUE;

    if(model == CSR_MESH_SENSOR_MODEL || model == CSR_MESH_ALL_MODELS)
    {
        if(index < NUM_SENSOR_MODEL_GROUPS)
        {
            CSR_MESH_ADVSCAN_PARAM_T param;
            bool old_config_status = IsHeaterConfigured();

            /* Store Group ID */
            sensor_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&sensor_model_groups[index], 
                      sizeof(uint16),
                      (NVM_OFFSET_SENSOR_MODEL_GROUPS + index));

            /* If heater was previously not grouped and has been grouped now, 
             * then the heater should move into low duty cycle 
             */
            if(!old_config_status && IsHeaterConfigured())
            {
                CsrMeshGetAdvScanParam(&param);
                param.scan_duty_cycle = DEFAULT_RX_DUTY_CYCLE;
                CsrMeshSetAdvScanParam(&param);
                DEBUG_STR("Heater Moving to Low Power Sniff Mode \r\n\r\n");
            }
            /* If heater was previously grouped and has been removed from group, 
             * then the heater should move into high duty cycle scan 
             */
            else if(old_config_status && !IsHeaterConfigured())
            {
                CsrMeshGetAdvScanParam(&param);
                param.scan_duty_cycle = HIGH_RX_DUTY_CYCLE;
                CsrMeshSetAdvScanParam(&param);

                DEBUG_STR("Heater Moving to active scan Mode \r\n\r\n");
            }

            /* A new group has been set. Hence start temp read and update */
            if(group_id != 0)
            {
                /* read the current temperature of the group */
                //read_value_transmit_count = 60;

                /* start a timer to read the temp from the group */
                //TimerDelete(read_val_tid);
                //read_val_tid = TIMER_INVALID;
                //startReadValueTimer();
            }
        }
        else
        {
            update_lastetag = FALSE;
        }
    }

    if(model == CSR_MESH_ATTENTION_MODEL || model == CSR_MESH_ALL_MODELS)
    {
        if(index < NUM_ATTENTION_MODEL_GROUPS)
        {
            attention_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&attention_model_groups[index],
                     sizeof(uint16),
                     NVM_OFFSET_ATTENTION_MODEL_GROUPS + index);
        }
        else
        {
            update_lastetag = FALSE;
        }
    }

#ifdef ENABLE_DATA_MODEL
    if(model == CSR_MESH_DATA_MODEL || model == CSR_MESH_ALL_MODELS)
    {
        if(index < NUM_DATA_MODEL_GROUPS)
        {
            data_model_groups[index] = group_id;

            /* Save to NVM */
            Nvm_Write(&data_model_groups[index],
                     sizeof(uint16),
                     NVM_OFFSET_DATA_MODEL_GROUPS + index);
        }
        else
        {
            update_lastetag = FALSE;
        }
    }
#endif /* ENABLE_DATA_MODEL */
    
    return update_lastetag;
}

/*test code 20160419*/
#ifdef PRESET_DEVID
static void presetDevid(void)
{
    sensor_model_groups[0] = 0x0002;
    attention_model_groups[0] = 0x0002;
    data_model_groups[0] = 0x0002;
    uint16 local_device_id=g_node_data.device_uuid.uuid[0];
    uint16 local_network_key[8]={0X1234,0X1234,0X1234,0X1234,
                                 0X1234,0X1234,0X1234,0X1234};
    CSR_MESH_ADVSCAN_PARAM_T param;
    
    /* Save group id to NVM */
    Nvm_Write(&sensor_model_groups[0],
              sizeof(uint16),NVM_OFFSET_SENSOR_MODEL_GROUPS);					 
    
    Nvm_Write(&attention_model_groups[0],
       sizeof(uint16),NVM_OFFSET_ATTENTION_MODEL_GROUPS);
    
    Nvm_Write(&data_model_groups[0],
       sizeof(uint16),NVM_OFFSET_DATA_MODEL_GROUPS);
    
    /*save device id to NVM*/			 
    
    Nvm_Write(&local_device_id, 1, NVM_OFFSET_DEVICE_ID);
    
    /*save network key to NVM*/
    Nvm_Write(&local_network_key[0], sizeof(CSR_MESH_NETWORK_KEY_T), 
              NVM_OFFSET_NETWORK_KEY);
    
       g_node_data.associated = TRUE;
    
    /* Read node data from NVM */
    /* Network key */
    Nvm_Read(g_node_data.nw_key.key, sizeof(CSR_MESH_NETWORK_KEY_T),
                                                    NVM_OFFSET_NETWORK_KEY);
    /* Device ID */
    Nvm_Read(&g_node_data.device_id, 1, NVM_OFFSET_DEVICE_ID);
    /* Sequence Number */
    Nvm_Read((uint16 *)&g_node_data.seq_number, 2,NVM_OFFSET_SEQUENCE_NUMBER);
    /***eof gnode data*/
    
    g_heater_app_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
    
    CsrMeshEnablePromiscuousMode(g_heater_app_data.bearer_data.bearerPromiscuous);										
    
    Nvm_Write((uint16 *)&g_heater_app_data.assoc_state, 1,
      NVM_OFFSET_ASSOCIATION_STATE);
      
    /* Update Bearer Model Data to NVM */
    Nvm_Write((uint16 *)&g_heater_app_data.bearer_data,
       sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);
    
    g_heater_app_data.assoc_state = app_state_associated;						
    
    /* When MESH_BRIDGE_SERVICE is not supported, Temperature Sensor 
    * needs to be associated with CSRmesh network, before it can send 
    * commands. Stop the blue led blinking visual indication, as 
    * Temperature Sensor is now associated to network.
    */
    IOTLightControlDevicePower(FALSE);
    
    /* If sensor was previously not grouped and has been grouped now, 
    * then the sensor should move into low duty cycle 
    */
    CsrMeshGetAdvScanParam(&param);
    param.scan_duty_cycle = DEFAULT_RX_DUTY_CYCLE;
    CsrMeshSetAdvScanParam(&param);
}
#endif

/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

#ifdef NVM_TYPE_FLASH
/*----------------------------------------------------------------------------*
 *  NAME
 *      WriteApplicationAndServiceDataToNVM
 *
 *  DESCRIPTION
 *      This function writes the application data to NVM. This function should
 *      be called on getting nvm_status_needs_erase
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void WriteApplicationAndServiceDataToNVM(void)
{
    uint16 nvm_sanity = 0xffff;
    nvm_sanity = NVM_SANITY_MAGIC;

    /* Write NVM sanity word to the NVM */
    Nvm_Write(&nvm_sanity, sizeof(nvm_sanity), NVM_OFFSET_SANITY_WORD);

    /* Store the Association State */
    Nvm_Write((uint16 *)&g_heater_app_data.assoc_state,
              sizeof(g_heater_app_data.assoc_state),
              NVM_OFFSET_ASSOCIATION_STATE);

    /* Write GAP service data into NVM */
    WriteGapServiceDataInNVM();
}
#endif /* NVM_TYPE_FLASH */

/*-----------------------------------------------------------------------------*
 *  NAME
 *      EnableHighDutyScanMode
 *
 *  DESCRIPTION
 *      The function enables/disables the active scan mode 
 *
 *  RETURNS/MODIFIES
 *      None
 *
 *----------------------------------------------------------------------------*/
extern void EnableHighDutyScanMode(bool enable)
{
    CSR_MESH_ADVSCAN_PARAM_T param;

    CsrMeshGetAdvScanParam(&param);

    if(enable)
    {
        param.scan_duty_cycle = HIGH_RX_DUTY_CYCLE;
    }
    else
    {
        if(IsHeaterConfigured())
        {
            /* Change the Rx scan duty cycle to default val on disabling 
             * attention 
             */
            param.scan_duty_cycle = DEFAULT_RX_DUTY_CYCLE;
        }
        else
        {
            /* Change the Rx scan duty cycle to active as the device 
             * is not grouped yet.
             */
            param.scan_duty_cycle = HIGH_RX_DUTY_CYCLE;
        }
    }
    CsrMeshSetAdvScanParam(&param);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppSetState
 *
 *  DESCRIPTION
 *      This function is used to set the state of the application.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void AppSetState(app_state new_state)
{
    /* Check if the new state to be set is not the same as the present state
     * of the application.
     */
    app_state old_state = g_heater_app_data.state;

    if (old_state != new_state)
    {
        /* Handle exiting old state */
        switch(old_state)
        {
            case app_state_init:
                /* Do nothing here */
            break;

            case app_state_disconnecting:
                /* Common things to do whenever application exits
                 * app_state_disconnecting state.
                 */

                /* Initialise CSRmesh Heater data and services
                 * data structure while exiting Disconnecting state.
                 */
                appDataInit();
            break;

            case app_state_advertising:
                /* Common things to do whenever application exits
                 * APP_*_ADVERTISING state.
                 */
                appAdvertisingExit();
            break;

            case app_state_connected:
                /* Do nothing here */
            break;

            default:
                /* Nothing to do */
            break;
        }

        /* Set new state */
        g_heater_app_data.state = new_state;

        /* Handle entering new state */
        switch(new_state)
        {
            case app_state_advertising:
            {
                GattTriggerFastAdverts();
                DEBUG_STR("app_state_advertising\r\n");
            }
            break;

            case app_state_connected:
            {
                DEBUG_STR("Connected\r\n");
            }
            break;

            case app_state_disconnecting:
                GattDisconnectReq(g_heater_app_data.gatt_data.st_ucid);
                DEBUG_STR("app_state_disconnecting\r\n");
            break;

            default:
            break;
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      ReportPanic
 *
 *  DESCRIPTION
 *      This function calls firmware panic routine and gives a single point
 *      of debugging any application level panics
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void ReportPanic(app_panic_code panic_code)
{
    /* Raise panic */
    Panic(panic_code);
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This user application function is called just after a power-on reset
 *      (including after a firmware panic), or after a wakeup from Hibernate or
 *      Dormant sleep states.
 *
 *      At the time this function is called, the last sleep state is not yet
 *      known.
 *
 *      NOTE: this function should only contain code to be executed after a
 *      power-on reset or panic. Code that should also be executed after an
 *      HCI_RESET should instead be placed in the AppInit() function.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppPowerOnReset(void)
{
}

/*-----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This user application function is called after a power-on reset
 *      (including after a firmware panic), after a wakeup from Hibernate or
 *      Dormant sleep states, or after an HCI Reset has been requested.
 *
 *      The last sleep state is provided to the application in the parameter.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after app_power_on_reset().
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void AppInit(sleep_state last_sleep_state)
{
    uint16 gatt_db_length = 0;
    uint16 *p_gatt_db_pointer = NULL;
    CSR_MESH_ADVSCAN_PARAM_T param;
    //DEVICE_T g_device[inputNumberOfSensor];
    //p_device=g_device;

#ifdef USE_STATIC_RANDOM_ADDRESS
    /* Generate static random address for the CSRmesh Device. */
    generateStaticRandomAddress(&g_heater_app_data.random_bd_addr);

    /* Set the Static Random Address of the device. */
    GapSetRandomAddress(&g_heater_app_data.random_bd_addr);
#endif /* USE_STATIC_RANDOM_ADDRESS */

    /* Initialise the application timers */
    TimerInit(MAX_APP_TIMERS, (void*)app_timers);

#ifdef DEBUG_ENABLE
    /* Initialise UART and configure with
     * default baud rate and port configuration.
     */

    //DebugInit(1, UartDataRxCallback, NULL);
    //uartInit();
    /* UART Rx threshold is set to 1,
     * so that every byte received will trigger the Rx callback.
     */
    //UartRead(1, 0);
    uartInit();//20160612

#endif /* DEBUG_ENABLE */

    /* Initialise Light and turn it off. */
    IOTLightControlDeviceInit();
    IOTLightControlDevicePower(FALSE);

    DEBUG_STR("\r\nCSRmesh Heater \r\n");
    
    /* Initialise GATT entity */
    GattInit();

    /* Install GATT Server support for the optional Write procedure
     * This is mandatory only if control point characteristic is supported.
     */
    GattInstallServerWriteLongReliable();
    
    /* Don't wakeup on UART RX line */
    SleepWakeOnUartRX(FALSE);

#ifdef NVM_TYPE_EEPROM
    /* Configure the NVM manager to use I2C EEPROM for NVM store */
    NvmConfigureI2cEeprom();
#elif NVM_TYPE_FLASH
    /* Configure the NVM Manager to use SPI flash for NVM store. */
    NvmConfigureSpiFlash();
#endif /* NVM_TYPE_EEPROM */

    NvmDisable();

    /* Initialise the GATT and GAP data.
     * Needs to be done before readPersistentStore
     */
    appDataInit();

    /* Initialise Application specific Sensor Model Data.
     * This needs to be done before readPersistentStore.
     */
    sensor_state[CURRENT_AIR_TEMP].type = sensor_type_internal_air_temperature;
    sensor_state[CURRENT_AIR_TEMP].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    sensor_state[CURRENT_AIR_TEMP].value       = &current_air_temp;
    sensor_state[CURRENT_AIR_TEMP].repeat_interval = 0;
    
    sensor_state[CURRENT_AIR_HUMI].type = sensor_type_internal_humidity;
    sensor_state[CURRENT_AIR_HUMI].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    sensor_state[CURRENT_AIR_HUMI].value       = &current_air_humi;
    sensor_state[CURRENT_AIR_HUMI].repeat_interval = 0;   
       
    sensor_state[CURRENT_PRESSURE_LOW].type = sensor_type_desired_air_temperature;
    sensor_state[CURRENT_PRESSURE_LOW].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    sensor_state[CURRENT_PRESSURE_LOW].value       = &current_press_low;
    sensor_state[CURRENT_PRESSURE_LOW].repeat_interval = 0;
       
    sensor_state[CURRENT_PRESSURE_HIGH].type = sensor_type_barometric_pressure;
    sensor_state[CURRENT_PRESSURE_HIGH].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    //sensor_state[CURRENT_PRESSURE_HIGH].value       = &current_press_high;//20160809
    sensor_state[CURRENT_PRESSURE_HIGH].value       = &current_air_press;
    sensor_state[CURRENT_PRESSURE_HIGH].repeat_interval = 0;

    g_heater_app_data.sensor_data.num_types   = NUM_SENSORS_SUPPORTED;
    g_heater_app_data.sensor_data.sensor_list = sensor_state;

    /* Initialise CSRmesh Heater Application State */
    g_heater_app_data.state = app_state_init;
    
    /* Read persistent storage.
     * Call this before CsrMeshInit.
     */
    readPersistentStore();
    
    /*test code 20160419*/    
    presetDevid();
    
    /* Initialise the CSRmesh */
    CsrMeshInit(&g_node_data);

    /* Update Relay status on Heater */
    CsrMeshRelayEnable(g_heater_app_data.bearer_data.bearerRelayActive);

    /* Update promiscuous status */
    CsrMeshEnablePromiscuousMode(
                            g_heater_app_data.bearer_data.bearerPromiscuous);

    /* Enable notifications */
    CsrMeshEnableRawMsgEvent(TRUE);

    /* Initialise Sensor Model. */
    SensorModelInit(sensor_model_groups, NUM_SENSOR_MODEL_GROUPS);

#ifdef ENABLE_FIRMWARE_MODEL
    /* Initialise Firmware Model */
    FirmwareModelInit();

    /* Set Firmware Version */
    g_heater_app_data.fw_version.major_version = APP_MAJOR_VERSION;
    g_heater_app_data.fw_version.minor_version = APP_MINOR_VERSION;
#endif /* ENABLE_FIRMWARE_MODEL */

    /* Initialise Attention Model */
    AttentionModelInit(attention_model_groups, NUM_ATTENTION_MODEL_GROUPS);

#ifdef ENABLE_DATA_MODEL
    AppDataStreamInit(data_model_groups, NUM_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

    /* Initialise Bearer Model */
    BearerModelInit();

#ifdef ENABLE_BATTERY_MODEL
    BatteryModelInit();
#endif

    /* Start CSRmesh */
    CsrMeshStart();

    /* Tell Security Manager module about the value it needs to Initialise it's
     * diversifier to.
     */
    SMInit(0);

    /* Get the stored adv scan parameters */
    CsrMeshGetAdvScanParam(&param);

    /* Read the mesh advertising parameter setting from the CS User Keys */
    param.advertising_interval =
                                CSReadUserKey(CSKEY_INDEX_CSRMESH_ADV_INTERVAL);
    param.advertising_time = CSReadUserKey(CSKEY_INDEX_CSRMESH_ADV_TIME);

    if (g_heater_app_data.assoc_state != app_state_associated)
    {
        initiateAssociation();
    }
    else 
    {
        DEBUG_STR("Heater is associated\r\n");
        if(IsHeaterConfigured())
        {
            DEBUG_STR("Heater Configured Moving to Low Power Mode \r\n\r\n");
            /* If the app is associated and grouped then move the application 
             * to low power sniff mode.
             */
            param.scan_duty_cycle =  DEFAULT_RX_DUTY_CYCLE;

            /* read the current temperature of the group */
            //read_value_transmit_count = 60;

            /* start a timer to read the temp from the group */
            //TimerDelete(read_val_tid);
            //read_val_tid = TIMER_INVALID;
            //startReadValueTimer();
        }
    }

    /* Set the Advertising and Scan parameters */
    CsrMeshSetAdvScanParam(&param);

#ifdef ENABLE_ACK_MODE
    resetDeviceList();
#endif /* ENABLE_ACK_MODE */

    /* Tell GATT about our database. We will get a GATT_ADD_DB_CFM event when
     * this has completed.
     */
    p_gatt_db_pointer = GattGetDatabase(&gatt_db_length);
    GattAddDatabaseReq(gatt_db_length, p_gatt_db_pointer); 
    
    PioSetMode(REC_IND,pio_mode_user);
    PioSetDir(REC_IND,1);
    PioSetPullModes((1L<<REC_IND),pio_mode_strong_pull_up);
    PioSet(REC_IND,1);//off

    //uint16 data=0xcdef;
    //Nvm_Write(&data,1,NVM_ALL_SENSORS_DB);
    //DEBUG_STR("Choose to display in CSV?(Y/N)\r\n");//ascii_num=0;
    //time_dat_tid=TimerCreate(5*SECOND,TRUE,printTimeAndData);//开启
    uint16 k;
    bool devidExist=FALSE;
    Nvm_Read(&endOfDevidNvmAddr,1,NVM_DEVID_ADDR_INDEX_OFFSET);
    //currentHeaterStatus=0xabce;
    //Nvm_Write(&test11,1,NVM_currentHeaterStatus_OFFSET);
    Nvm_Read(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);
    
    for(k=0;k<NUM_OF_DEVID_IN_NVM;k++)
    {   
        if((readDevIDFromNVM(k)==0)||(readDevIDFromNVM(k)==0xffff)) 
            devidExist=FALSE;
        else
        {
            devidExist=TRUE;
            break;
        }
    }
       
    /*if(devidExist==TRUE)//registered
    {   
        currentHeaterStatus=HEATER_STATUS_NORMAL;
        Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);//20160616
        Nvm_Read(nvmBuffer,NUM_OF_DEVID_IN_NVM,NVM_SENSORS_DEVID_OFFSET);               
        no_data_check_tid=
                TimerCreate(30*SECOND,TRUE,periodicCheckNoDataDevices);
    } 
    else
    {   
        currentHeaterStatus=HEATER_STATUS_INIT;//searching mode(advert)
        Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);//20160616
        info_tid=TimerCreate(5*SECOND,TRUE,infoTimerHandler);
        //DEBUG_STR("Please input sensors number:");//20160622
    }*/
    //while(1) DEBUG_STR("Heater test...\r\n");
        
    //currentHeaterStatus=HEATER_STATUS_NORMAL;//0718
    //TimerDelete(info_tid);
    //Nvm_Write(&currentHeaterStatus,1,NVM_HEATER_STATUS_OFFSET);
    
    SleepModeChange(sleep_mode_never);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern void AppProcessSystemEvent(sys_event_id id, void *data)
{
    switch(id)
    {
        case sys_event_pio_changed:
            /* Handle PIO Events */
        break;

        default:
            /* Ignore anything else */
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event is
 *      received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
extern bool AppProcessLmEvent(lm_event_code event_code,
                              LM_EVENT_T *p_event_data)
{

    switch(event_code)
    {
        /* Handle events received from Firmware */

        case GATT_ADD_DB_CFM:
            /* Attribute database registration confirmation */
            handleSignalGattAddDBCfm((GATT_ADD_DB_CFM_T*)p_event_data);
        break;

        case GATT_CANCEL_CONNECT_CFM:
            /* Confirmation for the completion of GattCancelConnectReq()
             * procedure
             */
            handleSignalGattCancelConnectCfm();
        break;

        case LM_EV_CONNECTION_COMPLETE:
            /* Handle the LM connection complete event. */
            handleSignalLmEvConnectionComplete((LM_EV_CONNECTION_COMPLETE_T*)
                                                                p_event_data);
        break;

        case GATT_CONNECT_CFM:
            /* Confirmation for the completion of GattConnectReq()
             * procedure
             */
            handleSignalGattConnectCfm((GATT_CONNECT_CFM_T*)p_event_data);
        break;

        case SM_SIMPLE_PAIRING_COMPLETE_IND:
            /* Indication for completion of Pairing procedure */
            handleSignalSmSimplePairingCompleteInd(
                (SM_SIMPLE_PAIRING_COMPLETE_IND_T*)p_event_data);
        break;

        case LM_EV_ENCRYPTION_CHANGE:
            /* Indication for encryption change event */
            /* Nothing to do */
        break;

        /* Received in response to the LsConnectionParamUpdateReq()
         * request sent from the slave after encryption is enabled. If
         * the request has failed, the device should again send the same
         * request only after Tgap(conn_param_timeout). Refer Bluetooth 4.0
         * spec Vol 3 Part C, Section 9.3.9 and HID over GATT profile spec
         * section 5.1.2.
         */
        case LS_CONNECTION_PARAM_UPDATE_CFM:
            handleSignalLsConnParamUpdateCfm(
                (LS_CONNECTION_PARAM_UPDATE_CFM_T*) p_event_data);
        break;

        case LM_EV_CONNECTION_UPDATE:
            /* This event is sent by the controller on connection parameter
             * update.
             */
            handleSignalLmConnectionUpdate(
                            (LM_EV_CONNECTION_UPDATE_T*)p_event_data);
        break;

        case LS_CONNECTION_PARAM_UPDATE_IND:
            /* Indicates completion of remotely triggered Connection
             * parameter update procedure
             */
            handleSignalLsConnParamUpdateInd(
                            (LS_CONNECTION_PARAM_UPDATE_IND_T *)p_event_data);
        break;

        case GATT_ACCESS_IND:
            /* Indicates that an attribute controlled directly by the
             * application (ATT_ATTR_IRQ attribute flag is set) is being
             * read from or written to.
             */
            handleSignalGattAccessInd((GATT_ACCESS_IND_T*)p_event_data);
        break;

        case GATT_DISCONNECT_IND:
            /* Disconnect procedure triggered by remote host or due to
             * link loss is considered complete on reception of
             * LM_EV_DISCONNECT_COMPLETE event. So, it gets handled on
             * reception of LM_EV_DISCONNECT_COMPLETE event.
             */
         break;

        case GATT_DISCONNECT_CFM:
            /* Confirmation for the completion of GattDisconnectReq()
             * procedure is ignored as the procedure is considered complete
             * on reception of LM_EV_DISCONNECT_COMPLETE event. So, it gets
             * handled on reception of LM_EV_DISCONNECT_COMPLETE event.
             */
        break;

        case LM_EV_DISCONNECT_COMPLETE:
        {
            /* Disconnect procedures either triggered by application or remote
             * host or link loss case are considered completed on reception
             * of LM_EV_DISCONNECT_COMPLETE event
             */
             handleSignalLmDisconnectComplete(
                    &((LM_EV_DISCONNECT_COMPLETE_T *)p_event_data)->data);
        }
        break;

        case LM_EV_ADVERTISING_REPORT:
            CsrMeshProcessMessage((LM_EV_ADVERTISING_REPORT_T*)p_event_data);
        break;

        case LS_RADIO_EVENT_IND:
        {
            CsrMeshHandleRadioEvent();
        }
        break;

        default:
            /* Ignore any other event */
        break;

    }

    return TRUE;
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessCsrMeshEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a CSRmesh event
 *      is received by the system.
 *
 * PARAMETERS
 *      event_code csr_mesh_event_t
 *      data       Data associated with the event
 *      length     Length of the data
 *      state_data Pointer to the variable pointing to state data.
 *
 * RETURNS
 *      TRUE if the app has finished with the event data; the control layer
 *      will free the buffer.
 *----------------------------------------------------------------------------*/
extern void AppProcessCsrMeshEvent(csr_mesh_event_t event_code, uint8* data,
                                   uint16 length, void **state_data)
{
    bool update_lastetag = FALSE;

    switch(event_code)
    {
        case CSR_MESH_ASSOCIATION_REQUEST:
        {
            DEBUG_STR("Association Started\r\n");
            if (g_heater_app_data.assoc_state != app_state_association_started)
            {
                g_heater_app_data.assoc_state = app_state_association_started;
            }
            TimerDelete(g_heater_app_data.mesh_device_id_advert_tid);
            g_heater_app_data.mesh_device_id_advert_tid = TIMER_INVALID;

            /* Blink Light in Yellow to indicate association started */
            IOTLightControlDeviceBlink(127, 127, 0, 32, 32);
        }
        break;

        case CSR_MESH_KEY_DISTRIBUTION:
        {
            DEBUG_STR("Association complete\r\n");
            g_heater_app_data.assoc_state = app_state_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_heater_app_data.assoc_state, 1,
                                                  NVM_OFFSET_ASSOCIATION_STATE);

            /* Save the network key on NVM */
            Nvm_Write((uint16 *)data, sizeof(CSR_MESH_NETWORK_KEY_T), 
                                                        NVM_OFFSET_NETWORK_KEY);

            /* The association is complete set LE bearer to non-promiscuous.*/
            g_heater_app_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
            CsrMeshEnablePromiscuousMode(
                               g_heater_app_data.bearer_data.bearerPromiscuous);

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_heater_app_data.bearer_data,
                    sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

            /* When MESH_BRIDGE_SERVICE is not supported, Heater needs to be
             * associated with CSRmesh network, before it can send commands.
             * Stop the blue led blinking visual indication, as Heater is
             * now associated to network.
             */
            IOTLightControlDevicePower(FALSE);

        }
        break;

        case CSR_MESH_ASSOCIATION_ATTENTION:
        {
            CSR_MESH_ASSOCIATION_ATTENTION_DATA_T *attn_data;

            attn_data = (CSR_MESH_ASSOCIATION_ATTENTION_DATA_T *)data;

            /* Delete attention timer if it exists */
            if (TIMER_INVALID != attn_tid)
            {
                TimerDelete(attn_tid);
                attn_tid = TIMER_INVALID;
            }
            /* If attention Enabled */
            if (attn_data->attract_attention)
            {
                /* Create attention duration timer if required */
                if(attn_data->duration != 0xFFFF)
                {
                    attn_tid = TimerCreate(attn_data->duration * MILLISECOND, 
                                                        TRUE, attnTimerHandler);
                }
                /* Enable Green light blinking to attract attention */
                IOTLightControlDeviceBlink(0, 127, 0, 16, 16);
            }
            else
            {
                if(g_heater_app_data.assoc_state == app_state_not_associated)
                {
                    /* Blink blue to indicate not associated status */
                    IOTLightControlDeviceBlink(0, 0, 127, 32, 32);
                }
                else
                {
                    /* Restore the heater status display */
                    //updateHeaterStatus();
                }
            }
        }
        break;

        case CSR_MESH_UPDATE_MSG_SEQ_NUMBER:
        {
            /* Sequence number has updated, store it in NVM */
            Nvm_Write((uint16 *)data, 2, NVM_OFFSET_SEQUENCE_NUMBER);
        }
        break;

        case CSR_MESH_CONFIG_RESET_DEVICE:
        {
            uint16 i;
            DEBUG_STR("Reset Device\r\n");

            /* Move device to dissociated state */
            g_heater_app_data.assoc_state = app_state_not_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_heater_app_data.assoc_state,
                     sizeof(g_heater_app_data.assoc_state),
                     NVM_OFFSET_ASSOCIATION_STATE);

            /* Reset the supported model groups and save it to NVM */
            /* Sensor model */
            MemSet(sensor_model_groups, 0x0000, sizeof(sensor_model_groups));
            Nvm_Write(sensor_model_groups, sizeof(sensor_model_groups),
                                            NVM_OFFSET_SENSOR_MODEL_GROUPS);

            /* Attention model */
            MemSet(attention_model_groups, 0x0000,
                                            sizeof(attention_model_groups));
            Nvm_Write(attention_model_groups, sizeof(attention_model_groups),
                                            NVM_OFFSET_ATTENTION_MODEL_GROUPS);

#ifdef ENABLE_DATA_MODEL
            /* Data stream model */
            MemSet(data_model_groups, 0x0000, sizeof(data_model_groups));
            Nvm_Write((uint16 *)data_model_groups, sizeof(data_model_groups),
                                            NVM_OFFSET_DATA_MODEL_GROUPS);
#endif /* ENABLE_DATA_MODEL */

            /* Reset sensor. */
            for (i = 0; i < NUM_SENSORS_SUPPORTED; i++)
            {
                writeSensorDataToNVM(i);
            }

            current_air_temp = 0;
            current_air_humi = 0;

            /* Enable promiscuous mode on un-associated devices so that they 
             * relay all the messages. This helps propagate messages(MCP) based 
             * on the newly assigned network key as they will be relayed also by
             * the devices not yet associated.
             */
            g_heater_app_data.bearer_data.bearerPromiscuous= 
                               (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);
            CsrMeshEnablePromiscuousMode(
                               g_heater_app_data.bearer_data.bearerPromiscuous);

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_heater_app_data.bearer_data,
                      sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);

            /* Start Mesh association again */
            initiateAssociation();
        }
        break;

        case CSR_MESH_CONFIG_GET_VID_PID_VERSION:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&vid_pid_info;
            }
        }
        break;

        case CSR_MESH_CONFIG_GET_APPEARANCE:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&device_appearance;
            }
        }
        break;

        case CSR_MESH_GROUP_SET_MODEL_GROUPID:
        {
            /* Save Group Information here */
            update_lastetag = handleCsrMeshGroupSetMsg(data, length);
        }
        break;

        case CSR_MESH_CONFIG_DEVICE_IDENTIFIER:
        {
            Nvm_Write((uint16 *)data, 1, NVM_OFFSET_DEVICE_ID);
            DEBUG_STR("Device ID received:");
            DEBUG_U16(((uint16)data[0]));
            DEBUG_STR("\r\n");
        }
        break;

#ifdef ENABLE_BATTERY_MODEL
        case CSR_MESH_BATTERY_GET_STATE:
        {
            /* Initialise battery state. IOT  boards (H13323) are battery powered */
            g_heater_app_data.battery_data.battery_state = 
                                            BATTERY_MODEL_STATE_POWERING_DEVICE;
            /* Read Battery Level */
            g_heater_app_data.battery_data.battery_level = ReadBatteryLevel();
            if(g_heater_app_data.battery_data.battery_level == 0)
            {
                /* Voltage is below flat battery voltage. Set the needs 
                 * replacement flag in the battery state
                 */
                g_heater_app_data.battery_data.battery_state |=
                                          BATTERY_MODEL_STATE_NEEDS_REPLACEMENT;
            }

            /* Pass Battery state data to model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_heater_app_data.battery_data;
            }
        }
        break;
#endif /* ENABLE_BATTERY_MODEL */

        case CSR_MESH_BEARER_SET_STATE:
        {
            uint8 *pData = data;
            g_heater_app_data.bearer_data.bearerRelayActive = 
                                                        BufReadUint16(&pData);
            g_heater_app_data.bearer_data.bearerEnabled     = 
                                                        BufReadUint16(&pData);
            g_heater_app_data.bearer_data.bearerPromiscuous = 
                                                        BufReadUint16(&pData);

            /* BLE Advert Bearer is always enabled on this device. */
            g_heater_app_data.bearer_data.bearerEnabled    |= BLE_BEARER_MASK;

            /* Filter the supported bearers from the bitmap received */
            g_heater_app_data.bearer_data.bearerRelayActive = 
                g_heater_app_data.bearer_data.bearerRelayActive & 
                    (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);

            /* Filter the supported bearers from the bitmap received */
            g_heater_app_data.bearer_data.bearerEnabled = 
                g_heater_app_data.bearer_data.bearerEnabled & 
                    (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);

            /* Filter the supported bearers from the bitmap received */
            g_heater_app_data.bearer_data.bearerPromiscuous = 
                g_heater_app_data.bearer_data.bearerPromiscuous & 
                    (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);

            /* Update the saved values */
            bearer_relay_active = 
                                g_heater_app_data.bearer_data.bearerRelayActive;
            bearer_promiscuous = 
                                g_heater_app_data.bearer_data.bearerPromiscuous;

            /* Update new bearer state */
            CsrMeshRelayEnable(g_heater_app_data.bearer_data.bearerRelayActive);
            CsrMeshEnablePromiscuousMode(
                               g_heater_app_data.bearer_data.bearerPromiscuous);

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_heater_app_data.bearer_data,
                                sizeof(BEARER_MODEL_STATE_DATA_T), 
                                NVM_BEARER_DATA_OFFSET);

            if(g_heater_app_data.state != app_state_connected) 
            {
                if(g_heater_app_data.bearer_data.bearerEnabled 
                                                & BLE_GATT_SERVER_BEARER_MASK)
                {
                    AppSetState(app_state_advertising);
                }
                else
                {
                    AppSetState(app_state_idle);
                }
            }

            update_lastetag = TRUE;
        }
        /* Fall through */
        case CSR_MESH_BEARER_GET_STATE:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&g_heater_app_data.bearer_data;
            }
        }
        break;

#ifdef ENABLE_FIRMWARE_MODEL
        case CSR_MESH_FIRMWARE_GET_VERSION_INFO:
        {
            /* Send Firmware Version data to the model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_heater_app_data.fw_version;
            }
        }
        break;

        case CSR_MESH_FIRMWARE_UPDATE_REQUIRED:
        {
            BD_ADDR_T *pBDAddr = NULL;
#ifdef USE_STATIC_RANDOM_ADDRESS
            pBDAddr = &g_heater_app_data.random_bd_addr;
#endif /* USE_STATIC_RANDOM_ADDRESS */

            DEBUG_STR("\r\n FIRMWARE UPDATE IN PROGRESS \r\n");

            /* Write the value CSR_OTA_BOOT_LOADER to NVM so that
             * it starts in OTA mode upon reset
             */
            OtaWriteCurrentApp(csr_ota_boot_loader,
                               FALSE,   /* is bonded */
                               NULL,    /* Typed host BD Address */
                               0,       /* Diversifier */
                               pBDAddr, /* local_random_address */
                               NULL,    /* irk */
                               FALSE    /* service_changed_config */
                              );

           /* Defer OTA Reset for half a second to ensure that,
            * acknowledgements are sent before reset.
            */
           ota_rst_tid = TimerCreate(OTA_RESET_DEFER_DURATION, TRUE,
                                     issueOTAReset);

           /* Update LastETag. */
           update_lastetag = TRUE;
        }
        break;
#endif /* ENABLE_FIRMWARE_MODEL */

#ifdef ENABLE_DATA_MODEL
        /* Data stream model messages */
        case CSR_MESH_DATA_STREAM_SEND_CFM://register也是关键字
        {   
            //DEBUG_STR("case CSR_MESH_DATA_STREAM_SEND_CFM\r\n");
            handleCSRmeshDataStreamSendCfm((CSR_MESH_STREAM_EVENT_T *)data);
            //20160519
            id_cfm=0;//表明id cfm发送和接受成功:阻断继续发id cmd指令
            
        }
        break;

        case CSR_MESH_DATA_STREAM_DATA_IND:
        {   //DEBUG_STR("case CSR_MESH_DATA_STREAM_DATA_IND\r\n");
            handleCSRmeshDataStreamDataInd((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;

        /* Stream flush indication */
        case CSR_MESH_DATA_STREAM_FLUSH_IND:
        {
            handleCSRmeshDataStreamFlushInd((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;

        /* Received a single block of data */
        case CSR_MESH_DATA_BLOCK_IND:
        {
            handleCSRmeshDataBlockInd((CSR_MESH_STREAM_EVENT_T *)data);
        }
        break;
#endif /* ENABLE_DATA_MODEL */

        /* Sensor Messages handling */
        case CSR_MESH_SENSOR_GET_TYPES:
        case CSR_MESH_SENSOR_GET_STATE:
        case CSR_MESH_SENSOR_READ_VALUE:
        case CSR_MESH_SENSOR_MISSING:
        {   
            DEBUG_STR("case CSR_MESH_SENSOR_MISSING\r\n");
            if (state_data != NULL)
            {
                *state_data = (void *)&g_heater_app_data.sensor_data;
            }
        }
        break;

        case CSR_MESH_SENSOR_WRITE_VALUE:
        case CSR_MESH_SENSOR_WRITE_VALUE_NO_ACK:
        case CSR_MESH_SENSOR_VALUE:
        {             
            SENSOR_MODEL_EVENT_T *sensor_event = (SENSOR_MODEL_EVENT_T*)data;
            uint8 *p_data = sensor_event->msg;
            uint8 *p_data_end = sensor_event->msg + sensor_event->msg_len;
            uint8 index = 0; //sensor type index
            
            sensor_type_t type[4] = {sensor_type_invalid};      
            *state_data = NULL;
            
            recSensorDataDevid = sensor_event->sensor_model_info.source_id;          
            /* if temp data received = 0000000000000001b;
               if humi data received = 0000000000000010b;
               if pressure high data recevied = 0000000000000100b;
               if pressure low  data recevied = 0000000000001000b;
               if data rec in progress = 1000000000000000b;
               
               e.g. if all data received = 0000000000001111b=0x000F*/
            
            while((p_data < p_data_end) &&(continueToRecv==TRUE))
            {
                type[index] = BufReadUint16(&p_data);
                switch(type[index])
                {   
                    case sensor_type_internal_air_temperature:
                    {
                        /* Length of the value of this type is 16 bits */
                        recvd_air_temp = BufReadUint16(&p_data);//接受抽取消息中数据
                        dataReceivedFlag|=TEMP_DATA_RECVD;
                        current_air_temp = recvd_air_temp;
                        
                        DEBUG_STR("[$RTD");DEBUG_U16(recSensorDataDevid);
                        DEBUG_STR("DT");printInDecimal(current_air_temp/100);DEBUG_STR(".");
                        printInDecimal(current_air_temp%100);DEBUG_STR("T]");/**/
                        
                        if((dataReceivedFlag&0xfffe)==0)
                            sensor_data_incomplete_tid=
                                TimerCreate(TIMER_INTERVAL_SENSOR_DATA_REC,TRUE,handleSensorDataIncomplete);
                        break;
                    }
                    case sensor_type_internal_humidity:
                    {
                        /* Length of the value of this type is 16 bits */
                        recvd_air_humi = BufReadUint16(&p_data);//接受抽取消息中数据
                        dataReceivedFlag|=HUMI_DATA_RECVD;
                        current_air_humi = recvd_air_humi;

                        DEBUG_STR("[$RHD");DEBUG_U16(recSensorDataDevid);
                        DEBUG_STR("DH");    printInDecimal(((uint32)current_air_humi<<4)/1024);DEBUG_STR(".");    
                        printInDecimal(( (((uint32)current_air_humi<<4)%1024) *1000)/1024);DEBUG_STR("H]");                        
                        if((dataReceivedFlag&0xfffd)==0)
                            sensor_data_incomplete_tid=
                                TimerCreate(TIMER_INTERVAL_SENSOR_DATA_REC,TRUE,handleSensorDataIncomplete);
                        break;
                    }
                    
                    case sensor_type_barometric_pressure:
                    {   
                        //recvd_press_high = BufReadUint16(&p_data);
                        recvd_air_press = BufReadUint16(&p_data);
                        //DEBUG_U16(recvd_press_high);DEBUG_STR("←P ");
                        dataReceivedFlag|=PRESS_HIGH_RECVD;
                        current_air_press = recvd_air_press;

                            DEBUG_STR("[$RPD");DEBUG_U16(recSensorDataDevid);
                            DEBUG_STR("DP");printInDecimal(current_air_press);DEBUG_STR("P]");/**/
                        
                        if((dataReceivedFlag&0xfffb)==0)
                            sensor_data_incomplete_tid=
                                TimerCreate(TIMER_INTERVAL_SENSOR_DATA_REC,TRUE,handleSensorDataIncomplete);
                        break;
                    }
                                
                    default:
                    {
                        //DEBUG_STR("\r\n");
                        break;
                    }                   
                }

                if(dataReceivedFlag==ALL_DATA_RECVD)
                {   
                    /*TimerDelete(sensor_data_incomplete_tid);
                    sensor_data_incomplete_tid=TIMER_INVALID;
                    dataReceivedFlag=NO_DATA_RECVD ;
                                        
                    PioSet(REC_IND,0);
                    TimerCreate(1*SECOND,TRUE,recDataInd);*/
                    
                    /*current_air_temp = recvd_air_temp;                    
                    current_battery_level = recvd_bat_level;

                    current_press_low = recvd_press_low;
                    current_press_high = recvd_press_high;
                    current_air_press = ((uint32)recvd_press_high)<<16 |
                                        (uint32)recvd_press_low;//接收
                    
                    current_humi_low = recvd_humi_low;
                    current_humi_high= recvd_humi_high;
                    current_air_humi = ((uint32)recvd_humi_low) | \
                                       ((uint32)recvd_humi_high)<<16;

                    if(outputSensorOneTimeFlag==TRUE)
                    {
                        DEBUG_STR("\r\n");DEBUG_U16(recSensorDataDevid);
                        DEBUG_STR(": ");
                        printInDecimal(current_air_temp/100);DEBUG_STR(".");
                        printInDecimal(current_air_temp%100);DEBUG_STR("DegC  ");
                        printInDecimal(current_air_humi/1024);DEBUG_STR(".");
                        printInDecimal((current_air_humi%1024)*1000/1024);DEBUG_STR("%  ");
                        printInDecimal(current_air_press);DEBUG_STR("Pa ");  \
                        printInDecimal(current_battery_level);DEBUG_STR("mV ");
                        //DebugWriteUint16(g_heater_app_data.state);DEBUG_STR(" ");//app_state
                        //DebugWriteUint16(g_heater_app_data.assoc_state);//app_association_state
                        outputSensorOneTimeFlag=FALSE;
                        TimerCreate(800*MILLISECOND,TRUE,avoidRepeatPrintSameDevTimerHandler);
                    }*/          
                }
            }
        }
        break;

        case CSR_MESH_ATTENTION_SET_STATE:
        {
            ATTENTION_MODEL_STATE_DATA_T   attn_data;

            /* Read the data */
            attn_data.attract_attn  = BufReadUint8(&data);
            attn_data.attn_duration = BufReadUint16(&data);

            /* Delete attention timer if it exists */
            if (TIMER_INVALID != attn_tid)
            {
                TimerDelete(attn_tid);
                attn_tid = TIMER_INVALID;
            }

            /* If attention Enabled */
            if (attn_data.attract_attn)
            {
                /* Create attention duration timer if required */
                if (attn_data.attn_duration != 0xFFFF)
                {
                    attn_tid = TimerCreate((uint32)attn_data.attn_duration * 
                                                MILLISECOND,
                                           TRUE, 
                                           attnTimerHandler);
                }
                /* Enable Red light blinking to attract attention */
                IOTLightControlDeviceBlink(127, 0, 0, 32, 32);

                /* Change the Rx scan duty cycle on enabling attention */
                EnableHighDutyScanMode(TRUE);
            }
            else
            {
                /* Restore Light State */
                if( g_heater_app_data.status == heater_off)
                {
                    IOTLightControlDeviceSetColor(0,0,0);

                    /* Restore the light Power State */
                    IOTLightControlDevicePower(FALSE);
                }
                else
                {
                    IOTLightControlDeviceSetColor(255,0,0);
                }

                /* Set back the scan to low duty cycle only if the device has
                 * already been grouped.
                 */
                EnableHighDutyScanMode(TRUE); //20160808 change from false to true
            }

            /* Send response data to model */
            if (state_data != NULL)
            {
                *state_data = (void *)&attn_data;
            }

            /* Debug logs */
            DEBUG_STR("\r\n ATTENTION_SET_STATE : Enable :");
            DEBUG_U8(attn_data.attract_attn);
            DEBUG_STR("Duration : ");
            printInDecimal(attn_data.attn_duration);
            DEBUG_STR(" Milliseconds\r\n");

        }
        break;

        /* Received a raw message from lower-layers.
         * Notify to the control device if connected.
         */
        case CSR_MESH_RAW_MESSAGE:
        {
            if (g_heater_app_data.state == app_state_connected)
            {
                MeshControlNotifyResponse(g_heater_app_data.gatt_data.st_ucid,
                                          data, length);
            }
        }
        break;

        default:
        break;
    }

    /* Commit LastETag Change. */
    if (update_lastetag)
    {
        CsrMeshUpdateLastETag(&g_node_data.device_ETag);
        Nvm_Write(g_node_data.device_ETag.ETag, sizeof(CSR_MESH_ETAG_T),
                                                        NVM_OFFSET_DEVICE_ETAG);
    }
}


/*----------------------------------------------------------------------------*
 * NAME
 *    UartDataRxCallback
 *
 * DESCRIPTION
 *     This callback is issued when data is received over UART. Application
 *     may ignore the data, if not required. For more information refer to
 *     the API documentation for the type "uart_data_out_fn"
 *
 * RETURNS
 *     The number of words processed, return data_count if all of the received
 *     data had been processed (or if application don't care about the data)
 *
 *----------------------------------------------------------------------------*/
#ifdef DEBUG_ENABLE
#if 0//
static uint16 UartDataRxCallback ( void* p_data, uint16 data_count,
        uint16* p_num_additional_words )    //mode selection
{   uint8 rx_byte;
    /* If device is not associated, return. */
    if (g_heater_app_data.assoc_state != app_state_associated)
    {
        return data_count;
    }
    uint8 *byte=(uint8 *)p_data;//接收ascii字符缓冲
    rx_byte=byte[0];
    if((rx_byte<'9')&&(rx_byte>'0')) ascii_num=1;//ascii coded number
    if(ascii_num==0)
    {   
        if((uart_status==PENDING_INPUT)&&(currentHeaterStatus==HEATER_STATUS_SEARCH))
        {
            DEBUG_STR("warning:input must be number\r\n");
            return 0;
        }
        
        switch(rx_byte)
        {                
            case 'Y':
            {   DEBUG_STR("CSV file format\r\n");
                //#define CSV_FORMAT 0
                mode_selection=1;
                list_devices=1;
            }
            break;
           
            case 'N':
            {   DEBUG_STR("List file format\r\n");
                //#define LIST_FORMAT 1
                mode_selection=0;
                list_devices=0;
            }
            break;
    
            default:
            break;
        }
    }
    else if(ascii_num==1)
    {   
        if((uart_status==PENDING_INPUT)&&(currentHeaterStatus==HEATER_STATUS_SEARCH))
        {   
            uint8 *number=(uint8 *)p_data;//接收数据缓冲(ASCII)           
            inputNumberOfSensor=string2int(number,data_count);
            printInDecimal(inputNumberOfSensor);
            DEBUG_STR("\r\n");            

            if(inputNumberOfSensor==0)
                DEBUG_STR("warning:input number cannot be 0\r\n");
            else
            {   saveInputSensorNumberToNvm();
                DEBUG_STR("Start searching......\r\n");
                sendCustomCmd(TIMER_RETRY_TIME_LENGTH,TIMER_RETRY_TIME_COUNT,REQ_ID_CMD,0x0000);//timelegth:300ms/timecount:50                            
            }    
        }
    }
    /* Application needs 1 additional data to be received */
    *p_num_additional_words = 1;
    return data_count;
}
#endif//

#endif /* DEBUG_ENABLE */


