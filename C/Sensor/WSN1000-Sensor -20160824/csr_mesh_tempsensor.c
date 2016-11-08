/******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 2.0 Release
 *  Application version 2.0
 *
 *  FILE
 *      csr_mesh_tempsensor.c
 *
 *  DESCRIPTION
 *      This file defines a CSRmesh Temperature Sensor application
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
#include <reset.h>

/*============================================================================*
 *  CSRmesh Header Files
 *============================================================================*/
#include <csr_mesh.h>
#include <bearer_model.h>
#include <sensor_model.h>
#include <attention_model.h>
#include <actuator_model.h>

/*============================================================================*
 *  Local Header Files
 *============================================================================*/
#include "user_config.h"
#include "app_debug.h"
#include "app_gatt.h"
#include "mesh_control_service.h"
#include "gap_service.h"
#include "csr_mesh_tempsensor.h"
#include "csr_mesh_tempsensor_gatt.h"
#include "app_gatt_db.h"
#include "nvm_access.h"
#include "iot_hw.h"
#include "tempsensor_hw.h"
#include "appearance.h"
/*test code 20160421*/
#ifdef ENABLE_BMP180
    #include "sensor_bmp180_barometer.h"
#endif
#include "i2c_sim.h"
/*test code 20160422*/
#ifdef ENABLE_SI7034
    #include "sensor_si7034.h"
#endif 
/*20160704*/
#ifdef ENABLE_BME280
    #include "sensor_bme280.h"
#endif

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
/*20160704*/
#include <battery.h>
/*20160706 test*/
#define RED   2
#define GREEN 3

/*============================================================================*
 *  Private Data Types
 *============================================================================*/
#ifdef ENABLE_ACK_MODE
    typedef struct 
    {
        uint16 dev_id;
        bool ack_recvd;
        uint16 no_response_count;
    }HEATER_INFO_T;
#endif /* ENABLE_ACK_MODE */
/*============================================================================*
 *  Private Definitions
 *============================================================================*/
/* Maximum number of timers */
//#define MAX_APP_TIMERS                 (13 + MAX_CSR_MESH_TIMERS + 
//                                       NUM_TEMP_SENSOR_TIMERS)

    
#define MAX_APP_TIMERS  (25)
    
/* Absolute Difference of two numbers. */
#define ABS_DIFF(x,y) (((x) > (y))?((x) - (y)):((y) - (x)))

/* CSRmesh device UUID size */
#define DEVICE_UUID_SIZE_WORDS         (8)

/* CSRmesh Authorization Code Size in Words */
#define DEVICE_AUTHCODE_SIZE_IN_WORDS  (4)

/* Default device UUID */       //小端存储
/*#define DEFAULT_UUID                   {0x3210, 0x7654, 0xBA98, 0xFEDC,\
                                         0xCDEF, 0x89AB, 0x4567, 0x0123}*/
#define DEFAULT_UUID                {0x8006,0x7777,0x6666,0x5555,\
                                     0x4444,0x3333,0x2222,0x1111}

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
#define NVM_SANITY_MAGIC               (0xAB86) //f802

/*Number of IRKs that application can store */
#define MAX_NUMBER_IRK_STORED          (1)

/* NVM offset for the application NVM version */
#define NVM_OFFSET_APP_NVM_VERSION     (0) //f800

/* NVM offset for NVM sanity word */
#define NVM_OFFSET_SANITY_WORD         (NVM_OFFSET_APP_NVM_VERSION + 1) //f802

/* NVM offset for NVM device uuid */
#define NVM_OFFSET_DEVICE_UUID         (NVM_OFFSET_SANITY_WORD + 1) //f804

/* NVM Offset for Authorization Code */
#define NVM_OFFSET_DEVICE_AUTHCODE     (NVM_OFFSET_DEVICE_UUID + \
                                        DEVICE_UUID_SIZE_WORDS)     //f814

#define NVM_OFFSET_NETWORK_KEY         (NVM_OFFSET_DEVICE_AUTHCODE + \
                                        sizeof(CSR_MESH_AUTH_CODE_T))  //f81c

#define NVM_OFFSET_DEVICE_ID           (NVM_OFFSET_NETWORK_KEY + \
                                        sizeof(CSR_MESH_NETWORK_KEY_T)) //f82c

#define NVM_OFFSET_SEQUENCE_NUMBER     (NVM_OFFSET_DEVICE_ID + 1)       //f82e

#define NVM_OFFSET_DEVICE_ETAG         (NVM_OFFSET_SEQUENCE_NUMBER + 2) //f832

#define NVM_OFFSET_ASSOCIATION_STATE   (NVM_OFFSET_DEVICE_ETAG + \
                                        sizeof(CSR_MESH_ETAG_T))        //f83a

/* NVM Offset for Sensor Model Groups. */   
#define NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS \
                                       (NVM_OFFSET_ASSOCIATION_STATE + 1) //f83c

/* NVM Offset for Attention Model Groups. */    //f844
#define NVM_OFFSET_ATTENTION_MODEL_GROUPS \
                                    (NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS + \
                                    sizeof(sensor_actuator_model_groups)) 

#ifdef ENABLE_DATA_MODEL                                        
    #define NVM_OFFSET_DATA_MODEL_GROUPS \
                                           (NVM_OFFSET_ATTENTION_MODEL_GROUPS + \
                                            sizeof(attention_model_groups)) //f84c

    /* NVM Offset for the Bearer State Data. */
    #define NVM_BEARER_DATA_OFFSET         (NVM_OFFSET_DATA_MODEL_GROUPS + \
                                            sizeof(data_model_groups))      //f856

#else

    /* NVM Offset for the Bearer State Data. */
    #define NVM_BEARER_DATA_OFFSET         (NVM_OFFSET_ATTENTION_MODEL_GROUPS + \
                                            sizeof(attention_model_groups)) //f856
#endif /* ENABLE_DATA_MODEL */

/*20160603*/
#define NVM_SENSOR_STATUS              (NVM_BEARER_DATA_OFFSET + \
                                        sizeof(BEARER_MODEL_STATE_DATA_T)) //f85a
/*20160825*/
#define NVM_SENSOR_SLEEP_STATE         (NVM_SENSOR_STATUS+1)  //f85c

#define NVM_SENSOR_STATE_OFFSET        (NVM_SENSOR_SLEEP_STATE + sizeof(uint16)) //f85e

/* NVM Offset for Sensor State Data */
/*#define NVM_SENSOR_STATE_OFFSET        (NVM_BEARER_DATA_OFFSET + \
                                        sizeof(BEARER_MODEL_STATE_DATA_T))*/

/* Size of Sensor State data to be stored in NVM */
#define SENSOR_SAVED_STATE_SIZE        (2 * sizeof(uint16))

/* Get NVM Offset of a sensor from it's index. */
#define GET_SENSOR_NVM_OFFSET(idx)     (NVM_SENSOR_STATE_OFFSET + \
                                        ((idx) * (SENSOR_SAVED_STATE_SIZE)))

/* NVM Offset for Application data */
#define NVM_MAX_APP_MEMORY_WORDS       (NVM_SENSOR_STATE_OFFSET + \
                                        (NUM_SENSORS_SUPPORTED * \
                                         SENSOR_SAVED_STATE_SIZE))          //f876

/* Timer to check if button was pressed for 1 second */
//#define BUTTON_ONE_SEC_PRESS_TIME      (1 * SECOND)
#define timer_1s      (1 * SECOND)


#define KEY_PRESSED                    (TRUE)

#define KEY_RELEASED                   (FALSE)

/* Number of Supported Sensors. */
#define NUM_SENSORS_SUPPORTED          (6)

/*test code20160422*/
#define CURRENT_AIR_TEMP_IDX           (0)  /*20160704*/
#define CURRENT_BATTERY_IDX            (1)  //CURRENT_AIR_HUMI_IDX
#define CURRENT_PRESSURE_LOW_IDX       (2)
#define CURRENT_PRESSURE_HIGH_IDX      (3)
#define CURRENT_HUMI_LOW_IDX           (4)
#define CURRENT_HUMI_HIGH_IDX          (5)

/* Increment/Decrement Step for Button Press. */
#define STEP_SIZE_PER_BUTTON_PRESS     (32)

/* Macro to convert Celsius to 1/32 kelvin units. */
#define CELSIUS_TO_BY32_KELVIN(x)      (((x)*32) + CELSIUS_TO_KELVIN_FACTOR)

/* Max. Temperature. */
#define MAX_DESIRED_TEMPERATURE        (CELSIUS_TO_BY32_KELVIN(40))

/* Min. Temperature. */
#define MIN_DESIRED_TEMPERATURE        (CELSIUS_TO_BY32_KELVIN(-5))

/* Max sensor type supp in this app sensor_type_desired_air_temperature = 3*/
#define SENSOR_TYPE_SUPPORTED_MAX      (sensor_type_desired_air_temperature)

/* Max transmit msg density */
#define MAX_TRANSMIT_MSG_DENSITY       (5)

#ifdef ENABLE_ACK_MODE
    /* The broadcast id for MESH is defined as 0 */
    #define CSR_MESH_BROADCAST_ID          (0)

    /* The maximum number of heater devices that in grp */
    #define MAX_HEATERS_IN_GROUP           (5)

    /* The heater will be removed from the added list if it does not respond to 
     * maximim no response count times.
     */
    #define MAX_NO_RESPONSE_COUNT          (5)
#endif /* ENABLE_ACK_MODE */


/*============================================================================*
 *  Public Data
 *============================================================================*/

/* CSRmesh Temperature Sensor application data instance */
CSRMESH_TEMPSENSOR_DATA_T g_tsapp_data;

/*TEST CODE 20160412*/
CSRMESH_HUMISENSOR_DATA_T humi_data;
                                       

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
/*20160602*/
static uint16 button_press_poweron_tid=0;
static uint16 button_longpress_send_tid=0;
/*20160801*/
//static uint16 idle_tid=TIMER_INVALID;
/*20160810*/
//static uint16 meshOnRelay_tid=TIMER_INVALID;
static uint16 sleep_tid=TIMER_INVALID;
static uint16 sendData_tid=TIMER_INVALID;

#ifdef ENABLE_FIRMWARE_MODEL
/* Firmware Reset Delay Timer Id */
static timer_id ota_rst_tid = TIMER_INVALID;
#endif /* ENABLE_FIRMWARE_MODEL */

/* Device Apprearance. */
CSR_MESH_APPEARANCE_T device_appearance = {APPEARANCE_ORG_BLUETOOTH_SIG,
                                         APPEARANCE_CSR_MESH_TEMP_SENSOR_VALUE};

/* Device Short name */
uint8 short_name[9] = "Sensor";

/*============================================================================*
 *  Private Data
 *============================================================================*/
/* Declare space for application timers. */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_APP_TIMERS];

/* Sensor Model Data */
static SENSOR_STATE_DATA_T sensor_state[NUM_SENSORS_SUPPORTED];

/* Actuator Model Data */
static ACTUATOR_STATE_DATA_T actuator_state[NUM_SENSORS_SUPPORTED];

#ifdef ENABLE_DATA_MODEL
static uint16 data_model_groups[NUM_DATA_MODEL_GROUPS];
#endif /* ENABLE_DATA_MODEL */

/* Sensor Model Grouping Data. */
static uint16 sensor_actuator_model_groups[NUM_SENSOR_ACTUATOR_MODEL_GROUPS];

/* Attention Model Grouping Data. */
static uint16 attention_model_groups[NUM_ATTENTION_MODEL_GROUPS];

/* Temperature Sensor Sample Timer ID. */
static timer_id tempsensor_sample_tid = TIMER_INVALID;

/* Retransmit Timer ID. */
static timer_id retransmit_tid = TIMER_INVALID;

/* Repeat Interval Timer ID. */
static timer_id repeat_interval_tid = TIMER_INVALID;

/* test code data stream timer ID */
static timer_id data_stream_tid = TIMER_INVALID;


/* Write Value Msg Retransmit counter */
static uint16 write_val_retransmit_count = 0;

/* test code 20160422*/
static uint16 current_air_temp;
static uint16 current_battery_level;

//static uint16 current_humi_high;
static uint16 current_humi_low;
//static uint32 current_air_humi;//20160809
static uint16 current_air_humi;

//static uint16 current_pressure_high;
static uint16 current_pressure_low;
//static uint32 current_air_pressure;//20160809
static uint16 current_air_pressure;

/* Retransmit interval based on the msg transmit density 
 * These values are calculated based on the number of tx msgs added in queue.
 * TRANSMIT_MSG_DENSITY = 1 -> 90ms + (random 0-12.8ms) * 5 -> 500ms.(Approx)
 * TRANSMIT_MSG_DENSITY = 2 -> 45ms + (random 0-12.8ms) * 11 -> 700ms.(Approx)
 * TRANSMIT_MSG_DENSITY = 3 -> 30ms + (random 0-12.8ms) * 17 -> 800ms.(Approx)
 * TRANSMIT_MSG_DENSITY = 4 -> 22.5ms + (random 0-12.8ms) * 23 -> 900ms.(Approx)
 * TRANSMIT_MSG_DENSITY = 5 -> 20ms + (random 0-12.8ms) * 35 -> 1100ms.(Approx)
 */
static uint32 retransmit_interval[MAX_TRANSMIT_MSG_DENSITY]={500 * MILLISECOND,
                                                             700 * MILLISECOND,
                                                             800 * MILLISECOND,
                                                             900 * MILLISECOND,
                                                            1100 * MILLISECOND};

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
    /* Stores the device info of the heaters participating in the group.*/
    static HEATER_INFO_T heater_list[MAX_HEATERS_IN_GROUP];
#endif /* ENABLE_ACK_MODE */

/*test code*/
//static uint8 teststream[]={0X11,0X12,0X13,0X14,0X15,0X16,0X17,0X18};
//uint16 sleepOrRelayCounter=0;
uint16 sleepState;
/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/
static void appDataInit(void);
static void readPersistentStore(void);
/*20160811*/
static void readPersistentStore1(void);
/*20160825*/
static void readPersistentStore2(void);
static void handleSignalLmEvConnectionComplete(
                                     LM_EV_CONNECTION_COMPLETE_T *p_event_data);
static void handleSignalLmConnectionUpdate(
                                     LM_EV_CONNECTION_UPDATE_T* p_event_data);
static void handleGapCppTimerExpiry(timer_id tid);
static void deviceIdAdvertTimeoutHandler(timer_id tid);
static void retransmitIntervalTimerHandler(timer_id tid);
static void startRetransmitTimer(void);
/*test code*/
//static void dataStreamIntervalTimeoutHandler(timer_id tid);
//static void dataStreamIntervalTimeoutHandler1(timer_id tid);
static void writeSensorDataToNVM(uint16 idx);

static bool IsSensorConfigured(void);

void handleLongButtonPress(timer_id tid);
void handleExitRegister(void);
void handleRegisterTimerExpire(timer_id tid);
void handleReceiveCmdTimeout(timer_id tid);

//20160720
//static void handleReadData(timer_id tid);
static void timerHandleStartBME280ForceMode(timer_id tid);
//static void sendDataInd(timer_id tid);
/*20160801*/
void handleLEDFlash1S(timer_id tid);
void slowBlinkTimerHandler(timer_id tid);
/*20160809*/
static void turnOFFMeshTimerHandler(timer_id tid);
/*20160824*/
static void changeDutyCycleTimerHandler(timer_id tid);

//static void sendSensorDataTimerHandler(timer_id tid);

#ifdef ENABLE_TEMPERATURE_CONTROLLER
    #ifdef DEBUG_ENABLE
        /* UART Receive callback */
        static uint16 UartDataRxCallback (void* p_data, uint16 data_count,
                                          uint16* p_num_additional_words);
    #endif /* DEBUG_ENABLE */
#endif /* ENABLE_TEMPERATURE_CONTROLLER */


#ifdef ENABLE_ACK_MODE
    static void resetHeaterList(void);
    static void resetAckInHeaterList(void);
#endif /* ENABLE_ACK_MODE */
    
/*test code 20160419*/
#ifdef PRESET_DEVID
    static void presetDevid(void);
#endif
   
/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/  
/*20160705*/
void saveSensorStatusToNVM()
{
    Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
}
    
/*20160530*/
void handleLongButtonPress(timer_id tid)
{   
    if(PioGet(PUSH_BUTTON)==FALSE) //>5s
    {   
        PioEnablePWM(1,FALSE);
        PioSetMode(SEND_IND,pio_mode_user);
        
        PioSet(SEND_IND,0);
        ledFlashtid=TimerCreate(1*SECOND,TRUE,handleLEDFlash1S);//idle
        //SleepModeChange(sleep_mode_never);
        //idle_tid=TimerCreate(20*SECOND,TRUE,slowBlinkTimerHandler);
        
        currentSensorStatus=SENSOR_STATUS_INIT;//超时处理--退出注册
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    }
    /*else //<5s
    {   if(streamCFM==TRUE) 
        {   
            currentSensorStatus=SENSOR_STATUS_NORMAL;
            Nvm_Write((uint16 *)&sensor_status,1,NVM_SENSOR_STATUS);
        }
        else 
        {   sensor_status = sensor_status_init;//待注册模式   
        }
    }*/
}
void handleRegisterTimerExpire(timer_id tid)
{   
    timer_id tid0=tid;
    tid0=TIMER_INVALID;
    TimerDelete(tid0);
    
    currentSensorStatus=SENSOR_STATUS_ERROR;
    Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    errorHandler(CANNOT_REC_DEVID_CMD);
}

void handleReceiveCmdTimeout(timer_id tid)
{   timer_id tid0=tid;
    TimerDelete(tid0);
    tid0=TIMER_INVALID;
    warning();
}


#if 0
static void sendDataInd(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;
    PioSet(SEND_IND,0);//off 
}
#endif

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
#if 0    
    static void printInDecimal(uint32 val)
    {
        if(val >= 10)
        {
            printInDecimal(val/10);
        }
            DebugWriteChar(('0' + (val%10)));
    }
#endif
    #else
    #define printInDecimal(n)
#endif /* DEBUG_ENABLE */

#ifdef ENABLE_TEMPERATURE_CONTROLLER
    #ifdef DEBUG_ENABLE

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
        static uint16 UartDataRxCallback ( void* p_data, uint16 data_count,
                uint16* p_num_additional_words )
        {
            
            /* Application needs 1 additional data to be received */
            *p_num_additional_words = 1;
        
            /* If device is not associated, return. */
            if (g_tsapp_data.assoc_state != app_state_associated)
            {
                return data_count;
            }
            return data_count;
        }
    
    #endif /* DEBUG_ENABLE */
#endif /* ENABLE_TEMPERATURE_CONTROLLER */
/*----------------------------------------------------------------------------*
 *  NAME
 *      readSensorDataFromNVM
 *
 *  DESCRIPTION
 *      This function reads sensor state data from NVM into state variable.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
//#if 0        
static void readSensorDataFromNVM(uint16 idx)
{
    Nvm_Read((uint16*)(sensor_state[idx].value), 
             sizeof(uint16),
             (GET_SENSOR_NVM_OFFSET(idx)));

    Nvm_Read((uint16*)&(sensor_state[idx].repeat_interval), 
             sizeof(uint8),
             (GET_SENSOR_NVM_OFFSET(idx) + sizeof(uint8)));

}
//#endif
/*----------------------------------------------------------------------------*
 *  NAME
 *      writeSensorDataToNVM
 *
 *  DESCRIPTION
 *      This function writes sensor state data from state variable into NVM.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void writeSensorDataToNVM(uint16 idx)
{
    Nvm_Write((uint16*)(sensor_state[idx].value), 
              sizeof(uint16),
              (GET_SENSOR_NVM_OFFSET(idx)));

    Nvm_Write((uint16*)&(sensor_state[idx].repeat_interval), 
              sizeof(uint8),
              (GET_SENSOR_NVM_OFFSET(idx) + sizeof(uint8)));
}

#ifdef ENABLE_ACK_MODE
    /*----------------------------------------------------------------------------*
     *  NAME
     *      resetHeaterList
     *
     *  DESCRIPTION
     *      The function resets the device id and the ack flag of complete db
     *
     *  RETURNS
     *      Nothing.
     *
     *---------------------------------------------------------------------------*/
    static void resetHeaterList(void)
    {
        uint16 idx;
    
        for(idx=0; idx < MAX_HEATERS_IN_GROUP; idx++)
        {
            heater_list[idx].dev_id = CSR_MESH_BROADCAST_ID;
            heater_list[idx].ack_recvd = FALSE;
            heater_list[idx].no_response_count = 0;
        }
    }
    /*----------------------------------------------------------------------------*
     *  NAME
     *      resetAckInHeaterList
     *
     *  DESCRIPTION
     *      The function resets the ack flag of complete db with valid dev id
     *
     *  RETURNS
     *      Nothing.
     *
     *---------------------------------------------------------------------------*/
    static void resetAckInHeaterList(void)
    {
        uint16 idx;
    
        for(idx=0; idx < MAX_HEATERS_IN_GROUP; idx++)
        {
            if(heater_list[idx].dev_id != CSR_MESH_BROADCAST_ID)
            {
                heater_list[idx].ack_recvd = FALSE;
            }
        }
    }
#endif /* ENABLE_ACK_MODE */
    
    
static void timerHandleStartBME280ForceMode(timer_id tid)
{
    TimerDelete(sendData_tid);sendData_tid=TIMER_INVALID;//    
    SleepModeChange(sleep_mode_never);//20160809
    CsrMeshEnableListening(TRUE);
    //CsrMeshStart();
    BME280_Trigger();
    TimerCreate(TIMER_SENSOR_READ_INIT_INTERVAL,TRUE,timerHandleReadBME280Data);
}    
    
void timerHandleReadBME280Data(timer_id tid)
{
     TimerDelete(tid);
     tid=TIMER_INVALID;
     uint32 air_temp=0;
     uint32 air_pressure=0;
     uint32 air_humi=0;//20160809
   
    BME280_compensation_int32(&air_temp,&air_pressure,\
                              &air_humi);     
    current_air_temp     =(uint16)(air_temp&0xffff);
    current_air_pressure =air_pressure/100;//丢弃最低两位
    current_air_humi     =air_humi>>4;//丢弃最低一位精度

    //20160808
    PioSetModes(0x0e10,pio_mode_user);//0x1110 0001 0000
    PioSetDirs(0x0e10,0x0e10);
    PioSetPullModes(0x0e10,pio_mode_strong_pull_up); 

    uint16 delayCount=20;
    while(delayCount--)
        TimeDelayUSec(50000);//50ms
    
    if(0) TimerCreate(5*SECOND,TRUE,changeDutyCycleTimerHandler);

    writeTempValue();/*readout and transmit*/
}

static void changeDutyCycleTimerHandler(timer_id tid)
{
    TimerDelete(tid);
    tid=TIMER_INVALID;
    CsrMeshEnableListening(FALSE);
    
    CSR_MESH_ADVSCAN_PARAM_T param;
    CsrMeshGetAdvScanParam(&param);
    param.scan_duty_cycle = CUSTOM_RX_DUTY_CYCLE;
    CsrMeshSetAdvScanParam(&param);  
    CsrMeshEnableListening(TRUE);
}
/*----------------------------------------------------------------------------*
 *  NAME
 *      writeTempValue
 *
 *  DESCRIPTION
 *      This function writes the current and the desired temp values onto the 
 *      groups.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
void writeTempValue()
{
    uint16 index, index1;   
    
    /*test code20160422*/
    sensor_type_t sensorTypeTemp[1] = {sensor_type_internal_air_temperature};//1
    sensor_type_t sensorTypeHumi[1] = {sensor_type_internal_humidity};//4   
    sensor_type_t sensorTypePress[1]= {sensor_type_barometric_pressure};//58
    
    bool ack_reqd = FALSE;
 
    #ifdef ENABLE_ACK_MODE 
        ack_reqd = TRUE;
    #endif /* ENABLE_ACK_MODE */      
        
    for(index1 = 0; index1 < TRANSMIT_MSG_DENSITY; index1 ++)
    {
        for(index = 0; index < NUM_SENSOR_ACTUATOR_MODEL_GROUPS; index++)
        {            
            if(sensor_actuator_model_groups[index] != 0)
            {                                   
                
                #ifdef ENABLE_BME280
                    
                    SensorWriteValue(sensor_actuator_model_groups[index], 
                                     ack_reqd, 
                                     sensorTypeTemp,
                                     1,
                                     &g_tsapp_data.sensor_data);
                    SensorWriteValue(sensor_actuator_model_groups[index], 
                                     ack_reqd, 
                                     sensorTypeHumi,
                                     1,
                                     &g_tsapp_data.sensor_data);
                    SensorWriteValue(sensor_actuator_model_groups[index], 
                                     ack_reqd, 
                                     sensorTypePress,
                                     1,
                                     &g_tsapp_data.sensor_data);/**/

                #endif                    
            }
        }
    }

    //SleepModeChange(sleep_mode_deep);//20160809
    //CsrMeshEnableListening(FALSE);
    //sendData_tid=TimerCreate(TIMER_SENDDATA_INTERVAL,TRUE,timerHandleStartBME280ForceMode);
    //TimerCreate(TIMER_MESHON_NOT_SLEEP,TRUE,turnOFFMeshTimerHandler);//20160811
}

/*
static void sendSensorDataTimerHandler(timer_id tid)
{   

    BME280_Trigger();
    TimerCreate(TIMER_SENSOR_READ_INIT_INTERVAL,TRUE,timerHandleReadBME280Data);
}*/


static void turnOFFMeshTimerHandler(timer_id tid)
{
    TimerDelete(tid);tid=TIMER_INVALID;
    //turn off SPI
    PioSetModes(0x0e10,pio_mode_user);//0x1110 0001 0000
    PioSetDirs(0x0e10,0x0e10);
    PioSetPullModes(0x0e10,pio_mode_strong_pull_up); 
    
    SleepModeChange(sleep_mode_deep);
    CsrMeshEnableListening(FALSE);
    
    CSR_MESH_ADVSCAN_PARAM_T param;//20160831
    CsrMeshGetAdvScanParam(&param);
    param.scan_duty_cycle = OFF_RX_DUTY_CYCLE;
    CsrMeshSetAdvScanParam(&param);
    CsrMeshEnableListening( TRUE );
        
    ledFlashFlag=0;    
    if(currentSensorStatus==SENSOR_STATUS_NORMAL)
    {
        PioSetMode(SEND_IND,pio_mode_user);
        PioSet(SEND_IND,0);//OFF
    }
    sleep_tid=TimerCreate(meshOFFSleepTime*MINUTE,TRUE,turnONMeshTimerHandler);//SECOND
    //sleep_tid=TimerCreate(TIMER_MESHOFF_SLEEP,TRUE,turnONMeshTimerHandler);//sleep
}

void turnONMeshTimerHandler(timer_id tid)
{
    TimerDelete(sleep_tid);tid=TIMER_INVALID;// 

    ledFlashFlag=1;
    if(currentSensorStatus==SENSOR_STATUS_NORMAL)
    {
        PioSetMode(SEND_IND,pio_mode_user);
        PioSet(SEND_IND,1);//ON
    }    
    SleepModeChange(sleep_mode_never);
    
    CSR_MESH_ADVSCAN_PARAM_T param;//20160831
    CsrMeshEnableListening( FALSE );
    CsrMeshGetAdvScanParam(&param);
    param.scan_duty_cycle = CUSTOM_RX_DUTY_CYCLE;
    CsrMeshSetAdvScanParam(&param);

    CsrMeshEnableListening(TRUE);
    TimerCreate(meshONNotSleepTime*MINUTE,TRUE,turnOFFMeshTimerHandler);//SECOND
    //TimerCreate(TIMER_MESHON_NOT_SLEEP,TRUE,turnOFFMeshTimerHandler);    
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
        bool start_timer = TRUE;

        retransmit_tid = TIMER_INVALID;

        /* transmit the pending message to all the groups*/
        writeTempValue();

        write_val_retransmit_count --;

        #ifdef ENABLE_ACK_MODE 
                /* After half of the max retransmissions are over then check whether
                 * ack has been received from all the heaters stored and if so then
                 * stop sending the packet as we have received acks for all heaters.
                 */
                if(write_val_retransmit_count < (NUM_OF_RETRANSMISSIONS/2))
                {
                    uint8 idx;
                    for(idx=0; idx < MAX_HEATERS_IN_GROUP; idx++)
                    {
                        if(heater_list[idx].dev_id != CSR_MESH_BROADCAST_ID &&
                           heater_list[idx].ack_recvd == FALSE)
                        {
                            break;
                        }
                        if(idx == (MAX_HEATERS_IN_GROUP-1))
                        {
                            start_timer = FALSE;
                            DEBUG_STR(" RECVD ALL ACK'S STOP TIMER : ");
                        }
                    }
                    /* One or more devices have not acked back increase the no response
                     * count. If the no response count reaches the maximum, remove the
                     * device from the heater list.
                     */
                    if(write_val_retransmit_count == 0)
                    {
                        for(idx=0; idx < MAX_HEATERS_IN_GROUP; idx++)
                        {
                            if(heater_list[idx].dev_id != CSR_MESH_BROADCAST_ID &&
                               heater_list[idx].ack_recvd == FALSE)
                            {
                                heater_list[idx].no_response_count++;
                                if(heater_list[idx].no_response_count >= 
                                                                MAX_NO_RESPONSE_COUNT)
                                {
                                    heater_list[idx].dev_id = CSR_MESH_BROADCAST_ID;
                                    heater_list[idx].no_response_count = 0;
                                }
                            }
                        }
                    }
                }
        #endif /* ENABLE_ACK_MODE */ 
 
        if(start_timer == TRUE)
        {
            /* start a timer to send the broadcast sensor data */
            startRetransmitTimer();
        }
    }
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      startRetransmitTimer
 *
 *  DESCRIPTION
 *      This function starts the broadcast timer for the current tempertature.
 *
 *  RETURNS
 *      None
 *
 *----------------------------------------------------------------------------*/
static void startRetransmitTimer(void)
{
    if(write_val_retransmit_count > 0)
    {
        retransmit_tid=TimerCreate(retransmit_interval[TRANSMIT_MSG_DENSITY-1],
                                   TRUE,retransmitIntervalTimerHandler);
    }
}
#if 0
/*test code*/
static void dataStreamIntervalTimeoutHandler(timer_id tid)
{
    if (tid == data_stream_tid)
    {
        data_stream_tid = TIMER_INVALID;

        /*test code*/        
        if(g_tsapp_data.assoc_state == app_state_associated)
        {
            DEBUG_STR("\r\nSend Stream\r\n");      
            StreamStartSender(0x0000);
            StreamSendData(&teststream[0], 8);
            DebugWriteUint8(teststream[0]);       
            DebugWriteUint8(teststream[1]);
            DebugWriteUint8(teststream[2]);
            DebugWriteUint8(teststream[3]);
            DebugWriteUint8(teststream[4]);
            DebugWriteUint8(teststream[5]);
            DebugWriteUint8(teststream[6]);
            DebugWriteUint8(teststream[7]);       
            DEBUG_STR("\r\n");
            datasentflag=0;
            EnableHighDutyScanMode(TRUE);
        }
        
        data_stream_tid = TimerCreate(  0.1*SECOND, 
                                        TRUE,
                                        dataStreamIntervalTimeoutHandler1);

    }
}

static void dataStreamIntervalTimeoutHandler1(timer_id tid)
{       
        /* Start the timer for next sample. */
        data_stream_tid = TimerCreate(  0.1*SECOND, 
                                        TRUE,
                                        dataStreamIntervalTimeoutHandler1);
}
#endif

/*-----------------------------------------------------------------------------*
 *  NAME
 *      IsSensorConfigured
 *
 *  DESCRIPTION
 *      This below function returns whether the sensor is configured or not
 *
 *  RETURNS/MODIFIES
 *      TRUE if the sensor has been grouped otherwise returns FALSE
 *
 *----------------------------------------------------------------------------*/
static bool IsSensorConfigured(void)
{
    uint16 index;

    for(index = 0; index < NUM_SENSOR_ACTUATOR_MODEL_GROUPS; index++)
    {
        if(sensor_actuator_model_groups[index] != 0)
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
    if(tid == g_tsapp_data.mesh_device_id_advert_tid)
    {
        g_tsapp_data.mesh_device_id_advert_tid = TIMER_INVALID;
        /* Start the timer only if the device is not associated */
        if(g_tsapp_data.assoc_state == app_state_not_associated)
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
            g_tsapp_data.mesh_device_id_advert_tid = TimerCreate(
                                       DEVICE_ID_ADVERT_TIMER_ID + random_delay, 
                                       TRUE,
                                       deviceIdAdvertTimeoutHandler);
        }
    }
}

//20160602
void errorHandler(uint16 error_code)
{
    DEBUG_STR("err");
    DEBUG_U16(error_code);
    DEBUG_STR("\r\n");
    switch(error_code)
    {    
        case CANNOT_REC_DEVID_CMD:
            currentSensorStatus=SENSOR_STATUS_ERROR;
            Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
        break;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      appDataInit
 *
 *  DESCRIPTION
 *      This function is called to initialise CSRmesh Temperature Sensor
 *      application data structure.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
static void appDataInit(void)
{
    /* Reset/Delete all the timers */
    TimerDelete(g_tsapp_data.gatt_data.app_tid);
    g_tsapp_data.gatt_data.app_tid = TIMER_INVALID;

    TimerDelete(g_tsapp_data.gatt_data.con_param_update_tid);
    g_tsapp_data.gatt_data.con_param_update_tid = TIMER_INVALID;
    g_tsapp_data.gatt_data.cpu_timer_value = 0;

    g_tsapp_data.gatt_data.st_ucid = GATT_INVALID_UCID;

    g_tsapp_data.gatt_data.advert_timer_value = 0;

    /* Reset the connection parameter variables. */
    g_tsapp_data.gatt_data.conn_interval = 0;
    g_tsapp_data.gatt_data.conn_latency = 0;
    g_tsapp_data.gatt_data.conn_timeout = 0;

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
 *      and also gives visual indication that Temperature Sensor is not 
 *      associated.
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
    g_tsapp_data.mesh_device_id_advert_tid = TimerCreate(
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

    if( app_nvm_version != APP_NVM_VERSION )    //first time
    {
        /* The NVM structure could have changed
         * with a new version of the application, due to NVM values getting
         * removed or added. Currently this application clears all application
         * and CSRmesh NVM values and writes to NVM
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
    g_tsapp_data.gatt_data.paired = FALSE;
//    sleepOrRelayCounter=0;//20160810

    if(nvm_sanity == NVM_SANITY_MAGIC)  //second time
    {
        
       //20160705
        //Nvm_Read(&currentSensorStatus,1,NVM_SENSOR_STATUS);        
        
        /* Read association state from NVM */
        Nvm_Read((uint16 *)&g_tsapp_data.assoc_state,
                sizeof(g_tsapp_data.assoc_state),
                NVM_OFFSET_ASSOCIATION_STATE);

        /* Read Bearer Model Data from NVM */
        Nvm_Read((uint16 *)&g_tsapp_data.bearer_data,
                 sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);
    
        
        /* Read Sensor State Data. */
        for (i = 0; i < NUM_SENSORS_SUPPORTED; i++)
        {
            readSensorDataFromNVM(i);
        }

        /* Read assigned Groups IDs for Sensor model from NVM */
        Nvm_Read((uint16 *)sensor_actuator_model_groups, 
                                    sizeof(sensor_actuator_model_groups),
                                    NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS);

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

    else 
    /* NVM Sanity check failed means either the device is being brought up
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
        
         //20160705       
        currentSensorStatus=SENSOR_STATUS_INIT;
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
        

        if (cskey_flags & RANDOM_UUID_ENABLE_MASK)
        {
            /* The flag is set so generate a random UUID NVM */
            for( i = 0 ; i < 8 ; i++)
            {
                g_node_data.device_uuid.uuid[i] = Random16();
            }
        }
        /* Write to NVM */
        Nvm_Write(g_node_data.device_uuid.uuid, DEVICE_UUID_SIZE_WORDS,
                  NVM_OFFSET_DEVICE_UUID);

        /* Update Bearer Model Data from CSKey flags for the first time. */
        g_tsapp_data.bearer_data.bearerPromiscuous = 0x0000;
        g_tsapp_data.bearer_data.bearerEnabled     = BLE_BEARER_MASK;
        g_tsapp_data.bearer_data.bearerRelayActive = 0x0000;

        if (cskey_flags & RELAY_ENABLE_MASK)
        {
            g_tsapp_data.bearer_data.bearerRelayActive |= BLE_BEARER_MASK;
            g_tsapp_data.bearer_data.bearerPromiscuous |= BLE_BEARER_MASK;
        }

        if (cskey_flags & BRIDGE_ENABLE_MASK)
        {
            g_tsapp_data.bearer_data.bearerEnabled     |= 
                                                    BLE_GATT_SERVER_BEARER_MASK;
            g_tsapp_data.bearer_data.bearerRelayActive |= 
                                                    BLE_GATT_SERVER_BEARER_MASK;
            g_tsapp_data.bearer_data.bearerPromiscuous |= 
                                                    BLE_GATT_SERVER_BEARER_MASK;
        }

        /* Update Bearer Model Data to NVM */
        Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
                  sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

        #ifdef USE_AUTHORIZATION_CODE
                /* Write Authorization Code to NVM */
                Nvm_Write(g_node_data.auth_code.auth_code,DEVICE_AUTHCODE_SIZE_IN_WORDS,
                          NVM_OFFSET_DEVICE_AUTHCODE);
        #endif /* USE_AUTHORIZATION_CODE */

        /* The device will not be associated as it is coming up for the
         * first time
         */
        g_tsapp_data.assoc_state = app_state_not_associated;

        /* Write association state to NVM */
        Nvm_Write((uint16 *)&g_tsapp_data.assoc_state,
                 sizeof(g_tsapp_data.assoc_state),
                 NVM_OFFSET_ASSOCIATION_STATE);

        /* Write Sensor State data to NVM. */
        for (i = 0; i < NUM_SENSORS_SUPPORTED; i++)
        {
            writeSensorDataToNVM(i);
        }

        MemSet(sensor_actuator_model_groups, 0x0000, 
                                        sizeof(sensor_actuator_model_groups)); 

        /* Write assigned Groups IDs for Sensor model from NVM */
        Nvm_Write((uint16 *)sensor_actuator_model_groups, 
                                    sizeof(sensor_actuator_model_groups),
                                    NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS);

        MemSet(attention_model_groups, 0x0000, sizeof(attention_model_groups));

        /* Write assigned Groups IDs for Sensor model from NVM */
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

    //20160722
    //currentSensorStatus=SENSOR_STATUS_NORMAL;
    //Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    Nvm_Read(&currentSensorStatus,1,NVM_SENSOR_STATUS); 
        
    /* Read the UUID from NVM */
    Nvm_Read(g_node_data.device_uuid.uuid, DEVICE_UUID_SIZE_WORDS, 
                       NVM_OFFSET_DEVICE_UUID);

    #ifdef USE_AUTHORIZATION_CODE
        /* Read Authorization Code from NVM */
        Nvm_Read(g_node_data.auth_code.auth_code, DEVICE_AUTHCODE_SIZE_IN_WORDS,
                 NVM_OFFSET_DEVICE_AUTHCODE);
        g_node_data.use_authorisation = TRUE;
    #endif /* USE_AUTHORIZATION_CODE */

    Nvm_Read((uint16 *)&g_tsapp_data.assoc_state,1, 
                                                  NVM_OFFSET_ASSOCIATION_STATE);

    if(g_tsapp_data.assoc_state == app_state_associated)
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
        /* As device is already associated set LE bearer to non-promiscuous.*/
        g_tsapp_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
    }

}

static void readPersistentStore1(void)
{    
    /* NVM offset for supported services */
    uint16 nvm_offset = 0;
    uint16 nvm_sanity = 0xffff;
    //bool nvm_start_fresh = FALSE;
    uint16 app_nvm_version;
    uint16 i;

    nvm_offset = NVM_MAX_APP_MEMORY_WORDS;

    /* Read the Application NVM version */
    Nvm_Read(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);

    if( app_nvm_version != APP_NVM_VERSION )    //first time
    {
        uint16 eeprom_erase = 0xFFFF;
        /* Erase a block in EEPROM to remove all sanity words */
        for (i = 0; i < 128; i++)
        {
            Nvm_Write(&eeprom_erase, 0x1, i);
        }                 
        
        app_nvm_version = APP_NVM_VERSION;
        Nvm_Write(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);
    }      
    
    /* Read the NVM sanity word to check if the NVM validity */
    Nvm_Read(&nvm_sanity, sizeof(nvm_sanity),
             NVM_OFFSET_SANITY_WORD);
    
    if(nvm_sanity == NVM_SANITY_MAGIC)  //second time
    {                
        Nvm_Read(&currentSensorStatus,1,NVM_SENSOR_STATUS);
        
          uint16 eeprom_erase = 0xFFFF;
                /* Erase a block in EEPROM to remove all sanity words */
          for (i = 0; i < 128; i++)
          {
               Nvm_Write(&eeprom_erase, 0x1, i);
           }                 
        
        app_nvm_version = APP_NVM_VERSION;
        Nvm_Write(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);
        nvm_sanity = NVM_SANITY_MAGIC;
        Nvm_Write(&nvm_sanity, sizeof(nvm_sanity),
                  NVM_OFFSET_SANITY_WORD);        
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    }

    else                                        //first time(save)    
    {
        nvm_sanity = NVM_SANITY_MAGIC;
        Nvm_Write(&nvm_sanity, sizeof(nvm_sanity),
                  NVM_OFFSET_SANITY_WORD);      //save
        currentSensorStatus=SENSOR_STATUS_INIT;
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    }
}

static void readPersistentStore2(void)
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

    if( app_nvm_version != APP_NVM_VERSION )    //first time
    {
        uint16 eeprom_erase = 0xFFFF;
        /* Erase a block in EEPROM to remove all sanity words */
        for (i = 0; i < 128; i++)
        {
            Nvm_Write(&eeprom_erase, 0x1, i);
        }                 
        
        app_nvm_version = APP_NVM_VERSION;
        Nvm_Write(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);
    }      

    /* Read the NVM sanity word to check if the NVM validity */
    Nvm_Read(&nvm_sanity, sizeof(nvm_sanity),
             NVM_OFFSET_SANITY_WORD);
    g_tsapp_data.gatt_data.paired = FALSE;//20160825
    
    if(nvm_sanity == NVM_SANITY_MAGIC)  //second time
    {                
        Nvm_Read(&currentSensorStatus,1,NVM_SENSOR_STATUS);
        
        /* Read association state from NVM */
        Nvm_Read((uint16 *)&g_tsapp_data.assoc_state,
                sizeof(g_tsapp_data.assoc_state),
                NVM_OFFSET_ASSOCIATION_STATE);

        /* Read Bearer Model Data from NVM */
        Nvm_Read((uint16 *)&g_tsapp_data.bearer_data,
                 sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);
        
        Nvm_Read((uint16 *)sensor_actuator_model_groups, 
                            sizeof(sensor_actuator_model_groups),
                            NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS);

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
        
          uint16 eeprom_erase = 0xFFFF;
                /* Erase a block in EEPROM to remove all sanity words */
          for (i = 0; i < 128; i++)
          {
               Nvm_Write(&eeprom_erase, 0x1, i);
           }                 
        
        app_nvm_version = APP_NVM_VERSION;
        Nvm_Write(&app_nvm_version, 1, NVM_OFFSET_APP_NVM_VERSION);
        nvm_sanity = NVM_SANITY_MAGIC;
        Nvm_Write(&nvm_sanity, sizeof(nvm_sanity),
                  NVM_OFFSET_SANITY_WORD);        
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    }

    else                                        //first time(save)    
    {
        /* Read Configuration flags from User CS Key */
        uint16 cskey_flags = CSReadUserKey(CSKEY_INDEX_USER_FLAGS);
        nvm_start_fresh = TRUE;
        
        nvm_sanity = NVM_SANITY_MAGIC;
        Nvm_Write(&nvm_sanity, sizeof(nvm_sanity),
                  NVM_OFFSET_SANITY_WORD);      //save
        currentSensorStatus=SENSOR_STATUS_INIT;
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
        
        if (cskey_flags & RANDOM_UUID_ENABLE_MASK)
        {
            /* The flag is set so generate a random UUID NVM */
            for( i = 0 ; i < 8 ; i++)
            {
                g_node_data.device_uuid.uuid[i] = Random16();
            }
        }
        /* Write to NVM */
        Nvm_Write(g_node_data.device_uuid.uuid, DEVICE_UUID_SIZE_WORDS,
                  NVM_OFFSET_DEVICE_UUID);

        /* Update Bearer Model Data from CSKey flags for the first time. */
        g_tsapp_data.bearer_data.bearerPromiscuous = 0x0000;
        g_tsapp_data.bearer_data.bearerEnabled     = BLE_BEARER_MASK;
        g_tsapp_data.bearer_data.bearerRelayActive = 0x0000;

        if (cskey_flags & RELAY_ENABLE_MASK)
        {
            g_tsapp_data.bearer_data.bearerRelayActive |= BLE_BEARER_MASK;
            g_tsapp_data.bearer_data.bearerPromiscuous |= BLE_BEARER_MASK;
        }

        if (cskey_flags & BRIDGE_ENABLE_MASK)
        {
            g_tsapp_data.bearer_data.bearerEnabled     |= 
                                                    BLE_GATT_SERVER_BEARER_MASK;
            g_tsapp_data.bearer_data.bearerRelayActive |= 
                                                    BLE_GATT_SERVER_BEARER_MASK;
            g_tsapp_data.bearer_data.bearerPromiscuous |= 
                                                    BLE_GATT_SERVER_BEARER_MASK;
        }

        /* Update Bearer Model Data to NVM */
        Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
                  sizeof(BEARER_MODEL_STATE_DATA_T), NVM_BEARER_DATA_OFFSET);

        #ifdef USE_AUTHORIZATION_CODE
                /* Write Authorization Code to NVM */
                Nvm_Write(g_node_data.auth_code.auth_code,DEVICE_AUTHCODE_SIZE_IN_WORDS,
                          NVM_OFFSET_DEVICE_AUTHCODE);
        #endif /* USE_AUTHORIZATION_CODE */

        /* The device will not be associated as it is coming up for the
         * first time
         */
        g_tsapp_data.assoc_state = app_state_not_associated;

        /* Write association state to NVM */
        Nvm_Write((uint16 *)&g_tsapp_data.assoc_state,
                 sizeof(g_tsapp_data.assoc_state),
                 NVM_OFFSET_ASSOCIATION_STATE);
 
        MemSet(sensor_actuator_model_groups, 0x0000, 
                                        sizeof(sensor_actuator_model_groups)); 

        /* Write assigned Groups IDs for Sensor model from NVM */
        Nvm_Write((uint16 *)sensor_actuator_model_groups, 
                                    sizeof(sensor_actuator_model_groups),
                                    NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS);

        MemSet(attention_model_groups, 0x0000, sizeof(attention_model_groups));

        /* Write assigned Groups IDs for Sensor model from NVM */
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
    /* Read the UUID from NVM */
    Nvm_Read(g_node_data.device_uuid.uuid, DEVICE_UUID_SIZE_WORDS, 
                       NVM_OFFSET_DEVICE_UUID);

    #ifdef USE_AUTHORIZATION_CODE
        /* Read Authorization Code from NVM */
        Nvm_Read(g_node_data.auth_code.auth_code, DEVICE_AUTHCODE_SIZE_IN_WORDS,
                 NVM_OFFSET_DEVICE_AUTHCODE);
        g_node_data.use_authorisation = TRUE;
    #endif /* USE_AUTHORIZATION_CODE */

    Nvm_Read((uint16 *)&g_tsapp_data.assoc_state,1, 
                                                  NVM_OFFSET_ASSOCIATION_STATE);

    if(g_tsapp_data.assoc_state == app_state_associated)
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
        /* As device is already associated set LE bearer to non-promiscuous.*/
        g_tsapp_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
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
        if(g_tsapp_data.assoc_state == app_state_associated)
        {
            /* Stop blink */
            IOTLightControlDevicePower(FALSE);

            /* Set back the scan to low duty cycle only if the device has
             * already been grouped.
             */
            EnableHighDutyScanMode(FALSE);
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

    if(g_tsapp_data.gatt_data.con_param_update_tid == tid)
    {
        g_tsapp_data.gatt_data.con_param_update_tid = TIMER_INVALID;
        g_tsapp_data.gatt_data.cpu_timer_value = 0;

        /*Handling signal as per current state */
        switch(g_tsapp_data.state)
        {
            case app_state_connected:
            {
                /* Increment the count for Connection Parameter Update
                 * requests
                 */
                ++ g_tsapp_data.gatt_data.num_conn_update_req;

                /* If it is first or second request, preferred connection
                 * parameters should be request
                 */
                if(g_tsapp_data.gatt_data.num_conn_update_req == 1 ||
                   g_tsapp_data.gatt_data.num_conn_update_req == 2)
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
                else if(g_tsapp_data.gatt_data.num_conn_update_req == 3 ||
                        g_tsapp_data.gatt_data.num_conn_update_req == 4)
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
                                            &g_tsapp_data.gatt_data.con_bd_addr,
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
    if(g_tsapp_data.gatt_data.con_param_update_tid == tid)
    {
        g_tsapp_data.gatt_data.con_param_update_tid =
                           TimerCreate(TGAP_CPC_PERIOD, TRUE,
                                       requestConnParamUpdate);
        g_tsapp_data.gatt_data.cpu_timer_value = TGAP_CPC_PERIOD;
    }
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      appAdvertisingExit
 *
 *  DESCRIPTION
 *      This function is called while exiting app_state_advertising
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
    TimerDelete(g_tsapp_data.gatt_data.app_tid);
    g_tsapp_data.gatt_data.app_tid = TIMER_INVALID;
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
    switch(g_tsapp_data.state)
    {
        case app_state_init:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* If GATT bearer is enabled move to advertisement state 
                 * otherwise move to idle state. The advertisement would be 
                 * triggerred once the GATT bearer is enabled again.
                 */
                if(g_tsapp_data.bearer_data.bearerEnabled & 
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
    switch(g_tsapp_data.state)
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
    g_tsapp_data.gatt_data.conn_interval = p_event_data->data.conn_interval;
    g_tsapp_data.gatt_data.conn_latency  = p_event_data->data.conn_latency;
    g_tsapp_data.gatt_data.conn_timeout  = 
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
    switch(g_tsapp_data.state)
    {
        case app_state_advertising:
        {
            if(p_event_data->result == sys_status_success)
            {
                /* Store received UCID */
                g_tsapp_data.gatt_data.st_ucid = p_event_data->cid;

                /* Store connected BD Address */
                g_tsapp_data.gatt_data.con_bd_addr = p_event_data->bd_addr;

                /* Store the bearer relay active and promiscuous onto global 
                 * as they need to be reverted after disconnection.
                 */
                bearer_relay_active = 
                    g_tsapp_data.bearer_data.bearerRelayActive;

                bearer_promiscuous = 
                    g_tsapp_data.bearer_data.bearerPromiscuous;

                g_tsapp_data.bearer_data.bearerRelayActive = 
                    BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK;

                g_tsapp_data.bearer_data.bearerPromiscuous = 
                    BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK;

                /* When device is connected as bridge enable the BLE and GATT 
                 * bearer relays otherwise mesh messages sent by control device 
                 * over GATT will not be forwarded on mesh.
                 */
                CsrMeshRelayEnable(
                                g_tsapp_data.bearer_data.bearerRelayActive);

                /* Enable the promiscuous mode on both the bearers which makes
                 * sure the connected control device can control any mesh n/w.
                 */
                CsrMeshEnablePromiscuousMode(
                                g_tsapp_data.bearer_data.bearerPromiscuous);

                /* Enter connected state */
                AppSetState(app_state_connected);

                /* Inform CSRmesh that we are connected now */
                CsrMeshHandleDataInConnection(
                                g_tsapp_data.gatt_data.st_ucid,
                                g_tsapp_data.gatt_data.conn_interval);


                /* Since CSRmesh Temperature Sensor app does not mandate
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
                                    &g_tsapp_data.gatt_data.con_bd_addr);
                 */

                /* If the current connection parameters being used don't
                 * comply with the application's preferred connection
                 * parameters and the timer is not running and, start timer
                 * to trigger Connection Parameter Update procedure
                 */
                if((g_tsapp_data.gatt_data.con_param_update_tid ==
                                                        TIMER_INVALID) &&
                   (g_tsapp_data.gatt_data.conn_interval <
                                             PREFERRED_MIN_CON_INTERVAL ||
                    g_tsapp_data.gatt_data.conn_interval >
                                             PREFERRED_MAX_CON_INTERVAL
                    #if PREFERRED_SLAVE_LATENCY
                                        || g_tsapp_data.gatt_data.conn_latency <
                                                                 PREFERRED_SLAVE_LATENCY
                    #endif
                   )
                  )
                {
                    /* Set the number of conn update attempts to zero */
                    g_tsapp_data.gatt_data.num_conn_update_req = 0;

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
                    g_tsapp_data.gatt_data.con_param_update_tid =
                                      TimerCreate(TGAP_CPP_PERIOD, TRUE,
                                                  handleGapCppTimerExpiry);
                    g_tsapp_data.gatt_data.cpu_timer_value =
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
                if(g_tsapp_data.bearer_data.bearerEnabled & 
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
    switch(g_tsapp_data.state)
    {
        case app_state_connected:
        {
            if(p_event_data->status == sys_status_success)
            {
                /* Store temporary pairing info. */
                g_tsapp_data.gatt_data.paired = TRUE;
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
    switch(g_tsapp_data.state)
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
                (g_tsapp_data.gatt_data.num_conn_update_req <
                                        MAX_NUM_CONN_PARAM_UPDATE_REQS))
            {
                /* Delete timer if running */
                TimerDelete(g_tsapp_data.gatt_data.con_param_update_tid);

                g_tsapp_data.gatt_data.con_param_update_tid =
                                 TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                             TRUE, requestConnParamUpdate);
                g_tsapp_data.gatt_data.cpu_timer_value =
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
    switch(g_tsapp_data.state)
    {
        case app_state_connected:
        case app_state_disconnecting:
        {
            /* Store the new connection parameters. */
            g_tsapp_data.gatt_data.conn_interval =
                                            p_event_data->data.conn_interval;
            g_tsapp_data.gatt_data.conn_latency =
                                            p_event_data->data.conn_latency;
            g_tsapp_data.gatt_data.conn_timeout =
                                        p_event_data->data.supervision_timeout;

            CsrMeshHandleDataInConnection(g_tsapp_data.gatt_data.st_ucid,
                                       g_tsapp_data.gatt_data.conn_interval);

            DEBUG_STR("Parameter Update Complete: ");
            DEBUG_U16(g_tsapp_data.gatt_data.conn_interval);
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
    switch(g_tsapp_data.state)
    {
        case app_state_connected:
        {
            /* Delete timer if running */
            TimerDelete(g_tsapp_data.gatt_data.con_param_update_tid);
            g_tsapp_data.gatt_data.con_param_update_tid = TIMER_INVALID;
            g_tsapp_data.gatt_data.cpu_timer_value = 0;

            /* The application had already received the new connection
             * parameters while handling event LM_EV_CONNECTION_UPDATE.
             * Check if new parameters comply with application preferred
             * parameters. If not, application shall trigger Connection
             * parameter update procedure
             */

            if(g_tsapp_data.gatt_data.conn_interval <
                                                PREFERRED_MIN_CON_INTERVAL ||
               g_tsapp_data.gatt_data.conn_interval >
                                                PREFERRED_MAX_CON_INTERVAL
                #if PREFERRED_SLAVE_LATENCY
                               || g_tsapp_data.gatt_data.conn_latency <
                                                                PREFERRED_SLAVE_LATENCY
                #endif
              )
            {
                /* Set the num of conn update attempts to zero */
                g_tsapp_data.gatt_data.num_conn_update_req = 0;

                /* Start timer to trigger Connection Parameter Update
                 * procedure
                 */
                g_tsapp_data.gatt_data.con_param_update_tid =
                                TimerCreate(GAP_CONN_PARAM_TIMEOUT,
                                            TRUE, requestConnParamUpdate);
                g_tsapp_data.gatt_data.cpu_timer_value =
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
    switch(g_tsapp_data.state)
    {
        case app_state_connected:
        {
            /* GATT_ACCESS_IND indicates that the central device is still disco-
             * -vering services. So, restart the connection parameter update
             * timer
             */
             if(g_tsapp_data.gatt_data.cpu_timer_value == TGAP_CPC_PERIOD &&
                g_tsapp_data.gatt_data.con_param_update_tid != TIMER_INVALID)
             {
                TimerDelete(g_tsapp_data.gatt_data.con_param_update_tid);
                g_tsapp_data.gatt_data.con_param_update_tid =
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
    g_tsapp_data.gatt_data.conn_interval = 0;
    g_tsapp_data.gatt_data.conn_latency = 0;
    g_tsapp_data.gatt_data.conn_timeout = 0;

    CsrMeshHandleDataInConnection(GATT_INVALID_UCID, 0);

    /* Restore the relay and the promiscuous settings to the last set values */
    g_tsapp_data.bearer_data.bearerRelayActive = bearer_relay_active;
    g_tsapp_data.bearer_data.bearerPromiscuous = bearer_promiscuous;

    CsrMeshRelayEnable(g_tsapp_data.bearer_data.bearerRelayActive);
    CsrMeshEnablePromiscuousMode(g_tsapp_data.bearer_data.bearerPromiscuous);

    #ifdef ENABLE_GATT_OTA_SERVICE
        if(OtaResetRequired())
        {
            OtaReset();
        }
    #endif /* ENABLE_GATT_OTA_SERVICE */


    /*Handling signal as per current state */
    switch(g_tsapp_data.state)
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
            if(g_tsapp_data.bearer_data.bearerEnabled & 
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
    /*test code 20160418*/    
    #ifndef PRESET_DEVID
        CSR_MESH_MODEL_TYPE_T model = msg[0];
        uint8 index = msg[1];
        uint16 group_id = msg[3] + (msg[4] << 8);
        bool update_lastetag = TRUE;
    
        if(model == CSR_MESH_SENSOR_MODEL || model == CSR_MESH_ACTUATOR_MODEL || 
           model == CSR_MESH_ALL_MODELS)
        {
            if(index < NUM_SENSOR_ACTUATOR_MODEL_GROUPS)
            {
                CSR_MESH_ADVSCAN_PARAM_T param;
                bool old_config_status = IsSensorConfigured();
    
                /* Store Group ID */
                sensor_actuator_model_groups[index] = group_id;
    
                /* Save to NVM */
                Nvm_Write(&sensor_actuator_model_groups[index], 
                          sizeof(uint16),
                         (NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS + index));
    
                /* If sensor was previously not grouped and has been grouped now, 
                 * then the sensor should move into low duty cycle 
                 */
                if(!old_config_status && IsSensorConfigured())
                {
                    #ifdef ENABLE_TEMPERATURE_CONTROLLER
                                    /* Print a message for temperature control. */
                                    DEBUG_STR("\r\nPress '+'/'-' Increase/Decrease Temp.\r\n");
                    #endif /* ENABLE_TEMPERATURE_CONTROLLER */
                    CsrMeshGetAdvScanParam(&param);
                    param.scan_duty_cycle = DEFAULT_RX_DUTY_CYCLE;
                    CsrMeshSetAdvScanParam(&param);
                    DEBUG_STR("Moving to Low Power Sniff Mode \r\n\r\n");
    
                    if(sensor_state[DESIRED_AIR_TEMP_IDX].repeat_interval !=0 ||
                       sensor_state[CURRENT_AIR_TEMP_IDX].repeat_interval !=0)
                    {
                        startRepeatIntervalTimer();
                    }
                }
                else if(old_config_status && !IsSensorConfigured())
                {
                    DEBUG_STR("Sensor Moving to active scan Mode \r\n\r\n");
                    CsrMeshGetAdvScanParam(&param);
                    param.scan_duty_cycle = HIGH_RX_DUTY_CYCLE;
                    CsrMeshSetAdvScanParam(&param);
    
                    /* Delete the repeat interval timer */
                    TimerDelete(repeat_interval_tid);
                    repeat_interval_tid = TIMER_INVALID;
    
                    /* Stop the periodic reading of the temp */
                    TimerDelete(tempsensor_sample_tid);
                    tempsensor_sample_tid = TIMER_INVALID;
                }
    
                /* A new group has been set. Hence start temp read and update */
                if(group_id != 0)
                {
                    /* Reset last bcast temp to trigger temp update on sensor read*/
                    last_bcast_air_temp = 0;
    
                    /* Issue a Read to start sampling timer. */
                    TempSensorRead();
    
                    /* Start the timer for next sample. */
                    tempsensor_sample_tid = TimerCreate(
                                            (uint32)TEMPERATURE_SAMPLING_INTERVAL, 
                                            TRUE,
                                            tempSensorSampleIntervalTimeoutHandler);
                    /*test code*/
                    //data_stream_tid = TimerCreate(
                    //                        5*SECOND, 
                    //                        TRUE,
                    //                        dataStreamIntervalTimeoutHandler);
                    
                    
                    
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
    #else
        bool update_lastetag=FALSE;
        return update_lastetag;
    #endif
}

/*test code 20160419*/
#ifdef PRESET_DEVID
    static void presetDevid(void)
    {
        sensor_actuator_model_groups[0] = 0x0002;
        attention_model_groups[0] = 0x0002;
        data_model_groups[0] = 0x0002;
        local_device_id=g_node_data.device_uuid.uuid[0];
        uint16 local_network_key[8]={0X1234,0X1234,0X1234,0X1234,
                                     0X1234,0X1234,0X1234,0X1234};
        CSR_MESH_ADVSCAN_PARAM_T param;
        
        /* Save group id to NVM */
        Nvm_Write(&sensor_actuator_model_groups[0], 
           sizeof(uint16),
           NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS);					 
        
        Nvm_Write(&attention_model_groups[0],
           sizeof(uint16),
           NVM_OFFSET_ATTENTION_MODEL_GROUPS);
        
        Nvm_Write(&data_model_groups[0],
           sizeof(uint16),
           NVM_OFFSET_DATA_MODEL_GROUPS);
        
        /*save device id to NVM*/			        
        Nvm_Write(&local_device_id, 1, NVM_OFFSET_DEVICE_ID);
        
        /*save network key to NVM*/
        Nvm_Write(&local_network_key[0], sizeof(CSR_MESH_NETWORK_KEY_T), 
                  NVM_OFFSET_NETWORK_KEY);
        
          /* Network key */
        g_node_data.associated = TRUE;
        Nvm_Read(g_node_data.nw_key.key, sizeof(CSR_MESH_NETWORK_KEY_T),
                                                        NVM_OFFSET_NETWORK_KEY);
        /* Device ID */
        Nvm_Read(&g_node_data.device_id, 1, NVM_OFFSET_DEVICE_ID);
        /* Sequence Number */
        Nvm_Read((uint16 *)&g_node_data.seq_number, 2,
                                                    NVM_OFFSET_SEQUENCE_NUMBER);
        /***eof g_node data*/
                  
        g_tsapp_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
        
        CsrMeshEnablePromiscuousMode(
             g_tsapp_data.bearer_data.bearerPromiscuous);										
        
        Nvm_Write((uint16 *)&g_tsapp_data.assoc_state, 1,
          NVM_OFFSET_ASSOCIATION_STATE);
          
        /* Update Bearer Model Data to NVM */
        Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
           sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);
        
        g_tsapp_data.assoc_state = app_state_associated;						
        
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

static void HandlePIOChangedEvent(pio_changed_data *pio_data)
{
    //uint8 sensor_status0;
    if(pio_data->pio_cause & (bitmask(PUSH_BUTTON)) )
    {
        //Nvm_Read((uint16 *)&sensor_status0,1,NVM_SENSOR_STATUS);
        if(PioGet(PUSH_BUTTON)==0)//falling edge, button pressed
        {   /*      
            if(currentSensorStatus==SENSOR_STATUS_INIT) //for power on long hold button
            {   TimerDelete(button_press_poweron_tid);
                //20160801
                TimerDelete(idle_tid);
                PioSetMode(SEND_IND,pio_mode_user);
                PioSet(SEND_IND,1);//on
                
                button_press_poweron_tid=
                        TimerCreate(TIMER_BUTTON_POWERON,TRUE,//5s
                                    handleLongButtonPress);
            }*/
            if(currentSensorStatus==SENSOR_STATUS_NORMAL)
            {   //TimerDelete(button_longpress_send_tid);
                //button_longpress_send_tid=
                //        TimerCreate(TIMER_BUTTON_FORCESEND,TRUE,timerHandleStartBME280ForceMode);
                BME280_Trigger();
                TimerCreate(TIMER_SENSOR_READ_INIT_INTERVAL,TRUE,timerHandleReadBME280Data);                
            }
        }
        else //rising edge, button released
        {
            /* This event comes when a button is released. */            
            /*if(button_press_poweron_tid != TIMER_INVALID) //button released within 5s during power on
            {   
                TimerDelete(button_press_poweron_tid);                               
                button_press_poweron_tid  = TIMER_INVALID;
            }*/
            if(button_longpress_send_tid != TIMER_INVALID)
            {   
                TimerDelete(button_longpress_send_tid); 
                button_longpress_send_tid = TIMER_INVALID;
            }
        }
    }
}

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
        Nvm_Write((uint16 *)&g_tsapp_data.assoc_state,
                  sizeof(g_tsapp_data.assoc_state),
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
        if(IsSensorConfigured())
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
    app_state old_state = g_tsapp_data.state;
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

                /* Initialise CSRmesh Temperature Sensor data and services
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
        g_tsapp_data.state = new_state;

        /* Handle entering new state */
        switch(new_state)
        {
            case app_state_advertising:
            {
                GattTriggerFastAdverts();
            }
            break;

            case app_state_connected:
            {
                DEBUG_STR("Connected\r\n");
            }
            break;

            case app_state_disconnecting:
                GattDisconnectReq(g_tsapp_data.gatt_data.st_ucid);
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
    
 //if(last_sleep_state!=sleep_state_warm_powerup)
 //{
    /*test code*/
    datasentflag=0;
    /*20160603*/    
    static uint16 testnvm;
    testnvm = NVM_MAX_APP_MEMORY_WORDS;
    
    uint16 gatt_db_length = 0;
    uint16 *p_gatt_db_pointer = NULL;
    CSR_MESH_ADVSCAN_PARAM_T param;
    
    #ifdef USE_STATIC_RANDOM_ADDRESS
        /* Generate static random address for the CSRmesh Device. */
        generateStaticRandomAddress(&g_tsapp_data.random_bd_addr);
    
        /* Set the Static Random Address of the device. */
        GapSetRandomAddress(&g_tsapp_data.random_bd_addr);
    #endif /* USE_STATIC_RANDOM_ADDRESS */

    /* Initialise the application timers */
    TimerInit(MAX_APP_TIMERS, (void*)app_timers);  
                       
    #ifdef DEBUG_ENABLE
        #ifdef ENABLE_TEMPERATURE_CONTROLLER
            /* Initialise UART and configure with
             * default baud rate and port configuration.
             */
            DebugInit(1, UartDataRxCallback, NULL);
        
            /* UART Rx threshold is set to 1,
             * so that every byte received will trigger the Rx callback.
             */
            UartRead(1, 0);
            

        #else /* ENABLE_TEMPERATURE_CONTROLLER */
            DebugInit(0, NULL, NULL);
        #endif /* ENABLE_TEMPERATURE_CONTROLLER */
    #endif /* DEBUG_ENABLE */

    //#if defined(ENABLE_TEMPERATURE_CONTROLLER) && !defined(DEBUG_ENABLE)
        /* Initialise the Switch Hardware */
        IOTSwitchInit();
    //#endif /* defined(ENABLE_TEMPERATURE_CONTROLLER) && !defined(DEBUG_ENABLE) */

    /* Initialise Light and turn it off. */
    IOTLightControlDeviceInit();
    IOTLightControlDevicePower(FALSE);
    DEBUG_STR("\r\nTemperature Sensor Application\r\n");
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
    
    /*testcode 20160421*/
    #ifdef ENABLE_BMP180
        BMP180_Init();
    #endif

    /*testcode 20160422*/
    #ifdef ENABLE_SI7034
        Si7034_Init();
    #endif
    
    /*20160704*/
    #ifdef ENABLE_BME280
        BME280_Init();
    #endif
    /* Initialise the GATT and GAP data.
     * Needs to be done before readPersistentStore
     */
    appDataInit();

    /* Initialise Application specific Sensor Model Data.
     * This needs to be done before readPersistentStore.
     */
    sensor_state[CURRENT_AIR_TEMP_IDX].type=sensor_type_internal_air_temperature;
    sensor_state[CURRENT_AIR_TEMP_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    sensor_state[CURRENT_AIR_TEMP_IDX].value = (uint16 *)&current_air_temp;
    sensor_state[CURRENT_AIR_TEMP_IDX].repeat_interval = 
                                        DEFAULT_REPEAT_INTERVAL & 0xFF;

    sensor_state[CURRENT_BATTERY_IDX].type=sensor_type_solar_energy;/*battery leval*/
    sensor_state[CURRENT_BATTERY_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    sensor_state[CURRENT_BATTERY_IDX].value = (uint16 *)&current_battery_level;
    sensor_state[CURRENT_BATTERY_IDX].repeat_interval = 
                                        DEFAULT_REPEAT_INTERVAL & 0xFF;
    
    sensor_state[CURRENT_HUMI_LOW_IDX].type = sensor_type_external_humidity;
    sensor_state[CURRENT_HUMI_LOW_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    sensor_state[CURRENT_HUMI_LOW_IDX].value = (uint16 *)&current_humi_low;
    sensor_state[CURRENT_HUMI_LOW_IDX].repeat_interval = 
                                        DEFAULT_REPEAT_INTERVAL & 0xFF;
    
    sensor_state[CURRENT_HUMI_HIGH_IDX].type = sensor_type_internal_humidity;
    sensor_state[CURRENT_HUMI_HIGH_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    //sensor_state[CURRENT_HUMI_HIGH_IDX].value = (uint16 *)&current_humi_high;//20160809
    sensor_state[CURRENT_HUMI_HIGH_IDX].value = (uint16 *)&current_air_humi;
    sensor_state[CURRENT_HUMI_HIGH_IDX].repeat_interval = 
                                        DEFAULT_REPEAT_INTERVAL & 0xFF;
    
    /*test code-20160422*/                                       
    sensor_state[CURRENT_PRESSURE_LOW_IDX].type= 
                                        sensor_type_desired_air_temperature;
    sensor_state[CURRENT_PRESSURE_LOW_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    sensor_state[CURRENT_PRESSURE_LOW_IDX].value = 
                                        (uint16 *)&current_pressure_low;
    sensor_state[CURRENT_PRESSURE_LOW_IDX].repeat_interval = 
                                                DEFAULT_REPEAT_INTERVAL & 0xFF; 
    
    sensor_state[CURRENT_PRESSURE_HIGH_IDX].type        = 
                                        sensor_type_barometric_pressure;      
    sensor_state[CURRENT_PRESSURE_HIGH_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));   
    //sensor_state[CURRENT_PRESSURE_HIGH_IDX].value       = 
    //                                    (uint16 *)&current_pressure_high;
    sensor_state[CURRENT_PRESSURE_HIGH_IDX].value       = 
                                        (uint16 *)&current_air_pressure;
    sensor_state[CURRENT_PRESSURE_HIGH_IDX].repeat_interval = 
                                        DEFAULT_REPEAT_INTERVAL & 0xFF;
        
    g_tsapp_data.sensor_data.num_types   = NUM_SENSORS_SUPPORTED;
    g_tsapp_data.sensor_data.sensor_list = sensor_state;

    /* Initialise the actuator model specific data */
    actuator_state[CURRENT_AIR_TEMP_IDX].type        = 
                                        sensor_type_internal_air_temperature;
    actuator_state[CURRENT_AIR_TEMP_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    actuator_state[CURRENT_AIR_TEMP_IDX].value = (uint16 *)&current_air_temp;   
    
    actuator_state[CURRENT_BATTERY_IDX].type        = 
                                        sensor_type_solar_energy;
    actuator_state[CURRENT_BATTERY_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    actuator_state[CURRENT_BATTERY_IDX].value = (uint16 *)&current_battery_level;
    
       
    actuator_state[CURRENT_HUMI_LOW_IDX].type = sensor_type_external_humidity;
    actuator_state[CURRENT_HUMI_LOW_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    actuator_state[CURRENT_HUMI_LOW_IDX].value = (uint16 *)&current_humi_low;  
    
    actuator_state[CURRENT_HUMI_HIGH_IDX].type = sensor_type_internal_humidity;
    actuator_state[CURRENT_HUMI_HIGH_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    //actuator_state[CURRENT_HUMI_HIGH_IDX].value = (uint16 *)&current_humi_high;
    actuator_state[CURRENT_HUMI_HIGH_IDX].value = (uint16 *)&current_air_humi; 
    
    /*test code-20160422*/    
    actuator_state[CURRENT_PRESSURE_LOW_IDX].type        = 
                                        sensor_type_desired_air_temperature;
    actuator_state[CURRENT_PRESSURE_LOW_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));
    actuator_state[CURRENT_PRESSURE_LOW_IDX].value       = 
                                        (uint16 *)&current_pressure_low;
    
    actuator_state[CURRENT_PRESSURE_HIGH_IDX].type        = 
                                        sensor_type_barometric_pressure;
    actuator_state[CURRENT_PRESSURE_HIGH_IDX].value_size  = 
                                        (2*sizeof(SENSOR_FORMAT_TEMPERATURE_T));      
    //actuator_state[CURRENT_PRESSURE_HIGH_IDX].value       = 
    //                                    (uint16 *)&current_pressure_high;    
    actuator_state[CURRENT_PRESSURE_HIGH_IDX].value       = 
                                        (uint16 *)&current_air_pressure;  
    
    g_tsapp_data.actuator_data.num_types   = NUM_SENSORS_SUPPORTED;
    g_tsapp_data.actuator_data.sensor_list = actuator_state;

    tempsensor_sample_tid = TIMER_INVALID;
    retransmit_tid  = TIMER_INVALID;
    repeat_interval_tid = TIMER_INVALID;
    
    /*test code*/
    data_stream_tid = TIMER_INVALID;
    
     /*test code*/    
    //data_stream_tid = TimerCreate(3*SECOND,TRUE,
    //                              dataStreamIntervalTimeoutHandler);
    

    /* Read persistent storage.
     * Call this before CsrMeshInit.
     */      
    if(0)readPersistentStore();
     readPersistentStore1();
    if(0) readPersistentStore2(); 
       
#if 0
    uint16 eeprom_erase = 0xeeee;
    /* Erase a block in EEPROM to remove all sanity words */
    uint16 i;
    for (i = 0; i < 128; i++)
    {
        Nvm_Write(&eeprom_erase, 0x1, i);
    }
#endif
    /*test code 20160419*/    
    presetDevid();   

    DEBUG_U16(g_node_data.device_id);DEBUG_STR(" ←Device ID\r\n");
    /* Initialise the CSRmesh */
    CsrMeshInit(&g_node_data);

    /* Update relay status */
    CsrMeshRelayEnable(g_tsapp_data.bearer_data.bearerRelayActive);

    /* Update promiscuous status */
    CsrMeshEnablePromiscuousMode(g_tsapp_data.bearer_data.bearerPromiscuous);

    /* Enable notifications */
    CsrMeshEnableRawMsgEvent(TRUE);

    /* Initialise Sensor Model. */
    SensorModelInit(sensor_actuator_model_groups, 
                                            NUM_SENSOR_ACTUATOR_MODEL_GROUPS);

    /* Initialise Actuator Model */
    ActuatorModelInit(sensor_actuator_model_groups, 
                                            NUM_SENSOR_ACTUATOR_MODEL_GROUPS);

    /* Initialise Attention Model */
    AttentionModelInit(attention_model_groups, NUM_ATTENTION_MODEL_GROUPS);

    #ifdef ENABLE_FIRMWARE_MODEL
        /* Initialise Firmware Model */
        FirmwareModelInit();
    
        /* Set Firmware Version */
        g_tsapp_data.fw_version.major_version = APP_MAJOR_VERSION;
        g_tsapp_data.fw_version.minor_version = APP_MINOR_VERSION;
    #endif /* ENABLE_FIRMWARE_MODEL */

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

    /* Initialise CSRmesh Temperature Sensor application State */
    g_tsapp_data.state = app_state_init;

    /* Get the stored adv scan parameters */
    CsrMeshGetAdvScanParam(&param);

    /* Read the mesh advertising parameter setting from the CS User Keys */
    param.advertising_interval =
                                CSReadUserKey(CSKEY_INDEX_CSRMESH_ADV_INTERVAL);
    param.advertising_time = CSReadUserKey(CSKEY_INDEX_CSRMESH_ADV_TIME);   
    
    if (g_tsapp_data.assoc_state != app_state_associated)
    {
        initiateAssociation();
    }
    else
    {
       DEBUG_STR("Temperature Sensor is associated\r\n");       
    }

    /* Set the Advertising and Scan parameters */
    CsrMeshSetAdvScanParam(&param);
    
    #ifdef ENABLE_ACK_MODE 
        resetHeaterList();
    #endif /* ENABLE_ACK_MODE */

    /* Tell GATT about our database. We will get a GATT_ADD_DB_CFM event when
     * this has completed.
     */
    p_gatt_db_pointer = GattGetDatabase(&gatt_db_length);
    
    GattAddDatabaseReq(gatt_db_length, p_gatt_db_pointer);

    if(PioGet(PUSH_BUTTON)==PRESSED) //if user press button during power on, unreg the device
    {   
        button_press_poweron_tid=
            //TimerCreate(TIMER_RESET_BUTTON_HOLD,TRUE,handleUnreg);
            TimerCreate(TIMER_RESET_BUTTON_HOLD,TRUE,handleLongButtonPress);    
        currentSensorStatus=SENSOR_STATUS_INIT_PRESSED;
        Nvm_Write(&currentSensorStatus,1,NVM_SENSOR_STATUS);
    }
     
    if(currentSensorStatus==SENSOR_STATUS_NORMAL) //if sensor state is normal, start regular send environemnt data
    {   
        /*20060803*/
        PioSetMode(SEND_IND,pio_mode_pwm1);
        PioConfigPWM(1,pio_pwm_mode_push_pull, 1, 200, 1,
                                               200, 1, 1, 5);//repower-on/reset
        PioEnablePWM(1,FALSE);
        
        tx_id=TimerCreate(TIMER_SENSOR_READ_INIT_INTERVAL,TRUE,timerHandleStartBME280ForceMode);        
    }
   
    if(currentSensorStatus==SENSOR_STATUS_REGISTER) //if sensor state is unreg, start timer for receving reg cmd
    {
         /*register_tid=
             TimerCreate(TIMER_STATUS_PEND_REG,TRUE,handleRegisterTimerExpire);*/
    }
    if(currentSensorStatus==SENSOR_STATUS_INIT)
    {            
        PioSetMode(SEND_IND,pio_mode_user);
        PioSet(SEND_IND,0);
        ledFlashtid=TimerCreate(1*SECOND,TRUE,handleLEDFlash1S);
        //SleepModeChange(sleep_mode_never);
        //idle_tid=TimerCreate(TIMER_IDLESTATE,TRUE,slowBlinkTimerHandler);
    }
    
    /******/
    /**/
    CsrMeshGetAdvScanParam(&param);
    param.scan_duty_cycle = CUSTOM_RX_DUTY_CYCLE;
    CsrMeshSetAdvScanParam(&param);
    
    g_tsapp_data.bearer_data.bearerRelayActive|= BLE_BEARER_MASK;
    g_tsapp_data.bearer_data.bearerEnabled|=BLE_BEARER_MASK;//0x0001
    g_tsapp_data.bearer_data.bearerPromiscuous=BLE_BEARER_MASK;//0x0001

    bearer_relay_active = g_tsapp_data.bearer_data.bearerRelayActive;
    bearer_promiscuous = g_tsapp_data.bearer_data.bearerPromiscuous;
    
    CsrMeshRelayEnable(g_tsapp_data.bearer_data.bearerEnabled);
    CsrMeshEnablePromiscuousMode(g_tsapp_data.bearer_data.bearerPromiscuous);

    Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
              sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);

    /*****/
    //SleepModeChange(sleep_mode_deep);//0718

    sleepState=last_sleep_state;  
    Nvm_Write(&sleepState,1,NVM_SENSOR_SLEEP_STATE);  //0xf85c
}    

/*20160801*/
void handleLEDFlash1S(timer_id tid)
{
    TimerDelete(ledFlashtid);
    ledFlashtid=TIMER_INVALID;
    if(PioGet(SEND_IND)==1) PioSet(SEND_IND,0);
    else PioSet(SEND_IND,1);
    ledFlashtid=TimerCreate(1*SECOND,TRUE,handleLEDFlash1S);
}
void slowBlinkTimerHandler(timer_id tid)
{
    TimerDelete(ledFlashtid);
    ledFlashtid=TIMER_INVALID;
    TimerDelete(tid);
    tid=TIMER_INVALID;

    PioSetMode(SEND_IND,pio_mode_user);
    PioSet(SEND_IND,0);//OFF
    SleepModeChange(sleep_mode_deep);
    //idle_tid=TimerCreate(3*SECOND,TRUE,slowBlinkTimerHandler);
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
        //#if defined(ENABLE_TEMPERATURE_CONTROLLER) && !defined(DEBUG_ENABLE)
        //        handlePIOChangedEvent(((pio_changed_data*)data)->pio_cause);
        //#endif /* defined(ENABLE_TEMPERATURE_CONTROLLER) && !defined(DEBUG_ENABLE) */
        HandlePIOChangedEvent((pio_changed_data*)data);  
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
        DEBUG_STR("\r\n CSR_MESH_ASSOCIATION_REQUEST: \r\n");        
        {
            DEBUG_STR("Association Started\r\n");
            if (g_tsapp_data.assoc_state != app_state_association_started)
            {
                g_tsapp_data.assoc_state = app_state_association_started;
            }
            TimerDelete(g_tsapp_data.mesh_device_id_advert_tid);
            g_tsapp_data.mesh_device_id_advert_tid = TIMER_INVALID;

            /* Blink Light in Yellow to indicate association started */
            IOTLightControlDeviceBlink(127, 127, 0, 32, 32);
        }
        break;

        case CSR_MESH_KEY_DISTRIBUTION:
        DEBUG_STR("\r\n CSR_MESH_KEY_DISTRIBUTION: \r\n");
        {
            DEBUG_STR("Association complete\r\n");
            g_tsapp_data.assoc_state = app_state_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_tsapp_data.assoc_state, 1,
                     NVM_OFFSET_ASSOCIATION_STATE);

            /* Save the network key on NVM */
            Nvm_Write((uint16 *)data, sizeof(CSR_MESH_NETWORK_KEY_T), 
                                                        NVM_OFFSET_NETWORK_KEY);

            /* The association is complete set LE bearer to non-promiscuous.*/
            g_tsapp_data.bearer_data.bearerPromiscuous &= ~BLE_BEARER_MASK;
            CsrMeshEnablePromiscuousMode(
                                    g_tsapp_data.bearer_data.bearerPromiscuous);

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
                      sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);

            /* When MESH_BRIDGE_SERVICE is not supported, Temperature Sensor 
             * needs to be associated with CSRmesh network, before it can send 
             * commands. Stop the blue led blinking visual indication, as 
             * Temperature Sensor is now associated to network.
             */
            IOTLightControlDevicePower(FALSE);
        }
        break;

        case CSR_MESH_ASSOCIATION_ATTENTION:
        DEBUG_STR("\r\n CSR_MESH_ASSOCIATION_ATTENTION: \r\n");
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
                if(g_tsapp_data.assoc_state == app_state_not_associated)
                {
                    /* Blink blue to indicate not associated status */
                    IOTLightControlDeviceBlink(0, 0, 127, 32, 32);
                }
                else
                {
                    /* Restore the light Power State */
                    IOTLightControlDevicePower(FALSE);
                }
            }
        }
        break;

        case CSR_MESH_UPDATE_MSG_SEQ_NUMBER:
        DEBUG_STR("\r\n CSR_MESH_UPDATE_MSG_SEQ_NUMBER: \r\n");
        {
            /* Sequence number has updated, store it in NVM */
            Nvm_Write((uint16 *)data, 2, NVM_OFFSET_SEQUENCE_NUMBER);
        }
        break;

        case CSR_MESH_CONFIG_RESET_DEVICE:
        DEBUG_STR("\r\n CSR_MESH_CONFIG_RESET_DEVICE: \r\n");
        {
            uint16 i;
            DEBUG_STR("Reset Device\r\n");

            /* Move device to dissociated state */
            g_tsapp_data.assoc_state = app_state_not_associated;

            /* Write association state to NVM */
            Nvm_Write((uint16 *)&g_tsapp_data.assoc_state,
                     sizeof(g_tsapp_data.assoc_state),
                     NVM_OFFSET_ASSOCIATION_STATE);

            /* Delete Temperature Sensor Timers. */
            TimerDelete(retransmit_tid);
            retransmit_tid = TIMER_INVALID;

            TimerDelete(tempsensor_sample_tid);
            tempsensor_sample_tid = TIMER_INVALID;


            /* Reset the supported model groups and save it to NVM */
            /* Sensor model */
            MemSet(sensor_actuator_model_groups, 0x0000,
                                        sizeof(sensor_actuator_model_groups));
            Nvm_Write(sensor_actuator_model_groups, 
                sizeof(sensor_actuator_model_groups),
                    NVM_OFFSET_SENSOR_ACTUATOR_MODEL_GROUPS);

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

            /* Reset Sensor State. */
            for (i = 0; i < NUM_SENSORS_SUPPORTED; i++)
            {
                writeSensorDataToNVM(i);
            }

            

            /* Enable promiscuous mode on un-associated devices so that they 
             * relay all the messages. This helps propagate messages(MCP) based 
             * on the newly assigned network key as they will be relayed also by
             * the devices not yet associated.
             */
            g_tsapp_data.bearer_data.bearerPromiscuous = 
                               (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);
            CsrMeshEnablePromiscuousMode(
                                g_tsapp_data.bearer_data.bearerPromiscuous);

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
                      sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);

            /* Start Mesh association again */
            initiateAssociation();
        }
        break;

        case CSR_MESH_CONFIG_GET_VID_PID_VERSION:
        DEBUG_STR("\r\n CSR_MESH_CONFIG_GET_VID_PID_VERSION: \r\n");
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&vid_pid_info;
            }
        }
        break;

        case CSR_MESH_CONFIG_GET_APPEARANCE:
        DEBUG_STR("\r\n CSR_MESH_CONFIG_GET_APPEARANCE: \r\n");
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&device_appearance;
            }
        }
        break;

        case CSR_MESH_GROUP_SET_MODEL_GROUPID:
        DEBUG_STR("\r\n CSR_MESH_GROUP_SET_MODEL_GROUPID: \r\n");
        {
            /* Save Group Information here */
            update_lastetag = handleCsrMeshGroupSetMsg(data, length);
        }
        break;

        case CSR_MESH_CONFIG_DEVICE_IDENTIFIER:
        DEBUG_STR("\r\n CSR_MESH_CONFIG_DEVICE_IDENTIFIER: \r\n");
        {
            /*test code 20160418*/
            #ifndef PRESET_DEVID
            Nvm_Write((uint16 *)data, 1, NVM_OFFSET_DEVICE_ID);
            DEBUG_STR("Device ID received:");
            DEBUG_U16(((uint16)data[0]));
            DEBUG_STR("\r\n");
            #endif
        }
        break;

#ifdef ENABLE_BATTERY_MODEL
        case CSR_MESH_BATTERY_GET_STATE:
        DEBUG_STR("\r\n CSR_MESH_BATTERY_GET_STATE: \r\n");
        {
            /* Initialise battery state. IOT  boards (H13323) are battery powered */
            g_tsapp_data.battery_data.battery_state = 
                                            BATTERY_MODEL_STATE_POWERING_DEVICE;
            /* Read Battery Level */
            g_tsapp_data.battery_data.battery_level = ReadBatteryLevel();

            if(g_tsapp_data.battery_data.battery_level == 0)
            {
                /* Voltage is below flat battery voltage. Set the needs 
                 * replacement flag in the battery state
                 */
                g_tsapp_data.battery_data.battery_state |=
                                          BATTERY_MODEL_STATE_NEEDS_REPLACEMENT;
            }

            /* Pass Battery state data to model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_tsapp_data.battery_data;
            }
        }
        break;
#endif /* ENABLE_BATTERY_MODEL */

        case CSR_MESH_BEARER_SET_STATE:
        DEBUG_STR("\r\n CSR_MESH_BEARER_SET_STATE: \r\n");
        {
            uint8 *pData = data;
            g_tsapp_data.bearer_data.bearerRelayActive = BufReadUint16(&pData);
            g_tsapp_data.bearer_data.bearerEnabled     = BufReadUint16(&pData);
            g_tsapp_data.bearer_data.bearerPromiscuous = BufReadUint16(&pData);

            /* BLE Advert Bearer is always enabled on this device. */
            g_tsapp_data.bearer_data.bearerEnabled    |= BLE_BEARER_MASK;

            /* Filter the supported bearers from the bitmap received */
            g_tsapp_data.bearer_data.bearerRelayActive = 
                g_tsapp_data.bearer_data.bearerRelayActive & 
                    (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);

            /* Filter the supported bearers from the bitmap received */
            g_tsapp_data.bearer_data.bearerEnabled = 
                g_tsapp_data.bearer_data.bearerEnabled & 
                    (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);

            /* Filter the supported bearers from the bitmap received */
            g_tsapp_data.bearer_data.bearerPromiscuous = 
                g_tsapp_data.bearer_data.bearerPromiscuous & 
                    (BLE_BEARER_MASK | BLE_GATT_SERVER_BEARER_MASK);

            /* Update the saved values */
            bearer_relay_active = g_tsapp_data.bearer_data.bearerRelayActive;
            bearer_promiscuous = g_tsapp_data.bearer_data.bearerPromiscuous;

            /* Update new bearer state */
            CsrMeshRelayEnable(g_tsapp_data.bearer_data.bearerRelayActive);
            CsrMeshEnablePromiscuousMode(
                                    g_tsapp_data.bearer_data.bearerPromiscuous);

            /* Update Bearer Model Data to NVM */
            Nvm_Write((uint16 *)&g_tsapp_data.bearer_data,
                      sizeof(BEARER_MODEL_STATE_DATA_T),NVM_BEARER_DATA_OFFSET);

            if(g_tsapp_data.state != app_state_connected) 
            {
                if(g_tsapp_data.bearer_data.bearerEnabled 
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
        DEBUG_STR("\r\n CSR_MESH_BEARER_GET_STATE: \r\n");
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&g_tsapp_data.bearer_data;
            }
        }
        break;

#ifdef ENABLE_FIRMWARE_MODEL
        case CSR_MESH_FIRMWARE_GET_VERSION_INFO:
        DEBUG_STR("\r\n CSR_MESH_FIRMWARE_GET_VERSION_INFO: \r\n");
        {
            /* Send Firmware Version data to the model */
            if (state_data != NULL)
            {
                *state_data = (void *)&g_tsapp_data.fw_version;
            }
        }
        break;

        case CSR_MESH_FIRMWARE_UPDATE_REQUIRED:
        DEBUG_STR("\r\n CSR_MESH_FIRMWARE_UPDATE_REQUIRED: \r\n");
        {
            BD_ADDR_T *pBDAddr = NULL;
#ifdef USE_STATIC_RANDOM_ADDRESS
            pBDAddr = &g_tsapp_data.random_bd_addr;
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

        case CSR_MESH_ATTENTION_SET_STATE:
        DEBUG_STR("\r\n CSR_MESH_ATTENTION_SET_STATE: \r\n");
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
                    attn_tid = TimerCreate(
                        (uint32)attn_data.attn_duration * MILLISECOND,
                            TRUE, attnTimerHandler);
                }
                /* Enable Red light blinking to attract attention */
                IOTLightControlDeviceBlink(127, 0, 0, 32, 32);

                /* Change the Rx scan duty cycle on enabling attention */
                EnableHighDutyScanMode(TRUE);
            }
            else
            {
                /* Restore Light State */
                IOTLightControlDeviceSetColor(0,0,0);

                /* Restore the light Power State */
                IOTLightControlDevicePower(FALSE);

                /* Set back the scan to low duty cycle only if the device has
                 * already been grouped.
                 */
                EnableHighDutyScanMode(FALSE);
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
            DEBUG_U16(attn_data.attn_duration);
            DEBUG_STR("\r\n");

        }
        break;

#ifdef ENABLE_DATA_MODEL
        /* Data stream model messages */
        case CSR_MESH_DATA_STREAM_SEND_CFM:
        {   /*new code-commented 20160513*/       
            //static uint32 sid;
            //uint16 t_id;
        
            /*CSR_MESH_STREAM_EVENT_T *pwatchdata=(CSR_MESH_STREAM_EVENT_T)*data;
            uint16 *pwatchdatalen=data->data_len;
            
            uint8 watchdata0 = BufReadUint8(&pwatchdata);
            uint8 watchdata1 = BufReadUint8(&pwatchdata);
            uint8 watchdata2 = BufReadUint8(&pwatchdata);
            uint16 *pwatchdatalen = (CSR_MESH_STREAM_EVENT_T *)data_len;
            uint16 watchdatalen = BufReadUint8(&pwatchdatalen);
            
            DEBUG_STR("\r\n");
            DEBUG_STR("\r\ndata pointer is\r\n");
            DEBUG_U16(pwatchdata);
            DEBUG_STR("\r\nwatchdata\r\n");
            DEBUG_U32(watchdata);
            DEBUG_STR("\r\n");   */   
            /*new code-commented 20160513*/
            /*uint8 *pData = data+1;
            DEBUG_U16(*data);
            DEBUG_U16(*pData);
            sid = BufReadUint32(&data);
            sid = BufReadUint32(&data);
            DEBUG_U32(sid);
            DEBUG_U32(*pData);
            t_id = BufReadUint16(&pData);
            DEBUG_U16(t_id);*/ 
            //DEBUG_STR("case CSR_MESH_DATA_STREAM_SEND_CFM\r\n");
            handleCSRmeshDataStreamSendCfm((CSR_MESH_STREAM_EVENT_T *)data);                       
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

        /* Actuator Messages handling */
        case CSR_MESH_ACTUATOR_GET_TYPES:
        {
            if (state_data != NULL)
            {
                *state_data = (void *)&g_tsapp_data.actuator_data;
            }
        }
        break;

        case CSR_MESH_ACTUATOR_SET_VALUE_NO_ACK:
        case CSR_MESH_ACTUATOR_SET_VALUE:
        DEBUG_STR("\r\n CSR_MESH_ACTUATOR_SET_VALUE: \r\n");
        {
            uint8 *p_data = data;
            sensor_type_t type = sensor_type_invalid;
            type = BufReadUint16(&p_data);
            *state_data = NULL;

             /*test code 20160422-commented*/  
            #if 0
            if(type == sensor_type_desired_air_temperature)
            {
                /* Length of the value of this type is 16 bits */
                current_desired_air_temp = BufReadUint16(&p_data);

                DEBUG_STR(" RECEIVED DESIRED TEMP : ");
                printInDecimal(current_desired_air_temp/32);
                DEBUG_STR(" kelvin\r\n");

                /* Duplicate messages could be filtered here */
                if(current_desired_air_temp != last_bcast_desired_air_temp)
                {
                    /* Set last Broadcast Temperature to current temperature. */
                    last_bcast_desired_air_temp = current_desired_air_temp;
                    startTempTransmission();
                }

                /* assign the state data to acknowledge the write */
                *state_data = (void *)&g_tsapp_data.actuator_data;
            }
            #endif
        }
        break;

        /* Sensor Messages handling */
        case CSR_MESH_SENSOR_READ_VALUE:
        case CSR_MESH_SENSOR_MISSING:       
        {
            DEBUG_STR("\r\n CSR_MESH_SENSOR_READ_VALUE&CSR_MESH_SENSOR_MISSING: \r\n");
        }
        
        /* Code Fall-Through */
        case CSR_MESH_SENSOR_GET_TYPES:
        case CSR_MESH_SENSOR_GET_STATE:        
        {DEBUG_STR("\r\n CSR_MESH_SENSOR_GET_TYPES&CSR_MESH_SENSOR_GET_STATE: \r\n");
            if (state_data != NULL)
            {
                *state_data = (void *)&g_tsapp_data.sensor_data;
            }
        }
        break;

        case CSR_MESH_SENSOR_SET_STATE:
        DEBUG_STR("\r\n CSR_MESH_SENSOR_SET_STATE: \r\n");
        {
            SENSOR_MODEL_EVENT_T *sensor_event = (SENSOR_MODEL_EVENT_T*)data;
            uint8 *p_data = sensor_event->msg;
            sensor_type_t type = sensor_type_invalid;
            type = BufReadUint16(&p_data);
            
            
            if(sensor_type_internal_air_temperature)
            {
                sensor_state[CURRENT_AIR_TEMP_IDX].repeat_interval = 
                                                   BufReadUint8(&p_data) & 0xFF;
                writeSensorDataToNVM(CURRENT_AIR_TEMP_IDX);
            }
            
            if (state_data != NULL)
            {
                *state_data = (void *)&g_tsapp_data.sensor_data;
            }
        }
        break;

        case CSR_MESH_SENSOR_VALUE:
        DEBUG_STR("\r\n CSR_MESH_SENSOR_VALUE: \r\n");
        {
#ifdef ENABLE_ACK_MODE
            SENSOR_MODEL_EVENT_T *sensor_event = (SENSOR_MODEL_EVENT_T*)data;
            uint8 *p_data = sensor_event->msg;
            uint8 *p_data_end = sensor_event->msg + sensor_event->msg_len;
            sensor_type_t type = sensor_type_invalid;
            
#endif /* ENABLE_ACK_MODE */
        }
        break;

        case CSR_MESH_SENSOR_WRITE_VALUE:
        case CSR_MESH_SENSOR_WRITE_VALUE_NO_ACK:
        {
        }

        
        break;

        /* Received a raw message from lower-layers.
         * Notify to the control device if connected.
         */
        case CSR_MESH_RAW_MESSAGE:
        {
            if (g_tsapp_data.state == app_state_connected)
            {
                MeshControlNotifyResponse(g_tsapp_data.gatt_data.st_ucid,
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
