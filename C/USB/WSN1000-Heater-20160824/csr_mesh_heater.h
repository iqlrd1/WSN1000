/*****************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      csr_mesh_heater.h
 *
 *  DESCRIPTION
 *      This file defines a simple implementation of CSRmesh Heater device
 *
 *****************************************************************************/

#ifndef __CSR_MESH_TEMPSENSOR_H__
#define __CSR_MESH_TEMPSENSOR_H__

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <types.h>
#include <bt_event_types.h>
#include <timer.h>
#include <byte_queue.h>

/*============================================================================*
 *  Local Header File
 *============================================================================*/
#ifdef ENABLE_BATTERY_MODEL
#include <battery_model.h>
#endif /* ENABLE_BATTERY_MODEL */

#ifdef ENABLE_FIRMWARE_MODEL
#include <firmware_model.h>
#endif

#include "csr_mesh_heater_gatt.h"
#include "uartio.h"
#include <bearer_model.h>
#include <sensor_model.h>
#include <attention_model.h>

#define TIMER_DELAY_RECEIVE_DEVID       5*SECOND  //Time interval for not sending req devid cmd
#define TIMER_INTERVAL_SENSOR_DATA_REC  100*SECOND//1*MINUTE

#define TIMER_RETRY_TIME_LENGTH     500    //in ms, Time interval for sending req devid cmd
#define TIMER_RETRY_TIME_COUNT      30     //Total duration for sending req devid cmd = TIMER_RETRY_TIME_LENGTH * TIMER_RETRY_TIME_COUNT

#define TIMER_START_MESHSWITCH      10*MILLISECOND
#define TIMER_HEATER_MESHON         3*MINUTE    //MINUTE
#define TIMER_HEATER_MESHOFF        10*SECOND
/*============================================================================*
 *  Public Definitions
 *============================================================================*/
#define TEMP_LOWEST     1 //-40
#define TEMP_HIGHEST    85
#define HUMI_LOWEST     10
#define HUMI_HIGHEST    100
#define PRESS_LOWEST    40000       //400hPa
#define PRESS_HIGHEST   200000      //2000hPa

/*20160612*/
#define NUM_OF_DEVID_IN_NVM             50
/*============================================================================*
 *  Public Data Types
 *============================================================================*/
/*20160608*/
typedef enum
{   
    ERR_CODE_SENSOR_NO_REPONSE,
    ERR_CODE_DATA_OUTOF_RANGE,        
} error_code;
/* CSRmesh device association state */
typedef enum
{

    /* Application Initial State */
    app_state_not_associated = 0,

    /* Application state association started */
    app_state_association_started,

    /* Application state associated */
    app_state_associated,

} app_association_state;


typedef enum
{
    /* Application Initial State */
    app_state_init = 0,

    /* Enters when fast undirected advertisements are configured */
    app_state_advertising,

    /* Enters when device is not connected and not advertising */
    app_state_idle,

    /* Enters when connection is established with the host */
    app_state_connected,

    /* Enters when disconnect is initiated by the application */
    app_state_disconnecting,

} app_state;

typedef enum
{
    heater_off = 0,
    heater_on = 1
}heater_status_t;

/* GATT Service Data Structure */
typedef struct
{
    /* TYPED_BD_ADDR_T of the host to which device is connected */
    TYPED_BD_ADDR_T                con_bd_addr;

    /* Track the UCID as Clients connect and disconnect */
    uint16                         st_ucid;

    /* Boolean flag indicates whether the device is temporary paired or not */
    bool                           paired;

    /* Value for which advertisement timer needs to be started */
    uint32                         advert_timer_value;

    /* Store timer id while doing 'UNDIRECTED ADVERTS' and for Idle timer
     * in CONNECTED' states.
     */
    timer_id                       app_tid;

    /* Variable to store the current connection interval being used. */
    uint16                         conn_interval;

    /* Variable to store the current slave latency. */
    uint16                         conn_latency;

    /* Variable to store the current connection time-out value. */
    uint16                         conn_timeout;

    /* Variable to keep track of number of connection
     * parameter update requests made.
     */
    uint8                          num_conn_update_req;

    /* Store timer id for Connection Parameter Update timer in Connected
     * state
     */
    timer_id                       con_param_update_tid;

    /* Connection Parameter Update timer value. Upon a connection, it's started
     * for a period of TGAP_CPP_PERIOD, upon the expiry of which it's restarted
     * for TGAP_CPC_PERIOD. When this timer is running, if a GATT_ACCESS_IND is
     * received, it means, the central device is still doing the service discov-
     * -ery procedure. So, the connection parameter update timer is deleted and
     * recreated. Upon the expiry of this timer, a connection parameter update
     * request is sent to the central device.
     */
    uint32                         cpu_timer_value;

} APP_GATT_SERVICE_DATA_T;

/* CSRmesh Heater application data structure */
typedef struct
{
    /* Application GATT data */
    APP_GATT_SERVICE_DATA_T        gatt_data;

    /* Current state of application */
    app_state                      state;

    /* CSRmesh Association State */
    app_association_state          assoc_state;

    /* CSRmesh transmit device id advertise timer id */
    timer_id                       mesh_device_id_advert_tid;

    /* Local Device's Random Bluetooth Device Address. */
#ifdef USE_STATIC_RANDOM_ADDRESS
    BD_ADDR_T                      random_bd_addr;
#endif /* USE_STATIC_RANDOM_ADDRESS */

    /* Firmware Version Information */
#ifdef ENABLE_FIRMWARE_MODEL
    FIRMWARE_MODEL_STATE_DATA_T    fw_version;
#endif

    /* Battery Level Data */
#ifdef ENABLE_BATTERY_MODEL
    BATTERY_MODEL_STATE_DATA_T     battery_data;
#endif

    /* Bearer Model Data */
    BEARER_MODEL_STATE_DATA_T      bearer_data;

    /* Sensor Model Data. */
    SENSOR_MODEL_STATE_DATA_T      sensor_data;
    
    /* Heater status */
    heater_status_t                status;

} CSRMESH_HEATER_DATA_T;
/*20160531*/
void saveInputSensorNumberToNvm(void);
extern void saveReceiveDeviceIDToNvm(uint16 *buffer,uint16 length);
/*20160612*/
uint16 string2int(char *str,uint16 data_count); //uint8
/*20160823*/
uint16 str2inthex(char *str,uint16 data_count);
/*20160726*/
void saveHeaterStatusToNVM(void);
/*20160817*/
void clockMeshONTimerHandler(timer_id tid);
/*20160830*/
void clockMeshOFFTimerHandler(timer_id tid);
/*============================================================================*
 *  Public Data
 *============================================================================*/

/* CSRmesh Heater application specific data */
extern CSRMESH_HEATER_DATA_T g_heater_app_data;

/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

/* This function is used to set the state of the application */
extern void AppSetState(app_state new_state);

/* This function enables/disables the active scan mode */
extern void EnableHighDutyScanMode(bool enable);

#endif /* __CSR_MESH_TEMPSENSOR_H__ */

