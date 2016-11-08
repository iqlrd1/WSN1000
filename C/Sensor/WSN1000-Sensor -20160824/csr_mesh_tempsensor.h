/*****************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2015
 *  CSR Bluetooth Low Energy CSRmesh 1.3 Release
 *  Application version 1.3
 *
 *  FILE
 *      csr_mesh_tempsensor.h
 *
 *  DESCRIPTION
 *      This file defines a simple implementation of CSRmesh
 *      Temperature Sensor device
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

/*============================================================================*
 *  Local Header File
 *============================================================================*/
#ifdef ENABLE_BATTERY_MODEL
#include <battery_model.h>
#endif /* ENABLE_BATTERY_MODEL */

#ifdef ENABLE_FIRMWARE_MODEL
#include <firmware_model.h>
#endif

#include "csr_mesh_tempsensor_gatt.h"
#include <bearer_model.h>
#include <sensor_model.h>
#include <actuator_model.h>
#include "app_data_stream.h"
/*============================================================================*
 *  Public Definitions
 *============================================================================*/
//#define SW1     4   /*20160530*/
/*20160602*/
#define TIMER_BUTTON_POWERON            5*SECOND    //press key after power on 
#define TIMER_BUTTON_FORCESEND          5*SECOND    //long press to start send environment parameters
#define TIMER_RESET_BUTTON_HOLD         5*SECOND    //long press to exit register
#define TIMER_SENSOR_READ_INIT_INTERVAL 10*MILLISECOND    //get immediate environment parameters once entering normal state
//#define TIMER_SENSOR_READ_INTERVAL      3*SECOND   //read/sampling time interval
#define TIMER_STATUS_PEND_REG           3*MINUTE    //register state 3min receive register-cmd 
#define TIMER_REPLY_DEVID_DELAY         5*SECOND    //resend deivice id delay (wait for heater CFM signal)

//#define TIMER_IDLESTATE                 2*MINUTE    //20*SECOND
//#define TIMER_MESHON_NOT_SLEEP          2*MINUTE     //5*SECOND
//#define TIMER_MESHOFF_SLEEP             13*MINUTE     //5*SECOND

#define TIMER_START_MESHSWITCH          10*MILLISECOND  //15*SECOND
#define TIMER_MESHON_NOT_SLEEP          1*MINUTE    //20*SECOND    
#define TIMER_MESHOFF_SLEEP             10*SECOND   //1*SECOND

#define TIMER_SENDDATA_INTERVAL         30*MINUTE   //20*SECOND 
#define PRESSED     0
#define RELESED     1
/*============================================================================*
 *  Public Data Types
 *============================================================================*/

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

    /* Enters when Undirected advertisements are configured */
    app_state_advertising,

    /* Enters when device is not connected and not advertising */
    app_state_idle,

    /* Enters when connection is established with the host */
    app_state_connected,

    /* Enters when disconnect is initiated by the application */
    app_state_disconnecting,

} app_state;

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

    /* Store timer id while doing 'UNDIRECTED ADVERTS' */
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

/* CSRmesh Temperature Sensor application data structure */
typedef struct
{
    /* Application GATT data */
    APP_GATT_SERVICE_DATA_T        gatt_data;

    /* Current state of application */
    app_state                       state;

    /* CSRmesh Association State */
    app_association_state           assoc_state;

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
    
    /* Actuator Model Data. */
    ACTUATOR_MODEL_STATE_DATA_T    actuator_data;

} CSRMESH_TEMPSENSOR_DATA_T;

/*TEST CODE 20160412*/
/* CSRmesh humidity Sensor application data structure */
typedef struct
{   
    /* Sensor Model Data. */
    SENSOR_MODEL_STATE_DATA_T      sensor_data;
    
    /* Actuator Model Data. */
    ACTUATOR_MODEL_STATE_DATA_T    actuator_data;

} CSRMESH_HUMISENSOR_DATA_T;
/*20160530*/
typedef enum
{   sensor_status_init=0,
    sensor_status_init_pressed,
    sensor_status_register,
    sensor_status_normal,
    sensor_status_error,
}sensor_status_t;
sensor_status_t sensor_status;
/*============================================================================*
 *  Public Data
 *============================================================================*/

/* CSRmesh Temperature Sensor application specific data */
extern CSRMESH_TEMPSENSOR_DATA_T g_tsapp_data;
/*new code20160513*/
extern void normalAdvertDevice(timer_id tid);
/*20160705*/
void saveSensorStatusToNVM(void);
/*20160721*/
void timerHandleReadBME280Data(timer_id tid);

uint16 local_device_id;
CSR_MESH_NODE_DATA_T g_node_data;
/*============================================================================*
 *  Public Function Prototypes
 *============================================================================*/

void writeTempValue(void);
/* This function is used to set the state of the application */
extern void AppSetState(app_state new_state);

/* This function enables/disables the active scan mode */
extern void EnableHighDutyScanMode(bool enable);

#endif /* __CSR_MESH_TEMPSENSOR_H__ */

