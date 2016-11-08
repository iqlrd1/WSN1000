###########################################################
# Makefile generated by xIDE for uEnergy                   
#                                                          
# Project: CSRMeshTempSensor
# Configuration: Release
# Generated: 周四 十一月 3 08:37:24 2016
#                                                          
# WARNING: Do not edit this file. Any changes will be lost 
#          when the project is rebuilt.                    
#                                                          
###########################################################

XIDE_PROJECT=CSRMeshTempSensor
XIDE_CONFIG=Release
OUTPUT=CSRMeshTempSensor
OUTDIR=C:/CSR_uEnergy_SDK-2.4.5.13/apps/CSRmesh-1.3-Examples-Applications_icp/applications/WSN1000-Sensor -20160824
DEFS=

OUTPUT_TYPE=0
USE_FLASH=0
ERASE_NVM=1
CSFILE_CSR101x_A05=tempsensor_csr101x_A05.keyr
MASTER_DB=app_gatt_db.db
LIBPATHS=..\..\libraries
INCPATHS=..\..\include
STRIP_SYMBOLS=0
OTAU_BOOTLOADER=1
OTAU_CSFILE=otau_bootloader.keyr
OTAU_NAME=CSRmesh-OTA
OTAU_SECRET=

LIBS=csrmesh bearermodel batterymodel sensormodel attentionmodel streammodel actuatormodel firmwaremodel 
DBS=\
\
      app_gatt_db.db\
      mesh_control_service_db.db\
      gap_service_db.db\
      gatt_service_db.db\
      csr_ota_db.db

INPUTS=\
      gap_service.c\
      mesh_control_service.c\
      csr_mesh_tempsensor.c\
      csr_mesh_tempsensor_gatt.c\
      nvm_access.c\
      iot_hw.c\
      csr_ota_service.c\
      gatt_service.c\
      battery_hw.c\
      tempsensor_hw.c\
      app_data_stream.c\
      iic.c\
      sensor_bmp180_barometer.c\
      sensor_si7034.c\
      i2c_sim.c\
      sensor_bme280.c\
      $(DBS)

KEYR=\
      tempsensor_csr101x_A05.keyr\
      otau_bootloader.keyr

# Project-specific options
hw_version=0

-include CSRMeshTempSensor.mak
include $(SDK)/genmakefile.uenergy
