/* 
 *  ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 * (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *  ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 * (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>

/************************************************************************
 * Defines
 ************************************************************************/
#define SENSOR_MNGR_NUMBERS     ((uint8_t)32u)

#define LAT_SIZE        ((uint8_t)10u)
#define LON_SIZE        ((uint8_t)11u)
#define GGA_MAX_SIZE    ((uint8_t)196u)//80u)   /* In case of OSP Geaodetic Data */
/************************************************************************
 * Type definitions
 ************************************************************************/
typedef union {
   struct
   {
      uint32_t bTemperature:1;      
      uint32_t bPressure:1;
      uint32_t bHumidity:1;
      uint32_t bUV:1;
		 
      uint32_t bLuminosity:1;
      uint32_t bAccelerometer:1; 
      uint32_t bGPS:1;
      uint32_t bMic:1;
		 
      uint32_t bCapaTouch:1;
      uint32_t bInertial:1;
      uint32_t bBattery:1;
		 
      uint32_t bUnused:21;
   }w32b;
   uint32_t u32SensorCfg;
}u_SensorMngtCfg_t;


typedef struct _SENSOR_DATA_  {
   int16_t s16Temp;
   uint16_t u16Pressure;
   uint8_t u8Humidity;
   uint16_t u16Brightness;
   int16_t s16AccelX;
   int16_t s16AccelY;
   int16_t s16AccelZ;
   uint8_t u8AccelState;
   uint8_t u8UVIndex;
   int16_t s16InAccelX;
   int16_t s16InAccelY;
   int16_t s16InAccelZ;
   int16_t s16InGyroX;
   int16_t s16InGyroY;
   int16_t s16InGyroZ;
   uint16_t u16InMagX;
   uint16_t u16InMagY;
   uint16_t u16InMagZ;
   uint8_t u8Mic;
   uint8_t u8Touch;
   uint16_t u16BatteryVoltage;
   int16_t s16BatteryAccCharge;
	uint8_t u8BatteryPercent;
   uint8_t u8GPSQuality;
   uint8_t au8Latitude[LAT_SIZE];   
   uint8_t au8Longitude[LON_SIZE];
   
   uint8_t u8SizeGGA;
   uint8_t au8GGAFrame[GGA_MAX_SIZE];
}s_SensorData_t;
/************************************************************************
 * Public function declarations
 ************************************************************************/
void vSensorMngr_Init(void);
void vSensorMngr_ShutDownAll(void);
void vSensorMngr_ShutdownSensor(uint32_t p_u32ActivatedSensors);
void vSensorMngr_UpdateSensorCfg(uint32_t p_u32SensorCfg);
void vSensorMngr_WakeUpSensor(uint32_t p_u32ActivatedSensors);
void vSensorMngr_DataUpdateSensor(uint32_t p_u32ActivatedSensors);
void vSensorMngr_DataUpdate(void);
void vSensorMngr_SigFoxDataUpdateStart(void);
void vSensorMngr_SigFoxDataUpdateStop(void);
void vSensorMngr_ConnectedWakeUp(void);

void vSensorMngr_TemperatureGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_PressureGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_HumidityGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_UvGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_BrightnessGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_AccelerometerGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_GPSGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_GPSQualityGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_CentralGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_MicGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_TouchGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);
void vSensorMngr_BatteryGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize);

uint8_t u8SensorMngr_IsSensorsDead(void);
void vSensorMngr_SensorsDeadGet(uint32_t *p_pu32DeadSensors);

void vRebootSensorsPower(void);

#endif /* SENSOR_MANAGER_H */

