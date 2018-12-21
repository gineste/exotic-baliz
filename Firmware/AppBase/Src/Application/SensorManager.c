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
 * Date:          09 08 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Sensor Manager file to get data value from Sensors. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>
#include <nrf_delay.h>

#include "HAL/HAL_SPI.h"
#include "HAL/HAL_I2C.h"
#include "HAL/HAL_GPIO.h"
#include "HAL/HAL_Timer.h"

#include "BoardConfig.h"

/* Sensor Drivers include */
#include "ADXL362/ADXL362.h"
#include "BME280/BME280.h"
//#include "ORG1411.h"
#include "MAX44009/MAX44009.h"
#include "LSM9DS1/LSM9DS1.h"
#include "VEML6075/VEML6075.h"
#include "AD7151/AD7151.h"
#include "LTC2942/LTC2942.h"
//#include "dBSPL.h"

#include "ES_Commands.h"
#include "UartManagement.h"  /* For GPS */
//#include "MemoryInterface.h"
//#include "DataUpdater.h"

//#include "SigFox.h"
//#include "AXSigFox.h"

//#include "MainStateMachine.h"
//#include "EventManager.h"

//#include "SensorCfgManagement.h"

#include "BoardConfig.h"
#include "GlobalDefs.h"

#include "SensorManager.h"

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef void (*fpvSensor_t)(void);
typedef enum _SENSOR_INIT_ {
   SENSOR_NOT_INIT = 0u,
   SENSOR_INIT
}e_SensorInit_t;

typedef struct _SENSOR_MNGNT_ {
   fpvSensor_t   fpvSensorsShutdown;
   fpvSensor_t   fpvSensorsWakeUp;
   fpvSensor_t   fpvSensorsDataGet;
}s_SensorManagement_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vInitnSleepAllSensors(void);

//static void vDataUpdateSensors(uint32_t p_u32ActivatedSensors);

static void vTemperatureStop(void);
static void vTemperatureActivate(void);

static void vPressureStop(void);
static void vPressureActivate(void);

static void vHumidityStop(void);
static void vHumidityActivate(void);

static void vBrightnessShutdown(void);
static void vBrightnessWakeUp(void);

static void vUvShutdown(void);
static void vUvWakeUp(void);

static void vAccelShutdown(void);
static void vAccelWakeUp(void);

static void vGPSShutdown(void);
static void vGPSWakeUp(void);

static void vMicShutdown(void);
static void vMicWakeUp(void);

static void vTouchShutdown(void);
static void vTouchWakeUp(void);

static void vCentralShutdown(void);
static void vCentralWakeUp(void);

static void vBatteryStop(void);
static void vBatteryActivate(void);

/************************************************************************
 * Variable declarations
 ************************************************************************/
#if (EN_BME280 == 1)
   static s_Bme280_Context_t g_sBME280Context = {
      .u8ChipId = 0u,
      .u8DevAddr = BME280_I2C_ADDR_1,
      .sSettings = {
         .eOversampling_P = BME280_OVERSAMPLING_1X,
         .eOversampling_T = BME280_OVERSAMPLING_1X,
         .eOversampling_H = BME280_OVERSAMPLING_1X,
         .eFilter = BME280_FILTER_COEFF_OFF,
         .eStandbyTime = BME280_STANDBY_TIME_1000_MS,
      },
   /*   .sCalibData = { 0 },*/
      .fpu32Write = NULL,
      .fpu32Read = NULL,
      .fpvDelayMs = &nrf_delay_ms,
   };
#endif /* EN_BME280 */
   
#if (EN_ADXL362 == 1)
   static s_ADXL362_Context_t g_sADXLContext = {
      .fp_u32SPITransfer = &u32Hal_SPI_Transfer,   /* Function pointer to a SPI transfer */
      .fp_vDelay_ms = &nrf_delay_ms,               /* Function pointer to a timer in ms */
      .fp_IntHandler = NULL/*&vDataUpdate_InterruptADXL362*/,/* Interrupt Handler for Activity and Inactivity */
      
      .eRange = ADXL362_RANGE_2G,                  /* Range of accelerometer */
      .eOutputDataRate = ADXL362_ODR_100_HZ,       /* Output Data Rate */
      .eLinkLoopMode = ADXL362_MODE_LOOP,          /* Functioning mode (Link/Loop) */
      .eNoiseCtrl = ADXL362_NOISE_MODE_NORMAL,     /* Noise control (Normal, Low, Ultra Low) */
      
      .u8ActivityDetection = 1u,			/* 0 - no activity detection, 1 - activity detection */
      .u8RefOrAbsActivity = 1u, 			/* 0 - absolute mode, 1 - referenced mode. */
      .u16ThresholdActivity = 0x280,   /* 11-bit unsigned value that the adxl362 samples are compared to. */
      .u16TimeActivity = 250u,			/* 16-bit value activity time in ms (from 2.5ms @ odr = 400Hz to 20s @ odr = 12.5Hz) */
      
      .u8InactivityDetection = 1u,		/* 0 - no activity detection, 1 - activity detection */
      .u8RefOrAbsInactivity = 1u,		/* 0 - absolute mode, 1 - referenced mode. */
      .u16ThresholdInactivity = 0x3FF, /* 11-bit unsigned value that the ADXL362 samples are compared to. */
      .u16TimeInactivity = 5000u,  		/* 16-bit value inactivity time in ms (from 164s @ odr = 400Hz to 87min @ odr = 12.5Hz) */
                                        
      .eInt1Map = ADXL362_INTMAPX_AWAKE,  /* Type of the interrupt n°1 */
      .eInt2Map = ADXL362_INTMAPX_OFF,    /* Type of the interrupt n°2 */
         
      .eWakeUpMode = ADXL362_WAKEUP_OFF,  /* Wake-up mode (Let it OFF if you adjustable ODR for (In)Activity detection) */
      .eMeasureMode = ADXL362_MEASURE_ON  /* Power mode (Standby or Measurement Mode) */
   };
#endif /* NO_ADXL362 */

#if (EN_MAX44009 == 1)
   static s_MAX44009_Context_t g_sMAXContext = {
      .eI2CAddress = MAX44009_ADDR1,               /* Sensor Address */
      .fp_u32I2C_Read = &u32Hal_I2C_WriteAndRead,  /* Function pointer to a read I2C transfer */
      .fp_u32I2C_Write = &u32Hal_I2C_Write,        /* Function pointer to a write I2C transfer */
      .fp_vDelay_ms = &nrf_delay_ms,               /* Function pointer to a timer in ms */
      
      .eCont = MAX44009_CONTINUOUS,                /* On demand conversion or Continuous(according to Integration time */
      .eMode = MAX44009_AUTOMATIC,                 /* Automatic or Manual Mode */   
      .eIntTime = MAX44009_INT_TIME_800MS,         /* Integration Time for conversion in manual mode */
      .eBrightnessMode = MAX44009_LOW_BRIGHTNESS,  /* Set to 1 for High Brightness Mode in Manual */
      
      .u8InterruptEnabled = 0u,                          /* Activate Interrupt with threshold */
      .u32UpperLuxThreshold = 100000u,                   /* Upper Luminosity threshold */
      .u32LowerLuxThreshold = 0u,                        /* Lower Luminosity threshold */
      .u16ThresholdTimer = 0u,                           /* Time necessary to handle interrupt after threshold reached */
      .fp_IntHandler = NULL/*&vDataUpdate_InterruptMAX44009*/,   /* Interrupt Handler for Lux threshold reached */
   };
#endif

//static u_SensorMngtCfg_t g_uSensorConfig = { .u32SensorCfg = 0u };
/*static*/ s_SensorData_t g_sSensorsData = { 0 };

/* Must be in the same order than SensorConfig in Flash Memory (u_MemSensorSection_t) */
static e_SensorInit_t g_eSensorsInit[SENSOR_MNGR_NUMBERS] = { SENSOR_NOT_INIT };
static const s_SensorManagement_t g_cafpvSensorsMngt[SENSOR_MNGR_NUMBERS] = {
   /*Shutdown,          WakeUp,                 DataUpdate */
   {vTemperatureStop,   vTemperatureActivate,   NULL,/*vDataUpdate_TPHGet*/},          /* Temperature */
   {vPressureStop,      vPressureActivate,      NULL/*vDataUpdate_TPHGet*/},          /* Pressure */
   {vHumidityStop,      vHumidityActivate,      NULL/*vDataUpdate_TPHGet*/},          /* Humidity */
   {vUvShutdown,        vUvWakeUp,              NULL/*vDataUpdate_UVGet*/},           /* UV */
   {vBrightnessShutdown,vBrightnessWakeUp,      NULL/*vDataUpdate_BrightnessGet*/},   /* Luminosity */
   {vAccelShutdown,     vAccelWakeUp,           NULL/*vDataUpdate_AccelGet*/},        /* ADXL */
   {vGPSShutdown,       vGPSWakeUp,             NULL/*vDataUpdate_GPSGet*/},          /* GPS */
   {vMicShutdown,       vMicWakeUp,             NULL/*vDataUpdate_MicGet*/},          /* Mic */
   {vTouchShutdown,     vTouchWakeUp,           NULL/*vDataUpdate_TouchGet*/},        /* Capa Touch */
   {vCentralShutdown,   vCentralWakeUp,         NULL/*vDataUpdate_InertialGet*/},     /* Inertial */
   {vBatteryStop,       vBatteryActivate,       NULL/*vDataUpdate_BatteryGet*/},      /* Battery Value */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
   {NULL,      NULL,      NULL}, /* Unused */
};


/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief   Function to Initialize sensor manager with the sensor from Configuration.
 * @return  None
 */
void vSensorMngr_Init(void)
{
//   u_SensorMngtCfg_t l_uInitSensorCfg = { .u32SensorCfg = 0u };
   /* Init and put sensors in sleep */
   vInitnSleepAllSensors();
   
//   /* Get Activated Sensors */
//   if(eMemItf_SensorConfigGet(&g_uSensorConfig.u32SensorCfg) == MEM_ITF_ERROR_NONE)
//   {  /* Add constant active sensor */
//      g_uSensorConfig.w32b.bBattery = 1u;      
//      g_uSensorConfig.w32b.bHumidity = 1u;
//      g_uSensorConfig.w32b.bTemperature = 1u;
//      g_uSensorConfig.w32b.bPressure = 1u;
//      g_uSensorConfig.w32b.bLuminosity = 1u;
//      g_uSensorConfig.w32b.bMic = 0u;
//      g_uSensorConfig.w32b.bUV = 1u;
//   }
//   else
//   {
//      g_uSensorConfig.u32SensorCfg = 0u;
//   }
//     
//   /* WakeUp Temperature and Battery for Advertise */
//   l_uInitSensorCfg.u32SensorCfg = 0u;
//   l_uInitSensorCfg.w32b.bTemperature = 1u;
//   l_uInitSensorCfg.w32b.bBattery = 1u;
//	 
//   vSensorCfgMng_SingleShotUpdate(l_uInitSensorCfg.u32SensorCfg);   
}

/**@brief   Function to Shutdown all sensors.
 * @return  None
 */
void vSensorMngr_ShutDownAll(void)
{
   /* Mask on all sensors */
   vSensorMngr_ShutdownSensor(0u);
}
/**@brief      Function to Shutdown specific sensors.
 * @param[in]  p_u32ActivatedSensors-
 * @return     None
 */
void vSensorMngr_ShutdownSensor(uint32_t p_u32ActivatedSensors)
{
   uint8_t l_u8Idx = 0u;
   uint32_t l_u32Mask = 0u;
   
   for(l_u8Idx = 0u;l_u8Idx < SENSOR_MNGR_NUMBERS; l_u8Idx++)
   {
      l_u32Mask = (uint32_t)((uint32_t)1u<<(uint32_t)l_u8Idx);
      if( BIT_MASK(p_u32ActivatedSensors,l_u32Mask,l_u8Idx) == 0u)
      {
         if(g_eSensorsInit[l_u8Idx] != SENSOR_NOT_INIT)
         {
            if(g_cafpvSensorsMngt[l_u8Idx].fpvSensorsShutdown != NULL)
            {
               (*g_cafpvSensorsMngt[l_u8Idx].fpvSensorsShutdown)();
            }
            g_eSensorsInit[l_u8Idx] = SENSOR_NOT_INIT;
         }
      }
   }
}

/**@brief      Function to Update current Sensor Configuration.
 * @param[in]  p_u32SensorCfg
 * @return     None
 */
void vSensorMngr_UpdateSensorCfg(uint32_t p_u32SensorCfg)
{
//   g_uSensorConfig.u32SensorCfg = p_u32SensorCfg;
//   g_uSensorConfig.w32b.bBattery = 1u;      
//   g_uSensorConfig.w32b.bHumidity = 1u;
//   g_uSensorConfig.w32b.bTemperature = 1u;
//   g_uSensorConfig.w32b.bPressure = 1u;
//   g_uSensorConfig.w32b.bLuminosity = 1u;
//   g_uSensorConfig.w32b.bMic = 0u;
//   g_uSensorConfig.w32b.bUV = 1u;
}

/**@brief      Function to WakeUp specific sensors.
 * @param[in]  p_u32ActivatedSensors
 * @return     None
 */
void vSensorMngr_WakeUpSensor(uint32_t p_u32ActivatedSensors)
{
   uint8_t l_u8Idx = 0u;
   uint32_t l_u32Mask = 0u;
   
   for(l_u8Idx = 0u;l_u8Idx < SENSOR_MNGR_NUMBERS; l_u8Idx++)
   {
      l_u32Mask = (uint32_t)((uint32_t)1u<<(uint32_t)l_u8Idx);
      if( BIT_MASK(p_u32ActivatedSensors,l_u32Mask,l_u8Idx) == 1u)
      {
         if(g_cafpvSensorsMngt[l_u8Idx].fpvSensorsWakeUp != NULL)
         {
            if(g_eSensorsInit[l_u8Idx] == SENSOR_NOT_INIT)
            {
               (*g_cafpvSensorsMngt[l_u8Idx].fpvSensorsWakeUp)();
               g_eSensorsInit[l_u8Idx] = SENSOR_INIT;
            }
         }
      }
   }
}

/**@brief      Function to Update Sensors on demand.
 * @return     None
 */
void vSensorMngr_DataUpdateSensor(uint32_t p_u32ActivatedSensors)
{
//   vDataUpdateSensors(p_u32ActivatedSensors);
}

/**@brief      Background function to Update Sensors.
 * @return     None
 */
void vSensorMngr_DataUpdate(void)
{
//   vDataUpdateSensors(g_uSensorConfig.u32SensorCfg);
}

/**@brief      Background function to Update Sensors when Sending Data sensors with Sigfox.
 * @return     None
 */
void vSensorMngr_SigFoxDataUpdateStart(void)
{
//   u_SensorMngtCfg_t l_uSensorCfg = { .u32SensorCfg = 0u };
//   static const u_SensorMngtCfg_t l_cuSigFoxSensorConfig = { 
//   .w32b = {
//      .bTemperature = 1u,
//      .bPressure = 1u,
//      .bHumidity = 1u,
//      .bLuminosity = 1u,
//      .bUV = 1u,
//      .bBattery = 1u,
//      .bGPS = 1u,
//      }
//   };
//   
//   l_uSensorCfg.u32SensorCfg = (g_uSensorConfig.u32SensorCfg | l_cuSigFoxSensorConfig.u32SensorCfg);
//   
////   vSensorMngr_WakeUpSensor(l_uSensorCfg.u32SensorCfg);
////   vDataUpdateSensors(l_uSensorCfg.u32SensorCfg);
//   
//   vSensorCfgMng_SingleShotUpdate(l_uSensorCfg.u32SensorCfg);
}
void vSensorMngr_SigFoxDataUpdateStop(void)
{
//   u_SensorMngtCfg_t l_uSensorCfg = { .u32SensorCfg = 0u };
////   static const u_SensorMngtCfg_t l_cuSigFoxSensorConfig = { 
////   .w32b = {
////      .bTemperature = 1u,
////      .bPressure = 1u,
////      .bHumidity = 1u,
////      .bLuminosity = 1u,
////      .bUV = 1u,
////      .bBattery = 1u,
////      .bGPS = 1u,
////      }
////   };
//   
////   l_uSensorCfg.u32SensorCfg = (g_uSensorConfig.u32SensorCfg | l_cuSigFoxSensorConfig.u32SensorCfg);
////   vDataUpdateSensors(l_uSensorCfg.u32SensorCfg);
//   
//   vSensorCfgMng_SingleShotUpdate(l_uSensorCfg.u32SensorCfg);
}

void vSensorMngr_ConnectedWakeUp(void)
{
//   vSensorMngr_ShutDownAll();n
}

/**@brief   Function to get sensor data from Temperature sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Temperature MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_TemperatureGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_TEMPERATURE;
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16Temp);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16Temp);
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Pressure sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Pressure MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_PressureGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_PRESSURE;
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.u16Pressure);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.u16Pressure);
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from humidity sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return humidity 
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_HumidityGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{ 
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_HUMIDITY;
   p_pu8Data[l_u8Size++] = g_sSensorsData.u8Humidity;
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Uv sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Uv (A,B) MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_UvGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_UV_INDEX;
   p_pu8Data[l_u8Size++] = g_sSensorsData.u8UVIndex;
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Brigthness sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Brigthness MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_BrightnessGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_BRIGHTNESS;
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.u16Brightness);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.u16Brightness);
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Accelerometer sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Accelerometer (X,Y,Z) MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_AccelerometerGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_ACCELEROMETER;
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16AccelX);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16AccelX);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16AccelY);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16AccelY);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16AccelZ);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16AccelZ);
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from GPS in FrameBuilder typedef.
 * @param[out] p_pu8Data : return GPS (Lat,Lon) MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_GPSGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   uint8_t l_u8Idx = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_GGA_LOGGING;   
   p_pu8Data[l_u8Size++] = (uint8_t)g_sSensorsData.u8SizeGGA;
   
//   memcpy(&p_pu8Data[l_u8Size], g_sSensorsData.au8GGAFrame, g_sSensorsData.u8SizeGGA);   
//   l_u8Size += g_sSensorsData.u8SizeGGA;
   
   for(l_u8Idx = 0u;l_u8Idx < g_sSensorsData.u8SizeGGA; l_u8Idx++)
   {
      p_pu8Data[l_u8Size++] = g_sSensorsData.au8GGAFrame[l_u8Idx];
   }
   
   /*for(l_u8Idx = 0u;l_u8Idx < LAT_SIZE; l_u8Idx++)
   {
      p_pu8Data[l_u8Size++] = g_sSensorsData.au8Latitude[l_u8Idx];
   }
   for(l_u8Idx = 0u;l_u8Idx < LON_SIZE; l_u8Idx++)
   {
      p_pu8Data[l_u8Size++] = g_sSensorsData.au8Longitude[l_u8Idx];
   }*/   
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get GPS quality in FrameBuilder typedef.
 * @param[out] p_pu8Data : return GPS quality
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_GPSQualityGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = 0xFFu;
   p_pu8Data[l_u8Size++] = g_sSensorsData.u8GPSQuality;
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Inertial sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Inertial (AccelX,AccelY,AccelZ,GiroX,
 *                         GiroY,GiroZ,MagX,MagY,MagZ) MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_CentralGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_CENTRAL;
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16InAccelX);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16InAccelX);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16InAccelY);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16InAccelY);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16InAccelZ);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16InAccelZ);
   
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16InGyroX);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16InGyroX);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16InGyroY);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16InGyroY);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16InGyroZ);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16InGyroZ);
   
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.u16InMagX);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.u16InMagX);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.u16InMagY);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.u16InMagY);
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.u16InMagZ);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.u16InMagZ);
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Microphone sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return frequency of Mic MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_MicGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_MICROPHONE;
   p_pu8Data[l_u8Size++] = g_sSensorsData.u8Mic;
   
   (*p_pu8DataSize) = l_u8Size;
}
/**@brief   Function to get sensor data from Capa Touch sensor in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Capacitive data MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_TouchGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_CAPACITIVE;
   p_pu8Data[l_u8Size++] = g_sSensorsData.u8Touch;
   
   (*p_pu8DataSize) = l_u8Size;
}


/**@brief   Function to get battery value from SAADC in FrameBuilder
 *          typedef.
 * @param[out] p_pu8Data : return Battery value MSB first
 * @param[out] p_pu8DataSize : return size of data
 * @return     None
 */
void vSensorMngr_BatteryGet(uint8_t * p_pu8Data, uint8_t * p_pu8DataSize)
{
   uint8_t l_u8Size = 0u;
	   
   p_pu8Data[l_u8Size++] = (uint8_t)OP_CODE_BATTERY;
	/* Percent 1byte */
   p_pu8Data[l_u8Size++] = g_sSensorsData.u8BatteryPercent;
	
	/* Voltage 2 bytes */
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.u16BatteryVoltage);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.u16BatteryVoltage);
	
	/* Charge 2 bytes */
   p_pu8Data[l_u8Size++] = MSB_16B_TO_8B(g_sSensorsData.s16BatteryAccCharge);
   p_pu8Data[l_u8Size++] = LSB_16B_TO_8B(g_sSensorsData.s16BatteryAccCharge);
   
   (*p_pu8DataSize) = l_u8Size;
}

/**@brief Check if sensors are dead
 * @return 1 if sensor dead else 0.
 */
uint8_t u8SensorMngr_IsSensorsDead(void)
{
   uint8_t l_u8IsAvailable = 0u;
   uint8_t l_u8SensorNb = 0u;
   
#if (EN_VEML6075 == 1)
   l_u8IsAvailable += u8VEML6075_IsAvailable();
   l_u8SensorNb++;
#endif
   
#if (EN_LSM9DS1 == 1)
   l_u8IsAvailable += u8LSM9DS1_IsAvailable();
   l_u8SensorNb++;
#endif
   
#if (EN_MAX44009 == 1)
   l_u8IsAvailable += u8MAX44009_IsAvailable();
   l_u8SensorNb++;
#endif
  
#if (EN_ADXL362 == 1)
   l_u8IsAvailable += u8ADXL362_IsAvailable();
   l_u8SensorNb++;
#endif
   
#if (EN_BME280 == 1)
   l_u8IsAvailable += u8BME280_IsAvailable();
   l_u8SensorNb++;
#endif

#if (EN_LTC2942 == 1)
      l_u8IsAvailable += u8LTC2942_IsAvailable();
      l_u8SensorNb++;
#endif
      
   return (l_u8IsAvailable == l_u8SensorNb)?0u:1u;
}

void vSensorMngr_SensorsDeadGet(uint32_t *p_pu32DeadSensors)
{   
   u_SensorMngtCfg_t l_uSensorDead = { .u32SensorCfg = 0u, };
   if(p_pu32DeadSensors != NULL)
   {
      
   #if (EN_VEML6075 == 1)
      l_uSensorDead.w32b.bUV = (u8VEML6075_IsAvailable() == 1u)?0u:1u;
   #endif
      
   #if (EN_LSM9DS1 == 1)
      l_uSensorDead.w32b.bInertial = (u8LSM9DS1_IsAvailable() == 1u)?0u:1u;
   #endif
      
   #if (EN_MAX44009 == 1)
      l_uSensorDead.w32b.bLuminosity = (u8MAX44009_IsAvailable() == 1u)?0u:1u;
   #endif
     
   #if (EN_ADXL362 == 1)
      l_uSensorDead.w32b.bAccelerometer = (u8ADXL362_IsAvailable() == 1u)?0u:1u;
   #endif
      
   #if (EN_BME280 == 1)
      l_uSensorDead.w32b.bTemperature = (u8BME280_IsAvailable() == 1u)?0u:1u;
      l_uSensorDead.w32b.bPressure = l_uSensorDead.w32b.bTemperature;
      l_uSensorDead.w32b.bHumidity = l_uSensorDead.w32b.bTemperature;
   #endif

#if (EN_LTC2942 == 1)
      l_uSensorDead.w32b.bBattery = (u8LTC2942_IsAvailable() == 1u)?0u:1u;
#endif
      (*p_pu32DeadSensors) = l_uSensorDead.u32SensorCfg;
   }
}

/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief   Function to Initialize all sensors and put in them in deepsleep mode
 *          if possible.
 * @return  None
 */
static void vInitnSleepAllSensors(void)
{
#if (EN_ADXL362 == 1)
   vADXL362_ContextSet(g_sADXLContext); 
   if(eADXL362_Init() != ADXL362_ERROR_NONE)
   {
      vRebootSensorsPower();
      (void)eADXL362_Init();     
   }
	(void)eADXL362_MeasureModeSet(ADXL362_MEASURE_STANDBY);
#endif
   
#if (EN_LTC2942 == 1)
      vLTC2942_Init();
#endif
   
#if (EN_BME280 == 1)
      eBME280_Initialization(g_sBME280Context);
#endif
   
#if (EN_LSM9DS1 == 1)
   (void)LSM9DS1_Init();
   //   (void)LSM9DS1_Shutdown();
#endif

#if (EN_VEML6075 == 1)
   vVEML6075_Init();
#endif /* EN_VEML6075 */

#if (EN_MAX44009 == 1)
   /*(void)MAX44009_Init();*/
   (void)eMAX44009_Initialization(g_sMAXContext);
#endif /* NO_MAX44009 */

#if (EN_AD7151 == 1)
   (void)eAD7151_Initialization();
#endif

#if (EN_MIC == 1)
   vMicInit();
#endif
}

/**@brief   Function to shutdown and restore power supply of sensors in case of failure.
 *          Note : Only available for Graal Board version 2.
 * @return  None
 */
 void vRebootSensorsPower(void)
{
#if defined(GRAAL_BOARD)
   /* Clear CS since it can produce voltage to others sensors */
   vHal_GPIO_Clear(ADXL_CS);   
   
   vHal_GPIO_Set(SW_POWER_SENSOR);   
   nrf_delay_ms(2000u);
   vHal_GPIO_Clear(SW_POWER_SENSOR);
   nrf_delay_ms(100u);
#endif
}

/**@brief      Function to update specific sensor data values.
 * @param[in]  p_u32ActivatedSensors
 * @return     None
 */
//static void vDataUpdateSensors(uint32_t p_u32ActivatedSensors)
//{
//   uint8_t l_u8Idx = 0u;
//   uint32_t l_u32Mask = 0u;
//   
//   for(l_u8Idx = 0u;l_u8Idx < SENSOR_MNGR_NUMBERS; l_u8Idx++)
//   {
//      l_u32Mask = (uint32_t)((uint32_t)1u<<(uint32_t)l_u8Idx);
//      if( ((p_u32ActivatedSensors & l_u32Mask)>>(uint32_t)l_u8Idx) == 1u)
//      {
//         if(g_cafpvSensorsMngt[l_u8Idx].fpvSensorsDataGet != NULL)
//         {
//            (*g_cafpvSensorsMngt[l_u8Idx].fpvSensorsDataGet)();
//         }
//      }
//   }
////   /* Get latest value of all sensors */
////   vDataUpdate_DataSensorValueGet(&g_sSensorsData);
//}


/**@brief Function to stop temperature acquisition of data from BME280.
 */
static void vTemperatureStop(void)
{
   eBME280_SensorDisable(BME280_TEMP_SENSOR);
}
/**@brief Function to activate temperature acquisition of data from sensor.
 */
static void vTemperatureActivate(void)
{
//   (void)eBME280_Initialization(g_sBME280Context);
   eBME280_SensorEnable(BME280_TEMP_SENSOR);
}

/**@brief Function to stop pressure acquisition of data from BME280.
 */
static void vPressureStop(void)
{   
   eBME280_SensorDisable(BME280_PRESS_SENSOR);
}
/**@brief Function to activate pressure acquisition of data from sensor.
 */
static void vPressureActivate(void)
{
//   (void)eBME280_Initialization(g_sBME280Context);
  eBME280_SensorEnable(BME280_PRESS_SENSOR);
}

/**@brief Function to stop humidity acquisition of data from BME280.
 */
static void vHumidityStop(void)
{
   eBME280_SensorDisable(BME280_HUM_SENSOR);
}
/**@brief Function to activate humidity acquisition of data from sensor.
 */
static void vHumidityActivate(void)
{
//   (void)eBME280_Initialization(g_sBME280Context);
   eBME280_SensorEnable(BME280_HUM_SENSOR);
}

/**@brief Function to stop luminosity acquisition of data from sensor.
 */
static void vBrightnessShutdown(void)
{
#if (EN_MAX44009 == 1)
   /*MAX44009_Shutdown();*/
#endif /* EN_MAX44009 */
}
/**@brief Function to activate luminosity acquisition of data from sensor.
 */
static void vBrightnessWakeUp(void)
{
#if (EN_MAX44009 == 1)
   /*(void)MAX44009_Init();*/
#endif /* EN_MAX44009 */
}

/**@brief Function to stop uv acquisition of data from sensor.
 */
static void vUvShutdown(void)
{   
#if (EN_VEML6075 == 1)
   /*vVEML6075_ShutDown();*/
#endif /* EN_VEML6075 */
}
/**@brief Function to activate uv acquisition of data from sensor.
 */
static void vUvWakeUp(void)
{
#if (EN_VEML6075 == 1)
   /*vVEML6075_Init();*/
#endif /* EN_VEML6075 */
}

/**@brief Function to stop accelerometer acquisition of data from sensor.
 */
static void vAccelShutdown(void)
{
#if (EN_ADXL362 == 1)
   //(void)eADXL362_MeasureModeSet(ADXL362_MEASURE_STANDBY);
#endif /* EN_ADXL362 */
}
/**@brief Function to activate accelerometer acquisition of data from sensor.
 */
static void vAccelWakeUp(void)
{
#if (EN_ADXL362 == 1)
   //(void)eADXL362_MeasureModeSet(ADXL362_MEASURE_ON);
#endif /* EN_ADXL362 */
}

/**@brief Function to stop GPS acquisition of data from sensor.
 */
static void vGPSShutdown(void)
{
#if (EN_ORG1411 == 1)
   //vORG1411_Shutdown();
#endif /* EN_ORG1411 */
}
/**@brief Function to activate GPS acquisition of data from sensor.
 */
static void vGPSWakeUp(void)
{
#if (EN_ORG1411 == 1)
   eUartMngt_StateSet(USM_GPS);
   //vUart_ModeSet(UART_GPS);
   //vORG1411_Init();
#endif /* EN_ORG1411 */
}

/**@brief Function to stop microphone acquisition of data from sensor.
 */
static void vMicShutdown(void)
{
#if (EN_MIC == 1)
   /*vMicUninit();*/
#endif /* EN_MIC */
}
/**@brief Function to activate mic acquisition of data from sensor.
 */
static void vMicWakeUp(void)
{
#if (EN_MIC == 1)
   /*vMicInit();*/
#endif /* EN_MIC */
}

/**@brief Function to stop touch acquisition of data from sensor.
 */
static void vTouchShutdown(void)
{   
}
/**@brief Function to activate capa touch acquisition of data from sensor.
 */
static void vTouchWakeUp(void)
{
}

/**@brief Function to stop inertial acquisition of data from sensor.
 */
static void vCentralShutdown(void)
{   
#if (EN_LSM9DS1 == 1)
   //(void)LSM9DS1_Shutdown();
#endif /* EN_LSM9DS1 */
}
/**@brief Function to activate intertial acquisition of data from sensor.
 */
static void vCentralWakeUp(void)
{
#if (EN_LSM9DS1 == 1)
   //(void)LSM9DS1_Init();
#endif /* EN_LSM9DS1 */
}

/**@brief Function to stop battery value acquisition from SAADC.
 */
static void vBatteryStop(void)
{   
}
/**@brief Function to activate battery value acquisition from sensor.
 */
static void vBatteryActivate(void)
{   
   #if (EN_LTC2942 == 1)
      //vLTC2942_WakeUp();
   #endif /* EN_LTC2942 */
}

/************************************************************************
 * End Of File
 ************************************************************************/


