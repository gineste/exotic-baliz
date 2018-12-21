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
 * Date:          22 08 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Exotic Driver for BME280
 *
 */

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>

#include "BME280.h"

/************************************************************************
 * Defines
 ************************************************************************/
/****************/
/* Register Map */
/****************/
/* Control registers */
#define CTRL_REG_HUM                   (uint8_t)(0xF2)
#define CTRL_REG_MEAS                  (uint8_t)(0xF4)
/* Configuration register */
#define CFG_REG_CONFIG                 (uint8_t)(0xF5)
/* Calibration registers */
#define CALIB_REG_TEMP_PRESS_DATA      (uint8_t)(0x88)
#define CALIB_REG_HUM_DATA_H1          (uint8_t)(0xA1)
#define CALIB_REG_HUMIDITY_DATA        (uint8_t)(0xE1)
/* Data registers */
#define DATA_REG_PRESS_TEMP_HUM        (uint8_t)(0xF7)
/* Status register */
#define STATUS_REG                     (uint8_t)(0xF3)
/* Info register */
#define INFO_REG_CHIP_ID               (uint8_t)(0xD0)
/* Reset register */
#define RESET_REG                      (uint8_t)(0xE0)

#define LAST_REGISTER                  (uint8_t)(0xFF)


#define CALIB_TEMP_PRESS_DATA_LEN	   (uint8_t)(26u)

#define CALIB_H1_DATA_LEN		         (uint8_t)(1u)
#define CALIB_H2_TO_H6_DATA_LEN		   (uint8_t)(7u)
#define RAW_SENSOR_P_T_H_DATA_LEN      (uint8_t)(8u)

/****************/
/* Pos and Mask */
/****************/
/* Config */
#define CFG_SPI3W_EN_POS               (uint8_t)(0)
#define CFG_SPI3W_EN_MSK               (uint8_t)(0x01)
#define CFG_FILTER_POS                 (uint8_t)(2)
#define CFG_FILTER_MSK                 (uint8_t)(0x1C)
#define CFG_T_STANDBY_POS              (uint8_t)(5)
#define CFG_T_STANDBY_MSK              (uint8_t)(0xE0)

/* Control */
#define CTRL_HUM_OSR_HUM_POS           (uint8_t)(0)
#define CTRL_HUM_OSR_HUM_MSK           (uint8_t)(0x07)

#define CTRL_MEAS_MODE_POS             (uint8_t)(0)
#define CTRL_MEAS_MODE_MSK             (uint8_t)(0x03)
#define CTRL_MEAS_OSR_PRESS_POS        (uint8_t)(2)
#define CTRL_MEAS_OSR_PRESS_MSK        (uint8_t)(0x1C)
#define CTRL_MEAS_OSR_TEMP_POS         (uint8_t)(5)
#define CTRL_MEAS_OSR_TEMP_MSK         (uint8_t)(0xE0)

/* Status */
#define STATUS_MEASURING_POS           (uint8_t)(3)
#define STATUS_MEASURING_MSK           (uint8_t)(0x08)
#define STATUS_NVM_INIT_POS            (uint8_t)(0)
#define STATUS_NVM_INIT_MSK            (uint8_t)(0x01)

#define VALUE_BITS_GET(Reg, Pos, Msk)  ((Reg & Msk) << Pos)

#define MEAS_TIME_MS(Tosr, Posr, Hosr) (uint8_t) (1.0f + (2.0f * Tosr) + (2.0f * Posr + 0.5f) + (2.0f * Hosr + 0.5f))

#define RESET_CMD_SOFTRESET            (uint8_t)(0xB6)

#define REGISTER_ADDR_IDX              (uint8_t)(0u)
#define DATA_ADDR_IDX                  (uint8_t)(1u)
#define REGISTER_SIZE                  (uint8_t)(1u)
#define BUFFER_SIZE                    (uint8_t)(8u)

#define INIT_RETRY                     (uint8_t)(5u)
#define MEAS_RETRY                     (uint8_t)(5u)

#define DELAY_WAKEUP                   (uint32_t)(300u)
#define DELAY_INIT                     (uint32_t)(100u)
#define DELAY_MEAS                     (uint32_t)(10u)
#define DELAY_NVM                      (uint32_t)(5u)

#define RESOLUTION_T                   (float)100.0f
#define RESOLUTION_P                   (float)100.0f
#define RESOLUTION_H                   (float)1000.0f

#define INVALID_UNCOMP_PRESSURE        (int32_t)0x80000
#define INVALID_UNCOMP_TEMPERATURE     (int32_t)0x80000
#define INVALID_UNCOMP_HUMIDITY        (uint16_t)0x8000

#define MAX_TEMPERATURE_DEG            (float)(85.0f)
#define MIN_TEMPERATURE_DEG            (float)(-40.0f)
#define MAX_PRESSURE_HPA               (float)(1100.0f)
#define MIN_PRESSURE_HPA               (float)(300.0f)
#define MAX_HUMIDITY_PERCENT           (float)(100.0f)
#define MIN_HUMIDITY_PERCENT           (float)(0.0f)

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != BME280_ERROR_NONE))    \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);

/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef struct _ADC_VALUES_ {
   uint32_t u32RegTemp;
   uint32_t u32RegPress;
   uint32_t u32RegHum;
}s_AdcRegisters_t;

typedef struct _CONFIG_REGISTER_ {
   uint8_t u8Config;
   uint8_t u8CtrlMeas;
   uint8_t u8CtrlHum;
}s_ConfigRegisters_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static e_BME280_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber);
static e_BME280_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Value);
static e_BME280_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask);

static e_BME280_Error_t eBasicInitialization(void);
static e_BME280_Error_t eReset(void);
static e_BME280_Error_t eChipIDValidation(void);
static e_BME280_Error_t eCalibrationDataRead(void);
static e_BME280_Error_t eIsUpdating(uint8_t * p_pu8Updating);
static e_BME280_Error_t eIsMeasuring(uint8_t * p_pu8IsMeasuring);

static e_BME280_Error_t eCompensate_P(int32_t p_s32UncompPress, uint32_t * p_pu32CompPress);
static e_BME280_Error_t eCompensate_H(int32_t p_s32UncompHum, uint32_t * p_pu32CompHum);
static e_BME280_Error_t eCompensate_T(int32_t p_s32UncompTemp, int32_t * p_s32CompTemp);

/************************************************************************
 * Variable declarations
 ************************************************************************/
static s_BME280_Context_t g_sBme280Context;
static s_BME280_Settings_t g_sBme280Settings;

static uint8_t g_u8BMEInitialized = 0u;
static uint8_t g_u8BMECommFailure = 0u;

static s_BME280_Calibration_Data_t g_sCalib;
static s_AdcRegisters_t g_sADCReg = { 0u };

/************************************************************************
 * Public functions
 ************************************************************************/
/**@brief Function to initialized communication and settings of BME280.
 * @param[in]  p_sContext
 * @return   Error Code
 */
e_BME280_Error_t eBME280_ContextSet(s_BME280_Context_t p_sContext)
{
   e_BME280_Error_t l_eErrCode = BME280_ERROR_NONE;

   if(p_sContext.fp_vTimerDelay_ms == NULL)
   {
      l_eErrCode = BME280_ERROR_PARAM;
   }
   else
   {
      g_sBme280Context.fp_vTimerDelay_ms = p_sContext.fp_vTimerDelay_ms;

      if(p_sContext.eInterface == BME280_I2C_ITF)
      {
         g_sBme280Context.eInterface = p_sContext.eInterface;
         if(   (p_sContext.sI2CCfg.fp_u32I2C_Write != NULL)
            && (p_sContext.sI2CCfg.fp_u32I2C_Read != NULL) )
         {
            g_sBme280Context.sI2CCfg.fp_u32I2C_Write = p_sContext.sI2CCfg.fp_u32I2C_Write;
            g_sBme280Context.sI2CCfg.fp_u32I2C_Read = p_sContext.sI2CCfg.fp_u32I2C_Read;
            g_sBme280Context.sI2CCfg.eI2CAddr = p_sContext.sI2CCfg.eI2CAddr;
         }
         else
         {
            l_eErrCode = BME280_ERROR_PARAM;
         }
      }
      else if(p_sContext.eInterface == BME280_SPI_ITF)
      {
         g_sBme280Context.eInterface = p_sContext.eInterface;
         if(   (p_sContext.sSPICfg.fp_vPinSet != NULL)
            && (p_sContext.sSPICfg.fp_vPinClear != NULL)
            && (p_sContext.sSPICfg.fp_u32SPI_Transfer != NULL)
            && (p_sContext.sSPICfg.u32ChipSelect != UINT32_MAX) )
         {
            g_sBme280Context.sSPICfg.fp_vPinSet = p_sContext.sSPICfg.fp_vPinSet;
            g_sBme280Context.sSPICfg.fp_vPinClear = p_sContext.sSPICfg.fp_vPinClear;
            g_sBme280Context.sSPICfg.fp_u32SPI_Transfer = p_sContext.sSPICfg.fp_u32SPI_Transfer;
            g_sBme280Context.sSPICfg.u32ChipSelect = p_sContext.sSPICfg.u32ChipSelect;
         }
         else
         {
            l_eErrCode = BME280_ERROR_PARAM;
         }
      }
      else
      {
         l_eErrCode = BME280_ERROR_PARAM;
      }
   }

   EXIT_ERROR_CHECK(l_eErrCode);

   l_eErrCode = eBasicInitialization();
   EXIT_ERROR_CHECK(l_eErrCode);

   g_u8BMEInitialized = 1u;

   return l_eErrCode;
}

/**@brief Function to softreset bme.
 * @return Error Code
 */
e_BME280_Error_t eBME280_Reset(void)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;

   if(g_u8BMEInitialized == 1u)
   {
      l_eError = eReset();
   }

   return l_eError;
}

/**@brief Function to get the chip ID.
 * @param[out] p_pu8ChipId
 * @return Error code
 */
e_BME280_Error_t eBME280_ChipIDGet(uint8_t * p_pu8ChipId)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;

   if(g_u8BMEInitialized == 1u)
   {
      if(p_pu8ChipId != NULL)
      {
         l_eError = eReadRegister(INFO_REG_CHIP_ID, p_pu8ChipId, REGISTER_SIZE);
      }
      else
      {
         l_eError = BME280_ERROR_PARAM;
      }
   }

   return l_eError;
}

/**@brief Function to get the calibration data.
 * @param[out] p_psCalibration
 * @return Error code
 */
e_BME280_Error_t eBME280_CalibrationDataGet(s_BME280_Calibration_Data_t * p_psCalibration)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;

   if(g_u8BMEInitialized == 1u)
   {
      if(p_psCalibration != NULL)
      {
         (*p_psCalibration) = g_sCalib;
      }
      else
      {
         l_eError = BME280_ERROR_PARAM;
      }
   }

   return l_eError;
}

/**@brief Function to set Oversampling of Temperature.
 * @param[in] p_eOSR
 * @return Error code
 */
e_BME280_Error_t eBME280_OSRTemperatureSet(e_BME280_Oversampling_t p_eOSR)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;

   l_eError = eWriteBitsReg(CTRL_REG_MEAS, (uint8_t)p_eOSR, CTRL_MEAS_OSR_TEMP_POS, CTRL_MEAS_OSR_TEMP_MSK);
   EXIT_ERROR_CHECK(l_eError);

   /* Store value to global settings */
   g_sBme280Settings.eOSR_T = p_eOSR;

   return l_eError;
}

/**@brief Function to set Oversampling of Pressure.
 * @param[in] p_eOSR
 * @return Error code
 */
e_BME280_Error_t eBME280_OSRPressureSet(e_BME280_Oversampling_t p_eOSR)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;

   l_eError = eWriteBitsReg(CTRL_REG_MEAS, (uint8_t)p_eOSR, CTRL_MEAS_OSR_PRESS_POS, CTRL_MEAS_OSR_PRESS_MSK);
   EXIT_ERROR_CHECK(l_eError);

   /* Store value to global settings */
   g_sBme280Settings.eOSR_P = p_eOSR;

   return l_eError;
}

/**@brief Function to set Oversampling of Humidity.
 * @param[in] p_eOSR
 * @return Error code
 */
e_BME280_Error_t eBME280_OSRHumiditySet(e_BME280_Oversampling_t p_eOSR)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;
   uint8_t l_u8MeasReg = 0u;

   l_eError = eWriteBitsReg(CTRL_REG_HUM, (uint8_t)p_eOSR, CTRL_HUM_OSR_HUM_POS, CTRL_HUM_OSR_HUM_MSK);
   EXIT_ERROR_CHECK(l_eError);

   /* AN : Change to CTRL_REG_HUM register become effective
      after a write operation to CTRL_REG_MEAS register */
   l_eError = eReadRegister(CTRL_REG_MEAS, &l_u8MeasReg, REGISTER_SIZE);
   EXIT_ERROR_CHECK(l_eError);
   l_eError = eWriteRegister(CTRL_REG_MEAS, l_u8MeasReg);
   EXIT_ERROR_CHECK(l_eError);

   /* Store value to global settings */
   g_sBme280Settings.eOSR_H = p_eOSR;

   return l_eError;
}

/**@brief Function to set IIF Filter setting.
 * @param[in] p_eFilter
 * @return Error code
 */
e_BME280_Error_t eBME280_IIRFilterSet(e_BME280_Filter_t p_eFilter)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;

   l_eError = eWriteBitsReg(CFG_REG_CONFIG, (uint8_t)p_eFilter, CFG_FILTER_POS, CFG_FILTER_MSK);
   EXIT_ERROR_CHECK(l_eError);

   /* Store value to global settings */
   g_sBme280Settings.eFilter = p_eFilter;

   return l_eError;
}

/**@brief Function to set Standby Time (for normal mode only).
 * @param[in] p_eStandbyTime
 * @return Error code
 */
e_BME280_Error_t eBME280_StandbyTimeSet(e_BME280_StandByTimer_t p_eStandbyTime)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;

   l_eError = eWriteBitsReg(CFG_REG_CONFIG, (uint8_t)p_eStandbyTime, CFG_T_STANDBY_POS, CFG_T_STANDBY_MSK);
   EXIT_ERROR_CHECK(l_eError);

   g_sBme280Settings.eStandbyTime = p_eStandbyTime;

   return l_eError;
}

/**@brief Function to set Mode (Idle, Forced or Normal).
 * @param[in] p_eMode
 * @return Error code
 */
e_BME280_Error_t eBME280_ModeSet(e_BME280_Mode_t p_eMode)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;

   l_eError = eWriteBitsReg(CTRL_REG_MEAS, (uint8_t)p_eMode, CTRL_MEAS_MODE_POS, CTRL_MEAS_MODE_MSK);
   EXIT_ERROR_CHECK(l_eError);

   g_sBme280Settings.eMode = p_eMode;

   return l_eError;
}

/**@brief Function to get Mode (Idle, Forced or Normal).
 * @param[out] p_peMode
 * @return Error code
 */
e_BME280_Error_t eBME280_ModeGet(e_BME280_Mode_t * p_peMode)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;
   uint8_t l_u8Data = 0u;

   if(g_u8BMEInitialized == 1u)
   {
      if(p_peMode != NULL)
      {
         l_eError = eReadRegister(CTRL_REG_MEAS, &l_u8Data, REGISTER_SIZE);
         EXIT_ERROR_CHECK(l_eError);

         (*p_peMode) = (e_BME280_Mode_t)VALUE_BITS_GET(l_u8Data, CTRL_MEAS_MODE_POS, CTRL_MEAS_MODE_MSK);
      }
      else
      {
         l_eError = BME280_ERROR_PARAM;
      }
   }

   return l_eError;
}

/**@brief Function to Read all data from BME280 according to OSR and Mode.
 * @return Error Code
 */
e_BME280_Error_t eBME280_TPHRead(void)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;
   uint8_t l_au8ADCData[RAW_SENSOR_P_T_H_DATA_LEN] = { 0u };
   uint32_t l_u32WaitTimeMs = 0u;
   uint8_t l_u8Updating = 1u;
   uint8_t l_u8Measuring = 0u;
   uint8_t l_u8Retry = MEAS_RETRY;

   if(g_u8BMEInitialized == 1u)
   {
      if(g_sBme280Settings.eMode == BME280_NORMAL)
      {  /* Check there is no update in progress */
         do {
            l_eError = eIsUpdating(&l_u8Updating);
            EXIT_ERROR_CHECK(l_eError);
            l_u8Retry--;
         } while( (l_u8Updating == 1u) && (l_u8Retry != 0u) );
      }
      else if( (g_sBme280Settings.eMode == BME280_SLEEP)
            || (g_sBme280Settings.eMode == BME280_FORCED) )
      {
         l_u32WaitTimeMs = MEAS_TIME_MS(g_sBme280Settings.eOSR_T,g_sBme280Settings.eOSR_P,g_sBme280Settings.eOSR_H);
         /* Force an update of sensor data register*/
         l_eError = eBME280_ModeSet(BME280_FORCED);
         EXIT_ERROR_CHECK(l_eError);

         /* Check Status Register */
         do {
            /* Wait Acquisition time, according to OSR */
            (*g_sBme280Context.fp_vTimerDelay_ms)(l_u32WaitTimeMs);
            l_eError = eIsMeasuring(&l_u8Measuring);
            EXIT_ERROR_CHECK(l_eError);
            l_u8Retry--;
         } while( (l_u8Measuring == 1u) && (l_u8Retry != 0u) );
         /* If measuring is done correctly */
         if(l_u8Retry != 0u)
         {   /* Check there is no update in progress */
            do {
               (*g_sBme280Context.fp_vTimerDelay_ms)(DELAY_NVM);
               l_eError = eIsUpdating(&l_u8Updating);
               EXIT_ERROR_CHECK(l_eError);
               l_u8Retry--;
            } while( (l_u8Updating == 1u) && (l_u8Retry != 0u) );
         }
      }
      else
      {
         l_eError = BME280_ERROR_INVALID;
         EXIT_ERROR_CHECK(l_eError);
      }

      if(l_u8Retry != 0u)
      {         
         l_eError = eReadRegister(DATA_REG_PRESS_TEMP_HUM, l_au8ADCData, RAW_SENSOR_P_T_H_DATA_LEN);
         EXIT_ERROR_CHECK(l_eError);

         g_sADCReg.u32RegPress  = ((uint32_t)l_au8ADCData[0u] << 12u);
         g_sADCReg.u32RegPress |= ((uint32_t)l_au8ADCData[1u] << 4u);
         g_sADCReg.u32RegPress |= ((uint32_t)l_au8ADCData[2u] >> 4u);

         g_sADCReg.u32RegTemp  = ((uint32_t)l_au8ADCData[3u] << 12u);
         g_sADCReg.u32RegTemp |= ((uint32_t)l_au8ADCData[4u] << 4u);
         g_sADCReg.u32RegTemp |= ((uint32_t)l_au8ADCData[5u] >> 4u);

         g_sADCReg.u32RegHum = ((uint32_t)l_au8ADCData[6u] << 8u) + (uint32_t)l_au8ADCData[7u];
         l_eError = BME280_ERROR_NONE;
      }
      else
      {
         l_eError = BME280_ERROR_BUSY;
      }
   }

   return l_eError;
}

/**@brief Function to get last raw temperature from BME280.
 * @param[out] p_pu32RawTemperature Pointer on temperature to return.
 * @return Error Code
 */
e_BME280_Error_t eBME280_RawTemperatureGet(uint32_t * p_pu32RawTemperature)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;

   if(p_pu32RawTemperature != NULL)
   {
      (*p_pu32RawTemperature) = g_sADCReg.u32RegTemp;
      l_eError = BME280_ERROR_NONE;
   }

   return l_eError;
}

/**@brief Function to get last raw pressure from BME280.
 * @param[out] p_pu32RawPressure Pointer on pressure to return.
 * @return Error Code
 */
e_BME280_Error_t eBME280_RawPressureGet(uint32_t * p_pu32RawPressure)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;

   if(p_pu32RawPressure != NULL)
   {
      (*p_pu32RawPressure) = g_sADCReg.u32RegPress;
      l_eError = BME280_ERROR_NONE;
   }

   return l_eError;
}

/**@brief Function to get last raw humidity from BME280.
 * @param[out] p_pu16RawHumidity Pointer on humidity to return.
 * @return Error Code
 */
e_BME280_Error_t eBME280_RawHumidityGet(uint16_t * p_pu16RawHumidity)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;

   if(p_pu16RawHumidity != NULL)
   {
      (*p_pu16RawHumidity) = g_sADCReg.u32RegHum;
      l_eError = BME280_ERROR_NONE;
   }

   return l_eError;
}

/**@brief Function to get temperature compensated.
 * @param[out] p_pf32Temperature
 * @return Error code
 */
e_BME280_Error_t eBME280_TemperatureGet(float * p_pf32Temperature)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;
   int32_t l_s32CompensatedValue = 0;

   if(p_pf32Temperature != NULL)
   {
      if(g_sADCReg.u32RegTemp != INVALID_UNCOMP_TEMPERATURE)
      {
         l_eError = eCompensate_T((int32_t) g_sADCReg.u32RegTemp, &l_s32CompensatedValue);
         EXIT_ERROR_CHECK(l_eError);

         (*p_pf32Temperature) = (float)((float)l_s32CompensatedValue / RESOLUTION_T);
         
         /* Overflow check */
         (*p_pf32Temperature) = (*p_pf32Temperature) > MAX_TEMPERATURE_DEG ? MAX_TEMPERATURE_DEG:(*p_pf32Temperature);
         (*p_pf32Temperature) = (*p_pf32Temperature) < MIN_TEMPERATURE_DEG ? MIN_TEMPERATURE_DEG:(*p_pf32Temperature);
         
         l_eError = BME280_ERROR_NONE;
      }
      else
      {
         l_eError = BME280_ERROR_INVALID;
      }
   }

   return l_eError;
}

/**@brief Function to get pressure compensated.
 * @param[out] p_pf32Pressure
 * @return Error code
 */
e_BME280_Error_t eBME280_PressureGet(float * p_pf32Pressure)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;
   uint32_t l_u32CompensatedValue = 0u;

   if(p_pf32Pressure != NULL)
   {
      if(g_sADCReg.u32RegPress != INVALID_UNCOMP_PRESSURE)
      {
         l_eError = eCompensate_P((int32_t) g_sADCReg.u32RegPress, &l_u32CompensatedValue);
         EXIT_ERROR_CHECK(l_eError);

         (*p_pf32Pressure) = (float)((float)l_u32CompensatedValue / RESOLUTION_P);
         
         /* Overflow check */
         (*p_pf32Pressure) = (*p_pf32Pressure) > MAX_PRESSURE_HPA ? MAX_PRESSURE_HPA:(*p_pf32Pressure);
         (*p_pf32Pressure) = (*p_pf32Pressure) < MIN_PRESSURE_HPA ? MIN_PRESSURE_HPA:(*p_pf32Pressure);
         
         l_eError = BME280_ERROR_NONE;
      }
      else
      {
         l_eError = BME280_ERROR_INVALID;
      }
   }

   return l_eError;
}

/**@brief Function to get humidity compensated.
 * @param[out] p_pf32Humidity
 * @return Error code
 */
e_BME280_Error_t eBME280_HumidityGet(float * p_pf32Humidity)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;
   uint32_t l_u32CompensatedValue = 0u;

   if(p_pf32Humidity != NULL)
   {
      if(g_sADCReg.u32RegHum != INVALID_UNCOMP_HUMIDITY)
      {
         l_eError = eCompensate_H((int32_t) g_sADCReg.u32RegHum, &l_u32CompensatedValue);
         EXIT_ERROR_CHECK(l_eError);

         (*p_pf32Humidity) = (float)((float)l_u32CompensatedValue / RESOLUTION_H);
         
         /* Overflow check */
         (*p_pf32Humidity) = (*p_pf32Humidity) > MAX_HUMIDITY_PERCENT ? MAX_HUMIDITY_PERCENT:(*p_pf32Humidity);
         (*p_pf32Humidity) = (*p_pf32Humidity) < MIN_HUMIDITY_PERCENT ? MIN_HUMIDITY_PERCENT:(*p_pf32Humidity);
         
         l_eError = BME280_ERROR_NONE;
      }
      else
      {
         l_eError = BME280_ERROR_INVALID;
      }
   }

   return l_eError;
}

/**@brief Function to check if Sensor is available.
 * @return 1 if available else 0
 */
uint8_t u8BME280_IsAvailable(void)
{
   return ((g_u8BMEInitialized == 1u) && (g_u8BMECommFailure == 1u))?0u:1u;
}

/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief Function to read on registers.
 * @param[in] p_u8Register
 * @param[in] p_u8RegNumber
 * @param[out] p_pu8Value
 * @return Error code
 */
static e_BME280_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber)
{
   e_BME280_Error_t l_eErrCode = BME280_ERROR_COMM;

   if(   (p_pu8Value != NULL)
      || ((p_u8Register + p_u8RegNumber) <= LAST_REGISTER) )
   {
      if(g_sBme280Context.eInterface == BME280_I2C_ITF)
      {
         if((*g_sBme280Context.sI2CCfg.fp_u32I2C_Read)(g_sBme280Context.sI2CCfg.eI2CAddr, &p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
         {
            l_eErrCode = BME280_ERROR_NONE;
            g_u8BMECommFailure = 0u;
         }
         else
         {
            g_u8BMECommFailure = 1u;
         }
      }
      else if(g_sBme280Context.eInterface == BME280_SPI_ITF)
      {
         (*g_sBme280Context.sSPICfg.fp_vPinClear)(g_sBme280Context.sSPICfg.u32ChipSelect);

         if((*g_sBme280Context.sSPICfg.fp_u32SPI_Transfer)(&p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
         {
            l_eErrCode = BME280_ERROR_NONE;
            g_u8BMECommFailure = 0u;
         }
         else
         {
            g_u8BMECommFailure = 1u;
         }

         (*g_sBme280Context.sSPICfg.fp_vPinSet)(g_sBme280Context.sSPICfg.u32ChipSelect);
      }
      else
      {
         l_eErrCode = BME280_ERROR_CONTEXT;
      }
   }
   else
   {
      l_eErrCode = BME280_ERROR_PARAM;
   }

   return l_eErrCode;
}
/**@brief Function to write on register.
 * @param[in] p_u8Register
 * @param[in] p_u8Value
 * @return Error code
 */
static e_BME280_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Value)
{
   e_BME280_Error_t l_eErrCode = BME280_ERROR_COMM;
   uint8_t l_au8Data[2u] = { 0u };
   uint8_t l_u8Ack = 0u;

   if(p_u8Register < LAST_REGISTER)
   {
      l_au8Data[0u] = p_u8Register;
      l_au8Data[1u] = p_u8Value;

      if(g_sBme280Context.eInterface == BME280_I2C_ITF)
      {
         if((*g_sBme280Context.sI2CCfg.fp_u32I2C_Write)((uint8_t)g_sBme280Context.sI2CCfg.eI2CAddr, l_au8Data, 2u) == 0u)
         {
            l_eErrCode = BME280_ERROR_NONE;
            g_u8BMECommFailure = 0u;
         }
         else
         {
            g_u8BMECommFailure = 1u;
         }
      }
      else if(g_sBme280Context.eInterface == BME280_SPI_ITF)
      {
         (*g_sBme280Context.sSPICfg.fp_vPinClear)(g_sBme280Context.sSPICfg.u32ChipSelect);

         if((*g_sBme280Context.sSPICfg.fp_u32SPI_Transfer)(l_au8Data, 2u, &l_u8Ack, 1u) == 0u)
         {
            (void)l_u8Ack;
            l_eErrCode = BME280_ERROR_NONE;
            g_u8BMECommFailure = 0u;
         }
         else
         {
            g_u8BMECommFailure = 1u;
         }

         (*g_sBme280Context.sSPICfg.fp_vPinSet)(g_sBme280Context.sSPICfg.u32ChipSelect);
      }
      else
      {
         l_eErrCode = BME280_ERROR_CONTEXT;
      }
   }
   else
   {
      l_eErrCode = BME280_ERROR_PARAM;
   }

   return l_eErrCode;
}
/**@brief Function to write specific bits on register.
 * @param[in] p_u8Register
 * @param[in] p_u8Value
 * @param[in] p_u8Pos
 * @param[in] p_u8Mask
 * @return Error code
 */
static e_BME280_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask)
{
   e_BME280_Error_t l_eErrCode = BME280_ERROR_PARAM;
   uint8_t l_u8RegVal = 0u;

   l_eErrCode = eReadRegister(p_u8Register, &l_u8RegVal, REGISTER_SIZE);

   if(l_eErrCode == BME280_ERROR_NONE)
   {
      l_u8RegVal &= ~p_u8Mask;

      l_u8RegVal |= ((uint8_t)p_u8Value << p_u8Pos);

      l_eErrCode = eWriteRegister(p_u8Register, l_u8RegVal);
   }

   return l_eErrCode;
}

/**@brief Function to basic intialize bme.
 * @return Error Code
 */
static e_BME280_Error_t eBasicInitialization(void)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;
   uint8_t l_u8IsUpdating = 1u;
   uint8_t l_u8Retry = INIT_RETRY;

   /* Control Chip ID */
   l_eError = eChipIDValidation();
   EXIT_ERROR_CHECK(l_eError);

   /* Soft Reset the BME to be in a known state */
   l_eError = eReset();
   EXIT_ERROR_CHECK(l_eError);
   
   do {
      l_eError = eIsUpdating(&l_u8IsUpdating);
      EXIT_ERROR_CHECK(l_eError);
      (*g_sBme280Context.fp_vTimerDelay_ms)(DELAY_INIT);
      l_u8Retry--;
   }while( (l_u8Retry != 0u) || (l_u8IsUpdating == 1u) );

   /* Get Calibration Data */
   l_eError = eCalibrationDataRead();
   EXIT_ERROR_CHECK(l_eError);

   return l_eError;
}


/**@brief Function to reset chip.
 * @return Error Code
 */
static e_BME280_Error_t eReset(void)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;
   
   l_eError = eWriteRegister(RESET_REG, RESET_CMD_SOFTRESET);

   (*g_sBme280Context.fp_vTimerDelay_ms)(DELAY_WAKEUP);

   return l_eError;
}
/**@brief Function to check chip id.
 * @return Error Code
 */
static e_BME280_Error_t eChipIDValidation(void)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;
   uint8_t l_u8ChipIDValue = 0u;

   l_eError = eReadRegister(INFO_REG_CHIP_ID, &l_u8ChipIDValue, REGISTER_SIZE);
   EXIT_ERROR_CHECK(l_eError);

   if(l_u8ChipIDValue == BME280_CHIP_ID)
   {
      l_eError = BME280_ERROR_NONE;
   }
   else
   {
      l_eError = BME280_ERROR_INVALID;
   }

   return l_eError;
}
/**@brief Function to read all calibration data of bme.
 * @return Error Code
 */
static e_BME280_Error_t eCalibrationDataRead(void)
{
   e_BME280_Error_t l_eError = BME280_ERROR_COMM;
   uint8_t l_au8CalibData[CALIB_TEMP_PRESS_DATA_LEN] = { 0u };

   l_eError = eReadRegister(CALIB_REG_TEMP_PRESS_DATA, l_au8CalibData, CALIB_TEMP_PRESS_DATA_LEN);
   EXIT_ERROR_CHECK(l_eError);

   g_sCalib.u16Dig_T1 = (uint16_t) ((uint16_t)l_au8CalibData[1u] << 8u) + ((uint16_t)l_au8CalibData[0u]);
   g_sCalib.s16Dig_T2 = (int16_t) ((uint16_t)l_au8CalibData[3u] << 8u) + ((uint16_t)l_au8CalibData[2u]);
   g_sCalib.s16Dig_T3 = (int16_t) ((uint16_t)l_au8CalibData[5u] << 8u) + ((uint16_t)l_au8CalibData[4u]);
   g_sCalib.u16Dig_P1 = (uint16_t) ((uint16_t)l_au8CalibData[7u] << 8u) + ((uint16_t)l_au8CalibData[6u]);
   g_sCalib.s16Dig_P2 = (int16_t) ((uint16_t)l_au8CalibData[9u] << 8u) + ((uint16_t)l_au8CalibData[8u]);
   g_sCalib.s16Dig_P3 = (int16_t) ((uint16_t)l_au8CalibData[11u] << 8u) + ((uint16_t)l_au8CalibData[10u]);
   g_sCalib.s16Dig_P4 = (int16_t) ((uint16_t)l_au8CalibData[13u] << 8u) + ((uint16_t)l_au8CalibData[12u]);
   g_sCalib.s16Dig_P5 = (int16_t) ((uint16_t)l_au8CalibData[15u] << 8u) + ((uint16_t)l_au8CalibData[14u]);
   g_sCalib.s16Dig_P6 = (int16_t) ((uint16_t)l_au8CalibData[17u] << 8u) + ((uint16_t)l_au8CalibData[16u]);
   g_sCalib.s16Dig_P7 = (int16_t) ((uint16_t)l_au8CalibData[19u] << 8u) + ((uint16_t)l_au8CalibData[18u]);
   g_sCalib.s16Dig_P8 = (int16_t) ((uint16_t)l_au8CalibData[21u] << 8u) + ((uint16_t)l_au8CalibData[20u]);
   g_sCalib.s16Dig_P9 = (int16_t) ((uint16_t)l_au8CalibData[23u] << 8u) + ((uint16_t)l_au8CalibData[22u]);

   l_eError = eReadRegister(CALIB_REG_HUM_DATA_H1, l_au8CalibData, CALIB_H1_DATA_LEN);
   EXIT_ERROR_CHECK(l_eError);

   g_sCalib.u8Dig_H1  = l_au8CalibData[0u];

   l_eError = eReadRegister(CALIB_REG_HUMIDITY_DATA, l_au8CalibData, CALIB_H2_TO_H6_DATA_LEN);
   EXIT_ERROR_CHECK(l_eError);

   g_sCalib.s16Dig_H2 = (int16_t) ((uint16_t)l_au8CalibData[1u] << 8u) + ((uint16_t)l_au8CalibData[0u]);
   g_sCalib.u8Dig_H3  = l_au8CalibData[2u];
   g_sCalib.s16Dig_H4 = (int16_t)((l_au8CalibData[3u] << 4u) + (l_au8CalibData[4u] & 0x0F));
   g_sCalib.s16Dig_H5 = (int16_t)((l_au8CalibData[5u] << 4u) + (l_au8CalibData[4u] >> 4u));
   g_sCalib.s8Dig_H6  = (int8_t)l_au8CalibData[6u];

   return l_eError;
}

static e_BME280_Error_t eIsUpdating(uint8_t * p_pu8Updating)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;
   uint8_t l_u8StatusReg = 0u;

   if(p_pu8Updating != NULL)
   {
      l_eError = eReadRegister(STATUS_REG, &l_u8StatusReg, REGISTER_SIZE);
      EXIT_ERROR_CHECK(l_eError);

      (*p_pu8Updating) = (uint8_t)VALUE_BITS_GET(l_u8StatusReg, STATUS_NVM_INIT_POS, STATUS_NVM_INIT_MSK);
   }

   return l_eError;
}

static e_BME280_Error_t eIsMeasuring(uint8_t * p_pu8IsMeasuring)
{
   e_BME280_Error_t l_eError = BME280_ERROR_PARAM;
   uint8_t l_u8StatusReg = 0u;

   if(p_pu8IsMeasuring != NULL)
   {
      l_eError = eReadRegister(STATUS_REG, &l_u8StatusReg, REGISTER_SIZE);
      EXIT_ERROR_CHECK(l_eError);

      (*p_pu8IsMeasuring) = (uint8_t)VALUE_BITS_GET(l_u8StatusReg, STATUS_MEASURING_POS, STATUS_MEASURING_MSK);
   }


   return l_eError;
}

/**@brief Compute the real value of BME adc register.
 * @param[in] p_s32UncompPress Uncompensated value of pressure
 * @param[out] p_pu32CompPress Compensated value of pressure
 * @return Error Code
 */
static e_BME280_Error_t eCompensate_P(int32_t p_s32UncompPress, uint32_t * p_pu32CompPress)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;
   int32_t l_s32Var1 = 0;
   int32_t l_s32Var2 = 0;
	s_BME280_Calibration_Data_t l_sCal = { 0 };

   if(p_s32UncompPress == INVALID_UNCOMP_PRESSURE)
   {
      l_eError = BME280_ERROR_INVALID;
   }
   else if(p_pu32CompPress != NULL)
   {
   	l_sCal = g_sCalib;

   	l_s32Var1 = ( ( (int32_t) l_sCal.s32TFine) >> 1 ) - ( (int32_t)64000 );
   	l_s32Var2 = (((l_s32Var1>>2) * (l_s32Var1 >> 2))  >>  11 ) * ((int32_t)l_sCal.s16Dig_P6);
   	l_s32Var2 = l_s32Var2 + ((l_s32Var1*((int32_t)l_sCal.s16Dig_P5))<<1);
   	l_s32Var2 = (l_s32Var2 >> 2)+(((int32_t)l_sCal.s16Dig_P4)<<16);
   	l_s32Var1 = (((l_sCal.s16Dig_P3 * (((l_s32Var1 >> 2) * (l_s32Var1 >> 2))  >>  13 ))  >>  3) + ((((int32_t)l_sCal.s16Dig_P2) * l_s32Var1) >> 1)) >> 18;
   	l_s32Var1 =((((32768+l_s32Var1))*((int32_t)l_sCal.u16Dig_P1)) >> 15);
   	if (l_s32Var1 == 0)
   	{ /* Avoid exception caused by division by zero */
         l_eError = BME280_ERROR_INVALID;
   	}
      else
      {
      	(*p_pu32CompPress) = (((uint32_t)(((int32_t)1048576)-p_s32UncompPress)-(l_s32Var2 >> 12)))*3125;
      	if ((*p_pu32CompPress) < 0x80000000)
      	{
      		(*p_pu32CompPress) = ((*p_pu32CompPress) << 1) / ((uint32_t)l_s32Var1);
      	}
      	else
      	{
      		(*p_pu32CompPress) = ((*p_pu32CompPress) / (uint32_t)l_s32Var1) * 2;
      	}
      	l_s32Var1 = (((int32_t)l_sCal.s16Dig_P9) * ((int32_t)((((*p_pu32CompPress) >> 3) * ((*p_pu32CompPress) >> 3)) >> 13))) >> 12;
      	l_s32Var2 = (((int32_t)((*p_pu32CompPress) >> 2)) * ((int32_t)l_sCal.s16Dig_P8)) >> 13;
      	(*p_pu32CompPress) = (uint32_t)((int32_t)(*p_pu32CompPress) + ((l_s32Var1 + l_s32Var2 + l_sCal.s16Dig_P7) >> 4));
         l_eError = BME280_ERROR_NONE;
      }
   }
   else
   {
      l_eError = BME280_ERROR_PARAM;
   }

   return l_eError;
}

/**@brief Compute the real value of BME adc register.
 * @param[in] p_s32UncompHum Uncompensated value of Humidity
 * @param[out] p_pu32CompHum Compensated value of Humidity
 * @return Error Code
 */

static e_BME280_Error_t eCompensate_H(int32_t p_s32UncompHum, uint32_t * p_pu32CompHum)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;
   int32_t l_s32Var = 0;
	s_BME280_Calibration_Data_t l_sCal = { 0 };

   if(p_s32UncompHum == INVALID_UNCOMP_HUMIDITY)
   {
      l_eError = BME280_ERROR_INVALID;
   }
   else if(p_pu32CompHum != NULL)
   {
      l_sCal = g_sCalib;

      l_s32Var = (l_sCal.s32TFine - ((int32_t)76800));
      l_s32Var = (((((p_s32UncompHum << 14) - (((int32_t)l_sCal.s16Dig_H4) << 20) - (((int32_t)l_sCal.s16Dig_H5) * l_s32Var)) +
               ((int32_t)16384)) >> 15) * (((((((l_s32Var * ((int32_t)l_sCal.s8Dig_H6)) >> 10) * (((l_s32Var * ((int32_t)l_sCal.u8Dig_H3)) >> 11) +
               ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)l_sCal.s16Dig_H2) + 8192) >> 14));

      l_s32Var = (l_s32Var - (((((l_s32Var >> 15) * (l_s32Var >> 15)) >> 7) * ((int32_t)l_sCal.u8Dig_H1)) >> 4));
      l_s32Var = (l_s32Var < 0 ? 0 : l_s32Var);
      l_s32Var = (l_s32Var > 419430400 ? 419430400 : l_s32Var);

      (*p_pu32CompHum) = (uint32_t)(l_s32Var >> 12);
      l_eError = BME280_ERROR_NONE;
   }
   else
   {
      l_eError = BME280_ERROR_PARAM;
   }

   return l_eError;
}

/**@brief Compute the real value of BME adc register.
 * @param[in] p_s32UncompTemp Uncompensated value of Temperature
 * @param[out] p_s32CompTemp Compensated value of Temperature
 * @return Error Code
 */
static e_BME280_Error_t eCompensate_T(int32_t p_s32UncompTemp, int32_t * p_s32CompTemp)
{
   e_BME280_Error_t l_eError = BME280_ERROR_CONTEXT;
   int32_t l_s32Var1 = 0;
   int32_t l_s32Var2 = 0;
	s_BME280_Calibration_Data_t l_sCal = { 0 };

   if(p_s32UncompTemp == INVALID_UNCOMP_TEMPERATURE)
   {
      l_eError = BME280_ERROR_INVALID;
   }
   else if(p_s32CompTemp != NULL)
   {
      l_sCal = g_sCalib;

      l_s32Var1 = ((((p_s32UncompTemp>>3) - ((int32_t)l_sCal.u16Dig_T1<<1))) *
                  ((int32_t)l_sCal.s16Dig_T2)) >> 11;
      l_s32Var2 = (((((p_s32UncompTemp>>4) - ((int32_t)l_sCal.u16Dig_T1)) *
                  ((p_s32UncompTemp>>4) - ((int32_t)l_sCal.u16Dig_T1))) >> 12) *
                  ((int32_t)l_sCal.s16Dig_T3)) >> 14;

      l_sCal.s32TFine = l_s32Var1 + l_s32Var2;

      g_sCalib.s32TFine = l_sCal.s32TFine;

      (*p_s32CompTemp) = (l_sCal.s32TFine * 5 + 128) >> 8;
      l_eError = BME280_ERROR_NONE;
   }
   else
   {
      l_eError = BME280_ERROR_PARAM;
   }

  return l_eError;
}

/************************************************************************
 * End Of File
 ************************************************************************/


