/*
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          15/11/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of Battery Gas Gauge LTC2943-1.
 *
 */

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

/* Self include */
#include "LTC2943.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define LTC2943_1_SLAVE_ADDR        (uint8_t)0x64

/*********************/
/*   REGISTER MAP    */
/*********************/
#define STATUS_REG                  (uint8_t)0x00
#define CONTROL_REG                 (uint8_t)0x01
#define ACCU_CHR_MSB_REG            (uint8_t)0x02
#define ACCU_CHR_LSB_REG            (uint8_t)0x03
#define CHR_THR_HIGH_MSB_REG        (uint8_t)0x04
#define CHR_THR_HIGH_LSB_REG        (uint8_t)0x05
#define CHR_THR_LOW_MSB_REG         (uint8_t)0x06
#define CHR_THR_LOW_LSB_REG         (uint8_t)0x07
#define VOLTAGE_MSB_REG             (uint8_t)0x08
#define VOLTAGE_LSB_REG             (uint8_t)0x09
#define VOLTAGE_THR_HIGH_MSB_REG    (uint8_t)0x0A
#define VOLTAGE_THR_HIGH_LSB_REG    (uint8_t)0x0B
#define VOLTAGE_THR_LOW_MSB_REG     (uint8_t)0x0C
#define VOLTAGE_THR_LOW_LSB_REG     (uint8_t)0x0D
#define CURRENT_MSB_REG             (uint8_t)0x0E
#define CURRENT_LSB_REG             (uint8_t)0x0F
#define CURRENT_THR_HIGH_MSB_REG    (uint8_t)0x10
#define CURRENT_THR_HIGH_LSB_REG    (uint8_t)0x11
#define CURRENT_THR_LOW_MSB_REG     (uint8_t)0x12
#define CURRENT_THR_LOW_LSB_REG     (uint8_t)0x13
#define TEMPERATURE_MSB_REG         (uint8_t)0x14
#define TEMPERATURE_LSB_REG         (uint8_t)0x15
#define TEMPERATURE_THR_HIGH_REG    (uint8_t)0x16
#define TEMPERATURE_THR_LOW_REG     (uint8_t)0x17

#define LAST_REGISTER               (uint8_t)(TEMPERATURE_THR_LOW_REG)

/*********************/
/* Mask and Position */
/*********************/
/* STATUS_REG Msk Pos */
#define STATUS_UVLO_POS             (uint8_t)0
#define STATUS_UVLO_MSK             (uint8_t)0x01
#define STATUS_VOLT_ALERT_POS       (uint8_t)1
#define STATUS_VOLT_ALERT_MSK       (uint8_t)0x02
#define STATUS_CHR_ALERT_LOW_POS    (uint8_t)2
#define STATUS_CHR_ALERT_LOW_MSK    (uint8_t)0x04
#define STATUS_CHR_ALERT_HIGH_POS   (uint8_t)3
#define STATUS_CHR_ALERT_HIGH_MSK   (uint8_t)0x08
#define STATUS_TEMP_ALERT_POS       (uint8_t)4
#define STATUS_TEMP_ALERT_MSK       (uint8_t)0x10
#define STATUS_ACR_POS              (uint8_t)5
#define STATUS_ACR_MSK              (uint8_t)0x20
#define STATUS_CURRENT_ALERT_POS    (uint8_t)6
#define STATUS_CURRENT_ALERT_MSK    (uint8_t)0x40
/* CONTROL_REG Msk Pos */
#define CONTROL_SHUTDOWN_POS        (uint8_t)0
#define CONTROL_SHUTDOWN_MSK        (uint8_t)0x01
#define CONTROL_ALCC_CFG_POS        (uint8_t)1
#define CONTROL_ALCC_CFG_MSK        (uint8_t)0x06
#define CONTROL_PRESCALER_M_POS     (uint8_t)3
#define CONTROL_PRESCALER_M_MSK     (uint8_t)0x38
#define CONTROL_ADC_MODE_POS        (uint8_t)6
#define CONTROL_ADC_MODE_MSK        (uint8_t)0xC0

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != LTC2943_ERROR_NONE))   \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);

#define ADC_MEAS_VOLT_TIME_MAX      (uint32_t)48u
#define ADC_MEAS_CURR_TIME_MAX      (uint32_t)8u
#define ADC_MEAS_TEMP_TIME_MAX      (uint32_t)8u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_LTC2943_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber);
static e_LTC2943_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data);
static e_LTC2943_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask);

static e_LTC2943_Error_t eADCReadModeCheck(uint8_t p_u8SensorMask);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static s_LTC2943_Context_t g_sLTC2943Context;
static uint8_t g_u8LTCCommFailure = 0u;
static uint8_t g_u8LTCInitialized = 0u;
static struct {
   uint16_t u16RawTemp;
   uint16_t u16RawVoltage;
   uint16_t u16RawCurrent;
} g_sLTC2943RawData;

static struct {
   uint16_t u16PrescalerM;
   float f32Qlsb;
   e_LTC2943_ADCMode_t eADCMode;
   uint8_t u8Shutdown;
} g_sLTC2943Cfg;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/
/**@brief  Initialize the module.
 * @param[in]  Context.
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_ContextSet(s_LTC2943_Context_t p_sContext)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_PARAM;

   if(   (p_sContext.fp_u32I2C_Write != NULL)
      && (p_sContext.fp_u32I2C_Read != NULL)
      && (p_sContext.fp_vDelay_ms != NULL) )
   {
      /* Set new context */
      g_sLTC2943Context.fp_u32I2C_Write = p_sContext.fp_u32I2C_Write;
      g_sLTC2943Context.fp_u32I2C_Read = p_sContext.fp_u32I2C_Read;
      g_sLTC2943Context.fp_vDelay_ms = p_sContext.fp_vDelay_ms;

      l_eErrCode = LTC2943_ERROR_NONE;
      g_u8LTCInitialized = 1u;
   }

   return l_eErrCode;
}
/**@brief  Shutdown the module.
 * @param[in]  p_u8Shutdown 1 to shutdown, 0 to wakeup
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_PowerDown(uint8_t p_u8Shutdown)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;

   if(g_u8LTCInitialized == 1u)
   {
      l_eErrCode = eWriteBitsReg(CONTROL_REG, p_u8Shutdown, CONTROL_SHUTDOWN_POS, CONTROL_SHUTDOWN_MSK);
   }

   return l_eErrCode;
}

/**@brief  Configure the ADC Mode.
 * @param[in]  p_eADCMode : Sleep/Manual/Scan(every 10sec)/Automatic.
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_ADCModeSet(e_LTC2943_ADCMode_t p_eADCMode)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;

   if(g_u8LTCInitialized == 1u)
   {
      l_eErrCode = eWriteBitsReg(CONTROL_REG, (uint8_t)p_eADCMode, CONTROL_ADC_MODE_POS, CONTROL_ADC_MODE_MSK);
   }

   return l_eErrCode;
}

/**@brief  Configure the Battery Capacity.
 * @param[in]  p_u16BatteryCapacitymAh
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_BatteryCapacitySet(uint16_t p_u16BatteryCapacitymAh)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;
   const uint16_t l_cau16PrescalerM[8u] = { 1u, 4u, 16u, 64u, 256u, 1024u, 4096u, 4096u };
   const uint32_t l_u32Denom = 26214u; // 2^16 * 0.4
   uint32_t l_u32Num = 0u;
   uint32_t l_u32PrescalerValue = 4096u;  // Default Value
   uint8_t l_u8PrescalerIdx = 8u;
   uint8_t l_u8Idx = 0u;

   if(g_u8LTCInitialized == 1u)
   {  /* Prescaler M >= 4096 * Qbat/(2^16 * 0.4mAh)
         Prescaler possible values are : 1, 4, 16, 64, 256, 1024, 4096.
       */
      l_u32Num = 4096u * p_u16BatteryCapacitymAh;
      l_u32PrescalerValue = l_u32Num / l_u32Denom;

      for(l_u8Idx = 0u; l_u8Idx < 8u; l_u8Idx++)
      {
         if(l_u32PrescalerValue > l_cau16PrescalerM[l_u8Idx])
         {
            l_u8PrescalerIdx = l_u8Idx;
         }
      }
      /* Write Prescaler M*/
      l_eErrCode = eWriteBitsReg(CONTROL_REG, (uint8_t)l_u8PrescalerIdx, CONTROL_PRESCALER_M_POS, CONTROL_PRESCALER_M_MSK);
      EXIT_ERROR_CHECK(l_eErrCode);
      /* Store Prescaler to compute data later */
      g_sLTC2943Cfg.u16PrescalerM = l_cau16PrescalerM[l_u8PrescalerIdx];
      g_sLTC2943Cfg.f32Qlsb = (float)((0.4f * (float)g_sLTC2943Cfg.u16PrescalerM) / 4096.0f);
   }

   return l_eErrCode;
}

/**@brief  Configure the Alert Mode of pin.
 * @param[in]  p_eAlert : Enable/Disable/Charge Complete
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_AlertCfgSet(e_LTC2943_AlertCfg_t p_eAlert)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;

   if(g_u8LTCInitialized == 1u)
   {
      l_eErrCode = eWriteBitsReg(CONTROL_REG, (uint8_t)p_eAlert, CONTROL_ALCC_CFG_POS, CONTROL_ALCC_CFG_MSK);
   }

   return l_eErrCode;
}

/**@brief  Configure the threshold High and Low of Temperature.
 * @param[in]  p_u16TemperatureHigh
 * @param[in]  p_u16TemperatureLow
 * @return Error Code.
 */
//e_LTC2943_Error_t eLTC2943_TemperatureThresholdSet(uint16_t p_u16TemperatureHigh, uint16_t p_u16TemperatureLow);
/**@brief  Configure the threshold High and Low of Charge.
 * @param[in]  p_u16ChargeHigh
 * @param[in]  p_u16ChargeLow
 * @return Error Code.
 */
//e_LTC2943_Error_t eLTC2943_ChargeThresholdSet(uint16_t p_u16ChargeHigh, uint16_t p_u16ChargeLow);
/**@brief  Configure the threshold High and Low of voltage.
 * @param[in]  p_u16VoltageHigh
 * @param[in]  p_u16VoltageLow
 * @return Error Code.
 */
//e_LTC2943_Error_t eLTC2943_VoltageThresholdSet(uint16_t p_u16VoltageHigh, uint16_t p_u16VoltageLow);

/**@brief  Read ADC Sensor of selected sensors.
 * @param[in]  p_u8SensorMask
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_SensorRead(uint8_t p_u8SensorMask)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;
   uint8_t l_au8RawData[2u] = { 0u };

   if(g_u8LTCInitialized == 1u)
   {
      l_eErrCode = eADCReadModeCheck(p_u8SensorMask);
      EXIT_ERROR_CHECK(l_eErrCode);

      if( (p_u8SensorMask & ADC_SENSOR_VOLTAGE_MASK) == ADC_SENSOR_VOLTAGE_MASK)
      {
         l_eErrCode = eReadRegister(VOLTAGE_MSB_REG, l_au8RawData, 2u);
         EXIT_ERROR_CHECK(l_eErrCode);
         g_sLTC2943RawData.u16RawVoltage = ((uint16_t)l_au8RawData[0u] << 8u) + l_au8RawData[1u];
      }

      if( (p_u8SensorMask & ADC_SENSOR_CURRENT_MASK) == ADC_SENSOR_CURRENT_MASK)
      {
         l_eErrCode = eReadRegister(CURRENT_MSB_REG, l_au8RawData, 2u);
         EXIT_ERROR_CHECK(l_eErrCode);
         g_sLTC2943RawData.u16RawCurrent = ((uint16_t)l_au8RawData[0u] << 8u) + l_au8RawData[1u];
      }

      if( (p_u8SensorMask & ADC_SENSOR_TEMP_MASK) == ADC_SENSOR_TEMP_MASK)
      {
         l_eErrCode = eReadRegister(TEMPERATURE_MSB_REG, l_au8RawData, 2u);
         EXIT_ERROR_CHECK(l_eErrCode);
         g_sLTC2943RawData.u16RawTemp = ((uint16_t)l_au8RawData[0u] << 8u) + l_au8RawData[1u];
      }
   }

   return l_eErrCode;
}

/**@brief  Get latest value of Temperature in 0.01C°.
 * @param[out]  p_ps16Temperature
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_TemperatureGet(int16_t * p_ps16Temperature)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;
   int32_t l_s32Result = 0;

   if(g_u8LTCInitialized == 1u)
   {
      if(p_ps16Temperature != NULL)
      {
         l_s32Result = (((int32_t)g_sLTC2943RawData.u16RawTemp * 51000) / 0xFFFF);
         l_s32Result -= 27315u;
         (*p_ps16Temperature) = (int16_t)l_s32Result;
         l_eErrCode = LTC2943_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LTC2943_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}

/**@brief  Get latest value of accumulated charge current in mAh.
 * @param[out]  p_ps16AccuCharge
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_AccumulatedChargeGet(int16_t * p_ps16AccuCharge)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;
   int32_t l_s32Result = 0;

   if(g_u8LTCInitialized == 1u)
   {
      if(p_ps16AccuCharge != NULL)
      {
         l_s32Result = (int32_t)((float)(((int32_t)g_sLTC2943RawData.u16RawCurrent - 0x7FFF) / 0x7FFF) * g_sLTC2943Cfg.f32Qlsb);
         (*p_ps16AccuCharge) = (int16_t)l_s32Result;
         l_eErrCode = LTC2943_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LTC2943_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}

/**@brief  Get latest value of voltage in 0.01V.
 * @param[out]  p_pu16Voltage
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_VoltageGet(uint16_t * p_pu16Voltage)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;
   uint16_t l_u16Result = 0u;

   if(g_u8LTCInitialized == 1u)
   {
      if(p_pu16Voltage != NULL)
      {
         l_u16Result = (((uint32_t)g_sLTC2943RawData.u16RawVoltage * 23600) / 65535);
         (*p_pu16Voltage) = l_u16Result;
         l_eErrCode = LTC2943_ERROR_NONE;
      }
      else
      {
         l_eErrCode = LTC2943_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}

/**@brief  Get value of status register.
 * @param[out]  p_pu8Status
 * @return Error Code.
 */
e_LTC2943_Error_t eLTC2943_StatusGet(uint8_t * p_pu8Status)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_CONTEXT;

   if(g_u8LTCInitialized == 1u)
   {
      if(p_pu8Status != NULL)
      {
         l_eErrCode = eReadRegister(STATUS_REG, p_pu8Status, 1u);
      }
      else
      {
         l_eErrCode = LTC2943_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}

/**@brief Function to check if device is available.
 * @return 1 if sensor is available, else 0
 */
uint8_t u8LTC2943_IsAvailable(void)
{
   return ((g_u8LTCInitialized == 1u) && (g_u8LTCCommFailure == 0u))?1u:0u;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to read on registers.
 * @param[in] p_u8Register
 * @param[out] p_pu8Value
 * @param[in] p_u8RegNumber
 * @return Error code
 */
static e_LTC2943_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_COMM;

   if(   (p_pu8Value != NULL)
      || ((p_u8Register + p_u8RegNumber) <= LAST_REGISTER) )
   {
      if((*g_sLTC2943Context.fp_u32I2C_Read)(LTC2943_1_SLAVE_ADDR, &p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
      {
         l_eErrCode = LTC2943_ERROR_NONE;
         g_u8LTCCommFailure = 0u;
      }
      else
      {
         g_u8LTCCommFailure = 1u;
      }
   }
   else
   {
      l_eErrCode = LTC2943_ERROR_PARAM;
   }

   return l_eErrCode;
}
/**@brief Function to write on register.
 * @param[in] p_u8Register
 * @param[in] p_u8Data
 * @return Error code
 */
static e_LTC2943_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_COMM;
   uint8_t l_au8Data[2u] = { 0u };

   if(p_u8Register < LAST_REGISTER)
   {
      l_au8Data[0u] = p_u8Register;
      l_au8Data[1u] = p_u8Data;

      if((*g_sLTC2943Context.fp_u32I2C_Write)(LTC2943_1_SLAVE_ADDR, l_au8Data, 2u) == 0u)
      {
         l_eErrCode = LTC2943_ERROR_NONE;
         g_u8LTCCommFailure = 0u;
      }
      else
      {
         g_u8LTCCommFailure = 1u;
      }
   }
   else
   {
      l_eErrCode = LTC2943_ERROR_PARAM;
   }

   return l_eErrCode;
}
/**@brief Function to write specific bits of one register.
 * @param[in] p_u8Register
 * @param[in] p_u8Data
 * @param[in] p_u8Pos
 * @param[in] p_u8Mask
 * @return Error code
 */
static e_LTC2943_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask)
{
   uint8_t l_u8RegVal = 0u;
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_PARAM;

   l_eErrCode = eReadRegister(p_u8Register, &l_u8RegVal, 1u);

   if(l_eErrCode == LTC2943_ERROR_NONE)
   {
      l_u8RegVal &= ~p_u8Mask;

      l_u8RegVal |= ((uint8_t)p_u8Value << p_u8Pos);

      l_eErrCode = eWriteRegister(p_u8Register, l_u8RegVal);
   }

   return l_eErrCode;
}

static e_LTC2943_Error_t eADCReadModeCheck(uint8_t p_u8SensorMask)
{
   e_LTC2943_Error_t l_eErrCode = LTC2943_ERROR_NONE;
   e_LTC2943_ADCMode_t l_eOldTempADCMode = g_sLTC2943Cfg.eADCMode;

   switch(g_sLTC2943Cfg.eADCMode)
   {
      case LTC2943_ADC_MODE_SLEEP:
      case LTC2943_ADC_MODE_MANUAL:
         l_eErrCode = eLTC2943_ADCModeSet(LTC2943_ADC_MODE_MANUAL);
         g_sLTC2943Cfg.eADCMode = l_eOldTempADCMode;
         EXIT_ERROR_CHECK(l_eErrCode);

         /* Wait until conversion done */
         if( (p_u8SensorMask & ADC_SENSOR_VOLTAGE_MASK) == ADC_SENSOR_VOLTAGE_MASK)
         {
            (*g_sLTC2943Context.fp_vDelay_ms)(ADC_MEAS_VOLT_TIME_MAX);
         }
         else
         {
            (*g_sLTC2943Context.fp_vDelay_ms)(ADC_MEAS_TEMP_TIME_MAX);
         }

         break;
      case LTC2943_ADC_MODE_SCAN:      /* Just get the latest value */
      case LTC2943_ADC_MODE_AUTOMATIC: /* Do nothing more */
      default:
         break;
   }

   return l_eErrCode;
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


