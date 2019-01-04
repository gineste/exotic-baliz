/*
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          16/11/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of Brigthness sensor MAX44009.
 *
 */

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "MAX44009.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
/* Register Map */
#define INT_STATUS		(uint8_t)0x00
#define INT_ENABLE		(uint8_t)0x01
#define CONFIG_REG		(uint8_t)0x02
#define HIGH_BYTE			(uint8_t)0x03
#define LOW_BYTE			(uint8_t)0x04
#define THR_HIGH		   (uint8_t)0x05
#define THR_LOW		   (uint8_t)0x06
#define THR_TIMER	      (uint8_t)0x07

#define LAST_REGISTER   (THR_TIMER + 1u)

#define W_REG_SIZE      ((uint8_t)2u)
#define R_LUXREG_SIZE   ((uint8_t)2u)

#define MIN_THR_TIMER   (uint16_t)100u
#define MAX_THR_TIMER   (uint16_t)25500u

/***** Mask and Pos *****/
/* STATUS */
#define INT_STATUS_REG_INTS_POS  (uint8_t)0
#define INT_STATUS_REG_INTS_MSK  (uint8_t)0x01
#define INT_ENABLE_REG_INTE_POS  (uint8_t)0
#define INT_ENABLE_REG_INTE_MSK  (uint8_t)0x01

/* CONFIGURATION */
#define CONFIG_REG_TIM_POS       (uint8_t)0
#define CONFIG_REG_TIM_MSK       (uint8_t)0x07
#define CONFIG_REG_CDR_POS       (uint8_t)3
#define CONFIG_REG_CDR_MSK       (uint8_t)0x08
#define CONFIG_REG_MANUAL_POS    (uint8_t)6
#define CONFIG_REG_MANUAL_MSK    (uint8_t)0x40
#define CONFIG_REG_CONT_POS      (uint8_t)7
#define CONFIG_REG_CONT_MSK      (uint8_t)0x80

/* LUX READING */
#define LUX_READ_HIGH_MANT_POS   (uint8_t)0
#define LUX_READ_HIGH_MANT_MSK   (uint8_t)0x0F
#define LUX_READ_HIGH_EXP_POS    (uint8_t)4
#define LUX_READ_HIGH_EXP_MSK    (uint8_t)0xF0
#define LUX_READ_LOW_MANT_POS    (uint8_t)0
#define LUX_READ_LOW_MANT_MSK    (uint8_t)0x0F

/* THRESHOLD SET */
#define THR_SET_HIGH_MANT_POS    (uint8_t)0
#define THR_SET_HIGH_MANT_MSK    (uint8_t)0x0F
#define THR_SET_HIGH_EXP_POS     (uint8_t)4
#define THR_SET_HIGH_EXP_MSK     (uint8_t)0xF0
#define THR_SET_LOW_MANT_POS     (uint8_t)0
#define THR_SET_LOW_MANT_MSK     (uint8_t)0x0F
#define THR_SET_LOW_EXP_POS      (uint8_t)4
#define THR_SET_LOW_EXP_MSK      (uint8_t)0xF0
#define THR_SET_TIMER_POS        (uint8_t)0
#define THR_SET_TIMER_MSK        (uint8_t)0xFF

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != MAX44009_ERROR_NONE))  \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef enum _COMM_ERROR_ {
   ERROR_NONE = 0u,
   ERROR_PARAM,
   ERROR_BUSY,
   ERROR_READ,
   ERROR_WRITE,
}e_CommError_t;

typedef struct _MAX44009_REG_ {
   uint8_t u8Exponent;
   uint8_t u8Mantissa;
   uint32_t u32Lux;
}s_MAX44009_Reg_t;

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_MAX44009_Error_t eReadRegisters(uint8_t p_u8Reg, uint8_t * p_pu8Value, uint8_t p_u8RegNumber);
static e_MAX44009_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data);
static e_MAX44009_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask);

static uint8_t u8ThresholdCompute(uint32_t p_u32Threshold);
static uint8_t u8TimerCompute(uint16_t p_u16Timer);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static s_MAX44009_Context_t g_sMax44009Context = {
   .eI2CAddress = MAX44009_ADDR1,      /* Sensor Address */
   .fp_u32I2C_Read = NULL,             /* Function pointer to a read I2C transfer */
   .fp_u32I2C_Write = NULL,            /* Function pointer to a write I2C transfer */
   .fp_vDelay_ms = NULL,               /* Function pointer to a timer in ms */
   };

static uint8_t g_u8IsInitialized = 0u;
static uint8_t g_u8MAXCommFailure = 0u;
static s_MAX44009_Reg_t g_sBrightness = { 0u };

/****************************************************************************************
 * Public functions
 ****************************************************************************************/
/**@brief  Initialise the MAX44009 module.
 * @param[in]  Context.
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_ContextSet(s_MAX44009_Context_t p_sContext)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_CONTEXT;

   /* Check function pointers - callbacks */
   if(   (p_sContext.fp_u32I2C_Write != NULL)
      && (p_sContext.fp_u32I2C_Read != NULL)
      && (p_sContext.fp_vDelay_ms != NULL) )
   {
      /* Store new context */
      g_sMax44009Context = p_sContext;

      g_u8IsInitialized = 1u;
      l_eError = MAX44009_ERROR_NONE;
   }

   return l_eError;
}

/**@brief  Initialize acquisition and conversion of data.
 * @param[in]  p_eConversion.
 * @param[in]  p_eAcquisitionMode.
 * @param[in]  p_eIntegration. Not used if Acquisition Mode is Automatic
 * @param[in]  p_eBrightness. Not used if Acquisition Mode is Automatic
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_ConversionModeSet(e_MAX44009_Cont_t p_eConversion,
                                          e_MAX44009_Mode_t p_eAcquisitionMode,
                                          e_MAX44009_IntegrationTime_t p_eIntegration,
                                          e_MAX44009_BrightessMode_t p_eBrightness)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_INIT;

   if(g_u8IsInitialized == 1u)
   {  /* Acquisition Mode Automatic or Manual */
      l_eError = eWriteBitsReg(CONFIG_REG, p_eAcquisitionMode, CONFIG_REG_MANUAL_POS, CONFIG_REG_MANUAL_MSK);
      EXIT_ERROR_CHECK(l_eError);

      if(p_eAcquisitionMode == MAX44009_MANUAL)
      {  /* Integration Time */
         l_eError = eWriteBitsReg(CONFIG_REG, p_eIntegration, CONFIG_REG_TIM_POS, CONFIG_REG_TIM_MSK);
         EXIT_ERROR_CHECK(l_eError);
         /* Brightness mode CDR */
         l_eError = eWriteBitsReg(CONFIG_REG, p_eBrightness, CONFIG_REG_CDR_POS, CONFIG_REG_CDR_MSK);
         EXIT_ERROR_CHECK(l_eError);
      }
      /* Continuous mode */
      l_eError = eWriteBitsReg(CONFIG_REG, p_eConversion, CONFIG_REG_CONT_POS, CONFIG_REG_CONT_MSK);
      EXIT_ERROR_CHECK(l_eError);
   }

   return l_eError;
}

/**@brief  Read Lux register of the MAX44009 module.
 * @param[out] p_pau8RawData: must be 2 uint8, LSB is Mantissa, MSB is Exponent.
 *                            Could be NULL if not needed.
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_BrightnessRead(uint8_t * p_pau8RawData)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_INIT;
   uint8_t l_au8ReadReg[2u] = { 0u };

   if(g_u8IsInitialized == 1u)
   {
      /* Read High Byte */
      l_eError = eReadRegisters(HIGH_BYTE, &l_au8ReadReg[0u], 1u);
      EXIT_ERROR_CHECK(l_eError);
      /* Read Low Byte */
      l_eError = eReadRegisters(LOW_BYTE, &l_au8ReadReg[1u], 1u);
      EXIT_ERROR_CHECK(l_eError);

      /* Upper four bits of High_Byte register are for Exponent */
      g_sBrightness.u8Exponent = (l_au8ReadReg[0u] & 0xF0) >> 4u;
      /* Lower four bits of High_Byte register  and
         lower four bits of Low_Byte register are for Mantissa */
      g_sBrightness.u8Mantissa = ((l_au8ReadReg[0u] & 0x0F) << 4u) | (l_au8ReadReg[1u] & 0x0F);

      g_sBrightness.u32Lux = (uint32_t)g_sBrightness.u8Mantissa << (uint32_t)g_sBrightness.u8Exponent;
      g_sBrightness.u32Lux = (float)g_sBrightness.u32Lux * MAX44009_COEF_MANTISSA;

      if(p_pau8RawData != NULL)
      {
         p_pau8RawData[0u] = g_sBrightness.u8Mantissa;
         p_pau8RawData[1u] = g_sBrightness.u8Exponent;
      }

      l_eError = MAX44009_ERROR_NONE;
   }

   return l_eError;
}

/**@brief  Get Brightness of the MAX44009 module.
 * @param[out]  p_pu32Brightness : Brightness.
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_BrightnessGet(uint32_t * p_pu32Brightness)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_INIT;

   if(g_u8IsInitialized == 1u)
   {
      if(p_pu32Brightness != NULL)
      {
         (*p_pu32Brightness) = g_sBrightness.u32Lux;
         l_eError = MAX44009_ERROR_NONE;
      }
      else
      {
         l_eError = MAX44009_ERROR_PARAM;
      }
   }

   return l_eError;
}

/**@brief  Configure Interrupt on the MAX44009 module.
 * @param[in]  p_u8Activate : Activate or Not Interrupt.
 * @param[in]  p_u32HighThr : High threshold detection.
 * @param[in]  p_u32LowThr : Low threshold detection.
 * @param[in]  p_u16Timer : Timer Threshold.
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_InterruptCfg(uint8_t p_u8Activate, uint32_t p_u32HighThr, uint32_t p_u32LowThr, uint16_t p_u16Timer)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_INIT;
   uint8_t l_u8Register = 0u;

   if(g_u8IsInitialized == 1u)
   {
      if(p_u8Activate == 1u)
      {  /* Check parameters */
         if(   (p_u32HighThr > p_u32LowThr)
            && (p_u16Timer > MIN_THR_TIMER)
            && (p_u16Timer < MAX_THR_TIMER))
         {
            /* Compute High Threshold Register */
            l_u8Register = u8ThresholdCompute(p_u32HighThr);
            l_eError = eWriteRegister(THR_HIGH, l_u8Register);
            EXIT_ERROR_CHECK(l_eError);

            /* Compute Low Threshold Register */
            l_u8Register = u8ThresholdCompute(p_u32LowThr);
            l_eError = eWriteRegister(THR_LOW, l_u8Register);
            EXIT_ERROR_CHECK(l_eError);

            /* Compute Timer Threshold Register */
            l_u8Register = u8TimerCompute(p_u16Timer);
            l_eError = eWriteRegister(THR_TIMER, l_u8Register);
            EXIT_ERROR_CHECK(l_eError);
         }
         else
         {
            l_eError = MAX44009_ERROR_PARAM;
         }
      }

      /* Enable or disable Interrupt */
      l_eError = eWriteRegister(INT_ENABLE, (p_u8Activate&0x01));
      EXIT_ERROR_CHECK(l_eError);
   }

   return l_eError;
}

/**@brief  Get Interrupt status of the MAX44009 module.
 * @param[out]  p_pu8IntStatus : 1 if interrupt is active else 0.
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_InterruptStatusGet(uint8_t * p_pu8IntStatus)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_INIT;

   if(g_u8IsInitialized == 1u)
   {
      if(p_pu8IntStatus != NULL)
      {
         l_eError = eReadRegisters(INT_STATUS, p_pu8IntStatus, 1u);
         EXIT_ERROR_CHECK(l_eError);
      }
      else
      {
         l_eError = MAX44009_ERROR_PARAM;
      }
   }

   return l_eError;
}

/**@brief  Get High/Low Threshold value of the MAX44009 module.
 * @param[out]  p_pu32ThrHigh
 * @param[out]  p_pu32ThrLow
 * @param[out]  p_pu16ThrTimer
 * @return Error Code.
 */
e_MAX44009_Error_t eMAX44009_ThresholdGet(uint32_t * p_pu32ThrHigh, uint32_t * p_pu32ThrLow, uint16_t * p_pu16ThrTimer)
{
   e_MAX44009_Error_t l_eError = MAX44009_ERROR_INIT;
   uint8_t l_u8Thr = 0u;
   uint8_t l_u8Exp = 0u;
   uint32_t l_u32Mantissa = 0u;
   
   if(g_u8IsInitialized == 1u)
   {
      if( (p_pu32ThrHigh != NULL) && (p_pu32ThrLow != NULL) && (p_pu16ThrTimer != NULL) )
      {
         l_eError = eReadRegisters(THR_LOW, &l_u8Thr, 1u);
         EXIT_ERROR_CHECK(l_eError);
         l_u8Exp = (l_u8Thr & 0xF0) >> 4u;
         l_u32Mantissa = ((l_u8Thr & 0x0F) << 4u) + 0x0F;
         l_u32Mantissa <<= l_u8Exp;
         (*p_pu32ThrLow) = (uint32_t)((float)l_u32Mantissa * MAX44009_COEF_MANTISSA);
         
         l_eError = eReadRegisters(THR_HIGH, &l_u8Thr, 1u);
         EXIT_ERROR_CHECK(l_eError);
         l_u8Exp = (l_u8Thr & 0xF0) >> 4u;
         l_u32Mantissa = ((l_u8Thr & 0x0F) << 4u) + 0x0F;
         l_u32Mantissa <<= l_u8Exp;
         (*p_pu32ThrHigh) = (uint32_t)((float)l_u32Mantissa * MAX44009_COEF_MANTISSA);
         
         l_eError = eReadRegisters(THR_TIMER, &l_u8Thr, 1u);
         EXIT_ERROR_CHECK(l_eError);
         (*p_pu16ThrTimer) = (((uint16_t)l_u8Thr) * MIN_THR_TIMER);
      }
      else
      {
         l_eError = MAX44009_ERROR_PARAM;
      }
   }

   return l_eError;
}
/**@brief  Check if MAX44009 module is still available.
 * @return 1 if available, else 0u.
 */
uint8_t u8MAX44009_IsAvailable(void)
{
   return ((g_u8IsInitialized == 1u) && (g_u8MAXCommFailure == 0u))?1u:0u;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to read on bme registers.
 * @param[in] p_u8Reg
 * @param[in] p_u8RegNumber
 * @param[out] p_pu8Value
 * @return Error code
 */
static e_MAX44009_Error_t eReadRegisters(uint8_t p_u8Reg, uint8_t * p_pu8Value, uint8_t p_u8RegNumber)
{
   e_MAX44009_Error_t l_eErrCode = MAX44009_ERROR_COMM;

   if(   (p_pu8Value != NULL)
      || ((p_u8Reg+p_u8RegNumber) <= LAST_REGISTER) )
   {
      if((*g_sMax44009Context.fp_u32I2C_Read)((uint8_t)g_sMax44009Context.eI2CAddress, &p_u8Reg, 1u, p_pu8Value, p_u8RegNumber) == 0u)
      {
         l_eErrCode = MAX44009_ERROR_NONE;
         g_u8MAXCommFailure = 0u;
      }
      else
      {
         g_u8MAXCommFailure = 1u;
      }
   }
   else
   {
      l_eErrCode = MAX44009_ERROR_PARAM;
   }

   return l_eErrCode;
}

/**@brief Function to write on bme register.
 * @param[in] p_u8Reg
 * @param[in] p_u8Data
 * @return Error code
 */
static e_MAX44009_Error_t eWriteRegister(uint8_t p_u8Reg, uint8_t p_u8Data)
{
   e_MAX44009_Error_t l_eErrCode = MAX44009_ERROR_COMM;
   uint8_t l_au8Data[2u] = { 0u };

   if(p_u8Reg < LAST_REGISTER)
   {
      l_au8Data[0u] = p_u8Reg;
      l_au8Data[1u] = p_u8Data;

      if((*g_sMax44009Context.fp_u32I2C_Write)((uint8_t)g_sMax44009Context.eI2CAddress, l_au8Data, W_REG_SIZE) == 0u)
      {
         l_eErrCode = MAX44009_ERROR_NONE;
         g_u8MAXCommFailure = 0u;
      }
      else
      {
         g_u8MAXCommFailure = 1u;
      }
   }
   else
   {
      l_eErrCode = MAX44009_ERROR_PARAM;
   }

   return l_eErrCode;
}

static e_MAX44009_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask)
{
   e_MAX44009_Error_t l_eErrCode = MAX44009_ERROR_PARAM;
   uint8_t l_u8RegVal = 0u;

   l_eErrCode = eReadRegisters(p_u8Register, &l_u8RegVal, 1u);

   if(l_eErrCode == MAX44009_ERROR_NONE)
   {
      l_u8RegVal &= ~p_u8Mask;
      l_u8RegVal |= ((uint8_t)p_u8Value << p_u8Pos);

      l_eErrCode = eWriteRegister(p_u8Register, l_u8RegVal);
   }

   return l_eErrCode;
}

/**@brief Function to compute Threshold for register.
 * @param[in] p_u32Threshold in Lux
 * @return u8 Threshold register to write.
 */
static uint8_t u8ThresholdCompute(uint32_t p_u32Threshold)
{
   uint8_t l_u8ThrRegister = 0u;
   uint32_t l_u32Mantissa = 0u;
   uint8_t l_u8Exp = 0u;

   l_u32Mantissa = (uint32_t)((float)p_u32Threshold) / MAX44009_COEF_MANTISSA;

   while(l_u32Mantissa > UINT8_MAX)
   {
      l_u32Mantissa >>= 1u;
      l_u8Exp++;
   }

   l_u32Mantissa = (l_u32Mantissa >> 4u) & 0x0F;
   l_u8Exp <<= 4u;

   l_u8ThrRegister = l_u32Mantissa | l_u8Exp;

   return l_u8ThrRegister;
}

/**@brief Function to compute Timer for register.
 * @param[in] p_u16Timer in ms
 * @return u8 Timer register to write.
 */
static uint8_t u8TimerCompute(uint16_t p_u16Timer)
{
   uint8_t l_u8TimerRegister = 0u;
   uint16_t l_u16Timer = 0u;

   /* Floar conversion */
   l_u16Timer = (p_u16Timer + 50u) / 100u;

   l_u8TimerRegister = (uint8_t)(l_u16Timer > UINT8_MAX)?UINT8_MAX:l_u16Timer;

   return l_u8TimerRegister;
}


/****************************************************************************************
 * End Of File
 ****************************************************************************************/


