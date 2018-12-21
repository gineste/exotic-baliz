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
 * Date:          08/06/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of magnetic sensor LIS2MDL. 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

/* Self include */
#include "LIS2MDL.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define LIS2MDL_I2C_ADDR     (uint8_t)0x1E
 
/* Register MAP */
/* Hard-iron registers */
#define OFFSET_X_REG_L        (uint8_t)0x45
#define OFFSET_X_REG_H        (uint8_t)0x46
#define OFFSET_Y_REG_L        (uint8_t)0x47
#define OFFSET_Y_REG_H        (uint8_t)0x48
#define OFFSET_Z_REG_L        (uint8_t)0x49
#define OFFSET_Z_REG_H        (uint8_t)0x4A
/* Chip ID */
#define WHO_AM_I              (uint8_t)0x4F
/* Configuration registers */
#define CFG_REG_A             (uint8_t)0x60
#define CFG_REG_B             (uint8_t)0x61
#define CFG_REG_C             (uint8_t)0x62
/* Interrupt configuration registers */
#define INT_CTRL_REG          (uint8_t)0x63
#define INT_SOURCE_REG        (uint8_t)0x64
#define INT_THS_L_REG         (uint8_t)0x65
#define INT_THS_H_REG         (uint8_t)0x66
/* Status register */
#define STATUS_REG            (uint8_t)0x67
/* Output registers */
#define OUT_X_L_REG           (uint8_t)0x68
#define OUT_X_H_REG           (uint8_t)0x69
#define OUT_Y_L_REG           (uint8_t)0x6A
#define OUT_Y_H_REG           (uint8_t)0x6B
#define OUT_Z_L_REG           (uint8_t)0x6C
#define OUT_Z_H_REG           (uint8_t)0x6D
/* Temperature sensor registers */
#define TEMP_OUT_L_REG        (uint8_t)0x6E
#define TEMP_OUT_H_REG        (uint8_t)0x6F

#define LAST_REGISTER         (uint8_t)(TEMP_OUT_H_REG + 1u)

#define WHO_I_AM_ID           (uint8_t)0x40 /*> 01000000 */
#define REBOOT_TIMEOUT        (uint32_t)20
#define DELAY_DRDY            (uint32_t)5

/***** Mask and Pos *****/
/* CFG REG A */
#define CFG_REG_A_MODE_POS    (uint8_t)0
#define CFG_REG_A_MODE_MSK    (uint8_t)0x02
#define CFG_REG_A_ODR_POS     (uint8_t)2
#define CFG_REG_A_ODR_MSK     (uint8_t)0x0C
#define CFG_REG_A_LP_POS      (uint8_t)4
#define CFG_REG_A_LP_MSK      (uint8_t)0x10
#define CFG_REG_A_SOFTRST_POS (uint8_t)5
#define CFG_REG_A_SOFTRST_MSK (uint8_t)0x20
#define CFG_REG_A_REBOOT_POS  (uint8_t)6
#define CFG_REG_A_REBOOT_MSK  (uint8_t)0x40
#define CFG_REG_A_TCOMP_POS   (uint8_t)7
#define CFG_REG_A_TCOMP_MSK   (uint8_t)0x80

/* CFG REG B */
#define CFG_REG_B_LPF_POS           (uint8_t)0   
#define CFG_REG_B_LPF_MSK           (uint8_t)0x01
#define CFG_REG_B_OFF_CAN_POS       (uint8_t)1   
#define CFG_REG_B_OFF_CAN_MSK       (uint8_t)0x02
#define CFG_REG_B_SFREQ_POS         (uint8_t)2   
#define CFG_REG_B_SFREQ_MSK         (uint8_t)0x04
#define CFG_REG_B_HARDIC_POS        (uint8_t)3   
#define CFG_REG_B_HARDIC_MSK        (uint8_t)0x08
#define CFG_REG_B_OFF_CAN_SIN_POS   (uint8_t)4   
#define CFG_REG_B_OFF_CAN_SIN_MSK   (uint8_t)0x10

/* CFG REG C */
#define CFG_REG_C_DRDYPIN_POS    (uint8_t)0
#define CFG_REG_C_DRDYPIN_MSK    (uint8_t)0x01
#define CFG_REG_C_SELFTEST_POS   (uint8_t)1
#define CFG_REG_C_SELFTEST_MSK   (uint8_t)0x02
#define CFG_REG_C_BLE_POS        (uint8_t)3
#define CFG_REG_C_BLE_MSK        (uint8_t)0x08
#define CFG_REG_C_BDU_POS        (uint8_t)4
#define CFG_REG_C_BDU_MSK        (uint8_t)0x10
#define CFG_REG_C_I2CDIS_POS     (uint8_t)5
#define CFG_REG_C_I2CDIS_MSK     (uint8_t)0x20
#define CFG_REG_C_INTPIN_POS     (uint8_t)6
#define CFG_REG_C_INTPIN_MSK     (uint8_t)0x40

/* INT CTRL REG*/
#define INT_CTRL_REG_EN_POS      (uint8_t)0
#define INT_CTRL_REG_EN_MSK      (uint8_t)0x01
#define INT_CTRL_REG_LATCH_POS   (uint8_t)1
#define INT_CTRL_REG_LATCH_MSK   (uint8_t)0x02
#define INT_CTRL_REG_POL_POS     (uint8_t)2
#define INT_CTRL_REG_POL_MSK     (uint8_t)0x04
#define INT_CTRL_REG_XYZEN_POS   (uint8_t)5
#define INT_CTRL_REG_XYZEN_MSK   (uint8_t)0x20

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != LIS2MDL_ERROR_NONE))   \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_LIS2MDL_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber);
static e_LIS2MDL_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data);  
static e_LIS2MDL_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static s_LIS2MDL_Context_t g_sLIS2MDLContext;
static e_LIS2MDL_Mode_t g_eReadMode = LIS2MDL_MODE_IDLE_DEFAULT;
static uint8_t g_u8LIS2Initialized = 0u;
static uint8_t g_u8LIS2CommFailure = 0u;
                     
#ifndef DEBUG
static 
#endif
uint16_t g_au16RawMag[3u] = { 0u };
#ifndef DEBUG
static 
#endif
uint16_t g_u16RawTemp = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Initialize the LIS2MDL module.
 * @param[in]  p_sContext.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_ContextSet(s_LIS2MDL_Context_t p_sContext)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_PARAM;

   if(   (p_sContext.eCommunicationUsed == LIS2MDL_COMM_I2C)
      && (p_sContext.sI2CCfg.fp_u32I2C_Write != NULL) 
      && (p_sContext.sI2CCfg.fp_u32I2C_Read != NULL) 
      && (p_sContext.fp_vDelay_ms != NULL) )
   {
      g_sLIS2MDLContext.sI2CCfg.fp_u32I2C_Write = p_sContext.sI2CCfg.fp_u32I2C_Write;
      g_sLIS2MDLContext.sI2CCfg.fp_u32I2C_Read = p_sContext.sI2CCfg.fp_u32I2C_Read;
      g_sLIS2MDLContext.fp_vDelay_ms = p_sContext.fp_vDelay_ms;
      l_eErrCode = LIS2MDL_ERROR_NONE;
      g_u8LIS2Initialized = 1u;
   }
   else if(    (p_sContext.eCommunicationUsed == LIS2MDL_COMM_SPI)
            && (p_sContext.sSPICfg.fp_u32SPI_Transfer != NULL) 
            && (p_sContext.sSPICfg.fp_vPinSet != NULL)
            && (p_sContext.sSPICfg.fp_vPinClear != NULL)
            && (p_sContext.fp_vDelay_ms != NULL) )
   {
      g_sLIS2MDLContext.sSPICfg.fp_u32SPI_Transfer = p_sContext.sSPICfg.fp_u32SPI_Transfer;
      g_sLIS2MDLContext.sSPICfg.fp_vPinSet = p_sContext.sSPICfg.fp_vPinSet;
      g_sLIS2MDLContext.sSPICfg.fp_vPinClear = p_sContext.sSPICfg.fp_vPinClear;
      g_sLIS2MDLContext.sSPICfg.u32ChipSelect = p_sContext.sSPICfg.u32ChipSelect;
      g_sLIS2MDLContext.fp_vDelay_ms = p_sContext.fp_vDelay_ms;
      l_eErrCode = LIS2MDL_ERROR_NONE;
      g_u8LIS2Initialized = 1u;
   }
   else
   {
      l_eErrCode = LIS2MDL_ERROR_PARAM;
      g_u8LIS2Initialized = 0u;
   }
   
   return l_eErrCode;
}

/**@brief Get Who Am I Register.
 * @param[in]  p_pu8WhoAmI : Who Am I value. (must be LIS2MDL_WHO_AM_I_ID)
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_WhoAmIGet(uint8_t * p_pu8WhoAmI)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_COMM;

   if((g_u8LIS2Initialized != 1u) || (p_pu8WhoAmI == NULL))
   {
      l_eErrCode = LIS2MDL_ERROR_PARAM;
   } 
   else
   {
      l_eErrCode = eReadRegister(WHO_AM_I, p_pu8WhoAmI, 1u);
   }

   return l_eErrCode;
}

/**@brief Reboot sensor.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_Reboot(void)
{
   return eWriteBitsReg(CFG_REG_A, 1u, CFG_REG_A_REBOOT_POS, CFG_REG_A_REBOOT_MSK);
}
/**@brief Soft reset configuration.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_SoftReset(void)
{
   return eWriteBitsReg(CFG_REG_A, 1u, CFG_REG_A_SOFTRST_POS, CFG_REG_A_SOFTRST_MSK);
}
/**@brief Activate Low Power Mode.
 * @param[in]  p_u8Activate : 1 to activate low power else normal mode.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_LowPower(uint8_t p_u8Activate)
{
   return eWriteBitsReg(CFG_REG_A, p_u8Activate, CFG_REG_A_LP_POS, CFG_REG_A_LP_MSK);
}

/**@brief Set new ODR.
 * @param[in]  p_eODR : Output Data Rate.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_OutputDataRateSet(e_LIS2MDL_ODR_t p_eODR)
{
   return eWriteBitsReg(CFG_REG_A, (uint8_t)p_eODR, CFG_REG_A_ODR_POS, CFG_REG_A_ODR_MSK);
}

/**@brief Set new working mode.
 * @param[in]  p_eReadMode : Could be Idle, Single Shot or Continuous.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_ModeSet(e_LIS2MDL_Mode_t p_eReadMode)
{
   g_eReadMode = p_eReadMode;
   return eWriteBitsReg(CFG_REG_A, (uint8_t)p_eReadMode, CFG_REG_A_MODE_POS, CFG_REG_A_MODE_MSK);
}

/**@brief Set BLE.
 * @param[in]  p_u8LittleEndian : 1 to active LE else BE.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_BigLittleEndian(uint8_t p_u8LittleEndian)
{
   return eWriteBitsReg(CFG_REG_C, (uint8_t)p_u8LittleEndian, CFG_REG_C_BLE_POS, CFG_REG_C_BLE_MSK);
}

/**@brief Self Test.
 * @param[in]  p_u8Activate : 1 to active
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_SelfTest(uint8_t p_u8Activate)
{
   return eWriteBitsReg(CFG_REG_C, p_u8Activate, CFG_REG_C_SELFTEST_POS, CFG_REG_C_SELFTEST_MSK);
}

/**@brief Set Offset.
 * @param[in]  p_s16OffsetX
 * @param[in]  p_s16OffsetY
 * @param[in]  p_s16OffsetZ
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_OffsetSet(int16_t p_s16OffsetX, int16_t p_s16OffsetY, int16_t p_s16OffsetZ)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_NONE;

   /* Write X offset */
   l_eErrCode = eWriteRegister(OUT_X_L_REG, (uint8_t)((uint16_t)p_s16OffsetX & 0x00FF));
   EXIT_ERROR_CHECK(l_eErrCode);
   l_eErrCode = eWriteRegister(OUT_X_H_REG, (uint8_t)(((uint16_t)p_s16OffsetX & 0xFF00) >> 8u));
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Write Y offset */
   l_eErrCode = eWriteRegister(OUT_Y_L_REG, (uint8_t)((uint16_t)p_s16OffsetY & 0x00FF));
   EXIT_ERROR_CHECK(l_eErrCode);
   l_eErrCode = eWriteRegister(OUT_Y_H_REG, (uint8_t)(((uint16_t)p_s16OffsetY & 0xFF00) >> 8u));
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Write Z offset */
   l_eErrCode = eWriteRegister(OUT_Z_L_REG, (uint8_t)((uint16_t)p_s16OffsetZ & 0x00FF));
   EXIT_ERROR_CHECK(l_eErrCode);
   l_eErrCode = eWriteRegister(OUT_Z_H_REG, (uint8_t)(((uint16_t)p_s16OffsetZ & 0xFF00) >> 8u));
   EXIT_ERROR_CHECK(l_eErrCode);

   return l_eErrCode;
}

/**@brief Activate Low Pass Filter.
 * @param[in]  p_u8Activate : 1 to active
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_LowPassFilter(uint8_t p_u8Activate)
{
   return eWriteBitsReg(CFG_REG_B, p_u8Activate, CFG_REG_B_LPF_POS, CFG_REG_B_LPF_MSK);
}

/**@brief Enable Temperature Compensation on Magnetic value.
 * @param[in]  p_u8Activate : 1 to active
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_TemperatureCompensation(uint8_t p_u8Activate)
{
   return eWriteBitsReg(CFG_REG_A, p_u8Activate, CFG_REG_A_TCOMP_POS, CFG_REG_A_TCOMP_MSK);
}

/**@brief Perform a read operation of magnetic data.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_MagneticRead(void)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_NOT_READY;
   
   uint8_t l_au8Data[6u] = { 0u };
   uint8_t l_u8DataRdy = 0u;
   uint8_t l_u8Retry = 3u;
      
   if(g_u8LIS2Initialized == 1u)
   {
      if(   (g_eReadMode == LIS2MDL_MODE_SINGLE_SHOT)
         || (g_eReadMode == LIS2MDL_MODE_IDLE)
         || (g_eReadMode == LIS2MDL_MODE_IDLE_DEFAULT) )
      {
         (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
      }
         
      eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
      l_u8DataRdy = (l_au8Data[0u] & 0x08);
      while((l_u8DataRdy == 0u) && (l_u8Retry > 0u))
      {
         (*g_sLIS2MDLContext.fp_vDelay_ms)(DELAY_DRDY);
         eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
         l_u8DataRdy = (l_au8Data[0u] & 0x08);
         l_u8Retry--;
      }
   
      if(eReadRegister(OUT_X_L_REG, l_au8Data, 6u) == LIS2MDL_ERROR_NONE)
      {
         g_au16RawMag[0u] = l_au8Data[0u] + ((uint16_t)l_au8Data[1u] << 8u);
         g_au16RawMag[1u] = l_au8Data[2u] + ((uint16_t)l_au8Data[3u] << 8u);
         g_au16RawMag[2u] = l_au8Data[4u] + ((uint16_t)l_au8Data[5u] << 8u);
         l_eErrCode = LIS2MDL_ERROR_NONE;
      }
   }
      
   return l_eErrCode;
}

/**@brief Perform a read operation of temperature data.
 * @return Error Code.
 */
e_LIS2MDL_Error_t eLIS2MDL_TemperatureRead(void)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_NOT_READY;
   
   uint8_t l_au8Data[2u] = { 0u };
   uint8_t l_u8DataRdy = 0u;
   uint8_t l_u8Retry = 3u;
      
   if(g_u8LIS2Initialized == 1u)
   {
      if(   (g_eReadMode == LIS2MDL_MODE_SINGLE_SHOT)
         || (g_eReadMode == LIS2MDL_MODE_IDLE)
         || (g_eReadMode == LIS2MDL_MODE_IDLE_DEFAULT) )
      {
         (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
      }
         
      eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
      l_u8DataRdy = (l_au8Data[0u] & 0x08);
      while((l_u8DataRdy == 0u) && (l_u8Retry > 0u))
      {
         (*g_sLIS2MDLContext.fp_vDelay_ms)(DELAY_DRDY);
         eReadRegister(STATUS_REG, &l_au8Data[0u], 1u);
         l_u8DataRdy = (l_au8Data[0u] & 0x08);
         l_u8Retry--;
      }
   
      if(l_u8Retry != 0u)
      {
         if(eReadRegister(TEMP_OUT_L_REG, l_au8Data, 2u) == LIS2MDL_ERROR_NONE)
         {
            g_u16RawTemp = l_au8Data[0u] + ((uint16_t)l_au8Data[1u] << 8u);
            l_eErrCode = LIS2MDL_ERROR_NONE;
         }
      }
   }

   return l_eErrCode;
}

/**@brief Configure threshold register.
 * @param[in]  p_u8Activate : 1 to activate Interrupt on pin.
 * @param[in]  p_eIntAxis : X, Y, Z, XY, XZ, YZ, XYZ.
 * @param[in]  p_u8Polarity : 0 or 1.
 * @param[in]  p_u8Latched : Interrupt Latched or pulsed.
 * @return Error code.
 */
e_LIS2MDL_Error_t eLIS2MDL_InterruptCtrlSet(uint8_t p_u8Activate, e_LIS2MDL_InterruptAxis_t p_eIntAxis, uint8_t p_u8Polarity, uint8_t p_u8Latched)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_PARAM;
   uint8_t l_u8RegVal = 0u;

   /* IEN value */
   l_u8RegVal &= ~INT_CTRL_REG_EN_MSK;          
   l_u8RegVal |= ((uint8_t)p_u8Activate << INT_CTRL_REG_EN_POS);
   
   /* XYZEN value */
   l_u8RegVal &= ~INT_CTRL_REG_XYZEN_MSK;          
   l_u8RegVal |= ((uint8_t)p_eIntAxis << INT_CTRL_REG_XYZEN_POS);
   
   /* IEA value */
   l_u8RegVal &= ~INT_CTRL_REG_POL_MSK;          
   l_u8RegVal |= ((uint8_t)p_u8Polarity << INT_CTRL_REG_POL_POS);
   
   /* IEL value */
   l_u8RegVal &= ~INT_CTRL_REG_LATCH_MSK;          
   l_u8RegVal |= ((uint8_t)p_u8Latched << INT_CTRL_REG_LATCH_POS);
      
   l_eErrCode = eWriteRegister(INT_CTRL_REG, l_u8RegVal);
   
   (void)eWriteRegister(INT_CTRL_REG, l_u8RegVal);
   
   return l_eErrCode;
}

/**@brief Set threshold.
 * @param[in]  p_u16TresholdmG : threshold in mG.
 * @return Error code.
 */
e_LIS2MDL_Error_t eLIS2MDL_ThresholdSet(uint16_t p_u16TresholdmG)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_PARAM;
   
   l_eErrCode = eWriteRegister(OUT_X_L_REG, (uint8_t)((uint16_t)p_u16TresholdmG & 0x00FF));
   EXIT_ERROR_CHECK(l_eErrCode);
   l_eErrCode = eWriteRegister(OUT_X_H_REG, (uint8_t)(((uint16_t)p_u16TresholdmG & 0xFF00) >> 8u));
   EXIT_ERROR_CHECK(l_eErrCode);
   
   return l_eErrCode;
}

/**@brief Get new data value.
 * @param[out]  p_ps16X : Pointer where the data is returned X.
 * @param[out]  p_ps16Y : Pointer where the data is returned Y.
 * @param[out]  p_ps16Z : Pointer where the data is returned Z.
 * @return Error code.
 */
e_LIS2MDL_Error_t eLIS2MDL_MagDataGet(int16_t * p_ps16X, int16_t * p_ps16Y, int16_t * p_ps16Z)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_PARAM;
   static const float l_fSensitivity = 1.5f;
   int32_t l_s32MagX = (int16_t)g_au16RawMag[0u];
   int32_t l_s32MagY = (int16_t)g_au16RawMag[1u];
   int32_t l_s32MagZ = (int16_t)g_au16RawMag[2u];
   
   if( (p_ps16X != NULL) && (p_ps16Y != NULL) && (p_ps16Z != NULL) )
   {
      l_s32MagX = (float)l_s32MagX * l_fSensitivity;
      (*p_ps16X) = (int16_t)((l_s32MagX > INT16_MAX)?INT16_MAX:(l_s32MagX < INT16_MIN)?INT16_MIN:l_s32MagX);
      
      l_s32MagY = (float)l_s32MagY * l_fSensitivity;
      (*p_ps16Y) = (int16_t)((l_s32MagY > INT16_MAX)?INT16_MAX:(l_s32MagY < INT16_MIN)?INT16_MIN:l_s32MagY);
      
      l_s32MagZ = (float)l_s32MagZ * l_fSensitivity;
      (*p_ps16Z) = (int16_t)((l_s32MagZ > INT16_MAX)?INT16_MAX:(l_s32MagZ < INT16_MIN)?INT16_MIN:l_s32MagZ);

      l_eErrCode = LIS2MDL_ERROR_NONE;
   }

   return l_eErrCode;
}

/**@brief Get new temperature data value.
 * @param[out]  p_ps16Temp : Pointer where the data is returned.
 * @return Error code.
 */
e_LIS2MDL_Error_t eLIS2MDL_TempDataGet(int16_t * p_ps16Temp)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_PARAM;
   static const uint8_t l_u8LSBPerDegree = 3u;
   int32_t l_s32Temp = 0;

   if(p_ps16Temp != NULL)
   {
      l_s32Temp = (int32_t)(((int16_t)g_u16RawTemp)>> l_u8LSBPerDegree);
      l_s32Temp = (int16_t)((l_s32Temp > INT16_MAX)?INT16_MAX:l_s32Temp);
      l_s32Temp = (int16_t)((l_s32Temp < INT16_MIN)?INT16_MIN:l_s32Temp);

      (*p_ps16Temp) = l_s32Temp;
      l_eErrCode = LIS2MDL_ERROR_NONE;
   }

   return l_eErrCode;
}

/**@brief Check if device is available.
 * @return 1 if available else 0.
 */
uint8_t u8LIS2MDL_IsAvailable(void)
{
   return ((g_u8LIS2Initialized == 1u) && (g_u8LIS2CommFailure == 0u))?1u:0u;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to read on registers.
 * @param[in] p_u8Reg
 * @param[in] p_u8RegNumber
 * @param[out] p_pu8Value
 * @return Error code
 */
static e_LIS2MDL_Error_t eReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Value, uint8_t p_u8RegNumber)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_COMM;
   
   if(   (p_pu8Value != NULL) 
      || ((p_u8Register + p_u8RegNumber) <= LAST_REGISTER) )
   {
      if(g_sLIS2MDLContext.eCommunicationUsed == LIS2MDL_COMM_I2C)
      {
         if((*g_sLIS2MDLContext.sI2CCfg.fp_u32I2C_Read)(LIS2MDL_I2C_ADDR, &p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
         {
            l_eErrCode = LIS2MDL_ERROR_NONE;
            g_u8LIS2CommFailure = 0u;
         }
         else
         {
            g_u8LIS2CommFailure = 1u;
         }
      }
      else if(g_sLIS2MDLContext.eCommunicationUsed == LIS2MDL_COMM_SPI)
      {         
         (*g_sLIS2MDLContext.sSPICfg.fp_vPinClear)(g_sLIS2MDLContext.sSPICfg.u32ChipSelect);

         if((*g_sLIS2MDLContext.sSPICfg.fp_u32SPI_Transfer)(&p_u8Register, 1u, p_pu8Value, p_u8RegNumber) == 0u)
         {
            l_eErrCode = LIS2MDL_ERROR_NONE;
            g_u8LIS2CommFailure = 0u;
         }
         else
         {
            g_u8LIS2CommFailure = 1u;
         }

         (*g_sLIS2MDLContext.sSPICfg.fp_vPinSet)(g_sLIS2MDLContext.sSPICfg.u32ChipSelect);
      }
      else
      {
         l_eErrCode = LIS2MDL_ERROR_CONTEXT;
      }
   }
   else
   {
      l_eErrCode = LIS2MDL_ERROR_PARAM;
   }

	return l_eErrCode;   
}

/**@brief Function to write on register.
 * @param[in] p_u8Reg
 * @param[in] p_u8Data
 * @return Error code
 */
static e_LIS2MDL_Error_t eWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data)
{
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_COMM;
   uint8_t l_au8Data[2u] = { 0u };
   uint8_t l_u8Ack = 0u;

   if(p_u8Register < LAST_REGISTER)
   {
      l_au8Data[0u] = p_u8Register;
      l_au8Data[1u] = p_u8Data;
      
      if(g_sLIS2MDLContext.eCommunicationUsed == LIS2MDL_COMM_I2C)
      {
         if((*g_sLIS2MDLContext.sI2CCfg.fp_u32I2C_Write)(LIS2MDL_I2C_ADDR, l_au8Data, 2u) == 0u)
         {
            l_eErrCode = LIS2MDL_ERROR_NONE;
            g_u8LIS2CommFailure = 0u;
         }
         else
         {
            g_u8LIS2CommFailure = 1u;
         }
      }
      else if(g_sLIS2MDLContext.eCommunicationUsed == LIS2MDL_COMM_SPI)
      {         
         (*g_sLIS2MDLContext.sSPICfg.fp_vPinClear)(g_sLIS2MDLContext.sSPICfg.u32ChipSelect);

         if((*g_sLIS2MDLContext.sSPICfg.fp_u32SPI_Transfer)(l_au8Data, 2u, &l_u8Ack, 1u) == 0u)
         {
            (void)l_u8Ack;
            l_eErrCode = LIS2MDL_ERROR_NONE;
            g_u8LIS2CommFailure = 0u;
         }
         else
         {
            g_u8LIS2CommFailure = 1u;
         }
         
         (*g_sLIS2MDLContext.sSPICfg.fp_vPinSet)(g_sLIS2MDLContext.sSPICfg.u32ChipSelect);
      }
      else
      {
         l_eErrCode = LIS2MDL_ERROR_CONTEXT;
      }
   }
   else
   {
      l_eErrCode = LIS2MDL_ERROR_PARAM;
   }
   
   return l_eErrCode;
}

static e_LIS2MDL_Error_t eWriteBitsReg(uint8_t p_u8Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask)
{
   uint8_t l_u8RegVal = 0u;
   e_LIS2MDL_Error_t l_eErrCode = LIS2MDL_ERROR_PARAM;
   
   l_eErrCode = eReadRegister(p_u8Register, &l_u8RegVal, 1u); 

   if(l_eErrCode == LIS2MDL_ERROR_NONE)
   {
      l_u8RegVal &= ~p_u8Mask;
          
      l_u8RegVal |= ((uint8_t)p_u8Value << p_u8Pos);

      l_eErrCode = eWriteRegister(p_u8Register, l_u8RegVal);
   }

   return l_eErrCode;
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
