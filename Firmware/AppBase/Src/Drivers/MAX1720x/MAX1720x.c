/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2019 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          03/12/2019 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of Fuel Gauge MAX1720x 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

/* Self include */
#include "MAX1720x.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define MAX1720X_I2C_ADDR_LOW       ((uint8_t)0x6C>>1)
#define MAX1720X_I2C_ADDR_HIGH      ((uint8_t)0x16>>1)

/* Useful registers to read */
#define MAX1720X_REG_STATUS         (uint16_t)0x000
#define MAX1720X_REG_VCELL          (uint16_t)0x009
#define MAX1720X_REG_REPSOC         (uint16_t)0x006
#define MAX1720X_REG_REPCAP         (uint16_t)0x005
#define MAX1720X_REG_TEMP           (uint16_t)0x008
#define MAX1720X_REG_CURRENT        (uint16_t)0x00A
#define MAX1720X_REG_TTE            (uint16_t)0x011
#define MAX1720X_REG_TTF            (uint16_t)0x020
/* Additional registers */
#define MAX1720X_REG_CELL1          (uint16_t)0x0D8
#define MAX1720X_REG_CELL2          (uint16_t)0x0D7
#define MAX1720X_REG_CELL3          (uint16_t)0x0D6
#define MAX1720X_REG_CELLX          (uint16_t)0x0D9
#define MAX1720X_REG_BATT           (uint16_t)0x0DA
#define MAX1720X_REG_TEMP1          (uint16_t)0x134
#define MAX1720X_REG_TEMP2          (uint16_t)0x13B
#define MAX1720X_REG_INT_TEMP       (uint16_t)0x135
#define MAX1720X_REG_AGEFORCAST     (uint16_t)0x0B9
#define MAX1720X_REG_AGE            (uint16_t)0x007
#define MAX1720X_REG_CYCLES         (uint16_t)0x017
#define MAX1720X_REG_ATTTE          (uint16_t)0x0DD
#define MAX1720X_REG_ATRATE         (uint16_t)0x004
#define MAX1720X_REG_FULLCAP        (uint16_t)0x010
#define MAX1720X_REG_TIMERH         (uint16_t)0x0BE
#define MAX1720X_REG_MAXMINVOLT     (uint16_t)0x01B
#define MAX1720X_REG_MAXMINCURRENT  (uint16_t)0x01C
#define MAX1720X_REG_MAXMINTEMP     (uint16_t)0x01A
#define MAX1720X_REG_RCELL          (uint16_t)0x014
#define MAX1720X_REG_AVGVCELL       (uint16_t)0x019
#define MAX1720X_REG_AVGCURR        (uint16_t)0x00B
#define MAX1720X_REG_AVGTA          (uint16_t)0x016
#define MAX1720X_REG_VRIPPLE        (uint16_t)0x0BC


#define MAX1720X_RSENSE_VAL         (float)0.620f

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != MAX1720X_ERROR_NONE))  \
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
static e_MAX1720X_Error_t eReadRegisters(uint16_t p_u16Register, uint16_t * p_pu16Value, uint8_t p_u8RegNumber);
static e_MAX1720X_Error_t eReadRegister(uint16_t p_u16Register, uint16_t * p_pu16Value);
static e_MAX1720X_Error_t eWriteRegister(uint16_t p_u16Register, uint16_t p_u16Data);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
#define START_ADDRESS_EZ_MODEL_CONFIG     0x180
const uint16_t g_cau16EZModel[] = {
   0x00F0,		//nXTable0 Register
   0x0686,		//nXTable1 Register
   0x071E,		//nXTable2 Register
   0x06E2,		//nXTable3 Register
   0x3AED,		//nXTable4 Register
   0x3104,		//nXTable5 Register
   0x1650,		//nXTable6 Register
   0x1659,		//nXTable7 Register
   0x0923,		//nXTable8 Register
   0x0930,		//nXTable9 Register
   0x0816,		//nXTable10 Register
   0x06EE,		//nXTable11 Register
   0x0000,		//nUser18C Register
   0x0000,		//nUser18D Register
   0x0000,		//nODSCTh Register
   0x0000,		//nODSCCfg Register
   0x961B,		//nOCVTable0 Register
   0xAD16,		//nOCVTable1 Register
   0xB2AB,		//nOCVTable2 Register
   0xB8CB,		//nOCVTable3 Register
   0xBADB,		//nOCVTable4 Register
   0xBB28,		//nOCVTable5 Register
   0xBC5C,		//nOCVTable6 Register
   0xBDE2,		//nOCVTable7 Register
   0xBF78,		//nOCVTable8 Register
   0xC59D,		//nOCVTable9 Register
   0xCC08,		//nOCVTable10 Register
   0xD01A,		//nOCVTable11 Register
   0x03C0,		//nIChgTerm Register
   0x0000,		//nFilterCfg Register
   0x0000,		//nVEmpty Register
   0xA602,		//nLearnCfg Register
   0x42B2,		//nQRTable00 Register
   0x3C18,		//nQRTable10 Register
   0x390B,		//nQRTable20 Register
   0x370B,		//nQRTable30 Register
   0x0000,		//nCycles Register
   0xFFFF,		//nFullCapNom Register
   0x1070,		//nRComp0 Register
   0x223E,		//nTempCo Register
   0x0001,		//nIAvgEmpty Register
   0xFFFF,		//nFullCapRep Register
   0x0000,		//nVoltTemp Register
   0x807F,		//nMaxMinCurr Register
   0x00FF,		//nMaxMinVolt Register
   0x807F,		//nMaxMinTemp Register
   0x0000,		//nSOC Register
   0x0000,		//nTimerH Register
   0x0000,		//nConfig Register
   0x0204,		//nRippleCfg Register
   0x0000,		//nMiscCfg Register
   0xFFFF,		//nDesignCap Register
   0x0000,		//nHibCfg Register
   0x0C03,		//nPackCfg Register
   0x0000,		//nRelaxCfg Register
   0x2241,		//nConvgCfg Register
   0xC190,		//nNVCfg0 Register
   0x2006,		//nNVCfg1 Register
   0xFE0A,		//nNVCfg2 Register
   0x0002,		//nSBSCfg Register
   0x0000,		//nROMID0 Register
   0x0000,		//nROMID1 Register
   0x0000,		//nROMID2 Register
   0x0000,		//nROMID3 Register
   0x0000,		//nVAlrtTh Register
   0x0000,		//nTAlrtTh Register
   0x0000,		//nSAlrtTh Register
   0x0000,		//nIAlrtTh Register
   0x0000,		//nUser1C4 Register
   0x0000,		//nUser1C5 Register
   0x5505,		//nFullSOCThr Register
   0x0000,		//nTTFCfg Register
   0x0000,		//nCGain Register
   0x0025,		//nTCurve Register
   0x0000,		//nTGain Register
   0x0000,		//nTOff Register
   0x0000,		//nManfctrName0 Register
   0x0000,		//nManfctrName1 Register
   0x0000,		//nManfctrName2 Register
   0xF230,		//nRSense Register
   0x0000,		//nUser1D0 Register
   0x0000,		//nUser1D1 Register
   0xD5E3,		//nAgeFcCfg Register
   0x0000,		//nDesignVoltage Register
   0x0000,		//nUser1D4 Register
   0x0000,		//nRFastVShdn Register
   0x0000,		//nManfctrDate Register
   0x0000,		//nFirstUsed Register
   0x0000,		//nSerialNumber0 Register
   0x0000,		//nSerialNumber1 Register
   0x0000,		//nSerialNumber2 Register
   0x0000,		//nDeviceName0 Register
   0x0000,		//nDeviceName1 Register
   0x0000,		//nDeviceName2 Register
   0x0000,		//nDeviceName3 Register
   0x0000		//nDeviceName4 Register
};

s_MAX1720X_Context_t g_sMAX1720XContext;
static uint8_t g_u8CtxtSet = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/
e_MAX1720X_Error_t eMAX1720X_ContextSet(s_MAX1720X_Context_t p_sContext)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   
   if(   (p_sContext.fp_u32I2C_Read != NULL)
      && (p_sContext.fp_u32I2C_Write != NULL) )
   {
      g_sMAX1720XContext.fp_u32I2C_Read = p_sContext.fp_u32I2C_Read;
      g_sMAX1720XContext.fp_u32I2C_Write = p_sContext.fp_u32I2C_Write;
      l_eErrCode = MAX1720X_ERROR_NONE;
      g_u8CtxtSet = 1u;
   }  
   
   return l_eErrCode;
}

e_MAX1720X_Error_t eMAX1720X_Init(void)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_INIT;
   uint8_t l_u8Idx = 0u;
   uint8_t l_u8SizeModel = sizeof(g_cau16EZModel);
   
   if(g_u8CtxtSet == 1u)
   {
      for(l_u8Idx = 0u; l_u8Idx < l_u8SizeModel; l_u8Idx++)
      {
         l_eErrCode = eWriteRegister((START_ADDRESS_EZ_MODEL_CONFIG+l_u8Idx), g_cau16EZModel[l_u8Idx]);
         EXIT_ERROR_CHECK(l_eErrCode);
      }
   }
   
   return l_eErrCode;
}
e_MAX1720X_Error_t eMAX1720X_StatusGet(uint16_t * p_pu16Status)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   
   if((p_pu16Status != NULL) && (g_u8CtxtSet == 1u))
   {
      l_eErrCode = eReadRegister(MAX1720X_REG_STATUS, p_pu16Status);
   }
   
   return l_eErrCode;
}
e_MAX1720X_Error_t eMAX1720X_TemperatureGet(uint16_t * p_pu16Temp)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   
   if((p_pu16Temp != NULL) && (g_u8CtxtSet == 1u))
   {
      l_eErrCode = eReadRegister(MAX1720X_REG_TEMP, p_pu16Temp);
   }
   
   return l_eErrCode;
}
e_MAX1720X_Error_t eMAX1720X_VCellGet(uint16_t * p_pu16VCell)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   
   if((p_pu16VCell != NULL) && (g_u8CtxtSet == 1u))
   {
      l_eErrCode = eReadRegister(MAX1720X_REG_VCELL, p_pu16VCell);
   }
   
   return l_eErrCode;
}

e_MAX1720X_Error_t eMAX1720X_DebugWrite(uint16_t p_u16Register, uint16_t p_u16Data)
{
   return eWriteRegister(p_u16Register, p_u16Data);
}
e_MAX1720X_Error_t eMAX1720X_DebugRead(uint16_t p_u16Register,  uint16_t * p_pu16Value)
{
   return eReadRegister(p_u16Register, p_pu16Value);
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to read on registers.
 * @param[in] p_u16Register
 * @param[out] p_pu16Value
 * @param[in] p_u8RegNumber
 * @return Error code
 */
static e_MAX1720X_Error_t eReadRegisters(uint16_t p_u16Register, uint16_t * p_pu16Value, uint8_t p_u8RegNumber)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_CONTEXT;
   uint8_t l_u8I2CAddr = (p_u16Register >= 0x100)?MAX1720X_I2C_ADDR_HIGH:MAX1720X_I2C_ADDR_LOW;
   uint8_t l_u8Idx = 0u;
   uint8_t l_au8ReadData[2u] = { 0u };
   uint8_t l_au8RegData[2u] = { 0u };
   l_au8RegData[0u] = (p_u16Register & 0xFF00)>>8;
   l_au8RegData[1u] = (p_u16Register & 0x00FF);

   if( (g_sMAX1720XContext.fp_u32I2C_Read != NULL) & (p_pu16Value != NULL) )
   {
      for(l_u8Idx = 0u; l_u8Idx < p_u8RegNumber; l_u8Idx++)
      {
         if((*g_sMAX1720XContext.fp_u32I2C_Read)(l_u8I2CAddr, l_au8RegData, 2u, l_au8ReadData, 2u) == 0u)
         {
            p_pu16Value[l_u8Idx] = ((uint16_t)l_au8ReadData[0u] << 8) + (uint16_t)l_au8ReadData[1u];
            l_eErrCode = MAX1720X_ERROR_NONE;
         }
         else
         {
            l_eErrCode = MAX1720X_ERROR_COMM;
         }
      }
   }   
   return l_eErrCode;
}
static e_MAX1720X_Error_t eReadRegister(uint16_t p_u16Register, uint16_t * p_pu16Value)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_CONTEXT;
   uint8_t l_u8I2CAddr = (p_u16Register >= 0x100)?MAX1720X_I2C_ADDR_HIGH:MAX1720X_I2C_ADDR_LOW;
   uint8_t l_au8ReadData[2u] = { 0u };
   uint8_t l_au8RegData[2u] = { 0u };
   l_au8RegData[0u] = (p_u16Register & 0x00FF);

   if( (g_sMAX1720XContext.fp_u32I2C_Read != NULL) & (p_pu16Value != NULL) )
   {
      if((*g_sMAX1720XContext.fp_u32I2C_Read)(l_u8I2CAddr, l_au8RegData, 1u, l_au8ReadData, 2u) == 0u)
      {
         (*p_pu16Value) = ((uint16_t)l_au8ReadData[0u] << 8) + (uint16_t)l_au8ReadData[1u];
         l_eErrCode = MAX1720X_ERROR_NONE;
      }
      else
      {
         l_eErrCode = MAX1720X_ERROR_COMM;
      }
   }   
   return l_eErrCode;
}
/**@brief Function to write on register.
 * @param[in] p_u16Register
 * @param[in] p_u16Data
 * @return Error code
 */
static e_MAX1720X_Error_t eWriteRegister(uint16_t p_u16Register, uint16_t p_u16Data)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_CONTEXT;
   uint8_t l_au8Data[3u] = { 0u };
   uint8_t l_u8I2CAddr = (p_u16Register >= 0x100)?MAX1720X_I2C_ADDR_HIGH:MAX1720X_I2C_ADDR_LOW;
   l_au8Data[0u] = (p_u16Register & 0x00FF);
   l_au8Data[1u] = (p_u16Data & 0x00FF);
   l_au8Data[2u] = (p_u16Data & 0xFF00)>>8;

   if(g_sMAX1720XContext.fp_u32I2C_Write != NULL)
   {
      if((*g_sMAX1720XContext.fp_u32I2C_Write)(l_u8I2CAddr, l_au8Data, 3u) == 0u)
      {
         l_eErrCode = MAX1720X_ERROR_NONE;
      }
      else
      {
         l_eErrCode = MAX1720X_ERROR_COMM;
      }
   }

   return l_eErrCode;
}
/****************************************************************************************
 * End Of File
 ****************************************************************************************/


