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
#include "GlobalDefs.h"

/* Self include */
#include "MAX1720x.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define MAX1720X_I2C_ADDR_LOW       ((uint8_t)0x6C>>1)
#define MAX1720X_I2C_ADDR_HIGH      ((uint8_t)0x16>>1)

/* Useful registers to read */
#define MAX1720X_REG_STATUS         (uint16_t)0x000
#define MAX1720X_REG_STATUS2        (uint16_t)0x0B0
#define MAX1720X_REG_VCELL          (uint16_t)0x009
#define MAX1720X_REG_REPSOC         (uint16_t)0x006
#define MAX1720X_REG_REPCAP         (uint16_t)0x005
#define MAX1720X_REG_TEMP           (uint16_t)0x008
#define MAX1720X_REG_CURRENT        (uint16_t)0x00A
#define MAX1720X_REG_TTE            (uint16_t)0x011
#define MAX1720X_REG_TTF            (uint16_t)0x020
#define MAX1720X_REG_DEVNAME        (uint16_t)0x021
/* Additional read registers */
#define MAX1720X_REG_CELL1          (uint16_t)0x0D8
#define MAX1720X_REG_CELL2          (uint16_t)0x0D7
#define MAX1720X_REG_CELL3          (uint16_t)0x0D6
#define MAX1720X_REG_CELL4          (uint16_t)0x0D5
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

#define MAX1720X_REG_COMMAND        (uint16_t)0x060
#define MAX1720X_REG_CONFIG2        (uint16_t)0x0BB


#define MAX1720X_RSENSE_VAL         (float)620.0f
#define CONTEXT_CHECK()          do {     \
      if((g_u8CtxtSet != 1))              \
      {                                   \
         return MAX1720X_ERROR_CONTEXT;   \
      }                                   \
   }while(0);
      
#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != MAX1720X_ERROR_NONE))  \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);
   
#define WAIT_TIME_TPOR_MS           (uint32_t)15u
   
/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
//static e_MAX1720X_Error_t eReadRegisters(uint16_t p_u16Register, uint16_t * p_pu16Value, uint8_t p_u8RegNumber);
static e_MAX1720X_Error_t eReadRegister(uint16_t p_u16Register, uint16_t * p_pu16Value);
static e_MAX1720X_Error_t eWriteRegister(uint16_t p_u16Register, uint16_t p_u16Data);
static e_MAX1720X_Error_t eWriteBitsReg(uint16_t p_u16Register, uint16_t p_u16Value, uint16_t p_u16Pos, uint16_t p_u16Mask);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
#define START_ADDRESS_EZ_MODEL_CONFIG     0x180
const uint16_t g_cau16EZModel[] = {
   0x0000,		//nXTable0 Register
   0x0000,		//nXTable1 Register
   0x0000,		//nXTable2 Register
   0x0000,		//nXTable3 Register
   0x0000,		//nXTable4 Register
   0x0000,		//nXTable5 Register
   0x0000,		//nXTable6 Register
   0x0000,		//nXTable7 Register
   0x0000,		//nXTable8 Register
   0x0000,		//nXTable9 Register
   0x0000,		//nXTable10 Register
   0x0000,		//nXTable11 Register
   0x0000,		//nUser18C Register
   0x0000,		//nUser18D Register
   0x0000,		//nODSCTh Register
   0x0000,		//nODSCCfg Register
   0x0000,		//nOCVTable0 Register
   0x0000,		//nOCVTable1 Register
   0x0000,		//nOCVTable2 Register
   0x0000,		//nOCVTable3 Register
   0x0000,		//nOCVTable4 Register
   0x0000,		//nOCVTable5 Register
   0x0000,		//nOCVTable6 Register
   0x0000,		//nOCVTable7 Register
   0x0000,		//nOCVTable8 Register
   0x0000,		//nOCVTable9 Register
   0x0000,		//nOCVTable10 Register
   0x0000,		//nOCVTable11 Register
   0x0000,		//nIChgTerm Register
   0x0000,		//nFilterCfg Register
   0x0000,		//nVEmpty Register
   0x0000,		//nLearnCfg Register
   0x3C00,		//nQRTable00 Register
   0x1B80,		//nQRTable10 Register
   0x0B04,		//nQRTable20 Register
   0x0885,		//nQRTable30 Register
   0x0000,		//nCycles Register
   0xFFFF,		//nFullCapNom Register
   0x0000,		//nRComp0 Register
   0x0000,		//nTempCo Register
   0x0001,		//nIAvgEmpty Register
   0xFFFF,		//nFullCapRep Register
   0x0000,		//nVoltTemp Register
   0x0000,		//nMaxMinCurr Register
   0x0000,		//nMaxMinVolt Register
   0x0000,		//nMaxMinTemp Register
   0x0000,		//nSOC Register
   0x0000,		//nTimerH Register
   0x0000,		//nConfig Register
   0x0000,		//nRippleCfg Register
   0x0000,		//nMiscCfg Register
   0xFFFF,		//nDesignCap Register
   0x0000,		//nHibCfg Register
   0x0F03,		//nPackCfg Register
   0x0000,		//nRelaxCfg Register
   0x2241,		//nConvgCfg Register
   0x0000,		//nNVCfg0 Register
   0x0006,		//nNVCfg1 Register
   0x0000,		//nNVCfg2 Register
   0x0000,		//nSBSCfg Register
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
   0x0000,		//nFullSOCThr Register
   0x0000,		//nTTFCfg Register
   0x0000,		//nCGain Register
   0x0000,		//nTCurve Register
   0x0000,		//nTGain Register
   0x0000,		//nTOff Register
   0x0000,		//nManfctrName0 Register
   0x0000,		//nManfctrName1 Register
   0x0000,		//nManfctrName2 Register
   0xF230,		//nRSense Register
   0x0000,		//nUser1D0 Register
   0x0000,		//nUser1D1 Register
   0x0000,		//nAgeFcCfg Register
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
      && (p_sContext.fp_u32I2C_Write != NULL)
      && (p_sContext.fp_vDelay_ms != NULL) )
   {
      g_sMAX1720XContext.fp_u32I2C_Read = p_sContext.fp_u32I2C_Read;
      g_sMAX1720XContext.fp_u32I2C_Write = p_sContext.fp_u32I2C_Write;
      g_sMAX1720XContext.fp_vDelay_ms = p_sContext.fp_vDelay_ms;
      l_eErrCode = MAX1720X_ERROR_NONE;
      g_u8CtxtSet = 1u;
   }  
   
   return l_eErrCode;
}

e_MAX1720X_Error_t eMAX1720X_Init(void)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_INIT;
   uint8_t l_u8Idx = 0u;
   uint16_t l_u16Idx = 0u;
   uint8_t l_u8SizeModel = (sizeof(g_cau16EZModel) / sizeof(g_cau16EZModel[0]));
   uint16_t l_u16Data = 0u;
   
   CONTEXT_CHECK();

   for(l_u16Idx = 0u; l_u16Idx < 0x100; l_u16Idx++)
   {
      l_eErrCode = eReadRegister(l_u16Idx, &l_u16Data);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sMAX1720XContext.fp_vDelay_ms(1u);
      PRINT_CUSTOM("0x%02X: x%04X\n",l_u16Idx, l_u16Data);
   }
   
   /* Start by a reset of the device */
   l_eErrCode = eWriteRegister(MAX1720X_REG_COMMAND,0x000F);
   EXIT_ERROR_CHECK(l_eErrCode);
   g_sMAX1720XContext.fp_vDelay_ms(WAIT_TIME_TPOR_MS);
    l_eErrCode = eWriteRegister(MAX1720X_REG_CONFIG2,0x0001);
   EXIT_ERROR_CHECK(l_eErrCode);
   g_sMAX1720XContext.fp_vDelay_ms(WAIT_TIME_TPOR_MS);
   
   /* Read status2 */
   l_eErrCode = eReadRegister(MAX1720X_REG_STATUS2, &l_u16Data);
   EXIT_ERROR_CHECK(l_eErrCode);
   if((l_u16Data & 0x8000) != 0x8000)
   {
      l_eErrCode = eWriteBitsReg(0x01B8,0x0002,1,0x0002);
      EXIT_ERROR_CHECK(l_eErrCode);
   }      
   
   /* Init all structure */
   for(l_u8Idx = 0u; l_u8Idx < l_u8SizeModel; l_u8Idx++)
   {
      l_eErrCode = eWriteRegister((START_ADDRESS_EZ_MODEL_CONFIG+l_u8Idx), g_cau16EZModel[l_u8Idx]);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sMAX1720XContext.fp_vDelay_ms(1u);
   }
   /* Check Device Name */
   l_eErrCode = eReadRegister(MAX1720X_REG_DEVNAME, &l_u16Data);
   EXIT_ERROR_CHECK(l_eErrCode);
   if((l_u16Data&0x0F) != 5)
   {
      l_eErrCode = MAX1720X_ERROR_NOT_FOUND;
   }
   
   return l_eErrCode;
}
e_MAX1720X_Error_t eMAX1720X_StatusGet(uint16_t * p_pu16Status)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   CONTEXT_CHECK();
   
   if(p_pu16Status != NULL)
   {
      l_eErrCode = eReadRegister(MAX1720X_REG_STATUS, p_pu16Status);
   }
   
   return l_eErrCode;
}
e_MAX1720X_Error_t eMAX1720X_TemperatureGet(e_MAX1720X_Temperature_t p_eTemp, int32_t * p_ps32mTemp)
{   
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   uint16_t l_u16Val = 0u;
   uint16_t l_u16Reg = 0u;
   CONTEXT_CHECK();
   
   if(p_ps32mTemp != NULL)
   {
      switch(p_eTemp)
      {
         case MAX1720X_TEMP_INT:
            l_u16Reg = MAX1720X_REG_INT_TEMP;
            break;
         case MAX1720X_TEMP_1:
            l_u16Reg = MAX1720X_REG_TEMP1;
            break;
         case MAX1720X_TEMP_2:
            l_u16Reg = MAX1720X_REG_TEMP2;
            break;
         default:
            return l_eErrCode;
      }
      l_eErrCode = eReadRegister(l_u16Reg, &l_u16Val);
      EXIT_ERROR_CHECK(l_eErrCode);
      (*p_ps32mTemp) = ((int32_t)l_u16Val*1000) >> 8;
   }
   
   return l_eErrCode;
}

e_MAX1720X_Error_t eMAX1720X_VoltageGet(e_MAX1720X_Cell_t p_eCell, uint16_t * p_pu16mVolt)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   uint16_t l_u16Val = 0u;
   uint16_t l_u16Reg = 0u;
   CONTEXT_CHECK();
   
   if(p_pu16mVolt != NULL)
   {
      switch(p_eCell)
      {
         case MAX1720X_CELL_1:
            l_u16Reg = MAX1720X_REG_CELL1;
            break;
         case MAX1720X_CELL_2:
            l_u16Reg = MAX1720X_REG_CELL2;
            break;
         case MAX1720X_CELL_3:
            l_u16Reg = MAX1720X_REG_CELL3;
            break;
         case MAX1720X_CELL_4:
            l_u16Reg = MAX1720X_REG_CELL4;
            break;
         case MAX1720X_CELL_X:
            l_u16Reg = MAX1720X_REG_CELLX;
            break;
         case MAX1720X_VBAT:
            l_u16Reg = MAX1720X_REG_BATT;
            break;
         default:
            return l_eErrCode;            
      }
      
      l_eErrCode = eReadRegister(l_u16Reg, &l_u16Val);
      EXIT_ERROR_CHECK(l_eErrCode);
      
      if(p_eCell == MAX1720X_VBAT)
      {
         (*p_pu16mVolt) = (l_u16Val + (l_u16Val >> 2));	// 1.25mV / LSB
      }
      else
      {
         (*p_pu16mVolt) = (uint16_t)((((uint64_t)l_u16Val)*78125)/1000000); // Ratio 0.000078125 V / LSB
      }
   }
   
	return l_eErrCode;
}

e_MAX1720X_Error_t eMAX1720X_CurrentGet(int32_t * p_ps32uAmps)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_PARAM;
   uint16_t l_u16Val = 0u;
   CONTEXT_CHECK();
   
   if(p_ps32uAmps != NULL)
   {
      l_eErrCode = eReadRegister(MAX1720X_REG_CURRENT, &l_u16Val);
      EXIT_ERROR_CHECK(l_eErrCode);

      (*p_ps32uAmps) = (int32_t)(((float)((int64_t)(l_u16Val)) * 15625.0f)/(10.0f*MAX1720X_RSENSE_VAL));
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
//static e_MAX1720X_Error_t eReadRegisters(uint16_t p_u16Register, uint16_t * p_pu16Value, uint8_t p_u8RegNumber)
//{
//   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_CONTEXT;
//   uint8_t l_u8I2CAddr = (p_u16Register >= 0x100)?MAX1720X_I2C_ADDR_HIGH:MAX1720X_I2C_ADDR_LOW;
//   uint8_t l_u8Idx = 0u;
//   uint8_t l_au8ReadData[2u] = { 0u };
//   uint8_t l_au8RegData[2u] = { 0u };
//   l_au8RegData[0u] = (p_u16Register & 0xFF00)>>8;
//   l_au8RegData[1u] = (p_u16Register & 0x00FF);

//   if( (g_sMAX1720XContext.fp_u32I2C_Read != NULL) & (p_pu16Value != NULL) )
//   {
//      for(l_u8Idx = 0u; l_u8Idx < p_u8RegNumber; l_u8Idx++)
//      {
//         if((*g_sMAX1720XContext.fp_u32I2C_Read)(l_u8I2CAddr, l_au8RegData, 2u, l_au8ReadData, 2u) == 0u)
//         {
//            p_pu16Value[l_u8Idx] = ((uint16_t)l_au8ReadData[0u] << 8) + (uint16_t)l_au8ReadData[1u];
//            l_eErrCode = MAX1720X_ERROR_NONE;
//         }
//         else
//         {
//            l_eErrCode = MAX1720X_ERROR_COMM;
//         }
//      }
//   }   
//   return l_eErrCode;
//}
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

static e_MAX1720X_Error_t eWriteBitsReg(uint16_t p_u16Register, uint16_t p_u16Value, uint16_t p_u16Pos, uint16_t p_u16Mask)
{
   uint16_t l_u16RegVal = 0u;
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_CONTEXT;
   
   l_eErrCode = eReadRegister(p_u16Register, &l_u16RegVal); 

   if(l_eErrCode == MAX1720X_ERROR_NONE)
   {
      l_u16RegVal &= ~p_u16Mask;
          
      l_u16RegVal |= ((uint8_t)p_u16Value << p_u16Pos);

      l_eErrCode = eWriteRegister(p_u16Register, l_u16RegVal);
   }

   return l_eErrCode;
}
/****************************************************************************************
 * End Of File
 ****************************************************************************************/


