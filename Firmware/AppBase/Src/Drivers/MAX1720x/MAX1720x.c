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
#include "HAL/HAL_Timer.h"

/* Self include */
#include "MAX1720x.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define NUMBER_OF_CELLS					(uint8_t)3u

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

#define MAX1720X_REG_CONFIG2_ADR			0xBB							/* Config 2 register */

#define MAX1720X_REG_NPACKCFG_ADR							0x1B5			/* Config 2 register */
#define MAX1720X_REG_BNPACKCFG_ADR						0x0BD			/* Config 2 Persistant */
#define MAX1720X_REG_NPACKCFG_NCELL_MSK				0x000F		/* Number of cells */
#define MAX1720X_REG_NPACKCFG_NCELL_SHIFT			0

#define MAX1720X_REG_NPACKCFG_CELLX_MSK	 			0x0100		/* Enable CELLs Channel */
#define MAX1720X_REG_NPACKCFG_CELLX_SHIFT 		8
#define MAX1720X_REG_NPACKCFG_CELLX_DISABLE		0x0000
#define MAX1720X_REG_NPACKCFG_CELLX_ENABLE		0x0100
#define MAX1720X_REG_NPACKCFG_VBAT_MSK	 			0x0200		/* Enable VBAT Channel */
#define MAX1720X_REG_NPACKCFG_VBAT_SHIFT 			9
#define MAX1720X_REG_NPACKCFG_VBAT_DISABLE		0x0000
#define MAX1720X_REG_NPACKCFG_VBAT_ENABLE			0x0200
#define MAX1720X_REG_NPACKCFG_CHEN_MSK	 			0x0400		/* Enable CELL1/CELL2/VBAT Channel */
#define MAX1720X_REG_NPACKCFG_CHEN_SHIFT 			10				/*  override the previous bits */
#define MAX1720X_REG_NPACKCFG_CHEN_DISABLE		0x0000
#define MAX1720X_REG_NPACKCFG_CHEN_ENABLE			0x0400

#define MAX1720X_REG_NPACKCFG_TDEN_MSK	 			0x0800		/* Enable Die Temperature */
#define MAX1720X_REG_NPACKCFG_TDEN_SHIFT 			11
#define MAX1720X_REG_NPACKCFG_TDEN_DISABLE		0x0000
#define MAX1720X_REG_NPACKCFG_TDEN_ENABLE			0x0800
#define MAX1720X_REG_NPACKCFG_A1EN_MSK	 			0x1000		/* Enable A1 Temperature */
#define MAX1720X_REG_NPACKCFG_A1EN_SHIFT 			12
#define MAX1720X_REG_NPACKCFG_A1EN_DISABLE		0x0000
#define MAX1720X_REG_NPACKCFG_A1EN_ENABLE			0x1000
#define MAX1720X_REG_NPACKCFG_FGT_MSK		 		0x8000		/* Select the Temperature source */
#define MAX1720X_REG_NPACKCFG_FGT_SHIFT 			15
#define MAX1720X_REG_NPACKCFG_FGT_DISABLE			0x0000
#define MAX1720X_REG_NPACKCFG_FGT_ENABLE			0x8000

#define MAX1720X_REG_NHIBCFG							0x1B4
#define MAX1720X_REG_NHIBCFG_ENHIB_MSK				0x8000		/* Enable Hibernate mode switch (1) or disable it (0) */

#define MAX1720X_REG_NPACKCFG_TEMP_MSK	 			0xB800		/* Mask to clean the Temperature source selection */

#define MAX1720X_REG_NPACKCFG_TEMP_INTERNAL_DIETEMP 0x0800	/* Configuration - internal DieTemp (135h) */

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
   uint16_t l_u16Data = 0u;
   
   CONTEXT_CHECK();

	/* Preconfiguration - 3 cells */	
	/* Shadow ram */
	eWriteRegister(MAX1720X_REG_NPACKCFG_ADR,
					  MAX1720X_REG_NPACKCFG_VBAT_DISABLE
					| MAX1720X_REG_NPACKCFG_CHEN_DISABLE
					| MAX1720X_REG_NPACKCFG_TDEN_DISABLE
					| MAX1720X_REG_NPACKCFG_A1EN_DISABLE
					| MAX1720X_REG_NPACKCFG_FGT_DISABLE
					| NUMBER_OF_CELLS << MAX1720X_REG_NPACKCFG_NCELL_SHIFT
					 );
	/* Volatile ram */
	eWriteRegister(MAX1720X_REG_BNPACKCFG_ADR,
					  MAX1720X_REG_NPACKCFG_VBAT_DISABLE
					| MAX1720X_REG_NPACKCFG_CHEN_DISABLE
					| MAX1720X_REG_NPACKCFG_TDEN_DISABLE
					| MAX1720X_REG_NPACKCFG_A1EN_DISABLE
					| MAX1720X_REG_NPACKCFG_FGT_DISABLE
					| NUMBER_OF_CELLS << MAX1720X_REG_NPACKCFG_NCELL_SHIFT
					 );
	/* Memory write secure delay */
	g_sMAX1720XContext.fp_vDelay_ms(1000u);
				
	/* Check Device Name */
   l_eErrCode = eReadRegister(MAX1720X_REG_DEVNAME, &l_u16Data);
   EXIT_ERROR_CHECK(l_eErrCode);
   if((l_u16Data&0x0F) != 5)	/* Multi cells type feature 5h */
   {
      l_eErrCode = MAX1720X_ERROR_NOT_FOUND;
   }
	 
	/* Reset gauge */
   l_eErrCode = eWriteRegister(MAX1720X_REG_COMMAND,0x000F);
   EXIT_ERROR_CHECK(l_eErrCode);
   g_sMAX1720XContext.fp_vDelay_ms(WAIT_TIME_TPOR_MS);
   l_eErrCode = eWriteRegister(MAX1720X_REG_CONFIG2,0x0001);
   EXIT_ERROR_CHECK(l_eErrCode);
   g_sMAX1720XContext.fp_vDelay_ms(WAIT_TIME_TPOR_MS);
	 
	/* Configure - 3 cells / internal temp */
	l_eErrCode = eReadRegister(MAX1720X_REG_NPACKCFG_ADR,&l_u16Data);
	EXIT_ERROR_CHECK(l_eErrCode);
	l_u16Data &= ~MAX1720X_REG_NPACKCFG_NCELL_MSK;
	l_u16Data |= NUMBER_OF_CELLS << MAX1720X_REG_NPACKCFG_NCELL_SHIFT;
	l_u16Data &= ~MAX1720X_REG_NPACKCFG_TEMP_MSK;
	l_u16Data |= MAX1720X_REG_NPACKCFG_TEMP_INTERNAL_DIETEMP;
	l_u16Data &= ~MAX1720X_REG_NPACKCFG_CELLX_MSK;
	l_u16Data |= MAX1720X_REG_NPACKCFG_CHEN_ENABLE;
	l_u16Data |= MAX1720X_REG_NPACKCFG_VBAT_ENABLE;
	l_u16Data |= MAX1720X_REG_NPACKCFG_TDEN_ENABLE;
	l_eErrCode = eWriteRegister(MAX1720X_REG_NPACKCFG_ADR,l_u16Data);
	EXIT_ERROR_CHECK(l_eErrCode);
	l_eErrCode = eWriteRegister(MAX1720X_REG_BNPACKCFG_ADR,l_u16Data);
	EXIT_ERROR_CHECK(l_eErrCode);
   
   return l_eErrCode;
}

e_MAX1720X_Error_t eMAX1720X_ReadRAM(void)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_NONE;
   uint16_t l_u16Idx = 0u;
   uint16_t l_u16Data = 0u;
   
   for(l_u16Idx = 0u; l_u16Idx < 0x100; l_u16Idx++)
   {
      l_eErrCode = eReadRegister(l_u16Idx, &l_u16Data);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sMAX1720XContext.fp_vDelay_ms(1u);
      PRINT_CUSTOM("0x%03X: x%04X\n",l_u16Idx, l_u16Data);
   }
   
   return l_eErrCode;
}
e_MAX1720X_Error_t eMAX1720X_ReadShadowRAM(void)
{
   e_MAX1720X_Error_t l_eErrCode = MAX1720X_ERROR_NONE;
   uint16_t l_u16Idx = 0u;
   uint16_t l_u16Data = 0u;
   
   for(l_u16Idx = 0x100; l_u16Idx < 0x200; l_u16Idx++)
   {
      l_eErrCode = eReadRegister(l_u16Idx, &l_u16Data);
      EXIT_ERROR_CHECK(l_eErrCode);
      g_sMAX1720XContext.fp_vDelay_ms(1u);
      PRINT_CUSTOM("0x%03X: x%04X\n",l_u16Idx, l_u16Data);
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
   int16_t l_s16Val = 0u;
   CONTEXT_CHECK();
   
   if(p_ps32uAmps != NULL)
   {
      l_eErrCode = eReadRegister(MAX1720X_REG_CURRENT, (uint16_t *) &l_s16Val);
      EXIT_ERROR_CHECK(l_eErrCode);

      (*p_ps32uAmps) = (int32_t)(((float)((int64_t)(l_s16Val)) * 15625.0f)/(10.0f*MAX1720X_RSENSE_VAL));
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
         (*p_pu16Value) = ((uint16_t)l_au8ReadData[1u] << 8) + (uint16_t)l_au8ReadData[0u];
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
   l_au8Data[1u] = (p_u16Data & 0x00FF);     // LSB First
   l_au8Data[2u] = (p_u16Data & 0xFF00)>>8;  // MSB next

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


