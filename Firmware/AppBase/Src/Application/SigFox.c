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
 * Date:          14/12/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Interface for SigFox module 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "Libraries/AT.h"
#include "Libraries/Tool.h"
#include "AXSIGFOX/AXSigFox.h"

#include "UartManagement.h"

#include "GlobalDefs.h"

#include "SigFox.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define MAX_SIGFOX_SIZE             (uint8_t)12u
#define PERIODIC_STATUS_SIZE        (uint8_t)12u
#define ASCII_AT_MSG_SIZE_MAX       (uint8_t)26u
#define SIGFOX_EVENT_SIZE           (uint8_t)7u


/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
//static void vCallback_Dummy(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size);
static void vCallback_Test(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size);
static void vCallback_OnDemand(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief  Initialize the SigFox module.
 * @return None.
 */
void vSigFox_Init(void)
{
   vAXSigFox_Init();
}

/**@brief  Process function for all SigFox features (AT/AxSigfox).
 * @return None.
 */
void vSigFox_Process(void)
{
   if(u8AT_PendingCommand() == 1u)
   {
      if(eUartMngt_StateGet() != USM_SIGFOX)
      {
         eUartMngt_StateSet(USM_SIGFOX);         
      }
      else
      {
         vAT_MessageProcess();
      }
   }
   else
   {  /* Go back in previous UART MODE */  }
}

void vSigFox_DeviceInit(void)
{
   vAXSigFox_DeviceIDRead();
   vAXSigFox_DevicePACRead();
}

void vSigFox_OnDemandSend(void)
{  
   uint8_t l_au8Buffer[2u] = { 0xD1, 0xA9 };
   uint8_t l_au8ASCIIBuffer[ASCII_AT_MSG_SIZE_MAX] = { 0u };
   uint8_t l_u8Size = 2u;
   uint8_t l_u8ASCIISize = 0u;
   
   vTool_ASCIIConvert(l_au8Buffer, l_u8Size, l_au8ASCIIBuffer, &l_u8ASCIISize);
   l_au8ASCIIBuffer[l_u8ASCIISize++] = ',';
   l_au8ASCIIBuffer[l_u8ASCIISize++] = '1';
   
   (void)eAXSigFox_SendCommand(AT_CMD_SF, l_au8ASCIIBuffer, l_u8ASCIISize, vCallback_OnDemand);   
   
}

uint8_t u8SigFox_IsDevicePACOk(void)
{
   uint8_t l_au8PAC[AXSF_DEV_PAC_SIZE] = { 0u };
   uint8_t l_u8Size = 0u;
   
   vAXSigFox_DeviceIdGet(l_au8PAC, &l_u8Size);
      
   return (l_u8Size != 0u)?1u:0u;
}
uint8_t u8SigFox_IsDeviceIDOk(void)
{
   uint8_t l_au8ID[AXSF_DEV_ID_SIZE] = { 0u };
   uint8_t l_u8Size = 0u;
   
   vAXSigFox_DeviceIdGet(l_au8ID, &l_u8Size);
      
   return (l_u8Size != 0u)?1u:0u;
}

/**@brief Function to Test Radio on SigFox.
 * @param[in] p_u8Enable
 * @param[in] p_u32Freq
 * @param[in] p_u8dBm
 * @return None
 */
void vSigFox_TestRadio(uint8_t p_u8Enable, uint32_t p_u32Freq, uint8_t p_u8dBm)
{
   uint8_t l_u8ASCIISize = 0u;
   char l_achBuffer[ASCII_AT_MSG_SIZE_MAX] = { 0u };
   //uint8_t l_u8Size = 0u;
   
   p_u32Freq = (p_u32Freq > 999999999u)?999999999u:p_u32Freq;
   p_u32Freq = (p_u32Freq < 800000000u)?800000000u:p_u32Freq;
   p_u8dBm = (p_u8dBm > 14u)?14u:p_u8dBm;
   p_u8Enable = (p_u8Enable != 1u)?0u:p_u8Enable;
      
   if(p_u8Enable == 1u)
   {
      snprintf(l_achBuffer, 10, "%d", p_u32Freq);
      l_achBuffer[9u] = ',';
      snprintf(&l_achBuffer[10u], 2, "%d", p_u8Enable);
      l_achBuffer[11u] = ',';
      
      //l_u8Size = (p_u8dBm < 10u)?1u:2u;
      //snprintf(&l_achBuffer[12u], l_u8Size, "%d", p_u8dBm);
           
      //l_u8ASCIISize = 12u + l_u8Size;
      snprintf(&l_achBuffer[12u], 2, "%d", p_u8dBm);
      l_u8ASCIISize = 14u;
   }
   else
   {    
      snprintf(l_achBuffer, 6, "0,0,0");
      l_u8ASCIISize = 6u;
   }
   
   
   (void)eAXSigFox_SendCommand(AT_CMD_CW, (uint8_t*)l_achBuffer, l_u8ASCIISize, vCallback_Test);
}
/****************************************************************************************
 * Private functions
 ****************************************************************************************/
//static void vCallback_Dummy(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size)
//{
//}
static void vCallback_Test(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size)
{
   switch(p_eResult)
   {
      case AT_RET_END:
         /* Send ACK */
         break;
      default:
         /* Send NACK */
         PRINT_WARNING("%s\n","FAIL TO SEND PERIODIC UPDATE SIGFOX");
      break;
   }
}
static void vCallback_OnDemand(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size)
{
   uint16_t l_u16Idx = 0u;
   switch(p_eResult)
   {
      case AT_RET_END:
         /* Send ACK */
         for(l_u16Idx = 0u; l_u16Idx < p_u8Size;l_u16Idx++)
         {
            PRINT_INFO("%c", p_pu8Buffer[l_u16Idx]);
         }
            PRINT_INFO("%s", "\n");
         break;
      default:
         /* Send NACK */
      PRINT_WARNING("%s\n","FAIL TO SEND ON DEMAND MSG SIGFOX");
      break;
   }
}


/****************************************************************************************
 * End Of File
 ****************************************************************************************/

