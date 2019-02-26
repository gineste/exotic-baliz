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
 * Description:   Interface for AX SigFox module 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
   
#include "board.h"
#include "HAL/HAL_GPIO.h"
#include "HAL/HAL_Timer.h"

#include "GlobalDefs.h"

#include "AXSigFox.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
static struct {
   uint8_t au8DeviceId[AXSF_DEV_ID_SIZE];
   uint8_t au8DevicePac[AXSF_DEV_PAC_SIZE];
}g_sAxSigFoxInfo;

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vCallback_Dummy(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size);

static void vCallback_DeviceID(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size);
static void vCallback_DevicePAC(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static uint8_t g_u8AxSigFoxInDeepSleep = 0u;
static uint8_t g_u8AxSigFoxInSleep = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief  Initialize the AxSigFox module.
 * @return None.
 */
void vAXSigFox_Init(void)
{
   vAT_Init();
   
   memset(g_sAxSigFoxInfo.au8DeviceId, 0u, AXSF_DEV_ID_SIZE);
   memset(g_sAxSigFoxInfo.au8DevicePac, 0u, AXSF_DEV_PAC_SIZE);
}

/**@brief Callback function handler for AxSigFox.
 * @param[in] p_u8Data : Input Char.
 * @return None
 */
void vAXSigFox_UpdateFrame(const uint8_t p_u8Data)
{
   vAT_UpdateFrame(p_u8Data);
}

/**@brief      Send Command to AxSigFox.
 * @param[in]  p_eCmd : Command.
 * @param[in]  p_pu8Bytes : Data to send.
 * @param[in]  p_u8Size : Size of message.
 * @param[in]  p_pfpCallback : Callback.
 * @return Error Code.
 */
e_AXSigFox_ErrorCode_t eAXSigFox_SendCommand(e_AXSFCommands_t p_eCmd, uint8_t * p_pu8Bytes, uint8_t p_u8Size, fp_AXSFCallback_t p_pfpCallback)
{
   e_AXSigFox_ErrorCode_t l_eErrCode = AXSIGFOX_ERROR_NONE;

   vAT_QueueSend((e_AT_Commands_t)p_eCmd, p_pu8Bytes, p_u8Size, (fp_vATCallback_t)p_pfpCallback);
   
	return l_eErrCode;
}

/**@brief Function to put AxSigFox in Deep Sleep Mode.
 * @return None
 */
void vAXSigFox_DeepSleepModeSet(void)
{
   uint8_t l_au8Cmd[6u] = { 'A','T','$','P','=','2' }; 
   
   vHal_GPIO_Set(SIGFOX_DP_WU);
   vHal_Timer_DelayMs(200u);  
   
   vAT_DirectSend(l_au8Cmd, 6u, vCallback_Dummy);
   vHal_Timer_DelayMs(200u);  /* To Get Callback from UART */
   
   g_u8AxSigFoxInDeepSleep = 1u;
}
/**@brief Function to Wake Up AxSigFox from Deep Sleep Mode.
 * @return None
 */
void vAXSigFox_WakeUpFromDeepSleep(void)
{
   if(g_u8AxSigFoxInDeepSleep == 1u)
   {
      vHal_GPIO_Clear(SIGFOX_DP_WU);
      vHal_Timer_DelayMs(200u);
      
      g_u8AxSigFoxInDeepSleep = 0u;
   }
}

/**@brief Function to put AxSigFox in Sleep Mode.
 * @return None
 */
void vAXSigFox_SleepModeSet(void)
{   
   uint8_t l_au8Cmd[6u] = { 'A','T','$','P','=','1' }; 
   vAT_DirectSend(l_au8Cmd, 6u, vCallback_Dummy);
   vHal_Timer_DelayMs(200u);  /* To Get Callback from UART */
   g_u8AxSigFoxInSleep = 1u;
}
/**@brief Function to Wake Up AxSigFox from Sleep Mode.
 * @return None
 */
void vAXSigFox_WakeUpFromSleep(void)
{
   if(g_u8AxSigFoxInSleep == 1u)
   {
      /* Must do an Uart Break */
      uint8_t l_au8Cmd[2u] = { 'A','T' };
      vAT_DirectSend(l_au8Cmd, 2u, vCallback_Dummy);
      vHal_Timer_DelayMs(200u);  /* Todo TEST if its enough */  
      g_u8AxSigFoxInSleep = 0u;
   }
}

/**@brief Function to send AT command to read Device ID.
 * @return None
 */
void vAXSigFox_DeviceIDRead(void)
{
   uint8_t l_au8Cmd[2u] = {'1','0'};
   (void)eAXSigFox_SendCommand(AT_CMD_I, l_au8Cmd,2u,vCallback_DeviceID);
}
/**@brief Function to send AT command to read Device PAC.
 * @return None
 */
void vAXSigFox_DevicePACRead(void)
{
   uint8_t l_au8Cmd[2u] = {'1','1'};
   (void)eAXSigFox_SendCommand(AT_CMD_I, l_au8Cmd,2u,vCallback_DevicePAC);   
}
/**@brief Function to send AT command to read Device Info.
 * @return None
 */
void vAXSigFox_DeviceInfoRead(void)
{
   uint8_t l_au8Cmd[1u] = {'0'};
   
   l_au8Cmd[0u] = '0';
   (void)eAXSigFox_SendCommand(AT_CMD_I, l_au8Cmd,1u,vCallback_Dummy);
   l_au8Cmd[0u] = '1';
   (void)eAXSigFox_SendCommand(AT_CMD_I, l_au8Cmd,1u,vCallback_Dummy);
   l_au8Cmd[0u] = '7';
   (void)eAXSigFox_SendCommand(AT_CMD_I, l_au8Cmd,1u,vCallback_Dummy);
   l_au8Cmd[0u] = '9';
   (void)eAXSigFox_SendCommand(AT_CMD_I, l_au8Cmd,1u,vCallback_Dummy);
}
/**@brief Function to send AT command to read Device Temperature.
 * @return None
 */
void vAXSigFox_DeviceTemperatureRead(void)
{
   (void)eAXSigFox_SendCommand(AT_CMD_T, NULL, 0u,vCallback_Dummy);
}
/**@brief Function to return read Device PAC.
 * @param[out]  p_pu8Pac : Data to send.
 * @param[out]  p_u8Size : Size of message.
 * @return None
 */
void vAXSigFox_DevicePacGet(uint8_t * p_pu8Pac, uint8_t * p_u8Size)
{
   uint8_t l_au8PAC[AXSF_DEV_PAC_SIZE] = { 0u };
   
   if(p_u8Size != NULL)
   {
      if(p_pu8Pac != NULL)
      {
         if(strcmp((char*)l_au8PAC, (char*)g_sAxSigFoxInfo.au8DevicePac) != 0u)
         {
            memcpy(p_pu8Pac, g_sAxSigFoxInfo.au8DevicePac, AXSF_DEV_PAC_SIZE);
            (*p_u8Size) = AXSF_DEV_PAC_SIZE;
         }
         else
         {
            (*p_u8Size) = 0u;
         }
      }    
   }
}
/**@brief Function to return read Device ID.
 * @param[out]  p_pu8Id : Data to send.
 * @param[out]  p_u8Size : Size of message.
 * @return None
 */
void vAXSigFox_DeviceIdGet(uint8_t * p_pu8Id, uint8_t * p_u8Size)
{
   uint8_t l_au8ID[AXSF_DEV_ID_SIZE] = { 0u };
   
   if(p_u8Size != NULL)
   {
      if(p_pu8Id != NULL)
      {
         if(strcmp((char*)l_au8ID, (char*)g_sAxSigFoxInfo.au8DeviceId) != 0u)
         {
            memcpy(p_pu8Id, g_sAxSigFoxInfo.au8DeviceId, AXSF_DEV_ID_SIZE);
            (*p_u8Size) = AXSF_DEV_ID_SIZE;
         }
         else
         {
            (*p_u8Size) = 0u;
         }
      }    
   }
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static void vCallback_Dummy(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size)
{
   switch(p_eResult)
   {
      default:
      break;
   }
}

static void vCallback_DeviceID(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size)
{
   memset(g_sAxSigFoxInfo.au8DeviceId, 0u, AXSF_DEV_ID_SIZE);
   switch(p_eResult)
   {
      case AT_RET_END:
         memcpy(g_sAxSigFoxInfo.au8DeviceId, p_pu8Buffer, p_u8Size);
      break;
      default:
         PRINT_WARNING("%s","FAIL TO READ DEVICE ID AXSIGFOX");
      break;
   }
}

static void vCallback_DevicePAC(e_AT_RetVal_t p_eResult, uint8_t * p_pu8Buffer, uint8_t p_u8Size)
{
   memset(g_sAxSigFoxInfo.au8DevicePac, 0u, AXSF_DEV_PAC_SIZE);
   switch(p_eResult)
   {
      case AT_RET_END:
         memcpy(g_sAxSigFoxInfo.au8DevicePac, p_pu8Buffer, p_u8Size);
      break;
      default:
         PRINT_WARNING("%s","FAIL TO READ DEVICE PAC AXSIGFOX");
      break;
   }
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
