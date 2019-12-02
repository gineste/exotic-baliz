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
 * Date:          10/10/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Sub State Machine Module
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <nrf_delay.h>
#include "HAL/HAL_GPIO.h"

#include "BLE/BLE_Application.h"
#include "Libraries/SimpleLED.h"

#include "HardwareTest.h"
#include "SigFox.h"
#include "OperationalSM.h"

#include "GlobalDefs.h"

#include "BME280/BME280.h"
#include "HAL/HAL_RTC.h"

#include "SEGGER_RTT.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
#ifndef SIGFOX_CEM_TEST
uint8_t g_au8Data[256u] = { 0u };
#endif

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Entry point of Operational State.
 * @return None
 */
void vOperational_Entry(void)
{
#ifndef SIGFOX_CEM_TEST
   uint8_t l_au8Data[6u] = { 0u };
   uint8_t l_u8Size = 0u;
   /* Set LED to Operational */
   vSimpleLED_BLACK();

   CLEAR_TERMINAL();
   vBLE_MACAddressGet(l_au8Data, &l_u8Size);
   PRINT_CUSTOM("%02X:%02X:%02X:%02X:%02X:%02X\n",  
   l_au8Data[5u],l_au8Data[4u],l_au8Data[3u],l_au8Data[2u],l_au8Data[1u],l_au8Data[0u]);    
#else
   vSigFox_TestRadio(1u,868000000u,14u);
#endif
}
/**@brief Main mode of operation, get data and send it over SigFox, advertise, etc ...
 * @return None
 */
void vOperational_Process(void)
{
#ifndef SIGFOX_CEM_TEST
   uint8_t l_u8SizeRead = 0u;
   
   vHT_Scanf("%s", g_au8Data);
   l_u8SizeRead = strlen((char*)g_au8Data);
   l_u8SizeRead++;
   
   if(l_u8SizeRead > 0u)
   {
      vHT_CheckInput(g_au8Data , l_u8SizeRead);
   
      memset(g_au8Data, 0u, FRAME_SIZE_W_ARG_MAX);
   }
#endif
}

/**@brief Function to check if device should go in Connected, Error or DeepSleep State.
 * @return None
 */
void vOperational_Check(void)
{
   /* Check Ble Event Status */
   e_Ble_Event_t p_eBleEvent;
   p_eBleEvent = eBLE_EventGet();
   if(p_eBleEvent == BLE_EVENT_CONNECTED)
   {
      vMSM_StateMachineSet(MSM_CONNECTED);
   }
   else if(p_eBleEvent == BLE_EVENT_ERROR)
   {
      vMSM_StateMachineSet(MSM_ERROR);
   }
//   else if(eBLE_AdvertiseStateGet() == BLE_ADV_EVT_IDLE)
//   {
//      vMSM_StateMachineSet(MSM_DEEP_SLEEP);
//   }
   else
   {  /* Stay in these state */  }
}

/**@brief Exit point of Operational State.
 * @return None
 */
void vOperational_Exit(void)
{

}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

