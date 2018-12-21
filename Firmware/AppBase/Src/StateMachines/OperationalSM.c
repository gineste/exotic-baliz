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
//uint8_t g_u8Char = 0u;
uint8_t g_au8Data[FRAME_SIZE_MAX] = { 0u };
//uint8_t g_u8PendingCmd = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Entry point of Operational State.
 * @return None
 */
void vOperational_Entry(void)
{
   /* Set LED to Operational */
   vSimpleLED_BLACK();

   CLEAR_TERMINAL();
   vHT_PrintHelp();   
}
/**@brief Main mode of operation, get data and send it over SigFox, advertise, etc ...
 * @return None
 */
void vOperational_Process(void)
{
   uint8_t l_u8RcvData = 0u;
   uint8_t l_u8Idx = 0u;
   
   //if(SEGGER_RTT_HasKey() == 1)
   {
      for(l_u8Idx = 0u; l_u8Idx < (FRAME_SIZE_MAX); l_u8Idx++)
      {      
         l_u8RcvData += SEGGER_RTT_HasData(l_u8Idx);
      }

      if(l_u8RcvData != 0u)//FRAME_SIZE_MAX
      {
         (void)SEGGER_RTT_Read(0,g_au8Data,FRAME_SIZE_MAX);
         
         vHT_CheckInput(g_au8Data, FRAME_SIZE_MAX);
         
         memset(g_au8Data, 0u, FRAME_SIZE_MAX);
      }
   }
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
 

