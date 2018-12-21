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

#include "BoardConfig.h"
#include "GlobalDefs.h"

/* BLE includes */
#include "BLE/BLE_Application.h"

/* Library includes */
#include "Libraries/SimpleLED.h"

/* Self + StateMachine includes */
#include "MainStateMachine.h"
#include "ConnectedSM.h"

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

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Entry point of Connected State.
 * @return None
 */
void vConnected_Entry(void)
{
   /* Set LED to Connected */
//   vSimpleLED_BLUE();
   
}

/**@brief Once connected functionalities if different from operational.
 * @return None
 */
void vConnected_Process(void)
{
}

/**@brief Function to check if device should go in Operational, Error or DeepSleep State.
 * @return None
 */
void vConnected_Check(void)
{
   /* Check Ble Event Status */
   e_Ble_Event_t p_eBleEvent;
   p_eBleEvent = eBLE_EventGet();
   if(   (p_eBleEvent == BLE_EVENT_DISCONNECTED)
      || (p_eBleEvent == BLE_EVENT_TIMEOUT) )
   {
      vMSM_StateMachineSet(MSM_OPERATIONAL);
   }
   else if(p_eBleEvent == BLE_EVENT_ERROR)
   {
      vMSM_StateMachineSet(MSM_ERROR);
   }
   else
   {  /* Stay in these state */  }
}

/**@brief Exit point of Connected State.
 * @return None
 */
void vConnected_Exit(void)
{
//   vSimpleLED_BLACK();
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

