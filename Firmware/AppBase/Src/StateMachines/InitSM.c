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
/* Proprietary includes */
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* Config include */
#include "BoardConfig.h"

/* HAL includes */
#include "HAL/HAL_UART.h"
#include "HAL/HAL_Timer.h"

/* Libraries includes */
#include "Libraries/SimpleLED.h"
/* BLE include */
#include "BLE/BLE_Application.h"
//#include "BLE/BT_Interface.h"

/* Application includes */
#include "SigFox.h"
//#include "SensorManager.h"
#include "InterruptManager.h"
#include "HardwareTest.h"

/* Self include + State Machines */
#include "MainStateMachine.h"
#include "InitSM.h"

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
static uint8_t g_u8IsInitialized = 0u;
static uint8_t g_u8Error = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Entry point of Init State.
 * @return None
 */
void vInit_Entry(void)
{      
   /* Set LED to Init */
//   vSimpleLED_YELLOW();
   
   /* BLE (ESI, BT_Itf, BLE_Application */
//   vBT_ITF_Init();	
      
   if(eIntMngr_Init() != INT_MNG_ERROR_NONE)
   {
      g_u8Error = 1u;
   }
   
   /* HT timer */
   vHT_Init();
   g_u8IsInitialized = 0u;
}

/**@brief Function to initialize State Machine/Sensors/SigFox/etc...
 * @return None
 */
void vInit_Process(void)
{
   /* Can switch to Operational now */
   g_u8IsInitialized = 1u; 
}

/**@brief Function to check if the system must go in Error or Operational
 * @return None
 */
void vInit_Check(void)
{
   if(g_u8Error == 1u)
   {  /* Something wrong happened go to Error */
      vMSM_StateMachineSet(MSM_ERROR);
   }
   else if(g_u8IsInitialized == 1u)
   {  /* Initialization OK then go to Operational */
      vMSM_StateMachineSet(MSM_OPERATIONAL);
   }
   else
   {  /* Stay in these state */  }
}

/**@brief Exit point of Init State.
 * @return None
 */
void vInit_Exit(void)
{

}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

