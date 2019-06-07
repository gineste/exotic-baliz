/*
 *  ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 * (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *  ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 * (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Date:          10 07 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Main State Machine of nRF BLE Gravity
 *
 */

/************************************************************************
 * Include Files
 ************************************************************************/
/* Proprietary includes */
#include <stdint.h>
#include <string.h>

#include "BoardConfig.h"
#include "GlobalDefs.h"
#include "board.h"   // For GPIO

/* Application includes */
#include "SigFox.h"
#include "UartManagement.h"
#include "HardwareTest.h"

/* HAL Includes */
#include "HAL/HAL_Timer.h"
#include "HAL/HAL_GPIO.h"

/* Libraries includes */

#include "BLE/BLE_Application.h"
//#include "ES_Interfaces.h"
//#include "bt_interface.h"
//#include "DataUpdater.h"
//#include "SensorManager.h"
//#include "FrameManager.h"
//#include "EventManager.h"

/* Self include + State Machines */
#include "PowerUpSM.h"
#include "InitSM.h"
#include "OperationalSM.h"
#include "ConnectedSM.h"
#include "DeepSleepSM.h"
#include "ErrorSM.h"

#include "Version.h"

#include "MainStateMachine.h"

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef struct _STATEMACHINE_ {
   void (*fp_vEntryPoint)(void);
   void (*fp_vIdleProcess)(void);
   void (*fp_vNextStateCheck)(void);
   void (*fp_vExitPoint)(void);
   uint32_t u32CallbackTimer;
}s_StateMachine_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vBackgroundProcess(void);
static void vCallbackProcess(void * p_pvContext);

/************************************************************************
 * Variable declarations
 ************************************************************************/
#ifndef DEBUG
static
#endif
e_MainStateMachine_t g_eMainState = MSM_POWER_UP;

HAL_TIMER_DEF(MSM_ProcessTimerIdx);

static uint8_t g_u8CyclicProcess = 1u;

static s_StateMachine_t g_sStateMachines[MSM_MAX_NUMBERS] = {
   /*Entry,             Process,                Check,               Exit                 CallbackTimer*/
   {NULL,               vPowerUp_Process,       vPowerUp_Check,      NULL,                POWERUP_CB_TIMER_MS},      /* MSM_POWER_UP */
   {vInit_Entry,        vInit_Process,          vInit_Check,         vInit_Exit,          INIT_CB_TIMER_MS},         /* MSM_INIT */
   {vOperational_Entry, vOperational_Process,   vOperational_Check,  vOperational_Exit,   NO_TIMER/*OPERATIONAL_CB_TIMER_MS*/},  /* MSM_OPERATIONAL */
   {vConnected_Entry,   vConnected_Process,     vConnected_Check,    vConnected_Exit,     CONNECTED_CB_TIMER_MS},    /* MSM_CONNECTED */
   {vDeepSleep_Entry,   vDeepSleep_Process,     NULL,                vDeepSleep_Exit,     NO_TIMER},                 /* MSM_DEEP_SLEEP */
   {vError_Entry,       vError_Process,         vError_Check,        vError_Exit,         NO_TIMER}                  /* MSM_ERROR */
};

#if (EN_LOG == 1)
   static uint8_t g_au8StateString[MSM_MAX_NUMBERS][12u] = 
   { "POWER UP\0", "INIT\0", "OPERATIONAL\0", "CONNECTED\0", "DEEP SLEEP\0", "ERROR\0" };
#endif

/************************************************************************
 * Public functions
 ************************************************************************/
/**@brief Function to generate an interruption in order to wakeup µC.
 * @return None
 */
void vMSM_Init(void)
{  
   /* Init Timer First */
   vHal_Timer_Init();   
   
   CLEAR_TERMINAL();
   PRINT_INFO("%s","\n/****************************/\n");
   PRINT_INFO("%s","/*           Baliz          */\n");
   PRINT_INFO("%s","/*      nRF v ");
   PRINT_INFO("%02d.",FW_VERSION_MAJOR);
      PRINT_INFO("%02d.",FW_VERSION_MINOR);
         PRINT_INFO("%02d",FW_VERSION_REVISION);
   PRINT_INFO("%s","      */\n/*      Commit ");
      PRINT_INFO("%s      */\n",COMMIT_NUMBER);
   PRINT_INFO("/*   %s ",__TIME__);PRINT_INFO("%s   */\n",__DATE__);
   PRINT_INFO("%s","/****************************/\n\n");
   
#if (EN_LOG == 1)
   PRINT_INFO("%s","Main StateMachine : ");
   PRINT_STATE("%s\n",g_au8StateString[g_eMainState]);
#endif
   
   /* GPIO Init */
   vHal_GPIO_Init();
      
   if(eHal_Timer_Create(&MSM_ProcessTimerIdx, HAL_TIMER_MODE_REPEATED, &vCallbackProcess) == HAL_TIMER_ERROR_NONE)
   {
      if(eHal_Timer_Start(MSM_ProcessTimerIdx, g_sStateMachines[g_eMainState].u32CallbackTimer) != HAL_TIMER_ERROR_NONE)
      {
         PRINT_ERROR("%s","Start Process Timer Fail !");
      }
   }
   else
   {
      PRINT_ERROR("%s","Create Process Timer Fail !");
   }
}

/**@brief Process function of Main State Machine.
 * @return None
 */
void vMSM_Process(void)
{
   /* Run Idle Process */
   if(g_u8CyclicProcess == 1u)
   {
      if(g_sStateMachines[g_eMainState].fp_vIdleProcess != NULL)
      {
         g_sStateMachines[g_eMainState].fp_vIdleProcess();
      }/* Else nothing to execute */

      g_u8CyclicProcess = 0u;
   }
   
   /* Re execute process directly (should generate event) for WFE__ */
   if(g_sStateMachines[g_eMainState].u32CallbackTimer == NO_TIMER)
   {
      g_u8CyclicProcess = 1u;
   }
      
   /* Then run Check for next State Process */
   if(g_sStateMachines[g_eMainState].fp_vNextStateCheck != NULL)
   {
      g_sStateMachines[g_eMainState].fp_vNextStateCheck();
   }/* Else nothing to execute */

   vBackgroundProcess();
}

/**@brief Function to switch between the different state machine.
 *
 * @param[in]  p_eState : The next state machine to run
 * @return     None
 */
void vMSM_StateMachineSet(e_MainStateMachine_t p_eState)
{   
   if(p_eState < MSM_MAX_NUMBERS)
   {      
      /* Execute Exit point of current */
      if(g_sStateMachines[g_eMainState].fp_vExitPoint != NULL)
      {
         g_sStateMachines[g_eMainState].fp_vExitPoint();
      }/* Else nothing to execute */
      
      /* Assign new State */
      g_eMainState = p_eState;
      
   #if (EN_LOG == 1)
      PRINT_INFO("%s","Main StateMachine : ");
      PRINT_STATE("%s\n",g_au8StateString[g_eMainState]);
   #endif
      
      /* Execute Entry point of new State */
      if(g_sStateMachines[g_eMainState].fp_vEntryPoint != NULL)
      {
         g_sStateMachines[g_eMainState].fp_vEntryPoint();
      }/* Else nothing to execute */
      
      /* Start new Timer */
      if(g_sStateMachines[g_eMainState].u32CallbackTimer != NO_TIMER)
      {
         (void)eHal_Timer_Start(MSM_ProcessTimerIdx,g_sStateMachines[g_eMainState].u32CallbackTimer);
      }
      else
      {  /* Stop Timer */
         (void)eHal_Timer_Stop(MSM_ProcessTimerIdx);
      }            
   } /* Else not a valid state machine */
}

/**@brief Get current state of Main State Machine.
 * @return State of Main State Machine
 */
e_MainStateMachine_t eMSM_GetState(void)
{
   return g_eMainState;
}

/**@brief Get Cyclic process run state of State Machine.
 * @return State of CyclicProcess variable.
 */
uint8_t u8MSM_CyclicProcessCheck(void)
{
   return g_u8CyclicProcess;
}

/**@brief Clear Cyclic Process variable.
 * @return None.
 */
void vMSM_CyclicProcessClear(void)
{
   g_u8CyclicProcess = 0u;
}

/**@brief Set refresh time of the current state of Main State Machine.
 * @param[in]  p_u32RefreshTime : New refresh rate of the current 
 *             state machine.
 * @return     None
 */
void vStateMachine_ChangeTime(uint32_t p_u32RefreshTime)
{
   p_u32RefreshTime = (p_u32RefreshTime < 20u)?20u:p_u32RefreshTime;
   /* Stop Timer */
   if(g_sStateMachines[g_eMainState].u32CallbackTimer != NO_TIMER)
   {
      (void)eHal_Timer_Stop(MSM_ProcessTimerIdx);
   }
   g_sStateMachines[g_eMainState].u32CallbackTimer = p_u32RefreshTime;
   /* Update Timer */
   if(g_sStateMachines[g_eMainState].u32CallbackTimer != NO_TIMER)
   {
      (void)eHal_Timer_Start(MSM_ProcessTimerIdx,g_sStateMachines[g_eMainState].u32CallbackTimer);
   }
}

/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief Idle process function necessary for UART/Event/Sensor/...
 * @return None
 */
static void vBackgroundProcess(void)
{
   /* UART */
   vUartMngt_Process();
   
   /* SigFox */
   vSigFox_Process();   
   
   /* Hardware Test */
   vHT_BackgroundProcess();
}

/**********************************************
 * Private Functions linked to State Machines *
 **********************************************/
static void vCallbackProcess(void * p_pvContext)
{
   g_u8CyclicProcess = 1u;
}

/************************************************************************
 * End Of File
 ************************************************************************/


