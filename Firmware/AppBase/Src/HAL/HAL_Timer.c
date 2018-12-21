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
 * Date:          13/09/2017
 * Author:        Yoann Rebischung
 * Description:   Habstraction Layer for Timer 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

#include "HAL_Timer.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define HAL_MAX_TIMER   32u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static uint8_t g_u8TimerCnt = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vHal_Timer_Init(void)
{
   app_timer_init();
	g_u8TimerCnt = 0;
}

e_HalTimerErrorCode_t eHal_Timer_Create(const s_HalTimerId_t * p_psTimerID, e_HalTimerMode_t p_eMode, fpvCallbackTimer_t p_fpvCallback)
{
   e_HalTimerErrorCode_t l_eErrCode = HAL_TIMER_ERROR_PARAM;
   
   if(g_u8TimerCnt >= HAL_MAX_TIMER)
   {
      l_eErrCode = HAL_TIMER_ERROR_NO_MEM;
   }
   else
   {
      if(p_fpvCallback != NULL)
      {         
         if(app_timer_create(p_psTimerID, (app_timer_mode_t)p_eMode, p_fpvCallback) != 0u)
         {
            l_eErrCode = HAL_TIMER_ERROR_STATE;
         }
         else
         {
            l_eErrCode = HAL_TIMER_ERROR_NONE;
            /* Increase counter for next timer */
            g_u8TimerCnt++;
         }
      }
      else
      {
         l_eErrCode = HAL_TIMER_ERROR_PARAM;
      }
   }
   
   return l_eErrCode;
}

e_HalTimerErrorCode_t eHal_Timer_Start(s_HalTimerId_t p_sTimerID, uint32_t p_u32TimeOutMs)
{
   e_HalTimerErrorCode_t l_eErrCode = HAL_TIMER_ERROR_PARAM;
   
   if(app_timer_start(p_sTimerID, APP_TIMER_TICKS(p_u32TimeOutMs), NULL) != 0u)
   {
      l_eErrCode = HAL_TIMER_ERROR_STATE;
   }
   else
   {
      l_eErrCode = HAL_TIMER_ERROR_NONE;
   }   
   
   return l_eErrCode;
}

e_HalTimerErrorCode_t eHal_Timer_Stop(s_HalTimerId_t p_sTimerID)
{
   e_HalTimerErrorCode_t l_eErrCode = HAL_TIMER_ERROR_PARAM;
   
   if(app_timer_stop(p_sTimerID) != 0u)
   {
      l_eErrCode = HAL_TIMER_ERROR_STATE;
   }
   else
   {
      l_eErrCode = HAL_TIMER_ERROR_NONE;
   }
   
   return l_eErrCode;
}

e_HalTimerErrorCode_t eHal_Timer_StopAll(void)
{
   e_HalTimerErrorCode_t l_eErrCode = HAL_TIMER_ERROR_STATE;
   
   if(app_timer_stop_all() == 0u)
   {
      l_eErrCode = HAL_TIMER_ERROR_NONE;
   }
   
   return l_eErrCode;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


