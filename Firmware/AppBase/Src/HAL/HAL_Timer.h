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
 */
#ifndef HAL_TIMER_H
#define HAL_TIMER_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <app_timer.h>
#include <nrf_delay.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define HAL_TIMER_DEF(TimerID)            APP_TIMER_DEF(TimerID)
#define HAL_TIMER_MS_TO_TICKS(milliSec)   APP_TIMER_TICKS(milliSec)

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _HAL_TIMER_ERR_CODE_ {
   HAL_TIMER_ERROR_NONE = 0u,
   HAL_TIMER_ERROR_PARAM,
   HAL_TIMER_ERROR_STATE,
   HAL_TIMER_ERROR_NO_MEM,
   HAL_TIMER_ERROR
}e_HalTimerErrorCode_t;

typedef enum _HAL_TIMER_MODE_ {
   HAL_TIMER_MODE_SINGLE_SHOT = APP_TIMER_MODE_SINGLE_SHOT,
   HAL_TIMER_MODE_REPEATED = APP_TIMER_MODE_REPEATED
}e_HalTimerMode_t;

typedef void (*fpvCallbackTimer_t)(void * p_pvContext);
typedef app_timer_id_t s_HalTimerId_t;
typedef app_timer_t s_HalTimer_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vHal_Timer_Init(void);
e_HalTimerErrorCode_t eHal_Timer_Create(const s_HalTimerId_t * p_psTimerID, e_HalTimerMode_t p_eMode, fpvCallbackTimer_t p_fpvCallback);
e_HalTimerErrorCode_t eHal_Timer_Start(s_HalTimerId_t p_sTimerID, uint32_t p_u32TimeOutMs);
e_HalTimerErrorCode_t eHal_Timer_Stop(s_HalTimerId_t p_sTimerID);
e_HalTimerErrorCode_t eHal_Timer_StopAll(void);
__STATIC_INLINE uint32_t u32Hal_Timer_GetTicks(void);
__STATIC_INLINE uint32_t u32Hal_Timer_GetTickDiff(uint32_t p_u32TickStart, uint32_t p_u32TickEnd);
__STATIC_INLINE void vHal_Timer_DelayMs(uint32_t p_u32DelayMs);

/************************************************************************
 * Inline Public functions
 ************************************************************************/
__STATIC_INLINE uint32_t u32Hal_Timer_GetTicks(void)
{
   return app_timer_cnt_get();
}
__STATIC_INLINE uint32_t u32Hal_Timer_GetTickDiff(uint32_t p_u32TickStart, uint32_t p_u32TickEnd)
{
   return app_timer_cnt_diff_compute(p_u32TickEnd, p_u32TickStart);
}
__STATIC_INLINE void vHal_Timer_DelayMs(uint32_t p_u32DelayMs)
{
   nrf_delay_ms(p_u32DelayMs);
}

#endif /* HAL_TIMER_H */

