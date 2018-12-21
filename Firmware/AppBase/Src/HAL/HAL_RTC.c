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
 * Date:          02 10 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   RTC abstraction layer 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <app_util_platform.h>

#ifdef RTC_ON_APP_TIMER
   #include "HAL_Timer.h"
   #include "GlobalDefs.h"
#endif

#include "Libraries/ES_Libraries/ExoTime.h"

#include "HAL_RTC.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#ifdef RTC_ON_APP_TIMER
   #define RTC_BASE              (uint32_t)SEC_TO_MS(1u)          /* Base is one sec for Timestamp */
#else
   /**< Prescaler used to get a Tick of ~1 s FreqRTC = (APP_TIMER_CLOCK_FREQ/(PRESCALER + 1))  (32.768/Freq) - 1 */
   #define RTC2_PRESCALER        ((uint16_t)0u)
   #define RTC2_COMPARE_COUNTER  ((uint16_t)32768u)
   #define RTC2_IRQ_PRI          APP_IRQ_PRIORITY_LOW             /**< Priority of the RTC1 interrupt (used for checking for timeouts and executing timeout handlers). */
   #define MAX_RTC_TASKS_DELAY   47                               /**< Maximum delay until an RTC task is executed. */
   #define MAX_RTC_COUNTER_VAL   0x00FFFFFF
#endif

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
#ifdef RTC_ON_APP_TIMER
   static void vCallback_RTCTimer(void * p_vContext);
#else
   static void vRTC2_Init(uint32_t p_u32Prescaler);
#endif
static void vRTC2_Start(void);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static volatile uint32_t g_u32RTCCounter = 0u;
static uint32_t g_u32BuildTimestamp = 0u;
#ifdef RTC_ON_APP_TIMER
   HAL_TIMER_DEF(HAL_RTC_TimerIdx);
#endif

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vHal_RTC_Init(void)
{
   vCompilationInfoToTimestamp(&g_u32BuildTimestamp);
#ifdef RTC_ON_APP_TIMER
   (void)eHal_Timer_Create(&HAL_RTC_TimerIdx, HAL_TIMER_MODE_REPEATED, &vCallback_RTCTimer);
#else
   vRTC2_Init(RTC2_PRESCALER);
#endif
   vRTC2_Start();
}


void vHal_RTC_TimeStampSet(uint32_t p_u32Timestamp)
{   
#if (LOG_BLE_RX == 1)
   PRINT_DEBUG("Old Timestamp: %d, ", g_u32RTCCounter);
   PRINT_DEBUG("New Timestamp: %d\n", p_u32Timestamp);
#endif
   
#ifdef RTC_ON_APP_TIMER
   g_u32RTCCounter = p_u32Timestamp;
#else
   NVIC_DisableIRQ(RTC2_IRQn);
   g_u32RTCCounter = p_u32Timestamp;
   NVIC_EnableIRQ(RTC2_IRQn);
#endif
}

uint32_t u32Hal_RTC_TimeStampGet(void)
{
   return g_u32RTCCounter;
}

uint8_t u8Hal_RTC_IsTimeStampValid(void)
{
   return (g_u32BuildTimestamp > g_u32RTCCounter)? 0u:1u;
}

/**@brief Function for handling the RTC2 interrupt.
 *
 * @details Checks for Compare Register, Event Register.
 */
void RTC2_IRQHandler(void)
{
   if(NRF_RTC2->EVENTS_COMPARE[0u] == 1u)
   {
      g_u32RTCCounter++;
      NRF_RTC2->EVENTS_COMPARE[0u] = 0u;
      NRF_RTC2->TASKS_CLEAR = 1u;
   }
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
#ifndef RTC_ON_APP_TIMER
/**@brief Function for initializing the RTC1 counter.
 *
 * @param[in] prescaler   Value of the RTC1 PRESCALER register. Set to 0 for no prescaling.
 */
static void vRTC2_Init(uint32_t p_u32Prescaler)
{
   NRF_RTC2->PRESCALER = p_u32Prescaler;
   NVIC_SetPriority(RTC2_IRQn, RTC2_IRQ_PRI);
}
#endif
/**@brief Function for starting the RTC1 timer.
 */
static void vRTC2_Start(void)
{
#ifdef RTC_ON_APP_TIMER
   (void)eHal_Timer_Start(HAL_RTC_TimerIdx, RTC_BASE);
#else
   NRF_RTC2->CC[0u] = RTC2_COMPARE_COUNTER;
   NRF_RTC2->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
   NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;

   NVIC_ClearPendingIRQ(RTC2_IRQn);
   NVIC_EnableIRQ(RTC2_IRQn);

   NRF_RTC2->TASKS_START = 1;
   //nrf_delay_us(MAX_RTC_TASKS_DELAY);
#endif
}

#ifdef RTC_ON_APP_TIMER
/**@brief Callback function in order to increase timestamp counter.
 * @param[in] p_vContext : Timer context.
 */
static void vCallback_RTCTimer(void * p_vContext)
{
   g_u32RTCCounter++;
}
#endif
/****************************************************************************************
 * End Of File
 ****************************************************************************************/


