/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          22/03/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Hardware Abstraction Layer for Watchdog Timer. 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#if (WDT_ENABLED == 1)

#include <stdint.h>
#include <string.h>

#include "nrf_drv_wdt.h"

/* Self include */
#include "HAL_WDT.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define WDT_CHANNEL_REGISTER_NB     NRF_WDT_CHANNEL_NUMBER
#define WDT_MIN_TIMEOUT             (uint32_t)15u
#define WDT_RR_VALUE                (uint32_t)0x6E524635UL

#define WDT_DEFAULT_CONFIG          NRF_DRV_WDT_DEAFULT_CONFIG
#define WDT_REGISTER_IDX            0

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vWDTHandler(void);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
//static uint32_t g_au32ReloadRegister[WDT_CHANNEL_REGISTER_NB] = { 0u };
static uint8_t g_u8IsInitialized = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Function for initializing WDT module.
 * @param[in] p_u8WatchdogId register idx: between 0 and 7
 * @param[in] p_u32TimeOut Timeout in ms before trig an interrupt (min allowed 15)
 * @param[in] p_sWatchdogHandler User callback at the end of timeout
 * @return Error Code
 */
e_HalWDT_Error_t eHal_WDT_Init(uint32_t p_u32TimeOut, fpv_WatchdogHandler_t p_sWatchdogHandler)
{
   e_HalWDT_Error_t l_eErrCode = HALWDT_ERROR_INIT;
   
   nrf_drv_wdt_config_t l_sConfig = WDT_DEFAULT_CONFIG;
   
   if(g_u8IsInitialized == 0u)
   {
      l_sConfig.reload_value = (p_u32TimeOut<WDT_MIN_TIMEOUT)?WDT_MIN_TIMEOUT:p_u32TimeOut;
      
      if(p_sWatchdogHandler != NULL)
      {
         if(nrf_drv_wdt_init(&l_sConfig, (nrf_wdt_event_handler_t)p_sWatchdogHandler) == 0u)
         {
            l_eErrCode = HALWDT_ERROR_NONE;
         }
      }
      else
      {
         if(nrf_drv_wdt_init(&l_sConfig, (nrf_wdt_event_handler_t)vWDTHandler) == 0u)
         {
            l_eErrCode = HALWDT_ERROR_NONE;
         }
      }
   }
   
   if(l_eErrCode == HALWDT_ERROR_NONE)
   {
      g_u8IsInitialized = 1u;
   }
   
   return l_eErrCode;
}

e_HalWDT_Error_t eHal_WDT_Start(void)
{
   nrf_drv_wdt_channel_id  l_sChannelId = (nrf_drv_wdt_channel_id)WDT_REGISTER_IDX;
   e_HalWDT_Error_t l_eErrCode = HALWDT_ERROR_PARAM;
   
   if(g_u8IsInitialized == 1u)
   {
      if(nrf_drv_wdt_channel_alloc(&l_sChannelId) == 0u)
      {
         l_eErrCode = HALWDT_ERROR_NONE;
         nrf_drv_wdt_enable();
      }
   }
   else
   {
      l_eErrCode = HALWDT_ERROR_INIT;
   }
   
   return l_eErrCode;
}

e_HalWDT_Error_t eHal_WDT_Reload(void)
{
   nrf_drv_wdt_channel_id  l_sChannelId = (nrf_drv_wdt_channel_id)WDT_REGISTER_IDX;
   e_HalWDT_Error_t l_eErrCode = HALWDT_ERROR_PARAM;
   
   if(g_u8IsInitialized == 1u)
   {
      l_eErrCode = HALWDT_ERROR_NONE;

      nrf_drv_wdt_channel_feed(l_sChannelId);
   }
   else
   {
      l_eErrCode = HALWDT_ERROR_INIT;
   }
   
   return l_eErrCode;
}

e_HalWDT_Error_t eHal_WDT_Stop(void)
{
   e_HalWDT_Error_t l_eErrCode = HALWDT_ERROR_INVALID;
      
   return l_eErrCode;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static void vWDTHandler(void)
{
   // NVIC System Reset
   __nop();
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/

#endif /* #if (WDT_ENABLED == 1) */

