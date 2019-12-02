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
 * Date:          03/01/2019 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Interrupt Handler GPIOTE 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "sdk_config.h"
#include "BoardConfig.h"
#include "GlobalDefs.h"

#include <nrf_drv_gpiote.h>
#include "board.h"

/* Self include */
#include "InterruptManager.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define MAX_INTERRUPT_NUM      GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef struct _INT_MNG_HANDLER_ {
   uint32_t u32Pin;
   fpvInterruptHandler_t fpvHandler;
}s_IntHandler_t;

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static inline void vInterruptHandler(nrf_drv_gpiote_pin_t p_u32Pin, nrf_gpiote_polarity_t p_eAction);
static inline e_IntMng_Error_t eFindIdx(uint32_t p_u32Pin,uint8_t * p_pu8Idx);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static uint8_t g_u8IntMngrInitialized = 0u;
static volatile s_IntHandler_t g_sIntHandler[MAX_INTERRUPT_NUM];
   
/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
e_IntMng_Error_t eIntMngr_Init(void)
{
   e_IntMng_Error_t l_eError = INT_MNG_ERROR_INIT;   
   uint32_t l_u32ErrCode = 0u;
   uint8_t l_u8Idx = 0u;
   
   l_u32ErrCode = nrf_drv_gpiote_init();
   
   if((l_u32ErrCode == NRF_SUCCESS) || (l_u32ErrCode == NRF_ERROR_INVALID_STATE))
   {
      for(l_u8Idx = 0u;l_u8Idx < MAX_INTERRUPT_NUM;l_u8Idx++)
      {
         g_sIntHandler[l_u8Idx].u32Pin = UINT8_MAX;
         g_sIntHandler[l_u8Idx].fpvHandler = NULL;
      }
      g_u8IntMngrInitialized = 1u;
      l_eError = INT_MNG_ERROR_NONE;
   }
   
   return l_eError;
}
   
e_IntMng_Error_t eIntMngr_Add(s_IntMng_Context_t p_sContext)
{
   e_IntMng_Error_t l_eError = INT_MNG_ERROR_INIT;
   uint32_t l_eErrorCode = 0u;
   uint8_t l_u8Idx = 0u;
   
   nrf_drv_gpiote_in_config_t l_sPinConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
   
   if(g_u8IntMngrInitialized == 1u)
   {
      if(   (p_sContext.fpvHandler != NULL)
         && (p_sContext.u32Pin < UINT8_MAX) )
      {
         l_sPinConfig.pull = (nrf_gpio_pin_pull_t)p_sContext.ePullMode;
         l_sPinConfig.sense = (nrf_gpiote_polarity_t)p_sContext.ePolarityDetection;
         
         if(eFindIdx(g_sIntHandler[l_u8Idx].u32Pin, &l_u8Idx) == INT_MNG_ERROR_NOT_FOUND)
         {  /* Find First Free */
            if(eFindIdx(UINT8_MAX, &l_u8Idx) != INT_MNG_ERROR_NONE)
            {
               l_eErrorCode = 1u;
            }
         }
         
         if(l_eErrorCode == 0u)
         {
            g_sIntHandler[l_u8Idx].u32Pin = p_sContext.u32Pin;
            g_sIntHandler[l_u8Idx].fpvHandler = p_sContext.fpvHandler;
            l_eErrorCode = nrf_drv_gpiote_in_init(p_sContext.u32Pin, &l_sPinConfig, vInterruptHandler);
            if(l_eErrorCode == 0u)
            {
               nrf_drv_gpiote_in_event_enable(p_sContext.u32Pin, true); 
               l_eError = INT_MNG_ERROR_NONE;
            }
            else
            {
               l_eError = INT_MNG_ERROR_STATE;
            }
         }
      }
      else
      {
         l_eError = INT_MNG_ERROR_PARAM;
      }
   }
   
   return l_eError;
}

e_IntMng_Error_t eIntMngr_Delete(uint32_t p_u32Pin)
{
   e_IntMng_Error_t l_eError = INT_MNG_ERROR_INIT;
   uint8_t l_u8Idx = 0u;
   
   if(g_u8IntMngrInitialized == 1u)
   {
      if(eFindIdx(p_u32Pin, &l_u8Idx) == INT_MNG_ERROR_NOT_FOUND)
      {  
         l_eError = INT_MNG_ERROR_PARAM;
      }
      else
      {
         g_sIntHandler[l_u8Idx].u32Pin = UINT8_MAX;
         g_sIntHandler[l_u8Idx].fpvHandler = NULL;
         nrf_drv_gpiote_in_uninit((nrf_drv_gpiote_pin_t)p_u32Pin);
         l_eError = INT_MNG_ERROR_NONE;
      }
   }
   
   return l_eError;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static inline void vInterruptHandler(nrf_drv_gpiote_pin_t p_u32Pin, nrf_gpiote_polarity_t p_eAction)
{
   uint8_t l_u8Idx = 0u;
   
   if(eFindIdx((uint32_t)p_u32Pin, &l_u8Idx) == INT_MNG_ERROR_NONE)
   {
      if(g_sIntHandler[l_u8Idx].fpvHandler != NULL)
      {
         (*g_sIntHandler[l_u8Idx].fpvHandler)(p_u32Pin, (e_IntMng_PolarityDetection_t)p_eAction);
      }
   }
}

static inline e_IntMng_Error_t eFindIdx(uint32_t p_u32Pin,uint8_t * p_pu8Idx)
{
   e_IntMng_Error_t l_eError = INT_MNG_ERROR_PARAM;
   uint8_t l_u8Idx = 0u;
   
   if(p_pu8Idx != NULL)
   {      
      while(   (g_sIntHandler[l_u8Idx].u32Pin != p_u32Pin) 
                  && (l_u8Idx < MAX_INTERRUPT_NUM) )
      {
         l_u8Idx++;
      }
      
      if(l_u8Idx < MAX_INTERRUPT_NUM)
      {
         (*p_pu8Idx) = l_u8Idx;
         l_eError = INT_MNG_ERROR_NONE;
      }
      else
      {
         l_eError = INT_MNG_ERROR_NOT_FOUND;
      }
   }
   
   return l_eError;
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


