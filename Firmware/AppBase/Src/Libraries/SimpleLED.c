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
 * Date:          04/12/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   LED interface to generate on/off etc
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include "BoardConfig.h"
#include "GlobalDefs.h"
#include "board.h"

#include "HAL/HAL_Timer.h"

#include "SimpleLED.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define SIMPLE_LED_NUMBER  3u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
#if (ENABLE_LED == 1)
   static void vEffectTimeOutHandler(void * p_vContext);
   static void vColorSet(e_SimpleLED_Color_t p_eColor);
#endif

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
#if (ENABLE_LED == 1)
   static uint8_t g_u8Active = 0u;
   static s_SimpleLED_Effect_t g_sCurrentLedEffect;
   static uint8_t g_u8SimpleLEDInit = 0u;
   HAL_TIMER_DEF(SimpleLED_TimerIdx);    
   s_SimpleLED_Context_t g_sSimpleLedContext = {
      .u32PinR = UINT32_MAX,
      .u32PinG = UINT32_MAX,
      .u32PinB = UINT32_MAX,
      .fp_vPinClear_t = NULL,
      .fp_vPinSet_t = NULL
   };
   static uint32_t g_au32LedList[SIMPLE_LED_NUMBER];
   e_SimpleLED_Color_t g_eLastColor = LED_BLACK;
#endif

   
/************************************************************************
 * Public functions
 ************************************************************************/
/**@brief  vLED_Init : Initialization of the Led management.
 * @param[in]  p_eContext Led Context
 * @return None.
 */
void vSimpleLED_Init(s_SimpleLED_Context_t p_eContext)
{
#if (ENABLE_LED == 1)
   if(   (p_eContext.fp_vPinClear_t != NULL)
      && (p_eContext.fp_vPinSet_t != NULL) )
   {  /* Function pointer assigned */
      g_sSimpleLedContext.fp_vPinClear_t = p_eContext.fp_vPinClear_t;
      g_sSimpleLedContext.fp_vPinSet_t = p_eContext.fp_vPinSet_t;
      /* Pin assigned */
      g_sSimpleLedContext.u32PinR = p_eContext.u32PinR;
      g_sSimpleLedContext.u32PinG = p_eContext.u32PinG;
      g_sSimpleLedContext.u32PinB = p_eContext.u32PinB;
      
      g_au32LedList[0u] = g_sSimpleLedContext.u32PinR;
      g_au32LedList[1u] = g_sSimpleLedContext.u32PinG;
      g_au32LedList[2u] = g_sSimpleLedContext.u32PinB;
      
      if(g_u8SimpleLEDInit == 0u)
      {  /* Create Timer */
         if(eHal_Timer_Create(&SimpleLED_TimerIdx, HAL_TIMER_MODE_SINGLE_SHOT, vEffectTimeOutHandler) == HAL_TIMER_ERROR_NONE)
         {
            g_u8SimpleLEDInit = 1u;
         }
      }
   }   
#endif
}

/**@brief  vSimpleLED_EffectSet : Change the effect color of the RGB Leds.
 * @param[in]  p_eEffect the new color effect to apply.
 * @return None.
 */
void vSimpleLED_EffectSet(s_SimpleLED_Effect_t p_eEffect)
{
#if (ENABLE_LED == 1)
   if(g_u8SimpleLEDInit == 1u)
   {
      g_sCurrentLedEffect = p_eEffect;
      /* Stop it just in case */
      (void)eHal_Timer_Stop(SimpleLED_TimerIdx);
      
      if(eHal_Timer_Start(SimpleLED_TimerIdx, g_sCurrentLedEffect.u32TimeOn) == HAL_TIMER_ERROR_NONE)
      {         
         vColorSet(g_sCurrentLedEffect.eColorOn);
         g_u8Active = 1u;
      }
   }
#endif
}

/**@brief  vSimpleLED_Off : Switch off LEDS.
 * @return None.
 */
void vSimpleLED_Off(void)
{  /* Switch Off all Leds in order to not have strange behavior */ 
   //LED_RED_OFF();
	//LED_GREEN_OFF();
	//LED_BLUE_OFF();
   vSimpleLED_BLACK();
}
  
/**@brief  vSimpleLED_EffectStop : Stop current LED Effect.
 * @return None.
 */
void vSimpleLED_EffectStop(void)
{   
#if (ENABLE_LED == 1)
   g_sCurrentLedEffect.u16Counter = 0u;
   vSimpleLED_PreviousColorSet();
#endif
}

/**@brief  vSimpleLED_PreviousColorSet : Set previous LED color.
 * @return None.
 */
void vSimpleLED_PreviousColorSet(void)
{
#if (ENABLE_LED == 1)
   /* Activate LEDs and Stop Timer */
   vColorSet(g_eLastColor);
   /* Stop secure */
   (void)eHal_Timer_Stop(SimpleLED_TimerIdx);
   g_u8Active = 0u;
#endif
}

/**@brief  vSimpleLED_ColorSet : Set LED color.
 * @param[in] p_eColor
 * @return None.
 */
void vSimpleLED_ColorSet(e_SimpleLED_Color_t p_eColor)
{
   if(p_eColor < LED_DEFAULTCOLOR)
   {
      vColorSet(p_eColor);      
   }
}

/************************************************************************
 * Private functions
 ************************************************************************/
#if (ENABLE_LED == 1)
static void vEffectTimeOutHandler(void * p_vContext)
{
   if(g_sCurrentLedEffect.u16Counter != 0u)
   {
      if(g_u8Active == 0u)
      {
         vColorSet(g_sCurrentLedEffect.eColorOn);      
         (void)eHal_Timer_Start(SimpleLED_TimerIdx, g_sCurrentLedEffect.u32TimeOn);
         g_u8Active = 1u;
      }
      else
      {
         vColorSet(g_sCurrentLedEffect.eColorOff); 
         (void)eHal_Timer_Start(SimpleLED_TimerIdx, g_sCurrentLedEffect.u32TimeOff);
         g_u8Active = 0u;
         g_sCurrentLedEffect.u16Counter--;
      }
   }
   else
   {
      vColorSet(g_eLastColor);
   }
}

/**@brief  vColorSet : Change the color of the RGB Leds.
 * @param[in]  p_eColor the new color to apply.
 * @return None.
 */
static void vColorSet(e_SimpleLED_Color_t p_eColor)
{
#if (ENABLE_LED == 1)
   uint32_t l_u32ColorMask = 0u;   
   uint8_t l_u8Idx = 0u;
   uint32_t l_u32Led = 0u;
   uint32_t l_u32RedMsk = (1u<<g_sSimpleLedContext.u32PinR);
   uint32_t l_u32GreenMsk = (1u<<g_sSimpleLedContext.u32PinG);
   uint32_t l_u32BlueMsk = (1u<<g_sSimpleLedContext.u32PinB);
   
   /* Switch Off all Leds in order to not have strange behavior */ 
   LED_RED_OFF();
	LED_GREEN_OFF();
	LED_BLUE_OFF();
      
   switch(p_eColor)
	{
		case LED_RED:
         l_u32ColorMask = l_u32RedMsk;
      break;
		case LED_GREEN:
         l_u32ColorMask = l_u32GreenMsk;
      break;
		case LED_BLUE:
         l_u32ColorMask = l_u32BlueMsk;
      break;
		case LED_YELLOW:
         l_u32ColorMask = l_u32RedMsk | l_u32GreenMsk;
      break;
		case LED_MAGENTA:
         l_u32ColorMask = l_u32RedMsk | l_u32BlueMsk;
      break;
		case LED_CYAN:
         l_u32ColorMask = l_u32GreenMsk | l_u32BlueMsk;
      break;
		case LED_WHITE:
         l_u32ColorMask = l_u32RedMsk | l_u32GreenMsk | l_u32BlueMsk;
      break;
      case LED_BLACK:
		default:		// Led Off		
         l_u32ColorMask = 0u;         
         break;
	}   
   
   
   for(l_u8Idx = 0u;l_u8Idx < SIMPLE_LED_NUMBER;l_u8Idx++)
   {
      l_u32Led = (l_u32ColorMask >> g_au32LedList[l_u8Idx]) & 0x01;
      if(l_u32Led == 1u)
      {
         if(g_sSimpleLedContext.fp_vPinClear_t != NULL)
         {
            (*g_sSimpleLedContext.fp_vPinClear_t)(g_au32LedList[l_u8Idx]);
         }
      }
      /*else
      {
         (*g_sSimpleLedContext.fp_vPinSet_t)(g_au32LedList[l_u8Idx]);
      }*/
   }  
   
#endif
}

#endif

