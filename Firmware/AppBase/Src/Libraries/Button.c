/* 
 *  ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 * (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *  ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 * (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Date:			   26/01/2018 (dd MM YYYY)
 * Author:			Yoann Rebischung
 * Description:   Buttpn library 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "BoardConfig.h"

#include "board.h"
#include "app_button.h"
#include "app_error.h"

#include "GlobalDefs.h"

#include "Button.h"

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Private type declarations
 ************************************************************************/
 
/************************************************************************
 * Private function declarations
 ************************************************************************/
#if (ENABLE_BUTTON == 1)
   static void vButtonHandler(uint8_t p_u8Pin, uint8_t p_u8Action);
#endif
uint8_t g_u8ButtonState = 0u;

/************************************************************************
 * Variable declarations
 ************************************************************************/
#if (ENABLE_BUTTON == 1)
//The array must be static because a pointer to it will be saved in the button handler module.
static app_button_cfg_t g_sButtonCfg[1u] = {
{
   .pin_no = BP1, 
   .active_state = false, 
   .pull_cfg = (nrf_gpio_pin_pull_t)BUTTON_NOPULL, 
   .button_handler = vButtonHandler
}};
#endif
   
/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief  vButtonInit : Initialization of the Button.
 * @return None.
 */
void vButton_Init(s_ButtonConfig_t p_sContext)
{   
#if (ENABLE_BUTTON == 1)
   uint32_t l_u32ErrCode = 0u;
   
   g_sButtonCfg[0u].pin_no = p_sContext.u8Pin;
   g_sButtonCfg[0u].active_state =(bool) p_sContext.u8ActiveState;
   g_sButtonCfg[0u].pull_cfg = (nrf_gpio_pin_pull_t)p_sContext.ePull;
   g_sButtonCfg[0u].button_handler = p_sContext.fpvHandler;
   
   l_u32ErrCode = app_button_init(g_sButtonCfg, 1u, p_sContext.u32DetectionDelay);
   APP_ERROR_CHECK(l_u32ErrCode);
#endif
}

/**@brief  vButtonEnable : Enable or not button interrupt.
 * @param[in] p_u8Activate
 * @return None.
 */
void vButton_Enable(uint8_t p_u8Activate)
{
#if (ENABLE_BUTTON == 1)
   if(p_u8Activate == 1u)
   {
      (void)app_button_enable();
   }
   else
   {
      (void)app_button_disable();
   }
   #endif
}

uint8_t u8Button_IsPushed(uint8_t p_u8Pin)
{
#if (ENABLE_BUTTON == 1)
   return (app_button_is_pushed(p_u8Pin) == true)?1u:0u;
#else
   return 0u;
#endif
}


/************************************************************************
 * Private functions
 ************************************************************************/
#if (ENABLE_BUTTON == 1)
static void vButtonHandler(uint8_t p_u8Pin, uint8_t p_u8Action)
{
   if(p_u8Action == 1u)
   {
      PRINT_FAST("Button Pressed\n");        
   }
   else
   {
      PRINT_FAST("Button Released\n");     
   }
}
#endif

