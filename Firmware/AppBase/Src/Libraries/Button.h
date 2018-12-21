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
 */
#ifndef BUTTON_H
#define BUTTON_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
 
#include "app_button.h"
 
/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Type definitions
 ************************************************************************/
typedef app_button_handler_t fpvButtonEvtHandler_t;

typedef enum _BUTTON_PULL_ {
   BUTTON_NOPULL = 0u,
   BUTTON_PULLDOWN,
   BUTTON_PULLUP
}e_ButtonPull_t;

typedef struct _BUTTON_CONFIG_ {
   uint8_t u8Pin;
   uint8_t u8ActiveState;
   e_ButtonPull_t ePull;
   uint32_t u32DetectionDelay;
   fpvButtonEvtHandler_t fpvHandler;
}s_ButtonConfig_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vButton_Init(s_ButtonConfig_t p_sContext);
void vButton_Enable(uint8_t p_u8Activate);
uint8_t u8Button_IsPushed(uint8_t p_u8Pin);

#endif // BUTTON_H

