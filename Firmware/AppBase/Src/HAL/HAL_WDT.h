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
 */
#ifndef HEADER_TEMPLATE_H
#define HEADER_TEMPLATE_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#if (WDT_ENABLED == 1)

#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _HAL_WDT_ERRORS_ {
   HALWDT_ERROR_NONE,
   HALWDT_ERROR_PARAM,
   HALWDT_ERROR_BUSY,
   HALWDT_ERROR_INIT,
   HALWDT_ERROR_INVALID
}e_HalWDT_Error_t;

typedef void (*fpv_WatchdogHandler_t)(void);

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_HalWDT_Error_t eHal_WDT_Init(uint32_t p_u32TimeOut, fpv_WatchdogHandler_t p_sWatchdogHandler);
e_HalWDT_Error_t eHal_WDT_Start(void);
e_HalWDT_Error_t eHal_WDT_Reload(void);
e_HalWDT_Error_t eHal_WDT_Stop(void);

#endif /* HEADER_TEMPLATE_H */

#endif /* #if (WDT_ENABLED == 1) */

