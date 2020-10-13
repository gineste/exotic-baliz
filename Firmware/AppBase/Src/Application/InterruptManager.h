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
#ifndef INTERRUPT_MANAGER_H
#define INTERRUPT_MANAGER_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include "HAL/HAL_GPIO.h"
#include "nrf_gpiote.h"
/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _INT_MNG_ERROR_CODE_ {
   INT_MNG_ERROR_NONE = 0u,
   INT_MNG_ERROR_INIT,
   INT_MNG_ERROR_PARAM,
   INT_MNG_ERROR_STATE,
   INT_MNG_ERROR_NOT_FOUND,
   INT_MNG_ERROR_FULL,
}e_IntMng_Error_t;

typedef enum
{
  INT_POL_DTCT_LOTOHI = NRF_GPIOTE_POLARITY_LOTOHI,       ///<  Low to high.
  INT_POL_DTCT_HITOLO = NRF_GPIOTE_POLARITY_HITOLO,       ///<  High to low.
  INT_POL_DTCT_TOGGLE = NRF_GPIOTE_POLARITY_TOGGLE        ///<  Toggle.
}e_IntMng_PolarityDetection_t;

typedef void (*fpvInterruptHandler_t)(uint32_t p_u32IntPin, e_IntMng_PolarityDetection_t p_ePolarity); 

typedef struct _INT_MNG_CONTEXT_ {
   uint32_t u32Pin;
   e_HalGPIO_PinPull_t ePullMode;
   e_IntMng_PolarityDetection_t ePolarityDetection;
   fpvInterruptHandler_t fpvHandler;
}s_IntMng_Context_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_IntMng_Error_t eIntMngr_Init(void);
e_IntMng_Error_t eIntMngr_Add(s_IntMng_Context_t p_sContext);
e_IntMng_Error_t eIntMngr_Delete(uint32_t p_u32Pin);
e_IntMng_Error_t eIntMngr_DisableAllInterrupt(void);

#endif /* INTERRUPT_MANAGER_H */

