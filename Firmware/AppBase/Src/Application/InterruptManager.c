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

#include "BoardConfig.h"
#include "GlobalDefs.h"

#include <nrf_drv_gpiote.h>
#include "board.h"

/* Self include */
#include "InterruptManager.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vInterruptHandler(nrf_drv_gpiote_pin_t p_u32Pin, nrf_gpiote_polarity_t p_eAction);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vIntManagerInit(void)
{
   uint32_t l_u32ErrCode = 0u;
   nrf_drv_gpiote_in_config_t l_sPinConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
   
   l_u32ErrCode = nrf_drv_gpiote_init();
   
   if((l_u32ErrCode == 0u) || (l_u32ErrCode == NRF_ERROR_INVALID_STATE))
   {
   #if (EN_ADXL362 == 1)
   /*	l_sPinConfig.is_watcher = false;
      l_sPinConfig.hi_accuracy = false;
      l_sPinConfig.pull = NRF_GPIO_PIN_NOPULL;
      l_sPinConfig.sense = NRF_GPIOTE_POLARITY_TOGGLE;*/
      
      l_u32ErrCode = nrf_drv_gpiote_in_init(ADXL_INT2, &l_sPinConfig, vInterruptHandler);
      nrf_drv_gpiote_in_event_enable(ADXL_INT2, true);   
   #endif /* NO_ADXL362 */

   #if (EN_MAX44009 == 1)
    	l_sPinConfig.is_watcher = false;
      l_sPinConfig.hi_accuracy = false;
      l_sPinConfig.pull = NRF_GPIO_PIN_PULLUP;
      l_sPinConfig.sense = NRF_GPIOTE_POLARITY_HITOLO;
      l_u32ErrCode = nrf_drv_gpiote_in_init(MAX44009_INT, &l_sPinConfig, vInterruptHandler);
      nrf_drv_gpiote_in_event_enable(MAX44009_INT, true);   
   #endif /* NO_MAX44009 */

   #if (EN_LSM6DSL == 1)
   	l_sPinConfig.is_watcher = false;
      l_sPinConfig.hi_accuracy = false;
      l_sPinConfig.pull = NRF_GPIO_PIN_NOPULL;
      l_sPinConfig.sense = NRF_GPIOTE_POLARITY_TOGGLE;
      l_u32ErrCode = nrf_drv_gpiote_in_init(LSM6_INT1, &l_sPinConfig, vInterruptHandler);
      nrf_drv_gpiote_in_event_enable(LSM6_INT1, true);
      l_u32ErrCode = nrf_drv_gpiote_in_init(LSM6_INT2, &l_sPinConfig, vInterruptHandler);
      nrf_drv_gpiote_in_event_enable(LSM6_INT2, true);   
   #endif /* NO_LSM6DSL */

   #if (EN_LIS2MDL == 1)
      l_u32ErrCode = nrf_drv_gpiote_in_init(LIS2_INT, &l_sPinConfig, vInterruptHandler);
      nrf_drv_gpiote_in_event_enable(LIS2_INT, true);
   #endif /* NO_LIS2MDL */

   #if (EN_ST25DV == 1)
      l_u32Errcode = nrf_drv_gpiote_in_init(ST25DV_GPO_INT, &l_sPinConfig, vInterruptHandler);
      nrf_drv_gpiote_in_event_enable(ST25DV_GPO_INT, true);
   #endif /* NO_ST25DV */

   #if (EN_LTC2943 == 1) 
   #endif /* NO_LTC2943 */
   }
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static void vInterruptHandler(nrf_drv_gpiote_pin_t p_u32Pin, nrf_gpiote_polarity_t p_eAction)
{
   switch(p_u32Pin)
   {
   #if (EN_ADXL362 == 1)
      case ADXL_INT2:
         PRINT_DEBUG("$RSL,INT+1+ADXL362+%d\n",(uint8_t)p_eAction);
      break;
   #endif /* NO_ADXL362 */

   #if (EN_MAX44009 == 1)
      case MAX44009_INT:
         PRINT_DEBUG("$RSL,INT+1+MAX44009+%d\n",(uint8_t)p_eAction);
      break;
   #endif /* NO_MAX44009 */

   #if (EN_LSM6DSL == 1)
      case LSM6_INT1:
         PRINT_DEBUG("$RSL,INT+1+LSM6_1+%d\n",(uint8_t)p_eAction);
      break;
      case LSM6_INT2:
         PRINT_DEBUG("$RSL,INT+1+LSM6_2+%d\n",(uint8_t)p_eAction);
      break;
   #endif /* NO_LSM6DSL */

   #if (EN_LIS2MDL == 1)
      case LIS2_INT:
         PRINT_DEBUG("$RSL,INT+1+LIS2+%d\n",(uint8_t)p_eAction);
      break;
   #endif /* NO_LIS2MDL */

   #if (EN_ST25DV == 1)
      case ST25DV_GPO_INT:
         PRINT_DEBUG("$RSL,INT+1+ST25DV+%d\n",(uint8_t)p_eAction);
      break;
   #endif /* NO_ST25DV */

   #if (EN_LTC2943 == 1) 
   #endif /* NO_LTC2943 */

      default: /* Unknown Pin Interrupt */
         PRINT_FAST("$RSL,INT+0+UNK\n");
      break;
   }
}


/****************************************************************************************
 * End Of File
 ****************************************************************************************/


