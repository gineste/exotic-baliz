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
 * Date:          19 01 2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Main entry of the Bootloader firmware 
 *
 */

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include "boards.h"
#include "HAL/HAL_GPIO.h"

#include "nrf_bootloader.h"
#include "nrf_bootloader_info.h"
#include "nrf_bootloader_app_start.h"

/************************************************************************
 * Defines
 ************************************************************************/
//#define BOOTLOADER_BUTTON   (BSP_BUTTON_0)      /**< Button for entering DFU mode. */

/************************************************************************
 * Private type declarations
 ************************************************************************/

/************************************************************************
 * Private function declarations
 ************************************************************************/

/************************************************************************
 * Variable declarations
 ************************************************************************/
/* 
Without DFU
   IROM1 Start 0x23000;       Size 0x5D000
   IRAM1 Start 0x20003000;    Size 0xD000
With SDFU
   IROM1 Start 0x23000;       Size 0x55000
   IRAM1 Start 0x20003000;    Size 0xD000
*/
/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief main function of project 
 */
int main(void)
{
   uint32_t l_u32ErrCode;
   
   vHal_GPIO_Init();
   
   l_u32ErrCode = nrf_bootloader_init();
   APP_ERROR_CHECK(l_u32ErrCode);
   
   // Either there was no DFU functionality enabled in this project or the DFU module detected
   // no ongoing DFU operation and found a valid main application.
   // Boot the main application.
   nrf_bootloader_app_start(MAIN_APPLICATION_START_ADDR);

}

void app_error_handler_bare(uint32_t error_code)
{
   (void)error_code;
   NVIC_SystemReset();
}

/**@brief Implementation to use button press to enter bootloader
 */
bool nrf_dfu_button_enter_check(void)
{
#if (BUTTONS_NUMBER > 0)
   if (nrf_gpio_pin_read(BOOTLOADER_BUTTON) == 0)
   {
      return true;
   }
#endif

   return false;
}
/************************************************************************
 * Private functions
 ************************************************************************/

/************************************************************************
 * End Of File
 ************************************************************************/

 
