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
 * Date:          10 08 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Main entry of the SuperFrog firmware 
 *
 */

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>

//#include "nrf_soc.h"
#include "nrf_pwr_mgmt.h"
#include "app_error.h"

#include "MainStateMachine.h"

/*************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Private type declarations
 ************************************************************************/

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vPowerManageInit(void);
static void vPowerManageProcess(void);

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
   vMSM_Init();
   vPowerManageInit();
   
   /* Enter main loop process */
   for (;;)
   {
      vMSM_Process();
      /* Wake Up by event (WFE)*/
      vPowerManageProcess();
   }
}

void HardFault_Handler(void)
{
	while(1);
}
void NMI_Handler(void)
{
	while(1);
}
void MemoryManagement_Handler(void)
{
	while(1);
}
void BusFault_Handler(void)
{
	while(1);
}
void UsageFault_Handler(void)
{
	while(1);
}

void FPU_IRQHandler(void)
{
   /* Clear exceptions and PendingIRQ from the FPU unit */
   __set_FPSCR(__get_FPSCR()  & ~(0x0000009F ));
  (void) __get_FPSCR();
   NVIC_ClearPendingIRQ(FPU_IRQn);
}

/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief Function for the Power manager initialization.
 */
static void vPowerManageInit(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for the Power manager.
 */
static void vPowerManageProcess(void)
{
   nrf_pwr_mgmt_run();
//   /* Clear exceptions and PendingIRQ from the FPU unit */
//   __set_FPSCR(__get_FPSCR()  & ~(0x0000009F ));      
//   (void) __get_FPSCR();
//   NVIC_ClearPendingIRQ(FPU_IRQn);
//   
//    uint32_t l_u32ErrCode = sd_app_evt_wait();
//    APP_ERROR_CHECK(l_u32ErrCode);
}

/************************************************************************
 * End Of File
 ************************************************************************/

 
