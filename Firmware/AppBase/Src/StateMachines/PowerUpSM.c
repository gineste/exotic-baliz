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
 * Date:          10/10/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Sub State Machine Module
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "BoardConfig.h"
#include "board.h"

#include "HAL/HAL_GPIO.h"
#include "HAL/HAL_I2C.h"
#include "HAL/HAL_SPI.h"
#include "HAL/HAL_RTC.h"

#include "BLE/BLE_Application.h"

#include "Libraries/Buzzer.h"
#include "Libraries/SimpleLED.h"

#include "SigFox.h"


#include "PowerUpSM.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief At Power up of board, initialize required modules/registers of µC that are needed
 *        in order to launch others code functions.
 * @return None
 */
void vPowerUp_Process(void)
{
   s_SimpleLED_Context_t l_sLEDContext = {
      .u32PinR = LEDR_PIN,
      .u32PinG = LEDG_PIN,
      .u32PinB = LEDB_PIN,
      .fp_vPinClear_t = &vHal_GPIO_Clear,
      .fp_vPinSet_t = &vHal_GPIO_Set,
   };
   
   /* LED */
   vSimpleLED_Init(l_sLEDContext);
   /* Set LED to Power Up */
//   vSimpleLED_RED();
      vSimpleLED_BLACK();
      
   /* Buzzer */
   vBuzzerInit();

   /* RTC */
   vHal_RTC_Init();   
   
   /* SigFox */
   vSigFox_Init();
   
   /* Stack BLE */
   vBLE_Init();
}

/**@brief Check point of PowerUp State.
 * @return None
 */
void vPowerUp_Check(void)
{
   /* Just go in Init State */
   vMSM_StateMachineSet(MSM_INIT);
}
/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

