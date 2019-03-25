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

#include "board.h"
#include "BoardConfig.h"

/* Application include */
#include "SensorManager.h"

/* Libraries includes */
#include "Libraries/Button.h"
#include "Libraries/Buzzer.h"
#include "Libraries/SimpleLED.h"

/* BLE include */
#include "BLE/BLE_Application.h"

/* Driver include */
#include "LTC2943/LTC2943.h"
#include "LIS2MDL/LIS2MDL.h"
#include "LSM6DSL/LSM6DSL.h"

/* HAL includes */
#include "HAL/HAL_GPIO.h"
#include "HAL/HAL_SPI.h"
#include "HAL/HAL_I2C.h"
#include "HAL/HAL_Timer.h"

/* Self include */
#include "DeepSleepSM.h"

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
/**@brief Entry point of DeepSleep State.
 * @return None
 */
void vDeepSleep_Entry(void)
{
//   vButton_Enable(0u);
   /* Stop Advertising (if operational)*/
   vBLE_AdvertisingStop();
   /* Set LED to DeepSleep */
   vSimpleLED_BLACK();
   
//#if (EN_LTC2943 == 1)  
//   eLTC2943_PowerDown(1u);
//#endif
   
   (void)eHal_Timer_StopAll();
   
   /* All registers of µC (ADC/SPI/I2C/UART) */
   vHal_I2C_Uninit();
   vHal_SPI_Uninit();
   vBuzzerUninit();
   
   /* Shutdown GPS Power */

   //vBLE_SoftDeviceDisable();
}

/**@brief Lowest Power Mode, do absolutely nothing.
 * @return None
 */
void vDeepSleep_Process(void)
{
}

/**@brief Exit point of DeepSleep State.
 * @return None
 */
void vDeepSleep_Exit(void)
{
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

