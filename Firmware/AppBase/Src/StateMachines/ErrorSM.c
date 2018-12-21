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
#include <nrf_nvic.h>

#include "ErrorSM.h"

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
/**@brief Entry point of Error State.
 * @return None
 */
void vError_Entry(void)
{
   
}
/**@brief Function in case of error (any kind of).
 * @return None
 */
void vError_Process(void)
{
   /* Report it if possible (SigFox or BLE) */
}
/**@brief Function to check if we can go to Init State.
 * @return None
 */
void vError_Check(void)
{
//   // TODO : what todo ?
//   vMSM_StateMachineSet(MSM_INIT);
   sd_nvic_SystemReset();
}
/**@brief Exit point of Error State.
 * @return None
 */
void vError_Exit(void)
{
//   g_u8Error = 0u;
//   g_u8IsInitialized = 0u;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

