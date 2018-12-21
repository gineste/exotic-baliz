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
 */
#ifndef POWER_UP_STATE_MACHINE_H
#define POWER_UP_STATE_MACHINE_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include "MainStateMachine.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
/**@brief At Power up of board, initialize required modules/registers of µC that are needed
 *        in order to launch others code functions.
 * @return None
 */
void vPowerUp_Process(void);
/**@brief Check point of PowerUp State.
 * @return None
 */
void vPowerUp_Check(void);


#endif /* POWER_UP_STATE_MACHINE_H */

