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
#ifndef CONNECTED_STATE_MACHINE_H
#define CONNECTED_STATE_MACHINE_H

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
/**@brief Entry point of Connected State.
 * @return None
 */
void vConnected_Entry(void);
/**@brief Once connected functionalities if different from operational.
 * @return None
 */
void vConnected_Process(void);
/**@brief Function to check if device should go in Operational, Error or DeepSleep State.
 * @return None
 */
void vConnected_Check(void);
/**@brief Exit point of Connected State.
 * @return None
 */
void vConnected_Exit(void);

#endif /* CONNECTED_STATE_MACHINE_H */

