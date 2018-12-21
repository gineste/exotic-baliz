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
#ifndef INIT_STATE_MACHINE_H
#define INIT_STATE_MACHINE_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
/**@brief Entry point of Init State.
 * @return None
 */
void vInit_Entry(void);
/**@brief Function to initialize State Machine/Sensors/SigFox/etc...
 * @return None
 */
void vInit_Process(void);
/**@brief Function to check if the system must go in Error or Operational
 * @return None
 */
void vInit_Check(void);
/**@brief Exit point of Init State.
 * @return None
 */
void vInit_Exit(void);

#endif /* INIT_STATE_MACHINE_H */

