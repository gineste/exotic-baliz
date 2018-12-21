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
#ifndef DEEPSLEEP_STATE_MACHINE_H
#define DEEPSLEEP_STATE_MACHINE_H

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
/**@brief Entry point of DeepSleep State.
 * @return None
 */
void vDeepSleep_Entry(void);
/**@brief Lowest Power Mode, do absolutely nothing.
 * @return None
 */
void vDeepSleep_Process(void);
/**@brief Exit point of DeepSleep State.
 * @return None
 */
void vDeepSleep_Exit(void);

#endif /* DEEPSLEEP_STATE_MACHINE_H */

