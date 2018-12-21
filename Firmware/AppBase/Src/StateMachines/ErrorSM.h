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
#ifndef ERROR_STATE_MACHINE_H
#define ERROR_STATE_MACHINE_H

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
/**@brief Entry point of Error State.
 * @return None
 */
void vError_Entry(void);
/**@brief Function in case of error (any kind of).
 * @return None
 */
void vError_Process(void);
/**@brief Function to check if we can go to Init State.
 * @return None
 */
void vError_Check(void);
/**@brief Exit point of Error State.
 * @return None
 */
void vError_Exit(void);

#endif /* ERROR_STATE_MACHINE_H */

