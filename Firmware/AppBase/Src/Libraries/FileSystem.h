/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2019 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 */
#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <string.h>
#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vFS_Init(void);
void vFS_Uninit(void);
void vFS_CreateFile(char* p_pchName);
void vFS_IsFileOpen(uint8_t * p_pu8IsOpen);
void vFS_CloseFile(void);
void vFS_Sync(void);
void vFS_Write(char * p_pchData, uint16_t p_u16Size);

#endif /* FILE_SYSTEM_H */

