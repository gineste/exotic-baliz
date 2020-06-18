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
 typedef enum _FILESYS_ERROR_CODE_ {
   FILESYS_ERROR_NONE,        /*! Success */
   FILESYS_ERROR_PARAM,       /*! Wrong parameter (Null pointer,...) */
	FILESYS_ERROR_SYNC,        /*! Error synchronizing file */
   FILESYS_ERROR_FAIL			/*! Fail (init, uninit,...) */
}e_FileSys_Error_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_FileSys_Error_t eFS_Init(void);
e_FileSys_Error_t eFS_Uninit(void);
e_FileSys_Error_t eFS_CreateFile(char* p_pchName);
void vFS_IsFileOpen(uint8_t * p_pu8IsOpen);
void vFS_CloseFile(void);
e_FileSys_Error_t eFS_Sync(void);
void vFS_Write(char * p_pchData, uint16_t p_u16Size);

#endif /* FILE_SYSTEM_H */

