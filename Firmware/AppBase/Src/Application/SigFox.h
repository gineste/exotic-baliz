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
#ifndef SIGFOX_H
#define SIGFOX_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include "GlobalDefs.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _SIGFOX_ERR_CODE_{
	SIGFOX_ERROR_NONE,
   SIGFOX_ERROR_PARAM,
	SIGFOX_ERROR_FAILED,
	SIGFOX_ERROR_TIMEOUT
}e_SigFox_ErrorCode_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vSigFox_Init(void);
void vSigFox_Process(void);
void vSigFox_DeviceInit(void);
void vSigFox_OnDemandSend(void);
uint8_t u8SigFox_IsDevicePACOk(void);
uint8_t u8SigFox_IsDeviceIDOk(void);
void vSigFox_TestRadio(uint8_t p_u8Enable, uint32_t p_u32Freq, uint8_t p_u8dBm);


#endif /* SIGFOX_H */

