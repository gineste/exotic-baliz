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
#ifndef AXSIGFOX_H
#define AXSIGFOX_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include "Libraries/AT.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define AXSF_DEV_ID_SIZE (9u)
#define AXSF_DEV_PAC_SIZE (17u)

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _AXSIGFOX_ERR_CODE_{
	AXSIGFOX_ERROR_NONE,
   AXSIGFOX_ERROR_PARAM,
	AXSIGFOX_ERROR_SUCCEED,
	AXSIGFOX_ERROR_FAILED,
	AXSIGFOX_ERROR_TIMEOUT,
}e_AXSigFox_ErrorCode_t;

typedef fp_vATCallback_t fp_AXSFCallback_t;
typedef e_AT_Commands_t e_AXSFCommands_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vAXSigFox_Init(void);
void vAXSigFox_UpdateFrame(const uint8_t p_u8Data);
e_AXSigFox_ErrorCode_t eAXSigFox_SendCommand(e_AXSFCommands_t p_eCmd, uint8_t * p_pu8Bytes, uint8_t p_u8Size, fp_AXSFCallback_t p_pfpCallback);

void vAXSigFox_DeepSleepModeSet(void);
void vAXSigFox_WakeUpFromDeepSleep(void);
void vAXSigFox_SleepModeSet(void);
void vAXSigFox_WakeUpFromSleep(void);

void vAXSigFox_DeviceIDRead(void);
void vAXSigFox_DevicePACRead(void);
void vAXSigFox_DeviceInfoRead(void);
void vAXSigFox_DeviceTemperatureRead(void);

void vAXSigFox_DevicePacGet(uint8_t * p_pu8Pac, uint8_t * p_u8Size);
void vAXSigFox_DeviceIdGet(uint8_t * p_pu8Id, uint8_t * p_u8Size);

#endif /* AXSIGFOX_H */

