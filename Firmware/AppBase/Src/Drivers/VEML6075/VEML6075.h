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
#ifndef VEML6075_H
#define VEML6075_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _VEML6075_ERROR_CODE_ 
{
   VEML6075_ERROR_NONE = 0u,
   VEML6075_ERROR_INIT,
   VEML6075_ERROR_PARAM,
   VEML6075_ERROR_CONTEXT,
   VEML6075_ERROR_UVA,
   VEML6075_ERROR_UVB,
   VEML6075_ERROR_UV_INDEX
}e_VEML6075_ErrCode_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vVEML6075_Init(void);
void vVEML6075_ShutDown(void);
void vVEML6075_PowerUp(void);
void vVEML6075_Configure(void);
void vVEML6075_IDRead(void);
void vVEML6075_PollingProcess(void);
uint8_t u8VEML6075_IsAvailable(void);
e_VEML6075_ErrCode_t eVEML6075_UVIndexGet(uint8_t * p_pu8UVIndex);

#endif /* VEML6075_H */


