/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 */
#ifndef ST25DV_H
#define ST25DV_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define ST25DV_MSK_GPO_ON_USER            (uint8_t)0x01
#define ST25DV_MSK_GPO_ON_ACTIVITY        (uint8_t)0x02
#define ST25DV_MSK_GPO_ON_INTERRUPT       (uint8_t)0x04
#define ST25DV_MSK_GPO_ON_FIELD_CHANGE    (uint8_t)0x08
#define ST25DV_MSK_GPO_ON_PUT_MSG         (uint8_t)0x10
#define ST25DV_MSK_GPO_ON_GET_MSG         (uint8_t)0x20
#define ST25DV_MSK_GPO_ON_WRITE           (uint8_t)0x40
#define ST25DV_MSK_GPO_ENABLED            (uint8_t)0x80

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _ST25DV_ERROR_ {
   ST25DV_ERROR_NONE = 0u,
   ST25DV_ERROR_READ,
   ST25DV_ERROR_WRITE,
   ST25DV_ERROR_PARAM,
   ST25DV_ERROR_BUSY,
   ST25DV_ERROR_ACCESS,
   ST25DV_ERROR_COMM,
   ST25DV_ERROR_FAILED,
}e_ST25DV_Error_t;

typedef enum _ST25DV_EEPROM_SIZE_ {
   ST25DV_EEPROM_SIZE_64K = 0u,
   ST25DV_EEPROM_SIZE_16K,
   ST25DV_EEPROM_SIZE_04K,
}e_ST25DV_EEPROM_Size_t;

typedef struct _ST25DV_CONTEXT_ {
   /* Function pointer for a read I2C transfer */
   uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
   /* Function pointer for a write I2C transfer */
   uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   /* Function pointer to a timer in ms */
   void (*fp_vDelay_ms)(uint32_t p_u32Timeout); 
   /* Config */
   e_ST25DV_EEPROM_Size_t eEepromSize;
}s_ST25DV_Context_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_ST25DV_Error_t eST25DV_ContextSet(s_ST25DV_Context_t p_sContext);
   
e_ST25DV_Error_t eST25DV_GPOConfigure(uint8_t p_u8MskGPO);
e_ST25DV_Error_t eST25DV_PasswordSet(uint64_t p_u64Pass);

uint8_t u8ST25DV_IsAvailable(void);

#endif /* ST25DV_H */

