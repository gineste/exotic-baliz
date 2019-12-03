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
#ifndef MAX1720X_H
#define MAX1720X_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _MAX1720X_ERROR_ {
   MAX1720X_ERROR_NONE = 0u,
   MAX1720X_ERROR_CONTEXT,
   MAX1720X_ERROR_INIT,
   MAX1720X_ERROR_PARAM,
   MAX1720X_ERROR_NOT_FOUND,
   MAX1720X_ERROR_COMM,
   MAX1720X_ERROR_BUSY
}e_MAX1720X_Error_t;

typedef struct _MAX1720X_CONTEXT_ {
   /* Function pointer to a read I2C transfer */
   uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   /* Function pointer to a write I2C transfer */
   uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
   /* Function pointer to a timer in ms */
   void (*fp_vDelay_ms)(uint32_t p_u32Timeout);
}s_MAX1720X_Context_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_MAX1720X_Error_t eMAX1720X_ContextSet(s_MAX1720X_Context_t p_sContext);
e_MAX1720X_Error_t eMAX1720X_Init(void);
e_MAX1720X_Error_t eMAX1720X_StatusGet(uint16_t * p_pu16Status);
e_MAX1720X_Error_t eMAX1720X_TemperatureGet(uint16_t * p_pu16Temp);
e_MAX1720X_Error_t eMAX1720X_VCellGet(uint16_t * p_pu16VCell);
e_MAX1720X_Error_t eMAX1720X_DebugWrite(uint16_t p_u16Register, uint16_t p_u16Data);
e_MAX1720X_Error_t eMAX1720X_DebugRead(uint16_t p_u16Register,  uint16_t * p_pu16Value);

#endif /* MAX1720X_H */

