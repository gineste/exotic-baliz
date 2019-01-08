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
#ifndef MAX44009_H
#define MAX44009_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define MAX44009_COEF_MANTISSA   (float)0.045f

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/ 
typedef enum _MAX44009_ERROR_ {
   MAX44009_ERROR_NONE = 0u,
   MAX44009_ERROR_CONTEXT,
   MAX44009_ERROR_INIT,
   MAX44009_ERROR_PARAM,
   MAX44009_ERROR_NOT_FOUND,
   MAX44009_ERROR_COMM,
   MAX44009_ERROR_BUSY
}e_MAX44009_Error_t;

typedef enum _MAX44009_ADDRESS_ {
   MAX44009_ADDR1 = 0x4A,
   MAX44009_ADDR2 = 0x4B
}e_MAX44009_Addr_t;

typedef enum _MAX44009_INT_TIME_ {
   MAX44009_INT_TIME_800MS = 0u,
   MAX44009_INT_TIME_400MS,
   MAX44009_INT_TIME_200MS,
   MAX44009_INT_TIME_100MS,
   MAX44009_INT_TIME_50MS,
   MAX44009_INT_TIME_25MS,
   MAX44009_INT_TIME_12_5MS,
   MAX44009_INT_TIME_6_25MS
}e_MAX44009_IntegrationTime_t;

typedef enum _MAX44009_CONT_ {
   MAX44009_DEFAULT = 0u, /* Conversion every 800ms, Low Power Mode */
   MAX44009_CONTINUOUS
}e_MAX44009_Cont_t;

typedef enum _MAX44009_MODE_ {
   MAX44009_AUTOMATIC = 0u,
   MAX44009_MANUAL
}e_MAX44009_Mode_t;

typedef enum _MAX44009_BRIGHTNESSMODE_ {
   MAX44009_LOW_BRIGHTNESS = 0u,
   MAX44009_HIGH_BRIGHTNESS
}e_MAX44009_BrightessMode_t;

typedef void (*fp_vIntHandler_t)(uint8_t p_u8Type);

typedef struct _MAX44009_CONTEXT_ {
   /* Sensor Address */
   e_MAX44009_Addr_t eI2CAddress;
   /* Function pointer to a read I2C transfer */
   uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   /* Function pointer to a write I2C transfer */
   uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
   /* Function pointer to a timer in ms */
   void (*fp_vDelay_ms)(uint32_t p_u32Timeout);
}s_MAX44009_Context_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_MAX44009_Error_t eMAX44009_ContextSet(s_MAX44009_Context_t p_sContext);
e_MAX44009_Error_t eMAX44009_ConversionModeSet(e_MAX44009_Cont_t p_eConversion, 
                                          e_MAX44009_Mode_t p_eAcquisitionMode, 
                                          e_MAX44009_IntegrationTime_t p_eIntegration, 
                                          e_MAX44009_BrightessMode_t p_eBrightness);
e_MAX44009_Error_t eMAX44009_BrightnessRead(uint8_t * p_pau8RawData);
e_MAX44009_Error_t eMAX44009_BrightnessGet(uint32_t * p_pu32Brightness);
e_MAX44009_Error_t eMAX44009_InterruptCfg(uint8_t p_u8Activate, uint32_t p_u32HighThr, uint32_t p_u32LowThr, uint16_t p_u16TimeMs);
e_MAX44009_Error_t eMAX44009_InterruptStatusGet(uint8_t * p_pu8IntStatus);
e_MAX44009_Error_t eMAX44009_ThresholdGet(uint32_t * p_pu32ThrHigh, uint32_t * p_pu32ThrLow, uint16_t * p_pu16ThrTimer);
uint8_t u8MAX44009_IsAvailable(void);
 
#endif /* MAX44009_H */

