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
#ifndef LTC2943_H
#define LTC2943_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define ADC_SENSOR_VOLTAGE_MASK      (uint8_t)0x01
#define ADC_SENSOR_CURRENT_MASK      (uint8_t)0x02
#define ADC_SENSOR_TEMP_MASK         (uint8_t)0x04

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _LTC2943_ERROR_ {
   LTC2943_ERROR_NONE = 0u,
   LTC2943_ERROR_CONTEXT,
   LTC2943_ERROR_PARAM,
   LTC2943_ERROR_BUSY,
   LTC2943_ERROR_NOT_READY,
   LTC2943_ERROR_COMM,
   LTC2943_ERROR_NOT_FOUND
}e_LTC2943_Error_t;

typedef enum _LTC2943_ADC_MODE_ {
   LTC2943_ADC_MODE_SLEEP = 0u,
   LTC2943_ADC_MODE_MANUAL,
   LTC2943_ADC_MODE_SCAN,
   LTC2943_ADC_MODE_AUTOMATIC
}e_LTC2943_ADCMode_t;

typedef enum _LTC2943_ALERT_CFG_ {
   LTC2943_ALERT_DISABLED = 0u,
   LTC2943_ALERT_CHARGE_COMPLETE_MODE,
   LTC2943_ALERT_ENABLED,
}e_LTC2943_AlertCfg_t;

typedef struct _LTC2943_CONTEXT_ {
   /* Function pointer to a read I2C transfer */
   uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
   /* Function pointer to a write I2C transfer */
   uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   /* Function pointer to a timer in ms */
   void (*fp_vDelay_ms)(uint32_t p_u32Timeout);   
}s_LTC2943_Context_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_LTC2943_Error_t eLTC2943_ContextSet(s_LTC2943_Context_t p_sContext);
e_LTC2943_Error_t eLTC2943_PowerDown(uint8_t p_u8Shutdown);
e_LTC2943_Error_t eLTC2943_ADCModeSet(e_LTC2943_ADCMode_t p_eADCMode);
e_LTC2943_Error_t eLTC2943_BatteryCapacitySet(uint16_t p_u16BatteryCapacitymAh);
e_LTC2943_Error_t eLTC2943_AlertCfgSet(e_LTC2943_AlertCfg_t p_eAlert);
e_LTC2943_Error_t eLTC2943_TemperatureThresholdSet(uint16_t p_u16TemperatureHigh, uint16_t p_u16TemperatureLow);
e_LTC2943_Error_t eLTC2943_ChargeThresholdSet(uint16_t p_u16ChargeHigh, uint16_t p_u16ChargeLow);
e_LTC2943_Error_t eLTC2943_VoltageThresholdSet(uint16_t p_u16VoltageHigh, uint16_t p_u16VoltageLow);

e_LTC2943_Error_t eLTC2943_SensorRead(uint8_t p_u8SensorMask);
e_LTC2943_Error_t eLTC2943_TemperatureGet(int16_t * p_ps16Temperature);
e_LTC2943_Error_t eLTC2943_AccumulatedChargeGet(int16_t * p_ps16AccuCharge);
e_LTC2943_Error_t eLTC2943_VoltageGet(uint16_t * p_pu16Voltage);
e_LTC2943_Error_t eLTC2943_StatusGet(uint8_t * p_pu8Status);
 
uint8_t u8LTC2943_IsAvailable(void);

#endif /* LTC2943_H */

