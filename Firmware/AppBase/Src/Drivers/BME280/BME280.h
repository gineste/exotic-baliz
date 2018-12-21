/* 
 *  ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 * (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *  ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 * (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BME280_H
#define BME280_H

/************************************************************************
 * Include Files
 ************************************************************************/
 #include <stdint.h>

/************************************************************************
 * Defines
 ************************************************************************/
#define BME280_CHIP_ID           (uint8_t)0x60

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _BME280_ERROR_CODE_ {
   BME280_ERROR_NONE,         /*! Success */
   BME280_ERROR_CONTEXT,      /*! Context not setted correclty */
   BME280_ERROR_BUSY,         /*! Communication Busy */
   BME280_ERROR_PARAM,        /*! Wrong parameter (Null pointer,...) */
   BME280_ERROR_INVALID,      /*! Invalid context (Wrong mode,...) */
   BME280_ERROR_COMM          /*! Communication error */
}e_BME280_Error_t;


typedef enum _BME280_INTERFACE_ {
   BME280_SPI_ITF = 0u,    /*! SPI interface */
   BME280_I2C_ITF          /*! I2C interface */
}e_BME280_Interface_t;

typedef enum _BME280_I2C_ADDR_ {
   BME280_I2C_ADDR_GND = 0x76,   
   BME280_I2C_ADDR_VDD = 0x77
}e_BME280_I2C_Address_t;

typedef enum _BME280_MODE_ {
   BME280_SLEEP = 0u,
   BME280_FORCED = 1u,
   BME280_NORMAL = 3u
}e_BME280_Mode_t;

typedef enum _BME280_OSR_ {
   BME280_OVERSAMPLING_OFF = 0u,  /*! Measurement is off */
   BME280_OVERSAMPLING_1X,
   BME280_OVERSAMPLING_2X,
   BME280_OVERSAMPLING_4X,
   BME280_OVERSAMPLING_8X,
   BME280_OVERSAMPLING_16X,
}e_BME280_Oversampling_t;

typedef enum _BME280_FILTER_ {
   BME280_FILTER_COEFF_OFF = 0u,
   BME280_FILTER_COEFF_2,
   BME280_FILTER_COEFF_4,
   BME280_FILTER_COEFF_8,
   BME280_FILTER_COEFF_16,   
}e_BME280_Filter_t;

typedef enum _BME280_STDBY_TIMER_ {
   BME280_STANDBY_TIME_0_5_MS = 0u,
   BME280_STANDBY_TIME_62_5_MS,
   BME280_STANDBY_TIME_125_MS,
   BME280_STANDBY_TIME_250_MS,
   BME280_STANDBY_TIME_500_MS,
   BME280_STANDBY_TIME_1000_MS,
   BME280_STANDBY_TIME_10_MS,
   BME280_STANDBY_TIME_20_MS,
}e_BME280_StandByTimer_t;

typedef struct _BME280_CALIB_DATA_ {
	uint16_t u16Dig_T1;
	int16_t  s16Dig_T2;
	int16_t  s16Dig_T3;
	uint16_t u16Dig_P1;
	int16_t  s16Dig_P2;
	int16_t  s16Dig_P3;
	int16_t  s16Dig_P4;
	int16_t  s16Dig_P5;
	int16_t  s16Dig_P6;
	int16_t  s16Dig_P7;
	int16_t  s16Dig_P8;
	int16_t  s16Dig_P9;
	uint8_t  u8Dig_H1;
	int16_t  s16Dig_H2;
	uint8_t  u8Dig_H3;
	int16_t  s16Dig_H4;
	int16_t  s16Dig_H5;
	int8_t   s8Dig_H6;
	int32_t  s32TFine;
}s_BME280_Calibration_Data_t;

typedef struct _BME280_STG_ {
   /*! Pressure oversampling */
   e_BME280_Oversampling_t eOSR_P;
   /*! Temperature oversampling */
   e_BME280_Oversampling_t eOSR_T;
   /*! Humidity oversampling */
   e_BME280_Oversampling_t eOSR_H;
   /*! Filter coefficient */
   e_BME280_Filter_t eFilter;
   /*! Standby time */
   e_BME280_StandByTimer_t eStandbyTime;
   /*! Mode */
   e_BME280_Mode_t eMode;
}s_BME280_Settings_t;

typedef struct _BME280_CONTEXT_ {
   e_BME280_Interface_t eInterface;
   /*! When eInterface is BME280_I2C_ITF */
   struct {
      e_BME280_I2C_Address_t eI2CAddr;
      uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
      uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   }sI2CCfg;
   /*! When eInterface is BME280_SPI_ITF */
   struct {
      uint32_t u32ChipSelect;
      void (*fp_vPinSet)(uint32_t p_u32PinNumber);
      void (*fp_vPinClear)(uint32_t p_u32PinNumber);
      uint32_t (*fp_u32SPI_Transfer)(uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   }sSPICfg;
   /*! Delay function pointer */
   void (*fp_vTimerDelay_ms)(uint32_t p_u32Timeout);
}s_BME280_Context_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
e_BME280_Error_t eBME280_ContextSet(s_BME280_Context_t p_sContext);
e_BME280_Error_t eBME280_Reset(void);
e_BME280_Error_t eBME280_ChipIDGet(uint8_t * p_pu8ChipId);
e_BME280_Error_t eBME280_CalibrationDataGet(s_BME280_Calibration_Data_t * p_psCalibration);
e_BME280_Error_t eBME280_OSRTemperatureSet(e_BME280_Oversampling_t p_eOSR);
e_BME280_Error_t eBME280_OSRPressureSet(e_BME280_Oversampling_t p_eOSR);
e_BME280_Error_t eBME280_OSRHumiditySet(e_BME280_Oversampling_t p_eOSR);
e_BME280_Error_t eBME280_IIRFilterSet(e_BME280_Filter_t p_eFilter);
e_BME280_Error_t eBME280_StandbyTimeSet(e_BME280_StandByTimer_t p_eStandbyTime);
e_BME280_Error_t eBME280_ModeSet(e_BME280_Mode_t p_eMode);
e_BME280_Error_t eBME280_ModeGet(e_BME280_Mode_t * p_peMode);

e_BME280_Error_t eBME280_TPHRead(void);
e_BME280_Error_t eBME280_RawTemperatureGet(uint32_t * p_pu32RawTemperature);
e_BME280_Error_t eBME280_RawPressureGet(uint32_t * p_pu32RawPressure);
e_BME280_Error_t eBME280_RawHumidityGet(uint16_t * p_pu16RawHumidity);
e_BME280_Error_t eBME280_TemperatureGet(float * p_pf32Temperature);
e_BME280_Error_t eBME280_PressureGet(float * p_pf32Pressure);
e_BME280_Error_t eBME280_HumidityGet(float * p_pf32Humidity);

uint8_t u8BME280_IsAvailable(void);

#endif /* BME280_H */

