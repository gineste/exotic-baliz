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
#ifndef ADXL362_H
#define ADXL362_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _ADXL362_ERROR_ {
   ADXL362_ERROR_NONE,
   ADXL362_ERROR_NOT_FOUND,
   ADXL362_ERROR_COMM,
   ADXL362_ERROR_BUSY,
   ADXL362_ERROR_PARAM,   
   ADXL362_ERROR_IC,   
   ADXL362_ERROR_INIT,
}e_ADXL362_ErrorCode_t;

typedef enum _ADXL362_RANGE_{
   ADXL362_RANGE_2G = 0u,
   ADXL362_RANGE_4G = 1u,
   ADXL362_RANGE_8G = 2u
}e_ADXL362_Range_t;

typedef enum _ADXL362_ODR_{
   ADXL362_ODR_12_5_HZ = 0u, 
   ADXL362_ODR_25_HZ   = 1u, 
   ADXL362_ODR_50_HZ   = 2u, 
   ADXL362_ODR_100_HZ  = 3u, 
   ADXL362_ODR_200_HZ  = 4u, 
   ADXL362_ODR_400_HZ  = 5u   
}e_ADXL362_ODR_t;

typedef enum _ADXL362_LL_MODE_{
   ADXL362_MODE_DEFAULT = 0u,
   ADXL362_MODE_LINK    = 1u,
   ADXL362_MODE_LOOP    = 3u
}e_ADXL362_LinkLoopMode_t;

typedef enum _ADXL362_MEAS_MODE_ {
   ADXL362_MEASURE_STANDBY = 0u,
   ADXL362_MEASURE_ON      = 2u
}e_ADXL362_MeasMode_t;

typedef enum _ADXL362_WAKEUP_MODE_ {
   ADXL362_WAKEUP_OFF = 0u,
   ADXL362_WAKEUP_ON  = 1u
}e_ADXL362_WakeUpMode_t;

typedef enum _ADXL362_NOISE_CTL_ {
   ADXL362_NOISE_MODE_NORMAL   = 0u,
   ADXL362_NOISE_MODE_LOW      = 1u,
   ADXL362_NOISE_MODE_ULTRALOW = 2u
}e_ADXL362_NoiseCtrl_t;

typedef enum _ADXL362_INT_CTL_ {
   ADXL362_INTMAPX_OFF     = 0u,
   ADXL362_INTMAPX_AWAKE   = (1 << 6u)   /* Bit 6 of IntMap Register */
}e_ADXL362_IntMapX_t;

typedef uint32_t (*fp_u32SPI_Transfer_t )(uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
typedef void (*fp_vTimer_Delay_ms)(uint32_t p_u32Timeout);
typedef void (*fp_vInterruptHandler_t)(uint8_t p_u8Type);

typedef struct _ADXL362_CONTEXT_ {
   fp_u32SPI_Transfer_t fp_u32SPITransfer;   /* Function pointer to a SPI transfer */
   fp_vTimer_Delay_ms fp_vDelay_ms;          /* Function pointer to a timer in ms */
	fp_vInterruptHandler_t fp_Int1Handler;    /* Interrupt Handler for Activity and Inactivity */
	fp_vInterruptHandler_t fp_Int2Handler;    /* Interrupt Handler for Activity and Inactivity */
   
   e_ADXL362_Range_t eRange;                 /* Range of accelerometer */
   e_ADXL362_ODR_t eOutputDataRate;          /* Output Data Rate */
   e_ADXL362_LinkLoopMode_t eLinkLoopMode;   /* Functionning mode (Link/Loop) */
   e_ADXL362_NoiseCtrl_t eNoiseCtrl;         /* Noise control (Normal, Low, Ultra Low) */

   uint8_t u8ActivityDetection;              /* 0 - no activity detection, 1 - activity detection */
   uint8_t u8RefOrAbsActivity;               /* 0 - absolute mode, 1 - referenced mode. */
   uint16_t u16ThresholdActivity;            /* 11-bit unsigned value that the adxl362 samples are compared to. */
   uint16_t u16TimeActivity;                 /* 16-bit value activity time in ms (from 2.5ms @ odr = 400Hz to 20s @ odr = 12.5Hz) */
	                                          
   uint8_t u8InactivityDetection;            /* 0 - no activity detection, 1 - activity detection */
   uint8_t u8RefOrAbsInactivity;             /* 0 - absolute mode, 1 - referenced mode. */
   uint16_t u16ThresholdInactivity;          /* 11-bit unsigned value that the ADXL362 samples are compared to. */
   uint32_t u32TimeInactivity;               /* 16-bit value inactivity time in ms (from 164s @ odr = 400Hz to 87min @ odr = 12.5Hz) */
	
   e_ADXL362_IntMapX_t eInt1Map;             /* Type of the interrupt n°1 */
   e_ADXL362_IntMapX_t eInt2Map;             /* Type of the interrupt n°2 */
   uint32_t u32Int1Pin;
   uint32_t u32Int2Pin;   
   
   e_ADXL362_WakeUpMode_t eWakeUpMode;       /* Wake-up mode (Let it OFF if you adjustable ODR for (In)Activity detection) */
   e_ADXL362_MeasMode_t eMeasureMode;		   /* Power mode (Standby or Measurement Mode) */
}s_ADXL362_Context_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vADXL362_ContextSet(s_ADXL362_Context_t p_sContext);
e_ADXL362_ErrorCode_t eADXL362_Init(void);
e_ADXL362_ErrorCode_t eADXL362_SoftReset(void);
e_ADXL362_ErrorCode_t eADXL362_PartIDGet(uint8_t * p_pu8PartID);
e_ADXL362_ErrorCode_t eADXL362_StatusGet(uint8_t * p_pu8Status);
uint8_t u8ADXL362_IsPowerGlitchDetected(void);
e_ADXL362_ErrorCode_t eADXL362_RangeSet(e_ADXL362_Range_t p_eRange);
e_ADXL362_ErrorCode_t eADXL362_OutputDataRateSet(e_ADXL362_ODR_t p_eOutputDataRate);
e_ADXL362_ErrorCode_t eADXL362_HalfBandwithSet(uint8_t p_u8HalfBandwith);
e_ADXL362_ErrorCode_t eADXL362_LinkLoopModeSet(e_ADXL362_LinkLoopMode_t p_eLinkLoopMode);
e_ADXL362_ErrorCode_t eADXL362_NoiseSet(e_ADXL362_NoiseCtrl_t p_eNoiseCtrl);
e_ADXL362_ErrorCode_t eADXL362_WakeUpModeSet(e_ADXL362_WakeUpMode_t p_eWakeUpMode);
e_ADXL362_ErrorCode_t eADXL362_MeasureModeSet(e_ADXL362_MeasMode_t p_eMeasureMode);
e_ADXL362_ErrorCode_t eADXL362_NoiseControlSet(e_ADXL362_NoiseCtrl_t p_eNoise);
e_ADXL362_ErrorCode_t eADXL362_ActivityDetectionSet(uint8_t p_u8RefOrAbs, uint16_t p_u16Threshold, uint16_t p_u16Time);
e_ADXL362_ErrorCode_t eADXL362_ActivityDetectionDisable(void);
e_ADXL362_ErrorCode_t eADXL362_InactivityDetectionSet(uint8_t p_u8RefOrAbs, uint16_t p_u16Threshold, uint32_t p_u32Time);
e_ADXL362_ErrorCode_t eADXL362_InactivityDetectionDisable(void);
e_ADXL362_ErrorCode_t eADXL362_InterruptMapSet(uint8_t p_u8InterruptNumber, e_ADXL362_IntMapX_t p_eIntMapStatus);
e_ADXL362_ErrorCode_t eADXL362_AccelerationRead(void);
void vADXL362_AccelerationGet(int16_t * p_ps16AccelX, int16_t * p_ps16AccelY, int16_t * p_ps16AccelZ);
uint8_t u8ADXL362_IsAvailable(void);
uint8_t u8ADXL362_IsAwake(void);
e_ADXL362_ErrorCode_t eADXL362_RegisterSet( uint8_t p_u8Address, uint8_t p_u8Data);
e_ADXL362_ErrorCode_t eADXL362_RegisterGet( uint8_t p_u8Address, uint8_t * p_pu8Data);

#endif /* ADXL362_H */

