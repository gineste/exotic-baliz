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
#ifndef LIS2MDL_H
#define LIS2MDL_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define LIS2MDL_WHO_AM_I_ID   (uint8_t)0x40

/****************************************************************************************
 * Type definitions
****************************************************************************************/
typedef enum _LIS2MDL_ERRCODE_ {
   LIS2MDL_ERROR_NONE = 0u,
   LIS2MDL_ERROR_CONTEXT,
   LIS2MDL_ERROR_PARAM,
   LIS2MDL_ERROR_BUSY,
   LIS2MDL_ERROR_NOT_READY,
   LIS2MDL_ERROR_COMM,
   LIS2MDL_ERROR_NOT_FOUND
}e_LIS2MDL_Error_t;

typedef enum _LIS2MDL_COMMUNICATION_ {
   LIS2MDL_COMM_I2C = 0u,        /*> 0x00 (default) */
   LIS2MDL_COMM_SPI              /*> 0x01 */
}e_LIS2MDL_Comm_t;

typedef enum _LIS2MDL_ODR_ {
   LIS2MDL_ODR_10Hz = 0u,        /*> 0x00 (default) */
   LIS2MDL_ODR_20Hz,             /*> 0x01 */
   LIS2MDL_ODR_50Hz,             /*> 0x02 */
   LIS2MDL_ODR_100Hz             /*> 0x03 */
}e_LIS2MDL_ODR_t;

typedef enum _LIS2MDL_MODE_ { 
   LIS2MDL_MODE_CONTINUOUS = 0u, /*> 0x00 */
   LIS2MDL_MODE_SINGLE_SHOT,     /*> 0x01 */
   LIS2MDL_MODE_IDLE,            /*> 0x02 */
   LIS2MDL_MODE_IDLE_DEFAULT     /*> 0x03 (default : as previous Idle mode) */
}e_LIS2MDL_Mode_t;

typedef enum _LIS2MDL_INT_AXIS_ {
   LIS2MDL_INT_NO_AXIS = 0u,  /*> 0x00 */
   LIS2MDL_INT_AXIS_Z,        /*> 0x01 */
   LIS2MDL_INT_AXIS_Y,        /*> 0x02 */
   LIS2MDL_INT_AXIS_YZ,       /*> 0x03 */
   LIS2MDL_INT_AXIS_X,        /*> 0x04 */
   LIS2MDL_INT_AXIS_XZ,       /*> 0x05 */
   LIS2MDL_INT_AXIS_XY,       /*> 0x06 */
   LIS2MDL_INT_AXIS_XYZ       /*> 0x07 (default) */
}e_LIS2MDL_InterruptAxis_t;

typedef struct _LIS2MDL_CONTEXT_ {
   e_LIS2MDL_Comm_t eCommunicationUsed;
   struct {
      /* Function pointer to a read I2C transfer */
      uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
      /* Function pointer to a write I2C transfer */
      uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   }sI2CCfg;   
   struct {
      uint32_t u32ChipSelect;
      uint32_t u32MISO;
      uint32_t u32MOSI;
      uint32_t u32Clock;
      void (*fp_vPinSet)(uint32_t p_u32PinNumber);
      void (*fp_vPinClear)(uint32_t p_u32PinNumber);
      /* Function pointer to perform a read/write SPI */
      uint32_t (*fp_u32SPI_Transfer)(uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   }sSPICfg;
   /* Function pointer to a timer in ms */
   void (*fp_vDelay_ms)(uint32_t p_u32Timeout);   
}s_LIS2MDL_Context_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_LIS2MDL_Error_t eLIS2MDL_ContextSet(s_LIS2MDL_Context_t p_sContext);
e_LIS2MDL_Error_t eLIS2MDL_WhoAmIGet(uint8_t * p_pu8WhoAmI);
e_LIS2MDL_Error_t eLIS2MDL_Reboot(void);
e_LIS2MDL_Error_t eLIS2MDL_SoftReset(void);
e_LIS2MDL_Error_t eLIS2MDL_LowPower(uint8_t p_u8Activate);
e_LIS2MDL_Error_t eLIS2MDL_OutputDataRateSet(e_LIS2MDL_ODR_t p_eODR);
e_LIS2MDL_Error_t eLIS2MDL_ModeSet(e_LIS2MDL_Mode_t p_eMode);
e_LIS2MDL_Error_t eLIS2MDL_BigLittleEndian(uint8_t p_u8LittleEndian);
e_LIS2MDL_Error_t eLIS2MDL_SelfTest(uint8_t p_u8Activate);
e_LIS2MDL_Error_t eLIS2MDL_OffsetSet(int16_t p_s16OffsetX, int16_t p_s16OffsetY, int16_t p_s16OffsetZ);
e_LIS2MDL_Error_t eLIS2MDL_LowPassFilter(uint8_t p_u8Activate);
e_LIS2MDL_Error_t eLIS2MDL_TemperatureCompensation(uint8_t p_u8Activate);
e_LIS2MDL_Error_t eLIS2MDL_MagneticRead(void);
e_LIS2MDL_Error_t eLIS2MDL_TemperatureRead(void);
e_LIS2MDL_Error_t eLIS2MDL_InterruptCtrlSet(uint8_t p_u8Activate, e_LIS2MDL_InterruptAxis_t p_eIntAxis, uint8_t p_u8Polarity, uint8_t p_u8Latched);
e_LIS2MDL_Error_t eLIS2MDL_ThresholdSet(uint16_t p_u16TresholdmG);
e_LIS2MDL_Error_t eLIS2MDL_MagDataGet(int16_t * p_ps16X, int16_t * p_ps16Y, int16_t * p_ps16Z);
e_LIS2MDL_Error_t eLIS2MDL_TempDataGet(int16_t * p_ps16Temp);
uint8_t u8LIS2MDL_IsAvailable(void);

#endif /* LIS2MDL_H */
