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
#ifndef LSM6DSL_H
#define LSM6DSL_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define LSM6DSL_SLAVE_ADDR_VCC      (uint8_t)0x6B  /*> 0b1101011 */
#define LSM6DSL_SLAVE_ADDR_GND      (uint8_t)0x6A  /*> 0b1101010 */

#define LSM6DSL_WHO_I_AM_ID         (uint8_t)0x6A  /*> 0b01101010 */

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _LSM6DSL_ERROR_ {
   LSM6DSL_ERROR_NONE = 0u,
   LSM6DSL_ERROR_CONTEXT,
   LSM6DSL_ERROR_PARAM,
   LSM6DSL_ERROR_BUSY,
   LSM6DSL_ERROR_NOT_READY,
   LSM6DSL_ERROR_COMM,
   LSM6DSL_ERROR_NOT_FOUND,
}e_LSM6DSL_Error_t;

typedef enum _LSM6DSL_INTERFACE_ {
   LSM6DSL_ITF_I2C = 0u,
   LSM6DSL_ITF_SPI
}e_LSM6DSL_Interface_t;

typedef enum _LSM6DSL_I2C_ADDR_ {
   LSM6DSL_I2C_ADDR_VCC = LSM6DSL_SLAVE_ADDR_VCC,
   LSM6DSL_I2C_ADDR_GND = LSM6DSL_SLAVE_ADDR_GND
}e_LSM6DSL_I2CAddr_t;

typedef enum _LSM6DSL_AXIS_ {
   LSM6DSL_AXIS_NONE = 0u,
   LSM6DSL_AXIS_Z,
   LSM6DSL_AXIS_Y,
   LSM6DSL_AXIS_YZ,
   LSM6DSL_AXIS_X,
   LSM6DSL_AXIS_XZ,
   LSM6DSL_AXIS_XY,
   LSM6DSL_AXIS_XYZ,
}e_LSM6DSL_Axis_t;

typedef enum _LSM6DSL_ODR_ {
   LSM6DSL_ODR_POWER_DOWN = 0u,  /* 0x00 */
   LSM6DSL_ODR_12_5Hz,           /* 0x01 */
   LSM6DSL_ODR_26Hz,             /* 0x02 */
   LSM6DSL_ODR_52Hz,             /* 0x03 */
   LSM6DSL_ODR_104Hz,            /* 0x04 */
   LSM6DSL_ODR_208Hz,            /* 0x05 */
   LSM6DSL_ODR_416Hz,            /* 0x06 */
   LSM6DSL_ODR_833Hz,            /* 0x07 */
   LSM6DSL_ODR_1_66kHz,          /* 0x08 */
   LSM6DSL_ODR_3_33kHz,          /* 0x09 */
   LSM6DSL_ODR_6_66kHz,          /* 0x0A */ 
   LSM6DSL_ODR_1_6Hz,            /* 0x0B */
}e_LSM6DSL_ODR_t;

typedef enum _LSM6DSL_RANGE_ACCEL_ {
   LSM6DSL_ACCEL_RANGE_2G = 0u,
   LSM6DSL_ACCEL_RANGE_16G,
   LSM6DSL_ACCEL_RANGE_4G,
   LSM6DSL_ACCEL_RANGE_8G,
}e_LSM6DSL_AccelFullScale_t;

typedef enum _LSM6DSL_MODE_ACCEL_GYRO_ {
   LSM6DSL_MODE_HIGH_PERF = 0u,
   LSM6DSL_MODE_LOW_POWER,
}e_LSM6DSL_Mode_t;

typedef enum _LSM6DSL_RANGE_GYRO_ {
   LSM6DSL_GYRO_RANGE_250DPS,
   LSM6DSL_GYRO_RANGE_500DPS,
   LSM6DSL_GYRO_RANGE_1000DPS,
   LSM6DSL_GYRO_RANGE_2000DPS,
   LSM6DSL_GYRO_RANGE_125DPS,
}e_LSM6DSL_GyroFullScale_t;


typedef enum _LSM6DSL_INTTERUPT_SOURCES_ {
   LSM6DSL_INT_WAKEUP,
   LSM6DSL_INT_TAP,
   LSM6DSL_INT_D6D,
   LSM6DSL_INT_WRIST,
}e_LSM6DSL_InterruptSources_t;

typedef struct _LSM6DSL_CONTEXT_ {
   e_LSM6DSL_Interface_t eInterface;
   struct {
      e_LSM6DSL_I2CAddr_t eI2CAddress; 
      /* Function pointer to a read I2C transfer */
      uint32_t (*fp_u32I2C_Write)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen);
      /* Function pointer to a write I2C transfer */
      uint32_t (*fp_u32I2C_Read)(uint8_t p_u8Address, uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   }sI2CCfg;
   struct {
      uint32_t u32ChipSelect;
      void (*fp_vPinSet)(uint32_t p_u32PinNumber);
      void (*fp_vPinClear)(uint32_t p_u32PinNumber);
      uint32_t (*fp_u32SPI_Transfer)(uint8_t *p_pu8TxData, uint8_t p_u8TxDataLen, uint8_t *p_pu8RxData, uint8_t p_u8RxDataLen);
   }sSPICfg;
   /* Function pointer to a timer in ms */
   void (*fp_vDelay_ms)(uint32_t p_u32Timeout);   
}s_LSM6DSL_Context_t;

typedef union _LSM6DSL_3AXIS_DATA_ {
   struct _US_ {
      uint16_t u16DataX;
      uint16_t u16DataY;
      uint16_t u16DataZ;
   }RawUnsigned;
   struct _U_ {
      int16_t s16DataX;
      int16_t s16DataY;
      int16_t s16DataZ;
   }RawSigned;
}s_LSM6DSL_3AxisRawData_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
e_LSM6DSL_Error_t eLSM6DSL_ContextSet(s_LSM6DSL_Context_t p_sContext);
e_LSM6DSL_Error_t eLSM6DSL_WhoAmIGet(uint8_t * p_pu8WhoAmI);
e_LSM6DSL_Error_t eLSM6DSL_SoftReset(void);

e_LSM6DSL_Error_t eLSM6DSL_BlockDataUpdateSet(uint8_t p_u8Enable);
e_LSM6DSL_Error_t eLSM6DSL_AccelCfgSet(e_LSM6DSL_ODR_t p_eODR, e_LSM6DSL_AccelFullScale_t p_eFullScale, e_LSM6DSL_Mode_t p_eMode);
e_LSM6DSL_Error_t eLSM6DSL_AccelRead(void);
e_LSM6DSL_Error_t eLSM6DSL_AccelGet(int16_t * p_ps16X, int16_t * p_ps16Y, int16_t * p_ps16Z);
e_LSM6DSL_Error_t eLSM6DSL_AccelRawGet(uint16_t * p_pu16X, uint16_t * p_pu16Y, uint16_t * p_pu16Z);
e_LSM6DSL_Error_t eLSM6DSL_GyroCfgSet(e_LSM6DSL_ODR_t p_eODR, e_LSM6DSL_GyroFullScale_t p_eFullScale, e_LSM6DSL_Mode_t p_eMode);
e_LSM6DSL_Error_t eLSM6DSL_GyroRead(void);
e_LSM6DSL_Error_t eLSM6DSL_GyroGet(int16_t * p_ps16X, int16_t * p_ps16Y, int16_t * p_ps16Z);
e_LSM6DSL_Error_t eLSM6DSL_GyroRawGet(uint16_t * p_pu16X, uint16_t * p_pu16Y, uint16_t * p_pu16Z);

//e_LSM6DSL_Error_t eLSM6DSL_AccelIntCfgSet( );
//e_LSM6DSL_Error_t eLSM6DSL_GyroIntCfgSet( );

uint8_t u8LSM6DSL_IsAvailable(void);

#endif /* LSM6DSL_H */

