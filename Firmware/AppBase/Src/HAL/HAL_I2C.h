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
#ifndef HAL_I2C_H
#define HAL_I2C_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include "app_twi.h"
#include "nrf_drv_twi.h"

/************************************************************************
 * Defines
 ************************************************************************/
#define HAL_I2C_FLAG_TX_POSTINC              NRF_DRV_TWI_FLAG_TX_POSTINC            /**< TX buffer address incremented after transfer. */
#define HAL_I2C_FLAG_RX_POSTINC              NRF_DRV_TWI_FLAG_RX_POSTINC            /**< RX buffer address incremented after transfer. */
#define HAL_I2C_FLAG_NO_XFER_EVT_HANDLER     NRF_DRV_TWI_FLAG_NO_XFER_EVT_HANDLER   /**< Interrupt after each transfer is suppressed, and the event handler is not called. */
#define HAL_I2C_FLAG_HOLD_XFER               NRF_DRV_TWI_FLAG_HOLD_XFER             /**< Set up the transfer but do not start it. */
#define HAL_I2C_FLAG_REPEATED_XFER           NRF_DRV_TWI_FLAG_REPEATED_XFER         /**< Flag indicating that the transfer will be executed multiple times. */
#define HAL_I2C_FLAG_TX_NO_STOP              NRF_DRV_TWI_FLAG_TX_NO_STOP            /**< Flag indicating that the TX transfer will not end with a stop condition. */

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _HAL_I2C_ERRORS_ {
   HALI2C_ERROR_NONE,
   HALI2C_ERROR_PARAM,
   HALI2C_ERROR_BUSY,
   HALI2C_ERROR_INVALID
}e_HalI2c_Error_t;

typedef enum _HAL_I2C_STOP_ {
   I2C_STOP = 0u,
   I2C_NO_STOP = APP_TWI_NO_STOP,
}e_I2CStop_t;

typedef void (*fpvI2CCallback)(void * p_pvData);

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vHal_I2C_Init(void);
void vHal_I2C_Uninit(void);
uint32_t u32Hal_I2C_WriteAndRead(uint8_t p_u8Address, uint8_t * p_pu8DataOut, uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen);
uint32_t u32Hal_I2C_WriteAndReadNoStop(uint8_t p_u8Address, uint8_t * p_pu8DataOut, uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen);
uint32_t u32Hal_I2C_WrnRd(uint8_t p_u8Address, uint8_t * p_pu8DataOut, uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen, uint32_t p_u32FlagsWr, uint32_t p_u32FlagsRd);
uint32_t u32Hal_I2C_WrnRdCb(uint8_t p_u8Address, uint8_t * p_pu8DataOut, uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen, e_I2CStop_t p_eStop, fpvI2CCallback p_pfCallback);
uint32_t u32Hal_I2C_Write(uint8_t p_u8Address, uint8_t * p_pu8Data, uint8_t p_u8DataLength);

#endif // HAL_I2C_H

