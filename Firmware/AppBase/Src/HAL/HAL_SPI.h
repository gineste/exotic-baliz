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
#ifndef HAL_SPI_H
#define HAL_SPI_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <nrf_drv_spi.h>

/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _HAL_SPI_MODE_ {
   HALSPI_MODE_0 = NRF_DRV_SPI_MODE_0,
   HALSPI_MODE_1 = NRF_DRV_SPI_MODE_1,
   HALSPI_MODE_2 = NRF_DRV_SPI_MODE_2,
   HALSPI_MODE_3 = NRF_DRV_SPI_MODE_3
}e_HalSpi_Mode_t;
 
typedef enum _HAL_SPI_FREQUENCY_ {
   HALSPI_FREQ_125K = NRF_DRV_SPI_FREQ_125K,
   HALSPI_FREQ_250K = NRF_DRV_SPI_FREQ_250K,
   HALSPI_FREQ_500K = NRF_DRV_SPI_FREQ_500K,
   HALSPI_FREQ_1M = NRF_DRV_SPI_FREQ_1M,
   HALSPI_FREQ_2M = NRF_DRV_SPI_FREQ_2M,
   HALSPI_FREQ_4M = NRF_DRV_SPI_FREQ_4M,
   HALSPI_FREQ_8M = NRF_DRV_SPI_FREQ_8M  
}e_HalSpi_Frequency_t;

typedef struct _HAL_SPI_CONTEXT_ {
   uint32_t u32MOSIPin;
   uint32_t u32MISOPin;
   uint32_t u32ClockPin;
   uint32_t u32ChipSelectPin;
   e_HalSpi_Mode_t eMode;
   e_HalSpi_Frequency_t eFrequency;
}s_HalSpi_Context_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vHal_SPI_ContextSet(s_HalSpi_Context_t p_sContext);
void vHal_SPI_Init(void);
void vHal_SPI_Uninit(void);
uint32_t u32Hal_SPI_Transfer(uint8_t *p_pu8TxBuffer, uint8_t p_u8TxBufLen, uint8_t *p_pu8RxBuffer, uint8_t p_u8RxBufLen);

#endif // HAL_SPI_H

