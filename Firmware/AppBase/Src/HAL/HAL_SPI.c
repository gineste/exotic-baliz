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
 * Date:          27 06 2017(DD MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Hardware Abstraction Layer for SPI communication. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <nrf_error.h>
//#include <nrf.h>
#include <nrf52.h>
#include <nrf_drv_spi.h>
#include <app_util_platform.h>   // For App IRQ Priority error

#include "board.h"

#include "HAL_GPIO.h"

#include "GlobalDefs.h"

#include "HAL_SPI.h"

/************************************************************************
 * Defines
 ************************************************************************/
#define SPI_USED     0
#define SPI_CALLBACK 0u

/**@brief Macro for calling error handler function specific for HAL_SPI
 *        if supplied error code any other than NRF_SUCCESS.
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define HAL_SPI_ERROR_CHECK(ERR_CODE)                       \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
        {                                                   \
           PRINT_ERROR("ERROR SPI %d",ERR_CODE);            \
           vDrvSPI_Uninit();                                \
           vDrvSPI_Init();                                  \
        }                                                   \
    } while (0)
    
/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef enum {
   SPI_DISABLE = 0,
   SPI_ENABLE = 1,
}e_SPI_State_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vSPI_Enable(e_SPI_State_t p_eEnable);
static void vDrvSPI_Init(void);
static void vDrvSPI_Uninit(void);
static void vSpiEventHandler(nrf_drv_spi_evt_t const * p_psEvent, void * p_pvContext);
    
/************************************************************************
 * Variable declarations
 ************************************************************************/
const nrf_drv_spi_t g_csSPIInstance = NRF_DRV_SPI_INSTANCE(SPI_USED);

static uint8_t g_u8Initialized = 0u;
static uint8_t g_u8ContextSet = 0u;
static volatile uint8_t g_u8XferDone = 1u;

static s_HalSpi_Context_t g_sSPIContext = {
   .u32MOSIPin = NRF_DRV_SPI_PIN_NOT_USED,
   .u32MISOPin = NRF_DRV_SPI_PIN_NOT_USED,
   .u32ClockPin = NRF_DRV_SPI_PIN_NOT_USED,
   .u32ChipSelectPin = NULL,
   .eMode = HALSPI_MODE_0,
   .eFrequency = HALSPI_FREQ_1M,
};
   
/************************************************************************
 * Public functions
 ************************************************************************/  
void vHal_SPI_ContextSet(s_HalSpi_Context_t p_sContext)
{
   g_sSPIContext = p_sContext;
   g_u8ContextSet = 1u;
}

/**@brief Function for initializing SPI module.
 */
void vHal_SPI_Init(void)
{
   uint32_t l_u32ErrCode = 0u;
   nrf_drv_spi_config_t l_sSPIConfig = NRF_DRV_SPI_DEFAULT_CONFIG;
   
   /* Uninit in any case */
   vHal_SPI_Uninit();
   
   /* If context not set, use default config */
   if(g_u8ContextSet == 0u)
   {
      l_sSPIConfig.frequency = NRF_DRV_SPI_FREQ_1M;
      l_sSPIConfig.mode = NRF_DRV_SPI_MODE_0;
      l_sSPIConfig.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;         /* User will have to driver Chip Select manually */
      l_sSPIConfig.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
      l_sSPIConfig.mosi_pin = NRF_DRV_SPI_PIN_NOT_USED;
      l_sSPIConfig.sck_pin = NRF_DRV_SPI_PIN_NOT_USED;
   }
   else
   {
      /* Overwrite default config */
      l_sSPIConfig.frequency = (nrf_drv_spi_frequency_t)g_sSPIContext.eFrequency;
      l_sSPIConfig.mode = (nrf_drv_spi_mode_t)g_sSPIContext.eMode;
      l_sSPIConfig.ss_pin = g_sSPIContext.u32ChipSelectPin;
      l_sSPIConfig.miso_pin = g_sSPIContext.u32MISOPin;
      l_sSPIConfig.mosi_pin = g_sSPIContext.u32MOSIPin;
      l_sSPIConfig.sck_pin = g_sSPIContext.u32ClockPin;   
   }
   
   if(g_u8Initialized == 0u)
   {
      g_u8Initialized = 1u;
      /* Call Spi Init Instance */
      l_u32ErrCode = (uint32_t)nrf_drv_spi_init(&g_csSPIInstance, &l_sSPIConfig, vSpiEventHandler, NULL);
      HAL_SPI_ERROR_CHECK(l_u32ErrCode);      
      
      vSPI_Enable(SPI_DISABLE); 
   }
}

/**@brief Function for uninitializing SPI module in order to
 * reconfigure it for example.
 */
void vHal_SPI_Uninit(void)
{
	if(g_u8Initialized == 1u)
	{
		nrf_drv_spi_uninit(&g_csSPIInstance);
		g_u8Initialized = 0u;
      
      
      vGPIO_InputCfg(g_sSPIContext.u32MISOPin, HALGPIO_PIN_PULLDOWN);
      vGPIO_InputCfg(g_sSPIContext.u32MOSIPin, HALGPIO_PIN_PULLDOWN); 
      vGPIO_InputCfg(g_sSPIContext.u32ClockPin, HALGPIO_PIN_PULLDOWN); 
	}	
}

/**@brief Performs a SPI transfer
 *
 * @param[in]  p_pu8TxBuffer : Transmition buffer
 *             p_u8TxBufLen  : Size of Tx Data
 *             p_u8RxBufLen  : Size of Rx Data
 * @param[out] p_pu8RxBuffer : Reception buffer
 *
 * @return Error Code, could be SUCCESS, ERROR_BUSY, or ERROR_INTERNAL(Hardware)
 */
uint32_t u32Hal_SPI_Transfer(uint8_t *p_pu8TxBuffer, uint8_t p_u8TxBufLen, uint8_t *p_pu8RxBuffer, uint8_t p_u8RxBufLen)
{
   uint32_t l_u32ErrCode = NRF_ERROR_INTERNAL;
   int32_t l_s32HwTimeout = 10000;
   if(g_u8Initialized == 1u)
   {
      vSPI_Enable(SPI_ENABLE);
      
      g_u8XferDone = 0u;
      l_u32ErrCode = (uint32_t)nrf_drv_spi_transfer(&g_csSPIInstance, p_pu8TxBuffer, p_u8TxBufLen, p_pu8RxBuffer, p_u8RxBufLen);
      HAL_SPI_ERROR_CHECK(l_u32ErrCode);
      while(   (!g_u8XferDone)           /* Blocking */
            && (l_s32HwTimeout > 0) )
      {
         l_s32HwTimeout--;
      }
      if(l_s32HwTimeout < 0)
      {
         l_u32ErrCode = NRF_ERROR_TIMEOUT;
      }
      
      vSPI_Enable(SPI_DISABLE);      
   }
   else
   {
      l_u32ErrCode = NRF_ERROR_FORBIDDEN;
   }
   
	return l_u32ErrCode;
}

/************************************************************************
 * Private functions
 ************************************************************************/
static void vSPI_Enable(e_SPI_State_t p_eEnable)
{
//   switch(SPI_USED)
//   {
//      case 0:
//         NRF_SPI0->ENABLE = p_eEnable;
//         break;
//      case 1:
//         NRF_SPI1->ENABLE = p_eEnable;
//         break;
//      case 2:
//         NRF_SPI2->ENABLE = p_eEnable;
//         break;
//      default:
//         break;
//   }
}

/**@brief Static Function to init driver spi nrf.
 */
static void vDrvSPI_Init(void)
{
   vHal_SPI_Init();
}
/**@brief Static Function to uninit driver spi nrf.
 */
static void vDrvSPI_Uninit(void)
{
   vHal_SPI_Uninit();
}

/**@brief SPI user event handler.
 * @param event
 */
static void vSpiEventHandler(nrf_drv_spi_evt_t const * p_psEvent, void * p_pvContext)
{  /* Event type is NRF_DRV_SPI_EVENT_DONE is all cases */
   g_u8XferDone = 1u;
}


/************************************************************************
 * End Of File
 ************************************************************************/

