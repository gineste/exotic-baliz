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
 * Date:			   27 06 2017 (DD MM YYYY)
 * Author:			Yoann Rebischung
 * Description:	Hardware Abstraction Layer for TWI communication 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <nrf.h>
#include <nrf52.h>
#include <nrf_drv_twi.h>
#include <app_util_platform.h>   /* For App IRQ Priority error */

#include "board.h"
#include "HAL_GPIO.h"   /* In order to reconfigure SDA, SCL Pins */

#include "app_twi.h"
#include "app_error.h"

#include "GlobalDefs.h"

#include "HAL_I2C.h"

#include "HAL_RTC.h"
#ifdef LOG_ERROR
   //#include "MemoryInterface.h"
#endif

/************************************************************************
 * Defines
 ************************************************************************/
#define I2C_USED		               1
#define TWI_INSTANCE_ID             I2C_USED

#define MAX_PENDING_TRANSACTIONS    ((uint8_t)5u)

#define XFER_RW                     ((uint8_t)2u)  //sizeof(l_sDataTransfer)/ sizeof(l_sDataTransfer[0u])
#define XFER_W                      ((uint8_t)1u)
#define XFER_R                      ((uint8_t)1u)
    
#define MULTIPLE_TWI_DEVICES        1

/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef enum _I2C_STATE_ {
   I2C_DISABLE  = 0u,
   I2C_ENABLE   = 1u
}e_I2C_State_t;


/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vI2C_Enable(e_I2C_State_t p_eEnable);
static void vDrvI2C_Init(void);
static void vDrvI2C_Uninit(void);
static void vI2CErrorLog(uint32_t l_u32Err);

/**@brief Macro for calling error handler function specific for HAL_I2C
 *        if supplied error code any other than NRF_SUCCESS.
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define HAL_I2C_ERROR_CHECK(ERR_CODE)                       \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
        {                                                   \
           PRINT_ERROR("ERROR I2C %d", ERR_CODE);           \
           vDrvI2C_Uninit();                                \
           vDrvI2C_Init();                                  \
        }                                                   \
    } while (0)
    
/************************************************************************
 * Variable declarations
 ************************************************************************/    
APP_TWI_DEF(g_sI2CInstance, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);
    
static nrf_drv_twi_config_t g_sI2CConfig = NRF_DRV_TWI_DEFAULT_CONFIG;
static bool g_bInitialized = false;

/************************************************************************
 * Public functions
 ************************************************************************/ 
/**@brief Function for initializing I2C module.
 */
void vHal_I2C_Init(void)
{
   uint32_t l_u32ErrCode = 0u;
   
   if(g_bInitialized == false)
   {
      /* Reconfigure I2C */
      g_sI2CConfig.interrupt_priority = APP_IRQ_PRIORITY_LOW;
      g_sI2CConfig.frequency = NRF_TWI_FREQ_400K;
      g_sI2CConfig.scl = I2C_SCL;
      g_sI2CConfig.sda = I2C_SDA;
      
      /* Init I2C in blocking mode */   
      l_u32ErrCode = app_twi_init(&g_sI2CInstance, &g_sI2CConfig);
      HAL_I2C_ERROR_CHECK(l_u32ErrCode); 
   #if (MULTIPLE_TWI_DEVICES == 1)
      vHal_GPIO_Cfg(g_sI2CConfig.sda,HALGPIO_PINDIR_INPUT,HALGPIO_PININ_CONNECT,
            HALGPIO_PIN_PULLUP,HALGPIO_PINDRV_H0D1,HALGPIO_PIN_NOSENSE);
      vHal_GPIO_Cfg(g_sI2CConfig.scl,HALGPIO_PINDIR_INPUT,HALGPIO_PININ_CONNECT,
            HALGPIO_PIN_PULLUP,HALGPIO_PINDRV_H0D1,HALGPIO_PIN_NOSENSE);
   #endif
  
      g_bInitialized = true;
   }
}

/**@brief Function for uninitializing I2C module.
 */
void vHal_I2C_Uninit(void)
{   
	if(g_bInitialized == true)
	{  /* Uninit I2C module */   
      app_twi_uninit(&g_sI2CInstance);      
      g_bInitialized = false;
           
      vGPIO_InputCfg(g_sI2CConfig.sda, HALGPIO_PIN_NOPULL);
      vGPIO_InputCfg(g_sI2CConfig.scl, HALGPIO_PIN_NOPULL);
   }
}

/**@brief Function to write and read with I2C module.
 *
 * @param[in]  p_u8Address : Address of the I2C module
 * @param[in]  p_pu8DataOut : Data you would like to send
 * @param[in]  p_u8DataOutLen : Length of data you would like to send
 * @param[in]  p_u8DataInLen : length of read data
 * @param[out] p_pu8DataIn : Read data from I2C module
 *
 * @return Error Code, could be SUCCESS, ERROR_BUSY, or ERROR_INTERNAL(Hardware)
 */
uint32_t u32Hal_I2C_WriteAndRead(uint8_t p_u8Address, uint8_t * p_pu8DataOut, 
                                          uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen)
{
   uint32_t l_u32ErrCode;
   app_twi_transfer_t l_sDataTransfer[XFER_RW] = {
      APP_TWI_WRITE(p_u8Address, p_pu8DataOut, p_u8DataOutLen, I2C_STOP),
      APP_TWI_READ(p_u8Address, p_pu8DataIn, p_u8DataInLen, I2C_STOP)
   };
   	
   vI2C_Enable(I2C_ENABLE);
   
   /* Write then Read */
   l_u32ErrCode = app_twi_perform(&g_sI2CInstance, l_sDataTransfer, XFER_RW, NULL);   
	HAL_I2C_ERROR_CHECK(l_u32ErrCode);
   if(l_u32ErrCode != NRF_SUCCESS)
   {
      vI2CErrorLog(l_u32ErrCode);
   }
            
   vI2C_Enable(I2C_DISABLE);
   
	return l_u32ErrCode;
}

/**@brief Function to write and read with I2C module.
 *
 * @param[in]  p_u8Address : Address of the I2C module
 * @param[in]  p_pu8DataOut : Data you would like to send
 * @param[in]  p_u8DataOutLen : Length of data you would like to send
 * @param[in]  p_u8DataInLen : length of read data
 * @param[out] p_pu8DataIn : Read data from I2C module
 *
 * @return Error Code, could be SUCCESS, ERROR_BUSY, or ERROR_INTERNAL(Hardware)
 */
uint32_t u32Hal_I2C_WriteAndReadNoStop(uint8_t p_u8Address, uint8_t * p_pu8DataOut, 
                                          uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen)
{
   uint32_t l_u32ErrCode;
   app_twi_transfer_t l_sDataTransfer[XFER_RW] = {
      APP_TWI_WRITE(p_u8Address, p_pu8DataOut, p_u8DataOutLen, I2C_NO_STOP),
      APP_TWI_READ(p_u8Address, p_pu8DataIn, p_u8DataInLen, I2C_STOP)
   };
   	
   vI2C_Enable(I2C_ENABLE);
   
   /* Write then Read */
   l_u32ErrCode = app_twi_perform(&g_sI2CInstance, l_sDataTransfer, XFER_RW, NULL);
	HAL_I2C_ERROR_CHECK(l_u32ErrCode);
   if(l_u32ErrCode != NRF_SUCCESS)
   {
      vI2CErrorLog(l_u32ErrCode);
   }
            
   vI2C_Enable(I2C_DISABLE);
   
	return l_u32ErrCode;
}


/**@brief Function to write and read with I2C module.
 *
 * @param[in]  p_u8Address : Address of the I2C module
 * @param[in]  p_pu8DataOut : Data you would like to send
 * @param[in]  p_u8DataOutLen : Length of data you would like to send
 * @param[in]  p_u8DataInLen : length of read data
 * @param[out] p_pu8DataIn : Read data from I2C module
 * @param[in]  p_u32Flags : Flags between write and read
 *
 * @return Error Code, could be SUCCESS, ERROR_BUSY, ERROR_TIMEOUT or ERROR_INTERNAL(Hardware)
 */
uint32_t u32Hal_I2C_WrnRd(uint8_t p_u8Address, uint8_t * p_pu8DataOut, uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen, uint32_t p_u32FlagsWr, uint32_t p_u32FlagsRd)
{
   uint32_t l_u32ErrCode;
   app_twi_transfer_t l_sDataTransfer[XFER_RW];
   
   /* Write first */
   l_sDataTransfer[0u].operation = APP_TWI_WRITE_OP(p_u8Address);
   l_sDataTransfer[0u].p_data = p_pu8DataOut;
   l_sDataTransfer[0u].length = p_u8DataOutLen;
   l_sDataTransfer[0u].flags = p_u32FlagsWr;
   /* Read next */
   l_sDataTransfer[1u].operation = APP_TWI_READ_OP(p_u8Address);
   l_sDataTransfer[1u].p_data = p_pu8DataIn;
   l_sDataTransfer[1u].length = p_u8DataInLen;
   l_sDataTransfer[1u].flags = p_u32FlagsRd;
     	
   vI2C_Enable(I2C_ENABLE);
   
   /* Write then Read */
   l_u32ErrCode = app_twi_perform(&g_sI2CInstance, l_sDataTransfer, XFER_RW, NULL);
	HAL_I2C_ERROR_CHECK(l_u32ErrCode);
   if(l_u32ErrCode != NRF_SUCCESS)
   {
      vI2CErrorLog(l_u32ErrCode);
   }
            
   vI2C_Enable(I2C_DISABLE);
   
	return l_u32ErrCode;
}
uint32_t u32Hal_I2C_WrnRdCb(uint8_t p_u8Address, uint8_t * p_pu8DataOut, uint8_t p_u8DataOutLen, uint8_t * p_pu8DataIn, uint8_t p_u8DataInLen, e_I2CStop_t p_eStop, fpvI2CCallback p_pfCallback)
{
   uint32_t l_u32ErrCode;
   app_twi_transfer_t l_sDataTransfer[XFER_RW];
   
   /* Write first */
   l_sDataTransfer[0u].operation = APP_TWI_WRITE_OP(p_u8Address);
   l_sDataTransfer[0u].p_data = p_pu8DataOut;
   l_sDataTransfer[0u].length = p_u8DataOutLen;
   l_sDataTransfer[0u].flags = p_eStop;
   /* Read next */
   l_sDataTransfer[1u].operation = APP_TWI_READ_OP(p_u8Address);
   l_sDataTransfer[1u].p_data = p_pu8DataIn;
   l_sDataTransfer[1u].length = p_u8DataInLen;
   l_sDataTransfer[1u].flags = I2C_STOP;
     	
   vI2C_Enable(I2C_ENABLE);
   
   /* Write then Read */
   l_u32ErrCode = app_twi_perform(&g_sI2CInstance, &l_sDataTransfer[0u], 1, NULL);   
   HAL_I2C_ERROR_CHECK(l_u32ErrCode);
   if(l_u32ErrCode != NRF_SUCCESS)
   {
      vI2CErrorLog(l_u32ErrCode);
   }
   else
   {
      if(p_pfCallback != NULL)
      {
         (*p_pfCallback)(NULL);
      }
      
      l_u32ErrCode = app_twi_perform(&g_sI2CInstance, &l_sDataTransfer[1u], 1, NULL);
      HAL_I2C_ERROR_CHECK(l_u32ErrCode);
      if(l_u32ErrCode != NRF_SUCCESS)
      {
         vI2CErrorLog(l_u32ErrCode);
      }
   }
      

            
   vI2C_Enable(I2C_DISABLE);
   
	return l_u32ErrCode;
}
/**@brief Function to write with I2C module.
 *
 * @details Description
 * @param[in]  p_u8Address : Address of the I2C module
 *             p_pu8Data : Data you would like to send
 *             p_u8DataLength : Length of data you would like to send
 *
 * @return Error Code, could be SUCCESS, ERROR_BUSY, or ERROR_INTERNAL(Hardware)
 */
uint32_t u32Hal_I2C_Write(uint8_t p_u8Address, uint8_t * p_pu8Data, uint8_t p_u8DataLength)
{	
   uint32_t l_u32ErrCode = 0u;
   app_twi_transfer_t l_sDataTransfer[XFER_W] = { 
      APP_TWI_WRITE(p_u8Address, p_pu8Data, p_u8DataLength, I2C_STOP)
   };
   
   vI2C_Enable(I2C_ENABLE);
   
   l_u32ErrCode = app_twi_perform(&g_sI2CInstance, l_sDataTransfer, XFER_W, NULL);
	HAL_I2C_ERROR_CHECK(l_u32ErrCode);
   if(l_u32ErrCode != NRF_SUCCESS)
   {
      vI2CErrorLog(l_u32ErrCode);
   }
      	
   vI2C_Enable(I2C_DISABLE);
   
	return l_u32ErrCode;
}

/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief Static Function to init driver twi nrf.
 */
static void vDrvI2C_Init(void)
{
   vHal_I2C_Init();
}
/**@brief Static Function to uninit driver twi nrf.
 */
static void vDrvI2C_Uninit(void)
{
   vHal_I2C_Uninit();
}

/**@brief Static Function to Enable I2C module.
 */
static void vI2C_Enable(e_I2C_State_t p_eEnable)
{
//   if(p_eEnable == I2C_ENABLE)
//   {
//      vHal_I2C_Init();
//      //nrf_drv_twi_enable(&g_sI2CInstance.twi);   
//   }
//   else
//   {
//      vDrvI2C_Uninit();
//      //nrf_drv_twi_disable(&g_sI2CInstance.twi);
//   }
}

static void vI2CErrorLog(uint32_t l_u32Err)
{
#ifdef LOG_ERROR
   static uint8_t l_u8AlrdyLog = 1u;
   
   if(l_u8AlrdyLog == 0u)
   {
      uint8_t l_au8Data[8u] = { 0u };
      uint32_t l_u32RTC = u32Hal_RTC_TimeStampGet();
      memcpy(&l_au8Data[0u], &l_u32Err, 4u);
      memcpy(&l_au8Data[4u], &l_u32RTC, 4u);
      (void)eMemItf_DbgLogWrite(l_au8Data, 8u);
      l_u8AlrdyLog = 1u;
   }
#endif
}

/************************************************************************
 * End Of File
 ************************************************************************/

 
