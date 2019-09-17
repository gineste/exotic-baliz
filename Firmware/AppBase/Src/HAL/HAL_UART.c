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
 * File name:		HAL_UART.c
 * Date:			   04 07 2017 (dd MM YYYY)
 * Author:			Yoann Rebischung
 * Description:	Hardware Abstraction Layer for UART communication module. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>
#include <nrf_drv_uart.h>

#include "board.h"
#include "sdk_config.h"
#include "app_uart.h"

#include "GlobalDefs.h"

#include "HAL_UART.h"

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Private type declarations
 ************************************************************************/
 /* Circular buffer definition */
typedef struct _UART_CIRC_BUFF_{
	uint8_t u8Buffer[UART_RX_FIFO_SIZE];
	uint8_t u8ReadCntr;
	uint8_t u8WriteCntr;
	uint8_t u8Size;
}s_UART_CircularBuffer_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vHal_UART_EventHandler(app_uart_evt_t * p_psUartEvent);
static void vDrvUART_Init(void);
static void vDrvUART_Uninit(void);

/**@brief Macro for calling error handler function specific for HAL_UART
 *        if supplied error code any other than NRF_SUCCESS.
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define HAL_UART_ERROR_CHECK(ERR_CODE)                      \
    do                                                      \
    {                                                       \
        const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
        if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
        {                                                   \
           PRINT_ERROR("%s","ERROR UART");                  \
           vDrvUART_Uninit();                               \
           vDrvUART_Init();                                 \
        }                                                   \
    } while (0)
    
/************************************************************************
 * Variable declarations
 ************************************************************************/
static bool g_bInitialized = false;                         /* Initialized status */
static app_uart_comm_params_t g_sUartCommParams;            /* Uart Cfg */
static fp_vUartRxCallback_t apf_vCallback;                  /* Rx Callback  */
static s_UART_CircularBuffer_t g_sCircularBuffer = { 0u };  /* Circular Buffer for reception */
volatile uint8_t g_u8DataSendingFlag = 0u;                    /* This will mark data sending */
static s_UART_Context_t g_sCurrentContext = { 0u };
   
/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief Function to configure UART module.
 *
 * @details Description
 * @param[in]  p_sContext : All data necessary to configure UART.
 *
 * @return Error Code, could be SUCCESS, NOT_SUPPORTED, or INVALID_STATE.
 */
void vHal_UART_Configure(s_UART_Context_t p_sContext)
{
   uint32_t l_u32ErrCode;
   
   /* Start by closing the current ComPort */
   vHal_UART_Close();
   
   if(g_bInitialized == false)
   {
      /* Assign current context to nRF context */
      memcpy(&g_sCurrentContext , &p_sContext, sizeof(s_UART_Context_t));
      g_sUartCommParams.rx_pin_no = p_sContext.sPinConfig.u8Rx;
      g_sUartCommParams.tx_pin_no = p_sContext.sPinConfig.u8Tx;
      g_sUartCommParams.rts_pin_no = p_sContext.sPinConfig.u8RTS;
      g_sUartCommParams.cts_pin_no = p_sContext.sPinConfig.u8CTS;
      g_sUartCommParams.flow_control = (app_uart_flow_control_t)p_sContext.eFlowCtrl;
      g_sUartCommParams.use_parity = p_sContext.bParity;
      g_sUartCommParams.baud_rate = (uint32_t)p_sContext.u32BaudRate;      /*(nrf_uart_baudrate_t)*/
      
      l_u32ErrCode = app_uart_init(&g_sUartCommParams, NULL, vHal_UART_EventHandler, APP_IRQ_PRIORITY_LOW);
      HAL_UART_ERROR_CHECK(l_u32ErrCode);
      /* Event if it is NULL, it is check in Handler */
      apf_vCallback = p_sContext.fctpCallback;
      
      g_bInitialized = true;
   }
   else
   {
      /* Nothing to do */
   }
}

/**@brief Function to write with UART module.
 *
 * @param[in]  p_cu8DataBuffer : Data you want to write
 * @param[in]  p_u8DataLen : Size of data
 *
 * @return Error Code, could be SUCCESS, NO_MEM, or ERROR_INTERNAL(Hardware)
 */
uint32_t u32Hal_UART_Write(const uint8_t * p_cu8DataBuffer, uint8_t p_u8DataLen)
{
   uint32_t l_u32ErrCode = 1u;
   uint8_t l_u8Size = 0u;
   
   if(g_bInitialized == true)
   {
   #if (LOG_UART == 1)
      PRINT_UART("%s\n",(char*)p_cu8DataBuffer);
   #elif (LOG_GPS == 1)
      PRINT_UART("%s\n",(char*)p_cu8DataBuffer);
   #endif
      
      for(l_u8Size  = 0u; l_u8Size < p_u8DataLen; l_u8Size++)
      {      
         do {
            l_u32ErrCode = app_uart_put(p_cu8DataBuffer[l_u8Size]);
         } while(l_u32ErrCode != NRF_SUCCESS);
      }  
      
      g_u8DataSendingFlag = 1u;
      while(g_u8DataSendingFlag == 1u); /* Wait for data to be sent (cleared by interrupt) */
   }

   return l_u32ErrCode;
}

/**@brief  Read received data.
 *
 * @param[in]  p_pu8DataBuffer : Pointer on buffer to copy received data.
 *
 * @return Number of data read.
 */
uint8_t u8Hal_UART_Read(uint8_t * p_pu8DataBuffer)
{
	uint8_t l_u8DataSizeRead = 0u;
	uint8_t l_u8CurrentChar = 0u;
	
   /* Read circular buffer */
   while( g_sCircularBuffer.u8Size > 0u )
   {
      l_u8CurrentChar = g_sCircularBuffer.u8Buffer[g_sCircularBuffer.u8ReadCntr++];
      
      *p_pu8DataBuffer++ = l_u8CurrentChar;

      g_sCircularBuffer.u8ReadCntr %= UART_RX_FIFO_SIZE;
      g_sCircularBuffer.u8Size--;
      l_u8DataSizeRead++;
   }

   return l_u8DataSizeRead; /* Return how much data readed */
}

void vHal_UART_Close(void)
{
   if(g_bInitialized == true)
   {
      app_uart_close();  
      g_bInitialized = false;
   } 
}

/************************************************************************
 * Private functions
 ************************************************************************/
static void vHal_UART_EventHandler(app_uart_evt_t * p_psUartEvent)
{
	switch ( p_psUartEvent->evt_type )
	{
      case APP_UART_DATA_READY:
         break;
      case APP_UART_FIFO_ERROR:
      case APP_UART_COMMUNICATION_ERROR:
      case APP_UART_TX_EMPTY:
         g_u8DataSendingFlag = 0u;
         break;

      case APP_UART_DATA:
         if((*apf_vCallback) != NULL)
         {  /* Callback function in order to decode each byte */
            (*apf_vCallback)(p_psUartEvent->data.value);
         }

      #if (LOG_UART == 1)
         //PRINT_UART("%c",(p_psUartEvent->data.value));
      #endif

			g_sCircularBuffer.u8Buffer[g_sCircularBuffer.u8WriteCntr++] = p_psUartEvent->data.value;
			g_sCircularBuffer.u8WriteCntr %= UART_RX_FIFO_SIZE;
		
			/* Check buffer is full */
			if ( g_sCircularBuffer.u8WriteCntr == g_sCircularBuffer.u8ReadCntr) 
         {
				/* Delete oldest byte */
				g_sCircularBuffer.u8ReadCntr++;
				g_sCircularBuffer.u8ReadCntr %= UART_RX_FIFO_SIZE;
			}
         else
         {
				g_sCircularBuffer.u8Size++;
			}
			break;
		default:
			break;
	}
}

/**@brief Static Function to init driver uart nrf.
 */
static void vDrvUART_Init(void)
{
   vHal_UART_Configure(g_sCurrentContext);
}
/**@brief Static Function to uninit driver uart nrf.
 */
static void vDrvUART_Uninit(void)
{
   vHal_UART_Close();
}
/************************************************************************
 * End Of File
 ************************************************************************/

