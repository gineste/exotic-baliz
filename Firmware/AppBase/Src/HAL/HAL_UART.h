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
#ifndef HAL_UART_H
#define HAL_UART_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "nrf_uart.h"   /* For Baud rate */

/************************************************************************
 * Defines
 ************************************************************************/
#define UART_RX_FIFO_SIZE     ((uint16_t)(256u))

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef void (*fp_vUartRxCallback_t)(uint8_t p_u8RxBuffer);
 
typedef struct _UART_PINS_ {
   uint8_t u8Rx;
   uint8_t u8Tx;
   uint8_t u8RTS;
   uint8_t u8CTS; 
}s_UART_Pin_Config_t;

/* Just a copy paste from nRF of Flow Control */
typedef enum _UART_FLOW_CTRL_{
   FLOW_CONTROL_DISABLED, /**< UART Hw Flow Control is disabled. */
   FLOW_CONTROL_ENABLED,  /**< Standard UART Hw Flow Control is enabled. */
   FLOW_CONTROL_LOW_POWER /**< Specialized UART Hw Flow Control is used. The Low Power setting allows the \nRFXX to Power Off the UART module when CTS is in-active, and re-enabling the UART when the CTS signal becomes active. This allows the \nRFXX to safe power by only using the UART module when it is needed by the remote site. */
} e_UART_FlowCtrl_t;

typedef enum _UART_BAUDRATE_ {
   UART_BaudRate_1200    = NRF_UART_BAUDRATE_1200   ,
   UART_BaudRate_2400    = NRF_UART_BAUDRATE_2400   ,
   UART_BaudRate_4800    = NRF_UART_BAUDRATE_4800   ,
   UART_BaudRate_9600    = NRF_UART_BAUDRATE_9600   ,
   UART_BaudRate_14400   = NRF_UART_BAUDRATE_14400  ,
   UART_BaudRate_19200   = NRF_UART_BAUDRATE_19200  ,
   UART_BaudRate_28800   = NRF_UART_BAUDRATE_28800  ,
   UART_BaudRate_38400   = NRF_UART_BAUDRATE_38400  ,
   UART_BaudRate_57600   = NRF_UART_BAUDRATE_57600  ,
   UART_BaudRate_76800   = NRF_UART_BAUDRATE_76800  ,
   UART_BaudRate_115200  = NRF_UART_BAUDRATE_115200 ,
   UART_BaudRate_230400  = NRF_UART_BAUDRATE_230400 ,
   UART_BaudRate_250000  = NRF_UART_BAUDRATE_250000 ,
   UART_BaudRate_460800  = NRF_UART_BAUDRATE_460800 ,
   UART_BaudRate_921600  = NRF_UART_BAUDRATE_921600 ,
   UART_BaudRate_1000000 = NRF_UART_BAUDRATE_1000000
}e_UART_BaudRate_t;

typedef struct _UART_CONTEXT_ {   
   s_UART_Pin_Config_t sPinConfig;
   e_UART_FlowCtrl_t eFlowCtrl;
   bool bParity;
   e_UART_BaudRate_t u32BaudRate;
   fp_vUartRxCallback_t fctpCallback;
}s_UART_Context_t;


/************************************************************************
 * Public function declarations
 ************************************************************************/
void vHal_UART_Configure(s_UART_Context_t p_sContext);
uint32_t u32Hal_UART_Write(const uint8_t * p_cu8DataBuffer, uint8_t p_u8DataLen);
uint8_t u8Hal_UART_Read(uint8_t * p_pu8DataBuffer);
void vHal_UART_Close(void);

#endif // HAL_UART_H

