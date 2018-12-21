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
#ifndef UART_MNGT_H
#define UART_MNGT_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include "HAL/HAL_Uart.h"

/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _UART_SM_ {
   UART_INIT,
   UART_IDLE,
   UART_GPS,
   UART_SIGFOX,
   UART_ERROR
}e_UART_StateMachine_t;


typedef enum _UART_STATE_ {
   USM_IDLE,
   USM_SIGFOX,
   USM_GPS,
   USM_NUMBER
}e_UART_State_t;

typedef enum _UART_MNG_ERROR_ {
   UART_MNG_ERROR_NONE,
   UART_MNG_ERROR_STATE,
   UART_MNG_ERROR_PARAM   
}e_UartMngt_ErrorCode_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vUartMngt_Process(void);
e_UART_State_t eUartMngt_StateGet(void);
e_UartMngt_ErrorCode_t eUartMngt_StateSet(e_UART_State_t p_eMode);
e_UartMngt_ErrorCode_t eUartMngt_UpdateBaudRate(e_UART_State_t p_eState, e_UART_BaudRate_t p_eBaudRate, uint8_t p_u8CfgNow);

#endif /* UART_MNGT_H */

