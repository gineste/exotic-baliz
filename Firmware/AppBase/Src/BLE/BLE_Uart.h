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
#ifndef BLE_UART_H
#define BLE_UART_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <ble.h>

#include "ble_nus.h"

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Type definitions
 ************************************************************************/
///**@brief Service event type. */
//typedef enum _BLE_UART_EVT_TYPE_ {
//    BLE_UART_EVT_DATA_RECEIVED,    		/**< Write Data event. */
//    BLE_UART_EVT_DATA_SENT,            /**< Data Written event. */
//} e_Ble_Uart_EventType_t;

/**@brief Redefine NUS event into Exotic Uart Evt */
typedef ble_nus_evt_t s_Ble_Uart_Evt_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vBLE_Uart_ServiceInit(void);
void vBLE_Uart_onBleEvt(ble_evt_t const * p_cpsBleEvent, void * p_pvContext);
uint32_t u32BLE_Uart_SendBuffer(uint8_t * p_pu8Buffer, uint16_t p_u16Length);

#endif /* BLE_UART_H */

