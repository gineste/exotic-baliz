/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 */
#ifndef BLE_APPLICATION_H
#define BLE_APPLICATION_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

#include "ble_advertising.h"
#include "ble_gap.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define ADV_DATA_SIZE_MAX                 (BLE_GAP_ADV_MAX_SIZE)        /* -> 31 for BLE 4.2, 255 for BLE 5.0 */
#define SR_ADV_DATA_SIZE_MAX              (BLE_GAP_ADV_MAX_SIZE - 3u)   /* -> 31 for BLE 4.2, 255 for BLE 5.0 */
#define ADV_SENS_DATA_SIZE                ((uint8_t)5u)
#define ADV_INFO_SIZE                     ((uint8_t)5u)
#define DEVICE_NAME_SIZE_MAX              (uint8_t)(ADV_DATA_SIZE_MAX - ADV_SENS_DATA_SIZE - ADV_INFO_SIZE - 1u)/* -1 for end char */

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _BLE_APP_EVENT_ {
   BLE_EVENT_INIT = 0u,
   BLE_EVENT_CONNECTED,
   BLE_EVENT_DISCONNECTED,
   BLE_EVENT_TIMEOUT,
   BLE_EVENT_ERROR
}e_Ble_Event_t;

typedef ble_adv_evt_t e_Ble_Adv_State_t;
typedef ble_adv_mode_t e_Ble_Adv_Mode_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vBLE_Init(void);
void vBLE_AdvertisingStart(e_Ble_Adv_Mode_t p_eAdvMode);
void vBLE_AdvertisingStop(void);
e_Ble_Event_t eBLE_EventGet(void);
e_Ble_Adv_State_t eBLE_AdvertiseStateGet(void);
void vBLE_SoftDeviceDisable(void);
void vBLE_SleepModeEnter(void);
void vBLE_SrAdvDataUpdate(uint8_t * p_pu8Data, uint8_t p_u8Size);
void vBLE_AdvDataUpdate(uint8_t * p_pu8Data, uint8_t p_u8Size);
void vBLE_UpdateName(char * p_chDeviceName, uint8_t p_u8Size);
void vBLE_MACAddressGet(uint8_t * p_pau8MAC, uint8_t * p_pu8Size);

#endif /* BLE_APPLICATION_H */

