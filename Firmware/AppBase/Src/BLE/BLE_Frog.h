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
#ifndef BLE_FROG_H
#define BLE_FROG_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _BLE_FROG_EVENT_ {
   BLE_EVENT_INIT = 0u,
   BLE_EVENT_CONNECTED,
   BLE_EVENT_DISCONNECTED,
   BLE_EVENT_TIMEOUT,
   BLE_EVENT_ERROR
}e_BleFrog_Event_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vBLE_Frog_Init(void);
void vBLE_Frog_AdvertisingStart(void);
void vBLE_Frog_AdvertisingStop(void);
e_BleFrog_Event_t eBleFrog_EventGet(void);
void vBLE_Frog_SleepModeEnter(void);

void log_init(void);
void timers_init(void);
void application_timers_start(void);


    
#endif /* BLE_FROG_H */

