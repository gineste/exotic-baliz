/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 */
#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <ble_srv_common.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/
 
/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
/**@brief Service characteritic rights. */
typedef enum _BLE_SERVICE_CH_RIGHTS_
{
	BLE_SERVICE_CH_READ 								= 0x01u,
	BLE_SERVICE_CH_WRITE								= 0x02u,
	BLE_SERVICE_CH_WRITE_WO_RESPONSE		      = 0x04u,
	BLE_SERVICE_CH_NOTIFY                     = 0x08u
} e_Ble_Service_CharRights_t;
	
/**@brief Service configuration. */
typedef enum _BLE_SERVICE_CONFIG_
{
	BLE_SERVICE_CFG_DISABLED 		= 0x00,			/**< Notification Disabled event. */
	BLE_SERVICE_CFG_ENABLED  		= 0x01,			/**< Notification Enabled event. */
} e_Ble_Service_Config_t;

/**@brief Service value type. */
typedef struct _BLE_SERVICE_VALUE_
{
	uint16_t		u16Length;              /**< Data len */
	uint16_t		u16MaxLen;              /**< Data max len (variable len) */
	uint8_t*		pu8Data;                /**< Data pointer */
} s_Ble_Service_Value_t;

/**@brief Service context data. */
typedef struct _BLE_SERVICE_CH_DATA_ {
	ble_uuid_t                 sBle_Uuid;        /**< Service UUID */
	ble_gatts_char_handles_t 	sHandle;          /**< Handles related to the characteristic. */
	s_Ble_Service_Value_t 		sData;            /**< Data   */
	e_Ble_Service_CharRights_t eRights;          /**< Data Rights  */
	uint8_t							*pu8Desc;         /**< Characteristic description  */
	bool								bIsNotificationEnabled;
	struct _BLE_SERVICE_CH_DATA_ *psNext;		   /**< Pointer to next characteristic  */
} s_Ble_Service_Char_Data_t;

typedef struct _BLE_SERVICE_EVT_ {
    uint16_t                  u16EvtType;       /**< Type of event. */
    s_Ble_Service_Char_Data_t	*psCh;	         /**< Src of event. */
} s_Ble_Service_Evt_t;

/**@brief Service event handler type. */
typedef void (*fpv_Ble_Service_EvtHandler_t) (s_Ble_Service_Evt_t * p_pEvt);

// Forward declaration of the s_Ble_Service_t type. 
typedef struct _BLE_SERVICE_ {
   ble_uuid_t                 sBle_Uuid;
   uint16_t                  	u16ServiceHandle;          /**< Handle of Immediate Alert Service (as provided by the BLE stack). */
   uint16_t                  	u16ConnHandle;             /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */	
	s_Ble_Service_Char_Data_t 	*psCharacteristics;        /**< Chained list for characteristics */
	fpv_Ble_Service_EvtHandler_t  sEvtHandler;            /**< Event handler to be called for handling events in the Immediate Alert Service. */
} s_Ble_Service_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
uint32_t u32Ble_Service_Init(s_Ble_Service_t * p_psService);
uint32_t u32Ble_Service_Init_Proprietary(s_Ble_Service_t * p_psService, ble_uuid128_t p_sBaseUuid);
uint32_t u32Ble_Service_AddCharacteristic(s_Ble_Service_t * p_psService, s_Ble_Service_Char_Data_t * p_pChData);

#endif /* BLE_SERVICE_H */
