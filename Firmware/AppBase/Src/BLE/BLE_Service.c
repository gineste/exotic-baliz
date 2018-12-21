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
 * Date:          06 07 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   BLE Service. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ble.h"
#include "Ble_Service.h"
	 
/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Private type declarations
 ************************************************************************/
 
 /************************************************************************
 * Private function declarations
 ************************************************************************/

/************************************************************************
 * Variable declarations
 ************************************************************************/
 
/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief  Initialise the ble_service.
 *
 * @param  None.
 * @return None.
 */
uint32_t u32Ble_Service_Init(s_Ble_Service_t * p_psService)
{
   uint32_t l_u32ErrCode;

   // Initialize context data
   p_psService->u16ConnHandle 			= BLE_CONN_HANDLE_INVALID;
   p_psService->psCharacteristics 	= NULL;

   // Add UUID service
   BLE_UUID_BLE_ASSIGN(p_psService->sBle_Uuid, p_psService->sBle_Uuid.uuid);

   l_u32ErrCode = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                     &p_psService->sBle_Uuid,
                                     &p_psService->u16ServiceHandle);

   return l_u32ErrCode;
}


/**
 * @brief  Initialise the ble_service.
 *
 * @param[in]   p_psService Service structure.
 * @param[in]   p_sBaseUuid UUID.
 * @return None.
 */
uint32_t u32Ble_Service_Init_Proprietary(s_Ble_Service_t * p_psService, ble_uuid128_t p_sBaseUuid)
{
   uint32_t l_u32ErrCode;
   uint16_t uuid = p_psService->sBle_Uuid.uuid;

   // Initialize service structure
   p_psService->u16ConnHandle = BLE_CONN_HANDLE_INVALID;
   p_psService->psCharacteristics = NULL;
		
   /**@snippet [Adding proprietary Service to S110 SoftDevice] */
   // Add a custom base UUID.
   l_u32ErrCode = sd_ble_uuid_vs_add(&p_sBaseUuid, &p_psService->sBle_Uuid.type);
   if (l_u32ErrCode != NRF_SUCCESS)
   {
      return l_u32ErrCode;
   }

   // Add UUID service
   p_psService->sBle_Uuid.uuid = uuid;

   l_u32ErrCode = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                     &p_psService->sBle_Uuid,
                                     &p_psService->u16ServiceHandle);

   return l_u32ErrCode;
}

/**@brief Function for adding characteristics.
 *
 * @param[in]   p_psService     Service structure.
 * @param[in]   p_pChData  Information needed to initialize the characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t u32Ble_Service_AddCharacteristic(s_Ble_Service_t * p_psService, s_Ble_Service_Char_Data_t * p_pChData)
{
   ble_gatts_char_md_t char_md;
   ble_gatts_attr_md_t cccd_md;
   ble_gatts_attr_t    attr_char_value;
   ble_gatts_attr_md_t attr_md;

   // Initialize pointer for adding characteristic to chained list of characteristics
   s_Ble_Service_Char_Data_t * pInsertChar = p_psService->psCharacteristics;
   s_Ble_Service_Char_Data_t * pSearchChar = p_psService->psCharacteristics;

   memset(&cccd_md, 0, sizeof(cccd_md));

   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
   cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

   memset(&char_md, 0, sizeof(char_md));

   char_md.char_props.read          = ((p_pChData->eRights & BLE_SERVICE_CH_READ) == BLE_SERVICE_CH_READ);
   char_md.char_props.write         = ((p_pChData->eRights & BLE_SERVICE_CH_WRITE) == BLE_SERVICE_CH_WRITE);
   char_md.char_props.write_wo_resp = ((p_pChData->eRights & BLE_SERVICE_CH_WRITE_WO_RESPONSE) == BLE_SERVICE_CH_WRITE_WO_RESPONSE);
   char_md.char_props.notify 			= ((p_pChData->eRights & BLE_SERVICE_CH_NOTIFY) == BLE_SERVICE_CH_NOTIFY);
         
   char_md.p_char_user_desc         = p_pChData->pu8Desc;
   char_md.char_user_desc_size      = (p_pChData->pu8Desc == NULL) ? 0u : strlen((char*)p_pChData->pu8Desc);
   char_md.char_user_desc_max_size  = char_md.char_user_desc_size;
   char_md.p_char_pf                = NULL;
   char_md.p_user_desc_md           = NULL;
   char_md.p_cccd_md                = (char_md.char_props.notify) ? &cccd_md : NULL;
   char_md.p_sccd_md                = NULL;

   memset(&attr_md, 0, sizeof(attr_md));

   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
   BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
		
   attr_md.vloc    = BLE_GATTS_VLOC_STACK; //BLE_GATTS_VLOC_USER;
   attr_md.rd_auth = ((p_pChData->eRights & BLE_SERVICE_CH_READ) == BLE_SERVICE_CH_READ) ? 1 : 0;
   attr_md.wr_auth = 0; //1;
   attr_md.vlen    = 1;

   memset(&attr_char_value, 0, sizeof(attr_char_value));

   p_pChData->sBle_Uuid.type = p_psService->sBle_Uuid.type;
   attr_char_value.p_uuid    = &p_pChData->sBle_Uuid;
   attr_char_value.p_attr_md = &attr_md;
   attr_char_value.init_len  = p_pChData->sData.u16Length;
   attr_char_value.init_offs = 0;
   if ( p_pChData->sData.u16MaxLen == 0 ) 
   {
      attr_char_value.max_len   = p_pChData->sData.u16Length;
   }
   else
   {
      attr_char_value.max_len   = p_pChData->sData.u16MaxLen;
	}
   attr_char_value.p_value   = p_pChData->sData.pu8Data;
		
   /* Insert new characteristics in service structure */
   if(pSearchChar == NULL)
   {
      p_psService->psCharacteristics = p_pChData;
   }
   else
   {
      while(pSearchChar != NULL)
      {
         pInsertChar = pSearchChar;
         pSearchChar = pSearchChar->psNext;
      }
      pInsertChar->psNext = p_pChData;
   }
		
    return sd_ble_gatts_characteristic_add(p_psService->u16ServiceHandle,
                                           &char_md,
                                           &attr_char_value,
                                           &(p_pChData->sHandle));
}

/************************************************************************
 * Private functions
 ************************************************************************/

/************************************************************************
 * End Of File
 ************************************************************************/

 
