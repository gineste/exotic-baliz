/* 
 *  ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 * (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *  ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 * (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Date:          16 01 2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   BLE Uart for nRF5x Devices SDK14 with NUS. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>

#include "app_error.h"

#include "BLE_Uart.h"

/************************************************************************
 * Defines
 ************************************************************************/
//#define BUFFER_MAX_LEN                       255u
//#define BLE_TX_FIFO_DEPTH                    4u

//#define UART_BUFFER_SIZE                     20u
//#define SEG_BUFFER_SIZE                      20u

/************************************************************************
 * Private type declarations
 ************************************************************************/
BLE_NUS_DEF(g_NUSInstance);            /**< BLE NUS service instance. (Nordic Uart Service) */

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vBle_Uart_onWrite(ble_evt_t const * p_cpsBleEvent);
static void vBLE_Uart_onTxComplete(ble_evt_t const * p_cpsBleEvent);
static void vEventHandler(s_Ble_Uart_Evt_t * p_psEvt);

/************************************************************************
 * Variable declarations
 ************************************************************************/

/************************************************************************
 * Public functions
 ************************************************************************/
/**@brief  Initialize the ble_uart.
 * @param  p_sEventHandler handler.
 * @return None.
 */
void vBLE_Uart_ServiceInit(void)
{
   uint32_t l_u32ErrCode;
   ble_nus_init_t l_sNUSInit;
   
   memset(&l_sNUSInit, 0, sizeof(l_sNUSInit));
   
   l_sNUSInit.data_handler = vEventHandler;
   
   l_u32ErrCode = ble_nus_init(&g_NUSInstance, &l_sNUSInit);
   APP_ERROR_CHECK(l_u32ErrCode);
}

/**@brief  BLE Uart Event Manager.
 * @param  p_cpsBleEvent.
 * @param  p_pvContext.
 * @return None.
 */
void vBLE_Uart_onBleEvt(ble_evt_t const * p_cpsBleEvent, void * p_pvContext)
{   
	switch (p_cpsBleEvent->header.evt_id)
	{
//		case BLE_GAP_EVT_CONNECTED:
//			g_sBle_Uart_Service.u16ConnHandle = p_psBleEvent->evt.gap_evt.conn_handle;
//			sd_ble_tx_packet_count_get(g_sBle_Uart_Service.u16ConnHandle, &g_u8AvailableSlots);
//			break;

//		case BLE_GAP_EVT_DISCONNECTED:
//			g_sBle_Uart_Service.u16ConnHandle = BLE_CONN_HANDLE_INVALID;
//			g_sBle_Uart_CharacteristicRx.bIsNotificationEnabled = false;
//		
//			ES_Queue_ClearAll(&g_sBleUartOutputQueue);
//			memset(&g_sBle_Uart_TxInfo, 0u, sizeof(g_sBle_Uart_TxInfo));
//			break;

//		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
//			break;

//		case BLE_GAP_EVT_TIMEOUT:
//			g_sBle_Uart_Service.u16ConnHandle = BLE_CONN_HANDLE_INVALID;
//			g_sBle_Uart_CharacteristicRx.bIsNotificationEnabled = false;

//			ES_Queue_ClearAll(&g_sBleUartOutputQueue);
//			memset(&g_sBle_Uart_TxInfo, 0u, sizeof(g_sBle_Uart_TxInfo));
//			break;

		case BLE_GATTS_EVT_WRITE:
			vBle_Uart_onWrite(p_cpsBleEvent);
			break;
		case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
			break;
		case BLE_GATTS_EVT_TIMEOUT:
			break;
		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
			vBLE_Uart_onTxComplete(p_cpsBleEvent);
			break;

		default:
         // No implementation needed.  
         break;
	}
}

/**@brief  Send buffer over BLE.
 * @param  p_pu8Buffer.
 * @param  p_u16Length.
 * @return l_u32ErrCode.
 */
uint32_t u32BLE_Uart_SendBuffer(uint8_t * p_pu8Buffer, uint16_t p_u16Length)
{
   uint16_t l_u16BufferLen = p_u16Length;
	uint32_t	l_u32ErrCode = 0u;
   
   l_u32ErrCode = ble_nus_string_send(&g_NUSInstance, p_pu8Buffer, &l_u16BufferLen);
   APP_ERROR_CHECK(l_u32ErrCode);
   
	return l_u32ErrCode;
}

/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief Function for handling the Write event.
 * @param[in]   p_cpsBleEvent  Event received from the BLE stack.
 */
static void vBle_Uart_onWrite(ble_evt_t const * p_cpsBleEvent)
{
//   ble_gatts_evt_write_t * l_psEvtWrite = &p_psBleEvt->evt.gatts_evt.params.write;
//   ble_service_char_data_t * l_psCh = g_sBle_Uart_Service.psCharacteristics;
//   ble_service_evt_t evt;
//
//   while(l_psCh)
//   {
//      if((l_psEvtWrite->handle == l_psCh->sHandle.cccd_handle) && (l_psEvtWrite->len == 2))
//      {
//         if (ble_srv_is_notification_enabled(l_psEvtWrite->data))
//         {
//            l_psCh->bIsNotificationEnabled = true;
//         }
//         else
//         {
//            l_psCh->bIsNotificationEnabled = false;
//         }
//         break;	/* EXIT */
//      }
//      
//      if(l_psEvtWrite->handle == l_psCh->sHandle.value_handle)
//      {
//         /* Something new */
//         memcpy(l_psCh->sData.pu8Data, l_psEvtWrite->data, l_psEvtWrite->len);
//         l_psCh->sData.u16Length = l_psEvtWrite->len;
//         if ( g_sBle_Uart_Service.sEvtHandler != NULL )
//         {
//            evt.psCh = l_psCh;
//            evt.u16EvtType = BLE_UART_EVT_DATA_RECEIVED;
//            g_sBle_Uart_Service.sEvtHandler(&evt);
//         }
//         break;	/* EXIT */         
//      }      
//      l_psCh = l_psCh->psNext;  // Next characteristic
//   }		
}

/**@brief Callback on Tx Complete.
 * @param[in]  p_psBleEvent  Event handler.
 * @return None.
 */
static void vBLE_Uart_onTxComplete(ble_evt_t const * p_cpsBleEvent)
{
}

static void vEventHandler(s_Ble_Uart_Evt_t * p_psEvt)
{
   uint8_t l_au8Data[250] = { 0u };
   uint16_t l_u16Size = 0u;
   switch(p_psEvt->type)
   {
      case BLE_NUS_EVT_RX_DATA:
         l_u16Size = p_psEvt->params.rx_data.length;
         memcpy(l_au8Data,p_psEvt->params.rx_data.p_data,l_u16Size);
         u32BLE_Uart_SendBuffer(l_au8Data,l_u16Size);
         break;
      case BLE_NUS_EVT_TX_RDY:
         break;
      case BLE_NUS_EVT_COMM_STARTED:
      case BLE_NUS_EVT_COMM_STOPPED:
      default:
         break;
   }
}
/************************************************************************
 * End Of File
 ************************************************************************/

 
