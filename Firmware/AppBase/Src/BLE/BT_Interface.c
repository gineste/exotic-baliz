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
 * Date:          17/01/2018 (dd MM YYYY)
 * Author:        
 * Description:   Bluetooth interface 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include <Libraries/ES_Protocol/ES_Interfaces.h>
#include <Libraries/ES_Protocol/ES_Protocol.h>
#include <Libraries/ES_Queue.h>
#include "BoardConfig.h"
#include "BLE/BLE_Uart.h"

#include "Application/ES_Commands.h"
#include "GlobalDefs.h"
/* Self include */
#include "BT_Interface.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define BTH_ESI_MAX_FRAME_SIZE      (254u)
#define BTH_ESI_BUFFER_SIZE		   (4u * BTH_ESI_MAX_FRAME_SIZE)

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef enum _BTH_ESI_SIGNALS_ {
	BTH_ESI_NO_SIG			      = 0x0000u,
	BTH_ESI_SIG_INBOX_READY	   = 0x0001u,
	BTH_ESI_SIG_OUTBOX_READY	= 0x0002u,
} eBTH_ESI_Signals_t;
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static uint8_t	u8BT_ESI_SendBuffer(uint8_t * p_pu8Buffer, uint16_t p_u16Length);

static uint8_t u8BT_ESI_CheckFlags(eBTH_ESI_Signals_t flags);
static void vBT_ESI_ClearFlags(eBTH_ESI_Signals_t flags);
static void vBT_ESI_SetFlags(eBTH_ESI_Signals_t flags);

static uint8_t u8BT_ESI_OnInboxWrite(uint8_t * p_pu8Buffer, uint16_t p_u16Length);
static uint8_t u8BT_ESI_OnInboxWriteFinish(void);
static uint8_t u8BT_ESI_OnInboxRead(uint8_t * p_pu8Buffer, uint16_t * p_pu16Length);
static uint8_t u8BT_ESI_OnInboxReadFinish(void);
static uint8_t u8BT_ESI_OnOutboxWrite(uint8_t * p_pu8Buffer, uint16_t p_u16Length);
static uint8_t u8BT_ESI_OnOutboxWriteFinish(void);
static uint8_t u8BT_ESI_OnOutboxRead(uint8_t * p_pu8Buffer, uint16_t * p_pu16Length);
static uint8_t u8BT_ESI_OnOutboxReadFinish(void);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static ESI_Interface_t g_sBthEsiItf;
static ES_Queue_t g_sBthEsiInboxQueue;
static ES_Queue_t g_sBthEsiOutboxQueue;
static uint32_t g_u32BthEsiFlags = BTH_ESI_NO_SIG;
static uint8_t g_au8BthInBuffer[BTH_ESI_MAX_FRAME_SIZE] = {0u};
static uint8_t g_u8BthInBufferSize = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vBT_ITF_Init(void)
{
   /* ES Interface settings */
   static uint8_t l_au8InQueueData[BTH_ESI_BUFFER_SIZE] = { 0u };
   static uint8_t l_au8OutQueueData[BTH_ESI_BUFFER_SIZE] = { 0u };
   
   /* Init transmit buffer */
   static uint8_t l_au8OutWriteBuffer[BTH_ESI_MAX_FRAME_SIZE] = { 0u };
   
   /* Init queue data */
   static uint8_t l_au8InReadBuffer[BTH_ESI_MAX_FRAME_SIZE] = { 0u };
   static ES_Queue_Buffer_t l_sInQueueBuffer =
   {
      .size = BTH_ESI_BUFFER_SIZE,
      .data = l_au8InQueueData
   };

   /* Init queue data */
   static uint8_t l_au8OutReadBuffer[BTH_ESI_MAX_FRAME_SIZE] = { 0u };
   static ES_Queue_Buffer_t l_sOutQueueBuffer = {
      .size = BTH_ESI_BUFFER_SIZE,
      .data = l_au8OutQueueData
   };

   /* Init frame box used with frame received */
   static ESI_FrameBox_t l_sInbox = {
      .read = 			      &u8BT_ESI_OnInboxRead,
      .write = 			   &u8BT_ESI_OnInboxWrite,
      .on_read_finish = 	&u8BT_ESI_OnInboxReadFinish,
      .on_write_finish = 	&u8BT_ESI_OnInboxWriteFinish,
      .buffer_read = {
         .data = l_au8InReadBuffer,
         .len = 0u,
         .size = BTH_ESI_MAX_FRAME_SIZE
      },
      .buffer_write = { /* No write buffer */
         .data = (void*) 0u,
         .len = 0u,
         .size = 0u
      }
   };

   /* Init frame box used with frame received */
   static ESI_FrameBox_t l_sOutbox = {
      .read = &u8BT_ESI_OnOutboxRead,
      .write = &u8BT_ESI_OnOutboxWrite,
      .on_read_finish = &u8BT_ESI_OnOutboxReadFinish,
      .on_write_finish = &u8BT_ESI_OnOutboxWriteFinish,
      .buffer_read = {
         .data = l_au8OutReadBuffer,
         .len = 0u,
         .size = BTH_ESI_MAX_FRAME_SIZE
      },
      .buffer_write = {
         .data = l_au8OutWriteBuffer,
         .len = 0u,
         .size = BTH_ESI_MAX_FRAME_SIZE
      },
   };

   /* Create frame handlers */
   static ESI_FrameHandler_t BTH_ESI_FrameHandler;
   static ESI_FrameSender_t BTH_ESI_FrameSender;
   
   /* Init frame handler */
   BTH_ESI_FrameHandler.handle = &vES_CommandHandler;
   BTH_ESI_FrameSender.send = &u8BT_ESI_SendBuffer;

   /* Init queue in Rx and Tx */
   ES_Queue_Init(&g_sBthEsiInboxQueue, &l_sInQueueBuffer, NULL);
   ES_Queue_Init(&g_sBthEsiOutboxQueue, &l_sOutQueueBuffer, NULL);

   /* Init interface */
   ESI_Init(&g_sBthEsiItf, &l_sInbox, &l_sOutbox, &BTH_ESI_FrameHandler, &BTH_ESI_FrameSender);
}

ESI_ReturnCode_t eBT_ITF_SendFrame(uint8_t p_u8OpCode, uint32_t p_u32Expdt, uint32_t p_u32Recpt, const uint8_t * p_cpu8Payload, uint16_t p_u16Length)
{
   return ESI_NewOutgoingFrame(&g_sBthEsiItf, p_u8OpCode, p_u32Expdt, p_u32Recpt, p_cpu8Payload, p_u16Length);
}

/**
* Callback with data received by Tx characteristic 
* @param p_pu8Buffer[] Data received
* @param p_u16Length Len of data
* @retval none
*/
void vBT_ITF_DataReceived(uint8_t * p_pu8Buffer, uint16_t p_u16Length)
{
	uint16_t l_u16Idx = 0u;
	
   for(l_u16Idx = 0u; l_u16Idx< p_u16Length; l_u16Idx++)
   {
      if(g_u8BthInBufferSize < BTH_ESI_MAX_FRAME_SIZE)
      {         
         #if (LOG_BLE_RX == 1)
            PRINT_T2("%02X", p_pu8Buffer[l_u16Idx]);
         #endif   /* LOG_BLE_RX */
         g_au8BthInBuffer[g_u8BthInBufferSize] = p_pu8Buffer[l_u16Idx];
         ++g_u8BthInBufferSize;
      }
   }
#if (LOG_BLE_RX == 1)
   PRINT_T2("%s","\n");
#endif   /* LOG_BLE_RX */
   
   if(ESP_Receiving != ESI_NewIncomingFrame(&g_sBthEsiItf, g_au8BthInBuffer, g_u8BthInBufferSize))
   {
      g_u8BthInBufferSize = 0u;
      memset(g_au8BthInBuffer, 0u, BTH_ESI_MAX_FRAME_SIZE);
   }
}

void vBT_ITF_IdleTask(void)
{
   if (u8BT_ESI_CheckFlags(BTH_ESI_SIG_INBOX_READY) == 1u)
   {
      if(ESI_Failed == ESI_ProcessIncomingFrame(&g_sBthEsiItf))
      {
         vBT_ESI_ClearFlags(BTH_ESI_SIG_INBOX_READY);
      }
      else{ /* Nothing to do */ }
   }
   
   if (u8BT_ESI_CheckFlags(BTH_ESI_SIG_OUTBOX_READY) == 1u)
   {
      if(ESI_Failed == ESI_ProcessOutgoingFrame(&g_sBthEsiItf))
      {
         vBT_ESI_ClearFlags(BTH_ESI_SIG_OUTBOX_READY);
      }
      else{ /* Nothing to do */ }
   }   
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static uint8_t	u8BT_ESI_SendBuffer(uint8_t * p_pu8Buffer, uint16_t p_u16Length)
{
   uint8_t l_u8ErrCode = (uint8_t)ESI_Failed;

	if(u32BLE_Uart_SendBuffer(p_pu8Buffer, p_u16Length) == 0u)
   {
      l_u8ErrCode = (uint8_t)ESI_Succeed;
   }
   else
   {
      l_u8ErrCode = (uint8_t)ESI_Failed;
   }
   
	return l_u8ErrCode;
}

static uint8_t u8BT_ESI_CheckFlags(eBTH_ESI_Signals_t flags)
{
   return ((g_u32BthEsiFlags & (flags)) == (flags)) ? 1u : 0u;
}

static void vBT_ESI_ClearFlags(eBTH_ESI_Signals_t flags)
{
   g_u32BthEsiFlags &= (eBTH_ESI_Signals_t) ~(flags);
}

static void vBT_ESI_SetFlags(eBTH_ESI_Signals_t flags)
{
   g_u32BthEsiFlags |= (flags);
}


static uint8_t u8BT_ESI_OnInboxWrite(uint8_t * p_pu8Buffer, uint16_t p_u16Length)
{
   ES_Queue_Write(&g_sBthEsiInboxQueue, p_pu8Buffer, p_u16Length);
   return ESI_Succeed;
}

static uint8_t u8BT_ESI_OnInboxWriteFinish(void)
{
   vBT_ESI_SetFlags(BTH_ESI_SIG_INBOX_READY);

   return ESI_Succeed;
}

static uint8_t u8BT_ESI_OnInboxRead(uint8_t * p_pu8Buffer, uint16_t * p_pu16Length)
{
   uint8_t retCode = ESI_Failed;
   
   if(ES_Queue_Read(&g_sBthEsiInboxQueue, p_pu8Buffer, p_pu16Length) == ES_Queue_Succeed)
   {
      retCode = ESI_Succeed;
   }
   else
   {
      retCode = ESI_Failed;
   }

   return retCode;
}

static uint8_t u8BT_ESI_OnInboxReadFinish(void)
{
   return ESI_Succeed;
}

static uint8_t u8BT_ESI_OnOutboxWrite(uint8_t * p_pu8Buffer, uint16_t p_u16Length)
{
   ES_Queue_Write(&g_sBthEsiOutboxQueue, p_pu8Buffer, p_u16Length);

   vBT_ESI_SetFlags(BTH_ESI_SIG_OUTBOX_READY);
   return ESI_Succeed;
}

static uint8_t u8BT_ESI_OnOutboxWriteFinish(void)
{
   return ESI_Succeed;
}

static uint8_t u8BT_ESI_OnOutboxRead(uint8_t * p_pu8Buffer, uint16_t * p_pu16Length)
{
   uint8_t retCode = ESI_Failed;

   if(ES_Queue_Failed != ES_Queue_Read(&g_sBthEsiOutboxQueue, p_pu8Buffer, p_pu16Length))
   {
      retCode = ESI_Succeed;
   }
   else
   {
      (*p_pu16Length) = 0u;
      retCode = ESI_Failed;
   }

   return retCode;
}

static uint8_t u8BT_ESI_OnOutboxReadFinish(void)
{
   return ESI_Succeed;
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


