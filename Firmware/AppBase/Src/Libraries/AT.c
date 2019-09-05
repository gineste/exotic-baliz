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
 * Date:          14/12/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   AT Interface to Send command. 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "HAL/HAL_UART.h"
#include "HAL/HAL_Timer.h"

#include "GlobalDefs.h"

#include "AT.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define AT_TIMEOUT_MIN     (uint32_t)10000u
#define AT_TIMEOUT_MAX     (uint32_t)60000u

#define AT_END_OF_FRAME    "\r\n"
#define AT_EOF_SIZE        (uint8_t)2u //sizeof(AT_END_OF_FRAME)

#define PREFIX_MSG         "AT"
#define PREFIX_MSG_SIZE    (uint8_t)sizeof(PREFIX_MSG)+4u
#define TX_MSG_SIZE        (uint8_t)28u
#define MAX_SIZE_MSG       (uint8_t)TX_MSG_SIZE + PREFIX_MSG_SIZE

#define QUEUE_SIZE         (uint8_t)10u

#define AT_RX_BUFFER_SIZE  (uint8_t)64u

#define DEFAULT_AT_MSG  {        \
      .eATCmd = AT_CMD_DUMMY,    \
      .au8Msg = { 0u },          \
      .u8MsgSize = 0u,           \
      .fpvCallback = NULL,       \
   }  
   
/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef struct _AT_MSG_QUEUE_ {
   e_AT_Commands_t eATCmd;
   uint8_t au8Msg[TX_MSG_SIZE];
   uint8_t u8MsgSize;
   fp_vATCallback_t fpvCallback;
}s_ATMsg_t;


/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vATTimeOutHandler(void * p_pvParameters);

static void vATQueueInit(void);
static void vATQueueMsg(e_AT_Commands_t p_eCmd, uint8_t * p_au8Message, uint8_t p_u8MsgSize, fp_vATCallback_t p_fpCallback);
static void vATEnqueueMsg(uint8_t * p_au8Message, uint8_t * p_pu8MsgSize, fp_vATCallback_t * p_pfpCallback);
static uint8_t u8ATIsQueueEmpty(void);
static void vConstructMsg(uint8_t p_u8QueueIdx, uint8_t * p_pu8Msg, uint8_t * p_pu8Size);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/ 
static s_ATMsg_t g_sMsgQueue[QUEUE_SIZE] = {
   DEFAULT_AT_MSG, DEFAULT_AT_MSG, DEFAULT_AT_MSG, DEFAULT_AT_MSG, DEFAULT_AT_MSG, 
   DEFAULT_AT_MSG, DEFAULT_AT_MSG, DEFAULT_AT_MSG, DEFAULT_AT_MSG, DEFAULT_AT_MSG  
};
static uint8_t g_u8QueueOutId = 0u;
static uint8_t g_u8QueueInId = 0u;

static const uint8_t g_cau8Cmd[AT_CMD_MAX][PREFIX_MSG_SIZE] = {
   { "AT" },      // AXSIGFOX_CMD_AT,         /* Dummy command */
   { "AT$I=" },   // AXSIGFOX_CMD_AT_I,       /* Information Get */
   { "AT$P=" },   // AXSIGFOX_CMD_AT_P,       /* Set Power Mode*/
   { "AT$SB=" },  // AXSIGFOX_CMD_AT_SB,      /* Send Bit */
   { "AT$SF=" },  // AXSIGFOX_CMD_AT_SF,      /* Send Frame */
   { "AT$SO" },   // AXSIGFOX_CMD_AT_SO,      /* Send Out of Band */
   { "AT$CW=" },  // AXSIGFOX_CMD_AT_CW,      /* Set Continuous Wave */
   { "AT$T?" },   // AXSIGFOX_CMD_AT_T,       /* Get Temperature in 1/10th */
   { "AT$V?" }    // AXSIGFOX_CMD_AT_V,       /* Get Voltages in mV */
};

volatile uint8_t g_u8WaitingReplyAT = 0u;
volatile uint8_t g_u8MsgReceived = 0u;
volatile uint8_t g_u8MultipleReply = 0u;

volatile uint8_t g_u8RxIdx = 0u;
volatile uint8_t g_u8EOFIdx = 0u;

static uint8_t g_au8ATBuffer[AT_RX_BUFFER_SIZE] = { 0u };
static uint8_t g_au8ATEndOfFrame[AT_EOF_SIZE] = { '\r', '\n' }; //AT_END_OF_FRAME;
static fp_vATCallback_t g_fpCallback;

HAL_TIMER_DEF(AT_TimeOutIdx);

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Initialize AT module. 
 * @return None 
 */
void vAT_Init(void)
{
   (void)eHal_Timer_Create(&AT_TimeOutIdx, HAL_TIMER_MODE_SINGLE_SHOT, &vATTimeOutHandler);
   vATQueueInit();
}

/**@brief Send AT command. Message is put in queue and Enqueue when UART is available.
 * @param [in]  p_eCmd           : AT Command type
 * @param [in]  p_pu8Msg         : AT message to send
 * @param [in]  p_u8Size         : Size of AT message
 * @param [in/out] p_fpCallback  : Callback for response
 * @return None
 */
void vAT_QueueSend(e_AT_Commands_t p_eCmd, uint8_t * p_pu8Msg, uint8_t p_u8Size, fp_vATCallback_t p_fpCallback)
{
   vATQueueMsg(p_eCmd, p_pu8Msg, p_u8Size, p_fpCallback);
}

/**@brief Direct send AT command. 
 * @param [in]  p_pu8Msg         : AT message to send
 * @param [in]  p_u8Size         : Size of AT message
 * @param [in/out] p_fpCallback  : Callback for response
 * @return None
 */
void vAT_DirectSend(uint8_t * p_pu8Msg, uint8_t p_u8Size, fp_vATCallback_t p_fpCallback)
{
   uint8_t l_au8Buffer[MAX_SIZE_MSG] = { 0u };
   uint8_t l_u8Size = 0u;
   
   memcpy(l_au8Buffer, p_pu8Msg, p_u8Size);
   l_u8Size = p_u8Size;
   /* End of frame with one of any kind of whitespace (space,tab,newline,etc.) */
   l_au8Buffer[l_u8Size++] = '\r';       /* Increase index for Uart Length */
   
   if(u32Hal_UART_Write(l_au8Buffer, l_u8Size) == 0u)
   {
      g_fpCallback = p_fpCallback;
   }
}

/**@brief Command send AT command. 
 * @param [in]  p_pu8Msg         : AT message to send
 * @param [in]  p_u8Size         : Size of AT message
 * @param [in/out] p_fpCallback  : Callback for response
 * @return None
 */
void vAT_CommandSend(uint8_t * p_pu8Msg, uint8_t p_u8Size, fp_vATCallback_t p_fpCallback)
{
   uint8_t l_au8Buffer[MAX_SIZE_MSG] = { 0u };
   uint8_t l_u8Size = 0u;
   uint32_t l_u32Timeout = AT_TIMEOUT_MAX;
   
   /* Security */
   if(eHal_Timer_Stop(AT_TimeOutIdx) == HAL_TIMER_ERROR_NONE)
   {
      
         memcpy(l_au8Buffer, p_pu8Msg, p_u8Size);
         l_u8Size = p_u8Size;
         
         /* Downlink message */
         if((p_pu8Msg[p_u8Size-1u] == '1') && (p_pu8Msg[p_u8Size-2u] == ','))
         {
            g_u8MultipleReply = 2u;
         }
         else
         {
            l_u32Timeout = AT_TIMEOUT_MIN;
         }
         
         /* End of frame with one of any kind of whitespace (space,tab,newline,etc.) */
         l_au8Buffer[l_u8Size++] = '\r';       /* Increase index for Uart Length */

         /* Clear Cursor and buffer */      
         /* Init Cursor Rx Buffer Idx */
         g_u8RxIdx = 0u;
         g_u8EOFIdx = 0u;
         if(eHal_Timer_Start(AT_TimeOutIdx, l_u32Timeout) == HAL_TIMER_ERROR_NONE)
         {
            if(u32Hal_UART_Write(l_au8Buffer, l_u8Size) == 0u)
            {         
               g_u8WaitingReplyAT = 1u;
               if(p_fpCallback != NULL)
               {
                  g_fpCallback = p_fpCallback;
               }
            }
         }
   }
}

/**@brief Manage message in queue.
 * @return None.
 */
void vAT_MessageProcess(void)
{
   uint8_t l_au8Msg[MAX_SIZE_MSG] = { 0u };
   uint8_t l_u8MsgSize = 0u;
   
   if((g_u8WaitingReplyAT == 1u) || (g_u8MultipleReply != 0u))
   {
      if(g_u8MsgReceived != 0u)
      {
         if(g_u8MultipleReply != 0u)
         {
            g_u8MultipleReply--;
         }
         else
         {
            if(eHal_Timer_Stop(AT_TimeOutIdx) != HAL_TIMER_ERROR_NONE)
            {
               #ifdef DEBUG
                  __nop();
               #endif
            }
            g_u8WaitingReplyAT = 0u;
         }
         
         if((*g_fpCallback) != NULL)
         {
            (*g_fpCallback)(AT_RET_END, g_au8ATBuffer, g_u8RxIdx);
         }
         memset(g_au8ATBuffer, 0u, AT_RX_BUFFER_SIZE);
         g_u8MsgReceived--;
      }
      else
      {  /* Still pending transfer (Wait for reply or Timeout */  }
   }
   else
   {  /* Check if queue is empty */
      if(u8ATIsQueueEmpty() == 0u)
      {
         static fp_vATCallback_t l_fpvCallback = NULL;
         vATEnqueueMsg(l_au8Msg, &l_u8MsgSize, &l_fpvCallback);
         vAT_CommandSend(l_au8Msg, l_u8MsgSize, l_fpvCallback);
      }
      else
      {  /* Put Device in Sleep Mode, initialize queue, what else ? */
         vATQueueInit();         
      }
   }
}

/**@brief Callback function handler for AT command.
 * @param[in] p_u8Data : Input Char.
 * @return None
 */
void vAT_UpdateFrame(const uint8_t p_u8Data)
{
	g_au8ATBuffer[g_u8RxIdx] = p_u8Data;
#if (LOG_SIGFOX == 1)
   PRINT_UART("%c", p_u8Data);
#endif
   
   if(g_au8ATBuffer[g_u8RxIdx] == g_au8ATEndOfFrame[g_u8EOFIdx])
	{
      g_u8EOFIdx++;
      if(g_u8EOFIdx >= AT_EOF_SIZE)
      {
         g_u8MsgReceived++;
         g_u8EOFIdx = 0u;
      }
   }
   
   g_u8RxIdx++;
   if(g_u8RxIdx > AT_RX_BUFFER_SIZE)
   {
      g_u8RxIdx = 0u;
   }
}

/**@brief  Check if there are pending message to transmit.
 * @return 1 if pending command else 0.
 */
uint8_t u8AT_PendingCommand(void)
{
   uint8_t l_u8SomethingInQueue = (u8ATIsQueueEmpty() == 1u)? 0u: 1u;

   return ((l_u8SomethingInQueue == 1u) || (g_u8WaitingReplyAT == 1u))?1u:0u;
}

uint8_t u8AT_PendingReply(void)
{
   return g_u8MultipleReply;
}

void vAT_ClearPending(void)
{
   g_u8MsgReceived = 0u;
   g_u8MultipleReply = 0u;
   g_u8WaitingReplyAT = 0u;
}
/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief  Time out Handler.
 * @param[in] p_pvParameters : Information about timeout
 * @return None.
 */
static void vATTimeOutHandler(void * p_pvParameters)
{
   if(g_fpCallback != NULL)
   {
      (*g_fpCallback)(AT_RET_TIMEOUT, NULL, 0u);
   }
   /* We are not waiting reply anymore */
   g_u8WaitingReplyAT = 0u;
   g_u8MultipleReply = 0u;
}

/**@brief  Initialize of message.
 * @return None.
 */
static void vATQueueInit(void)
{
   g_u8QueueOutId = 0u;
   g_u8QueueInId = 0u;
}

/**@brief  Check if queue of message is empty.
 * @return 1 if Queue is empty else 0.
 */
static uint8_t u8ATIsQueueEmpty(void)
{
   return (g_u8QueueOutId == g_u8QueueInId)?1u:0u;
}

/**@brief  Queue new message.
 * @param[in] p_eCmd : Type of command.
 * @param[in] p_au8Message : data to send later.
 * @param[in] p_u8MsgSize : size of data.
 * @return None.
 */
static void vATQueueMsg(e_AT_Commands_t p_eCmd, uint8_t * p_au8Message, uint8_t p_u8MsgSize, fp_vATCallback_t p_fpCallback)
{
   int8_t l_s8Diff = 0;
   
   /* Check Overflow */
   if(g_u8QueueInId >= g_u8QueueOutId)
   {
      l_s8Diff = (int8_t)g_u8QueueOutId - (int8_t)g_u8QueueInId;
   }
   else
   {
      l_s8Diff = ((int8_t)QUEUE_SIZE - (int8_t)g_u8QueueOutId) + (int8_t)g_u8QueueInId;
   }
   
   if(l_s8Diff < QUEUE_SIZE) 
   {
      g_sMsgQueue[g_u8QueueInId].eATCmd = p_eCmd;
      memset(&g_sMsgQueue[g_u8QueueInId].au8Msg[0u], 0u, TX_MSG_SIZE);
      memcpy(&g_sMsgQueue[g_u8QueueInId].au8Msg[0u],p_au8Message, p_u8MsgSize);
      g_sMsgQueue[g_u8QueueInId].u8MsgSize = p_u8MsgSize;
      g_sMsgQueue[g_u8QueueInId].fpvCallback = p_fpCallback;
      
      g_u8QueueInId++;
      if(g_u8QueueInId >= QUEUE_SIZE)
      {
         g_u8QueueInId = 0u;
      }
   }
}

/**@brief  Enqueue message.
 * @param[out] p_au8Message : data to send.
 * @param[out] p_u8MsgSize : size of data.
 * @return None.
 */
static void vATEnqueueMsg(uint8_t * p_au8Message, uint8_t * p_pu8MsgSize, fp_vATCallback_t * p_pfpCallback)
{
   vConstructMsg(g_u8QueueOutId, p_au8Message, p_pu8MsgSize);
   (*p_pfpCallback) = g_sMsgQueue[g_u8QueueOutId].fpvCallback;
   
   /* Clear queue index */
   memset(&(g_sMsgQueue[g_u8QueueOutId].au8Msg[0u]), 0u, TX_MSG_SIZE);
   g_sMsgQueue[g_u8QueueOutId].u8MsgSize = 0u;
   g_sMsgQueue[g_u8QueueOutId].eATCmd = AT_CMD_DUMMY;
   g_sMsgQueue[g_u8QueueOutId].fpvCallback = NULL;
   
   g_u8QueueOutId++;   
   if(g_u8QueueOutId >= QUEUE_SIZE)
   {
      g_u8QueueOutId = 0u;
   }
}

/**@brief  Construct AT message.
 * @param[in]  p_u8QueueIdx : Data to send.
 * @param[out] p_pu8Msg : Message.
 * @param[out] p_pu8Size : Size of message.
 * @return None.
 */
static void vConstructMsg(uint8_t p_u8QueueIdx, uint8_t * p_pu8Msg, uint8_t * p_pu8Size)
{
   uint8_t l_au8Msg[MAX_SIZE_MSG] = { 0u };
   uint8_t l_u8PrefixSize = 0u;
   uint8_t l_u8MsgSize = 0u;
   
   if(   (g_sMsgQueue[p_u8QueueIdx].eATCmd < AT_CMD_MAX)
      && (p_pu8Msg != NULL)
      && (p_pu8Size != NULL) )
   {
      /* Compute prefix command size */
      l_u8PrefixSize = strlen((char*)g_cau8Cmd[g_sMsgQueue[p_u8QueueIdx].eATCmd]);
      /* Copy prefix */
      memcpy(&l_au8Msg[0u], &(g_cau8Cmd[g_sMsgQueue[p_u8QueueIdx].eATCmd][0u]), l_u8PrefixSize); 
      /* Copy message */
      memcpy(&l_au8Msg[l_u8PrefixSize],&(g_sMsgQueue[p_u8QueueIdx].au8Msg[0u]), g_sMsgQueue[p_u8QueueIdx].u8MsgSize);
      /* Compute Message Size */
      l_u8MsgSize = l_u8PrefixSize + g_sMsgQueue[p_u8QueueIdx].u8MsgSize;
      
      memcpy(p_pu8Msg, l_au8Msg, l_u8MsgSize);
      (*p_pu8Size) = l_u8MsgSize;
   }
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/
 

