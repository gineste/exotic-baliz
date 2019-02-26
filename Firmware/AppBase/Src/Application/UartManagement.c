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
 * Date:          12 07 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   State Machine for UART Management in order to choose 
 *                between SigFox and GPS. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>

#include "BoardConfig.h"
#include "GlobalDefs.h"
#include "board.h"

#include "HAL/HAL_UART.h"

#include "ORG1510/ORG1510.h"

#include "SigFox.h"
#include "AXSIGFOX/AXSigFox.h"

#include "Libraries/AT.h"
#include "Libraries/NMEA.h"

#include "UartManagement.h"

/************************************************************************
 * Defines
 ************************************************************************/
#define UART_SIGFOX_CFG_BAUDRATE       (UART_BaudRate_9600)
#define UART_SIGFOX_CFG_PSEL_TXD       ((uint8_t)SIGFOX_UART_TX)
#define UART_SIGFOX_CFG_PSEL_RXD       ((uint8_t)SIGFOX_UART_RX)
#define UART_SIGFOX_CFG_PSEL_RTS       ((uint8_t)0u)
#define UART_SIGFOX_CFG_PSEL_CTS       ((uint8_t)0u)

#define UART_GPS_CFG_BAUDRATE          (UART_BaudRate_9600)
#define UART_GPS_CFG_PSEL_TXD          ((uint8_t)GPS_UART_TX)
#define UART_GPS_CFG_PSEL_RXD          ((uint8_t)GPS_UART_RX)
#define UART_GPS_CFG_PSEL_RTS          ((uint8_t)0u)  
#define UART_GPS_CFG_PSEL_CTS          ((uint8_t)0u)
   
#define UART_CFG_PARITY                (false)

/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef struct _U_STATEMACHINE_ {
   void (*fp_vEntryPoint)(void);
   void (*fp_vProcess)(void);
   void (*fp_vExitPoint)(void);
}s_USM_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static void vPriorityCheck(void);

static void vUSM_IdleEntry(void);
static void vUSM_IdleProcess(void);
static void vUSM_IdleExit(void);

static void vUSM_SigFoxEntry(void);
static void vUSM_SigFoxProcess(void);
static void vUSM_SigFoxExit(void);

static void vUSM_GpsEntry(void);
static void vUSM_GpsProcess(void);
static void vUSM_GpsExit(void);

/************************************************************************
 * Variable declarations
 ************************************************************************/
/* UART Context for GPS */
static s_UART_Context_t g_sUartGPSCfg_t = {   
   .sPinConfig.u8Rx = UART_GPS_CFG_PSEL_RXD,
   .sPinConfig.u8Tx = UART_GPS_CFG_PSEL_TXD,
   .sPinConfig.u8RTS = UART_GPS_CFG_PSEL_RTS,
   .sPinConfig.u8CTS = UART_GPS_CFG_PSEL_CTS,
   .eFlowCtrl = FLOW_CONTROL_DISABLED,
   .bParity = UART_CFG_PARITY,
   .u32BaudRate = UART_GPS_CFG_BAUDRATE,
   .fctpCallback = &vNMEA_UpdateFrame,       /* User handler */
};

/* UART Context for SigFox */
static s_UART_Context_t g_sUartAxSigFoxCfg_t = {   
   .sPinConfig.u8Rx = UART_SIGFOX_CFG_PSEL_RXD,
   .sPinConfig.u8Tx = UART_SIGFOX_CFG_PSEL_TXD,
   .sPinConfig.u8RTS = UART_SIGFOX_CFG_PSEL_RTS,
   .sPinConfig.u8CTS = UART_SIGFOX_CFG_PSEL_CTS,
   .eFlowCtrl = FLOW_CONTROL_DISABLED,
   .bParity = UART_CFG_PARITY,
   .u32BaudRate = UART_SIGFOX_CFG_BAUDRATE,
   .fctpCallback = &vAXSigFox_UpdateFrame,   /* User handler */
};

static uint8_t g_au8Dummy[UART_RX_FIFO_SIZE] = { 0u };
#ifndef DEBUG
static 
#endif
e_UART_State_t g_eUartState = USM_IDLE;
static e_UART_State_t g_ePrevUartState = USM_IDLE;
static e_UART_State_t g_ePendingUartState = USM_NUMBER;
static uint8_t g_u8SubStateIdle = 0u;  /* SubState of current SM is in Idle */

static const s_USM_t g_sUartStateMachine[USM_NUMBER] = {
   /* Entry,            Process,             Exit */
   {vUSM_IdleEntry,     vUSM_IdleProcess,    vUSM_IdleExit},      /* USM_IDLE    */
   {vUSM_SigFoxEntry,   vUSM_SigFoxProcess,  vUSM_SigFoxExit},    /* USM_SIGFOX  */
   {vUSM_GpsEntry,      vUSM_GpsProcess,     vUSM_GpsExit}        /* USM_GPS     */
};

#if (LOG_UART == 1)
static uint8_t g_au8UartStateString[USM_NUMBER][8u] = 
{ "IDLE\0", "SIGFOX\0", "GPS\0" };
#endif
/************************************************************************
 * Public functions
 ************************************************************************/
/**@brief State Machine Uart Process
 * @return None
 */  
void vUartMngt_Process(void)
{
   /* Check if anything is prioritary */
   vPriorityCheck();
   
   /* Execute Process function */
   if(g_sUartStateMachine[g_eUartState].fp_vProcess != NULL)
   {
      g_sUartStateMachine[g_eUartState].fp_vProcess();
   }/* Else nothing to execute */
}

/**@brief Get current state of UART State Machine.
 * @return State of UART State Machine
 */
e_UART_State_t eUartMngt_StateGet(void)
{
   return g_eUartState;
}

/**@brief Set new mode of Uart management.
 * @param[in] p_eMode : New State of UART
 * @return Error Code
 */  
e_UartMngt_ErrorCode_t eUartMngt_StateSet(e_UART_State_t p_eMode)
{
   e_UartMngt_ErrorCode_t l_eErrorCode = UART_MNG_ERROR_PARAM;
   
   if(   (p_eMode < USM_NUMBER)
      && (g_eUartState != p_eMode) )
   {
      if(   (g_u8SubStateIdle == 0u)
         &&
            (  (  (p_eMode == USM_GPS)
               || (p_eMode == USM_IDLE) )
            && (g_eUartState == USM_SIGFOX)) )
      {
         l_eErrorCode = UART_MNG_ERROR_STATE;
         g_ePendingUartState = p_eMode;   //YRE TEST
      }
      else
      {
         /* Execute Exit point of current */
         if(g_sUartStateMachine[g_eUartState].fp_vExitPoint != NULL)
         {
            g_sUartStateMachine[g_eUartState].fp_vExitPoint();
         }
         /* Store Previous Mode */
         g_ePrevUartState = g_eUartState;
         /* Assign new State */
         g_eUartState = p_eMode;
         
//      #if (EN_LOG == 1)
//         PRINT_INFO("%s","UART State : ");
//         PRINT_STATE("%s\n",g_au8UartStateString[g_eUartState]);  
//      #endif
         
         /* Execute Entry point of new State */
         if(g_sUartStateMachine[g_eUartState].fp_vEntryPoint != NULL)
         {
            g_sUartStateMachine[g_eUartState].fp_vEntryPoint();
         }
         l_eErrorCode = UART_MNG_ERROR_NONE;         
      }    
   }
   else
   {
      l_eErrorCode = UART_MNG_ERROR_PARAM;
   }
   
   return l_eErrorCode;
}

e_UartMngt_ErrorCode_t eUartMngt_UpdateBaudRate(e_UART_State_t p_eState, e_UART_BaudRate_t p_eBaudRate, uint8_t p_u8CfgNow)
{
   e_UartMngt_ErrorCode_t l_eErrCode = UART_MNG_ERROR_PARAM;
   
   /* NOTE Update Baud Rate before switching to mode */
   if(p_eState == USM_SIGFOX)
   {
      g_sUartAxSigFoxCfg_t.u32BaudRate = p_eBaudRate;
      if(p_u8CfgNow == 1u)
      {
         vHal_UART_Configure(g_sUartAxSigFoxCfg_t);
      }
      l_eErrCode = UART_MNG_ERROR_NONE;
   }
   else if(p_eState == USM_GPS)
   {
      g_sUartGPSCfg_t.u32BaudRate = p_eBaudRate;
      if(p_u8CfgNow == 1u)
      {
         vHal_UART_Configure(g_sUartGPSCfg_t);
      }
      l_eErrCode = UART_MNG_ERROR_NONE;
   }
   else
   {
      l_eErrCode = UART_MNG_ERROR_PARAM;
   }
   
   return l_eErrCode;
}
/************************************************************************
 * Private functions
 ************************************************************************/
static void vPriorityCheck(void)
{
   if(   (g_u8SubStateIdle == 1u)
      && (g_eUartState == USM_SIGFOX) )
   {
      if(g_ePendingUartState != USM_NUMBER)
      {
         eUartMngt_StateSet(g_ePendingUartState);
         g_ePendingUartState = USM_NUMBER;
      }
      else
      {
         /* Go back in previous mode */
         eUartMngt_StateSet(g_ePrevUartState);
      }
      //g_ePrevUartState = USM_IDLE;
   }
}

static void vUSM_IdleEntry(void)
{
   /* Close pending Uart ComPort open */
   vHal_UART_Close();
   g_u8SubStateIdle = 1u;
}
static void vUSM_IdleProcess(void)
{
   /* Nothing to do, waiting for new mode */
}
static void vUSM_IdleExit(void)
{
   /* Nothing to do */
   g_u8SubStateIdle = 0u;
}

static void vUSM_SigFoxEntry(void)
{
   vAT_ClearPending();
   /* Configure UART for SigFox */
   vHal_UART_Configure(g_sUartAxSigFoxCfg_t);
   (void)u8Hal_UART_Read(g_au8Dummy);
   /* WakeUp SigFox */
   vAXSigFox_WakeUpFromDeepSleep();
   
   g_u8SubStateIdle = 0u;   
}
static void vUSM_SigFoxProcess(void)
{
   /* Process pending command in queue */
   if((u8AT_PendingCommand() == 0u) && (u8AT_PendingReply() == 0u))
   {
      g_u8SubStateIdle = 1u;
   }  /* Processing AT commands */
   vAT_MessageProcess();
}
static void vUSM_SigFoxExit(void)
{
   /* Put SigFox in DeepSleep or Sleep Mode */
   vAXSigFox_DeepSleepModeSet();  /* -> Go back in SigFox since queue is no more empty... */
   /* Close current Uart ComPort */
   vHal_UART_Close();
   g_u8SubStateIdle = 0u;
}

static void vUSM_GpsEntry(void)
{
   /* Configure UART for GPS */
   vHal_UART_Configure(g_sUartGPSCfg_t);
   
   (void)u8Hal_UART_Read(g_au8Dummy);
   /* Switch ON GPS */
   vORG1510_WakeUp(ORG1510_WU_NO_RESTART);
}
static void vUSM_GpsProcess(void)
{   
   /* GPS Frame Check */
   vNMEA_FrameProcessing();

   g_u8SubStateIdle = 1u;
}
static void vUSM_GpsExit(void)
{
   /* Switch OFF GPS */
   vORG1510_Shutdown();
   
   /* Run Process one more time, just in case buffer is not empty */
   vNMEA_FrameProcessing();

   /* Close current Uart ComPort */
   vHal_UART_Close();
   g_u8SubStateIdle = 0u;
}

/************************************************************************
 * End Of File
 ************************************************************************/

