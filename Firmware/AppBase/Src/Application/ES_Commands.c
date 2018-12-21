/* 
 ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
(  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
(____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Date:          18 07 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   ES Commands Interface 
 *
 */
 
/******************************************************************************
 * Include Files
 ******************************************************************************/
#include <stdint.h>
#include <string.h>

#include "GlobalDefs.h"

//#include "ADXL362.h"

//#include "FrameManager.h"
//#include "MemoryInterface.h"
//#include "EventManager.h"
//#include "SensorManager.h"
//#include "AxSigFox.h"
//#include "SigFox.h"
//#include "Buzzer.h"
//#include "Led.h"
//#include "LED_ModeEffects.h"
//#include "OSP.h"
//#include "ORG1411.h"
//#include "VEML6075.h"

#include "Libraries/ES_Protocol/ES_Interfaces.h"
//#include "BLE/BT_Interface.h"
#include "BLE/BLE_Frog.h"

//#include "AT.h"

//#include "Version.h"

//#include "HAL_RTC.h"

//#ifdef DATA_LOGGER_EN
//  #include "ES_Log.h"
//  #include "ES_Log_Itf.h"
//#endif

#include "ES_Commands.h"

/************************************************************************
 * Defines
 ************************************************************************/
#define ACK_SIZE     ((uint8_t)2u)

/************************************************************************
 * Private type declarations
 ************************************************************************/
 
/************************************************************************
 * Private function declarations
 ************************************************************************/

/************************************************************************
 * Variable declarations
 ************************************************************************/
uint8_t g_u8CmdReceived = 0u;
e_ESOpCode_Rcv_t g_eListCmd[255];
uint8_t g_u8Cnt = 0u;

/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief Function called by bt_interface when a message is received by 
 *        smartphone application.
 * @param[in]  p_u8OpCode : Op Code of the command reveiced
 * @param[in]  p_u32Expdt : Source
 * @param[in]  p_u32Recpt : Destination
 * @param[in]  p_pu8Data : Buffer on data received
 * @param[in]  p_u16DataLen : Data lenght
 * @return None
 */
void vES_ReadCommand(uint8_t p_u8OpCode,	uint32_t p_u32Expdt,	uint32_t p_u32Recpt,	uint8_t *p_pu8Data, uint16_t p_u16DataLen)
{
//   uint32_t l_u32Temp = 0u;
//   uint8_t l_au8Temp[2u] = { 0u };
//   uint8_t l_au8Buffer[32u] = { 0u };
//   uint16_t l_u16Thd = 0u;
//   uint32_t l_u32Duration = 0u;
//   uint8_t l_u8SendAck = 0u;
//   uint8_t l_au8Ack[2u] = { 0u };
//#ifdef DATA_LOGGER_EN
//   uint32_t l_u32StartTimestamp = 0u;
//#endif
//   
//   l_au8Ack[0u] = p_u8OpCode;
//   e_ES_Ack_Code_t l_eAckRet = ES_ACK_NOK;
//   
//   g_eListCmd[g_u8CmdReceived++] = (e_ESOpCode_Rcv_t)p_u8OpCode;
//   
//   PRINT_DEBUG("%s","Ble Cmd : ");
//   PRINT_INFO("x%02x\n", p_u8OpCode);
//   
//   switch((e_ESOpCode_Rcv_t)p_u8OpCode)
//   {
//      /* Trigger commands */
//      case SIGFOX_CMD:
//         vSigFox_OnDemandSend();
//         l_u8SendAck = 0u;
//         break;
//      case FIND_ME_CMD:
//         vBuzzerSeqCmd();      
//         vLed_TimedEffectSet(LED_EFFECT_CMD, 1000u);  
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      case SENSOR_CFG_GET:
//         vFrameMngr_ConfigSend();
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      case DEAD_SENSORS_GET:
//         vSensorMngr_SensorsDeadGet(&l_u32Temp);
//         l_au8Buffer[0u] = XMSB_32B_TO_8B(l_u32Temp);
//         l_au8Buffer[1u] = MSB_32B_TO_8B(l_u32Temp);
//         l_au8Buffer[2u] = LSB_32B_TO_8B(l_u32Temp);
//         l_au8Buffer[3u] = XLSB_32B_TO_8B(l_u32Temp);
//         eBT_ITF_SendFrame((uint8_t)DEAD_SENSOR_SEND, NULL, NULL, l_au8Buffer, 4u);      
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      case EVENT_CFG_GET:
//         if(eEventMngr_EventCfgSend() != EVT_ERROR_NONE)
//         {
//            l_eAckRet = ES_ACK_NOK;
//         }
//         else
//         {
//            l_eAckRet = ES_ACK_OK;
//         }
//         l_u8SendAck = 1u;
//         break;
//      case USER_ID_GET:
//         break;
//      case GROUP_ID_GET:
//         break;
//      case SIGFOX_INFO_GET:
//         vAXSigFox_DevicePacGet(&l_au8Buffer[0u],&l_au8Temp[0u]);
//         vAXSigFox_DeviceIdGet(&(l_au8Buffer[l_au8Temp[0u]]),&(l_au8Temp[1u]));
//         eBT_ITF_SendFrame((uint8_t)SIGFOX_INFO_SEND, NULL, NULL, l_au8Buffer, l_au8Temp[0u]+l_au8Temp[1u]);      
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      
//      /* Commands with arg */
//      case TIMESTAMP_SET:
//         l_u32Temp = U8_TO_U32(p_pu8Data[0u],p_pu8Data[1u],p_pu8Data[2u],p_pu8Data[3u]);
//         vHal_RTC_TimeStampSet(l_u32Temp);
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      
//      case NAME_SET:
//         if(eMemItf_BoardNameSet((char*)p_pu8Data, p_u16DataLen) == MEM_ITF_ERROR_NONE)
//         {
//            vBleDevInfo_UpdateName((char*)p_pu8Data, (uint8_t)p_u16DataLen);
//            l_eAckRet = ES_ACK_OK;
//         }
//         else
//         {
//            l_eAckRet = ES_ACK_NOK;
//         }
//         l_u8SendAck = 1u;
//         break;
//      
//      case SENSOR_SET:
//         if(p_u16DataLen == 4u)// Must be size of uint32_t(l_u32SensorCfg) = 4u
//         {
//            l_u32Temp = U8_TO_U32(p_pu8Data[0u],p_pu8Data[1u],p_pu8Data[2u],p_pu8Data[3u]);
//            /* Save new config in Flash */
//            if(eMemItf_SensorConfigSet(l_u32Temp) == MEM_ITF_ERROR_NONE)
//            {  /* Shutdown all Sensors */
//               vSensorMngr_ShutDownAll();
//               /* Update Sensor Cfg */
//               l_u32Temp |= 0x4BF;
//               vSensorMngr_UpdateSensorCfg(l_u32Temp);
//               /* Wake Up desired Sensors */
//               vSensorMngr_WakeUpSensor(l_u32Temp);
//               l_eAckRet = ES_ACK_OK;
//            }
//         }
//         l_u8SendAck = 1u;
//         break;
//      case EVENT_SET:
//         /* Temp code will be deleted */
//         if(   (p_pu8Data[5u] == EVT_OPCODE_SENS_ACCEL_ACT)
//            || (p_pu8Data[5u] == EVT_OPCODE_SENS_ACCEL_INACT) )
//         {
//            if   (p_pu8Data[5u] == EVT_OPCODE_SENS_ACCEL_ACT)
//            {
//               l_u16Thd = (uint16_t)U8_TO_U16(p_pu8Data[7],p_pu8Data[8]);
//               l_u32Duration = (uint32_t)U8_TO_U32(p_pu8Data[11],p_pu8Data[12],p_pu8Data[13],p_pu8Data[14]);         
//               l_u32Duration = (l_u32Duration > UINT16_MAX)?UINT16_MAX:l_u32Duration;
//               (void)eADXL362_ActivityDetectionSet(1u,l_u16Thd,(uint16_t)l_u32Duration);
//            }
//            else if(p_pu8Data[15u] == EVT_OPCODE_SENS_ACCEL_ACT)
//            {
//               l_u16Thd = (uint16_t)U8_TO_U16(p_pu8Data[17],p_pu8Data[18]);
//               l_u32Duration = (uint32_t)U8_TO_U32(p_pu8Data[21],p_pu8Data[22],p_pu8Data[23],p_pu8Data[24]);
//               l_u32Duration = (l_u32Duration > UINT16_MAX)?UINT16_MAX:l_u32Duration;
//               (void)eADXL362_ActivityDetectionSet(1u,l_u16Thd,(uint16_t)l_u32Duration);
//            }else{}
//               
//            if   (p_pu8Data[5u] == EVT_OPCODE_SENS_ACCEL_INACT)
//            {
//               l_u16Thd = (uint16_t)U8_TO_U16(p_pu8Data[7],p_pu8Data[8]);
//               l_u32Duration = (uint32_t)U8_TO_U32(p_pu8Data[11],p_pu8Data[12],p_pu8Data[13],p_pu8Data[14]);         
//               l_u32Duration = (l_u32Duration > UINT16_MAX)?UINT16_MAX:l_u32Duration;
//               (void)eADXL362_InactivityDetectionSet(1u,l_u16Thd,(uint16_t)l_u32Duration);
//            }
//            else if(p_pu8Data[15u] == EVT_OPCODE_SENS_ACCEL_INACT)
//            {
//               l_u16Thd = (uint16_t)U8_TO_U16(p_pu8Data[17],p_pu8Data[18]);
//               l_u32Duration = (uint32_t)U8_TO_U32(p_pu8Data[21],p_pu8Data[22],p_pu8Data[23],p_pu8Data[24]);
//               l_u32Duration = (l_u32Duration > UINT16_MAX)?UINT16_MAX:l_u32Duration;
//               (void)eADXL362_InactivityDetectionSet(1u,l_u16Thd,(uint16_t)l_u32Duration);
//            }else{}
//            l_eAckRet = ES_ACK_OK;
//         }
//         else
//         {
//            if(eEventMngr_CheckFrame(p_pu8Data, (uint8_t)p_u16DataLen) != EVT_ERROR_NONE)
//            {  /* ACK Not OK */
//               l_eAckRet = ES_ACK_NOK;
//            }
//            else
//            {  /* Add Event to Flash Storage */
//               if(eEventMngr_EventSave(p_pu8Data, (uint8_t)p_u16DataLen) != EVT_ERROR_NONE)
//               {  /* ACK Not OK */
//                  l_eAckRet = ES_ACK_NOK;
//               }
//               else
//               {  /* ACK OK */ 
//                  l_eAckRet = ES_ACK_OK; 
//               }
//            }
//         }
//         l_u8SendAck = 1u;
//         break;
//      case EVENT_DELETE:
//         if(eEventMngr_EventDelete(U8_TO_U16(p_pu8Data[0u],p_pu8Data[1u])) == EVT_ERROR_NONE)
//         {  /* ACK OK */
//            l_eAckRet = ES_ACK_OK;
//         }
//         else
//         {  /* ACK Not OK */
//            l_eAckRet = ES_ACK_NOK;
//         }
//         l_u8SendAck = 1u;
//         break;
//      
//      case USER_ID_SET:
//         break;
//      case USER_ID_DELETE:
//         break;
//      
//      case GROUP_ID_SET:
//         break;
//      case GROUP_ID_DELETE:
//         break;
//         
//      /* Data Logger Command */
//   #ifdef DATA_LOGGER_EN
//      case DL_START_LOG:
//         l_u32Temp = U8_TO_U32(p_pu8Data[0u],p_pu8Data[1u],p_pu8Data[2u],p_pu8Data[3u]);
//         l_u32Duration = U8_TO_U32(p_pu8Data[4u],p_pu8Data[5u],p_pu8Data[6u],p_pu8Data[7u]);
//         l_u32StartTimestamp = U8_TO_U32(p_pu8Data[8u],p_pu8Data[9u],p_pu8Data[10u],p_pu8Data[11u]);
//         if(eES_Log_StartLog(l_u32Temp, l_u32Duration, l_u32StartTimestamp,&p_pu8Data[13u],p_pu8Data[12u]) != ES_LOG_ERROR_NONE)
//         {  /* Send directly if we are not in good state, else wait for ack from DataLogger */
//            l_eAckRet = ES_ACK_NOK;
//            l_u8SendAck = 1u;
//         }
//         break;
//      case DL_STOP_LOG:
//         if(ES_LOG_ERROR_NONE != eES_Log_StopLog())
//         {
//            l_eAckRet = ES_ACK_NOK;
//         }
//         else
//         {
//            l_eAckRet = ES_ACK_OK;
//         }
//         l_u8SendAck = 1u;
//         break;
//      case DL_STATUS_GET:         
//         vESLog_StatusInfoSend();
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      case DL_FORMAT_SUPPORT:
//         break;
//      case DL_SELECT_SUPPORT:
//         break;
//      
//      case DL_SUPPORT_INFO_GET:
//         vESLog_SupportInfoSend();
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      case DL_RECORD_INFO_GET:
//         if(eES_Log_RecordInfoSend() != ES_LOG_ERROR_NONE)
//         {
//            l_eAckRet = ES_ACK_NOK;
//         }
//         else
//         {
//            l_eAckRet = ES_ACK_OK;
//         }
//         l_u8SendAck = 1u;
//         break;   
//      case DL_FLAG_ADD:
//         vES_Log_Write(ES_LOG_FLAG, p_pu8Data, p_u16DataLen);
//         break;
//   #endif
//      
//      /* Acknowledges */
//      case ACKNOWLEDGE_GET:
//         switch((e_ESOpCode_Rcv_t)p_pu8Data[0u])
//         {
//            case SENSOR_CFG_GET:
//               vFrameMngr_ConfigReceived();
//               break;
//            default:
//               break;
//         }
//      break;
//      
//      /* Diag Commands */
//      case DIAG_TIMESTAMP_GET:
//         l_u32Temp = u32Hal_RTC_TimeStampGet();
//         l_au8Buffer[0u] = XMSB_32B_TO_8B(l_u32Temp);
//         l_au8Buffer[1u] = MSB_32B_TO_8B(l_u32Temp);
//         l_au8Buffer[2u] = LSB_32B_TO_8B(l_u32Temp);
//         l_au8Buffer[3u] = XLSB_32B_TO_8B(l_u32Temp);
//         (void)eBT_ITF_SendFrame((uint8_t)DIAG_TIMESTAMP_SEND, NULL, NULL, l_au8Buffer, 4u);               
//         break;
//      case MIC_CFG_SET:
//         break;
//      case BLE_CFG_DIAG_SET:
//         break;
//      case SIGFOX_DIAG:
//         if(p_pu8Data[0u] == 1u)
//         {
////YRE            vSigFox_WakeUpFromSleep();
//         }
//         else if(p_pu8Data[0u] == 0u)
//         {
////YRE            vSigFox_SleepModeSet();
//         }
//         else
//         {
//         }
////         if(p_u16DataLen == 6u)
////         {
////            l_u32Temp = U8_TO_U32(p_pu8Data[0u],p_pu8Data[1u],p_pu8Data[2u],p_pu8Data[3u]);
////            vSigFox_DiagCW(l_u32Temp,p_pu8Data[4u],p_pu8Data[5u]);
////         }
////         else
////         {
////            l_u8SendAck = 1u;
////            l_eAckRet = ES_ACK_NOK;
////         }
//         break;
//      case OSP_CFG_SET:
//         switch(p_pu8Data[0u])
//         {
//            case 0:
//               vOSP_SwitchToProtocol(GPS_PROTOCOL_NMEA);
//               vORG1411_GPSModeSet(0u);
//            break;
//            case 1:
//               vNMEAToSirf();
//               vORG1411_GPSModeSet(1u);
////            vOSP_SwitchToProtocol(GPS_PROTOCOL_OSP);
//            break;
//            case 2:
//               vOSP_NMEA_OnlyGGA_Every_5sec();
//            break;
//            case 3:
//               vOSP_DisableAllMessages();
//            break;
//            case 4: 
//               vOSP_PushToFixCfgSet(U8_TO_U32(p_pu8Data[1u],p_pu8Data[2u],p_pu8Data[3u],p_pu8Data[4u]),
//                                    U8_TO_U32(p_pu8Data[5u],p_pu8Data[6u],p_pu8Data[7u],p_pu8Data[8u]),
//                                    U8_TO_U32(p_pu8Data[9u],p_pu8Data[10u],p_pu8Data[11u],p_pu8Data[12u]));
//               break;
//            case 5:
//               vOSP_ActivateGEODETICMessage();
//               break;
//            default:
//               break;
//         }         
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      case UV_DIAG_CMD:
//         if(p_pu8Data[0u] == 0u) // Shutdown
//         {
//            vVEML6075_ShutDown();
//         }
//         else if(p_pu8Data[0u] == 1u) // Power Up
//         {
//            vVEML6075_Init();
//         }
//         else if(p_pu8Data[0u] == 2u)  // Configure
//         {
//            vVEML6075_Configure();
//         }
//         else if(p_pu8Data[0u] == 3u)  // Get RawData
//         {
//            vVEML6075_PollingProcess();
//         }
//         else if(p_pu8Data[0u] == 4u)  // Read ID
//         {
//            vVEML6075_IDRead();
//         }
//         
//         break;
//      case DATA_LOG_DIAG_CMD:
//      #ifdef DATA_LOGGER_EN
//         l_au8Temp[0u] = g_u8Cnt;
//         vES_Log_Write(ES_LOG_DATA, l_au8Temp, 1u);
//         g_u8Cnt++;
//         l_u8SendAck = 1u;
//         l_eAckRet = ES_ACK_OK;
//      #endif
//         break;
//      
//      case BLE_TEST_MODE:
//         (void)eBT_ITF_SendFrame((uint8_t)BLE_TEST_MODE, p_u32Recpt, p_u32Expdt, p_pu8Data, p_u16DataLen);     
//         break;
//      
//      case HW_VERSION_GET:
//         l_au8Buffer[0u] = HW_VERSION_MAJOR;
//         l_au8Buffer[1u] = HW_VERSION_MINOR;
//         l_au8Buffer[2u] = HW_VERSION_REVISION;
//      
//         eBT_ITF_SendFrame((uint8_t)HW_VERSION_SEND, NULL, NULL, l_au8Buffer, 3);
//      
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;     
//         break;
//      
//      case FW_VERSION_NRF_GET:
//         l_au8Temp[0u] = strlen(FW_VERSION);
//         l_au8Temp[1u] = strlen(COMMIT_NUMBER);
//         #ifndef RELEASE   /* So in Debug */      
//            l_au8Buffer[0u] = l_au8Temp[0u] + l_au8Temp[1u] + 1u;
//            memcpy(&l_au8Buffer[1u],FW_VERSION,l_au8Temp[0u]);
//            l_au8Buffer[l_au8Temp[0u]+1u] = '.';
//            memcpy(&l_au8Buffer[l_au8Temp[0u]+2u],COMMIT_NUMBER,l_au8Temp[1u]);
//            PRINT_DEBUG("Version %s\n",&l_au8Buffer[1u]);
//         #else
//            l_au8Buffer[0u] = l_au8Temp[0u];
//            memcpy(&l_au8Buffer[1u],FW_VERSION,l_au8Temp[0u]);
//         #endif
//      
//         eBT_ITF_SendFrame((uint8_t)FW_VERSION_NRF_SEND, NULL, NULL, l_au8Buffer, l_au8Buffer[0u] + 1u);
//      
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;
//         break;
//      
//      #ifdef DATA_LOGGER_EN
//      case FW_VERSION_STM_GET:
//         vES_Log_VersionGet(&l_au8Buffer[0]);
//      
//         eBT_ITF_SendFrame((uint8_t)FW_VERSION_STM_SEND, NULL, NULL, l_au8Buffer, 3);
//      
//         l_eAckRet = ES_ACK_OK;
//         l_u8SendAck = 1u;  
//         break;
//      #endif
//      
//      default:
//         PRINT_WARNING("%s","BLE COMMAND UNKNOWN");
//         break;
//      
//   }
//   
//   if(l_u8SendAck == 1u)
//   {  /* Send Ack */
//      l_au8Ack[1u] = (uint8_t)l_eAckRet;
//      (void)eBT_ITF_SendFrame((uint8_t)ACKNOWLEDGE_SEND, NULL, NULL, l_au8Ack, ACK_SIZE);
//   }
}

void vES_AckSendCommand(uint8_t p_u8OpCode, uint8_t p_u8Acknowledge)
{
//   uint8_t l_au8Ack[2u] = { 0u };
//   
//   l_au8Ack[0u] = p_u8OpCode;
//   l_au8Ack[1u] = p_u8Acknowledge;
//   
//   (void)eBT_ITF_SendFrame((uint8_t)ACKNOWLEDGE_SEND, NULL, NULL, l_au8Ack, ACK_SIZE);
}

/************************************************************************
 * Private functions
 ************************************************************************/

/************************************************************************
 * End Of File
 ************************************************************************/

