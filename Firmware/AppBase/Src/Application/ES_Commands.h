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
#ifndef ES_COMMANDS_H
#define ES_COMMANDS_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _ES_OP_CODE_SEND_CMD_ {
   /* Commands with arg */   
   CYCLIC_DATA_SEND        = 0x01,     /* Cmd to send every second once connected */  
   
   SENSOR_CFG_SEND         = 0x10,     /* Cmd to send Sensor Config */
   DEAD_SENSOR_SEND        = 0x11, 
   
   EVENT_CFG_SEND          = 0x20,     /* Cmd to send Event Config */
   EVENT_RAISED            = 0x21,     /* Cmd to send if connected when event is raised */
   
   USER_ID_SEND            = 0x30,     /* Send User ID */
   GROUP_ID_SEND           = 0x31,     /* Send Group ID */
   
   SIGFOX_INFO_SEND        = 0x40,     /* Send SigFox Device PAC and Device ID */
   
   /* Acknowledges */
   ACKNOWLEDGE_SEND        = 0xA0,
   
   /* DataLogger */
   DL_STATUS_SEND          = 0xB0,
   DL_RECORD_SEND          = 0xB1,     /* Send Current Record infomartion */
   DL_SUPPORT_INFO_SEND    = 0xB2,     /* Send support information */
   
   /* Diag Commands */
   DIAG_TIMESTAMP_SEND     = 0xD0,     /* Send Current TimeStamp */
   
    /* Info Send */
   FW_VERSION_NRF_SEND     = 0xE0,     /* Send NRF52 firmware version */
   FW_VERSION_STM_SEND     = 0xE1,     /* Send STM32 firmware version */
   HW_VERSION_SEND         = 0xEA,     /* Send hardware version */
   
   /* Diag */
}e_ESOpCode_Send_t;

typedef enum _ES_OP_CODE_RCV_CMD_ {   
   /* Trigger commands */
   SIGFOX_CMD              = 0x01,     /* Cmd receive when application would like to send sigfox msg on demand */ 
   FIND_ME_CMD             = 0x02,     /* Run a specific sequence of Buzzer and LED Color */
   
   SENSOR_CFG_GET          = 0x10,     /* Cmd receive when application want current sensor config */ 
   DEAD_SENSORS_GET        = 0x11,     /* */
   
   EVENT_CFG_GET           = 0x20,     /* Cmd receive when application want current event config */ 
   
   USER_ID_GET             = 0x30,     /* Get User ID */
   GROUP_ID_GET            = 0x31,     /* Get Group ID */
   
   SIGFOX_INFO_GET         = 0x40,     /* Get SigFox ID and PAC */
   
   /* Commands with arg */
   
   SENSOR_SET              = 0x60,     /* Cmd receive when application send sensor config */ 
   TIMESTAMP_SET           = 0x61,     /* Set Current Time of Smartphone */
   NAME_SET                = 0x62,     /* Set Name of Device */
   
   EVENT_SET               = 0x70,     /* Cmd receive when application send event config */ 
   EVENT_DELETE            = 0x71,     /* Delete event by ID */
   
   USER_ID_SET             = 0x80,     /* Set User ID */
   USER_ID_DELETE          = 0x81,     /* Delete current User ID */
   
   GROUP_ID_SET            = 0x87,     /* Set Group ID */
   GROUP_ID_DELETE         = 0x88,     /* Delete Group ID */ 
   
   /* Acknowledges */
   ACKNOWLEDGE_GET         = 0xA0,     /* Acknowledge receive when application got sensor config */ 
   
   /* DataLogger commands */
   DL_START_LOG            = 0xB0,     /* Start Logging on DataLogger */
   DL_STOP_LOG             = 0xB1,     /* Stop Logging on DataLogger */
   DL_STATUS_GET           = 0xB2,     /* Get DataLogger Status */
   DL_FORMAT_SUPPORT       = 0xB3,     /* Send Format command to DataLogger */
   DL_SELECT_SUPPORT       = 0xB4,     /* Select support for DataLogger */

   DL_SUPPORT_INFO_GET     = 0xB6,     /* Get support information */
   DL_RECORD_INFO_GET      = 0xB7,     /* Get Current Record Info */
   DL_FLAG_ADD             = 0xB8,     /* Add Flag in current record */
      
   /* Info Get */
   FW_VERSION_NRF_GET      = 0xE0,     /* Get NRF52 firmware version */
   FW_VERSION_STM_GET      = 0xE1,     /* Get STM32 firmware version */
   HW_VERSION_GET          = 0xEA,     /* Get hardware version */
   
   /* Diag Commands */
   DIAG_TIMESTAMP_GET      = 0xD0,     /* Get Current TimeStamp */
   MIC_CFG_SET             = 0xD1,     /* Set Threshold/Time detection of Microphone */
   SIGFOX_DIAG             = 0xD5,     /* SigFox Diag Commands */
   BLE_CFG_DIAG_SET        = 0xD7,     /* Change BLE parameters */ 
   OSP_CFG_SET             = 0xD8,     /* Change Push-To-Fix parameters */ 
   UV_DIAG_CMD             = 0xD9,     /* VEML6075 Diag */ 
   DATA_LOG_DIAG_CMD       = 0xDA,     /* DataLogger Diag */ 
   BLE_TEST_MODE           = 0x9F,     /* TEST BLE Charge */ 
   TEST_MODE               = 0xFE,     /* TEST Diag */ 
}e_ESOpCode_Rcv_t;

typedef enum _ES_ACK_ {
   ES_ACK_NOK = 0x00,
   ES_ACK_OK = 0x01,
}e_ES_Ack_Code_t;

typedef enum _OP_CODE_SENSORS_ {
   OP_CODE_TEMPERATURE = 0x01,
   OP_CODE_PRESSURE,
   OP_CODE_HUMIDITY,
   OP_CODE_UV_INDEX,
   OP_CODE_BRIGHTNESS,
   OP_CODE_ACCELEROMETER,
   OP_CODE_GPS,
   OP_CODE_MICROPHONE,
   OP_CODE_CAPACITIVE,
   OP_CODE_CENTRAL,
   OP_CODE_BATTERY,
   OP_CODE_GGA_LOGGING,
   OP_CODE_LAST_SENSOR,
}e_OpCodeSensors_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vES_ReadCommand(uint8_t p_u8OpCode,	uint32_t p_u32Expdt,	uint32_t p_u32Recpt,	uint8_t *p_pu8Data, uint16_t p_u16DataLen);
void vES_AckSendCommand(uint8_t p_u8OpCode, uint8_t p_u8Acknowledge);

#endif /* ES_COMMANDS_H */

