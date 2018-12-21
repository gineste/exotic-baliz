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
#ifndef AT_H
#define AT_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _AT_RETURN_VAL_ {
   AT_RET_OK,
   AT_RET_ERROR,
   AT_RET_TIMEOUT,
   AT_RET_END
}e_AT_RetVal_t;

typedef enum _AT_COMMANDS_ {
   AT_CMD_DUMMY,     /* Dummy command */
   AT_CMD_I,         /* Information Get */
   AT_CMD_P,         /* Set Power Mode*/
   AT_CMD_SB,        /* Send Bit */
   AT_CMD_SF,        /* Send Frame */
   AT_CMD_SO,        /* Send Out of Band */
   AT_CMD_CW,        /* Set Continuous Wave */
   AT_CMD_T,         /* Get Temperature in 1/10th */
   AT_CMD_V,         /* Get Voltages in mV */
   AT_CMD_MAX
}e_AT_Commands_t;

typedef void (*fp_vATCallback_t)(e_AT_RetVal_t p_eRet, uint8_t * p_pu8Buffer, uint8_t p_u8Size);

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vAT_Init(void);
void vAT_QueueSend(e_AT_Commands_t p_eCmd, uint8_t * p_pu8Msg, uint8_t p_u8Size, fp_vATCallback_t p_fpCallback);
void vAT_CommandSend(uint8_t * p_pu8Msg, uint8_t p_u8Size, fp_vATCallback_t p_fpCallback);
void vAT_DirectSend(uint8_t * p_pu8Msg, uint8_t p_u8Size, fp_vATCallback_t p_fpCallback);
void vAT_MessageProcess(void);
void vAT_UpdateFrame(const uint8_t p_u8Data);
uint8_t u8AT_PendingCommand(void);
uint8_t u8AT_PendingReply(void);
void vAT_ClearPending(void);
   
#endif /* AT_H */

