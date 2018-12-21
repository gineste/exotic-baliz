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
#ifndef BT_INTERFACE_H
#define BT_INTERFACE_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <Libraries/ES_Protocol/ES_Interfaces.h>
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define ESP_BT_ITF 0u

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vBT_ITF_Init(void);
ESI_ReturnCode_t eBT_ITF_SendFrame(uint8_t p_u8OpCode, uint32_t p_u32Expdt, uint32_t p_u32Recpt, const uint8_t * p_cpu8Payload, uint16_t p_u16Length);
void vBT_ITF_DataReceived(uint8_t * p_pu8Buffer, uint16_t p_u16Length);
void vBT_ITF_IdleTask(void);
 
#endif /* BT_INTERFACE_H */

