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
#ifndef HARDWARE_TEST_H
#define HARDWARE_TEST_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define FRAME_SIZE_W_ARG_MAX        (uint8_t)19u
#define FRAME_SIZE_MAX              (uint8_t)9u
#define START_FRAME_SIZE            (uint8_t)5u
#define CMD_FRAME_SIZE              (uint8_t)(FRAME_SIZE_MAX - START_FRAME_SIZE - 1u)
#define END_FRAME_CHAR              (uint8_t)'\n'

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vHT_PrintHelp(void);
void vHT_Init(void);
void vHT_CheckInput(uint8_t * p_au8Frame, uint8_t p_u8Size);

void vHT_BackgroundProcess(void);

#endif /* HARDWARE_TEST_H */

