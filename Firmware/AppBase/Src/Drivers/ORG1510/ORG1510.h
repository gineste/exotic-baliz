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
#ifndef ORG1510_H
#define ORG1510_H

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
typedef enum _ORG1510_ERROR_CODE_ {
   ORG1510_ERROR_NONE = 0u,
   ORG1510_ERROR_INIT,
   ORG1510_ERROR_PARAM,
   ORG1510_ERROR_BUSY,
   ORG1510_ERROR_NOT_READY,
   ORG1510_ERROR_COMM,
   ORG1510_ERROR_NOT_FOUND,
}e_ORG1510_Error_t;

typedef struct _ORG1510_CONTEXT_ {
   uint32_t (*fp_u32UART_Write_t)(const uint8_t * p_u8DataBuffer, uint8_t p_u8DataLen);
   uint8_t (*fp_u8UART_Read_t)(uint8_t * p_pu8DataBuffer);
   void (*fp_vTimerDelay_ms_t)(uint32_t p_u32DelayMs);
   void (*fp_vGPIO_Set_t)(uint32_t p_u32GPIONumber);
   void (*fp_vGPIO_Clear_t)(uint32_t p_u32GPIONumber);
   uint32_t u32IO_ForceON;
   uint32_t u32IO_RESET;
}s_ORG1510_Context_t;

typedef enum _ORG1510_WAKEUP_MODE_ {
   ORG1510_WU_NO_RESTART = 0u,
   ORG1510_WU_HOT,
   ORG1510_WU_WARM,
   ORG1510_WU_COLD,
   ORG1510_WU_FULLCOLD,
   ORG1510_WU_MODE_NB
}e_ORG1510_WakeUpMode_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vORG1510_Init(s_ORG1510_Context_t p_sContext);
void vORG1510_WakeUp(e_ORG1510_WakeUpMode_t p_eWakeUpMode);
void vORG1510_Shutdown(void);
void vORG1510_OnePPS(uint8_t p_u8Enable);
void vORG1510_SBAS(uint8_t p_u8Activate);
void vORG1510_Constellation(uint8_t p_u8GPS, uint8_t p_u8Glonass, uint8_t p_u8Galileo, uint8_t p_u8Beidou);
void vORG1510_Version(void);
void vORG1510_StaticNav(uint8_t p_u8Threshold);
void vORG1510_HDOPThresholdSet(uint8_t p_u8Threshold);
void vORG1510_PeriodicModeSet(uint8_t p_u8Type, uint32_t p_u32RunTime, uint32_t p_u32SleepTime, 
                              uint32_t p_u32SecondRunTime, uint32_t p_u32SecondSleepTime);
void vORG1510_SentencesUpdate(uint8_t p_u8GLL, uint8_t p_u8RMC, uint8_t p_u8VTG, 
                              uint8_t p_u8GGA, uint8_t p_u8GSA, uint8_t p_u8GSV, uint8_t p_u8ZDA);
void vORG1510_GLP(uint8_t p_u8Enable);
void vORG1510_SolutionPriority(uint8_t p_u8TTFForHPA);

/* Debug Utility */
void vORG1510_Test(uint8_t * p_pau8Sentence, uint8_t p_u8Size);

#endif /* ORG1510_H */

