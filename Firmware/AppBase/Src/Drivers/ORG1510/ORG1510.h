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
/**@brief   Function to set the context and configure Pin by default.
 * @param[in] p_sContext
 * @return None
 */
void vORG1510_Init(s_ORG1510_Context_t p_sContext);
/**@brief   Command to wakeup MT3333 module of ORG1510.
 * @param[in] p_eWakeUpMode Wakeup Hot, Warn, Cold or FullCold. 
 *                          Or possibility to wake up wo restart with ORG1510_WU_NO_RESTART.
 * @return None
 */
void vORG1510_WakeUp(e_ORG1510_WakeUpMode_t p_eWakeUpMode);
/**@brief   Option to shutdow module MT3333 of ORG1510.
 * @return None
 */
void vORG1510_Shutdown(void);
/**@brief   Option to enable or disable the fix NMEA output time behind PPS.
 * @param[in] p_u8Enable 0 to disable One PPS else 1 to enable
 * @return None
 */
void vORG1510_OnePPS(uint8_t p_u8Enable);
/**@brief   Option to enable or disable SBAS satellite search.
 * @param[in] p_u8Activate 0 to disable SBAS else 1 to enable
 * @return None
 */
void vORG1510_SBAS(uint8_t p_u8Activate);
/**@brief   Option for DGPS.
 * @param[in] p_u8SrcMode : 0 No DGPS Source
 *                          1 RTCM
 *                          2 WAAS
 * @return None
 */
void vORG1510_SetDGPS(uint8_t p_u8SrcMode);
/**@brief   Option to set the receiver to search specified satellite systems.
 *          The setting will be available when NVRAM data is valid.
 * @param[in] p_u8GPS 0 to disable GPS Constellation else 1 to enable
 * @param[in] p_u8Glonass 0 to disable Glonass Constellation else 1 to enable
 * @param[in] p_u8Galileo 0 to disable Galileo Constellation else 1 to enable
 * @param[in] p_u8Beidou 0 to disable Beidou Constellation else 1 to enable
 * @return None
 */
void vORG1510_Constellation(uint8_t p_u8GPS, uint8_t p_u8Glonass, uint8_t p_u8Galileo, uint8_t p_u8Beidou);
/**@brief   Option to query the receiver for the firmware release information.
 * @return None
 */
void vORG1510_Version(void);
/**@brief   Option to set the speed threshold for static navigation.
 *          Note: If the actual speed is below the specified threshold, the output position
 *                will remain the same and the output speed will be zero. If the threshold
 *                value is set to 0, then this function is disabled.
 * @param[in] p_u8Threshold in m/s * 10
 * @return None
 */
void vORG1510_StaticNav(uint8_t p_u8Threshold);
/**@brief   Option to set the HDOP threshold. 
 *          Note: If the HDOP value is larger than this threshold value, then the position 
 *                will not be fixed.
 * @param[in] p_u8Threshold in unit * 10 (HDOP 25.5 -> 255), 0 to disable
 * @return None
 */
void vORG1510_HDOPThresholdSet(uint8_t p_u8Threshold);
/**@brief   This mode allows autonomous power on/off with reduced fix rate to reduce average power
 *          consumption. In periodic mode, the main power supply VCC is still powered, but power distribution
 *          to internal circuits is controlled by the receiver.
 * @param[in] p_u8Type 1 for Periodic backup mode, 2 for Periodic Standby mode
 * @param[in] p_u32RunTime Full Power period in ms
 * @param[in] p_u32SleepTime Standby period in ms
 * @param[in] p_u32SecondRunTime Full Power period in ms for extended acquisition if GNSS
 *                               acquisition fails during p_u32RunTime.
 * @param[in] p_u32SecondSleepTime Standby period (ms) for extended sleep if GNSS acquisition fails during Run_time
 * @return None
 */
void vORG1510_PeriodicModeSet(uint8_t p_u8Type, uint32_t p_u32RunTime, uint32_t p_u32SleepTime, 
                              uint32_t p_u32SecondRunTime, uint32_t p_u32SecondSleepTime);
/**@brief   Option to set the NMEA sentence output rates.
 * @param[in] p_u8GLL 0 to disable sentence GLL else update rate
 * @param[in] p_u8RMC 0 to disable sentence RMC else update rate
 * @param[in] p_u8VTG 0 to disable sentence VTG else update rate
 * @param[in] p_u8GGA 0 to disable sentence GGA else update rate
 * @param[in] p_u8GSA 0 to disable sentence GSA else update rate
 * @param[in] p_u8GSV 0 to disable sentence GSV else update rate
 * @param[in] p_u8GST 0 to disable sentence GST else update rate
 * @param[in] p_u8GRS 0 to disable sentence GRS else update rate
 * @param[in] p_u8ZDA 0 to disable sentence ZDA else update rate
 * @param[in] p_u8MCHN 0 to disable sentence MCHN else update rate
 * @param[in] p_u8DTM 0 to disable sentence DTM else update rate
 * @param[in] p_u8GBS 0 to disable sentence GBS else update rate
 * @return None
 */
void vORG1510_SentencesUpdate(uint8_t p_u8GLL, uint8_t p_u8RMC, uint8_t p_u8VTG, uint8_t p_u8GGA, 
                              uint8_t p_u8GSA, uint8_t p_u8GSV, uint8_t p_u8GRS, uint8_t p_u8GST,
                              uint8_t p_u8ZDA, uint8_t p_u8MCHN,uint8_t p_u8DTM, uint8_t p_u8GBS);
/**@brief   Option to enables or disables FLP/GLP mode.
            The default mode is set at disabled.
 * @param[in] p_u8Enable 0 to disable, else enable
 * @return None
 */
void vORG1510_GLP(uint8_t p_u8Enable);
/**@brief   Option to set the solution priority either a fast TTFF or a high position accuracy.
 *          The default is set at the high position accuracy.
 * @param[in] p_u8TTFForHPA 0 to use fast TTFF priority, else High Accuracy Positionning is selected
 * @return None
 */
void vORG1510_SolutionPriority(uint8_t p_u8TTFForHPA);
/**@brief   Activate AlwaysLocate mode of MT3333/ORG1510 GPS.
 * @param[in] p_u8Enable 1 to enable, 0 to disable
 * @return None
 */
void vORG1510_AlwaysLocate(uint8_t p_u8Enable);
/**@brief   Option to test command, checksum is automaticaly computed do not include '*' and checksum.
 *          Note : Max Size of full sentence is 100 chars !
 * @param[in] p_pau8Sentence Pointer to the sentence's array
 * @param[in] p_u8Size Size of the sentence
 * @return None
 */
void vORG1510_Test(uint8_t * p_pau8Sentence, uint8_t p_u8Size);

#endif /* ORG1510_H */

