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
 * Date:          07/08/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of multi constellation GPS ORG1510
 *
 */

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* Self include */
#include "ORG1510.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define CHAR_SOF_DOLLAR          (char)'$'
#define DELAY_WU_FROM_BCKUP_MS   (uint32_t)1000u
#define RESPONSE_DELAY_MS        (uint32_t)100u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vChecksumCompute(uint8_t * p_pu8Frame, uint8_t p_u8Size, uint8_t * p_pchChecksum, uint8_t * p_pu8ChecksumSize);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static s_ORG1510_Context_t g_sORG1510Context = {
   .fp_vTimerDelay_ms_t = NULL,
   .fp_u8UART_Read_t = NULL,
   .fp_u32UART_Write_t = NULL,
   .fp_vGPIO_Set_t = NULL,
   .fp_vGPIO_Clear_t = NULL,
   .u32IO_ForceON = 0xFF,
   .u32IO_RESET = 0xFF
};

static uint8_t g_u8ORG1510Initialized = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/
/**@brief   Function to set the context and configure Pin by default.
 * @param[in] p_sContext
 * @return None
 */
void vORG1510_Init(s_ORG1510_Context_t p_sContext)
{
   if(   (p_sContext.fp_u32UART_Write_t != NULL)
      && (p_sContext.fp_u8UART_Read_t != NULL)
      && (p_sContext.fp_vTimerDelay_ms_t != NULL)
      && (p_sContext.fp_vGPIO_Set_t != NULL)
      && (p_sContext.fp_vGPIO_Clear_t != NULL) )
   {
      g_sORG1510Context = p_sContext;

      (*g_sORG1510Context.fp_vGPIO_Set_t)(g_sORG1510Context.u32IO_RESET);
      (*g_sORG1510Context.fp_vGPIO_Clear_t)(g_sORG1510Context.u32IO_ForceON);

      g_u8ORG1510Initialized = 1u;
   }
}

/**@brief   Command to wakeup MT3333 module of ORG1510.
 * @param[in] p_eWakeUpMode Wakeup Hot, Warn, Cold or FullCold. 
 *                          Or possibility to wake up wo restart with ORG1510_WU_NO_RESTART.
 * @return None
 */
void vORG1510_WakeUp(e_ORG1510_WakeUpMode_t p_eWakeUpMode)
{
   const uint8_t l_cau8CmdHOT[] = "$PMTK101*32\r\n\0";
   const uint8_t l_cau8CmdWARM[] = "$PMTK102*31\r\n\0";
   const uint8_t l_cau8CmdCOLD[] = "$PMTK103*30\r\n\0";
   const uint8_t l_cau8CmdFULLCOLD[] = "$PMTK104*2F\r\n\0";
   char l_achCmd[16u] = { 0u };

   if((g_u8ORG1510Initialized == 1u) && (p_eWakeUpMode < ORG1510_WU_MODE_NB))
   {
      (*g_sORG1510Context.fp_vGPIO_Set_t)(g_sORG1510Context.u32IO_ForceON);

      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(DELAY_WU_FROM_BCKUP_MS);
      switch(p_eWakeUpMode)
      {
         case ORG1510_WU_HOT:
            strcpy(l_achCmd, (char*)l_cau8CmdHOT);
            break;
         case ORG1510_WU_WARM:
            strcpy(l_achCmd, (char*)l_cau8CmdWARM);
            break;
         case ORG1510_WU_COLD:
            strcpy(l_achCmd, (char*)l_cau8CmdCOLD);
            break;
         case ORG1510_WU_FULLCOLD:
            strcpy(l_achCmd, (char*)l_cau8CmdFULLCOLD);
            break;
         default:
            break;
      }
      if(p_eWakeUpMode != ORG1510_WU_NO_RESTART)
      {
         (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      }
   }
}

/**@brief   Option to shutdow module MT3333 of ORG1510.
 * @return None
 */
void vORG1510_Shutdown(void)
{
   char l_achCmd[20u] = "$PMTK225,4*2F\r\n\0";

   //char l_achCmd[20u] = "$PMTK161,0*28\0";//\r\n\0";
   if(g_u8ORG1510Initialized == 1u)
   {
      (*g_sORG1510Context.fp_vGPIO_Clear_t)(g_sORG1510Context.u32IO_ForceON);
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);

      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);
   }
}

/**@brief   Option to enable or disable the fix NMEA output time behind PPS.
 * @param[in] p_u8Enable 0 to disable One PPS else 1 to enable
 * @return None
 */
void vORG1510_OnePPS(uint8_t p_u8Enable)
{
   char l_achCmd[20u] = "$PMTK255,";

   if(g_u8ORG1510Initialized == 1u)
   {
      (p_u8Enable != 0u) ? strcat(l_achCmd, "1*2D\r\n\0") : strcat(l_achCmd, "0*2C\r\n\0");
      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
   }
}

/**@brief   Option to enable or disable SBAS satellite search.
 * @param[in] p_u8Activate 0 to disable SBAS else 1 to enable
 * @return None
 */
void vORG1510_SBAS(uint8_t p_u8Activate)
{
   char l_achCmd[20u] = "$PMTK313,";

   if(g_u8ORG1510Initialized == 1u)
   {
      (p_u8Activate != 0u) ? strcat(l_achCmd, "1*2E\r\n\0") : strcat(l_achCmd, "0*2F\r\n\0");
      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);
   }
}

/**@brief   Option to set the receiver to search specified satellite systems.
 *          The setting will be available when NVRAM data is valid.
 * @param[in] p_u8GPS 0 to disable GPS Constellation else 1 to enable
 * @param[in] p_u8Glonass 0 to disable Glonass Constellation else 1 to enable
 * @param[in] p_u8Galileo 0 to disable Galileo Constellation else 1 to enable
 * @param[in] p_u8Beidou 0 to disable Beidou Constellation else 1 to enable
 * @return None
 */
void vORG1510_Constellation(uint8_t p_u8GPS, uint8_t p_u8Glonass, uint8_t p_u8Galileo, uint8_t p_u8Beidou)
{
   char l_achCmd[22u] = "$PMTK353,";
   uint8_t l_u8ChecksumAscii[2u] = { 0u };
   uint8_t l_u8Size = 0u;

   if(g_u8ORG1510Initialized == 1u)
   {
      (p_u8GPS != 0u) ? strcat(l_achCmd, "1,") : strcat(l_achCmd, "0,");
      (p_u8Glonass != 0u) ? strcat(l_achCmd, "1,") : strcat(l_achCmd, "0,");
      (p_u8Galileo != 0u) ? strcat(l_achCmd, "1,") : strcat(l_achCmd, "0,");
      strcat(l_achCmd, "0,");
      (p_u8Beidou != 0u) ? strcat(l_achCmd, "1") : strcat(l_achCmd, "0");

      /* Compte checksum */
      vChecksumCompute((uint8_t*)l_achCmd, strlen(l_achCmd), l_u8ChecksumAscii, &l_u8Size);

      strcat(l_achCmd, "*");
      strcat(l_achCmd, (char*)l_u8ChecksumAscii);
      strcat(l_achCmd, "\r\n");

      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);
   }
}

/**@brief   Option to query the receiver for the firmware release information.
 * @return None
 */
void vORG1510_Version(void)
{
   char l_achCmd[22u] = "$PMTK605*31\r\n";
   (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
}

/**@brief   Option to set the speed threshold for static navigation.
 *          Note: If the actual speed is below the specified threshold, the output position
 *                will remain the same and the output speed will be zero. If the threshold
 *                value is set to 0, then this function is disabled.
 * @param[in] p_u8Threshold in m/s * 10
 * @return None
 */
void vORG1510_StaticNav(uint8_t p_u8Threshold)
{
   char l_achCmd[22u] = "$PMTK386,";  /* static navigation threshold set to 1.0 m/s */
   uint8_t l_u8ChecksumAscii[2u] = { 0u };
   uint8_t l_u8Size = 0u;
   char l_achParam[2u] = { 0 };

   /* Check param */
   p_u8Threshold = (p_u8Threshold < 1)?1:p_u8Threshold;
   p_u8Threshold = (p_u8Threshold > 20)?20:p_u8Threshold;

   if(p_u8Threshold < 10)
   {
      strcat(l_achCmd, "0.");
      sprintf(l_achParam, "%1d", p_u8Threshold);
      strcat(l_achCmd, l_achParam);
   }
   else
   {
      sprintf(l_achParam, "%2d", p_u8Threshold);
      l_u8Size = strlen(l_achCmd);
      l_achCmd[l_u8Size] = l_achParam[0u];
      strcat(l_achCmd, ".");
      l_u8Size = strlen(l_achCmd);
      l_achCmd[l_u8Size] = l_achParam[1u];
   }

   /* Compte checksum */
   vChecksumCompute((uint8_t*)l_achCmd, strlen(l_achCmd), l_u8ChecksumAscii, &l_u8Size);

   strcat(l_achCmd, "*");
   strcat(l_achCmd, (char*)l_u8ChecksumAscii);
   strcat(l_achCmd, "\r\n");

   (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
   (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);

}

/**@brief   Option to set the HDOP threshold. 
 *          Note: If the HDOP value is larger than this threshold value, then the position 
 *                will not be fixed.
 * @param[in] p_u8Threshold in unit * 10 (HDOP 25.5 -> 255), 0 to disable
 * @return None
 */
void vORG1510_HDOPThresholdSet(uint8_t p_u8Threshold)
{
   char l_achCmd[22u] = "$PMTK356,";
   uint8_t l_u8ChecksumAscii[2u] = { 0u };
   uint8_t l_u8Size = 0u;
   char l_achParam[2u] = { 0 };

   if(p_u8Threshold < 10)
   {
      strcat(l_achCmd, "0.");
      sprintf(l_achParam, "%1d", p_u8Threshold);
      strcat(l_achCmd, l_achParam);
   }
   else
   {
      sprintf(l_achParam, "%2d", p_u8Threshold);
      l_u8Size = strlen(l_achCmd);
      l_achCmd[l_u8Size] = l_achParam[0u];
      strcat(l_achCmd, ".");
      l_u8Size = strlen(l_achCmd);
      l_achCmd[l_u8Size] = l_achParam[1u];
   }

   /* Compte checksum */
   vChecksumCompute((uint8_t*)l_achCmd, strlen(l_achCmd), l_u8ChecksumAscii, &l_u8Size);

   strcat(l_achCmd, "*");
   strcat(l_achCmd, (char*)l_u8ChecksumAscii);
   strcat(l_achCmd, "\r\n");

   (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
   (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);

}

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
                              uint32_t p_u32SecondRunTime, uint32_t p_u32SecondSleepTime)
{
   char l_achCmd[80u] = "$PMTK225,";
   char l_achParam[9u] = { 0 };
   uint8_t l_u8ChecksumAscii[2u] = { 0u };
   uint8_t l_u8Size = 0u;

   if(g_u8ORG1510Initialized == 1u)
   {
      sprintf(l_achParam, "%d", p_u8Type);
      strcat(l_achCmd, l_achParam);

      if(p_u8Type != 0u)
      {
         strcat(l_achCmd, ",") ;
         /* First try */
   //      if(p_u32RunTime >= 1000u)
         {
            sprintf(l_achParam, "%d,", p_u32RunTime);
            strcat(l_achCmd, l_achParam) ;
         }
   //      else
   //      {
   //         strcat(l_achCmd, "1000,");
   //      }
   //      if(p_u32SleepTime >= 1000u)
         {
            sprintf(l_achParam, "%d,", p_u32SleepTime);
            strcat(l_achCmd, l_achParam) ;
         }
   //      else
   //      {
   //         strcat(l_achCmd, "1000,");
   //      }

         /* Second try */
   //      if(p_u32SecondRunTime >= 1000u)
         {
            sprintf(l_achParam, "%d,", p_u32SecondRunTime);
            strcat(l_achCmd, l_achParam) ;
         }
   //      else
   //      {
   //         strcat(l_achCmd, "1000,");
   //      }
   //      if(p_u32SecondSleepTime >= 1000u)
         {
            sprintf(l_achParam, "%d", p_u32SecondSleepTime);
            strcat(l_achCmd, l_achParam) ;
         }
   //      else
   //      {
   //         strcat(l_achCmd, "1000");
   //      }
   }
   else
   {
   }


      vChecksumCompute((uint8_t*)l_achCmd, strlen(l_achCmd), l_u8ChecksumAscii, &l_u8Size);

      strcat(l_achCmd, "*");
      strcat(l_achCmd, (char*)l_u8ChecksumAscii);
      strcat(l_achCmd, "\r\n");

      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);
   }
}

/**@brief   Option to set the NMEA sentence output rates.
 * @param[in] p_u8GLL 0 to disable sentence GLL else 1 to enable
 * @param[in] p_u8RMC 0 to disable sentence RMC else 1 to enable
 * @param[in] p_u8VTG 0 to disable sentence VTG else 1 to enable
 * @param[in] p_u8GGA 0 to disable sentence GGA else 1 to enable
 * @param[in] p_u8GSA 0 to disable sentence GSA else 1 to enable
 * @param[in] p_u8GSV 0 to disable sentence GSV else 1 to enable
 * @param[in] p_u8ZDA 0 to disable sentence ZDA else 1 to enable
 * @return None
 */
void vORG1510_SentencesUpdate(uint8_t p_u8GLL, uint8_t p_u8RMC, uint8_t p_u8VTG,
                              uint8_t p_u8GGA, uint8_t p_u8GSA, uint8_t p_u8GSV, uint8_t p_u8ZDA)
{
   char l_achCmd[60u] = "$PMTK314,";
   uint8_t l_u8ChecksumAscii[2u] = { 0u };
   uint8_t l_u8Size = 0u;
   char l_achParam[3u] = { 0 };

   sprintf(l_achParam, "%d", p_u8GLL);
   strcat(l_achCmd, l_achParam);  strcat(l_achCmd, ",");
   sprintf(l_achParam, "%d", p_u8RMC);
   strcat(l_achCmd, l_achParam);  strcat(l_achCmd, ",");
   sprintf(l_achParam, "%d", p_u8VTG);
   strcat(l_achCmd, l_achParam);  strcat(l_achCmd, ",");
   sprintf(l_achParam, "%d", p_u8GGA);
   strcat(l_achCmd, l_achParam);  strcat(l_achCmd, ",");
   sprintf(l_achParam, "%d", p_u8GSA);
   strcat(l_achCmd, l_achParam);  strcat(l_achCmd, ",");
   sprintf(l_achParam, "%d", p_u8GSV);
   strcat(l_achCmd, l_achParam);  strcat(l_achCmd, ",");
   strcat(l_achCmd, "0,0,0,0,0,0,0,0,0,0,0,");
   sprintf(l_achParam, "%d", p_u8ZDA);
   strcat(l_achCmd, l_achParam);
   strcat(l_achCmd, "0,0,0");

   /* Compte checksum */
   vChecksumCompute((uint8_t*)l_achCmd, strlen(l_achCmd), l_u8ChecksumAscii, &l_u8Size);

   strcat(l_achCmd, "*");
   strcat(l_achCmd, (char*)l_u8ChecksumAscii);
   strcat(l_achCmd, "\r\n");

   (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
   (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);

}

/**@brief   Option to enables or disables FLP/GLP mode.
            The default mode is set at disabled.
 * @param[in] p_u8Enable 0 to disable, else enable
 * @return None
 */
void vORG1510_GLP(uint8_t p_u8Enable)
{
   char l_achCmd[20u] = "$PMTK262,";

   if(g_u8ORG1510Initialized == 1u)
   {
      (p_u8Enable != 0u) ? strcat(l_achCmd, "2*2A\r\n\0") : strcat(l_achCmd, "0*28\r\n\0");
      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);
   }
}

/**@brief   Option to set the solution priority either a fast TTFF or a high position accuracy.
 *          The default is set at the high position accuracy.
 * @param[in] p_u8TTFForHPA 0 to use fast TTFF priority, else High Accuracy Positionning is selected
 * @return None
 */
void vORG1510_SolutionPriority(uint8_t p_u8TTFForHPA)
{
   char l_achCmd[20u] = "$PMTK257,";

   if(g_u8ORG1510Initialized == 1u)
   {
      (p_u8TTFForHPA != 0u) ? strcat(l_achCmd, "1*2F\r\n\0") : strcat(l_achCmd, "0*2E\r\n\0");
      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
      (*g_sORG1510Context.fp_vTimerDelay_ms_t)(RESPONSE_DELAY_MS);
   }
}

/**@brief   Option to test command, checksum is automaticaly computed do not include '*' and checksum.
 *          Note : Max Size of full sentence is 100 chars !
 * @param[in] p_pau8Sentence Pointer to the sentence's array
 * @param[in] p_u8Size Size of the sentence
 * @return None
 */
void vORG1510_Test(uint8_t * p_pau8Sentence, uint8_t p_u8Size)
{
   char l_achCmd[100u] = { '\0' };
   uint8_t l_u8ChecksumAscii[2u] = { 0u };
   uint8_t l_u8Size = 0u;

   if( (p_pau8Sentence != NULL) && (p_u8Size != 0u) )
   {
      memcpy(l_achCmd, p_pau8Sentence, p_u8Size);

      /* Compte checksum */
      vChecksumCompute((uint8_t*)l_achCmd, strlen(l_achCmd), l_u8ChecksumAscii, &l_u8Size);

      strcat(l_achCmd, "*");
      strcat(l_achCmd, (char*)l_u8ChecksumAscii);
      strcat(l_achCmd, "\r\n");

      (void)(*g_sORG1510Context.fp_u32UART_Write_t)((uint8_t*)l_achCmd, strlen(l_achCmd));
   }
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static void vChecksumCompute(uint8_t * p_pu8Frame, uint8_t p_u8Size, uint8_t * p_pchChecksum, uint8_t * p_pu8ChecksumSize)
{
   uint8_t l_u8Idx = 1u;
   uint8_t l_u8Checksum = 0u;

   if((p_pchChecksum != NULL) && (p_pu8ChecksumSize != NULL))
   {
      /* Compte checksum */
      while(l_u8Idx < p_u8Size)
      {
         l_u8Checksum ^= p_pu8Frame[l_u8Idx];
         l_u8Idx++;
      }

      sprintf((char*)p_pchChecksum, "%02X", l_u8Checksum);
      (*p_pu8ChecksumSize) = 2u;
   }
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


