/* =============================================================================
 ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
(  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
(____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/

  Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.

  Licensees are granted free, non-transferable use of the information. NO
  WARRANTY of ANY KIND is provided. This heading must NOT be removed from
  the file.

  File name:    ExoTime.c
  Date:         02 01 2018
  Author:       Emeric L.
  Description:  EXOTIME - Body file.
============================================================================= */

/* =============================================================================
                                 DEBUG Section
============================================================================= */

/* =============================================================================
                                 Include Files
============================================================================= */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ExoTypes.h"
#include "ExoDefs.h"
#include "ExoErrors.h"
#include "ExoTime.h"

#include "Version.h"

/* =============================================================================
                          Private defines and typedefs
============================================================================= */
enum _LEAP_YEAR_ {
  LEAP_YEAR_YES = 0u, 
  LEAP_YEAR_NO, 
  LEAP_YEAR_MAX 
};

#define AVERAGE_DAY_PER_YEAR    ((float)365.2425)
#define DAY_PER_LEAP_YEAR       ((uint16_t)366)
#define DAY_PER_NOT_LEAP_YEAR   ((uint16_t)365)
#define LEAP_YEAR_PERIOD        ((uint8_t)4)
#define LEAP_YEAR_INDEX         ((uint8_t)LEAP_YEAR_YES)
#define NOT_LEAP_YEAR_INDEX     ((uint8_t)LEAP_YEAR_NO)
#define DAYS_PER_WEEK           ((uint8_t)7)
#define HOURS_PER_DAY           ((uint8_t)24)
#define MINUTES_PER_HOUR        ((uint8_t)60)
#define SECONDS_PER_MINUTE      ((uint8_t)60)
#define SECONDS_PER_HOUR        ((uint16_t)(SECONDS_PER_MINUTE * MINUTES_PER_HOUR))
#define FOURHUNDRED_TO_UNITY    ((uint16_t)(4 * HUNDRED_TO_UNITY))
#define EXOTIME_DAY_STRING_SIZE ((uint8_t)4)
/* =============================================================================
                        Private constants and variables
============================================================================= */
/* Day per month for leap and not leap year. */
static const uint8_t g_cau8LUTDaysPerMonth[LEAP_YEAR_MAX][EXOTIME_MONTHS_PER_YEAR] = {
  {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
  {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};

/* Day of weekend LUT for Sakamoto's methods. */
static const uint8_t g_cau8LUTDayOfWeek[EXOTIME_MONTHS_PER_YEAR] = {
  0u, 3u, 2u, 5u, 0u, 3u, 5u, 1u, 4u, 6u, 2u, 4u
};

/* English short name for days. */
static const char g_cachLUTDayName[DAYS_PER_WEEK][EXOTIME_DAY_STRING_SIZE] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

/* Output buffer for pachMkTimeToAscii(). */
static char g_achTimeAsciiFormat[32];

/* =============================================================================
                        Public constants and variables
============================================================================= */
/* English short name for months. */
const char g_cachLUTMonName[EXOTIME_MONTHS_PER_YEAR][EXOTIME_MONTHS_STRING_SIZE] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};


/* =============================================================================
                        Private function declarations
============================================================================= */

/* =============================================================================
                               Public functions
============================================================================= */

void vCompilationInfoToTimestamp(uint32_t * p_pu32Timestamp)
{
   s_ExoTime_t l_eDataTime = { 0 };
   uint8_t l_u8StrLengthDate = 0u;
   uint8_t l_u8StrLengthTime = 0u;   
   uint32_t l_u32TimeStamp = 0u;
   
   char l_pchDate[20u] = { 0 };
   char l_pchDateMonth[4u] = { 0 };
   char l_pchDateDay[3u] = { 0 };
   char l_pchDateYear[5u] = { 0 };
   char l_pchTime[20u] = { 0 };
   char l_pchTimeHour[3u] = { 0 };
   char l_pchTimeMin[3u] = { 0 };
   char l_pchTimeSec[3u] = { 0 };
   
   l_u8StrLengthDate = strlen(__DATE__);
   l_u8StrLengthTime = strlen(__TIME__);
   memcpy(l_pchDate, __DATE__, l_u8StrLengthDate);
   memcpy(l_pchTime, __TIME__, l_u8StrLengthTime);
   
   memcpy(l_pchDateMonth, &l_pchDate[0u], 3u);
   memcpy(l_pchDateDay, &l_pchDate[4u], 2u);
   memcpy(l_pchDateYear, &l_pchDate[7u], 4u);

   memcpy(l_pchTimeHour, &l_pchTime[0u], 2u);
   memcpy(l_pchTimeMin, &l_pchTime[3u], 2u);
   memcpy(l_pchTimeSec, &l_pchTime[6u], 2u);
   
   l_eDataTime.u16Year = strtol(l_pchDateYear, NULL, 10);   
   l_eDataTime.u8DayOfMonth = strtol(l_pchDateDay, NULL, 10);
   
   if(strncmp(l_pchDateMonth, "Jan", 3) == 0)
   {
      l_eDataTime.u8Month = 1u;
   } 
   else if (strncmp(l_pchDateMonth, "Feb", 3) == 0)
   {
      l_eDataTime.u8Month = 2u;
   }
   else if (strncmp(l_pchDateMonth, "Mar", 3) == 0)
   {
      l_eDataTime.u8Month = 3u;
   }
   else if (strncmp(l_pchDateMonth, "Apr", 3) == 0)
   {
      l_eDataTime.u8Month = 4u;
   }
   else if (strncmp(l_pchDateMonth, "May", 3) == 0)
   {
      l_eDataTime.u8Month = 5u;
   }
   else if (strncmp(l_pchDateMonth, "Jun", 3) == 0)
   {
      l_eDataTime.u8Month = 6u;
   }
   else if (strncmp(l_pchDateMonth, "Jul", 3) == 0)
   {
      l_eDataTime.u8Month = 7u;
   }
   else if (strncmp(l_pchDateMonth, "Aug", 3) == 0)
   {
      l_eDataTime.u8Month = 8u;
   }
   else if (strncmp(l_pchDateMonth, "Sep", 3) == 0)
   {
      l_eDataTime.u8Month = 9u;
   }
   else if (strncmp(l_pchDateMonth, "Oct", 3) == 0)
   {
      l_eDataTime.u8Month = 10u;
   }
   else if (strncmp(l_pchDateMonth, "Nov", 3) == 0)
   {
      l_eDataTime.u8Month = 11u;
   }
   else if (strncmp(l_pchDateMonth, "Dec", 3) == 0)
   {
      l_eDataTime.u8Month = 12u;
   }
   else
   {
      l_eDataTime.u8Month = 0u;
   }
      
   l_eDataTime.u8Hour = strtol(l_pchTimeHour, NULL, 10);   
   l_eDataTime.u8Minute = strtol(l_pchTimeMin, NULL, 10);
   l_eDataTime.u8Second = strtol(l_pchTimeSec, NULL, 10);  
   
   if(eDateToTimestamp(&l_eDataTime, &l_u32TimeStamp) == EXOERRORS_NO)
   {
      if(p_pu32Timestamp != NULL)
      {
         (*p_pu32Timestamp) = l_u32TimeStamp;
      }
   }
   
}

/* =============================================================================
Function    :   eDateToTimestamp

Description :   Convert s_ExoTime_t to EPOCH time-stamp format.

Parameters  :   p_psExoTime = Pointer to s_ExoTime_t to convert,
                p_pu32Timestamp = Pointer to uint32_t for time-stamp result.

Return      :   e_ExoErrors_t.
============================================================================= */
e_ExoErrors_t eDateToTimestamp (s_ExoTime_t *p_psExoTime, uint32_t *p_pu32Timestamp)
{
  /* Locals variables declaration. */
  e_ExoErrors_t l_eReturn = EXOERRORS_PARAM;
  uint8_t l_u8Month       = UINT8_MIN_VALUE;
  uint16_t l_u16Year      = UINT16_MIN_VALUE;

  /* Input parameters checking. */
  if (   (EXO_PNULL != p_psExoTime)
      && (EXO_PNULL != p_pu32Timestamp))
  {
    /* Initialize time-stamp result. */
    (*p_pu32Timestamp) = UINT32_MIN_VALUE;

    /* Valid time-date ? */
    if (   (EXOTIME_MAX_HOUR_VALUE >= p_psExoTime->u8Hour)
        && (EXOTIME_MAX_MINUTE_VALUE >= p_psExoTime->u8Minute)
        && (EXOTIME_MAX_SECOND_VALUE >= p_psExoTime->u8Second)
        && (   (EXOTIME_MAX_DAYOFMONTH_VALUE >= p_psExoTime->u8DayOfMonth)
            && (EXOTIME_MIN_DAYOFMONTH_VALUE <= p_psExoTime->u8DayOfMonth))
        && (   (EXOTIME_MAX_MONTH_VALUE >= p_psExoTime->u8Month)
            && (EXOTIME_MIN_MONTH_VALUE <= p_psExoTime->u8Month))
        && (   (EXOTIME_MAX_YEAR_VALUE >= p_psExoTime->u16Year)
            && (EXOTIME_MIN_YEAR_VALUE <= p_psExoTime->u16Year)))
    {
      /* Add day per year since 1970. */
      for (l_u16Year = EXOTIME_MIN_YEAR_VALUE; l_u16Year < p_psExoTime->u16Year; l_u16Year++)
      {
        /* Leap year ? */
        if (LEAP_YEAR_YES == (l_u16Year % LEAP_YEAR_PERIOD))
        {
          (*p_pu32Timestamp) += DAY_PER_LEAP_YEAR;
        }
        else
        {
          (*p_pu32Timestamp) += DAY_PER_NOT_LEAP_YEAR;
        }
      }

      /* Add day per month since January. */
      for (l_u8Month = EXOTIME_MONTH_JANUARY; l_u8Month < (p_psExoTime->u8Month - 1u); l_u8Month++)
      {
        /* Current year is leap year ? */
        if (LEAP_YEAR_YES == (l_u16Year % LEAP_YEAR_PERIOD))
        {
          (*p_pu32Timestamp) += g_cau8LUTDaysPerMonth[LEAP_YEAR_INDEX][l_u8Month];
        }
        else
        {
          (*p_pu32Timestamp) += g_cau8LUTDaysPerMonth[NOT_LEAP_YEAR_INDEX][l_u8Month];
        }
      }

      /* Add day since begin of month. */
      (*p_pu32Timestamp) += (p_psExoTime->u8DayOfMonth - 1u);

      /* Convert to hours. */
      (*p_pu32Timestamp) *= HOURS_PER_DAY;

      /* Remove/Add UTC Offset. */
      (*p_pu32Timestamp) -= EXOTIME_UTC_OFFSET;

      /* Add hours since begin of day. */
      (*p_pu32Timestamp) += p_psExoTime->u8Hour;

      /* Convert to minutes. */
      (*p_pu32Timestamp) *= MINUTES_PER_HOUR;

      /* Add minutes since begin of hour. */
      (*p_pu32Timestamp) += p_psExoTime->u8Minute;

      /* Convert to seconds. */
      (*p_pu32Timestamp) *= SECONDS_PER_MINUTE;

      /* Add remaining seconds. */
      (*p_pu32Timestamp) += p_psExoTime->u8Second;

      /* Time is converted into EPOCH format. */
      l_eReturn = EXOERRORS_NO;
    }
    else
    {
      /* Nothing to do. */
    }
  }
  else
  {
    /* Nothing to do. */
  }

  return (l_eReturn);
}


/* =============================================================================
Function    :   eTimestampToDate

Description :   Convert EPOCH time-stamp format to s_ExoTime_t.

Parameters  :   p_u32Timestamp = Time-stamp to convert,
                p_psExoTime = Pointer to s_ExoTime_t result.

Return      :   e_ExoErrors_t.
============================================================================= */
e_ExoErrors_t eTimestampToDate (uint32_t p_u32Timestamp, s_ExoTime_t *p_psExoTime)
{
  /* Locals variables declaration. */
  e_ExoErrors_t l_eReturn = EXOERRORS_PARAM;
  uint8_t l_u8Leap        = UINT8_MIN_VALUE;
  uint16_t l_u16Year      = UINT16_MIN_VALUE;
  uint16_t l_u16Result    = UINT16_MIN_VALUE;

  /* Input parameters checking. */
  if (EXO_PNULL != p_psExoTime)
  {
    /* Add/Remove UTC Offset. */
    p_u32Timestamp += (EXOTIME_UTC_OFFSET*SECONDS_PER_HOUR);		
		
    /* Save remaining seconds. */
    p_psExoTime->u8Second = (p_u32Timestamp % SECONDS_PER_MINUTE);
    p_u32Timestamp /= SECONDS_PER_MINUTE;

    /* Save remaining minutes. */
    p_psExoTime->u8Minute = (p_u32Timestamp % MINUTES_PER_HOUR);
    p_u32Timestamp /= MINUTES_PER_HOUR;

    /* Save remaining hours. */
    p_psExoTime->u8Hour = (p_u32Timestamp % HOURS_PER_DAY);
    p_u32Timestamp /= HOURS_PER_DAY;

    /* Compute year elapsed since 1970. */
    p_psExoTime->u16Year = (p_u32Timestamp / AVERAGE_DAY_PER_YEAR);
    p_u32Timestamp -= ((p_psExoTime->u16Year * DAY_PER_NOT_LEAP_YEAR) + ((p_psExoTime->u16Year + 1) / LEAP_YEAR_PERIOD));

    /* Adjust for actual year. */
    p_psExoTime->u16Year += EXOTIME_MIN_YEAR_VALUE;

    /* Compute month and day. */
    if (LEAP_YEAR_YES == (p_psExoTime->u16Year % LEAP_YEAR_PERIOD))
    {
      l_u8Leap = LEAP_YEAR_INDEX;
    }
    else
    {
      l_u8Leap = NOT_LEAP_YEAR_INDEX;
    }
    p_psExoTime->u8Month = EXOTIME_MONTH_JANUARY;

    while (0 < (int16_t)(p_u32Timestamp - g_cau8LUTDaysPerMonth[l_u8Leap][p_psExoTime->u8Month]))
    {
      p_u32Timestamp -= g_cau8LUTDaysPerMonth[l_u8Leap][p_psExoTime->u8Month];
      p_psExoTime->u8Month++;
    }

    /* Adjust month and day. */
    p_psExoTime->u8Month++;
    p_psExoTime->u8DayOfMonth = p_u32Timestamp + 1u;

    /* Compute day of week (Sakamoto's methods). */

    /* Remove 1 year for january/february. */
    l_u16Year = p_psExoTime->u16Year - (p_psExoTime->u8Month < 3u);
    l_u16Result = l_u16Year;

    /* Add extra (leap) days. */
    l_u16Result += (l_u16Year/LEAP_YEAR_PERIOD);
    l_u16Result -= (l_u16Year/HUNDRED_TO_UNITY);
    l_u16Result += (l_u16Year/FOURHUNDRED_TO_UNITY);

    /* The first days of each month are offset with  respect to January 1. */
    l_u16Result += g_cau8LUTDayOfWeek[p_psExoTime->u8Month - 1u];

    /* Add remaining days. */
    l_u16Result += p_psExoTime->u8DayOfMonth;

    /* Save day of week. */
    p_psExoTime->u8DayOfWeek = (uint8_t) (l_u16Result % DAYS_PER_WEEK);

    /* Time-stamp converted to p_psExoTime. */
    l_eReturn = EXOERRORS_NO;
  }
  else
  {
    /* Nothing to do. */
  }

  return (l_eReturn);
}


/* =============================================================================
Function    :   pachMkTimeToAscii

Description :   Make ASCII string from s_ExoTime_t.

Parameters  :   p_psExoTime = Constant pointer to s_ExoTime_t.

Return      :   Pointer to ASCII string.
============================================================================= */
char* pachMkTimeToAscii (const s_ExoTime_t *p_psExoTime)
{
  snprintf (g_achTimeAsciiFormat, 32, "%.3s %.3s %d %.2d:%.2d:%.2d UTC %d"
                                    , g_cachLUTDayName[p_psExoTime->u8DayOfWeek]
                                    , g_cachLUTMonName[p_psExoTime->u8Month - 1u]
                                    , p_psExoTime->u8DayOfMonth
                                    , p_psExoTime->u8Hour
                                    , p_psExoTime->u8Minute
                                    , p_psExoTime->u8Second
                                    , p_psExoTime->u16Year);

  return (g_achTimeAsciiFormat);
}

/* =============================================================================
                              Private functions
============================================================================= */
