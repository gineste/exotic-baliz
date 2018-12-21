/* =============================================================================
 ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
(  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
(____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/

  Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.

  Licensees are granted free, non-transferable use of the information. NO
  WARRANTY of ANY KIND is provided. This heading must NOT be removed from
  the file.

  File name:    ExoTime.h
  Date:         02 01 2018
  Author:       Emeric L.
  Description:  EXOTIME - Header file.
============================================================================= */
#ifndef EXOTIME__H
  #define EXOTIME__H

  #include <stdint.h>
  #include "ExoErrors.h"
  /* ===========================================================================
                                   DEBUG Section
  =========================================================================== */

  /* ===========================================================================
                            Public defines and typedefs
  =========================================================================== */
  #define EXOTIME_DELAYMS_1MS           ((uint32_t)1)
  #define EXOTIME_DELAYMS_10MS          ((uint32_t)(10*EXOTIME_DELAYMS_1MS))
  #define EXOTIME_DELAYMS_20MS          ((uint32_t)( 2*EXOTIME_DELAYMS_10MS))
  #define EXOTIME_DELAYMS_100MS         ((uint32_t)(10*EXOTIME_DELAYMS_10MS))
  #define EXOTIME_DELAYMS_300MS         ((uint32_t)(3*EXOTIME_DELAYMS_100MS)) 
  #define EXOTIME_DELAYMS_1S            ((uint32_t)(10*EXOTIME_DELAYMS_100MS))
  #define EXOTIME_DELAYMS_2S            ((uint32_t)( 2*EXOTIME_DELAYMS_1S))
  #define EXOTIME_DELAYMS_3S            ((uint32_t)( 3*EXOTIME_DELAYMS_1S))
  #define EXOTIME_DELAYMS_5S            ((uint32_t)( 5*EXOTIME_DELAYMS_1S))
  #define EXOTIME_DELAYMS_10S           ((uint32_t)(10*EXOTIME_DELAYMS_1S))
  #define EXOTIME_DELAYMS_20S           ((uint32_t)( 2*EXOTIME_DELAYMS_10S))
  #define EXOTIME_DELAYMS_30S           ((uint32_t)( 3*EXOTIME_DELAYMS_10S))
  #define EXOTIME_DELAYMS_60S           ((uint32_t)( 6*EXOTIME_DELAYMS_10S))

  #define EXOTIME_UTC_OFFSET            ((int8_t)2)

  #define EXOTIME_MIN_HOUR_VALUE        ((uint8_t)0)
  #define EXOTIME_MAX_HOUR_VALUE        ((uint8_t)23)

  #define EXOTIME_MIN_MINUTE_VALUE      ((uint8_t)0)
  #define EXOTIME_MAX_MINUTE_VALUE      ((uint8_t)59)

  #define EXOTIME_MIN_SECOND_VALUE      ((uint8_t)0)
  #define EXOTIME_MAX_SECOND_VALUE      ((uint8_t)59)

  #define EXOTIME_MIN_DAYOFWEEK_VALUE   ((uint8_t)0)
  #define EXOTIME_MAX_DAYOFWEEK_VALUE   ((uint8_t)6)

  #define EXOTIME_MIN_DAYOFMONTH_VALUE  ((uint8_t)1)
  #define EXOTIME_MAX_DAYOFMONTH_VALUE  ((uint8_t)31)
  
  #define EXOTIME_MIN_MONTH_VALUE       ((uint8_t)1)
  #define EXOTIME_MAX_MONTH_VALUE       ((uint8_t)12)

  #define EXOTIME_MIN_YEAR_VALUE        ((uint16_t)1970)
  #define EXOTIME_MAX_YEAR_VALUE        ((uint16_t)2038)

  #define EXOTIME_MONTHS_PER_YEAR       ((uint8_t)12)
  #define EXOTIME_MONTHS_STRING_SIZE    ((uint8_t)4)

  typedef struct _EXOTIME_ {
    uint8_t u8Hour;         /* 0 - 23 */
    uint8_t u8Minute;       /* 0 - 59 */
    uint8_t u8Second;       /* 0 - 59 */
    uint8_t u8DayOfMonth;   /* 1 - 31 */
    uint8_t u8DayOfWeek;    /* 0 (sunday) - 6 (saturday) */
    uint8_t u8Month;        /* 1 - 12 */
    uint16_t u16Year;       /* 1970 - 2038 */
  } s_ExoTime_t;

  typedef enum _EXOTIME_MONTH_ {
    EXOTIME_MONTH_JANUARY = 0u,
    EXOTIME_MONTH_FEBRUARY,
    EXOTIME_MONTH_MARCH,
    EXOTIME_MONTH_APRIL,
    EXOTIME_MONTH_MAY,
    EXOTIME_MONTH_JUNE,
    EXOTIME_MONTH_JULY,
    EXOTIME_MONTH_AUGUST,
    EXOTIME_MONTH_SEPTEMBER,
    EXOTIME_MONTH_OCTOBER,
    EXOTIME_MONTH_NOVEMBER,
    EXOTIME_MONTH_DECEMBER
  } e_ExoTime_Month_t;

  typedef enum _EXOTIME_WEEKDAY_ {
    EXOTIME_WEEKDAY_SUNDAY = 0u,
    EXOTIME_WEEKDAY_MONDAY,
    EXOTIME_WEEKDAY_TUESDAY,
    EXOTIME_WEEKDAY_WEDNESDAY,
    EXOTIME_WEEKDAY_THURSDAY,
    EXOTIME_WEEKDAY_FRIDAY,
    EXOTIME_WEEKDAY_SATURDAY
  } e_ExoTime_WeekDay_t;

  /* ===========================================================================
                          Public constants and variables
  =========================================================================== */
  extern const char g_cachLUTMonName[EXOTIME_MONTHS_PER_YEAR][EXOTIME_MONTHS_STRING_SIZE];

  /* ===========================================================================
                          Public function declarations
  =========================================================================== */
  extern void vCompilationInfoToTimestamp (uint32_t * p_pu32Timestamp);
  extern e_ExoErrors_t eDateToTimestamp (s_ExoTime_t *p_psExoTime, uint32_t *p_pu32Timestamp);
  extern e_ExoErrors_t eTimestampToDate (uint32_t p_u32Timestamp, s_ExoTime_t *p_psExoTime);
  extern char* pachMkTimeToAscii (const s_ExoTime_t *p_psExoTime);

#endif /* EXOTIME__H */
