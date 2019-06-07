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
 * Date:          25 07 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of Accelerometer ADXL362.
 *
 */

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>

#include "ADXL362.h"

/************************************************************************
 * Defines
 ************************************************************************/
/* ADXL362 communication commands */
#define ADXL362_WRITE_REG                    0x0A
#define ADXL362_READ_REG                     0x0B
#define ADXL362_WRITE_FIFO                   0x0D

/* Registers */
#define ADXL362_REG_DEVID_AD                 0x00
#define ADXL362_REG_DEVID_MST                0x01
#define ADXL362_REG_PARTID                   0x02
#define ADXL362_REG_REVID                    0x03
#define ADXL362_REG_XDATA                    0x08
#define ADXL362_REG_YDATA                    0x09
#define ADXL362_REG_ZDATA                    0x0A
#define ADXL362_REG_STATUS                   0x0B
#define ADXL362_REG_FIFO_L                   0x0C
#define ADXL362_REG_FIFO_H                   0x0D
#define ADXL362_REG_XDATA_L                  0x0E
#define ADXL362_REG_XDATA_H                  0x0F
#define ADXL362_REG_YDATA_L                  0x10
#define ADXL362_REG_YDATA_H                  0x11
#define ADXL362_REG_ZDATA_L                  0x12
#define ADXL362_REG_ZDATA_H                  0x13
#define ADXL362_REG_TEMP_L                   0x14
#define ADXL362_REG_TEMP_H                   0x15
#define ADXL362_REG_SOFT_RESET               0x1F
#define ADXL362_REG_THRESH_ACT_L             0x20
#define ADXL362_REG_THRESH_ACT_H             0x21
#define ADXL362_REG_TIME_ACT                 0x22
#define ADXL362_REG_THRESH_INACT_L           0x23
#define ADXL362_REG_THRESH_INACT_H           0x24
#define ADXL362_REG_TIME_INACT_L             0x25
#define ADXL362_REG_TIME_INACT_H             0x26
#define ADXL362_REG_ACT_INACT_CTL            0x27
#define ADXL362_REG_FIFO_CTL                 0x28
#define ADXL362_REG_FIFO_SAMPLES             0x29
#define ADXL362_REG_INTMAP1                  0x2A
#define ADXL362_REG_INTMAP2                  0x2B
#define ADXL362_REG_FILTER_CTL               0x2C
#define ADXL362_REG_POWER_CTL                0x2D
#define ADXL362_REG_SELF_TEST                0x2E

#define ADXL362_REG_NB                       ((uint8_t)ADXL362_REG_SELF_TEST + 1u)

#define ADXL362_CTL_MASK                     (0x3)
#define ADXL362_CTL_ODR_MASK                 (0x7)

/* ADXL362_REG_STATUS definitions */
#define ADXL362_STATUS_ERR_USER_REGS         (1 << 7u)
#define ADXL362_STATUS_AWAKE                 (1 << 6u)
#define ADXL362_STATUS_INACT                 (1 << 5u)
#define ADXL362_STATUS_ACT                   (1 << 4u)
#define ADXL362_STATUS_FIFO_OVERRUN          (1 << 3u)
#define ADXL362_STATUS_FIFO_WATERMARK        (1 << 2u)
#define ADXL362_STATUS_FIFO_RDY              (1 << 1u)
#define ADXL362_STATUS_DATA_RDY              (1 << 0u)

/* ADXL362_REG_ACT_INACT_CTL definitions */
#define ADXL362_ACT_INACT_CTL_LINKLOOP(x)    (((x) & ADXL362_CTL_MASK) << 4u)
#define ADXL362_ACT_INACT_CTL_INACT_REF      (1 << 3u)
#define ADXL362_ACT_INACT_CTL_INACT_EN       (1 << 2u)
#define ADXL362_ACT_INACT_CTL_ACT_REF        (1 << 1u)
#define ADXL362_ACT_INACT_CTL_ACT_EN         (1 << 0u)

/* ADXL362_REG_FIFO_CTL */
#define ADXL362_FIFO_CTL_AH                  (1 << 3u)
#define ADXL362_FIFO_CTL_FIFO_TEMP           (1 << 2u)
#define ADXL362_FIFO_CTL_FIFO_MODE(x)        (((x) & ADXL362_CTL_MASK) << 0u)

/* ADXL362_FIFO_CTL_FIFO_MODE(x) options */
#define ADXL362_FIFO_DISABLE                 0u
#define ADXL362_FIFO_OLDEST_SAVED            1u
#define ADXL362_FIFO_STREAM                  2u
#define ADXL362_FIFO_TRIGGERED               3u

/* ADXL362_REG_INTMAP1 */
#define ADXL362_INTMAP1_INT_LOW              (1 << 7u)
#define ADXL362_INTMAP1_AWAKE                (1 << 6u)
#define ADXL362_INTMAP1_INACT                (1 << 5u)
#define ADXL362_INTMAP1_ACT                  (1 << 4u)
#define ADXL362_INTMAP1_FIFO_OVERRUN         (1 << 3u)
#define ADXL362_INTMAP1_FIFO_WATERMARK       (1 << 2u)
#define ADXL362_INTMAP1_FIFO_READY           (1 << 1u)
#define ADXL362_INTMAP1_DATA_READY           (1 << 0u)

/* ADXL362_REG_INTMAP2 definitions */
#define ADXL362_INTMAP2_INT_LOW              (1 << 7u)
#define ADXL362_INTMAP2_AWAKE                (1 << 6u)
#define ADXL362_INTMAP2_INACT                (1 << 5u)
#define ADXL362_INTMAP2_ACT                  (1 << 4u)
#define ADXL362_INTMAP2_FIFO_OVERRUN         (1 << 3u)
#define ADXL362_INTMAP2_FIFO_WATERMARK       (1 << 2u)
#define ADXL362_INTMAP2_FIFO_READY           (1 << 1u)
#define ADXL362_INTMAP2_DATA_READY           (1 << 0u)

/* ADXL362_REG_FILTER_CTL definitions */
#define ADXL362_FILTER_CTL_RANGE(x)          (((x) & ADXL362_CTL_MASK) << 6u)
#define ADXL362_FILTER_CTL_RES               (1 << 5u)
#define ADXL362_FILTER_CTL_HALF_BW           (1 << 4u)
#define ADXL362_FILTER_CTL_EXT_SAMPLE        (1 << 3u)
#define ADXL362_FILTER_CTL_ODR(x)            (((x) & ADXL362_CTL_ODR_MASK) << 0u)

/* ADXL362_REG_POWER_CTL definitions */
#define ADXL362_POWER_CTL_RES                (1 << 7u)
#define ADXL362_POWER_CTL_EXT_CLK            (1 << 6u)
#define ADXL362_POWER_CTL_NOISE(x)           (((x) & ADXL362_CTL_MASK) << 4u)
#define ADXL362_POWER_CTL_WAKEUP             (1 << 3u)
#define ADXL362_POWER_CTL_AUTOSLEEP          (1 << 2u)
#define ADXL362_POWER_CTL_MEASURE(x)         (((x) & ADXL362_CTL_MASK) << 0u)

/* ADXL362_POWER_CTL_MEASURE(x) options */
#define ADXL362_MEASURE_STANDBY              0u
#define ADXL362_MEASURE_ON                   2u

/* ADXL362_REG_SELF_TEST */
#define ADXL362_SELF_TEST_ST                 (1 << 0)

/* ADXL362 device information */
#define ADXL362_DEVICE_AD                    0xAD
#define ADXL362_DEVICE_MST                   0x1D
#define ADXL362_PART_ID                      0xF2

/* ADXL362 Reset settings */
#define ADXL362_RESET_KEY                    0x52


/* Function specific constant */
#define BURST_WRITE_REG_NB                   ((uint8_t)16u)
#define READ_CMD_BUFFER_SIZE                 ((uint8_t)2u)

#define CMD_ADDR_DATA_SIZE                   ((uint8_t)3u)
#define DELAY_SOFTRESET                      ((uint8_t)500u)

#define MASK_THRESHOLD_DATA                  ((uint16_t)0x07FF)

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != ADXL362_ERROR_NONE))   \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);


/************************************************************************
 * Private type declarations
 ************************************************************************/
typedef enum _ACCEL_ACT_INACT_ {
   ACCEL_INACT = 0u,
   ACCEL_ACT = 1u
}e_Accel_ActIn_t;

typedef struct _ACCEL_DATA_ {
   int16_t s16AccelX;
   int16_t s16AccelY;
   int16_t s16AccelZ;
}s_AccelXYZ_t;

/************************************************************************
 * Private function declarations
 ************************************************************************/
static e_ADXL362_ErrorCode_t eRegisterSet( uint8_t p_u8Address, uint8_t * p_pu8Data, uint8_t p_u8DataSize);
static e_ADXL362_ErrorCode_t eRegisterGet( uint8_t p_u8Address, uint8_t * p_pu8Data, uint8_t p_u8DataSize);
static uint16_t u16ComputeThreshold(uint16_t p_u16Threshold);
static uint16_t u16ComputeTime(e_Accel_ActIn_t p_eActOrInact, uint32_t p_u32Time);

/************************************************************************
 * Variable declarations
 ************************************************************************/
static s_ADXL362_Context_t g_sADXL362;
static uint8_t g_u8IsContextSet = 0u;
static uint8_t g_u8IsInitialized = 0u;
static uint8_t g_u8CommFailure = 0u;
static s_AccelXYZ_t g_sAccelXYZ;

/************************************************************************
 * Public functions
 ************************************************************************/
/**@brief Function to set context of ADXL362.
 * @param[in]  p_sContext : All context of ADXL362.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_ContextSet(s_ADXL362_Context_t p_sContext)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_PARAM;
   if(p_sContext.fp_u32SPITransfer != NULL)
   {
      g_sADXL362 = p_sContext;
      g_u8IsContextSet = 1u;
      l_eErrCode = ADXL362_ERROR_NONE;
   }
   else
   {  /* Could not communicate with sensor */
      g_u8IsContextSet = 0u;
   }

   return l_eErrCode;
}

/**@brief Function to initialize ADXL362.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_Init(void)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_NONE;
   uint8_t l_au8ReadBuffer[2u] = { 0u };

   if(g_u8IsInitialized == 0u)
   {
      if(g_u8IsContextSet == 1u)
      {
         /* Get accelerometer ID */
         l_eErrCode = eRegisterGet(ADXL362_REG_PARTID, l_au8ReadBuffer, 1u);
         EXIT_ERROR_CHECK(l_eErrCode);

         if(l_au8ReadBuffer[0u] == ADXL362_PART_ID)
         {
            /* Set Range */
            l_eErrCode = eADXL362_RangeSet(g_sADXL362.eRange);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set Output Data Rate */
            l_eErrCode = eADXL362_OutputDataRateSet(g_sADXL362.eOutputDataRate);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set Activity Detection */
            l_eErrCode = eADXL362_ActivityDetectionSet(  g_sADXL362.u8RefOrAbsActivity,
                                                         g_sADXL362.u16ThresholdActivity,
                                                         g_sADXL362.u16TimeActivity);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set Inactivity Detection */
            l_eErrCode = eADXL362_InactivityDetectionSet(  g_sADXL362.u8RefOrAbsInactivity,
                                                         g_sADXL362.u16ThresholdInactivity,
                                                         g_sADXL362.u32TimeInactivity);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set Interrupt 1 Map */
            l_eErrCode = eADXL362_InterruptMapSet(1u,g_sADXL362.eInt1Map);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set Interrupt 2 Map */
            l_eErrCode |= eADXL362_InterruptMapSet(2u,g_sADXL362.eInt2Map);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set LinkLoop Mode */
            l_eErrCode = eADXL362_LinkLoopModeSet(g_sADXL362.eLinkLoopMode);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set WakeUp Mode */
            l_eErrCode = eADXL362_WakeUpModeSet(g_sADXL362.eWakeUpMode);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Set Measure Mode */
            l_eErrCode = eADXL362_MeasureModeSet(g_sADXL362.eMeasureMode);
            EXIT_ERROR_CHECK(l_eErrCode);

            /* Init Finished ! */
            g_u8IsInitialized = 1u;

         }/* not good part id */
         else
         {
            l_eErrCode = ADXL362_ERROR_IC;
         }
      }
      else
      {  /* Context not set */
         l_eErrCode = ADXL362_ERROR_CONTEXT;
      }
   }
   else
   {  /* Already initialized */
      l_eErrCode = ADXL362_ERROR_NONE;
   }

   return l_eErrCode;
}

/**@brief Function to Reset ADXL362.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_SoftReset(void)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8Buffer = ADXL362_RESET_KEY;

   l_eErrCode = eRegisterSet(ADXL362_REG_SOFT_RESET, &l_u8Buffer, 1u);

   return l_eErrCode;
}
/**@brief Function to Get Part ID.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_PartIDGet(uint8_t * p_pu8PartID)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_PARAM;

   if(p_pu8PartID != NULL)
   {
      l_eErrCode = eRegisterGet(ADXL362_REG_PARTID, p_pu8PartID, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

      if((*p_pu8PartID) != ADXL362_PART_ID)
      {
         l_eErrCode = ADXL362_ERROR_IC;
      }
   }

   return l_eErrCode;
}
/**@brief Function to Get Status register of ADXL362.
 * @param[out] p_pu8Status
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_StatusGet(uint8_t * p_pu8Status)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_PARAM;
   uint8_t l_u8Status = 0u;

   l_eErrCode = eRegisterGet(ADXL362_REG_STATUS, &l_u8Status, 1u);

   if(p_pu8Status != NULL)
   {
      (*p_pu8Status) = l_u8Status;
   }

   return l_eErrCode;
}

/**@brief Function to Get Power glitch detection of ADXL362.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_IsPowerGlitchDetected(void)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_NONE;
   uint8_t l_u8StatusReg = 0xFF;

   l_eErrCode = eRegisterGet(ADXL362_REG_STATUS, &l_u8StatusReg, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   if((l_u8StatusReg & 0x80) == 0x80)
   {
      l_eErrCode = ADXL362_ERROR_POWER_GLITCH;
   }

   return l_eErrCode;
}

/**@brief Function to Set Range of ADXL362.
 * @param[in] p_eRange - Range option.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_RangeSet(e_ADXL362_Range_t p_eRange)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_FILTER_CTL_RANGE(ADXL362_CTL_MASK));
   l_u8NewReg |= ADXL362_FILTER_CTL_RANGE(g_sADXL362.eRange);
   l_au8Buffer[0u] = l_u8NewReg;
   l_eErrCode = eRegisterSet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

  /* Update ADXL362 Context */
  g_sADXL362.eRange = p_eRange;

   return l_eErrCode;
}

/**@brief Function to Set Bandwith Half or quarter of ADXL362.
 * @param[in] p_u8HalfBandwith - HalfBW option.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_HalfBandwithSet(uint8_t p_u8HalfBandwith)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_FILTER_CTL_HALF_BW);
   l_u8NewReg |= (p_u8HalfBandwith * ADXL362_FILTER_CTL_HALF_BW);
   l_au8Buffer[0u] = l_u8NewReg;

   l_eErrCode = eRegisterSet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   return l_eErrCode;
}

/**@brief Function to Set Output Data Rate of ADXL362.
 * @param[in] p_eOutputDataRate - ODR option.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_OutputDataRateSet(e_ADXL362_ODR_t p_eOutputDataRate)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_FILTER_CTL_ODR(ADXL362_CTL_ODR_MASK));
   l_u8NewReg |= ADXL362_FILTER_CTL_ODR(p_eOutputDataRate);
   l_au8Buffer[0u] = l_u8NewReg;
   l_eErrCode = eRegisterSet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);

   l_eErrCode = eRegisterGet(ADXL362_REG_FILTER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Update ADXL362 Context */
   g_sADXL362.eOutputDataRate = p_eOutputDataRate;

   return l_eErrCode;
}

/**@brief Function to Set Link Loop Mode of ADXL362.
 * @param[in] p_eLinkLoopMode - Link Loop Mode option.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_LinkLoopModeSet(e_ADXL362_LinkLoopMode_t p_eLinkLoopMode)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_ACT_INACT_CTL_LINKLOOP(ADXL362_MODE_LOOP));
   l_u8NewReg |= ADXL362_ACT_INACT_CTL_LINKLOOP(p_eLinkLoopMode);
   l_au8Buffer[0u] = l_u8NewReg;
   l_eErrCode = eRegisterSet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Update ADXL362 Context */
   g_sADXL362.eLinkLoopMode = p_eLinkLoopMode;

   return l_eErrCode;
}

/**@brief Function to Set WakeUp Mode of ADXL362.
 * @param[in] p_u8WakeUpMode - WakeUp Mode option.
 * NOTE : Measurement Mode must be set to Measure On in order to use WakeUp
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_WakeUpModeSet(e_ADXL362_WakeUpMode_t p_eWakeUpMode)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_POWER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_POWER_CTL_WAKEUP);
   l_u8NewReg |= (p_eWakeUpMode * ADXL362_POWER_CTL_WAKEUP);
   l_au8Buffer[0u] = l_u8NewReg;

   l_eErrCode = eRegisterSet(ADXL362_REG_POWER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Update ADXL362 Context */
   g_sADXL362.eWakeUpMode = p_eWakeUpMode;

   return l_eErrCode;
}

/**@brief Function to Set Measure Mode of ADXL362.
 * @param[in] p_eMeasureMode - Measure Mode option(Standby or Measure).
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_MeasureModeSet(e_ADXL362_MeasMode_t p_eMeasureMode)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_POWER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_POWER_CTL_MEASURE(ADXL362_CTL_MASK));
   l_u8NewReg |= ADXL362_POWER_CTL_MEASURE(p_eMeasureMode);
   l_au8Buffer[0u] = l_u8NewReg;

   l_eErrCode = eRegisterSet(ADXL362_REG_POWER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Update ADXL362 Context */
   g_sADXL362.eMeasureMode = p_eMeasureMode;

   return l_eErrCode;
}

/**@brief Function to Set Noise Mode of ADXL362.
 * @param[in] p_eNoiseMode - Noise Mode option(Normal, Low or UltraLow).
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_NoiseModeSet(e_ADXL362_NoiseCtrl_t p_eNoiseMode)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_POWER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_POWER_CTL_NOISE(ADXL362_CTL_MASK));
   l_u8NewReg |= ADXL362_POWER_CTL_NOISE(p_eNoiseMode);
   l_au8Buffer[0u] = l_u8NewReg;

   l_eErrCode = eRegisterSet(ADXL362_REG_POWER_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   /* Update ADXL362 Context */
   g_sADXL362.eNoiseCtrl = p_eNoiseMode;

   return l_eErrCode;
}

/**@brief Configures activity detection.
 * @param[in] p_u8RefOrAbs  - Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param[in] p_u16Threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param[in] p_u16Time      - 16-bit value activity time in ms
 *                  (from 2.5ms @ odr = 400Hz to 20s @ odr = 12.5Hz)
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_ActivityDetectionSet(uint8_t p_u8RefOrAbs, uint16_t p_u16Threshold, uint16_t p_u16Time)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[2u] = { 0u };
   uint16_t l_u16Temp = 0u;

   /* Check Range of Threshold */
   l_u16Temp = u16ComputeThreshold(p_u16Threshold);
   if(l_u16Temp > MASK_THRESHOLD_DATA)
   {
      l_eErrCode = ADXL362_ERROR_PARAM;
   }
   else
   {  /* Compute threshold */
      l_au8Buffer[1u] = (uint8_t)((l_u16Temp >> 8u) & 0x07);
      l_au8Buffer[0u] = (uint8_t)(l_u16Temp & 0xFF);

      l_eErrCode = eRegisterSet(ADXL362_REG_THRESH_ACT_L, l_au8Buffer, 2u);
      EXIT_ERROR_CHECK(l_eErrCode);

      /* Compute time */
      l_u16Temp = u16ComputeTime(ACCEL_ACT, p_u16Time);
      l_au8Buffer[0u] = (uint8_t)l_u16Temp;
      l_eErrCode = eRegisterSet(ADXL362_REG_TIME_ACT, l_au8Buffer, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

      /* Enable activity interrupt and select a referenced or absolute configuration. */
      l_eErrCode = eRegisterGet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

      l_u8OldReg = l_au8Buffer[0u];
      l_u8NewReg = l_u8OldReg & (~ADXL362_ACT_INACT_CTL_ACT_REF);
      l_u8NewReg = l_u8NewReg | ADXL362_ACT_INACT_CTL_ACT_EN | (p_u8RefOrAbs * ADXL362_ACT_INACT_CTL_ACT_REF);
      l_au8Buffer[0u] = l_u8NewReg;

      l_eErrCode = eRegisterSet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

      /* Update ADXL362 Context */
      g_sADXL362.u8RefOrAbsActivity = p_u8RefOrAbs;
      g_sADXL362.u16ThresholdActivity = p_u16Threshold;
      g_sADXL362.u16TimeActivity = p_u16Time;
   }

   return l_eErrCode;
}

/**@brief Disable activity detection.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_ActivityDetectionDisable(void)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_ACT_INACT_CTL_ACT_EN);
   l_au8Buffer[0u] = l_u8NewReg;
   l_eErrCode = eRegisterSet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);

   return l_eErrCode;
}

/**@brief Configures inactivity detection.
 * @param[in] p_u8RefOrAbs  - Referenced/Absolute Activity Select.
 *                    Example: 0 - absolute mode.
 *                             1 - referenced mode.
 * @param[in] p_u16Threshold - 11-bit unsigned value that the adxl362 samples are
 *                    compared to.
 * @param[in] p_u16Time      - 16-bit value inactivity time in ms
 *                  (from 2.5ms @ odr = 400Hz to 20s @ odr = 12.5Hz)
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_InactivityDetectionSet(uint8_t p_u8RefOrAbs, uint16_t p_u16Threshold, uint32_t p_u32Time)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[2u] = { 0u, 0u };
   uint16_t l_u16Temp = 0u;

   /* Check Range of computed threshold */
   l_u16Temp = u16ComputeThreshold(p_u16Threshold);
   if(l_u16Temp > MASK_THRESHOLD_DATA)
   {
      l_eErrCode = ADXL362_ERROR_PARAM;
   }
   else
   {  /* Compute threshold */
      l_au8Buffer[1u] = (uint8_t)((l_u16Temp >> 8u) & 0x07);
      l_au8Buffer[0u] = (uint8_t)(l_u16Temp & 0xFF);
      l_eErrCode = eRegisterSet(ADXL362_REG_THRESH_INACT_L, l_au8Buffer, 2u);
      EXIT_ERROR_CHECK(l_eErrCode);

      /* Compute time */
      l_u16Temp = u16ComputeTime(ACCEL_INACT, p_u32Time);
      l_au8Buffer[1u] = (uint8_t)(l_u16Temp >> 8u);
      l_au8Buffer[0u] = (uint8_t)(l_u16Temp & 0xFF);
      l_eErrCode = eRegisterSet(ADXL362_REG_TIME_INACT_L, l_au8Buffer, 2u);
      EXIT_ERROR_CHECK(l_eErrCode);

      /* Enable activity interrupt and select a referenced or absolute configuration. */
      l_eErrCode = eRegisterGet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

      l_u8OldReg = l_au8Buffer[0u];
      l_u8NewReg = l_u8OldReg & (~ADXL362_ACT_INACT_CTL_INACT_REF);
      l_u8NewReg = l_u8NewReg | ADXL362_ACT_INACT_CTL_INACT_EN | (p_u8RefOrAbs * ADXL362_ACT_INACT_CTL_INACT_REF);
      l_au8Buffer[0u] = l_u8NewReg;
      l_eErrCode = eRegisterSet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
      EXIT_ERROR_CHECK(l_eErrCode);

      /* Update ADXL362 Context */
      g_sADXL362.u8RefOrAbsInactivity = p_u8RefOrAbs;
      g_sADXL362.u16ThresholdInactivity = p_u16Threshold;
      g_sADXL362.u32TimeInactivity = p_u32Time;
   }/* else Error already set, do nothing more */

   return l_eErrCode;
}

/**@brief Disable inactivity detection.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_InactivityDetectionDisable(void)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_u8OldReg = 0u;
   uint8_t l_u8NewReg = 0u;
   uint8_t l_au8Buffer[1u] = { 0u };

   l_eErrCode = eRegisterGet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);
   EXIT_ERROR_CHECK(l_eErrCode);

   l_u8OldReg = l_au8Buffer[0u];
   l_u8NewReg = l_u8OldReg & (~ADXL362_ACT_INACT_CTL_INACT_EN);
   l_au8Buffer[0u] = l_u8NewReg;
   l_eErrCode = eRegisterSet(ADXL362_REG_ACT_INACT_CTL, l_au8Buffer, 1u);

   return l_eErrCode;
}

/**@brief Function to Set Interrupt Map of ADXL362.
 * @param[in] p_u8InterruptNumber - Interrupt Map number (1 or 2).
 * @param[in] p_u8Status - Active or not Interrupt Map 1.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_InterruptMapSet(uint8_t p_u8InterruptNumber, e_ADXL362_IntMapX_t p_eIntMapStatus)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_NONE;
   uint8_t l_au8Buffer[1u] = { 0u };
   uint8_t l_u8RegisterAddress = 0u;

   l_au8Buffer[0u] = p_eIntMapStatus;
   if(p_u8InterruptNumber == 1u)
   {
      l_u8RegisterAddress = ADXL362_REG_INTMAP1;
   }
   else if(p_u8InterruptNumber == 2u)
   {
      l_u8RegisterAddress = ADXL362_REG_INTMAP2;
   }
   else
   {
      l_eErrCode = ADXL362_ERROR_PARAM;
   }

   if(l_eErrCode == ADXL362_ERROR_NONE)
   {
      l_eErrCode = eRegisterSet(l_u8RegisterAddress, l_au8Buffer, 1u);
      if(l_eErrCode == ADXL362_ERROR_NONE)
      {  /* Update ADXL362 Context */
         if(p_u8InterruptNumber == 1u)
         {
            g_sADXL362.eInt1Map = p_eIntMapStatus;
         }
         else  // It is Interrupt Map 2 !
         {
            g_sADXL362.eInt2Map = p_eIntMapStatus;
         }
      }/* else Error already set, do nothing more */
   }/* else Error already set, do nothing more */

   return l_eErrCode;
}

/**@brief Function to Read acceleration x, y and z of ADXL362.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_AccelerationRead(void)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_au8Buffer[2u] = { 0u };

   if(g_u8IsInitialized == 1u)
   {  /* Get X Data */
      l_eErrCode = eRegisterGet(ADXL362_REG_XDATA_L, l_au8Buffer, 2u);
      EXIT_ERROR_CHECK(l_eErrCode);

      g_sAccelXYZ.s16AccelX = (int16_t)((((uint16_t)l_au8Buffer[1u]) << 8u) + ((uint16_t)l_au8Buffer[0u]));

      /* Get Y Data */
      l_eErrCode = eRegisterGet(ADXL362_REG_YDATA_L, l_au8Buffer, 2u);
      EXIT_ERROR_CHECK(l_eErrCode);

      g_sAccelXYZ.s16AccelY = (int16_t)((((uint16_t)l_au8Buffer[1u]) << 8u) + ((uint16_t)l_au8Buffer[0u]));

      /* Get Z Data */
      l_eErrCode = eRegisterGet(ADXL362_REG_ZDATA_L, l_au8Buffer, 2u);
      EXIT_ERROR_CHECK(l_eErrCode);

      g_sAccelXYZ.s16AccelZ = (int16_t)((((uint16_t)l_au8Buffer[1u]) << 8u) + ((uint16_t)l_au8Buffer[0u]));
   }
   else
   {  /* ADXL Not Initialized */
      l_eErrCode = ADXL362_ERROR_INIT;
   }

   return l_eErrCode;
}

/**@brief Function to get last acceleration data read of ADXL362.
 * @param[in] p_ps16AccelX : Pointer to the acceleration data on X.
 * @param[in] p_ps16AccelY : Pointer to the acceleration data on Y.
 * @param[in] p_ps16AccelZ : Pointer to the acceleration data on Z.
 * @return None
 */
void vADXL362_AccelerationGet(int16_t * p_ps16AccelX, int16_t * p_ps16AccelY, int16_t * p_ps16AccelZ)
{
   (*p_ps16AccelX) = g_sAccelXYZ.s16AccelX;
   (*p_ps16AccelY) = g_sAccelXYZ.s16AccelY;
   (*p_ps16AccelZ) = g_sAccelXYZ.s16AccelZ;
}

/**@brief Function to enable self test mode of ADXL362.
 * @param[in] p_u8Enable
 * @return Error Code
 */
e_ADXL362_ErrorCode_t eADXL362_SelfTest(uint8_t p_u8Enable)
{
   e_ADXL362_ErrorCode_t l_eErrCode = ADXL362_ERROR_NONE;
   uint8_t l_u8Data = (p_u8Enable & 0x01);

   l_eErrCode = eRegisterSet(ADXL362_REG_SELF_TEST, &l_u8Data, 1u);

   return l_eErrCode;
}

/**@brief Function to get the availability status of the ADXL362.
 * @return Status 1 for available else 0.
 */
uint8_t u8ADXL362_IsAvailable(void)
{
   return ((g_u8IsInitialized == 1u) && (g_u8CommFailure == 0u))?1u:0u;
}

/**@brief Debug function to write byte on register.
 * @param[in]  p_u8Address : Register where to write.
 * @param[in]  p_u8Data : Data to write.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_RegisterSet(uint8_t p_u8Address, uint8_t p_u8Data)
{
   e_ADXL362_ErrorCode_t l_eErrCode;

   l_eErrCode = eRegisterSet(p_u8Address, &p_u8Data, 1u);

   return l_eErrCode;
}
/**@brief Debug function to read byte on register.
 * @param[in]  p_u8Address : Register where to read.
 * @param[in]  p_u8Data : Read Data.
 * @return Error Code.
 */
e_ADXL362_ErrorCode_t eADXL362_RegisterGet(uint8_t p_u8Address, uint8_t * p_pu8Data)
{
   e_ADXL362_ErrorCode_t l_eErrCode;

   l_eErrCode = eRegisterGet(p_u8Address, p_pu8Data, 1u);

   return l_eErrCode;
}
/************************************************************************
 * Private functions
 ************************************************************************/
/**@brief Static function to Write register of ADXL362.
 * @details Controls are done on registers which have Write Protect
 * @param[in]  p_u8Address : Start address of register.
 * @param[in]  p_pu8Data : Data that will be written.
 * @param[in]  p_u8DataSize : Size of Data.
 * @return Error Code.
 */
static e_ADXL362_ErrorCode_t eRegisterSet(uint8_t p_u8Address, uint8_t * p_pu8Data, uint8_t p_u8DataSize)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_au8bufferOut[BURST_WRITE_REG_NB] = { 0u };
   uint8_t l_au8bufferIn[BURST_WRITE_REG_NB]= { 0u };
   uint8_t l_u8Size = 0u;
   uint8_t l_u8Idx = 0u;

   /* Do not write over the last register */
   if( (p_u8Address + p_u8DataSize) > ADXL362_REG_NB)
   {
      l_eErrCode = ADXL362_ERROR_PARAM;
   }
   else
   {
      l_au8bufferOut[0u] = ADXL362_WRITE_REG;
      l_au8bufferOut[1u] = p_u8Address;

      for(l_u8Idx = 0u;l_u8Idx<p_u8DataSize;l_u8Idx++)
      {
         l_au8bufferOut[l_u8Idx + 2u] = p_pu8Data[l_u8Idx];
      }

      l_u8Size = p_u8DataSize + 2u;

      if(g_sADXL362.fp_u32SPITransfer != NULL)
      {
         if((g_sADXL362.fp_u32SPITransfer)(l_au8bufferOut, l_u8Size, l_au8bufferIn, l_u8Size) != 0u)
         {
            l_eErrCode = ADXL362_ERROR_COMM;
            g_u8CommFailure = 1u;
         }
         else
         {
            l_eErrCode = ADXL362_ERROR_NONE;
            g_u8CommFailure = 0u;
         }
      }
      else
      {
         l_eErrCode = ADXL362_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}

/**@brief Static function to Read register of ADXL362.
 * @param[in]  p_u8Address : Start address of register.
 * @param[in]  p_pu8Data : Data that will be read.
 * @param[in]  p_u8DataSize : Size of Data.
 * @return Error Code.
 */
static e_ADXL362_ErrorCode_t eRegisterGet(uint8_t p_u8Address, uint8_t * p_pu8Data, uint8_t p_u8DataSize)
{
   e_ADXL362_ErrorCode_t l_eErrCode;
   uint8_t l_au8bufferOut[CMD_ADDR_DATA_SIZE] = { 0u };
   uint8_t l_au8bufferIn[ADXL362_REG_NB] = { 0u };
   uint8_t l_u8Size = 0u;
   uint8_t l_u8Idx = 0u;

   /* Do not read over the last register */
   if( (p_u8Address + p_u8DataSize) > ADXL362_REG_NB)
   {
      l_eErrCode = ADXL362_ERROR_PARAM;
   }
   else
   {
      l_au8bufferOut[0u] = ADXL362_READ_REG;
      l_au8bufferOut[1u] = p_u8Address;
      l_u8Size = p_u8DataSize + READ_CMD_BUFFER_SIZE; /* 2u since there are CMD Read and Start Address in data */

      if(g_sADXL362.fp_u32SPITransfer != NULL)
      {
         if((g_sADXL362.fp_u32SPITransfer)(l_au8bufferOut, READ_CMD_BUFFER_SIZE, l_au8bufferIn, l_u8Size) != 0u)
         {
            l_eErrCode = ADXL362_ERROR_COMM;
            g_u8CommFailure = 1u;
         }
         else
         {
            for(l_u8Idx = 0u; l_u8Idx < p_u8DataSize; l_u8Idx++)
            {
               p_pu8Data[l_u8Idx] = l_au8bufferIn[l_u8Idx+READ_CMD_BUFFER_SIZE];
            }
            l_eErrCode = ADXL362_ERROR_NONE;
            g_u8CommFailure = 0u;
         }
      }
      else
      {
         l_eErrCode = ADXL362_ERROR_PARAM;
      }
   }

   return l_eErrCode;
}

/**@brief Static function to compute Threshold for activity or
 *        inactivity detection of ADXL362.
 * @param[in]  p_u16Threshold : Threshold in mg.
 *
 * @return The threshold that should be write in register.
 */
static uint16_t u16ComputeThreshold(uint16_t p_u16Threshold)
{
   uint16_t l_u16Thr = 0u;
   uint16_t l_u16Range = 0u;

   switch(g_sADXL362.eRange)
   {
      case ADXL362_RANGE_2G:
         l_u16Range = 2000u;
         break;
      case ADXL362_RANGE_4G:
         l_u16Range = 4000u;
         break;
      case ADXL362_RANGE_8G:
         l_u16Range = 8000u;
         break;
      default:
         l_u16Range = 2000u;
         break;
   }

   /* Compute Threshold according to the range, ( Thr * (2*POW(11)-1) )/ Range */
   l_u16Thr = (uint16_t)(((((uint32_t)(p_u16Threshold))*2047u) + ((((uint32_t)l_u16Range))>>1)) / ((uint32_t)l_u16Range));

   return l_u16Thr;
}

/**@brief Static function to compute Time for activity
 *        or inactivity detection of ADXL362.
 * @param[in]  p_eActOrInact : 0 for Inact - 1 for Act.
 * @param[in]  p_u16Time : Time in ms.
 *
 * @return  The time that should be write in register,
 *          (uint8_t register for activity and uint16_t for inactivity).
 */
static uint16_t u16ComputeTime(e_Accel_ActIn_t p_eActOrInact, uint32_t p_u32Time)
{
   uint32_t l_u32Time = 0u;
   uint32_t l_u32ODR = 0u;

   if(g_sADXL362.eWakeUpMode == ADXL362_WAKEUP_OFF)
   {
      switch(g_sADXL362.eOutputDataRate)
      {  /* Corresponds to (1 / ODR _Frequency) */
         case ADXL362_ODR_12_5_HZ :
               l_u32ODR = 80u;
            break;
         case ADXL362_ODR_25_HZ :
               l_u32ODR = 40u;
            break;
         case ADXL362_ODR_50_HZ :
               l_u32ODR = 20u;
            break;
         case ADXL362_ODR_100_HZ :
               l_u32ODR = 10u;
            break;
         case ADXL362_ODR_200_HZ :
               l_u32ODR = 5u;
            break;
         case ADXL362_ODR_400_HZ :
               l_u32ODR = 3u;// 2.5
            break;
         default:
            l_u32ODR = 10u;
            break;
      }
   }
   else
   {  /* Correspond to 6(.25) Hz*/
      l_u32ODR = 160u;
   }

   l_u32Time = (uint32_t)((((uint32_t)(p_u32Time))+(l_u32ODR>>1u))/l_u32ODR);

   if(p_eActOrInact == ACCEL_ACT) // Activity
   {
      if(l_u32Time > UINT8_MAX)
      {
         l_u32Time = UINT8_MAX;
      }
   }
   else if(p_eActOrInact == ACCEL_INACT)
   {
      if(l_u32Time > UINT16_MAX)
      {
         l_u32Time = UINT16_MAX;
      }
   }
   else
   {
      /* Nothing to do, should not happen */
   }
   return (uint16_t)l_u32Time;
}

/************************************************************************
 * End Of File
 ************************************************************************/


