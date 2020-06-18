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
 * Date:          14/06/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   NMEA Frame parser 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "BoardConfig.h"
#include "GlobalDefs.h"

#ifdef SDCARD_LOG
   #include "Libraries/FileSystem.h"
#endif
/* Self include */
#include "NMEA.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define NMEA_MAX_SIZE               (uint8_t)85
#define NMEA_CHECKSUM_SIZE          (uint8_t)2

#define NMEA_SENTENCE_SIZE          (uint8_t)6u

#define NMEA_CHAR_SOF_DOLLAR        (char)'$'
#define NMEA_CHAR_START             (char)'*'
#define NMEA_CHAR_EOF_CR            (char)'\r'
#define NMEA_CHAR_EOF_LF            (char)'\n'
#define NMEA_CHAR_COMMA             (char)','
#define NMEA_STRG_COMMA             ","

#define NMEA_SCHEDULER_QUEUE_SIZE   (uint8_t)15u
#define NMEA_PMTK_QUEUE_SIZE        (uint8_t)1u

#define NMEA_SPLIT_DATA_SIZE        (uint8_t)20u

#define NMEA_PMTK_DECODER_FCT_NB    (uint8_t)3u
#define NMEA_GGA_DECODER_FCT_NB     (uint8_t)14u
#define NMEA_RMC_DECODER_FCT_NB     (uint8_t)12u
#define NMEA_GSA_DECODER_FCT_NB     (uint8_t)17u
#define NMEA_GLL_DECODER_FCT_NB     (uint8_t)6u
#define NMEA_GST_DECODER_FCT_NB     (uint8_t)8u
#define NMEA_ZDA_DECODER_FCT_NB     (uint8_t)6u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
struct {
   uint8_t u8Size;
   uint8_t au8NMEAFrame[NMEA_MAX_SIZE];
}g_sNMEAQueue[NMEA_SCHEDULER_QUEUE_SIZE];

typedef enum _NMEA_SENTENCE_ID_ {
   NMEA_PSRF = 0u,
   NMEA_PMTK,
   NMEA_GPRMC,
   NMEA_GNRMC,
   NMEA_GPGGA,
   NMEA_GNGGA,
   NMEA_GPGSA,
   NMEA_GPGSV,
   NMEA_GPGLL,
   NMEA_GNGLL,
   NMEA_GPGST,
   NMEA_GNZDA,
   NMEA_SENTENCES_NB,
   NMEA_UNKNOWN = 0xFF
}e_NMEASentence_t;

typedef void (*fpvNMEAPMTKDecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded);
typedef void (*fpvNMEAGGADecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
typedef void (*fpvNMEARMCDecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
typedef void (*fpvNMEAGSADecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
typedef void (*fpvNMEAGLLDecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);
typedef void (*fpvNMEAGSTDecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
typedef void (*fpvNMEAZDADecoder_t)(uint8_t * p_pau8Frame, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vNMEAQueueMsg(uint8_t * p_au8Message, uint8_t p_u8MsgSize);
static void vNMEAEnqueueMsg(uint8_t * p_au8Message, uint8_t * p_pu8MsgSize);
static uint8_t u8NMEAIsQueueEmpty(void);

static float fDMStoDD(double p_dDegMinSec, char p_chCardinalPt);
static inline uint8_t u8CharHexToByte(char p_chChar);
static uint8_t u8SentenceIDGet(uint8_t * p_pau8Sentence);
//static void vParserPMTK(uint8_t * p_pau8Frame, uint8_t p_u8Size, void * p_pvDecoded);
static void vParserNMEA(e_NMEASentence_t p_eSentenceID, uint8_t * p_pau8Frame, uint8_t p_u8Size, void * p_pvDecoded);
static void vDecoderNMEA(e_NMEASentence_t p_eSentenceID, uint8_t p_u8FunctionIdx, uint8_t * p_pau8Buffer, uint8_t p_u8Size, void * p_pvDecoded);


static void vPMTKType(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded);
static void vPMTKCmd(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded);
static void vPMTKFlag(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded);


static void vGGAFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGALatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGALatitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGALongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGALongitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGAFixIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGASatellitesNumber(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGAHDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGAAltitudeAboveSea(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGAAltitudeAboveSeaUnit(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGAHeightEllipsoid(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);
static void vGGAHeightEllipsoidUnit(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded);


static void vRMCFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCStatus(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCLatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCLatitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCLongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCLongitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCSpeed(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCTrackAngle(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCMagneticVariation(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCMagneticIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);
static void vRMCModePosition(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded);


static void vGSAAutoManuSel(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAFixType(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN0(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN1(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN2(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN3(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN4(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN5(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN6(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN7(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN8(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN9(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN10(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPRN11(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAPDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAHDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);
static void vGSAVDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded);

static void vGLLLatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);
static void vGLLLatitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);
static void vGLLLongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);
static void vGLLLongitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);
static void vGLLFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);
static void vGLLModePosition(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded);

static void vGSTFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTRMS(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTErrorSemiMajor(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTErrorSemiMinor(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTErrorOrientation(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTErrorLatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTErrorLongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);
static void vGSTErrorHeight(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded);

static void vZDAUTCTime(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);
static void vZDADay(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);
static void vZDAMonth(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);
static void vZDAYear(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);
static void vZDAOffsetGMTHour(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);
static void vZDAOffsetGMTMinute(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded);


/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static const char g_cachNMEAID[NMEA_SENTENCES_NB][NMEA_SENTENCE_SIZE] = {   
/* Must be the same order as e_NMEASentence_t */
   "PSRF",
   "PMTK",
   "GPRMC",
   "GNRMC",
   "GPGGA",
   "GNGGA",
   "GPGSA",
   "GPGSV",
   "GPGLL",
   "GNGLL",
   "GPGST",
   "GNZDA"
};

#ifndef DEBUG
static 
#endif
volatile uint8_t g_au8NMEABuffer[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8NMEAMsgIn = 0u;
volatile uint8_t g_u8NMEAMsgOut = 0u;
//static uint8_t g_u8AckMsgIn = 0u;
//static uint8_t g_u8AckMsgOut = 0u;

volatile uint8_t g_au8NMEALastGGA[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastGGASize = 0u;
volatile uint8_t g_au8NMEALastGSA[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastGSASize = 0u;
volatile uint8_t g_au8NMEALastRMC[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastRMCSize = 0u;
volatile uint8_t g_au8NMEALastGLL[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastGLLSize = 0u;
volatile uint8_t g_au8NMEALastZDA[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastZDASize = 0u;
volatile uint8_t g_au8NMEALastGSV[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastGSVSize = 0u;
volatile uint8_t g_au8NMEALastGST[NMEA_MAX_SIZE] = { 0u };
volatile uint8_t g_u8LastGSTSize = 0u;

static uint8_t g_u8LocationFixed = 0u;

#ifndef DEBUG
static 
#endif
s_NMEA_PMTK_t g_sNMEAPMTK = {
   .u16Type = 0u,
   .u16Cmd = 0u,
   .eAck = PMTK_ACK_UNKNOWN
};

#ifndef DEBUG
static 
#endif
s_NMEA_GGA_t g_sNMEAGGA = {
   .au8FixDateUTC = { 0u },
   .dLatitude = 0.0f,
   .dLongitude = 0.0f,
   .u8LatNS = (uint8_t)'V',
   .u8LonEW = (uint8_t)'V',
   .u8FixIndicator = 0u,
   .u8NbSatTracked = 0u,
   .u16HDOP = 0u, /* 0.1 unit */
   .s16AltSeaLevel = 0, /* 0.1 unit */
   .s16HeightEllipsoid = 0, /* 0.1 unit */
   .u16TimeSinceLastDGPSUpdate = 0u,
   .u16DGPSStationID = 0u
};

#ifndef DEBUG
static 
#endif
s_NMEA_RMC_t g_sNMEARMC = {
   .au8FixDateUTC = { 0u },
   .dLatitude = 0.0f,
   .dLongitude = 0.0f,
   .u8LatNS = (uint8_t)'V',
   .u8LonEW = (uint8_t)'V',
   .u8Status= 0u,
   .u16Speed = 0u,   /* 0.1 unit */
   .u16TrackAngle = 0u, /* 0.1 unit */
   .au8Date = { 0u },
   .u16MagneticVariation = 0u, /* 0.1 unit */
   .u8MagneticSens = (uint8_t)'V',
   .u8ModePosition = (uint8_t)'V'
};

#ifndef DEBUG
static 
#endif
s_NMEA_GSA_t g_sNMEAGSA = {
   .u8AutoManuSelect = (uint8_t)'V',
   .u8FixType = 0u,
   .au8PRNSat = { 0u },
   .u16PDOP = 0u, /* 0.1 unit */
   .u16HDOP = 0u, /* 0.1 unit */
   .u16VDOP = 0u  /* 0.1 unit */
};

#ifndef DEBUG
static 
#endif
s_NMEA_GLL_t g_sNMEAGLL = {
   .dLatitude = 0.0f,
   .dLongitude = 0.0f,
   .u8LatNS = (uint8_t)'V',
   .u8LonEW = (uint8_t)'V',
   .au8FixDateUTC = { 0u },
   .u8ModePosition = (uint8_t)'V',
};

#ifndef DEBUG
static 
#endif
s_NMEA_ZDA_t g_sNMEAZDA = {
   .au8DateUTC = { 0u },
   .u8Day = 0u,
   .u8Month = 0u,
   .u16Year = 0u,
   .s8OffsetGMTHours = 0,
   .u8OffsetGMTMinutes = 0u,
};

#ifndef DEBUG
static 
#endif
s_NMEA_GSV_t g_sNMEAGSV[NMEA_GSV_STRUCT_ARRAY] = {
   {
      .u8SentenceNb = 0u,
      .u8SentenceIdx = 0u,
      .u8SatInView = 0u,
      .asSatInfo = { 
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         },
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         },
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         },
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         }
      }
   },
   {
      .u8SentenceNb = 0u,
      .u8SentenceIdx = 0u,
      .u8SatInView = 0u,
      .asSatInfo = { 
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         },
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         },
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         },
         {
            .u8SatPRNNb = 0u,
            .u8ElevationDeg = 0u,
            .u16AzimuthDeg = 0u,
            .u8SNR = 0u,
         }
      }
   },
};


#ifndef DEBUG
static 
#endif
s_NMEA_GST_t g_sNMEAGST = {
   .au8FixDateUTC = { 0u },
   .u16RMSPseudoRangeResiduals = 0u,      /* 0.1 unit */
   .f32SigmaErrorSemiMajor = 99999.9f,    /* in m */
   .f32SigmaErrorSemiMinor = 99999.9f,    /* in m */
   .f32ErrorOrientation = 99999.9f,       /* in degrees */
   .f32SigmaErrorLatitude = 99999.9f,     /* in m */
   .f32SigmaErrorLongitude = 99999.9f,    /* in m */
   .f32SigmaErrorHeight = 99999.9f,       /* in m */
};

static const fpvNMEAPMTKDecoder_t g_cafpvPMTKDecoder[NMEA_PMTK_DECODER_FCT_NB] = { 
   vPMTKType, vPMTKCmd, vPMTKFlag
};

static const fpvNMEAGGADecoder_t g_cafpvGGADecoder[NMEA_GGA_DECODER_FCT_NB] = { 
	vGGAFixUTCDate, vGGALatitude, vGGALatitudeIndicator, vGGALongitude, vGGALongitudeIndicator, 
   vGGAFixIndicator, vGGASatellitesNumber, vGGAHDOP, vGGAAltitudeAboveSea, vGGAAltitudeAboveSeaUnit, 
   vGGAHeightEllipsoid, vGGAHeightEllipsoidUnit, NULL, NULL
};

static const fpvNMEARMCDecoder_t g_cafpvRMCDecoder[NMEA_RMC_DECODER_FCT_NB] = { 
	vRMCFixUTCDate, vRMCStatus, vRMCLatitude, vRMCLatitudeIndicator, 
   vRMCLongitude, vRMCLongitudeIndicator, vRMCSpeed, vRMCTrackAngle, vRMCDate, 
   vRMCMagneticVariation, vRMCMagneticIndicator, vRMCModePosition
};

static const fpvNMEAGSADecoder_t g_cafpvGSADecoder[NMEA_GSA_DECODER_FCT_NB] = { 
	vGSAAutoManuSel, vGSAFixType, vGSAPRN0, vGSAPRN1, vGSAPRN2, 
   vGSAPRN3, vGSAPRN4, vGSAPRN5, vGSAPRN6, vGSAPRN7, 
   vGSAPRN8, vGSAPRN9, vGSAPRN10, vGSAPRN11, vGSAPDOP,
   vGSAHDOP, vGSAVDOP
};

static const fpvNMEAGLLDecoder_t g_cafpvGLLDecoder[NMEA_GLL_DECODER_FCT_NB] = { 
	vGLLLatitude, vGLLLatitudeIndicator, vGLLLongitude, vGLLLongitudeIndicator, 
   vGLLFixUTCDate, vGLLModePosition
};

static const fpvNMEAGSTDecoder_t g_cafpvGSTDecoder[NMEA_GST_DECODER_FCT_NB] = { 
	vGSTFixUTCDate, vGSTRMS, vGSTErrorSemiMajor, vGSTErrorSemiMinor, vGSTErrorOrientation, 
   vGSTErrorLatitude, vGSTErrorLongitude, vGSTErrorHeight
};

static const fpvNMEAZDADecoder_t g_cafpvZDADecoder[NMEA_ZDA_DECODER_FCT_NB] = { 
   vZDAUTCTime, vZDADay, vZDAMonth, vZDAYear, vZDAOffsetGMTHour, vZDAOffsetGMTMinute
};

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vNMEA_InitProtocol(void)
{
   memset(g_sNMEAQueue, 0u, sizeof(g_sNMEAQueue));   
}

void vNMEA_UpdateFrame(uint8_t p_u8Byte)
{
   static uint8_t l_u8SentenceIdx = 0u;
   static uint8_t l_u8ChecksumStart = 0u;
   static uint8_t l_u8EndIdx = 0u;
   static uint8_t l_au8ChecksumCompute = 0u;
   static uint8_t l_u8ChecksumRead = 0u;
   static uint8_t l_u8ValidFrame = 0u;
   static uint8_t l_au8NMEABuffer[NMEA_MAX_SIZE] = { 0u };
#ifdef HARDWARE_TEST
   static uint8_t l_au8HardwareTestNMEABuffer[NMEA_MAX_SIZE+12] = { '$','R','S','L',',','G','P','S','+','1',',', 0u };
#endif
	/* Cursor overflow or beginning of a frame */
	if(   (l_u8SentenceIdx >= NMEA_MAX_SIZE) 
      || (p_u8Byte == NMEA_CHAR_SOF_DOLLAR) )
	{  /* Restart from the beginning */
		l_u8SentenceIdx = 0u;
      l_u8ChecksumStart = 0u;
      l_u8ChecksumRead = 0u;
      l_u8EndIdx = 0u;
      l_au8ChecksumCompute = 0u;
      l_u8ValidFrame = 0u;
	}
	else
   {  /* Processing data */
      if(l_u8ChecksumStart == 0u)
      {
         /* End of frame */
         if(p_u8Byte != NMEA_CHAR_START)
         {
            if(p_u8Byte != NMEA_CHAR_SOF_DOLLAR)
            {  /* Checksum compute on the fly */
               l_au8ChecksumCompute ^= p_u8Byte;
            }
         }
         else
         {  /* Start Read Checksum */
            l_u8ChecksumStart = 1u;
         }
      }
      else
      {  /* Read Checksum */
         if(l_u8EndIdx < NMEA_CHECKSUM_SIZE )
         {
            l_u8ChecksumRead = (l_u8ChecksumRead << 4u) + u8CharHexToByte((char)p_u8Byte);
            l_u8EndIdx++;
            /* Compare Computed and read Checksum of NMEA frame */
            if(l_au8ChecksumCompute == l_u8ChecksumRead)
            {  /* Valid Frame ! */
               l_u8ValidFrame = 1u;
            }
         }
      }
   }
   /* Assign new byte to buffer */
   l_au8NMEABuffer[l_u8SentenceIdx++] = p_u8Byte;
   
   if(   (l_u8ValidFrame == 1u)
      && (p_u8Byte == NMEA_CHAR_EOF_LF) )
   {
      strcpy((char*)g_au8NMEABuffer, (char*)l_au8NMEABuffer);
            
   #ifdef HARDWARE_TEST
      strcpy((char*)&l_au8HardwareTestNMEABuffer[11u], (char*)l_au8NMEABuffer);
      
      l_au8HardwareTestNMEABuffer[l_u8SentenceIdx+11u] = '\0';
      PRINT_UART("%s",l_au8HardwareTestNMEABuffer);
   #elif (LOG_GPS == 1)
      l_au8NMEABuffer[l_u8SentenceIdx] = '\0';
      PRINT_UART("%s",l_au8NMEABuffer);
   #endif
      /* Add it to queue (Frame and Size). Decode it later */
      vNMEAQueueMsg((uint8_t *)g_au8NMEABuffer, l_u8SentenceIdx);      
   }
}

void vNMEA_FrameProcessing(void)
{
   uint8_t l_u8SentenceID = 0xFE;
   uint8_t l_u8Size = 0u;
   uint8_t l_au8NMEABuffer[NMEA_MAX_SIZE] = { 0u };
   
   #ifdef SDCARD_LOG
      static uint8_t l_u8Idx = 0u;
   #endif
   /* Check if queue is empty */
   while(u8NMEAIsQueueEmpty() == 0u)
   {
      /* Enqueue Frame and process it */
      vNMEAEnqueueMsg(l_au8NMEABuffer, &l_u8Size);
      l_u8SentenceID = u8SentenceIDGet(l_au8NMEABuffer);
      
      switch((e_NMEASentence_t)l_u8SentenceID)
      {
         case NMEA_PSRF:
            break;
         case NMEA_PMTK:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEAPMTK);
            break;
         case NMEA_GPRMC:
         case NMEA_GNRMC:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEARMC);
            g_u8LastRMCSize = l_u8Size;
            memcpy((void*)g_au8NMEALastRMC, l_au8NMEABuffer, l_u8Size);
            break;
         case NMEA_GPGGA:
         case NMEA_GNGGA:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEAGGA);
            g_u8LastGGASize = l_u8Size;
            memcpy((void*)g_au8NMEALastGGA, l_au8NMEABuffer, l_u8Size);
            break;
         case NMEA_GPGSA:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEAGSA);
            g_u8LastGSASize = l_u8Size;
            memcpy((void*)g_au8NMEALastGSA, l_au8NMEABuffer, l_u8Size);
            break;
         case NMEA_GPGLL:
         case NMEA_GNGLL:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEAGLL);
            g_u8LastGLLSize = l_u8Size;
            memcpy((void*)g_au8NMEALastGLL, l_au8NMEABuffer, l_u8Size);
            break;
         case NMEA_GPGSV:
            break;
         case NMEA_GPGST:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEAGST);
            g_u8LastGSTSize = l_u8Size;
            memcpy((void*)g_au8NMEALastGST, l_au8NMEABuffer, l_u8Size);
            break;
         case NMEA_GNZDA:
            vParserNMEA((e_NMEASentence_t)l_u8SentenceID, l_au8NMEABuffer, l_u8Size, (void*)&g_sNMEAZDA);
            g_u8LastZDASize = l_u8Size;
            memcpy((void*)g_au8NMEALastZDA, l_au8NMEABuffer, l_u8Size);
            break;
         default:
            break;
      }
      
      if(   (g_sNMEAGSA.u8FixType >= 2u) 
         || (g_sNMEAGGA.u8FixIndicator == 1u) 
         || (g_sNMEARMC.u8Status == 'A') ) 
      {
         g_u8LocationFixed = 1u;
      }
      else
      {
         g_u8LocationFixed = 0u;
      }
      #ifdef SDCARD_LOG
         vFS_Write((char*)l_au8NMEABuffer, l_u8Size);
         l_u8Idx++;
         if(l_u8Idx % 200)
         {
            eFS_Sync();
         }
      #endif
   }
   
}


void vNMEA_LastGGAFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size)
{
   if((p_pau8Frame != NULL) && (p_pu8Size != NULL))
   {
      memcpy(p_pau8Frame, (void*)g_au8NMEALastGGA, g_u8LastGGASize);
      (*p_pu8Size) = g_u8LastGGASize;
   }
}
void vNMEA_LastDecodedGGAGet(s_NMEA_GGA_t * p_psGGA)
{
   if(p_psGGA != NULL)
   {
      (*p_psGGA) = g_sNMEAGGA;
   }
}

void vNMEA_LastRMCFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size)
{
   if((p_pau8Frame != NULL) && (p_pu8Size != NULL))
   {
      memcpy(p_pau8Frame, (void*)g_au8NMEALastRMC, g_u8LastRMCSize);
      (*p_pu8Size) = g_u8LastRMCSize;
   }
}
void vNMEA_LastDecodedRMCGet(s_NMEA_RMC_t * p_psRMC)
{
   if(p_psRMC != NULL)
   {
      (*p_psRMC) = g_sNMEARMC;
   }
}

void vNMEA_LastGSAFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size)
{
   if((p_pau8Frame != NULL) && (p_pu8Size != NULL))
   {
      memcpy(p_pau8Frame, (void*)g_au8NMEALastGSA, g_u8LastGSASize);
      (*p_pu8Size) = g_u8LastGSASize;
   }
}
void vNMEA_LastDecodedGSAGet(s_NMEA_GSA_t * p_psGSA)
{
   if(p_psGSA != NULL)
   {
      (*p_psGSA) = g_sNMEAGSA;
   }
}


void vNMEA_LastGSTFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size)
{
   if((p_pau8Frame != NULL) && (p_pu8Size != NULL))
   {
      memcpy(p_pau8Frame, (void*)g_au8NMEALastGST, g_u8LastGSTSize);
      (*p_pu8Size) = g_u8LastGSTSize;
   }
}
void vNMEA_LastDecodedGSTGet(s_NMEA_GST_t * p_psGST)
{
   if(p_psGST != NULL)
   {
      (*p_psGST) = g_sNMEAGST;
   }
}

void vNMEA_LastZDAFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size)
{
   if((p_pau8Frame != NULL) && (p_pu8Size != NULL))
   {
      memcpy(p_pau8Frame, (void*)g_au8NMEALastZDA, g_u8LastZDASize);
      (*p_pu8Size) = g_u8LastZDASize;
   }
}
void vNMEA_LastDecodedZDAGet(s_NMEA_ZDA_t * p_psZDA)
{
   if(p_psZDA != NULL)
   {
      (*p_psZDA) = g_sNMEAZDA;
   }
}

void vNMEA_LastGSVFrameGet(uint8_t * p_pau8FrameOne, uint8_t * p_pu8SizeOne, 
                           uint8_t * p_pau8FrameTwo, uint8_t * p_pu8SizeTwo, 
                           uint8_t * p_pau8FrameThree, uint8_t * p_pu8SizeThree)
{
   if((p_pau8FrameOne != NULL) && (p_pu8SizeOne != NULL))
   {
      memcpy(p_pau8FrameOne, (void*)g_au8NMEALastGSV, g_u8LastGSVSize);
      (*p_pu8SizeOne) = g_u8LastGSVSize;
   }
   if((p_pau8FrameTwo != NULL) && (p_pu8SizeTwo != NULL))
   {
      memcpy(p_pau8FrameTwo, (void*)g_au8NMEALastGSV, g_u8LastGSVSize);
      (*p_pu8SizeTwo) = g_u8LastGSVSize;
   }
   if((p_pau8FrameThree != NULL) && (p_pu8SizeThree != NULL))
   {
      memcpy(p_pau8FrameThree, (void*)g_au8NMEALastGSV, g_u8LastGSVSize);
      (*p_pu8SizeThree) = g_u8LastGSVSize;
   }
}

void vNMEA_LastDecodedGSVGet(s_NMEA_GSV_t * p_psGSV)
{   
   if(p_psGSV != NULL)
   {
      p_psGSV = g_sNMEAGSV;
   }
}


void vNMEA_PMTKGet(s_NMEA_PMTK_t * p_psPMTK)
{
   if(p_psPMTK != NULL)
   {
      (*p_psPMTK) = g_sNMEAPMTK;
   }
}

void vNMEA_PMTKClear(void)
{
   g_sNMEAPMTK.eAck = PMTK_ACK_INVALID_PCK;
   g_sNMEAPMTK.u16Cmd = UINT16_MAX;
   g_sNMEAPMTK.u16Type = UINT16_MAX;
}

void vNMEA_IsFixed(uint8_t * p_pu8IsFixed)
{
   if(p_pu8IsFixed != NULL)
   {
      (*p_pu8IsFixed) = g_u8LocationFixed;
   }
}

void vNMEA_FixReset(void)
{
   g_u8LocationFixed = 0u;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/

/**@brief  Queue new message.
 * @param[in] p_au8Message : data to send later.
 * @param[in] p_u8MsgSize : size of data.
 * @return None.
 */
static void vNMEAQueueMsg(uint8_t * p_au8Message, uint8_t p_u8MsgSize)
{
   int8_t l_s8Diff = 0;
   
   /* Check Overflow */
   if(g_u8NMEAMsgIn >= g_u8NMEAMsgOut)
   {
      l_s8Diff = (int8_t)g_u8NMEAMsgOut - (int8_t)g_u8NMEAMsgIn;
   }
   else
   {
      l_s8Diff = ((int8_t)NMEA_SCHEDULER_QUEUE_SIZE - (int8_t)g_u8NMEAMsgOut) + (int8_t)g_u8NMEAMsgIn;
   }
   
   if(l_s8Diff < NMEA_SCHEDULER_QUEUE_SIZE)
   {
      //memset(&g_sNMEAQueue[g_u8NMEAMsgIn].au8NMEAFrame[0u], 0u, NMEA_MAX_SIZE);
      memcpy(&g_sNMEAQueue[g_u8NMEAMsgIn].au8NMEAFrame[0u], p_au8Message, p_u8MsgSize);
      g_sNMEAQueue[g_u8NMEAMsgIn].u8Size = p_u8MsgSize;      
      
      g_u8NMEAMsgIn++;
      g_u8NMEAMsgIn %= NMEA_SCHEDULER_QUEUE_SIZE;
   }
   else
   {
      __nop();
   }
}

/**@brief  Enqueue message.
 * @param[out] p_au8Message : data to send.
 * @param[out] p_u8MsgSize : size of data.
 * @return None.
 */
static void vNMEAEnqueueMsg(uint8_t * p_au8Message, uint8_t * p_pu8MsgSize)
{   
   memcpy(p_au8Message, &(g_sNMEAQueue[g_u8NMEAMsgOut].au8NMEAFrame[0u]), g_sNMEAQueue[g_u8NMEAMsgOut].u8Size);
   (*p_pu8MsgSize) = g_sNMEAQueue[g_u8NMEAMsgOut].u8Size;
   
   /* Clear queue index */
   //memset(&(g_sNMEAQueue[g_u8NMEAMsgOut].au8NMEAFrame[0u]), 0u, NMEA_MAX_SIZE);
   //g_sNMEAQueue[g_u8NMEAMsgOut].u8Size = 0u;
   
   g_u8NMEAMsgOut++;
   g_u8NMEAMsgOut %= NMEA_SCHEDULER_QUEUE_SIZE;
}

/**@brief  Check if queue of message is empty.
 * @return 1 if Queue is empty else 0.
 */
static uint8_t u8NMEAIsQueueEmpty(void)
{
   return (g_u8NMEAMsgOut == g_u8NMEAMsgIn)?1u:0u;
}

static float fDMStoDD(double p_dDegMinSec, char p_chCardinalPt)
{
   float l_fDMS = 0.0f;
   float l_fIntPart = 0.0f;
   float l_fFloatingPart = 0.0f;
   
   /* Get the first two digits by turning f into an integer, then doing an integer divide by 100;
      firsttowdigits should be 77 at this point. */
   l_fIntPart = (int32_t)(p_dDegMinSec/100.0f);
   l_fFloatingPart = p_dDegMinSec - (float)(l_fIntPart*100.0f);
   l_fDMS = (float)((float)l_fIntPart + (l_fFloatingPart/60.0f));

   if(   (p_chCardinalPt == 'S') 
      || (p_chCardinalPt == 'O') )
   {
      l_fDMS = -(l_fDMS);
   }
   return l_fDMS;    
}

static inline uint8_t u8CharHexToByte(char p_chChar)
{
   return (uint8_t)(p_chChar - ( p_chChar <= '9' ? '0' : p_chChar <= 'F' ? ('A'-10) : ('a'-10)));
}

static uint8_t u8SentenceIDGet(uint8_t * p_pau8Sentence)
{
   uint8_t l_u8SentenceID = 0u;
   uint8_t l_u8IDFound = 0u;
   char l_achSentenceIdx[NMEA_SENTENCE_SIZE] = { 0 };
   
   do 
   {
      strcpy(l_achSentenceIdx, g_cachNMEAID[l_u8SentenceID]);
      if(strstr((char*)p_pau8Sentence, l_achSentenceIdx) != NULL)
      {
         l_u8IDFound = 1u;
      }
      else
      {
         l_u8SentenceID++;
      }
   }while(  (l_u8SentenceID < NMEA_SENTENCES_NB)
         && (l_u8IDFound != 1u) );
   
   if(l_u8SentenceID >= NMEA_SENTENCES_NB)
   {
      l_u8SentenceID = NMEA_UNKNOWN;
   }
   
   return l_u8SentenceID;   
}

/* PMTK Parser */
//static void vParserPMTK(uint8_t * p_pau8Frame, uint8_t p_u8Size, void * p_pvDecoded)
//{
//	uint8_t l_u8Idx = 0u;
//	uint8_t l_u8Size = 0u;
//	uint8_t l_au8Buffer[NMEA_SPLIT_DATA_SIZE] = { 0u };
//	uint8_t l_u8DecoderFunction = 0u;
//	uint8_t l_u8End = 0u;
//   const uint8_t l_cu8SentenceSize = strlen(g_cachNMEAID[NMEA_PMTK]) + 1u; //for NMEA_CHAR_SOF_DOLLAR 
//   
//   if(   (p_pau8Frame != NULL)
//      && (p_pvDecoded != NULL) )
//   {
//      l_u8Idx = l_cu8SentenceSize;
//            
//      while(((l_u8Idx + l_u8Size) < p_u8Size)
//         && (l_u8End != 1u))
//      {
//         l_u8Size = 0u;
//          
//         if(l_u8Idx == l_cu8SentenceSize)
//         {  /* For first param (Type) */
//            l_u8Size = 3u;
//         }
//         else
//         {
//            // Getting size of data between comma char ',' (and '*')
//            while((p_pau8Frame[l_u8Idx + l_u8Size] != NMEA_CHAR_COMMA) 
//               && (p_pau8Frame[l_u8Idx + l_u8Size] != NMEA_CHAR_START))
//            {
//               l_u8Size++;
//            }
//         }

//         if(p_pau8Frame[l_u8Idx + l_u8Size] == NMEA_CHAR_START)
//         {
//            if(l_u8Size != 0u)
//            {
//               memset(l_au8Buffer, 0u, NMEA_SPLIT_DATA_SIZE);
//               memcpy(l_au8Buffer, p_pau8Frame + l_u8Idx, l_u8Size);
//               
//               vDecoderNMEA(NMEA_PMTK, l_u8DecoderFunction, l_au8Buffer, l_u8Size, p_pvDecoded);
//            }

//            l_u8Idx++; // To pass the '*'
//            memcpy(l_au8Buffer, p_pau8Frame + l_u8Idx + l_u8Size, 2);

//            l_u8End = 1;
//         }
//         else
//         {
//            if(l_u8Size != 0u)
//            {
//               memset(l_au8Buffer, 0u, NMEA_SPLIT_DATA_SIZE);
//               memcpy(l_au8Buffer, p_pau8Frame + l_u8Idx, l_u8Size);
//               
//               vDecoderNMEA(NMEA_PMTK, l_u8DecoderFunction, l_au8Buffer, l_u8Size, p_pvDecoded);

//               l_u8Idx += l_u8Size;
//            }

//            l_u8DecoderFunction++;	// To call the next decoder function

//            l_u8Idx++; // To pass the comma
//         }
//      }
//   }
//}

/* Common Parser */
static void vParserNMEA(e_NMEASentence_t p_eSentenceID, uint8_t * p_pau8Frame, uint8_t p_u8Size, void * p_pvDecoded)
{
	uint8_t l_u8Idx = 0u;
	uint8_t l_u8Size = 0u;
	uint8_t l_au8Buffer[NMEA_SPLIT_DATA_SIZE] = { 0u };
	uint8_t l_u8DecoderFunction = 0u;
	uint8_t l_u8End = 0u;
   const uint8_t l_cu8PMTKSize = strlen(g_cachNMEAID[NMEA_PMTK]) + 1u; //for NMEA_CHAR_SOF_DOLLAR 

   if(   (p_pau8Frame != NULL)
      && (p_pvDecoded != NULL) )
   {
      if(p_eSentenceID != NMEA_PMTK)
      {  // Avoid Sentence ID
         while(p_pau8Frame[l_u8Idx++] != NMEA_CHAR_COMMA);
      }
      else
      {
         l_u8Idx = strlen(g_cachNMEAID[p_eSentenceID]) + 1u; //for NMEA_CHAR_SOF_DOLLAR 
      }
      
      while(((l_u8Idx + l_u8Size) < p_u8Size)
         && (l_u8End != 1u))
      {
         l_u8Size = 0u;
         
         if(l_u8Idx == l_cu8PMTKSize)
         {  // For first param (Type)
            l_u8Size = 3u;
         }
         else
         {  // Getting size of data between comma char ',' (and '*')
            while((p_pau8Frame[l_u8Idx + l_u8Size] != NMEA_CHAR_COMMA) 
               && (p_pau8Frame[l_u8Idx + l_u8Size] != NMEA_CHAR_START))
            {
               l_u8Size++;
            }
         }
         
         if(p_pau8Frame[l_u8Idx + l_u8Size] == NMEA_CHAR_START)
         {
            if(l_u8Size != 0u)
            {
               memset(l_au8Buffer, 0u, NMEA_SPLIT_DATA_SIZE);
               memcpy(l_au8Buffer, p_pau8Frame + l_u8Idx, l_u8Size);
               
               vDecoderNMEA(p_eSentenceID, l_u8DecoderFunction, l_au8Buffer, l_u8Size, p_pvDecoded);
            }

            l_u8Idx++; // To pass the '*'
            memcpy(l_au8Buffer, p_pau8Frame + l_u8Idx + l_u8Size, 2);

            l_u8End = 1;
         }
         else
         {
            if(l_u8Size != 0u)
            {
               memset(l_au8Buffer, 0u, NMEA_SPLIT_DATA_SIZE);
               memcpy(l_au8Buffer, p_pau8Frame + l_u8Idx, l_u8Size);
               
               vDecoderNMEA(p_eSentenceID, l_u8DecoderFunction, l_au8Buffer, l_u8Size, p_pvDecoded);

               l_u8Idx += l_u8Size;
            }

            l_u8DecoderFunction++;	// To call the next decoder function

            l_u8Idx++; // To pass the comma
         }
      }
   }
}

static void vDecoderNMEA(e_NMEASentence_t p_eSentenceID, uint8_t p_u8FunctionIdx, uint8_t * p_pau8Buffer, uint8_t p_u8Size, void * p_pvDecoded)
{
   switch(p_eSentenceID)
   {
      case NMEA_PSRF:
         break;
      case NMEA_PMTK:
         if( ((*g_cafpvPMTKDecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_PMTK_DECODER_FCT_NB) ) 
         {
            (*g_cafpvPMTKDecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_PMTK_t*)p_pvDecoded);
         }
         break;
      case NMEA_GPRMC:
      case NMEA_GNRMC:
         if( ((*g_cafpvRMCDecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_RMC_DECODER_FCT_NB) ) 
         {
            (*g_cafpvRMCDecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_RMC_t*)p_pvDecoded);
         }
         break;
      case NMEA_GPGGA:
      case NMEA_GNGGA:
         if( ((*g_cafpvGGADecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_GGA_DECODER_FCT_NB) ) 
         {
            (*g_cafpvGGADecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_GGA_t*)p_pvDecoded);
         }
         break;
      case NMEA_GPGSA:
         if( ((*g_cafpvGSADecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_GSA_DECODER_FCT_NB) ) 
         {
            (*g_cafpvGSADecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_GSA_t*)p_pvDecoded);
         }
         break;
      case NMEA_GPGLL:
      case NMEA_GNGLL:
         if( ((*g_cafpvGLLDecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_GLL_DECODER_FCT_NB) ) 
         {
            (*g_cafpvGLLDecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_GLL_t*)p_pvDecoded);
         }
         break;
      case NMEA_GPGSV:
//         if( ((*g_cafpvGSVDecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_GSV_DECODER_FCT_NB) ) 
//         {
//            (*g_cafpvGSVDecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_GSV_t*)p_pvDecoded);
//         }
         break;
      case NMEA_GPGST:
         if( ((*g_cafpvGSTDecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_GST_DECODER_FCT_NB) ) 
         {
            (*g_cafpvGSTDecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_GST_t*)p_pvDecoded);
         }
         break;
      case NMEA_GNZDA:
         if( ((*g_cafpvZDADecoder[p_u8FunctionIdx]) != NULL) && (p_u8FunctionIdx < NMEA_ZDA_DECODER_FCT_NB) ) 
         {
            (*g_cafpvZDADecoder[p_u8FunctionIdx])(p_pau8Buffer, p_u8Size, (s_NMEA_ZDA_t*)p_pvDecoded);
         }
         break;
      default:
         break;
   }
}


/*************************
 * Decoder functions ACK *
 *************************/
static void vPMTKType(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16Type = (uint16_t)((uint16_t)u8CharHexToByte((char)p_pu8Data[0u])*100u + 
                                       (uint16_t)u8CharHexToByte((char)p_pu8Data[1u])*10u + 
                                       (uint16_t)u8CharHexToByte((char)p_pu8Data[2u]));
   }
}
static void vPMTKCmd(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16Cmd = (uint16_t)((uint16_t)u8CharHexToByte((char)p_pu8Data[0u])*100u + 
                                       (uint16_t)u8CharHexToByte((char)p_pu8Data[1u])*10u + 
                                       (uint16_t)u8CharHexToByte((char)p_pu8Data[2u]));
   }
}
static void vPMTKFlag(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_PMTK_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      if(p_psDecoded->u16Type == 1)
      {
         p_psDecoded->eAck = (e_PMTK_Ack_t)u8CharHexToByte((char)p_pu8Data[0u]);
      }
      else
      {
         p_psDecoded->eAck = PMTK_ACK_UNKNOWN;
      }
   }
}

/*************************
 * Decoder functions GGA *
 *************************/
static void vGGAFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(p_psDecoded->au8FixDateUTC, p_pu8Data, p_u8Size);
   }
}
static void vGGALatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->dLatitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGGALatitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size == 1u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(&p_psDecoded->u8LatNS, p_pu8Data, p_u8Size);
      p_psDecoded->dLatitude = fDMStoDD(p_psDecoded->dLatitude, p_psDecoded->u8LatNS);
   }
}
static void vGGALongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->dLongitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGGALongitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size == 1u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(&p_psDecoded->u8LonEW, p_pu8Data, p_u8Size);
      p_psDecoded->dLongitude = fDMStoDD(p_psDecoded->dLongitude, p_psDecoded->u8LonEW);
   }
}
static void vGGAFixIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{   
   if( (p_u8Size == 1u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8FixIndicator = u8CharHexToByte((char)p_pu8Data[0u]);
   }
}
static void vGGASatellitesNumber(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8NbSatTracked = (uint8_t)(strtod((char*)p_pu8Data, NULL));
   }
}
static void vGGAHDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16HDOP = (uint16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vGGAAltitudeAboveSea(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->s16AltSeaLevel = (int16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vGGAAltitudeAboveSeaUnit(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8AltSeaLevelUnit = p_pu8Data[0u];
   }
}
static void vGGAHeightEllipsoid(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->s16HeightEllipsoid = (int16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vGGAHeightEllipsoidUnit(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GGA_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8HeightEllipsoidUnit = p_pu8Data[0u];
   }
}


/*************************
 * Decoder functions RMC *
 *************************/
static void vRMCFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(p_psDecoded->au8FixDateUTC, p_pu8Data, p_u8Size);
   }
}
static void vRMCStatus(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8Status = p_pu8Data[0u];
   }
}
static void vRMCLatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->dLatitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vRMCLatitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size == 1u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(&p_psDecoded->u8LatNS, p_pu8Data, p_u8Size);
   }
}
static void vRMCLongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->dLongitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vRMCLongitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size == 1u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(&p_psDecoded->u8LonEW, p_pu8Data, p_u8Size);
   }
}
static void vRMCSpeed(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16Speed= (int16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vRMCTrackAngle(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16TrackAngle= (int16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vRMCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(p_psDecoded->au8Date, p_pu8Data, p_u8Size);
   }
}
static void vRMCMagneticVariation(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16MagneticVariation = (int16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vRMCMagneticIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8MagneticSens = p_pu8Data[0u];
   }
}
static void vRMCModePosition(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_RMC_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8ModePosition = p_pu8Data[0u];
   }
}


/*************************
 * Decoder functions GSA *
 *************************/
static void vGSAAutoManuSel(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8AutoManuSelect = p_pu8Data[0u];
   }
}
static void vGSAFixType(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8FixType = u8CharHexToByte((char)p_pu8Data[0u]);
   }
}
static void vGSAPRN0(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[0u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN1(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[1u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN2(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[2u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN3(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[3u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN4(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[4u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN5(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[5u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN6(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[6u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN7(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[7u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN8(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[8u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN9(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[9u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN10(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[10u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPRN11(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size == 2u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->au8PRNSat[11u] = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]);
   }
}
static void vGSAPDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16PDOP = (uint16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vGSAHDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16HDOP = (uint16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vGSAVDOP(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GSA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16VDOP = (uint16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}


/*************************
 * Decoder functions GLL *
 *************************/
static void vGLLLatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->dLatitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGLLLatitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(&p_psDecoded->u8LatNS, p_pu8Data, p_u8Size);
      p_psDecoded->dLatitude = fDMStoDD(p_psDecoded->dLatitude, p_psDecoded->u8LatNS);
   }
}
static void vGLLLongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->dLongitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGLLLongitudeIndicator(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(&p_psDecoded->u8LonEW, p_pu8Data, p_u8Size);
      p_psDecoded->dLongitude = fDMStoDD(p_psDecoded->dLongitude, p_psDecoded->u8LonEW);
   }
}
static void vGLLFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(p_psDecoded->au8FixDateUTC, p_pu8Data, p_u8Size);
   }
}
static void vGLLModePosition(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GLL_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8ModePosition = p_pu8Data[0u];
   }
}

/*************************
 * Decoder functions GST *
 *************************/
static void vGSTFixUTCDate(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(p_psDecoded->au8FixDateUTC, p_pu8Data, p_u8Size);
   }
}
static void vGSTRMS(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16RMSPseudoRangeResiduals = (uint16_t)(strtod((char*)p_pu8Data, NULL)*10.0f);
   }
}
static void vGSTErrorSemiMajor(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->f32SigmaErrorSemiMajor = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGSTErrorSemiMinor(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->f32SigmaErrorSemiMinor = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGSTErrorOrientation(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->f32ErrorOrientation = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGSTErrorLatitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->f32SigmaErrorLatitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGSTErrorLongitude(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->f32SigmaErrorLongitude = strtod((char*)p_pu8Data, NULL);
   }
}
static void vGSTErrorHeight(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_GST_t * p_psDecoded)
{
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->f32SigmaErrorHeight = strtod((char*)p_pu8Data, NULL);
   }
}

/*************************
 * Decoder functions ZDA *
 *************************/
static void vZDAUTCTime(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      memcpy(p_psDecoded->au8DateUTC, p_pu8Data, p_u8Size);
   }
}
static void vZDADay(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8Day = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]); 
   }
}
static void vZDAMonth(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8Month = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]); 
   }
}
static void vZDAYear(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u16Year = u8CharHexToByte((char)p_pu8Data[0u])*1000u + u8CharHexToByte((char)p_pu8Data[1u])*100u 
                              + u8CharHexToByte((char)p_pu8Data[2u])*10u + u8CharHexToByte((char)p_pu8Data[3u]);
   }
}
static void vZDAOffsetGMTHour(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->s8OffsetGMTHours = (int8_t)u8CharHexToByte((char)p_pu8Data[0u])*10u + (int8_t)u8CharHexToByte((char)p_pu8Data[1u]); 
   }
}
static void vZDAOffsetGMTMinute(uint8_t * p_pu8Data, uint8_t p_u8Size, s_NMEA_ZDA_t * p_psDecoded)
{   
   if( (p_u8Size != 0u) && (p_pu8Data != NULL) && (p_psDecoded != NULL) )
   {
      p_psDecoded->u8OffsetGMTMinutes = u8CharHexToByte((char)p_pu8Data[0u])*10u + u8CharHexToByte((char)p_pu8Data[1u]); 
   }
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/

