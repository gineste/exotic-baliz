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
#ifndef NMEA_H
#define NMEA_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define NMEA_FIX_DATE_UTC  (uint8_t)10
#define NMEA_PRN_SAT_NB    (uint8_t)12
#define NMEA_DATE_SIZE     (uint8_t)6

#define NMEA_GSV_STRUCT_ARRAY    (uint8_t)3u
#define NMEA_GSV_SAT_INFO_ARRAY  (uint8_t)4u

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
 typedef enum _NMEA_PMTK_ACK_ENUM_ {
    PMTK_ACK_INVALID_PCK = 0u,
    PMTK_ACK_UNSUPPORTED_PCK_TYPE,
    PMTK_ACK_VALID_PCK_BUT_ACT_FAILED,
    PMTK_ACK_VALID_PCK_ACT_SUCCEEDED,
    PMTK_ACK_UNKNOWN
 }e_PMTK_Ack_t;
 
 typedef struct _NMEA_PMTK_STRUCT_ {
   uint16_t u16Type;
   uint16_t u16Cmd;
   e_PMTK_Ack_t eAck; 
}s_NMEA_PMTK_t;
 
typedef struct _NMEA_GGA_STRUCT_ {
   uint8_t au8FixDateUTC[NMEA_FIX_DATE_UTC];
   double dLatitude;
   double dLongitude;
   uint8_t u8LatNS;
   uint8_t u8LonEW;
   uint8_t u8FixIndicator;
   uint8_t u8NbSatTracked;
   uint16_t u16HDOP; /* 0.1 unit */
   int16_t s16AltSeaLevel; /* 0.1 unit */
   uint8_t u8AltSeaLevelUnit; 
   int16_t s16HeightEllipsoid; /* 0.1 unit */
   uint8_t u8HeightEllipsoidUnit;
   uint16_t u16TimeSinceLastDGPSUpdate;
   uint16_t u16DGPSStationID;
}s_NMEA_GGA_t;

typedef struct _NMEA_RMC_STRUCT_ {
   uint8_t au8FixDateUTC[NMEA_FIX_DATE_UTC];
   double dLatitude;
   double dLongitude;
   uint8_t u8LatNS;
   uint8_t u8LonEW;
   uint8_t u8Status;
   uint16_t u16Speed;   /* 0.1 unit */
   uint16_t u16TrackAngle; /* 0.1 unit */
   uint8_t au8Date[NMEA_DATE_SIZE];
   uint16_t u16MagneticVariation; /* 0.1 unit */
   uint8_t u8MagneticSens;
   uint8_t u8ModePosition;
}s_NMEA_RMC_t;

typedef struct _NMEA_GSA_STRUCT_ {
   uint8_t u8AutoManuSelect;
   uint8_t u8FixType;
   uint8_t au8PRNSat[NMEA_PRN_SAT_NB];
   uint16_t u16PDOP;    /* 0.1 unit */
   uint16_t u16HDOP;    /* 0.1 unit */
   uint16_t u16VDOP;    /* 0.1 unit */
}s_NMEA_GSA_t;

typedef struct _NMEA_GLL_STRUCT_ {
   double dLatitude;
   double dLongitude;
   uint8_t u8LatNS;
   uint8_t u8LonEW;
   uint8_t au8FixDateUTC[NMEA_FIX_DATE_UTC];
   uint8_t u8ModePosition;
}s_NMEA_GLL_t;

typedef struct _NMEA_ZDA_STRUCT_ {
   uint8_t au8DateUTC[NMEA_FIX_DATE_UTC];
   uint8_t u8Day;
   uint8_t u8Month;
   uint16_t u16Year;
   int8_t s8OffsetGMTHours;
   uint8_t u8OffsetGMTMinutes;
}s_NMEA_ZDA_t;

typedef struct _NMEA_GSV_STRUCT_ {
   uint8_t u8SentenceNb;
   uint8_t u8SentenceIdx;
   uint8_t u8SatInView;
   struct {
      uint8_t u8SatPRNNb;
      uint8_t u8ElevationDeg;
      uint16_t u16AzimuthDeg;
      uint8_t u8SNR;
   }asSatInfo[NMEA_GSV_SAT_INFO_ARRAY];
}s_NMEA_GSV_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vNMEA_InitProtocol(void);
void vNMEA_UpdateFrame(uint8_t p_u8Byte);
void vNMEA_FrameProcessing(void);
void vNMEA_LastGGAFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size);
void vNMEA_LastDecodedGGAGet(s_NMEA_GGA_t * p_psGGA);
void vNMEA_LastRMCFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size);
void vNMEA_LastDecodedRMCGet(s_NMEA_RMC_t * p_psRMC);
void vNMEA_LastGSAFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size);
void vNMEA_LastDecodedGSAGet(s_NMEA_GSA_t * p_psGSA);
void vNMEA_LastZDAFrameGet(uint8_t * p_pau8Frame, uint8_t * p_pu8Size);
void vNMEA_LastDecodedZDAGet(s_NMEA_ZDA_t * p_psZDA);
void vNMEA_LastGSVFrameGet(uint8_t * p_pau8FrameOne, uint8_t * p_pu8SizeOne, 
                           uint8_t * p_pau8FrameTwo, uint8_t * p_pu8SizeTwo, 
                           uint8_t * p_pau8FrameThree, uint8_t * p_pu8SizeThree);
void vNMEA_LastDecodedGSVGet(s_NMEA_GSV_t * p_psGSV);

void vNMEA_PMTKGet(s_NMEA_PMTK_t * p_psPMTK);

void vNMEA_IsFixed(uint8_t * p_pu8IsFixed);

#endif /* NMEA_H */

