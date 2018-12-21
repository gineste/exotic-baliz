/****************** (C) COPYRIGHT 2017 EXOTIC SYSTEMS *********************
*
* File name:	GPS.h
*
* Copyright:	EXOTIC SYSTEMS
* Date:			06 Jan. 2017
* Author:		Thomas Impéry
* Description: Module AT
*
*******************************************************************************/

/* Multi-include protection --------------------------------------------------*/
#ifndef __GPS_H__
#define __GPS_H__


/* Defines -------------------------------------------------------------------*/
#define NB_FUNCTION_GGA 			13

/* Includes ------------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Public types -------------------------------------------------------------*/
typedef struct GGA
{
	uint8_t	 	UTC[10];					//	hhmmss.sss
	float			latitude;					// 	in degrees
	uint8_t		latitudeStr[10];		// 	in degrees
	uint8_t		NSIndicator;			// 	N or S
	float			longitude;				// 	in degrees 
	uint8_t		longitudeStr[11];	// 	in degrees 
	uint8_t		EWIndicator;			// 	E or W
	uint8_t		fixIndicator;			// 	0: not available, 1: SPS valid , 2: differential + SPS valid
	uint8_t		nbSatellites;			//	Range 0 to 12
	float 		HDOP;							// 	Horizontal Dilution  of Precision (1 is the best)
	float 		MSLAltitude;			//	Altitude 
	uint8_t		unitAltitude;			// 	Unit used for altitude
	float			geoidSeparation;	//	Geoid-to-ellipsoid separation (in meters)
	uint8_t		unitGeoid;				// 	Unit used for Geoid
	uint8_t		diffRefStationID;	//
} GGA_t;

typedef struct _GEODETIC_DATA_ {
   uint16_t u16NavValid;
   uint16_t u16NavType;
   uint16_t u16ExtWeekNb;
   uint32_t u32TOW;
   uint16_t u16Year;
   uint8_t  u8Month;
   uint8_t  u8Day;
   uint8_t  u8Hour;
   uint8_t  u8Minute;
   uint8_t  u8Second;
   uint32_t u32SatelliteIDList;
   int32_t  s32Latitude;
   int32_t  s32Longitude;
   int32_t  s32AltEllipsoid;
   int32_t  s32AltMSL;
   uint8_t  u8MapDatum;
   uint16_t u16SpeedOverGround;
   uint16_t u16CourseOverGround;
   int16_t  s16MagneticVariation;
   int16_t  s16ClimbRate;
   int16_t  s16HeadingRate;
   uint32_t u32EHPE;
   uint32_t u32EVPE;
   uint32_t u32ETE;
   uint32_t u32EHVE;
   uint32_t u32ClockBias;
   uint32_t u32ClockBiasError;
   int32_t  s32ClockDrift;
   uint32_t u32ClockDriftError;
   uint32_t u32Distance;
   uint16_t u16DistanceError;
   uint16_t u16HeadingError;
   uint8_t  u8NbSVinFix;
   uint8_t  u8HDOP;
   uint8_t  u8AdditionalModeInfo;
}s_Geodetic_t;

/* Private types -------------------------------------------------------------*/
typedef void ( *functionsGGA )( uint8_t* payload, GGA_t* GGAFrame, uint8_t size);


/* Private Variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
//GGA_t GPS_decodeGGA(const uint8_t* payload, uint8_t size);

uint8_t u8GPS_decodeGGA(const uint8_t* payload, uint8_t size, GGA_t * p_psGGA);
s_Geodetic_t GPS_DecodeGeodetic(uint8_t* p_u8Payload);

#endif /*__GPS_H__*/
