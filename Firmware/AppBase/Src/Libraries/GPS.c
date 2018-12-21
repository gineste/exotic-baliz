/****************** (C) COPYRIGHT 2017 EXOTIC SYSTEMS *********************
*
* File name:	GPS.c
*
* Copyright:	EXOTIC SYSTEMS
* Date:			06 Jan. 2017
* Author:		Thomas Impéry
* Description: Module GPS nRF52
*
*******************************************************************************/

/* Defines -------------------------------------------------------------------*/



/* Multi-include protection --------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "GPS.h"

#include "Tool.h"
#include "GlobalDefs.h"

#include <string.h>

static void GPS_decodeUTC(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeLatitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeNSIndicator(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeLongitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeEWIndicator(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeFixIndicator(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeNbSatellites(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeHDOP(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeAltitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeUnitAltitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeGeoid(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeGeoidUnit(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);
static void GPS_decodeDiffRedStationID(uint8_t* payload, GGA_t* GGAFrame, uint8_t size);

//static uint8_t GPS_validateChecksum(const uint8_t* payload, const uint8_t* checksum);

/* Private constants ---------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Public types -------------------------------------------------------------*/

/* Private Variables ---------------------------------------------------------*/
static functionsGGA functions_decoder[NB_FUNCTION_GGA] = { 
	GPS_decodeUTC, GPS_decodeLatitude,
	GPS_decodeNSIndicator, GPS_decodeLongitude,
	GPS_decodeEWIndicator, GPS_decodeFixIndicator,
	GPS_decodeNbSatellites, GPS_decodeHDOP,
	GPS_decodeAltitude, GPS_decodeUnitAltitude, GPS_decodeGeoid,
	GPS_decodeGeoidUnit, GPS_decodeDiffRedStationID
};

// Latitude : 4547.5636 = 45.7848989
// Longitude : 00308.2302 = 3.139727777777778


static float GPS_Float2Degres(float p_fCoord)
{
   // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
   // firsttowdigits should be 77 at this point.
   float l_fDeg = (int32_t)(p_fCoord/100.0f); //This assumes that f < 10000.
   float l_fDeg2 = p_fCoord - (float)(l_fDeg*100);
   float l_fTheFinalAnswer = (float)((float)l_fDeg + (l_fDeg2/60.0f));

   return l_fTheFinalAnswer; 
}

static void GPS_decodeUTC(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
   if(size !=0u)
   {
      memcpy(GGAFrame->UTC, payload, size);
   }
}

static void GPS_decodeLatitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{	
   if(size != 0u)
   {
      GGAFrame->latitude = GPS_Float2Degres(Tool_StringToFloat(payload, size));
      memcpy(&(GGAFrame->latitudeStr), payload, size);
      GGAFrame->latitudeStr[size] = '\0';
   }
}

static void GPS_decodeNSIndicator(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->NSIndicator = payload[0];
}

static void GPS_decodeLongitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{	
   if(size != 0u)
   {
      GGAFrame->longitude = GPS_Float2Degres(Tool_StringToFloat(payload, size));
      memcpy(GGAFrame->longitudeStr, payload, size);
      GGAFrame->longitudeStr[size] = '\0';
   }
}

static void GPS_decodeEWIndicator(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->EWIndicator = payload[0];
}

static void GPS_decodeFixIndicator(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->fixIndicator = Tool_StringToInt(payload, size);
}

static void GPS_decodeNbSatellites(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->nbSatellites = Tool_StringToInt(payload, size);
}

static void GPS_decodeHDOP(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->HDOP = Tool_StringToFloat(payload, size);
}

static void GPS_decodeAltitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->MSLAltitude = Tool_StringToFloat(payload, size);
}

static void GPS_decodeUnitAltitude(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->unitAltitude = payload[0];
}

static void GPS_decodeGeoid(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->geoidSeparation = Tool_StringToFloat(payload, size);
}

static void GPS_decodeGeoidUnit(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->unitGeoid = payload[0];
}

static void GPS_decodeDiffRedStationID(uint8_t* payload, GGA_t* GGAFrame, uint8_t size)
{
	GGAFrame->diffRefStationID = Tool_StringToInt(payload, size);
}

/*static uint8_t GPS_validateChecksum(const uint8_t* payload, const uint8_t* check)
{
	uint8_t checksum;
	uint8_t crc = 0;
	
	checksum	= Tool_AsciiToHexa(check[0]) << 4;
	checksum	|= Tool_AsciiToHexa(check[1]);
  
  ++payload; // '$'
  
  while(*payload != '*')
  {
    crc ^= *payload;
    ++payload;
  }
	
  return (checksum == crc)? 1u : 0u;
}*/
static uint8_t GPS_Checksum(const uint8_t* payload, uint8_t size)
{
	uint8_t checksum;
	uint8_t crc = 0;
  
   if(payload[0u] == '$' && size > 0)
   {
      ++payload; // '$'

      while(*payload != '*')
      {
         crc ^= *payload;
         ++payload;
      }
         
      ++payload;
      checksum	= Tool_AsciiToHexa(*payload) << 4;
      ++payload;
      checksum	|= Tool_AsciiToHexa(*payload);      
   }
  
	
  return (checksum == crc)? 1u : 0u;
}



/* Public Variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static uint8_t GPS_splitGGA(const uint8_t* payload, uint8_t size, GGA_t* GGAFrame)
{
	uint8_t cursor = 0;
	uint8_t cursorComma = 0;
	uint8_t buffer[20] = { 0u };
	uint8_t cursorDecoderFunction = 0;
	uint8_t finished = 0;
	uint8_t l_u8CRCOk = 0u;
   
   
	l_u8CRCOk = GPS_Checksum(payload, size);
   
   if(l_u8CRCOk == 1u)
   {
      // Avoid Message ID
      while(payload[cursor++] != ',');
      
      while(cursor + cursorComma < size && !finished)
      {
         cursorComma = 0;
         
         // Getting size of datas between ','
         while((payload[cursor + cursorComma] != ',') && (payload[cursor + cursorComma] != '*'))
         {
            ++cursorComma;
         }
         
         if(payload[cursor + cursorComma] == '*')
         {
            ++cursor; // To pass the '*'			
            memcpy(buffer, payload + cursor + cursorComma, 2);	
            
            finished = 1;
         }
         else
         {
            if(cursorComma != 0u)
            {
               memset(buffer, 0u, 20u);
               memcpy(buffer, payload + cursor, cursorComma);		
            
               if((*functions_decoder[cursorDecoderFunction]) != NULL)
               {
                  (*functions_decoder[cursorDecoderFunction])(buffer, GGAFrame, cursorComma);	// calling the good decoder 
               }
            
               cursor += cursorComma;
            }
            ++cursorDecoderFunction;	// To call the next GGA decoder function
         
            ++cursor; // To pass the comma
         }
      }
      
//      l_u8CRCOk = GPS_validateChecksum(payload, buffer);
   }
   
	return l_u8CRCOk;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Send AT command
 *
* @param  ATMsg 		: AT message to send
* @param  sizeMsg		: Size of AT message
* @param  AT_OK			: AT message expected
* @param  ATError		: AT message for error
* @param  timeOut		: timeOut of the function in µs
* @param  callback	:	function called to manage message

 * @return
 */
uint8_t u8GPS_decodeGGA(const uint8_t* payload, uint8_t size, GGA_t * p_psGGA)
{
	//GGA_t GGAFrame = { 0 }; /* Always initialized variable ! RTFCSD */
   uint8_t l_u8CRCOk = 0u;
   
   if((p_psGGA != NULL) && (payload != NULL) && (size != 0u))
   {
      l_u8CRCOk = GPS_splitGGA(payload, size, p_psGGA);
	}
   
	return l_u8CRCOk;
}

s_Geodetic_t GPS_DecodeGeodetic(uint8_t* p_pu8Payload)
{
   uint8_t l_au8StartnPayload[5u] = { 0xA0, 0xA2, 0x00, 0x5B, 0x29 };
   uint8_t l_u8Idx = 0u;
   uint8_t l_u8StartOk = 1u;
   s_Geodetic_t l_sGeodetic;
   
   do
   {
      if(p_pu8Payload[l_u8Idx] != l_au8StartnPayload[l_u8Idx])
      {
         l_u8StartOk = 0u;
      }      
      l_u8Idx++;
   }while( (l_u8Idx < 5u) && (l_u8StartOk == 1u) );
  
   if(l_u8StartOk == 1u)
   {
     // memcpy(&l_sGeodetic, &p_pu8Payload[5u], sizeof(s_Geodetic_t));
      l_sGeodetic.u16NavValid = U8_TO_U16(p_pu8Payload[5u],p_pu8Payload[6u]);
      l_sGeodetic.u16NavType = U8_TO_U16(p_pu8Payload[7u],p_pu8Payload[8u]);
      l_sGeodetic.u16ExtWeekNb = U8_TO_U16(p_pu8Payload[9u],p_pu8Payload[10u]);
      l_sGeodetic.u32TOW = U8_TO_U32(p_pu8Payload[11u],p_pu8Payload[12u],p_pu8Payload[13u],p_pu8Payload[14u]);
      l_sGeodetic.u16Year = U8_TO_U16(p_pu8Payload[15u],p_pu8Payload[16u]);
      l_sGeodetic.u8Month = p_pu8Payload[17u];
      l_sGeodetic.u8Day = p_pu8Payload[18u];
      l_sGeodetic.u8Hour = p_pu8Payload[19u];
      l_sGeodetic.u8Minute = p_pu8Payload[20u];
      l_sGeodetic.u8Second = U8_TO_U16(p_pu8Payload[22u],p_pu8Payload[21u]);
      l_sGeodetic.u32SatelliteIDList = (uint32_t)U8_TO_U32(p_pu8Payload[26u],p_pu8Payload[25u],p_pu8Payload[24u],p_pu8Payload[23u]);
      l_sGeodetic.s32Latitude = (int32_t)U8_TO_U32(p_pu8Payload[30u],p_pu8Payload[29u],p_pu8Payload[28u],p_pu8Payload[27u]);
      l_sGeodetic.s32Longitude = (int32_t)U8_TO_U32(p_pu8Payload[38u],p_pu8Payload[37u],p_pu8Payload[36u],p_pu8Payload[35u]);
      l_sGeodetic.s32AltEllipsoid = (int32_t)U8_TO_U32(p_pu8Payload[42u],p_pu8Payload[41u],p_pu8Payload[40u],p_pu8Payload[39u]);
      l_sGeodetic.s32AltMSL = (int32_t)U8_TO_U32(p_pu8Payload[47u],p_pu8Payload[46u],p_pu8Payload[45u],p_pu8Payload[44u]);
      
      l_sGeodetic.u16SpeedOverGround = U8_TO_U16(p_pu8Payload[49u],p_pu8Payload[48u]);
      l_sGeodetic.u16CourseOverGround = U8_TO_U16(p_pu8Payload[51u],p_pu8Payload[50u]);
      
      l_sGeodetic.s16ClimbRate = (int16_t)U8_TO_U16(p_pu8Payload[55u],p_pu8Payload[54u]);
      l_sGeodetic.s16HeadingRate = (int16_t)U8_TO_U16(p_pu8Payload[57u],p_pu8Payload[56u]);
      l_sGeodetic.u32EHPE = U8_TO_U32(p_pu8Payload[61u],p_pu8Payload[60u],p_pu8Payload[59u],p_pu8Payload[58u]);
      l_sGeodetic.u32EVPE = U8_TO_U32(p_pu8Payload[65u],p_pu8Payload[64u],p_pu8Payload[63u],p_pu8Payload[62u]);
      l_sGeodetic.u32ETE = U8_TO_U32(p_pu8Payload[69u],p_pu8Payload[68u],p_pu8Payload[67u],p_pu8Payload[66u]);
      l_sGeodetic.u32EHVE = U8_TO_U32(p_pu8Payload[73u],p_pu8Payload[72u],p_pu8Payload[71u],p_pu8Payload[70u]);

   }
   else
   {
      memset(&l_sGeodetic, 0, sizeof(l_sGeodetic));
   }
      
   return l_sGeodetic;
}

