/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          14/12/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Tool Library. 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/

#define VOLTAGE_MV_100    (uint16_t)1400*3
#define VOLTAGE_MV_90     (uint16_t)1300*3
#define VOLTAGE_MV_20     (uint16_t)1200*3
#define VOLTAGE_MV_10     (uint16_t)1150*3
#define VOLTAGE_MV_0      (uint16_t)850*3

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static uint32_t u32Quick_Pow10(uint8_t p_u8n);
   
/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/ 
 
/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/** Transform a string into float
* @param p 		: the string to convert
* @param size : size of the string
* @return float converted
*/
float Tool_StringToFloat(uint8_t * p, uint8_t size)
{
  float tmpEnt = 0.0f, tmpDec = 0.0f, div = 1.0f;
	uint8_t cursor = 0u;
	float sign = 1.0f;
	float result = 0.0f;
	float result2 = 0.0f;
	float result3 = 0.0f;
	
	if(*p == '-')
	{
		sign = -1.0f;
		++p;
		++cursor;
	}
  
  while((*p != '.') && (*p != ',') && (*p != '*') && (cursor < size))
  {
    tmpEnt *= 10.0f;
    tmpEnt += (*p - '0');
    ++p;
		++cursor;
  }
  
  if(*p == '.')
  {
    ++p;
		++cursor;
    while((*p != ',') && (cursor < size))
    {
      tmpDec *= 10.0f;
      tmpDec += (*p - '0');
      div*=10.0f;
      ++p;
			++cursor;
    }
	}		
	
	result = tmpDec/div;
	result2 = result + tmpEnt;
	result3 = result2 * sign;
	
	return result3;
}

/** Transform a string into int
* @param p 		: the string to convert
* @param size : size of the string
* @return uint16_t converted
*/
uint16_t Tool_StringToInt(uint8_t * p, uint8_t size)
{
  uint16_t tmpEnt = 0;
	uint8_t cursor = 0;
  
  while(cursor < size)
  {
    tmpEnt *= 10;
    tmpEnt += (*p - '0');
    ++p;
		++cursor;
  }
  
  return tmpEnt;
}

/** Transform a byte into hexa number
* @param ascii 		: the ascii byte to convert into hexa
* @return the hexa number or 0 if there is an error
*/
uint8_t Tool_AsciiToHexa(uint8_t ascii)
{
	uint8_t result = 0;
	
	// 0 to 9
	if(ascii >= 48 && ascii <= 57)
	{
		result = ascii - '0';
	}
	// A to F
	else if(ascii >= 65 && ascii <= 70)
	{
		result = ascii - '0' - 7;
	}
	//a to f
	else if(ascii >= 97 && ascii <= 102)
	{
		result = ascii - '0' - 39;
	}
	
	return result;
}

float Tool_Abs(float a)
{
	if(a > 0)
	{
		return a;
	}
	
	return -a;
}


void vMantExpEncoder(uint32_t p_u32DataIn, uint8_t p_u8MantBits, uint8_t p_u8ExpBits, float p_fCoef, uint32_t * p_pu32Mantissa, uint32_t * p_pu32Exponant, uint32_t * p_pu32DataOut)
{
    uint32_t l_u32Threshold = 0u;
    uint8_t l_u8Exp = 0u;
    uint32_t l_u32Mant = 0u;
    uint32_t l_u32ExpMax = 0u;
    uint32_t l_u32MantMax = 0u;
    uint8_t l_u8Found = 0u;
    float l_fDenom = 1.0f;
   

   if( (p_u8MantBits != 0u) && (p_u8ExpBits != 0u) && (p_fCoef != 0.0f))
   {
      l_u32MantMax = (1u << p_u8MantBits) - 1u;
      l_u32ExpMax = (1u << p_u8ExpBits) - 1u ;
      
      do {
         l_u32Threshold = (uint32_t)(p_fCoef * (float)l_u32MantMax) * (float)u32Quick_Pow10(l_u8Exp);
         if(p_u32DataIn < l_u32Threshold)
         {
            l_u8Found = 1u;
         }
         else
         {
            l_u8Exp++;
         }
      }while((l_u8Exp < l_u32ExpMax) && (l_u8Found == 0u));

      l_fDenom = (float)(p_fCoef * (float)u32Quick_Pow10(l_u8Exp));

      l_u32Mant = (uint32_t)(((float)p_u32DataIn + (float)(l_fDenom / 2.0f))/ (l_fDenom));
      l_u32Mant = (l_u32Mant > l_u32MantMax)?l_u32MantMax:l_u32Mant;

      (*p_pu32Exponant) = l_u8Exp;
      (*p_pu32Mantissa) = l_u32Mant;
      
      /* Exponent MSB Mantissa LSB */
//      (*p_pu32DataOut) = (l_u8Exp & l_u32ExpMax) << p_u8MantBits;
//      (*p_pu32DataOut) |= (l_u32Mant & l_u32MantMax);
      
      /* Exponent LSB Mantissa MSB */
      (*p_pu32DataOut) = (l_u32Mant & l_u32MantMax) << p_u8ExpBits;
      (*p_pu32DataOut) |= (l_u8Exp & l_u32ExpMax);
   }
}

void vMantExpDecoder(uint32_t * p_pu32DataOut, uint16_t p_u16Mantissa, uint16_t p_u16Exponant, float p_fCoef)
{
    (*p_pu32DataOut) = (uint32_t)((float)p_u16Mantissa * (float)u32Quick_Pow10(p_u16Exponant) * p_fCoef);
}

uint8_t u8BattPackToPercent(uint8_t p_u8PackNb, uint16_t p_u16VoltagemV)
{
   uint16_t l_u8GlobalVoltage = p_u16VoltagemV;
   uint16_t l_u16UnitVoltage = 0u;
   uint8_t l_u8BatteryPercent = 0xFF;
   
   l_u16UnitVoltage = l_u8GlobalVoltage;// * p_u8PackNb;
   
   
   if(l_u16UnitVoltage > VOLTAGE_MV_100)
   {
      l_u8BatteryPercent = 100u;
   }
   else if((l_u16UnitVoltage <= VOLTAGE_MV_100) && (l_u16UnitVoltage > VOLTAGE_MV_90))
   {
      l_u8BatteryPercent = 100u - ((VOLTAGE_MV_100 - l_u16UnitVoltage) * 10u / ((float)VOLTAGE_MV_100 - (float)VOLTAGE_MV_90));
   }
   else if((l_u16UnitVoltage <= VOLTAGE_MV_90) && (l_u16UnitVoltage > VOLTAGE_MV_20))
   {
      l_u8BatteryPercent = 90u - ((VOLTAGE_MV_90 - l_u16UnitVoltage) * 70u / ((float)VOLTAGE_MV_90 - (float)VOLTAGE_MV_20));
   }
   else if((l_u16UnitVoltage <= VOLTAGE_MV_20) && (l_u16UnitVoltage > VOLTAGE_MV_10))
   {
      l_u8BatteryPercent = 20u - ((VOLTAGE_MV_20 - l_u16UnitVoltage) * 10u / ((float)VOLTAGE_MV_20 - (float)VOLTAGE_MV_10));
   }
   else if((l_u16UnitVoltage <= VOLTAGE_MV_10) && (l_u16UnitVoltage > VOLTAGE_MV_0))
   {
      l_u8BatteryPercent = 10u - ((VOLTAGE_MV_10 - l_u16UnitVoltage) * 10u / ((float)VOLTAGE_MV_10 - (float)VOLTAGE_MV_0));
   }
   else
   {
      l_u8BatteryPercent = 0u;
   }
   
   return l_u8BatteryPercent;
   
}

void vTool_EncodeGPSPosition(double p_dLatitude, double p_dLongitude, char p_chLatNS, char p_chLonEW, uint8_t * p_pau8EncodedData, uint8_t * p_pu8Size)
{
   uint64_t l_u64EncodedLat;
   uint64_t l_u64EncodedLng;
   uint8_t l_u8LatSign = 0u;
   uint8_t l_u8LngSign = 0u;
   
   if(   (p_pau8EncodedData != NULL)
      && (p_pu8Size != NULL) )
   {
      if ( p_chLatNS == 'S' )
      {
         l_u8LatSign = 1u;
      }

      if ( p_chLonEW == 'W' )
      {
         l_u8LngSign = 1u;
      }
      
      l_u64EncodedLat = p_dLatitude * 10000000;
      l_u64EncodedLat -= 53;
      l_u64EncodedLat /= 108;
      l_u64EncodedLng = p_dLongitude * 10000000;
      l_u64EncodedLng -= 107;
      l_u64EncodedLng /= 215;

      p_pau8EncodedData[0u]   = (l_u8LngSign ? 0x80 : 0x00) + (l_u8LatSign ? 0x40 : 0x00);
      p_pau8EncodedData[0u]  |= ((l_u64EncodedLat & 0x7E0000) >> 17u);
      p_pau8EncodedData[1u]   = ((l_u64EncodedLat & 0x01FE00) >> 9u);
      p_pau8EncodedData[2u]   = ((l_u64EncodedLat & 0x0001FE) >> 1u);
      p_pau8EncodedData[3u]   = (l_u64EncodedLat & 0x000001) ? 0x80 : 0x00;
      p_pau8EncodedData[3u]  |= ((l_u64EncodedLng & 0x7F0000) >> 16u);
      p_pau8EncodedData[4u]   = ((l_u64EncodedLng & 0x00FF00) >> 8u);
      p_pau8EncodedData[5u]   = (l_u64EncodedLng & 0x0000FF);
      
      (*p_pu8Size) = 6u;
   }
}

void vTool_ASCIIConvert(uint8_t * p_pu8DataIn, uint8_t p_u8SizeIn, uint8_t * p_pu8DataOut, uint8_t * p_pu8SizeOut)
{
   const uint8_t l_achConvertToChar[16u] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
   uint8_t l_u8Idx = 0u;
   uint8_t l_au8DataOut[128u] = { 0u };
   uint8_t l_u8SizeOut = 0u;
   
   for(l_u8Idx = 0; l_u8Idx < p_u8SizeIn; l_u8Idx++ )
   {
      if(1u+(2u*l_u8Idx) < 128u)
      {
         l_au8DataOut[(2u*l_u8Idx)]     = l_achConvertToChar[(((*p_pu8DataIn)>>4u) & 0x0F)];   // MSB
         l_au8DataOut[1u+(2u*l_u8Idx)]  = l_achConvertToChar[( (*p_pu8DataIn++)    & 0x0F)];   // LSB
      }
   }
   
   l_u8SizeOut = 2u * l_u8Idx;
   (*p_pu8SizeOut) = l_u8SizeOut;
   
   memcpy(p_pu8DataOut, l_au8DataOut, (*p_pu8SizeOut));
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static uint32_t u32Quick_Pow10(uint8_t p_u8n)
{
    uint32_t l_u32Ret = 0u;

    static uint32_t l_u32Pow10[6u] = {
        1u, 10u, 100u, 1000u, 10000u,
        100000u};

    if(p_u8n < 6u)
    {
        l_u32Ret = l_u32Pow10[p_u8n];
    }

    return l_u32Ret;
}


/****************************************************************************************
 * End Of File
 ****************************************************************************************/

