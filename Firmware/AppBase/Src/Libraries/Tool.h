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
 */
#ifndef TOOL_H
#define TOOL_H
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
 
/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
float Tool_StringToFloat(uint8_t * p, uint8_t size);
uint16_t Tool_StringToInt(uint8_t * p, uint8_t size);

uint8_t Tool_AsciiToHexa(uint8_t ascii);

float Tool_Abs(float a);


void vMantExpEncoder(uint32_t p_u32DataIn, uint8_t p_u8MantBits, uint8_t p_u8ExpBits, float p_fCoef, uint32_t * p_pu32Mantissa, uint32_t * p_pu32Exponant, uint32_t * p_pu32DataOut);
void vMantExpDecoder(uint32_t * p_pu32DataOut, uint16_t p_u16Mantissa, uint16_t p_u16Exponant, float p_fCoef);

uint8_t u8BattPackToPercent(uint8_t p_u8PackNb, uint16_t p_u16VoltagemV);

void vTool_EncodeGPSPosition(double p_dLatitude, double p_dLongitude, char p_chLatNS, char p_chLonEW, uint8_t * p_pau8EncodedData, uint8_t * p_pu8Size);
void vTool_ASCIIConvert(uint8_t * p_pu8DataIn, uint8_t p_u8SizeIn, uint8_t * p_pu8DataOut, uint8_t * p_pu8SizeOut);

#endif /* TOOL_H */
