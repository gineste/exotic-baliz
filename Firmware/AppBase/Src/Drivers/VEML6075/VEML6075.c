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
 * Date:          28/10/2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of UV Sensor VEML6075. 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "HAL/HAL_I2C.h"

#include "GlobalDefs.h"

#include "VEML6075.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define VEML6075_ADDRESS         (uint8_t)(0x10)
#define VEML6075_DEVID           (uint8_t)(0x26)

#define VEML6075_REG_CONF        (uint8_t)(0x00) /* Configuration register (options below) */
#define VEML6075_REG_UVA         (uint8_t)(0x07) /* UVA register                           */
#define VEML6075_REG_DARK        (uint8_t)(0x08) /* Dark current register                  */
#define VEML6075_REG_UVB         (uint8_t)(0x09) /* UVB register                           */
#define VEML6075_REG_UVCOMP1     (uint8_t)(0x0A) /* Visible compensation register          */
#define VEML6075_REG_UVCOMP2     (uint8_t)(0x0B) /* IR compensation register               */
#define VEML6075_REG_DEVID       (uint8_t)(0x0C) /* Device ID register                     */

#define VEML6075_CONF_IT_50MS    (uint8_t)(0x00) /* Integration time = 50ms (default)      */
#define VEML6075_CONF_IT_100MS   (uint8_t)(0x10) /* Integration time = 100ms               */
#define VEML6075_CONF_IT_200MS   (uint8_t)(0x20) /* Integration time = 200ms               */
#define VEML6075_CONF_IT_400MS   (uint8_t)(0x30) /* Integration time = 400ms               */
#define VEML6075_CONF_IT_800MS   (uint8_t)(0x40) /* Integration time = 800ms               */

#define VEML6075_CONF_HD_NORM    (uint8_t)(0x00) /* Normal dynamic seetting (default)      */
#define VEML6075_CONF_HD_HIGH    (uint8_t)(0x08) /* High dynamic seetting                  */

#define VEML6075_CONF_TRIG_NO    (uint8_t)(0x00) /* Trigger disable */
#define VEML6075_CONF_TRIG_ONCE  (uint8_t)(0x04) /* Trigger measurement, clears by itself  */

#define VEML6075_CONF_AF_OFF     (uint8_t)(0x00) /* Active force mode disabled (default)   */
#define VEML6075_CONF_AF_ON      (uint8_t)(0x02) /* Active force mode enabled (?)          */

#define VEML6075_CONF_SD_OFF     (uint8_t)(0x00) /* Power up (ShutDown Off)                */
#define VEML6075_CONF_SD_ON      (uint8_t)(0x01) /* Power down (ShutDown On)               */

#define VEML6075_CONF_DEFAULT    (uint8_t)(  VEML6075_CONF_AF_OFF     | \
                                             VEML6075_CONF_TRIG_NO    | \
                                             VEML6075_CONF_HD_NORM    | \
                                             VEML6075_CONF_IT_800MS   )

/* To calculate the UV Index, a bunch of empirical/magical coefficients need to
   be applied to UVA and UVB readings to get a proper composite index value.
   Seems pretty hand wavey, though not nearly as annoying as the dark current
   not being subtracted out by default. */

#define VEML6075_UVI_UVA_VIS_COEFF  (float)(3.33f) //(2.22f)
#define VEML6075_UVI_UVA_IR_COEFF   (float)(2.5f)  //(1.33f)
#define VEML6075_UVI_UVB_VIS_COEFF  (float)(3.66f) //(2.95f)
#define VEML6075_UVI_UVB_IR_COEFF   (float)(2.75f) //(1.74f)

/* Once the above offsets and crunching is done, there's a last weighting
   function to convert the ADC counts into the UV index values. This handles
   both the conversion into irradiance (W/m^2) and the skin erythema weighting
   by wavelength--UVB is way more dangerous than UVA, due to shorter
   wavelengths and thus more energy per photon. These values convert the compensated values */

#define VEML6075_UVI_UVA_RESPONSE   (float)(0.001461f)//(1.0f / 909.0f)
#define VEML6075_UVI_UVB_RESPONSE   (float)(0.002591f)//(1.0f / 800.0f)

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef enum _VEML6075_IT_TIME_ 
{
  VEML6075_IT_50MS,
  VEML6075_IT_100MS,
  VEML6075_IT_200MS,
  VEML6075_IT_400MS,
  VEML6075_IT_800MS
}e_VEML6075_IntTime_t;

typedef struct _VEML6075_RAW_DATA_ 
{
   uint16_t u16RawUVA;
   uint16_t u16RawUVB;
   uint16_t u16RawDark;
   uint16_t u16RawVis;
   uint16_t u16RawIR;
}s_VEML6075_RawData_t;

typedef struct _VEML6075_COMPUTED_DATA_ 
{
   float fComputedVis;
   float fComputedIR;
   float fComputedUVA;
   float fComputedUVB;
   uint8_t u8ComputedUVIndex;
}s_VEML6075_ComputedData_t;

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static void vWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data);
static void vReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Data, uint8_t p_u8Size);
static void vUVAGet(void);
static void vUVBGet(void);
static void vDarkGet(void);
static void vVISGet(void);
static void vIRGet(void);
static uint8_t u8IDGet(void);
static uint8_t u8ConfigGet(void);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static uint8_t g_u8Initialized = 0u;
static uint8_t g_u8CommFailure = 0u;
static s_VEML6075_RawData_t g_sRawData = { 0u };
static s_VEML6075_ComputedData_t g_sComputedData = { 0.0f , 0.0f };
static uint8_t g_u8DeviceID = 0u;
static uint8_t g_u8Config = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**@brief Function to initialize VEML6075.
 * @return None.
 */
void vVEML6075_Init(void)
{   
   uint8_t l_au8ConfReg[2u] = { 0u };
   vWriteRegister(VEML6075_REG_CONF, VEML6075_CONF_DEFAULT | VEML6075_CONF_SD_OFF);
   
   if(u8IDGet() != VEML6075_DEVID)
   {
      g_u8Initialized = 0u;
   }
   else
   {
      g_u8Initialized = 1u;
      //vVEML6075_ShutDown();
      //vVEML6075_PowerUp();
      vReadRegister(VEML6075_REG_CONF, l_au8ConfReg, 2u);
   }   
}

/**@brief Function to WakeUp VEML6075.
 * @return None.
 */
void vVEML6075_PowerUp(void)
{
   vWriteRegister(VEML6075_REG_CONF, VEML6075_CONF_SD_OFF);
}

/**@brief Function to Read Device ID VEML6075.
 * @return None.
 */
void vVEML6075_IDRead(void)
{
   (void)u8IDGet();
}

/**@brief Function to ShutDown VEML6075.
 * @return None.
 */
void vVEML6075_ShutDown(void)
{   
   vWriteRegister(VEML6075_REG_CONF, VEML6075_CONF_SD_ON);
   g_u8Initialized = 0u;
}

/**@brief Function to Configure VEML6075.
 * @return None.
 */
void vVEML6075_Configure(void)
{
   uint8_t l_u8Config = VEML6075_CONF_DEFAULT | VEML6075_CONF_TRIG_ONCE | VEML6075_CONF_SD_OFF;

   vWriteRegister(VEML6075_REG_CONF, l_u8Config);
   
   if(u8ConfigGet() != l_u8Config)
   {
      (void)l_u8Config;
   }
}

/**@brief Function to Poll data from VEML6075.
 * @return None.
 */
void vVEML6075_PollingProcess(void)
{
   float l_fCompUV = 0.0f;
 
   if(g_u8Initialized == 1u)   
   {
      vUVAGet();
      vUVBGet();
      vDarkGet();
      vVISGet();
      vIRGet();
      
      g_sComputedData.fComputedVis = (float)((float)g_sRawData.u16RawVis - (float)g_sRawData.u16RawDark);
      g_sComputedData.fComputedIR = (float)((float)g_sRawData.u16RawIR - (float)g_sRawData.u16RawDark);
      
      l_fCompUV = (float)((float)g_sRawData.u16RawUVA - (float)g_sRawData.u16RawDark);
      g_sComputedData.fComputedUVA = l_fCompUV - ((VEML6075_UVI_UVA_VIS_COEFF * g_sComputedData.fComputedVis) 
                                               - (VEML6075_UVI_UVA_IR_COEFF * g_sComputedData.fComputedIR));
                                               
      l_fCompUV = (float)((float)g_sRawData.u16RawUVB - (float)g_sRawData.u16RawDark);
      g_sComputedData.fComputedUVB = l_fCompUV - ((VEML6075_UVI_UVB_VIS_COEFF * g_sComputedData.fComputedVis) 
                                               - (VEML6075_UVI_UVB_IR_COEFF * g_sComputedData.fComputedIR));
   }
}


/**@brief Function to check if VEML6075 is still available.
 * @return 1 if available, else 0.
 */
uint8_t u8VEML6075_IsAvailable(void)
{
   return ((g_u8Initialized == 1u) && (g_u8CommFailure == 0u))?1u:0u;
}

/**@brief Function to compute UV Index from VEML6075.
 * @param[out] p_pu8UVIndex : Compute UV Index from UVA and UVB
 * @return Error Code.
 */
e_VEML6075_ErrCode_t eVEML6075_UVIndexGet(uint8_t * p_pu8UVIndex)
{
   e_VEML6075_ErrCode_t l_eErrCode = VEML6075_ERROR_NONE;  
   
   if(g_u8Initialized == 1u)   
   {
      float l_fUVAWeighted = (float)(g_sComputedData.fComputedUVA * VEML6075_UVI_UVA_RESPONSE);
      float l_fUVBWeighted = (float)(g_sComputedData.fComputedUVB * VEML6075_UVI_UVB_RESPONSE);
      
      (*p_pu8UVIndex) = (uint8_t)((float)((l_fUVAWeighted + l_fUVBWeighted) / 2.0f));
      /* Add saturation check */
      if((*p_pu8UVIndex) > 12u)
      {
         (*p_pu8UVIndex) = 12u;
      }
      g_sComputedData.u8ComputedUVIndex = (*p_pu8UVIndex);
   }
   else
   {
      l_eErrCode = VEML6075_ERROR_INIT;
   }
   
   return l_eErrCode;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to write register of VEML6075.
 * @param[in] p_u8Register : Register to write
 * @param[in] p_u16Data : Data to write on register
 * @return None.
 */
static void vWriteRegister(uint8_t p_u8Register, uint8_t p_u8Data)
{
   uint8_t l_au8WrData[3u] = { 0u };
   
   if(p_u8Register == VEML6075_REG_CONF)
   {
      l_au8WrData[0u] = p_u8Register;
      l_au8WrData[1u] = p_u8Data;
      l_au8WrData[2u] = 0u;
      
      if(u32Hal_I2C_Write(VEML6075_ADDRESS, l_au8WrData, 2u) != 0u)
      {
         g_u8CommFailure = 1u;
      }
      else
      {
         g_u8CommFailure = 0u;
      }
   }
}
/**@brief Function to Read register of VEML6075.
 * @param[in] p_u8Register : Register to read
 * @param[out] p_pu8Data : Data read 
 * @param[in] p_u8Size : Size of data to read
 * @return None.
 */
static void vReadRegister(uint8_t p_u8Register, uint8_t * p_pu8Data, uint8_t p_u8Size)
{
   uint8_t l_au8Data[2u] = { 0u };
   l_au8Data[0u] = p_u8Register;
   
   if(u32Hal_I2C_WriteAndReadNoStop(VEML6075_ADDRESS, l_au8Data, 1u, p_pu8Data, p_u8Size) != 0u)
   {
      g_u8CommFailure = 1u;
   }
   else
   {
      g_u8CommFailure = 0u;
   }
}
/**@brief Function to Read UVA Raw Data of VEML6075.
 * @return None.
 */
static void vUVAGet(void)
{
   uint8_t l_au8Data[2u] = { 0u };
   
   vReadRegister(VEML6075_REG_UVA, l_au8Data, 2u);
   g_sRawData.u16RawUVA = U8_TO_U16(l_au8Data[1u],l_au8Data[0u]);
}
/**@brief Function to Read UVB Raw Data of VEML6075.
 * @return None.
 */
static void vUVBGet(void)
{
   uint8_t l_au8Data[2u] = { 0u };
   
   vReadRegister(VEML6075_REG_UVB, l_au8Data, 2u);   
   g_sRawData.u16RawUVB = U8_TO_U16(l_au8Data[1u],l_au8Data[0u]);
}
/**@brief Function to Read Visibility Raw Data of VEML6075.
 * @return None.
 */
static void vVISGet(void)
{
   uint8_t l_au8Data[2u] = { 0u };
   
   vReadRegister(VEML6075_REG_UVCOMP1, l_au8Data, 2u);   
   g_sRawData.u16RawVis = U8_TO_U16(l_au8Data[1u],l_au8Data[0u]);   
}
/**@brief Function to Read Darkness Raw Data of VEML6075.
 * @return None.
 */
static void vDarkGet(void)
{  
   uint8_t l_au8Data[2u] = { 0u };
   
   vReadRegister(VEML6075_REG_DARK, l_au8Data, 2u);   
   g_sRawData.u16RawDark = U8_TO_U16(l_au8Data[1u],l_au8Data[0u]);    
}
/**@brief Function to Read Infra Red Raw Data of VEML6075.
 * @return None.
 */
static void vIRGet(void)
{
   uint8_t l_au8Data[2u] = { 0u };
   
   vReadRegister(VEML6075_REG_UVCOMP2, l_au8Data, 2u);   
   g_sRawData.u16RawIR = U8_TO_U16(l_au8Data[1u],l_au8Data[0u]); 
}
/**@brief Function to Read Device ID of VEML6075.
 * @return ID of the Device (should be VEML6075_DEVID).
 */
static uint8_t u8IDGet(void)
{
   uint8_t l_au8Data[2u] = { 0u };
   vReadRegister(VEML6075_REG_DEVID,l_au8Data,2u);
   
   g_u8DeviceID = l_au8Data[0u]; 
   return g_u8DeviceID;
}

/**@brief Function to Read Configuration register of VEML6075.
 * @return Congifuration register.
 */
static uint8_t u8ConfigGet(void)
{
   uint8_t l_au8Data[2u] = { 0u };
   
   vReadRegister(VEML6075_REG_CONF, l_au8Data, 2u);
   
   g_u8Config = l_au8Data[0u]; 
   return g_u8Config;
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


