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
 * Date:          23/01/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Description of your source file 
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>
#include <nrf.h>
#include <nrf52_bitfields.h>
#include <nrf_sdh.h>
#include <nrf_sdm.h>
#include "app_error.h"

#include "BoardConfig.h"

#include "HAL/HAL_GPIO.h"
#include "HAL/HAL_I2C.h"
#include "HAL/HAL_SPI.h"
#include "HAL/HAL_RTC.h"
#include "HAL/HAL_Timer.h"

#include "BLE/BLE_Application.h"

#include "ADXL362/ADXL362.h"
#include "BME280/BME280.h"
#include "ORG1510/ORG1510.h"
#include "MAX44009/MAX44009.h"
#include "LSM6DSL/LSM6DSL.h"
#include "LIS2MDL/LIS2MDL.h"
#include "VEML6075/VEML6075.h"
#include "ST25DV/ST25DV.h"
#include "LTC2943/LTC2943.h"

#include <nrf_delay.h>
#include "Libraries/SimpleLED.h"
#include "Libraries/Buzzer.h"
#include "Libraries/FlashMemory.h"
#include "Libraries/NMEA.h"
#include "SigFox.h"
#include "SEGGER_RTT.h"
#include "UartManagement.h"
#include "radio_config.h"

#include "GlobalDefs.h"

#include "MainStateMachine.h"
/* Self include */
#include "HardwareTest.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define DELAY_LED_SWITCH            500u
#define DELAY_BUZ_TIMEOUT           600u
#define DELAY_SENSORS_READ          500u

#define BLE_RADIO_TXPOWER           RADIO_TXPOWER_TXPOWER_Pos4dBm
#define BLE_RADIO_MODE              RADIO_MODE_MODE_Nrf_2Mbit

#define TIMER_TIMEOUT_MS            500u
#define TIMEOUT_SIGFOX_INIT         20000u // AT_TIMEOUT_MIN * 2
#define RTC_TIMER_TIMEOUT_MS        1u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef enum _HARDWARE_TEST_CMD_ {
   HT_CMD_BLE = 0u,
   HT_CMD_SWP,
   HT_CMD_SPI,
   HT_CMD_CSF,
   HT_CMD_SFC,
   HT_CMD_GPS,
   HT_CMD_RTC,
   HT_CMD_RAX,
   HT_CMD_ISS,
   HT_CMD_RSS,
   HT_CMD_LED,
   HT_CMD_BUZ,
   HT_CMD_NFC,
   HT_CMD_LPM,
   HT_CMD_HLP,
   HT_CMD_RST,
   HT_CMD_LAST,
}e_HT_Commands_t;

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_HT_Commands_t eCommandParser(uint8_t * p_au8CmdStr, uint8_t p_u8Size);
static void vHT_NewTestProcess(e_HT_Commands_t p_eCmd);

static void vStartLEDTest(void);
static void vStartBuzzerTest(void);
static void vStartSPITest(void);
static void vStartADXLTest(void);
static void vStartI2CSensorsInitTest(void);
static void vStartI2CSensorsReadTest(void);
static void vStartGPSTest(void);
static void vStopGPSTest(void);
static void vStartSigFoxInitTest(void);
static void vStartSigFoxCWTest(void);
static void vStopSigFoxCWTest(void);
static void vStartRadioBLETest(void);
static void vStopRadioBLETest(void);
static void vSweepRadioBLEChTest(void);
static void vStartRTCTest(void);
static void vStopRTCTest(void);
static void vLPMTest(void);
   
static void vHTTimeOutHandler(void * p_pvContext);
static void vHTRTCHandler(void * p_pvContext);

static void vGPSInit(void);
static void vBME_SingleShotRead(void);
   
/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
#if (EN_BME280 == 1)
   static s_BME280_Context_t g_sBMEContext = {
      .eInterface = BME280_I2C_ITF,
      /*! When eInterface is BME280_I2C_ITF */
      .sI2CCfg = {         
         .eI2CAddr = BME280_I2C_ADDR_GND,
         .fp_u32I2C_Write = &u32Hal_I2C_Write,
         .fp_u32I2C_Read = &u32Hal_I2C_WriteAndRead,
      },
      /*! Delay function pointer */
      .fp_vTimerDelay_ms = &vHal_Timer_DelayMs,   
   };
#endif /* EN_BME280 */
   
#if (EN_ADXL362 == 1)
   static s_ADXL362_Context_t g_sADXLContext = {
      .fp_u32SPITransfer = &u32Hal_SPI_Transfer,   /* Function pointer to a SPI transfer */
      .fp_vDelay_ms = &nrf_delay_ms,               /* Function pointer to a timer in ms */
      .fp_Int1Handler = NULL,                      ///*&vDataUpdate_InterruptADXL362*//* Interrupt Handler for Activity and Inactivity */
      .fp_Int2Handler = NULL,                      ///*&vDataUpdate_InterruptADXL362*//* Interrupt Handler for Activity and Inactivity */
      
      .eRange = ADXL362_RANGE_2G,                  /* Range of accelerometer */
      .eOutputDataRate = ADXL362_ODR_100_HZ,       /* Output Data Rate */
      .eLinkLoopMode = ADXL362_MODE_LOOP,          /* Functioning mode (Link/Loop) */
      .eNoiseCtrl = ADXL362_NOISE_MODE_NORMAL,     /* Noise control (Normal, Low, Ultra Low) */
      
      .u8ActivityDetection = 1u,			/* 0 - no activity detection, 1 - activity detection */
      .u8RefOrAbsActivity = 1u, 			/* 0 - absolute mode, 1 - referenced mode. */
      .u16ThresholdActivity = 0x280,   /* 11-bit unsigned value that the adxl362 samples are compared to. */
      .u16TimeActivity = 250u,			/* 16-bit value activity time in ms (from 2.5ms @ odr = 400Hz to 20s @ odr = 12.5Hz) */
      
      .u8InactivityDetection = 1u,		/* 0 - no activity detection, 1 - activity detection */
      .u8RefOrAbsInactivity = 1u,		/* 0 - absolute mode, 1 - referenced mode. */
      .u16ThresholdInactivity = 0x3FF, /* 11-bit unsigned value that the ADXL362 samples are compared to. */
      .u32TimeInactivity = 5000u,  		/* 16-bit value inactivity time in ms (from 164s @ odr = 400Hz to 87min @ odr = 12.5Hz) */
                                        
      .eInt1Map = ADXL362_INTMAPX_AWAKE,  /* Type of the interrupt n°1 */
      .eInt2Map = ADXL362_INTMAPX_OFF,    /* Type of the interrupt n°2 */
         
      .eWakeUpMode = ADXL362_WAKEUP_OFF,  /* Wake-up mode (Let it OFF if you adjustable ODR for (In)Activity detection) */
      .eMeasureMode = ADXL362_MEASURE_ON  /* Power mode (Standby or Measurement Mode) */
   };
#endif /* NO_ADXL362 */

#if (EN_MAX44009 == 1)
   static s_MAX44009_Context_t g_sMAXContext = {
      .eI2CAddress = MAX44009_ADDR1,               /* Sensor Address */
      .fp_u32I2C_Read = &u32Hal_I2C_WriteAndRead,  /* Function pointer to a read I2C transfer */
      .fp_u32I2C_Write = &u32Hal_I2C_Write,        /* Function pointer to a write I2C transfer */
      .fp_vDelay_ms = &nrf_delay_ms,               /* Function pointer to a timer in ms */
   };
#endif

#if (EN_LSM6DSL == 1)
   static s_LSM6DSL_Context_t g_sLSM6Context = {
   .eInterface = LSM6DSL_ITF_I2C,
   .sI2CCfg = {
      .eI2CAddress = LSM6DSL_I2C_ADDR_VCC,
      /* Function pointer to a read I2C transfer */
      .fp_u32I2C_Write = &u32Hal_I2C_Write,
      /* Function pointer to a write I2C transfer */
      .fp_u32I2C_Read = &u32Hal_I2C_WriteAndReadNoStop,
   },
   /* Function pointer to a timer in ms */
   .fp_vDelay_ms = &nrf_delay_ms,
};
#endif

#if (EN_LIS2MDL == 1)
   static s_LIS2MDL_Context_t g_sLIS2Context = {
   .eCommunicationUsed = LIS2MDL_COMM_I2C,
   .sI2CCfg = {
      /* Function pointer to a read I2C transfer */
      .fp_u32I2C_Write = &u32Hal_I2C_Write,
      /* Function pointer to a write I2C transfer */
      .fp_u32I2C_Read = &u32Hal_I2C_WriteAndReadNoStop,
   },
   /* Function pointer to a timer in ms */
   .fp_vDelay_ms = &nrf_delay_ms,
};
#endif

#if (EN_ORG1510 == 1)
   static s_ORG1510_Context_t g_sORG1510Context = {
      .fp_vTimerDelay_ms_t = &nrf_delay_ms,
      .fp_u8UART_Read_t = &u8Hal_UART_Read,
      .fp_u32UART_Write_t = &u32Hal_UART_Write,
      .fp_vGPIO_Set_t = &vHal_GPIO_Set,
      .fp_vGPIO_Clear_t = &vHal_GPIO_Clear,
      .u32IO_ForceON = GPS_ON,
      .u32IO_RESET = GPS_RST
};
#endif

#if (EN_ST25DV == 1)
static s_ST25DV_Context_t g_sST25DVContext = {
      /* Function pointer for a read I2C transfer */
      .fp_u32I2C_Write = &u32Hal_I2C_Write,
      /* Function pointer for a write I2C transfer */
      .fp_u32I2C_Read = &u32Hal_I2C_WriteAndReadNoStop,
      /* Function pointer to a timer in ms */
      .fp_vDelay_ms = &nrf_delay_ms,
      /* Config */
      .eEepromSize = ST25DV_EEPROM_SIZE_64K,
   };
#endif
#if (EN_LTC2943 == 1)   
   static s_LTC2943_Context_t g_sLTCContext = {
      /* Function pointer to a read I2C transfer */
      .fp_u32I2C_Write = &u32Hal_I2C_Write,
      /* Function pointer to a write I2C transfer */
      .fp_u32I2C_Read = &u32Hal_I2C_WriteAndReadNoStop,
      /* Function pointer to a timer in ms */
      .fp_vDelay_ms = &nrf_delay_ms,
   };
#endif
   
static uint8_t g_u8StopTest = 0u;
static uint8_t g_u8TestInProgress = 0u;
static uint8_t g_u8I2CInitSensors = 0u;
static uint8_t g_u8RTCCheck = 0u;   
   
static uint8_t g_u8BLERadioInit = 0u;
static uint8_t g_u8BLERadioChannel = 0u;
static uint8_t g_u8BLERadioChannelSweep = 0u;

static uint8_t g_u8SigFoxCWTest = 0u;

HAL_TIMER_DEF(g_TimeOutTestIdx);
HAL_TIMER_DEF(g_RTCTestIdx);

/* Must be in the same order of e_HT_Commands_t */
const char g_cachCmd[HT_CMD_LAST][CMD_FRAME_SIZE+1u] = {
   "BLE\n\0",
   "SWP\n\0",
   "SPI\n\0",
   "CSF\n\0",
   "SFC\n\0",
   "GPS\n\0",
   "RTC\n\0",
   "RAX\n\0",
   "ISS\n\0",
   "RSS\n\0",
   "LED\n\0",
   "BUZ\n\0",
   "NFC\n\0",
   "LPM\n\0",
   "HLP\n\0",
   "RST\n\0",
};

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vHT_PrintHelp(void)
{
   PRINT_FAST("\n**********************************\n");
   PRINT_FAST("*              Help              *\n");
   PRINT_FAST("**********************************\n");
   PRINT_FAST("Commands start with $CHK,XXX e.g. : $CHK,BLE\\n \n");
   PRINT_FAST("Acknowledges start with $ACK,XXX+Y i.e. : $ACK,BLE+0\\n \n");
   PRINT_FAST("Results start with $RSL,XXX+Y(+Z..Z) i.e. : $RSL,ISS+1+BME280\\n \n");
   PRINT_FAST("\nCommands List :\n");
   PRINT_FAST("BLE: Start/Stop Radio BLE(Scope)\n");
   PRINT_FAST("SWP: Sweep BLE Radio Channel(Scope)\n");
   PRINT_FAST("CSF: Check Comm UART SigFox (Blocking Test)\n");
   PRINT_FAST("SFC: Check Radio Tx SigFox 868MHz 14dBm(Scope)\n");
   PRINT_FAST("GPS: Start/Stop GPS\n");
   PRINT_FAST("RTC: Start/Stop Timer RTC\n");
   PRINT_FAST("SPI: Check SPI ADXL362\n");
   PRINT_FAST("RAX: Read SPI ADXL362\n");
   PRINT_FAST("ISS: Init All I2C Sensors\n");
   PRINT_FAST("RSS: Read All I2C Sensors\n");   
   PRINT_FAST("LED: Check LED (Visual)\n");
   PRINT_FAST("BUZ: Check Buzzer (Audio)\n");
   PRINT_FAST("NFC: Write Data on \n");
   PRINT_FAST("LPM: Lowest Power Mode Set\n");
   PRINT_FAST("HLP: Print Help\n");
   PRINT_FAST("RST: RESET\n");
}

void vHT_Init(void)
{
   (void)eHal_Timer_Create(&g_TimeOutTestIdx, HAL_TIMER_MODE_SINGLE_SHOT, &vHTTimeOutHandler);
   (void)eHal_Timer_Create(&g_RTCTestIdx, HAL_TIMER_MODE_REPEATED, &vHTRTCHandler);   
}

void vHT_CheckInput(uint8_t * p_au8Frame, uint8_t p_u8Size)
{
   uint8_t l_u8GoodStartSentence = 0u;
   e_HT_Commands_t l_eCmd = HT_CMD_LAST;
   
   if(   (p_u8Size == FRAME_SIZE_MAX)
      && (p_au8Frame[0u] == '$')
      && (p_au8Frame[1u] == 'C')
      && (p_au8Frame[2u] == 'H')
      && (p_au8Frame[3u] == 'K')
      && (p_au8Frame[4u] == ',') )
   {
      l_u8GoodStartSentence = 1u;
   }
   
   if(l_u8GoodStartSentence == 1u)
   {
      l_eCmd = eCommandParser(&p_au8Frame[5u], CMD_FRAME_SIZE);
      vHT_NewTestProcess(l_eCmd);
   }
}



/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static e_HT_Commands_t eCommandParser(uint8_t * p_au8CmdStr, uint8_t p_u8Size)
{
   e_HT_Commands_t l_eCmd = HT_CMD_LAST;
   uint8_t l_u8Idx = 0u;
   char l_achCmd[CMD_FRAME_SIZE+1u] = { '\0' };
   
   memcpy(l_achCmd, (char*)p_au8CmdStr, p_u8Size);
   
   for(l_u8Idx = 0u; l_u8Idx < (uint8_t)HT_CMD_LAST;l_u8Idx++)
   {
      if(strcmp(l_achCmd, g_cachCmd[l_u8Idx]) == 0u)
      {
         l_eCmd = (e_HT_Commands_t)l_u8Idx;
         return l_eCmd;
      }
   }
   
   return l_eCmd;
}
static void vHT_NewTestProcess(e_HT_Commands_t p_eCmd)
{
   switch(p_eCmd)
   {
      case HT_CMD_BLE:
         PRINT_FAST("$ACK,BLE+1\n");
         (g_u8BLERadioInit == 0u) ? vStartRadioBLETest():vStopRadioBLETest();               
         break;
      case HT_CMD_SWP:
         PRINT_FAST("$ACK,SWP+1\n");
         vSweepRadioBLEChTest();
         break;
      case HT_CMD_CSF:
         PRINT_FAST("$ACK,CSF+1\n");
         vStartSigFoxInitTest();
         break;
      case HT_CMD_SFC:
         PRINT_FAST("$ACK,SFC+1\n");
         (g_u8SigFoxCWTest == 0u) ? vStartSigFoxCWTest():vStopSigFoxCWTest();
         break;
      case HT_CMD_GPS:
         PRINT_FAST("$ACK,GPS+1\n");          
         (g_u8TestInProgress == 0u) ? vStartGPSTest():vStopGPSTest();     
         break;
      case HT_CMD_RTC:
         PRINT_FAST("$ACK,RTC+1\n");
         (g_u8RTCCheck == 0u) ? vStartRTCTest():vStopRTCTest();         
         break;
      case HT_CMD_SPI:
         PRINT_FAST("$ACK,SPI+1\n");
         vStartSPITest();
         break;
      case HT_CMD_RAX:
         PRINT_FAST("$ACK,RAX+1\n");
         vStartADXLTest();
         break;
      case HT_CMD_ISS:
         PRINT_FAST("$ACK,ISS+1\n");
         vStartI2CSensorsInitTest();
         break;
      case HT_CMD_RSS:
         PRINT_FAST("$ACK,RSS+1\n");
         vStartI2CSensorsReadTest();
         break;
      case HT_CMD_LED:
         PRINT_FAST("$ACK,LED+1\n");
         vStartLEDTest();
         break;
      case HT_CMD_BUZ:
         PRINT_FAST("$ACK,BUZ+1\n");
         vStartBuzzerTest();
         break;
      case HT_CMD_NFC:
         PRINT_FAST("$ACK,NFC+1\n");
         break;
      case HT_CMD_LPM:
         PRINT_FAST("$ACK,LPM+1\n");
         vLPMTest();
         break;
      case HT_CMD_HLP:
         PRINT_FAST("$ACK,HLP+1\n");
         vHT_PrintHelp();
         break;
      case HT_CMD_RST:
         PRINT_FAST("$ACK,RST+1\n");     
         NVIC_SystemReset();   
         break;
      default:
         PRINT_FAST("$ACK,UKN+0\n");
         break;
   }
}

static void vStartLEDTest(void)
{
   uint8_t l_u8Idx = 0u;
   for(l_u8Idx = 0u; l_u8Idx < LED_DEFAULTCOLOR; l_u8Idx++)
   {
      vSimpleLED_ColorSet((e_SimpleLED_Color_t)l_u8Idx);
      nrf_delay_ms(DELAY_LED_SWITCH);      
   }
   PRINT_FAST("$RSL,LED+1\n");
}

static void vStartBuzzerTest(void)
{
   vBuzzerSeqCmd();
   PRINT_FAST("$RSL,BUZ+1\n");
}


static void vStartSPITest(void)
{
   uint8_t l_u8PartID = 0u;
   
//   vHal_GPIO_Clear(ADXL_POWER_EN);
//   nrf_delay_ms(500u);
   vHal_GPIO_Set(ADXL_POWER_EN);
   nrf_delay_ms(10u);
   /* Since it's the first process we run, there is no entry point for PowerUp SM */
   /* Module : SPI, I2C, etc. */
   vHal_I2C_Init();
   s_HalSpi_Context_t l_sSPIContext = {
   .u32MOSIPin = SPI_MOSI,
   .u32MISOPin = SPI_MISO,
   .u32ClockPin = SPI_SCLK,
   .u32ChipSelectPin = ADXL_CS,
   .eMode = HALSPI_MODE_0,
   .eFrequency = HALSPI_FREQ_1M,
};
   
   vHal_SPI_ContextSet(l_sSPIContext);
   vHal_SPI_Init();

   vADXL362_ContextSet(g_sADXLContext);
   if(eADXL362_SoftReset() != ADXL362_ERROR_NONE)
   {
      PRINT_FAST("$RSL,SPI+0\n");
   }
   nrf_delay_ms(1u);

   if(eADXL362_PartIDGet(&l_u8PartID) != ADXL362_ERROR_NONE)
   {
      PRINT_FAST("$RSL,SPI+0\n");
   }
   else
   {
      PRINT_FAST("$RSL,SPI+1\n");
   };   
}
static void vStartADXLTest(void)
{
#if (EN_ADXL362 == 1)
   int16_t l_s16Data[3u] = { 0 };
   vADXL362_ContextSet(g_sADXLContext); 
   
   if(u8ADXL362_IsAvailable() == 0u)
   {
      if(eADXL362_Init() != ADXL362_ERROR_NONE)
      {
         PRINT_FAST("$RSL,RAX+0\n");
      }
   }
   
   if(u8ADXL362_IsAvailable() == 1u)
   {
      if(eADXL362_AccelerationRead() != ADXL362_ERROR_NONE)
      {
         PRINT_FAST("$RSL,RAX+0\n");
      }
      else
      {
         vADXL362_AccelerationGet(&l_s16Data[0u],&l_s16Data[1u],&l_s16Data[2u]);
         PRINT_DEBUG("X : %d, ",l_s16Data[0u]);
         PRINT_DEBUG("Y : %d, ",l_s16Data[1u]);
         PRINT_DEBUG("Z : %d\n",l_s16Data[2u]);
         PRINT_FAST("$RSL,RAX+1\n");
      }
   }
   
#endif
}

static void vStartI2CSensorsInitTest(void)
{   
   uint8_t l_u8ChipID = 0u;
   uint8_t l_u8Error = 1u;
   
   vHal_I2C_Init();
   
#if (EN_BME280 == 1) 
   if(eBME280_ContextSet(g_sBMEContext) == BME280_ERROR_NONE)
   {
      if(eBME280_ChipIDGet(&l_u8ChipID) == BME280_ERROR_NONE)
      {
         l_u8Error = (l_u8ChipID == BME280_CHIP_ID)? 0u : 1u;
      }  
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+BME280\n");
   }
   else
   {
      PRINT_FAST("$RSL,ISS+1+BME280\n");
   }
   
#endif
#if (EN_MAX44009 == 1)  
   l_u8Error = 1u;
   // NOTE : No part ID available for MAX44009 just try to write something on it
   if(eMAX44009_ContextSet(g_sMAXContext) == MAX44009_ERROR_NONE)
   { 
      if(eMAX44009_ConversionModeSet(MAX44009_DEFAULT,MAX44009_AUTOMATIC, 
                                    MAX44009_INT_TIME_800MS,MAX44009_HIGH_BRIGHTNESS) == MAX44009_ERROR_NONE)
      {
         l_u8Error = 0u;
      }
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+MAX44009\n");
   }
   else
   {
      PRINT_FAST("$RSL,ISS+1+MAX44009\n");
   }
#endif
#if (EN_LSM6DSL == 1)
   l_u8Error = 1u;
   l_u8ChipID = 0u;
   if(eLSM6DSL_ContextSet(g_sLSM6Context) == LSM6DSL_ERROR_NONE)
   {
      if(eLSM6DSL_WhoAmIGet(&l_u8ChipID) == LSM6DSL_ERROR_NONE)
      {
         l_u8Error = (l_u8ChipID == LSM6DSL_WHO_I_AM_ID)? 0u : 1u;
      }
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+LSM6DSL\n");
   }
   else
   {
      PRINT_FAST("$RSL,ISS+1+LSM6DSL\n");
   }
#endif
#if (EN_LIS2MDL == 1)
   l_u8Error = 1u;
   l_u8ChipID = 0u;
   if(eLIS2MDL_ContextSet(g_sLIS2Context) == LIS2MDL_ERROR_NONE)
   {
      if(eLIS2MDL_WhoAmIGet(&l_u8ChipID) == LIS2MDL_ERROR_NONE)
      {
         eLIS2MDL_LowPower(1u);
         l_u8Error = (l_u8ChipID == LIS2MDL_WHO_AM_I_ID)? 0u : 1u;
      }
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+LIS2MDL\n");
   }
   else
   {
      PRINT_FAST("$RSL,ISS+1+LIS2MDL\n");
   }
   
#endif

#if (EN_VEML6075 == 1)

#endif
   
#if (EN_ST25DV == 1)

#endif
   
#if (EN_LTC2943 == 1)
   l_u8Error = 1u;
   l_u8ChipID = 0u;
   if(eLTC2943_ContextSet(g_sLTCContext) == LTC2943_ERROR_NONE)
   {
      if(eLTC2943_StatusGet(&l_u8ChipID) == LTC2943_ERROR_NONE)
      {
         l_u8Error = 0u;
      }
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+LTC2943\n");
   }
   else
   {
      PRINT_FAST("$RSL,ISS+1+LTC2943\n");
   }
#endif
   
   g_u8I2CInitSensors = 1u;
}
static void vStartI2CSensorsReadTest(void)
{   
   uint8_t l_u8Error = 1u;
   int16_t l_s16X = 0;
   int16_t l_s16Y = 0;
   int16_t l_s16Z = 0;
   
   if(g_u8I2CInitSensors == 1u)
   { 
      #if (EN_BME280 == 1)
         vBME_SingleShotRead();
      #endif
      
      #if (EN_MAX44009 == 1)  
         l_u8Error = 1u;
         uint32_t l_u32Brightness = 0u;
         if(eMAX44009_BrightnessRead(NULL) == MAX44009_ERROR_NONE)
         {
            if(eMAX44009_BrightnessGet(&l_u32Brightness) == MAX44009_ERROR_NONE)
            {
               PRINT_DEBUG("L : %d Lux\n", l_u32Brightness);
               l_u8Error = 0u;
            }            
         }
         if(l_u8Error == 1u)
         {
            PRINT_FAST("$RSL,RSS+0+MAX44009\n");
         }
         else
         {
            PRINT_FAST("$RSL,RSS+1+MAX44009\n");
         }
      #endif
      
      #if (EN_LSM6DSL == 1)
         l_u8Error = 1u;    
         if(eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_833Hz, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER) == LSM6DSL_ERROR_NONE)
         {
            if(eLSM6DSL_AccelRead() == LSM6DSL_ERROR_NONE)
            {
               if(eLSM6DSL_AccelGet(&l_s16X, &l_s16Y, &l_s16Z) == LSM6DSL_ERROR_NONE)
               {
                  PRINT_DEBUG("Acc X : %d mG, ", l_s16X);
                  PRINT_DEBUG("Acc Y : %d mG, ", l_s16Y);
                  PRINT_DEBUG("Acc Z : %d mG\n", l_s16Z);
                  eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
                  l_u8Error = 0u;
               }
            } 
         }
         if(l_u8Error == 0u)
         {
            l_u8Error = 1u;
            if(eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_833Hz, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER) == LSM6DSL_ERROR_NONE)
            {
               if(eLSM6DSL_GyroRead() == LSM6DSL_ERROR_NONE)
               {
                  if(eLSM6DSL_GyroGet(&l_s16X, &l_s16Y, &l_s16Z) == LSM6DSL_ERROR_NONE)
                  {
                     PRINT_DEBUG("Gyr X : %d dps, ", l_s16X);
                     PRINT_DEBUG("Gyr Y : %d dps, ", l_s16Y);
                     PRINT_DEBUG("Gyr Z : %d dps\n", l_s16Z);
                     eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
                     l_u8Error = 0u;
                  }
               }
            }
         }
         
         if(l_u8Error == 1u)
         {
            PRINT_FAST("$RSL,RSS+0+LSM6DSL\n");
         }
         else
         {
            PRINT_FAST("$RSL,RSS+1+LSM6DSL\n");
         }
      #endif
      
      #if (EN_LIS2MDL == 1)
         l_u8Error = 1u;
//         if(eLIS2MDL_TemperatureRead() == LIS2MDL_ERROR_NONE)
//         {
//            if(eLIS2MDL_TempDataGet(&l_s16X) == LIS2MDL_ERROR_NONE)
//            {
//               PRINT_DEBUG("T : %d\n", l_s16X);
//            }
//         }
//         if(l_u8Error == 0u)
//         {
//            l_u8Error = 1u;
            if(eLIS2MDL_MagneticRead() == LIS2MDL_ERROR_NONE)
            {
               if(eLIS2MDL_MagDataGet(&l_s16X, &l_s16Y, &l_s16Z) == LIS2MDL_ERROR_NONE)
               {
                  PRINT_DEBUG("Mag X : %d G, ", l_s16X);
                  PRINT_DEBUG("Mag Y : %d G, ", l_s16Y);
                  PRINT_DEBUG("Mag Z : %d G\n", l_s16Z);
                  l_u8Error = 0u;
               }
            }
//         }
         if(l_u8Error == 1u)
         {
            PRINT_FAST("$RSL,RSS+0+LIS2MDL\n");
         }
         else
         {
            PRINT_FAST("$RSL,RSS+1+LIS2MDL\n");
         }
      #endif
      
      #if (EN_VEML6075 == 1)
      
      #endif
      
      #if (EN_ST25DV == 1)
      
      #endif
      
      #if (EN_LTC2943 == 1)
         l_u8Error = 0u;
         uint16_t l_u16Volt = 0u;
         
         eLTC2943_PowerDown(0u);
         if(eLTC2943_ADCModeSet(LTC2943_ADC_MODE_MANUAL) == LTC2943_ERROR_NONE)
         {
            if(eLTC2943_SensorRead(ADC_SENSOR_VOLTAGE_MASK | ADC_SENSOR_CURRENT_MASK) == LTC2943_ERROR_NONE)
            {
//               if(eLTC2943_TemperatureGet(&l_s16X) == LTC2943_ERROR_NONE)
//               {
//                  PRINT_DEBUG("T : %d\n", l_s16X);
//               }
//               else
//               {
//                  l_u8Error += 1u;
//               }
               
               if(eLTC2943_AccumulatedChargeGet(&l_s16Y) == LTC2943_ERROR_NONE)
               {
                  PRINT_DEBUG("Acc I : %d mA\n", l_s16Y);
               }
               else
               {
                  l_u8Error += 1u;
               }
                  
               if(eLTC2943_VoltageGet(&l_u16Volt) == LTC2943_ERROR_NONE)
               {
                  PRINT_DEBUG("V : %d mV\n", l_u16Volt);
               }
               else
               {
                  l_u8Error += 1u;
               }
            }
            else
            {
               l_u8Error += 1u;
            }         
         }
         else
         {
            l_u8Error += 1u;
         }
         if(l_u8Error != 0u)
         {
            PRINT_FAST("$RSL,RSS+0+LTC2943\n");
         }
         else
         {
            PRINT_FAST("$RSL,RSS+1+LTC2943\n");
         }
      #endif
   }
   else
   {
      PRINT_FAST("$RSL,RSS+0\n");
   }

//   #if (EN_VEML6075 == 1)   
//      uint8_t l_u8Index = 0xFFu;
//      PRINT_DEBUG("%s","VEML6075 Test: ");
//      vVEML6075_Configure();
//      vVEML6075_PollingProcess();
//      (void)eVEML6075_UVIndexGet(&l_u8Index);
//      PRINT_DEBUG("UV Index %d\n",l_u8Index);
//      nrf_delay_ms(DELAY_SENSORS_READ);
//   #endif

}

static void vStartGPSTest(void)
{   
#if (EN_ORG1510 == 1)
   vHal_GPIO_Set(GPS_POWER_EN);  
   vGPSInit();
   
   g_u8TestInProgress = 1u;
   //eUartMngt_StateSet(USM_GPS);
#endif
}
static void vStopGPSTest(void)
{
#if (EN_ORG1510 == 1)
   eUartMngt_StateSet(USM_IDLE);
   vHal_GPIO_Clear(GPS_POWER_EN);
   g_u8TestInProgress = 0u;
#endif
}
static void vStartSigFoxInitTest(void)
{
   uint8_t l_u8SigFoxInfoOk = 0u;
   enum {
      SM_SIGFOX_INIT = 0u,
      SM_SIGFOX_DEVICE_PAC,
      SM_SIGFOX_DEVICE_ID,
      SM_SIGFOX_IDLE_WAIT,
      SM_SIGFOX_FINISHED_OK,
      SM_SIGFOX_FINISHED_NOK
   }l_eIdxState = SM_SIGFOX_INIT;
      
   do {
      switch(l_eIdxState)
      {
         case SM_SIGFOX_INIT:
            vSigFox_DeviceInit();
            /* ReStart Timer */
            (void)eHal_Timer_Stop(g_TimeOutTestIdx);
            g_u8StopTest = 0u;
            (void)eHal_Timer_Start(g_TimeOutTestIdx, TIMEOUT_SIGFOX_INIT);
            l_eIdxState = SM_SIGFOX_DEVICE_PAC;
            break;         
         case SM_SIGFOX_DEVICE_PAC:
            l_u8SigFoxInfoOk = u8SigFox_IsDevicePACOk();
            if(g_u8StopTest == 0u)
            {
               if(l_u8SigFoxInfoOk == 1u)
               {
                  l_eIdxState = SM_SIGFOX_DEVICE_ID;
               }
            }
            else
            {
               l_eIdxState = SM_SIGFOX_FINISHED_NOK;
            }
            break;
         case SM_SIGFOX_DEVICE_ID:
            l_u8SigFoxInfoOk = u8SigFox_IsDeviceIDOk();
            if(g_u8StopTest == 0u)
            {
               if(l_u8SigFoxInfoOk == 1u)
               {
                  l_eIdxState = SM_SIGFOX_IDLE_WAIT;
               }
            }
            else
            {
               l_eIdxState = SM_SIGFOX_FINISHED_NOK;
            }
            break;
         case SM_SIGFOX_IDLE_WAIT:
            if(g_u8StopTest == 0u)
            {
               if(eUartMngt_StateGet() == USM_IDLE)
               {
                  l_eIdxState = SM_SIGFOX_FINISHED_OK;
               }
            }
            else
            {
               l_eIdxState = SM_SIGFOX_FINISHED_NOK;
            }
            break;
         case SM_SIGFOX_FINISHED_OK:
            break;
         case SM_SIGFOX_FINISHED_NOK:
         default:
            l_eIdxState = SM_SIGFOX_FINISHED_NOK;
            break;         
      }
      vUartMngt_Process();
      vSigFox_Process();
   }while((l_eIdxState != SM_SIGFOX_FINISHED_OK) && (l_eIdxState != SM_SIGFOX_FINISHED_NOK));
   
   if(l_eIdxState == SM_SIGFOX_FINISHED_OK)
   {
      PRINT_FAST("$RSL,CSF+1\n");
   }
   else
   {
      PRINT_FAST("$RSL,CSF+0\n");
   }
   
   (void)eHal_Timer_Stop(g_TimeOutTestIdx);
}

static void vStartSigFoxCWTest(void)
{
   if(g_u8SigFoxCWTest == 0u)
   {
      vSigFox_TestRadio(1u,868000000u,14u);
      g_u8SigFoxCWTest = 1u;
      PRINT_FAST("$RSL,SFC+1\n");
   }
}
static void vStopSigFoxCWTest(void)
{
   if(g_u8SigFoxCWTest == 1u)
   {
      vSigFox_TestRadio(0u,0u,0u);
      g_u8SigFoxCWTest = 0u;
      PRINT_FAST("$RSL,SFC+1\n");
   }
}


static void vStartRadioBLETest(void)
{
//   (void)nrf_sdh_disable_request();
//   while(nrf_sdh_is_enabled() == 1u);
      //(void)nrf_sdh_suspend();
   if(nrf_sdh_is_enabled() == true)
   {
      sd_softdevice_disable();
   }
   // Start 16 MHz crystal oscillator
   NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
   NRF_CLOCK->TASKS_HFCLKSTART     = 1;

   // Wait for the external oscillator to start up
   while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
   {
     // Do nothing.
   }
    
   /*radio_disable*/
   NRF_RADIO->SHORTS          = 0;
   NRF_RADIO->EVENTS_DISABLED = 0;
   NRF_RADIO->TASKS_DISABLE   = 1;
   while (NRF_RADIO->EVENTS_DISABLED == 0)
   {
     // Do nothing.
   }
   NRF_RADIO->EVENTS_DISABLED = 0;
   g_u8BLERadioChannel = 0u;

   NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
   NRF_RADIO->TXPOWER    = (BLE_RADIO_TXPOWER << RADIO_TXPOWER_TXPOWER_Pos);
   NRF_RADIO->MODE       = (BLE_RADIO_MODE << RADIO_MODE_MODE_Pos);
   NRF_RADIO->FREQUENCY  = g_u8BLERadioChannel;

   NRF_RADIO->TASKS_TXEN = 1;
   
   PRINT_DEBUG("BLE Radio : Power %d, ",BLE_RADIO_TXPOWER);
   PRINT_DEBUG("Mode %d, ",BLE_RADIO_MODE);
   PRINT_DEBUG("Channel %d\n",g_u8BLERadioChannel);
   PRINT_FAST("$RSL,BLE+1\n");
    
   g_u8BLERadioInit = 1u;
}

static void vStopRadioBLETest(void)
{
   g_u8BLERadioInit = 0u;
   NRF_RADIO->SHORTS          = 0;
   NRF_RADIO->EVENTS_DISABLED = 0;
   NRF_RADIO->TASKS_DISABLE   = 1;
   while (NRF_RADIO->EVENTS_DISABLED == 0)
   {
   // Do nothing.
   }
   NRF_RADIO->EVENTS_DISABLED = 0;
   
   NRF_RADIO->TASKS_TXEN = 0;
   NRF_RADIO->TASKS_STOP = 1;
   if(nrf_sdh_is_enabled() == true)
   {
      (void)nrf_sdh_disable_request();
      (void)nrf_sdh_enable_request();
      PRINT_FAST("$RSL,BLE+1\n");      
   }
   else
   {
      PRINT_FAST("$RSL,BLE+1\n");
   }
}


static void vSweepRadioBLEChTest(void)
{
   /*radio_disable*/
   NRF_RADIO->SHORTS          = 0;
   NRF_RADIO->EVENTS_DISABLED = 0;
   NRF_RADIO->TASKS_DISABLE   = 1;
   while (NRF_RADIO->EVENTS_DISABLED == 0)
   {
     // Do nothing.
   }
   NRF_RADIO->EVENTS_DISABLED = 0;
   g_u8BLERadioChannelSweep++;
   g_u8BLERadioChannelSweep = (g_u8BLERadioChannelSweep%80);
   
   NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
   NRF_RADIO->TXPOWER    = (BLE_RADIO_TXPOWER << RADIO_TXPOWER_TXPOWER_Pos);
   NRF_RADIO->MODE       = (BLE_RADIO_MODE << RADIO_MODE_MODE_Pos);
   NRF_RADIO->FREQUENCY  = g_u8BLERadioChannelSweep;

   NRF_RADIO->TASKS_TXEN = 1;

   PRINT_DEBUG("BLE Radio Channel %d\n",g_u8BLERadioChannelSweep);
   PRINT_FAST("$RSL,SWP+1\n");
}

static void vStartRTCTest(void)
{
   if(g_u8RTCCheck == 0u)
   {
      if(eHal_Timer_Start(g_RTCTestIdx, RTC_TIMER_TIMEOUT_MS) == HAL_TIMER_ERROR_NONE)
      {
         g_u8RTCCheck = 1u;
         PRINT_FAST("$RSL,RTC+1\n");
      }
      else
      {      
         PRINT_FAST("$RSL,RTC+0\n");
      }
   }
   else
   {      
      PRINT_FAST("$RSL,RTC+0\n");
   }
}
static void vStopRTCTest(void)
{
   if(eHal_Timer_Stop(g_RTCTestIdx) == HAL_TIMER_ERROR_NONE)
   {
      g_u8RTCCheck = 0u;
      vHal_GPIO_Clear(BP1);
      PRINT_FAST("$RSL,RTC+1\n");
   }
   else
   {      
      PRINT_FAST("$RSL,RTC+0\n");
   }
}

static void vLPMTest(void)
{
   if(g_u8I2CInitSensors == 0u)
   {      
      PRINT_FAST("$RSL,LPM+0+ISS\n");
   }
   else
   {
   #if (EN_ORG1510 == 1)
      if(g_u8TestInProgress == 1u)
      {
         eUartMngt_StateSet(USM_IDLE);
         do {
            vUartMngt_Process();
         }while(eUartMngt_StateGet() != USM_IDLE);
      }
      vHal_GPIO_Clear(GPS_POWER_EN);
      vGPIO_DefaultCfg(GPS_ON);
      vGPIO_DefaultCfg(GPS_RST);
      vGPIO_DefaultCfg(GPS_POWER_EN);
      vGPIO_DefaultCfg(GPS_UART_RX);
      vGPIO_DefaultCfg(GPS_UART_TX);
   #endif
      
   #if (EN_ADXL362 == 1)
      vHal_GPIO_Clear(ADXL_POWER_EN);
      vGPIO_DefaultCfg(ADXL_INT2);
      /* SPI */
      vHal_SPI_Uninit();
      vGPIO_DefaultCfg(ADXL_CS);
      vGPIO_DefaultCfg(SPI_SCLK);
      vGPIO_DefaultCfg(SPI_MOSI);
      vGPIO_DefaultCfg(SPI_MISO);
   #endif
      
   #if (EN_BME280 == 1)
      eBME280_OSRTemperatureSet(BME280_OVERSAMPLING_OFF);
      eBME280_OSRPressureSet(BME280_OVERSAMPLING_OFF);
      eBME280_OSRHumiditySet(BME280_OVERSAMPLING_OFF);
   #endif
   #if (EN_MAX44009 == 1)
      eMAX44009_ConversionModeSet(MAX44009_DEFAULT, MAX44009_MANUAL,MAX44009_INT_TIME_800MS,MAX44009_HIGH_BRIGHTNESS);
      eMAX44009_InterruptCfg(0u, 0u, 0u, 0u);
      vGPIO_DefaultCfg(MAX44009_INT);
   #endif
   #if (EN_LSM6DSL == 1)
      eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
      eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
      vGPIO_DefaultCfg(LSM6_INT1);
      vGPIO_DefaultCfg(LSM6_INT2);
   #endif
   #if (EN_LIS2MDL == 1)
      eLIS2MDL_LowPower(1u);
      eLIS2MDL_OutputDataRateSet(LIS2MDL_ODR_10Hz);
      eLIS2MDL_ModeSet(LIS2MDL_MODE_IDLE_DEFAULT);
      vGPIO_DefaultCfg(LIS2_INT);
   #endif
   #if (EN_LTC2943 == 1)
      eLTC2943_AlertCfgSet(LTC2943_ALERT_DISABLED);
      eLTC2943_ADCModeSet(LTC2943_ADC_MODE_SLEEP);
      eLTC2943_PowerDown(1u);
   #endif
   #if (EN_VEML6075 == 1)
   #endif
   #if (EN_ST25DV == 1)
      vGPIO_DefaultCfg(ST25DV_GPO_INT);
   #endif
      /* I2C */
      vHal_I2C_Uninit();      
      vGPIO_DefaultCfg(I2C_SDA);
      vGPIO_DefaultCfg(I2C_SCL);
      
      /* LED */
      vGPIO_DefaultCfg(LEDR_PIN);
      vGPIO_DefaultCfg(LEDG_PIN);
      vGPIO_DefaultCfg(LEDB_PIN);
      
      /* BUZZER */
      vBuzzerUninit();
      vGPIO_DefaultCfg(BUZZER_PIN);
      
      /* RTC/BP1 */
      if(g_u8RTCCheck == 1u)
      {
         eHal_Timer_Stop(g_RTCTestIdx);
      }
      vGPIO_DefaultCfg(BP1);
      
      /* SigFox */
      if(g_u8SigFoxCWTest == 1u)
      {
         vSigFox_TestRadio(0u,0u,0u);      
      }
      else
      {
         eUartMngt_StateSet(USM_SIGFOX);
         do {
            vUartMngt_Process();
         }while(eUartMngt_StateGet() != USM_IDLE);
      }
      
      vGPIO_DefaultCfg(SIGFOX_UART_RX);
      vGPIO_DefaultCfg(SIGFOX_UART_TX);
//      vGPIO_DefaultCfg(SIGFOX_DP_WU);
//      vGPIO_DefaultCfg(SIGFOX_RST); 
            
      /* Radio BLE*/   
      if(g_u8BLERadioInit == 1u)
      {
         vStopRadioBLETest();
      }
      
      PRINT_FAST("$RSL,LPM+1\n");
      
      /* Deep Sleep Mode */
      vMSM_StateMachineSet(MSM_DEEP_SLEEP);
   }
}
static void vHTTimeOutHandler(void * p_pvContext)
{
   g_u8StopTest = 1u;   
}

static void vHTRTCHandler(void * p_pvContext)
{
   vHal_GPIO_Toggle(BP1);
}

static void vGPSInit(void)
{
      enum {
      SM_GPS_INIT = 0u,
      SM_GPS_USM_GPS,
      SM_GPS_CMD_GLP,
      SM_GPS_WAIT_ACK_GLP,
      SM_GPS_CMD_HDOP,
      SM_GPS_WAIT_ACK_HDOP,
      SM_GPS_CMD_PRIORITY,
      SM_GPS_WAIT_ACK_PRIORITY,
      SM_GPS_CMD_CONSTELLATION,
      SM_GPS_WAIT_ACK_CONSTELLATION,
      SM_GPS_CMD_STATIC_NAV,
      SM_GPS_WAIT_ACK_STATIC_NAV,
      SM_GPS_CMD_SENTENCE,
      SM_GPS_WAIT_ACK_SENTENCE,
//      SM_GPS_IDLE,
      SM_GPS_FINISHED = 0xFF
   }l_eIdxState = SM_GPS_INIT;
   
   s_NMEA_PMTK_t l_sACK = { 0u };
   
   do {
      switch(l_eIdxState)
      {
         case SM_GPS_INIT:
            vORG1510_Init(g_sORG1510Context);
            eUartMngt_StateSet(USM_GPS);
            l_eIdxState = SM_GPS_USM_GPS;
            break;
         case SM_GPS_USM_GPS:
            if(eUartMngt_StateGet() == USM_GPS)
            {
               l_eIdxState = SM_GPS_CMD_GLP;
            }
            break;
         case SM_GPS_CMD_GLP:
            vORG1510_GLP(1u);
            l_eIdxState = SM_GPS_WAIT_ACK_GLP;
            break;
         case SM_GPS_WAIT_ACK_GLP:
            vNMEA_PMTKGet(&l_sACK);
            if((l_sACK.u16Cmd == 262) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
            {
               l_eIdxState = SM_GPS_CMD_HDOP;
            }
            break;
         case SM_GPS_CMD_HDOP:
            vORG1510_HDOPThresholdSet(25);
            l_eIdxState = SM_GPS_WAIT_ACK_HDOP;
            break;
         case SM_GPS_WAIT_ACK_HDOP:
            vNMEA_PMTKGet(&l_sACK);
            if(l_sACK.u16Type == 356)
            {
               l_eIdxState = SM_GPS_CMD_PRIORITY;
            }
            break;
         case SM_GPS_CMD_PRIORITY:
            vORG1510_SolutionPriority(0u);
            l_eIdxState = SM_GPS_WAIT_ACK_PRIORITY;
            break;
         case SM_GPS_WAIT_ACK_PRIORITY:
            vNMEA_PMTKGet(&l_sACK);
            if((l_sACK.u16Cmd == 257) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
            {
               l_eIdxState = SM_GPS_CMD_CONSTELLATION;
            }
            break;
         case SM_GPS_CMD_CONSTELLATION:
            vORG1510_Constellation(1,1,1,0);
            l_eIdxState = SM_GPS_WAIT_ACK_CONSTELLATION;
            break;
         case SM_GPS_WAIT_ACK_CONSTELLATION:
            vNMEA_PMTKGet(&l_sACK);
            if((l_sACK.u16Cmd == 353) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
            {
               l_eIdxState = SM_GPS_CMD_SENTENCE;
            }
            break;
         case SM_GPS_CMD_SENTENCE:
            vORG1510_SentencesUpdate(0,1,0,0,0,0,60);
            l_eIdxState = SM_GPS_WAIT_ACK_SENTENCE;
            break;
         case SM_GPS_WAIT_ACK_SENTENCE:
            vNMEA_PMTKGet(&l_sACK);
            if((l_sACK.u16Cmd == 314) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
            {
               l_eIdxState = SM_GPS_CMD_STATIC_NAV;
            }
            break;
         case SM_GPS_CMD_STATIC_NAV:
            vORG1510_StaticNav(10);
            l_eIdxState = SM_GPS_WAIT_ACK_STATIC_NAV;
            break;
         case SM_GPS_WAIT_ACK_STATIC_NAV:
            vNMEA_PMTKGet(&l_sACK);
            if((l_sACK.u16Cmd == 386) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
            {
               l_eIdxState = SM_GPS_FINISHED;
//               eUartMngt_StateSet(USM_IDLE);
            }
            break;
//         case SM_GPS_IDLE:
//            vNMEA_PMTKGet(&l_sACK);
//            if((l_sACK.u16Cmd == 225) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
//            {
//               l_eIdxState = SM_GPS_FINISHED;
//            }
//            /* ISSUE WITH ACK OF CMD 225,4 */
//               l_eIdxState = SM_GPS_FINISHED;
//            break;
         default:
            l_eIdxState = SM_GPS_FINISHED;
            break;               
      }
      vUartMngt_Process();
   }while(l_eIdxState != SM_GPS_FINISHED);
}

static void vBME_SingleShotRead(void)
{
   enum {
      SM_BME_INIT = 0u,
      SM_BME_CFG_FILTER,
      SM_BME_CFG_OSR_T,
      SM_BME_CFG_OSR_P,
      SM_BME_CFG_OSR_H,
      SM_BME_CFG_MODE,
      SM_BME_READ_TPH,
      SM_BME_GET_TPH,
      SM_BME_FINISHED = 0xFF
   }l_eIdxState = SM_BME_INIT;
   uint8_t l_u8Error = 0u;
   uint8_t l_u8RdRetry = 3u;
   
   do {
      switch(l_eIdxState)
      {
         case SM_BME_INIT:
            if(eBME280_ContextSet(g_sBMEContext) == BME280_ERROR_NONE)
            {
               l_eIdxState = SM_BME_CFG_FILTER;
            }
            else
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            break;
         case SM_BME_CFG_FILTER:
            if(eBME280_IIRFilterSet(BME280_FILTER_COEFF_OFF) == BME280_ERROR_NONE)
            {
               l_eIdxState = SM_BME_CFG_OSR_T;
            }
            else
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            break;
         case SM_BME_CFG_OSR_T:
            if(eBME280_OSRTemperatureSet(BME280_OVERSAMPLING_1X) == BME280_ERROR_NONE)
            {
               l_eIdxState = SM_BME_CFG_OSR_P;
            }
            else
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            break;
         case SM_BME_CFG_OSR_P:
            if(eBME280_OSRPressureSet(BME280_OVERSAMPLING_1X) == BME280_ERROR_NONE)
            {
               l_eIdxState = SM_BME_CFG_OSR_H;
            }
            else
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            break;
         case SM_BME_CFG_OSR_H:
            if(eBME280_OSRHumiditySet(BME280_OVERSAMPLING_1X) == BME280_ERROR_NONE)
            {
               l_eIdxState = SM_BME_CFG_MODE;
            }
            else
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            break;
         case SM_BME_CFG_MODE:
            if(eBME280_ModeSet(BME280_FORCED) == BME280_ERROR_NONE)
            {
               l_eIdxState = SM_BME_READ_TPH;
            }
            else
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            break;
         case SM_BME_READ_TPH:
            l_u8RdRetry--;
            if(l_u8RdRetry == 0u)
            {
               l_u8Error = 1u;
               l_eIdxState = SM_BME_FINISHED;
            }
            else
            {
               if(eBME280_TPHRead() == BME280_ERROR_NONE)
               {
                  l_eIdxState = SM_BME_GET_TPH;
               }
               else
               {
                  l_u8Error = 1u;
                  l_eIdxState = SM_BME_FINISHED;
               }
            }
            break;
         case SM_BME_GET_TPH:
         {            
            float l_f32Data = 0.0f;
            e_BME280_Error_t l_eErr = BME280_ERROR_NONE;
            l_u8Error = 0u;
            
            l_eErr = eBME280_TemperatureGet(&l_f32Data);
            if(l_eErr == BME280_ERROR_NONE)
            {
               PRINT_DEBUG("T : %d.",(int32_t)(l_f32Data));
               PRINT_DEBUG("%d °C, ",(int32_t)((l_f32Data-((int32_t)l_f32Data))*100.0f));
            }
            else if(l_eErr == BME280_ERROR_INVALID)
            {
               l_u8Error = 1u;
            }
            
            l_eErr = eBME280_PressureGet(&l_f32Data);
            if(l_eErr == BME280_ERROR_NONE)
            {
               PRINT_DEBUG("P : %d.",(int32_t)(l_f32Data));
               PRINT_DEBUG("%d hPa, ",(int32_t)((l_f32Data-((int32_t)l_f32Data))*10.0f));
            }
            else if(l_eErr == BME280_ERROR_INVALID)
            {
               l_u8Error = 1u;
            }
            
            l_eErr = eBME280_HumidityGet(&l_f32Data);
            if(l_eErr == BME280_ERROR_NONE)
            {
               PRINT_DEBUG("H : %d.",(int32_t)(l_f32Data));
               PRINT_DEBUG("%d %%\n",(int32_t)((l_f32Data-((int32_t)l_f32Data))*10.0f));
            }
            else if(l_eErr == BME280_ERROR_INVALID)
            {
               l_u8Error = 1u;
            }
            
            if(l_u8Error == 1u)
            {  /* Retry */
               l_eIdxState = SM_BME_READ_TPH;
            }
            else
            {
               l_u8Error = 0u;
               l_eIdxState = SM_BME_FINISHED;
            }
         }
            break;
         case SM_BME_FINISHED:
         default:
            l_eIdxState = SM_BME_FINISHED;
            break;
      }
   }while(l_eIdxState != SM_BME_FINISHED);
      
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,RSS+0+BME280\n");         
   }
   else
   {
      PRINT_FAST("$RSL,RSS+1+BME280\n");
   }
}
/****************************************************************************************
 * End Of File
 ****************************************************************************************/


