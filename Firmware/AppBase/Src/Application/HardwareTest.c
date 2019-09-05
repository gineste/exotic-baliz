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
#include <stdlib.h>
#include <nrf.h>
#include <nrf52.h>
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
#include "AxSigFox/AxSigFox.h"

#include <nrf_delay.h>
#include "Libraries/AT.h"
#include "Libraries/SimpleLED.h"
#include "Libraries/Buzzer.h"
#include "Libraries/FlashMemory.h"
#include "Libraries/NMEA.h"
#include "SigFox.h"
#include "SEGGER_RTT.h"
#include "UartManagement.h"
#include "radio_config.h"
#include "InterruptManager.h"


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
#define INT_SENSOR_UPDATE           100u

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
typedef enum _HARDWARE_TEST_CMD_ {
   HT_CMD_MAC = 0u,
   HT_CMD_BLE,
   HT_CMD_SWP,
   HT_CMD_SPI,
   HT_CMD_CSF,
   HT_CMD_SFC,
   HT_CMD_GPS,
   HT_CMD_RTC,
   HT_CMD_CFG,
   HT_CMD_RAX,
   HT_CMD_I2C,
   HT_CMD_ISS,
   HT_CMD_RSS,
   HT_CMD_LED,
   HT_CMD_BUZ,
   HT_CMD_NFC,
   HT_CMD_INT,
   HT_CMD_LPM,
   HT_CMD_BTL,
   HT_CMD_HLP,
   HT_CMD_RST,
   HT_CMD_LAST,
}e_HT_Commands_t;

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_HT_Commands_t eCommandParser(uint8_t * p_au8CmdStr, uint8_t p_u8Size);
static void vHT_NewTestProcess(e_HT_Commands_t p_eCmd, uint8_t * p_au8Arg, uint8_t p_u8Size);

static void vStartMACGet(void);
static void vStartLEDTest(void);
static void vStartBuzzerTest(void);
static void vStartSPITest(void);
static void vStartADXLTest(void);
static void vInitI2CTest(void);
static void vStartI2CSensorInitTest(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vStartI2CSensorsInitTest(void);
static void vStartI2CSensorsReadTest(void);
static void vStartSensorReadTest(uint8_t * p_pu8Arg, uint8_t p_u8Size);
//static void vStartI2CSensorsReadCustom(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vStartGPSTest(void);
static void vStopGPSTest(void);
static void vStartSigFoxInitTest(void);
static void vStartSigFoxCWTest(void);
static void vStopSigFoxCWTest(void);
static void vSigFoxCWTestCustom(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vStartRadioBLETest(void);
static void vStopRadioBLETest(void);
static void vSweepRadioBLEChTest(void);
static void vStartRTCTest(void);
static void vStopRTCTest(void);
static void vLPMTest(void);
static void vBTL_Start(void);
static void vINT_Configure(void);
static void vINT_Clear(void);
      
static void vHTTimeOutHandler(void * p_pvContext);
static void vHTRTCHandler(void * p_pvContext);
static void vHTSensorUpdateHandler(void * p_pvContext);

static void vGPSInit(void);
static uint8_t u8BME_SingleShotRead(float *p_pfT, float *p_pfP, float *p_pfH);
   
static void vCfgSensor(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vCfgBME(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vCfgADX(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vCfgLIS(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vCfgLSM(uint8_t * p_pu8Arg, uint8_t p_u8Size);
static void vCfgORG(uint8_t * p_pu8Arg, uint8_t p_u8Size);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
#if (SENSOR_NUMBER != 0)
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
      .fp_vDelay_ms = &vHal_Timer_DelayMs,         /* Function pointer to a timer in ms */
      
      .eRange = ADXL362_RANGE_2G,                  /* Range of accelerometer */
      .eOutputDataRate = ADXL362_ODR_12_5_HZ,       /* Output Data Rate */
      .eLinkLoopMode = ADXL362_MODE_LOOP,          /* Functioning mode (Link/Loop) */
      .eNoiseCtrl = ADXL362_NOISE_MODE_NORMAL,     /* Noise control (Normal, Low, Ultra Low) */
      
      .u8ActivityDetection = 1u,			/* 0 - no activity detection, 1 - activity detection */
      .u8RefOrAbsActivity = 1u, 			/* 0 - absolute mode, 1 - referenced mode. */
      .u16ThresholdActivity = 250,   /* 11-bit unsigned value that the adxl362 samples are compared to. */
      .u16TimeActivity = 250u,			/* 16-bit value activity time in ms (from 2.5ms @ odr = 400Hz to 20s @ odr = 12.5Hz) */
      
      .u8InactivityDetection = 1u,		/* 0 - no activity detection, 1 - activity detection */
      .u8RefOrAbsInactivity = 1u,		/* 0 - absolute mode, 1 - referenced mode. */
      .u16ThresholdInactivity = 400, /* 11-bit unsigned value that the ADXL362 samples are compared to. */
      .u32TimeInactivity = 1000u,  		/* 16-bit value inactivity time in ms (from 164s @ odr = 400Hz to 87min @ odr = 12.5Hz) */
                                        
      .eInt1Map = ADXL362_INTMAPX_OFF,  /* Type of the interrupt n�1 */
      .eInt2Map = ADXL362_INTMAPX_AWAKE,    /* Type of the interrupt n�2 */
//      .u32Int1Pin = 0xFF, 
//      .u32Int2Pin = ADXL_INT2,
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
   
#endif
static uint8_t g_u8SPIInit = 0u;
static uint8_t g_u8I2CInit = 0u;
static uint8_t g_u8BMEInit = 0u;
static uint8_t g_u8MAXInit = 0u;
static uint8_t g_u8LISInit = 0u;
static uint8_t g_u8LSMInit = 0u;
static uint8_t g_u8LTCInit = 0u;
static uint8_t g_u8ADXInit = 0u;
   
static uint8_t g_u8StopTest = 0u;
static uint8_t g_u8TestInProgress = 0u;
static uint8_t g_u8I2CInitSensors = 0u;
static uint8_t g_u8RTCCheck = 0u;   
static uint8_t g_u8INTCheck = 0u;   
   
static uint8_t g_u8UpdateSensor = 0u;
   
static uint8_t g_u8BLERadioInit = 0u;
static uint8_t g_u8BLERadioChannel = 0u;
static uint8_t g_u8BLERadioChannelSweep = 0u;

static uint8_t g_u8SigFoxCWTest = 0u;

HAL_TIMER_DEF(g_TimeOutTestIdx);
HAL_TIMER_DEF(g_RTCTestIdx);
HAL_TIMER_DEF(g_SensorUpdateIdx);

/* Must be in the same order of e_HT_Commands_t */
const char g_cachCmd[HT_CMD_LAST][CMD_FRAME_SIZE+1u] = {
   "MAC\0",
   "BLE\0",
   "SWP\0",
   "SPI\0",
   "CSF\0",
   "SFC\0",
   "GPS\0",
   "RTC\0",
   "CFG\0",
   "RAX\0",
   "I2C\0",
   "ISS\0",
   "RSS\0",
   "LED\0",
   "BUZ\0",
   "NFC\0",
   "INT\0",
   "LPM\0",
   "BTL\0",
   "HLP\0",
   "RST\0",
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
   PRINT_FAST("MAC: Get BLE MAC Address\n");
   PRINT_FAST("BLE: Start/Stop Radio BLE(Scope)\n");
   PRINT_FAST("SWP: Sweep BLE Radio Channel(Scope)\n");
   PRINT_FAST("CSF: Check Comm UART SigFox (Blocking Test)\n");
   PRINT_FAST("SFC: Check Radio Tx SigFox 868MHz 14dBm(Scope)\n");
   PRINT_FAST("GPS: Start/Stop GPS\n");
   PRINT_FAST("RTC: Start/Stop Timer RTC\n");
   PRINT_FAST("CFG: Configure Sensors\n");
   PRINT_FAST("SPI: Check SPI ADXL362\n");
   PRINT_FAST("RAX: Read SPI ADXL362\n");
   PRINT_FAST("I2C: Init I2C Layer\n");
   PRINT_FAST("ISS: Init All I2C Sensors\n");
   PRINT_FAST("RSS: Read All I2C Sensors\n");   
   PRINT_FAST("LED: Check LED (Visual)\n");
   PRINT_FAST("BUZ: Check Buzzer (Audio)\n");
   PRINT_FAST("NFC: Write Data on \n");
   PRINT_FAST("INT: Initialize/Clear Interrupt\n");
   PRINT_FAST("LPM: Lowest Power Mode Set\n");
   PRINT_FAST("BTL: Put Device in Bootloader for Firmware Update\n");
   PRINT_FAST("HLP: Print Help\n");
   PRINT_FAST("RST: RESET\n");
}

void vHT_Init(void)
{
   (void)eHal_Timer_Create(&g_TimeOutTestIdx, HAL_TIMER_MODE_SINGLE_SHOT, &vHTTimeOutHandler);
   (void)eHal_Timer_Create(&g_RTCTestIdx, HAL_TIMER_MODE_REPEATED, &vHTRTCHandler);   
   (void)eHal_Timer_Create(&g_SensorUpdateIdx, HAL_TIMER_MODE_REPEATED, &vHTSensorUpdateHandler);   
}

void vHT_CheckInput(uint8_t * p_au8Frame, uint8_t p_u8Size)
{
   e_HT_Commands_t l_eCmd = HT_CMD_LAST;
   
   if(   (p_au8Frame[0u] == '$')
      && (p_au8Frame[1u] == 'C')
      && (p_au8Frame[2u] == 'H')
      && (p_au8Frame[3u] == 'K')
      && (p_au8Frame[4u] == ',') )
   {
      
      l_eCmd = eCommandParser(&p_au8Frame[5u], 3u/*CMD_FRAME_SIZE*/);
      if(l_eCmd != HT_CMD_LAST)
      {
         if(p_u8Size == FRAME_SIZE_MAX)
         {
            vHT_NewTestProcess(l_eCmd, NULL, 0u);
         }
         else if (p_u8Size > FRAME_SIZE_MAX)
         {  /* Cmd with arguments */
            vHT_NewTestProcess(l_eCmd, &p_au8Frame[9u], p_u8Size - 9u);
         }
         else
         {  /* Cmd not valid */
         }
      }
      else
      {  /* Cmd not valid */
      }
   }
}

void vHT_BackgroundProcess(void)
{
   if( (g_u8INTCheck == 1u) && (g_u8UpdateSensor == 1u) )
   {
      if(g_u8I2CInitSensors == 1u)
      { 
         #if (EN_BME280 == 1)
         #endif
         
         #if (EN_MAX44009 == 1)  
         #endif
         
//         #if (EN_LSM6DSL == 1)
//            (void)eLSM6DSL_AccelRead();
//            (void)eLSM6DSL_GyroRead();
//         #endif
         
         #if (EN_LIS2MDL == 1)
            (void)eLIS2MDL_MagneticRead();
         #endif
      }
      
      g_u8UpdateSensor = 0u;
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
static void vHT_NewTestProcess(e_HT_Commands_t p_eCmd, uint8_t * p_au8Arg, uint8_t p_u8Size)
{
   switch(p_eCmd)
   {
      case HT_CMD_MAC:
         PRINT_FAST("$ACK,MAC+1\n");
         vStartMACGet();
         break;
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
         if(p_u8Size == 0u)
         {
            (g_u8SigFoxCWTest == 0u) ? vStartSigFoxCWTest():vStopSigFoxCWTest();
         }
         else
         {
            vSigFoxCWTestCustom(p_au8Arg,p_u8Size);
         }
         break;
      case HT_CMD_GPS:
         PRINT_FAST("$ACK,GPS+1\n");          
         (g_u8TestInProgress == 0u) ? vStartGPSTest():vStopGPSTest();     
         break;
      case HT_CMD_RTC:
         PRINT_FAST("$ACK,RTC+1\n");
         (g_u8RTCCheck == 0u) ? vStartRTCTest():vStopRTCTest();         
         break;
      case HT_CMD_CFG:
         vCfgSensor(p_au8Arg,p_u8Size);
         break;
      case HT_CMD_SPI:
         PRINT_FAST("$ACK,SPI+1\n");
         vStartSPITest();
         break;
      case HT_CMD_RAX:
         PRINT_FAST("$ACK,RAX+1\n");
         vStartADXLTest();
         break;
      case HT_CMD_I2C:
         PRINT_FAST("$ACK,I2C+1\n");
         vInitI2CTest();
         break;
      case HT_CMD_ISS:
         if(p_u8Size == 0u)
         {
            PRINT_FAST("$ACK,ISS+1\n");
            vStartI2CSensorsInitTest();
         }
         else
         {
            vStartI2CSensorInitTest(p_au8Arg, p_u8Size);
         }
         break;
      case HT_CMD_RSS:
         if(p_u8Size == 0u)
         {
            PRINT_FAST("$ACK,RSS+1\n");
            vStartI2CSensorsReadTest();
         }
         else
         {
            vStartSensorReadTest(p_au8Arg, p_u8Size);
            //vStartI2CSensorsReadCustom(p_au8Arg,p_u8Size);
         }
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
      case HT_CMD_INT:
         PRINT_FAST("$ACK,INT+1\n");
         if(g_u8INTCheck == 0u){
            vIntManagerInit();
            vINT_Configure();
            (void)eHal_Timer_Start(g_SensorUpdateIdx, INT_SENSOR_UPDATE);
            g_u8INTCheck = 1u;
         }
         else{
            vINT_Clear();
         }
         break;
      case HT_CMD_LPM:
         PRINT_FAST("$ACK,LPM+1\n");
         vLPMTest();
         break;
      case HT_CMD_BTL:
         PRINT_FAST("$ACK,DFU+1\n");
         vBTL_Start();
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

static void vStartMACGet(void)
{
   uint8_t l_au8Data[6u] = { 0u };
   uint8_t l_u8Size = 0u;
   vBLE_MACAddressGet(l_au8Data, &l_u8Size);
   PRINT_CUSTOM("$RSL,MAC+1,%02X:%02X:%02X:%02X:%02X:%02X\n",  
         l_au8Data[5u],l_au8Data[4u],l_au8Data[3u],l_au8Data[2u],l_au8Data[1u],l_au8Data[0u]);   
}
static void vStartLEDTest(void)
{
   uint8_t l_u8Idx = 0u;
   for(l_u8Idx = 0u; l_u8Idx < LED_DEFAULTCOLOR; l_u8Idx++)
   {
      vSimpleLED_ColorSet((e_SimpleLED_Color_t)l_u8Idx);
      nrf_delay_ms(DELAY_LED_SWITCH);      
   }
   PRINT_FAST("$RSL,LED+?\n");
}

static void vStartBuzzerTest(void)
{
   //vBuzzerSeqCmd();
   vBuzzerStartSequence(1u);
   while(u8BuzzerIsStopped() != 1u);
   PRINT_FAST("$RSL,BUZ+?\n");
}


static void vStartSPITest(void)
{   
   vHal_GPIO_Set(ADXL_POWER_EN);
   vHal_Timer_DelayMs(10u);
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
   PRINT_FAST("$RSL,SPI+1\n");

   g_u8SPIInit = 1u;
}
static void vStartADXLTest(void)
{
#if (EN_ADXL362 == 1)
   int16_t l_s16Data[3u] = { 0 };
   
   if(eADXL362_ContextSet(g_sADXLContext) == ADXL362_ERROR_NONE)
   {
      if(u8ADXL362_IsAvailable() == 0u)
      {
         if(eADXL362_Init() != ADXL362_ERROR_NONE)
         {
            PRINT_FAST("$RSL,RAX+0\n");
         }
      }
      vHal_Timer_DelayMs(200u);
      if(u8ADXL362_IsAvailable() == 1u)
      {
         if(eADXL362_AccelerationRead() != ADXL362_ERROR_NONE)
         {
            PRINT_FAST("$RSL,RAX+0\n");
         }
         else
         {
            vADXL362_AccelerationGet(&l_s16Data[0u],&l_s16Data[1u],&l_s16Data[2u]);
            PRINT_CUSTOM("$RSL,RAX+1,%d,%d,%d\n",l_s16Data[0u],l_s16Data[1u],l_s16Data[2u]);
         }
      }
   }
   
#endif
}

static void vInitI2CTest(void)
{
   s_HalI2C_Context_t l_sI2CContext = {
      .u32SCLPin = I2C_SCL,
      .u32SDAPin = I2C_SDA,
      .eFrequency = HALI2C_FREQ_400K,
      .u8EnablePullUp = 1u,
   };
   vHal_I2C_ContextSet(l_sI2CContext);
   vHal_I2C_Init();
   g_u8I2CInit = 1u;
   PRINT_FAST("$RSL,I2C+1\n");
}

static void vStartI2CSensorsInitTest(void)
{   
   uint8_t l_u8ChipID = 0u;
   uint8_t l_u8Error = 1u;
   
   s_HalI2C_Context_t l_sI2CContext = {
      .u32SCLPin = I2C_SCL,
      .u32SDAPin = I2C_SDA,
      .eFrequency = HALI2C_FREQ_400K,
      .u8EnablePullUp = 1u,
   };
   
   vHal_I2C_ContextSet(l_sI2CContext);
   vHal_I2C_Init();
   g_u8I2CInit = 1u;
   
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
      return;
   }
//   else
//   {
//      PRINT_FAST("$RSL,ISS+1+BME280\n");
//   }
   
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
      return;
   }
//   else
//   {
//      PRINT_FAST("$RSL,ISS+1+MAX44009\n");
//   }
#endif
#if (EN_LSM6DSL == 1)
   l_u8Error = 1u;
   l_u8ChipID = 0u;
   if(eLSM6DSL_ContextSet(g_sLSM6Context) == LSM6DSL_ERROR_NONE)
   {
      if(eLSM6DSL_WhoAmIGet(&l_u8ChipID) == LSM6DSL_ERROR_NONE)
      {
         l_u8Error = (l_u8ChipID == LSM6DSL_WHO_I_AM_ID)? 0u : 1u;
         (void)eLSM6DSL_BlockDataUpdateSet(1u);
         (void)eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_833Hz, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
         (void)eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_833Hz, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
            
      }
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+LSM6DSL\n");
      return;
   }
//   else
//   {
//      PRINT_FAST("$RSL,ISS+1+LSM6DSL\n");
//   }
#endif
#if (EN_LIS2MDL == 1)
   l_u8Error = 1u;
   l_u8ChipID = 0u;
   if(eLIS2MDL_ContextSet(g_sLIS2Context) == LIS2MDL_ERROR_NONE)
   {
      if(eLIS2MDL_WhoAmIGet(&l_u8ChipID) == LIS2MDL_ERROR_NONE)
      {
         (void)eLIS2MDL_LowPower(1u);
         (void)eLIS2MDL_BlockDataUpdateSet(1u);
         (void)eLIS2MDL_OutputDataRateSet(LIS2MDL_ODR_10Hz);
         (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
         l_u8Error = (l_u8ChipID == LIS2MDL_WHO_AM_I_ID)? 0u : 1u;
      }
   }
   
   if(l_u8Error == 1u)
   {
      PRINT_FAST("$RSL,ISS+0+LIS2MDL\n");
      return;
   }
//   else
//   {
//      PRINT_FAST("$RSL,ISS+1+LIS2MDL\n");
//   }
   
#endif

#if (EN_VEML6075 == 1)

#endif
   
#if (EN_ST25DV == 1)
   if(eST25DV_ContextSet(g_sST25DVContext) != ST25DV_ERROR_NONE)
   {
      PRINT_FAST("$RSL,ISS+0+ST25DV\n");
      return;
   }
//   else
//   {
//      PRINT_FAST("$RSL,ISS+0+ST25DV\n");
//   }
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
      return;
   }
//   else
//   {
//      PRINT_FAST("$RSL,ISS+1+LTC2943\n");
//   }
#endif
   
   PRINT_FAST("$RSL,ISS+1\n");
   g_u8I2CInitSensors = 1u;
}

static void vStartI2CSensorInitTest(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   uint8_t l_u8Error = 1u;
   uint8_t l_u8ChipID = 0u;   
   
   if(p_u8Size >= 4u)
   {
      if(   (g_u8SPIInit == 1)
         && (strstr((char*)p_pu8Arg, "ADX") != NULL) )
      {
         l_u8Error = 1u;
         PRINT_FAST("$ACK,ISS,ADX+1\n");
         #if (EN_ADXL362 == 1)
            if(eADXL362_ContextSet(g_sADXLContext) == ADXL362_ERROR_NONE)
            {
               if(eADXL362_SoftReset() == ADXL362_ERROR_NONE)
               {
                  vHal_Timer_DelayMs(1u);
                  if(eADXL362_PartIDGet(&l_u8ChipID) == ADXL362_ERROR_NONE)
                  {
                     l_u8Error = 0u;
                  }
               }            
            }
         
            if(l_u8Error == 0u)
            {
               g_u8ADXInit = 1u;
               PRINT_FAST("$RSL,ISS,ADX+1\n");
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,ISS,ADX+0\n");
            }
      }
      else if(g_u8I2CInit == 1u)
      {
         if(strstr((char*)p_pu8Arg, "BME") != NULL)
         {
            PRINT_FAST("$ACK,ISS,BME+1\n");
         #if (EN_BME280 == 1) 
            l_u8Error = 1u;
            if(eBME280_ContextSet(g_sBMEContext) == BME280_ERROR_NONE)
            {
               if(eBME280_ChipIDGet(&l_u8ChipID) == BME280_ERROR_NONE)
               {
                  l_u8Error = (l_u8ChipID == BME280_CHIP_ID)? 0u : 1u;
               }
            }
            
            if(l_u8Error == 0u)
            {
               g_u8BMEInit = 1u;
               PRINT_FAST("$RSL,ISS,BME+1\n");
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,ISS,BME+0\n");
            }
         }
         else if(strstr((char*)p_pu8Arg, "MAX") != NULL)
         {
            PRINT_FAST("$ACK,ISS,MAX+1\n");
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
            
            if(l_u8Error == 0u)
            {
               g_u8MAXInit = 1u;
               PRINT_FAST("$RSL,ISS,MAX+1\n");
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,ISS,MAX+0\n");
            }
         }
         else if(strstr((char*)p_pu8Arg, "LIS") != NULL)
         {
            PRINT_FAST("$ACK,ISS,LIS+1\n");
         #if (EN_LIS2MDL == 1)
            l_u8Error = 1u;
            l_u8ChipID = 0u;
            if(eLIS2MDL_ContextSet(g_sLIS2Context) == LIS2MDL_ERROR_NONE)
            {
               if(eLIS2MDL_WhoAmIGet(&l_u8ChipID) == LIS2MDL_ERROR_NONE)
               {
                  l_u8Error = (l_u8ChipID == LIS2MDL_WHO_AM_I_ID)? 0u : 1u;
               }
            }
            
            if(l_u8Error == 0u)
            {
               g_u8LISInit = 1u;
               PRINT_FAST("$RSL,ISS,LIS+1\n");
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,ISS,LIS+0\n");
            }
         }
         else if(strstr((char*)p_pu8Arg, "LSM") != NULL)
         {
            PRINT_FAST("$ACK,ISS,LSM+1\n");
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
            
            if(l_u8Error == 0u)
            {
               g_u8LSMInit = 1u;
               PRINT_FAST("$RSL,ISS,LSM+1\n");
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,ISS,LSM+0\n");
            }
         }
         else if(strstr((char*)p_pu8Arg, "LTC") != NULL)
         {         
            PRINT_FAST("$ACK,ISS,LTC+1\n");
         #if (EN_LTC2943 == 1)
            l_u8Error = 1u;
            if(eLTC2943_ContextSet(g_sLTCContext) == LTC2943_ERROR_NONE)
            {
               if(eLTC2943_StatusGet(&l_u8ChipID) == LTC2943_ERROR_NONE)
               {
                  l_u8Error = 0u;
               }
            }
            
            if(l_u8Error == 0u)
            {
               g_u8LTCInit = 1u;
               PRINT_FAST("$RSL,ISS,LTC+1\n");
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,ISS,LTC+0\n");
            }
         }
         else
         {
            PRINT_FAST("$ACK,ISS+0\n");
         }
      }
      else
      {
         PRINT_FAST("$ACK,ISS+0\n");
      }
   }
   else
   {
      PRINT_FAST("$ACK,ISS+0\n");
   }
}

static void vStartI2CSensorsReadTest(void)
{
   uint8_t l_u8Error = 1u;
   int16_t l_s16X = 0;
   int16_t l_s16Y = 0;
   int16_t l_s16Z = 0;
   float l_fT;
   float l_fP;
   float l_fH;
   char l_achData[40] = { 0 };
   char l_achDataResult[256] = { 0u };
      
   if(g_u8I2CInitSensors == 1u)
   {      
      strcat(l_achDataResult, "$RSL,RSS+1");
      #if (EN_BME280 == 1)
         l_u8Error = u8BME_SingleShotRead(&l_fT, &l_fP, &l_fH);
         if(l_u8Error == 1)
         {
            PRINT_FAST("$RSL,RSS+0+BME280\n");
            return;
         }
         else
         {
            sprintf(l_achData, ", T=%.2f", l_fT);
            strcat(l_achDataResult, l_achData);    
            sprintf(l_achData, ", P=%.2f", l_fP);
            strcat(l_achDataResult, l_achData);         
            sprintf(l_achData, ", H=%.2f", l_fH);
            strcat(l_achDataResult, l_achData);    
         }
      #endif
      
      #if (EN_MAX44009 == 1)  
         l_u8Error = 1u;
         uint32_t l_u32Brightness = 0u;
         if(eMAX44009_BrightnessRead(NULL) == MAX44009_ERROR_NONE)
         {
            if(eMAX44009_BrightnessGet(&l_u32Brightness) == MAX44009_ERROR_NONE)
            {
//               PRINT_DEBUG("L : %d Lux\n", l_u32Brightness);
               sprintf(l_achData, ", L=%d", l_u32Brightness);
               strcat(l_achDataResult, l_achData);
               l_u8Error = 0u;
            }            
         }
         if(l_u8Error == 1u)
         {
            PRINT_FAST("$RSL,RSS+0+MAX44009\n");
            return;
         }
//         else
//         {
//            PRINT_FAST("$RSL,RSS+1+MAX44009\n");
//         }
      #endif
      
      #if (EN_LSM6DSL == 1)
         l_u8Error = 1u;    
//         if(eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_833Hz, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER) == LSM6DSL_ERROR_NONE)
         {
            if(eLSM6DSL_AccelRead() == LSM6DSL_ERROR_NONE)
            {
               if(eLSM6DSL_AccelGet(&l_s16X, &l_s16Y, &l_s16Z) == LSM6DSL_ERROR_NONE)
               {
//                  PRINT_DEBUG("Acc X : %d mG, ", l_s16X);
//                  PRINT_DEBUG("Acc Y : %d mG, ", l_s16Y);
//                  PRINT_DEBUG("Acc Z : %d mG\n", l_s16Z);
                  sprintf(l_achData, ", Acc=%d, %d, %d", l_s16X,l_s16Y,l_s16Z);
                  strcat(l_achDataResult, l_achData);
                  l_u8Error = 0u;
               }
            } 
         }
         if(l_u8Error == 0u)
         {
            l_u8Error = 1u;
//            if(eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_833Hz, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER) == LSM6DSL_ERROR_NONE)
            {
               if(eLSM6DSL_GyroRead() == LSM6DSL_ERROR_NONE)
               {
                  if(eLSM6DSL_GyroGet(&l_s16X, &l_s16Y, &l_s16Z) == LSM6DSL_ERROR_NONE)
                  {
//                     PRINT_DEBUG("Gyr X : %d dps, ", l_s16X);
//                     PRINT_DEBUG("Gyr Y : %d dps, ", l_s16Y);
//                     PRINT_DEBUG("Gyr Z : %d dps\n", l_s16Z);
                     sprintf(l_achData, ", Gyr=%d, %d, %d", l_s16X,l_s16Y,l_s16Z);
                     strcat(l_achDataResult, l_achData);
                     l_u8Error = 0u;
                  }
               }
            }
         }
         
         if(l_u8Error == 1u)
         {
            PRINT_FAST("$RSL,RSS+0+LSM6DSL\n");
            return;
         }
//         else
//         {
//            PRINT_FAST("$RSL,RSS+1+LSM6DSL\n");
//         }
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
//                  PRINT_DEBUG("Mag X : %d G, ", l_s16X);
//                  PRINT_DEBUG("Mag Y : %d G, ", l_s16Y);
//                  PRINT_DEBUG("Mag Z : %d G\n", l_s16Z);
                  sprintf(l_achData, ", Mag=%d, %d, %d", l_s16X,l_s16Y,l_s16Z);
                  strcat(l_achDataResult, l_achData);
                  l_u8Error = 0u;
               }
            }
//         }
         if(l_u8Error == 1u)
         {
            PRINT_FAST("$RSL,RSS+0+LIS2MDL\n");
            return;
         }
//         else
//         {
//            PRINT_FAST("$RSL,RSS+1+LIS2MDL\n");
//         }
      #endif
      
      #if (EN_VEML6075 == 1)
//      uint8_t l_u8Index = 0xFFu;
//      PRINT_DEBUG("%s","VEML6075 Test: ");
//      vVEML6075_Configure();
//      vVEML6075_PollingProcess();
//      (void)eVEML6075_UVIndexGet(&l_u8Index);
//      PRINT_DEBUG("UV Index %d\n",l_u8Index);
//      nrf_delay_ms(DELAY_SENSORS_READ);
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
//                  PRINT_DEBUG("Acc I : %d mA\n", l_s16Y);
                  sprintf(l_achData, ", Iacc=%d", l_s16Y);
                  strcat(l_achDataResult, l_achData);
               }
               else
               {
                  l_u8Error += 1u;
               }
                  
               if(eLTC2943_VoltageGet(&l_u16Volt) == LTC2943_ERROR_NONE)
               {
//                  PRINT_DEBUG("V : %d mV\n", l_u16Volt);
                  sprintf(l_achData, ", V=%d", l_u16Volt);
                  strcat(l_achDataResult, l_achData);
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
            return;
         }
//         else
//         {
//            PRINT_FAST("$RSL,RSS+1+LTC2943\n");
//         }
      #endif
     
   PRINT_CUSTOM("%s\n",l_achDataResult);
   }
   else
   {
      PRINT_FAST("$RSL,RSS+0\n");
      return;
   }  
}

static void vStartSensorReadTest(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   uint8_t l_u8Error = 1u;
   int16_t l_s16X = 0;
   int16_t l_s16Y = 0;
   int16_t l_s16Z = 0;
   char l_achData[40] = { 0 };
   char l_achDataResult[256] = { 0u };
   
   if(p_u8Size >= 4u)
   {
      if(strstr((char*)p_pu8Arg, "BME") != NULL)
      {
         PRINT_FAST("$ACK,RSS,BME+1\n");
         l_u8Error = 1u;
         if(g_u8BMEInit == 1u)
         {
         #if (EN_BME280 == 1)
            float l_fT;
            float l_fP;
            float l_fH;
            l_u8Error = u8BME_SingleShotRead(&l_fT, &l_fP, &l_fH);
            if(l_u8Error == 0)
            {
               strcat(l_achDataResult, "$RSL,RSS,BME+1");
               sprintf(l_achData, ",%.2f", l_fT);
               strcat(l_achDataResult, l_achData);    
               sprintf(l_achData, ",%.2f", l_fP);
               strcat(l_achDataResult, l_achData);         
               sprintf(l_achData, ",%.2f", l_fH);
               strcat(l_achDataResult, l_achData); 
               PRINT_CUSTOM("%s\n",l_achDataResult); 
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,RSS,BME+0\n");
            }
         }
         else
         {
            PRINT_FAST("$RSL,RSS,BME+0,ISS\n");
         }
      }
      else if(strstr((char*)p_pu8Arg, "MAX") != NULL)
      {
         PRINT_FAST("$ACK,RSS,MAX+1\n");
         l_u8Error = 1u;
         if(g_u8MAXInit == 1u)
         {
         #if (EN_MAX44009 == 1)        
            uint32_t l_u32Brightness = 0u;
            if(eMAX44009_BrightnessRead(NULL) == MAX44009_ERROR_NONE)
            {
               if(eMAX44009_BrightnessGet(&l_u32Brightness) == MAX44009_ERROR_NONE)
               {
                  l_u8Error = 0u;
               }            
            }            
            
            if(l_u8Error == 0u)
            {
               strcat(l_achDataResult, "$RSL,RSS,MAX+1");
               sprintf(l_achData, ",%d", l_u32Brightness);
               strcat(l_achDataResult, l_achData);
               PRINT_CUSTOM("%s\n",l_achDataResult);
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,RSS,MAX+0\n");
            }
         }
         else
         {
            PRINT_FAST("$RSL,RSS,MAX+0,ISS\n");
         }
      }
      else if(strstr((char*)p_pu8Arg, "LIS") != NULL)
      {
         PRINT_FAST("$ACK,RSS,LIS+1\n");
         l_u8Error = 1u;
         if(g_u8LISInit == 1u)
         {
         #if (EN_LIS2MDL == 1)
               
            (void)eLIS2MDL_LowPower(1u);
            (void)eLIS2MDL_BlockDataUpdateSet(1u);
            (void)eLIS2MDL_OutputDataRateSet(LIS2MDL_ODR_100Hz);
            (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
            vHal_Timer_DelayMs(100u);
            if(eLIS2MDL_MagneticRead() == LIS2MDL_ERROR_NONE)
            {
               if(eLIS2MDL_MagDataGet(&l_s16X, &l_s16Y, &l_s16Z) == LIS2MDL_ERROR_NONE)
               {
                  l_u8Error = 0u;
                  strcat(l_achDataResult, "$RSL,RSS,LIS+1");
                  sprintf(l_achData, ",%d,%d,%d", l_s16X,l_s16Y,l_s16Z);
                  strcat(l_achDataResult, l_achData);
               }
            }
            
            if(l_u8Error == 0u)
            {
               (void)eLIS2MDL_OutputDataRateSet(LIS2MDL_ODR_10Hz);
               PRINT_CUSTOM("%s\n",l_achDataResult);
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,RSS,LIS+0\n");
            }
         }
         else
         {
            PRINT_FAST("$RSL,RSS,LIS+0,ISS\n");
         }
      }
      else if(strstr((char*)p_pu8Arg, "LSM") != NULL)
      {
         PRINT_FAST("$ACK,RSS,LSM+1\n");
         l_u8Error = 1u;
         if(g_u8LSMInit == 1u)
         {
      #if (EN_LSM6DSL == 1)
            (void)eLSM6DSL_BlockDataUpdateSet(1u);
            (void)eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_6_66kHz, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
            (void)eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_6_66kHz, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
            vHal_Timer_DelayMs(100u);
            if(   (eLSM6DSL_GyroRead() == LSM6DSL_ERROR_NONE)
               && (eLSM6DSL_AccelRead() == LSM6DSL_ERROR_NONE) )
            {
               l_u8Error = 0u;
               strcat(l_achDataResult, "$RSL,RSS,LSM+1");
               if(eLSM6DSL_AccelGet(&l_s16X, &l_s16Y, &l_s16Z) == LSM6DSL_ERROR_NONE)
               {
                  sprintf(l_achData, ",%d,%d,%d", l_s16X,l_s16Y,l_s16Z);
                  strcat(l_achDataResult, l_achData);
               }
               if(eLSM6DSL_GyroGet(&l_s16X, &l_s16Y, &l_s16Z) == LSM6DSL_ERROR_NONE)
               {
                  sprintf(l_achData, ",%d,%d,%d", l_s16X,l_s16Y,l_s16Z);
                  strcat(l_achDataResult, l_achData);
               }
            }
            
            if(l_u8Error == 0u)
            {
               (void)eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
               (void)eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
               PRINT_CUSTOM("%s\n",l_achDataResult);
            }
            else
      #endif
            {
               PRINT_FAST("$RSL,RSS,LSM+0\n");
            }
         }
         else
         {
            PRINT_FAST("$RSL,RSS,LSM+0,ISS\n");
         }
      }
      else if(strstr((char*)p_pu8Arg, "LTC") != NULL)
      {
         PRINT_FAST("$ACK,ISS,LTC+1\n");
         l_u8Error = 1u;
         if(g_u8LTCInit == 1u)
         {
      #if (EN_LTC2943 == 1)
            uint16_t l_u16Volt = 0u;
            int16_t l_s16Curr = 0u;
         
            eLTC2943_PowerDown(0u);
            if(eLTC2943_ADCModeSet(LTC2943_ADC_MODE_MANUAL) == LTC2943_ERROR_NONE)
            {
               if(eLTC2943_SensorRead(ADC_SENSOR_VOLTAGE_MASK | ADC_SENSOR_CURRENT_MASK) == LTC2943_ERROR_NONE)
               {
                  if(   (eLTC2943_VoltageGet(&l_u16Volt) == LTC2943_ERROR_NONE)
                     && (eLTC2943_AccumulatedChargeGet(&l_s16Curr) == LTC2943_ERROR_NONE) )
                  {
                     l_u8Error = 0u;
                     strcat(l_achDataResult, "$RSL,RSS,LSM+1");
                     sprintf(l_achData, ",%d,%d", l_u16Volt,l_s16Curr);
                     strcat(l_achDataResult, l_achData);
                  }
               }
            }
            
            if(l_u8Error == 0u)
            {
               PRINT_CUSTOM("%s\n",l_achDataResult);
            }
            else
      #endif
            {
               PRINT_FAST("$RSL,RSS,LTC+0\n");
            }
         }
         else
         {
            PRINT_FAST("$RSL,RSS,LTC+0,ISS\n");
         }
      }
      else if(strstr((char*)p_pu8Arg, "ADX") != NULL)
      {
         PRINT_FAST("$ACK,RSS,ADX+1\n");
         l_u8Error = 1u;
         if(g_u8ADXInit == 1u)
         {
         #if (EN_ADXL362 == 1)    
            if(u8ADXL362_IsAvailable() == 0u)
            {
               if(eADXL362_Init() != ADXL362_ERROR_NONE)
               {
                  PRINT_FAST("$RSL,RAX+0\n");
               }
            }
            vHal_Timer_DelayMs(200u);            
            if(u8ADXL362_IsAvailable() == 1u)
            {
               if(eADXL362_AccelerationRead() == ADXL362_ERROR_NONE)
               {
                  vADXL362_AccelerationGet(&l_s16X,&l_s16Y,&l_s16Z);
                  strcat(l_achDataResult, "$RSL,RSS,ADX+1");
                  sprintf(l_achData, ",%d,%d,%d", l_s16X,l_s16Y,l_s16Z);
                  strcat(l_achDataResult, l_achData);
                  l_u8Error = 0u;
               }
            }
            
            if(l_u8Error == 0u)
            {
               PRINT_CUSTOM("%s\n",l_achDataResult);
            }
            else
         #endif
            {
               PRINT_FAST("$RSL,RSS,ADX+0\n");
            }
         }
         else
         {
            PRINT_FAST("$RSL,RSS,ADX+0,ISS\n");
         }
      }
      else
      {
         PRINT_FAST("$ACK,RSS+0\n");
      }
   }
}


static void vStartGPSTest(void)
{   
#if (EN_ORG1510 == 1)
   vHal_GPIO_Set(GPS_POWER_EN);  
   // ORG1510 MK05 Issue
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
   uint8_t l_u8Size = 0u;
   uint8_t l_au8Info[25] = {0u};
   char l_achData[40] = { 0 };
   char l_achDataResult[256] = { 0u };
   
   strcat(l_achDataResult, "$RSL,CSF+1");
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
                  memset(l_achData, 0u, 40);// ex 9D3B7E11D7333FB7
                  vAXSigFox_DevicePacGet(l_au8Info, &l_u8Size);
                  l_au8Info[l_u8Size-1] = 0u;
                  sprintf(l_achData, ",%s",l_au8Info);
                  strcat(l_achDataResult, l_achData);
                  l_eIdxState = SM_SIGFOX_DEVICE_ID;
                  /* for next use */
                  memset(l_au8Info, 0u, l_u8Size);
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
                  vAXSigFox_DeviceIdGet(l_au8Info, &l_u8Size);
                  memset(l_achData, 0u, 40); // ex 0030D1A1 
                  l_au8Info[l_u8Size-1] = 0u;
                  sprintf(l_achData, ",%s", l_au8Info);
                  strcat(l_achDataResult, l_achData);
                  l_eIdxState = SM_SIGFOX_IDLE_WAIT;
                  /* for next use */
                  memset(l_au8Info, 0u, l_u8Size);
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
      PRINT_CUSTOM("%s\n",l_achDataResult);
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
      do {   
         /* SigFox */
         vSigFox_Process();  
         /* UART */
         vUartMngt_Process();
      
      }while(u8AT_PendingCommand() == 1u);
      g_u8SigFoxCWTest = 1u;
      PRINT_FAST("$RSL,SFC+?\n");
   }
}
static void vSigFoxCWTestCustom(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   uint32_t l_u32FreqKhz = 0u;
   uint8_t l_u8dBm = 0u;
   if(   (p_pu8Arg[6u] == ',') 
      && (p_u8Size == 10u) )
   {
      l_u32FreqKhz = strtol((char*)p_pu8Arg, NULL, 10);
      l_u8dBm = strtol((char*)&p_pu8Arg[7u], NULL, 10);
      vSigFox_TestRadio(1u,l_u32FreqKhz*1000u,l_u8dBm);
      do {   
         /* SigFox */
         vSigFox_Process();  
         /* UART */
         vUartMngt_Process();
      
      }while(u8AT_PendingCommand() == 1u);
      
      PRINT_FAST("$RSL,SFC+?\n");
      g_u8SigFoxCWTest = 1u;
   }
   else
   {
      PRINT_FAST("$RSL,SFC+0\n");
   }
}   
static void vStopSigFoxCWTest(void)
{
   if(g_u8SigFoxCWTest == 1u)
   {
      vSigFox_TestRadio(0u,0u,0u);
      do {   
         /* SigFox */
         vSigFox_Process();  
         /* UART */
         vUartMngt_Process();
      
      }while(u8AT_PendingCommand() == 1u);
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
   
//   PRINT_DEBUG("BLE Radio : Power %d, ",BLE_RADIO_TXPOWER);
//   PRINT_DEBUG("Mode %d, ",BLE_RADIO_MODE);
//   PRINT_DEBUG("Channel %d\n",g_u8BLERadioChannel);
   PRINT_FAST("$RSL,BLE+?\n");
    
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
//      PRINT_FAST("$RSL,BLE+2\n");      
   }
   else
   {
//      PRINT_FAST("$RSL,BLE+2\n");
   }
      PRINT_FAST("$RSL,BLE+1\n");      
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

//   PRINT_DEBUG("BLE Radio Channel %d\n",g_u8BLERadioChannelSweep);
   PRINT_CUSTOM("$RSL,SWP+?,CH=%d\n",g_u8BLERadioChannelSweep);
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
      eLIS2MDL_ModeSet(LIS2MDL_MODE_IDLE);
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
static void vBTL_Start(void)
{
   NRF_POWER->GPREGRET = 0xB1; 
   PRINT_FAST("$RSL,BTL+1+CLOSE_VIEWER\n");
   
   NVIC_SystemReset();
   
}

static void vHTTimeOutHandler(void * p_pvContext)
{
   g_u8StopTest = 1u;   
}

static void vHTSensorUpdateHandler(void * p_pvContext)
{
   g_u8UpdateSensor = 1u;
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
            //if((l_sACK.u16Cmd == 262) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
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
            //if(l_sACK.u16Type == 356)
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
            //if((l_sACK.u16Cmd == 257) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
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
            //if((l_sACK.u16Cmd == 353) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
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
            //if((l_sACK.u16Cmd == 314) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
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
            //if((l_sACK.u16Cmd == 386) && (l_sACK.eAck == PMTK_ACK_VALID_PCK_ACT_SUCCEEDED))
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

static uint8_t u8BME_SingleShotRead(float *p_pfT, float *p_pfP, float *p_pfH)
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
               (*p_pfT) = (p_pfT != NULL)? l_f32Data:NULL;
//               PRINT_DEBUG("T : %d.",(int32_t)(l_f32Data));
//               PRINT_DEBUG("%d �C, ",(int32_t)((l_f32Data-((int32_t)l_f32Data))*100.0f));
            }
            else if(l_eErr == BME280_ERROR_INVALID)
            {
               l_u8Error = 1u;
            }
            
            l_eErr = eBME280_PressureGet(&l_f32Data);
            if(l_eErr == BME280_ERROR_NONE)
            {
               (*p_pfP) = (p_pfP != NULL)? l_f32Data:NULL;
//               PRINT_DEBUG("P : %d.",(int32_t)(l_f32Data));
//               PRINT_DEBUG("%d hPa, ",(int32_t)((l_f32Data-((int32_t)l_f32Data))*10.0f));
            }
            else if(l_eErr == BME280_ERROR_INVALID)
            {
               l_u8Error = 1u;
            }
            
            l_eErr = eBME280_HumidityGet(&l_f32Data);
            if(l_eErr == BME280_ERROR_NONE)
            {
               (*p_pfH) = (p_pfH != NULL)? l_f32Data:NULL;
//               PRINT_DEBUG("H : %d.",(int32_t)(l_f32Data));
//               PRINT_DEBUG("%d %%\n",(int32_t)((l_f32Data-((int32_t)l_f32Data))*10.0f));
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
      
   return l_u8Error;
}


static void vINT_Configure(void)
{
#if (EN_BME280 == 1)   
#endif
   
#if (EN_MAX44009 == 1)
   uint8_t l_u8Status = 0u;
   (void)eMAX44009_InterruptStatusGet(&l_u8Status);
   (void)eMAX44009_InterruptCfg(1u, 5000u, 50u, 1000u);
#endif
#if (EN_LSM6DSL == 1)

#endif
#if (EN_LIS2MDL == 1)
   (void)eLIS2MDL_ThresholdSet(800u);
   (void)eLIS2MDL_InterruptCtrlSet(1u, LIS2MDL_INT_AXIS_XYZ, 1u, 0u);
#endif

#if (EN_VEML6075 == 1)

#endif
   
#if (EN_ST25DV == 1)

#endif
   
#if (EN_LTC2943 == 1)
#endif

}

static void vINT_Clear(void)
{
   uint8_t l_u8Status = 0u;
#if (EN_BME280 == 1)   
#endif   
#if (EN_MAX44009 == 1)
   (void)eMAX44009_InterruptStatusGet(&l_u8Status);
#endif
#if (EN_LSM6DSL == 1)
#endif
#if (EN_LIS2MDL == 1)
   uint8_t l_u8Int = 0u;
   int8_t l_s8XAxis = 0;
   int8_t l_s8YAxis = 0;
   int8_t l_s8ZAxis = 0;
   (void)eLIS2MDL_InterruptStatusGet(&l_s8XAxis, &l_s8YAxis, &l_s8ZAxis, &l_u8Int);
#endif
#if (EN_VEML6075 == 1)
#endif   
#if (EN_ST25DV == 1)
#endif   
#if (EN_LTC2943 == 1)
#endif
   
}
static void vCfgSensor(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   uint8_t l_u8Len = strlen((char*)p_pu8Arg);
   
   if(l_u8Len >= 6u)
   {
      if((uint8_t*)strstr((char*)p_pu8Arg, ",") == (&p_pu8Arg[3u]))
      {
         if(strstr((char*)p_pu8Arg, "BME") != NULL)
         {
            PRINT_CUSTOM("$ACK,CFG+%s\n","1");
            l_u8Len = strlen((char*)&p_pu8Arg[4u]);
            vCfgBME(&p_pu8Arg[4u], l_u8Len);
         }
         else if(strstr((char*)p_pu8Arg, "ADX") != NULL)
         {
            PRINT_CUSTOM("$ACK,CFG+%s\n","1");
            l_u8Len = strlen((char*)&p_pu8Arg[4u]);
            vCfgADX(&p_pu8Arg[4u], l_u8Len);
         }
         else if(strstr((char*)p_pu8Arg, "LIS") != NULL)
         {
            PRINT_CUSTOM("$ACK,CFG+%s\n","1");
            l_u8Len = strlen((char*)&p_pu8Arg[4u]);
            vCfgLIS(&p_pu8Arg[4u], l_u8Len);
         }
         else if(strstr((char*)p_pu8Arg, "LSM") != NULL)
         {
            PRINT_CUSTOM("$ACK,CFG+%s\n","1");
            l_u8Len = strlen((char*)&p_pu8Arg[4u]);
            vCfgLSM(&p_pu8Arg[4u], l_u8Len);
         }
         else if(strstr((char*)p_pu8Arg, "ORG") != NULL)
         {
            if(g_u8TestInProgress == 1u)
            {
               PRINT_CUSTOM("$ACK,CFG+%s\n","1");
               l_u8Len = strlen((char*)&p_pu8Arg[4u]);
               vCfgORG(&p_pu8Arg[4u], l_u8Len);
            }
            else
            {
               PRINT_CUSTOM("$ACK,CFG+%s\n","0");
            }
         }
         else
         {
            PRINT_CUSTOM("$ACK,CFG+%s\n","0");
         }
      }
   }
   else if(l_u8Len == 4u)
   {
      if(strstr((char*)p_pu8Arg, "BME") != NULL)
      {
         PRINT_CUSTOM("$ACK,CFG+%s\n","1");
         vCfgBME(NULL, 0u);
      }
      else if(strstr((char*)p_pu8Arg, "ADX") != NULL)
      {
         PRINT_CUSTOM("$ACK,CFG+%s\n","1");
         vCfgADX(NULL, 0u);
      }
      else if(strstr((char*)p_pu8Arg, "LIS") != NULL)
      {
         PRINT_CUSTOM("$ACK,CFG+%s\n","1");
         vCfgLIS(NULL, 0u);
      }
      else if(strstr((char*)p_pu8Arg, "LSM") != NULL)
      {
         PRINT_CUSTOM("$ACK,CFG+%s\n","1");
         vCfgLSM(NULL, 0u);
      }
      else
      {
         PRINT_CUSTOM("$ACK,CFG+%s\n","0");
      }
   }
   else
   {
      PRINT_CUSTOM("$ACK,CFG+%s\n","0");
   }
}

static void vCfgBME(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   if(p_u8Size != 0u)
   {
      switch(p_pu8Arg[0u])
      {
         case '1':
            (void)eBME280_OSRTemperatureSet(BME280_OVERSAMPLING_2X);
            (void)eBME280_OSRPressureSet(BME280_OVERSAMPLING_16X);
            (void)eBME280_OSRHumiditySet(BME280_OVERSAMPLING_OFF);
            (void)eBME280_ModeSet(BME280_FORCED);
            break;
         case '2':
            (void)eBME280_OSRTemperatureSet(BME280_OVERSAMPLING_1X);
            (void)eBME280_OSRPressureSet(BME280_OVERSAMPLING_OFF);
            (void)eBME280_OSRHumiditySet(BME280_OVERSAMPLING_16X);
            (void)eBME280_ModeSet(BME280_FORCED);
            break;
         default:
         case '0':
            (void)eBME280_OSRTemperatureSet(BME280_OVERSAMPLING_1X);
            (void)eBME280_OSRPressureSet(BME280_OVERSAMPLING_1X);
            (void)eBME280_OSRHumiditySet(BME280_OVERSAMPLING_1X);
            (void)eBME280_ModeSet(BME280_FORCED);
            break;
      }
      
   }
   else
   {
      eBME280_TPHRead();
   }
   PRINT_CUSTOM("$RSL,CFG,BME,%c+%s\n",(char)p_pu8Arg[0u],"1");
}
static void vCfgADX(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   if(p_u8Size != 0u)
   {
      switch(p_pu8Arg[0u])
      {
         case '1':
            (void)eADXL362_MeasureModeSet(ADXL362_MEASURE_ON);
            (void)eADXL362_WakeUpModeSet(ADXL362_WAKEUP_ON);
            break;
         case '2':
            (void)eADXL362_WakeUpModeSet(ADXL362_WAKEUP_OFF);
            (void)eADXL362_MeasureModeSet(ADXL362_MEASURE_ON);
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_12_5_HZ);
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_NORMAL);
            break;
         case '3':
            (void)eADXL362_WakeUpModeSet(ADXL362_WAKEUP_OFF);
            (void)eADXL362_MeasureModeSet(ADXL362_MEASURE_ON);
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_100_HZ);
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_NORMAL);
            break;
         case '4':
            (void)eADXL362_WakeUpModeSet(ADXL362_WAKEUP_OFF);
            (void)eADXL362_MeasureModeSet(ADXL362_MEASURE_ON);
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_100_HZ);
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_LOW);
            break;
         case '5':
            (void)eADXL362_WakeUpModeSet(ADXL362_WAKEUP_OFF);
            (void)eADXL362_MeasureModeSet(ADXL362_MEASURE_ON);
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_100_HZ);
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_ULTRALOW);
            break;
         case 'a':
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_NORMAL);
            break;
         case 'b':
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_LOW);
            break;
         case 'c':
            eADXL362_NoiseModeSet(ADXL362_NOISE_MODE_ULTRALOW);
            break;
         case 'd':
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_12_5_HZ);
            break;
         case 'e':
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_100_HZ);
            break;
         case 'f':
            (void)eADXL362_OutputDataRateSet(ADXL362_ODR_400_HZ);
            break;
         default:
         case '0':
            (void)eADXL362_Init();
            (void)eADXL362_MeasureModeSet(ADXL362_MEASURE_STANDBY);
            (void)eADXL362_WakeUpModeSet(ADXL362_WAKEUP_OFF);
            break;
      }
   }
   else
   {
      (void)eADXL362_AccelerationRead();
   }
   PRINT_CUSTOM("$RSL,CFG,ADX,%c+%s\n",(char)p_pu8Arg[0u],"1");
}
static void vCfgLIS(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   if(p_u8Size != 0u)
   {
      switch(p_pu8Arg[0u])
      {
         case '1':
            (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);  
            (void)eLIS2MDL_LowPower(1u);   
            (void)eLIS2MDL_HardIronOffsetEnable(0u);        
            break;
         case '2':
            (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
            (void)eLIS2MDL_LowPower(1u); 
            (void)eLIS2MDL_HardIronOffsetEnable(1u);
            break;
         case '3':
            (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
            (void)eLIS2MDL_LowPower(0u); 
            (void)eLIS2MDL_HardIronOffsetEnable(0u);
            break;
         case '4':
            (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_SINGLE_SHOT);
            (void)eLIS2MDL_LowPower(0u);
            (void)eLIS2MDL_HardIronOffsetEnable(1u);
            break;
         default:
         case '0':
            (void)eLIS2MDL_LowPower(1u);      
            (void)eLIS2MDL_OutputDataRateSet(LIS2MDL_ODR_10Hz);
            (void)eLIS2MDL_ModeSet(LIS2MDL_MODE_IDLE);
            break;
      }
   }
   else
   {
      eLIS2MDL_MagneticRead();
   }
   PRINT_CUSTOM("$RSL,CFG,LIS,%c+%s\n",(char)p_pu8Arg[0u],"1");
}
static void vCfgLSM(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   static uint8_t l_u8GyroEN = 0u;
   if(p_u8Size != 0u)
   {
      switch(p_pu8Arg[0u])
      {
         case '1':
            (void)eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_1_6Hz, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
            (void)eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
            l_u8GyroEN = 0u;
            break;
         case '2':
            (void)eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_52Hz, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
            (void)eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_52Hz, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER);
            l_u8GyroEN = 1u;
            break;
         default:
         case '0':
            (void)eLSM6DSL_AccelCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_ACCEL_RANGE_2G, LSM6DSL_MODE_LOW_POWER);
            (void)eLSM6DSL_GyroCfgSet(LSM6DSL_ODR_POWER_DOWN, LSM6DSL_GYRO_RANGE_250DPS, LSM6DSL_MODE_LOW_POWER); 
            l_u8GyroEN = 0u;
            break;
      }
   }
   else
   {
      (void)eLSM6DSL_AccelRead();
      if(l_u8GyroEN != 0u)
      {
         (void)eLSM6DSL_GyroRead();
      }
   }
   PRINT_CUSTOM("$RSL,CFG,LSM,%c+%s\n",(char)p_pu8Arg[0u],"1");
}

static void vCfgORG(uint8_t * p_pu8Arg, uint8_t p_u8Size)
{
   if(p_u8Size != 0u)
   {
      switch(p_pu8Arg[0u])
      {
         case '0':
            vORG1510_Constellation(1,0,0,0);
            break;
         case '1':
            vORG1510_Constellation(1,1,1,0);
            break;
         case 'o':
            vHal_GPIO_Set(GPS_POWER_EN);            
            break;
         case 'f':
            vHal_GPIO_Clear(GPS_POWER_EN);            
            break;
         case 'i':
            vORG1510_Init(g_sORG1510Context);
            eUartMngt_StateSet(USM_GPS);
            break;
         case 'u':
            eUartMngt_StateSet(USM_IDLE);
            break;
         case 'b':
            vHal_GPIO_Clear(GPS_RST);  
            break;
         case 'c':
            vHal_GPIO_Set(GPS_RST);  
            break;
         default:
            break;
      }
   }      
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


