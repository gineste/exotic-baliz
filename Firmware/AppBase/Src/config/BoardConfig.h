/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( ( oO )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 */
#ifndef BOARDCONFIG_H
#define BOARDCONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>\n
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
// <h> Sensors Configuration
//==========================================================
// <q> BME280 Present
// <i> The Bosch BME280 sensor measures temperature, pressure and humidity data.
#ifndef EN_BME280
#define EN_BME280 1
#endif
// <q> MAX44009 Present
// <i> The MAX44009 sensor measures brightness data.
#ifndef EN_MAX44009
#define EN_MAX44009 1
#endif
// <q> ADXL362 Present
// <i> The ADXL362 sensor measures acceleration data on x, y and z axis.
#ifndef EN_ADXL362
#define EN_ADXL362 1
#endif
// <q> ORG1510 Present
// <i> The ORG1510 is a GPS sensor. It provides location information.
#ifndef EN_ORG1510
#define EN_ORG1510 1
#endif
// <q> LSM6DSL Present
// <i> The LSM6DSL is a inertial sensor. It is an accelerometer, gyroscope sensor on x, y and z axis.
#ifndef EN_LSM6DSL
#define EN_LSM6DSL 1
#endif
// <q> LIS2MDL Present
// <i> The LIS2MDL is a mangetometer on 3 axis.
#ifndef EN_LIS2MDL
#define EN_LIS2MDL 1
#endif
// <q> VEML6075 Present
// <i> The VEML6075 is an UV sensor. 
#ifndef EN_VEML6075
#define EN_VEML6075 0
#endif
// <q> LTC243X Present
// <i> The LTC2943 is a gauge sensor. It provides information on battery.
#ifndef EN_LTC2943
#define EN_LTC2943 0
#endif
// <q> ST25DV Present
// <i> The ST25DV is an EEPROM/NFC. It provides storage support.
#ifndef EN_ST25DV
#define EN_ST25DV 1
#endif

// <q> MAX17205 Present
// <i> The MAX17205 is a gauge sensor. It provides information on battery.
#ifndef EN_MAX17205
#define EN_MAX17205 0
#endif
#ifndef SENSOR_NUMBER
#define SENSOR_NUMBER ( EN_LTC2943     +  \
                        EN_VEML6075    +  \
                        EN_LSM9DS1     +  \
                        EN_ORG1510     +  \
                        EN_ADXL362     +  \
                        EN_MAX44009    +  \
                        EN_LSM6DSL     +  \
                        EN_LIS2MDL     +  \
                        EN_BME280 )
#endif
// </h>

//==========================================================
// <h> BLE Configuration
//==========================================================
// <e.0> BLE Enabled
// <i> ENABLE_BLE
#ifndef ENABLE_BLE
#define ENABLE_BLE 1
#endif
// <o> Fast Advertise Interval - mseconds <10-1000:5>
// <i> FAST_ADV_INTERVAL_BLE
#ifndef FAST_ADV_INTERVAL_BLE
#define FAST_ADV_INTERVAL_BLE 250
#endif
#define FAST_ADV_INT_BLE_COMP (FAST_ADV_INTERVAL_BLE<<3)/5
// <o> Fast Advertise Time out - seconds <0-500:5>
// <i> Timeout between 0 sec(unlimited) and 600 sec with step of 5 sec
// <i> FAST_ADV_TIMEOUT_BLE
#ifndef FAST_ADV_TIMEOUT_BLE
#define FAST_ADV_TIMEOUT_BLE 120
#endif
// <o> Slow Advertise Interval - mseconds <500-10000:5>
#ifndef SLOW_ADV_INTERVAL_BLE
#define SLOW_ADV_INTERVAL_BLE 1000
#endif
#define SLOW_ADV_INT_BLE_COMP (SLOW_ADV_INTERVAL_BLE<<3)/5
// <o> Slow Advertise Time out - seconds <0-3600:5>
// <i> Timeout between  0 sec(unlimited) and 1h with step of 5 sec
// <i> SLOW_ADV_TIMEOUT_BLE
#ifndef SLOW_ADV_TIMEOUT_BLE
#define SLOW_ADV_TIMEOUT_BLE 120
#endif
// <o> WakeUp BLE after end of advertising Time out - seconds <00-600:10>
// <i> Timeout between 0 sec(unlimited) and 10 min with step of 10 sec
// <i> WAKE_ADV_TIMEOUT_BLE
#ifndef WAKE_ADV_TIMEOUT_BLE
#define WAKE_ADV_TIMEOUT_BLE 0
#endif
// <s> Default Prefix BLE Name
// <i> Always start with PREFIX "SF_"
// <i> DEFAULT_PREFIX_BLE_DEVICE_NAME
#ifndef DEFAULT_PREFIX_BLE_DEVICE_NAME
#define DEFAULT_PREFIX_BLE_DEVICE_NAME "HT_"
#endif
// <s> Default BLE Name
// <i> DEFAULT_BLE_DEVICE_NAME
#ifndef DEFAULT_BLE_DEVICE_NAME
#define DEFAULT_BLE_DEVICE_NAME ""
#endif
// </e> BLE DISABLED
// </h> 
//==========================================================

// <h> Modules Configuration
//==========================================================
// <o> Board Type
// <i> BOARD_TYPE
// <i> Select board type for specific advertise/Ext update (Baliz, Graal, SID)
// <0=> None 
// <1=> Baliz
// <2=> Graal
// <3=> SID
// <4=> other
#ifndef BOARD_TYPE
#define BOARD_TYPE 1
#endif
// <o> Use Case number <0-255:1>
// <i> USE_CASE_NUMBER
#ifndef USE_CASE_NUMBER
#define USE_CASE_NUMBER 255
#endif
// <q> LED Enabled
// <i> Check it if you want to disable LED function(blink/fade).
// <i> ENABLE_LED
#ifndef ENABLE_LED
#define ENABLE_LED 1
#endif
// <q> Use Button
// <i> ENABLE_BUTTON
#ifndef ENABLE_BUTTON
#define ENABLE_BUTTON 0
#endif

// <q> Use Buzzer
// <i> ENABLE_BUZZER
#ifndef ENABLE_BUZZER
#define ENABLE_BUZZER 1
#endif
// </h> 
//==========================================================

// <h> Debug Configuration
//==========================================================
// <e> Enable LOG
// <i> EN_LOG
#ifndef EN_LOG
#define EN_LOG 1
#endif
// <q> Log BLE Transmit
// <i> LOG_BLE_TX
#ifndef LOG_BLE_TX
#define LOG_BLE_TX 0
#endif
// <q> Log BLE Received
// <i> LOG_BLE_RX
#ifndef LOG_BLE_RX
#define LOG_BLE_RX 0
#endif

// <q> Log Uart
// <i> LOG_UART
#ifndef LOG_UART
#define LOG_UART 0
#endif
// <q> Log SigFox
// <i> LOG_SIGFOX
#ifndef LOG_SIGFOX
#define LOG_SIGFOX 0
#endif

// <q> Log GPS
// <i> LOG_GPS
#ifndef LOG_GPS
#define LOG_GPS 1
#endif
// </e>

#if (EN_LOG == 0)
   #undef LOG_BLE_RX
   #define LOG_BLE_RX 0
   #undef LOG_BLE_TX
   #define LOG_BLE_TX 0
   #undef LOG_UART
   #define LOG_UART 0
#endif

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
 
// <<< end of configuration section >>>
#endif /* BOARDCONFIG_H */

