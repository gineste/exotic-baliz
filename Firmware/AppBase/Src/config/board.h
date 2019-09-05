/* 
 *  ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 * (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *  ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
 * (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef BOARD_H
#define BOARD_H

/************************************************************************
 * Include Files
 ************************************************************************/

/************************************************************************
 * Defines
 ************************************************************************/
#if (BALIZ_V == 2)
   #define GPS_POWER_EN       24u
   #define GPS_RST            18u
   #define GPS_ON             17u
   #define GPS_UART_TX        22u
   #define GPS_UART_RX        20u

   #define SIGFOX_RST         25u
   #define SIGFOX_DP_WU       28u
   #define SIGFOX_UART_RX     27u
   #define SIGFOX_UART_TX     26u

   #define SPI_SCLK           12u
   #define SPI_MOSI           11u
   #define SPI_MISO           10u

   #define I2C_SDA            14u
   #define I2C_SCL            15u
   /*** define CONFIG_NFCT_PINS_AS_GPIOS for next 2 IOs */
   #define ADXL_CS            9u
   #define ADXL_INT2          8u
   /***/
   #define ADXL_POWER_EN      13u

   #define ST25DV_GPO_INT     5u

   #define MAX44009_INT       19u

   #define LSM6_INT1          3u
   #define LSM6_INT2          4u

   #define LIS2_INT           16u

   #define LEDR_PIN			   29u
   #define LEDG_PIN			   31u
   #define LEDB_PIN			   30u

   /*** define CONFIG_GPIO_AS_PINRESET for next IO if populated*/
   #define BP1				      21u

//   #define BUZZER_PIN		   23u

#elif (BALIZ_V == 3)
   #define GPS_POWER_EN       26u
   #define GPS_BACKUP         30u
   #define GPS_ON             27u
   #define GPS_UART_TX        29u
   #define GPS_UART_RX        28u

   #define SIGFOX_RST         14u
   #define SIGFOX_DP_WU       17u
   #define SIGFOX_UART_RX     16u
   #define SIGFOX_UART_TX     15u

   #define SPI_SCLK           10u
   /*** define CONFIG_NFCT_PINS_AS_GPIOS for next 2 IOs */
   #define SPI_MOSI           9u
   #define SPI_MISO           8u

   #define I2C_SDA            19u
   #define I2C_SCL            20u
   
   #define ADXL_CS            7u
   #define ADXL_INT1          5u
   #define ADXL_INT2          6u
   /***/
   #define ADXL_POWER_EN      4u

   #define ST25DV_GPO_INT     3u
   #define ST25DV_LPD         2u

   #define MAX44009_INT       31u

   #define LSM6_INT1          13u
   #define LSM6_INT2          12u

   #define LIS2_INT           18u

   #define LEDR_PIN			   22u
   #define LEDG_PIN			   23u
   #define LEDB_PIN			   24u

   #define GAUGE_ALERT        25u
   
   #define SDCARD_CS          11u
   
   #define BUZZER_PIN		   0xFF
   /*** undefine CONFIG_GPIO_AS_PINRESET for next IO if pin 21 is not used */
#else
   #error "Board version not defined"
#endif


#define BUTTONS_NUMBER     0

#define BSP_LED_0          LEDR_PIN
#define BSP_LED_1          LEDG_PIN
#define BSP_LED_2          LEDB_PIN

#define LEDS_NUMBER        3

#define LEDS_ACTIVE_STATE  0

#define LEDS_LIST          { BSP_LED_0, BSP_LED_1, BSP_LED_2 }

#define BSP_BUZZER_MASK    (1u<<BUZZER_PIN)

/************************************************************************
 * Type definitions
 ************************************************************************/

/************************************************************************
 * Public function declarations
 ************************************************************************/
 
#endif /* BOARD_H */

