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

#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

/************************************************************************
 * Include Files
 ************************************************************************/
 
/************************************************************************
 * Defines
 ************************************************************************/
#define LEDR_PIN			25u
#define LEDG_PIN			27u
#define LEDB_PIN			26u

#define BSP_LED_0       LEDR_PIN
#define BSP_LED_1       LEDG_PIN
#define BSP_LED_2       LEDB_PIN

/*#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)
*/
#define BP1					12u
#define BP2					11u

#define SDA_PIN         20u
#define SCL_PIN         21u

#define ADXL_SCLK       10u
#define ADXL_MOSI       9u
#define ADXL_MISO       8u
#define ADXL_CS         7u
#define ADXL_INT1       6u
#define ADXL_INT2       15u

#define GPS_ON          28u
#define GPS_RST         29u

#define DEN_A_G         2u	// Accelerometer and gyroscope data enable

#define BOOST_SHDN		14u

#define AUDIO_SHDN		13u

#define BUZZER_PIN		22u
#define BSP_BUZZER_MASK (1<<BUZZER_PIN)

#define SIGFOX_TX       30u
#define SIGFOX_RX       31u

/************************************************************************
 * Type definitions
 ************************************************************************/
 
/************************************************************************
 * Public function declarations
 ************************************************************************/
 
#endif /* CUSTOM_BOARD_H */

