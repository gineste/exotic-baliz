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
 * File name:		Hal_GPIO.c
 * Date:			   27 06 2017(dd MM YYYY)
 * Author:			Yoann Rebischung
 * Description:	Hardware Abstraction Layer for GPIO configuration
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <nrf_delay.h>
 
#include "boards.h"

#include "HAL_GPIO.h"

/************************************************************************
 * Defines
 ************************************************************************/

/************************************************************************
 * Private type declarations
 ************************************************************************/

/************************************************************************
 * Private function declarations
 ************************************************************************/

/************************************************************************
 * Variable declarations
 ************************************************************************/

/************************************************************************
 * Public functions
 ************************************************************************/  
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
void vHal_GPIO_Init(void)
{   
   /* Init IO here */
   /* Power switch off GPS */
	vGPIO_OutputCfg(GPS_POWER_EN, HALGPIO_PIN_NOPULL); 
   vHal_GPIO_Clear(GPS_POWER_EN);      
   /* Power switch off ADXL */
	vGPIO_OutputCfg(ADXL_POWER_EN, HALGPIO_PIN_NOPULL); 
   vHal_GPIO_Clear(ADXL_POWER_EN);     
   
   /* LEDs */
   vGPIO_OutputCfg(LEDR_PIN, HALGPIO_PIN_NOPULL);
	vHal_GPIO_Set(LEDR_PIN);
   vGPIO_OutputCfg(LEDG_PIN, HALGPIO_PIN_NOPULL);
	vHal_GPIO_Set(LEDG_PIN);
   vGPIO_OutputCfg(LEDB_PIN, HALGPIO_PIN_NOPULL);
	vHal_GPIO_Set(LEDB_PIN);
   
   /* GPS */
//   vGPIO_OutputCfg(GPS_ON, HALGPIO_PIN_NOPULL);
//   vHal_GPIO_Clear(GPS_ON);
//   vGPIO_OutputCfg(GPS_RST, HALGPIO_PIN_NOPULL);
//   vHal_GPIO_Set(GPS_RST);
   
//   vGPIO_OutputCfg(SIGFOX_RST, HALGPIO_PIN_NOPULL);
//   vGPIO_OutputCfg(SIGFOX_DP_WU, HALGPIO_PIN_NOPULL);
//   vHal_GPIO_Set(SIGFOX_RST);	   /* Must be connected to VDD */
//   vHal_GPIO_Set(SIGFOX_DP_WU);
   
   /* UART GPS */
//	vGPIO_InputCfg(GPS_RX, HALGPIO_PIN_NOPULL);
//	vGPIO_OutputCfg(GPS_TX, HALGPIO_PIN_NOPULL);

   /* IMU LSM6DSL */
//	vGPIO_InputCfg(LSM6_INT1, HALGPIO_PIN_NOPULL);
//	vGPIO_InputCfg(LSM6_INT2, HALGPIO_PIN_NOPULL);
   
   /* TWI */
//   vGPIO_InputCfg(I2C_SDA, HALGPIO_PIN_NOPULL);
//   vGPIO_InputCfg(I2C_SCL, HALGPIO_PIN_NOPULL); 
   
   /* SPI */
//   vGPIO_OutputCfg(SPI_SCLK, HALGPIO_PIN_NOPULL);
//   vGPIO_OutputCfg(SPI_MOSI, HALGPIO_PIN_NOPULL);
//   vGPIO_InputCfg(SPI_MISO, HALGPIO_PIN_NOPULL);
   
}

void vHal_GPIO_Cfg(uint32_t p_u32Pin, e_HalGPIO_PinDir_t p_ePinDir, e_HalGPIO_PinInConn_t p_ePinInConn,
               e_HalGPIO_PinPull_t p_ePinPull, e_HalGPIO_PinDrive_t p_ePinDrive, e_HalGPIO_PinSense_t p_ePinSense)
{
   if(p_ePinDir == HALGPIO_PINDIR_OUTPUT)
   {
      p_ePinInConn = HALGPIO_PININ_DISCONNECT;
   }
   
   nrf_gpio_cfg(p_u32Pin,(nrf_gpio_pin_dir_t)p_ePinDir,(nrf_gpio_pin_input_t)p_ePinInConn,
        (nrf_gpio_pin_pull_t)p_ePinPull,(nrf_gpio_pin_drive_t)p_ePinDrive,(nrf_gpio_pin_sense_t)p_ePinSense);
   
}
/************************************************************************
 * Private functions
 ************************************************************************/

/************************************************************************
 * End of File
 ************************************************************************/

