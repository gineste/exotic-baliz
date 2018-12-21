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
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <nrf_gpio.h>
#include <nrf_gpiote.h>

#include "boards.h"

/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _PIN_PULL_ {
   HALGPIO_PIN_NOPULL = NRF_GPIO_PIN_NOPULL,       /* Pin pullup resistor disabled  */
   HALGPIO_PIN_PULLDOWN = NRF_GPIO_PIN_PULLDOWN,   /* Pin pulldown resistor enabled */
   HALGPIO_PIN_PULLUP = NRF_GPIO_PIN_PULLUP,       /* Pin pullup resistor enabled   */
}e_HalGPIO_PinPull_t;

typedef enum _PIN_DIR_ {
   HALGPIO_PINDIR_INPUT = NRF_GPIO_PIN_DIR_INPUT,     /* Pin direction Input  */
   HALGPIO_PINDIR_OUTPUT = NRF_GPIO_PIN_DIR_OUTPUT,   /* Pin direction Output */
}e_HalGPIO_PinDir_t;

typedef enum _PIN_IN_CON_ {
   HALGPIO_PININ_CONNECT = NRF_GPIO_PIN_INPUT_CONNECT,        /* Pin Input Connected   */
   HALGPIO_PININ_DISCONNECT = NRF_GPIO_PIN_INPUT_DISCONNECT,  /* Pin Input Disconnect  */
}e_HalGPIO_PinInConn_t;

typedef enum _PIN_DRIVE_ {
   HALGPIO_PINDRV_S0S1 = NRF_GPIO_PIN_S0S1,     /* Standard '0', standard '1'       */
   HALGPIO_PINDRV_H0S1 = NRF_GPIO_PIN_H0S1,     /* High drive '0', standard '1'     */
   HALGPIO_PINDRV_S0H1 = NRF_GPIO_PIN_S0H1,     /* Standard '0', high drive '1'     */
   HALGPIO_PINDRV_H0H1 = NRF_GPIO_PIN_H0H1,     /* High drive '0', high 'drive '1'' */
   HALGPIO_PINDRV_D0S1 = NRF_GPIO_PIN_D0S1,     /* Disconnect '0' standard '1'      */
   HALGPIO_PINDRV_D0H1 = NRF_GPIO_PIN_D0H1,     /* Disconnect '0', high drive '1'   */
   HALGPIO_PINDRV_S0D1 = NRF_GPIO_PIN_S0D1,     /* Standard '0'. disconnect '1'     */
   HALGPIO_PINDRV_H0D1 = NRF_GPIO_PIN_H0D1,     /* High drive '0', disconnect '1'   */
}e_HalGPIO_PinDrive_t;

typedef enum _PIN_SENSE_ {
   HALGPIO_PIN_NOSENSE = NRF_GPIO_PIN_NOSENSE,       /*  Pin sense level disabled.  */
   HALGPIO_PIN_SENSELOW = NRF_GPIO_PIN_SENSE_LOW,    /*  Pin sense low level.       */
   HALGPIO_PIN_SENSEHIGH = NRF_GPIO_PIN_SENSE_HIGH,  /*  Pin sense high level.      */
}e_HalGPIO_PinSense_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vHal_GPIO_Init(void);
void vHal_GPIO_Cfg(uint32_t p_u32Pin, e_HalGPIO_PinDir_t p_ePinDir, e_HalGPIO_PinInConn_t p_ePinInConn,
               e_HalGPIO_PinPull_t p_ePinPull, e_HalGPIO_PinDrive_t p_ePinDrive, e_HalGPIO_PinSense_t p_ePinSense);

__STATIC_INLINE void vHal_GPIO_Set(uint32_t p_u32Pin);
__STATIC_INLINE void vHal_GPIO_Clear(uint32_t p_u32Pin);
__STATIC_INLINE uint32_t u32Hal_GPIO_Read(uint32_t p_u32Pin);
__STATIC_INLINE void vGPIO_InputCfg(uint32_t p_u32Pin, e_HalGPIO_PinPull_t p_ePinPull);
__STATIC_INLINE void vGPIO_OutputCfg(uint32_t p_u32Pin, e_HalGPIO_PinPull_t p_ePinPull);

/************************************************************************
 * Inline Public functions
 ************************************************************************/
/**@brief Function for setting pin to 1. 
 * @param[in]  p_u32Pin : IO pin number 
 * @return None
 */
__STATIC_INLINE void vHal_GPIO_Set(uint32_t p_u32Pin)
{
   nrf_gpio_pin_set(p_u32Pin);
}

/**@brief Function for clearing pin to 0. 
 * @param[in]  p_u32Pin : IO pin number 
 * @return None
 */
__STATIC_INLINE void vHal_GPIO_Clear(uint32_t p_u32Pin)
{
   nrf_gpio_pin_clear(p_u32Pin);
}

/**@brief Function for read pin. 
 * @param[in]  p_u32Pin : IO pin number 
 * @return 0 if voltage null, else positive value
 */
__STATIC_INLINE uint32_t u32Hal_GPIO_Read(uint32_t p_u32Pin)
{
   return nrf_gpio_pin_read(p_u32Pin);
}

/**@brief Function for read pin. 
 * @param[in]  p_u32Pin : IO pin number 
 * @param[in]  p_ePinPull : Pull Up/Down/Nothing
 * @return None
 */
__STATIC_INLINE void vGPIO_InputCfg(uint32_t p_u32Pin, e_HalGPIO_PinPull_t p_ePinPull)
{
   nrf_gpio_cfg_input(p_u32Pin, (nrf_gpio_pin_pull_t)p_ePinPull);
}

/**@brief Function to configure Output pin. 
 * @param[in]  p_u32Pin : IO pin number 
 * @param[in]  p_ePinPull : Pull Up/Down/Nothing
 * @return None
 */
__STATIC_INLINE void vGPIO_OutputCfg(uint32_t p_u32Pin, e_HalGPIO_PinPull_t p_ePinPull)
{   
   nrf_gpio_cfg(p_u32Pin,NRF_GPIO_PIN_DIR_OUTPUT,NRF_GPIO_PIN_INPUT_DISCONNECT,
        (nrf_gpio_pin_pull_t)p_ePinPull,NRF_GPIO_PIN_S0S1,NRF_GPIO_PIN_NOSENSE);
}


#endif // HAL_GPIO_H

