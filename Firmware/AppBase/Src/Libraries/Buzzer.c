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
 * File name:		Buzzer.c
 * Date:			   12 06 2017
 * Author:			Yoann R
 * Description:   Buzzer library for SuperFrog 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <nrf_pwm.h>
#include "BoardConfig.h"

#include "board.h"
#include "nordic_common.h"
#include "nrf_drv_pwm.h"
#include "app_util.h" 

#include "HAL/HAL_GPIO.h"

#include "Buzzer.h"

/************************************************************************
 * Defines
 ************************************************************************/
#define PWM_USED		0 // Do not add u for type here since the macro concat

#define BUZ_RESONNANT_FREQ     2730u
#define BUZ_CONFIG_BASE_CLOCK  NRF_PWM_CLK_8MHz
#define BUZ_CONFIG_TOP_VALUE   2930u /* = (BUZ_CONFIG_BASE_CLOCK/BUZ_RESONNANT_FREQ) */

/************************************************************************
 * Private type declarations
 ************************************************************************/
 
/************************************************************************
 * Private function declarations
 ************************************************************************/
 
/************************************************************************
 * Variable declarations
 ************************************************************************/
 #if (ENABLE_BUZZER == 1)
 static nrf_drv_pwm_t g_sPWMBuzzer = NRF_DRV_PWM_INSTANCE(PWM_USED);
 nrf_drv_pwm_config_t g_sConfigBuzzer0 = NRF_DRV_PWM_DEFAULT_CONFIG;

const uint8_t g_cu8SinTable[] = {
   0,0,1,2,4,6,9,12,16,20,24,29,35,40,46,53,59,66,74,81,88,96,104,112,120,
   128,136,144,152,160,168,175,182,190,197,203,210,216,221,227,232,236,240,
   244,247,250,252,254,255,255,255,255,255,254,252,250,247,244,240,236,232,
   227,221,216,210,203,197,190,182,175,168,160,152,144,136,128,120,112,104,
   96,88,81,74,66,59,53,46,40,35,29,24,20,16,12,9,6,4,2,1,0 
};

static nrf_pwm_values_common_t g_sSequenceAct[] =
{	/*Duty Cyle Sequence*/    
    250,0
};
nrf_pwm_sequence_t const g_sBuzSeqAct =
{
    .values.p_common = g_sSequenceAct,
    .length          = NRF_PWM_VALUES_LENGTH(g_sSequenceAct),
    .repeats         = 200u,
    .end_delay       = 0u
};
static nrf_pwm_values_common_t g_sSequenceInAct[] =
{	/*Duty Cyle Sequence*/
    250,0
};
nrf_pwm_sequence_t const g_sBuzSeqInAct =
{
    .values.p_common = g_sSequenceInAct,
    .length          = NRF_PWM_VALUES_LENGTH(g_sSequenceInAct),
    .repeats         = 500u,
    .end_delay       = 0u
};

static nrf_pwm_values_common_t g_sSequenceInitOk[] =
{	/*Duty Cyle Sequence*/
   250,0,250,0
};
nrf_pwm_sequence_t const g_sBuzSeqInitOk =
{
    .values.p_common = g_sSequenceInitOk,
    .length          = NRF_PWM_VALUES_LENGTH(g_sSequenceInitOk),
    .repeats         = 150u,
    .end_delay       = 0u
};
static nrf_pwm_values_common_t g_sSequenceInitNOk[] =
{	/*Duty Cyle Sequence*/
   250,0,250,0
};
nrf_pwm_sequence_t const g_sBuzSeqInitNOk =
{
    .values.p_common = g_sSequenceInitNOk,
    .length          = NRF_PWM_VALUES_LENGTH(g_sSequenceInitNOk),
    .repeats         = 150u,
    .end_delay       = 0u
};

static nrf_pwm_values_common_t g_sBuzCmd[] =
{	/*Duty Cyle Sequence*/
   5,10,20,30,40,60,80,100,125,150,175,200,250,0,0,0,0
};
nrf_pwm_sequence_t const g_sBuzSeqCmd =
{
    .values.p_common = g_sBuzCmd,
    .length          = NRF_PWM_VALUES_LENGTH(g_sBuzCmd),
    .repeats         = 150u,
    .end_delay       = 0u
};


static nrf_pwm_values_common_t g_sBuzTestCmd[] =
{
    250,250,0,0
};

nrf_pwm_sequence_t const g_sBuzTestSeqCmd =
{
    .values.p_common = g_sBuzTestCmd,
    .length          = NRF_PWM_VALUES_LENGTH(g_sBuzTestCmd),
    .repeats         = 150u,
    .end_delay       = 0u
};

//static nrf_pwm_values_common_t g_sBuzTestCmd2[] =
//{
//   0,0,1,2,4,6,9,12,16,20,24,29,35,40,46,53,59,66,74,81,88,96,104,112,120,
//   128,136,144,152,160,168,175,182,190,197,203,210,216,221,227,232,236,240,
//   244,247,250,252,254,255,255,255,255,255,254,252,250,247,244,240,236,232,
//   227,221,216,210,203,197,190,182,175,168,160,152,144,136,128,120,112,104,
//   96,88,81,74,66,59,53,46,40,35,29,24,20,16,12,9,6,4,2,1,0 
//};
static nrf_pwm_values_common_t g_sBuzTestCmd2[] =
{
   0,0,1,2,4,6,9,12,16,20,24,29,35,40,46,53,59,66,74,81,88,96,104,112,120,
   128,136,144,152,160,168,175,182,190,197,203,210,216,221,227,232,236,240,
   244,247,250,252,254,255,255,255,255,255,255,255,255,255,255,255,255,255,
   255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
   254,252,250,247,244,240,236,232,227,221,216,210,203,197,190,182,175,168,
   160,152,144,136,128,120,112,104,96,88,81,74,66,59,53,46,40,35,29,24,20,
   16,12,9,6,4,2,1,0 
};
nrf_pwm_sequence_t const g_sBuzTestSeqCmd2 =
{
    .values.p_common = g_sBuzTestCmd2,
    .length          = NRF_PWM_VALUES_LENGTH(g_sBuzTestCmd2),
    .repeats         = 100u,
    .end_delay       = 0u
};
#endif
/************************************************************************
 * Public functions
 ************************************************************************/  
/**
 * @brief  vBuzzerInit : Initialization of the Buzzer management.
 *
 * @return None.
 */
 void vBuzzerInit(void)
{   
#if (ENABLE_BUZZER == 1)
   uint32_t l_u32ErrCode;
   g_sConfigBuzzer0.output_pins[0u] = BUZZER_PIN;
   g_sConfigBuzzer0.output_pins[1u] = NRF_DRV_PWM_PIN_NOT_USED;
   g_sConfigBuzzer0.output_pins[2u] = NRF_DRV_PWM_PIN_NOT_USED;
   g_sConfigBuzzer0.output_pins[3u] = NRF_DRV_PWM_PIN_NOT_USED;
   g_sConfigBuzzer0.base_clock = (nrf_pwm_clk_t)BUZ_CONFIG_BASE_CLOCK;
   g_sConfigBuzzer0.top_value = BUZ_CONFIG_TOP_VALUE;
   

   l_u32ErrCode = nrf_drv_pwm_init(&g_sPWMBuzzer, &g_sConfigBuzzer0, NULL);
   APP_ERROR_CHECK(l_u32ErrCode);
#endif
}

void vBuzzerUninit(void)
{
   nrf_drv_pwm_uninit(&g_sPWMBuzzer);
}

void vBuzzerStartSequence(uint8_t p_u8IsOk)
{
#if (ENABLE_BUZZER == 1)
   //   NRF_DRV_PWM_FLAG_STOP    NRF_DRV_PWM_FLAG_LOOP
   if(p_u8IsOk == 1u)
   {
      nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzSeqInitOk, 1u, NRF_DRV_PWM_FLAG_STOP);
   }
   else
   {
      nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzSeqInitNOk, 3u, NRF_DRV_PWM_FLAG_STOP);
   }
#endif
}

void vBuzzerSeqAct(void)
{
#if (ENABLE_BUZZER == 1)
   //   NRF_DRV_PWM_FLAG_STOP    NRF_DRV_PWM_FLAG_LOOP
   nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzSeqAct, 4, NRF_DRV_PWM_FLAG_STOP);
#endif
}
void vBuzzerSeqInAct(void)
{
#if (ENABLE_BUZZER == 1)
   //   NRF_DRV_PWM_FLAG_STOP    NRF_DRV_PWM_FLAG_LOOP
   nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzSeqInAct, 2, NRF_DRV_PWM_FLAG_STOP);
#endif
}

void vBuzzerSeqCmd(void)
{
#if (ENABLE_BUZZER == 1)
   nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzTestSeqCmd2, 1, NRF_DRV_PWM_FLAG_STOP);

   //   NRF_DRV_PWM_FLAG_STOP    NRF_DRV_PWM_FLAG_LOOP
//   nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzSeqCmd, 2, NRF_DRV_PWM_FLAG_STOP);
   //nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzTestSeqCmd, 30, NRF_DRV_PWM_FLAG_STOP);
#endif
}

void vBuzzerSeqCmd2(void)
{
#if (ENABLE_BUZZER == 1)
   nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzTestSeqCmd, 6, NRF_DRV_PWM_FLAG_STOP);

   //   NRF_DRV_PWM_FLAG_STOP    NRF_DRV_PWM_FLAG_LOOP
//   nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzSeqCmd, 2, NRF_DRV_PWM_FLAG_STOP);
   //nrf_drv_pwm_simple_playback(&g_sPWMBuzzer, &g_sBuzTestSeqCmd, 30, NRF_DRV_PWM_FLAG_STOP);
#endif
}
/************************************************************************
 * Private functions
 ************************************************************************/
 /**
 * @brief Function to be called in timer interrupt.
 *
 * @param[in] p_context     General purpose pointer (unused).
 */
//static void vBuzzerCallback(uint32_t p_u32PwmId)
//{
//   /* TODO For complexe sequence */
//}

//static void vBuzzerToneCreate(nrf_pwm_values_common_t * p_pPWMValues, uint16_t p_u16Size)
//{
//   uint32_t l_u32Value = 0u;
//   uint16_t l_u16Idx = 0u;
//   
//   for(l_u16Idx = 0u; l_u16Idx < p_u16Size; l_u16Idx++)
//   {
//      l_u32Value = ((float)g_cu8SinTable[l_u16Idx%100u]/255.0f) * (float)(BUZ_CONFIG_TOP_VALUE >> 1u);
//      p_pPWMValues[l_u16Idx] = l_u32Value;
//   }
//}

