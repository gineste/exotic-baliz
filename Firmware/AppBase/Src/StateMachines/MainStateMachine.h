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
#ifndef MAIN_STATE_MACHINE_H
#define MAIN_STATE_MACHINE_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include "GlobalDefs.h"

/************************************************************************
 * Defines
 ************************************************************************/
#define NO_TIMER                       ((uint32_t)0u)

#define POWERUP_CB_TIMER_MS            ((uint32_t)200u)
#define INIT_CB_TIMER_MS               ((uint32_t)500u)
#define OPERATIONAL_CB_TIMER_MS        NO_TIMER//SEC_TO_MS(10u)
#define CONNECTED_CB_TIMER_MS          SEC_TO_MS(1u)

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _MAIN_SM_ {
   MSM_POWER_UP,
   MSM_INIT,
   MSM_OPERATIONAL,
   MSM_CONNECTED,
   MSM_DEEP_SLEEP,
   MSM_ERROR,
   MSM_MAX_NUMBERS
}e_MainStateMachine_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vMSM_Init(void);
void vMSM_Process(void);
void vMSM_StateMachineSet(e_MainStateMachine_t p_eState);
e_MainStateMachine_t eMSM_GetState(void);

void vStateMachine_ChangeTime(uint32_t p_u32RefreshTime);

uint8_t u8MSM_CyclicProcessCheck(void);
void vMSM_CyclicProcessClear(void);

#endif /* MAIN_STATE_MACHINE_H */

