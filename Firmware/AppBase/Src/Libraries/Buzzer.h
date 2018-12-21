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
#ifndef BUZZER_H
#define BUZZER_H

/************************************************************************
 * Include Files
 ************************************************************************/
 
/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Type definitions
 ************************************************************************/

/************************************************************************
 * Public function declarations
 ************************************************************************/
void vBuzzerInit(void);
void vBuzzerUninit(void);
void vBuzzerStartSequence(uint8_t p_u8IsOk);
void vBuzzerSeqAct(void);
void vBuzzerSeqInAct(void);
void vBuzzerSeqCmd(void);
void vBuzzerSeqCmd2(void);

#endif // BUZZER_H

