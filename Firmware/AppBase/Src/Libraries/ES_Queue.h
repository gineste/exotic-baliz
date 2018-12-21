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
 */
#ifndef ES_QUEUE_H
#define ES_QUEUE_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>

/****************************************************************************************
 * Defines
 ****************************************************************************************/
 
/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef enum _ES_QUEUE_RETURNCODE_{
    ES_Queue_Failed          = 0x00,
    ES_Queue_Succeed         = 0x01,
    ES_Queue_Overlap         = 0x02,
    ES_Queue_Cycle           = 0x04,
} ES_Queue_ReturnCode_t;

typedef void (*ES_Queue_Empty_Function)(void);

typedef struct __ES_QUEUE_SEMAPHORE__ {
	  ES_Queue_Empty_Function lock;
	  ES_Queue_Empty_Function unlock;
} ES_Queue_Semaphore_t;

typedef struct __ES_QUEUE_BUFFER__ {
	  uint16_t size;
	  uint8_t * data;
} ES_Queue_Buffer_t;

typedef struct __ES_QUEUE__ {
  uint16_t index_write;
  uint16_t index_read;
  uint16_t element_counter;
  uint32_t overlap_counter;
  ES_Queue_Buffer_t * buffer;
  ES_Queue_Semaphore_t * semaphore;
} ES_Queue_t;

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
ES_Queue_ReturnCode_t ES_Queue_Init(ES_Queue_t * queue, ES_Queue_Buffer_t * queue_buffer, ES_Queue_Semaphore_t * semaphore);
ES_Queue_ReturnCode_t ES_Queue_Read(ES_Queue_t * queue, uint8_t * data, uint16_t * len);
ES_Queue_ReturnCode_t ES_Queue_Write(ES_Queue_t * queue, uint8_t * data, uint16_t len);

uint32_t ES_Queue_GetOverlapCounter(ES_Queue_t * queue);
uint32_t ES_Queue_GetElementCounter(ES_Queue_t * queue);

ES_Queue_ReturnCode_t ES_Queue_ClearAll(ES_Queue_t * queue);

#endif /* ES_QUEUE_H */
