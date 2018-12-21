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
#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H

/************************************************************************
 * Include Files
 ************************************************************************/
/* Propietary Includes */
#include <stdint.h>

/* nRF Includes */
#include "fds.h"
#include "nrf_fstorage.h"

/************************************************************************
 * Defines
 ************************************************************************/
 
/************************************************************************
 * Type definitions
 ************************************************************************/
typedef enum _MEM_ERROR_CODE_ {
   MEM_ERROR_NONE = 0u,
   MEM_ERROR_PARAM,
   MEM_ERROR_NOT_FOUND,
   MEM_ERROR_BUSY,
   MEM_ERROR_RECORD,
   MEM_ERROR_ID,
   MEM_ERROR_INIT,
   MEM_ERROR_READ,
   MEM_ERROR_WRITE,
   MEM_ERROR_ERASE,
   MEM_ERROR_FULL,
}e_MemoryErrCode_t;

typedef void (*fctpReadCallback)(void * p_pvData, uint16_t p_u16SizeInDWord);

/************************************************************************
 * Public function declarations
 ************************************************************************/
e_MemoryErrCode_t eMemory_Init(void);
e_MemoryErrCode_t eMemory_Write(const uint16_t p_cu16FileId, const uint16_t p_cu16Key, const void * p_pvData, uint16_t p_u16ByteNumber);
e_MemoryErrCode_t eMemory_Read(const uint16_t p_cu16FileId, const uint16_t p_cu16Key, fctpReadCallback p_fctpCallback);
e_MemoryErrCode_t eMemory_Flush(void);
e_MemoryErrCode_t eMemory_Erase_File(const uint16_t p_cu16FileId);
uint8_t u8MemWriteFlagGet(void);
uint8_t u8MemDeleteFlagGet(void);

#endif /* FLASH_MEMORY_H */

