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
 * Date:          02 08 2017 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Memory management of nRF. 
 *
 */
 
/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include <string.h>

/* nRF Libraries */
#include "fds.h"

/* nRF SOC */
#include "nrf_sdh.h"

/* Local include */
#include "FlashMemory.h"

/************************************************************************
 * Defines
 ************************************************************************/
//#define BYTES_TO_WORDS(x)     ((uint32_t)(((x) + 3u) >> 2u ))  /* Word is u32 */
#define MAX_SIZE_DATA         ((uint8_t)64u) /* Max size for Event 64 * word size (32b) = 64 * 4 = 256 */

#define HW_TIMEOUT            (uint32_t)2000000ul

/************************************************************************
 * Private type declarations
 ************************************************************************/
static void vMemoryEventHandler(fds_evt_t const * p_pcsFdsEvent);

/************************************************************************
 * Private function declarations
 ************************************************************************/
static volatile uint8_t g_u8MemoryInitialized = 0u;
static volatile uint8_t g_u8PendingWrite = 0u;
static volatile uint8_t g_u8PendingDelete = 0u;

/* Must create a static const pointer to keep data until recorded on flash */
static uint32_t const * g_cpu32Data;

/************************************************************************
 * Variable declarations
 ************************************************************************/

/************************************************************************
 * Public functions
 ************************************************************************/  
/**@brief Function to initialize Memory management.
 * @return Error Code
 */
e_MemoryErrCode_t eMemory_Init(void)
{
   e_MemoryErrCode_t l_eErrCode = MEM_ERROR_INIT;
   ret_code_t l_eFDSError = FDS_ERR_NOT_INITIALIZED;
   uint32_t l_u32Timeout = HW_TIMEOUT;
   uint8_t l_u8SoftDeviceEnabled = 0u;
   
   /* Set the handler of Flash Data Storage module */
   l_eFDSError = fds_register(vMemoryEventHandler);
   
   /* fds need softdevice to be enable */
   l_u8SoftDeviceEnabled = (uint8_t)nrf_sdh_is_enabled();
   
   /* Done in PeerDataStorage*/
   if(l_eFDSError == FDS_SUCCESS)
   {
      if(l_u8SoftDeviceEnabled != 0u)
      {
         /* Init Flash Data Storage */
         l_eFDSError = fds_init();
         
         if(l_eFDSError == FDS_SUCCESS)
         {
            while(   (l_u32Timeout > 0u) 
                  && (g_u8MemoryInitialized == 0u) )
            {
               l_u32Timeout--;
            }

            if(l_u32Timeout == 0u)
            {
               l_eErrCode = MEM_ERROR_BUSY;
            }
         }
      }
   }
   
   if(l_eFDSError == FDS_SUCCESS)
   {
      l_eErrCode = MEM_ERROR_NONE;
   }
   
   return l_eErrCode;
}

/**@brief Function to write in flash Memory.
 * @return Error Code
 */
e_MemoryErrCode_t eMemory_Write(const uint16_t p_cu16FileId, const uint16_t p_cu16Key, const void * p_pvData, uint16_t p_u16ByteNumber)
{
   e_MemoryErrCode_t l_eErrCode = MEM_ERROR_NONE;
   ret_code_t l_eFDSError;   
   uint32_t l_u32LengthWords = 0u;
   uint32_t l_u32Timeout = HW_TIMEOUT;
   
   fds_record_t l_sRecord;
   fds_record_desc_t l_sRecordDesc; 
   fds_find_token_t l_sFindToken = {
      .page = 0u,
      .p_addr = NULL,
   };

   /* Set Data in global var */
   g_cpu32Data = (uint32_t *)p_pvData;

   /* Compute Byte numbers into length words */
   l_u32LengthWords = (uint32_t)((p_u16ByteNumber + 3u)>>2u);
   
   /* Set Record */
   l_sRecord.file_id = p_cu16FileId;
   l_sRecord.key = p_cu16Key;
   l_sRecord.data.p_data = p_pvData;
   l_sRecord.data.length_words = l_u32LengthWords;
   
   /* Check if record already exist and update it if it's the case */ 
   l_eFDSError = fds_record_find_in_file(p_cu16FileId, &l_sRecordDesc, &l_sFindToken);
   if(l_eFDSError == FDS_SUCCESS)
   {      
      l_eFDSError = fds_record_update(&l_sRecordDesc, &l_sRecord);
      
      if(l_eFDSError != FDS_SUCCESS)
      {
         l_eErrCode = MEM_ERROR_WRITE;
      }
      else
      {
         g_u8PendingWrite = 1u;   
      }
   } 
   else 
   {
      if(fds_record_write(&l_sRecordDesc, &l_sRecord) != FDS_SUCCESS)
      {
         l_eErrCode = MEM_ERROR_WRITE;
      }
      else
      {         
         g_u8PendingWrite = 1u;
      }
   }
   
   if(l_eErrCode == MEM_ERROR_NONE)
   {      
      while(   (l_u32Timeout > 0u) 
            && (g_u8PendingWrite == 1u) )
      {
         l_u32Timeout--;
      }
      
      if(l_u32Timeout == 0u)
      {
         l_eErrCode = MEM_ERROR_BUSY;
      }
   }
   
   return l_eErrCode;
}

/**@brief Function to read in flash Memory.
 * @return Error Code
 */
e_MemoryErrCode_t eMemory_Read(const uint16_t p_cu16FileId, const uint16_t p_cu16Key, fctpReadCallback p_fctpCallback)
{
   e_MemoryErrCode_t l_eErrCode = MEM_ERROR_READ;
   ret_code_t l_eFDSError;
   fds_flash_record_t  l_sFlashRecord;
   fds_record_desc_t l_sRecordDesc;
   fds_find_token_t l_sFindToken = {
      .page = 0u,
      .p_addr = NULL,
   }; //Important, make sure you zero init the l_sFindToken
   
   if(p_fctpCallback == NULL) 
   {
      l_eErrCode = MEM_ERROR_PARAM;
   }
   else
   {
      /* Do not do loop since we use unique key */
      l_eFDSError = fds_record_find(p_cu16FileId, p_cu16Key, &l_sRecordDesc, &l_sFindToken);
      if(l_eFDSError == FDS_SUCCESS)
      {
         l_eFDSError = fds_record_open(&l_sRecordDesc, &l_sFlashRecord);
         if(l_eFDSError != FDS_SUCCESS)
         {
            // Handle error.
            l_eErrCode = MEM_ERROR_READ;
         }
         else
         {      
            /* Access the record through the flash_record structure. */
            g_cpu32Data = (uint32_t*)l_sFlashRecord.p_data;
            /* Callback to get data, since once record closed we can not have access to it */
            (*p_fctpCallback)((void*)g_cpu32Data, l_sFlashRecord.p_header->length_words);
            
            /* Close the record when done. */
            l_eFDSError = fds_record_close(&l_sRecordDesc);
            if(l_eFDSError != FDS_SUCCESS)
            {
               // Handle error.
               l_eErrCode = MEM_ERROR_READ;
            }
            else
            {
               l_eErrCode = MEM_ERROR_NONE;
            }
         }
      }
      else
      {
         l_eErrCode = MEM_ERROR_NOT_FOUND;
      }
   }
   
   return l_eErrCode;
}


/**@brief Function to flush flash Memory.
 * @return Error Code
 */
e_MemoryErrCode_t eMemory_Flush(void)
{
   e_MemoryErrCode_t l_eErrCode = MEM_ERROR_ERASE;
   /* Call Garbage Collect to delete from memory */
   if(fds_gc() == FDS_SUCCESS)
   {
      l_eErrCode = MEM_ERROR_NONE;
   }
   return l_eErrCode;
}

   
/**@brief Function to erase flash Memory.
 * @return Error Code
 */
e_MemoryErrCode_t eMemory_Erase_File(const uint16_t p_cu16FileId)
{
   e_MemoryErrCode_t l_eErrCode = MEM_ERROR_ERASE;
   uint32_t l_u32Timeout = HW_TIMEOUT;
   
   if(fds_file_delete(p_cu16FileId) == FDS_SUCCESS)
   {         
      g_u8PendingDelete = 1u;
      while(   (l_u32Timeout > 0u) 
            && (g_u8PendingDelete == 1u) )
      {
         l_u32Timeout--;
      }
      
      if(l_u32Timeout == 0u)
      {
         l_eErrCode = MEM_ERROR_BUSY;
      }
      else
      {
         l_eErrCode = MEM_ERROR_NONE;
      }
   }
   
   /* Call Garbage Collect to delete from memory */
   //(void)fds_gc();
   
   return l_eErrCode;
}

e_MemoryErrCode_t eMemory_Erase_Record(const uint16_t p_cu16FileId, const uint16_t p_cu16Key)
{
   e_MemoryErrCode_t l_eErrCode = MEM_ERROR_ERASE;
   fds_record_desc_t l_sRecordDesc;
   fds_find_token_t l_sFindToken = { 
      .page = 0u,
      .p_addr = NULL,
   }; //Important, make sure you zero init the l_sFindToken
   ret_code_t l_eFDSError;
   uint32_t l_u32Timeout = HW_TIMEOUT;
   
   l_eFDSError = fds_record_find(p_cu16FileId, p_cu16Key, &l_sRecordDesc, &l_sFindToken);
   if(l_eFDSError == FDS_SUCCESS)
   {
      if(fds_record_delete(&l_sRecordDesc) == FDS_SUCCESS)
      {
         g_u8PendingDelete = 1u;
         while(   (l_u32Timeout > 0u) 
               && (g_u8PendingDelete == 1u) )
         {
            l_u32Timeout--;
         }
      
         if(l_u32Timeout == 0u)
         {
            l_eErrCode = MEM_ERROR_BUSY;
         }
         else
         {
            l_eErrCode = MEM_ERROR_NONE;
         }
         
         /* Call Garbage Collect to delete from memory */
         //(void)fds_gc();
      }
     
   }
   return l_eErrCode;
}

uint8_t u8MemWriteFlagGet(void)
{
   return g_u8PendingWrite;
}
uint8_t u8MemDeleteFlagGet(void)
{
   return g_u8PendingDelete;
}

/************************************************************************
 * Private functions
 ************************************************************************/
static void vMemoryEventHandler(fds_evt_t const * p_pcsFdsEvent)
{
   switch (p_pcsFdsEvent->id)
   {
      case FDS_EVT_INIT:
         if (p_pcsFdsEvent->result == FDS_SUCCESS)
         {
            g_u8MemoryInitialized = 1u;
         }
         else
         {
            // Initialization failed. Timeout occured
         }
         break;
      case FDS_EVT_WRITE:
      case FDS_EVT_UPDATE:
         if (p_pcsFdsEvent->result == FDS_SUCCESS)
         {
            g_u8PendingWrite = 0u;
         }
         break;  
      case FDS_EVT_DEL_RECORD:  
      case FDS_EVT_DEL_FILE:
         if (p_pcsFdsEvent->result == FDS_SUCCESS)
         {
            g_u8PendingDelete = 0u;
         }
      case FDS_EVT_GC:         
      default:
         break;
   }
}

/************************************************************************
 * End Of File
 ************************************************************************/


