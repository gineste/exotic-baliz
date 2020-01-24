/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2019 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          17/01/2020 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   File System Library for nRF52
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

#include "BoardConfig.h"
#include "board.h"

#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

/* Self include */
#include "FileSystem.h"

#if (EN_LOG == 1) && defined(DEBUG)
   #include "SEGGER_RTT.h"
   /* Retarget printf and scanf */
   #define printf(...)           do {    \
      SEGGER_RTT_printf(0, __VA_ARGS__); \
   } while(0);
#else
   #define printf(...)
#endif

/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define FILE_NAME   "Log.txt"
#define TEST_STRING "SD card example."

#define SDC_SCK_PIN     SPI_SCLK  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    SPI_MOSI  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    SPI_MISO  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      SDCARD_CS  ///< SDC chip select (CS) pin.

/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
//static void vWaitFunc(void);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static FATFS fs;
static FIL file;
static DIR dir;
static FILINFO fno;
static uint8_t g_u8FSInit = 0u;
static uint8_t g_u8FSFileCreated = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
void vFS_Init(void)
{   
   FRESULT ff_result;
   DSTATUS disk_state = STA_NOINIT;

   // Initialize FATFS disk I/O interface by providing the block device.
   static diskio_blkdev_t drives[] =
   {
         DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
   };

   diskio_blockdev_register(drives, ARRAY_SIZE(drives));

   printf("Initializing disk 0 (SDC)...\n\0");
   for (uint32_t retries = 3; retries && disk_state; --retries)
   {
     disk_state = disk_initialize(0);
   }
   if (disk_state)
   {
     printf("Disk initialization failed.\n\0");
     return;
   }

   uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
   uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
   printf("Capacity: %d MB\n\0", capacity);

   printf("Mounting volume...\n\0");
   ff_result = f_mount(&fs, "", 1);
   if (ff_result)
   {
     printf("Mount failed.\n\0");
     return;
   }
   g_u8FSInit = 1u;
}

void vFS_Uninit(void)
{
   FRESULT ff_result;
   printf("Unmounting volume...\n\0");
   ff_result = f_mount(NULL, "", 1);
   if (ff_result)
   {
     printf("Unmount failed.\n\0");
     return;
   }
   g_u8FSInit = 0u;
}

void vFS_CreateFile(char* p_pchName)
{
   FRESULT ff_result;
   char l_achFileName[20u] = { '\0' };
   if(g_u8FSInit == 1u)
   {
      printf("\r\n Listing directory: /\n\0");
      ff_result = f_opendir(&dir, "/");
      if (ff_result)
      {
        printf("Directory listing failed!\n\0");
        return;
      }

      do
      {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            printf("Directory read failed.\n\0");
            return;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                printf("   <DIR>   %s\n\0",fno.fname);
            }
            else
            {
                printf("%9lu  %s\n\0", fno.fsize, fno.fname);
            }
        }
      }
      while (fno.fname[0]);
      printf("");

      if(p_pchName != NULL)
      {
         uint8_t l_u8Idx = 0u;
         uint8_t l_u8Size = strlen(p_pchName);
         char l_achParam[5u] = { '\0' };
         strcpy(l_achFileName, p_pchName);
         
         do {            
            //sscanf( l_achParam, "%3d", l_u8Idx);
            sprintf( l_achParam, "%u", l_u8Idx);
            strcpy(&l_achFileName[l_u8Size], l_achParam);
            memset( l_achParam, '\0', 5);
            ff_result = f_open(&file, l_achFileName, FA_OPEN_EXISTING);
            if(ff_result == FR_OK)
            {
               // Close 
               (void) f_close(&file);
               l_u8Idx++;
            }
         }while((ff_result != FR_NO_FILE) && (l_u8Idx < 200) );
         
         printf("Openning file %s...\n\0", l_achFileName);
         ff_result = f_open(&file, l_achFileName, FA_WRITE | FA_OPEN_ALWAYS);
         if (ff_result != FR_OK)
         {
           printf("Unable to open or create file: %s.\n\0", l_achFileName);
           return;
         }
         g_u8FSFileCreated = 1u;
      }
   }
}

void vFS_IsFileOpen(uint8_t * p_pu8IsOpen)
{
}

void vFS_CloseFile(void)
{
   (void) f_close(&file);
   g_u8FSFileCreated = 0u;
}

void vFS_Write(char * p_pchData, uint16_t p_u16Size)
{   
   FRESULT ff_result;
   uint32_t bytes_written;
   
   if(g_u8FSInit && g_u8FSFileCreated)
   {
      ff_result = f_write(&file, p_pchData, p_u16Size - 1, (UINT *) &bytes_written);
      if (ff_result != FR_OK)
      {
         printf("Write failed.\n\0");
      }
//      else
//      {
//         printf("%d bytes written.\n\0", bytes_written);
//      }
   }
}

void vFS_Sync(void)
{
   FRESULT ff_result;
   if(g_u8FSInit && g_u8FSFileCreated)
   {
      ff_result = f_sync(&file);
      if (ff_result != FR_OK)
      {
         printf("Sync failed.\n\0");
      }
      else
      {
         printf("Sync ok.\n\0");
      }
   }
}
/****************************************************************************************
 * Private functions
 ****************************************************************************************/
//static void vWaitFunc(void)
//{
//   uint16_t l_u16Idx = 0u;
//   for(l_u16Idx = 0u;l_u16Idx < 0xFFFE;l_u16Idx++)
//   {
//      __nop();
//   }
//   return;
//}
/****************************************************************************************
 * End Of File
 ****************************************************************************************/


