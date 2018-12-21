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
#ifndef GLOBAL_DEFS_H
#define GLOBAL_DEFS_H

/************************************************************************
 * Include Files
 ************************************************************************/
#include <stdint.h>
#include "BoardConfig.h"

#if (EN_LOG == 1)
   #include "SEGGER_RTT.h"
#endif

/************************************************************************
 * Defines
 ************************************************************************/
#ifdef boolean_t
   #undef boolean_t
   typedef enum _BOOLEAN_ {
      FALSE = 0u,
      TRUE = 1u,
   }boolean_t;
#endif 
   
#define BIT_MASK(val,mask,shift)    ((((uint32_t)val) & ((uint32_t)mask)) >> ((uint32_t)shift))
   
/** The higher 8 bits of a upper 16 bits of a 32 bit value */
#define XMSB_32B_TO_8B(x)   ((uint8_t)((((uint32_t)x) & 0xFF000000) >> 24u))
/** The lower 8 bits of a upper 16 bits of a 32 bit value */
#define MSB_32B_TO_8B(x)    ((uint8_t)((((uint32_t)x) & 0x00FF0000) >> 16u))
/** The higher 8 bits of a lower 16 bits of a 32 bit value */
#define LSB_32B_TO_8B(x)    ((uint8_t)((((uint32_t)x) & 0x0000FF00) >> 8u))
/** The lowest 8 bits of a 32 bit value */
#define XLSB_32B_TO_8B(x)   ((uint8_t)(((uint32_t)x) & 0x000000FF))

/** The lower 16 bits of a 32 bit value */
#define MSB_32B_TO_16B(x)    ((uint16_t)((((uint32_t)x) & 0xFFFF0000) >> 16u))
/** The higher 16 bits of a 32 bit value */
#define LSB_32B_TO_16B(x)    ((uint16_t)(((uint32_t)x) & 0x0000FFFF))
   
/** The upper 8 bits of a 16 bit value */
#define MSB_16B_TO_8B(x)    ((uint8_t)((((uint16_t)x) & 0xFF00) >> 8u))
/** The lower 8 bits (of a 16 bit value) */
#define LSB_16B_TO_8B(x)    ((uint8_t)(((uint16_t)x) & 0x00FF))

/** The upper 4 bits of a 8 bit value */
#define MSB_8B(x)    ((uint8_t)(((uint8_t)x) & 0xF0))
/** The lower 4 bits (of a 8 bit value) */
#define LSB_8B(x)    ((uint8_t)(((uint8_t)x) & 0x0F))

/** Assembly two 8 bits into 16 bit value */
#define U8_TO_U16(msb,lsb)             ((uint16_t) ((((uint16_t)msb)<<8u) + ((uint16_t)lsb)) )
/** Assembly four 8 bits into 32 bit value */
#define U8_TO_U32(xmsb,msb,lsb,xlsb)   ((uint32_t) ((((uint32_t)xmsb)<<24u) + (((uint32_t)msb)<<16u) + \
                                                    (((uint32_t)lsb) <<8u)  +  ((uint32_t)xlsb)) )
/** Assembly two 16 bits into 32 bit value */
#define U16_TO_U32(msb,lsb)            ((uint32_t) ((((uint32_t)msb)<<16u) + ((uint32_t)lsb)) )

/* Conversion for Timer */
#define SEC_TO_MS(sec)                 (uint32_t)(sec*1000u)
#define MIN_TO_MS(min)                 (uint32_t)(SEC_TO_MS(min*60u))
#define HOUR_TO_MS(hour)               (uint32_t)(MIN_TO_MS(hour*60u))

#define MIN_TO_SEC(min)                (uint32_t)(min*60u)
#define HOUR_TO_MIN(hour)              (uint32_t)(hour*60u)
#define HOUR_TO_SEC(hour)              (uint32_t)(hour*3600u)

/* Min Max value */
#define MINI(a, b)        (((a) < (b)) ? (a) : (b))
#define MAXI(a, b)        (((a) > (b)) ? (a) : (b))

#define MAX_SENSOR_CFG_VALUE   ((uint32_t)2047u) /* (2*POW(OP_CODE_LAST_SENSOR) - 1)*/

/* Event/Node/BinaryTree Defines */
#define MAX_EVENTS         ((uint8_t)5u)
#define EVENT_MAX          ((uint8_t)MAX_EVENTS)
#define MAX_TREES          ((uint8_t)MAX_EVENTS)
   

#if (EN_LOG == 1)
   #define CLEAR_TERMINAL()                     \
      do {                                      \
         SEGGER_RTT_SetTerminal(0u);            \
         SEGGER_RTT_printf(0, RTT_CTRL_CLEAR);  \
         SEGGER_RTT_SetTerminal(1u);            \
         SEGGER_RTT_printf(0, RTT_CTRL_CLEAR);  \
         SEGGER_RTT_SetTerminal(2u);            \
         SEGGER_RTT_printf(0, RTT_CTRL_CLEAR);  \
         SEGGER_RTT_SetTerminal(0u);            \
      }while(0)
      
   #define RESET_STYLE()                     SEGGER_RTT_printf(0, RTT_CTRL_RESET)
   #define PRINT_FAST(p_data)                do { RESET_STYLE(); SEGGER_RTT_WriteString(0, p_data); } while(0)
   #define PRINT_ARRAY(p_type, p_data, p_size)              \
      do {                                                  \
         RESET_STYLE();                                     \
         uint16_t l_u16Size = 0u;                           \
         for(l_u16Size = 0u; l_u16Size<p_size;l_u16Size++)  \
         {                                                  \
            PRINT_DEBUG(p_type, p_data[l_u16Size]);         \
         }                                                  \
      } while(0)                                         
   
   #define PRINT_UART(p_type, p_data)        \
      do {                                   \
         SEGGER_RTT_SetTerminal(0u);                                                     \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      } while(0)
      
   #define PRINT_UART_ARRAY(p_type, p_data, p_size)                                                      \
      do {                                                                                               \
         RESET_STYLE();                                                                                  \
         SEGGER_RTT_SetTerminal(0u);                                                                     \
         uint16_t l_u16Size = 0u;                                                                        \
         for(l_u16Size = 0u; l_u16Size<p_size;l_u16Size++)                                               \
         {                                                                                               \
            SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, *(p_data+l_u16Size)); \
         }                                                                                               \
      } while(0)  
      
   #define PRINT_GREEN(p_type, p_data)                                                   \
      do {                                                                               \
         SEGGER_RTT_SetTerminal(0u);                                                     \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_DEBUG(p_type, p_data)                                                   \
      do {                                                                               \
         SEGGER_RTT_SetTerminal(0u);                                                     \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_T2(p_type, p_data)                                                \
      do {                                                                               \
         SEGGER_RTT_SetTerminal(0u);                                                     \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_T2_ARRAY(p_type, p_data, p_size)                                                        \
      do {                                                                                               \
         RESET_STYLE();                                                                                  \
         SEGGER_RTT_SetTerminal(0u);                                                                     \
         uint16_t l_u16Size = 0u;                                                                        \
         for(l_u16Size = 0u; l_u16Size<p_size;l_u16Size++)                                               \
         {                                                                                               \
            SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, *(p_data+l_u16Size));   \
         }                                                                                               \
      } while(0)
      
   #define PRINT_INFO(p_type, p_data)                                                    \
      do {                                                                               \
         SEGGER_RTT_SetTerminal(0u);                                                     \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_STATE(p_type, p_data)                                                  \
      do {                                                                              \
         SEGGER_RTT_SetTerminal(0u);                                                    \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_WARNING(p_type, p_data)                                                       \
      do {                                                                                     \
         SEGGER_RTT_SetTerminal(0u);                                                           \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK "/!\\ " p_type " /!\\\n" RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_EVENT(p_type, p_data)                                                     \
      do {                                                                                 \
         SEGGER_RTT_SetTerminal(0u);                                                       \
         SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET, p_data); \
      }while(0)
      
   #define PRINT_ERROR(p_type, p_data)                            \
      do {                                                        \
         SEGGER_RTT_SetTerminal(0u);                              \
         SEGGER_RTT_printf(0, RTT_CTRL_BG_BRIGHT_RED              \
         RTT_CTRL_TEXT_BRIGHT_BLACK p_type RTT_CTRL_RESET "\n", p_data); \
      }while(0)
      
#else
   #define CLEAR_TERMINAL()
   #define RESET_STYLE()                     
   #define PRINT_FAST(p_data)     
   #define PRINT_ARRAY(p_type, p_data, p_size)  
   #define PRINT_UART(p_type, p_data)   
   #define PRINT_UART_ARRAY(p_type, p_data, p_size)   
   #define PRINT_GREEN(p_type, p_data)
   #define PRINT_DEBUG(p_type, p_data) 
   #define PRINT_T2(p_type, p_data)
   #define PRINT_T2_ARRAY(p_type, p_data, p_size)
   #define PRINT_INFO(p_type, p_data)   
   #define PRINT_STATE(p_type, p_data)  
   #define PRINT_WARNING(p_type, p_data)
   #define PRINT_EVENT(p_type, p_data)
   #define PRINT_ERROR(p_type, p_data)  
#endif

/************************************************************************
 * Type definitions
 ************************************************************************/
typedef union _U32_U_ {
   struct
   {
      uint32_t b0to15:16;
      uint32_t b16to31:16;
   }w16b;
   struct
   {
      uint32_t b0to7:8;
      uint32_t b8to15:8;
      uint32_t b16to23:8;
      uint32_t b24to31:8;
   }w8b;
   uint32_t u32DWord;
}u_DWord_t;

typedef union _U16_U_ {
   struct
   {
      uint16_t b0to7:8;
      uint16_t b8to15:8;
   }w8b;   
   struct
   {
      uint16_t b0:1;
      uint16_t b1:1;
      uint16_t b2:1;
      uint16_t b3:1;
      uint16_t b4:1;
      uint16_t b5:1;
      uint16_t b6:1;
      uint16_t b7:1;
      uint16_t b8:1;
      uint16_t b9:1;
      uint16_t b10:1;
      uint16_t b11:1;
      uint16_t b12:1;
      uint16_t b13:1;
      uint16_t b14:1;
      uint16_t b15:1;
   }w1b;
   uint16_t u16Word;
}u_Word_t;

/************************************************************************
 * Public function declarations
 ************************************************************************/
 
#endif /* GLOBAL_DEFS_H */

