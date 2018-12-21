/* =============================================================================
 ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
(  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
(____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/

  Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.

  Licensees are granted free, non-transferable use of the information. NO
  WARRANTY of ANY KIND is provided. This heading must NOT be removed from
  the file.

  File name:    ExoDefs.h
  Date:         02 01 2018
  Author:       Emeric L.
  Description:  EXODEFS - Header file.
============================================================================= */
#ifndef EXODEFS__H
  #define EXODEFS__H

  /* ===========================================================================
                                   DEBUG Section
  =========================================================================== */

  /* ===========================================================================
                            Public defines and typedefs
  =========================================================================== */
  
  /* Miscellaneous defines. */
  #define EXO_PNULL             ((void *)0)
  
  #define TEN_TO_UNITY          ((uint8_t)10)
  #define HUNDRED_TO_UNITY      ((uint8_t)100)
  #define THOUSAND_TO_UNITY     ((uint8_t)1000)
  
  #define BUFFER_IDX0           ((uint32_t)0)
  
  /* Shift defines. */
  #define BIT_SHIFT             ((uint8_t)1)
  #define QUARTET_SHIFT         ((uint8_t)4)
  #define UINT8_SHIFT           ((uint8_t)8)
  #define UINT16_SHIFT          ((uint8_t)16)
  #define UINT32_SHIFT          ((uint8_t)32)
  #define UINT64_SHIFT          ((uint8_t)64)

  /* char defines. */
  #define CHAR_MIN_VALUE        ((char)0)
  #define CHAR_MAX_VALUE        ((char)127)
  
  #define CHAR_MASK             ((uint8_t)0x7F)

  #define CHAR_NUL              ((char)'\0')
  #define CHAR_LF               ((char)'\n')
  #define CHAR_CR               ((char)'\r')
  
  /* 8_bit defines. */
  #define BITS_IN_INT8          ((int8_t)8)

  #define INT8_MIN_VALUE        ((int8_t)-128)
  #define INT8_MAX_VALUE        ((int8_t)127) 

  #define BITS_IN_UINT8         ((uint8_t)8)
  
  #define UINT8_MIN_VALUE       ((uint8_t)0)
  #define UINT8_MAX_VALUE       ((uint8_t)255)

  #define UINT8_LSB_SHIFT       ((uint8_t)0)
  #define UINT8_BIT1_SHIFT      ((uint8_t)1)
  #define UINT8_BIT2_SHIFT      ((uint8_t)2)
  #define UINT8_BIT3_SHIFT      ((uint8_t)3)
  #define UINT8_BIT4_SHIFT      ((uint8_t)4)
  #define UINT8_BIT5_SHIFT      ((uint8_t)5)
  #define UINT8_BIT6_SHIFT      ((uint8_t)6)
  #define UINT8_MSB_SHIFT       ((uint8_t)7)  

  #define UINT8_LSB_MASK        ((uint8_t)0x01)
  #define UINT8_BIT1_MASK       ((uint8_t)0x02)
  #define UINT8_BIT2_MASK       ((uint8_t)0x04)
  #define UINT8_BIT3_MASK       ((uint8_t)0x08)
  #define UINT8_BIT4_MASK       ((uint8_t)0x10)
  #define UINT8_BIT5_MASK       ((uint8_t)0x20)
  #define UINT8_BIT6_MASK       ((uint8_t)0x40)
  #define UINT8_MSB_MASK        ((uint8_t)0x80)
  
  #define UINT8_MASK            ((uint8_t)0xff)

  /* 16-bit defines. */
  #define BITS_IN_INT16         ((int16_t)16)

  #define INT16_MIN_VALUE       ((int16_t)-32768)
  #define INT16_MAX_VALUE       ((int16_t)32767)

  #define BITS_IN_UINT16        ((uint16_t)16)

  #define UINT16_MIN_VALUE      ((uint16_t)0)
  #define UINT16_MAX_VALUE      ((uint16_t)65535)

  #define UINT16_LSB_SHIFT      ((uint16_t)0)
  #define UINT16_BIT1_SHIFT     ((uint16_t)1)
  #define UINT16_BIT2_SHIFT     ((uint16_t)2)
  #define UINT16_BIT3_SHIFT     ((uint16_t)3)
  #define UINT16_BIT4_SHIFT     ((uint16_t)4)
  #define UINT16_BIT5_SHIFT     ((uint16_t)5)
  #define UINT16_BIT6_SHIFT     ((uint16_t)6)
  #define UINT16_BIT7_SHIFT     ((uint16_t)7)
  #define UINT16_BIT8_SHIFT     ((uint16_t)8)
  #define UINT16_BIT9_SHIFT     ((uint16_t)9)
  #define UINT16_BIT10_SHIFT    ((uint16_t)10)
  #define UINT16_BIT11_SHIFT    ((uint16_t)11)
  #define UINT16_BIT12_SHIFT    ((uint16_t)12)
  #define UINT16_BIT13_SHIFT    ((uint16_t)13)
  #define UINT16_BIT14_SHIFT    ((uint16_t)14)
  #define UINT16_MSB_SHIFT      ((uint16_t)15)

  #define UINT16_LSB_MASK       ((uint16_t)0x0001)
  #define UINT16_BIT1_MASK      ((uint16_t)0x0002)
  #define UINT16_BIT2_MASK      ((uint16_t)0x0004)
  #define UINT16_BIT3_MASK      ((uint16_t)0x0008)
  #define UINT16_BIT4_MASK      ((uint16_t)0x0010)
  #define UINT16_BIT5_MASK      ((uint16_t)0x0020)
  #define UINT16_BIT6_MASK      ((uint16_t)0x0040)
  #define UINT16_BIT7_MASK      ((uint16_t)0x0080)
  #define UINT16_BIT8_MASK      ((uint16_t)0x0100)
  #define UINT16_BIT9_MASK      ((uint16_t)0x0200)
  #define UINT16_BIT10_MASK     ((uint16_t)0x0400)
  #define UINT16_BIT11_MASK     ((uint16_t)0x0800)
  #define UINT16_BIT12_MASK     ((uint16_t)0x1000)
  #define UINT16_BIT13_MASK     ((uint16_t)0x2000)
  #define UINT16_BIT14_MASK     ((uint16_t)0x4000)
  #define UINT16_MSB_MASK       ((uint16_t)0x8000)

  #define UINT16_MASK           ((uint16_t)0xffff)
  
  /* 32-bit defines. */
  #define BITS_IN_INT32         ((int32_t)32)

  #define INT32_MIN_VALUE       ((int32_t)-2147483648)
  #define INT32_MAX_VALUE       ((int32_t)2147483647)

  #define BITS_IN_UINT32        ((uint32_t)32)

  #define UINT32_MIN_VALUE      ((uint32_t)0)
  #define UINT32_MAX_VALUE      ((uint32_t)4294967295)

  #define UINT32_LSB_SHIFT      ((uint32_t)0)
  #define UINT32_BIT1_SHIFT     ((uint32_t)1)
  #define UINT32_BIT2_SHIFT     ((uint32_t)2)
  #define UINT32_BIT3_SHIFT     ((uint32_t)3)
  #define UINT32_BIT4_SHIFT     ((uint32_t)4)
  #define UINT32_BIT5_SHIFT     ((uint32_t)5)
  #define UINT32_BIT6_SHIFT     ((uint32_t)6)
  #define UINT32_BIT7_SHIFT     ((uint32_t)7)
  #define UINT32_BIT8_SHIFT     ((uint32_t)8)
  #define UINT32_BIT9_SHIFT     ((uint32_t)9)
  #define UINT32_BIT10_SHIFT    ((uint32_t)10)
  #define UINT32_BIT11_SHIFT    ((uint32_t)11)
  #define UINT32_BIT12_SHIFT    ((uint32_t)12)
  #define UINT32_BIT13_SHIFT    ((uint32_t)13)
  #define UINT32_BIT14_SHIFT    ((uint32_t)14)
  #define UINT32_BIT15_SHIFT    ((uint32_t)15)
  #define UINT32_BIT16_SHIFT    ((uint32_t)16)
  #define UINT32_BIT17_SHIFT    ((uint32_t)17)
  #define UINT32_BIT18_SHIFT    ((uint32_t)18)
  #define UINT32_BIT19_SHIFT    ((uint32_t)19)
  #define UINT32_BIT20_SHIFT    ((uint32_t)20)
  #define UINT32_BIT21_SHIFT    ((uint32_t)21)
  #define UINT32_BIT22_SHIFT    ((uint32_t)22)
  #define UINT32_BIT23_SHIFT    ((uint32_t)23)
  #define UINT32_BIT24_SHIFT    ((uint32_t)24)
  #define UINT32_BIT25_SHIFT    ((uint32_t)25)
  #define UINT32_BIT26_SHIFT    ((uint32_t)26)
  #define UINT32_BIT27_SHIFT    ((uint32_t)27)
  #define UINT32_BIT28_SHIFT    ((uint32_t)28)
  #define UINT32_BIT29_SHIFT    ((uint32_t)29)
  #define UINT32_BIT30_SHIFT    ((uint32_t)30)
  #define UINT32_MSB_SHIFT      ((uint32_t)31)

  #define UINT32_LSB_MASK       ((uint32_t)0x00000001)
  #define UINT32_BIT1_MASK      ((uint32_t)0x00000002)
  #define UINT32_BIT2_MASK      ((uint32_t)0x00000004)
  #define UINT32_BIT3_MASK      ((uint32_t)0x00000008)
  #define UINT32_BIT4_MASK      ((uint32_t)0x00000010)
  #define UINT32_BIT5_MASK      ((uint32_t)0x00000020)
  #define UINT32_BIT6_MASK      ((uint32_t)0x00000040)
  #define UINT32_BIT7_MASK      ((uint32_t)0x00000080)
  #define UINT32_BIT8_MASK      ((uint32_t)0x00000100)
  #define UINT32_BIT9_MASK      ((uint32_t)0x00000200)
  #define UINT32_BIT10_MASK     ((uint32_t)0x00000400)
  #define UINT32_BIT11_MASK     ((uint32_t)0x00000800)
  #define UINT32_BIT12_MASK     ((uint32_t)0x00001000)
  #define UINT32_BIT13_MASK     ((uint32_t)0x00002000)
  #define UINT32_BIT14_MASK     ((uint32_t)0x00004000)
  #define UINT32_BIT15_MASK     ((uint32_t)0x00008000)
  #define UINT32_BIT16_MASK     ((uint32_t)0x00010000)
  #define UINT32_BIT17_MASK     ((uint32_t)0x00020000)
  #define UINT32_BIT18_MASK     ((uint32_t)0x00040000)
  #define UINT32_BIT19_MASK     ((uint32_t)0x00080000)
  #define UINT32_BIT20_MASK     ((uint32_t)0x00100000)
  #define UINT32_BIT21_MASK     ((uint32_t)0x00200000)
  #define UINT32_BIT22_MASK     ((uint32_t)0x00400000)
  #define UINT32_BIT23_MASK     ((uint32_t)0x00800000)
  #define UINT32_BIT24_MASK     ((uint32_t)0x01000000)
  #define UINT32_BIT25_MASK     ((uint32_t)0x02000000)
  #define UINT32_BIT26_MASK     ((uint32_t)0x04000000)
  #define UINT32_BIT27_MASK     ((uint32_t)0x08000000)
  #define UINT32_BIT28_MASK     ((uint32_t)0x10000000)
  #define UINT32_BIT29_MASK     ((uint32_t)0x20000000)
  #define UINT32_BIT30_MASK     ((uint32_t)0x40000000)
  #define UINT32_MSB_MASK       ((uint32_t)0x80000000)

  /* 64-bit defines. */
  #define BITS_IN_INT64         ((int64_t)64)

  #define INT64_MIN_VALUE       ((int64_t)-9223372036854775808)
  #define INT64_MAX_VALUE       ((int64_t)9223372036854775807)

  #define BITS_IN_UINT64        ((uint64_t)64)

  #define UINT64_MIN_VALUE      ((uint64_t)0)
  #define UINT64_MAX_VALUE      ((uint64_t)18446744073709551615)

  #define UINT64_LSB_SHIFT      ((uint64_t)0)
  #define UINT64_BIT1_SHIFT     ((uint64_t)1)
  #define UINT64_BIT2_SHIFT     ((uint64_t)2)
  #define UINT64_BIT3_SHIFT     ((uint64_t)3)
  #define UINT64_BIT4_SHIFT     ((uint64_t)4)
  #define UINT64_BIT5_SHIFT     ((uint64_t)5)
  #define UINT64_BIT6_SHIFT     ((uint64_t)6)
  #define UINT64_BIT7_SHIFT     ((uint64_t)7)
  #define UINT64_BIT8_SHIFT     ((uint64_t)8)
  #define UINT64_BIT9_SHIFT     ((uint64_t)9)
  #define UINT64_BIT10_SHIFT    ((uint64_t)10)
  #define UINT64_BIT11_SHIFT    ((uint64_t)11)
  #define UINT64_BIT12_SHIFT    ((uint64_t)12)
  #define UINT64_BIT13_SHIFT    ((uint64_t)13)
  #define UINT64_BIT14_SHIFT    ((uint64_t)14)
  #define UINT64_BIT15_SHIFT    ((uint64_t)15)
  #define UINT64_BIT16_SHIFT    ((uint64_t)16)
  #define UINT64_BIT17_SHIFT    ((uint64_t)17)
  #define UINT64_BIT18_SHIFT    ((uint64_t)18)
  #define UINT64_BIT19_SHIFT    ((uint64_t)19)
  #define UINT64_BIT20_SHIFT    ((uint64_t)20)
  #define UINT64_BIT21_SHIFT    ((uint64_t)21)
  #define UINT64_BIT22_SHIFT    ((uint64_t)22)
  #define UINT64_BIT23_SHIFT    ((uint64_t)23)
  #define UINT64_BIT24_SHIFT    ((uint64_t)24)
  #define UINT64_BIT25_SHIFT    ((uint64_t)25)
  #define UINT64_BIT26_SHIFT    ((uint64_t)26)
  #define UINT64_BIT27_SHIFT    ((uint64_t)27)
  #define UINT64_BIT28_SHIFT    ((uint64_t)28)
  #define UINT64_BIT29_SHIFT    ((uint64_t)29)
  #define UINT64_BIT30_SHIFT    ((uint64_t)30)
  #define UINT64_BIT31_SHIFT    ((uint64_t)31)
  #define UINT64_BIT32_SHIFT    ((uint64_t)32)
  #define UINT64_BIT33_SHIFT    ((uint64_t)33)
  #define UINT64_BIT34_SHIFT    ((uint64_t)34)
  #define UINT64_BIT35_SHIFT    ((uint64_t)35)
  #define UINT64_BIT36_SHIFT    ((uint64_t)36)
  #define UINT64_BIT37_SHIFT    ((uint64_t)37)
  #define UINT64_BIT38_SHIFT    ((uint64_t)38)
  #define UINT64_BIT39_SHIFT    ((uint64_t)39)
  #define UINT64_BIT40_SHIFT    ((uint64_t)40)
  #define UINT64_BIT41_SHIFT    ((uint64_t)41)
  #define UINT64_BIT42_SHIFT    ((uint64_t)42)
  #define UINT64_BIT43_SHIFT    ((uint64_t)43)
  #define UINT64_BIT44_SHIFT    ((uint64_t)44)
  #define UINT64_BIT45_SHIFT    ((uint64_t)45)
  #define UINT64_BIT46_SHIFT    ((uint64_t)46)
  #define UINT64_BIT47_SHIFT    ((uint64_t)47)
  #define UINT64_BIT48_SHIFT    ((uint64_t)48)
  #define UINT64_BIT49_SHIFT    ((uint64_t)49)
  #define UINT64_BIT50_SHIFT    ((uint64_t)50)
  #define UINT64_BIT51_SHIFT    ((uint64_t)51)
  #define UINT64_BIT52_SHIFT    ((uint64_t)52)
  #define UINT64_BIT53_SHIFT    ((uint64_t)53)
  #define UINT64_BIT54_SHIFT    ((uint64_t)54)
  #define UINT64_BIT55_SHIFT    ((uint64_t)55)
  #define UINT64_BIT56_SHIFT    ((uint64_t)56)
  #define UINT64_BIT57_SHIFT    ((uint64_t)57)
  #define UINT64_BIT58_SHIFT    ((uint64_t)58)
  #define UINT64_BIT59_SHIFT    ((uint64_t)59)
  #define UINT64_BIT60_SHIFT    ((uint64_t)60)
  #define UINT64_BIT61_SHIFT    ((uint64_t)61)
  #define UINT64_BIT62_SHIFT    ((uint64_t)62)
  #define UINT64_MSB_SHIFT      ((uint64_t)63)

  #define UINT64_LSB_MASK       ((uint64_t)0x0000000000000001)
  #define UINT64_BIT1_MASK      ((uint64_t)0x0000000000000002)
  #define UINT64_BIT2_MASK      ((uint64_t)0x0000000000000004)
  #define UINT64_BIT3_MASK      ((uint64_t)0x0000000000000008)
  #define UINT64_BIT4_MASK      ((uint64_t)0x0000000000000010)
  #define UINT64_BIT5_MASK      ((uint64_t)0x0000000000000020)
  #define UINT64_BIT6_MASK      ((uint64_t)0x0000000000000040)
  #define UINT64_BIT7_MASK      ((uint64_t)0x0000000000000080)
  #define UINT64_BIT8_MASK      ((uint64_t)0x0000000000000100)
  #define UINT64_BIT9_MASK      ((uint64_t)0x0000000000000200)
  #define UINT64_BIT10_MASK     ((uint64_t)0x0000000000000400)
  #define UINT64_BIT11_MASK     ((uint64_t)0x0000000000000800)
  #define UINT64_BIT12_MASK     ((uint64_t)0x0000000000001000)
  #define UINT64_BIT13_MASK     ((uint64_t)0x0000000000002000)
  #define UINT64_BIT14_MASK     ((uint64_t)0x0000000000004000)
  #define UINT64_BIT15_MASK     ((uint64_t)0x0000000000008000)
  #define UINT64_BIT16_MASK     ((uint64_t)0x0000000000010000)
  #define UINT64_BIT17_MASK     ((uint64_t)0x0000000000020000)
  #define UINT64_BIT18_MASK     ((uint64_t)0x0000000000040000)
  #define UINT64_BIT19_MASK     ((uint64_t)0x0000000000080000)
  #define UINT64_BIT20_MASK     ((uint64_t)0x0000000000100000)
  #define UINT64_BIT21_MASK     ((uint64_t)0x0000000000200000)
  #define UINT64_BIT22_MASK     ((uint64_t)0x0000000000400000)
  #define UINT64_BIT23_MASK     ((uint64_t)0x0000000000800000)
  #define UINT64_BIT24_MASK     ((uint64_t)0x0000000001000000)
  #define UINT64_BIT25_MASK     ((uint64_t)0x0000000002000000)
  #define UINT64_BIT26_MASK     ((uint64_t)0x0000000004000000)
  #define UINT64_BIT27_MASK     ((uint64_t)0x0000000008000000)
  #define UINT64_BIT28_MASK     ((uint64_t)0x0000000010000000)
  #define UINT64_BIT29_MASK     ((uint64_t)0x0000000020000000)
  #define UINT64_BIT30_MASK     ((uint64_t)0x0000000040000000)
  #define UINT64_BIT31_MASK     ((uint64_t)0x0000000080000000)
  #define UINT64_BIT32_MASK     ((uint64_t)0x0000000100000000)
  #define UINT64_BIT33_MASK     ((uint64_t)0x0000000200000000)
  #define UINT64_BIT34_MASK     ((uint64_t)0x0000000400000000)
  #define UINT64_BIT35_MASK     ((uint64_t)0x0000000800000000)
  #define UINT64_BIT36_MASK     ((uint64_t)0x0000001000000000)
  #define UINT64_BIT37_MASK     ((uint64_t)0x0000002000000000)
  #define UINT64_BIT38_MASK     ((uint64_t)0x0000004000000000)
  #define UINT64_BIT39_MASK     ((uint64_t)0x0000008000000000)
  #define UINT64_BIT40_MASK     ((uint64_t)0x0000010000000000)
  #define UINT64_BIT41_MASK     ((uint64_t)0x0000020000000000)
  #define UINT64_BIT42_MASK     ((uint64_t)0x0000040000000000)
  #define UINT64_BIT43_MASK     ((uint64_t)0x0000080000000000)
  #define UINT64_BIT44_MASK     ((uint64_t)0x0000100000000000)
  #define UINT64_BIT45_MASK     ((uint64_t)0x0000200000000000)
  #define UINT64_BIT46_MASK     ((uint64_t)0x0000400000000000)
  #define UINT64_BIT47_MASK     ((uint64_t)0x0000800000000000)
  #define UINT64_BIT48_MASK     ((uint64_t)0x0001000000000000)
  #define UINT64_BIT49_MASK     ((uint64_t)0x0002000000000000)
  #define UINT64_BIT50_MASK     ((uint64_t)0x0004000000000000)
  #define UINT64_BIT51_MASK     ((uint64_t)0x0008000000000000)
  #define UINT64_BIT52_MASK     ((uint64_t)0x0010000000000000)
  #define UINT64_BIT53_MASK     ((uint64_t)0x0020000000000000)
  #define UINT64_BIT54_MASK     ((uint64_t)0x0040000000000000)
  #define UINT64_BIT55_MASK     ((uint64_t)0x0080000000000000)
  #define UINT64_BIT56_MASK     ((uint64_t)0x0100000000000000)
  #define UINT64_BIT57_MASK     ((uint64_t)0x0200000000000000)
  #define UINT64_BIT58_MASK     ((uint64_t)0x0400000000000000)
  #define UINT64_BIT59_MASK     ((uint64_t)0x0800000000000000)
  #define UINT64_BIT60_MASK     ((uint64_t)0x1000000000000000)
  #define UINT64_BIT61_MASK     ((uint64_t)0x2000000000000000)
  #define UINT64_BIT62_MASK     ((uint64_t)0x4000000000000000)
  #define UINT64_MSB_MASK       ((uint64_t)0x8000000000000000)
  #define UINT64_MASK           ((uint64_t)0xffffffffffffffff)

  /* ===========================================================================
                          Public constants and variables
  =========================================================================== */


  /* ===========================================================================
                          Public function declarations
  =========================================================================== */

#endif /* EXOTYPES__H */
