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
 * Date:          23/05/2018 (dd MM YYYY)
 * Author:        Yoann Rebischung
 * Description:   Driver of ST25DV Eeprom NFC/RFID
 *
 */

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
#include <string.h>

/* Self include */
#include "ST25DV.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/
/* Slave addresses (see datasheet Rev 6 B.1 Device select codes) */
#define ST25DV_I2C_USR_DYN_FTM_ADDR                         (uint8_t)0x53     /* E2 = 0 */
#define ST25DV_I2C_SYS_ADDR                                 (uint8_t)0x57     /* E2 = 1 */

/* Command Codes (RF) Not used here */
#define ST25DV_RF_CMD_INVENTORY                             (uint8_t)0x01
#define ST25DV_RF_CMD_STAY_QUIET                            (uint8_t)0x02
#define ST25DV_RF_CMD_RD_SINGLE_BLOCK                       (uint8_t)0x20
#define ST25DV_RF_CMD_WR_SINGLE_BLOCK                       (uint8_t)0x21
#define ST25DV_RF_CMD_LOCK_BLOCK                            (uint8_t)0x22
#define ST25DV_RF_CMD_RD_MULTPLIE_BLOCKS                    (uint8_t)0x23
#define ST25DV_RF_CMD_WR_MULTIPLE_BLOCKS                    (uint8_t)0x24
#define ST25DV_RF_CMD_SELECT                                (uint8_t)0x25
#define ST25DV_RF_CMD_RESET_TO_READY                        (uint8_t)0x26
#define ST25DV_RF_CMD_WR_AFI                                (uint8_t)0x27
#define ST25DV_RF_CMD_LOCK_AFI                              (uint8_t)0x28
#define ST25DV_RF_CMD_WR_DSFID                              (uint8_t)0x29
#define ST25DV_RF_CMD_LOCK_DSFID                            (uint8_t)0x2A
#define ST25DV_RF_CMD_GET_SYST_INFO                         (uint8_t)0x2B
#define ST25DV_RF_CMD_GET_MULTIPLE_BLOCK_SECU_STATUS        (uint8_t)0x2C
#define ST25DV_RF_CMD_EXTD_RD_SINGLE_BLOCK                  (uint8_t)0x30
#define ST25DV_RF_CMD_EXTD_WR_SINGLE_BLOCK                  (uint8_t)0x31
#define ST25DV_RF_CMD_EXTD_LOCK_BLOCK                       (uint8_t)0x32
#define ST25DV_RF_CMD_EXTD_RD_MULTIPLE_BLOCKS               (uint8_t)0x33
#define ST25DV_RF_CMD_EXTD_WR_MULTIPLE_BLOCKS               (uint8_t)0x34
#define ST25DV_RF_CMD_EXTD_GET_SYS_INFO                     (uint8_t)0x3B
#define ST25DV_RF_CMD_EXTD_GET_MULTIPLE_BLOCK_SECU_STATUS   (uint8_t)0x3C
#define ST25DV_RF_CMD_RD_CFG                                (uint8_t)0xA0
#define ST25DV_RF_CMD_WR_CFG                                (uint8_t)0xA1
#define ST25DV_RF_CMD_MANAGE_GPO                            (uint8_t)0xA9
#define ST25DV_RF_CMD_WR_MSG                                (uint8_t)0xAA
#define ST25DV_RF_CMD_RD_MSG_LEN                            (uint8_t)0xAB
#define ST25DV_RF_CMD_RD_MSG                                (uint8_t)0xAC
#define ST25DV_RF_CMD_RD_DYN_CFG                            (uint8_t)0xAD
#define ST25DV_RF_CMD_WR_DYN_CFG                            (uint8_t)0xAE
#define ST25DV_RF_CMD_WR_PASSWORD                           (uint8_t)0xB1
#define ST25DV_RF_CMD_PRESENT_PASSWORD                      (uint8_t)0xB3
#define ST25DV_RF_CMD_FAST_RD_SINGLE_BLOCK                  (uint8_t)0xC0
#define ST25DV_RF_CMD_FAST_RD_MULTPLE_BLOCKS                (uint8_t)0xC3
#define ST25DV_RF_CMD_FAST_EXTD_RD_SINGLE_BLOCK             (uint8_t)0xC4
#define ST25DV_RF_CMD_FAST_EXTD_RD_MULTIPLE_BLOCKS          (uint8_t)0xC5
#define ST25DV_RF_CMD_FAST_WR_MSG                           (uint8_t)0xCA
#define ST25DV_RF_CMD_FAST_RD_MSG_LEN                       (uint8_t)0xCB
#define ST25DV_RF_CMD_FAST_RD_MSG                           (uint8_t)0xCC
#define ST25DV_RF_CMD_FAST_RD_DYN_CFG                       (uint8_t)0xCD
#define ST25DV_RF_CMD_FAST_WR_DYN_CFG                       (uint8_t)0xCE

/* Error Codes (RF) Not used here */
#define ST25DV_RF_ERROR_CMD_NOT_SUPPORTED                   (uint8_t)0x01
#define ST25DV_RF_ERROR_CMD_NOT_RECOGNIZED                  (uint8_t)0x02
#define ST25DV_RF_ERROR_OPT_NOT_SUPPORTED                   (uint8_t)0x03
#define ST25DV_RF_ERROR_NO_INFO                             (uint8_t)0x0F
#define ST25DV_RF_ERROR_SPEC_BLCK_NA                        (uint8_t)0x10
#define ST25DV_RF_ERROR_SPEC_BLCK_ALRDY_LOCKED              (uint8_t)0x11
#define ST25DV_RF_ERROR_SPEC_BLCK_LOCKED                    (uint8_t)0x12
#define ST25DV_RF_ERROR_SPEC_BLCK_NOT_SCFLY_PROG            (uint8_t)0x13
#define ST25DV_RF_ERROR_SPEC_BLCK_NOT_SCFLY_LCKED           (uint8_t)0x14
#define ST25DV_RF_ERROR_SPEC_BLCK_PROTECTED                 (uint8_t)0x15

/* Register Map System Cfg E2 = 1 */
#define SYS_REG_GPO                                         (uint16_t)0x0000
#define SYS_REG_IT_TIME                                     (uint16_t)0x0001
#define SYS_REG_EH_MODE                                     (uint16_t)0x0002
#define SYS_REG_RF_MNGT                                     (uint16_t)0x0003
#define SYS_REG_RFA1SS                                      (uint16_t)0x0004
#define SYS_REG_ENDA1                                       (uint16_t)0x0005
#define SYS_REG_RFA2SS                                      (uint16_t)0x0006
#define SYS_REG_ENDA2                                       (uint16_t)0x0007
#define SYS_REG_RFA3SS                                      (uint16_t)0x0008
#define SYS_REG_ENDA3                                       (uint16_t)0x0009
#define SYS_REG_RFA4SS                                      (uint16_t)0x000A
#define SYS_REG_I2CSS                                       (uint16_t)0x000B
#define SYS_REG_LOCK_CCFILE                                 (uint16_t)0x000C
#define SYS_REG_MB_MODE                                     (uint16_t)0x000D
#define SYS_REG_MB_WDG                                      (uint16_t)0x000E
#define SYS_REG_LOCK_CFG                                    (uint16_t)0x000F
#define SYS_REG_LOCK_DSFID                                  (uint16_t)0x0010
#define SYS_REG_LOCK_AFI                                    (uint16_t)0x0011
#define SYS_REG_DSFID                                       (uint16_t)0x0012
#define SYS_REG_AFI                                         (uint16_t)0x0013
#define SYS_REG_MEM_SIZE                                    (uint16_t)0x0014
#define SYS_REG_BLK_SIZE                                    (uint16_t)0x0016
#define SYS_REG_IC_REF                                      (uint16_t)0x0017
#define SYS_REG_UID                                         (uint16_t)0x0018
#define SYS_REG_IC_REV                                      (uint16_t)0x0020
#define SYS_REG_I2C_PSW                                     (uint16_t)0x0900

/* Register Map Dynamic Reg E2 = 0 */
#define DYN_REG_GPO_CTRL                                    (uint16_t)0x2000
#define DYN_REG_EH_CTRL                                     (uint16_t)0x2002
#define DYN_REG_RF_MNGT                                     (uint16_t)0x2003
#define DYN_REG_I2C_SSO                                     (uint16_t)0x2004
#define DYN_REG_IT_STS                                      (uint16_t)0x2005
#define DYN_REG_MB_CTRL                                     (uint16_t)0x2006
#define DYN_REG_MB_LEN                                      (uint16_t)0x2007

/* Register Map Fast Transfer Mode/Mailbox Reg E2 = 0 */
#define FTM_REG_BYTE_START                                  (uint16_t)0x2008
#define FTM_REG_SIZE                                        (uint8_t)UINT8_MAX

/* Msk Pos */
#define RFAxSS_PWD_CTRL_Ax_POS                              (uint8_t)0
#define RFAxSS_PWD_CTRL_Ax_MSK                              (uint8_t)0x03
#define RFAxSS_RW_PROTECTION_Ax_POS                         (uint8_t)2
#define RFAxSS_RW_PROTECTION_Ax_MSK                         (uint8_t)0x0C

/* Block Addresses */
#define ST25DV_USER_MEM_START_ADDR                          (uint16_t)0x0000

//#if (ST25DV == 4)
#define ST25DV04K_USER_MEM_END_ADDR                         (uint16_t)0x007F
#define ST25DV04K_IC_REF                                    (uint8_t)0x24
//#elif (ST25DV == 16)
#define ST25DV16K_USER_MEM_END_ADDR                         (uint16_t)0x01FF
#define ST25DV16K_IC_REF                                    (uint8_t)0x26
//#elif (ST25DV == 64)
#define ST25DV64K_USER_MEM_END_ADDR                         (uint16_t)0x07FF
#define ST25DV64K_IC_REF                                    (uint8_t)0x26
//#else
//#error "ST25DV not supported !"
//#endif

#define DEFAULT_PASSWORD                                    (uint64_t)0u

#define EXIT_ERROR_CHECK(error)  do {     \
      if((error != ST25DV_ERROR_NONE))    \
      {                                   \
         return error;                    \
      }                                   \
   }while(0);

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/

/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static e_ST25DV_Error_t eReadRegisters(uint8_t p_u8Addr, uint16_t p_u16Reg, uint8_t * p_pu8Value, uint8_t p_u8RegNumber);
static e_ST25DV_Error_t eWriteRegisters(uint8_t p_u8Addr, uint16_t p_u16Reg, uint8_t * p_pau8Data, uint8_t p_u8DataSize);

   
static e_ST25DV_Error_t eST25DV_SecuritySessionGet(uint8_t * p_pu8SSO);
static e_ST25DV_Error_t eST25DV_SecuritySessionClose(void);
//static e_ST25DV_Error_t eST25DV_PasswordSet(uint64_t p_u64Pass);
static e_ST25DV_Error_t eST25DV_PasswordPresent(uint64_t p_u64Pass);
   
/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/
static uint8_t g_u8ST25DVInitialized = 0u;

static s_ST25DV_Context_t g_sST25DVContext;
static uint8_t g_u8ST25DVCommFailure = 0u;

/****************************************************************************************
 * Public functions
 ****************************************************************************************/
/**@brief  Initialize the ST25DV module.
 * @param[in]  p_sContext.
 * @return Error Code.
 */
e_ST25DV_Error_t eST25DV_ContextSet(s_ST25DV_Context_t p_sContext)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_PARAM;
   uint8_t l_u8ICRef = 0u;
   uint8_t l_u8ICRefRead = 0u;
   uint8_t l_au8MemRead[2u] = { 0u };
   uint16_t l_u16Mem = 0u;

   if(   (p_sContext.fp_vDelay_ms != NULL)
      && (p_sContext.fp_u32I2C_Write != NULL)
      && (p_sContext.fp_u32I2C_Read != NULL) )
   {
      g_sST25DVContext.fp_vDelay_ms = p_sContext.fp_vDelay_ms;
      g_sST25DVContext.fp_u32I2C_Write = p_sContext.fp_u32I2C_Write;
      g_sST25DVContext.fp_u32I2C_Read = p_sContext.fp_u32I2C_Read;
      g_sST25DVContext.eEepromSize = p_sContext.eEepromSize;

      /* Check Ref IC */
      l_eError = eReadRegisters(ST25DV_I2C_SYS_ADDR, SYS_REG_IC_REF, &l_u8ICRefRead, 1u);
      EXIT_ERROR_CHECK(l_eError);
      /* Check Mem Size */
      l_eError = eReadRegisters(ST25DV_I2C_SYS_ADDR, SYS_REG_MEM_SIZE, l_au8MemRead, 2u);
      EXIT_ERROR_CHECK(l_eError);

      switch(g_sST25DVContext.eEepromSize)
      {
         case ST25DV_EEPROM_SIZE_64K:
            l_u8ICRef = ST25DV64K_IC_REF;
            l_u16Mem = ST25DV64K_USER_MEM_END_ADDR;
            break;
         case ST25DV_EEPROM_SIZE_16K:
            l_u8ICRef = ST25DV16K_IC_REF;
            l_u16Mem = ST25DV16K_USER_MEM_END_ADDR;
            break;
         case ST25DV_EEPROM_SIZE_04K:
            l_u8ICRef = ST25DV04K_IC_REF;
            l_u16Mem = ST25DV04K_USER_MEM_END_ADDR;
            break;
         default:
            break;
      }

      if(    (l_u8ICRef == l_u8ICRefRead)
         &&  (l_u16Mem == (uint16_t)(((uint16_t)l_au8MemRead[1u] << 8u) + l_au8MemRead[0u])) )
      {
         uint8_t l_u8SSO = 8u;
         
         l_eError = eST25DV_SecuritySessionClose();
         EXIT_ERROR_CHECK(l_eError);
         l_eError = eST25DV_SecuritySessionGet(&l_u8SSO);
         EXIT_ERROR_CHECK(l_eError);
         
         if(l_u8SSO != 1u)
         {
            l_eError = eST25DV_PasswordPresent(DEFAULT_PASSWORD);
            EXIT_ERROR_CHECK(l_eError);
            l_eError = eST25DV_SecuritySessionGet(&l_u8SSO);
            EXIT_ERROR_CHECK(l_eError);
         }         
         
         if(l_u8SSO != 1u)
         {
            g_u8ST25DVInitialized = 0u;   
            l_eError = ST25DV_ERROR_ACCESS;
         }
         else
         {
            g_u8ST25DVInitialized = 1u;  
            l_eError = ST25DV_ERROR_NONE;          
         }         
      }
   }

   return l_eError;
}

e_ST25DV_Error_t eST25DV_GPOConfigure(uint8_t p_u8MskGPO)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_ACCESS;
   
   if(g_u8ST25DVInitialized == 1u)
   {
      l_eError = eWriteRegisters(ST25DV_I2C_SYS_ADDR, SYS_REG_GPO, &p_u8MskGPO , 1u);
   }
   
   return l_eError;
}

e_ST25DV_Error_t eST25DV_EndAreaSet(uint8_t p_u8Area, uint16_t p_u16EndAddress)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_PARAM;
   
   return l_eError;
}

uint8_t u8ST25DV_IsAvailable(void)
{
   return ((g_u8ST25DVInitialized == 1u) && (g_u8ST25DVCommFailure == 0u))?1u:0u;
}

e_ST25DV_Error_t eST25DV_PasswordSet(uint64_t p_u64Pass)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_PARAM;
   uint8_t l_u8Size = 0u;
   uint8_t l_au8Cmd[20u] = { 0u };
   
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0xFF00000000000000) >> 56u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00FF000000000000) >> 48u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000FF0000000000) >> 40u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000FF00000000) >> 32u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000FF000000) >> 24u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000000000FF0000) >> 16u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000000000FF00) >> 8u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000000000FF)));
   l_au8Cmd[l_u8Size++] = 0x07; 
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0xFF00000000000000) >> 56u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00FF000000000000) >> 48u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000FF0000000000) >> 40u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000FF00000000) >> 32u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000FF000000) >> 24u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000000000FF0000) >> 16u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000000000FF00) >> 8u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000000000FF)));
   
   l_eError = eWriteRegisters(ST25DV_I2C_SYS_ADDR, SYS_REG_I2C_PSW, l_au8Cmd , l_u8Size);
   EXIT_ERROR_CHECK(l_eError);
   
   return l_eError;
}
/****************************************************************************************
 * Private functions
 ****************************************************************************************/
/**@brief Function to read on registers.
 * @param[in] p_u8Addr
 * @param[in] p_u16Reg
 * @param[out] p_pu8Value
 * @param[in] p_u8RegNumber
 * @return Error code
 */
static e_ST25DV_Error_t eReadRegisters(uint8_t p_u8Addr, uint16_t p_u16Reg, uint8_t * p_pu8Value, uint8_t p_u8RegNumber)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_COMM;
   uint8_t l_au8Reg[2u] = { 0u };

   if(   (  (p_u8Addr == ST25DV_I2C_USR_DYN_FTM_ADDR)
         || (p_u8Addr == ST25DV_I2C_SYS_ADDR) )
      && (p_pu8Value != NULL) )
   {
      l_au8Reg[0u] = (uint8_t)((p_u16Reg & 0xFF00) >> 8u);
      l_au8Reg[1u] = (uint8_t)(p_u16Reg & 0x00FF);

      if((*g_sST25DVContext.fp_u32I2C_Read)(p_u8Addr, l_au8Reg, 2u, p_pu8Value, p_u8RegNumber) == 0u)
      {
         l_eError = ST25DV_ERROR_NONE;
         g_u8ST25DVCommFailure = 0u;
      }
      else
      {
         g_u8ST25DVCommFailure = 1u;
      }
   }
   else
   {
      l_eError = ST25DV_ERROR_PARAM;
   }

   return l_eError;
}

/**@brief Function to write on register.
 * @param[in] p_u8Addr
 * @param[in] p_u16Reg
 * @param[in] p_pau8Data
 * @param[in] p_u8DataSize
 * @return Error code
 */
static e_ST25DV_Error_t eWriteRegisters(uint8_t p_u8Addr, uint16_t p_u16Reg, uint8_t * p_pau8Data, uint8_t p_u8DataSize)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_COMM;
   uint8_t l_au8Reg[256u] = { 0u };
   uint8_t l_u8Size = 0u;

   if(   (  (p_u8Addr == ST25DV_I2C_USR_DYN_FTM_ADDR)
         || (p_u8Addr == ST25DV_I2C_SYS_ADDR) )
      && (p_pau8Data != NULL) )
   {
      l_au8Reg[l_u8Size++] = (uint8_t)((p_u16Reg & 0xFF00) >> 8u);
      l_au8Reg[l_u8Size++] = (uint8_t)(p_u16Reg & 0x00FF);

      memcpy(&l_au8Reg[l_u8Size], p_pau8Data, p_u8DataSize);
      l_u8Size += p_u8DataSize;

      if((*g_sST25DVContext.fp_u32I2C_Write)(p_u8Addr, l_au8Reg, l_u8Size) == 0u)
      {
         l_eError = ST25DV_ERROR_NONE;
         g_u8ST25DVCommFailure = 0u;
      }
      else
      {
         g_u8ST25DVCommFailure = 1u;
      }
   }
   else
   {
      l_eError = ST25DV_ERROR_PARAM;
   }

   return l_eError;
}

/**@brief Function to write specific bits of one register.
 * @param[in] p_u8Register
 * @param[in] p_u8Data
 * @param[in] p_u8Pos
 * @param[in] p_u8Mask
 * @return Error code
 */
//static e_ST25DV_Error_t eWriteBitsReg(uint16_t p_u16Register, uint8_t p_u8Value, uint8_t p_u8Pos, uint8_t p_u8Mask)
//{
//   uint8_t l_u8RegVal = 0u;
//   e_ST25DV_Error_t l_eError = ST25DV_ERROR_COMM;

//   l_eError = eReadRegisters(p_u16Register, &l_u8RegVal, 1u);

//   if(l_eError == ST25DV_ERROR_NONE)
//   {
//      l_u8RegVal &= ~p_u8Mask;

//      l_u8RegVal |= ((uint8_t)p_u8Value << p_u8Pos);

//      l_eError = eWriteRegister(p_u8Register, l_u8RegVal);
//   }

//   return l_eError;
//}

static e_ST25DV_Error_t eST25DV_SecuritySessionGet(uint8_t * p_pu8SSO)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_PARAM;
   
   if(p_pu8SSO != NULL)
   {
      l_eError = eReadRegisters(ST25DV_I2C_USR_DYN_FTM_ADDR, DYN_REG_I2C_SSO, p_pu8SSO, 1u);
      EXIT_ERROR_CHECK(l_eError);
   }
   
   return l_eError;
}

static e_ST25DV_Error_t eST25DV_SecuritySessionClose(void)
{
   return eST25DV_PasswordPresent(DEFAULT_PASSWORD + 1u);
}

static e_ST25DV_Error_t eST25DV_PasswordPresent(uint64_t p_u64Pass)
{
   e_ST25DV_Error_t l_eError = ST25DV_ERROR_PARAM;
   uint8_t l_u8Size = 0u;
   uint8_t l_au8Cmd[20u] = { 0u };
   
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0xFF00000000000000) >> 56u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00FF000000000000) >> 48u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000FF0000000000) >> 40u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000FF00000000) >> 32u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000FF000000) >> 24u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000000000FF0000) >> 16u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000000000FF00) >> 8u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000000000FF)));
   l_au8Cmd[l_u8Size++] = 0x09; 
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0xFF00000000000000) >> 56u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00FF000000000000) >> 48u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000FF0000000000) >> 40u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000FF00000000) >> 32u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000FF000000) >> 24u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x0000000000FF0000) >> 16u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x000000000000FF00) >> 8u));
   l_au8Cmd[l_u8Size++] = ((uint8_t)((((uint64_t)p_u64Pass) & 0x00000000000000FF)));
   
   l_eError = eWriteRegisters(ST25DV_I2C_SYS_ADDR, SYS_REG_I2C_PSW, l_au8Cmd , l_u8Size);
   EXIT_ERROR_CHECK(l_eError);
   
   return l_eError;
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/


