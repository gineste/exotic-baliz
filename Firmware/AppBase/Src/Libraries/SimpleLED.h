/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 */
#ifndef SIMPLE_LED_H
#define SIMPLE_LED_H

/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <stdint.h>
 
/****************************************************************************************
 * Defines
 ****************************************************************************************/
#define LED_RED_OFF()      do {                                      \
   if(g_sSimpleLedContext.fp_vPinSet_t != NULL) {                    \
      g_sSimpleLedContext.fp_vPinSet_t(g_sSimpleLedContext.u32PinR); \
   }                                                                 \
}while(0);

#define LED_GREEN_OFF()      do {                                    \
   if(g_sSimpleLedContext.fp_vPinSet_t != NULL) {                    \
      g_sSimpleLedContext.fp_vPinSet_t(g_sSimpleLedContext.u32PinG); \
   }                                                                 \
}while(0);

#define LED_BLUE_OFF()      do {                                     \
   if(g_sSimpleLedContext.fp_vPinSet_t != NULL) {                    \
      g_sSimpleLedContext.fp_vPinSet_t(g_sSimpleLedContext.u32PinB); \
   }                                                                 \
}while(0);

#define LED_RED_ON()      do {                                         \
   if(g_sSimpleLedContext.fp_vPinClear_t != NULL) {                    \
      g_sSimpleLedContext.fp_vPinClear_t(g_sSimpleLedContext.u32PinR); \
   }                                                                   \
}while(0);

#define LED_GREEN_ON()      do {                                       \
   if(g_sSimpleLedContext.fp_vPinClear_t != NULL) {                    \
      g_sSimpleLedContext.fp_vPinClear_t(g_sSimpleLedContext.u32PinG); \
   }                                                                   \
}while(0);

#define LED_BLUE_ON()      do {                                        \
   if(g_sSimpleLedContext.fp_vPinClear_t != NULL) {                    \
      g_sSimpleLedContext.fp_vPinClear_t(g_sSimpleLedContext.u32PinB); \
   }                                                                   \
}while(0);

/****************************************************************************************
 * Type definitions
 ****************************************************************************************/
typedef struct _SIMPLE_LED_PIN_ {
   uint32_t u32PinR;
   uint32_t u32PinG;
   uint32_t u32PinB;
   void (*fp_vPinClear_t)(uint32_t p_u32Pin);
   void (*fp_vPinSet_t)(uint32_t p_u32Pin);
}s_SimpleLED_Context_t;

typedef enum _SIMPLE_LED_COLOR_ {
	LED_RED = 0u,
	LED_GREEN,
	LED_BLUE,
	LED_YELLOW,
	LED_MAGENTA,
	LED_CYAN,
	LED_WHITE,
   LED_BLACK,
	LED_DEFAULTCOLOR
}e_SimpleLED_Color_t;

typedef struct _SIMPLE_LED_EFFECT_ {
   uint32_t u32TimeOn;
   uint32_t u32TimeOff;
   e_SimpleLED_Color_t eColorOn;
   e_SimpleLED_Color_t eColorOff;
   uint16_t u16Counter;
}s_SimpleLED_Effect_t;

#if (ENABLE_LED == 1)
   extern e_SimpleLED_Color_t g_eLastColor;
   extern s_SimpleLED_Context_t g_sSimpleLedContext;
#endif

/****************************************************************************************
 * Public function declarations
 ****************************************************************************************/
void vSimpleLED_Init(s_SimpleLED_Context_t p_eContext);
void vSimpleLED_EffectSet(s_SimpleLED_Effect_t p_eEffect);
void vSimpleLED_Off(void);
void vSimpleLED_EffectStop(void);
void vSimpleLED_PreviousColorSet(void);
void vSimpleLED_ColorSet(e_SimpleLED_Color_t p_eColor);
uint8_t u8SimpleLED_IsEffectInProgress(void);

static __inline void vSimpleLED_RED(void);
static __inline void vSimpleLED_GREEN(void);
static __inline void vSimpleLED_BLUE(void);
static __inline void vSimpleLED_CYAN(void);
static __inline void vSimpleLED_YELLOW(void);
static __inline void vSimpleLED_MAGENTA(void);
static __inline void vSimpleLED_WHITE(void);
static __inline void vSimpleLED_BLACK(void);


/**@brief Function for set color RED. 
 * @return None
 */
static __inline void vSimpleLED_RED(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_ON();
      LED_GREEN_OFF();
      LED_BLUE_OFF();
   }
   g_eLastColor = LED_RED;
#endif
}
/**@brief Function for set color GREEN. 
 * @return None
 */
static __inline void vSimpleLED_GREEN(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_OFF();
      LED_GREEN_ON();
      LED_BLUE_OFF();
   }
   g_eLastColor = LED_GREEN;
#endif
}
/**@brief Function for set color BLUE. 
 * @return None
 */
static __inline void vSimpleLED_BLUE(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_OFF();
      LED_GREEN_OFF();
      LED_BLUE_ON();
   }
   g_eLastColor = LED_BLUE;
#endif
}
/**@brief Function for set color CYAN. 
 * @return None
 */
static __inline void vSimpleLED_CYAN(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_OFF();
      LED_GREEN_ON();
      LED_BLUE_ON();
   }
   g_eLastColor = LED_CYAN;
#endif
}
/**@brief Function for set color YELLOW. 
 * @return None
 */
static __inline void vSimpleLED_YELLOW(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_ON();
      LED_GREEN_ON();
      LED_BLUE_OFF();
   }
   g_eLastColor = LED_YELLOW;
#endif
}
/**@brief Function for set color MAGENTA. 
 * @return None
 */
static __inline void vSimpleLED_MAGENTA(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_ON();
      LED_GREEN_OFF();
      LED_BLUE_ON();
   }
   g_eLastColor = LED_MAGENTA;
#endif
}
/**@brief Function for set color WHITE. 
 * @return None
 */
static __inline void vSimpleLED_WHITE(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_ON();
      LED_GREEN_ON();
      LED_BLUE_ON();
   }
   g_eLastColor = LED_WHITE;
#endif
}
/**@brief Function for set color DARK. 
 * @return None
 */
static __inline void vSimpleLED_BLACK(void)
{
#if (ENABLE_LED == 1)
   if(u8SimpleLED_IsEffectInProgress() == 0u)
   {
      LED_RED_OFF();
      LED_GREEN_OFF();
      LED_BLUE_OFF();
   }
   g_eLastColor = LED_BLACK;
#endif
}

#endif /* SIMPLE_LED_H */
