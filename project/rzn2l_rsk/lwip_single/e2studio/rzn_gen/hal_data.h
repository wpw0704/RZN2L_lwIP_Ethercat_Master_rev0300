/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_gpt.h"
#include "r_timer_api.h"
FSP_HEADER
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer0;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

#ifndef gpt0_callback
void gpt0_callback(timer_callback_args_t *p_args);
#endif
#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif

/** Error check the duplicated channel number, same GPT_INT number between MTU3 and GPT */
#if (1 == BSP_FEATURE_BSP_IRQ_GPT_SEL_SUPPORTED)
 #ifndef TIMER_GPT00_0_INT0_DISABLE
  #define TIMER_GPT00_0_INT0_DISABLE
 #else
  #ifdef TIMER_GPT00_0_INT0_ENABLE
   #error "GPT_INT0 of GPT_SEL cannot be duplicated"
  #endif
 #endif
 #ifndef TIMER_GPT00_0_INT1_DISABLE
  #define TIMER_GPT00_0_INT1_DISABLE
 #else
  #ifdef TIMER_GPT00_0_INT1_ENABLE
   #error "GPT_INT1 of GPT_SEL cannot be duplicated"
  #endif
 #endif
 #ifndef TIMER_GPT00_0_INT2_DISABLE
  #define TIMER_GPT00_0_INT2_DISABLE
 #else
  #ifdef TIMER_GPT00_0_INT2_ENABLE
   #error "GPT_INT2 of GPT_SEL cannot be duplicated"
  #endif
 #endif
 #ifndef TIMER_GPT00_0_INT3_DISABLE
  #define TIMER_GPT00_0_INT3_DISABLE
 #else
  #ifdef TIMER_GPT00_0_INT3_ENABLE
   #error "GPT_INT3 of GPT_SEL cannot be duplicated"
  #endif
 #endif
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
