/* generated HAL source file - do not edit */
#include "hal_data.h"

gpt_instance_ctrl_t g_timer0_ctrl;
#if 0
const gpt_extended_pwm_cfg_t g_timer0_pwm_extend =
{
#if defined(VECTOR_NUMBER_GPT0_UDF)
    .trough_ipl          = (BSP_IRQ_DISABLED),
    .trough_irq          = VECTOR_NUMBER_GPT0_UDF,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .trough_ipl          = FSP_NOT_DEFINED,
    .trough_irq          = VECTOR_NUMBER_GPT00_0_INT,
#else
    .trough_ipl          = (BSP_IRQ_DISABLED),
    .trough_irq          = FSP_INVALID_VECTOR,
#endif
    .poeg_link           = GPT_POEG_LINK_POEG0,
    .output_disable      =  GPT_OUTPUT_DISABLE_NONE,
    .adc_trigger         =  GPT_ADC_TRIGGER_NONE,
    .dead_time_count_up  = 0,
    .dead_time_count_down = 0,
    .adc_a_compare_match = 0,
    .adc_b_compare_match = 0,
    .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_adc    = GPT_INTERRUPT_SKIP_ADC_NONE,
    .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,
    .interrupt_skip_source_ext1 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext1  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_source_ext2 = GPT_INTERRUPT_SKIP_SOURCE_NONE,
    .interrupt_skip_count_ext2  = GPT_INTERRUPT_SKIP_COUNT_0,
    .interrupt_skip_func_ovf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_unf    = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_a  = GPT_INTERRUPT_SKIP_SELECT_NONE,
    .interrupt_skip_func_adc_b  = GPT_INTERRUPT_SKIP_SELECT_NONE,
};
#endif
const gpt_extended_cfg_t g_timer0_extend =
        { .gtioca =
        { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .gtiocb =
          { .output_enabled = false, .stop_level = GPT_PIN_LEVEL_LOW },
          .start_source = (gpt_source_t) (GPT_SOURCE_NONE), .stop_source = (gpt_source_t) (GPT_SOURCE_NONE), .clear_source =
                  (gpt_source_t) (GPT_SOURCE_NONE),
#if (0 == (0))
          .count_up_source = (gpt_source_t) (GPT_SOURCE_NONE),
          .count_down_source = (gpt_source_t) (GPT_SOURCE_NONE),
#else
    .count_up_source     = (gpt_source_t) ((GPT_PHASE_COUNTING_MODE_1_UP | (GPT_PHASE_COUNTING_MODE_1_DN << 16)) & 0x0000FFFFU),
    .count_down_source   = (gpt_source_t) (((GPT_PHASE_COUNTING_MODE_1_UP | (GPT_PHASE_COUNTING_MODE_1_DN << 16)) & 0xFFFF0000U) >> 16),
#endif
          .capture_a_source = (gpt_source_t) (GPT_SOURCE_NONE),
          .capture_b_source = (gpt_source_t) (GPT_SOURCE_NONE),
#if defined(VECTOR_NUMBER_GPT0_CCMPA)
    .capture_a_ipl       = (BSP_IRQ_DISABLED),
    .capture_a_irq       = VECTOR_NUMBER_GPT0_CCMPA,
    .capture_a_source_select = BSP_IRQ_DISABLED,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .capture_a_ipl       = FSP_NOT_DEFINED,
    .capture_a_irq       = VECTOR_NUMBER_GPT00_0_INT,
    .capture_a_source_select = ,
#else
          .capture_a_ipl = (BSP_IRQ_DISABLED),
          .capture_a_irq = FSP_INVALID_VECTOR, .capture_a_source_select = BSP_IRQ_DISABLED,
#endif
#if defined(VECTOR_NUMBER_GPT0_CCMPB)
    .capture_b_irq       = VECTOR_NUMBER_GPT0_CCMPB,
    .capture_b_ipl       = (BSP_IRQ_DISABLED),
    .capture_b_source_select = BSP_IRQ_DISABLED,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .capture_b_irq       = VECTOR_NUMBER_GPT00_0_INT,
    .capture_b_ipl       = FSP_NOT_DEFINED,
    .capture_b_source_select = ,
#else
          .capture_b_ipl = (BSP_IRQ_DISABLED),
          .capture_b_irq = FSP_INVALID_VECTOR, .capture_b_source_select = BSP_IRQ_DISABLED,
#endif
          .compare_match_value =
          { /* CMP_A */(uint32_t) 0x0, /* CMP_B */(uint32_t) 0x0 },
          .compare_match_status = (0U << 1U) | 0U, .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE, .capture_filter_gtiocb =
                  GPT_CAPTURE_FILTER_NONE,
#if 0
    .p_pwm_cfg                   = &g_timer0_pwm_extend,
#else
          .p_pwm_cfg = NULL,
#endif
#if defined(VECTOR_NUMBER_GPT0_DTE)
    .dead_time_ipl       = (BSP_IRQ_DISABLED),
    .dead_time_irq       = VECTOR_NUMBER_GPT0_DTE,
    .dead_time_error_source_select = BSP_IRQ_DISABLED,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .dead_time_ipl       = FSP_NOT_DEFINED,
    .dead_time_irq       = VECTOR_NUMBER_GPT00_0_INT,
    .dead_time_error_source_select = ,
#else
          .dead_time_ipl = (BSP_IRQ_DISABLED),
          .dead_time_irq = FSP_INVALID_VECTOR, .dead_time_error_source_select = BSP_IRQ_DISABLED,
#endif
          .icds = 0,
#if (2U == BSP_FEATURE_GPT_REGISTER_MASK_TYPE)
 #if (1U == BSP_FEATURE_GPT_INPUT_CAPTURE_SIGNAL_SELECTABLE)
    .gtioc_isel          = 0,
 #endif
#endif
#if defined(VECTOR_NUMBER_GPT0_OVF)
    .cycle_end_source_select = BSP_IRQ_DISABLED,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .cycle_end_source_select = ,
#else
          .cycle_end_source_select = BSP_IRQ_DISABLED,
#endif
#if defined(VECTOR_NUMBER_GPT0_UDF)
    .trough_source_select = BSP_IRQ_DISABLED,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .trough_source_select = ,
#else
          .trough_source_select = BSP_IRQ_DISABLED,
#endif
#if 0
    .gtior_setting.gtior_b.gtioa  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.oadflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.oahld  = 0U,
    .gtior_setting.gtior_b.oae    = (uint32_t) false,
    .gtior_setting.gtior_b.oadf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfaen  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsa  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
    .gtior_setting.gtior_b.gtiob  = (0U << 4U) | (0U << 2U) | (0U << 0U),
    .gtior_setting.gtior_b.obdflt = (uint32_t) GPT_PIN_LEVEL_LOW,
    .gtior_setting.gtior_b.obhld  = 0U,
    .gtior_setting.gtior_b.obe    = (uint32_t) false,
    .gtior_setting.gtior_b.obdf   = (uint32_t) GPT_GTIOC_DISABLE_PROHIBITED,
    .gtior_setting.gtior_b.nfben  = ((uint32_t) GPT_CAPTURE_FILTER_NONE & 1U),
    .gtior_setting.gtior_b.nfcsb  = ((uint32_t) GPT_CAPTURE_FILTER_NONE >> 1U),
#else
          .gtior_setting.gtior = 0U,
#endif
        };
const timer_cfg_t g_timer0_cfg =
{ .mode = TIMER_MODE_PERIODIC,
/* Actual period: 0.004 seconds. Actual duty: 50%. */.period_counts = (uint32_t) 0x186a00,
  .duty_cycle_counts = 0xc3500, .source_div = (timer_source_div_t) 0, .channel = GPT_CHANNEL_UNIT0_0,
#if (1 == BSP_FEATURE_BSP_IRQ_GPT_SEL_SUPPORTED)
    .p_callback          = NULL,
#else
  .p_callback = gpt0_callback,
#endif
  .p_context = NULL,
  .p_extend = &g_timer0_extend,
#if defined(VECTOR_NUMBER_GPT0_OVF)
    .cycle_end_ipl       = (12),
    .cycle_end_irq       = VECTOR_NUMBER_GPT0_OVF,
#elif defined(VECTOR_NUMBER_GPT00_0_INT)
    .cycle_end_ipl       = FSP_NOT_DEFINED,
    .cycle_end_irq       = VECTOR_NUMBER_GPT00_0_INT,
#else
  .cycle_end_ipl = (12),
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer0 =
{ .p_ctrl = &g_timer0_ctrl, .p_cfg = &g_timer0_cfg, .p_api = &g_timer_on_gpt };
void g_hal_init(void)
{
    g_common_init ();
}
