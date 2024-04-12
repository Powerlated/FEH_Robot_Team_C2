/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v14.0
processor: MK60DN512xxx10
package_id: MK60DN512VLQ10
mcu_data: ksdk2_0
processor_version: 13.0.1
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 97c47cc8-e724-4f1a-8b08-b4d66c59e471
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * FTM1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM1'
- type: 'ftm'
- mode: 'QuadratureDecoder'
- custom_name_enabled: 'false'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM1'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - prescale: 'kFTM_Prescale_Divide_1'
      - bdmMode: 'kFTM_BdmMode_0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM1_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - ftm_quadrature_decoder_mode:
    - timerModuloVal: '0xFFFF'
    - timerInitVal: '0'
    - ftm_quad_decoder_mode: 'kFTM_QuadCountAndDir'
    - ftm_phase_a_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
    - ftm_phase_b_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM1_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0U,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0UL,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_phase_params_t FTM1_phaseAParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};
const ftm_phase_params_t FTM1_phaseBParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};

static void FTM1_init(void) {
  FTM_Init(FTM1_PERIPHERAL, &FTM1_config);
/* Initialization of the timer initial value and modulo value */
  FTM_SetQuadDecoderModuloValue(FTM1_PERIPHERAL, 0,0);
  FTM_SetupQuadDecode(FTM1_PERIPHERAL, &FTM1_phaseAParams, &FTM1_phaseBParams, kFTM_QuadCountAndDir);
  FTM_StartTimer(FTM1_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * FTM2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM2'
- type: 'ftm'
- mode: 'QuadratureDecoder'
- custom_name_enabled: 'false'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM2'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - prescale: 'kFTM_Prescale_Divide_1'
      - bdmMode: 'kFTM_BdmMode_0'
      - pwmSyncMode: 'kFTM_SoftwareTrigger'
      - reloadPoints: ''
      - faultMode: 'kFTM_Fault_Disable'
      - faultFilterValue: '0'
      - deadTimePrescale: 'kFTM_Deadtime_Prescale_1'
      - deadTimeValue: '0'
      - extTriggers: ''
      - chnlInitState: ''
      - chnlPolarity: ''
      - useGlobalTimeBase: 'false'
    - timer_interrupts: ''
    - enable_irq: 'false'
    - ftm_interrupt:
      - IRQn: 'FTM2_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
  - ftm_quadrature_decoder_mode:
    - timerModuloVal: '0xFFFF'
    - timerInitVal: '0'
    - ftm_quad_decoder_mode: 'kFTM_QuadCountAndDir'
    - ftm_phase_a_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
    - ftm_phase_b_params:
      - enablePhaseFilter: 'false'
      - phasePolarity: 'kFTM_QuadPhaseNormal'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM2_config = {
  .prescale = kFTM_Prescale_Divide_1,
  .bdmMode = kFTM_BdmMode_0,
  .pwmSyncMode = kFTM_SoftwareTrigger,
  .reloadPoints = 0,
  .faultMode = kFTM_Fault_Disable,
  .faultFilterValue = 0U,
  .deadTimePrescale = kFTM_Deadtime_Prescale_1,
  .deadTimeValue = 0UL,
  .extTriggers = 0,
  .chnlInitState = 0,
  .chnlPolarity = 0,
  .useGlobalTimeBase = false
};
const ftm_phase_params_t FTM2_phaseAParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};
const ftm_phase_params_t FTM2_phaseBParams = { 
  .enablePhaseFilter = false,
  .phasePolarity = kFTM_QuadPhaseNormal

};

static void FTM2_init(void) {
  FTM_Init(FTM2_PERIPHERAL, &FTM2_config);
/* Initialization of the timer initial value and modulo value */
  FTM_SetQuadDecoderModuloValue(FTM2_PERIPHERAL, 0,0);
  FTM_SetupQuadDecode(FTM2_PERIPHERAL, &FTM2_phaseAParams, &FTM2_phaseBParams, kFTM_QuadCountAndDir);
  FTM_StartTimer(FTM2_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  FTM1_init();
  FTM2_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
