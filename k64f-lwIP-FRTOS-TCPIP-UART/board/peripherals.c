/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v11.0
processor: MK64FN1M0xxx12
package_id: MK64FN1M0VLL12
mcu_data: ksdk2_0
processor_version: 11.0.1
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: 8fa2f775-3cd2-44ce-8dab-c599a6ee0c2b
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
 * UART_0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART_0'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'true'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '115200'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
    - quick_selection: 'QuickSelection1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART_0_config = {
  .baudRate_Bps = 115200UL,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0U,
  .rxFifoWatermark = 1U,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART_0_init(void) {
  UART_Init(UART_0_PERIPHERAL, &UART_0_config, UART_0_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * I2C_1 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C_1'
- type: 'i2c'
- mode: 'I2C_Polling'
- custom_name_enabled: 'true'
- type_id: 'i2c_2566d7363e7e9aaedabb432110e372d7'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2C1'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - i2c_master_config:
      - enableMaster: 'true'
      - enableStopHold: 'false'
      - baudRate_Bps: '100000'
      - glitchFilterWidth: '0'
    - quick_selection: 'QS_I2C_1'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_master_config_t I2C_1_config = {
  .enableMaster = true,
  .enableStopHold = false,
  .baudRate_Bps = 100000UL,
  .glitchFilterWidth = 0U
};

static void I2C_1_init(void) {
  /* Initialization function */
  I2C_MasterInit(I2C_1_PERIPHERAL, &I2C_1_config, I2C_1_CLK_FREQ);
}

/***********************************************************************************************************************
 * FTM0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM0'
- type: 'ftm'
- mode: 'CenterAligned'
- custom_name_enabled: 'true'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM0'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '10000'
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
      - IRQn: 'FTM0_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - ftm_center_aligned_mode:
    - ftm_center_aligned_channels_config:
      - 0:
        - channelId: ''
        - chnlNumber: 'kFTM_Chnl_1'
        - level: 'kFTM_LowTrue'
        - dutyCyclePercent: '100'
        - enable_chan_irq: 'false'
      - 1:
        - channelId: ''
        - chnlNumber: 'kFTM_Chnl_3'
        - level: 'kFTM_LowTrue'
        - dutyCyclePercent: '100'
        - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM0_config = {
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
const ftm_chnl_pwm_signal_param_t FTM0_centerPwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_1,
    .level = kFTM_LowTrue,
    .dutyCyclePercent = 100U
  },
  {
    .chnlNumber = kFTM_Chnl_3,
    .level = kFTM_LowTrue,
    .dutyCyclePercent = 100U
  }
};

static void FTM0_init(void) {
  FTM_Init(FTM0_PERIPHERAL, &FTM0_config);
  FTM_SetupPwm(FTM0_PERIPHERAL, FTM0_centerPwmSignalParams, sizeof(FTM0_centerPwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_CenterAlignedPwm, 10000U, FTM0_CLOCK_SOURCE);
  FTM_StartTimer(FTM0_PERIPHERAL, kFTM_SystemClock);
}

/***********************************************************************************************************************
 * FTM3 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'FTM3'
- type: 'ftm'
- mode: 'CenterAligned'
- custom_name_enabled: 'true'
- type_id: 'ftm_5e037045c21cf6f361184c371dbbbab2'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'FTM3'
- config_sets:
  - ftm_main_config:
    - ftm_config:
      - clockSource: 'kFTM_SystemClock'
      - clockSourceFreq: 'GetFreq'
      - prescale: 'kFTM_Prescale_Divide_1'
      - timerFrequency: '10000'
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
      - IRQn: 'FTM3_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - EnableTimerInInit: 'true'
    - quick_selection: 'QuickSelectionDefault'
  - ftm_center_aligned_mode:
    - ftm_center_aligned_channels_config:
      - 0:
        - channelId: ''
        - chnlNumber: 'kFTM_Chnl_0'
        - level: 'kFTM_LowTrue'
        - dutyCyclePercent: '100'
        - enable_chan_irq: 'false'
      - 1:
        - channelId: ''
        - chnlNumber: 'kFTM_Chnl_2'
        - level: 'kFTM_LowTrue'
        - dutyCyclePercent: '100'
        - enable_chan_irq: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ftm_config_t FTM3_config = {
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
const ftm_chnl_pwm_signal_param_t FTM3_centerPwmSignalParams[] = { 
  {
    .chnlNumber = kFTM_Chnl_0,
    .level = kFTM_LowTrue,
    .dutyCyclePercent = 100U
  },
  {
    .chnlNumber = kFTM_Chnl_2,
    .level = kFTM_LowTrue,
    .dutyCyclePercent = 100U
  }
};

static void FTM3_init(void) {
  FTM_Init(FTM3_PERIPHERAL, &FTM3_config);
  FTM_SetupPwm(FTM3_PERIPHERAL, FTM3_centerPwmSignalParams, sizeof(FTM3_centerPwmSignalParams) / sizeof(ftm_chnl_pwm_signal_param_t), kFTM_CenterAlignedPwm, 10000U, FTM3_CLOCK_SOURCE);
  FTM_StartTimer(FTM3_PERIPHERAL, kFTM_SystemClock);
}

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
 * UART4 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART4'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'uart_cd31a12aa8c79051fda42cc851a27c37'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART4'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '9600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_TwoStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART4_config = {
  .baudRate_Bps = 9600UL,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_TwoStopBit,
  .txFifoWatermark = 0U,
  .rxFifoWatermark = 1U,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART4_init(void) {
  UART_Init(UART4_PERIPHERAL, &UART4_config, UART4_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  UART_0_init();
  I2C_1_init();
  FTM0_init();
  FTM3_init();
  UART4_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
