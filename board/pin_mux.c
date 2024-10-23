/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v11.0
processor: S32K146
package_id: S32K146_LQFP144
mcu_data: s32sdk_s32k1xx_rtm_401
processor_version: 0.0.0
pin_labels:
- {pin_num: '28', pin_signal: PTD16, label: led0_evb, identifier: led0_evb;led1_evb}
- {pin_num: '29', pin_signal: PTD15, label: led0_evb, identifier: led0_evb}
- {pin_num: '53', pin_signal: PTC0, label: led0_mb, identifier: led0_mb}
- {pin_num: '52', pin_signal: PTC1, label: led0_mb, identifier: led0_mb;led1_mb}
- {pin_num: '71', pin_signal: PTC12, label: btn2, identifier: btn2}
- {pin_num: '70', pin_signal: PTC13, label: btn1, identifier: btn1}
- {pin_num: '8', pin_signal: PTE5, label: CAN_TX, identifier: CAN_TX}
- {pin_num: '9', pin_signal: PTE4, label: CAN_RX, identifier: CAN_RX}
- {pin_num: '109', pin_signal: PTB9, label: INH1}
- {pin_num: '68', pin_signal: PTB2, label: INH2}
- {pin_num: '40', pin_signal: PTB5, label: IN1}
- {pin_num: '34', pin_signal: PTD14, label: IN2}
- {pin_num: '98', pin_signal: PTB12, label: IS1}
- {pin_num: '100', pin_signal: PTD4, label: IS2}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variables will be used in other source files in application code.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 11.4, Conversion between a pointer and integer type.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 * @section [global]
 * Violates MISRA 2012 Required Rule 11.6, Cast from unsigned int to pointer.
 * The cast is required to initialize a pointer with an unsigned long define, representing an address.
 *
 */

#include "pin_mux.h"

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0}
- pin_list:
  - {pin_num: '8', peripheral: CAN0, signal: 'txd, txd', pin_signal: PTE5}
  - {pin_num: '9', peripheral: CAN0, signal: 'rxd, rxd', pin_signal: PTE4}
  - {pin_num: '29', peripheral: PORTD, signal: 'port, 15', pin_signal: PTD15, direction: OUTPUT}
  - {pin_num: '28', peripheral: PORTD, signal: 'port, 16', pin_signal: PTD16, identifier: led1_evb, direction: OUTPUT}
  - {pin_num: '53', peripheral: PORTC, signal: 'port, 0', pin_signal: PTC0, direction: OUTPUT}
  - {pin_num: '52', peripheral: PORTC, signal: 'port, 1', pin_signal: PTC1, identifier: led1_mb, direction: OUTPUT}
  - {pin_num: '71', peripheral: PORTC, signal: 'port, 12', pin_signal: PTC12, direction: INPUT}
  - {pin_num: '70', peripheral: PORTC, signal: 'port, 13', pin_signal: PTC13, direction: INPUT}
  - {pin_num: '100', peripheral: ADC1, signal: 'se, 6', pin_signal: PTD4}
  - {pin_num: '98', peripheral: ADC1, signal: 'se, 7', pin_signal: PTB12}
  - {pin_num: '34', peripheral: PORTD, signal: 'port, 14', pin_signal: PTD14, direction: OUTPUT}
  - {pin_num: '40', peripheral: PORTB, signal: 'port, 5', pin_signal: PTB5, direction: OUTPUT}
  - {pin_num: '68', peripheral: PORTB, signal: 'port, 2', pin_signal: PTB2, direction: OUTPUT}
  - {pin_num: '109', peripheral: PORTB, signal: 'port, 9', pin_signal: PTB9, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* Generate array of configured pin structures */
pin_settings_config_t g_pin_mux_InitConfigArr0[NUM_OF_CONFIGURED_PINS0] = {
    {
        .base            = PORTB,
        .pinPortIdx      = 12U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_PIN_DISABLED,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = NULL,
        .digitalFilter   = false,
    },
    {
        .base            = PORTB,
        .pinPortIdx      = 2U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTB,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTB,
        .pinPortIdx      = 5U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTB,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTB,
        .pinPortIdx      = 9U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTB,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTC,
        .pinPortIdx      = 0U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTC,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTC,
        .pinPortIdx      = 1U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTC,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTC,
        .pinPortIdx      = 12U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTC,
        .direction       = GPIO_INPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTC,
        .pinPortIdx      = 13U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTC,
        .direction       = GPIO_INPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTD,
        .pinPortIdx      = 14U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTD,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTD,
        .pinPortIdx      = 15U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTD,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTD,
        .pinPortIdx      = 16U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_AS_GPIO,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = PTD,
        .direction       = GPIO_OUTPUT_DIRECTION,
        .digitalFilter   = false,
        .initValue       = 0U,
    },
    {
        .base            = PORTD,
        .pinPortIdx      = 4U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_PIN_DISABLED,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = NULL,
        .digitalFilter   = false,
    },
    {
        .base            = PORTE,
        .pinPortIdx      = 4U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_ALT5,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = NULL,
        .digitalFilter   = false,
    },
    {
        .base            = PORTE,
        .pinPortIdx      = 5U,
        .pullConfig      = PORT_INTERNAL_PULL_NOT_ENABLED,
        .driveSelect     = PORT_LOW_DRIVE_STRENGTH,
        .passiveFilter   = false,
        .mux             = PORT_MUX_ALT5,
        .pinLock         = false,
        .intConfig       = PORT_DMA_INT_DISABLED,
        .clearIntFlag    = false,
        .gpioBase        = NULL,
        .digitalFilter   = false,
    },
};
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
