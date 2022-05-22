/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_uart.h"
#include "fsl_clock.h"
#include "fsl_ftm.h"
#include "fsl_i2c.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Definition of peripheral ID */
#define UART_0_PERIPHERAL UART0
/* Definition of the clock source frequency */
#define UART_0_CLOCK_SOURCE CLOCK_GetFreq(UART0_CLK_SRC)
/* Definition of peripheral ID */
#define FTM0_PERIPHERAL FTM0
/* Definition of the clock source frequency */
#define FTM0_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* FTM0 interrupt vector ID (number). */
#define FTM0_IRQN FTM0_IRQn
/* FTM0 interrupt handler identifier. */
#define FTM0_IRQHANDLER FTM0_IRQHandler
/* Definition of peripheral ID */
#define UART4_PERIPHERAL UART4
/* Definition of the clock source frequency */
#define UART4_CLOCK_SOURCE CLOCK_GetFreq(UART4_CLK_SRC)
/* BOARD_InitPeripherals defines for I2C0 */
/* Definition of peripheral ID */
#define I2C0_PERIPHERAL I2C0
/* Definition of the clock source */
#define I2C0_CLOCK_SOURCE I2C0_CLK_SRC
/* Definition of the clock source frequency */
#define I2C0_CLK_FREQ CLOCK_GetFreq(I2C0_CLOCK_SOURCE)

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const uart_config_t UART_0_config;
extern const ftm_config_t FTM0_config;
extern const uart_config_t UART4_config;
extern const i2c_master_config_t I2C0_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/

void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
