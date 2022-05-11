/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features for the Ethernet.
 *
 */
void BOARD_InitENET(void);

/*! @name PORTE26 (number 33), LED_GREEN
  @{ */

/* Symbols to be used with GPIO driver */
#define LED_GREEN_GPIO GPIOE                /*!<@brief GPIO peripheral base pointer */
#define LED_GREEN_GPIO_PIN_MASK (1U << 26U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define LED_GREEN_PORT PORTE                /*!<@brief PORT peripheral base pointer */
#define LED_GREEN_PIN 26U                   /*!<@brief PORT pin number */
#define LED_GREEN_PIN_MASK (1U << 26U)      /*!<@brief PORT pin mask */
                                            /* @} */

/*! @name PORTB21 (number 67), LED_BLUE
  @{ */

/* Symbols to be used with GPIO driver */
#define LED_BLUE_GPIO GPIOB                /*!<@brief GPIO peripheral base pointer */
#define LED_BLUE_GPIO_PIN_MASK (1U << 21U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define LED_BLUE_PORT PORTB                /*!<@brief PORT peripheral base pointer */
#define LED_BLUE_PIN 21U                   /*!<@brief PORT pin number */
#define LED_BLUE_PIN_MASK (1U << 21U)      /*!<@brief PORT pin mask */
                                           /* @} */

/*! @name PORTB22 (number 68), LED_RED
  @{ */

/* Symbols to be used with GPIO driver */
#define LED_RED_GPIO GPIOB                /*!<@brief GPIO peripheral base pointer */
#define LED_RED_GPIO_PIN_MASK (1U << 22U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define LED_RED_PORT PORTB                /*!<@brief PORT peripheral base pointer */
#define LED_RED_PIN 22U                   /*!<@brief PORT pin number */
#define LED_RED_PIN_MASK (1U << 22U)      /*!<@brief PORT pin mask */
                                          /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features for the LEDs.
 *
 */
void BOARD_InitLEDs(void);

#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*!
 * @brief Configures pin routing and optionally pin electrical features for serial comm.
 *
 */
void BOARD_InitSerial(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features for PWM.
 *
 */
void BOARD_InitPWM(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
