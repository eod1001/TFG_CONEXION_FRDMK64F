/**
 * Copyright 2019 rpc0027
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file led.c
 * @author RPC0027
 * @date December 2018
 * @version 1.0
 *
 * @brief Turn on and off on boards LEDs.
 *
 * The colors of the on board LED are red, green and blue.
 * Mixing these basic colors more can be obtained
 * like yellow, magenta, cyan and white.
 *
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "fsl_gpio.h"
#include "pin_mux.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
/**
 * @brief Turn off all LEDs.
 */
void turn_off_leds()
{
	GPIO_PinWrite(LED_RED_GPIO, LED_RED_PIN, 1U);
	GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 1U);
	GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PIN, 1U);
}

/**
 * @brief Turn on red color.
 */
void turn_on_red()
{
	turn_off_leds();
	GPIO_PinWrite(LED_RED_GPIO, LED_RED_PIN, 0U);
}

/**
 * @brief Turn on green color.
 */
void turn_on_green()
{
	turn_off_leds();
	GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 0U);
}

/**
 * @brief Turn on blue color.
 */
void turn_on_blue()
{
	turn_off_leds();
	GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PIN, 0U);
}

/**
 * @brief Turn on yellow color.
 */
void turn_on_yellow()
{
	turn_off_leds();
	GPIO_PinWrite(LED_RED_GPIO, LED_RED_PIN, 0U);
	GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 0U);
}

/**
 * @brief Turn on magenta color.
 */
void turn_on_magenta()
{
	turn_off_leds();
	GPIO_PinWrite(LED_RED_GPIO, LED_RED_PIN, 0U);
	GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PIN, 0U);
}

/**
 * @brief Turn on cyan color.
 */
void turn_on_cyan()
{
	turn_off_leds();
	GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 0U);
	GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PIN, 0U);
}

/**
 * @brief Turn on white color.
 */
void turn_on_white()
{
	GPIO_PinWrite(LED_RED_GPIO, LED_RED_PIN, 0U);
	GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 0U);
	GPIO_PinWrite(LED_BLUE_GPIO, LED_BLUE_PIN, 0U);
}

/*** end of file ***/
