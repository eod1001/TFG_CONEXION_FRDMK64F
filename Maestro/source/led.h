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
 * @file led.h
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

#ifndef LED_H
#define LED_H

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void turn_off_leds();
void turn_on_red();
void turn_on_green();
void turn_on_blue();
void turn_on_yellow();
void turn_on_magenta();
void turn_on_cyan();
void turn_on_white();

#endif /* LED_H */

/*** end of file ***/
