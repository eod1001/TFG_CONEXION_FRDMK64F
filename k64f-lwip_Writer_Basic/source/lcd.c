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
 * @file lcd.c
 * @author RPC0027
 * @date December 2018
 * @version 1.0
 *
 * @brief Functions for using LCD displays through I2C bus.
 *
 * Provides several functions to carry out operations with the display.
 *
 * Created by: Frank de Brabander
 * @see https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
 * Modified by: rpc0027
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "i2c.h"
#include "wait.h"
#include "lcd.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_lcd_addr;            /**< Display address. */
uint8_t g_lcd_cols;            /**< Number of available characters columns. */
uint8_t g_lcd_rows;            /**< Number of available text rows. */
uint8_t g_lcd_charsize;        /**< Size of each character. */
uint8_t g_lcd_backlightval;    /**< Backlight status. */
uint8_t g_lcd_displayfunction; /**< Display function mode. */
uint8_t g_lcd_displaycontrol;  /**< Display control mode. */
uint8_t g_lcd_displaymode;     /**< Display text direction mode. */

/*******************************************************************************
 * Code
 ******************************************************************************/
/**
 * @brief Initialize the LCD.
 *
 * Displays can differ among models so some configuration is needed.
 *
 * @param[in] lcd_addr Hexadecimal address of the I2C slave.
 * @param[in] lcd_cols Number of columns in the display.
 * @param[in] lcd_rows Number of rows in the display.
 * @param[in] charsize Font size of the characters.
 */
void LCD_Init(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows, uint8_t charsize)
{
	g_lcd_addr         = lcd_addr;
	g_lcd_cols         = lcd_cols;
	g_lcd_rows         = lcd_rows;
	g_lcd_charsize     = charsize;
	g_lcd_backlightval = LCD_BACKLIGHT;

	select_slave(g_lcd_addr);

	/* Set bit mode, number of rows, and font size. */
	g_lcd_displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

	/* For some 1 line displays you can select a 10 pixel high font. */
	if ((g_lcd_charsize != 0) && (g_lcd_rows == 1))
	{
		g_lcd_displayfunction |= LCD_5x10DOTS;
	}

	/* Set 4 bit mode. */
	LCD_write4bits(0x03 << 4);
	/* New attempt, just in case.*/
	LCD_write4bits(0x03 << 4);
	/* Set 4 bit interface. */
	LCD_write4bits(0x02 << 4);

	/* Issue the setting command. */
	LCD_command(LCD_FUNCTIONSET | g_lcd_displayfunction);

	/* Set the display on, no cursor, nor blinking by default. */
	g_lcd_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	LCD_command(LCD_ENTRYMODESET | g_lcd_displayfunction);

	/* Set the text direction. */
	g_lcd_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	/* Go home. */
	LCD_home();
}

/**
 * @brief Remove the characters from the display.
 */
void LCD_clear()
{
	LCD_command(LCD_CLEARDISPLAY);
	wait_Waitms(2);
}

/**
 * @brief Set cursor position to zero.
 */
void LCD_home()
{
	LCD_command(LCD_RETURNHOME);
}

/**
 * @brief Set cursor to given position.
 *
 * @param[in] col the column of the display.
 * @param[in] row the row of the display.
 */
void LCD_setCursor(uint8_t col, uint8_t row)
{
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };

	if (row > g_lcd_rows)
	{
		row = g_lcd_rows-1;
	}

	LCD_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/**
 * @brief Turn the display off.
 */
void LCD_noDisplay()
{
	g_lcd_displaycontrol &= ~LCD_DISPLAYON;
	LCD_command(LCD_DISPLAYCONTROL | g_lcd_displaycontrol);
}

/**
 * @brief Turn the display on.
 */
void LCD_display()
{
	g_lcd_displaycontrol |= LCD_DISPLAYON;
	LCD_command(LCD_DISPLAYCONTROL | g_lcd_displaycontrol);
}

/**
 * @brief Turn the underline cursor off.
 */
void LCD_noCursor()
{
	g_lcd_displaycontrol &= ~LCD_CURSORON;
	LCD_command(LCD_DISPLAYCONTROL | g_lcd_displaycontrol);
}

/**
 * @brief Turn the underline cursor on.
 */
void LCD_cursor()
{
	g_lcd_displaycontrol |= LCD_CURSORON;
	LCD_command(LCD_DISPLAYCONTROL | g_lcd_displaycontrol);
}

/**
 * @brief Turn off the blinking cursor.
 */
void LCD_noBlink()
{
	g_lcd_displaycontrol &= ~LCD_BLINKON;
	LCD_command(LCD_DISPLAYCONTROL | g_lcd_displaycontrol);
}

/**
 * @brief Turn on the blinking cursor.
 */
void LCD_blink()
{
	g_lcd_displaycontrol |= LCD_BLINKON;
	LCD_command(LCD_DISPLAYCONTROL | g_lcd_displaycontrol);
}

/**
 * @brief Scroll the display to the left without changing the RAM.
 */
void LCD_scrollDisplayLeft(void)
{
	LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

/**
 * @brief Scroll the display to the right without changing the RAM.
 */
void LCD_scrollDisplayRight(void)
{
	LCD_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/**
 * @brief Show text that flows left to right.
 */
void LCD_leftToRight(void)
{
	g_lcd_displaymode |= LCD_ENTRYLEFT;
	LCD_command(LCD_ENTRYMODESET | g_lcd_displaymode);
}

/**
 * @brief Show text that flows right to left.
 */
void LCD_rightToLeft(void)
{
	g_lcd_displaymode &= ~LCD_ENTRYLEFT;
	LCD_command(LCD_ENTRYMODESET | g_lcd_displaymode);
}

/**
 * @brief Right justify text from the cursor.
 */
void LCD_autoscroll(void)
{
	g_lcd_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	LCD_command(LCD_ENTRYMODESET | g_lcd_displaymode);
}

/**
 * @brief Left justify text from the cursor.
 */
void LCD_noAutoscroll(void)
{
	g_lcd_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	LCD_command(LCD_ENTRYMODESET | g_lcd_displaymode);
}

/**
 * @brief Fill the first 8 CGRAM locations with custom characters.
 */
void LCD_createChar(uint8_t location, uint8_t charmap[])
{
	location &= 0x7;

	LCD_command(LCD_SETCGRAMADDR | (location << 3));

	for (int i = 0; i < 8; i++)
	{
		LCD_write(charmap[i]);
	}
}

/**
 * @brief Turn the (optional) backlight off.
 */
void LCD_noBacklight(void)
{
	g_lcd_backlightval = LCD_NOBACKLIGHT;
	LCD_expanderWrite(0);
}

/**
 * @brief Turn the (optional) backlight on.
 */
void LCD_backlight(void)
{
	g_lcd_backlightval = LCD_BACKLIGHT;
	LCD_expanderWrite(0);
}

/**
 * @brief Clear a single row.
 *
 * @param[in] row the row to clean.
 */
void LCD_clear_row(uint8_t row)
{
	LCD_setCursor(0, row);
	LCD_printstr("                ");
	LCD_setCursor(0, row);
}

/**
 * @brief Send a byte to the display.
 *
 * @param[in] _data the byte to write.
 */
void LCD_expanderWrite(uint8_t _data)
{
	send_byte((_data) | g_lcd_backlightval);
}

/**
 * @brief Send 4 bits to the display.
 *
 * @param[in] value the bits to write.
 */
void LCD_write4bits(uint8_t value)
{
	LCD_expanderWrite(value);
	LCD_pulseEnable(value);
}

/**
 * @brief Write either command or data.
 *
 * @param[in] value the byte to write.
 * @param[in] mode the byte to write.
 */
void LCD_send(uint8_t value, uint8_t mode)
{
	uint8_t highnib =  value & 0xf0;
	uint8_t lownib  = (value << 4) & 0xf0;

	LCD_write4bits((highnib) | mode);
	LCD_write4bits((lownib) | mode);
}

/**
 * @brief Send a pulse to the display.
 *
 * @param[in] _data the byte with the pulse.
 */
void LCD_pulseEnable(uint8_t _data)
{
	LCD_expanderWrite(_data | LCD_En);
	LCD_expanderWrite(_data & ~LCD_En);
}

/**
 * @brief Send a command to the display.
 *
 * @param[in] value the byte with the command.
 */
void LCD_command(uint8_t value)
{
	LCD_send(value, 0);
}

/**
 * @brief Send data to the display.
 *
 * @param[in] value the byte with the data.
 * @return always return 1.
 */
uint8_t LCD_write(uint8_t value)
{
	LCD_send(value, LCD_Rs);
	return 1;
}

/**
 * @brief Load a new character.
 *
 * @param[in] char_num the position of the character.
 * @param[in] rows the custom character.
 */
void LCD_load_custom_character(uint8_t char_num, uint8_t * rows){
	LCD_createChar(char_num, rows);
}

/**
 * @brief Change the backlight status.
 *
 * @param[in] new_val if not zero, turn on the backlight.
 */
void LCD_setBacklight(uint8_t new_val)
{
	if (new_val)
	{
		LCD_backlight();
	}
	else
	{
		LCD_noBacklight();
	}
}

/**
 * @brief Print a string in the current row.
 *
 * @param[in] c pointer to the characters to print.
 */
void LCD_printstr(const char * c)
{
	while (*c)
	{
		LCD_write(*c);
		c++;
	}
}

/*** end of file ***/
