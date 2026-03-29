/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>

   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#include "ssd1306.h"

extern I2C_HandleTypeDef hi2c1;

/* SSD1306 data buffer. This is the buffer you must write to for setting pixel values. */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* The set of registers to write to when initializing the SSD1306 screen. */
static uint8_t SSD1306_Init_Config [] = {0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40, 0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3, 0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB, 0x20, 0x8D, 0x14, 0xAF, 0x2E};

/* An array you can use for the commands to be sent for scrolling. */
static uint8_t SSD1306_Scroll_Commands [8] = {0};

static uint16_t ssd1306_CurrentX = 0;
static uint16_t ssd1306_CurrentY = 0;

/* The following functions are provided for you to use, without requiring any modifications */

void SSD1306_UpdateScreen(void) {
	uint8_t m;
	uint8_t packet[3] = {0x00, 0x00, 0x10};

	for (m = 0; m < 8; m++) {
		packet[0] = (0xB0 + m);
		ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, packet, 3);
		ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x40, &SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH);
	}
}

void SSD1306_Fill(SSD1306_COLOR_t color) {
	/* Use memset to efficiently set the entire SSD1306_Buffer to a single value. */
	memset(SSD1306_Buffer, (color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void SSD1306_Clear (void)
{
	SSD1306_Fill (0);
    SSD1306_UpdateScreen();
}

void SSD1306_GotoXY(uint16_t x, uint16_t y) {
    ssd1306_CurrentX = x;
    ssd1306_CurrentY = y;
}

/* Start of the functions you must complete for this lab. */
void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count) {
    /* TODO */
	uint8_t buf[SSD1306_WIDTH + 1]; // 1 control byte + count data bytes
	buf[0] = reg;
	memcpy(&buf[1], data, count); // buf[1..count] = data[0..count-1]

	HAL_I2C_Master_Transmit(&hi2c1, address, buf, count + 1, 10);
}

HAL_StatusTypeDef SSD1306_Init(void) {

	/* Check if the OLED is connected to I2C */
	if (HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 20000) != HAL_OK) {
		return HAL_ERROR;
	}

    /* Keep this delay to prevent overflowing the I2C controller */
	HAL_Delay(10);

	/* Init LCD */
	/* TODO */
	for (uint16_t i = 0; i < sizeof(SSD1306_Init_Config); i++) {
		uint8_t cmd = SSD1306_Init_Config[i];
		ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, &cmd, 1);
	}

	ssd1306_CurrentX = 0;
	ssd1306_CurrentY = 0;

	/* Clear screen */
	SSD1306_Fill(SSD1306_COLOR_BLACK);

	/* Update screen */
	SSD1306_UpdateScreen();

	/* Return OK */
	return HAL_OK;
}



HAL_StatusTypeDef SSD1306_SetPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color) {
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
		return HAL_ERROR;
	}

    /* Set the pixel at position (x,y) to 'color'. */
	/* TODO */

	// 128 x 64 screen
	// 1 page = 8 vertical pixels -> 64 / 8 = 8 pages, 128 columns
	// 128 columns * 8 pages = 1024 entries
	// eg. buf[0..127] -> page 0 (index = x, value = bit of 8 y)
	uint32_t index = x + (y / 8) * SSD1306_WIDTH;
	uint8_t mask = 1 << (y % 8);

	if (color == SSD1306_COLOR_WHITE) {
		SSD1306_Buffer[index] |= mask;
	} else {
		SSD1306_Buffer[index] &= ~mask;
	}

	return HAL_OK;
}



void SSD1306_Scroll(SSD1306_SCROLL_DIR_t direction, uint8_t start_row, uint8_t end_row)
{
    /* TO DO */
	// choose direction
	uint8_t cmds[8];

	if (direction == SSD1306_SCROLL_RIGHT) {
		cmds[0] = SSD1306_RIGHT_HORIZONTAL_SCROLL;
	} else {
		cmds[0] = SSD1306_LEFT_HORIZONTAL_SCROLL;
	}

	cmds[1] = 0x00;          		   // dummy
	cmds[2] = start_row;     		   // first page to scroll
	cmds[3] = 0x00;          		   // scroll frequency
	cmds[4] = end_row;       		   // end page to scroll
	cmds[5] = 0x00;          		   // dummy
	cmds[6] = 0xFF;                    // dummy
	cmds[7] = SSD1306_ACTIVATE_SCROLL; // activate scrolling

	ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, cmds, 8); // write control byte
}

void SSD1306_Stopscroll(void)
{
	/* TO DO */
	uint8_t cmd = SSD1306_DEACTIVATE_SCROLL;
	ssd1306_I2C_Write(SSD1306_I2C_ADDR, 0x00, &cmd, 1);
}


void SSD1306_Putc(uint16_t x, uint16_t y, char ch, FontDef_t* Font) {
    /* TODO */
	uint32_t char_index = (ch - 32) * Font->FontHeight; // check which char

	for (uint16_t row = 0; row < Font->FontHeight; row++) {
		uint16_t bits = Font->data[char_index + row];

		for (uint16_t col = 0; col < Font->FontWidth; col++) {
			if (bits & (0x8000u >> col)) { // from left to right
				SSD1306_SetPixel(x + col, y + row, SSD1306_COLOR_WHITE);
			}
		}
	}
}

HAL_StatusTypeDef SSD1306_Puts(char* str, FontDef_t* Font) {

	/* Loop over every character until we see \0. */
	while (*str != '\0') {

		/* TODO */
		if (*str == '\n') {
			ssd1306_CurrentX = 0;
			ssd1306_CurrentY += Font->FontHeight;
			str++;
			continue;
		}

		if (ssd1306_CurrentX + Font->FontWidth > SSD1306_WIDTH) { // change line if row no space
			ssd1306_CurrentX = 0;
			ssd1306_CurrentY += Font->FontHeight;
		}

		if (ssd1306_CurrentY + Font->FontHeight > SSD1306_HEIGHT) { // error if column no space
			return HAL_ERROR;
		}

		SSD1306_Putc(ssd1306_CurrentX, ssd1306_CurrentY, *str, Font);
		ssd1306_CurrentX += Font->FontWidth;

        /* Increase string pointer */
        str++;
	}

	return HAL_OK;
}
