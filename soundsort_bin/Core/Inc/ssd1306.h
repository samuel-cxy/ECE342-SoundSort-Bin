/*
 * ssd1306.h
 *
 *  Created on: Feb 3, 2026
 *      Author: filip
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include "stm32f4xx_hal.h"
#include "fonts.h"
#include "stdlib.h"
#include "string.h"

/* Basic parameters for the SSD1306 screen. */
#define SSD1306_I2C_ADDR         0x78  /* I2C address */
#define SSD1306_WIDTH            128   /* SSD1306 width in pixels */
#define SSD1306_HEIGHT           64    /* SSD1306 LCD height in pixels */
#define ssd1306_I2C_TIMEOUT		 20000

typedef enum {
	SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR_t;

/* Functions given to you that you do not need to change. */
void SSD1306_UpdateScreen(void);
void SSD1306_Fill(SSD1306_COLOR_t Color);
void SSD1306_Clear (void);
void SSD1306_GotoXY(uint16_t x, uint16_t y);

/* PART I: I2C operations */
void ssd1306_I2C_Init();
void ssd1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

/* PART II: Basic screen operations */
HAL_StatusTypeDef SSD1306_Init(void);
// HAL_StatusTypeDef SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);
HAL_StatusTypeDef SSD1306_SetPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color);

/* PART III: Scrolling content on the screen */
#define SSD1306_RIGHT_HORIZONTAL_SCROLL              0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL               0x27
#define SSD1306_DEACTIVATE_SCROLL                    0x2E
#define SSD1306_ACTIVATE_SCROLL                      0x2F

typedef enum {
	SSD1306_SCROLL_RIGHT = 0x00, /* Scroll display content right */
	SSD1306_SCROLL_LEFT  = 0x01  /* Scroll display content left */
} SSD1306_SCROLL_DIR_t;

void SSD1306_Scroll(SSD1306_SCROLL_DIR_t direction, uint8_t start_row, uint8_t end_row);
void SSD1306_Stopscroll(void);

/* PART IV: Operations for writing text */
void SSD1306_Putc(uint16_t x, uint16_t y, char ch, FontDef_t* Font);
HAL_StatusTypeDef SSD1306_Puts(char* str, FontDef_t* Font);

#endif /* INC_SSD1306_H_ */
