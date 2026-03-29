/*
 * fonts.h
 *
 *  Created on: Feb 3, 2026
 *      Author: filip
 */

#ifndef FONTS_H_
#define FONTS_H_

#include "stm32f4xx_hal.h"
#include "string.h"

typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef_t;

extern FontDef_t Font_7x10;
extern FontDef_t Font_11x18;

#endif /* FONTS_H_ */
