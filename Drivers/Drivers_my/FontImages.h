// FontImages.h
#ifndef _IMAGE_H
#define _IMAGE_H
#include "stm32f1xx_hal.h"
#include <stdint.h>

#define FONT_HEADER_SIZE 4
#define FONT_CHAR_WIDTH 6
#define FONT_CHAR_HEIGHT 8

extern const uint8_t image_data1[1024]; // 128x64 图片数据
extern const uint8_t image_data2[1024]; // 128x64 图片数据
extern const uint8_t image_data3[1024];
extern const uint8_t ERRORData[] ;
extern const uint8_t ssd1306xled_font6x8[];

#endif
