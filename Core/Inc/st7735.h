#ifndef __ST7735_H
#define __ST7735_H

#include "main.h"
#include "fonts.h"
#include <stdint.h>
#include <stdbool.h>

/* Display Dimensions (Landscape: 160 x 128) */
#define ST7735_WIDTH             160
#define ST7735_HEIGHT            128

/* Pin Mapping (STM32F411RE Nucleo-64) */
#define TFT_PORT_CS              GPIOB
#define TFT_PIN_CS               GPIO_PIN_12

#define TFT_PORT_RESET           GPIOB
#define TFT_PIN_RESET            GPIO_PIN_1

#define TFT_PORT_DC              GPIOB
#define TFT_PIN_DC               GPIO_PIN_14

#define TFT_PORT_SDA             GPIOB
#define TFT_PIN_SDA              GPIO_PIN_15

#define TFT_PORT_SCK             GPIOB
#define TFT_PIN_SCK              GPIO_PIN_13

#define TFT_PORT_LED             GPIOB
#define TFT_PIN_LED              GPIO_PIN_2

/* Professional Pure Black Dark Theme Color Palette (RGB565) */
#define COLOR_BG                 0x0000  /* Pure Pitch Black */
#define COLOR_CARD_BG            0x0841  /* Dark Titanium Card */
#define COLOR_CARD_BORDER        0x2945  /* Crisp Slate Border */
#define COLOR_HEADER_BG          0x0861  /* Sleek Header Bar */
#define COLOR_TEXT_WHITE         0xFFFF  /* Ultra-Crisp White */
#define COLOR_TEXT_MUTED         0x94B2  /* High-Contrast Silver-Gray */
#define COLOR_CYAN               0x07FF  /* Electric Cyan (Bat1) */
#define COLOR_LIME               0x07E0  /* Neon Lime Green (Bat2) */
#define COLOR_EMERALD            0x2FE0  /* Bright Emerald (Total Pack / OK) */
#define COLOR_YELLOW             0xFFE0  /* Solar Gold (Current / Power) */
#define COLOR_AMBER              0xFDE0  /* Bright Amber Warning */
#define COLOR_ORANGE             0xFD20  /* Vivid Thermal Orange (Temp) */
#define COLOR_SKY                0x55FF  /* Glacier Sky Blue (Humidity) */
#define COLOR_RED                0xF800  /* Alert Red */
#define COLOR_BLACK              0x0000

/* ST7735 Commands */
#define ST7735_NOP               0x00
#define ST7735_SWRESET           0x01
#define ST7735_RDDID             0x04
#define ST7735_RDDST             0x09
#define ST7735_SLPIN             0x10
#define ST7735_SLPOUT            0x11
#define ST7735_PTLON             0x12
#define ST7735_NORON             0x13
#define ST7735_INVOFF            0x20
#define ST7735_INVON             0x21
#define ST7735_DISPOFF           0x28
#define ST7735_DISPON            0x29
#define ST7735_CASET             0x2A
#define ST7735_RASET             0x2B
#define ST7735_RAMWR             0x2C
#define ST7735_RAMRD             0x2E
#define ST7735_MADCTL            0x36
#define ST7735_COLMOD            0x3A

/* Function Prototypes */
void ST7735_Init(void);
void ST7735_InvertColors(bool invert);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_DrawLineH(uint16_t x, uint16_t y, uint16_t w, uint16_t color);
void ST7735_DrawLineV(uint16_t x, uint16_t y, uint16_t h, uint16_t color);
void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, FontDef_t font, uint16_t color, uint16_t bgcolor);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, FontDef_t font, uint16_t color, uint16_t bgcolor);
void ST7735_DrawCard(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *title, uint16_t title_color);

#endif /* __ST7735_H */
