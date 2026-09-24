#include "st7735.h"

/* Fast Bit-Bang / SPI GPIO Macros */
#define CS_LOW()     HAL_GPIO_WritePin(TFT_PORT_CS, TFT_PIN_CS, GPIO_PIN_RESET)
#define CS_HIGH()    HAL_GPIO_WritePin(TFT_PORT_CS, TFT_PIN_CS, GPIO_PIN_SET)
#define DC_COMMAND() HAL_GPIO_WritePin(TFT_PORT_DC, TFT_PIN_DC, GPIO_PIN_RESET)
#define DC_DATA()    HAL_GPIO_WritePin(TFT_PORT_DC, TFT_PIN_DC, GPIO_PIN_SET)
#define RST_LOW()    HAL_GPIO_WritePin(TFT_PORT_RESET, TFT_PIN_RESET, GPIO_PIN_RESET)
#define RST_HIGH()   HAL_GPIO_WritePin(TFT_PORT_RESET, TFT_PIN_RESET, GPIO_PIN_SET)

static void ST7735_SendByte(uint8_t byte)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (byte & 0x80)
      HAL_GPIO_WritePin(TFT_PORT_SDA, TFT_PIN_SDA, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(TFT_PORT_SDA, TFT_PIN_SDA, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(TFT_PORT_SCK, TFT_PIN_SCK, GPIO_PIN_SET);
    byte <<= 1;
    HAL_GPIO_WritePin(TFT_PORT_SCK, TFT_PIN_SCK, GPIO_PIN_RESET);
  }
}

static void ST7735_WriteCommand(uint8_t cmd)
{
  DC_COMMAND();
  CS_LOW();
  ST7735_SendByte(cmd);
  CS_HIGH();
}

static void ST7735_WriteData(uint8_t data)
{
  DC_DATA();
  CS_LOW();
  ST7735_SendByte(data);
  CS_HIGH();
}

static void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  /* Offset for standard 1.8" 160x128 Red/Green tab ST7735 in Landscape */
  x0 += 0;
  x1 += 0;
  y0 += 0;
  y1 += 0;

  ST7735_WriteCommand(ST7735_CASET);
  ST7735_WriteData(x0 >> 8);
  ST7735_WriteData(x0 & 0xFF);
  ST7735_WriteData(x1 >> 8);
  ST7735_WriteData(x1 & 0xFF);

  ST7735_WriteCommand(ST7735_RASET);
  ST7735_WriteData(y0 >> 8);
  ST7735_WriteData(y0 & 0xFF);
  ST7735_WriteData(y1 >> 8);
  ST7735_WriteData(y1 & 0xFF);

  ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_Init(void)
{
  /* 1. Initialize GPIO Pins */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = TFT_PIN_CS | TFT_PIN_RESET | TFT_PIN_DC | TFT_PIN_SDA | TFT_PIN_SCK | TFT_PIN_LED;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Turn on Backlight */
  HAL_GPIO_WritePin(TFT_PORT_LED, TFT_PIN_LED, GPIO_PIN_SET);

  /* 2. Hardware Reset Pulse */
  RST_HIGH();
  HAL_Delay(5);
  RST_LOW();
  HAL_Delay(20);
  RST_HIGH();
  HAL_Delay(150);

  /* 3. Software Reset & Sleep Out */
  ST7735_WriteCommand(ST7735_SWRESET);
  HAL_Delay(120);

  ST7735_WriteCommand(ST7735_SLPOUT);
  HAL_Delay(120);

  /* 4. Color Mode: 16-bit / pixel (RGB565) */
  ST7735_WriteCommand(ST7735_COLMOD);
  ST7735_WriteData(0x05);

  /* 5. Memory Access Control (Orientation: Landscape 160x128 Flipped 180 deg) */
  ST7735_WriteCommand(ST7735_MADCTL);
  ST7735_WriteData(0x68); /* 0x68: Landscape 180-deg flipped with BGR color filter */

  /* 6. Display Inversion Off (ensures pitch black background #000000) */
  ST7735_WriteCommand(ST7735_INVOFF);

  /* 7. Normal Display Mode On & Display ON */
  ST7735_WriteCommand(ST7735_NORON);
  HAL_Delay(10);

  ST7735_WriteCommand(ST7735_DISPON);
  HAL_Delay(100);

  /* Fill with pure pitch black background */
  ST7735_FillScreen(COLOR_BG);
}

void ST7735_InvertColors(bool invert)
{
  ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
}

void ST7735_FillScreen(uint16_t color)
{
  ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;

  ST7735_SetAddressWindow(x, y, x, y);
  DC_DATA();
  CS_LOW();
  ST7735_SendByte(color >> 8);
  ST7735_SendByte(color & 0xFF);
  CS_HIGH();
}

void ST7735_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || (w == 0) || (h == 0)) return;
  if ((x + w - 1) >= ST7735_WIDTH)  w = ST7735_WIDTH - x;
  if ((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

  ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xFF;

  DC_DATA();
  CS_LOW();
  for (uint32_t i = 0; i < (uint32_t)w * h; i++)
  {
    ST7735_SendByte(hi);
    ST7735_SendByte(lo);
  }
  CS_HIGH();
}

void ST7735_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  ST7735_DrawLineH(x, y, w, color);
  ST7735_DrawLineH(x, y + h - 1, w, color);
  ST7735_DrawLineV(x, y, h, color);
  ST7735_DrawLineV(x + w - 1, y, h, color);
}

void ST7735_DrawLineH(uint16_t x, uint16_t y, uint16_t w, uint16_t color)
{
  ST7735_FillRect(x, y, w, 1, color);
}

void ST7735_DrawLineV(uint16_t x, uint16_t y, uint16_t h, uint16_t color)
{
  ST7735_FillRect(x, y, 1, h, color);
}

/**
  * @brief  High-speed block streaming text renderer (100x faster than single-pixel draw)
  */
void ST7735_DrawChar(uint16_t x, uint16_t y, char ch, FontDef_t font, uint16_t color, uint16_t bgcolor)
{
  if (ch < 32 || ch > 126) ch = ' ';
  if ((x + font.width > ST7735_WIDTH) || (y + font.height > ST7735_HEIGHT)) return;

  uint32_t char_idx = (ch - 32) * font.width;
  ST7735_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

  uint8_t hi_fg = color >> 8;
  uint8_t lo_fg = color & 0xFF;
  uint8_t hi_bg = bgcolor >> 8;
  uint8_t lo_bg = bgcolor & 0xFF;

  DC_DATA();
  CS_LOW();
  for (uint8_t j = 0; j < font.height; j++)
  {
    for (uint8_t i = 0; i < font.width; i++)
    {
      uint16_t line = font.data[char_idx + i];
      if (line & (1 << j))
      {
        ST7735_SendByte(hi_fg);
        ST7735_SendByte(lo_fg);
      }
      else
      {
        ST7735_SendByte(hi_bg);
        ST7735_SendByte(lo_bg);
      }
    }
  }
  CS_HIGH();
}

void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, FontDef_t font, uint16_t color, uint16_t bgcolor)
{
  while (*str)
  {
    if (x + font.width > ST7735_WIDTH)
    {
      x = 0;
      y += font.height;
      if (y + font.height > ST7735_HEIGHT) break;
    }
    ST7735_DrawChar(x, y, *str, font, color, bgcolor);
    x += font.width;
    str++;
  }
}

void ST7735_DrawCard(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *title, uint16_t title_color)
{
  /* Pure Dark Card Background */
  ST7735_FillRect(x + 1, y + 1, w - 2, h - 2, COLOR_CARD_BG);
  /* Crisp Card Outline */
  ST7735_DrawRect(x, y, w, h, COLOR_CARD_BORDER);

  /* Title Badge */
  if (title && title[0] != '\0')
  {
    ST7735_DrawString(x + 4, y + 3, title, Font_7x10, title_color, COLOR_CARD_BG);
  }
}
