#ifndef __FONTS_H
#define __FONTS_H

#include <stdint.h>

typedef struct {
  const uint8_t width;
  const uint8_t height;
  const uint16_t *data;
} FontDef_t;

extern const FontDef_t Font_7x10;
extern const FontDef_t Font_11x18;
extern const FontDef_t Font_16x26;

#endif /* __FONTS_H */
