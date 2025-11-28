#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U

void SSD1306_Init(void);
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateScreen(void);
void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteString(const char *text);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_DrawBitmap(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *bitmap);

#endif // SSD1306_H
